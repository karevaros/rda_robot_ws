#!/usr/bin/env python3
"""Unity ↔ ROS 연동 검증 — 살아 있는 ROS 그래프의 자세를 Unity 가 그대로 재현하는가.

    python3 docs/scripts/test_unity_ros_bridge.py [--project ~/robot_ws/export/unity]
                                                  [--pose 0.30 -0.70 0.90 -0.45 0.25 0.15]
                                                  [--no-move]

■ 무엇을 재는가
  ROS 쪽에서 팔을 임의 자세로 보낸 뒤, **Unity 플레이어를 헤드리스로 띄워** 그 자세를
  받아 그린 결과의 tcp 위치를 URDF FK 와 대조한다. 즉 "Unity 가 같은 로봇인가"(정적)가
  아니라 **"살아 있는 ROS 상태를 Unity 가 맞게 재현하는가"**(동적)를 잰다.

■ 선행 조건 (없으면 친절히 알려주고 종료한다)
  ① ROS 스택이 떠서 /joint_states 가 발행 중일 것 (예: perception_demo.launch.py)
  ② ros_tcp_endpoint 가 떠 있을 것
       ros2 run ros_tcp_endpoint default_server_endpoint \
         --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
     ⚠ endpoint 는 설치 시 패치가 필요하다 → docs/scripts/setup_ros_tcp_endpoint.sh
  ③ Unity 플레이어가 빌드돼 있을 것 (<project>/Build/RdaPlayer)
       Unity -batchmode -nographics -quit -projectPath <p> -executeMethod RdaBatch.BuildScene
       Unity -batchmode -nographics -quit -projectPath <p> -executeMethod RdaBatch.BuildPlayer

■ ⚠ 함정 (전부 실측으로 밟은 것들 — 2026-08-16)
  · endpoint 는 플레이어가 끊고 나가면 스레드가 죽은 채 살아 있는 것처럼 보인다
    (`Bad file descriptor`). 수신 0 이면 **먼저 endpoint 를 재기동**할 것.
  · `ros2 topic pub --once` 는 구독자 연결 전에 끝나 **명령이 조용히 유실된다** → `-w 1 -t 3`.
  · Unity 는 ROS1/ROS2 를 **컴파일 타임 define** 으로 가른다. `ROS2` 가 없으면 연결·구독은
    되는데 역직렬화만 깨져 수신 0 이 된다(생성기가 ProjectSettings 에 써 넣는다).
"""
import argparse
import importlib.util
import json
import os
import subprocess
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(_HERE, "..", "..", "rda_robot_bringup", "scripts")

_ok, _fail = [], []


def check(name, cond, detail=""):
    (_ok if cond else _fail).append(name)
    print(f"{'✅' if cond else '❌'} {name}" + (f"  {detail}" if detail else ""))


def _imp(mod, fname):
    spec = importlib.util.spec_from_file_location(mod, os.path.join(_SCRIPTS, fname))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", default=os.path.expanduser("~/robot_ws/export/unity"))
    ap.add_argument("--pose", nargs=6, type=float,
                    default=[0.30, -0.70, 0.90, -0.45, 0.25, 0.15],
                    help="base shoulder elbow wrist1 wrist2 wrist3 [rad]")
    ap.add_argument("--no-move", action="store_true", help="팔을 옮기지 않고 현재 자세로 검증")
    ap.add_argument("--tol", type=float, default=1e-3, help="허용 오차[m]")
    a = ap.parse_args()

    proj = os.path.abspath(os.path.expanduser(a.project))
    player = os.path.join(proj, "Build", "RdaPlayer")
    report = os.path.join(proj, "rda_ros_bridge_report.json")

    if not os.path.exists(player):
        print(f"❌ 플레이어가 없다: {player}\n   먼저 RdaBatch.BuildScene → RdaBatch.BuildPlayer")
        sys.exit(2)

    if not a.no_move:
        names = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
        pts = ", ".join(str(v) for v in a.pose)
        msg = ("{joint_names: [" + ", ".join(names) + "], points: [{positions: ["
               + pts + "], time_from_start: {sec: 4}}]}")
        # ⚠ --once 는 구독자 연결 전에 끝나 유실된다 → -w 1(구독자 대기) -t 3(3회)
        subprocess.run(["ros2", "topic", "pub", "-w", "1", "-t", "3",
                        "/arm_controller/joint_trajectory",
                        "trajectory_msgs/msg/JointTrajectory", msg],
                       capture_output=True, timeout=60)
        subprocess.run(["sleep", "6"])

    if os.path.exists(report):
        os.remove(report)
    subprocess.run([player, "-batchmode", "-nographics", "-logFile", "/dev/null"],
                   cwd=proj, capture_output=True, timeout=180)

    check("플레이어가 리포트를 남겼다", os.path.exists(report), report)
    if not os.path.exists(report):
        _summary()
    rep = json.load(open(report))

    n = int(rep.get("received", 0))
    check("Unity 가 /joint_states 를 받았다", n > 0,
          f"{n}건" + ("" if n else "  ← endpoint 재기동 후 다시 시도(죽은 스레드가 살아 있는 것처럼 보인다)"))
    if not n:
        _summary()

    q = rep.get("last") or {}
    check("관절 매핑이 ROS 이름으로 됐다", len(q) >= 6,
          f"{len(q)}개: {','.join(sorted(q))}")

    g = _imp("_gen_usd_scene", "gen_usd_scene.py")
    RI = _imp("_robot_introspect", "robot_introspect.py")
    sys.path.insert(0, _HERE)
    from test_unity_export import ros_to_unity_pos          # noqa: E402

    urdf = os.path.join(proj, "rda_robot.urdf")
    links, joints = g.parse_urdf(urdf)
    c2j = {j["child"]: nm for nm, j in joints.items()}
    chain = RI.fk_chain(joints, c2j, "base_link", "tcp")
    p_base = RI.fk_pos(joints, chain, q)
    _, W = g.link_world_tf(links, joints)
    p_world = (W["base_link"] @ np.append(p_base, 1.0))[:3]

    want = np.asarray(ros_to_unity_pos(p_world), float)
    got = np.asarray(rep.get("tcp") or [np.nan] * 3, float)
    err = float(np.abs(got - want).max())
    check("Unity tcp = 수신 관절각의 URDF FK", err <= a.tol,
          f"오차 {err:.3e} m (허용 {a.tol:g}) · FK {np.round(want, 6).tolist()} vs Unity {np.round(got, 6).tolist()}")

    if not a.no_move:
        moved = max(abs(float(q.get(k, 0.0)) - v)
                    for k, v in zip(["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"],
                                    a.pose))
        check("명령한 자세가 실제로 반영됐다", moved < 1e-2,
              f"명령↔수신 최대차 {moved:.2e} rad")

    _summary()


def _summary():
    print(f"\n통과 {len(_ok)} / 실패 {len(_fail)}  (총 {len(_ok) + len(_fail)})")
    if _fail:
        print("실패:", ", ".join(_fail))
    sys.exit(1 if _fail else 0)


if __name__ == "__main__":
    main()
