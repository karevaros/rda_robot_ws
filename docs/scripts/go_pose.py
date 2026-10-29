"""실기 이름 붙은 자세 — 저장하거나 그 자세로 이동한다(config/real_poses.yaml).

    python3 go_pose.py start              # 'start' 로 이동: MoveIt 계획 → 검사 → 실행
    python3 go_pose.py start --force      # 경로 검사(직선 구간 ±10°)를 넘어도 실행
    python3 go_pose.py --save 이름         # 지금 관절각을 그 이름으로 저장(덮어씀)
    python3 go_pose.py --list

real_moveit.launch.py 가 떠 있어야 한다(camera:=true 면 옥토맵 장애물도 피한다).
🔴 이동은 실제 팔이 움직인다 — 비상정지 옆에 사람.

검사(어기면 실행하지 않음, 종료코드 3): 계획 성공 · 첫 점=현재(0.01 rad) · 끝=목표(0.01 rad) ·
모든 경유점이 시작↔목표 관절 구간 ±BAND 안(멀리 돌아가는 경로 차단 — 실기 주변은 모델과 다르다).
"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
BAND = math.radians(10.0)
TOL = 0.01
#: 저장·읽기는 **소스 트리**(정본) 쪽 파일 — 설치본을 고치면 다음 빌드에 덮인다.
SRC = os.path.expanduser("~/robot_ws/src/rda_robot_description/config/real_poses.yaml")


def poses_file():
    if os.path.exists(SRC):
        return SRC
    return os.path.join(get_package_share_directory("rda_robot_description"), "config", "real_poses.yaml")


def load():
    with open(poses_file()) as f:
        return (yaml.safe_load(f) or {}).get("poses", {}) or {}


def save(name, q):
    """주석을 지키려고 줄 단위로 쓴다(yaml.safe_dump 는 주석을 날린다)."""
    path = poses_file()
    lines = open(path).read().splitlines()
    block = [f"  {name}:",
             f"    note: \"{time.strftime('%Y-%m-%d %H:%M')} go_pose.py --save\"",
             "    deg: {" + ", ".join(f"{n}: {math.degrees(v):.4f}" for n, v in zip(ARM, q)) + "}",
             "    rad: {" + ", ".join(f"{n}: {v:.6f}" for n, v in zip(ARM, q)) + "}"]
    out, i, replaced = [], 0, False
    while i < len(lines):
        if lines[i] == f"  {name}:":
            i += 1
            while i < len(lines) and lines[i].startswith("    "):
                i += 1
            out += block; replaced = True
            continue
        out.append(lines[i]); i += 1
    if not replaced:
        out += block
    open(path, "w").write("\n".join(out) + "\n")
    return path, replaced


class Mover:
    def __init__(self):
        self.n = rclpy.create_node("go_pose")
        self.cur = None
        self.n.create_subscription(JointState, "/joint_states", self._js, 50)
        self.plan_ac = ActionClient(self.n, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(self.n, ExecuteTrajectory, "/execute_trajectory")

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM):
            self.cur = [d[j] for j in ARM]

    def fresh(self, sec=3.0):
        self.cur = None
        t = time.time()
        while self.cur is None and time.time() - t < sec:
            rclpy.spin_once(self.n, timeout_sec=0.05)
        return list(self.cur) if self.cur else None

    def wait(self, fut, sec):
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=sec)
        return fut.result()

    def plan(self, goal):
        req = MotionPlanRequest(group_name="arm", num_planning_attempts=3, allowed_planning_time=5.0,
                                max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1)
        req.start_state.is_diff = True
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=j, position=q, tolerance_above=1e-3,
                                               tolerance_below=1e-3, weight=1.0) for j, q in zip(ARM, goal)]
        req.goal_constraints = [c]
        g = MoveGroup.Goal(request=req)
        g.planning_options.plan_only = True
        h = self.wait(self.plan_ac.send_goal_async(g), 10)
        if not h or not h.accepted:
            return None, "plan goal 거부"
        r = self.wait(h.get_result_async(), 20)
        if r is None or r.result.error_code.val != 1:
            return None, f"계획 실패(error_code={None if r is None else r.result.error_code.val})"
        return r.result.planned_trajectory, "ok"

    def check(self, traj, start, goal):
        jt = traj.joint_trajectory
        idx = [jt.joint_names.index(j) for j in ARM]
        pts = [[p.positions[i] for i in idx] for p in jt.points]
        if max(abs(a - b) for a, b in zip(pts[0], start)) > TOL:
            return False, "첫 점이 현재 자세와 다름"
        if max(abs(a - b) for a, b in zip(pts[-1], goal)) > TOL:
            return False, "끝 점이 목표와 다름"
        worst = 0.0
        for p in pts:
            for k in range(6):
                lo, hi = min(start[k], goal[k]) - BAND, max(start[k], goal[k]) + BAND
                worst = max(worst, lo - p[k], p[k] - hi)
        d = jt.points[-1].time_from_start
        msg = f"점 {len(pts)}개 · {d.sec + d.nanosec * 1e-9:.1f}s · 최대 관절 이동 " \
              f"{math.degrees(max(abs(a - b) for a, b in zip(start, goal))):.1f}°"
        if worst > 0:
            return False, msg + f" · 경유점이 직선 구간 ±10° 를 {math.degrees(worst):.1f}° 벗어남"
        return True, msg

    def execute(self, traj):
        h = self.wait(self.exec_ac.send_goal_async(ExecuteTrajectory.Goal(trajectory=traj)), 10)
        if not h or not h.accepted:
            return False, "execute goal 거부"
        r = self.wait(h.get_result_async(), 120)
        return (r is not None and r.result.error_code.val == 1), \
            f"error_code={None if r is None else r.result.error_code.val}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name", nargs="?")
    ap.add_argument("--save", metavar="이름")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()

    if a.list:
        for k, v in load().items():
            print(f"{k:12s} {v.get('deg')}  — {v.get('note', '')}")
        return 0

    rclpy.init()
    m = Mover()
    if a.save:
        q = m.fresh()
        if q is None:
            print("/joint_states 없음 — real_robot/real_moveit launch 가 떠 있어야 한다"); return 1
        path, rep = save(a.save, q)
        print(f"{'덮어씀' if rep else '추가'}: '{a.save}' = {[round(math.degrees(v), 2) for v in q]} deg → {path}")
        return 0

    poses = load()
    if a.name not in poses:
        print(f"'{a.name}' 없음 — 있는 자세: {list(poses)}"); return 1
    goal = [math.radians(float(poses[a.name]["deg"][j])) for j in ARM]
    for ac, nm in ((m.plan_ac, "/move_action"), (m.exec_ac, "/execute_trajectory")):
        if not ac.wait_for_server(timeout_sec=15):
            print(f"{nm} 없음 — real_moveit.launch.py 가 떠 있는지 확인"); return 1
    start = m.fresh()
    if start is None:
        print("/joint_states 없음"); return 1
    if max(abs(a_ - b) for a_, b in zip(start, goal)) < TOL:
        print(f"이미 '{a.name}' 자세다"); return 0
    traj, msg = m.plan(goal)
    if traj is None:
        print("✖", msg); return 3
    ok, msg = m.check(traj, start, goal)
    print(f"계획 검사: {'통과' if ok else '✖'} — {msg}")
    if not ok and not a.force:
        print("실행하지 않음(--force 로 강제 가능)"); return 3
    ok, msg = m.execute(traj)
    end = m.fresh()
    err = max(abs(x - y) for x, y in zip(end, goal))
    print(f"실행: {'SUCCESS' if ok else '✖'} ({msg}) · 도착 오차 {math.degrees(err):.3f}°")
    return 0 if ok and err <= TOL else 4


if __name__ == "__main__":
    sys.exit(main())
