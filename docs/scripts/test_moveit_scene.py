#!/usr/bin/env python3
"""MoveIt 셋업 검증 — 장애물이 planning scene 에 실제로 반영되는지.

검사 항목
  1. /get_planning_scene 에 obstacles.yaml 의 장애물이 전부 들어와 있는가
  2. /check_state_validity 가 자세를 옳게 판정하는가
     · 홈 자세            → 유효(충돌 없음)
     · 팔을 바닥에 처박은 자세 → 무효(ground_plane 과 충돌)
       ※ 이게 핵심 — 기구학 분석에서 "샘플 27% 가 z<0" 이었고 바닥은 링크가
         없어 자충돌 검사로는 못 잡는다. MoveIt 이 잡아야 의미가 있다.
     · 벤더 SRDF 가 Never 라 한 자충돌 자세 → 무효(자충돌)

실행: (moveit_demo.launch.py 가 떠 있는 상태에서)
  python3 docs/scripts/test_moveit_scene.py
"""
import math
import sys

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents, RobotState
from sensor_msgs.msg import JointState

ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]


def rs(vals):
    st = RobotState()
    js = JointState()
    js.name = list(ARM) + ["rg2_finger_joint1"]
    js.position = [float(v) for v in vals] + [0.0]
    st.joint_state = js
    return st


def main():
    rclpy.init()
    n = Node("test_moveit_scene")
    ok = True

    # ── 1. planning scene 의 장애물 ──────────────────────────────────────
    cli = n.create_client(GetPlanningScene, "/get_planning_scene")
    if not cli.wait_for_service(timeout_sec=15.0):
        print("❌ /get_planning_scene 없음 — moveit_demo.launch.py 가 떠 있나?")
        return 1
    req = GetPlanningScene.Request()
    # NAMES 만 요청하면 채워지는 필드가 다를 수 있어 GEOMETRY 까지 함께 요청
    req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES
                                 | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                 | PlanningSceneComponents.SCENE_SETTINGS)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=15.0)
    scene = fut.result().scene
    names = sorted(o.id for o in scene.world.collision_objects)
    want = sorted(["ground_plane", "table", "pillar", "target_ball", "side_wall"])
    print(f"planning scene 장애물 {len(names)}개: {names}")
    print(f"  planning frame = {scene.robot_model_name!r} / world frame = {scene.world.collision_objects[0].header.frame_id if names else '-'}")
    if names == want:
        print("  ✅ obstacles.yaml 의 5개가 전부 planning scene 에 있음")
    else:
        print(f"  ❌ 불일치 — 기대 {want}")
        ok = False

    # ── 2. 상태 유효성 판정 ─────────────────────────────────────────────
    vcli = n.create_client(GetStateValidity, "/check_state_validity")
    if not vcli.wait_for_service(timeout_sec=15.0):
        print("❌ /check_state_validity 없음")
        return 1

    def valid(vals):
        r = GetStateValidity.Request()
        r.group_name = "arm"
        r.robot_state = rs(vals)
        f = vcli.call_async(r)
        rclpy.spin_until_future_complete(n, f, timeout_sec=15.0)
        res = f.result()
        return res.valid, [f"{c.contact_body_1}↔{c.contact_body_2}" for c in res.contacts]

    d = math.radians
    cases = [
        ("홈 자세(전관절 0)", [0, 0, 0, 0, 0, 0], True,
         "시작 자세는 충돌이 없어야 한다"),
        ("팔을 바닥으로 처박음(shoulder 90°, elbow 90°)", [0, d(90), d(90), 0, 0, 0], False,
         "ground_plane 과 충돌해야 한다 — 바닥은 링크가 없어 MoveIt 장애물로만 잡힌다"),
        ("벤더가 Never 라 한 자충돌(link1↔link3)",
         [d(-42.7), d(1.4), d(-174.0), d(-2.3), d(169.8), d(-77.2)], False,
         "elbow -174° = 팔을 접은 자세. 벤더 SRDF 를 썼다면 놓쳤을 자충돌"),
    ]
    print()
    for label, vals, want_valid, why in cases:
        v, contacts = valid(vals)
        mark = "✅" if v == want_valid else "❌"
        if v != want_valid:
            ok = False
        exp = "유효" if want_valid else "무효(충돌)"
        got = "유효" if v else "무효(충돌)"
        print(f"{mark} {label}\n     기대={exp} 실제={got}  ({why})")
        if contacts:
            print(f"     접촉: {sorted(set(contacts))[:4]}")

    # ── 3. 실제 경로계획이 되는가 (5주차 기반 확인) ──────────────────────
    from moveit_msgs.srv import GetMotionPlan
    from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
    pcli = n.create_client(GetMotionPlan, "/plan_kinematic_path")
    if pcli.wait_for_service(timeout_sec=15.0):
        goal = {"base": d(60), "shoulder": d(-40), "elbow": d(60),
                "wrist1": d(-20), "wrist2": d(0), "wrist3": d(0)}
        r = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = "arm"
        mpr.num_planning_attempts = 5
        mpr.allowed_planning_time = 5.0
        mpr.start_state = rs([0] * 6)
        c = Constraints()
        for j, v in goal.items():
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = v
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        mpr.goal_constraints.append(c)
        r.motion_plan_request = mpr
        f = pcli.call_async(r)
        rclpy.spin_until_future_complete(n, f, timeout_sec=25.0)
        res = f.result().motion_plan_response
        pts = len(res.trajectory.joint_trajectory.points)
        # error_code 1 = SUCCESS
        if res.error_code.val == 1 and pts > 0:
            secs = res.trajectory.joint_trajectory.points[-1].time_from_start.sec
            print(f"\n✅ 경로계획 성공 — 궤적 {pts}점, 소요 {secs}s, planning time {res.planning_time:.2f}s")
            print("     (시간 파라미터화가 됐다 = joint_limits.yaml 의 가속도 제한이 먹혔다)")
        else:
            print(f"\n❌ 경로계획 실패 — error_code={res.error_code.val}, 점={pts}")
            ok = False
    else:
        print("\n⚠ /plan_kinematic_path 없음 — 계획 테스트 생략")

    n.destroy_node()
    rclpy.shutdown()
    print("\n" + ("전체 통과 ✅" if ok else "실패 있음 ❌"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
