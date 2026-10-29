"""실기 MoveIt 스모크 시험: 현재 자세 → (관절별 +Δ) → 원위치. **계획만 받고 검사한 뒤에만 실행**한다.

    python3 real_moveit_smoke.py [Δ°=5] [관절들=shoulder,elbow,wrist1]

real_moveit.launch.py 를 먼저 띄울 것. 🔴 cb_simulation:=false 면 실제 팔이 움직인다.

검사(하나라도 어기면 실행하지 않고 종료코드 3):
  · 계획 성공(error_code 1) · 궤적 점 ≥ 2
  · 첫 점이 현재 자세와 0.01 rad 이내
  · **모든 경유점이 시작↔목표 관절 구간 ±3° 안** — 장애물 없는 짧은 이동인데 OMPL 이
    돌아가는 경로를 내면(실기 주변은 모델과 다르다) 실행하지 않는다
  · 마지막 점 = 목표(0.01 rad)
실행 후: ExecuteTrajectory 결과 SUCCESS + 실제 자세가 목표와 0.01 rad 이내.
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from sensor_msgs.msg import JointState

ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
BAND = math.radians(3.0)
TOL = 0.01


class Smoke(Node):
    def __init__(self):
        super().__init__("real_moveit_smoke")
        self.cur = None
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.plan_ac = ActionClient(self, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM):
            self.cur = [d[j] for j in ARM]

    def wait(self, fut, t):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=t)
        return fut.result()

    def fresh(self):
        self.cur = None
        t = time.time()
        while self.cur is None and time.time() - t < 3:
            rclpy.spin_once(self, timeout_sec=0.05)
        return list(self.cur) if self.cur else None

    def plan(self, goal):
        req = MotionPlanRequest()
        req.group_name = "arm"
        req.start_state.is_diff = True        # 비우면 MoveIt 이 "Found empty JointState" 를 낸다(현재 상태 사용)
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.1
        c = Constraints()
        for j, q in zip(ARM, goal):
            c.joint_constraints.append(JointConstraint(joint_name=j, position=q, tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        req.goal_constraints = [c]
        g = MoveGroup.Goal()
        g.request = req
        g.planning_options.plan_only = True
        h = self.wait(self.plan_ac.send_goal_async(g), 10)
        if not h or not h.accepted:
            return None, "plan goal 거부"
        r = self.wait(h.get_result_async(), 15)
        if r is None:
            return None, "plan 응답 없음"
        if r.result.error_code.val != 1:
            return None, f"plan 실패 error_code={r.result.error_code.val}"
        return r.result.planned_trajectory, "ok"

    def check(self, traj, start, goal):
        jt = traj.joint_trajectory
        idx = [jt.joint_names.index(j) for j in ARM]
        pts = [[p.positions[i] for i in idx] for p in jt.points]
        if len(pts) < 2:
            return False, "궤적 점 부족"
        if max(abs(a - b) for a, b in zip(pts[0], start)) > TOL:
            return False, "첫 점이 현재 자세와 다름"
        if max(abs(a - b) for a, b in zip(pts[-1], goal)) > TOL:
            return False, "마지막 점이 목표와 다름"
        worst = 0.0
        for p in pts:
            for k in range(6):
                lo, hi = min(start[k], goal[k]) - BAND, max(start[k], goal[k]) + BAND
                worst = max(worst, lo - p[k], p[k] - hi)
        if worst > 0:
            return False, f"경유점이 직선 구간 ±3° 를 {math.degrees(worst):.2f}° 벗어남"
        dur = jt.points[-1].time_from_start
        return True, f"점 {len(pts)}개 · {dur.sec + dur.nanosec * 1e-9:.2f}s · 구간 이탈 없음"

    def execute(self, traj):
        g = ExecuteTrajectory.Goal()
        g.trajectory = traj
        h = self.wait(self.exec_ac.send_goal_async(g), 10)
        if not h or not h.accepted:
            return False, "execute goal 거부"
        r = self.wait(h.get_result_async(), 60)
        if r is None:
            return False, "execute 응답 없음"
        return r.result.error_code.val == 1, f"error_code={r.result.error_code.val}"

    def leg(self, name, goal):
        start = self.fresh()
        traj, msg = self.plan(goal)
        if traj is None:
            print(f"[{name}] ✖ {msg}"); return 3
        ok, msg = self.check(traj, start, goal)
        print(f"[{name}] 계획 검사: {'통과' if ok else '✖'} — {msg}")
        if not ok:
            return 3
        ok, msg = self.execute(traj)
        end = self.fresh()
        err = max(abs(a - b) for a, b in zip(end, goal))
        print(f"[{name}] 실행: {'SUCCESS' if ok else '✖'} ({msg}) · 도착 오차 {math.degrees(err):.3f}°")
        return 0 if (ok and err <= TOL) else 4


def main():
    delta = math.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 5.0)
    joints = (sys.argv[2] if len(sys.argv) > 2 else "shoulder,elbow,wrist1").split(",")
    rclpy.init()
    n = Smoke()
    for ac, nm in ((n.plan_ac, "/move_action"), (n.exec_ac, "/execute_trajectory")):
        if not ac.wait_for_server(timeout_sec=15):
            print(f"{nm} 없음 — real_moveit.launch.py 가 떠 있는지 확인"); return 1
    q0 = n.fresh()
    if q0 is None:
        print("/joint_states 없음"); return 1
    print("시작 자세(deg):", [round(math.degrees(x), 2) for x in q0])
    q1 = list(q0)
    for j in joints:
        q1[ARM.index(j)] += delta
    rc = n.leg("가기", q1)
    if rc == 0:
        rc = n.leg("돌아오기", q0)
    return rc


if __name__ == "__main__":
    sys.exit(main())
