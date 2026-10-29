"""실기 접근(pre-grasp)까지 — 검출한 열매 하나 앞으로 그리퍼를 가져간다(집지는 않는다).

    python3 real_approach.py                 # 계획·검사만(실행 안 함)
    python3 real_approach.py --execute       # 통과하면 실행
    python3 real_approach.py --target 2 --standoff 0.10 --execute

전제: real_moveit.launch.py camera:=true detect:=true (옥토맵 + /detected_fruits).
🔴 --execute 는 cb_simulation:=false 면 실제 팔이 움직인다(비상정지 옆에 사람).

자세 결정(단순·보수적):
  · 그리퍼 **방향은 지금 그대로**(start 자세에서 손가락이 이미 열매 쪽을 본다)
  · TCP 접근축(tcp −y = 손가락 방향)이 열매 중심을 지나도록 옮기고,
    **손끝**(tcp 에서 0.2125 m)이 열매 표면에서 standoff 만큼 떨어지게 TCP 를 둔다.
    🔴 standoff 를 손바닥 기준으로 재면 손끝이 표면 ~2 cm 까지 들어간다(2026-10-29 첫 계획에서 발견) —
       시뮬 때 정리한 '침투 기준은 TCP 가 아니라 손끝' 과 같은 원칙.
    파지거리(집을 때 TCP↔열매 중심) = 손바닥 앞면 0.1272 + 반경 + 0.005 — 다음 단계(직선 접근)에서 쓴다.
  · IK = MoveIt /compute_ik(avoid_collisions, 현재 자세 시드) → 관절 목표 계획
검사(어기면 실행 안 함, 종료코드 3): 계획 성공 · 첫 점=현재 · 끝=목표 · 경유점이 직선 관절 구간 ±15° 안.
실행 후(실기에서만 의미): 다시 검출해 열매가 TCP 접근축 위·예상 거리에 있는지 보고.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
PALM_FRONT = 0.1272          # tcp → 손바닥 앞면 [m] (RG2 mesh 실측 · 근거대장 10장)
FINGERTIP = 0.2125           # tcp → 손끝 최대 [m] (RG2 패드 9.07~21.25 cm)
APPROACH_TCP = np.array([0.0, -1.0, 0.0])   # tcp 로컬 접근축(손가락 방향) — robot_introspect 자동감지 값
BAND = math.radians(15.0)
TOL = 0.01


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Approach:
    def __init__(self):
        self.n = rclpy.create_node("real_approach")
        self.js = None
        self.fruits = None
        self.n.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "js", m), 50)
        self.fruits_t = 0.0
        self.n.create_subscription(MarkerArray, "/detected_fruits", self._on_fruits,
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                              reliability=ReliabilityPolicy.RELIABLE))
        self.buf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.buf, self.n)
        self.ik = self.n.create_client(GetPositionIK, "/compute_ik")
        self.plan_ac = ActionClient(self.n, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(self.n, ExecuteTrajectory, "/execute_trajectory")

    def _on_fruits(self, m):
        self.fruits, self.fruits_t = m, time.time()

    def spin(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def wait(self, fut, sec):
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=sec)
        return fut.result()

    def q(self):
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM]

    def tcp(self):
        t = self.buf.lookup_transform("base_link", "tcp", rclpy.time.Time()).transform
        return np.array([t.translation.x, t.translation.y, t.translation.z]), t.rotation

    def detect(self, wait=3.0, after=0.0):
        """⚠ /detected_fruits 는 래치(TRANSIENT_LOCAL)이고 검출기는 바뀔 때만 다시 보낸다 —
        받은 걸 지우고 기다리면 영영 안 올 수 있다(실측). 대신 `after` 이후 받은 것만 쓴다."""
        t = time.time()
        while time.time() - t < wait and not (self.fruits is not None and self.fruits_t > after):
            rclpy.spin_once(self.n, timeout_sec=0.05)
        if self.fruits_t <= after:
            return []
        ms = [m for m in (self.fruits.markers if self.fruits else []) if m.action == 0 and m.ns == "fruit"]
        return [(m.id, np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]), m.scale.x / 2)
                for m in ms]

    def solve_ik(self, pos, quat):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = "arm"
        r.ik_link_name = "tcp"
        r.avoid_collisions = True
        r.robot_state = RobotState(joint_state=self.js)
        r.timeout.sec = 1
        ps = PoseStamped()
        ps.header.frame_id = "base_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pos)
        ps.pose.orientation = quat
        r.pose_stamped = ps
        res = self.wait(self.ik.call_async(req), 10)
        if res is None or res.error_code.val != 1:
            return None, (None if res is None else res.error_code.val)
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in ARM], 1

    def plan(self, goal):
        req = MotionPlanRequest(group_name="arm", num_planning_attempts=3, allowed_planning_time=5.0,
                                max_velocity_scaling_factor=0.1, max_acceleration_scaling_factor=0.1)
        req.start_state.is_diff = True
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=j, position=v, tolerance_above=1e-3,
                                               tolerance_below=1e-3, weight=1.0) for j, v in zip(ARM, goal)]
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
        worst = max(max(min(start[k], goal[k]) - BAND - p[k], p[k] - max(start[k], goal[k]) - BAND)
                    for p in pts for k in range(6))
        d = jt.points[-1].time_from_start
        msg = f"점 {len(pts)}개 · {d.sec + d.nanosec * 1e-9:.1f}s"
        if worst > 0:
            return False, msg + f" · 경유점이 직선 구간 ±15° 를 {math.degrees(worst):.1f}° 벗어남"
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
    ap.add_argument("--target", type=int, default=None, help="검출 id (기본: 카메라에서 가장 가까운 것)")
    ap.add_argument("--standoff", type=float, default=0.10, help="열매 표면 앞 여유[m]")
    ap.add_argument("--execute", action="store_true")
    a = ap.parse_args()

    rclpy.init()
    A = Approach()
    if not A.ik.wait_for_service(timeout_sec=15) or not A.plan_ac.wait_for_server(timeout_sec=15):
        print("MoveIt 없음 — real_moveit.launch.py 확인"); return 1
    A.spin(1.0)
    if A.js is None:
        print("/joint_states 없음"); return 1
    fr = A.detect()
    if not fr:
        print("검출 열매 없음 — detect:=true 확인"); return 1
    cam = A.buf.lookup_transform("base_link", "sensor1_camera_color_optical_frame", rclpy.time.Time()).transform.translation
    cam = np.array([cam.x, cam.y, cam.z])
    if a.target is None:
        fid, p, r = min(fr, key=lambda f: np.linalg.norm(f[1] - cam))
    else:
        cand = [f for f in fr if f[0] == a.target]
        if not cand:
            print(f"id {a.target} 없음 — 검출 id: {[f[0] for f in fr]}"); return 1
        fid, p, r = cand[0]
    print(f"검출 {len(fr)}개 → 목표 id {fid} · 위치 {p.round(3)} · 반경 {r*1000:.0f} mm · 카메라에서 {np.linalg.norm(p-cam)*1000:.0f} mm")

    t0, q0 = A.tcp()
    R0 = quat_to_R(q0)
    ax = R0 @ APPROACH_TCP
    goff = PALM_FRONT + r + 0.005
    pre = FINGERTIP + r + a.standoff                 # TCP ↔ 열매 중심(손끝 기준 standoff)
    tgt = p - ax * pre
    off = (p - t0) - ax * float(ax @ (p - t0))
    print(f"현재 TCP {t0.round(3)} · 접근축 {ax.round(3)} · 열매의 접근축 옆 어긋남 {np.linalg.norm(off)*1000:.0f} mm")
    print(f"pre-grasp TCP {tgt.round(3)} (열매 중심에서 {pre*1000:.0f} mm 앞 · 손끝↔표면 {a.standoff*1000:.0f} mm · 이후 직선 접근 {(pre - goff)*1000:.0f} mm) · 이동 {np.linalg.norm(tgt-t0)*1000:.0f} mm")

    start = A.q()
    goal, code = A.solve_ik(tgt, q0)
    if goal is None:
        print(f"✖ IK 실패(error_code={code}) — 도달 불가 또는 충돌"); return 3
    print("IK 관절 변화(deg):", [round(math.degrees(g - s), 1) for g, s in zip(goal, start)])
    traj, msg = A.plan(goal)
    if traj is None:
        print("✖", msg); return 3
    ok, msg = A.check(traj, start, goal)
    print(f"계획 검사: {'통과' if ok else '✖'} — {msg}")
    if not ok:
        return 3
    if not a.execute:
        print("(계획만 — 실행하려면 --execute)"); return 0
    ok, msg = A.execute(traj)
    A.spin(1.0)
    t1, q1 = A.tcp()
    print(f"실행: {'SUCCESS' if ok else '✖'} ({msg}) · TCP 도착 오차 {np.linalg.norm(t1 - tgt)*1000:.1f} mm")
    fr2 = A.detect(8.0, after=time.time())
    if fr2:
        ax1 = quat_to_R(q1) @ APPROACH_TCP
        f2 = min(fr2, key=lambda f: np.linalg.norm(f[1] - p))
        d = f2[1] - t1
        along = float(ax1 @ d)
        side = np.linalg.norm(d - ax1 * along)
        print(f"도착 후 재검출: 목표 근처 열매 id {f2[0]} · 접근축 방향 거리 {along*1000:.0f} mm(기대 {pre*1000:.0f}) · 축 옆 어긋남 {side*1000:.0f} mm")
    return 0 if ok else 4


if __name__ == "__main__":
    sys.exit(main())
