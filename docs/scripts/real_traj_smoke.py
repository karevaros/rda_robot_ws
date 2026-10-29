"""실기 ④ 스모크 시험: wrist3 만 +5° (3s) → 원위치 (3s). 실행 중 /arm/joint_states 기록.

real_robot.launch.py 를 먼저 띄운 뒤 실행. 🔴 제어박스가 Simulation 모드인지 먼저 확인할 것.
2026-10-29 실측(rb850_old · Simulation · servo_t2 0.1): SUCCEEDED · 최대 5.00° @3.06s · 끝 0.000° · 다른 관절 0.
"""
import math, time, sys, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
DELTA = math.radians(5.0)

class T(Node):
    def __init__(s):
        super().__init__("traj_test")
        s.cur = None; s.log = []
        s.create_subscription(JointState, "/arm/joint_states", s.cb, 50)
        s.ac = ActionClient(s, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    def cb(s, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM):
            s.cur = [d[j] for j in ARM]; s.log.append((time.time(), s.cur[5]))

def main():
    rclpy.init(); n = T()
    t0 = time.time()
    while n.cur is None and time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)
    if n.cur is None: print("joint_states 없음"); return 1
    q0 = list(n.cur); q1 = list(q0); q1[5] += DELTA
    print("시작 자세(rad):", [round(x, 4) for x in q0])
    if not n.ac.wait_for_server(timeout_sec=5): print("액션 서버 없음"); return 1
    g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ARM
    for q, sec in ((q1, 3), (q0, 6)):
        p = JointTrajectoryPoint(); p.positions = q; p.velocities = [0.0]*6
        p.time_from_start = Duration(sec=sec); g.trajectory.points.append(p)
    ts = time.time(); f = n.ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f)
    h = f.result()
    if not h.accepted: print("goal 거부"); return 1
    r = h.get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=15)
    for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
    res = r.result()
    print("결과 status=%s error_code=%s %s" % (res.status, res.result.error_code, res.result.error_string))
    w = [(t - ts, math.degrees(v - q0[5])) for t, v in n.log if t >= ts]
    peak = max(w, key=lambda x: x[1])
    print("wrist3 변화(deg): 최대 %.2f @ %.2fs · 끝 %.3f" % (peak[1], peak[0], w[-1][1]))
    for tt in (1.5, 3.0, 4.5, 6.0):
        near = min(w, key=lambda x: abs(x[0] - tt)); print("  t=%.1fs → %.2f°" % (near[0], near[1]))
    others = max(abs(a - b) for a, b in zip(n.cur[:5], q0[:5]))
    print("다른 관절 최대 변화(rad): %.5f" % others)
    return 0

sys.exit(main())
