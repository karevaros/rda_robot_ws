"""실기 스모크 시험: 관절 하나를 +Δ° → 원위치(각 구간 T초). /arm/joint_states 로 추종 기록.

    python3 real_traj_smoke.py [Δ°=5] [T=3] [관절=5]   # 관절 0=base … 5=wrist3

real_robot.launch.py 를 먼저 띄운 뒤 실행. 종료코드 0 = SUCCEEDED 이고 다른 관절이 안 움직임.
2026-10-29 실측(rb850_old · 유선): Simulation·Real 모두 6관절 ±5° SUCCEEDED · 최대 5.00° · 다른 관절 0.
"""
import math, time, sys, rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
DELTA = math.radians(float(sys.argv[1]) if len(sys.argv) > 1 else 5.0)
T1 = int(sys.argv[2]) if len(sys.argv) > 2 else 3   # 왕복 각 구간 시간[s]
J = int(sys.argv[3]) if len(sys.argv) > 3 else 5   # 움직일 관절 인덱스(0=base … 5=wrist3)

class T(Node):
    def __init__(s):
        super().__init__("traj_test")
        s.cur = None; s.log = []
        s.create_subscription(JointState, "/arm/joint_states", s.cb, 50)
        s.ac = ActionClient(s, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
    def cb(s, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM):
            s.cur = [d[j] for j in ARM]; s.log.append((time.time(), s.cur[J]))

def main():
    rclpy.init(); n = T()
    t0 = time.time()
    while n.cur is None and time.time() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)
    if n.cur is None: print("joint_states 없음"); return 1
    q0 = list(n.cur); q1 = list(q0); q1[J] += DELTA
    print("시작 자세(rad):", [round(x, 4) for x in q0])
    if not n.ac.wait_for_server(timeout_sec=5): print("액션 서버 없음"); return 1
    g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = ARM
    for q, sec in ((q1, T1), (q0, 2 * T1)):
        p = JointTrajectoryPoint(); p.positions = q; p.velocities = [0.0]*6
        p.time_from_start = Duration(sec=sec); g.trajectory.points.append(p)
    ts = time.time(); f = n.ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f)
    h = f.result()
    if not h.accepted: print("goal 거부"); return 1
    r = h.get_result_async(); rclpy.spin_until_future_complete(n, r, timeout_sec=4 * T1 + 10)
    for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
    res = r.result()
    print("결과 status=%s error_code=%s %s" % (res.status, res.result.error_code, res.result.error_string))
    w = [(t - ts, math.degrees(v - q0[J])) for t, v in n.log if t >= ts]
    peak = max(w, key=lambda x: x[1])
    print(ARM[J] + " 변화(deg): 최대 %.2f @ %.2fs · 끝 %.3f" % (peak[1], peak[0], w[-1][1]))
    for tt in (T1 / 2, T1, 1.5 * T1, 2 * T1):
        near = min(w, key=lambda x: abs(x[0] - tt)); print("  t=%.1fs → %.2f°" % (near[0], near[1]))
    others = max(abs(n.cur[i] - q0[i]) for i in range(6) if i != J)
    print("다른 관절 최대 변화(rad): %.5f" % others)
    return 0 if (res.status == 4 and others < 1e-3) else 2

sys.exit(main())
