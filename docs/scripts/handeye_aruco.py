"""손목 카메라 hand-eye 보정(eye-in-hand) — ArUco 마커 1장, 위치는 몰라도 된다.

    ros2 launch rda_robot_description real_robot.launch.py robot_ip:=… cb_simulation:=false gripper:=false
    python3 handeye_aruco.py <마커 한 변[m]> <URDF(현재)> [URDF(비교용)…]

🔴 실제 팔이 움직인다(현재 자세 기준 관절 ±12° 이내 · 이동당 4 s). 비상정지 옆에 사람.
    실패·안전 감시 abort 시 그 자리에서 멈춘다(원위치 복귀를 시도하지 않는다 — 갈 길을 모른다).

판정 두 가지:
  ① cv2.calibrateHandEye(PARK)로 tcp→color_optical 을 구해 URDF 값과 비교
  ② 각 URDF 의 외부값으로 '로봇 기준 마커 위치'를 자세마다 계산 → 흩어짐(mm). 마커는 고정이므로
     맞는 외부값일수록 흩어짐이 작다(계산식에 기대지 않는 직관적 비교).
카메라는 pyrealsense2 로 직접 연다(realsense2_camera 노드와 동시에 쓰지 말 것).

2026-10-29 실측(Real · 마커 180 mm 를 1.62 m 칸막이에 · 11자세 전부 유효):
  ① calibrateHandEye(PARK) **실패**(1.13 m · 97° — 무의미) — 회전이 ±12° 로 작고 마커가 멀어
     문제가 나쁘게 조건화된다. hand-eye 는 보통 ≥30° 회전 다양성이 필요하다.
  ② 흩어짐: 실측 반영 URDF 평균 4.4·최대 7.1 mm ↔ 옛 URDF 5.5·8.6 mm — 실측 쪽이 낫지만
     두 외부값 차(27 mm)가 작은 회전에선 흩어짐에 4~5 mm 로만 드러나 잡음과 비슷하다(약한 근거).
  ⇒ 확정하려면 마커를 0.4~0.6 m 로 가깝게 + 회전을 키운 자세(±25° 이상)로 다시.
"""
import math
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yourdfpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
#: 현재 자세 기준 변화[deg] — 마커가 화면에 남도록 손목 위주
POSES = [{}, {"wrist3": 12}, {"wrist3": -12}, {"wrist1": 8}, {"wrist1": -8}, {"wrist2": 8},
         {"wrist2": -8}, {"wrist1": 6, "wrist2": 6, "wrist3": 10},
         {"wrist1": -6, "wrist2": -6, "wrist3": -10}, {"elbow": 5, "wrist1": -5},
         {"shoulder": 4, "elbow": -4}]
MOVE_SEC = 4
FRAMES = 10
CAM = "sensor1_camera_color_optical_frame"
TCP = "tcp"


def resolve(fname):
    if fname.startswith("package://"):
        pkg, rest = fname[len("package://"):].split("/", 1)
        return get_package_share_directory(pkg) + "/" + rest
    return fname.replace("file://", "")


class Robot:
    def __init__(self):
        self.n = rclpy.create_node("handeye_aruco")
        self.q = None
        self.abort = False
        self.n.create_subscription(JointState, "/arm/joint_states", self._js, 50)
        self.n.create_subscription(Bool, "/safety_monitor/abort", lambda m: setattr(self, "abort", m.data),
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ac = ActionClient(self.n, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in ARM):
            self.q = [d[j] for j in ARM]

    def spin(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self.n, timeout_sec=0.02)

    def move(self, target):
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        p = JointTrajectoryPoint(positions=list(target), velocities=[0.0] * 6,
                                 time_from_start=Duration(sec=MOVE_SEC))
        g.trajectory.points = [p]
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=5)
        h = f.result()
        if not h or not h.accepted:
            return False
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self.n, r, timeout_sec=MOVE_SEC + 10)
        return r.result() is not None and r.result().status == 4


def main():
    side = float(sys.argv[1])
    urdfs = sys.argv[2:]
    rclpy.init()
    rb = Robot()
    if not rb.ac.wait_for_server(timeout_sec=10):
        print("arm_controller 없음"); return 1
    rb.spin(1.0)
    if rb.q is None or rb.abort:
        print("관절 상태 없음 또는 안전 감시 abort 상태"); return 1
    q0 = list(rb.q)

    pipe = rs.pipeline(); cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    k = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[k.fx, 0, k.ppx], [0, k.fy, k.ppy], [0, 0, 1]])
    D = np.array(k.coeffs)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                  cv2.aruco.DetectorParameters())
    h = side / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)

    def look():
        tv, rvs = [], []
        for _ in range(FRAMES * 2):
            img = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data())
            cs, ids, _ = det.detectMarkers(img)
            if ids is None or 0 not in ids.ravel():
                continue
            c = cs[list(ids.ravel()).index(0)].reshape(4, 2)
            ok, rv, t = cv2.solvePnP(obj, c, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                tv.append(t.ravel()); rvs.append(rv.ravel())
            if len(tv) >= FRAMES:
                break
        if len(tv) < FRAMES // 2:
            return None
        t = np.median(np.array(tv), axis=0)
        R, _ = cv2.Rodrigues(np.median(np.array(rvs), axis=0))
        return R, t, np.std(np.array(tv), axis=0)

    models = [yourdfpy.URDF.load(u, filename_handler=resolve, load_meshes=False) for u in urdfs]
    rec = []                                                    # (q, R_t2c, t_t2c)
    try:
        for i, d in enumerate(POSES):
            tgt = [q0[j] + math.radians(d.get(n, 0.0)) for j, n in enumerate(ARM)]
            if i > 0 and not rb.move(tgt):
                print(f"[{i}] 이동 실패 — 여기서 멈춘다"); break
            rb.spin(1.0)
            if rb.abort:
                print(f"[{i}] 안전 감시 abort — 여기서 멈춘다"); break
            obs = look()
            q = list(rb.q)
            if obs is None:
                print(f"[{i}] {d or '현재'} · 마커 안 보임 — 건너뜀"); continue
            R, t, sd = obs
            rec.append((q, R, t))
            print(f"[{i}] {d or '현재'} · 마커 {np.linalg.norm(t):.3f} m · 프레임 흔들림 {np.linalg.norm(sd)*1000:.1f} mm")
        if rb.abort or len(rec) < 3:
            pass
        elif not rb.move(q0):
            print("원위치 이동 실패")
        else:
            print("원위치 복귀")
    finally:
        pipe.stop()

    if len(rec) < 3:
        print(f"유효 자세 {len(rec)}개 — 계산 불가"); return 2

    def fk(m, q):
        m.update_cfg(dict(zip(ARM, q)))
        return m.get_transform(TCP)                              # base_link → tcp

    Rg, tg, Rt, tt = [], [], [], []
    for q, R, t in rec:
        T = fk(models[0], q)
        Rg.append(T[:3, :3]); tg.append(T[:3, 3]); Rt.append(R); tt.append(t)
    Rx, tx = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=cv2.CALIB_HAND_EYE_PARK)
    X = np.eye(4); X[:3, :3] = Rx; X[:3, 3] = tx.ravel()
    print(f"\n유효 자세 {len(rec)}개")
    print("① calibrateHandEye: tcp→color_optical t = %s mm" % (X[:3, 3] * 1000).round(1))

    def spread(Xc):
        P = []
        for q, R, t in rec:
            P.append((fk(models[0], q) @ Xc @ np.r_[t, 1.0])[:3])
        P = np.array(P)
        return np.linalg.norm(P - P.mean(axis=0), axis=1)

    s = spread(X)
    print("   이 값으로 마커 위치 흩어짐: 평균 %.1f · 최대 %.1f mm" % (s.mean() * 1000, s.max() * 1000))
    for u, m in zip(urdfs, models):
        m.update_cfg(dict(zip(ARM, q0)))
        Xm = np.linalg.inv(m.get_transform(TCP)) @ m.get_transform(CAM)
        dR = np.degrees(np.arccos(np.clip((np.trace(Xm[:3, :3].T @ Rx) - 1) / 2, -1, 1)))
        s = spread(Xm)
        print("② %-22s t = %s mm · 보정값과 차 %.1f mm / %.2f° · 마커 흩어짐 평균 %.1f · 최대 %.1f mm"
              % (u.split("/")[-1], (Xm[:3, 3] * 1000).round(1), np.linalg.norm(Xm[:3, 3] - X[:3, 3]) * 1000, dR,
                 s.mean() * 1000, s.max() * 1000))
    np.save("handeye_X.npy", X)
    return 0


if __name__ == "__main__":
    sys.exit(main())
