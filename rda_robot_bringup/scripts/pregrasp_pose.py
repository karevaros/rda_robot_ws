#!/usr/bin/env python3
"""5주차 1차 — pre-grasp(접근) 자세 추정 : "토마토를 바라보는" 팔 자세 찾기.

문제(사용자 정의): 목표 토마토(위치=입력)에 대해, **TCP 가 열매를 바라보고(gaze)
→ 직선으로 접근 가능한** 팔 자세(joint config)를 구한다. 그 자세는 **자가충돌 + 환경충돌
(줄기·거터·레일)** 이 없어야 한다. (실제 궤적/줄기 회피 경로는 다음 단계.)

핵심 기하(실측, docs/기구학-분석 · compose_urdf FK):
  · 그리퍼 **접근축 = TCP 로컬 −Y** (손끝이 −Y 로 뻗음, link6 은 +Y 뒤쪽).
  · 손가락 벌어짐 축 = TCP 로컬 X.
  ⇒ "바라보는 자세" = TCP 의 −Y 축을 접근방향 a(열매 중심 향함)에 정렬.

알고리즘 = **후보 샘플링(candidate sampling)** 구조 (1차 = 명목 수평 후보 중심):
  1) 접근방향 a 후보 생성 — 명목 = 통로(로봇)쪽 수평 = horizontal(target − base).
     여기에 방위각 φ · 고각 θ · 롤 ψ · standoff d 를 그리드로 흔들어 후보 다발 생성.
  2) 각 후보 → TCP 목표 pose(위치=standoff 점, 방향=−Y∥a) → MoveIt /compute_ik
     (avoid_collisions=True : 자가충돌 + planning scene 환경충돌 동시 검사).
  3) 통과 후보를 비용함수(명목근접·롤·standoff·관절중앙성)로 점수화 → 최소비용 선택.
  1차는 후보 1개(명목)만 두면 "통로쪽 수평 고정"과 동일. 그리드를 넓히면 도달률↑.

파라미터(요약, 전부 ros2 param):  target[x,y,z]·fruit_radius·standoff(0.15,튜너조절)
  ·approach_yaw_deg(빈값=base→target 자동)·sample_phi/theta/psi_deg·sample_standoff·weights.
target 미지정 시 /obstacle_markers 의 '[목표]' 열매 중 target_index 번째를 자동 선택.

출력:  /pregrasp_markers(target·standoff·접근직선·TCP 축)  ·  /pregrasp_robot_state
  (DisplayRobotState = RViz 에서 팔이 pre-grasp 자세로 표시)  ·  로그(관절해/도달불가).
"""

import math
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

try:
    from moveit_msgs.srv import GetPositionIK
    from moveit_msgs.msg import (PositionIKRequest, RobotState,
                                 DisplayRobotState)
    _HAVE_MOVEIT = True
except Exception as e:                                   # pragma: no cover
    _HAVE_MOVEIT = False
    _MOVEIT_ERR = e

_RI = None


def _import_ri():
    """robot_introspect.py(형제 파일)를 동적 임포트(모델 introspection)."""
    global _RI
    if _RI is None:
        import importlib.util
        import os
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "robot_introspect.py")
        spec = importlib.util.spec_from_file_location("_robot_introspect", p)
        _RI = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(_RI)
    return _RI


# ───────────────────────── 순수 기하 (ROS 무관, 단위테스트 용이) ─────────────────────────
def _unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else v


def _rot_axis(axis, ang):
    """Rodrigues: axis(단위) 둘레 ang 회전행렬."""
    a = _unit(axis)
    c, s = math.cos(ang), math.sin(ang)
    x, y, z = a
    C = 1.0 - c
    return np.array([
        [c + x*x*C,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C],
    ])


def _rot_u_to_v(u, v):
    """u 를 v 로 보내는 최소 회전(둘 다 단위벡터)."""
    u, v = _unit(u), _unit(v)
    c = float(np.dot(u, v))
    if c > 1.0 - 1e-9:
        return np.eye(3)
    if c < -1.0 + 1e-9:                                   # 정반대 → 수직축 180°
        perp = np.cross(u, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(u, np.array([0.0, 1.0, 0.0]))
        return _rot_axis(perp, math.pi)
    return _rot_axis(np.cross(u, v), math.acos(max(-1.0, min(1.0, c))))


def approach_dir(nominal, phi, theta):
    """명목 접근축(단위)에 방위각 φ(수직축 둘레)·고각 θ(수평 직교축 둘레) 적용."""
    a = _unit(nominal)
    a = _rot_axis([0, 0, 1], phi) @ a                     # 방위각: world Z 둘레
    horiz = _unit(np.cross([0, 0, 1], a))                 # a 에 수직인 수평축
    if np.linalg.norm(horiz) < 1e-6:                      # a 가 수직이면 X 로 대체
        horiz = np.array([1.0, 0.0, 0.0])
    a = _rot_axis(horiz, theta) @ a                       # 고각: 위/아래로 기울임
    return _unit(a)


def gaze_rotation(a, roll, approach_axis=(0.0, -1.0, 0.0)):
    """접근방향 a 를 바라보는 TCP 회전행렬 R(=[x|y|z] 열, tcp→world).

    제약: 그리퍼 **접근축(tcp 로컬, approach_axis)** 이 world 상 a 와 정렬.
    나머지 자유도(접근축 둘레 롤)는 roll 로 지정.

    구현: 먼저 접근축=−Y 기준의 gaze 프레임 G(그리퍼 손끝을 −Y 로 가정)를 만든 뒤,
    실제 접근축을 −Y 로 보내는 상수회전 M 을 곱한다 → R=G·M 이면 R·approach_axis=a
    가 (그리퍼 종류와 무관하게) 성립. RG2 는 approach_axis=(0,−1,0) → M=I(하위호환).
    """
    a = _unit(a)
    y = -a                                                # (−Y 기준) tcp +Y 의 world 방향
    up = np.array([0.0, 0.0, 1.0])
    # 손가락축(−Y 기준 tcp X)은 수평, tcp Z('위')는 world 위쪽.
    x = np.cross(y, up)
    if np.linalg.norm(x) < 1e-6:                          # y 가 수직이면 X 로 대체
        x = np.cross(y, np.array([1.0, 0.0, 0.0]))
    x = _unit(x)
    z = _unit(np.cross(x, y))                             # 우수좌표계 z=x×y (위쪽)
    R0 = np.column_stack([x, y, z])
    Ry = np.array([[math.cos(roll), 0, math.sin(roll)],   # 롤 = 접근축 둘레 회전
                   [0, 1, 0],
                   [-math.sin(roll), 0, math.cos(roll)]])
    G = R0 @ Ry
    # 실제 그리퍼 접근축을 기준(−Y)으로 보내는 상수회전 M → 그리퍼 불문 일반화
    M = _rot_u_to_v(np.asarray(approach_axis, float), np.array([0.0, -1.0, 0.0]))
    return G @ M


def mat_to_quat(R):
    """회전행렬 → 쿼터니언 [x,y,z,w]."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / (np.linalg.norm(q) + 1e-12)


# 후보 = (phi, theta, psi, d).  prior_cost = 명목에서 벗어난 정도(관절해 무관).
class Candidate:
    __slots__ = ("phi", "theta", "psi", "d", "prior")

    def __init__(self, phi, theta, psi, d, w_ang, w_roll, w_d, d0):
        self.phi, self.theta, self.psi, self.d = phi, theta, psi, d
        self.prior = (w_ang * (abs(phi) + abs(theta))
                      + w_roll * abs(psi) + w_d * abs(d - d0))


def build_candidates(phis, thetas, psis, ds, w_ang, w_roll, w_d, d0):
    """그리드 후보를 prior_cost 오름차순으로. → 명목(0,0,0,d0)이 맨 앞."""
    cands = [Candidate(math.radians(p), math.radians(t), math.radians(s), d,
                       w_ang, w_roll, w_d, d0)
             for p in phis for t in thetas for s in psis for d in ds]
    cands.sort(key=lambda c: c.prior)
    return cands


# ───────────────────────────────── ROS 노드 ─────────────────────────────────
class PregraspPose(Node):
    def __init__(self):
        super().__init__("pregrasp_pose")

        # ---- 파라미터 ----
        self.declare_parameter("target", [float("nan")] * 3)   # [x,y,z] world (미지정=NaN)
        self.declare_parameter("target_index", 0)              # target 미지정 시 몇번째 열매
        self.declare_parameter("obstacles_file", "")           # 비우면 description 기본 yaml
        self.declare_parameter("fruit_radius", 0.035)
        self.declare_parameter("standoff", 0.15)               # 열매 표면→TCP (튜너 조절)
        self.declare_parameter("approach_yaw_deg", float("nan"))  # 빈값=base→target 자동
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_link", "link0")
        self.declare_parameter("group", "arm")
        self.declare_parameter("ik_link", "tcp")
        self.declare_parameter("ik_timeout", 0.1)
        # 그리퍼 접근축(tcp 로컬). 'auto'=URDF/SRDF 에서 손끝 방향 자동감지(모델 불문).
        #  또는 "x,y,z" 로 직접 지정. RG2 는 auto→(0,-1,0).
        self.declare_parameter("approach_axis", "auto")
        # 샘플링 그리드 (1차 기본 = 명목 중심 소량. 넓히면 도달률↑)
        self.declare_parameter("sample_phi_deg", [0.0, -20.0, 20.0, -40.0, 40.0])
        self.declare_parameter("sample_theta_deg", [0.0, -15.0, 15.0])
        self.declare_parameter("sample_psi_deg", [0.0])
        self.declare_parameter("sample_standoff", [])          # 빈값=[standoff]
        # 비용 가중치
        self.declare_parameter("w_ang", 1.0)      # 방위/고각 벗어남
        self.declare_parameter("w_roll", 0.5)     # 롤 벗어남
        self.declare_parameter("w_d", 2.0)        # standoff 벗어남
        self.declare_parameter("w_center", 3.0)   # 관절 중앙성(특이자세 회피 근사)
        self.declare_parameter("period", 2.0)     # 재평가 주기(장애물 리로드 반영)

        gp = self.get_parameter
        self.world = gp("world_frame").value
        self.base_link = gp("base_link").value
        self.group = gp("group").value
        self.ik_link = gp("ik_link").value

        if not _HAVE_MOVEIT:
            self.get_logger().error(
                f"moveit_msgs import 실패 → IK 불가: {_MOVEIT_ERR}. "
                "`source install/setup.bash` 후 move_group 실행 필요.")
            raise SystemExit(1)

        # ---- TF / 현재 관절(시드) ----
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.cur_js = None
        self.create_subscription(JointState, "joint_states", self._on_js, 10)

        # ---- 마커에서 target 자동선택용 구독(래치) ----
        latched = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_targets = []      # [(name, np.array xyz, radius)]
        self.create_subscription(MarkerArray, "obstacle_markers",
                                 self._on_markers, latched)

        # ---- 출력 ----
        self.mpub = self.create_publisher(MarkerArray, "pregrasp_markers", latched)
        self.spub = self.create_publisher(DisplayRobotState, "pregrasp_robot_state",
                                          latched)

        # ---- IK 서비스 ----
        self.ik_cli = self.create_client(GetPositionIK, "compute_ik")
        self.get_logger().info("compute_ik 서비스 대기중…")
        self.ik_cli.wait_for_service()
        self.get_logger().info("compute_ik 연결됨.")

        self._joint_limits = self._fetch_joint_limits()   # {name:(lo,hi)} best-effort
        self.approach_axis = self._determine_approach_axis()
        self.get_logger().info(
            f"그리퍼 접근축(tcp 로컬) = {[round(v,3) for v in self.approach_axis]}")
        self.period = float(gp("period").value)
        # ※ solve_once 는 내부에서 동기 서비스(compute_ik)를 호출하므로 타이머 콜백이
        #   아니라 main 루프에서 직접 구동한다(콜백 안에서 spin 하면 충돌).
        self.get_logger().info("pregrasp_pose 시작 — 첫 해 계산까지 잠시.")

    # ---------- 콜백 ----------
    def _on_js(self, msg):
        self.cur_js = msg

    def _on_markers(self, arr):
        """'[목표]' 라벨 마커 ↔ 같은 id 의 obstacles 구(球)를 짝지어 target 목록 구성."""
        spheres = {}   # id -> (name, xyz, radius)
        labels = {}    # id -> name(without suffix)
        for m in arr.markers:
            if m.ns == "obstacles" and m.type == Marker.SPHERE:
                spheres[m.id] = (np.array([m.pose.position.x, m.pose.position.y,
                                           m.pose.position.z]), m.scale.x * 0.5)
            elif m.ns == "obstacle_labels" and "[목표]" in (m.text or ""):
                labels[m.id] = m.text.replace(" [목표]", "")
        tl = []
        for mid, name in labels.items():
            if mid in spheres:
                xyz, r = spheres[mid]
                tl.append((name, xyz, r))
        self.marker_targets = tl

    # ---------- 관절 한계(관절중앙성 비용용) ----------
    def _get_str_param(self, node, param):
        """다른 노드의 문자열 파라미터를 GetParameters 로 조회(best-effort)."""
        try:
            from rcl_interfaces.srv import GetParameters
            cli = self.create_client(GetParameters, f"/{node}/get_parameters")
            if not cli.wait_for_service(timeout_sec=3.0):
                return None
            fut = cli.call_async(GetParameters.Request(names=[param]))
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            if fut.result() is None or not fut.result().values:
                return None
            return fut.result().values[0].string_value or None
        except Exception:
            return None

    def _determine_approach_axis(self):
        """그리퍼 접근축(tcp 로컬). param='auto' 면 URDF/SRDF 로 자동감지, 아니면 "x,y,z"."""
        raw = str(self.get_parameter("approach_axis").value).strip().lower()
        if raw not in ("", "auto"):
            try:
                v = [float(x) for x in raw.replace("[", "").replace("]", "").split(",")]
                if len(v) == 3:
                    return v
            except ValueError:
                pass
            self.get_logger().warn(f"approach_axis 파싱 실패('{raw}') → auto 시도")
        urdf = self._get_str_param("robot_state_publisher", "robot_description")
        srdf = self._get_str_param("move_group", "robot_description_semantic")
        if urdf and srdf:
            try:
                ri = _import_ri()
                joints, c2j = ri.parse_urdf(urdf)
                ax = ri.detect_approach_axis(srdf, joints, c2j, self.ik_link)
                if ax:
                    return ax
            except Exception as e:
                self.get_logger().warn(f"접근축 자동감지 실패({e}) → 기본 −Y")
        else:
            self.get_logger().warn("URDF/SRDF 조회 실패 → 접근축 기본 −Y")
        return [0.0, -1.0, 0.0]

    def _fetch_joint_limits(self):
        """robot_state_publisher 의 robot_description 파라미터에서 revolute 한계 파싱(best-effort)."""
        xml = self._get_str_param("robot_state_publisher", "robot_description")
        if not xml:
            return {}
        import re
        lims = {}
        for jm in re.finditer(r'<joint\b[^>]*name="([^"]+)"[^>]*type="revolute"(.*?)</joint>',
                              xml, re.S):
            name, body = jm.group(1), jm.group(2)
            lm = re.search(r'<limit\b[^>]*lower="([-\d.eE+]+)"[^>]*upper="([-\d.eE+]+)"',
                           body) or re.search(
                           r'<limit\b[^>]*upper="([-\d.eE+]+)"[^>]*lower="([-\d.eE+]+)"', body)
            if lm:
                try:
                    a, b = float(lm.group(1)), float(lm.group(2))
                    lims[name] = (min(a, b), max(a, b))
                except ValueError:
                    pass
        return lims

    # ---------- 목표/베이스 조회 ----------
    def _load_yaml_targets(self):
        """obstacle_publisher 를 임포트해 obstacles.yaml 을 펼쳐 kind==target 열매 열거.

        마커(nolabel 이라 라벨 없음)에 의존하지 않고 단일 진실원(yaml)에서 직접 읽는다.
        crop_tuner 로 yaml 이 바뀌면 다음 호출에서 자동 반영.
        """
        import importlib.util
        import os
        if getattr(self, "_op_mod", None) is None:
            p = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "obstacle_publisher.py")
            spec = importlib.util.spec_from_file_location("_obstacle_publisher", p)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            self._op_mod = mod
        op = self._op_mod
        path = self.get_parameter("obstacles_file").value or op.default_yaml()
        try:
            import yaml
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warn(f"obstacles.yaml 로드 실패({e})",
                                   throttle_duration_sec=10.0)
            return []
        try:
            op.expand_crops(data)
        except Exception as e:
            self.get_logger().warn(f"expand_crops 실패({e})", throttle_duration_sec=10.0)
        out = []
        for o in data.get("obstacles", []):
            if o.get("kind") != "target":
                continue
            xyz = (o.get("pose", {}) or {}).get("xyz")
            if not xyz:
                xyz = [o.get("x", 0.0), o.get("y", 0.0), o.get("z", 0.0)]
            out.append((o["name"], np.array([float(v) for v in xyz]),
                        float(o.get("radius", 0.035))))
        return out

    def _resolve_target(self):
        t = self.get_parameter("target").value
        r = float(self.get_parameter("fruit_radius").value)
        if t and len(t) == 3 and not any(math.isnan(float(v)) for v in t):
            return "param_target", np.array([float(v) for v in t]), r
        targets = self._load_yaml_targets() or self.marker_targets
        if targets:
            idx = int(self.get_parameter("target_index").value)
            idx = max(0, min(idx, len(targets) - 1))
            return targets[idx]
        return None

    def _base_xy(self):
        """world 상 base_link(link0) 위치 → 접근 명목방향(수평 base→target)용."""
        try:
            tf = self.tf_buf.lookup_transform(self.world, self.base_link,
                                              rclpy.time.Time())
            return np.array([tf.transform.translation.x,
                             tf.transform.translation.y])
        except Exception:
            return None

    # ---------- IK 1회 ----------
    def _call_ik(self, pos, quat):
        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = self.group
        ik.ik_link_name = self.ik_link
        ik.avoid_collisions = True                        # 자가충돌 + 환경충돌
        ps = PoseStamped()
        ps.header.frame_id = self.world
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pos)
        ps.pose.orientation.x, ps.pose.orientation.y, \
            ps.pose.orientation.z, ps.pose.orientation.w = map(float, quat)
        ik.pose_stamped = ps
        if self.cur_js is not None:                       # 현재자세를 시드로
            rs = RobotState()
            rs.joint_state = self.cur_js
            ik.robot_state = rs
        to = float(self.get_parameter("ik_timeout").value)
        ik.timeout = DurationMsg(sec=int(to), nanosec=int((to % 1.0) * 1e9))
        req.ik_request = ik
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=to + 1.0)
        res = fut.result()
        if res is None or res.error_code.val != 1:        # 1 = SUCCESS
            return None
        return res.solution.joint_state

    def _center_cost(self, js):
        lims = self._joint_limits
        if not lims:
            return 0.0
        acc, n = 0.0, 0
        for name, val in zip(js.name, js.position):
            if name in lims:
                lo, hi = lims[name]
                if hi - lo > 1e-6:
                    mid = 0.5 * (lo + hi)
                    acc += ((val - mid) / (0.5 * (hi - lo))) ** 2
                    n += 1
        return acc / n if n else 0.0

    # ---------- 메인: 후보 샘플링 → IK/충돌 → 선택 ----------
    def solve_once(self):
        tgt = self._resolve_target()
        if tgt is None:
            self.get_logger().warn(
                "목표 미지정 — `-p target:=[x,y,z]` 지정하거나 /obstacle_markers "
                "의 '[목표]' 열매가 발행되길 기다리는 중.", throttle_duration_sec=10.0)
            return
        name, p_fruit, r = tgt
        d0 = float(self.get_parameter("standoff").value)

        # 명목 접근방향 a0 = 통로(로봇)쪽 수평 = horizontal(target − base)
        yaw = float(self.get_parameter("approach_yaw_deg").value)
        if not math.isnan(yaw):
            a0 = np.array([math.cos(math.radians(yaw)),
                           math.sin(math.radians(yaw)), 0.0])
        else:
            bxy = self._base_xy()
            if bxy is None:
                self.get_logger().warn("base TF 없음 — approach_yaw_deg 지정 필요.",
                                       throttle_duration_sec=10.0)
                return
            hv = p_fruit[:2] - bxy
            if np.linalg.norm(hv) < 1e-6:
                hv = np.array([1.0, 0.0])
            a0 = np.array([hv[0], hv[1], 0.0])
        a0 = _unit(a0)

        gp = self.get_parameter
        ds = list(gp("sample_standoff").value) or [d0]
        cands = build_candidates(
            list(gp("sample_phi_deg").value), list(gp("sample_theta_deg").value),
            list(gp("sample_psi_deg").value), ds,
            float(gp("w_ang").value), float(gp("w_roll").value),
            float(gp("w_d").value), d0)
        w_center = float(gp("w_center").value)

        best = None   # (total_cost, cand, a, pos, quat, js)
        tried = feasible = 0
        for c in cands:
            tried += 1
            a = approach_dir(a0, c.phi, c.theta)
            R = gaze_rotation(a, c.psi, self.approach_axis)
            quat = mat_to_quat(R)
            p_pre = p_fruit - a * (c.d + r)               # 열매 '표면'에서 d 만큼
            js = self._call_ik(p_pre, quat)
            if js is None:
                continue
            feasible += 1
            total = c.prior + w_center * self._center_cost(js)
            if best is None or total < best[0]:
                best = (total, c, a, p_pre, quat, js)

        if best is None:
            self.get_logger().warn(
                f"[{name}] 도달 불가 — 후보 {tried}개 모두 IK/충돌 실패. "
                f"standoff↑ 또는 sample_phi/theta 범위↑, base_placement 로 로봇 이동 검토.")
            self._publish_markers(p_fruit, r, None, None, None, reachable=False)
            return

        total, c, a, p_pre, quat, js = best
        deg = math.degrees
        self.get_logger().info(
            f"[{name}] pre-grasp 해 채택 — φ={deg(c.phi):+.0f}° θ={deg(c.theta):+.0f}° "
            f"ψ={deg(c.psi):+.0f}° d={c.d:.3f}m | 후보 {feasible}/{tried} 통과 | "
            f"cost={total:.3f}\n  joints=" +
            ", ".join(f"{n}={v:+.3f}" for n, v in zip(js.name, js.position)))
        self._publish_markers(p_fruit, r, p_pre, a, js, reachable=True)
        self._publish_state(js)

    # ---------- 시각화 ----------
    def _publish_state(self, js):
        msg = DisplayRobotState()
        msg.state.joint_state = js
        self.spub.publish(msg)

    def _publish_markers(self, p_fruit, r, p_pre, a, js, reachable):
        arr = MarkerArray()
        frame = self.world

        def base(mid, ns, mtype):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            return m

        # 목표 열매(반투명 노랑=선택 강조)
        m = base(0, "pregrasp", Marker.SPHERE)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p_fruit)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = float(2 * r * 1.15)
        m.color.r, m.color.g, m.color.b, m.color.a = (0.2, 1.0, 0.2, 0.5) \
            if reachable else (1.0, 0.3, 0.3, 0.5)
        arr.markers.append(m)

        if reachable and p_pre is not None:
            # standoff(대기)점
            sp = base(1, "pregrasp", Marker.SPHERE)
            sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = map(float, p_pre)
            sp.pose.orientation.w = 1.0
            sp.scale.x = sp.scale.y = sp.scale.z = 0.03
            sp.color.r, sp.color.g, sp.color.b, sp.color.a = 0.2, 0.5, 1.0, 0.9
            arr.markers.append(sp)
            # 접근 직선(대기점→열매) 화살표
            ar = base(2, "pregrasp", Marker.ARROW)
            ar.points = [Point(x=float(p_pre[0]), y=float(p_pre[1]), z=float(p_pre[2])),
                         Point(x=float(p_fruit[0]), y=float(p_fruit[1]),
                               z=float(p_fruit[2]))]
            ar.scale.x, ar.scale.y, ar.scale.z = 0.008, 0.02, 0.03
            ar.color.r, ar.color.g, ar.color.b, ar.color.a = 0.1, 0.9, 0.9, 0.95
            arr.markers.append(ar)
            # TCP 좌표축 삼각대(대기점에서): X=빨강 손가락축, -Y=초록 접근축, Z=파랑
            R = gaze_rotation(a, 0.0, getattr(self, "approach_axis", (0.0, -1.0, 0.0)))
            axes = [(R[:, 0], (1.0, 0.0, 0.0), "x"),
                    (-R[:, 1], (0.0, 1.0, 0.0), "approach(-Y)"),
                    (R[:, 2], (0.0, 0.0, 1.0), "z")]
            for k, (vec, col, _lbl) in enumerate(axes):
                ax = base(3 + k, "pregrasp", Marker.ARROW)
                tip = p_pre + _unit(vec) * 0.08
                ax.points = [Point(x=float(p_pre[0]), y=float(p_pre[1]),
                                   z=float(p_pre[2])),
                             Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2]))]
                ax.scale.x, ax.scale.y, ax.scale.z = 0.006, 0.012, 0.0
                ax.color.r, ax.color.g, ax.color.b = float(col[0]), float(col[1]), float(col[2])
                ax.color.a = 0.9
                arr.markers.append(ax)
        self.mpub.publish(arr)


def main():
    import time
    rclpy.init()
    try:
        node = PregraspPose()
    except SystemExit:
        rclpy.shutdown()
        return

    def pump(dur):
        """dur 초 동안 콜백만 처리(구독/TF 갱신)."""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < dur:
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        pump(1.5)                       # 워밍업: joint_states/markers/TF 수집
        while rclpy.ok():
            node.solve_once()           # 내부 동기 IK 호출 (spin 중이 아님 → 안전)
            pump(node.period)           # 다음 평가까지 콜백 처리
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
