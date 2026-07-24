#!/usr/bin/env python3
"""5주차 pre-grasp 데모 — 알고리즘이 이끄는 로봇 집기 동작(RViz 애니메이션).

pregrasp_pose.py 의 접근-자세 추정 알고리즘을 그대로 써서, 목표 토마토에 대해
로봇이 다음 시퀀스를 수행하는 것을 보여준다(컨트롤러 없이 /joint_states 로 재생):

  ① home → pre-grasp(열매 바라보는 자세)   : MoveIt OMPL 계획(실패 시 관절보간)
  ② pre-grasp → grasp(직선 접근)           : MoveIt Cartesian(실패 시 관절보간)
  ③ 그리퍼 닫기(파지)
  ④ grasp → pre-grasp(후퇴)  →  home

⚠ execute(실제 컨트롤러)는 6주차. 여기서는 계획된 궤적을 /joint_states 로 '재생'만 한다
   (jsp_gui 대신 이 노드가 유일한 /joint_states 발행자 — 데모 launch 는 jsp_gui 를 뺀다).

핵심: pre-grasp 자세는 pregrasp_pose 의 후보 샘플링 + /compute_ik(avoid_collisions)
로 구한다(자가충돌+환경충돌 없는 '바라보는' 자세). 접근축 = TCP 로컬 −Y(FK 실측).
"""

import importlib.util
import math
import os
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from moveit_msgs.srv import (GetPositionIK, GetMotionPlan, GetCartesianPath,
                             GetPlanningScene, ApplyPlanningScene, GetStateValidity)
from moveit_msgs.msg import (PositionIKRequest, RobotState, MotionPlanRequest,
                             Constraints, JointConstraint, DisplayRobotState,
                             PlanningScene, PlanningSceneComponents, CollisionObject)
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Empty


def _import_sibling(mod_name, file_name):
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
    spec = importlib.util.spec_from_file_location(mod_name, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# pregrasp_pose 의 순수 기하/후보 함수 재사용(단일 진실원)
PG = _import_sibling("_pregrasp_pose", "pregrasp_pose.py")


# robot_introspect(형제): SRDF/URDF 에서 관절·접근축 자동 유도(모델 불문)
RI = _import_sibling("_robot_introspect", "robot_introspect.py")


class PregraspDemo(Node):
    # ↓ 폴백 기본값(RB5+RG2). __init__ 에서 SRDF/URDF introspection 으로 자동 대체됨.
    ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
    FINGERS = ["rg2_finger_joint1", "rg2_finger_joint2"]

    def __init__(self):
        super().__init__("pregrasp_demo")
        # ---- 파라미터 ----
        self.declare_parameter("target", [float("nan")] * 3)
        self.declare_parameter("target_index", 0)
        self.declare_parameter("auto_reachable", True)   # 현 위치서 도달가능 열매 자동선택
        self.declare_parameter("max_scan", 12)           # 자동선택 시 가까운 열매 몇개까지 시도
        self.declare_parameter("scan_all", False)        # True=데모 대신 전체 열매 도달 리포트 후 종료
        self.declare_parameter("diag_straight", False)   # True=선택 열매의 접근각별 직선 fraction 진단 후 종료
        self.declare_parameter("bench", False)           # True=조건별 비교실험(ablation) 후 종료
        self.declare_parameter("verify_region", False)   # True=구 영역 효과 전/후 측정 후 종료 [Stage 5]
        self.declare_parameter("bench_n", 8)             # 비교실험 표본 열매 수(0=도달 가능 전부)
        # 표본 고정용(재현성): 열매 이름을 직접 주면 그 열매들만 쓴다. IK 가 확률적이라
        # 매 실행마다 '도달 가능' 집합이 조금씩 달라지므로, 조건 간 비교는 같은 표본에서 해야 한다.
        self.declare_parameter("bench_targets", [""])
        self.declare_parameter("obstacles_file", "")
        # Stage 4: 목표 출처 — 'yaml'(설계값) 또는 'perception'(카메라 인지 결과)
        self.declare_parameter("target_source", "yaml")
        self.declare_parameter("targets_topic", "detected_fruits")
        self.declare_parameter("targets_wait", 20.0)   # 첫 인지 결과 대기 한도[s]
        self.declare_parameter("fruit_radius", 0.035)
        # Stage 5: 접근 시 충돌 허용 방식
        #   region — 목표 열매 주변 **구 영역**을 허용(이름표 불필요 → 인지 타깃/옥토맵에서도 동작)
        #   stalk  — 이름 기반(fruit_… → rachis_…). 설계값 장면에서만 가능(기존 방식, 비교용)
        #   none   — 아무것도 허용하지 않음(비교실험 기준선)
        self.declare_parameter("acm_mode", "region")
        # ρ = 열매반경 + 여유. 실측(이 온실 모델): 목표 화방대 표면이 열매 중심에서 5.8cm
        #   → 여유 3cm(ρ=6.5cm)면 자기 화방대를 포함하고 주 줄기(15.4cm)는 안 건드린다.
        self.declare_parameter("region_margin", 0.03)
        # 구에 닿아도 이 크기(중심→최원점)를 넘는 객체는 허용하지 않는다 — 거터·레일 등
        # 구조물이 통째로 허용되는 것을 막는다(ACM 은 부분 허용이 안 되므로 필요).
        self.declare_parameter("region_max_object", 0.15)
        # 옥토맵 초기화 후 재구축 대기[s] (센서 max_update_rate 1Hz → 최소 1.5s 이상)
        self.declare_parameter("region_octomap_wait", 2.0)
        # ρ 스윕(검증용): 열매반경에 더할 여유 목록[m]. 비우면 단일 ρ 로 전/후만 잰다.
        self.declare_parameter("region_margin_sweep", [0.0])
        # 구 적용 후 옥토맵이 침식돼 효과가 나타날 때까지 기다리는 한도[s]
        self.declare_parameter("region_settle_timeout", 60.0)
        # 구 배치 후 옥토맵 전체 초기화 여부. 기본 off — PointCloudOctomapUpdater 는 마스크된
        # 점(model_cells)을 매 프레임 **free 로 갱신**하므로 구 안쪽은 저절로 비워진다.
        # 전체 초기화는 누적 관측을 날려 지도를 성기게 만들어 측정을 오염시킨다(실측 확인).
        self.declare_parameter("region_clear_octomap", False)
        self.declare_parameter("standoff", 0.15)
        self.declare_parameter("grasp_offset", 0.10)     # 파지 시 열매 중심 앞 TCP 정지거리
        self.declare_parameter("approach_yaw_deg", float("nan"))
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_link", "link0")
        self.declare_parameter("group", "arm")
        self.declare_parameter("ik_link", "tcp")
        self.declare_parameter("ik_timeout", 0.1)
        self.declare_parameter("gripper_group", "gripper")  # SRDF 그리퍼 그룹명
        self.declare_parameter("approach_axis", "auto")     # 'auto'=SRDF/URDF 자동감지 or "x,y,z"
        self.declare_parameter("min_scene_objects", 10)   # 배경 로드 확인 최소 collision object 수
        self.declare_parameter("scene_wait", 15.0)         # 배경 로드 대기 한도(초)
        self.declare_parameter("sample_phi_deg", [0.0, -20.0, 20.0, -40.0, 40.0])
        self.declare_parameter("sample_theta_deg", [0.0, -15.0, 15.0, -30.0])
        self.declare_parameter("sample_psi_deg", [0.0])
        # ⚠ RG2 실측(FK): 관절 1.18=벌림(open, 152mm) · 0=닫힘(close, 34mm). 값이 직관과 반대.
        self.declare_parameter("gripper_open", 1.18)     # 파지 전 벌린 상태
        self.declare_parameter("gripper_close", 0.35)    # 파지(대과 토마토 Ø70mm 물기)
        self.declare_parameter("plan_time", 5.0)
        # 데모 재생 시간(초) — 보기 좋게 각 구간을 늘림
        self.declare_parameter("dur_approach_plan", 3.5)  # home→pre-grasp
        self.declare_parameter("dur_approach_line", 2.0)  # pre-grasp→grasp
        self.declare_parameter("dur_gripper", 1.2)
        self.declare_parameter("dur_retreat", 2.0)
        self.declare_parameter("dur_home", 3.0)
        self.declare_parameter("pause", 1.2)              # 구간 사이 정지
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("loop", True)

        gp = self.get_parameter
        self.world = gp("world_frame").value
        self.group = gp("group").value
        self.ik_link = gp("ik_link").value
        self.rate = float(gp("rate").value)

        # ---- 현재 관절 상태(이 노드가 유일 발행자) : home + 그리퍼 벌림(open) ----
        self.cur = {j: 0.0 for j in self.ARM}
        _go = float(gp("gripper_open").value)
        self.cur.update({f: _go for f in self.FINGERS})

        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.js_pub = self.create_publisher(JointState, "joint_states", 10)
        self.mk_pub = self.create_publisher(MarkerArray, "pregrasp_markers", latched)
        self.state_pub = self.create_publisher(DisplayRobotState, "pregrasp_robot_state",
                                               latched)

        # 발행 시작(RViz TF 확보) — 서비스 대기 동안에도 home 자세 유지
        self._publish_js()

        # ---- 서비스 ----
        self.ik = self.create_client(GetPositionIK, "compute_ik")
        self.plan = self.create_client(GetMotionPlan, "plan_kinematic_path")
        self.cart = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.scene = self.create_client(GetPlanningScene, "get_planning_scene")
        self.apply_scene = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        for cli, nm in ((self.ik, "compute_ik"), (self.plan, "plan_kinematic_path"),
                        (self.cart, "compute_cartesian_path"),
                        (self.scene, "get_planning_scene")):
            self.get_logger().info(f"{nm} 대기중…")
            cli.wait_for_service()
        self.get_logger().info("MoveIt 서비스 연결됨.")

        self._sv = self.create_client(GetStateValidity, "check_state_validity")
        if not self._sv.wait_for_service(timeout_sec=3.0):
            self._sv = None
        # Stage 5: 공간 기반 ACM 상태
        self._clear_octomap = self.create_client(Empty, "clear_octomap")
        self._acm_mode = str(gp("acm_mode").value).strip().lower()
        self._tgt_geom = {}          # name -> (center, radius) : 이름→기하 조회(구 영역 배치용)
        self._zone_at = None         # 현재 배치된 구 영역 중심(중복 적용 방지)
        self._zone_allowed = []      # 구 영역이 ACM 허용시킨 명명 객체들(되돌리기용)
        self._op = _import_sibling("_obstacle_publisher", "obstacle_publisher.py")
        # ★ B: 팔/그리퍼 관절 이름 + A: 접근축을 SRDF/URDF 에서 자동 유도(모델 불문)
        self._setup_model()
        # ★ 배경(온실 구조·줄기) 충돌체크 보장: 첫 모션 계획 전에 planning scene 에
        #   장애물(CollisionObject)이 실제로 로드될 때까지 기다린다. (안 기다리면 첫
        #   사이클이 빈 scene 에서 계획돼 배경을 안 피할 수 있다.)
        self._wait_scene(min_objects=int(self.get_parameter("min_scene_objects").value),
                         timeout=float(self.get_parameter("scene_wait").value))

    def _get_str_param(self, node, param):
        """다른 노드의 문자열 파라미터 조회(URDF/SRDF 획득용)."""
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

    def _setup_model(self):
        """B: 팔/그리퍼 관절 이름을 SRDF 그룹에서, A: 접근축을 URDF/SRDF 로 자동 유도.
        실패하면 클래스 폴백(RB5+RG2) 유지. param 으로 덮어쓰기 가능."""
        urdf = self._get_str_param("robot_state_publisher", "robot_description")
        srdf = self._get_str_param("move_group", "robot_description_semantic")
        info = None
        if urdf and srdf:
            try:
                info = RI.playback_joints(srdf, urdf, self.group,
                                          self.get_parameter("gripper_group").value)
                if info["arm"]:
                    self.ARM = info["arm"]
                if info["gripper_all"]:
                    self.FINGERS = info["gripper_all"]
            except Exception as e:
                self.get_logger().warn(f"관절 introspection 실패({e}) → 폴백 유지")
        else:
            self.get_logger().warn("URDF/SRDF 조회 실패 → 관절/접근축 폴백")
        # A: 접근축
        raw = str(self.get_parameter("approach_axis").value).strip().lower()
        self.approach_axis = [0.0, -1.0, 0.0]
        if raw not in ("", "auto"):
            try:
                v = [float(x) for x in raw.replace("[", "").replace("]", "").split(",")]
                if len(v) == 3:
                    self.approach_axis = v
            except ValueError:
                self.get_logger().warn(f"approach_axis 파싱 실패('{raw}') → auto")
                raw = "auto"
        if raw in ("", "auto") and info is not None:
            try:
                ax = RI.detect_approach_axis(srdf, info["joints"],
                                             info["child_to_joint"], self.ik_link)
                if ax:
                    self.approach_axis = ax
            except Exception as e:
                self.get_logger().warn(f"접근축 자동감지 실패({e}) → 기본 −Y")
        # 관절 이름이 바뀌었을 수 있으니 현재자세 dict 재구성(그리퍼는 벌림)
        gopen = float(self.get_parameter("gripper_open").value)
        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        self.get_logger().info(
            f"모델 자동설정 — arm{self.ARM} gripper{self.FINGERS} "
            f"접근축(tcp){[round(v,3) for v in self.approach_axis]}")

    def _scene_object_count(self):
        """planning scene 의 world collision object 개수(배경 로드 확인용)."""
        req = GetPlanningScene.Request()
        req.components.components = 1023
        res = self._call(self.scene, req, 2.0)
        if res is None:
            return -1
        return len(res.scene.world.collision_objects)

    def _wait_scene(self, min_objects, timeout):
        import time
        t0 = time.time()
        n = -1
        while rclpy.ok() and time.time() - t0 < timeout:
            n = self._scene_object_count()
            if n >= min_objects:
                self.get_logger().info(f"배경 collision object {n}개 로드 확인 → 충돌체크 활성.")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.3)
        self.get_logger().warn(
            f"배경 collision object 대기 시간초과(현재 {n}개, 요구 {min_objects}). "
            "충돌체크가 불완전할 수 있음 — obstacle_publisher/타이밍 확인.")
        return False

    # ══════════════════ 유틸: 발행/재생 ══════════════════
    def _publish_js(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.cur.keys())
        js.position = [float(self.cur[j]) for j in js.name]
        self.js_pub.publish(js)

    def _set(self, joint_vals):
        for k, v in joint_vals.items():
            if k in self.cur:
                self.cur[k] = float(v)

    def _hold(self, sec):
        """sec 초 동안 현재 자세를 계속 발행(정지 구간)."""
        dt = 1.0 / self.rate
        t = 0.0
        while rclpy.ok() and t < sec:
            self._publish_js()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)
            t += dt

    def _play_waypoints(self, names, waypts, duration):
        """waypts = [pos_array,...] (각 array 는 names 순서). duration 초에 걸쳐 선형보간 재생."""
        if not waypts:
            return
        dt = 1.0 / self.rate
        nsteps = max(1, int(duration * self.rate))
        segs = len(waypts) - 1
        for s in range(nsteps + 1):
            u = s / nsteps                       # 0..1 전체 진행
            if segs <= 0:
                pos = waypts[0]
            else:
                f = u * segs
                i = min(int(f), segs - 1)
                w = f - i
                pos = [(1 - w) * waypts[i][k] + w * waypts[i + 1][k]
                       for k in range(len(names))]
            self._set({names[k]: pos[k] for k in range(len(names))})
            self._publish_js()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

    def _traj_to_waypts(self, joint_traj):
        names = list(joint_traj.joint_names)
        wp = [[float(v) for v in p.positions] for p in joint_traj.points]
        return names, wp

    # ══════════════════ 서비스 호출 ══════════════════
    def _call(self, cli, req, timeout):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def _robot_state(self):
        rs = RobotState()
        js = JointState()
        js.name = list(self.cur.keys())
        js.position = [float(self.cur[j]) for j in js.name]
        rs.joint_state = js
        return rs

    def solve_ik(self, pos, quat, avoid=True):
        req = GetPositionIK.Request()
        ikr = PositionIKRequest()
        ikr.group_name = self.group
        ikr.ik_link_name = self.ik_link
        ikr.avoid_collisions = avoid
        ps = PoseStamped()
        ps.header.frame_id = self.world
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, pos)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = map(float, quat)
        ikr.pose_stamped = ps
        ikr.robot_state = self._robot_state()
        to = float(self.get_parameter("ik_timeout").value)
        ikr.timeout = DurationMsg(sec=int(to), nanosec=int((to % 1) * 1e9))
        req.ik_request = ikr
        res = self._call(self.ik, req, to + 1.0)
        if res is None or res.error_code.val != 1:
            return None
        return {n: v for n, v in zip(res.solution.joint_state.name,
                                     res.solution.joint_state.position)}

    def plan_to(self, q_goal_arm):
        """현재→목표 관절(arm) OMPL 계획. 성공 시 (names, waypts), 실패 시 None."""
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = self.group
        mpr.start_state = self._robot_state()
        mpr.num_planning_attempts = 5
        mpr.allowed_planning_time = float(self.get_parameter("plan_time").value)
        c = Constraints()
        for j in self.ARM:
            jc = JointConstraint()
            jc.joint_name = j
            jc.position = float(q_goal_arm[j])
            jc.tolerance_above = jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        mpr.goal_constraints.append(c)
        req.motion_plan_request = mpr
        res = self._call(self.plan, req, mpr.allowed_planning_time + 2.0)
        if res is None or res.motion_plan_response.error_code.val != 1:
            return None
        return self._traj_to_waypts(res.motion_plan_response.trajectory.joint_trajectory)

    def cartesian_to(self, pose_goal):
        """현재→목표 TCP pose 직선(Cartesian) 경로. (names, waypts, fraction) 또는 None."""
        req = GetCartesianPath.Request()
        req.header.frame_id = self.world
        req.start_state = self._robot_state()
        req.group_name = self.group
        req.link_name = self.ik_link
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.waypoints = [pose_goal]
        res = self._call(self.cart, req, 5.0)
        if res is None or not res.solution.joint_trajectory.points:
            return None
        names, wp = self._traj_to_waypts(res.solution.joint_trajectory)
        return names, wp, float(res.fraction)

    # ── 5주차 2차: 목표 화방대(수확 대상 줄기) 충돌 허용 ──────────────────
    @staticmethod
    def _stalk_of(fruit_name):
        """열매 이름 fruit_r{ri}_p{pi}_t{ti}_f{fi} → 그 화방대 rachis_r{ri}_p{pi}_t{ti}.
        열매는 자기 화방대(줄기 곁가지)에 매달려 있어, 그 화방대는 접근 시 불가피하게
        스친다 → '수확 대상 줄기'로 보고 접근 궤적에서만 충돌 제외. (주 줄기·다른 화방대는
        장애물 유지 → 진짜 회피.) 파싱 실패 시 None."""
        import re
        m = re.match(r"fruit_(r\d+_p\d+_t\d+)_f\d+$", str(fruit_name))
        return f"rachis_{m.group(1)}" if m else None

    def _allow_collision(self, obj_name):
        """planning scene ACM 에서 obj_name 을 '모든 링크와 충돌 무시'로 표시(default entry).
        obstacle_publisher 는 CollisionObject 만 재발행하고 ACM 은 안 건드리므로 유지된다.
        현재 ACM 을 받아 default_entry 에 추가 후 되돌려 적용(diff)."""
        if obj_name is None or self.apply_scene is None:
            return False
        if not self.apply_scene.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("apply_planning_scene 서비스 없음 → 화방대 충돌 허용 생략")
            return False
        req = GetPlanningScene.Request()
        # ⚠ 상수 주의: ACM = 128(ALLOWED_COLLISION_MATRIX). 2 는 ROBOT_STATE 라
        #   빈 ACM 이 돌아오고, 그걸 diff 로 되돌리면 **아무 효과가 없다**(조용한 무효).
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        res = self._call(self.scene, req, 2.0)
        if res is None:
            return False
        acm = res.scene.allowed_collision_matrix
        if obj_name in acm.default_entry_names:
            i = list(acm.default_entry_names).index(obj_name)
            acm.default_entry_values[i] = True
        else:
            acm.default_entry_names.append(obj_name)
            acm.default_entry_values.append(True)
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True     # ⚠ 안 세우면 빈 robot_state 를 적용하려다 실패를 반환한다
        ps.allowed_collision_matrix = acm
        areq = ApplyPlanningScene.Request(scene=ps)
        ares = self._call(self.apply_scene, areq, 2.0)
        ok = ares is not None and ares.success
        if ok:
            self.get_logger().info(f"ACM: '{obj_name}' 충돌 허용(수확 대상 줄기) — 접근 궤적에서 제외.")
        return ok

    # ── Stage 5: 공간(구 영역) 기반 충돌 허용 ────────────────────────────
    #  이름 기반(_stalk_of)은 설계값 장면에서만 통한다. 센싱 장면의 장애물은 `<octomap>`
    #  하나뿐이고 인지 열매(det_N)엔 이름표가 없어 "목표 화방대만 제외"가 성립하지 않는다.
    #  → 목표 열매 중심 반경 ρ 의 **구 영역**을 수확 작업 공간으로 보고 그 안만 허용한다.
    #  구현: 구 모양 CollisionObject 를 장면에 넣고 ACM 에서 그것만 허용한다. 이때
    #  MoveIt PlanningSceneMonitor 가 새 월드 객체를 옥토맵 센서 마스크에 등록하므로
    #  (excludeWorldObjectFromOctree) **구 안의 센서 점이 옥토맵에 들어오지 않는다** →
    #  이름 없는 옥토맵에서도 구 영역만 통과 가능해진다. 명명 객체(설계값 장면)는 구와
    #  실제로 겹치는 것만 골라 ACM 허용 → 같은 규칙이 두 장면에 그대로 적용된다.
    ZONE_ID = "harvest_zone"

    @staticmethod
    def _quat_mat(q):
        x, y, z, w = [float(v) for v in q]
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    @classmethod
    def _primitive_dist(cls, p, prim, pose):
        """점 p 에서 primitive(pose 기준) 표면까지의 최단거리(내부면 0). 겹침 판정용."""
        c = np.array([pose.position.x, pose.position.y, pose.position.z])
        R = cls._quat_mat([pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w])
        v = R.T @ (np.asarray(p, float) - c)          # 로컬 좌표
        d = list(prim.dimensions)
        if prim.type == SolidPrimitive.SPHERE:
            return max(0.0, float(np.linalg.norm(v)) - d[0])
        if prim.type == SolidPrimitive.BOX:
            h = np.array(d[:3]) / 2.0
            return float(np.linalg.norm(np.maximum(np.abs(v) - h, 0.0)))
        if prim.type == SolidPrimitive.CYLINDER:            # d = [height, radius]
            dz = max(abs(v[2]) - d[0] / 2.0, 0.0)
            dr = max(float(np.linalg.norm(v[:2])) - d[1], 0.0)
            return float(math.hypot(dr, dz))
        return float("inf")                                  # mesh 등은 판정 생략

    @staticmethod
    def _primitive_scale(prim):
        """primitive 의 대표 크기(중심에서 가장 먼 점까지) — '구조물 통째 허용' 방지용."""
        d = list(prim.dimensions)
        if prim.type == SolidPrimitive.SPHERE:
            return float(d[0])
        if prim.type == SolidPrimitive.BOX:
            return float(np.linalg.norm(np.array(d[:3]) / 2.0))
        if prim.type == SolidPrimitive.CYLINDER:
            return float(math.hypot(d[1], d[0] / 2.0))
        return float("inf")

    def _objects_in_region(self, center, rho):
        """구 영역과 실제로 겹치는 **명명 collision object** 이름 목록(옥토맵·구 자신 제외).

        ⚠ ACM default entry 는 그 객체를 **전부** 허용한다(구 안쪽만이 아님). 그래서 거터·레일
        같은 큰 구조물은 구에 살짝 닿기만 해도 통째로 허용돼 버린다(실측: 이 온실에서 거터
        앞면이 목표 열매 중심 5.9cm — 목표 화방대 5.8cm 와 거의 같다). → **구 영역 규모 이하로
        작은 객체만** 허용한다(`region_max_object`). 옥토맵 쪽은 센서 마스크가 구 안쪽 점만
        지우므로 이런 문제가 없다(진짜 공간 연산)."""
        lim = float(self.get_parameter("region_max_object").value)
        center = np.asarray(center, float)
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_NAMES |
                                     PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        res = self._call(self.scene, req, 3.0)
        if res is None:
            return []
        hit, big = [], []
        for co in res.scene.world.collision_objects:
            if co.id in (self.ZONE_ID, "<octomap>"):
                continue
            # ⚠ MoveIt 은 월드 객체를 **planning frame**(로봇 모델 루트=base_link)으로 변환해
            #   돌려준다. 열매 중심은 world 좌표라 그대로 비교하면 전부 빗나간다(실측: 겹침 0개).
            pc0 = self._point_in_frame(center, co.header.frame_id)
            # 객체 pose ∘ primitive pose (obstacle_publisher 는 primitive_poses 를 월드로 준다)
            oc = np.array([co.pose.position.x, co.pose.position.y, co.pose.position.z])
            oR = self._quat_mat([co.pose.orientation.x, co.pose.orientation.y,
                                 co.pose.orientation.z, co.pose.orientation.w])
            for prim, ppose in zip(co.primitives, co.primitive_poses):
                w = Pose()
                pc = oc + oR @ np.array([ppose.position.x, ppose.position.y,
                                         ppose.position.z])
                pR = oR @ self._quat_mat([ppose.orientation.x, ppose.orientation.y,
                                          ppose.orientation.z, ppose.orientation.w])
                w.position.x, w.position.y, w.position.z = map(float, pc)
                q = PG.mat_to_quat(pR)
                (w.orientation.x, w.orientation.y,
                 w.orientation.z, w.orientation.w) = map(float, q)
                if self._primitive_dist(pc0, prim, w) <= rho:
                    if self._primitive_scale(prim) > lim:
                        big.append(co.id)          # 구조물 — 통째 허용 금지(장애물 유지)
                    else:
                        hit.append(co.id)
                    break
        if big:
            self.get_logger().info(
                f"구 영역에 닿았지만 커서 제외(장애물 유지): {sorted(set(big))[:4]}")
        return hit

    def _ensure_tf(self):
        import tf2_ros
        if not hasattr(self, "_tf"):
            self._tf = tf2_ros.Buffer()
            self._tfl = tf2_ros.TransformListener(self._tf, self)
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.05)

    def _point_in_frame(self, p, frame):
        """world 좌표 p 를 frame 좌표로. 조회 실패/동일 프레임이면 그대로."""
        p = np.asarray(p, float)
        if not frame or frame.lstrip("/") == self.world.lstrip("/"):
            return p
        self._ensure_tf()
        try:
            tf = self._tf.lookup_transform(frame, self.world, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF {self.world}→{frame} 실패({e}) → 변환 없이 비교")
            return p
        t, q = tf.transform.translation, tf.transform.rotation
        return self._quat_mat([q.x, q.y, q.z, q.w]) @ p + np.array([t.x, t.y, t.z])

    def _apply_zone_object(self, center, rho, remove=False):
        """구 영역 CollisionObject 를 장면에 넣거나(ADD) 뺀다(REMOVE)."""
        if self.apply_scene is None:
            return False
        co = CollisionObject()
        co.header.frame_id = self.world
        co.id = self.ZONE_ID
        if remove:
            co.operation = CollisionObject.REMOVE
        else:
            co.operation = CollisionObject.ADD
            sp = SolidPrimitive()
            sp.type = SolidPrimitive.SPHERE
            sp.dimensions = [float(rho)]
            po = Pose()
            po.position.x, po.position.y, po.position.z = map(float, center)
            po.orientation.w = 1.0
            co.primitives.append(sp)
            co.primitive_poses.append(po)
        ps = PlanningScene()
        ps.is_diff = True
        # ⚠ robot_state.is_diff 를 안 세우면 MoveIt 이 빈 robot_state 를 적용하려다
        #   ApplyPlanningScene 이 success=False 를 돌려준다(객체는 들어가는데 실패로 보임).
        #   그러면 뒤따르는 ACM 허용이 생략돼 구가 '장애물'로 남는다 — 실측으로 확인한 함정.
        ps.robot_state.is_diff = True
        ps.world.collision_objects.append(co)
        self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 20.0)
        # ⚠ success 를 믿지 않는다: 이 장면은 옥토맵이 200KB 라 apply/get 응답이 자주 밀린다
        #   (타임아웃이어도 실제로는 반영된다). **장면을 재조회해** 실제 상태로 판정한다.
        for _ in range(6):
            names = self._world_object_names()
            if names is not None:
                return (self.ZONE_ID not in names) if remove else (self.ZONE_ID in names)
            time.sleep(1.0)
        return False

    def _world_object_names(self):
        """장면의 명명 객체 이름 목록(조회 실패 시 None)."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        res = self._call(self.scene, req, 15.0)
        return None if res is None else [o.id for o in res.scene.world.collision_objects]

    def _wait_octomap_stable(self, timeout=40.0):
        """옥토맵 크기가 안정될 때까지 대기 → 측정 기준선을 재현 가능하게 만든다."""
        t0, prev, same = time.time(), None, 0
        while time.time() - t0 < timeout:
            b = self._octomap_bytes()
            if b < 0:
                time.sleep(1.0)
                continue
            same = same + 1 if (prev is not None and b == prev) else 0
            prev = b
            if same >= 2:
                return b
            time.sleep(2.0)
        return prev if prev is not None else -1

    def _clear_zone(self, force=False):
        """구 영역 해제 — 객체 제거 + 그 때문에 허용했던 ACM 항목 되돌림.
        force=True 면 이 노드가 안 만든(이전 실행이 남긴) 구도 지운다."""
        if self._zone_at is None and not force:
            return
        self._apply_zone_object(None, 0.0, remove=True)   # 먼저 객체를 뺀 뒤에
        self._set_allow([self.ZONE_ID] + list(self._zone_allowed), False)   # 허용을 되돌린다
        self._zone_at, self._zone_allowed = None, []

    def _allow_region(self, center, rho):
        """목표 열매 중심 반경 ρ 구 영역을 '수확 작업 공간'으로 허용한다."""
        center = np.asarray(center, float)
        if self._zone_at is not None and float(np.linalg.norm(center - self._zone_at)) < 1e-6:
            return True                                    # 이미 같은 위치에 배치됨
        self._clear_zone()
        # ★ 순서가 중요하다: **ACM 허용을 먼저** 걸고 객체를 넣는다.
        #   반대로 하면 허용이 반영되기 전까지 구가 '열매 자리에 박힌 장애물'이 되고,
        #   그 사이 ACM 적용이 밀리면(응답 타임아웃) 조용히 장애물로 남는다 — 실측으로 겪은 함정.
        #   ACM default entry 는 아직 없는 이름에도 미리 걸어둘 수 있다.
        if not self._set_allow([self.ZONE_ID], True):
            self.get_logger().warn("구 영역 ACM 허용 실패 → 배치 취소")
            return False
        if not self._apply_zone_object(center, rho):
            self.get_logger().warn("구 영역 CollisionObject 배치 실패 → 공간 ACM 생략")
            self._set_allow([self.ZONE_ID], False)
            return False
        inside = self._objects_in_region(center, rho)
        # ★ 객체를 넣은 **뒤에 한 번 더** 허용한다. MoveIt 은 새 월드 객체가 들어오면 ACM 의
        #   쌍별(entry) 행/열을 그 객체에 대해 확장하는데, 미리 걸어둔 default entry 가 거기에
        #   전파되지 않아 **default 는 True 인데 쌍별은 불허**인 상태가 생긴다(실측: 파지 자세가
        #   `rg2_hand|harvest_zone` 으로 충돌 판정). 추가 후 재적용해야 쌍별까지 True 가 된다.
        if not self._set_allow([self.ZONE_ID] + inside, True, check_pairs=True):
            self.get_logger().warn("구 영역 ACM 허용(객체 추가 후 재적용) 실패")
            return False
        self._zone_at, self._zone_allowed = center, inside
        # 이미 쌓인 옥토맵 복셀은 마스크로 사라지지 않는다(마스크는 새로 들어오는 점만 거른다)
        # → 1회 초기화하면 다음 갱신부터 구 안이 빈 채로 재구축된다.
        if self._has_octomap():
            if bool(self.get_parameter("region_clear_octomap").value) and \
                    self._clear_octomap.wait_for_service(timeout_sec=1.0):
                self._call(self._clear_octomap, Empty.Request(), 3.0)
                self.get_logger().info("옥토맵 초기화 후 재구축(옵션)")
            # 센서 마스크가 구 안쪽 복셀을 free 로 지울 때까지 대기(업데이터 갱신 주기 의존)
            time.sleep(float(self.get_parameter("region_octomap_wait").value))
        self.get_logger().info(
            f"ACM(공간): 열매 중심 ({center[0]:.2f},{center[1]:.2f},{center[2]:.2f}) 반경 "
            f"{rho*100:.1f}cm 구 영역 허용 — 겹친 명명객체 {len(inside)}개"
            + (f" {inside[:4]}" if inside else "") + " · 옥토맵은 센서 마스크로 제외")
        return True

    def _has_octomap(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.OCTOMAP
        res = self._call(self.scene, req, 2.0)
        return bool(res is not None and len(res.scene.world.octomap.octomap.data) > 0)

    def _octomap_bytes(self):
        """옥토맵 크기[B] — 구 영역이 실제로 복셀을 지웠는지 **수치로** 확인하는 지표."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.OCTOMAP
        for _ in range(3):                       # 옥토맵 포함 조회는 무거워 한 번은 밀릴 수 있다
            res = self._call(self.scene, req, 10.0)
            if res is not None:
                return len(res.scene.world.octomap.octomap.data)
        return -1

    def _remember_target(self, name, p, r):
        self._tgt_geom[str(name)] = (np.asarray(p, float), float(r))

    def _allow_for_target(self, name):
        """접근 계획 전 충돌 허용 적용. 방식은 `acm_mode`(region|stalk|none)로 결정한다.

        ⚠ 이전에는 이 호출이 `plan_approach`/`_best_straight_candidate` 안에 이름 기반으로
        **하드코딩**돼 있어서, 비교실험의 'ACM 완화 없음' 조건에서도 목표 화방대가 허용됐다
        (조건 오염). 여기로 모아 모드로 제어한다."""
        mode = getattr(self, "_acm_mode", "region")
        if mode == "none":
            return False
        if mode == "stalk":
            return self._allow_collision(self._stalk_of(name))
        g = self._tgt_geom.get(str(name))
        if g is None:                                   # 기하를 모르면 이름 기반으로 폴백
            return self._allow_collision(self._stalk_of(name))
        rho = g[1] + float(self.get_parameter("region_margin").value)
        return self._allow_region(g[0], rho)

    # ══════════════════ 알고리즘: pre-grasp 자세 ══════════════════
    def _perception_targets(self):
        """인지 노드(/detected_fruits)가 낸 열매 → [(name, xyz, r), ...].

        Stage 4. yaml 의 이름표 대신 **카메라가 본 것**을 타깃으로 쓴다. 최초 호출에서만
        구독을 만들고 첫 메시지를 기다린다(latched 라 늦게 붙어도 즉시 받는다)."""
        topic = self.get_parameter("targets_topic").value
        if not hasattr(self, "_det"):
            self._det = []
            latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

            def _cb(msg):
                self._det = [(f"det_{m.id}",
                              np.array([m.pose.position.x, m.pose.position.y,
                                        m.pose.position.z]),
                              float(m.scale.x) / 2.0)
                             for m in msg.markers if m.action == Marker.ADD]
            # ⚠ 이 데모 노드에 직접 구독을 달고 spin_once 로 돌리면 메시지가 안 들어온다
            #   (서비스 클라이언트가 많은 노드라 대기셋 처리에서 밀린다 — 실측). 인지 결과는
            #   **전용 보조 노드**로 받는다.
            self._det_node = rclpy.create_node("pregrasp_targets_sub")
            self._det_node.create_subscription(MarkerArray, topic, _cb, latched)
            self.get_logger().info(f"인지 타깃 구독: {topic} (첫 관측 대기…)")
            t0 = time.time()
            wait = float(self.get_parameter("targets_wait").value)
            while not self._det and time.time() - t0 < wait:
                rclpy.spin_once(self._det_node, timeout_sec=0.2)
            self.get_logger().info(f"인지 열매 {len(self._det)}개 수신"
                                   f" ({time.time() - t0:.1f}s)")
        else:
            rclpy.spin_once(self._det_node, timeout_sec=0.0)   # 최신 관측 반영
        return list(self._det)

    def _all_targets(self):
        """집기 목표 열매 전부 [(name, xyz, r), ...].

        `target_source` 로 출처를 고른다:
          · yaml       — obstacles.yaml 의 kind:target (설계값, 이름표 있음)
          · perception — 카메라 인지 결과 /detected_fruits (Stage 4, 이름표 없음)
        """
        if str(self.get_parameter("target_source").value).lower().startswith("percep"):
            tg = self._perception_targets()
            for nm, p, r in tg:
                self._remember_target(nm, p, r)     # Stage 5: 구 영역 배치용 기하 기억
            return tg
        r0 = float(self.get_parameter("fruit_radius").value)
        path = self.get_parameter("obstacles_file").value or self._op.default_yaml()
        import yaml
        data = yaml.safe_load(open(path)) or {}
        try:
            self._op.expand_crops(data)
        except Exception:
            pass
        tg = [(o["name"], np.array([float(v) for v in o["pose"]["xyz"]]),
               float(o.get("radius", r0)))
              for o in data.get("obstacles", []) if o.get("kind") == "target"]
        for nm, p, r in tg:
            self._remember_target(nm, p, r)          # Stage 5: 구 영역 배치용 기하 기억
        return tg

    def _target(self):
        """단일 목표: param target 우선, 없으면 target_index 열매."""
        t = self.get_parameter("target").value
        r = float(self.get_parameter("fruit_radius").value)
        if t and len(t) == 3 and not any(math.isnan(float(v)) for v in t):
            self._remember_target("param_target", [float(v) for v in t], r)
            return "param_target", np.array([float(v) for v in t]), r
        tg = self._all_targets()
        if not tg:
            return None
        idx = max(0, min(int(self.get_parameter("target_index").value), len(tg) - 1))
        return tg[idx]

    def _base_xy(self):
        self._ensure_tf()
        try:
            tf = self._tf.lookup_transform(self.world,
                                           self.get_parameter("base_link").value,
                                           rclpy.time.Time())
            return np.array([tf.transform.translation.x, tf.transform.translation.y])
        except Exception:
            return None

    def solve_pregrasp(self, p_fruit, r):
        """자연스러운 접근 기하 + 후보 샘플링.

          · grasp 점  = p_fruit − a·grasp_offset  (TCP 가 열매 앞 grasp_offset 에서 파지)
          · pre-grasp = grasp − a·standoff         (grasp 에서 standoff 만큼 뒤로)
          → pre→grasp 직선이동 거리 = **정확히 standoff**. (사용자 요청 시퀀스)

        **pre-grasp 와 grasp 둘 다** IK/충돌 통과해야 채택 → 직선 접근이 실제로 가능한
        후보만 고른다(그래야 Cartesian 이 폴백 없이 곧게 들어간다).
        반환: dict(q_pre, q_grasp, a, p_pre, p_grasp, quat, c) 또는 None."""
        d0 = float(self.get_parameter("standoff").value)
        goff = float(self.get_parameter("grasp_offset").value)
        yaw = float(self.get_parameter("approach_yaw_deg").value)
        if not math.isnan(yaw):
            a0 = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.0])
        else:
            bxy = self._base_xy()
            hv = (p_fruit[:2] - bxy) if bxy is not None else np.array([1.0, 0.0])
            if np.linalg.norm(hv) < 1e-6:
                hv = np.array([1.0, 0.0])
            a0 = np.array([hv[0], hv[1], 0.0])
        a0 = PG._unit(a0)
        gp = self.get_parameter
        cands = PG.build_candidates(list(gp("sample_phi_deg").value),
                                    list(gp("sample_theta_deg").value),
                                    list(gp("sample_psi_deg").value), [d0],
                                    1.0, 0.5, 2.0, d0)
        best = None
        for c in cands:
            a = PG.approach_dir(a0, c.phi, c.theta)
            quat = PG.mat_to_quat(PG.gaze_rotation(a, c.psi, self.approach_axis))
            p_grasp = p_fruit - a * goff
            p_pre = p_grasp - a * c.d                       # grasp 에서 standoff 뒤
            q = self.solve_ik(p_pre, quat, avoid=True)      # pre-grasp 도달?
            if q is None:
                continue
            qg = self.solve_ik(p_grasp, quat, avoid=True)   # grasp 도 도달? (직선 접근 보장)
            if qg is None:
                continue
            if best is None or c.prior < best[0]:
                best = (c.prior, c, a, p_pre, p_grasp, quat, q, qg)
                if c.prior == 0.0:
                    break
        if best is None:
            return None
        _, c, a, p_pre, p_grasp, quat, q, qg = best
        return dict(c=c, a=a, p_pre=p_pre, p_grasp=p_grasp, quat=quat, q=q, q_grasp=qg)

    def _diag_straight(self):
        """[진단] 선택된 도달 열매에 대해 넓은 접근각 격자를 훑어, 각 후보의 pre/grasp IK
        통과 여부 + **직선 Cartesian fraction** 을 실측 보고한다. '집기 전 직선이동'이
        어떤 각도에서 가능한지(=fraction≈1.0 후보 존재 여부) 판정용. 데모는 재생 안 함."""
        sel = self._select_reachable()
        if sel is None or sel[3] is None:
            self.get_logger().error("진단: 도달 가능한 열매가 없음."); return
        name, p_fruit, r, sol = sel
        self._allow_for_target(name)     # 수확 작업공간 허용(직선 판정 공정)
        goff = float(self.get_parameter("grasp_offset").value)
        d0 = float(self.get_parameter("standoff").value)
        bxy = self._base_xy()
        hv = (p_fruit[:2] - bxy) if bxy is not None else np.array([1.0, 0.0])
        a0 = PG._unit(np.array([hv[0], hv[1], 0.0]))
        phis = [0, -10, 10, -20, 20, -30, 30, -40, 40]
        thetas = [0, -15, 15, -30, 30]
        self.get_logger().info(f"=== 직선접근 진단: {name} @({p_fruit[0]:.2f},{p_fruit[1]:.2f},"
                               f"{p_fruit[2]:.2f}) · standoff {d0*100:.0f}cm ===")
        rows = []
        for phd in phis:
            for thd in thetas:
                a = PG.approach_dir(a0, math.radians(phd), math.radians(thd))
                quat = PG.mat_to_quat(PG.gaze_rotation(a, 0.0, self.approach_axis))
                p_grasp = p_fruit - a * goff
                p_pre = p_grasp - a * d0
                q = self.solve_ik(p_pre, quat, avoid=True)
                if q is None:
                    continue
                qg = self.solve_ik(p_grasp, quat, avoid=True)
                if qg is None:
                    continue
                self._set(q)                              # 시작 = pre-grasp
                gp_pose = Pose()
                gp_pose.position.x, gp_pose.position.y, gp_pose.position.z = map(float, p_grasp)
                (gp_pose.orientation.x, gp_pose.orientation.y,
                 gp_pose.orientation.z, gp_pose.orientation.w) = map(float, quat)
                cart = self.cartesian_to(gp_pose)
                frac = cart[2] if cart is not None else 0.0
                rows.append((frac, phd, thd))
        rows.sort(reverse=True)
        for frac, phd, thd in rows[:12]:
            self.get_logger().info(f"  φ={phd:+3d}° θ={thd:+3d}° → 직선 fraction={frac:.2f}"
                                   + ("  ★거의 직선" if frac >= 0.95 else
                                      "  (부분)" if frac >= 0.5 else ""))
        if rows:
            best = rows[0]
            self.get_logger().info(
                f"=== 최고 직선 fraction={best[0]:.2f} @ φ={best[1]:+d}° θ={best[2]:+d}° "
                + ("→ 직선 파지 가능(그 각도 채택하면 됨)" if best[0] >= 0.95
                   else "→ 완전 직선은 불가(열매가 좁은 포켓). 부분직선+미세우회가 최선") + " ===")
        else:
            self.get_logger().info("=== IK 통과 후보 없음 ===")

    def _best_straight_candidate(self, name, p_fruit, r, thr=0.99):
        """'집기 전 직선이동'이 되는 접근각을 찾는다: 넓은 각도 격자에서 pre/grasp IK 통과 +
        **직선 Cartesian fraction 최대**인 후보 선택. fraction≥thr 이면 그 sol(dict) 반환
        → ②가 완전 직선 접근. 아니면 None(→기존 OMPL 우회 폴백). solve_pregrasp 는 엔드포인트
        IK 만 보므로 직선 경로가 막히는 각도를 고를 수 있다 → 여기서 직선 경로까지 검증해 교정.
        수확 작업공간(구 영역 / 목표 화방대)은 ACM 허용 후 판정."""
        from types import SimpleNamespace
        self._allow_for_target(name)
        goff = float(self.get_parameter("grasp_offset").value)
        d0 = float(self.get_parameter("standoff").value)
        bxy = self._base_xy()
        hv = (p_fruit[:2] - bxy) if bxy is not None else np.array([1.0, 0.0])
        a0 = PG._unit(np.array([hv[0], hv[1], 0.0]))
        phis = [0, -10, 10, -20, 20, -30, 30, -40, 40]
        thetas = [0, 15, 30, -15, -30]        # +θ = 위에서 접근(매달린 열매에 유리)
        best = None                            # (key, frac, sol)
        for phd in phis:
            for thd in thetas:
                a = PG.approach_dir(a0, math.radians(phd), math.radians(thd))
                quat = PG.mat_to_quat(PG.gaze_rotation(a, 0.0, self.approach_axis))
                p_grasp = p_fruit - a * goff
                p_pre = p_grasp - a * d0
                q = self.solve_ik(p_pre, quat, avoid=True)
                if q is None:
                    continue
                qg = self.solve_ik(p_grasp, quat, avoid=True)
                if qg is None:
                    continue
                self._set(q)
                gp_pose = Pose()
                gp_pose.position.x, gp_pose.position.y, gp_pose.position.z = map(float, p_grasp)
                (gp_pose.orientation.x, gp_pose.orientation.y,
                 gp_pose.orientation.z, gp_pose.orientation.w) = map(float, quat)
                cart = self.cartesian_to(gp_pose)
                frac = cart[2] if cart is not None else 0.0
                prior = abs(phd) + abs(thd)          # nominal(수평) 근접 — tie-break
                key = (round(frac, 3), -prior)
                if best is None or key > best[0]:
                    c = SimpleNamespace(phi=math.radians(phd), theta=math.radians(thd),
                                        psi=0.0, d=d0, prior=prior)
                    best = (key, frac, dict(c=c, a=a, p_pre=p_pre, p_grasp=p_grasp,
                                            quat=quat, q=q, q_grasp=qg))
        if best is not None and best[1] >= thr:
            c = best[2]["c"]
            self.get_logger().info(
                f"직선접근 각도 채택: φ={math.degrees(c.phi):+.0f}° θ={math.degrees(c.theta):+.0f}° "
                f"→ 직선 Cartesian fraction={best[1]:.2f}(집기 전 곧게 접근)")
            return best[2]
        if best is not None:
            self.get_logger().info(
                f"완전 직선 각도 없음(최고 fraction={best[1]:.2f}) → OMPL 우회로 접근")
        return None

    # ══════════════════ 5주차 2차: 접근 궤적 생성(줄기 회피) ══════════════════
    def plan_approach(self, name, grasp_pose, q_pre, q_grasp_ik, retries=1):
        """pre-grasp → grasp 접근 궤적을 생성한다(줄기 회피). 반환 (names, wp, method, checked).

        · 목표 화방대(수확 대상 줄기)는 열매가 거기 매달려 불가피 → 처음부터 ACM 충돌 제외.
        · 주 줄기·다른 화방대·거터·레일은 장애물 유지(진짜 회피 대상).
        우선순위: ① 직선 Cartesian(경로 개방 시) → ② OMPL 회피(장애물 막으면 경유점 자동
        생성해 우회, 좁은 공간이라 `retries` 회 재시도) → ③ Cartesian 부분경로(충돌검증·매끈)
        → ④ 최후 무검증 보간(경고). ①②③은 avoid_collisions 라 충돌free."""
        # 수확 작업공간 충돌 허용(acm_mode: region=구 영역 / stalk=목표 화방대 / none=없음)
        self._allow_for_target(name)
        # ① 직선 Cartesian (열매까지 곧게 들어갈 수 있으면 최선)
        cart = self.cartesian_to(grasp_pose)
        if cart is not None and cart[2] >= 0.99:
            n, wp, frac = cart
            self.get_logger().info(
                f"② 접근=직선 Cartesian {len(wp)}점(fraction={frac:.2f}) — 주 줄기 등 경로상 장애물 없음")
            return n, wp, f"cartesian(frac={frac:.2f})", True
        frac0 = cart[2] if cart is not None else 0.0
        # ② OMPL 로 grasp 자세까지 충돌회피 계획(좁은 공간 → 여러 번 재시도해 매끈한 경로 확보)
        self.get_logger().info(
            f"② 직선 접근 부분차단(fraction={frac0:.2f}, 주 줄기/구조가 경로 막음) "
            f"→ OMPL 회피 궤적 생성(줄기 회피 경유점, 최대 {max(1, retries)}회 시도)")
        for k in range(max(1, retries)):
            plan = self.plan_to(q_grasp_ik)
            if plan is not None:
                n, wp = plan
                self.get_logger().info(
                    f"② 접근=OMPL 회피 {len(wp)}점 궤적(줄기 우회, 충돌free)"
                    + (f" [{k + 1}번째 시도 성공]" if k else ""))
                return n, wp, f"ompl-avoid({len(wp)}pt)", True
        # ③ Cartesian 부분경로 폴백 — grasp 직전까지 충돌free·매끈(2점 스냅 방지)
        if cart is not None and frac0 >= 0.5:
            n, wp, _ = cart
            self.get_logger().info(
                f"② 접근=Cartesian 부분경로 {len(wp)}점(fraction={frac0:.2f}, grasp 근처까지 매끈·충돌free)")
            return n, wp, f"cartesian-partial(frac={frac0:.2f})", True
        # ④ 최후: 무검증 보간
        self.get_logger().warn("② 접근 계획 실패(도달권/공간 부족) → 무검증 관절보간 폴백(충돌 가능)")
        return (self.ARM,
                [[q_pre[j] for j in self.ARM], [q_grasp_ik[j] for j in self.ARM]],
                "interp(unchecked)", False)

    # ══════════════════ 데모 시퀀스 ══════════════════
    def _select_reachable(self):
        """현재 로봇 위치(어셈블러 base_placement)에서 도달 가능한 열매를 가까운 것부터
        찾아 (name, p_fruit, r, sol) 반환. 없으면 None."""
        param_t = self.get_parameter("target").value
        has_param = (param_t and len(param_t) == 3
                     and not any(math.isnan(float(v)) for v in param_t))
        if has_param or not self.get_parameter("auto_reachable").value:
            tgt = self._target()
            if tgt is None:
                return None
            sol = self.solve_pregrasp(tgt[1], tgt[2])
            return (tgt[0], tgt[1], tgt[2], sol) if sol else (tgt[0], tgt[1], tgt[2], None)
        # 자동: 전체 열매를 base(link0)로부터 가까운 순으로 정렬 → 앞 max_scan 개 시도
        tg = self._all_targets()
        if not tg:
            return None
        bxy = self._base_xy()
        if bxy is not None:
            # link0(≈base xy, z 0.35) 로부터 **3D 거리**로 정렬 → 가장 낮고 가까운(도달 쉬운)
            #  열매 우선. (수평거리만 쓰면 팔 한계인 높은 열매를 골라 접근이 어려움.)
            l0 = np.array([bxy[0], bxy[1], 0.35])
            tg.sort(key=lambda t: float(np.linalg.norm(t[1] - l0)))
        n = int(self.get_parameter("max_scan").value)
        self.get_logger().info(f"도달 가능한 열매 탐색(가까운 {min(n, len(tg))}개, 전체 {len(tg)})…")
        for name, p_fruit, r in tg[:n]:
            sol = self.solve_pregrasp(p_fruit, r)
            if sol is not None:
                return name, p_fruit, r, sol
        return tg[0][0], tg[0][1], tg[0][2], None      # 전부 실패 → 가장 가까운 것으로 안내

    def scan_all(self):
        """전체 kind:target 열매를 실제 IK 파이프라인(solve_pregrasp = pre+grasp 둘 다
        avoid_collisions IK)으로 훑어 **도달 가능 열매 목록**을 리포트한다. 데모는 재생하지 않음.
        '토마토 모델을 팔 도달권에 맞추는' 작업의 근거 수치(닿는 실열매 집합)를 뽑는 용도."""
        tg = self._all_targets()
        if not tg:
            self.get_logger().error("kind:target 열매가 없음 — obstacles.yaml 확인.")
            return
        bxy = self._base_xy()
        l0 = np.array([bxy[0], bxy[1], 0.35]) if bxy is not None else np.array([0.0, 0.0, 0.35])
        tg.sort(key=lambda t: float(np.linalg.norm(t[1] - l0)))
        self.get_logger().info(f"=== 도달 스캔 시작: 전체 {len(tg)}개 열매 (base link0 xy={l0[:2]}) ===")
        reach = []
        for name, p, r in tg:
            d = float(np.linalg.norm(p - l0))
            sol = self.solve_pregrasp(p, r)
            ok = sol is not None
            if ok:
                reach.append((name, p, d, sol))
            self.get_logger().info(
                f"  [{'O' if ok else 'X'}] {name:22s} ({p[0]:+.2f},{p[1]:+.2f},{p[2]:.2f}) "
                f"link0거리 {d:.3f}m" + (f"  φ={math.degrees(sol['c'].phi):+.0f}°" if ok else ""))
        self.get_logger().info(f"=== 도달 가능 {len(reach)}/{len(tg)}개 ===")
        if reach:
            zs = [p[2] for _, p, _, _ in reach]
            ds = [d for _, _, d, _ in reach]
            self.get_logger().info(
                f"    도달 열매 z {min(zs):.2f}~{max(zs):.2f}m · link0거리 {min(ds):.2f}~{max(ds):.2f}m")
            self.get_logger().info("    수확 대상 후보: " + ", ".join(n for n, _, _, _ in reach))
        return reach

    # ══════════════════ Stage 5 검증: 구 영역이 실제로 먹는가 ══════════════════
    def _state_report(self, q):
        """자세 q 의 충돌 상태 → (valid, 접촉객체 카운트 dict). 로그가 아니라 이 수치로 판정한다."""
        if self._sv is None or q is None:
            return None, {}
        req = GetStateValidity.Request()
        req.group_name = self.group
        js = JointState()
        js.name = list(q.keys()) + [f for f in self.FINGERS if f not in q]
        js.position = ([float(v) for v in q.values()]
                       + [float(self.cur.get(f, 0.0)) for f in self.FINGERS if f not in q])
        req.robot_state.joint_state = js
        req.robot_state.is_diff = True
        res = None
        for _ in range(3):
            res = self._call(self._sv, req, 10.0)
            if res is not None:
                break
        if res is None:
            return None, {}
        cnt = {}
        for c in res.contacts:
            key = f"{c.contact_body_1}|{c.contact_body_2}"
            cnt[key] = cnt.get(key, 0) + 1
        return bool(res.valid), cnt

    def verify_region(self):
        """[검증] 구 영역 허용 **전/후**를 같은 목표·같은 자세로 재어 비교한다.

        어제 ACM 조작이 조용히 무효였던 사고(components=2) 때문에, '적용됐다'는 로그가 아니라
        ① 옥토맵 크기 ② 파지 자세의 충돌 유효성 ③ 접촉 객체 ④ avoid_collisions IK 성공
        네 가지 **수치**로 확인한다. 센싱 장면(옥토맵)·설계값 장면 모두에서 쓸 수 있다."""
        tg = self._all_targets()
        if not tg:
            self.get_logger().error("목표 열매가 없음.")
            return
        bxy = self._base_xy()
        l0 = np.array([bxy[0], bxy[1], 0.35]) if bxy is not None else np.array([0., 0., 0.35])
        tg.sort(key=lambda t: float(np.linalg.norm(t[1] - l0)))
        goff = float(self.get_parameter("grasp_offset").value)
        # 기준 자세 = '열매를 잡는 자세'(충돌 무시 IK). 전/후 측정에 동일하게 쓴다.
        #  도달권 경계 열매는 IK 가 확률적으로 실패하므로, 가까운 열매·여러 접근각을 훑어
        #  **확실히 잡히는 목표**를 고른다(측정 대상 고정이 목적).
        idx0 = max(0, min(int(self.get_parameter("target_index").value), len(tg) - 1))
        name = q_ref = None
        for cand in range(idx0, min(idx0 + int(self.get_parameter("max_scan").value), len(tg))):
            nm, pf, rr = tg[cand]
            hv = pf[:2] - l0[:2]
            a0 = PG._unit(np.array([hv[0], hv[1], 0.0]))
            for phd, thd in [(0, 0), (0, 15), (-20, 0), (20, 0), (0, -15), (-20, 15), (20, 15)]:
                a_i = PG.approach_dir(a0, math.radians(phd), math.radians(thd))
                quat_i = PG.mat_to_quat(PG.gaze_rotation(a_i, 0.0, self.approach_axis))
                p_i = pf - a_i * goff
                q_ref = self.solve_ik(p_i, quat_i, avoid=False)
                if q_ref is not None:
                    name, p_fruit, r = nm, pf, rr
                    a, quat, p_grasp = a_i, quat_i, p_i
                    self.get_logger().info(
                        f"측정 목표 = {nm} (거리순 {cand}번째) · 접근각 φ={phd:+d}° θ={thd:+d}°")
                    break
            if q_ref is not None:
                break
        if q_ref is None:
            self.get_logger().error("파지 자세 IK(충돌무시)가 되는 열매를 찾지 못함 — 도달권 밖.")
            return
        q_arm = {j: q_ref[j] for j in self.ARM if j in q_ref}

        def probe(tag):
            octo = self._octomap_bytes()
            valid, cnt = self._state_report(q_arm)
            ik = self.solve_ik(p_grasp, quat, avoid=True) is not None
            self.get_logger().info(
                f"  [{tag}] 옥토맵 {octo}B · 파지자세 valid={valid} · 접촉 {sum(cnt.values())}건 "
                f"{list(cnt)[:4]} · avoid_collisions IK={'성공' if ik else '실패'}")
            return dict(tag=tag, octomap=octo, valid=valid,
                        contacts=sum(cnt.values()), pairs=cnt, ik=ik)

        wait = float(self.get_parameter("region_octomap_wait").value)
        sweep = [float(v) for v in (self.get_parameter("region_margin_sweep").value or [])
                 if float(v) > 0]
        self.get_logger().info(
            f"=== 구 영역 검증: {name} @({p_fruit[0]:.2f},{p_fruit[1]:.2f},{p_fruit[2]:.2f}) "
            f"r={r*100:.1f}cm · 파지점 {np.round(p_grasp, 3)} ===")
        self._clear_zone(force=True)                     # 이전 실행이 남긴 구까지 제거
        b0 = self._wait_octomap_stable()                 # 지도가 완전히 복구될 때까지 대기
        self.get_logger().info(f"기준선: 옥토맵 {b0}B 안정화 · 명명객체 {self._world_object_names()}")
        if sweep:
            # ρ 를 훑어 '파지가 성립하는 최소 영역'과 그 대가(지워진 복셀량)를 같이 잰다.
            base = probe("ρ 없음")
            rows = []
            for mg in sorted(sweep):
                rho_i = r + mg
                ok = self._allow_region(p_fruit, rho_i)
                self._wait_octomap_stable()
                p = probe(f"ρ={rho_i*100:.1f}cm")
                p["rho"], p["applied"] = rho_i, ok
                p["erased"] = base["octomap"] - p["octomap"]
                rows.append(p)
                self._clear_zone(force=True)
                self._wait_octomap_stable()
            self.get_logger().info("=== ρ 스윕 결과 (기준: 옥토맵 %dB · valid=%s · 접촉 %d건) ==="
                                   % (base["octomap"], base["valid"], base["contacts"]))
            for p in rows:
                self.get_logger().info(
                    f"  ρ={p['rho']*100:5.1f}cm → 파지 valid={str(p['valid']):5s} 접촉 {p['contacts']}건 · "
                    f"IK {'O' if p['ik'] else 'X'} · 지워진 옥토맵 {p['erased']:+d}B"
                    + ("" if p.get("applied") else "  ⚠적용실패"))
            good = [p for p in rows if p["valid"] and p["ik"]]
            if good:
                self.get_logger().info(
                    f"=== 파지 성립 최소 ρ = {min(p['rho'] for p in good)*100:.1f}cm "
                    f"(열매반경 {r*100:.1f}cm + 여유 {(min(p['rho'] for p in good)-r)*100:.1f}cm) ===")
            else:
                self.get_logger().warn("=== 스윕 전 구간에서 파지 자세가 성립하지 않음 ===")
            return rows
        rho = r + float(self.get_parameter("region_margin").value)
        self.get_logger().info(f"    ρ={rho*100:.1f}cm")
        before = probe("전")
        self._remember_target(name, p_fruit, r)
        self._acm_mode = "region"
        self._allow_for_target(name)
        # ★ 옥토맵은 **확률 갱신**이다. 마스크된 복셀은 매 프레임 'free' 한 표씩 받을 뿐이라
        #   강하게 점유된 복셀이 비점유로 내려가는 데 여러 프레임(초 단위)이 걸린다.
        #   고정 대기로는 판정이 흔들리므로 **효과가 나타날 때까지 폴링하고 그 시간을 잰다.**
        t_settle = float(self.get_parameter("region_settle_timeout").value)
        t0 = time.time()
        while time.time() - t0 < t_settle:
            v, _c = self._state_report(q_arm)
            if v:
                break
            time.sleep(3.0)
        settled = time.time() - t0
        after = probe("후")
        after["settle_s"] = settled
        self.get_logger().info(f"  구 적용 후 파지자세가 유효해지기까지 {settled:.0f}s "
                               f"(옥토맵 확률 침식 대기, 한도 {t_settle:.0f}s)")
        # ★ 3점째 '복원' — 구를 빼고 옥토맵을 같은 방식으로 다시 채운 뒤 재측정한다.
        #   이게 없으면 개선이 구 영역 덕인지 clear_octomap 으로 지도가 성겨진 덕인지 못 가른다.
        self._clear_zone()
        time.sleep(float(self.get_parameter("region_octomap_wait").value))
        back = probe("복원")   # 구를 빼면 다음 관측에서 복셀이 되살아나야 한다
        d = after["octomap"] - before["octomap"]
        self.get_logger().info(
            f"=== 결과: 옥토맵 {before['octomap']}→{after['octomap']}→{back['octomap']}B({d:+d}) · "
            f"valid {before['valid']}→{after['valid']}→{back['valid']} · "
            f"접촉 {before['contacts']}→{after['contacts']}→{back['contacts']}건 · "
            f"IK {'O' if before['ik'] else 'X'}→{'O' if after['ik'] else 'X'}"
            f"→{'O' if back['ik'] else 'X'} ===")
        gained = ((before["valid"] is False and after["valid"] is True)
                  or after["contacts"] < before["contacts"]
                  or (after["ik"] and not before["ik"]))
        reverted = ((back["valid"] is False and after["valid"] is True)
                    or back["contacts"] > after["contacts"]
                    or (before["ik"] is False and back["ik"] is False and after["ik"]))
        if gained and reverted:
            self.get_logger().info("=== 판정: 구 영역이 실제로 효과 있음(빼면 원상 복귀 — 인과 확인) ===")
        elif gained:
            self.get_logger().warn("=== 판정: 개선은 있으나 구를 빼도 안 돌아옴 → 옥토맵 재구축 등 "
                                   "다른 요인일 수 있음(재측정 필요) ===")
        else:
            self.get_logger().error("=== 판정: 변화 없음 — 구 영역이 먹지 않았다(원인 조사 필요) ===")
        return before, after, back

    # ══════════════════ 비교 실험(ablation) ══════════════════
    def _crop_objects(self):
        """장면의 작물 객체 이름(줄기·화방대·열매) 목록. ACM 일괄 조작용."""
        import yaml
        path = self.get_parameter("obstacles_file").value or self._op.default_yaml()
        data = yaml.safe_load(open(path)) or {}
        try:
            self._op.expand_crops(data)
        except Exception:
            pass
        return [o["name"] for o in data.get("obstacles", [])
                if str(o.get("name", "")).startswith(("stem_", "rachis_", "fruit_"))]

    def _read_acm(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        res = self._call(self.scene, req, 15.0)
        return None if res is None else res.scene.allowed_collision_matrix

    def _set_allow(self, names, value, tries=4, check_pairs=False):
        """ACM default entry 를 diff 로 일괄 설정(True=충돌 무시).

        ⚠ 응답의 success 를 믿지 않고 **ACM 을 재조회해 값이 실제로 반영됐는지 확인**하고,
        아니면 다시 시도한다. (객체를 ADD 하면 MoveIt 이 그 객체의 default entry 를 False 로
        자동 생성하므로, 우리 허용이 조용히 누락되면 구가 그대로 장애물이 된다 — 실측 확인.)"""
        if not names or self.apply_scene is None:
            return False
        want = bool(value)
        for k in range(tries):
            acm = self._read_acm()
            if acm is None:
                time.sleep(1.0)
                continue
            idx = {n: i for i, n in enumerate(acm.default_entry_names)}
            for n in names:
                if n in idx:
                    acm.default_entry_values[idx[n]] = want
                else:
                    acm.default_entry_names.append(n)
                    acm.default_entry_values.append(want)
            ps = PlanningScene()
            ps.is_diff = True
            ps.robot_state.is_diff = True
            ps.allowed_collision_matrix = acm
            self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 20.0)
            chk = self._read_acm()
            if chk is not None:
                cur = dict(zip(chk.default_entry_names, chk.default_entry_values))
                ok = all(cur.get(n) == want for n in names)
                # 쌍별 entry 는 **객체가 장면에 있을 때만** 생긴다. 그래서 객체 추가 뒤 호출에서만
                # 확인한다(객체 없을 때 확인하면 영영 실패해 배치 자체가 취소된다 — 실측).
                if check_pairs and ok:
                    rows = {nm: chk.entry_values[i].enabled
                            for i, nm in enumerate(chk.entry_names)}
                    cols = {nm: i for i, nm in enumerate(chk.entry_names)}
                    for n in names:
                        if n in rows:
                            vals = [rows[n][cols[m]] for m in chk.entry_names if m != n]
                            if vals and not all(v == want for v in vals):
                                ok = False
                if ok:
                    return True
            time.sleep(1.0)
        self.get_logger().warn(f"ACM 설정 확인 실패({len(names)}개 → {want}) — 반영 안 됐을 수 있음")
        return False

    def _invalid_waypoints(self, names, wp, sample=1):
        """궤적 웨이포인트를 **현재 ACM 기준**으로 검사해 충돌 상태 개수를 센다.
        조건 C(전 작물 무시)로 만든 궤적이 실제로는 얼마나 위험한지 정량화하는 지표."""
        if self._sv is None:
            return None
        bad = 0
        for i, q in enumerate(wp):
            if i % max(1, sample):
                continue
            req = GetStateValidity.Request()
            req.group_name = self.group
            js = JointState()
            # ⚠ 그리퍼 관절까지 명시해야 계획 당시와 같은 형상으로 검사된다.
            #   (팔 관절만 주면 손가락이 scene monitor 의 기본값으로 남아 다른 결과가 나온다.)
            js.name = list(names) + list(self.FINGERS)
            js.position = ([float(v) for v in q]
                           + [float(self.cur.get(f, 0.0)) for f in self.FINGERS])
            req.robot_state.joint_state = js
            req.robot_state.is_diff = True
            res = self._call(self._sv, req, 3.0)
            if res is not None and not res.valid:
                bad += 1
        return bad

    # Stage 5: 조건 = (직선 접근각 탐색 여부, 충돌 허용 방식)
    #   ⚠ 이전에는 허용 호출이 계획 함수 안에 하드코딩돼 있어 'no_acm' 조건에서도 목표
    #     화방대가 허용됐다(조건 오염). 이제 `self._acm_mode` 로만 결정된다.
    BENCH_CONDS = ("proposed", "stalk_acm", "no_search", "no_acm",
                   "no_search_no_acm", "ignore_all")
    _BENCH_ACM = {"proposed": "region", "stalk_acm": "stalk", "no_search": "region",
                  "no_acm": "none", "no_search_no_acm": "none", "ignore_all": "none"}

    def _bench_one(self, name, p_fruit, r, cond):
        """한 열매·한 조건을 평가해 dict 리포트. cond ∈ BENCH_CONDS"""
        crops = self._bench_crops
        stalk = self._stalk_of(name)
        home = {j: 0.0 for j in self.ARM}
        self._set(home)
        # ── 조건별 ACM 설정 ──
        self._clear_zone()                                  # 이전 조건의 구 영역 제거
        self._set_allow(crops, False)                       # 기준: 전 작물 장애물
        self._acm_mode = self._BENCH_ACM[cond]              # 허용 방식(계획 중 적용됨)
        if cond == "ignore_all":
            self._set_allow(crops, True)                    # 전 작물 무시(순진한 완화)
        self._remember_target(name, p_fruit, r)

        t0 = time.time()
        sol = self.solve_pregrasp(p_fruit, r)
        if sol is None:
            return dict(name=name, cond=cond, ik=False, frac=None, method="IK 실패",
                        n=0, bad=None, t=time.time() - t0)
        if cond not in ("no_search", "no_search_no_acm"):
            st = self._best_straight_candidate(name, p_fruit, r)
            if st is not None:
                sol = st
        # 접근 구간 평가
        self._set(sol["q"])
        gp = Pose()
        gp.position.x, gp.position.y, gp.position.z = map(float, sol["p_grasp"])
        (gp.orientation.x, gp.orientation.y,
         gp.orientation.z, gp.orientation.w) = map(float, sol["quat"])
        cart = self.cartesian_to(gp)
        frac = cart[2] if cart is not None else 0.0
        names_j, wp, method, checked = self.plan_approach(
            name, gp, sol["q"], sol["q_grasp"], retries=6)
        t = time.time() - t0
        # ── 안전성 재검증: 항상 '전 작물 장애물 + 수확 대상 줄기만 해제' 기준으로 ──
        #   ★ 구 영역을 먼저 걷어내야 한다. 남겨 두면 허용 해제된 구가 열매 자리에 박힌
        #     장애물이 되어 모든 조건이 충돌로 나온다.
        self._clear_zone()
        self._set_allow(crops, False)
        if stalk:
            self._set_allow([stalk], True)
        bad = self._invalid_waypoints(names_j, wp)
        self._set(home)
        return dict(name=name, cond=cond, ik=True, frac=frac, method=method,
                    n=len(wp), bad=bad, t=t)

    def bench_compare(self):
        """§비교실험: 도달 가능 열매마다 4개 조건을 돌려 표로 출력한다.

          proposed   = 제안(수확 대상 줄기만 ACM 해제 + 직선 접근각 탐색)
          no_search  = 직선 접근각 탐색 제거(명목 후보 격자만)
          no_acm     = 선택적 해제 없음(전 작물 장애물)
          ignore_all = 전 작물 충돌 무시(순진한 완화)
        """
        self._bench_crops = self._crop_objects()
        self.get_logger().info(f"작물 객체 {len(self._bench_crops)}개를 ACM 조작 대상으로 잡음")
        # ★ 장면이 **전부** 로드될 때까지 기다린다. _wait_scene 은 최소 개수만 보므로,
        #   부분 로드 상태에서 재면 장애물이 덜 실린 채로 IK 가 통과해 결과가 실행마다 달라진다
        #   (실제로 같은 표본이 8/8 ↔ 3/8 로 흔들렸다). 개수가 안정될 때까지 대기.
        prev, stable = -1, 0
        for _ in range(60):
            req = GetPlanningScene.Request()
            req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
            res = self._call(self.scene, req, 3.0)
            n = len(res.scene.world.collision_objects) if res else 0
            stable = stable + 1 if n == prev and n > 0 else 0
            prev = n
            if stable >= 3:
                break
            time.sleep(1.0)
        self.get_logger().info(f"장면 안정화: collision object {prev}개")
        tg = self._all_targets()
        bxy = self._base_xy()
        l0 = np.array([bxy[0], bxy[1], 0.35]) if bxy is not None else np.array([0.0, 0.0, 0.35])
        tg.sort(key=lambda t: float(np.linalg.norm(t[1] - l0)))
        nmax = int(self.get_parameter("bench_n").value)
        fixed = [t for t in (self.get_parameter("bench_targets").value or []) if t]
        if fixed:
            byname = {nm: (nm, p, r) for nm, p, r in tg}
            picked = [byname[n] for n in fixed if n in byname]
            self.get_logger().info(f"표본 고정: {len(picked)}개 (bench_targets)")
            missing = [n for n in fixed if n not in byname]
            if missing:
                self.get_logger().warn(f"표본에 없는 이름: {missing}")
            return self._bench_run(picked)
        # 제안 조건(구 영역 허용)에서 도달 가능한 열매만 표본으로
        picked = []
        for nm, p, r in tg:
            self._clear_zone()
            self._set_allow(self._bench_crops, False)
            self._acm_mode = "region"
            self._remember_target(nm, p, r)
            self._allow_for_target(nm)
            self._set({j: 0.0 for j in self.ARM})
            if self.solve_pregrasp(p, r) is not None:
                picked.append((nm, p, r))
            if nmax and len(picked) >= nmax:
                break
        return self._bench_run(picked)

    def _bench_run(self, picked):
        self.get_logger().info(f"=== 비교실험 표본 {len(picked)}개 열매 ===")
        rows = []
        for nm, p, r in picked:
            for cond in self.BENCH_CONDS:
                res = self._bench_one(nm, p, r, cond)
                rows.append(res)
                self.get_logger().info(
                    f"  {nm:22s} {cond:10s} IK={'O' if res['ik'] else 'X'} "
                    f"frac={('%.2f' % res['frac']) if res['frac'] is not None else '  - '} "
                    f"method={res['method']:24s} pts={res['n']:3d} "
                    f"충돌wp={res['bad'] if res['bad'] is not None else '-'} "
                    f"t={res['t']:.1f}s")
        # ── 요약 ──
        self.get_logger().info("=== 비교실험 요약 ===")
        for cond in self.BENCH_CONDS:
            rs = [x for x in rows if x["cond"] == cond]
            if not rs:
                continue
            ik = sum(1 for x in rs if x["ik"])
            fr = [x["frac"] for x in rs if x["frac"] is not None]
            straight = sum(1 for x in rs if x["method"].startswith("cartesian("))
            unver = sum(1 for x in rs if "interp" in x["method"])
            bad = [x["bad"] for x in rs if x["bad"] is not None]
            badsum = sum(bad) if bad else 0
            badcnt = sum(1 for b in bad if b > 0)
            self.get_logger().info(
                f"  {cond:10s} IK성공 {ik}/{len(rs)} · 직선fraction 평균 "
                f"{(sum(fr)/len(fr) if fr else 0):.2f} · 완전직선 {straight}/{len(rs)} · "
                f"무검증보간 {unver}/{len(rs)} · 충돌궤적 {badcnt}/{len(rs)}(총 {badsum}wp) · "
                f"평균 {sum(x['t'] for x in rs)/len(rs):.1f}s")
        import json
        out = "/tmp/bench_approach.json"
        with open(out, "w") as f:
            json.dump(rows, f, ensure_ascii=False, indent=1)
        self.get_logger().info(f"원자료 저장: {out}")
        return rows

    def _precompute(self, name, p_fruit, r, sol):
        """목표 열매에 대한 전체 데모 궤적(①home→pre ②접근 ④home복귀)을 한 번만 계획해 캐시.
        base·목표가 고정이라 매 루프 재계획할 필요가 없다 → 이후 반복은 재생만(버퍼링/튐 제거).
        계획 중 self.cur 를 가상 시작상태로 바꿔가며 각 구간을 계획한 뒤 home 으로 복원.

        ★ '집기 전 직선이동' 우선: 먼저 직선 Cartesian 이 뚫리는 접근각을 찾아(있으면) 그 자세로
        교체 → ②가 완전 직선. 없으면 원래 sol(OMPL 우회)."""
        straight = self._best_straight_candidate(name, p_fruit, r)
        if straight is not None:
            sol = straight
        q_pre, q_grasp_ik = sol["q"], sol["q_grasp"]
        p_grasp, quat = sol["p_grasp"], sol["quat"]
        gopen = float(self.get_parameter("gripper_open").value)
        close = float(self.get_parameter("gripper_close").value)
        q_home = {j: 0.0 for j in self.ARM}
        grasp_pose = Pose()
        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z = map(float, p_grasp)
        (grasp_pose.orientation.x, grasp_pose.orientation.y,
         grasp_pose.orientation.z, grasp_pose.orientation.w) = map(float, quat)

        # ① home → pre-grasp (시작 = home)
        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        plan = self.plan_to(q_pre)
        pre = plan if plan is not None else (
            self.ARM, [[0.0] * len(self.ARM), [q_pre[j] for j in self.ARM]])
        self.get_logger().info(f"① home→pre-grasp 계획 {len(pre[1])}점")

        # ② pre-grasp → grasp 접근(줄기 회피) — 시작 = q_pre, OMPL 우회 여러 번 재시도
        self._set({j: q_pre[j] for j in self.ARM})
        app_n, app_wp, method, checked = self.plan_approach(
            name, grasp_pose, q_pre, q_grasp_ik, retries=6)
        q_grasp = ({n: app_wp[-1][i] for i, n in enumerate(app_n)}
                   if app_wp else dict(q_grasp_ik))

        # ④ home 복귀 (시작 = q_pre, 후퇴=접근 역재생으로 q_pre 도달 후)
        self._set({j: q_pre.get(j, 0.0) for j in self.ARM})
        home = self.plan_to(q_home)

        # self.cur 복원(재생은 home+벌림에서 시작)
        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        return dict(pre=pre, app=(app_n, app_wp, method, checked),
                    q_pre=q_pre, q_grasp=q_grasp, q_home=q_home,
                    gopen=gopen, close=close, home=home, sol=sol)

    def run(self):
        # 목표 선택·전체 궤적 계획은 최초 1회만(base·목표 고정) → 캐시. 이후 반복은 재생만.
        if getattr(self, "_sel_cache", None) is None:
            self._sel_cache = self._select_reachable()
        sel = self._sel_cache
        if sel is None:
            self.get_logger().error("목표 열매를 찾지 못함 — obstacles.yaml 의 kind:target 확인.")
            return False
        name, p_fruit, r, sol = sel
        if sol is None:
            self.get_logger().error(
                f"[{name}] 현재 로봇 위치에서 도달 가능한 열매가 없음(도달불가/충돌). "
                "어셈블러(base_placement)로 로봇을 열매 앞으로 옮겨 저장하거나 base_x/base_y 로 조정.")
            self._publish_markers(p_fruit, r, None, None, reachable=False)
            self._hold(3.0)
            return False

        if getattr(self, "_plan_cache", None) is None:
            d0 = float(self.get_parameter("standoff").value)
            deg = math.degrees
            self.get_logger().info(
                f"목표 = {name} @ ({p_fruit[0]:.2f},{p_fruit[1]:.2f},{p_fruit[2]:.2f}) r={r:.3f} · "
                f"pre-grasp φ={deg(sol['c'].phi):+.0f}° θ={deg(sol['c'].theta):+.0f}° "
                f"(standoff {d0*100:.0f}cm)")
            self.get_logger().info("전체 궤적 계획 중(최초 1회, 몇 초 소요)…")
            self._plan_cache = self._precompute(name, p_fruit, r, sol)
            _, _, method, checked = self._plan_cache["app"]
            self.get_logger().info(
                f"계획 완료 → 접근 방식={method}" + ("" if checked else " ⚠충돌검증 안됨")
                + ". 이후 반복은 이 궤적을 매끈하게 재생만 함(재계획 없음).")
        c = self._plan_cache
        c_sol = c.get("sol", sol)
        self._publish_markers(p_fruit, r, c_sol["p_pre"], c_sol["a"], reachable=True)

        gopen, close = c["gopen"], c["close"]
        q_pre, q_grasp, q_home = c["q_pre"], c["q_grasp"], c["q_home"]
        app_n, app_wp, _, _ = c["app"]

        # 시작 = home + 그리퍼 벌림
        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        self._publish_js()

        # ── ① home → pre-grasp ──
        pn, pwp = c["pre"]
        self._play_waypoints(pn, pwp, float(self.get_parameter("dur_approach_plan").value))
        self._set({j: q_pre[j] for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))

        # ── ② pre-grasp → grasp : 줄기 회피 접근 궤적 재생 [5주차 2차] ──
        self._play_waypoints(app_n, app_wp, float(self.get_parameter("dur_approach_line").value))
        self._set({j: q_grasp.get(j, self.cur[j]) for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))

        # ── ③ 그리퍼 닫기(벌림 → 닫힘) ──
        self._play_waypoints(self.FINGERS,
                             [[gopen] * len(self.FINGERS), [close] * len(self.FINGERS)],
                             float(self.get_parameter("dur_gripper").value))

        # ── ④ 후퇴(접근 궤적 역재생 → 충돌free 경로로 안전 이탈) → home ──
        self._play_waypoints(app_n, list(reversed(app_wp)),
                             float(self.get_parameter("dur_retreat").value))
        self._set({j: q_pre.get(j, self.cur[j]) for j in self.ARM})
        if c["home"] is not None:
            hn, hwp = c["home"]
            self._play_waypoints(hn, hwp, float(self.get_parameter("dur_home").value))
        else:
            self._play_waypoints(self.ARM,
                                 [[q_pre[j] for j in self.ARM], [0.0] * len(self.ARM)],
                                 float(self.get_parameter("dur_home").value))
        self._set({j: 0.0 for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))
        self.get_logger().info(f"[{name}] 데모 1회 완료(재생).")
        return True


def main():
    rclpy.init()
    try:
        node = PregraspDemo()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        if node.get_parameter("scan_all").value:
            node.scan_all()
            node.destroy_node()
            rclpy.shutdown()
            return
        if node.get_parameter("verify_region").value:
            node.verify_region()
            node.destroy_node()
            rclpy.shutdown()
            return
        if node.get_parameter("bench").value:
            node.bench_compare()
            node.destroy_node()
            rclpy.shutdown()
            return
        if node.get_parameter("diag_straight").value:
            node._diag_straight()
            node.destroy_node()
            rclpy.shutdown()
            return
        while rclpy.ok():
            ok = node.run()
            if not node.get_parameter("loop").value:
                # 마지막 자세 유지 발행
                while rclpy.ok():
                    node._hold(1.0)
                break
            node._hold(1.5)
            if not ok:
                node._hold(3.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# 시각화(pregrasp_pose 와 동일 형식) — 클래스에 메서드로 부착
def _publish_markers(self, p_fruit, r, p_pre, a, reachable):
    arr = MarkerArray()

    def base(mid, mtype):
        m = Marker()
        m.header.frame_id = self.world
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "pregrasp", mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m
    m = base(0, Marker.SPHERE)
    m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p_fruit)
    m.scale.x = m.scale.y = m.scale.z = float(2 * r * 1.15)
    m.color.r, m.color.g, m.color.b, m.color.a = (
        (0.2, 1.0, 0.2, 0.5) if reachable else (1.0, 0.3, 0.3, 0.5))
    arr.markers.append(m)
    if reachable and p_pre is not None:
        sp = base(1, Marker.SPHERE)
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = map(float, p_pre)
        sp.scale.x = sp.scale.y = sp.scale.z = 0.03
        sp.color.r, sp.color.g, sp.color.b, sp.color.a = 0.2, 0.5, 1.0, 0.9
        arr.markers.append(sp)
        ar = base(2, Marker.ARROW)
        ar.points = [Point(x=float(p_pre[0]), y=float(p_pre[1]), z=float(p_pre[2])),
                     Point(x=float(p_fruit[0]), y=float(p_fruit[1]), z=float(p_fruit[2]))]
        ar.scale.x, ar.scale.y, ar.scale.z = 0.008, 0.02, 0.03
        ar.color.r, ar.color.g, ar.color.b, ar.color.a = 0.1, 0.9, 0.9, 0.95
        arr.markers.append(ar)
    self.mk_pub.publish(arr)


PregraspDemo._publish_markers = _publish_markers


if __name__ == "__main__":
    main()
