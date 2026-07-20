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

from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetCartesianPath
from moveit_msgs.msg import (PositionIKRequest, RobotState, MotionPlanRequest,
                             Constraints, JointConstraint, DisplayRobotState)


def _import_sibling(mod_name, file_name):
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
    spec = importlib.util.spec_from_file_location(mod_name, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# pregrasp_pose 의 순수 기하/후보 함수 재사용(단일 진실원)
PG = _import_sibling("_pregrasp_pose", "pregrasp_pose.py")


class PregraspDemo(Node):
    ARM = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
    FINGERS = ["rg2_finger_joint1", "rg2_finger_joint2"]

    def __init__(self):
        super().__init__("pregrasp_demo")
        # ---- 파라미터 ----
        self.declare_parameter("target", [float("nan")] * 3)
        self.declare_parameter("target_index", 0)
        self.declare_parameter("auto_reachable", True)   # 현 위치서 도달가능 열매 자동선택
        self.declare_parameter("max_scan", 12)           # 자동선택 시 가까운 열매 몇개까지 시도
        self.declare_parameter("obstacles_file", "")
        self.declare_parameter("fruit_radius", 0.035)
        self.declare_parameter("standoff", 0.15)
        self.declare_parameter("grasp_offset", 0.10)     # 파지 시 열매 중심 앞 TCP 정지거리
        self.declare_parameter("approach_yaw_deg", float("nan"))
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_link", "link0")
        self.declare_parameter("group", "arm")
        self.declare_parameter("ik_link", "tcp")
        self.declare_parameter("ik_timeout", 0.1)
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
        for cli, nm in ((self.ik, "compute_ik"), (self.plan, "plan_kinematic_path"),
                        (self.cart, "compute_cartesian_path")):
            self.get_logger().info(f"{nm} 대기중…")
            cli.wait_for_service()
        self.get_logger().info("MoveIt 서비스 연결됨.")

        self._op = _import_sibling("_obstacle_publisher", "obstacle_publisher.py")

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

    # ══════════════════ 알고리즘: pre-grasp 자세 ══════════════════
    def _all_targets(self):
        """obstacles.yaml 의 kind:target 열매 전부 [(name, xyz, r), ...]."""
        r0 = float(self.get_parameter("fruit_radius").value)
        path = self.get_parameter("obstacles_file").value or self._op.default_yaml()
        import yaml
        data = yaml.safe_load(open(path)) or {}
        try:
            self._op.expand_crops(data)
        except Exception:
            pass
        return [(o["name"], np.array([float(v) for v in o["pose"]["xyz"]]),
                 float(o.get("radius", r0)))
                for o in data.get("obstacles", []) if o.get("kind") == "target"]

    def _target(self):
        """단일 목표: param target 우선, 없으면 target_index 열매."""
        t = self.get_parameter("target").value
        r = float(self.get_parameter("fruit_radius").value)
        if t and len(t) == 3 and not any(math.isnan(float(v)) for v in t):
            return "param_target", np.array([float(v) for v in t]), r
        tg = self._all_targets()
        if not tg:
            return None
        idx = max(0, min(int(self.get_parameter("target_index").value), len(tg) - 1))
        return tg[idx]

    def _base_xy(self):
        import tf2_ros
        if not hasattr(self, "_tf"):
            self._tf = tf2_ros.Buffer()
            self._tfl = tf2_ros.TransformListener(self._tf, self)
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.05)
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
            quat = PG.mat_to_quat(PG.gaze_rotation(a, c.psi))
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

    def run(self):
        sel = self._select_reachable()
        if sel is None:
            self.get_logger().error("목표 열매를 찾지 못함 — obstacles.yaml 의 kind:target 확인.")
            return False
        name, p_fruit, r, sol = sel
        self.get_logger().info(f"목표 = {name} @ ({p_fruit[0]:.2f},{p_fruit[1]:.2f},"
                               f"{p_fruit[2]:.2f}) r={r:.3f}")

        if sol is None:
            self.get_logger().error(
                f"[{name}] 현재 로봇 위치에서 도달 가능한 열매가 없음(도달불가/충돌). "
                "어셈블러(base_placement)로 로봇을 열매 앞으로 옮겨 저장하거나 base_x/base_y 로 조정.")
            self._publish_markers(p_fruit, r, None, None, reachable=False)
            self._hold(3.0)
            return False
        q_pre, q_grasp_ik, a = sol["q"], sol["q_grasp"], sol["a"]
        p_pre, p_grasp, quat = sol["p_pre"], sol["p_grasp"], sol["quat"]
        d0 = float(self.get_parameter("standoff").value)
        self._publish_markers(p_fruit, r, p_pre, a, reachable=True)
        deg = math.degrees
        self.get_logger().info(
            f"[{name}] pre-grasp 채택 φ={deg(sol['c'].phi):+.0f}° θ={deg(sol['c'].theta):+.0f}° "
            f"(standoff {d0*100:.0f}cm 뒤 → 직선 접근) → 데모 재생 시작.")

        grasp_pose = Pose()
        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z = map(float, p_grasp)
        (grasp_pose.orientation.x, grasp_pose.orientation.y,
         grasp_pose.orientation.z, grasp_pose.orientation.w) = map(float, quat)

        # ── ① home → pre-grasp (OMPL, 실패 시 관절보간) ──
        gopen = float(self.get_parameter("gripper_open").value)
        q_home = {j: 0.0 for j in self.ARM}
        self._set(dict(q_home, **{f: gopen for f in self.FINGERS}))  # 시작 = home + 그리퍼 벌림
        self._publish_js()
        plan = self.plan_to(q_pre)
        if plan is not None:
            names, wp = plan
            self.get_logger().info(f"① home→pre-grasp OMPL 계획 {len(wp)}점 재생")
            self._play_waypoints(names, wp, float(self.get_parameter("dur_approach_plan").value))
        else:
            self.get_logger().warn("① OMPL 실패 → 관절보간 폴백")
            names = self.ARM
            wp = [[q_home[j] for j in names], [q_pre[j] for j in names]]
            self._play_waypoints(names, wp, float(self.get_parameter("dur_approach_plan").value))
        self._set({j: q_pre[j] for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))

        # ── ② pre-grasp → grasp : standoff 거리만큼 직선 이동 (Cartesian) ──
        #    grasp 는 선택 단계에서 이미 도달 검증됨 → Cartesian 이 곧게 들어간다.
        cart = self.cartesian_to(grasp_pose)
        q_grasp = q_grasp_ik
        if cart is not None and cart[2] > 0.9:
            names, wp, frac = cart
            self.get_logger().info(f"② 직선 접근({d0*100:.0f}cm) Cartesian {len(wp)}점(fraction={frac:.2f}) 재생")
            self._play_waypoints(names, wp, float(self.get_parameter("dur_approach_line").value))
            q_grasp = {n: wp[-1][i] for i, n in enumerate(names)}
        else:
            frac = cart[2] if cart is not None else 0.0
            self.get_logger().info(f"② 직선 접근({d0*100:.0f}cm) — Cartesian fraction={frac:.2f}, "
                                   "검증된 grasp 해로 직선 보간")
            names = self.ARM
            wp = [[q_pre[j] for j in names], [q_grasp_ik[j] for j in names]]
            self._play_waypoints(names, wp, float(self.get_parameter("dur_approach_line").value))
        self._set({j: q_grasp.get(j, self.cur[j]) for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))

        # ── ③ 그리퍼 닫기(벌림 gopen → 닫힘 close) ──
        close = float(self.get_parameter("gripper_close").value)
        self.get_logger().info("③ 그리퍼 닫기(파지)")
        self._play_waypoints(self.FINGERS,
                             [[gopen, gopen], [close, close]],
                             float(self.get_parameter("dur_gripper").value))

        # ── ④ 후퇴 grasp→pre-grasp → home ──
        self.get_logger().info("④ 후퇴 → home")
        names = self.ARM
        self._play_waypoints(names,
                             [[q_grasp.get(j, q_pre[j]) for j in names],
                              [q_pre[j] for j in names]],
                             float(self.get_parameter("dur_retreat").value))
        self._set({j: q_pre[j] for j in self.ARM})
        # home 복귀(그리퍼는 닫은 채로 '수확물' 이송 느낌)
        self._play_waypoints(names,
                             [[q_pre[j] for j in names], [0.0] * len(names)],
                             float(self.get_parameter("dur_home").value))
        self._set({j: 0.0 for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))
        self.get_logger().info(f"[{name}] 데모 1회 완료.")
        return True


def main():
    rclpy.init()
    try:
        node = PregraspDemo()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
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
