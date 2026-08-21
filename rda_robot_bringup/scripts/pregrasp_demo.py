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
from rclpy.action import ActionClient
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from moveit_msgs.srv import (GetPositionIK, GetMotionPlan, GetCartesianPath,
                             GetPlanningScene, ApplyPlanningScene, GetStateValidity)
from moveit_msgs.msg import (PositionIKRequest, RobotState, MotionPlanRequest,
                             Constraints, JointConstraint, DisplayRobotState,
                             PlanningScene, PlanningSceneComponents, CollisionObject,
                             AllowedCollisionEntry, AttachedCollisionObject)
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
        # 목표를 **이름으로 고정**(예: fruit_r0_p3_t0_f2). 비교 실행에서 두 조건이 같은
        #   열매를 잡게 하려면 필요하다 — 자동 선택은 IK 무작위 때문에 실행마다 달라질 수 있다.
        self.declare_parameter("target_name", "")
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
        # 파지 시 **TCP 가 열매 중심 앞에서 멈추는 거리**. 상수로 두면 그리퍼를 바꿨을 때
        #   조용히 틀린다 — 손가락이 짧은 그리퍼(예 Robotiq 2F-85, 패드 ~5cm)로 바꾸면 열매가
        #   손끝 **너머**에 놓여 빈손으로 닫힌다. → `grasp_offset_auto` 로 URDF 에서 유도한다.
        self.declare_parameter("grasp_offset", 0.10)
        # true = 그리퍼 손가락이 접근축으로 차지하는 깊이(near,far)를 URDF 에서 재서
        #   **열매 중심이 패드 사이 grasp_depth 지점에 오도록** grasp_offset 을 계산한다.
        #   (RG2 실측: 패드 0.091~0.213m → 0.5 면 0.152m)
        self.declare_parameter("grasp_offset_auto", True)
        # grasp_depth = 열매 중심을 패드의 어디에 둘지(0=집게 뿌리·1=손끝).
        #   🔴 **0.33 은 패드가 아니라 집게 구동부에 무는 값이었다**(2026-08-21 사용자 지적 →
        #     mesh 실측으로 확인). RG2 손가락 안쪽면은 tcp 기준 **9.66~21.25cm** 인데
        #     그중 **9.92~11.34cm 는 집게가 도는 두꺼운 뿌리**다(바깥면 폭이 그 구간에만 있다).
        #     0.33 → grasp_offset 13.1cm ⇒ 열매(r 3.5cm) 앞면이 **9.6cm** = 뿌리에 박힌다.
        #   → 기본 0.50 → 15.2cm ⇒ 열매가 **11.7~18.7cm** 를 점유해 패드 유효면 안에 들어온다.
        #   ⚠ 종전 주석은 "깊게 물수록 손끝이 열매 뒤로 더 들어간다" 고 적고 있었으나 **반대다** —
        #     goff 가 커지면 TCP 가 뒤로 물러서므로 손끝이 열매 뒤로 나가는 양은 오히려 준다
        #     (0.33: 손끝 여유 +4.7cm → 0.50: +2.6cm). 캐노피 침투는 줄어든다.
        #   ⚠ 다만 **센싱(옥토맵) 장면에서 0.5 로 파지 자세가 안 나온 실측이 있다**(수확 1→0).
        #     그건 TCP 가 물러서는 자리(pre-grasp)가 옥토맵에 막힌 것 — 인지 장면에서 쓸 때는
        #     그 실측을 다시 확인할 것.
        #   **기본 -1 = auto** — 비율이 아니라 **파지면(맞은편 손가락을 향한 면)의 면적중심**을
        #     열매 중심에 맞춘다. 비율(0~1)은 뿌리의 두꺼운 구동부와 얇은 날 끝을 함께 세므로
        #     그리퍼가 바뀌면 같은 값이 다른 곳을 뜻한다. 0~1 을 주면 종전(구간 비율) 방식.
        self.declare_parameter("grasp_depth", -1.0)
        # 손바닥(그리퍼 몸체) 앞면과 열매 표면 사이 최소 여유[m]. auto 파지깊이의 하한을 만든다.
        #   🔴 이게 없으면 파지점이 얕아져 **열매가 손바닥을 파고드는** 자세가 나온다
        #     (RG2 실측: 여유 없이 두면 1.0cm 침투 — 실제로는 물 수 없다).
        self.declare_parameter("palm_clearance", 0.005)
        # ── 직선접근 궤적의 '관절 건전성' 검사 (2026-07-29) ─────────────────
        # TCP 가 직선이어도(fraction=1.00) 손목이 특이점 근처를 지나며 관절이 크게 도는 해가
        # 섞여 나온다. 실측: 같은 열매·같은 직선 15cm 에서 접근구간 관절길이가 **0.65rad ↔
        # 5.3rad** 두 해로 확률적으로 갈렸고, **큰 쪽만 오른손가락이 주 줄기(stem_r0_p2_s0)에
        # 닿았다**(216회 측정의 접촉 10건 전부 이 경우). `jump_threshold=0` 이라 MoveIt 의
        # 관절점프 필터도 꺼져 있어 그대로 통과했다.
        # → 접근 구간 관절 경로길이가 이 한도를 넘으면 **'직선 성공'으로 인정하지 않는다.**
        self.declare_parameter("approach_jl_max", 2.0)      # [rad] 0=검사 안 함
        # MoveIt 자체 관절점프 필터(0=끔). relative=평균 대비 배수 · revolute=스텝당 절대[rad].
        self.declare_parameter("cartesian_jump", 0.0)
        self.declare_parameter("cartesian_revolute_jump", 0.0)
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
        # ── 수확 모드 ────────────────────────────────────────────────────────
        #  grasp = 열매를 하나씩 잡아 딴다(종전 동작, 기본값).
        #  cut   = **화방대(rachis)를 잘라** 화방 통째로 수확한다. 열매에 손이 안 닿으므로
        #          손상이 없다. 기하 제약이 달라진다 — 아래 solve_cut 주석 참조.
        self.declare_parameter("harvest_mode", "grasp")   # grasp | cut
        self.declare_parameter("cut_ratio", 0.5)          # 절단점: 0=줄기쪽 끝, 1=열매쪽 끝
        self.declare_parameter("cut_depth", 0.5)          # 날 위치(패드 깊이 구간 비율). 0.5=패드 중앙
        # 접근축을 줄기 축 둘레로 돌려 보는 각도(수직성은 어느 값에서도 유지된다)
        self.declare_parameter("cut_beta_deg",
                               [0.0, -20.0, 20.0, -40.0, 40.0, -60.0, 60.0, -80.0, 80.0])
        self.declare_parameter("cut_gripper_close", 0.0)  # 절단 = 끝까지 닫는다
        self.declare_parameter("cut_axis", [float("nan")] * 3)   # 좌표 목표일 때 줄기 축
        self.declare_parameter("cut_object", "")          # 좌표 목표일 때 충돌허용할 줄기 객체명
        self.declare_parameter("cut_remove_fruits", True) # 자르면 그 화방 열매도 함께 사라진다
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
        # ── 접근 전략(작업 플래너) 선택 ──────────────────────────────
        # strategy = 수확 작업을 어떻게 풀지(모션플래너 알고리즘을 고르듯 접근 방식을 고름):
        #   harvest_linear : 수확 특화 — pre-grasp 자세 → Cartesian 직선접근 → 파지 → 역경로 후퇴 → home
        #   direct         : 자유공간 모션플래너로 grasp pose 직행 → 파지 → 자유공간 복귀(직선접근 구조 없음)
        # 새 전략은 _STRATEGIES 에 함수 하나 추가로 확장(레지스트리).
        self.declare_parameter("strategy", "harvest_linear")
        # planner_id = 자유공간 구간에 쓸 OMPL 알고리즘. ompl_planning.yaml 의 arm 목록과 일치해야 함
        #   (RRTConnect·RRTstar·PRM·RRT·BiTRRT·EST). 빈 값이면 그룹 기본(RRTConnect).
        self.declare_parameter("planner_id", "RRTConnect")
        # [대조군] naive 전략의 관절보간 분해 점수. 충돌 검사 해상도를 정하는 값이라
        #   너무 성기면 얇은 줄기를 통과해도 '충돌 없음' 으로 보인다.
        self.declare_parameter("naive_steps", 40)
        # frontal(회피 없음) 대조군 기하 ─────────────────────────────────
        #   정면 법선을 어느 객체에서 유도할지(이름 접두사). 재배 거터 = 행잉베드.
        self.declare_parameter("gutter_prefix", "gutter")
        #   직선 진입을 **캐노피 밖에서** 시작한다(사용자 정의 2026-08-21). 캐노피 바깥
        #   지점에서 이 여유[m]만큼 더 물러선 곳이 진입 시작점.
        self.declare_parameter("canopy_margin", 0.05)
        #   캐노피 경계를 잴 때 '같은 행'으로 볼 측방 폭[m]. 이걸 안 두면 다음 행 거터까지
        #   잡혀 진입 거리가 통로 전체로 부풀어 무의미해진다.
        self.declare_parameter("canopy_row_width", 0.45)
        #   접근축 방향으로 '같은 행'으로 볼 깊이[m]. 행은 접근축 방향으로 떨어져 있으므로
        #   **이것이 행을 가르는 실제 기준**이다(측방 폭만으로는 못 가른다).
        self.declare_parameter("canopy_depth", 0.6)
        #   캐노피 밖 진입을 쓰지 않고 종전처럼 standoff 만 직선으로 하려면 false.
        self.declare_parameter("frontal_from_canopy", True)
        # ── 수확 대상 선정 규칙 (사용자 정의 2026-08-21) ────────────────
        #   select_order = 어떤 거리로 '가까운 것부터' 를 정할지
        #     normal : **거터(행잉베드) 법선 방향 수평거리** — 통로에서 작물행 쪽으로 잰
        #              수직(perpendicular) 거리. 얕게 박힌 열매(통로에 가까운 것)부터.
        #     dist3d : 종전 — 팔 베이스에서의 3D 유클리드 거리
        #     z      : 팔 베이스와의 높이(z) 차이
        self.declare_parameter("select_order", "normal")
        #   require_visible = **다른 열매에 가려진 열매는 대상에서 뺀다**(카메라 시선 기준).
        #     eye-to-hand 카메라 원점에서 열매 중심으로 그은 선분을 다른 열매 구가 막으면 제외.
        #     🔴 줄기·화방대는 세지 않는다(사용자 정의: "다른 토마토에 가려지지 않은 것").
        self.declare_parameter("require_visible", True)
        #   시선 기준 카메라 링크. 'auto' 면 sensor2(eye-to-hand) 계열을 TF 에서 찾는다.
        self.declare_parameter("camera_link", "auto")
        #   가림 판정 여유[m]. 가리는 열매 반경에 더해 본다(스쳐 지나가는 경우도 가림으로).
        self.declare_parameter("occlusion_margin", 0.005)
        # 안전성 재검증 해상도[rad]. 웨이포인트 사이를 이 간격 이하로 잘게 나눠 검사한다.
        #   🔴 전략마다 웨이포인트 밀도가 다르면(플래너 44점 vs 보간 40점) 충돌 개수를
        #     그대로 비교할 수 없고, 성긴 표본은 **얇은 줄기를 지나치며 놓친다**.
        #     0=끔(종전). bench_strategy 재검증에만 적용.
        self.declare_parameter("verify_max_step", 0.02)
        # 재생 중 **현재 자세의 충돌 여부**를 조회해 접촉점을 RViz 마커로 띄운다.
        #   회피 있음/없음 비교를 화면에서 가르는 유일한 표시(재생 모드는 물리가 없어
        #   부딪혀도 아무 일이 안 일어난다 — 표시가 없으면 두 동작이 똑같아 보인다).
        self.declare_parameter("show_collisions", False)
        self.declare_parameter("collision_probe_hz", 10.0)
        # execute = true 면 /joint_states 재생 대신 **실제 컨트롤러(FollowJointTrajectory)로 실구동**.
        #   6주차 ros2_control(gazebo control 모드)과 묶어 센싱 장면에서 팔을 실제로 움직인다.
        #   이때 관절 상태는 gazebo joint_state_broadcaster 가 발행 → 이 노드는 /joint_states 를
        #   발행하지 않고 **구독**해 현재 상태를 추종한다.
        self.declare_parameter("execute", False)
        self.declare_parameter("arm_controller", "arm_controller")
        self.declare_parameter("gripper_controller", "gripper_controller")
        # ── 실행 중 감시·재계획 트리거 (6주차 '피드백 루프')
        #   execute 모드에서 궤적을 흘리고 끝까지 기다리기만 하면, 실행 도중 센서로 장면이
        #   바뀌어도(팔이 움직이며 eye-in-hand 카메라가 새 복셀을 쌓는다) 알 수가 없다.
        #   실행 중 **남은 웨이포인트**를 현재 장면으로 재검사해, 막히면 멈추고 다시 계획한다.
        #   ⚠ 검사는 계획 때와 **같은 ACM**(구 영역 허용이 걸린 상태)에서 해야 한다 —
        #     아니면 계획이 정당하게 허용한 접촉까지 '막혔다'로 읽혀 매번 트리거된다.
        self.declare_parameter("replan", True)             # false=종전 동작(감시 없음)
        self.declare_parameter("replan_period", 0.5)       # 검사 주기 s
        self.declare_parameter("replan_sample", 3)         # 남은 웨이포인트 N개마다 1개 검사
        self.declare_parameter("replan_lookahead", 0.05)   # 진행률 여유(현재 지점 바로 뒤는 건너뜀)
        self.declare_parameter("replan_max", 2)            # 한 구간당 재계획 시도 횟수
        # 안전 감시(safety_monitor)가 결함을 래치하면 실행을 즉시 포기한다.
        self.declare_parameter("abort_topic", "/safety_monitor/abort")
        # harvest_all = 도달 가능한 열매를 하나씩 **연속 수확**(단일 열매 반복 대신). harvest_max 개까지.
        self.declare_parameter("harvest_all", False)
        self.declare_parameter("harvest_max", 5)
        # harvest_remove = 수확한 열매를 **장면에서 없앤다**(실제 수확처럼). 그래야 뒤쪽 열매의
        #   가림(인지)·막힘(계획)이 풀려 수확 가능 범위가 는다. 세 곳을 함께 처리한다:
        #     ① planning scene 에서 REMOVE  ② obstacle_publisher 재발행 제외(`/harvested`)
        #     ③ Gazebo 에서 모델 삭제(`/delete_entity`) — 시뮬 구동 중일 때만(카메라·옥토맵 반영)
        #   ②를 빼면 재발행 주기(1s) 때문에 곧바로 되살아난다.
        self.declare_parameter("harvest_remove", True)
        self.declare_parameter("harvested_topic", "harvested")
        self.declare_parameter("harvest_delete_gazebo", True)
        # 인지 타깃(det_N)은 이름표가 없어 Gazebo 모델 이름과 다르다 → **위치로 매칭**할 때
        #   허용 오차[m]. 인지 중심오차 중앙값 1.6cm · 한 화방 열매 간격 6cm 사이 값.
        self.declare_parameter("harvest_match_tol", 0.06)
        # 파지한 열매를 그리퍼에 **부착**해 계획에 반영(손에 든 열매가 옆 줄기를 쓸지 않게).
        #   끄면 종전처럼 '빈손' 가정으로 후퇴·복귀 경로를 짠다.
        self.declare_parameter("attach_fruit", True)
        # 수확가능 판정에서 IK 를 몇 번까지 다시 물을지.
        #   🔴 **기본 1 — 재시도는 효과가 없었다**(2026-07-29 실측). KDL IK 의 무작위 재시작이
        #   원인일 거라 보고 3회로 올려 봤지만 흔들림이 그대로였고(들쭉날쭉 4개 동일) 라운드
        #   시간만 2~4배로 늘었다(46~116s → 92~185s). 원인은 다른 데 있다(아래 reach_noise 참조).
        self.declare_parameter("harvest_ik_tries", 1)
        # 선별(수확가능 판정)을 **비파괴적**으로 — 후보마다 구 영역 객체를 넣지 않고, 라운드
        #   전체에 ACM(옥토맵↔그리퍼 링크)만 한 번 걸었다 되돌린다. 종전 방식은 장면을 누적
        #   변형시켜(옥토맵 마스킹 비가역) 라운드마다 판정이 달라졌다. false=종전 동작.
        self.declare_parameter("screen_nondestructive", True)
        # [진단] 선별에서 탈락한 자세를 **무엇이 막는지**(접촉 쌍) 조회해 로그에 남긴다.
        #   개수만으로는 옥토맵 탓인지 다른 탓인지 못 가른다. 후보당 IK+검사 1회씩 더 든다.
        self.declare_parameter("screen_why_detail", False)
        # [진단] 수확 1회 후 선별을 K 회 반복하며 개수·옥토맵 크기·막는 것을 추적하고 종료.
        #   "수확하면 뒤가 열리는가 / 오히려 새 복셀이 막는가" 를 가르는 측정.
        self.declare_parameter("probe_after_harvest", 0)
        self.declare_parameter("probe_interval", 20.0)
        # [진단] 수확가능 판정을 K 회 반복해 **흔들림 자체를 측정**하고 종료(0=사용 안 함).
        #   판정 잡음이 '수확하면 뒤가 열린다' 신호보다 크면 그 효과는 측정 자체가 불가능하다.
        self.declare_parameter("reach_repeat", 0)
        # 센싱 장면에서 **옥토맵이 생길 때까지** 기다리는 한도[s]. 빈 지도 위에서 기준선을
        #   잡으면 전부 '수확 가능'으로 나왔다가 지도가 자라며 사라진다(실측 4 → 0).
        self.declare_parameter("octomap_wait", 60.0)
        # prefer_near_home = pre-grasp 자세 후보를 **home 과 관절거리가 가까운** 것으로 우선 선택
        #   → phase ①(home→pre-grasp)에서 손목/관절이 크게 뒤집혀 도는 동작을 없앤다.
        #   (false 면 종전대로 접근각 자연스러움(prior)만으로 선택)
        self.declare_parameter("prefer_near_home", True)
        # interactive = 실제 로봇 조작하듯 **사용자가 타깃을 골라 명령**하는 모드. 자동 수확 대신
        #   /harvest_targets(목록 발행) + /harvest_cmd(선택 수신)으로 동작. harvest_operator 로 조작.
        self.declare_parameter("interactive", False)
        # reachable_only = 목록/선택 대상을 **워크스페이스 내 수확 가능한 열매**로 한정(도달·충돌 통과).
        #   arm_reach = 기하 프리필터용 팔 최대 도달반경[m](link0 기준) — IK 호출 전 명백히 먼 열매 제거.
        self.declare_parameter("reachable_only", True)
        self.declare_parameter("arm_reach", 1.0)
        # ── 전략·planner 비교 측정(bench_strategy) ─────────────────────
        #   같은 표본 열매에 대해 (전략 × planner) 조합마다 **전체 수확 궤적을 계획**해
        #   계획시간·경로길이(관절/TCP)·성공률·충돌 웨이포인트를 잰다. 재생/실행은 하지 않는다.
        #   ⚠ OMPL 은 확률적이라 1회 측정은 흔들린다 → bench_repeat 로 반복 평균.
        self.declare_parameter("bench_strategy", False)
        self.declare_parameter("bench_strategies", ["harvest_linear", "direct"])
        self.declare_parameter("bench_planners",
                               ["RRTConnect", "RRT", "RRTstar", "BiTRRT", "EST", "PRM"])
        self.declare_parameter("bench_repeat", 3)
        self.declare_parameter("bench_out", "/tmp/bench_strategy")   # .json/.csv 로 저장

        gp = self.get_parameter
        self.world = gp("world_frame").value
        self.group = gp("group").value
        self.ik_link = gp("ik_link").value
        self.rate = float(gp("rate").value)
        self.strategy = str(gp("strategy").value).strip().lower()
        self.planner_id = str(gp("planner_id").value).strip()
        self.execute = bool(gp("execute").value)
        self.show_col = bool(gp("show_collisions").value)

        # ---- 현재 관절 상태(이 노드가 유일 발행자) : home + 그리퍼 벌림(open) ----
        self.cur = {j: 0.0 for j in self.ARM}
        _go = float(gp("gripper_open").value)
        self.cur.update({f: _go for f in self.FINGERS})

        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # ⚠ execute 모드에선 gazebo joint_state_broadcaster 가 /joint_states 를 발행하므로
        #   이 노드는 발행하지 않는다(이중 발행 충돌 방지). 대신 구독해 실제 상태를 추종.
        self.js_pub = None if self.execute else self.create_publisher(JointState, "joint_states", 10)
        self.mk_pub = self.create_publisher(MarkerArray, "pregrasp_markers", latched)
        self.state_pub = self.create_publisher(DisplayRobotState, "pregrasp_robot_state",
                                               latched)
        # 재생 중 접촉점 표시(show_collisions). 라칭 — 멈춘 순간의 접촉이 화면에 남는다.
        self.col_pub = self.create_publisher(MarkerArray, "collision_markers", latched)
        self._col_hit = 0        # 이번 사이클 누적 충돌 표본 수
        self._col_n = 0          # 이번 사이클 총 조회 표본 수(비율의 분모)
        self._col_marks = []     # 이번 사이클 접촉점(누적 표시용)
        self._col_pairs = {}     # 접촉 쌍별 횟수

        if self.execute:
            # 컨트롤러 액션 클라이언트. (계획은 upfront·가상 시작상태로 하고 캐시 궤적을 실행하므로
            #  실제 상태 readback 은 불필요 — SetPosition 실행이 정확해 로봇이 계획을 그대로 따른다.
            #  /joint_states 구독은 오히려 _precompute 의 가상 시작상태를 덮어써 방해하므로 안 함.)
            self._arm_ac = ActionClient(
                self, FollowJointTrajectory,
                f"/{gp('arm_controller').value}/follow_joint_trajectory")
            self._grip_ac = ActionClient(
                self, FollowJointTrajectory,
                f"/{gp('gripper_controller').value}/follow_joint_trajectory")
            self.get_logger().info("execute 모드: FollowJointTrajectory 로 실구동(재생 안 함).")
        else:
            self._arm_ac = self._grip_ac = None
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

        # 안전 감시(safety_monitor)의 결함 래치를 따라간다. 실기에서만 발행되지만
        # 구독은 항상 걸어 둔다 — 없으면 조용히 안 걸리고, 있으면 즉시 반영된다.
        # (TRANSIENT_LOCAL 래치라 늦게 붙어도 이미 난 결함을 받는다.)
        self._aborted = False
        _qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, str(gp("abort_topic").value),
                                 self._on_abort, _qos)
        # Stage 5: 공간 기반 ACM 상태
        self._clear_octomap = self.create_client(Empty, "clear_octomap")
        self._acm_mode = str(gp("acm_mode").value).strip().lower()
        self._tgt_geom = {}          # name -> (center, radius) : 이름→기하 조회(구 영역 배치용)
        self._tgt_axis = {}          # name -> 줄기 축(단위) : cut 모드 전용
        self._axis_at = {}           # 좌표 → 줄기 축 : solve_pregrasp 가 이름 없이 호출돼도 찾게
        self._zone_at = None         # 현재 배치된 구 영역 중심(중복 적용 방지)
        self._zone_allowed = []      # 구 영역이 ACM 허용시킨 명명 객체들(되돌리기용)
        # 수확 완료로 장면에서 없앤 열매(실제 수확처럼) — 목록·재계획에서 제외한다.
        self._harvested = set()
        self._harvest_pub = self.create_publisher(
            String, str(gp("harvested_topic").value),
            QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=QoSHistoryPolicy.KEEP_LAST))
        self._del_entity = None      # Gazebo /delete_entity (있을 때만 lazy 생성)
        self._hv_why = {}            # 수확가능 판정 탈락 사유 집계(라운드마다 초기화)
        self._hv_pairs = {}          # 탈락 자세를 막은 접촉 쌍(진단용)
        self._grip_links = []        # 그리퍼 링크(파지한 열매의 touch_links)
        self._attached = None        # 현재 그리퍼에 부착된 열매 객체 id
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
        self._fk = None                      # (joints, chain) — 로컬 FK(경로길이 측정용)
        self._link_names = set()             # 로봇 링크 이름(ACM 검증용)
        if urdf and srdf:
            try:
                info = RI.playback_joints(srdf, urdf, self.group,
                                          self.get_parameter("gripper_group").value)
                if info["arm"]:
                    self.ARM = info["arm"]
                if info["gripper_all"]:
                    self.FINGERS = info["gripper_all"]
                # 그리퍼 링크 = 파지한 열매의 `touch_links`(붙잡은 열매가 손가락과 닿는 건
                #   충돌이 아니다). 손(부착 링크)까지 포함.
                self._grip_links = sorted(
                    {info["joints"][j]["child"] for j in info["gripper_all"]
                     if j in info["joints"]}
                    | {info["joints"][j]["parent"] for j in info["gripper_all"]
                       if j in info["joints"]})
                chain = RI.fk_chain(info["joints"], info["child_to_joint"],
                                    self.get_parameter("base_link").value, self.ik_link)
                if chain:
                    self._fk = (info["joints"], chain)
                # 로봇 링크 이름 집합 — ACM 검증에서 '월드 객체'와 '로봇 링크'를 구분하는 데 쓴다.
                for j in info["joints"].values():
                    self._link_names.update((j["parent"], j["child"]))
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
        # C: 파지 거리 — 그리퍼 손가락이 접근축으로 차지하는 실제 깊이에서 유도(모델 불문)
        self._pad_span = None
        self.grasp_offset = float(self.get_parameter("grasp_offset").value)
        if bool(self.get_parameter("grasp_offset_auto").value) and urdf and srdf:
            try:
                span = RI.gripper_span(urdf, srdf, self.ik_link, self.approach_axis,
                                       self.get_parameter("gripper_group").value,
                                       mesh_resolver=self._resolve_mesh)
            except Exception as e:                          # noqa: BLE001
                span, _ = None, self.get_logger().warn(f"그리퍼 깊이 측정 실패({e}) → 상수 사용")
            # ★ 건전성 검사 — 손가락이 **접근축 방향으로 뻗어야** 이 값이 의미가 있다.
            #   접근축이 그 그리퍼와 안 맞으면(SRDF 가 옛 그리퍼 기준이면) 깊이가 0 을 중심으로
            #   대칭으로 나온다(실측: Robotiq 2F-85 에서 −7.4~+7.4cm = 옆으로 잰 값).
            #   그걸 그대로 쓰면 파지점이 **TCP 원점**이 되어 손목으로 열매를 들이받는다.
            ok = bool(span) and span[1] > 0.02 and span[1] > abs(span[0]) * 1.5
            if ok:
                gd = float(self.get_parameter("grasp_depth").value)
                pad_c = None
                if gd < 0.0:
                    # auto = **파지면 면적중심**(사용자 규칙: 구 중심 깊이에 집게의 잡는 부분)
                    try:
                        ca = RI.gripper_closing_axis(urdf, srdf, self.ik_link,
                                                     self.approach_axis,
                                                     self.get_parameter("gripper_group").value)
                        pad_c = RI.gripper_pad_center(
                            urdf, srdf, self.ik_link, self.approach_axis,
                            ca or [1.0, 0.0, 0.0],
                            self.get_parameter("gripper_group").value,
                            mesh_resolver=self._resolve_mesh)
                    except Exception as e:                  # noqa: BLE001
                        self.get_logger().warn(f"파지면 중심 유도 실패({e})")
                    if pad_c is None:
                        self.get_logger().warn(
                            "파지면 중심을 못 구했다(mesh 없음?) → grasp_depth 0.50 으로 폴백")
                        gd = 0.50
                if pad_c is not None:
                    pc, palm = float(pad_c[0]), float(pad_c[1])
                    rr0 = float(self.get_parameter("fruit_radius").value)
                    clr = float(self.get_parameter("palm_clearance").value)
                    need = palm + rr0 + clr        # 손바닥이 열매를 파고들지 않을 최소 거리
                    auto = max(pc, need)
                    f = (auto - span[0]) / max(1e-9, span[1] - span[0])
                    self.get_logger().info(
                        f"파지 깊이 auto — 손 앞으로 나온 집게 구간의 파지면 면적중심 "
                        f"{pc*100:.2f}cm · 손바닥 앞면 {palm*100:.2f}cm "
                        f"(+열매 {rr0*100:.1f}cm +여유 {clr*100:.1f}cm ⇒ 최소 {need*100:.2f}cm) "
                        f"→ 파지거리 {auto*100:.2f}cm"
                        + ("" if auto <= pc + 1e-9 else " ★손바닥 여유가 지배")
                        + f" (패드 구간 {span[0]*100:.1f}~{span[1]*100:.1f}cm 의 {f:.3f} 지점)")
                else:
                    f = min(max(gd, 0.0), 1.0)
                    auto = span[0] + f * (span[1] - span[0])
                # 열매가 패드 어디에 물리는지 함께 남긴다 — 값만으로는 '손끝에 걸렸는지
                #   집게 구동부에 박혔는지' 를 알 수 없다(2026-08-21 사용자 지적).
                rr = float(self.get_parameter("fruit_radius").value)
                self.get_logger().info(
                    f"파지 거리 자동유도 — 손가락 패드 깊이 {span[0]*100:.1f}~{span[1]*100:.1f}cm"
                    f"(tcp 기준, 접근축 투영) · grasp_depth={f:.2f} → grasp_offset "
                    f"{self.grasp_offset*100:.1f} → {auto*100:.1f}cm · "
                    f"열매(r={rr*100:.1f}cm)가 패드 {(auto - rr)*100:.1f}~{(auto + rr)*100:.1f}cm "
                    f"구간을 점유 · 손끝 여유 {(span[1] - auto - rr)*100:+.1f}cm")
                self.grasp_offset, self._pad_span = auto, span
            else:
                self.get_logger().warn(
                    "그리퍼 손가락이 접근축 방향으로 뻗지 않는다"
                    + (f"(깊이 {span[0]*100:.1f}~{span[1]*100:.1f}cm — 0 대칭이면 옆으로 잰 것)"
                       if span else "(측정 실패)")
                    + f" → grasp_offset 상수 {self.grasp_offset*100:.1f}cm 유지. "
                      "그리퍼를 바꿨다면 `gen_srdf.py` 를 다시 돌려 SRDF/접근축을 맞출 것.")
        # D: 절단 모드 — 닫힘축과 날 위치. 파지는 접근축만 알면 되지만 절단은 **줄기가
        #    패드 사이를 가로질러야** 하므로 닫힘축이 따로 필요하다(모델 불문 유도).
        self.cut_mode = str(self.get_parameter("harvest_mode").value).strip().lower() == "cut"
        self.closing_axis = [1.0, 0.0, 0.0]
        self.cut_offset = self.grasp_offset
        if urdf and srdf:
            try:
                ca = RI.gripper_closing_axis(urdf, srdf, self.ik_link, self.approach_axis,
                                             self.get_parameter("gripper_group").value)
                if ca:
                    self.closing_axis = ca
                elif self.cut_mode:
                    self.get_logger().warn(
                        "닫힘축 자동유도 실패 → tcp X 가정. 절단 자세가 틀어질 수 있다.")
            except Exception as e:                          # noqa: BLE001
                self.get_logger().warn(f"닫힘축 유도 실패({e}) → tcp X 가정")
        if self._pad_span:      # 날은 패드 중앙(cut_depth) — 파지점(0.33)보다 깊다
            f = min(max(float(self.get_parameter("cut_depth").value), 0.0), 1.0)
            self.cut_offset = self._pad_span[0] + f * (self._pad_span[1] - self._pad_span[0])
        if self.cut_mode:
            self.get_logger().info(
                f"수확 모드 = cut(화방대 절단) · 닫힘축(tcp)"
                f"{[round(v, 3) for v in self.closing_axis]} · 날 위치 "
                f"{self.cut_offset*100:.1f}cm(cut_depth="
                f"{float(self.get_parameter('cut_depth').value):.2f}) · 절단점 비율 "
                f"{float(self.get_parameter('cut_ratio').value):.2f}")

        # 관절 이름이 바뀌었을 수 있으니 현재자세 dict 재구성(그리퍼는 벌림)
        gopen = float(self.get_parameter("gripper_open").value)
        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        self.get_logger().info(
            f"모델 자동설정 — arm{self.ARM} gripper{self.FINGERS} "
            f"접근축(tcp){[round(v,3) for v in self.approach_axis]} "
            f"파지거리 {self.grasp_offset*100:.1f}cm")

    @staticmethod
    def _resolve_mesh(uri):
        """URDF 의 `package://pkg/rel/path` → 로컬 절대경로(없으면 None)."""
        import os
        if not uri:
            return None
        if uri.startswith("file://"):
            return uri[7:]
        if not uri.startswith("package://"):
            return uri if os.path.exists(uri) else None
        pkg, _, rel = uri[len("package://"):].partition("/")
        try:
            from ament_index_python.packages import get_package_share_directory
            p = os.path.join(get_package_share_directory(pkg), rel)
        except Exception:                                   # noqa: BLE001
            return None
        return p if os.path.exists(p) else None

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
        if self.execute:      # broadcaster 가 발행 → 이 노드는 발행 안 함
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self.cur.keys())
        js.position = [float(self.cur[j]) for j in js.name]
        self.js_pub.publish(js)

    def _set(self, joint_vals):
        for k, v in joint_vals.items():
            if k in self.cur:
                self.cur[k] = float(v)

    def _on_abort(self, msg):
        if bool(msg.data) and not self._aborted:
            self.get_logger().error("안전 감시 결함 래치 수신 — 이후 실행을 중단한다")
        self._aborted = bool(msg.data)

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
        """waypts = [pos_array,...] (각 array 는 names 순서). duration 초에 걸쳐 선형보간 재생.
        execute 모드면 재생 대신 컨트롤러(FollowJointTrajectory)로 실제 실행."""
        if not waypts:
            return self.DONE
        if self.execute:
            return self._execute_traj(names, waypts, duration)
        dt = 1.0 / self.rate
        nsteps = max(1, int(duration * self.rate))
        segs = len(waypts) - 1
        probe_every = max(1, int(round(
            self.rate / max(0.1, float(self.get_parameter("collision_probe_hz").value)))))
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
            if self.show_col and (s % probe_every == 0):
                self._probe_collision()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)
        return self.DONE      # 재생 모드는 장면과 무관하므로 항상 완주

    def _probe_collision(self):
        """[표시 전용] 지금 자세가 장면과 충돌하는지 조회해 접촉점을 마커로 띄운다.

        🔴 재생 모드에는 물리가 없다 — 회피 없는 궤적이 줄기를 관통해도 화면에서는
        아무 일도 안 일어난다. 그래서 '무엇을 언제 뚫었는지'를 보이게 만드는 표시가
        비교의 전부다. 판정 기준은 계획 때와 같은 planning scene·ACM 을 그대로 쓴다
        (별도 기준을 쓰면 '충돌' 이 측정자 탓인지 궤적 탓인지 못 가린다)."""
        if self._sv is None:
            return
        req = GetStateValidity.Request()
        req.group_name = self.group
        js = JointState()
        js.name = list(self.cur.keys())
        js.position = [float(v) for v in self.cur.values()]
        req.robot_state.joint_state = js
        req.robot_state.is_diff = True
        res = self._call(self._sv, req, 1.0)
        if res is None:
            return
        self._col_n += 1
        if res.valid or not res.contacts:
            return                    # 🔴 지우지 않는다 — 아래 참조(누적)
        self._col_hit += 1
        # ★ 접촉점을 **사이클 동안 누적**한다. 순간만 띄우면 궤적이 어디를 스쳤는지
        #   한 장에 안 남고(캡처·GIF 에서 대부분 프레임이 비어 보인다), 무엇보다
        #   "몇 번 닿았나" 를 눈으로 셀 수가 없다. 사이클 시작에서만 지운다.
        for c in res.contacts[:20]:
            k = f"{c.contact_body_1}|{c.contact_body_2}"
            self._col_pairs[k] = self._col_pairs.get(k, 0) + 1
            self._col_marks.append((float(c.position.x), float(c.position.y),
                                    float(c.position.z)))
        arr = MarkerArray()
        for i, (x, y, z) in enumerate(self._col_marks[-200:]):
            m = Marker()
            m.header.frame_id = self.world
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = "collision", i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            # ⚠ 열매가 빨강이다 — 접촉 마커까지 빨강이면 화면에서 구분이 안 된다(실측).
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.92, 0.0, 1.0
            arr.markers.append(m)
        t = Marker()
        t.header.frame_id = self.world
        t.header.stamp = self.get_clock().now().to_msg()
        t.ns, t.id, t.type, t.action = "collision", 9000, Marker.TEXT_VIEW_FACING, Marker.ADD
        t.pose.position = res.contacts[0].position
        t.pose.position.z += 0.22
        t.pose.orientation.w = 1.0
        t.scale.z = 0.07
        t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 0.92, 0.0, 1.0
        t.text = f"충돌 {self._col_hit}/{self._col_n}"
        arr.markers.append(t)
        self.col_pub.publish(arr)
        pair = f"{res.contacts[0].contact_body_1} ↔ {res.contacts[0].contact_body_2}"
        self.get_logger().warn(f"  💥 충돌 {len(res.contacts)}건 — {pair}")

    #: _execute_traj / _play_waypoints 반환값
    DONE, REPLAN, FAILED = "done", "replan", "failed"
    #: 감시 1회에 검사할 웨이포인트 최대 개수(위 step 계산 참조)
    REPLAN_MAX_CHECKS = 6

    def _sync_actual_joints(self, timeout=2.0):
        """🔴 execute 모드에서 궤적을 중간에 끊은 뒤 **실제 관절값을 한 번 읽어** `self.cur` 에 넣는다.

        왜 필요한가 — 이 노드는 execute 모드에서 `/joint_states` 를 **구독하지 않는다**.
        계획을 미리(upfront) 해 두고 캐시한 궤적을 실행하며, `self.cur` 는 '로봇이 계획을
        그대로 따랐다' 는 가정 위의 **가상 상태**다(상단 __init__ 주석 참조 — 상시 구독은
        `_precompute` 의 가상 시작상태를 덮어써 방해한다).
        그런데 재계획은 **끊긴 실제 자세에서** 시작해야 한다. 가상 상태로 계획하면 로봇이
        있지도 않은 자세에서 출발하는 궤적이 나오고, 실행 순간 관절이 튄다.
        ⇒ 상시 구독은 하지 않고, **끊은 그 순간에만** 한 번 읽는다(그 시점엔 캐시 궤적을
        이미 버리는 것이므로 가상 상태를 덮어써도 잃을 게 없다).
        """
        if not self.execute:
            return True          # 재생 모드는 가상 상태가 곧 진실
        try:
            from rclpy.wait_for_message import wait_for_message
        except ImportError:      # 아주 오래된 rclpy
            self.get_logger().error("wait_for_message 를 쓸 수 없다 → 실제 자세 동기화 불가")
            return False
        ok, msg = wait_for_message(JointState, self, "joint_states",
                                   time_to_wait=float(timeout))
        if not ok or msg is None:
            self.get_logger().error(
                f"실제 관절값을 읽지 못했다(/joint_states {timeout}s) → 재계획을 포기한다"
                "(틀린 시작상태로 계획하면 관절이 튄다)")
            return False
        moved = 0.0
        for n, p in zip(msg.name, msg.position):
            if n in self.cur:
                moved = max(moved, abs(float(p) - float(self.cur[n])))
                self.cur[n] = float(p)
        self.get_logger().info(
            f"실제 자세 동기화 — 가정했던 자세와 최대 {moved:.3f}rad 차이")
        return True

    def _retreat_safely(self, goals, name):
        """구간 중간에서 중단됐을 때 팔을 빼낸다. goals = [(관절dict, 라벨), …] 순서대로 시도.

        어디쯤에서 멈췄는지 모르므로 역재생을 쓸 수 없다 → 현재 자세에서 다시 계획한다.
        🔴 전부 실패하면 **움직이지 않는다.** 갈 길을 모르는 채로 관절을 돌리는 것이
        작물 사이에서 가장 위험하다 — 사람이 보고 판단하도록 그 자리에 세워 두고 알린다.
        ⚠ 다음 사이클이 '지금 home 에 있다' 고 가정하므로, 이탈 성공/실패를 반환해
        호출자가 그 가정을 쓸 수 있는지 알 수 있게 한다."""
        if not self._sync_actual_joints():
            self.get_logger().error(
                f"[{name}] 🔴 실제 자세를 모르는 채로는 이탈 계획을 세우지 않는다 — 정지 유지")
            return False
        for goal, label in goals:
            plan = self.plan_to(goal)
            if plan is None:
                continue
            self.get_logger().info(f"[{name}] 중단 지점에서 {label} 로 이탈")
            if self._play_waypoints(*plan, float(self.get_parameter("dur_home").value)) \
                    == self.DONE:
                self._set(dict(goal))
                return True
        self.get_logger().error(
            f"[{name}] 🔴 이탈 경로를 찾지 못했다 — 팔을 그 자리에 세워 둔다. "
            "장면(옥토맵)과 팔 자세를 사람이 확인해야 한다.")
        return False

    def _play_replan(self, names, waypts, duration, replan_fn, what):
        """구간을 실행하고, 실행 중 막히면 `replan_fn()` 으로 다시 계획해 재시도한다.

        replan_fn() → (names, waypts) 또는 None(계획 실패).
        반환: DONE · REPLAN(재시도를 다 썼는데도 막힘) · FAILED

        🔴 왜 구간마다 다르게 다루나
          자유공간 구간(home↔pre-grasp)은 다시 계획하면 된다. 반면 **직선 접근 구간은
          재계획 대상이 아니다** — 열매를 향해 곧게 들어가는 게 그 구간의 정의라서,
          중간에 경로를 바꾸면 파지 기하가 깨진다. 그 경우는 이 열매를 포기하는 게 맞다
          (호출자가 replan_fn=None 으로 넘겨 REPLAN 을 그대로 받는다)."""
        tries = max(0, int(self.get_parameter("replan_max").value))
        for attempt in range(tries + 1):
            st = self._play_waypoints(names, waypts, duration)
            if st != self.REPLAN:
                return st
            if replan_fn is None:
                return self.REPLAN
            if attempt >= tries:
                self.get_logger().error(
                    f"[{what}] 재계획 {tries}회를 다 썼는데도 막혔다 → 포기")
                return self.REPLAN
            self.get_logger().info(
                f"[{what}] 재계획 {attempt + 1}/{tries} — 현재 자세에서 다시 계획한다")
            new = replan_fn()
            if new is None:
                self.get_logger().error(f"[{what}] 재계획 실패(계획 자체가 안 나온다) → 포기")
                return self.REPLAN
            names, waypts = new
        return self.REPLAN

    def _execute_traj(self, names, waypts, duration):
        """execute 모드: waypts 를 FollowJointTrajectory 로 컨트롤러에 보내 실제 실행.

        반환: DONE(완주) · REPLAN(장면이 막혀 중단 — 다시 계획해야 함) · FAILED(보낼 수 없었음)

        완료를 그냥 기다리지 않고 **실행 중 남은 경로를 재검사**한다(replan=true 기본).
        팔이 움직이면 eye-in-hand 카메라가 새 복셀을 쌓아 장면이 계획 당시와 달라지는데,
        기다리기만 하면 그 사실을 알 방법이 없다."""
        nameset = set(names)
        if nameset & set(self.FINGERS):
            ac, ctrl_joints = self._grip_ac, ["rg2_finger_joint1"]  # 나머지 finger 는 mimic
        else:
            ac, ctrl_joints = self._arm_ac, list(self.ARM)
        idx = {n: names.index(n) for n in ctrl_joints if n in names}
        js = [j for j in ctrl_joints if j in idx]
        if not js:
            return self.DONE
        traj = JointTrajectory()
        traj.joint_names = js
        n = len(waypts)
        for k, wp in enumerate(waypts):
            pt = JointTrajectoryPoint()
            pt.positions = [float(wp[idx[j]]) for j in js]
            t = duration * (k + 1) / n            # 모두 양수·단조증가(첫 점도 t>0)
            pt.time_from_start = DurationMsg(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            traj.points.append(pt)
        if not ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "컨트롤러 액션 서버 없음 — gazebo 가 control 모드(execute:=true)로 떴는지 확인.")
            return self.FAILED
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        gh_fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_fut, timeout_sec=15.0)
        gh = gh_fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn("궤적 goal 거부/무응답.")
            return self.FAILED

        rf = gh.get_result_async()
        # 그리퍼 구간은 감시하지 않는다 — 손가락만 닫는 동작이라 재계획할 경로가 없다.
        watch = (bool(self.get_parameter("replan").value)
                 and self._sv is not None and not (nameset & set(self.FINGERS)))
        if not watch:
            rclpy.spin_until_future_complete(self, rf, timeout_sec=duration + 20.0)
            return self.DONE

        period = float(self.get_parameter("replan_period").value)
        sample = max(1, int(self.get_parameter("replan_sample").value))
        look = float(self.get_parameter("replan_lookahead").value)
        t0 = time.time()
        deadline = t0 + duration + 20.0
        next_check = t0 + period
        while not rf.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            if now < next_check:
                continue
            next_check = now + period

            if self._aborted:
                self.get_logger().error("안전 감시가 결함을 알렸다 → 실행 중단")
                gh.cancel_goal_async()
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.05)
                self._sync_actual_joints()
                return self.FAILED

            # 진행률로 남은 구간을 잡는다(컨트롤러 피드백 대신 경과시간 — 컨트롤러가
            # 시간 파라미터화대로 따라간다는 가정. 정밀하지 않아도 lookahead 로 덮인다).
            frac = min(1.0, (now - t0) / max(1e-6, duration)) + look
            start = int(frac * len(waypts))
            rest = waypts[start:]
            if len(rest) < 2:
                continue
            # ⚠ 검사 1회의 비용을 묶는다. `_invalid_waypoints` 는 웨이포인트마다
            #   check_state_validity 를 **동기 호출**(최대 3s)한다 — 남은 점이 많으면 한 번의
            #   검사가 구간 길이보다 오래 걸려 감시가 실행을 앞질러 버린다.
            step = max(sample, -(-len(rest) // self.REPLAN_MAX_CHECKS))
            pairs = {}
            bad = self._invalid_waypoints(names, rest, sample=step, pairs=pairs)
            if bad:
                top = sorted(pairs.items(), key=lambda kv: -kv[1])[:3]
                self.get_logger().warn(
                    f"실행 중 장면 변화 감지 — 남은 {len(rest)}점 중 {bad}점이 막혔다"
                    f"(진행 {frac*100:.0f}%) · 접촉: "
                    + ", ".join(f"{k}×{v}" for k, v in top))
                gh.cancel_goal_async()
                # 취소가 반영될 틈을 준다(안 주면 다음 계획의 시작 상태가 아직 움직이는 중이다)
                for _ in range(20):
                    rclpy.spin_once(self, timeout_sec=0.05)
                # 🔴 그 다음 반드시 실제 자세를 읽는다 — self.cur 는 가상 상태다.
                if not self._sync_actual_joints():
                    return self.FAILED
                return self.REPLAN
        if not rf.done():
            self.get_logger().warn("궤적 결과 대기 타임아웃.")
            return self.FAILED
        return self.DONE

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
        if self.planner_id:                       # 선택한 OMPL 알고리즘(RRTConnect/RRTstar/PRM/…)
            mpr.planner_id = self.planner_id
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
        t0 = time.time()
        res = self._call(self.plan, req, mpr.allowed_planning_time + 2.0)
        ok = res is not None and res.motion_plan_response.error_code.val == 1
        st = getattr(self, "_plan_stat", None)     # bench_strategy 측정용(평소엔 None)
        if st is not None:
            st["calls"] += 1
            st["t"] += time.time() - t0
            if not ok:
                st["fail"] += 1
        if not ok:
            return None
        return self._traj_to_waypts(res.motion_plan_response.trajectory.joint_trajectory)

    def cartesian_to(self, pose_goal, avoid=True):
        """현재→목표 TCP pose 직선(Cartesian) 경로. (names, waypts, fraction) 또는 None.

        avoid=False 면 **충돌을 보지 않고** 직선을 뽑는다 — 회피 없는 대조군 전용."""
        req = GetCartesianPath.Request()
        req.header.frame_id = self.world
        req.start_state = self._robot_state()
        req.group_name = self.group
        req.link_name = self.ik_link
        req.max_step = 0.01
        # 기본 0(끔) — 관절 점프/과회전은 `_jl_of` 로 **경로 전체를 보고** 판정한다(부분
        # 절단보다 후보 교체가 낫다). 실험용으로 MoveIt 필터도 파라미터로 열어둔다.
        req.jump_threshold = float(self.get_parameter("cartesian_jump").value)
        req.revolute_jump_threshold = float(
            self.get_parameter("cartesian_revolute_jump").value)
        req.avoid_collisions = bool(avoid)
        req.waypoints = [pose_goal]
        res = self._call(self.cart, req, 5.0)
        if res is None or not res.solution.joint_trajectory.points:
            return None
        names, wp = self._traj_to_waypts(res.solution.joint_trajectory)
        return names, wp, float(res.fraction)

    @staticmethod
    def _jl_of(wp):
        """웨이포인트 열의 (관절 경로길이[rad], 최대 1스텝 관절변화[rad]).
        직선 접근이 '기하학적으로만' 직선이고 관절은 크게 도는 해를 걸러내는 지표."""
        if not wp or len(wp) < 2:
            return 0.0, 0.0
        a = np.asarray(wp, float)
        d = np.diff(a, axis=0)
        return (float(np.sum(np.linalg.norm(d, axis=1))), float(np.max(np.abs(d))))

    def _straight_ok(self, wp, tag=""):
        """직선 Cartesian 결과가 **관절 관점에서도 건전한지**. (ok, jl, jmax).

        TCP 직선(fraction=1.00)이어도 손목이 뒤집히며 관절이 크게 도는 해가 섞여 나오고,
        그 해에서만 손가락이 주 줄기에 닿았다(2026-07-29 실측). 한도는 `approach_jl_max`."""
        jl, jmax = self._jl_of(wp)
        lim = float(self.get_parameter("approach_jl_max").value)
        ok = (lim <= 0.0) or (jl <= lim)
        if not ok:
            self.get_logger().warn(
                f"직선 경로가 관절을 과도하게 돌린다{tag}: 관절 {jl:.2f}rad "
                f"(스텝최대 {jmax:.2f}rad) > 한도 {lim:.2f}rad → 직선으로 인정 안 함")
        return ok, jl, jmax

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

    def _allow_cut_only(self, obj_name):
        """cut 모드 ACM — **지금 자를 그 객체 하나만** 허용하고, 앞서 허용했던 절단 대상은
        도로 막는다.

        🔴 이게 없으면 조용히 오염된다: 21개 화방대를 훑으며 하나씩 허용만 하면 뒤로 갈수록
        앞서 훑은 화방대가 전부 '없는 것'이 되어, 뒤쪽 판정이 실제보다 후해진다(이 프로젝트에서
        여러 번 데인 '판정이 장면을 더럽힌다' 부류). 허용/해제를 **한 번의 apply 로 함께** 건다."""
        if self.apply_scene is None:
            return False
        prev = getattr(self, "_cut_allowed", set())
        if obj_name is not None and prev == {obj_name}:
            return True                                   # 이미 그 상태 — 왕복 생략
        if not self.apply_scene.wait_for_service(timeout_sec=2.0):
            return False
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        res = self._call(self.scene, req, 2.0)
        if res is None:
            return False
        acm = res.scene.allowed_collision_matrix
        want = {} if obj_name is None else {str(obj_name): True}
        for old in prev:
            if old != obj_name:
                want[str(old)] = False                     # 도로 장애물로
        for nm, val in want.items():
            if nm in acm.default_entry_names:
                acm.default_entry_values[list(acm.default_entry_names).index(nm)] = val
            else:
                acm.default_entry_names.append(nm)
                acm.default_entry_values.append(val)
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.allowed_collision_matrix = acm
        ares = self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 2.0)
        ok = ares is not None and ares.success
        if ok:
            self._cut_allowed = set() if obj_name is None else {str(obj_name)}
        return ok

    def _cut_pairs(self, q, pairs, tag):
        """자세 q 에서 무엇이 무엇에 닿는지 접촉 쌍을 세어 `pairs` 에 누적.

        '충돌로 막혔다'까지만 알면 고칠 수가 없다 — 주 줄기인지·거터인지·자기 몸인지에 따라
        처방이 완전히 다르다(각도 확대 / 베이스 이동 / 툴 형상). 서비스 왕복이 있으므로
        `screen_why_detail:=true` 일 때만 켠다."""
        if not bool(self.get_parameter("screen_why_detail").value):
            return
        try:
            _v, cnt = self._state_report({j: q[j] for j in self.ARM if j in q})
        except Exception:                                   # noqa: BLE001
            return
        for k in (cnt or []):
            key = f"{tag}:{k}"
            pairs[key] = pairs.get(key, 0) + 1

    def _cut_object_of(self, p_cut, name=None):
        """절단점 → 그 줄기의 **장면 객체 이름**(없으면 `cut_object` 파라미터, 그것도 없으면 None)."""
        if name is not None and str(name).startswith("rachis_"):
            return str(name)
        pt = np.asarray(p_cut, float)
        for nm, (q, _r) in self._tgt_geom.items():
            if nm.startswith("rachis_") and float(np.linalg.norm(np.asarray(q, float) - pt)) < 1e-6:
                return nm
        obj = str(self.get_parameter("cut_object").value).strip()
        return obj or None

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

    def _allow_region(self, center, rho, settle=True):
        """목표 열매 중심 반경 ρ 구 영역을 '수확 작업 공간'으로 허용한다.

        `settle=False` 면 옥토맵 침식 대기를 건너뛴다 — **선별(수확가능 판정)용**. 후보마다
        2초씩 기다리면 목록 도출이 수십 초 늘어난다(계획 직전에는 항상 True 로 건다)."""
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
        if settle and self._has_octomap():
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

    def _remember_target(self, name, p, r, axis=None):
        p = np.asarray(p, float)
        self._tgt_geom[str(name)] = (p, float(r))
        if axis is not None:
            u = np.asarray(axis, float)
            nu = float(np.linalg.norm(u))
            if nu > 1e-9:
                u = u / nu
                self._tgt_axis[str(name)] = u
                # ★ `solve_pregrasp(p, r)` 은 **이름을 안 받는다**(호출처 10곳). 좌표로도
                #   찾을 수 있게 함께 넣어 둔다 — 호출처를 전부 고치지 않고 축을 잇는 경로.
                self._axis_at[tuple(np.round(p, 6))] = u

    def _axis_for(self, p, name=None):
        """절단 대상 줄기 축(단위) 또는 None. 이름 → 좌표 → `cut_axis` 파라미터 순."""
        if name is not None and str(name) in self._tgt_axis:
            return self._tgt_axis[str(name)]
        u = self._axis_at.get(tuple(np.round(np.asarray(p, float), 6)))
        if u is not None:
            return u
        v = self.get_parameter("cut_axis").value
        if v and len(v) == 3 and not any(math.isnan(float(x)) for x in v):
            a = np.asarray([float(x) for x in v], float)
            n = float(np.linalg.norm(a))
            if n > 1e-9:
                return a / n
        return None

    def _allow_for_target(self, name, settle=True):
        """접근 계획 전 충돌 허용 적용. 방식은 `acm_mode`(region|stalk|none)로 결정한다.

        `settle=False` = 선별용(옥토맵 침식 대기 생략). 계획 직전 호출은 기본값(True)을 쓴다.

        ⚠ 이전에는 이 호출이 `plan_approach`/`_best_straight_candidate` 안에 이름 기반으로
        **하드코딩**돼 있어서, 비교실험의 'ACM 완화 없음' 조건에서도 목표 화방대가 허용됐다
        (조건 오염). 여기로 모아 모드로 제어한다."""
        mode = getattr(self, "_acm_mode", "region")
        if mode == "none":
            return False
        if self.cut_mode:
            # ★ 절단은 **점 하나**에 접근한다 → 구 영역을 쓰지 않는다.
            #   구(ρ)를 쓰면 그 안의 이웃 화방대·줄기까지 통째로 충돌 허용돼 '회피'가
            #   무의미해진다. 자를 대상 **그 객체 하나만** 이름으로 허용한다(날이 닿아야
            #   자르므로 그 하나는 반드시 허용해야 한다).
            obj = self._cut_object_of(self._tgt_geom.get(str(name), (np.zeros(3), 0.0))[0],
                                      str(name))
            if obj:
                return self._allow_cut_only(obj)
            self.get_logger().warn(
                f"cut 모드: 허용할 줄기 객체를 못 정했다(name={tgt}). 좌표 목표면 "
                "`cut_object:=<객체명>` 을 줄 것 — 없으면 날이 줄기에 닿는 순간 충돌로 기각된다.")
            return False
        if mode == "stalk":
            return self._allow_collision(self._stalk_of(name))
        g = self._tgt_geom.get(str(name))
        if g is None:                                   # 기하를 모르면 이름 기반으로 폴백
            return self._allow_collision(self._stalk_of(name))
        rho = g[1] + float(self.get_parameter("region_margin").value)
        return self._allow_region(g[0], rho, settle=settle)

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
            # ★ 첫 메시지에서 바로 시작하면 검출이 아직 쌓이는 중이다(실측: 3개에서 시작했는데
            #   곧 19개가 됐다). **개수가 안정될 때까지** 더 기다린다 — 안 그러면 첫 라운드의
            #   후보가 부당하게 적어 '수확하면 뒤가 열린다'를 볼 수가 없다.
            n_prev, same = -1, 0
            while time.time() - t0 < wait and same < 3:
                rclpy.spin_once(self._det_node, timeout_sec=0.5)
                n = len(self._det)
                same = same + 1 if n == n_prev else 0
                n_prev = n
                time.sleep(0.5)
            self.get_logger().info(f"인지 열매 {len(self._det)}개 수신"
                                   f" ({time.time() - t0:.1f}s, 개수 안정화 후)")
        else:
            rclpy.spin_once(self._det_node, timeout_sec=0.0)   # 최신 관측 반영
        return list(self._det)

    @staticmethod
    def _axis_from_rpy(rpy):
        """`_seg_cylinder` 가 쓴 rpy=(0, θ, φ) → 원통 축 방향(단위). Rz(φ)Ry(θ)·ez."""
        _, th, ph = [float(v) for v in rpy]
        return np.array([math.cos(ph) * math.sin(th),
                         math.sin(ph) * math.sin(th),
                         math.cos(th)])

    def _cut_targets(self):
        """**절단 목표 = 화방대(rachis) 전부** [(name, 절단점, 반경), ...].

        열매(kind:target)와 달리 화방대는 kind:obstacle 이다 — 자를 대상이지 잡을 대상이
        아니기 때문. obstacles.yaml 을 같은 방식으로 펼쳐 `rachis_*` 만 고른다.
        절단점 = 원통 중심에서 `cut_ratio` 만큼 이동(0=줄기쪽 끝, 1=열매쪽 끝, 0.5=중앙).
        축 u 는 rpy 에서 복원해 `_remember_target` 에 함께 저장한다 — **절단 자세는 이 축이
        없으면 못 푼다**(수직 접근·닫힘축 정렬이 전부 u 기준).

        ⚠ 인지(perception) 출처는 지원하지 않는다 — `fruit_detector` 는 빨강 세그로 열매만
          찾고 줄기는 안 낸다. cut 모드는 설계값(yaml) 장면 전용이다.
        """
        if str(self.get_parameter("target_source").value).lower().startswith("percep"):
            self.get_logger().warn(
                "cut 모드는 인지 타깃을 쓸 수 없다(줄기 인지 없음) → yaml 로 진행.")
        path = self.get_parameter("obstacles_file").value or self._op.default_yaml()
        import yaml
        data = yaml.safe_load(open(path)) or {}
        try:
            self._op.expand_crops(data)
        except Exception:
            pass
        ratio = min(max(float(self.get_parameter("cut_ratio").value), 0.0), 1.0)
        out = []
        for o in data.get("obstacles", []):
            nm = str(o.get("name", ""))
            if not nm.startswith("rachis_") or nm in self._harvested:
                continue
            if o.get("type") != "cylinder":
                continue
            c = np.array([float(v) for v in o["pose"]["xyz"]])
            u = self._axis_from_rpy(o["pose"].get("rpy", [0.0, 0.0, 0.0]))
            h = float(o.get("height", 0.0))
            r = float(o.get("radius", 0.004))
            p_cut = c + (ratio - 0.5) * h * u          # 중심 기준 ±h/2 구간 위의 한 점
            self._remember_target(nm, p_cut, r, axis=u)
            out.append((nm, p_cut, r))
        return out

    def _all_targets(self):
        """집기 목표 열매 전부 [(name, xyz, r), ...].

        `target_source` 로 출처를 고른다:
          · yaml       — obstacles.yaml 의 kind:target (설계값, 이름표 있음)
          · perception — 카메라 인지 결과 /detected_fruits (Stage 4, 이름표 없음)

        **cut 모드에서는 열매가 아니라 화방대(rachis)가 목표**다 → `_cut_targets()`.
        """
        if self.cut_mode:
            return self._cut_targets()
        if str(self.get_parameter("target_source").value).lower().startswith("percep"):
            # 인지 타깃은 Gazebo 에서 열매를 지우면 저절로 사라지지만, 삭제 직후 한두 프레임은
            # 남아 있을 수 있어 이름으로도 한 번 더 거른다.
            tg = [t for t in self._perception_targets() if t[0] not in self._harvested]
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
              for o in data.get("obstacles", [])
              if o.get("kind") == "target" and o["name"] not in self._harvested]
        for nm, p, r in tg:
            self._remember_target(nm, p, r)          # Stage 5: 구 영역 배치용 기하 기억
        return tg

    def _target(self):
        """단일 목표: param target 우선, 없으면 target_index 열매."""
        t = self.get_parameter("target").value
        r = float(self.get_parameter("fruit_radius").value)
        if t and len(t) == 3 and not any(math.isnan(float(v)) for v in t):
            pt = [float(v) for v in t]
            ax = None
            if self.cut_mode:
                v = self.get_parameter("cut_axis").value
                if v and len(v) == 3 and not any(math.isnan(float(x)) for x in v):
                    ax = [float(x) for x in v]
                else:
                    self.get_logger().error(
                        "cut 모드에서 좌표 목표를 주려면 `cut_axis:=\"[ux,uy,uz]\"`(줄기 축)도 "
                        "함께 줘야 한다 — 축이 없으면 '수직으로 들어간다'가 정의되지 않는다.")
            self._remember_target("param_target", pt, r, axis=ax)
            return "param_target", np.array(pt), r
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

    def _base_xyz(self):
        """팔 베이스(link0) 의 **world 3D 위치**. 거리 계산·정렬·도달 사전필터가 쓴다.

        🔴 종전에는 xy 만 TF 로 읽고 **z 를 0.35 로 하드코딩**했다(RB5 를 Scout 상판에 바로
           얹었을 때의 값). 팔 베이스 높이가 달라지는 구성(예: 스탠드/라이저를 끼운 구성)에서는
           그 값이 틀리고, 그러면 `_is_harvestable` 의 사전 필터가 **닿는 열매를 배제**한다
           (2026-08-18 실측으로 드러남: 스탠드 50cm 구성에서 거리 지표가 최대 0.5m 어긋났다).
           TF 에 이미 z 가 들어 있으므로 그냥 같이 읽는다.
        """
        self._ensure_tf()
        try:
            tf = self._tf.lookup_transform(self.world,
                                           self.get_parameter("base_link").value,
                                           rclpy.time.Time())
            t = tf.transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def solve_pregrasp(self, p_fruit, r, name=None):
        """자연스러운 접근 기하 + 후보 샘플링. (cut 모드면 `solve_cut` 으로 넘긴다.)

          · grasp 점  = p_fruit − a·grasp_offset  (TCP 가 열매 앞 grasp_offset 에서 파지)
          · pre-grasp = grasp − a·standoff         (grasp 에서 standoff 만큼 뒤로)
          → pre→grasp 직선이동 거리 = **정확히 standoff**. (사용자 요청 시퀀스)

        **pre-grasp 와 grasp 둘 다** IK/충돌 통과해야 채택 → 직선 접근이 실제로 가능한
        후보만 고른다(그래야 Cartesian 이 폴백 없이 곧게 들어간다).
        반환: dict(q_pre, q_grasp, a, p_pre, p_grasp, quat, c) 또는 None."""
        if self.cut_mode:
            u = self._axis_for(p_fruit, name)
            if u is None:
                self.get_logger().error(
                    "cut 모드인데 줄기 축을 모른다 — 좌표 목표면 `cut_axis:=\"[ux,uy,uz]\"` "
                    "를 함께 줄 것(축이 없으면 수직 접근 자체가 정의되지 않는다).")
                return None
            return self.solve_cut(np.asarray(p_fruit, float), u)
        d0 = float(self.get_parameter("standoff").value)
        goff = self.grasp_offset      # URDF 에서 유도(또는 파라미터 상수)
        prefer_home = bool(self.get_parameter("prefer_near_home").value)
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
            if prefer_home:
                # home(관절 0) 과의 L1 관절거리 우선 + prior 는 가벼운 보조항 → 가장 덜 도는 자세
                jd = sum(abs(float(q.get(j, 0.0))) for j in self.ARM)
                score = jd + 0.15 * c.prior
            else:
                score = c.prior
            if best is None or score < best[0]:
                best = (score, c, a, p_pre, p_grasp, quat, q, qg)
                if not prefer_home and c.prior == 0.0:
                    break                                    # 종전: 첫 정면각에서 종료
        if best is None:
            return None
        _, c, a, p_pre, p_grasp, quat, q, qg = best
        return dict(c=c, a=a, p_pre=p_pre, p_grasp=p_grasp, quat=quat, q=q, q_grasp=qg)

    def solve_cut(self, p_cut, u):
        """**화방대 절단 자세** — 줄기 축 u 에 수직으로 들어가 날이 줄기를 가로지르게.

        파지와 다른 점(둘 다 지켜야 자른다):
          ① 접근축 a ⟂ u — 비스듬히 들어가면 날이 줄기를 타고 미끄러진다.
          ② 닫힘축 c ⟂ u — c 가 u 와 나란하면 줄기가 패드 사이를 **가로지르지 못한다**.
        ①이 자유도를 1개(줄기 둘레 회전 β)로 줄이고, ②가 **롤을 계산값으로 고정**한다.
        ⇒ 파지 쪽 (φ,θ,ψ) 격자 샘플링은 여기 쓸 수 없다. 실측 근거: 현 온실 화방대에서
           롤 0°(종전 `sample_psi_deg` 의 유일한 값) 의 닫힘각은 **20.9°** — 원리적으로
           못 자르는 자세다. 필요한 롤은 −69.8°(β=0 기준).

        점 하나에 접근한다 — 열매처럼 반경 r 의 구로 두지 않는다(구로 두면 ρ 안의 이웃
        줄기·열매까지 통째로 충돌 허용돼 '피한다'는 말이 무의미해진다).

          · 날 위치   = p_cut − a·cut_offset   (패드 중앙이 줄기에 오도록)
          · pre-cut   = 날 위치 − a·standoff
        반환 형식은 `solve_pregrasp` 와 같다(전략·재생 코드가 그대로 쓴다)."""
        # ★ 절단은 **날이 대상에 닿아야** 성립한다 → 판정 자체가 그 객체 허용을 전제한다.
        #   (파지는 열매가 애초에 충돌객체가 아니라 이 단계가 필요 없었다. 실측으로 드러난
        #    두 모드의 구조적 차이 — 허용 없이 훑으면 21개 전부 IK 실패로 나온다.)
        if str(self.get_parameter("acm_mode").value).strip().lower() != "none":
            obj = self._cut_object_of(p_cut)
            if obj:
                self._allow_cut_only(obj)
            else:
                self.get_logger().warn(
                    "cut 모드: 허용할 줄기 객체를 못 정했다 → 좌표 목표면 `cut_object:=<객체명>` "
                    "을 줄 것(없으면 날이 닿는 순간 충돌로 기각된다).")
        d0 = float(self.get_parameter("standoff").value)
        off = self.cut_offset
        prefer_home = bool(self.get_parameter("prefer_near_home").value)
        base = self._base_xyz()
        if base is None:
            bxy = self._base_xy()
            base = np.array([bxy[0], bxy[1], 0.35]) if bxy is not None else np.zeros(3)
        a_nom = np.array([p_cut[0] - base[0], p_cut[1] - base[1], 0.0])   # 로봇→목표 수평
        if float(np.linalg.norm(a_nom)) < 1e-6:
            a_nom = np.array([1.0, 0.0, 0.0])
        betas = [math.radians(float(b)) for b in self.get_parameter("cut_beta_deg").value] or [0.0]
        best = None
        # 실패 원인 분류 — '도달 불가' 한 줄로 뭉개면 도달권 문제인지 줄기가 막는 건지 못 가른다.
        why = dict(back=0, ang=0, pre_ik=0, pre_kin=0, blade_ik=0, blade_kin=0)
        pairs = {}      # 무엇이 막았나(접촉 쌍) — screen_why_detail 일 때만 채운다
        for bi, beta in enumerate(betas):
            a = PG.approach_perpendicular(u, a_nom, beta)
            # 로봇 쪽에서 들어가는 방향만(반대편에서 찌르는 해는 팔이 작물을 관통해야 한다)
            if float(a @ (p_cut - base)) <= 0.0:
                why["back"] += 1
                continue
            rolls = PG.rolls_for_closing_perp(a, u, self.approach_axis, self.closing_axis)
            for ri, roll in enumerate(rolls):
                ang = PG.closing_angle_deg(a, roll, u, self.approach_axis, self.closing_axis)
                if ang < 60.0:        # 수직에서 30° 이상 벗어나면 절단으로 인정 안 함
                    why["ang"] += 1
                    continue
                quat = PG.mat_to_quat(PG.gaze_rotation(a, roll, self.approach_axis))
                p_blade = p_cut - a * off
                p_pre = p_blade - a * d0
                q = self.solve_ik(p_pre, quat, avoid=True)
                if q is None:
                    why["pre_ik"] += 1
                    # 충돌을 빼고도 안 되면 **기구학**(도달권/자세) 문제다 — 원인이 갈린다.
                    qf = self.solve_ik(p_pre, quat, avoid=False)
                    if qf is None:
                        why["pre_kin"] += 1
                    else:
                        self._cut_pairs(qf, pairs, "pre")
                    continue
                qg = self.solve_ik(p_blade, quat, avoid=True)
                if qg is None:
                    why["blade_ik"] += 1
                    qf = self.solve_ik(p_blade, quat, avoid=False)
                    if qf is None:
                        why["blade_kin"] += 1
                    else:
                        self._cut_pairs(qf, pairs, "날")
                    continue
                prior = abs(beta) + 0.3 * math.radians(90.0 - ang)   # 명목 β=0·수직 우선
                if prefer_home:
                    jd = sum(abs(float(q.get(j, 0.0))) for j in self.ARM)
                    score = jd + 0.15 * prior
                else:
                    score = prior
                if best is None or score < best[0]:
                    best = (score, beta, roll, ang, a, p_pre, p_blade, quat, q, qg)
        if best is None:
            self.get_logger().info(
                f"  절단 자세 없음 — 뒤쪽방향 {why['back']} · 닫힘각부족 {why['ang']} · "
                f"pre-IK 실패 {why['pre_ik']}(그중 기구학 {why['pre_kin']}, 충돌 "
                f"{why['pre_ik'] - why['pre_kin']}) · 날-IK 실패 {why['blade_ik']}(기구학 "
                f"{why['blade_kin']}, 충돌 {why['blade_ik'] - why['blade_kin']})")
            if pairs:
                top = sorted(pairs.items(), key=lambda kv: -kv[1])[:5]
                self.get_logger().info(
                    "    막은 접촉: " + " · ".join(f"{k}×{v}" for k, v in top))
            return None
        _, beta, roll, ang, a, p_pre, p_blade, quat, q, qg = best
        self.get_logger().info(
            f"절단 자세: 접근축 β={math.degrees(beta):+.0f}° · 롤 {math.degrees(roll):+.1f}° · "
            f"a·u={float(a @ u):+.1e}(수직) · 닫힘축∠줄기 {ang:.1f}°(90 이 이상적) · "
            f"날 {off*100:.1f}cm · standoff {d0*100:.0f}cm")
        c = PG.Candidate(0.0, 0.0, roll, d0, 1.0, 0.5, 2.0, d0)   # 하위 코드가 c.d/c.psi 를 읽는다
        return dict(c=c, a=a, p_pre=p_pre, p_grasp=p_blade, quat=quat, q=q, q_grasp=qg)

    def _diag_straight(self):
        """[진단] 선택된 도달 열매에 대해 넓은 접근각 격자를 훑어, 각 후보의 pre/grasp IK
        통과 여부 + **직선 Cartesian fraction** 을 실측 보고한다. '집기 전 직선이동'이
        어떤 각도에서 가능한지(=fraction≈1.0 후보 존재 여부) 판정용. 데모는 재생 안 함."""
        sel = self._select_reachable()
        if sel is None or sel[3] is None:
            self.get_logger().error("진단: 도달 가능한 열매가 없음."); return
        name, p_fruit, r, sol = sel
        self._allow_for_target(name)     # 수확 작업공간 허용(직선 판정 공정)
        goff = self.grasp_offset      # URDF 에서 유도(또는 파라미터 상수)
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
        수확 작업공간(구 영역 / 목표 화방대)은 ACM 허용 후 판정.

        ★ 2026-07-29: fraction 만으로 고르면 **TCP 는 직선인데 관절이 크게 도는 해**가 섞인다
          (같은 열매에서 0.65rad ↔ 5.3rad 로 확률적으로 갈렸고, 큰 쪽만 손가락이 주 줄기에
          닿았다). → `approach_jl_max` 초과는 직선으로 인정하지 않고, 동률이면 덜 도는 해 우선."""
        from types import SimpleNamespace
        self._allow_for_target(name)
        if self.cut_mode:
            # ★ 파지용 (φ,θ) 격자는 절단에 쓸 수 없다 — 그 격자는 롤을 0 으로 두므로
            #   닫힘축이 줄기와 나란해진다(현 온실 실측 20.9°). 절단 전용 탐색으로 간다.
            u = self._axis_for(p_fruit, name)
            return None if u is None else self._best_straight_cut(name, p_fruit, u, thr)
        goff = self.grasp_offset      # URDF 에서 유도(또는 파라미터 상수)
        d0 = float(self.get_parameter("standoff").value)
        bxy = self._base_xy()
        hv = (p_fruit[:2] - bxy) if bxy is not None else np.array([1.0, 0.0])
        a0 = PG._unit(np.array([hv[0], hv[1], 0.0]))
        prefer_home = bool(self.get_parameter("prefer_near_home").value)
        jlim = float(self.get_parameter("approach_jl_max").value)
        phis = [0, -10, 10, -20, 20, -30, 30, -40, 40]
        thetas = [0, 15, 30, -15, -30]        # +θ = 위에서 접근(매달린 열매에 유리)
        best = None                            # (key, frac, sol)
        for phd in phis:
            for thd in thetas:
                a = PG.approach_dir(a0, math.radians(phd), math.radians(thd))
                quat = PG.mat_to_quat(PG.gaze_rotation(a, 0.0, self.approach_axis))
                p_grasp = p_fruit - a * goff
                p_pre = p_grasp - a * d0
                self._set({j: 0.0 for j in self.ARM})   # IK 시드=home → 후보 자세를 home 근처로
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
                # ★ 접근 구간 관절 경로길이 — TCP 직선이어도 손목이 뒤집혀 크게 도는 해가
                #   있고, 그 해에서만 손가락이 주 줄기에 닿았다(2026-07-29 실측).
                #   한도를 넘으면 '직선 아님'으로 낮추고, 같은 fraction 이면 덜 도는 해 우선.
                jl = self._jl_of(cart[1])[0] if cart is not None else 0.0
                if jlim > 0.0 and frac >= thr and jl > jlim:
                    frac = 0.0
                prior = abs(phd) + abs(thd)          # nominal(수평) 근접 — tie-break
                # prefer_home: 같은 fraction 이면 **home 과 관절거리가 작은**(덜 도는) 자세 우선.
                hdist = sum(abs(float(q.get(j, 0.0))) for j in self.ARM)
                # ⚠ 관절길이는 **한도 검사**(위)로 거르고, 순위에서는 맨 뒤에만 둔다.
                #   home 근접도보다 앞에 놓았더니 home 에서 먼 자세가 뽑혀 자유공간 계획이
                #   실패했다(실측: 한 열매가 18/18 폴백). 순위 기준은 종전대로 둔다.
                key = ((round(frac, 3), -hdist, -prior, -round(jl, 2)) if prefer_home
                       else (round(frac, 3), -prior, -round(jl, 2)))
                if best is None or key > best[0]:
                    c = SimpleNamespace(phi=math.radians(phd), theta=math.radians(thd),
                                        psi=0.0, d=d0, prior=prior)
                    best = (key, frac, dict(c=c, a=a, p_pre=p_pre, p_grasp=p_grasp,
                                            quat=quat, q=q, q_grasp=qg), jl)
        if best is not None and best[1] >= thr:
            c = best[2]["c"]
            self.get_logger().info(
                f"직선접근 각도 채택: φ={math.degrees(c.phi):+.0f}° θ={math.degrees(c.theta):+.0f}° "
                f"→ 직선 Cartesian fraction={best[1]:.2f} · 접근구간 관절 {best[3]:.2f}rad"
                f"(집기 전 곧게 접근)")
            return best[2]
        if best is not None:
            self.get_logger().info(
                f"완전 직선 각도 없음(최고 fraction={best[1]:.2f}"
                + (f", 관절한도 {jlim:.1f}rad 초과분 제외" if jlim > 0 else "")
                + ") → OMPL 우회로 접근")
        return None

    def _best_straight_cut(self, name, p_cut, u, thr=0.99):
        """절단판 `_best_straight_candidate` — **줄기 수직 제약을 지키면서** 직선 접근이
        뚫리는 β 를 고른다.

        파지판과 다른 점: 자유변수가 (φ,θ) 격자가 아니라 **줄기 축 둘레 각 β 하나**이고,
        롤은 매 β 마다 '닫힘축 ⟂ 줄기'로 계산된다(자유변수 아님). 나머지 판정 기준
        (fraction · 관절경로 한도 · home 근접)은 파지판과 같게 둔다 — 같은 이유로 필요하다."""
        from types import SimpleNamespace
        d0 = float(self.get_parameter("standoff").value)
        off = self.cut_offset
        prefer_home = bool(self.get_parameter("prefer_near_home").value)
        jlim = float(self.get_parameter("approach_jl_max").value)
        base = self._base_xyz()
        if base is None:
            bxy = self._base_xy()
            base = np.array([bxy[0], bxy[1], 0.35]) if bxy is not None else np.zeros(3)
        a_nom = np.array([p_cut[0] - base[0], p_cut[1] - base[1], 0.0])
        betas = [float(b) for b in self.get_parameter("cut_beta_deg").value] or [0.0]
        best = None
        for bd in betas:
            a = PG.approach_perpendicular(u, a_nom, math.radians(bd))
            if float(a @ (p_cut - base)) <= 0.0:
                continue
            for roll in PG.rolls_for_closing_perp(a, u, self.approach_axis, self.closing_axis):
                ang = PG.closing_angle_deg(a, roll, u, self.approach_axis, self.closing_axis)
                if ang < 60.0:
                    continue
                quat = PG.mat_to_quat(PG.gaze_rotation(a, roll, self.approach_axis))
                p_blade = p_cut - a * off
                p_pre = p_blade - a * d0
                self._set({j: 0.0 for j in self.ARM})
                q = self.solve_ik(p_pre, quat, avoid=True)
                if q is None:
                    continue
                qg = self.solve_ik(p_blade, quat, avoid=True)
                if qg is None:
                    continue
                self._set(q)
                gp = Pose()
                gp.position.x, gp.position.y, gp.position.z = map(float, p_blade)
                (gp.orientation.x, gp.orientation.y,
                 gp.orientation.z, gp.orientation.w) = map(float, quat)
                cart = self.cartesian_to(gp)
                frac = cart[2] if cart is not None else 0.0
                jl = self._jl_of(cart[1])[0] if cart is not None else 0.0
                if jlim > 0.0 and frac >= thr and jl > jlim:
                    frac = 0.0
                prior = abs(bd) + 0.3 * (90.0 - ang)
                hdist = sum(abs(float(q.get(j, 0.0))) for j in self.ARM)
                key = ((round(frac, 3), -hdist, -prior, -round(jl, 2)) if prefer_home
                       else (round(frac, 3), -prior, -round(jl, 2)))
                if best is None or key > best[0]:
                    c = SimpleNamespace(phi=math.radians(bd), theta=0.0, psi=roll,
                                        d=d0, prior=prior)
                    best = (key, frac, dict(c=c, a=a, p_pre=p_pre, p_grasp=p_blade,
                                            quat=quat, q=q, q_grasp=qg), jl, ang)
        if best is not None and best[1] >= thr:
            c = best[2]["c"]
            self.get_logger().info(
                f"직선 절단접근 채택: β={math.degrees(c.phi):+.0f}° 롤={math.degrees(c.psi):+.1f}° "
                f"닫힘축∠줄기 {best[4]:.1f}° → fraction={best[1]:.2f} · 접근구간 관절 "
                f"{best[3]:.2f}rad")
            return best[2]
        if best is not None:
            self.get_logger().info(
                f"완전 직선 절단각 없음(최고 fraction={best[1]:.2f}) → OMPL 우회로 접근")
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
        frac0 = cart[2] if cart is not None else 0.0
        if cart is not None and frac0 >= 0.99:
            n, wp, frac = cart
            # ★ TCP 가 직선이어도 관절이 크게 도는 해는 거른다 — 그 해에서만 손가락이 주 줄기에
            #   닿았다(2026-07-29). 걸리면 ②(OMPL 우회)로 내려간다.
            ok, jl, _ = self._straight_ok(wp, f"(접근, {name})")
            if ok:
                self.get_logger().info(
                    f"② 접근=직선 Cartesian {len(wp)}점(fraction={frac:.2f}, 관절 {jl:.2f}rad) "
                    f"— 주 줄기 등 경로상 장애물 없음")
                return n, wp, f"cartesian(frac={frac:.2f})", True
            frac0 = 0.0          # 같은 경로를 ③ 부분경로로 되쓰지 않는다
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
        if cart is not None and frac0 >= 0.5 and self._straight_ok(cart[1], "(부분경로)")[0]:
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
        # 자동: **선정 규칙**(거터 법선 수평거리 · 가림 제외)대로 정렬 → 앞 max_scan 개 시도
        #   ⚠ 종전에는 여기서만 따로 3D 거리로 정렬했다. 목록(`_sorted_targets`)과 기준이
        #     달라지면 '목록 1순위와 데모가 고르는 열매가 다른' 조용한 불일치가 생긴다.
        tg = self._sorted_targets()
        want = str(self.get_parameter("target_name").value or "").strip()
        if want:
            # 이름으로 못박은 목표는 **선정 규칙을 우회**한다 — 가림 필터가 걷어낸 열매도
            #   비교 실험에서는 일부러 지정할 수 있어야 한다.
            hit = [t for t in self._all_targets() if str(t[0]) == want]
            if not hit:
                self.get_logger().error(f"target_name '{want}' 을 장면에서 못 찾음 → 자동 선택으로")
            else:
                name, p_fruit, r = hit[0]
                self.get_logger().info(f"목표 고정(target_name) = {name} (선정 규칙 우회)")
                sol = self.solve_pregrasp(p_fruit, r)
                if sol is None and getattr(self, "_acm_mode", "region") != "none":
                    self._remember_target(name, p_fruit, r)
                    if self._allow_for_target(name):
                        sol = self.solve_pregrasp(p_fruit, r)
                return name, p_fruit, r, sol
        if not tg:
            return None
        n = int(self.get_parameter("max_scan").value)
        self.get_logger().info(f"도달 가능한 열매 탐색(가까운 {min(n, len(tg))}개, 전체 {len(tg)})…")
        for name, p_fruit, r in tg[:n]:
            sol = self.solve_pregrasp(p_fruit, r)
            if sol is None and getattr(self, "_acm_mode", "region") != "none":
                # ★ 수확 사이클과 같은 기준으로 한 번 더 — 허용 전에만 보면 실제로는 수확
                #   가능한 열매를 놓친다(2026-07-29 정합성 수정, 실측 3개 → 6개).
                self._remember_target(name, p_fruit, r)
                if self._allow_for_target(name):
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
        l0 = self._base_xyz()
        if l0 is None:
            l0 = np.array([bxy[0], bxy[1], 0.35]) if bxy is not None else np.array([0.0, 0.0, 0.35])
        tg.sort(key=lambda t: float(np.linalg.norm(t[1] - l0)))
        self.get_logger().info(
            f"=== 도달 스캔 시작: 전체 {len(tg)}개 "
            + ("화방대(절단 대상) " if self.cut_mode else "열매 ")
            + f"(팔 베이스 link0 world = [{l0[0]:.3f}, {l0[1]:.3f}, {l0[2]:.3f}]) ===")
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
        self.get_logger().info(
            f"=== {'절단' if self.cut_mode else '도달'} 가능 {len(reach)}/{len(tg)}개 ===")
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
        l0 = self._base_xyz()
        if l0 is None:
            l0 = np.array([bxy[0], bxy[1], 0.35]) if bxy is not None else np.array([0., 0., 0.35])
        tg.sort(key=lambda t: float(np.linalg.norm(t[1] - l0)))
        goff = self.grasp_offset      # URDF 에서 유도(또는 파라미터 상수)
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
        """ACM 을 diff 로 일괄 설정(True=충돌 무시). **default + 쌍별(entry) 을 함께** 쓴다.

        ⚠ 2026-07-28 규명 — 예전에는 default entry 만 썼는데, MoveIt 은 **쌍별 entry 가 있으면
          그쪽을 우선**한다. 이 장면은 객체가 장면에 들어올 때 로봇 링크와의 쌍별 entry(False)가
          이미 만들어져 있어서, default=True 를 아무리 걸어도 **목표 화방대 허용이 실제로는
          안 걸렸다**('조용한 무효'). 그래서 이제 허용/차단할 이름의 **행·열을 직접 써넣는다.**
        ⚠ 검증도 두 군데가 틀렸었다:
          ① `want=False` 인데 항목이 **없는** 것을 불일치로 봤다 — MoveIt 은 중복인 False default
             를 저장하지 않는다(실측: stem 84개가 매번 '누락'으로 잡혀 4회 재시도 낭비).
          ② 쌍별을 **월드 객체끼리**의 쌍까지 확인했다 — MoveIt 이 검사하는 건 로봇↔월드와
             자기충돌뿐이라 객체끼리의 False 는 정상이다. 이제 **로봇 링크와의 쌍만** 본다.
        ⚠ 응답의 success 는 믿지 않는다(옥토맵이 크면 자주 밀린다) — 재조회로 판정."""
        if not names or self.apply_scene is None:
            return False
        want = bool(value)
        world = set(self._world_object_names() or [])
        bad_def, bad_pair = [], []
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
            # 쌍별 — 장면에 실제로 있는 객체(또는 이미 행이 있는 이름)만. 없는 이름에 행을
            # 만들면 ACM 만 부풀고 MoveIt 이 어차피 버린다.
            cols = {nm: i for i, nm in enumerate(acm.entry_names)}
            for n in names:
                if n not in cols:
                    if n not in world:
                        continue
                    acm.entry_names.append(n)
                    for e in acm.entry_values:
                        e.enabled.append(want)
                    acm.entry_values.append(
                        AllowedCollisionEntry(enabled=[want] * len(acm.entry_names)))
                    cols = {nm: i for i, nm in enumerate(acm.entry_names)}
                i = cols[n]
                for j in range(len(acm.entry_names)):
                    if j != i:                      # 자기 자신 쌍은 두고, n 이 낀 쌍만 바꾼다
                        acm.entry_values[i].enabled[j] = want
                        acm.entry_values[j].enabled[i] = want
            ps = PlanningScene()
            ps.is_diff = True
            ps.robot_state.is_diff = True
            ps.allowed_collision_matrix = acm
            self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 20.0)
            chk = self._read_acm()
            bad_def, bad_pair = [], []
            if chk is not None:
                cur = dict(zip(chk.default_entry_names, chk.default_entry_values))
                # want=False 는 '항목 없음'도 통과(없음 = 기본 불허 = 우리가 원하는 상태)
                bad_def = [n for n in names
                           if cur.get(n) != want and not (want is False and n not in cur)]
                ok = not bad_def
                if check_pairs and ok:
                    cols = {nm: i for i, nm in enumerate(chk.entry_names)}
                    known = getattr(self, "_link_names", set())   # introspection 실패 시 빈 집합
                    links = [nm for nm in chk.entry_names if nm in known]
                    for n in names:
                        if n in cols:
                            row = chk.entry_values[cols[n]].enabled
                            bad = [m for m in links if m != n and row[cols[m]] != want]
                            if bad:
                                ok = False
                                bad_pair.append((n, bad[:3]))
                if ok:
                    return True
            time.sleep(1.0)
        # 무엇이 안 맞았는지 남긴다 — 원인을 로그만 보고 좁힐 수 있게(2026-07-28).
        self.get_logger().warn(
            f"ACM 설정 확인 실패({len(names)}개 → {want}) — 반영 안 됐을 수 있음"
            + (f" · default 불일치 {len(bad_def)}개 {bad_def[:3]}" if bad_def else "")
            + (f" · 쌍별 불일치 {bad_pair[:2]}" if bad_pair else ""))
        return False

    @staticmethod
    def _densify(wp, max_step):
        """웨이포인트 열을 관절 최대 스텝 max_step[rad] 이하로 선형 세분. 경로 자체는
        바뀌지 않는다(구간 안이 직선이므로) — 검사 해상도만 올린다."""
        if max_step <= 0 or not wp or len(wp) < 2:
            return wp
        out = [list(map(float, wp[0]))]
        for a, b in zip(wp[:-1], wp[1:]):
            a = np.asarray(a, float)
            b = np.asarray(b, float)
            k = int(math.ceil(float(np.max(np.abs(b - a))) / max_step))
            for i in range(1, max(1, k) + 1):
                out.append(list(a + (b - a) * (i / max(1, k))))
        return out

    def _invalid_waypoints(self, names, wp, sample=1, pairs=None):
        """궤적 웨이포인트를 **현재 ACM 기준**으로 검사해 충돌 상태 개수를 센다.
        조건 C(전 작물 무시)로 만든 궤적이 실제로는 얼마나 위험한지 정량화하는 지표.

        `pairs` 에 dict 를 주면 **접촉 쌍('링크|객체')별 횟수**를 누적한다 — 개수만으로는
        '무엇을 스쳤는지' 모르므로 허용 영역 형상을 고칠 근거가 안 된다(2026-07-29)."""
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
                if pairs is not None:
                    for c in res.contacts:
                        k = f"{c.contact_body_1}|{c.contact_body_2}"
                        pairs[k] = pairs.get(k, 0) + 1
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
        picked = self._bench_pick_samples()
        return self._bench_run(picked)

    def _bench_wait_scene_stable(self):
        """장면이 **전부** 로드될 때까지 대기 → 측정 재현성 확보.

        `_wait_scene` 은 최소 개수만 보므로, 부분 로드 상태에서 재면 장애물이 덜 실린 채로
        IK 가 통과해 결과가 실행마다 달라진다(실제로 같은 표본이 8/8 ↔ 3/8 로 흔들렸다)."""
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
        return prev

    def _bench_pick_samples(self):
        """비교실험 표본 열매 [(name,p,r), …]. bench_targets 가 있으면 그것으로 고정(재현성),
        없으면 제안 조건(구 영역 허용)에서 도달 가능한 열매를 가까운 순으로 bench_n 개."""
        if not hasattr(self, "_bench_crops") or self._bench_crops is None:
            self._bench_crops = self._crop_objects()
            self.get_logger().info(f"작물 객체 {len(self._bench_crops)}개를 ACM 조작 대상으로 잡음")
        self._bench_wait_scene_stable()
        # ★ 표본도 **운용과 같은 선정 규칙**으로 뽑는다(거터 법선 수평거리 · 가림 제외).
        #   기준이 다르면 '실제로는 안 고를 열매'에서 잰 수치를 보고서에 싣게 된다.
        tg = self._sorted_targets()
        nmax = int(self.get_parameter("bench_n").value)
        fixed = [t for t in (self.get_parameter("bench_targets").value or []) if t]
        if fixed:
            byname = {nm: (nm, p, r) for nm, p, r in tg}
            picked = [byname[n] for n in fixed if n in byname]
            self.get_logger().info(f"표본 고정: {len(picked)}개 (bench_targets)")
            missing = [n for n in fixed if n not in byname]
            if missing:
                self.get_logger().warn(f"표본에 없는 이름: {missing}")
            return picked
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
        return picked

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

    # ══════════ 전략·planner 비교 측정(bench_strategy) — 6주차 후속 ══════════
    # "어느 수확 전략·어느 OMPL 알고리즘이 나은가"를 **같은 표본 열매**에서 정량 비교한다.
    #   측정: 계획시간 · 경로길이(관절 rad / TCP m) · 성공률 · 접근 검증여부 · 충돌 웨이포인트
    #   ⚠ 재생/실행은 하지 않는다(계획만). OMPL 이 확률적이라 bench_repeat 회 반복해 평균낸다.
    def _seg_len(self, names, wp):
        """한 구간의 (관절 경로길이[rad], TCP 경로길이[m]). wp=[pos_array,…](names 순서)."""
        if not wp or len(wp) < 2:
            return 0.0, 0.0
        a = np.asarray(wp, float)
        jl = float(np.sum(np.linalg.norm(np.diff(a, axis=0), axis=1)))
        cl = 0.0
        if self._fk is not None:                    # 로컬 FK(서비스 왕복 없이 수백 점 처리)
            joints, chain = self._fk
            pts = np.asarray([RI.fk_pos(joints, chain, dict(zip(names, row))) for row in a])
            cl = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        return jl, cl

    def _bench_strategy_one(self, name, p_fruit, r, strat, planner):
        """한 열매 × (전략, planner) 1회 계획을 측정 → dict 리포트."""
        import re
        home = {j: 0.0 for j in self.ARM}
        self._set(home)
        # ── 조건 고정: 공간 ACM(제안 방식) + 전 작물 장애물 ──
        self._clear_zone()
        self._set_allow(self._bench_crops, False)
        self._acm_mode = "region"
        self._remember_target(name, p_fruit, r)
        # ★ IK 전에 구 영역 허용을 건다 — 표본 선정(_bench_pick_samples)과 같은 기준이어야
        #   "뽑힌 열매가 측정에선 IK 실패" 같은 불일치가 안 생긴다. (ACM 이 무효였던 동안에는
        #   이 차이가 드러나지 않았다. 수정 후 실측: 허용 전 IK 는 6열매 중 3개가 실패.)
        #   ⚠ 데모 경로(`_select_reachable`)는 여전히 허용 전에 IK 를 본다 — 별건.
        self._allow_for_target(name)
        self.strategy, self.planner_id = strat, planner
        self._plan_stat = dict(calls=0, fail=0, t=0.0)   # plan_to 가 여기에 적는다
        t0 = time.time()
        sol = self.solve_pregrasp(p_fruit, r)
        if sol is None:
            self._plan_stat = None
            return dict(name=name, strategy=strat, planner=planner, ok=False,
                        method="IK 실패", checked=False, t=time.time() - t0,
                        jl=None, cl=None, n=0, bad=None, frac=None,
                        fallback=False, pre_fb=False, home_fb=False, n_chk=0,
                        bad_coarse=None,
                        plan_calls=0, plan_fail=0, plan_t=0.0)
        c = self._precompute(name, p_fruit, r, sol)
        t = time.time() - t0
        stat, self._plan_stat = self._plan_stat, None
        pre_n, pre_wp = c["pre"]
        app_n, app_wp, method, checked = c["app"]
        home_n, home_wp = c["home"] if c["home"] is not None else (self.ARM, [])
        # 폴백 판정 — 계획 실패 시 전략이 2점 관절보간으로 대체한다(충돌 미검증·길이 과소평가).
        #   pre  : [home(전부 0), 목표] 2점  ·  home : plan_to 실패 시 c["home"] is None
        pre_fb = (len(pre_wp) == 2 and max(abs(float(v)) for v in pre_wp[0]) < 1e-9)
        home_fb = c["home"] is None
        s_pre = self._seg_len(pre_n, pre_wp)
        s_app = self._seg_len(app_n, app_wp)
        s_home = self._seg_len(home_n, home_wp)
        # 한 수확 사이클의 실제 이동 = pre + 접근 + 후퇴(접근 역재생) + home → 접근 구간은 2회
        jl = s_pre[0] + 2 * s_app[0] + s_home[0]
        cl = s_pre[1] + 2 * s_app[1] + s_home[1]
        nwp = len(pre_wp) + 2 * len(app_wp) + len(home_wp)
        # ── 안전성 재검증: 항상 '전 작물 장애물 + 수확 대상 줄기만 해제' 기준으로 ──
        #   (구 영역을 먼저 걷어내지 않으면 허용 해제된 구가 열매 자리의 장애물이 된다)
        self._clear_zone()
        self._set_allow(self._bench_crops, False)
        # 🔴 **수확 대상 자신은 허용한다** — 목표 화방대(불가피하게 스친다)와 **목표 열매**.
        #   열매를 충돌객체로 발행하면(publish_targets) 손에 든 사본 `grasped_<이름>` 이
        #   장면에 남은 원본과 겹쳐 **자기 자신과의 충돌**이 접촉의 대부분으로 잡힌다
        #   (2026-08-21 실측: 접촉 쌍 상위가 전부 `fruit_X|grasped_fruit_X` 였다).
        #   무는 대상을 만지는 것은 충돌이 아니다.
        allow = [x for x in (self._stalk_of(name), name) if x]
        if allow:
            self._set_allow(allow, True)
        cpairs = {}
        # ★ 두 전략을 **같은 해상도**로 검사한다 — 웨이포인트 밀도가 다르면 충돌 개수를
        #   비교할 수 없고, 성긴 표본은 얇은 줄기 사이를 그냥 지나친다.
        vstep = float(self.get_parameter("verify_max_step").value)
        d_pre = self._densify(pre_wp, vstep)
        d_app = self._densify(app_wp, vstep)
        d_home = self._densify(home_wp, vstep) if home_wp else []
        n_chk = len(d_pre) + len(d_app) + (len(d_home) if d_home else 0)
        bad = (self._invalid_waypoints(pre_n, d_pre, pairs=cpairs)
               + self._invalid_waypoints(app_n, d_app, pairs=cpairs)
               + (self._invalid_waypoints(home_n, d_home, pairs=cpairs) if d_home else 0))
        # 같은 궤적을 **원래 웨이포인트만으로도** 재본다. 두 값이 다르면 차이는 궤적이
        #   아니라 **검사 해상도** 탓이다(성긴 검사는 얇은 줄기 사이를 그냥 지나친다).
        bad_coarse = (self._invalid_waypoints(pre_n, pre_wp)
                      + self._invalid_waypoints(app_n, app_wp)
                      + (self._invalid_waypoints(home_n, home_wp) if home_wp else 0)) \
            if vstep > 0 else bad
        if cpairs:
            self.get_logger().warn(
                f"  ⚠ 안전기준 접촉 {bad}wp — 쌍 "
                + ", ".join(f"{k}×{v}" for k, v in sorted(cpairs.items(),
                                                          key=lambda x: -x[1])[:4]))
        self._set(home)
        m = re.search(r"frac=([0-9.]+)", str(method))
        return dict(name=name, strategy=strat, planner=planner, ok=True,
                    method=method, checked=bool(checked), t=t,
                    jl=jl, cl=cl, n=nwp, bad=bad, n_chk=n_chk, bad_coarse=bad_coarse,
                    bad_pairs=";".join(f"{k}×{v}" for k, v in sorted(
                        cpairs.items(), key=lambda x: -x[1])),
                    frac=(float(m.group(1)) if m else None),
                    fallback=bool(pre_fb or home_fb), pre_fb=pre_fb, home_fb=home_fb,
                    jl_pre=s_pre[0], jl_app=s_app[0], jl_home=s_home[0],
                    plan_calls=stat["calls"], plan_fail=stat["fail"], plan_t=stat["t"])

    def bench_strategy_compare(self):
        """전략 × planner 격자를 같은 표본 열매에서 돌려 표·요약·원자료(JSON/CSV)로 낸다."""
        strat0, planner0 = self.strategy, self.planner_id     # 원상복구용
        picked = self._bench_pick_samples()
        strats = [str(s).strip() for s in (self.get_parameter("bench_strategies").value or [])
                  if str(s).strip()]
        planners = [str(p).strip() for p in (self.get_parameter("bench_planners").value or [])
                    if str(p).strip()]
        rep = max(1, int(self.get_parameter("bench_repeat").value))
        unknown = [s for s in strats if s not in self._STRATEGIES]
        if unknown:
            self.get_logger().warn(f"알 수 없는 전략 {unknown} → 제외 (가능: {list(self._STRATEGIES)})")
            strats = [s for s in strats if s in self._STRATEGIES]
        if not picked or not strats or not planners:
            self.get_logger().error("측정 표본/전략/planner 가 비었다 — bench_n·bench_strategies·"
                                    "bench_planners 확인.")
            return []
        total = len(picked) * len(strats) * len(planners) * rep
        # 표본을 남긴다 → 다음 실행을 같은 열매로 고정할 수 있다(bench_targets:=…).
        self.get_logger().info(f"표본 열매: {','.join(nm for nm, _, _ in picked)}")
        self.get_logger().info(
            f"=== 전략·planner 비교 측정 === 표본 {len(picked)}열매 × 전략 {len(strats)} × "
            f"planner {len(planners)} × 반복 {rep} = {total}회 계획 (수 분~수십 분 소요)")
        if self._fk is None:
            self.get_logger().warn("로컬 FK 미구성(URDF 조회 실패) → TCP 경로길이는 0 으로 기록된다.")
        rows = []
        for nm, p, r in picked:
            for strat in strats:
                for pl in planners:
                    for k in range(rep):
                        res = self._bench_strategy_one(nm, p, r, strat, pl)
                        res["rep"] = k
                        rows.append(res)
                        # ★ 매 행마다 저장한다. 전체 격자는 한 시간을 넘길 수 있어, 끝에서만
                        #   저장하면 중간에 죽었을 때(타임아웃·Ctrl-C) 측정을 통째로 잃는다.
                        self._bench_strategy_save(rows)
                        self.get_logger().info(
                            f"  [{len(rows):3d}/{total}] {nm:20s} {strat:14s} {pl:11s} "
                            f"{'OK ' if res['ok'] else '실패'} t={res['t']:5.1f}s "
                            f"관절={('%.2f' % res['jl']) if res['jl'] is not None else '  - '}rad "
                            f"TCP={('%.2f' % res['cl']) if res['cl'] is not None else '  - '}m "
                            f"pts={res['n']:3d} "
                            f"충돌wp={res['bad'] if res['bad'] is not None else '-'}"
                            f"/{res.get('n_chk', 0)}(성긴검사 {res.get('bad_coarse')}) "
                            f"{'검증O' if res['checked'] else '검증X'}"
                            f"{' 폴백' if res.get('fallback') else '    '} method={res['method']}")
        self._bench_strategy_summary(rows, strats, planners, len(picked), rep)
        self.strategy, self.planner_id = strat0, planner0
        return rows

    def _bench_strategy_summary(self, rows, strats, planners, n_fruit, rep):
        def agg(rs):
            n = len(rs)
            ok = [x for x in rs if x["ok"]]
            # 유효 = 충돌free · 접근 검증됨 · 폴백(2점 보간) 없음 → 경로길이는 이 행들로만 평균낸다
            #   (폴백 행은 계획 없이 관절을 직선보간한 것이라 길이를 과소평가한다)
            good = [x for x in ok if x["checked"] and not x["bad"] and not x.get("fallback")]
            def mean(key):
                v = [x[key] for x in good if x[key] is not None]
                return sum(v) / len(v) if v else 0.0
            return dict(n=n, ok=len(ok), good=len(good),
                        t=sum(x["t"] for x in rs) / max(1, n),
                        jl=mean("jl"), cl=mean("cl"),
                        bad=sum(x["bad"] for x in ok if x["bad"] is not None),
                        straight=sum(1 for x in ok if str(x["method"]).startswith("cartesian(")),
                        unver=sum(1 for x in ok if not x["checked"]),
                        fb=sum(1 for x in ok if x.get("fallback")),
                        pfail=sum(x["plan_fail"] for x in rs))
        self.get_logger().info(
            f"=== 요약(표본 {n_fruit}열매 × 반복 {rep}) — 성공=IK+계획 · "
            f"유효=충돌free·검증·폴백없음 · 길이 평균은 유효 행만 ===")
        self.get_logger().info(
            f"  {'전략':14s} {'planner':11s} {'성공':>7s} {'유효':>7s} {'계획시간':>8s} "
            f"{'관절길이':>9s} {'TCP길이':>8s} {'완전직선':>7s} {'폴백':>4s} {'충돌wp':>6s} "
            f"{'플래너실패':>7s}")
        for strat in strats:
            for pl in planners:
                rs = [x for x in rows if x["strategy"] == strat and x["planner"] == pl]
                if not rs:
                    continue
                a = agg(rs)
                self.get_logger().info(
                    f"  {strat:14s} {pl:11s} {a['ok']:3d}/{a['n']:<3d} {a['good']:3d}/{a['n']:<3d} "
                    f"{a['t']:7.2f}s {a['jl']:8.2f}rad {a['cl']:7.3f}m "
                    f"{a['straight']:3d}/{a['n']:<3d} {a['fb']:4d} {a['bad']:6d} {a['pfail']:7d}")
        for strat in strats:                       # 전략별 총평(planner 통합)
            rs = [x for x in rows if x["strategy"] == strat]
            if rs:
                a = agg(rs)
                self.get_logger().info(
                    f"  ▶ {strat:12s} 전체: 성공 {a['ok']}/{a['n']} · 유효 {a['good']}/{a['n']} · "
                    f"평균 {a['t']:.2f}s · 관절 {a['jl']:.2f}rad · TCP {a['cl']:.3f}m · "
                    f"완전직선 {a['straight']}/{a['n']} · 폴백 {a['fb']} · 무검증 {a['unver']}")
        base = self._bench_strategy_save(rows)
        self.get_logger().info(f"원자료 저장: {base}.json · {base}.csv")

    def _bench_strategy_save(self, rows):
        """원자료를 JSON/CSV 로 덮어쓴다(측정 중 매 행마다 호출 — 중간 종료 대비)."""
        import csv
        import json
        base = str(self.get_parameter("bench_out").value) or "/tmp/bench_strategy"
        with open(base + ".json", "w") as f:
            json.dump(rows, f, ensure_ascii=False, indent=1)
        # bad_pairs = 접촉 쌍('링크|객체×횟수'). 개수만으론 원인을 못 좁힌다(2026-07-29 교훈).
        keys = ["name", "strategy", "planner", "rep", "ok", "checked", "fallback",
                "pre_fb", "home_fb", "t", "jl", "cl", "n", "bad", "bad_pairs", "frac",
                "jl_pre", "jl_app", "jl_home",
                "plan_calls", "plan_fail", "plan_t", "method"]
        with open(base + ".csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        return base

    # ══════════════════ 접근 전략(작업 플래너) 레지스트리 ══════════════════
    # 각 전략은 (name, p_fruit, r, sol) → 재생용 계획 dict 을 돌려준다. 반환 규격은 run() 이 쓰는:
    #   pre=(names,wp)  home→시작자세 구간
    #   app=(names,wp,method,checked)  시작자세→grasp 접근 구간(direct 는 자리표시)
    #   q_pre,q_grasp,q_home,gopen,close,home,sol
    # 새 전략 추가 = 메서드 하나 만들고 아래 _STRATEGIES 에 등록만.
    def _precompute(self, name, p_fruit, r, sol):
        """선택된 strategy 로 전체 데모 궤적을 한 번만 계획해 캐시(이후 반복은 재생만)."""
        self._detach_fruit()          # 이전 사이클의 부착이 남아 있으면 떼고 시작
        fn = self._STRATEGIES.get(self.strategy)
        if fn is None:
            self.get_logger().warn(
                f"알 수 없는 strategy '{self.strategy}' → harvest_linear 사용. "
                f"(가능: {', '.join(self._STRATEGIES)})")
            fn = self._STRATEGIES["harvest_linear"]
        self.get_logger().info(
            f"접근 전략 = {self.strategy} · planner_id = {self.planner_id or '(그룹기본)'}")
        return fn(self, name, p_fruit, r, sol)

    def _strategy_harvest_linear(self, name, p_fruit, r, sol):
        """수확 특화: pre-grasp 자세 → Cartesian 직선접근 → 파지 → 역경로 후퇴 → home.
        ★ '집기 전 직선이동' 우선: 직선 Cartesian 이 뚫리는 접근각을 찾아(있으면) 그 자세로
        교체 → ②가 완전 직선. 없으면 원래 sol(OMPL 우회)."""
        straight = self._best_straight_candidate(name, p_fruit, r)
        if straight is not None:
            sol = straight
        q_pre, q_grasp_ik = sol["q"], sol["q_grasp"]
        p_grasp, quat = sol["p_grasp"], sol["quat"]
        gopen = float(self.get_parameter("gripper_open").value)
        close = self._close_value()
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
        #    ★ 이 구간은 **열매를 든 상태**로 움직인다 → 먼저 부착하고 계획한다. 안 그러면
        #      MoveIt 이 빈손인 줄 알고 경로를 짜서, 손에 든 열매가 옆 줄기를 쓸고 지나간다.
        self._attach_fruit(name, r)
        self._set({j: q_pre.get(j, 0.0) for j in self.ARM})
        home = self.plan_to(q_home)

        # self.cur 복원(재생은 home+벌림에서 시작)
        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        return dict(pre=pre, app=(app_n, app_wp, method, checked),
                    q_pre=q_pre, q_grasp=q_grasp, q_home=q_home,
                    gopen=gopen, close=close, home=home, sol=sol)

    def _strategy_direct(self, name, p_fruit, r, sol):
        """자유공간 직행: 모션플래너(planner_id, 예 RRTConnect)로 home→grasp pose 를 **직접**
        계획, 파지 후 자유공간으로 home 복귀. pre-grasp·Cartesian 직선접근 구조가 없다 →
        순수 모션플래너가 푸는 경로를 그대로 보는 대조군(줄기 사이 직선 진입을 강제하지 않음)."""
        q_grasp = dict(sol["q_grasp"])
        gopen = float(self.get_parameter("gripper_open").value)
        close = self._close_value()
        q_home = {j: 0.0 for j in self.ARM}

        # ① home → grasp 직행 (자유공간 OMPL)
        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        plan = self.plan_to(q_grasp)
        pre = plan if plan is not None else (
            self.ARM, [[0.0] * len(self.ARM), [q_grasp[j] for j in self.ARM]])
        method = f"direct/{self.planner_id or 'default'}" + ("" if plan is not None else "(폴백)")
        self.get_logger().info(f"① home→grasp 직행 계획 {len(pre[1])}점 ({method})")

        # ② 접근 구간 없음(이미 grasp) → 자리표시(역재생 후퇴도 자리표시가 됨; 실제 후퇴는 home 구간)
        app_wp = [[q_grasp[j] for j in self.ARM], [q_grasp[j] for j in self.ARM]]

        # ④ grasp → home 복귀 (자유공간 OMPL) — 열매를 든 상태로 움직인다(부착 후 계획)
        self._attach_fruit(name, r)
        self._set({j: q_grasp[j] for j in self.ARM})
        home = self.plan_to(q_home)

        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        return dict(pre=pre, app=(self.ARM, app_wp, method, plan is not None),
                    q_pre=q_grasp, q_grasp=q_grasp, q_home=q_home,
                    gopen=gopen, close=close, home=home, sol=sol)

    def _strategy_naive(self, name, p_fruit, r, sol):
        """[대조군 · 회피 알고리즘 없음] home → grasp 자세를 **관절 직선보간**으로 잇는다.

        모션플래너도 충돌검사도 쓰지 않는다 — 시작 관절값과 목표 관절값을 선형으로 섞을
        뿐이다. 산업용 팔의 기본 이동(MoveJ)과 같은 동작이고, 장애물이 정의돼 있어도
        경로는 그 존재를 모른다.

        🔴 **이 대조군은 일부러 유리하게 만들었다.** 목표 자세 `sol["q_grasp"]` 는
        회피 있는 쪽이 찾아낸 것과 **같은 값**(충돌 없는 IK 해)을 그대로 쓴다. 즉
        '목표를 못 찾아서' 생기는 차이를 배제하고 **경로 생성에 회피가 들어갔는가**
        하나만 남겼다. 그런데도 충돌이 나온다면 그것은 전적으로 경로 탓이다.

        열매 부착은 **한다** — 손에 열매가 들린 것은 알고리즘의 선택이 아니라 사실이고,
        양쪽을 같은 기준으로 재검증하려면 장면 상태가 같아야 한다. 다만 이 전략은
        그 부착물을 **계획에 쓰지 않는다**(계획 자체가 없다)."""
        q_grasp = dict(sol["q_grasp"])
        gopen = float(self.get_parameter("gripper_open").value)
        close = self._close_value()
        q_home = {j: 0.0 for j in self.ARM}
        n = max(2, int(self.get_parameter("naive_steps").value))

        def lerp(qa, qb):
            return [[float(qa[j] + (qb[j] - qa[j]) * k / (n - 1)) for j in self.ARM]
                    for k in range(n)]

        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})

        # ① home → grasp : 관절 직선보간(플래너·충돌검사 없음)
        wp = lerp(q_home, q_grasp)
        method = f"naive/관절보간 {n}점 (회피 없음)"
        self.get_logger().info(f"① home→grasp 관절보간 {n}점 — 플래너·충돌검사 사용 안 함")

        # ② 접근 구간 없음(①이 곧장 grasp 까지 간다) → 자리표시
        app_wp = [[q_grasp[j] for j in self.ARM], [q_grasp[j] for j in self.ARM]]

        # ④ grasp → home : 같은 보간의 역재생(이 역시 회피 없음)
        self._attach_fruit(name, r)
        self._set({j: q_grasp[j] for j in self.ARM})
        home = (self.ARM, list(reversed(wp)))

        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        return dict(pre=(self.ARM, wp), app=(self.ARM, app_wp, method, False),
                    q_pre=q_grasp, q_grasp=q_grasp, q_home=q_home,
                    gopen=gopen, close=close, home=home, sol=sol)

    def _scene_objects(self):
        """장면 yaml 의 obstacles 목록(작물 전개 포함). 실패 시 [].

        🔴 **캐시한다** — 정렬키(`_order_key`)가 열매마다 이걸 부르는데, 캐시가 없으면
        열매 74개 × 라운드마다 yaml 파싱+작물 전개가 돌아 선별이 수 분씩 걸린다
        (2026-08-21 실측으로 드러남). 파일 mtime 이 바뀌면 자동으로 다시 읽는다."""
        path = self.get_parameter("obstacles_file").value or self._op.default_yaml()
        try:
            mt = os.path.getmtime(path)
        except OSError:
            mt = 0.0
        c = getattr(self, "_scene_cache", None)
        if c and c[0] == path and c[1] == mt:
            return c[2]
        try:
            import yaml
            data = yaml.safe_load(open(path)) or {}
            try:
                self._op.expand_crops(data)
            except Exception:                           # noqa: BLE001
                pass
            objs = list(data.get("obstacles", []))
        except Exception:                               # noqa: BLE001
            objs = []
        self._scene_cache = (path, mt, objs)
        self._gutter_cache = None                       # 장면이 바뀌면 거터도 다시
        return objs

    def _gutters(self):
        """장면의 재배 거터(행잉베드) 목록 [(중심 xyz, rpy, size), ...].

        이름 접두사(`gutter_prefix`, 기본 'gutter')로 고른다. 거터는 박스이고
        **긴 축이 행 방향**, 짧은 수평축이 **법선**이다."""
        objs = self._scene_objects()                    # 캐시 무효화도 여기서 일어난다
        c = getattr(self, "_gutter_cache", None)
        if c is not None:
            return c
        pre = str(self.get_parameter("gutter_prefix").value or "gutter").strip()
        out = []
        for o in objs:
            if not str(o.get("name", "")).startswith(pre):
                continue
            if o.get("type") != "box" or "size" not in o:
                continue
            pose = o.get("pose") or {}
            out.append((str(o["name"]),
                        np.array([float(v) for v in pose.get("xyz", [0, 0, 0])]),
                        np.array([float(v) for v in pose.get("rpy", [0, 0, 0])]),
                        np.array([float(v) for v in o["size"]])))
        self._gutter_cache = out
        return out

    @staticmethod
    def _rpy_mat(rpy):
        cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
        cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
        cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp,     cp * sr,                cp * cr]])

    def _frontal_dir(self, p_fruit):
        """**정면 = 재배 거터(행잉베드) 면에 수직인 수평 방향** (사용자 정의 2026-08-21).

        🔴 'base 에서 열매로 가는 방향' 이 아니다 — 그건 로봇이 거터 앞에 정확히 서 있을
        때만 법선과 같고, 옆으로 비켜 있으면 어긋난다(이 장면 실측 12.7°).

        유도: 목표 열매에 가장 가까운 거터 박스를 골라, 그 **로컬 축 중 수평 성분이
        가장 큰 '짧은 축'** 을 법선으로 쓴다(긴 축 = 행 방향). 박스의 `rpy` 를 그대로
        반영하므로 거터가 돌아가 있어도 따라온다. 부호는 **통로(로봇) → 작물** 방향.

        · `approach_yaw_deg` 를 주면 그 값이 우선(월드 yaw[도]).
        · 거터를 못 찾으면 `crops.rows[*].x` 로, 그것도 없으면 base→열매로 물러선다(경고).
        """
        yaw = float(self.get_parameter("approach_yaw_deg").value)
        if not math.isnan(yaw):
            return PG._unit(np.array([math.cos(math.radians(yaw)),
                                      math.sin(math.radians(yaw)), 0.0]))
        p = np.asarray(p_fruit, float)
        bxy = self._base_xy()
        ref = np.array([bxy[0], bxy[1]]) if bxy is not None else np.zeros(2)

        g = self._gutters()
        if g:
            # 같은 거터에 대해서는 결과가 같다 → 거터 이름으로 캐시(정렬이 열매마다 부른다)
            near = min(g, key=lambda t: float(np.linalg.norm(t[1][:2] - p[:2])))
            fc = getattr(self, "_fdir_cache", None)
            if fc is not None and fc[0] == near[0]:
                self._frontal_src = f"거터 {near[0]}"
                return fc[1]
            # 열매에서 가장 가까운 거터(수평거리)
            name, c, rpy, size = min(
                g, key=lambda t: float(np.linalg.norm(t[1][:2] - p[:2])))
            R = self._rpy_mat(rpy)
            # 로컬 축 3개 중 **수평 성분이 큰 것 2개**가 상면 위 방향들 → 그중 size 가 작은 쪽이 법선
            cand = []
            for i in range(3):
                ax = R[:, i]
                horiz = float(np.hypot(ax[0], ax[1]))
                if horiz < 0.5:                     # 거의 수직축(두께) → 법선 후보 아님
                    continue
                cand.append((float(size[i]), i, ax))
            if cand:
                _, i, ax = min(cand, key=lambda t: t[0])       # 짧은 수평축 = 법선
                n = PG._unit(np.array([ax[0], ax[1], 0.0]))
                if float(np.dot(n[:2], c[:2] - ref)) < 0:      # 통로 → 작물 쪽으로
                    n = -n
                self._frontal_src = f"거터 {name}"
                self._fdir_cache = (name, n)
                return n

        rows = []
        try:
            import yaml
            path = self.get_parameter("obstacles_file").value or self._op.default_yaml()
            data = yaml.safe_load(open(path)) or {}
            rows = [float(r["x"]) for r in ((data.get("crops") or {}).get("rows") or [])
                    if "x" in r]
        except Exception:                                   # noqa: BLE001
            rows = []
        if rows:
            self.get_logger().warn("거터 객체를 못 찾음 → crops.rows[*].x 로 법선 유도")
            x_row = min(rows, key=lambda x: abs(x - float(p[0])))
            self._frontal_src = "crops.rows"
            return np.array([1.0 if (x_row - ref[0]) >= 0 else -1.0, 0.0, 0.0])
        self.get_logger().warn(
            "거터·행 정보를 못 읽음 → 정면을 base→열매 방향으로 대체(법선이 아니다)")
        self._frontal_src = "base→열매(폴백)"
        hv = (p[:2] - ref)
        if np.linalg.norm(hv) < 1e-6:
            hv = np.array([1.0, 0.0])
        return PG._unit(np.array([hv[0], hv[1], 0.0]))

    def _canopy_entry_dist(self, a, p_grasp):
        """직선 진입을 시작할 거리[m] — **p_grasp 에서 뒤로 얼마나 물러설지**. 못 재면 None.

        회피 없는 대조군의 직선 구간은 *캐노피 밖에서* 시작해야 한다(사용자 정의
        2026-08-21). 마지막 12cm 만 직선이면 그 앞 구간을 무엇으로 가든 결과가 섞인다.

        🔴 캐노피에 들어가는 것은 **TCP 가 아니라 그리퍼 손끝**이다. 이 팔은 파지 자세에서
        이미 TCP 가 작물행 앞면보다 바깥에 있고(실측 0.663 vs 0.71), 손가락만 21cm 들어간다.
        그래서 기준을 **손끝**으로 잡는다: 진입 시작점에서 손끝이 캐노피 앞면보다 밖에 있을 것.

        🔴 **행 하나만 본다.** 접근축 방향으로 `canopy_depth` 안쪽에 있는 객체만 센다.
        이걸 안 두면 통로 건너 뒷줄 거터까지 '캐노피'로 잡혀 진입 거리가 1.7m 로 부푼다
        (2026-08-21 실측 버그). 측방 폭(`canopy_row_width`)만으로는 행을 못 가른다 —
        행은 **접근축 방향**으로 떨어져 있기 때문이다.
        """
        p = np.asarray(p_grasp, float)
        s_g = float(np.dot(a, p))
        lat = float(self.get_parameter("canopy_row_width").value)
        depth = float(self.get_parameter("canopy_depth").value)
        s_min = None
        for o in self._scene_objects():
            pose = o.get("pose") or {}
            if "xyz" not in pose:
                continue
            c = np.array([float(v) for v in pose["xyz"]])
            s = float(np.dot(a, c))
            if s > s_g + depth or s < s_g - depth:      # 다른 행 — 제외
                continue
            d = c - p
            if float(np.linalg.norm(d - np.dot(d, a) * a)) > lat:
                continue
            t = o.get("type")
            if t == "box" and "size" in o:
                R = self._rpy_mat([float(v) for v in (pose.get("rpy") or [0, 0, 0])])
                size = [float(v) for v in o["size"]]
                rad = sum(0.5 * size[k] * abs(float(np.dot(R[:, k], a))) for k in range(3))
            elif t == "cylinder" and "radius" in o and "height" in o:
                R = self._rpy_mat([float(v) for v in (pose.get("rpy") or [0, 0, 0])])
                zc = abs(float(np.dot(R[:, 2], a)))
                rad = (0.5 * float(o["height"]) * zc
                       + float(o["radius"]) * math.sqrt(max(0.0, 1.0 - zc * zc)))
            else:
                rad = float(o.get("radius", 0.0))
            s0 = s - rad                                # 이 객체의 로봇 쪽 끝
            s_min = s0 if s_min is None else min(s_min, s0)
        if s_min is None:
            return None
        margin = float(self.get_parameter("canopy_margin").value)
        # 손끝이 캐노피 앞면(−여유)보다 밖에 있으려면 TCP 를 이만큼 뒤로 뺀다
        tip = float(self._pad_span[1]) if getattr(self, "_pad_span", None) else 0.0
        return max(0.0, (s_g + tip) - (s_min - margin))

    def _strategy_frontal(self, name, p_fruit, r, sol):
        """[대조군 · 회피 알고리즘 없음] **정면에서 직선으로 진입**한다.

        제안 방식(harvest_linear)과의 차이는 세 가지이고, 셋 다 '회피'에 해당한다:
          ① **접근각을 고르지 않는다** — φ=0·θ=0 정면 고정. 제안은 후보격자(φ×θ×ψ)를
             훑어 *줄기를 피해 곧게 들어갈 수 있는 각*을 찾는다.
          ② **IK 가 충돌을 보지 않는다**(`avoid_collisions=False`). 제안은 pre-grasp·grasp
             둘 다 충돌 통과해야 채택한다.
          ③ **경로가 충돌을 보지 않는다** — 진입은 충돌검사 없는 Cartesian 직선,
             왕복(home↔pre-grasp)은 플래너 없이 관절보간.

        즉 "열매 정면에 서서 곧장 찔러 넣는" 동작이다. 목표점(파지 자세·파지 깊이)은
        제안과 같은 기하로 잡으므로, 차이는 **경로와 접근 방향** 하나로 좁혀진다."""
        d0 = float(self.get_parameter("standoff").value)
        goff = self.grasp_offset
        gopen = float(self.get_parameter("gripper_open").value)
        close = self._close_value()
        q_home = {j: 0.0 for j in self.ARM}
        n = max(2, int(self.get_parameter("naive_steps").value))

        p_fruit = np.asarray(p_fruit, float)
        self._frontal_src = "?"
        a = self._frontal_dir(p_fruit)
        bxy = self._base_xy()
        if bxy is not None:
            hv = p_fruit[:2] - bxy
            if np.linalg.norm(hv) > 1e-6:
                hv = PG._unit(np.array([hv[0], hv[1], 0.0]))
                self.get_logger().info(
                    f"정면(행잉베드 법선) a=[{a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f}] · "
                    f"base→열매 방향과 "
                    f"{math.degrees(math.acos(max(-1.0, min(1.0, float(np.dot(a, hv)))))):.1f}° 차이")
        quat = PG.mat_to_quat(PG.gaze_rotation(a, 0.0, self.approach_axis))
        p_grasp = p_fruit - a * goff
        # 🔴 직선 구간은 **캐노피 밖에서** 시작한다(사용자 정의 2026-08-21) — 마지막
        #   12cm 만 직선이면 그 앞 구간을 무엇으로 가든 결과가 섞여, '회피 없이 곧장
        #   찔러 넣는' 동작이 온전히 드러나지 않는다.
        entry = None
        if bool(self.get_parameter("frontal_from_canopy").value):
            entry = self._canopy_entry_dist(a, p_grasp)
        if entry is None:
            entry, how = d0, f"standoff {d0*100:.0f}cm(캐노피 경계 불명)"
        else:
            entry = max(entry, d0)              # 최소한 standoff 만큼은 직선으로
            how = f"캐노피 밖 {entry*100:.0f}cm(손끝 기준)"
        p_pre = p_grasp - a * entry
        # 🔴 회피 없음 = IK 도 충돌을 보지 않는다.
        q_pre = self.solve_ik(p_pre, quat, avoid=False)
        q_grasp_ik = self.solve_ik(p_grasp, quat, avoid=False)
        if q_pre is None or q_grasp_ik is None:
            # 정면 자세가 기구학적으로 아예 없다 — **회피 문제가 아니라 자세 제약**이다.
            #   (정면 고정은 롤·피치까지 묶으므로 IK 해가 없을 수 있다. 어느 쪽이 없었는지
            #    같이 남겨야 '도달 불가'와 '자세 불가'를 구분할 수 있다.)
            self.get_logger().error(
                f"[{name}] 정면 진입 자세 IK 실패 — "
                f"pre={'✗' if q_pre is None else '○'} grasp={'✗' if q_grasp_ik is None else '○'}"
                f" · {how} · 법선 출처={self._frontal_src} "
                f"a=[{a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f}] · "
                f"진입점 [{p_pre[0]:.3f},{p_pre[1]:.3f},{p_pre[2]:.3f}] → "
                f"파지점 [{p_grasp[0]:.3f},{p_grasp[1]:.3f},{p_grasp[2]:.3f}]")
            return dict(pre=(self.ARM, []), app=(self.ARM, [], "frontal/IK 실패", False),
                        q_pre=q_home, q_grasp=q_home, q_home=q_home,
                        gopen=gopen, close=close, home=None, sol=None)

        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})

        # ① home → pre-grasp(정면) : 플래너 없이 관절보간
        pre_wp = [[float(q_home[j] + (q_pre[j] - q_home[j]) * k / (n - 1)) for j in self.ARM]
                  for k in range(n)]
        self.get_logger().info(
            f"① home→진입시작점({how}) 관절보간 {n}점 — 플래너 사용 안 함")

        # ② pre-grasp → grasp : 정면 직선(충돌검사 없음)
        self._set({j: q_pre[j] for j in self.ARM})
        gp_pose = Pose()
        gp_pose.position.x, gp_pose.position.y, gp_pose.position.z = map(float, p_grasp)
        (gp_pose.orientation.x, gp_pose.orientation.y,
         gp_pose.orientation.z, gp_pose.orientation.w) = map(float, quat)
        cart = self.cartesian_to(gp_pose, avoid=False)
        if cart is not None and cart[2] > 0.99:
            app_n, app_wp = cart[0], cart[1]
            method = (f"frontal/거터법선 직선진입 {entry*100:.0f}cm "
                      f"fraction={cart[2]:.2f} (회피 없음)")
        else:
            frac = cart[2] if cart is not None else 0.0
            app_n = self.ARM
            app_wp = [[float(q_pre[j] + (q_grasp_ik[j] - q_pre[j]) * k / 9)
                       for j in self.ARM] for k in range(10)]
            method = f"frontal/관절보간 진입(Cartesian frac={frac:.2f}) (회피 없음)"
        self.get_logger().info(
            f"② 정면 진입 {len(app_wp)}점 — {method} · 법선 출처={self._frontal_src} "
            f"a=[{a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f}]")
        q_grasp = ({nm: app_wp[-1][i] for i, nm in enumerate(app_n)}
                   if app_wp else dict(q_grasp_ik))

        # ④ 후퇴 = ② 역재생 → home 은 ① 역재생(둘 다 회피 없음)
        self._attach_fruit(name, r)
        home = (self.ARM, list(reversed(pre_wp)))

        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        frontal_sol = dict(c=PG.build_candidates([0.0], [0.0], [0.0], [entry],
                                                 1.0, 0.5, 2.0, entry)[0],
                           a=a, p_pre=p_pre, p_grasp=p_grasp, quat=quat,
                           q=q_pre, q_grasp=q_grasp_ik)
        return dict(pre=(self.ARM, pre_wp), app=(app_n, app_wp, method, False),
                    q_pre=q_pre, q_grasp=q_grasp, q_home=q_home,
                    gopen=gopen, close=close, home=home, sol=frontal_sol)

    # 전략 레지스트리 — 새 전략은 위에 메서드 추가 후 여기에 한 줄 등록만.
    _STRATEGIES = {
        "harvest_linear": _strategy_harvest_linear,
        "direct": _strategy_direct,
        "naive": _strategy_naive,
        "frontal": _strategy_frontal,
    }

    def run(self):
        # 목표 선택·전체 궤적 계획은 최초 1회만(base·목표 고정) → 캐시. 이후 반복은 재생만.
        if getattr(self, "_sel_cache", None) is None:
            self._sel_cache = self._select_reachable()
        sel = self._sel_cache
        if sel is None:
            self.get_logger().error(
                "목표 화방대를 찾지 못함 — obstacles.yaml 의 crops(rachis_*) 확인."
                if self.cut_mode else
                "목표 열매를 찾지 못함 — obstacles.yaml 의 kind:target 확인.")
            return False
        name, p_fruit, r, sol = sel
        if sol is None:
            self.get_logger().error(
                f"[{name}] 절단 자세를 못 찾음 — 도달권 밖이거나 줄기에 수직으로 들어갈 틈이 "
                "없다(cut_beta_deg 를 넓히거나 base_x/base_y·스탠드로 자세를 바꿔 볼 것)."
                if self.cut_mode else
                f"[{name}] 현재 로봇 위치에서 도달 가능한 열매가 없음(도달불가/충돌). "
                "어셈블러(base_placement)로 로봇을 열매 앞으로 옮겨 저장하거나 base_x/base_y 로 조정.")
            self._publish_markers(p_fruit, r, None, None, reachable=False)
            self._hold(3.0)
            return False

        if getattr(self, "_plan_cache", None) is None:
            d0 = float(self.get_parameter("standoff").value)
            deg = math.degrees
            self.get_logger().info(
                (f"목표(절단) = {name} @ ({p_fruit[0]:.2f},{p_fruit[1]:.2f},{p_fruit[2]:.2f}) · "
                 f"β={deg(sol['c'].phi):+.0f}° 롤={deg(sol['c'].psi):+.1f}° "
                 f"(standoff {d0*100:.0f}cm · 날 {self.cut_offset*100:.1f}cm)")
                if self.cut_mode else
                (f"목표 = {name} @ ({p_fruit[0]:.2f},{p_fruit[1]:.2f},{p_fruit[2]:.2f}) r={r:.3f} · "
                 f"pre-grasp φ={deg(sol['c'].phi):+.0f}° θ={deg(sol['c'].theta):+.0f}° "
                 f"(standoff {d0*100:.0f}cm)"))
            self.get_logger().info("전체 궤적 계획 중(최초 1회, 몇 초 소요)…")
            self._plan_cache = self._precompute(name, p_fruit, r, sol)
            _, _, method, checked = self._plan_cache["app"]
            self.get_logger().info(
                f"계획 완료 → 접근 방식={method}" + ("" if checked else " ⚠충돌검증 안됨")
                + ". 이후 반복은 이 궤적을 매끈하게 재생만 함(재계획 없음).")
        return self._play_cycle(name, p_fruit, r, self._plan_cache)

    def _safety_acm(self, name):
        """[show_collisions 전용] 재생 중 충돌 표시를 **측정과 같은 안전 기준**으로 맞춘다:
        전 작물이 장애물 · 수확 대상 화방대만 허용 · 구 영역 제거.

        🔴 이걸 안 하면 계획이 남긴 완화(구 영역 허용)가 그대로 남아 조건이 흐려지고,
        반대로 목표 화방대까지 장애물이 되어 **막 딴 열매가 제 화방대에 닿는 것**을
        충돌로 센다(실측: 그 쌍 하나가 표시의 대부분을 차지했다). 두 실행을 비교하려면
        기준이 하나여야 한다 — bench_strategy 재검증과 같은 기준을 쓴다."""
        if getattr(self, "_crops_cache", None) is None:
            self._crops_cache = self._crop_objects()
        self._clear_zone()
        self._set_allow(self._crops_cache, False)
        # 목표 화방대 + **목표 열매 자신**을 허용(무는 대상을 만지는 건 충돌이 아니다).
        allow = [x for x in (self._stalk_of(name), name) if x]
        if allow:
            self._set_allow(allow, True)

    def _play_cycle(self, name, p_fruit, r, c):
        """계획 dict c(전략 산출)의 한 수확 사이클(home→pre→접근→파지→후퇴→home)을 실행/재생.
        execute 모드면 각 구간이 컨트롤러로 실구동된다."""
        if self.show_col:
            self._safety_acm(name)
            self._col_hit, self._col_n, self._col_pairs = 0, 0, {}
            self._col_marks = []
            _clr = MarkerArray()
            _m = Marker()
            _m.header.frame_id, _m.ns, _m.id = self.world, "collision", 0
            _m.action = Marker.DELETEALL
            _clr.markers.append(_m)
            self.col_pub.publish(_clr)
        c_sol = c.get("sol")
        if c_sol is not None:
            self._publish_markers(p_fruit, r, c_sol["p_pre"], c_sol["a"], reachable=True)

        gopen, close = c["gopen"], c["close"]
        q_pre, q_grasp, q_home = c["q_pre"], c["q_grasp"], c["q_home"]
        app_n, app_wp, _, _ = c["app"]

        # 시작 = home + 그리퍼 벌림
        self.cur = {j: 0.0 for j in self.ARM}
        self.cur.update({f: gopen for f in self.FINGERS})
        self._publish_js()

        # ── ① home → pre-grasp ── (자유공간 → 막히면 현재 자세에서 다시 계획)
        pn, pwp = c["pre"]
        st = self._play_replan(pn, pwp,
                              float(self.get_parameter("dur_approach_plan").value),
                              lambda: self.plan_to(q_pre), f"{name} ①pre-grasp")
        if st != self.DONE:
            self.get_logger().error(f"[{name}] ① 구간에서 중단 — 이 열매를 건너뛴다")
            # ⚠ 다음 사이클은 '지금 home' 을 가정하고 시작한다(아래 `self.cur = 0` ).
            #   중간에서 멈춘 채 넘기면 그 가정이 거짓이 되므로 home 으로 빼낸다.
            self._retreat_safely([({j: 0.0 for j in self.ARM}, "home")], name)
            return False
        self._set({j: q_pre[j] for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))

        # ── ② pre-grasp → grasp : 줄기 회피 접근 궤적 재생 [5주차 2차] ──
        #   🔴 이 구간은 재계획하지 않는다(replan_fn=None). 열매를 향해 곧게 들어가는 것이
        #      이 구간의 정의라서 경로를 바꾸면 파지 기하가 깨진다 → 막히면 이 열매를 포기.
        st = self._play_replan(app_n, app_wp,
                              float(self.get_parameter("dur_approach_line").value),
                              None, f"{name} ②접근")
        if st != self.DONE:
            self.get_logger().error(
                f"[{name}] ② 직선 접근 중 장면이 막혔다 — 접근 경로는 바꿀 수 없으므로 "
                "이 열매를 건너뛴다(후퇴는 아래에서 처리)")
            self._retreat_safely([(q_pre, "pre-grasp"), ({j: 0.0 for j in self.ARM}, "home")],
                                 name)
            return False
        self._set({j: q_grasp.get(j, self.cur[j]) for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))

        # ── ③ 그리퍼 닫기(벌림 → 닫힘) ──
        self._play_waypoints(self.FINGERS,
                             [[gopen] * len(self.FINGERS), [close] * len(self.FINGERS)],
                             float(self.get_parameter("dur_gripper").value))

        # ── ④ 후퇴(접근 궤적 역재생 → 충돌free 경로로 안전 이탈) → home ──
        #   후퇴도 역재생이라 경로를 바꾸지 않는다. 막히면 알리기만 하고 계속 뺀다 —
        #   열매를 든 채 접근 자세에 멈춰 있는 게 더 위험하다.
        st = self._play_replan(app_n, list(reversed(app_wp)),
                              float(self.get_parameter("dur_retreat").value),
                              None, f"{name} ④후퇴")
        if st == self.REPLAN:
            self.get_logger().warn(
                f"[{name}] 후퇴 중 장면 변화 감지 — 역재생 경로는 유지한다"
                "(들어온 길이라 나갈 수는 있다)")
            self._play_waypoints(app_n, list(reversed(app_wp)),
                                 float(self.get_parameter("dur_retreat").value))
        self._set({j: q_pre.get(j, self.cur[j]) for j in self.ARM})
        if c["home"] is not None:
            hn, hwp = c["home"]
            self._play_replan(hn, hwp, float(self.get_parameter("dur_home").value),
                              lambda: self.plan_to({j: 0.0 for j in self.ARM}),
                              f"{name} ④home")
        else:
            self._play_replan(self.ARM,
                              [[q_pre[j] for j in self.ARM], [0.0] * len(self.ARM)],
                              float(self.get_parameter("dur_home").value),
                              lambda: self.plan_to({j: 0.0 for j in self.ARM}),
                              f"{name} ④home")
        self._set({j: 0.0 for j in self.ARM})
        self._hold(float(self.get_parameter("pause").value))
        if self.show_col:
            if self._col_hit:
                top = ", ".join(f"{k}×{v}" for k, v in sorted(
                    self._col_pairs.items(), key=lambda x: -x[1])[:4])
                self.get_logger().warn(
                    f"[{name}] 충돌 표본 {self._col_hit}/{self._col_n}"
                    f"({100.0 * self._col_hit / max(1, self._col_n):.1f}%) — {top}")
            else:
                self.get_logger().info(
                    f"[{name}] 충돌 표본 0/{self._col_n}(안전기준).")
        self.get_logger().info(f"[{name}] 수확 사이클 완료.")
        return True

    def run_harvest_all(self):
        """도달 가능한 열매를 가까운 순으로 **하나씩 연속 수확**(harvest_max 개까지). 각 열매마다
        선택 전략으로 계획→실행. 단일 열매 반복(run) 대신 실제 수확 런에 가깝다."""
        # ★ 센싱 장면에서는 **옥토맵이 안정된 뒤** 기준선을 잡는다. 안 그러면 지도가 비어 있는
        #   동안 전부 '수확 가능'으로 잡히고, 그 뒤 지도가 자라면서 후보가 사라진다 —
        #   그걸 '수확해서 줄었다'로 오독하게 된다(실측: 첫 라운드 옥토맵 0B·4개 → 수확 후
        #   364KB·0개. 줄어든 원인은 수확이 아니라 지도 성장이었다).
        #   ⚠ 옥토맵이 **아직 생기지도 않았을 수 있다**(센서·업데이터가 뜨는 데 수십 초).
        #     `_has_octomap()` 만 보고 건너뛰면 바로 그 '빈 지도' 상태에서 재게 된다.
        if str(self.get_parameter("target_source").value).lower().startswith("percep"):
            t0, wait = time.time(), float(self.get_parameter("octomap_wait").value)
            while not self._has_octomap() and time.time() - t0 < wait:
                time.sleep(2.0)
        if self._has_octomap():
            b = self._wait_octomap_stable()
            self.get_logger().info(f"옥토맵 안정화 대기 완료({b}B) → 이제 기준선을 잡는다")
        else:
            self.get_logger().warn(
                "옥토맵이 없다 — 센싱 장면이라면 지도가 아직 안 쌓인 것이다. "
                "이 상태의 '수확 가능' 수치는 낙관적이니 그대로 믿지 말 것.")
        # reachable_only(기본): 수확 가능한 열매만(가까운 순). 아니면 전체를 가까운 순.
        want_filter = bool(self.get_parameter("reachable_only").value)
        tg = self._harvestable_targets() if want_filter else self._sorted_targets()
        if not tg:
            self.get_logger().error("수확 가능한 열매를 찾지 못함 — 도달권/타깃/base 위치 확인.")
            return 0
        hmax = int(self.get_parameter("harvest_max").value)
        n0 = len(tg)
        self.get_logger().info(
            f"연속 수확 시작 — 전략={self.strategy} · 수확가능 {n0}개(가까운 순)"
            f" · 한도 {hmax if hmax > 0 else '없음'}")
        harvested = attempted = skipped = 0
        failed = set()          # 이번 런에서 실패한 열매(같은 것을 무한 재시도하지 않게)
        counts = [n0]           # 수확할 때마다 다시 센 '수확 가능 열매 수' 추이
        while hmax <= 0 or harvested < hmax:
            # ★ 매 수확 후 **다시 도출**한다 — 앞 열매를 따면 가림(인지)·막힘(계획)이 풀려
            #   뒤쪽 열매가 새로 수확 가능해진다(실제 수확 런과 같은 순서).
            cand = [t for t in (self._harvestable_targets() if want_filter
                                else self._sorted_targets()) if t[0] not in failed]
            if not cand:
                break
            name, p_fruit, r = cand[0]
            # ★ IK 전에 수확 작업공간을 허용한다 — 목록(_is_harvestable)과 같은 기준이어야
            #   '목록엔 있는데 계획에선 도달불가'가 안 생긴다. 열매를 충돌객체로 발행하면
            #   (publish_targets) 허용 없이는 목표 열매 자신이 파지 IK 를 막는다.
            self._remember_target(name, p_fruit, r)
            self._allow_for_target(name)
            sol = self.solve_pregrasp(p_fruit, r)
            if sol is None:
                skipped += 1
                failed.add(name)
                # ★ 실패한 시도가 남긴 구 영역을 **반드시 치운다**. 안 치우면 ρ=16.5cm 짜리
                #   구가 캐노피에 남아 다음 선별을 통째로 망친다(실측: 수확가능 3 → 0,
                #   탈락 사유가 전부 pre IK 로 바뀜). 성공 경로는 `_remove_fruit` 이 치운다.
                self._clear_zone()
                self.get_logger().info(f"  건너뜀 {name}(도달불가/충돌) · 구 영역 정리")
                continue
            attempted += 1
            self.get_logger().info(
                f"── 수확 {attempted}: {name} @ "
                f"({p_fruit[0]:.2f},{p_fruit[1]:.2f},{p_fruit[2]:.2f}) r={r:.3f} "
                f"· 남은 수확가능 {len(cand)}개 ──")
            ok = False
            try:
                c = self._precompute(name, p_fruit, r, sol)
                ok = bool(self._play_cycle(name, p_fruit, r, c))
            except Exception as e:
                self.get_logger().warn(f"  {name} 수확 실패: {e}")
            if ok:
                harvested += 1
                self._remove_fruit(name, p_fruit)   # 딴 열매는 장면에서 사라진다
                counts.append(len(self._harvestable_targets()) if want_filter
                              else len(self._sorted_targets()))
            else:
                failed.add(name)
            self._hold(float(self.get_parameter("pause").value))
        self.get_logger().info(
            f"연속 수확 완료 — 수확 {harvested} / 시도 {attempted} / 건너뜀 {skipped}")
        if len(counts) > 1:
            # 이 추이가 "앞을 따면 뒤가 열린다"의 실측 근거다(수확할 때마다 재도출한 값).
            self.get_logger().info(
                "  수확 가능 열매 수 추이(수확할 때마다 재도출): "
                + " → ".join(str(c) for c in counts))
        return harvested

    # ══════════════════ 파지한 열매를 그리퍼에 부착 ══════════════════
    #  파지 후에는 **손에 Ø7cm 짜리 열매를 든 상태**로 후퇴·이동한다. 그걸 계획에 알리지 않으면
    #  MoveIt 은 빈손인 줄 알고 경로를 짜고, 손에 든 열매가 옆 줄기를 쓸고 지나가도 모른다.
    #  → 파지 시점에 `AttachedCollisionObject` 로 붙이고(그리퍼 링크 기준), 수확이 끝나면 뗀다.
    #  부착 객체는 링크에 매달리므로 로봇이 움직이면 같이 움직인다(자세와 무관하게 한 번만 걸면 됨).
    def _close_value(self):
        """그리퍼 닫힘 목표값. cut 모드는 **끝까지** 닫는다(줄기를 끊어야 하므로)."""
        if self.cut_mode:
            return float(self.get_parameter("cut_gripper_close").value)
        return float(self.get_parameter("gripper_close").value)

    def _attach_fruit(self, name, r):
        """파지한 열매를 그리퍼(`ik_link`)에 부착. 성공 여부(bool)."""
        if self.cut_mode:
            # ⚠ cut 모드는 부착하지 않는다 — 잘린 화방이 손에 남는지(집게형)·떨어지는지
            #   (절단형)는 **툴에 달렸고 아직 정해지지 않았다**. 모르는 것을 계획에
            #   가정으로 넣지 않는다. 후퇴는 접근 역재생이라 부착 없이도 안전하다.
            #   ⇒ 툴이 정해지면 여기에 '잘린 화방 형상'을 부착할 것.
            return False
        if not bool(self.get_parameter("attach_fruit").value) or self.apply_scene is None:
            return False
        self._detach_fruit()                       # 이전 것이 남아 있으면 먼저 뗀다
        aid = f"grasped_{name}"
        aco = AttachedCollisionObject()
        aco.link_name = self.ik_link
        aco.object.header.frame_id = self.ik_link
        aco.object.id = aid
        aco.object.operation = CollisionObject.ADD
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [float(r)]
        po = Pose()
        # 열매 중심은 tcp 에서 **접근축 방향으로 grasp_offset** 만큼 앞(파지 기하 그대로).
        a = np.asarray(self.approach_axis, float)
        a = a / max(float(np.linalg.norm(a)), 1e-9)
        po.position.x, po.position.y, po.position.z = map(float, a * self.grasp_offset)
        po.orientation.w = 1.0
        aco.object.primitives.append(sp)
        aco.object.primitive_poses.append(po)
        # 손가락·손과 닿는 건 '잡고 있는 것'이지 충돌이 아니다.
        aco.touch_links = list(self._grip_links) or [self.ik_link]
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects.append(aco)
        self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 15.0)
        if not self._attached_ids(aid):            # 응답을 믿지 않고 재조회로 확인
            self.get_logger().warn(f"열매 부착 실패({aid}) — 계획이 '빈손'으로 짜인다")
            return False
        self._attached = aid
        # 수확 작업공간이 허용한 객체(자기 화방대 등)는 **부착된 열매와도** 허용해야 한다
        #   — 열매는 그 화방대에 매달려 있던 것이라 파지 순간 닿아 있다.
        if self._zone_allowed:
            self._set_allow(list(self._zone_allowed), True)
        self.get_logger().info(
            f"파지한 열매를 그리퍼에 부착: {aid} (r={r*100:.1f}cm, {self.ik_link} 기준 "
            f"접근축 {self.grasp_offset*100:.1f}cm) → 이후 계획이 '든 상태'로 충돌검사")
        return True

    def _attached_ids(self, want=None):
        """현재 부착된 객체 id 목록(조회 실패 시 []). want 를 주면 포함 여부(bool)."""
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        res = self._call(self.scene, req, 10.0)
        ids = ([a.object.id for a in res.scene.robot_state.attached_collision_objects]
               if res is not None else [])
        return (want in ids) if want is not None else ids

    def _detach_fruit(self, remove=True):
        """부착 해제(+장면에서도 제거). 수확이 끝났거나 다음 사이클을 시작할 때."""
        aid = self._attached
        if aid is None or self.apply_scene is None:
            return False
        aco = AttachedCollisionObject()
        aco.link_name = self.ik_link
        aco.object.id = aid
        aco.object.operation = CollisionObject.REMOVE      # 부착 해제
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects.append(aco)
        if remove:
            # 떼면 월드 객체로 남는다(손에서 놓은 자리에) → 수확한 것이니 장면에서도 없앤다.
            co = CollisionObject()
            co.header.frame_id = self.world
            co.id = aid
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 15.0)
        self._attached = None
        return True

    # ══════════════════ 수확 완료 → 장면에서 제거 ══════════════════
    def _remove_fruit(self, name, p_fruit=None):
        """수확한 열매를 **장면에서 없앤다**(실제 수확과 같은 상태로 만든다).

        딴 열매가 그대로 남아 있으면 ① 계획에선 여전히 장애물(publish_targets 사용 시)이고
        ② 인지에선 뒤쪽 열매를 계속 가린다 → 수확 가능 범위가 실제보다 좁게 나온다.
        세 곳을 함께 처리: planning scene REMOVE · 재발행 제외 통보 · Gazebo 모델 삭제."""
        if not bool(self.get_parameter("harvest_remove").value):
            return False
        name = str(name)
        self._harvested.add(name)
        self._tgt_geom.pop(name, None)
        self._detach_fruit()          # 수확 끝 = 손에서 놓는다(부착 해제 + 장면에서 제거)
        # ① 재발행 제외 통보 — 이걸 먼저 보낸다. planning scene 에서 지우기만 하면
        #    obstacle_publisher 가 다음 주기(1s)에 되살린다.
        # 🔴 **해소된 모델 이름**을 보낸다(2026-08-16 수정). 인지 타깃은 `det_N` 이라
        #    yaml/Gazebo 모델 이름과 다른데, 종전에는 그 `det_N` 을 그대로 보냈다.
        #    ⇒ obstacle_publisher 의 재발행 제외가 **인지 모드에서 한 번도 안 먹었다**
        #      (이름이 안 맞으니 조용히 무시된다 — 이번에 Unity 연동하며 발견한 잠재 결함).
        #    ⇒ Unity 온실도 yaml 이름이라 같은 이유로 열매를 못 지웠다.
        #    `_gazebo_name` 은 위치로 매칭하고 **모호하면 None** 을 준다(엉뚱한 열매 방지) →
        #    해소 실패 시에는 종전대로 원래 이름을 보낸다.
        model = self._gazebo_name(name, p_fruit) or name
        self._harvest_pub.publish(String(data=model))
        # ② planning scene 에서 즉시 REMOVE(재발행 주기를 기다리지 않게)
        if self.apply_scene is not None:
            co = CollisionObject()
            co.header.frame_id = self.world
            co.id = name
            co.operation = CollisionObject.REMOVE
            ps = PlanningScene()
            ps.is_diff = True
            ps.robot_state.is_diff = True        # 없으면 apply 가 실패 반환(기록된 함정)
            ps.world.collision_objects.append(co)
            self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 10.0)
        # ③ Gazebo 에서도 삭제 → 카메라가 더는 못 보고, 옥토맵·인지 결과가 따라 갱신된다.
        #    (설계값 장면이나 시뮬 미구동이면 서비스가 없다 → 조용히 건너뛴다.)
        gz = self._delete_in_gazebo(name, p_fruit)
        # 구 영역이 이 열매에 걸려 있었으면 해제(허용이 유령 객체에 남지 않게)
        self._clear_zone()
        self.get_logger().info(
            f"수확 완료 → 장면에서 제거: {name}"
            + (" · Gazebo 모델 삭제" if gz else "") + f" (누적 {len(self._harvested)}개)")
        # ★ cut 모드: 화방대를 자르면 **그 화방에 달린 열매도 함께** 떨어진다.
        #   (grasp 모드는 열매 하나만 없어진다 — 이게 두 모드의 눈에 보이는 차이다.)
        if (self.cut_mode and name.startswith("rachis_")
                and bool(self.get_parameter("cut_remove_fruits").value)):
            for fn, fp in self._truss_fruits(name):
                if fn in self._harvested:
                    continue
                self._harvested.add(fn)
                self._tgt_geom.pop(fn, None)
                self._harvest_pub.publish(String(data=self._gazebo_name(fn, fp) or fn))
                if self.apply_scene is not None:
                    co = CollisionObject()
                    co.header.frame_id = self.world
                    co.id = fn
                    co.operation = CollisionObject.REMOVE
                    ps = PlanningScene()
                    ps.is_diff = True
                    ps.robot_state.is_diff = True
                    ps.world.collision_objects.append(co)
                    self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 10.0)
                self._delete_in_gazebo(fn, fp)
            self.get_logger().info(
                f"  ↳ 화방 통째 수확 — 딸린 열매 {len(self._truss_fruits(name))}개 함께 제거")
        return True

    def _truss_fruits(self, rachis_name):
        """`rachis_r{ri}_p{pi}_t{ti}` → 그 화방에 달린 열매 [(name, xyz), ...].

        절단은 열매를 건드리지 않지만 **결과적으로 화방 전체가 떨어진다** → 장면에서도
        같이 없애야 다음 판정이 실제와 맞는다(안 그러면 공중에 뜬 열매가 남는다)."""
        import re
        m = re.match(r"rachis_(r\d+_p\d+_t\d+)$", str(rachis_name))
        if not m:
            return []
        pref = f"fruit_{m.group(1)}_f"
        path = self.get_parameter("obstacles_file").value or self._op.default_yaml()
        try:
            import yaml
            data = yaml.safe_load(open(path)) or {}
            self._op.expand_crops(data)
        except Exception:                                   # noqa: BLE001
            return []
        return [(str(o["name"]), np.array([float(v) for v in o["pose"]["xyz"]]))
                for o in data.get("obstacles", [])
                if str(o.get("name", "")).startswith(pref)]

    def reset_harvested(self):
        """수확 제외 목록을 비워 장면을 원복(재생 데모 반복용).
        ⚠ Gazebo 에서 실제로 삭제한 모델은 되살릴 수 없다 → 그 경우엔 아무것도 안 한다."""
        if not self._harvested:
            return False
        if getattr(self, "_gz_deleted", False):
            self.get_logger().info(
                "수확한 열매가 Gazebo 에서 삭제돼 원복하지 않는다(시뮬 재시작 필요).")
            return False
        n = len(self._harvested)
        self._harvested.clear()
        self._harvest_pub.publish(String(data="reset"))
        self.get_logger().info(f"장면 원복 — 수확 표시 {n}개 해제(다음 회차부터 다시 등장)")
        return True

    def _gazebo_name(self, name, p_fruit=None):
        """Gazebo 모델 이름을 찾는다. 인지 타깃은 이름이 `det_N` 이라 **모델 이름과 다르다**
        (모델은 `fruit_r0_p2_t0_f1` 처럼 yaml 이름) → **위치로 매칭**한다.

        시뮬 월드가 `obstacles.yaml` 에서 생성되므로(gen_gazebo_world) yaml 의 kind:target
        중 가장 가까운 것을 고르면 된다. 허용 오차 `harvest_match_tol`(기본 6cm — 인지 중심
        오차 중앙값이 1.6cm 이고 한 화방 열매 간격이 6cm 이라 그 사이).
        ※ 실물에서는 딴 열매가 물리적으로 사라지므로 이 매칭 자체가 필요 없다(시뮬 전용)."""
        if p_fruit is None or not str(name).startswith("det_"):
            return str(name)                      # yaml 타깃이면 이름이 곧 모델 이름
        tol = float(self.get_parameter("harvest_match_tol").value)
        try:
            import yaml
            path = self.get_parameter("obstacles_file").value or self._op.default_yaml()
            data = yaml.safe_load(open(path)) or {}
            self._op.expand_crops(data)
        except Exception as e:                     # noqa: BLE001
            self.get_logger().warn(f"Gazebo 이름 매칭용 yaml 로드 실패({e})")
            return str(name)
        cand = sorted(
            (float(np.linalg.norm(np.array([float(v) for v in o["pose"]["xyz"]])
                                  - np.asarray(p_fruit, float))), o["name"])
            for o in data.get("obstacles", []) if o.get("kind") == "target")
        if not cand or cand[0][0] > tol:
            self.get_logger().warn(
                f"{name} 에 해당하는 Gazebo 모델을 못 찾음"
                + (f"(최근접 {cand[0][1]} {cand[0][0]*100:.1f}cm > 허용 {tol*100:.0f}cm)"
                   if cand else "") + " → 삭제 생략")
            return None
        bd, best = cand[0]
        # ★ 모호하면 지우지 않는다 — 한 화방 열매는 6cm 간격이라 인지 오차가 크면 **엉뚱한
        #   열매를 지울** 수 있고, 그건 조용한 오류가 된다. 2등과 충분히 벌어져야 채택.
        if len(cand) > 1 and (cand[1][0] - bd) < 0.02:
            self.get_logger().warn(
                f"{name} 의 Gazebo 모델 매칭이 모호하다({best} {bd*100:.1f}cm vs "
                f"{cand[1][1]} {cand[1][0]*100:.1f}cm) → 삭제 생략(엉뚱한 열매 삭제 방지)")
            return None
        self.get_logger().info(
            f"인지 타깃 {name} → Gazebo 모델 {best} (거리 {bd*100:.1f}cm, "
            f"2등 {cand[1][0]*100:.1f}cm)" if len(cand) > 1 else
            f"인지 타깃 {name} → Gazebo 모델 {best} (거리 {bd*100:.1f}cm)")
        return best

    def _delete_in_gazebo(self, name, p_fruit=None):
        """Gazebo 에서 모델 삭제. 서비스가 없으면(설계값 장면·시뮬 미구동) False."""
        if not bool(self.get_parameter("harvest_delete_gazebo").value):
            return False
        try:
            from gazebo_msgs.srv import DeleteEntity
        except ImportError:
            return False
        if self._del_entity is None:
            self._del_entity = self.create_client(DeleteEntity, "delete_entity")
        if not self._del_entity.wait_for_service(timeout_sec=1.0):
            return False
        gz = self._gazebo_name(name, p_fruit)
        if gz is None:
            return False
        req = DeleteEntity.Request()
        req.name = str(gz)
        res = self._call(self._del_entity, req, 5.0)
        if res is None or not res.success:
            self.get_logger().warn(
                f"Gazebo 모델 삭제 실패: {name}"
                + (f" ({res.status_message})" if res is not None else " (응답 없음)"))
            return False
        self._gz_deleted = True          # 되돌릴 수 없다 → reset_harvested 가 참조
        return True

    # ══════════════════ 수확 가능(워크스페이스) 필터 ══════════════════
    def _is_harvestable(self, p_fruit, r, name=None):
        """빠른 수확가능 판정 — 전체 후보 샘플링 없이:
        ① 기하 프리필터: link0(팔 base) 기준 거리가 도달반경(arm_reach) 밖이면 즉시 제외.
        ② 명목 접근자세(base→열매 수평)로 **pre-grasp·grasp IK** 통과 확인(충돌 포함).
        (정밀 판정·직선성은 선택 시 solve_pregrasp 가 다시 확정. 여기선 목록용 빠른 선별.)

        ★ 2026-07-29: `name` 을 주면 **수확 작업공간 허용(구 영역)을 걸고 나서** IK 를 본다.
          실제 수확 사이클은 허용을 건 상태로 계획하는데 판정만 허용 전에 하면 기준이 어긋나
          '수확 가능한데 목록에 없는' 열매가 생긴다(실측: 3개 → 6개). 열매를 충돌객체로
          발행하면(`publish_targets`) 허용 없이는 목표 열매 자신이 IK 를 막아 아예 0개가 된다.
          비용을 아끼려고 **2단계**로 본다: 허용 없이 통과하면 그대로 통과(빠름), 실패한
          것만 구 영역을 걸고 한 번 더 본다(열매마다 장면 조작을 하면 목록 도출이 느려진다)."""
        bxy = self._base_xy()
        goff = self.cut_offset if self.cut_mode else self.grasp_offset
        d0 = float(self.get_parameter("standoff").value)
        reach = float(self.get_parameter("arm_reach").value)
        if bxy is not None:
            l0 = self._base_xyz()
            if l0 is None:
                l0 = np.array([bxy[0], bxy[1], 0.35])   # TF 실패 시에만 종전 상수
            # link0 world(팔 베이스) — 높이는 URDF/TF 에서 온다(스탠드 구성 대응)
            d = float(np.linalg.norm(p_fruit - l0))
            if d > reach + goff + 0.05 or d < 0.15:      # 명백히 밖/안 → 제외
                self._hv_why["기하"] = self._hv_why.get("기하", 0) + 1
                return False
            hv = p_fruit[:2] - bxy
        else:
            hv = np.array([1.0, 0.0])
        if self.cut_mode:
            # ★ 절단은 '명목 자세(롤 0)로 빠르게 본다'가 성립하지 않는다 — 롤이 자유변수가
            #   아니라 **계산값**이고 접근축도 줄기 축에 묶여 있다. 롤 0 으로 물으면 닫힘축이
            #   줄기와 나란한(못 자르는) 자세를 묻는 셈이라 실제로 되는 것도 전부 탈락한다
            #   (실측: 같은 구성에서 scan 은 8/21 인데 이 필터로는 0/21 이었다).
            #   ⇒ 여기서는 축약하지 않고 solve_cut 을 그대로 쓴다(느리지만 기준이 일치한다).
            ok = self.solve_pregrasp(p_fruit, r, name) is not None
            if not ok:
                self._hv_why["절단자세"] = self._hv_why.get("절단자세", 0) + 1
            return ok
        a = PG._unit(np.array([hv[0], hv[1], 0.0])) if np.linalg.norm(hv) > 1e-6 \
            else np.array([1.0, 0.0, 0.0])
        quat = PG.mat_to_quat(PG.gaze_rotation(a, 0.0, self.approach_axis))
        p_grasp = p_fruit - a * goff
        p_pre = p_grasp - a * d0

        # ⚠ KDL IK 는 **무작위 재시작**이라 같은 질문에도 실패/성공이 갈린다. 한 번만 물으면
        #   판정이 흔들려(실측: 같은 옥토맵에서 수확가능 수가 1↔5) '수확해서 열렸다' 같은
        #   신호를 잡음이 덮는다 → `harvest_ik_tries` 회까지 다시 묻는다(성공하면 즉시 종료).
        tries = max(1, int(self.get_parameter("harvest_ik_tries").value))

        def _ik1(p):
            for _ in range(tries):
                self._set({j: 0.0 for j in self.ARM})   # IK 시드 = home(매 시도 동일 기준)
                q = self.solve_ik(p, quat, avoid=True)
                if q is not None:
                    return q
            return None

        def _why(tag, p):
            """[진단] 그 자세를 **무엇이 막는지** 접촉 쌍으로 남긴다.
            충돌 무시 IK 로 자세를 구한 뒤 그 자세의 접촉을 조회한다 — '탈락 개수'만으로는
            옥토맵 탓인지 다른 탓인지 못 가른다(2026-07-29 가설 검증용)."""
            if not bool(self.get_parameter("screen_why_detail").value):
                return
            q = self.solve_ik(p, quat, avoid=False)     # 기구학만 — 도달 자체가 안 되면 None
            if q is None:
                self._hv_pairs["도달불가(기구학)"] = self._hv_pairs.get("도달불가(기구학)", 0) + 1
                return
            _v, cnt = self._state_report({j: q[j] for j in self.ARM if j in q})
            if not cnt:
                self._hv_pairs[f"{tag}:접촉없음(원인불명)"] = \
                    self._hv_pairs.get(f"{tag}:접촉없음(원인불명)", 0) + 1
            for k in cnt:
                key = f"{tag}:{k}"
                self._hv_pairs[key] = self._hv_pairs.get(key, 0) + 1

        def _ik_ok():
            if _ik1(p_pre) is None:
                self._hv_why["pre IK"] = self._hv_why.get("pre IK", 0) + 1
                _why("pre", p_pre)
                return False
            if _ik1(p_grasp) is None:
                self._hv_why["grasp IK"] = self._hv_why.get("grasp IK", 0) + 1
                _why("grasp", p_grasp)
                return False
            return True

        if _ik_ok():
            return True
        # 2단계: 수확 작업공간(구 영역)을 걸고 재판정 — 실제 수확 사이클과 같은 기준.
        #   ⚠ 이 단계가 **장면을 바꾼다**(구 객체 삽입 → 옥토맵 마스킹은 비가역) → 판정이
        #     라운드마다 달라진다. `screen_nondestructive`(기본 true) 면 이 단계를 쓰지 않고,
        #     대신 라운드 전체에 걸린 ACM(옥토맵↔그리퍼)으로 같은 목적을 달성한다.
        if bool(self.get_parameter("screen_nondestructive").value):
            return False
        if name is None or getattr(self, "_acm_mode", "region") == "none":
            return False
        self._remember_target(name, p_fruit, r)
        if not self._allow_for_target(name, settle=False):   # 선별 — 옥토맵 대기 생략
            return False
        return _ik_ok()

    def _camera_xyz(self):
        """가림 판정에 쓸 **카메라 원점의 world 좌표**. 못 찾으면 None.

        `camera_link:='auto'` 면 eye-to-hand(sensor2) 계열 링크를 TF 에서 차례로 찾는다.
        손끝 카메라(sensor1)는 팔이 움직이면 시점이 따라 움직여 '지금 보이는가' 가
        후보마다 달라지므로 선정 기준으로는 쓰지 않는다."""
        want = str(self.get_parameter("camera_link").value or "auto").strip()
        cands = ([want] if want and want.lower() != "auto" else
                 ["sensor2_camera_color_optical_frame", "sensor2_camera_depth_optical_frame",
                  "sensor2_camera_link", "sensor2_base_link"])
        self._ensure_tf()
        for ln in cands:
            try:
                tf = self._tf.lookup_transform(self.world, ln, rclpy.time.Time())
            except Exception:                               # noqa: BLE001
                continue
            t = tf.transform.translation
            self._cam_link_used = ln
            return np.array([t.x, t.y, t.z])
        return None

    @staticmethod
    def _seg_point_dist(a, b, p):
        """선분 a→b 와 점 p 의 최단거리, 그리고 발의 매개변수 t(0~1)."""
        d = b - a
        L2 = float(np.dot(d, d))
        if L2 < 1e-12:
            return float(np.linalg.norm(p - a)), 0.0
        t = float(np.dot(p - a, d) / L2)
        t = max(0.0, min(1.0, t))
        return float(np.linalg.norm(a + t * d - p)), t

    def _occluders(self, name, p, others, cam):
        """열매 `p` 가 카메라 시선에서 **다른 열매**에 가려지는지 → 가리는 이름 목록.

        판정: 카메라→열매중심 선분에 대해, 다른 열매 구(반경 r_j + 여유)가 그 선분과
        만나고 **그 열매보다 앞(카메라 쪽)** 에 있으면 가림.
        🔴 줄기·화방대는 세지 않는다 — 사용자 정의가 "다른 토마토에 가려지지 않은 것"이다."""
        m = float(self.get_parameter("occlusion_margin").value)
        seg = float(np.linalg.norm(p - cam))
        hit = []
        for nm, q, rq in others:
            if nm == name:
                continue
            if float(np.linalg.norm(q - cam)) >= seg:       # 목표보다 뒤 → 못 가린다
                continue
            d, t = self._seg_point_dist(cam, p, q)
            if t <= 0.0 or t >= 1.0:
                continue
            if d < float(rq) + m:
                hit.append(nm)
        return hit

    def _visible_targets(self, tg):
        """가림 필터. `require_visible` 이 꺼져 있거나 카메라를 못 찾으면 그대로 통과."""
        if not tg or not bool(self.get_parameter("require_visible").value):
            return tg
        cam = self._camera_xyz()
        if cam is None:
            self.get_logger().warn(
                "가림 판정용 카메라 TF 를 못 찾음 → 가림 필터를 건너뛴다"
                "(camera_link 인자로 링크를 직접 지정할 수 있다)")
            return tg
        out, blocked = [], []
        for nm, p, r in tg:
            occ = self._occluders(nm, p, tg, cam)
            if occ:
                blocked.append((nm, occ))
            else:
                out.append((nm, p, r))
        if blocked:
            top = ", ".join(f"{n}←{'/'.join(o[:2])}" for n, o in blocked[:4])
            self.get_logger().info(
                f"가림 제외 {len(blocked)}개 (카메라 {getattr(self, '_cam_link_used', '?')} "
                f"시선 기준): {top}" + (" …" if len(blocked) > 4 else ""))
        if not out and tg:
            self.get_logger().warn(
                "가림 필터가 후보를 전부 배제했다 — require_visible:=false 로 끄거나 "
                "occlusion_margin 을 줄여 볼 것.")
        return out

    def _order_key(self, p):
        """선정 정렬키. `select_order` 규칙에 따라 결정.

        normal(기본) = **거터 법선 방향 수평거리** — 통로에서 작물행 쪽으로 잰 수직거리다.
        같은 값이 많이 나올 수 있으므로(같은 행) 3D 거리를 2차 키로 쓴다."""
        l0 = self._base_xyz()
        if l0 is None:
            bxy = self._base_xy()
            l0 = np.array([bxy[0], bxy[1], 0.35]) if bxy is not None else np.zeros(3)
        p = np.asarray(p, float)
        d3 = float(np.linalg.norm(p - l0))
        mode = str(self.get_parameter("select_order").value or "normal").strip().lower()
        if mode.startswith("z"):
            return (abs(float(p[2] - l0[2])), d3)
        if mode.startswith("dist"):
            return (d3, d3)
        a = self._frontal_dir(p)                    # 거터 법선(= 정면 접근축)
        return (abs(float(np.dot(a, p - l0))), d3)

    def _sorted_targets(self):
        """목표 열매를 **선정 규칙**대로 정렬해 돌려준다. 수확된 것은 이미 빠져 있다.

        규칙(사용자 정의 2026-08-21):
          ① **거터 법선 방향 수평거리가 가까운 것부터**(통로에 가까운 = 얕게 박힌 열매)
          ② **다른 토마토에 가려진 것은 뺀다**(eye-to-hand 카메라 시선 기준)
        `select_order` / `require_visible` 로 종전 동작(3D 거리 · 필터 없음)으로 되돌릴 수 있다."""
        tg = self._visible_targets(self._all_targets())
        if tg and (self._base_xy() is not None or self._base_xyz() is not None):
            tg.sort(key=lambda t: self._order_key(t[1]))
            # 순서를 눈으로 확인할 수 있게 상위 몇 개만 남긴다. 이 함수는 자주 불리므로
            #   **1순위가 바뀔 때만** 찍는다(로그 폭주 방지).
            head = tg[0][0]
            if getattr(self, "_order_head", None) != head:
                self._order_head = head
                mode = str(self.get_parameter("select_order").value or "normal")
                top = " · ".join(f"{n}({self._order_key(p)[0]*100:.0f}cm)"
                                 for n, p, _ in tg[:4])
                self.get_logger().info(f"선정 순서[{mode}] {len(tg)}개 → {top}"
                                       + (" …" if len(tg) > 4 else ""))
        return tg

    OCTOMAP_ID = "<octomap>"

    def _set_pair_allow(self, obj, links, value, verify=True):
        """ACM 에서 **obj ↔ 지정한 로봇 링크들** 쌍만 허용/차단한다(전체가 아니라 그 쌍만).

        `_set_allow` 는 이름 하나를 **모든 것과** 허용해 버려 선별용으로는 너무 거칠다.
        여기서는 필요한 쌍만 건드리므로 되돌리기도 정확하다."""
        if self.apply_scene is None or not links:
            return False
        acm = self._read_acm()
        if acm is None:
            return False
        if obj not in acm.entry_names:                 # 아직 행이 없으면 만든다
            acm.entry_names.append(obj)
            for e in acm.entry_values:
                e.enabled.append(False)
            acm.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
        idx = {n: i for i, n in enumerate(acm.entry_names)}
        i = idx[obj]
        touched = 0
        for ln in links:
            j = idx.get(ln)
            if j is None:
                continue
            acm.entry_values[i].enabled[j] = bool(value)
            acm.entry_values[j].enabled[i] = bool(value)
            touched += 1
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.allowed_collision_matrix = acm
        self._call(self.apply_scene, ApplyPlanningScene.Request(scene=ps), 15.0)
        if not verify:
            return touched > 0
        chk = self._read_acm()                          # 응답을 믿지 않고 재조회
        if chk is None or obj not in chk.entry_names:
            return False
        c = {n: k for k, n in enumerate(chk.entry_names)}
        row = chk.entry_values[c[obj]].enabled
        bad = [ln for ln in links if ln in c and row[c[ln]] != bool(value)]
        if bad:
            self.get_logger().warn(f"ACM 쌍 설정 확인 실패({obj}↔{bad[:3]} → {value})")
            return False
        return True

    def _screen_acm(self, on):
        """[선별 전용] **옥토맵 ↔ 그리퍼 링크** 충돌만 잠시 허용하고, 끝나면 되돌린다.

        ★ 판정을 **비파괴적**으로 만드는 장치다(2026-07-29). 종전에는 후보마다 구 영역
          CollisionObject 를 넣었다 뺐고, 그때마다 MoveIt 이 그 구를 **옥토맵 센서 마스크**에
          등록해 안쪽 복셀을 지웠다 — 그리고 **그 마스킹은 비가역**이다(정적 카메라). 그래서
          라운드마다 장면이 달라졌다(실측: 1라운드만 5개·29초, 2라운드부터 흔들림·느려짐).
          ACM 만 건드리면 **지도를 손대지 않고**, 정확히 되돌릴 수 있다.
        허용 범위를 **그리퍼 링크로 한정**하는 이유: 열매를 감싸는 손만 그 열매 복셀을 무시하면
        되고, 팔은 여전히 캐노피를 피해야 한다(전부 허용하면 판정이 무의미해진다)."""
        links = list(self._grip_links) or [self.ik_link]
        if not self._has_octomap():
            return False                                # 설계값 장면 — 옥토맵 자체가 없다
        return self._set_pair_allow(self.OCTOMAP_ID, links, bool(on))

    def _harvestable_targets(self):
        """워크스페이스 내 수확 가능한 열매만 도출(가까운 순). 기하 프리필터 후 IK 로 확정."""
        tg = self._sorted_targets()
        self._hv_why, self._hv_pairs = {}, {}    # 탈락 사유·접촉 쌍(이 라운드)
        # ★ 비파괴 선별: 라운드 시작에 ACM 을 **한 번만** 열고 끝나면 되돌린다.
        #   (종전: 후보마다 구 영역 객체를 넣었다 뺐다 → 장면이 누적 변형됐다)
        nd = bool(self.get_parameter("screen_nondestructive").value)
        opened = self._screen_acm(True) if nd else False
        try:
            out = [(n, p, r) for (n, p, r) in tg if self._is_harvestable(p, r, n)]
        finally:
            if opened:
                self._screen_acm(False)
        why = " · ".join(f"{k} {v}" for k, v in sorted(self._hv_why.items(),
                                                       key=lambda x: -x[1]))
        # 옥토맵 크기를 같이 남긴다 — 수확 후 후보가 줄면 '지도가 불어난 탓'인지 바로 갈린다.
        ob = self._octomap_bytes()
        self.get_logger().info(
            f"수확 가능 열매 {len(out)} / 전체 {len(tg)} (워크스페이스 필터)"
            + (f" · 탈락: {why}" if why else "")
            + (f" · 옥토맵 {ob}B" if ob > 0 else ""))
        if self._hv_pairs:
            top = sorted(self._hv_pairs.items(), key=lambda x: -x[1])[:5]
            self.get_logger().info(
                "    막은 것: " + ", ".join(f"{k}×{v}" for k, v in top))
        return out

    def reach_noise(self):
        """[진단] 수확가능 판정을 K 회 반복해 **판정 자체의 흔들림**을 수치로 낸다.

        같은 질문을 K 번 한다 → 편차는 전부 판정 과정에서 온다. 이 편차가 '수확하면 뒤가
        열린다'(1~2개)보다 크면 그 효과는 측정 자체가 불가능하다.

        실측(인지 장면, 검출 19개): tries=1 → 5,1,4,3,4개 · tries=3 → 5,2,2,3,4개.
        **재시도로는 안 줄었다.** 대신 **1라운드만 5개·29초로 두 실행 모두 동일**하고 2라운드
        부터 흔들리며 느려진다 → 순수 무작위가 아니라 **판정이 장면을 건드려 생기는 누적**이
        섞여 있다는 뜻(판정이 후보마다 구 영역을 넣었다 빼며, 옥토맵 마스킹은 비가역이다)."""
        k = max(1, int(self.get_parameter("reach_repeat").value))
        tries = int(self.get_parameter("harvest_ik_tries").value)
        if self._has_octomap():          # 센싱 장면이면 지도부터 안정화(빈 지도 = 낙관값)
            b = self._wait_octomap_stable()
            self.get_logger().info(f"옥토맵 안정화 {b}B → 판정 반복 시작")
        self.get_logger().info(
            f"=== 수확가능 판정 잡음 측정: {k}회 반복 · harvest_ik_tries={tries} ===")
        counts, sets = [], []
        for i in range(k):
            t0 = time.time()
            out = self._harvestable_targets()
            names = {n for n, _, _ in out}
            counts.append(len(names))
            sets.append(names)
            self.get_logger().info(
                f"  [{i+1}/{k}] 수확가능 {len(names)}개 ({time.time()-t0:.0f}s) "
                f"{sorted(names)}")
        union = set().union(*sets) if sets else set()
        inter = set.intersection(*sets) if sets else set()
        always = sorted(inter)
        sometimes = sorted(union - inter)
        import statistics as st
        self.get_logger().info(
            f"=== 결과: {min(counts)}~{max(counts)}개 (중앙 {st.median(counts)}, "
            f"평균 {sum(counts)/len(counts):.1f}) ===")
        self.get_logger().info(f"  항상 수확가능 {len(always)}개: {always}")
        self.get_logger().info(
            f"  ⚠ 들쭉날쭉 {len(sometimes)}개(= 판정 잡음): {sometimes}")
        for n in sometimes:
            hit = sum(1 for s in sets if n in s)
            self.get_logger().info(f"     {n}: {hit}/{k}회")
        self.get_logger().info(
            "  ⇒ '수확하면 뒤가 열린다'는 신호가 1~2개 규모이므로, 잡음(들쭉날쭉 개수)이 "
            "그보다 작아야 그 효과를 측정할 수 있다.")
        return dict(counts=counts, always=always, sometimes=sometimes)

    def probe_after_harvest(self):
        """[진단] **수확 1회 전후**로 선별을 반복 추적한다 — "수확하면 뒤가 열리는가,
        아니면 드러난 배경이 새 복셀로 막는가" 를 가르는 측정.

        절차: 옥토맵 안정화 → 수확 전 선별 2회(기준) → 가장 가까운 열매 1개 수확·제거 →
        `probe_after_harvest` 회 만큼 `probe_interval` 초 간격으로 선별 반복.
        매 회 **개수·옥토맵 크기·막은 접촉 쌍**을 남긴다."""
        k = max(1, int(self.get_parameter("probe_after_harvest").value))
        dt = float(self.get_parameter("probe_interval").value)
        if self._has_octomap():
            b = self._wait_octomap_stable()
            self.get_logger().info(f"옥토맵 안정화 {b}B → 기준선 측정")
        self.get_logger().info("=== [수확 전] 기준 선별 2회 ===")
        base = [len(self._harvestable_targets()) for _ in range(2)]
        tg = self._harvestable_targets()
        base.append(len(tg))        # ★ 대상 고르는 이 선별이 '수확 직전' 상태다 — 판정 기준은
                                    #   이것이어야 한다(앞 2회로 판정하면 그 사이 변동을 수확 탓으로 돌린다)
        if not tg:
            self.get_logger().error("수확 가능한 열매가 없어 측정을 중단한다.")
            return
        name, p_fruit, r = tg[0]
        self.get_logger().info(f"=== 수확 대상 {name} @ "
                               f"({p_fruit[0]:.2f},{p_fruit[1]:.2f},{p_fruit[2]:.2f}) ===")
        self._remember_target(name, p_fruit, r)
        self._allow_for_target(name)
        sol = self.solve_pregrasp(p_fruit, r)
        if sol is None:
            self.get_logger().error(f"{name} 계획 실패 — 측정 중단"); self._clear_zone(); return
        try:
            c = self._precompute(name, p_fruit, r, sol)
            self._play_cycle(name, p_fruit, r, c)
        except Exception as e:                                   # noqa: BLE001
            self.get_logger().warn(f"수확 사이클 예외: {e}")
        self._remove_fruit(name, p_fruit)
        self.get_logger().info(f"=== [수확 후] {k}회 추적 ({dt:.0f}s 간격) ===")
        after = []
        for i in range(k):
            n = len(self._harvestable_targets())
            after.append(n)
            self.get_logger().info(f"  [{i+1}/{k}] 수확가능 {n}개")
            if i < k - 1:
                self._hold(dt)
        self.get_logger().info(
            f"=== 판정: 수확 전 {base} → 수확 후 {after} (대상 1개 제거됨) ===")
        exp = base[-1] - 1
        if after and max(after) > exp:
            self.get_logger().info(
                f"  ⇒ **뒤가 열렸다**: 제거분을 빼고도 {max(after)} > {exp} 개")
        elif after and min(after) < exp:
            self.get_logger().info(
                f"  ⇒ **오히려 막혔다**: 제거분을 빼면 {exp} 개여야 하는데 {min(after)} 개 "
                "— 드러난 배경이 새 복셀로 막았을 가능성(위 '막은 것' 로그로 확인)")
        else:
            self.get_logger().info(f"  ⇒ 변화 없음: 딱 제거분 1개만 줄었다({exp}개)")
        return dict(before=base, after=after)

    # ══════════════════ 인터랙티브(조작기) 모드 ══════════════════
    def _itarget_list(self):
        """조작기 목록·번호의 단일 기준. reachable_only 면 수확 가능한 열매만, 아니면 전체(가까운 순)."""
        if bool(self.get_parameter("reachable_only").value):
            return self._harvestable_targets()
        return self._sorted_targets()

    def _publish_target_list(self):
        tg = self._itarget_list()
        self._itargets = tg
        lines = [f"=== 수확 타깃 {len(tg)}개 (가까운 순) ==="]
        for i, (name, p, r) in enumerate(tg):
            lines.append(f"  [{i}] {name}  ({p[0]:.2f},{p[1]:.2f},{p[2]:.2f})  r={r:.3f}")
        lines.append("명령: 번호=수확 · l=목록 · h=home · q=종료")
        text = "\n".join(lines)
        m = String(); m.data = text
        self._targets_pub.publish(m)
        self.get_logger().info(text)

    def _resolve_target(self, cmd):
        tg = getattr(self, "_itargets", None) or self._itarget_list()
        if cmd.isdigit():
            i = int(cmd)
            return tg[i] if 0 <= i < len(tg) else None
        for t in tg:
            if t[0] == cmd:
                return t
        return None

    def _on_cmd(self, msg):
        self._cmd = msg.data.strip()

    def _go_home(self):
        self.get_logger().info("home 복귀…")
        plan = self.plan_to({j: 0.0 for j in self.ARM})
        dur = float(self.get_parameter("dur_home").value)
        if plan is not None:
            self._play_replan(plan[0], plan[1], dur,
                              lambda: self.plan_to({j: 0.0 for j in self.ARM}), "home 복귀")
        else:
            self._play_replan(self.ARM,
                              [[self.cur[j] for j in self.ARM], [0.0] * len(self.ARM)], dur,
                              lambda: self.plan_to({j: 0.0 for j in self.ARM}), "home 복귀")
        self._set({j: 0.0 for j in self.ARM})

    def _handle_cmd(self, cmd):
        c = cmd.lower()
        if c in ("list", "l", "refresh"):
            return                                 # 목록은 루프 끝에서 재발행
        if c in ("home", "h"):
            self._go_home(); return
        tgt = self._resolve_target(cmd)
        if tgt is None:
            self.get_logger().warn(f"타깃 '{cmd}' 못 찾음 (l 로 목록 확인)"); return
        name, p, r = tgt
        self.get_logger().info(
            f"▶ 수확 명령: [{cmd}] {name} @ ({p[0]:.2f},{p[1]:.2f},{p[2]:.2f}) · 전략={self.strategy}")
        self._remember_target(name, p, r)
        self._allow_for_target(name)       # 목록과 같은 기준으로 IK(수확 작업공간 허용 후)
        sol = self.solve_pregrasp(p, r)
        if sol is None:
            self.get_logger().warn(f"{name} 도달불가/충돌 — 다른 타깃 선택 or base 위치 조정"); return
        c2 = self._precompute(name, p, r, sol)
        ok = self._play_cycle(name, p, r, c2)
        if ok:
            self._remove_fruit(name, p)   # 딴 열매는 장면에서 사라진다(다음 목록에서 제외)
        self.get_logger().info(f"[{name}] 수확 {'완료' if ok else '실패'}. 다음 명령 대기…")

    def run_interactive(self):
        """실제 로봇 조작하듯 사용자가 타깃을 골라 명령하는 모드. /harvest_targets 로 목록을 발행하고
        /harvest_cmd(String: 번호|이름|home|list)를 받아 선택 타깃을 전체 수확. harvest_operator 로 조작."""
        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._targets_pub = self.create_publisher(String, "harvest_targets", latched)
        self._cmd = None
        self.create_subscription(String, "harvest_cmd", self._on_cmd, 10)
        self._publish_target_list()
        self.get_logger().info(
            "🕹 인터랙티브 모드 — 다른 터미널에서 "
            "'ros2 run rda_robot_bringup harvest_operator.py' 로 타깃 선택. 명령 대기…")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._cmd is not None:
                cmd, self._cmd = self._cmd, None
                try:
                    self._handle_cmd(cmd)
                except Exception as e:
                    self.get_logger().warn(f"명령 '{cmd}' 처리 실패: {e}")
                self._publish_target_list()        # 명령 후 목록 갱신 재발행


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
        if node.get_parameter("bench_strategy").value:
            node.bench_strategy_compare()      # 전략 × planner 정량 비교(계획만, 재생 없음)
            node.destroy_node()
            rclpy.shutdown()
            return
        if int(node.get_parameter("probe_after_harvest").value) > 0:
            node.probe_after_harvest()      # [진단] 수확 전후 선별 추적 후 종료
            node.destroy_node()
            rclpy.shutdown()
            return
        if int(node.get_parameter("reach_repeat").value) > 0:
            node.reach_noise()          # [진단] 수확가능 판정의 흔들림 측정 후 종료
            node.destroy_node()
            rclpy.shutdown()
            return
        if node.get_parameter("diag_straight").value:
            node._diag_straight()
            node.destroy_node()
            rclpy.shutdown()
            return
        if node.get_parameter("interactive").value:
            node.run_interactive()      # 사용자가 타깃 골라 명령(harvest_operator)
            node.destroy_node()
            rclpy.shutdown()
            return
        if node.get_parameter("harvest_all").value:
            # 연속 수확: 도달 열매를 하나씩. loop 면 전체 세트를 반복.
            while rclpy.ok():
                n = node.run_harvest_all()
                if not node.get_parameter("loop").value:
                    while rclpy.ok():
                        node._hold(1.0)
                    break
                # 다 따서 더 딸 게 없으면 장면을 되돌려 반복(재생 데모용). Gazebo 에서 실제로
                # 지운 경우는 되돌릴 수 없으므로 그대로 둔다.
                if n == 0:
                    node.reset_harvested()
                node._hold(2.0)
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
