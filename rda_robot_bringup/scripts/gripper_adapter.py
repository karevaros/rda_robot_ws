#!/usr/bin/env python3
"""실기 그리퍼 어댑터 — RG2 를 레인보우 제어박스 명령으로 움직인다 (실기 준비 D).

■ 왜 필요한가
  시뮬에선 그리퍼가 ros2_control 관절이다(gazebo_sim.launch.py 가 rg2_finger_joint1 을
  ros2_control 블록에 **주입**한다). 실기에선 그렇지 않다 — RG2 는 레인보우 제어박스가
  RS485 로 물고 있고, 제어박스 스크립트 명령으로 움직인다. 따라서 실기 모드에는
  `gripper_controller`(JTC)가 존재하지 않는다.

■ 설계 — 수확 시퀀스를 고치지 않는다
  `pregrasp_demo.py` 는 그리퍼를 **FollowJointTrajectory 액션**으로 보낸다
  (`_execute_traj` 가 손가락 관절이면 gripper_controller 로 라우팅). 이 노드는 **같은 액션
  서버를 그대로 흉내낸다** → 수확 시퀀스 코드는 한 줄도 바뀌지 않고, 시뮬/실기가
  같은 경로를 쓴다. 관절 목표(rad)를 제어박스 명령으로 번역하는 게 이 노드의 전부다.

■ 🔴 명령 문자열을 하드코딩하지 않는다 (2026-07-30 조사 결과)
  레인보우 UI Script 문서(v6.10, 챕터 8 은 8.28 에서 끝)에는 **OnRobot RG2 가 없다**
  (Robotiq·ROBOTIS RH-P12-RN·주강·DH AG-95·Setech 만 있다). 반면 SDK 의
  `GripperModel` enum 에는 `OnRobot_RG2 = 12` 가 있고, SDK 는 일반형
  `gripper_macro <model>,<conn>,<func>,<a3>..<a9>` (총 10필드)로 명령을 만든다.
  ⇒ **RG2 의 func 코드·인자 배치는 문서로 확인되지 않았다.** 그래서 명령을 파라미터
  (`cmd_move`·`cmd_init`)로 빼 둔다. 티치펜던트에서 쓰는 실제 문자열을 넣으면 그대로 나간다.
  기본값은 SDK 형식을 따른 **미검증 추정값**이며, 실기 첫 구동 전에 반드시 확인할 것.

■ 🔴 이 노드는 '잡혔는지' 를 알 수 없다
  `Eval` 서비스의 success 는 **스크립트가 접수됐다**는 뜻일 뿐, 그리퍼가 움직였다는 뜻이
  아니다. 제어박스의 `SystemState`(tfb_digital_in 등)는 **ROS 토픽으로 발행되지 않는다** —
  벤더 robot_node 는 그것을 move_j/move_l 같은 **액션 feedback 안에서만** 내보낸다.
  따라서 여기서는 명령 후 `settle_sec` 만큼 기다린 뒤 성공으로 간주한다. 진짜 확인이
  필요하면 데이터 채널(포트 5001) 구독 노드가 따로 있어야 한다(후속 G 항목).

실행:
    ros2 run rda_robot_bringup gripper_adapter.py --ros-args \
        -r __ns:=/gripper_controller -p dry_run:=true      # 오프라인 점검
"""
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

try:
    from rbpodo_msgs.srv import Eval
except ImportError:      # SDK 미설치 환경에서도 dry_run 점검은 되게 한다
    Eval = None


class GripperAdapter(Node):
    def __init__(self):
        super().__init__("gripper_adapter")

        self.declare_parameter("eval_service", "/rbpodo_hardware/eval")
        self.declare_parameter("joint", "rg2_finger_joint1")
        self.declare_parameter("mimic_joint", "rg2_finger_joint2")   # URDF mimic(배율 1)
        # 관절 범위 — URDF 실측(lower 0.0=닫힘 / upper 1.18=열림)
        self.declare_parameter("joint_closed", 0.0)
        self.declare_parameter("joint_open", 1.18)
        self.declare_parameter("stroke_mm", 100.0)   # RG2 최대 개구(제원 확인 후 조정)

        # 🔴 미검증 추정값 — 위 주석 참조. 티치펜던트 문자열로 교체할 것.
        #    {conn}=연결점(0 ToolFlange / 1 ControlBox / 2 ToolFlange_Advanced)
        #    {width_mm}·{ratio_pct}·{force_pct} 를 쓸 수 있다.
        self.declare_parameter("cmd_move",
                               "gripper_macro 12,{conn},2,0,{width_mm:.0f},0,0,0,0,0")
        self.declare_parameter("cmd_init", "")       # 비우면 초기화 명령을 보내지 않는다
        self.declare_parameter("conn_point", 2)
        self.declare_parameter("force_pct", 40)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("dry_run", False)     # true=명령을 보내지 않고 로그만

        p = lambda n: self.get_parameter(n).value
        self.joint, self.mimic = p("joint"), p("mimic_joint")
        self.j_closed, self.j_open = float(p("joint_closed")), float(p("joint_open"))
        if abs(self.j_open - self.j_closed) < 1e-6:
            raise RuntimeError("joint_open 과 joint_closed 가 같습니다 — 관절 범위 확인 필요")
        self.dry = bool(p("dry_run"))

        self._cli = None
        if not self.dry:
            if Eval is None:
                raise RuntimeError(
                    "rbpodo_msgs 를 import 할 수 없습니다.\n"
                    "  bash src/docs/scripts/setup_rbpodo_sdk.sh 로 빌드하거나, "
                    "오프라인 점검이면 dry_run:=true 로 실행하세요.")
            self._cli = self.create_client(Eval, p("eval_service"))

        self._pos = self.j_open          # 시작 자세 가정 = 열림
        # ⚠ 이름에 `~` 를 쓰지 말 것 — 노드 이름까지 들어가
        #   /gripper_controller/gripper_adapter/... 가 되어 수확 시퀀스가 기대하는
        #   /gripper_controller/follow_joint_trajectory 와 어긋난다(실측으로 확인).
        #   상대 이름이면 네임스페이스(/gripper_controller) 바로 아래에 붙는다.
        self._js_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / 30.0, self._pub_js)

        self._srv = ActionServer(self, FollowJointTrajectory,
                                 "follow_joint_trajectory", self._on_goal)

        self.get_logger().info(
            f"그리퍼 어댑터 시작 — 관절 {self.joint} [{self.j_closed}, {self.j_open}] rad → "
            f"0~{float(p('stroke_mm')):.0f}mm" + ("  [dry_run]" if self.dry else
                                                  f"  eval={p('eval_service')}"))
        if p("cmd_init"):
            self._send(self._fmt(p("cmd_init"), self._pos))

    # ------------------------------------------------------------------ 변환
    def _ratio(self, pos):
        """관절 위치(rad) → 개구 비율 0(닫힘)~1(열림). 범위를 벗어나면 자른다."""
        r = (pos - self.j_closed) / (self.j_open - self.j_closed)
        return min(1.0, max(0.0, r))

    def _fmt(self, template, pos):
        r = self._ratio(pos)
        return template.format(
            conn=int(self.get_parameter("conn_point").value),
            ratio_pct=r * 100.0,
            width_mm=r * float(self.get_parameter("stroke_mm").value),
            force_pct=int(self.get_parameter("force_pct").value),
        )

    # ------------------------------------------------------------------ 전송
    def _send(self, script):
        if self.dry:
            self.get_logger().info(f"[dry_run] 보낼 명령: {script!r}")
            return True
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"eval 서비스 없음: {self.get_parameter('eval_service').value}\n"
                "  실기 하드웨어 인터페이스(rbpodo_hardware)가 활성인지 확인하세요 "
                "— mock/Gazebo 모드에는 이 서비스가 없습니다.")
            return False
        req = Eval.Request()
        req.script = script
        fut = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            self.get_logger().error(f"eval 응답 없음: {script!r}")
            return False
        if not res.success:
            self.get_logger().error(f"제어박스가 거부: {script!r}")
            return False
        self.get_logger().info(f"보냄: {script!r}")
        return True

    # ------------------------------------------------------------------ 액션
    def _on_goal(self, goal_handle):
        """FollowJointTrajectory — 마지막 점의 목표 관절값만 쓴다.
        그리퍼는 경유점을 따라갈 수 없다(제어박스가 목표 개구만 받는다)."""
        traj = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        if self.joint not in traj.joint_names or not traj.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"궤적에 {self.joint} 이(가) 없습니다: {traj.joint_names}"
            self.get_logger().warn(result.error_string)
            return result

        i = traj.joint_names.index(self.joint)
        target = float(traj.points[-1].positions[i])
        if len(traj.points) > 1:
            self.get_logger().debug(
                f"경유점 {len(traj.points)}개 중 마지막만 사용(그리퍼는 개구 목표만 받는다)")

        ok = self._send(self._fmt(self.get_parameter("cmd_move").value, target))
        if not ok:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "제어박스 명령 전송 실패"
            return result

        # ⚠ 도달 확인 수단이 없다(위 주석) — 정해진 시간만 기다리고 성공으로 본다.
        settle = float(self.get_parameter("settle_sec").value)
        end = self.get_clock().now().nanoseconds + int(settle * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        self._pos = target

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "명령 전송 완료(도달 여부는 확인 불가 — 주석 참조)"
        return result

    # ------------------------------------------------------------------ 상태
    def _pub_js(self):
        """손가락 관절 상태 발행. real_robot.launch.py 의 joint_state_publisher
        source_list 에 이 토픽을 넣으면 /joint_states·TF 가 손가락까지 채워진다."""
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = [self.joint, self.mimic]
        m.position = [self._pos, self._pos]      # URDF mimic 배율 1
        self._js_pub.publish(m)


def main():
    rclpy.init()
    node = None
    try:
        node = GripperAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as e:
        if isinstance(e, RuntimeError):
            print(f"error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
