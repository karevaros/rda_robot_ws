#!/usr/bin/env python3
"""실기 실행 감시 — 제어박스 결함 시 즉시 정지 (실기 준비 G).

■ 무엇을 하나
  `cb_safety_publisher` 가 내는 SafetyState 를 구독하다가 결함(충돌·E-stop·장치오류)이
  뜨면 **한 번에 세 가지**를 한다. 하나만으로는 확실히 안 멈춘다:
    ① 진행 중인 궤적 goal 을 **취소**(arm_controller / gripper_controller 액션)
    ② 제어박스 `~/task_stop` 호출 — 제어박스 쪽 동작 정지
    ③ `SetSpeedBar(0)` — 다음 명령이 곧바로 튀어나가지 못하게 속도 배율을 0 으로
  그리고 `~/abort` 를 **TRANSIENT_LOCAL 로 래치**해 발행한다. 수확 시퀀스는 이걸 보고
  다음 사이클로 넘어가지 않는다(늦게 뜬 구독자도 이미 난 결함을 받는다).

■ 🔴 정지는 '복구'가 아니다
  이 노드는 멈추기만 한다. 결함 해제 후 재개는 **사람이 판단**해야 한다
  (`~/reset` 서비스로 명시적으로 풀어야 abort 가 내려간다). 자동 재개를 넣지 않은 이유:
  충돌 원인이 그대로면 재개가 곧 재충돌이다.

■ ⚠ 감시가 성립하려면
  제어박스의 외부충돌 감지가 **켜져 있어야** 한다(SafetyState.collision_detect_on).
  꺼져 있으면 collision_occur 는 영원히 false 다 — '결함 없음'이 아니라 '감시 안 함'이다.
  그리고 상태가 아예 안 들어오는 것(발행 노드 죽음·네트워크 끊김)도 결함으로 본다
  (`stale_sec`). 조용히 감시가 사라지는 게 제일 위험하다.

실행(합성 메시지로 오프라인 점검):
    ros2 run rda_robot_bringup safety_monitor.py --ros-args -p require_services:=false
    ros2 topic pub --once /safety_monitor/safety_state rda_robot_msgs/msg/SafetyState \
        "{collision_occur: true, fault: true, fault_reason: '외부충돌'}"
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from control_msgs.action import FollowJointTrajectory
from rda_robot_msgs.msg import SafetyState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

try:
    from rbpodo_msgs.srv import TaskStop, SetSpeedBar
except ImportError:      # SDK 미설치 환경에서도 감시 로직은 점검 가능
    TaskStop = SetSpeedBar = None


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__("safety_monitor")

        self.declare_parameter("controllers", ["arm_controller", "gripper_controller"])
        self.declare_parameter("task_stop_service", "/rbpodo_hardware/task_stop")
        self.declare_parameter("speed_bar_service", "/rbpodo_hardware/set_speed_bar")
        self.declare_parameter("stale_sec", 2.0)     # 이 시간 넘게 상태가 없으면 결함
        self.declare_parameter("require_services", True)   # false=서비스 없어도 기동(점검용)
        self.declare_parameter("warn_detect_off", True)
        # 🔴 상태가 **한 번도** 안 들어오는 경우(발행 노드를 안 띄웠다·이름이 틀렸다)를
        #    막는다. 이걸 빼면 감시 노드가 떠 있는데 실제로는 아무것도 감시하지 않는
        #    상태가 조용히 성립한다 — 이 프로젝트가 반복해 밟은 '조용한 무효' 그대로다.
        #    require_state=false 로 끄는 건 **명시적 선택**이어야 한다(기본은 켜짐).
        self.declare_parameter("require_state", True)
        self.declare_parameter("startup_grace_sec", 10.0)

        p = lambda n: self.get_parameter(n).value
        self.stale_sec = float(p("stale_sec"))

        # 래치 발행 — 늦게 뜬 구독자도 이미 난 결함을 받는다.
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self._abort_pub = self.create_publisher(Bool, "~/abort", latched)
        self._reason_pub = self.create_publisher(String, "~/abort_reason", latched)

        self._acs = {c: ActionClient(self, FollowJointTrajectory,
                                     f"/{c}/follow_joint_trajectory")
                     for c in p("controllers")}

        self._stop_cli = self._speed_cli = None
        if TaskStop is not None:
            self._stop_cli = self.create_client(TaskStop, p("task_stop_service"))
            self._speed_cli = self.create_client(SetSpeedBar, p("speed_bar_service"))
        elif p("require_services"):
            raise RuntimeError(
                "rbpodo_msgs 를 import 할 수 없습니다(제어박스 정지 서비스 사용 불가).\n"
                "  bash src/docs/scripts/setup_rbpodo_sdk.sh 로 빌드하거나,\n"
                "  로직만 점검하려면 require_services:=false 로 실행하세요.")

        sub = self.create_subscription(SafetyState, "~/safety_state", self._on_state, 10)
        self._state_topic = sub.topic_name
        self.create_service(Trigger, "~/reset", self._on_reset)

        self._aborted = False
        self._last_stamp = None
        self._started = self.get_clock().now()
        self._detect_off_warned = False
        self._no_state_warned = False
        self._cancel_clis = {}
        self.create_timer(0.5, self._check_stale)

        self._publish_abort(False, "")
        self.get_logger().info(
            f"안전 감시 시작 — 컨트롤러 {list(self._acs)} · stale {self.stale_sec}s"
            + ("" if self._stop_cli else "  [서비스 없음: 취소·래치만 동작]"))

    # ------------------------------------------------------------------ 수신
    def _on_state(self, msg: SafetyState):
        self._last_stamp = self.get_clock().now()

        if (self.get_parameter("warn_detect_off").value
                and not msg.collision_detect_on and not self._detect_off_warned):
            self._detect_off_warned = True
            self.get_logger().warn(
                "제어박스 외부충돌 감지가 꺼져 있습니다 — 충돌 플래그가 항상 false 입니다. "
                "('결함 없음'이 아니라 '감시 안 함')")

        if msg.fault and not self._aborted:
            self._trip(msg.fault_reason or "결함(사유 없음)")

    def _check_stale(self):
        """상태가 끊긴 것도 결함으로 본다 — 감시가 조용히 사라지는 게 제일 위험하다."""
        if self._aborted:
            return
        now = self.get_clock().now()

        if self._last_stamp is None:
            # 아직 한 번도 못 받았다. 기동 유예 뒤에도 없으면 '감시 없음' 이므로 결함.
            grace = float(self.get_parameter("startup_grace_sec").value)
            dt = (now - self._started).nanoseconds / 1e9
            if dt < grace:
                return
            msg = (f"안전 상태를 한 번도 받지 못했습니다({dt:.0f}s) — "
                   f"'{self._state_topic}' 발행 노드(cb_safety_publisher)가 떠 있는지 확인하세요.")
            if self.get_parameter("require_state").value:
                self._trip(msg)
            elif not self._no_state_warned:
                self._no_state_warned = True
                self.get_logger().warn(
                    msg + "  (require_state:=false 이므로 정지시키지 않습니다 — "
                          "이 세션은 실행 감시가 없는 상태입니다.)")
            return

        dt = (now - self._last_stamp).nanoseconds / 1e9
        if dt > self.stale_sec:
            self._trip(f"안전 상태 수신 끊김({dt:.1f}s > {self.stale_sec}s)")

    # ------------------------------------------------------------------ 정지
    def _trip(self, reason):
        self._aborted = True
        self.get_logger().error(f"🔴 결함 감지 — 정지합니다: {reason}")
        self._publish_abort(True, reason)      # ① 먼저 알린다(취소가 막혀도 전파는 된다)

        for name, ac in self._acs.items():     # ② 진행 중 궤적 취소
            if ac.server_is_ready():
                self._cancel_all(name)
            else:
                self.get_logger().warn(f"  {name}: 액션 서버 없음 — 취소 생략")

        self._call(self._stop_cli, "task_stop")            # ③ 제어박스 정지
        self._call(self._speed_cli, "set_speed_bar", speed_bar=0.0)

    def _cancel_all(self, name):
        """해당 액션 서버의 goal 을 전부 취소한다.

        rclpy ActionClient 는 goal handle 을 들고 있어야 취소할 수 있는데, 감시 노드는
        남이 보낸 goal 의 handle 을 모른다. 그래서 액션의 취소 서비스를 직접 부른다 —
        `goal_info` 를 비우면(=goal_id 0, stamp 0) **해당 서버의 모든 goal 취소**다.
        실패해도 ①(래치 통보)③(제어박스 정지)이 남으므로 단일 실패점은 아니다."""
        try:
            from action_msgs.srv import CancelGoal
            cli = self._cancel_clis.get(name)
            if cli is None:
                cli = self.create_client(
                    CancelGoal, f"/{name}/follow_joint_trajectory/_action/cancel_goal")
                self._cancel_clis[name] = cli
            if cli.wait_for_service(timeout_sec=0.5):
                cli.call_async(CancelGoal.Request())
                self.get_logger().info(f"  {name}: 궤적 전체 취소 요청")
            else:
                self.get_logger().warn(f"  {name}: 취소 서비스 없음")
        except Exception as e:                 # noqa: BLE001
            self.get_logger().warn(f"  {name}: 취소 실패 — {e}")

    def _call(self, cli, what, **kw):
        if cli is None:
            return
        if not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"  {what}: 서비스 없음 — 건너뜀")
            return
        req = cli.srv_type.Request()
        for k, v in kw.items():
            setattr(req, k, v)
        cli.call_async(req)
        self.get_logger().info(f"  {what}: 호출")

    # ------------------------------------------------------------------ 해제
    def _on_reset(self, _req, res):
        """🔴 사람이 명시적으로 풀어야 한다 — 자동 재개는 넣지 않았다(주석 참조)."""
        if not self._aborted:
            res.success, res.message = True, "결함 없음(이미 정상)"
            return res
        self._aborted = False
        self._last_stamp = self.get_clock().now()
        self._publish_abort(False, "")
        self.get_logger().warn("정지 상태를 해제했습니다 — 원인이 제거됐는지 확인하세요.")
        res.success, res.message = True, "해제됨"
        return res

    def _publish_abort(self, val, reason):
        self._abort_pub.publish(Bool(data=val))
        self._reason_pub.publish(String(data=reason))


def main():
    rclpy.init()
    node = None
    try:
        node = SafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(f"error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
