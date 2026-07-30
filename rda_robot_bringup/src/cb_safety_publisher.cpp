// 레인보우 제어박스 안전 상태 발행 노드 — 실기 실행 감시 (실기 준비 G)
//
// ■ 왜 필요한가
//   제어박스는 충돌·E-stop 상태를 다 알고 있는데 **ROS 로 나오는 길이 없다**.
//   벤더 robot_node 는 SystemState 를 move_j/move_l/move_jb2/move_pb **액션의
//   feedback 안에서만** 내보내고, 우리는 JointTrajectoryController 로 움직이므로
//   그 액션을 타지 않는다. 그래서 데이터 채널(포트 5001)에서 직접 읽어 발행한다.
//
// ■ 🔴 검증되지 않은 전제 — 데이터 채널 동시 접속
//   rbpodo_hardware 가 이미 CobotData 연결 하나를 쓰고 있다(500Hz 상태 수신).
//   이 노드는 **두 번째 연결**을 연다. 제어박스가 데이터 채널 다중 접속을 허용하는지
//   문서로 확인하지 못했다. 허용하지 않으면 하드웨어 인터페이스의 상태 수신이
//   끊길 수 있다 — 그건 팔 제어에 직접 영향을 준다.
//   ⇒ 그래서 launch 기본값은 **꺼짐**이다. 실기 첫 연결 때 이것부터 확인할 것:
//      ① 이 노드만 단독 실행 → 값이 들어오는지
//      ② 하드웨어 인터페이스와 동시 실행 → 양쪽 다 정상인지 (팔 /joint_states 가 계속 갱신되는지)
//   만약 다중 접속이 안 되면 대안은 벤더 robot_node 에 퍼블리셔를 넣는 것(벤더 수정)이다.
//
// ■ 판정은 여기서 한 번만 한다
//   구독자마다 규칙을 다시 짜면 조용히 갈린다. fault/fault_reason 을 여기서 채운다.

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rbpodo/rbpodo.hpp>

#include "rda_robot_msgs/msg/safety_state.hpp"

using namespace std::chrono_literals;

class CBSafetyPublisher : public rclcpp::Node {
 public:
  CBSafetyPublisher() : rclcpp::Node("cb_safety_publisher") {
    ip_ = declare_parameter<std::string>("robot_ip", "10.0.2.7");
    const double rate = declare_parameter<double>("rate", 20.0);
    // 충돌 감지 기능이 꺼져 있으면 collision_occur 는 영원히 false 다 —
    // '결함 없음'과 '감시 안 함'을 구분하기 위해 경고를 낸다.
    warn_detect_off_ = declare_parameter<bool>("warn_if_detect_off", true);

    pub_ = create_publisher<rda_robot_msgs::msg::SafetyState>("~/safety_state", 10);

    RCLCPP_INFO(get_logger(), "제어박스 데이터 채널 접속 시도: %s:5001", ip_.c_str());
    try {
      data_ = std::make_unique<rb::podo::CobotData>(ip_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(),
                   "데이터 채널 접속 실패: %s\n"
                   "  · 제어박스 전원·네트워크를 확인하세요(robot_ip:=<주소>).\n"
                   "  · 이 노드는 하드웨어 없이는 동작할 수 없습니다.",
                   e.what());
      throw;
    }
    RCLCPP_INFO(get_logger(), "접속 성공 — %.0fHz 로 발행합니다.", rate);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });
  }

 private:
  void tick() {
    auto s = data_->request_data();
    if (!s.has_value()) {
      if (++miss_ % 20 == 1) {
        RCLCPP_WARN(get_logger(), "데이터 채널 응답 없음 (%d회 연속)", miss_);
      }
      return;
    }
    miss_ = 0;
    const auto& d = s.value().sdata;

    rda_robot_msgs::msg::SafetyState m;
    m.header.stamp = now();
    m.collision_occur = (d.op_stat_collision_occur != 0);
    m.self_collision = (d.op_stat_self_collision != 0);
    m.soft_estop = (d.op_stat_soft_estop_occur != 0);
    m.ems_flag = static_cast<int8_t>(d.op_stat_ems_flag);
    m.sos_flag = static_cast<int8_t>(d.op_stat_sos_flag);
    m.collision_detect_on = (d.collision_detect_onoff != 0);
    m.task_state = d.task_state;
    m.robot_state = d.robot_state;
    m.default_speed = d.default_speed;
    m.init_state_info = static_cast<int8_t>(d.init_state_info);

    // 판정 — 하나라도 걸리면 결함. 사유는 겹쳐 적는다(첫 번째만 적으면 원인을 놓친다).
    std::string why;
    auto add = [&why](const char* s) { why += (why.empty() ? "" : " + "); why += s; };
    if (m.collision_occur) add("외부충돌");
    if (m.self_collision) add("자가충돌");
    if (m.soft_estop) add("소프트 E-stop");
    if (m.ems_flag != 0) add("비상정지(ems)");
    if (m.sos_flag != 0) add("장치오류(sos)");
    m.fault = !why.empty();
    m.fault_reason = why;

    if (warn_detect_off_ && !m.collision_detect_on && (warned_off_++ % 200 == 0)) {
      RCLCPP_WARN(get_logger(),
                  "제어박스 외부충돌 감지가 꺼져 있습니다 — collision_occur 는 "
                  "항상 false 입니다('결함 없음'이 아니라 '감시 안 함').");
    }
    pub_->publish(m);
  }

  std::string ip_;
  bool warn_detect_off_{true};
  int miss_{0};
  int warned_off_{0};
  std::unique_ptr<rb::podo::CobotData> data_;
  rclcpp::Publisher<rda_robot_msgs::msg::SafetyState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CBSafetyPublisher>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("cb_safety_publisher"), "종료: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
