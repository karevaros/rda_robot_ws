# 레인보우 제어박스 안전 상태 — 실기 실행 감시용 (실기 준비 G)
#
# 왜 자체 메시지인가
#   벤더 rbpodo_msgs/SystemState 는 필드가 40개가 넘는데, 그것을 발행하는 ROS 노드가
#   **없다** — 벤더 robot_node 는 move_j/move_l 등 액션의 feedback 안에서만 내보낸다.
#   우리는 JointTrajectoryController 로 움직이므로 그 액션을 타지 않는다.
#   그래서 데이터 채널(포트 5001)에서 직접 읽되, **감시에 실제로 쓰는 필드만** 담는다.
#   (전체를 반만 채워 발행하면 '다 채워진 SystemState' 로 오해되기 쉽다.)

std_msgs/Header header

# --- 결함 플래그 (하나라도 true 면 즉시 정지 대상) -------------------------------
bool collision_occur        # 외부 충돌 감지 (op_stat_collision_occur)
bool self_collision         # 자가 충돌 범위 진입 (op_stat_self_collision)
bool soft_estop             # 소프트 E-stop / 일시정지 (op_stat_soft_estop_occur)
int8 ems_flag               # 소프트웨어(기구학) 비상정지 상황 (op_stat_ems_flag, 0=정상)
int8 sos_flag               # 동작 중 장치 오류 코드 (op_stat_sos_flag, 0=정상)

# --- 설정/상태 ------------------------------------------------------------------
bool collision_detect_on    # 외부 충돌 감지 기능 On/Off (꺼져 있으면 위 플래그는 무의미)
int32 task_state            # 1=Idle / 2=Paused / 3=Running
int32 robot_state           # 1=정지 / 3=모션 수행 중 / 5=컨베이어·힘제어 / 60+=MovePB 경유점
float32 default_speed       # 속도 배율(UI 의 speed bar), 0~1
int8 init_state_info        # 팔 활성화(초기화) 단계 0~6

# --- 요약 -----------------------------------------------------------------------
# 위 플래그로 계산한 결론. 구독자가 판정 규칙을 각자 다시 짜지 않도록 여기서 한 번만 정한다.
bool fault                  # true = 결함(정지해야 함)
string fault_reason         # 사람이 읽을 사유(빈 문자열이면 정상)
