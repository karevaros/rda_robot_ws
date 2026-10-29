#!/usr/bin/env bash
# 사용: bash real_joint_sweep.sh   🔴 실제 팔이 움직인다 — 비상정지 옆에 사람 · 팔 끝 반경 10cm 비울 것
#   제어박스가 Real·활성화(6)·오류 0 이 아니면 시작하지 않는다. 진단 도구 cb_diag(아래 DIAG) 필요.
# 2026-10-29 실측(rb850_old · Real · 유선): wrist2→wrist1→elbow→shoulder→base 각 ±5° 전부 SUCCEEDED · 다른 관절 0 · 이상로그 0.
# Real 모드 관절별 ±5° — 실패·이상 시 즉시 중단, 끝나면 launch 프로세스 그룹 정리
source /opt/ros/humble/setup.bash; source ~/robot_ws/install/setup.bash
cd "$(dirname "$0")"
DIAG=${DIAG:-$HOME/robot_ws/build/cb_diag}   # g++ -std=c++17 cb_diag.cpp -I ~/robot_ws/thirdparty/install/include -L ~/robot_ws/thirdparty/install/lib -lrbpodo -o $DIAG
LOG=${LOG:-/tmp/real_joint_sweep.log}
cleanup() {
  # 실제 ros2 launch 프로세스(comm=ros2)의 그룹만 — pgrep -f 는 같은 문자열의 다른 프로세스
  # (이 스크립트 자신의 그룹)를 집어 스크립트를 스스로 죽였다(2026-10-29).
  G=$(ps -eo pgid=,comm=,args= | awk '$2=="ros2" && /real_robot.launch.py/ {print $1; exit}')
  [ -n "$G" ] && [ "$G" != "$(ps -o pgid= -p $$ | tr -d ' ')" ] && kill -INT -- -$G
  for i in $(seq 1 20); do
    n=$(ps -eo comm | grep -cE '^(robot_state_pub|ros2_control_no|cb_safety_publi|joint_state_pub)$')
    m=$(ps -eo args | awk '$1 ~ /python3/ && ($2 ~ /gripper_adapter|safety_monitor|real_robot/ || $3 ~ /real_robot/)' | wc -l)
    [ $((n+m)) -eq 0 ] && break; sleep 1
  done
  echo "정리: 잔여 $((n+m))"; $DIAG | sed -n 2p | cut -c1-60
}
trap cleanup EXIT
health() {
  local hw; hw=$(ros2 control list_hardware_components | grep -o "label=[^ ]*" | head -1)
  local ab; ab=$(timeout 3 ros2 topic echo --once /safety_monitor/abort | head -1)
  local bad; bad=$(grep -cE "ERROR|state was updataed|clamped" $LOG)
  echo "  [상태] hw $hw · abort ${ab#data: } · 이상로그 $bad"
  [[ "$hw" == *active* && "$ab" == "data: false" && "$bad" == 0 ]]
}
$DIAG | sed -n 2p | grep -q "sim_mode=0 init_state=6 init_error=0" || { echo "ABORT: Real/활성 아님"; exit 1; }
setsid ros2 launch rda_robot_description real_robot.launch.py robot_ip:=192.168.35.100 cb_simulation:=false gripper:=false > $LOG 2>&1 < /dev/null &
until grep -qE "Configured and activated.*arm_controller|Caught exception|ERROR" $LOG; do sleep 1; done; sleep 3
grep -q "mode: Real" $LOG || { echo "ABORT: 하드웨어가 Real 로 접속하지 않음"; exit 1; }
health || { echo "ABORT: 시작 상태 이상"; exit 1; }
NAMES=(base shoulder elbow wrist1 wrist2 wrist3)
for J in 4 3 2 1 0; do
  echo "== ${NAMES[$J]} ±5°"
  python3 real_traj_smoke.py 5 3 $J | grep -vE "시작 자세"; rc=${PIPESTATUS[0]}
  sleep 1
  health || { echo "ABORT: ${NAMES[$J]} 뒤 상태 이상"; exit 1; }
  [ $rc -eq 0 ] || { echo "ABORT: ${NAMES[$J]} 실패(rc=$rc)"; exit 1; }
done
echo "== 전 관절 통과"
