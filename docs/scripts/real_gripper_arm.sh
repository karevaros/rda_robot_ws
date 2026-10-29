#!/usr/bin/env bash
# 실기 2단계: 그리퍼 + 팔 동시 동작(Real). 🔴 실제 팔·그리퍼가 움직인다 — 비상정지 옆에 사람 · 손가락 사이 비울 것.
# 순서: 접속(그리퍼 초기화) → 그리퍼 100% → wrist3 ±5° 도중 1.5s 에 그리퍼 48% → 그리퍼 100%. 매 단계 hw·abort·이상로그 확인, 이상이면 중단.
# 2026-10-29 실측(rb850_old · Real · 유선): 전 단계 통과 · 궤적 SUCCEEDED · 이상로그 0 (패치 03 script_grace 의 실기 확인).
source /opt/ros/humble/setup.bash; source ~/robot_ws/install/setup.bash
cd "$(dirname "$0")"
DIAG=${DIAG:-$HOME/robot_ws/build/cb_diag}; LOG=${LOG:-/tmp/real_gripper_arm.log}
cleanup() {
  G=$(ps -eo pgid=,comm=,args= | awk '$2=="ros2" && /real_robot.launch.py/ {print $1; exit}')
  [ -n "$G" ] && [ "$G" != "$(ps -o pgid= -p $$ | tr -d ' ')" ] && kill -INT -- -$G
  for i in $(seq 1 20); do
    n=$(ps -eo comm | grep -cE '^(robot_state_pub|ros2_control_no|cb_safety_publi|joint_state_pub)$')
    m=$(ps -eo args | awk '$1 ~ /python3/ && ($2 ~ /gripper_adapter|safety_monitor/)' | wc -l)
    [ $((n+m)) -eq 0 ] && break; sleep 1
  done
  echo "정리: 잔여 $((n+m))"; $DIAG | sed -n 2p | cut -c1-60
}
trap cleanup EXIT
health() {
  local hw ab bad
  hw=$(ros2 control list_hardware_components 2>/dev/null | grep -o "label=[^ ]*" | head -1)
  ab=$(timeout 3 ros2 topic echo --once /safety_monitor/abort | head -1)
  bad=$(grep -cE "ERROR|state was updataed|clamped|거부|응답 없음" $LOG)
  echo "  [상태 $1] hw $hw · abort ${ab#data: } · 이상로그 $bad"
  [[ "$hw" == *active* && "$ab" == "data: false" && "$bad" == 0 ]]
}
grip() { ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [rg2_finger_joint1], points: [{positions: [$1], time_from_start: {sec: 1}}]}}" 2>&1 | grep -oE "status: [A-Z]+"; }
$DIAG | sed -n 2p | grep -q "sim_mode=0 init_state=6 init_error=0" || { echo "ABORT: Real/활성 아님"; exit 1; }
setsid ros2 launch rda_robot_description real_robot.launch.py robot_ip:=192.168.35.100 cb_simulation:=false > $LOG 2>&1 < /dev/null &
until grep -qE "Configured and activated.*arm_controller|Caught exception|ERROR" $LOG; do sleep 1; done; sleep 4
grep -q "mode: Real" $LOG || { echo "ABORT: Real 접속 아님"; exit 1; }
grep -E "보냄" $LOG | cut -c60-140
health 시작 || { echo "ABORT"; exit 1; }
echo "== 그리퍼 100%"; grip 1.18; sleep 2; health "100% 뒤" || { echo "ABORT"; exit 1; }
echo "== wrist3 ±5° + 1.5s 에 그리퍼 48%"
(sleep 1.5; grip 0.5664 >/dev/null) & GP=$!
python3 real_traj_smoke.py 5 3 5 | grep -vE "시작 자세"; rc=${PIPESTATUS[0]}
wait $GP   # ⚠ 인자 없는 wait 는 백그라운드 launch 까지 기다려 멈춘다(2026-10-29)
health "동시동작 뒤" || { echo "ABORT"; exit 1; }; [ $rc -eq 0 ] || { echo "ABORT: 궤적 rc=$rc"; exit 1; }
echo "== 그리퍼 100% 복귀"; grip 1.18; sleep 2; health "복귀 뒤" || { echo "ABORT"; exit 1; }
grep -E "보냄" $LOG | cut -c60-140
echo "== 2번 통과"
