#!/usr/bin/env bash
# ROS-TCP-Endpoint 설치 — Unity ↔ ROS 연동의 ROS 쪽 짝 (2026-08-16 신설)
#
# `src/vendor/` 는 .gitignore 대상이라 저장소에 안 들어간다. 다른 PC 에서는 이 스크립트를
# 먼저 돌려야 Unity 연동이 된다(모델 벤더의 `setup_vendor_models.sh` 와 같은 성격).
#
# 🔴 왜 패치가 필요한가 — upstream 0.7.0(ROS2) 버그
#   endpoint 의 `handle_syscommand` 가 `data.decode("utf-8")[:-1]` 로 **마지막 글자를 무조건**
#   잘라낸다(널 종단자 가정). 그런데 ROS-TCP-Connector **v0.7.0 은 널을 붙이지 않아**
#   JSON 의 닫는 '}' 가 잘려 나가고 `JSONDecodeError` 로 구독이 통째로 실패한다.
#   증상은 Unity 쪽에서 "Connection reset by peer" 반복 + 수신 0건이라 원인이 안 보인다.
#   (양쪽 다 0.7.0 = 버전을 맞춰도 안 된다. 실측으로 확인했다.)
#   → 널/공백일 때만 잘라 두 형식을 다 받게 고친다.
set -e

WS="${1:-$HOME/robot_ws}"
VENDOR="$WS/src/vendor"
DIR="$VENDOR/ros_tcp_endpoint"

mkdir -p "$VENDOR"
if [ ! -d "$DIR" ]; then
  echo "[setup] clone ROS-TCP-Endpoint (main-ros2, Apache-2.0)"
  git clone --depth 1 -b main-ros2 \
    https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git "$DIR"
else
  echo "[setup] 이미 있음: $DIR"
fi

SRV="$DIR/ros_tcp_endpoint/server.py"
if grep -q 'data.decode("utf-8")\[:-1\]' "$SRV"; then
  echo "[setup] upstream 버그 패치 적용"
  python3 - "$SRV" <<'PY'
import io, sys
p = sys.argv[1]
s = io.open(p, encoding="utf-8").read()
old = '            message_json = data.decode("utf-8")[:-1]\n'
new = ('            # 🔴 upstream 0.7.0(ROS2) 버그 패치 — 커넥터 v0.7.0 은 널 종단자를 안 붙이는데\n'
       '            #    원본은 무조건 마지막 글자를 잘라 닫는 \'}\' 가 날아간다(JSONDecodeError →\n'
       '            #    구독 실패 → Unity 쪽엔 "Connection reset by peer" 로만 보인다).\n'
       '            #    널/공백일 때만 잘라 양쪽 형식을 다 받는다.  docs/scripts/setup_ros_tcp_endpoint.sh\n'
       '            message_json = data.decode("utf-8").rstrip("\\x00").rstrip()\n')
assert s.count(old) == 1, f"패치 지점 {s.count(old)}개 — upstream 이 바뀌었다"
io.open(p, "w", encoding="utf-8").write(s.replace(old, new))
print("[setup]   server.py 패치 완료")
PY
else
  echo "[setup] 패치 이미 적용됨(또는 upstream 변경) — 건너뜀"
fi

echo "[setup] 빌드"
cd "$WS"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# ⚠ 벤더는 --symlink-install 을 섞지 않는다(캐시가 깨져 이후 빌드까지 실패한다 — 기록된 함정).
colcon build --packages-select ros_tcp_endpoint

echo "[setup] 완료. 실행:"
echo "  ros2 run ros_tcp_endpoint default_server_endpoint \\"
echo "    --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000"
