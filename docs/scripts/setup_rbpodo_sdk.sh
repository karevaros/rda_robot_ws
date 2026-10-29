#!/usr/bin/env bash
# 레인보우 로봇 C++ SDK(rbpodo) 설치 + rbpodo_hardware 빌드 — 실기 연동 선행 조건
#
# 왜 필요한가
#   vendor/rbpodo_ros2/rbpodo_hardware 는 `find_package(rbpodo REQUIRED)` 를 요구하는데
#   이 SDK 는 apt 에 없다. 없으면 rbpodo_hardware 가 **빌드에서 조용히 빠진다**
#   (colcon 은 그 패키지만 실패시키고 나머지는 성공으로 끝난다) → 실기 모드로 띄우면
#   "plugin rbpodo_hardware/RBPodoHardwareInterface not found" 로 뒤늦게 드러난다.
#
# 사용법
#   bash src/docs/scripts/setup_rbpodo_sdk.sh          # 로컬 설치(sudo 불필요, 기본)
#   SYSTEM_INSTALL=1 bash src/docs/scripts/setup_rbpodo_sdk.sh   # /usr/local 에 설치(sudo)
#
# ⚠ 로컬 설치를 쓰면 rbpodo_hardware 를 **다시 빌드할 때마다** CMAKE_PREFIX_PATH 가 필요하다.
#   이 스크립트가 그 빌드까지 대신 해 준다 — 직접 colcon 을 돌릴 때는 아래 명령을 그대로 쓸 것.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"   # …/robot_ws
SRC="$WS/thirdparty/rbpodo"
PREFIX="$WS/thirdparty/install"

echo "[1/3] SDK 소스 준비 ($SRC)"
mkdir -p "$WS/thirdparty"
if [ -d "$SRC/.git" ]; then
  echo "  이미 있음 — 건너뜀"
else
  git clone --depth 1 https://github.com/RainbowRobotics/rbpodo.git "$SRC"
fi

echo "[2/3] SDK 빌드·설치"
mkdir -p "$SRC/build"
cd "$SRC/build"
if [ "${SYSTEM_INSTALL:-0}" = "1" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make -j"$(nproc)"
  sudo make install
  EXTRA_PREFIX=""
else
  cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" ..
  make -j"$(nproc)"
  make install
  EXTRA_PREFIX="$PREFIX:"
fi

echo "[3/3] rbpodo_hardware 패치 + rbpodo_msgs·rbpodo_hardware 빌드"
# 벤더는 gitignore 라 수정은 패치로만 남는다. 패치는 **순서대로 누적**된다(같은 자리를 이어서 고친다)
# → 개별 '이미 적용됨' 판정이 성립하지 않는다(뒤 패치가 앞 패치의 문맥을 바꾼다).
#   ⇒ 깨끗하면 전부 적용 · 마지막 패치까지 적용돼 있으면 건너뜀 · 그 밖이면 멈추고 안내.
#   01 servo_t2       — servo_j 유지시간(무선 링크에서 0.03s 는 끊긴다, 2026-10-29 실측)
#   02 speed_limit    — speed_bar 설정 + 관절 속도 상한 max_joint_velocity(속도바는 servo_j 에 안 먹는다)
VENDOR="$WS/src/vendor/rbpodo_ros2"
PATCHES=("$WS/src/docs/scripts/patches/"[0-9][0-9]-rbpodo_hardware_*.patch)
if git -C "$VENDOR" diff --quiet -- rbpodo_hardware; then
  for P in "${PATCHES[@]}"; do git -C "$VENDOR" apply "$P"; echo "  패치 적용: $(basename "$P")"; done
elif git -C "$VENDOR" apply --reverse --check "${PATCHES[-1]}" 2>/dev/null; then
  echo "  패치 이미 적용됨(마지막: $(basename "${PATCHES[-1]}"))"
else
  echo "  🔴 vendor/rbpodo_ros2/rbpodo_hardware 에 패치와 다른 변경이 있습니다." >&2
  echo "     확인 후 되돌리고 다시 실행하세요: git -C $VENDOR checkout -- rbpodo_hardware" >&2
  exit 1
fi
# ⚠ 벤더 패키지에 --symlink-install 을 섞지 말 것(CLAUDE.md 빌드 함정)
cd "$WS"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
CMAKE_PREFIX_PATH="${EXTRA_PREFIX}${CMAKE_PREFIX_PATH:-}" \
  colcon build --packages-select rbpodo_msgs rbpodo_hardware \
               --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "완료. 검증:"
echo "  source $WS/install/setup.bash"
echo "  ros2 launch rda_robot_description real_robot.launch.py use_fake_hardware:=true"
