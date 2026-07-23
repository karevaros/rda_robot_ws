#!/bin/bash
# 모델 라이브러리용 벤더 레포 clone + description 패키지 빌드.
#
# 왜 필요한가: config/models/<슬롯>/*.yaml 이 `pkg:` 로 참조하는 description
# 패키지들은 vendor/ 에 clone 되는데 vendor/ 는 .gitignore 다. 새 PC 에서 이
# 스크립트를 돌려야 조립기 드롭다운의 모델들이 실제로 로드된다.
#
# 사용: bash docs/scripts/setup_vendor_models.sh
set -u
WS=~/robot_ws
V=$WS/src/vendor
mkdir -p "$V" && cd "$V" || exit 1

clone() {  # clone <url> <branch> <dir>
  if [ -d "$3" ]; then echo "SKIP  $3"; return; fi
  if git clone -q --depth 1 -b "$2" "$1" "$3" 2>/dev/null; then echo "OK    $3 ($2)"
  else echo "FAIL  $3  <-- $1 @ $2"; fi
}

echo "== clone =="
# ── 기본 로봇(기본 조합에 반드시 필요) ────────────────────────────────────
#    이 3개가 없으면 통합 URDF 조립 자체가 실패한다(기본 = Scout 2.0 + RB5-850e + D405/D435i).
clone https://github.com/agilexrobotics/scout_ros2.git humble scout_ros2                                    # Apache-2.0 (scout_description)
clone https://github.com/RainbowRobotics/rbpodo_ros2.git main rbpodo_ros2                                   # Apache-2.0 (rbpodo_description)
clone https://github.com/IntelRealSense/realsense-ros.git ros2-master realsense-ros                         # Apache-2.0 (realsense2_description)
# 팔
clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git humble ur_description_ros2   # BSD-3
clone https://github.com/xArm-Developer/xarm_ros2.git humble xarm_ros2                                      # BSD-3
# 엔드이펙터
clone https://github.com/ABC-iRobotics/onrobot-ros2.git main onrobot_ros2                                   # MIT
clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git humble ros2_robotiq_gripper               # BSD-3
clone https://github.com/frankaemika/franka_description.git humble franka_description                       # Apache-2.0
clone https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5.git master-4finger allegro_hand_ros2_v5     # BSD-2
# 베이스
clone https://github.com/clearpathrobotics/clearpath_common.git humble clearpath_common                     # BSD-3
clone https://github.com/RobotnikAutomation/robotnik_description.git humble-devel robotnik_description      # BSD-3
clone https://github.com/RobotnikAutomation/robotnik_sensors.git humble-devel robotnik_sensors              # BSD-3
clone https://github.com/turtlebot/turtlebot4.git humble turtlebot4                                         # Apache-2.0
clone https://github.com/iRobotEducation/create3_sim.git humble create3_sim                                 # BSD-3 (turtlebot4 의존)

# ── Allegro description 셰임 ──────────────────────────────────────────────
# 상류엔 description 패키지가 없고 URDF·mesh 가 드라이버 패키지
# (allegro_hand_controllers, allegro_hand_driver·bhand 의존) 안에 있다.
# 표시만 하려고 드라이버 스택을 빌드할 이유가 없어 mesh 를 심링크로 재노출한다.
if [ -d allegro_hand_ros2_v5 ] && [ ! -d allegro_hand_description ]; then
  mkdir -p allegro_hand_description/urdf
  ln -sfn ../allegro_hand_ros2_v5/src/allegro_hand_controllers/meshes allegro_hand_description/meshes
  sed 's|package://allegro_hand_controllers/|package://allegro_hand_description/|g' \
    allegro_hand_ros2_v5/src/allegro_hand_controllers/urdf/allegro_hand_description_right_A.urdf \
    > allegro_hand_description/urdf/allegro_hand_right.urdf
  cat > allegro_hand_description/package.xml <<'XML'
<?xml version="1.0"?>
<package format="3">
  <name>allegro_hand_description</name>
  <version>0.0.1</version>
  <description>Allegro Hand V5 표시용 description 셰임(상류엔 description 패키지가 없음).</description>
  <maintainer email="akswnddl255@gmail.com">kim</maintainer>
  <license>BSD-2-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
XML
  cat > allegro_hand_description/CMakeLists.txt <<'CMK'
cmake_minimum_required(VERSION 3.8)
project(allegro_hand_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
ament_package()
CMK
  echo "OK    allegro_hand_description (셰임 생성)"
fi

echo "== build (description 패키지만) =="
cd "$WS" || exit 1
# shellcheck disable=SC1091
# ⚠ set -u 상태로 ROS setup.bash 를 소싱하면 'AMENT_TRACE_SETUP_FILES: unbound variable'
#   로 스크립트가 여기서 죽는다(= clone 만 되고 빌드가 안 돼 모델 로드가 실패). 잠시 끈다.
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --packages-select \
  scout_description rbpodo_description realsense2_description \
  ur_description xarm_description onrobot_rg_description robotiq_description \
  franka_description allegro_hand_description \
  robotnik_description robotnik_sensors \
  clearpath_platform_description clearpath_control \
  irobot_create_description irobot_create_control turtlebot4_description \
  2>&1 | tail -3

echo
echo "완료. 확인: python3 src/docs/scripts/test_models.py"
echo "  (조립기에서 F5 '모델 새로고침' 하면 드롭다운에 반영됨)"

# ── 등록하지 않은 것 (이유) ───────────────────────────────────────────────
# · MiR100 (DFKI-NI/mir_robot): humble 브랜치인데도 mir_description 이 catkin
#   (package format=2, find_package(catkin)) → ROS2 에서 빌드 불가. ROS1 그대로다.
# · AgileX Tracer/Scout Mini/Bunker: URDF·mesh 가 ugv_gazebo_sim 에만 있는데
#   그 레포엔 LICENSE 파일이 없다(= All rights reserved). scout_ros2 안에
#   description 이 있는 건 Scout 2.0 뿐(현재 사용 중).
# · AgileX Ranger Mini: 어느 레포에도 URDF/mesh 가 없다(빈 폴더).
# · OnRobot RG2 (ABC-iRobotics): 상류 버그 — 매크로가 존재하지 않는 링크
#   onrobot_rg2_base_link 를 parent 로 참조해 check_urdf 트리 생성 실패.
#   RG2 는 rda_robot_description 에 편입된 기존 모델을 쓴다.
# · Summit-XL (summit_xl_common): xacro:sensor_robosense_helios 매크로 없음
#   (robotnik_sensors 와 버전 불일치). robotnik_description 의 rbsummit 으로 대체.
# · UR20/UR30: 코드는 BSD-3 지만 mesh 만 별도 제한 라이선스(UR Graphical
#   Documentation T&C) → 등록하지 않음.
