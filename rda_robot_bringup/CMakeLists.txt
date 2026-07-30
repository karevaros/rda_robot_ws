cmake_minimum_required(VERSION 3.8)
project(rda_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 제어박스 안전 상태 발행 노드(C++) — 레인보우 SDK 가 있을 때만 빌드한다.
#   SDK 는 apt 에 없다(bash src/docs/scripts/setup_rbpodo_sdk.sh 로 설치).
#   ⚠ REQUIRED 로 두면 SDK 없는 PC 에서 이 패키지 전체가 실패해 파이썬 노드까지 못 쓴다.
#      그래서 QUIET 로 찾고, 없으면 건너뛰되 **빌드 로그에 이유를 남긴다**(조용한 누락 방지).
find_package(rbpodo QUIET)
if(rbpodo_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(rda_robot_msgs REQUIRED)
  add_executable(cb_safety_publisher src/cb_safety_publisher.cpp)
  target_compile_features(cb_safety_publisher PUBLIC cxx_std_17)
  ament_target_dependencies(cb_safety_publisher rclcpp rda_robot_msgs)
  target_link_libraries(cb_safety_publisher rbpodo::rbpodo)
  install(TARGETS cb_safety_publisher DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "rbpodo SDK 를 찾지 못해 cb_safety_publisher 를 건너뜁니다. "
    "실기 안전 감시가 필요하면 bash src/docs/scripts/setup_rbpodo_sdk.sh 를 실행하세요.")
endif()

# 파이썬 노드 설치
# ※ symlink-install 시 소스 파일에 chmod +x 가 안 돼 있으면 ros2 run 이 못 찾음
install(PROGRAMS
  scripts/self_collision_monitor.py
  scripts/obstacle_publisher.py
  scripts/crop_tuner.py
  scripts/pregrasp_pose.py
  scripts/pregrasp_demo.py
  scripts/robot_introspect.py
  scripts/gen_gazebo_world.py
  scripts/fruit_detector.py
  scripts/harvest_operator.py
  scripts/gripper_adapter.py
  scripts/safety_monitor.py
  scripts/harvest_panel.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
