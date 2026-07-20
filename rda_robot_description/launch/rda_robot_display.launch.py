"""통합 로봇(rda_robot) RViz2 표시 launch.

config/mounts.yaml(조립기 앱이 저장) 을 읽어 통합 URDF 를 만들고
robot_state_publisher + joint_state_publisher_gui + rviz2 로 표시한다.
mounts.yaml 의 initial_pose(관절 초기 포즈)는 joint_state_publisher 의
zeros 파라미터로 전달되어 시작 포즈가 그 값이 된다.

통합 URDF 는 **Python 컴포저**(`ros2 run rda_robot_assembler compose_urdf`)가 만든다.
조립기와 같은 모델 정의를 읽으므로 앱 화면과 RViz 형상이 일치한다.
(이전에는 rda_robot.urdf.xacro 를 썼는데, 슬롯별 xacro:if 분기가 있는 모델만 반영돼
 나머지 21종이 조용히 누락됐고, 벤더 tcp 오프셋 96.7mm 도 빠져 앱과 형상이 달랐다.)

기본 mounts_file 은 소스 워크스페이스의 config/mounts.yaml → 앱 저장 후
colcon 재빌드 없이 반영. 다른 파일:  mounts_file:=/경로/mounts.yaml
"""
import os
import subprocess
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def compose_urdf(mounts_file):
    """mounts.yaml → 통합 URDF XML. 실패하면 이유를 그대로 보이고 중단한다.

    subprocess(ros2 run)로 부르는 이유: assembler 가 이미 이 패키지(description)를
    exec_depend 로 참조하므로, 반대 방향 의존을 선언하면 순환이 된다.
    """
    try:
        return subprocess.check_output(
            ["ros2", "run", "rda_robot_assembler", "compose_urdf",
             "--mounts", mounts_file],
            text=True, stderr=subprocess.PIPE, timeout=180)
    except subprocess.CalledProcessError as e:
        raise RuntimeError("통합 URDF 조립 실패:\n" + (e.stderr or "").strip())
    except FileNotFoundError:
        raise RuntimeError("ros2 실행파일이 없습니다. 환경을 source 했는지 확인하세요.")


def _ground_offset(urdf_xml):
    """world -> base_link 의 z 오프셋을 URDF 에서 유도한다.

    base_link 는 바닥이 아니다. Scout 는 base_footprint_joint 로
    base_link -> base_footprint 를 z=-0.23479 에 두므로(바퀴 최저점 -0.2352),
    base_link 를 world 원점에 그냥 붙이면 로봇이 바닥에 파묻힌다.

    베이스 모델을 바꾸면 이 값도 달라지므로 상수로 박지 않고 URDF 에서 읽는다.
    base_footprint 가 없는 모델(예: box_base)이면 0 을 쓰되 **경고**한다
    (조용히 0 으로 떨어뜨리면 로봇이 파묻혀도 눈치채기 어렵다).
    """
    try:
        for j in ET.fromstring(urdf_xml).findall("joint"):
            if j.find("child").get("link") == "base_footprint":
                o = j.find("origin")
                z = float((o.get("xyz") or "0 0 0").split()[2])
                return -z, None       # base_footprint 를 z=0 으로
        return 0.0, ("base_footprint 링크가 없어 world->base_link z=0 을 씁니다. "
                     "베이스가 바닥에 파묻히거나 떠 보이면 obstacles.yaml 의 "
                     "ground_plane 높이 또는 이 오프셋을 확인하세요.")
    except Exception as e:
        return 0.0, f"URDF 에서 바닥 오프셋을 못 읽어 z=0 을 씁니다 ({e})."


def _launch_setup(context, *args, **kwargs):
    pkg = FindPackageShare("rda_robot_description")
    rviz_path = PathJoinSubstitution([pkg, "rviz", "rda_robot.rviz"])
    mounts_file = LaunchConfiguration("mounts_file").perform(context)

    # initial_pose · base_placement 읽기(있으면 jsp zeros / 로봇 배치로)
    zeros = {}
    base_place = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
    try:
        with open(mounts_file) as f:
            d = yaml.safe_load(f) or {}
        zeros = {k: float(v) for k, v in (d.get("initial_pose") or {}).items()}
        bp = d.get("base_placement") or {}
        for k in base_place:
            if k in bp:
                base_place[k] = float(bp[k])
    except Exception:
        pass

    # 통합 URDF 는 한 번만 조립해 robot_description 과 바닥 오프셋에 함께 쓴다.
    urdf_xml = compose_urdf(mounts_file)
    robot_description = ParameterValue(urdf_xml, value_type=str)

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[{"robot_description": robot_description}])
    # robot_description 은 rsp 가 /robot_description 토픽(latched)으로 발행 → jsp_gui 가 구독.
    # (주의) robot_description 을 CLI '-p' 로 넘기면 XML 개행/특수문자로 파싱 실패하니 금지.
    jsp = Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
               parameters=[{"zeros": zeros}] if zeros else [])
    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", rviz_path])
    # 좌표계 통일: world 를 항상 발행한다(RViz Fixed Frame 이 world 이므로
    # obstacles:=false 여도 있어야 함). 로봇 루트(base_link)를 world 에 고정.
    # 모바일 베이스가 실제 주행하면 이 static TF 를 odom/localization 이 대체해야 함.
    z, warn = _ground_offset(urdf_xml)
    if warn:
        print(f"[rda_robot_display] ⚠ {warn}")
    else:
        print(f"[rda_robot_display] world->base_link z={z:.5f} "
              f"(base_footprint 를 바닥 z=0 에 맞춤)")
    # 배경 기준 로봇 배치: 어셈블러가 mounts.yaml 에 저장한 base_placement 를
    #  world→base_link 에 반영한다(x/y/yaw + z 는 바닥오프셋에 더함). 그러면
    #  고정된 온실(obstacles world 좌표) 안에서 로봇이 저장한 위치로 놓인다.
    #  base_yaw 인자를 명시(기본 'auto' 아님)하면 저장된 yaw 를 덮어쓴다.
    import math as _math
    bx = base_place["x"]
    by = base_place["y"]
    bz = z + base_place["z"]
    yaw_arg = LaunchConfiguration("base_yaw").perform(context).strip().lower()
    if yaw_arg in ("", "auto"):
        yaw = base_place["yaw_deg"] * _math.pi / 180.0
    else:
        yaw = float(yaw_arg)
    world_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    name="world_to_base_link",
                    arguments=["--x", f"{bx:.6f}", "--y", f"{by:.6f}",
                               "--z", f"{bz:.6f}", "--yaw", f"{yaw:.6f}",
                               "--frame-id", "world",
                               "--child-frame-id", "base_link"])
    nodes = [rsp, jsp, rviz, world_tf]

    # 자충돌 모니터(기본 RViz 에서 움직임 시 충돌 감지) — collision:=false 로 끔.
    if LaunchConfiguration("collision").perform(context).lower() in ("1", "true", "yes"):
        nodes.append(Node(package="rda_robot_bringup",
                          executable="self_collision_monitor.py",
                          output="screen"))

    # 장애물 환경(4주차) — obstacles:=false 로 끔.
    if LaunchConfiguration("obstacles").perform(context).lower() in ("1", "true", "yes"):
        nodes.append(Node(package="rda_robot_bringup",
                          executable="obstacle_publisher.py",
                          output="screen",
                          parameters=[{"obstacles_file":
                                       LaunchConfiguration("obstacles_file").perform(context)}]))
    return nodes


def generate_launch_description():
    default_mounts = os.path.expanduser(
        "~/robot_ws/src/rda_robot_description/config/mounts.yaml")
    mounts_arg = DeclareLaunchArgument(
        "mounts_file", default_value=default_mounts,
        description="결합/초기포즈 yaml 경로")
    collision_arg = DeclareLaunchArgument(
        "collision", default_value="true",
        description="자충돌 모니터 실행 여부(RViz 빨강 마커 표시)")
    default_obstacles = os.path.expanduser(
        "~/robot_ws/src/rda_robot_description/config/obstacles.yaml")
    obstacles_arg = DeclareLaunchArgument(
        "obstacles", default_value="true",
        description="장애물 환경 표시 여부(world TF + 장애물 마커)")
    obstacles_file_arg = DeclareLaunchArgument(
        "obstacles_file", default_value=default_obstacles,
        description="장애물 정의 yaml 경로")
    base_yaw_arg = DeclareLaunchArgument(
        "base_yaw", default_value="auto",
        description="world→base_link yaw[rad]. 'auto'=mounts.yaml 의 base_placement 값 사용. "
                    "숫자를 주면 그 값으로 덮어씀(예: -1.5708=시계 90°).")
    return LaunchDescription([mounts_arg, collision_arg, obstacles_arg,
                              obstacles_file_arg, base_yaw_arg,
                              OpaqueFunction(function=_launch_setup)])
