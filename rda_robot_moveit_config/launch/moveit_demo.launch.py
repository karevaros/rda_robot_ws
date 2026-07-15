"""통합 로봇 MoveIt2 데모 — 장애물 planning scene + 경로계획 (4주차).

구성: robot_state_publisher + joint_state_publisher_gui + move_group + rviz2(MotionPlanning)
      + world static TF + 장애물 퍼블리셔(obstacles.yaml → CollisionObject).

RViz 의 MotionPlanning 패널에서 목표 자세를 끌어다 Plan 하면
장애물(table/pillar/ground_plane)을 피하는 경로가 나온다.

⚠ 실행(execute)은 안 된다 — 컨트롤러가 아직 없다(6주차 통합제어에서 붙임).
   allow_trajectory_execution=false 로 명시해 두었다. Plan 까지가 이번 주 범위.

사용:
  ros2 launch rda_robot_moveit_config moveit_demo.launch.py
  ros2 launch rda_robot_moveit_config moveit_demo.launch.py obstacles:=false
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DESC_SRC = os.path.expanduser("~/robot_ws/src/rda_robot_description")


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _ground_offset(urdf_xml):
    """world -> base_link z. base_footprint 를 바닥 z=0 에 맞춘다.

    base_link 는 바닥이 아니다(Scout: base_footprint 가 0.23479 m 아래).
    상수로 박으면 베이스 모델 교체 시 조용히 깨지므로 URDF 에서 읽는다.
    """
    import xml.etree.ElementTree as ET
    try:
        for j in ET.fromstring(urdf_xml).findall("joint"):
            if j.find("child").get("link") == "base_footprint":
                return -float((j.find("origin").get("xyz")).split()[2]), None
        return 0.0, "base_footprint 가 없어 world->base_link z=0 을 씁니다."
    except Exception as e:
        return 0.0, f"바닥 오프셋 파싱 실패 → z=0 ({e})."


def _setup(context, *args, **kwargs):
    import subprocess

    cfg = get_package_share_directory("rda_robot_moveit_config")
    mounts = LaunchConfiguration("mounts_file").perform(context)
    xacro_file = os.path.join(DESC_SRC, "urdf", "rda_robot.urdf.xacro")

    urdf_xml = subprocess.check_output(
        ["xacro", xacro_file, f"mounts_file:={mounts}"], text=True)
    with open(os.path.join(cfg, "config", "rda_robot.srdf")) as f:
        srdf_xml = f.read()

    robot_description = {"robot_description": urdf_xml}
    robot_description_semantic = {"robot_description_semantic": srdf_xml}
    kinematics = {"robot_description_kinematics":
                  _load_yaml(os.path.join(cfg, "config", "kinematics.yaml"))}
    jl = _load_yaml(os.path.join(cfg, "config", "joint_limits.yaml"))
    planning_limits = {"robot_description_planning": jl}

    ompl = _load_yaml(os.path.join(cfg, "config", "ompl_planning.yaml"))
    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl,
    }
    # 컨트롤러가 없으므로 실행은 끈다(6주차에 붙임). 켜두면 move_group 이
    # 컨트롤러를 찾다 실패해 원인을 오해하기 쉽다.
    execution = {
        "allow_trajectory_execution": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics,
                    planning_limits, planning_pipeline, execution],
    )
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[robot_description])
    zeros = {}
    try:
        d = _load_yaml(mounts) or {}
        zeros = {k: float(v) for k, v in (d.get("initial_pose") or {}).items()}
    except Exception as e:
        print(f"[moveit_demo] ⚠ initial_pose 를 못 읽었습니다({e}) — jsp 기본값 사용.")
    jsp = Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
               parameters=[{"zeros": zeros}] if zeros else [])
    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", os.path.join(cfg, "config", "moveit.rviz")],
                parameters=[robot_description, robot_description_semantic,
                            kinematics, planning_limits])

    z, warn = _ground_offset(urdf_xml)
    if warn:
        print(f"[moveit_demo] ⚠ {warn}")
    else:
        print(f"[moveit_demo] world->base_link z={z:.5f}")
    world_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    name="world_to_base_link",
                    arguments=["--x", "0", "--y", "0", "--z", f"{z:.6f}",
                               "--frame-id", "world", "--child-frame-id", "base_link"])

    nodes = [rsp, jsp, move_group, rviz, world_tf]
    if LaunchConfiguration("obstacles").perform(context).lower() in ("1", "true", "yes"):
        nodes.append(Node(package="rda_robot_bringup", executable="obstacle_publisher.py",
                          output="screen",
                          parameters=[{"obstacles_file": os.path.join(
                              DESC_SRC, "config", "obstacles.yaml")}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mounts_file",
            default_value=os.path.join(DESC_SRC, "config", "mounts.yaml"),
            description="결합/초기포즈 yaml"),
        DeclareLaunchArgument(
            "obstacles", default_value="true",
            description="장애물을 planning scene 에 넣을지"),
        OpaqueFunction(function=_setup),
    ])
