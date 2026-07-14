"""통합 로봇(rda_robot) RViz2 표시 launch.

config/mounts.yaml(조립기 앱이 저장) 을 읽어 통합 URDF 를 만들고
robot_state_publisher + joint_state_publisher_gui + rviz2 로 표시한다.
mounts.yaml 의 initial_pose(관절 초기 포즈)는 joint_state_publisher 의
zeros 파라미터로 전달되어 시작 포즈가 그 값이 된다.

기본 mounts_file 은 소스 워크스페이스의 config/mounts.yaml → 앱 저장 후
colcon 재빌드 없이 반영. 다른 파일:  mounts_file:=/경로/mounts.yaml
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    pkg = FindPackageShare("rda_robot_description")
    xacro_path = PathJoinSubstitution([pkg, "urdf", "rda_robot.urdf.xacro"])
    rviz_path = PathJoinSubstitution([pkg, "rviz", "rda_robot.rviz"])
    mounts_file = LaunchConfiguration("mounts_file").perform(context)

    # initial_pose 읽기(있으면 jsp zeros 로)
    zeros = {}
    try:
        with open(mounts_file) as f:
            d = yaml.safe_load(f) or {}
        zeros = {k: float(v) for k, v in (d.get("initial_pose") or {}).items()}
    except Exception:
        pass

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path,
                 " mounts_file:=", mounts_file]),
        value_type=str,
    )

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[{"robot_description": robot_description}])
    # robot_description 은 rsp 가 /robot_description 토픽(latched)으로 발행 → jsp_gui 가 구독.
    # (주의) robot_description 을 CLI '-p' 로 넘기면 XML 개행/특수문자로 파싱 실패하니 금지.
    jsp = Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
               parameters=[{"zeros": zeros}] if zeros else [])
    rviz = Node(package="rviz2", executable="rviz2", output="screen",
                arguments=["-d", rviz_path])
    nodes = [rsp, jsp, rviz]

    # 자충돌 모니터(기본 RViz 에서 움직임 시 충돌 감지) — collision:=false 로 끔.
    if LaunchConfiguration("collision").perform(context).lower() in ("1", "true", "yes"):
        nodes.append(Node(package="rda_robot_bringup",
                          executable="self_collision_monitor.py",
                          output="screen"))
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
    return LaunchDescription([mounts_arg, collision_arg,
                              OpaqueFunction(function=_launch_setup)])
