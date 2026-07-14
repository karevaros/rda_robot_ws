"""통합 로봇(rda_robot) RViz2 표시 launch.

config/mounts.yaml(조립기 앱이 저장) 을 읽어 통합 URDF 를 만들고
robot_state_publisher + joint_state_publisher_gui + rviz2 로 표시한다.

기본 mounts_file 은 **소스 워크스페이스**의 config/mounts.yaml 을 가리켜,
앱에서 저장 후 colcon 재빌드 없이 바로 반영된다.
다른 파일을 보려면:  mounts_file:=/경로/mounts.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = FindPackageShare("rda_robot_description")
    xacro_path = PathJoinSubstitution([pkg, "urdf", "rda_robot.urdf.xacro"])
    rviz_path = PathJoinSubstitution([pkg, "rviz", "rda_robot.rviz"])

    # 기본: 소스 워크스페이스의 mounts.yaml (앱 저장 → 재빌드 없이 반영)
    default_mounts = os.path.expanduser(
        "~/robot_ws/src/rda_robot_description/config/mounts.yaml")
    if not os.path.exists(default_mounts):
        # 소스가 없으면 설치본 사용
        default_mounts = ""  # 빈 값이면 xacro 기본(installed) 사용

    mounts_arg = DeclareLaunchArgument(
        "mounts_file", default_value=default_mounts,
        description="결합 설정 yaml 경로(비우면 설치본 config/mounts.yaml 사용)")
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true", description="joint_state_publisher_gui 사용")

    mounts_file = LaunchConfiguration("mounts_file")

    # xacro 명령: mounts_file 이 있으면 인자로 전달
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_path,
            " mounts_file:=", mounts_file,
        ]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[{"robot_description": robot_description}],
    )
    jsp_gui = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
        condition=None,
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", rviz_path],
    )

    return LaunchDescription([mounts_arg, gui_arg, rsp, jsp_gui, rviz])
