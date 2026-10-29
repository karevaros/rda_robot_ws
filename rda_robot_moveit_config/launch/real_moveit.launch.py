"""실기(Rainbow RB5) + MoveIt — 계획한 궤적을 실제 컨트롤러로 실행한다.

    ros2 launch rda_robot_moveit_config real_moveit.launch.py robot_ip:=192.168.35.100 \
        cb_simulation:=true          # 🔴 false 면 실제 팔이 움직인다

구성 = real_robot.launch.py(ros2_control·안전 감시·그리퍼 어댑터) + move_group(execute 켬).
real_robot 의 인자(robot_ip·cb_simulation·gripper·speed_bar·max_joint_velocity …)는 그대로 넘긴다.

■ 시뮬용 perception_demo(execute:=true)와 다른 점
  · use_sim_time=false (실시간). Gazebo 가 없다.
  · 장면에 온실 장애물을 넣지 않는다 — 실제 주변은 모델과 다르다. **충돌 회피는 자기충돌뿐**이므로
    실기에서는 짧은 이동만 하고, 계획 결과를 실행 전에 검사할 것(docs/scripts/real_moveit_smoke.py).
  · 속도·가속 배율은 joint_limits.yaml 기본(0.1). 하드웨어 상한(max_joint_velocity)이 그 아래를 한 번 더 막는다.
"""
import os
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: real_robot.launch.py 에 그대로 넘기는 인자(기본값은 그쪽 것을 따른다 — 빈 값이면 안 넘김)
PASS = ("robot_ip", "cb_simulation", "use_fake_hardware", "mounts", "gripper", "gripper_dry_run",
        "safety", "servo_t2", "speed_bar", "max_joint_velocity", "script_grace", "preflight")


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _setup(context, *args, **kwargs):
    cfg = get_package_share_directory("rda_robot_moveit_config")
    desc = get_package_share_directory("rda_robot_description")
    lc = lambda n: LaunchConfiguration(n).perform(context)

    mounts = lc("mounts") or os.path.join(desc, "config", "mounts.yaml")
    # move_group 용 URDF — 기하만 쓰므로 하드웨어 인자 없이 조립해도 된다(링크·관절 동일).
    env = dict(os.environ)
    us = os.path.join(os.path.expanduser("~"), ".local", "lib",
                      f"python{sys.version_info.major}.{sys.version_info.minor}", "site-packages")
    if os.path.isdir(us) and us not in env.get("PYTHONPATH", "").split(os.pathsep):
        env["PYTHONPATH"] = us + os.pathsep + env.get("PYTHONPATH", "")
    urdf_xml = subprocess.check_output(
        ["ros2", "run", "rda_robot_assembler", "compose_urdf", "--mounts", mounts],
        text=True, stderr=subprocess.PIPE, timeout=180, env=env)
    with open(os.path.join(cfg, "config", "rda_robot.srdf")) as f:
        srdf_xml = f.read()

    params = [
        {"robot_description": urdf_xml},
        {"robot_description_semantic": srdf_xml},
        {"robot_description_kinematics": _load_yaml(os.path.join(cfg, "config", "kinematics.yaml"))},
        {"robot_description_planning": _load_yaml(os.path.join(cfg, "config", "joint_limits.yaml"))},
        {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl",
         "ompl": _load_yaml(os.path.join(cfg, "config", "ompl_planning.yaml"))},
        {"allow_trajectory_execution": True, "publish_planning_scene": True,
         "publish_geometry_updates": True, "publish_state_updates": True,
         "publish_transforms_updates": True},
        _load_yaml(os.path.join(cfg, "config", "moveit_controllers.yaml")),
        {"use_sim_time": False},
    ]
    move_group = Node(package="moveit_ros_move_group", executable="move_group",
                      output="screen", parameters=params)

    real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc, "launch", "real_robot.launch.py")),
        launch_arguments={k: lc(k) for k in PASS if lc(k)}.items())

    nodes = [real, move_group]
    if lc("rviz").lower() in ("true", "1"):
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          arguments=["-d", os.path.join(cfg, "config", "moveit.rviz")],
                          parameters=params[:4]))
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value="") for k in PASS]
        + [DeclareLaunchArgument("rviz", default_value="false"),
           OpaqueFunction(function=_setup)])
