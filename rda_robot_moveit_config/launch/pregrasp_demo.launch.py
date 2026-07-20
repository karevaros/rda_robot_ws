"""5주차 pre-grasp 집기 데모 launch — 알고리즘이 이끄는 로봇 동작을 RViz 에서 재생.

구성: robot_state_publisher + move_group + obstacle_publisher + world→base_link(재배치)
      + rviz2 + pregrasp_demo(노드). ⚠ jsp_gui 는 넣지 않는다 — 데모 노드가 유일한
      /joint_states 발행자다(둘이 같이 발행하면 자세가 튄다).

사용:
  ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py
  # 특정 목표점(월드 좌표)으로 — 기본값도 이 방식(도달권 내 예시 토마토):
  ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py target:="[0.5,-0.4,0.9]"
  ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py rviz:=false     # 헤드리스

  # (실험) 온실 실열매 자동선택 + base 재배치:
  #   ros2 launch ... use_yaml_target:=true target_index:=0 base_x:=0.83 base_y:=-0.85
  #   ⚠ 이 팔(reach 0.93m)로 고설 토마토(z 1.2~1.7m)는 도달권이 marginal 하다.
  #     base_x/base_y 를 열매 바로 아래로 미세조정해야 겨우 닿고, 안 닿으면 '자세 없음'.
  #     기본 데모가 도달권 내 예시 목표를 쓰는 이유(도달권 상세=_작업기록.md).

⚠ execute(실제 컨트롤러)는 6주차. 여기서는 계획된 궤적을 /joint_states 로 재생만 한다.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DESC_SRC = os.path.expanduser("~/robot_ws/src/rda_robot_description")


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _ground_offset(urdf_xml):
    import xml.etree.ElementTree as ET
    try:
        for j in ET.fromstring(urdf_xml).findall("joint"):
            if j.find("child").get("link") == "base_footprint":
                return -float((j.find("origin").get("xyz")).split()[2])
        return 0.0
    except Exception:
        return 0.0


def _setup(context, *args, **kwargs):
    import subprocess

    cfg = get_package_share_directory("rda_robot_moveit_config")
    lc = LaunchConfiguration
    mounts = lc("mounts_file").perform(context)

    urdf_xml = subprocess.check_output(
        ["ros2", "run", "rda_robot_assembler", "compose_urdf", "--mounts", mounts],
        text=True, stderr=subprocess.PIPE, timeout=180)
    with open(os.path.join(cfg, "config", "rda_robot.srdf")) as f:
        srdf_xml = f.read()

    rd = {"robot_description": urdf_xml}
    rds = {"robot_description_semantic": srdf_xml}
    kin = {"robot_description_kinematics": _load_yaml(os.path.join(cfg, "config", "kinematics.yaml"))}
    plan_lim = {"robot_description_planning": _load_yaml(os.path.join(cfg, "config", "joint_limits.yaml"))}
    ompl = _load_yaml(os.path.join(cfg, "config", "ompl_planning.yaml"))
    pipe = {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl", "ompl": ompl}
    execu = {"allow_trajectory_execution": False, "publish_planning_scene": True,
             "publish_geometry_updates": True, "publish_state_updates": True,
             "publish_transforms_updates": True}

    move_group = Node(package="moveit_ros_move_group", executable="move_group",
                      output="screen", parameters=[rd, rds, kin, plan_lim, pipe, execu])
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[rd])

    # base 재배치: link0 는 world→base_link 병진만큼 평행이동(z 는 바닥맞춤).
    z = _ground_offset(urdf_xml)
    bx = lc("base_x").perform(context)
    by = lc("base_y").perform(context)
    world_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    name="world_to_base_link",
                    arguments=["--x", bx, "--y", by, "--z", f"{z:.6f}",
                               "--frame-id", "world", "--child-frame-id", "base_link"])

    obstacles = Node(package="rda_robot_bringup", executable="obstacle_publisher.py",
                     output="screen",
                     parameters=[{"obstacles_file": os.path.join(DESC_SRC, "config", "obstacles.yaml")}])

    # 데모 노드 파라미터
    demo_params = {
        "standoff": float(lc("standoff").perform(context)),
        "grasp_offset": float(lc("grasp_offset").perform(context)),
        "target_index": int(lc("target_index").perform(context)),
        "loop": lc("loop").perform(context).lower() in ("1", "true", "yes"),
    }
    use_yaml = lc("use_yaml_target").perform(context).lower() in ("1", "true", "yes")
    if not use_yaml:
        demo_params["target"] = [float(v) for v in
                                 lc("target").perform(context).strip("[] ").split(",")]
    demo = Node(package="rda_robot_bringup", executable="pregrasp_demo.py",
                output="screen", parameters=[demo_params])

    nodes = [rsp, move_group, world_tf, obstacles, demo]
    if lc("rviz").perform(context).lower() in ("1", "true", "yes"):
        rviz_cfg = os.path.join(cfg, "config", "pregrasp_demo.rviz")
        if not os.path.exists(rviz_cfg):
            rviz_cfg = os.path.join(cfg, "config", "moveit.rviz")
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", rviz_cfg],
                          parameters=[rd, rds, kin, plan_lim]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mounts_file",
                              default_value=os.path.join(DESC_SRC, "config", "mounts.yaml")),
        DeclareLaunchArgument("target", default_value="[0.5,-0.4,0.9]",
                              description="목표 토마토 월드좌표 [x,y,z] (use_yaml_target=false 일 때)"),
        DeclareLaunchArgument("use_yaml_target", default_value="true",
                              description="RViz 의 실제 빨간 토마토(obstacles.yaml kind:target)를 target_index 로 선택"),
        DeclareLaunchArgument("target_index", default_value="3",
                              description="열매 인덱스(0~: 앞줄 최하단 화방부터). 기본=저설 열매"),
        DeclareLaunchArgument("base_x", default_value="0.86",
                              description="world→base_link X (로봇을 열매 앞으로 재배치, 도달권 확보)"),
        DeclareLaunchArgument("base_y", default_value="-0.80"),
        DeclareLaunchArgument("standoff", default_value="0.15"),
        DeclareLaunchArgument("grasp_offset", default_value="0.10"),
        DeclareLaunchArgument("loop", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
