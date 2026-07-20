"""5주차 pre-grasp 집기 데모 launch — 알고리즘이 이끄는 로봇 동작을 RViz 에서 재생.

구성: robot_state_publisher + move_group + obstacle_publisher + world→base_link(재배치)
      + rviz2 + pregrasp_demo(노드). ⚠ jsp_gui 는 넣지 않는다 — 데모 노드가 유일한
      /joint_states 발행자다(둘이 같이 발행하면 자세가 튄다).

사용:
  ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py
  #  로봇 위치 = 어셈블러가 mounts.yaml 에 저장한 base_placement 를 그대로 불러옴.
  #  그 위치에서 도달 가능한 실제 빨간 토마토를 자동 선택해 집기 시퀀스 재생.
  ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py rviz:=false     # 헤드리스
  # 특정 열매만: auto_reachable:=false target_index:=3
  # 위치 임시 덮어쓰기: base_x:=0.86 base_y:=-0.80 base_yaw:=-1.5708
  # 좌표 목표(열매 대신): use_yaml_target:=false target:="[0.5,-0.4,0.9]"

  ⚠ 이 팔(reach 0.93m)로 고설 토마토(z 1.2~1.7m)는 도달권이 좁다. 어셈블러에서
    로봇(base_placement)을 열매 앞으로 옮겨 저장해야 닿는다. 도달 가능한 열매가
    하나도 없으면 데모가 '도달 가능한 열매 없음'을 알리고 로봇 이동을 권한다.

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

    # 로봇 위치 = 어셈블러가 mounts.yaml 에 저장한 base_placement(x·y·z·yaw) 를 불러온다.
    #  (display launch 와 동일 규칙: x/y/yaw + z 는 바닥오프셋에 더함) → 고정 온실 안에서
    #  로봇이 어셈블러에서 놓은 위치로 배치된다. base_x/base_y/base_yaw 인자를 'auto' 가
    #  아닌 값으로 주면 저장값을 덮어쓴다.
    import math as _math
    gz = _ground_offset(urdf_xml)
    bp = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
    try:
        d = _load_yaml(mounts) or {}
        for k in bp:
            if k in (d.get("base_placement") or {}):
                bp[k] = float(d["base_placement"][k])
    except Exception:
        pass

    def _override(argname, saved):
        v = lc(argname).perform(context).strip().lower()
        return saved if v in ("", "auto") else float(v)
    bx = _override("base_x", bp["x"])
    by = _override("base_y", bp["y"])
    bz = gz + bp["z"]
    yaw = _override("base_yaw", bp["yaw_deg"] * _math.pi / 180.0)
    world_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    name="world_to_base_link",
                    arguments=["--x", f"{bx:.6f}", "--y", f"{by:.6f}", "--z", f"{bz:.6f}",
                               "--yaw", f"{yaw:.6f}",
                               "--frame-id", "world", "--child-frame-id", "base_link"])

    obstacles = Node(package="rda_robot_bringup", executable="obstacle_publisher.py",
                     output="screen",
                     parameters=[{"obstacles_file": os.path.join(DESC_SRC, "config", "obstacles.yaml")}])

    # 데모 노드 파라미터
    demo_params = {
        "standoff": float(lc("standoff").perform(context)),
        "grasp_offset": float(lc("grasp_offset").perform(context)),
        "target_index": int(lc("target_index").perform(context)),
        "auto_reachable": lc("auto_reachable").perform(context).lower() in ("1", "true", "yes"),
        "loop": lc("loop").perform(context).lower() in ("1", "true", "yes"),
        # 팔 프레임 — 팔 스왑 시 지정(예: UR10e=arm_base_link/arm_tool0). 기본=RB5 계약.
        "base_link": lc("base_link").perform(context),
        "ik_link": lc("ik_link").perform(context),
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
                              description="RViz 의 실제 빨간 토마토(obstacles.yaml kind:target)를 대상으로"),
        DeclareLaunchArgument("auto_reachable", default_value="true",
                              description="현재 로봇 위치에서 도달 가능한 열매를 자동 선택(가까운 것부터). "
                                          "false 면 target_index 열매만."),
        DeclareLaunchArgument("target_index", default_value="0",
                              description="auto_reachable=false 일 때 쓸 열매 인덱스"),
        DeclareLaunchArgument("base_x", default_value="auto",
                              description="world→base_link X. 'auto'=mounts.yaml base_placement(어셈블러 저장값). 숫자면 덮어씀."),
        DeclareLaunchArgument("base_y", default_value="auto"),
        DeclareLaunchArgument("base_yaw", default_value="auto",
                              description="world→base_link yaw[rad]. 'auto'=mounts.yaml base_placement.yaw_deg."),
        DeclareLaunchArgument("base_link", default_value="link0",
                              description="팔 루트 링크(접근방향 base TF). 팔 스왑 시 예: arm_base_link"),
        DeclareLaunchArgument("ik_link", default_value="tcp",
                              description="IK/파지 기준 링크(그리퍼 부착 플랜지). 팔 스왑 시 예: arm_tool0"),
        DeclareLaunchArgument("standoff", default_value="0.12",
                              description="pre-grasp 가 grasp 에서 뒤로 떨어진 거리(=직선 접근 이동거리)"),
        DeclareLaunchArgument("grasp_offset", default_value="0.13",
                              description="파지 시 TCP 가 열매 중심 앞에 멈추는 거리(손끝이 열매 표면에 닿게)"),
        DeclareLaunchArgument("loop", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
