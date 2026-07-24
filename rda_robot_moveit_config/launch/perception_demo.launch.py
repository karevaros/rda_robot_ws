"""Gazebo 센싱 → MoveIt 옥토맵 (5주차 perception 통합 · Stage 3)

구성: gazebo_sim.launch.py(온실 월드 + 로봇 + D435i depth 카메라 + rsp + jsp)
      + move_group(**sensors_3d.yaml = PointCloudOctomapUpdater**) + world→base_link
      + rviz2. 선택으로 obstacle_publisher(명명 장애물)도 같이 띄울 수 있다.

핵심: 카메라가 본 포인트클라우드가 **자동으로 planning scene 충돌객체(octomap)** 가 된다.
      obstacles.yaml 로 손으로 넣던 장애물을 '센싱'이 대신한다 → 실환경 이행의 관문.

실행:
  ros2 launch rda_robot_moveit_config perception_demo.launch.py
  ros2 launch rda_robot_moveit_config perception_demo.launch.py gui:=false        # gzclient off
  ros2 launch rda_robot_moveit_config perception_demo.launch.py obstacles:=true   # 명명 장애물 병행(비교용)
  ros2 launch rda_robot_moveit_config perception_demo.launch.py rviz:=false       # 헤드리스 검증

확인:
  ros2 topic hz /d435i/depth/points          # 센서 입력
  ros2 topic echo /monitored_planning_scene --once --field world.octomap.octomap.id   # 'OcTree'
  RViz → MotionPlanning → Scene Geometry 의 옥토맵 복셀(작물/거터가 복셀로 보임)

⚠ 시간: Gazebo 가 /clock 을 내므로 **모든 노드 use_sim_time=true**. 안 맞추면
   TF 시각이 어긋나 옥토맵 업데이터가 클라우드를 통째로 버린다(조용히 빈 옥토맵).
⚠ execute(컨트롤러)는 6주차 — 여기서도 allow_trajectory_execution=false.
⚠ 이전 launch 의 robot_state_publisher 가 남아 있으면 옛 URDF 가 latched 로 계속
   발행돼 엉뚱한 로봇이 스폰된다. 종료는 `ros2 launch` **부모 PID** 를 kill 할 것.
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DESC_SRC = os.path.expanduser("~/robot_ws/src/rda_robot_description")


def _load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _ground_offset(urdf_xml):
    """world→base_link z (base_footprint 를 바닥 z=0 으로). pregrasp_demo 와 동일 규칙."""
    import xml.etree.ElementTree as ET
    try:
        for j in ET.fromstring(urdf_xml).findall("joint"):
            if j.find("child").get("link") == "base_footprint":
                return -float((j.find("origin").get("xyz")).split()[2])
    except Exception:
        pass
    return 0.0


def _setup(context, *args, **kwargs):
    import subprocess

    cfg = get_package_share_directory("rda_robot_moveit_config")
    desc = get_package_share_directory("rda_robot_description")
    lc = LaunchConfiguration
    mounts = lc("mounts_file").perform(context)

    # move_group 이 쓸 URDF = 컴포저 원본(gazebo 오버레이 없이). Gazebo 쪽 rsp 는
    # 오버레이 주입본을 쓰지만 링크/조인트 기하는 동일하므로 TF 가 정합된다.
    urdf_xml = subprocess.check_output(
        ["ros2", "run", "rda_robot_assembler", "compose_urdf", "--mounts", mounts],
        text=True, stderr=subprocess.PIPE, timeout=180)
    with open(os.path.join(cfg, "config", "rda_robot.srdf")) as f:
        srdf_xml = f.read()

    rd = {"robot_description": urdf_xml}
    rds = {"robot_description_semantic": srdf_xml}
    kin = {"robot_description_kinematics":
           _load_yaml(os.path.join(cfg, "config", "kinematics.yaml"))}
    plan_lim = {"robot_description_planning":
                _load_yaml(os.path.join(cfg, "config", "joint_limits.yaml"))}
    ompl = _load_yaml(os.path.join(cfg, "config", "ompl_planning.yaml"))
    pipe = {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl", "ompl": ompl}
    execu = {"allow_trajectory_execution": False, "publish_planning_scene": True,
             "publish_geometry_updates": True, "publish_state_updates": True,
             "publish_transforms_updates": True}
    sim = {"use_sim_time": True}

    # ★ Stage 3 의 본체: 3D 센서 → 옥토맵.
    #   octomap_frame 은 **고정 프레임(world)** 이어야 한다. 센서 프레임으로 두면
    #   카메라가 움직일 때 지도가 따라 움직여 무의미해진다.
    sensors = _load_yaml(os.path.join(cfg, "config", "sensors_3d.yaml"))
    # 사용할 3D 센서 선택. ⚠ 업데이터가 2개 이상이면 MoveIt 이 shape 핸들을 간접 매핑하는데,
    #   그 경로에서 월드 CollisionObject 를 추가하면 transform cache 조회가 깨져
    #   "Missing transform for shape …" 가 쏟아지고 마스킹·서비스 응답이 망가진다(실측).
    #   Stage 5(구 영역 허용)를 쓰려면 단일 센서로 두는 편이 안전하다.
    want = lc("sensors").perform(context).strip().lower()
    if want not in ("", "both", "all"):
        keep = [n for n in sensors.get("sensors", []) if want in n]
        sensors = dict(sensors)
        sensors["sensors"] = keep or sensors.get("sensors", [])
    octomap = {"octomap_frame": lc("octomap_frame").perform(context),
               "octomap_resolution": float(lc("octomap_resolution").perform(context))}

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[rd, rds, kin, plan_lim, pipe, execu, sensors, octomap, sim])

    # 로봇 배치 = 어셈블러 base_placement(Gazebo 스폰 위치와 동일 규칙 → 좌표 정합).
    gz = _ground_offset(urdf_xml)
    bp = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}
    try:
        d = _load_yaml(mounts) or {}
        for k in bp:
            if k in (d.get("base_placement") or {}):
                bp[k] = float(d["base_placement"][k])
    except Exception:
        pass
    world_tf = Node(package="tf2_ros", executable="static_transform_publisher",
                    name="world_to_base_link", parameters=[sim],
                    arguments=["--x", f"{bp['x']:.6f}", "--y", f"{bp['y']:.6f}",
                               "--z", f"{gz + bp['z']:.6f}",
                               "--yaw", f"{math.radians(bp['yaw_deg']):.6f}",
                               "--frame-id", "world", "--child-frame-id", "base_link"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc, "launch", "gazebo_sim.launch.py")),
        launch_arguments={"mounts_file": mounts,
                          "gui": lc("gui").perform(context),
                          "world": lc("world").perform(context)}.items())

    nodes = [gazebo, move_group, world_tf]

    # 명명 장애물(obstacles.yaml)은 기본 off — Stage 3 의 목적이 '센싱만으로 장애물이
    # 서는가' 이므로 켜 두면 무엇이 옥토맵인지 구분이 안 된다. 비교용으로만 켠다.
    if lc("obstacles").perform(context).lower() in ("1", "true", "yes"):
        nodes.append(Node(package="rda_robot_bringup", executable="obstacle_publisher.py",
                          output="screen",
                          parameters=[{"obstacles_file": os.path.join(
                              DESC_SRC, "config", "obstacles.yaml")}, sim]))

    # Stage 4: 열매 인지 — 클라우드의 빨강 영역 → 3D 구(중심·반경) → /detected_fruits.
    if lc("detect").perform(context).lower() in ("1", "true", "yes"):
        nodes.append(Node(package="rda_robot_bringup", executable="fruit_detector.py",
                          output="screen",
                          parameters=[{"world_frame": "world",
                                       "cloud_topics": [s.strip() for s in
                                                        lc("cloud_topics").perform(context).split(",")
                                                        if s.strip()]}, sim]))

    if lc("rviz").perform(context).lower() in ("1", "true", "yes"):
        rviz_cfg = os.path.join(cfg, "config", "perception_demo.rviz")
        if not os.path.exists(rviz_cfg):
            rviz_cfg = os.path.join(cfg, "config", "moveit.rviz")
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", rviz_cfg],
                          parameters=[rd, rds, kin, plan_lim, sim]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mounts_file",
                              default_value=os.path.join(DESC_SRC, "config", "mounts.yaml")),
        DeclareLaunchArgument("gui", default_value="true",
                              description="gzclient(Gazebo GUI) 실행 여부"),
        DeclareLaunchArgument("world", default_value="greenhouse",
                              description="greenhouse(온실+작물) 또는 empty"),
        DeclareLaunchArgument("obstacles", default_value="false",
                              description="명명 장애물(obstacles.yaml)도 planning scene 에 넣을지(비교용)"),
        DeclareLaunchArgument("sensors", default_value="both",
                              description="옥토맵에 쓸 3D 센서: both(D435i+D405) | d435i | d405. "
                                          "단일 센서면 월드 객체 추가 시 shape mask 가 안전하다 [Stage 5]"),
        DeclareLaunchArgument("octomap_frame", default_value="world",
                              description="옥토맵을 유지할 고정 프레임"),
        DeclareLaunchArgument("octomap_resolution", default_value="0.03",
                              description="복셀 한 변[m]. 줄기(지름 2~3cm)를 보려면 0.02~0.03."),
        DeclareLaunchArgument("detect", default_value="true",
                              description="열매 인지 노드(fruit_detector) 실행 — /detected_fruits 발행"),
        DeclareLaunchArgument("cloud_topics",
                              default_value="/d435i/depth/points,/d405/depth/points",
                              description="인지에 쓸 포인트클라우드 토픽(쉼표 구분)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
