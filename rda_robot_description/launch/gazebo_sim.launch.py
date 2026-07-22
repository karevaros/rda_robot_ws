#!/usr/bin/env python3
"""Gazebo Classic 시뮬 (5주차 perception 통합 · Stage 1)

통합 URDF(compose_urdf)에 config/gazebo_overlay.xml 을 주입해 Gazebo 로 스폰한다.
목표: D435i(eye-to-hand) depth 카메라가 **PointCloud2 + camera_info** 를 발행하고
TF 가 정합되는 것을 확인(빈 월드). 이후 Stage 2 에서 온실/작물 Gazebo 모델을 세우면
카메라가 그것을 '보고', Stage 3~ 에서 인지→planning 으로 연결된다.

실행:
  ros2 launch rda_robot_description gazebo_sim.launch.py
  # 토픽 확인: ros2 topic list | grep d435i ; ros2 topic echo /d435i/depth/points --once

⚠ 로봇은 static(오버레이) — 물리 구동 없음(관성 누락 무해, 팔은 스폰 자세 유지).
   실제 팔 구동(execute)은 6주차 ros2_control 도입 시.
"""
import math
import os
import subprocess
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def compose_urdf(mounts_file):
    """mounts.yaml → 통합 URDF XML(문자열). rda_robot_display.launch.py 와 동일 방식."""
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
    """world→base_link z 오프셋(base_footprint 를 z=0 으로). 없으면 0."""
    try:
        for j in ET.fromstring(urdf_xml).findall("joint"):
            c = j.find("child")
            if c is not None and c.get("link") == "base_footprint":
                o = j.find("origin")
                return -float((o.get("xyz") or "0 0 0").split()[2])
    except Exception:
        pass
    return 0.0


def _inject_overlay(urdf_xml, overlay_file):
    """통합 URDF 의 </robot> 앞에 gazebo 오버레이(정적+카메라)를 주입."""
    with open(overlay_file) as f:
        overlay = f.read()
    idx = urdf_xml.rfind("</robot>")
    if idx < 0:
        raise RuntimeError("URDF 에 </robot> 가 없습니다.")
    return urdf_xml[:idx] + "\n" + overlay + "\n" + urdf_xml[idx:]


def _add_missing_inertials(urdf_xml):
    """visual/collision 은 있는데 inertial 이 없는 링크에 기본 관성을 넣는다.

    Gazebo(sdformat)의 URDF→SDF 변환은 **무질량 중간 링크에서 운동학 체인을 끊고 그
    위 링크(팔·EE·카메라)를 통째로 드롭**한다. 통합 URDF 는 link0(팔 루트)에 관성이
    없어, Gazebo 에선 팔 이상이 전부 사라졌다(로봇 하반부 base 만 보임). 여기서 누락
    링크에 형식적 관성을 채워 체인을 잇는다(로봇은 static 이라 물리값은 무의미)."""
    root = ET.fromstring(urdf_xml)
    added = []
    for link in root.findall("link"):
        if link.find("inertial") is None and (link.find("visual") is not None
                                              or link.find("collision") is not None):
            ine = ET.SubElement(link, "inertial")
            ET.SubElement(ine, "mass").set("value", "0.1")
            inertia = ET.SubElement(ine, "inertia")
            for k, v in (("ixx", "0.001"), ("ixy", "0"), ("ixz", "0"),
                         ("iyy", "0.001"), ("iyz", "0"), ("izz", "0.001")):
                inertia.set(k, v)
            added.append(link.get("name"))
    if added:
        print(f"[gazebo_sim] 관성 보완(체인 유지): {', '.join(added)}")
    return ET.tostring(root, encoding="unicode")


def gen_world(obstacles_yaml):
    """obstacles.yaml → Gazebo SDF 월드(온실 구조+작물). gen_gazebo_world.py 재사용.
    좌표·형상이 RViz/MoveIt 장면(obstacle_publisher)과 단일 진실원으로 정합."""
    try:
        sdf = subprocess.check_output(
            ["ros2", "run", "rda_robot_bringup", "gen_gazebo_world.py", obstacles_yaml],
            text=True, stderr=subprocess.PIPE, timeout=60)
    except subprocess.CalledProcessError as e:
        raise RuntimeError("Gazebo 월드 생성 실패:\n" + (e.stderr or "").strip())
    path = "/tmp/rda_greenhouse.world"
    with open(path, "w") as f:
        f.write(sdf)
    return path


def _base_placement(mounts_file):
    """mounts.yaml base_placement → (x, y, z, yaw[rad]). 없으면 0."""
    try:
        d = yaml.safe_load(open(mounts_file)) or {}
        bp = d.get("base_placement", {}) or {}
        return (float(bp.get("x", 0.0)), float(bp.get("y", 0.0)),
                float(bp.get("z", 0.0)), math.radians(float(bp.get("yaw_deg", 0.0))))
    except Exception:
        return (0.0, 0.0, 0.0, 0.0)


def _setup(context, *args, **kwargs):
    desc = get_package_share_directory("rda_robot_description")
    mounts_file = LaunchConfiguration("mounts_file").perform(context)
    overlay = os.path.join(desc, "config", "gazebo_overlay.xml")

    urdf_xml = _inject_overlay(_add_missing_inertials(compose_urdf(mounts_file)), overlay)
    z = _ground_offset(urdf_xml)
    robot_description = ParameterValue(urdf_xml, value_type=str)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")

    # ★ 메시 해석: URDF 의 package://<pkg>/meshes 는 Gazebo 에서 model://<pkg>/meshes 로
    #   변환된다 → GAZEBO_MODEL_PATH 에 각 패키지 share(install/<pkg>/share)를 넣어야
    #   model://<pkg>/... 가 install/<pkg>/share/<pkg>/... 로 해석된다. 안 넣으면 로봇이
    #   메시 없이(투명) 스폰된다. AMENT_PREFIX_PATH(=install/<pkg>) 에서 유도.
    #   ⚠ GAZEBO_RESOURCE_PATH 는 건드리지 않는다 — 값을 덮어쓰면 gzserver 가 컴파일 기본값
    #     (worlds/empty.world·셰이더 lib)을 잃어 월드 로드 실패→/spawn_entity 미등록.
    ament = os.environ.get("AMENT_PREFIX_PATH", "")
    shares = [os.path.join(p, "share") for p in ament.split(os.pathsep) if p]
    cur = os.environ.get("GAZEBO_MODEL_PATH", "")
    os.environ["GAZEBO_MODEL_PATH"] = os.pathsep.join(shares) + (os.pathsep + cur if cur else "")

    # 월드: greenhouse(온실 구조+작물, 카메라가 봄) 또는 empty.
    world_mode = LaunchConfiguration("world").perform(context)
    if world_mode == "empty":
        world_file = "worlds/empty.world"
    else:
        world_file = gen_world(LaunchConfiguration("obstacles_file").perform(context))

    # gzserver 를 명시적으로 ros_init + ros_factory 플러그인과 함께 실행(그래야 /spawn_entity
    # 서비스가 뜬다). 온라인 모델 DB 조회는 비활성(기동 지연·인터넷 의존 제거).
    gzserver = ExecuteProcess(
        cmd=["gzserver", "--verbose",
             "-s", "libgazebo_ros_init.so",
             "-s", "libgazebo_ros_factory.so",
             world_file],
        output="screen")
    procs = [gzserver]
    if gui:
        procs.append(ExecuteProcess(cmd=["gzclient", "--verbose"], output="screen"))

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen", parameters=[{"robot_description": robot_description,
                                             "use_sim_time": True}])
    # TF 완성용(팔 관절 0). static 모델이라 값은 0 고정.
    jsp = Node(package="joint_state_publisher", executable="joint_state_publisher",
               output="screen", parameters=[{"use_sim_time": True}])
    # /robot_description(latched) 를 읽어 스폰. 로봇을 어셈블러 base_placement 자세로
    # 배치(월드 프레임의 온실/작물을 카메라가 실제 배치대로 보도록). base_footprint z 보정.
    bx, by, bz, byaw = _base_placement(mounts_file)
    # gzserver 가 /spawn_entity 를 띄울 시간을 주려고 5초 뒤에 스폰.
    spawn = TimerAction(period=5.0, actions=[
        Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
             arguments=["-topic", "robot_description", "-entity", "rda_robot",
                        "-x", f"{bx:.5f}", "-y", f"{by:.5f}", "-z", f"{z + bz:.5f}",
                        "-Y", f"{byaw:.6f}", "-timeout", "60"])])
    return procs + [rsp, jsp, spawn]


def generate_launch_description():
    return LaunchDescription([
        # 온라인 모델 DB 조회 비활성(기동 지연·인터넷 의존 제거).
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        DeclareLaunchArgument(
            "mounts_file",
            default_value=os.path.join(
                get_package_share_directory("rda_robot_description"),
                "config", "mounts.yaml")),
        DeclareLaunchArgument("gui", default_value="true",
                              description="gzclient(GUI) 실행 여부. false=헤드리스."),
        DeclareLaunchArgument("world", default_value="greenhouse",
                              description="greenhouse(온실 구조+작물, 카메라가 봄) 또는 empty."),
        DeclareLaunchArgument(
            "obstacles_file",
            default_value=os.path.join(
                get_package_share_directory("rda_robot_description"),
                "config", "obstacles.yaml"),
            description="Gazebo 월드로 세울 온실/작물 정의(단일 진실원)."),
        OpaqueFunction(function=_setup),
    ])
