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
import re
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


def _compose_env():
    """compose_urdf 서브프로세스용 env. ⚠ 비대화형(bash script.sh)·헤드리스·ros2 launch
    내부 실행에선 user site(~/.local/lib/pythonX.Y/site-packages)가 PYTHONPATH 에서 빠져
    compose 가 의존하는 yourdfpy 를 못 찾는다(ModuleNotFoundError). 여기서 강제로 넣어준다."""
    import sys
    env = dict(os.environ)
    us = os.path.join(os.path.expanduser("~"), ".local", "lib",
                      f"python{sys.version_info.major}.{sys.version_info.minor}",
                      "site-packages")
    if os.path.isdir(us):
        cur = env.get("PYTHONPATH", "")
        if us not in cur.split(os.pathsep):
            env["PYTHONPATH"] = us + (os.pathsep + cur if cur else "")
    return env


def compose_urdf(mounts_file):
    """mounts.yaml → 통합 URDF XML(문자열). rda_robot_display.launch.py 와 동일 방식."""
    try:
        return subprocess.check_output(
            ["ros2", "run", "rda_robot_assembler", "compose_urdf",
             "--mounts", mounts_file],
            text=True, stderr=subprocess.PIPE, timeout=180, env=_compose_env())
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


def _inject_text(urdf_xml, text):
    """통합 URDF 의 </robot> 앞에 임의의 gazebo 스니펫(문자열)을 주입."""
    idx = urdf_xml.rfind("</robot>")
    if idx < 0:
        raise RuntimeError("URDF 에 </robot> 가 없습니다.")
    return urdf_xml[:idx] + "\n" + text + "\n" + urdf_xml[idx:]


def _inject_overlay(urdf_xml, overlay_file):
    """통합 URDF 의 </robot> 앞에 gazebo 오버레이(카메라 센서)를 주입."""
    with open(overlay_file) as f:
        return _inject_text(urdf_xml, f.read())


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


_ARM_JOINTS = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
_GRIPPER_JOINT = "rg2_finger_joint1"


def _static_overlay():
    """control:=false 일 때 주입하는 정적 블록(구 gazebo_overlay.xml 의 <static>)."""
    return "<gazebo>\n  <static>true</static>\n</gazebo>\n"


def _gazeboize_control(urdf_xml, controllers_yaml, placement):
    """control:=true 용 URDF 변형 (compose_urdf 산출물은 손대지 않고 여기서만 가공).

    ① 벤더 ros2_control(rbpodo 실하드웨어 IP)를 gazebo_ros2_control/GazeboSystem 으로 교체하고
       팔 6조인트 + 그리퍼(rg2_finger_joint1)에 position 명령 / position·velocity 상태
       인터페이스를 건다. position 명령 → GazeboSystem 이 PID 미설정 시 kinematic SetPosition
       으로 이동(placeholder 관성으로도 안정).
    ② base_link 를 world 에 고정(world 링크 + fixed 조인트, 원점=base_placement). 모델을
       non-static 으로 풀면 떠다니는 베이스가 되므로 고정해 팔만 움직이게 한다. 스폰 pose 는
       world 링크가 전역 원점을 강제하므로 무시된다 → 배치는 이 조인트 origin 으로 넣는다.
    ③ libgazebo_ros2_control.so 플러그인 주입(<parameters>=controllers.yaml)."""
    root = ET.fromstring(urdf_xml)

    rc = root.find("ros2_control")
    if rc is None:
        raise RuntimeError("URDF 에 <ros2_control> 블록이 없습니다(벤더 팔 xacro 확인).")

    # ① 하드웨어 플러그인 교체 + 벤더 param 제거
    hw = rc.find("hardware")
    if hw is not None:
        for p in list(hw.findall("param")):
            hw.remove(p)
        plugin = hw.find("plugin")
        if plugin is None:
            plugin = ET.SubElement(hw, "plugin")
        plugin.text = "gazebo_ros2_control/GazeboSystem"

    def _set_interfaces(joint_el):
        # 기존 인터페이스/파라미터 제거 후 position 명령 + position·velocity 상태로 재구성
        for ch in list(joint_el):
            if ch.tag in ("command_interface", "state_interface"):
                joint_el.remove(ch)
        ET.SubElement(joint_el, "command_interface").set("name", "position")
        ET.SubElement(joint_el, "state_interface").set("name", "position")
        ET.SubElement(joint_el, "state_interface").set("name", "velocity")

    existing = {j.get("name") for j in rc.findall("joint")}
    for j in rc.findall("joint"):
        _set_interfaces(j)

    # 그리퍼 조인트가 벤더 블록엔 없으므로 추가(finger_joint2 는 URDF mimic 이 자동 처리)
    if _GRIPPER_JOINT not in existing:
        gj = ET.SubElement(rc, "joint")
        gj.set("name", _GRIPPER_JOINT)
        _set_interfaces(gj)

    # ② world 고정
    bx, by, bz, byaw = placement
    if root.find("link[@name='world']") is None:
        ET.SubElement(root, "link").set("name", "world")
        wj = ET.SubElement(root, "joint")
        wj.set("name", "world_fixed")
        wj.set("type", "fixed")
        ET.SubElement(wj, "parent").set("link", "world")
        ET.SubElement(wj, "child").set("link", "base_link")
        o = ET.SubElement(wj, "origin")
        o.set("xyz", f"{bx:.6f} {by:.6f} {bz:.6f}")
        o.set("rpy", f"0 0 {byaw:.6f}")

    # ③ gazebo_ros2_control 플러그인
    gz = ET.SubElement(root, "gazebo")
    pl = ET.SubElement(gz, "plugin")
    pl.set("filename", "libgazebo_ros2_control.so")
    pl.set("name", "gazebo_ros2_control")
    ET.SubElement(pl, "parameters").text = controllers_yaml

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

    control = LaunchConfiguration("control").perform(context).lower() in ("1", "true", "yes")

    composed = _add_missing_inertials(compose_urdf(mounts_file))
    z = _ground_offset(composed)          # world→base_footprint 보정(world 링크 추가 전 계산)
    bx, by, bz, byaw = _base_placement(mounts_file)

    if control:
        # control 모드: ros2_control(GazeboSystem)+world 고정. static 은 주입하지 않는다.
        # world 링크가 전역 원점을 강제하므로 배치는 world_fixed 조인트 origin 에 넣고
        # spawn pose 는 0 으로 둔다.
        controllers_yaml = os.path.join(desc, "config", "controllers.yaml")
        composed = _gazeboize_control(composed, controllers_yaml,
                                      (bx, by, z + bz, byaw))
        urdf_xml = _inject_overlay(composed, overlay)
        # 🔴 gazebo_ros2_control(0.4.10)은 robot_description(URDF)을 controller_manager 노드에
        #   `-p robot_description:=<URDF>` CLI 인자로 넘긴다. rcl 은 이 값을 **YAML 로 파싱**하는데
        #   pretty-print 된 여러 줄 URDF 는 YAML 로 유효하지 않아 파싱이 깨진다("Couldn't parse
        #   parameter override rule") → CM 노드 미생성 → 스포너 타임아웃 → 컨트롤러 0개 →
        #   팔이 중력에 처짐. ⇒ control 모드에선 **주석 제거 + 한 줄로 압축**해 rsp 에 준다
        #   (rsp 가 재발행 → 플러그인이 한 줄 URDF 를 넘김 → rcl YAML 스칼라로 파싱 OK).
        urdf_xml = re.sub(r"<!--.*?-->", "", urdf_xml, flags=re.DOTALL)
        urdf_xml = re.sub(r"\s+", " ", urdf_xml).strip()
    else:
        # 기본(Stage 1~5): 정적 블록 + 센서 오버레이. 배치는 spawn pose 로.
        with open(overlay) as f:
            urdf_xml = _inject_text(composed, _static_overlay() + f.read())

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

    if control:
        # control 모드: 관절 상태는 joint_state_broadcaster 가 발행(jsp 금지 — 충돌).
        # 배치는 world_fixed 조인트 origin 에 이미 반영 → spawn pose 는 0.
        spawn = TimerAction(period=5.0, actions=[
            Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
                 arguments=["-topic", "robot_description", "-entity", "rda_robot",
                            "-timeout", "60"])])
        # gazebo_ros2_control 플러그인이 controller_manager 를 띄운 뒤 스포너로 활성화.
        # ⚠ 스폰(5s)→Gazebo 모델 로드→플러그인 CM 생성까지 시간이 더 걸린다. 스포너가 너무
        #   일찍 뜨면 CM 을 못 찾아 실패한다 → 12s 로 미루고, 그래도 못 찾으면 60s 까지
        #   기다리도록 --controller-manager-timeout 을 준다(활성 실패 시 팔이 중력에 처짐).
        def _spawner(name):
            return Node(package="controller_manager", executable="spawner", output="screen",
                        arguments=[name, "--controller-manager", "/controller_manager",
                                   "--controller-manager-timeout", "60"])
        # ⚠ gazebo 안의 CM 은 기동 직후 서비스 응답이 느려(sim 루프에 묶임) 스포너들을 동시에
        #   띄우면 configure 호출이 타임아웃→재시도→"이미 active" 로 실패한다(컨트롤러 자체는
        #   1차 시도로 active 가 됨). ⇒ **시차를 두고 하나씩** 띄워 CM 부하를 분산한다.
        #   broadcaster(관절상태 발행) 먼저, 그다음 arm, gripper.
        spawners = [
            TimerAction(period=12.0, actions=[_spawner("joint_state_broadcaster")]),
            TimerAction(period=16.0, actions=[_spawner("arm_controller")]),
            TimerAction(period=20.0, actions=[_spawner("gripper_controller")]),
        ]
        return procs + [rsp, spawn] + spawners

    # 기본(static) 모드: TF 완성용 jsp(팔 관절 0 고정) + base_placement spawn pose.
    jsp = Node(package="joint_state_publisher", executable="joint_state_publisher",
               output="screen", parameters=[{"use_sim_time": True}])
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
        DeclareLaunchArgument("control", default_value="false",
                              description="true=ros2_control(GazeboSystem) 로 팔 실구동"
                                          "(execute·실시간 옥토맵). false=static(Stage1~5)."),
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
