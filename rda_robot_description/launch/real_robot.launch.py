"""실기(Rainbow RB5 제어박스) 연동 bringup — 7주차 실기 적용 준비.

    ros2 launch rda_robot_description real_robot.launch.py \
        use_fake_hardware:=true                  # ① 하드웨어 없이 배선만 검증
    ros2 launch rda_robot_description real_robot.launch.py \
        robot_ip:=10.0.2.7 cb_simulation:=true   # ② 제어박스 연결·팔은 안 움직임
    ros2 launch rda_robot_description real_robot.launch.py \
        robot_ip:=10.0.2.7 cb_simulation:=false  # ③ 실제 구동 🔴

■ 시뮬(gazebo_sim.launch.py control:=true)과의 관계
  gazebo_sim 은 통합 URDF 의 **벤더 ros2_control 블록을 지우고** GazeboSystem 으로
  갈아끼운다(_gazeboize_control). 이 launch 는 정반대로 **그 블록을 그대로 둔다** —
  즉 실기 경로는 새로 만든 게 아니라 이미 통합 URDF 안에 있던 것이다.
  통합 URDF 는 두 경로가 같은 컴포저 산출물을 쓰므로 형상이 갈릴 일이 없다.

■ 세 모드의 차이 (실측으로 확인한 것)
  use_fake_hardware:=true  → mock_components/GenericSystem. 네트워크·제어박스 **불필요**.
                             명령을 그대로 상태로 되돌려준다(배선·컨트롤러 활성 검증용).
  cb_simulation:=true      → 🔴 **제어박스에 실제로 접속한다**(Robot 생성자가 IP 로 연결).
                             제어박스의 OperationMode 만 Simulation 이라 팔이 안 움직일 뿐,
                             제어박스가 네트워크에 없으면 그냥 실패한다. '오프라인 모드'가 아니다.
  cb_simulation:=false     → Real 모드. 팔이 실제로 움직인다.

■ 그리퍼는 여기서 다루지 않는다
  실기 RG2 는 제어박스 명령(gripper_macro)으로 움직이므로 ros2_control 관절이 아니다.
  따라서 joint_state_broadcaster 는 손가락 관절을 발행하지 않는다 → 손가락 TF 가 비게 된다.
  joint_state_publisher 를 끼워 **URDF 의 나머지 관절을 기본값으로 채운다**(아래 ③).
  그리퍼 어댑터 노드가 생기면 그 노드의 관절 상태를 source_list 에 추가하면 된다.
"""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "rda_robot_description"


def _compose_env():
    """compose_urdf 서브프로세스용 env — gazebo_sim.launch.py 와 동일 이유(user site 누락)."""
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


def compose_urdf(mounts_file, model_args):
    """mounts.yaml + 팔 xacro 인자 → 통합 URDF XML(문자열).

    model_args 로 robot_ip·cb_simulation·use_fake_hardware 를 넘긴다(컴포저 --model-arg).
    이 경로가 없으면 벤더 기본값(robot_ip=10.0.2.7)이 URDF 에 그대로 굳는다."""
    cmd = ["ros2", "run", "rda_robot_assembler", "compose_urdf", "--mounts", mounts_file]
    for k, v in model_args.items():
        cmd += ["--model-arg", f"arm:{k}={v}"]
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.PIPE,
                                       timeout=180, env=_compose_env())
    except subprocess.CalledProcessError as e:
        raise RuntimeError("통합 URDF 조립 실패:\n" + (e.stderr or "").strip())
    except FileNotFoundError:
        raise RuntimeError("ros2 실행파일이 없습니다. 환경을 source 했는지 확인하세요.")


#: 레인보우 제어박스 TCP 포트 — 명령 5000 / 데이터 5001 (rbpodo SDK 기본값)
CB_PORTS = (5000, 5001)


def _preflight(ip, timeout=3.0):
    """제어박스 접속 가능한지 먼저 확인한다.

    🔴 왜 필요한가 (2026-07-30 실측): 제어박스가 없는데 실기 모드로 띄우면
    RBPodoHardwareInterface 가 `Connecting to robot at "..."` 에서 **에러도 타임아웃도 없이
    무한 대기**한다. controller_manager 자체가 안 올라와 스포너만
    "waiting for service /controller_manager/list_controllers" 를 반복하는데,
    이 로그는 '느린 기동'과 구분이 안 된다 — 원인이 네트워크라는 단서가 어디에도 안 남는다.
    여기서 먼저 소켓을 열어 보고 실패하면 무엇이 문제인지 말하고 즉시 멈춘다."""
    import socket
    for port in CB_PORTS:
        try:
            with socket.create_connection((ip, port), timeout=timeout):
                pass
        except OSError as e:
            raise RuntimeError(
                f"레인보우 제어박스에 접속할 수 없습니다: {ip}:{port} ({e})\n"
                "  · 제어박스 전원·네트워크(무선 AP) 연결을 확인하세요.\n"
                f"  · IP 가 다르면 robot_ip:=<주소> 로 넘기세요(현재 {ip}).\n"
                "  · 하드웨어 없이 배선만 확인하려면 use_fake_hardware:=true 로 실행하세요.\n"
                "  ⚠ 이 확인을 건너뛰면 하드웨어 인터페이스가 조용히 무한 대기합니다.")


def _setup(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)

    share = get_package_share_directory(PKG)
    mounts = cfg("mounts") or os.path.join(share, "config", "mounts.yaml")
    controllers = cfg("controllers") or os.path.join(share, "config", "controllers_real.yaml")

    fake = cfg("use_fake_hardware").lower() in ("true", "1")
    if not fake and cfg("preflight").lower() in ("true", "1"):
        _preflight(cfg("robot_ip"))

    urdf_xml = compose_urdf(mounts, {
        "robot_ip": cfg("robot_ip"),
        "cb_simulation": cfg("cb_simulation"),
        "use_fake_hardware": cfg("use_fake_hardware"),
    })

    # 조립 결과가 정말 의도한 하드웨어인지 확인하고 알린다.
    # (플러그인 이름이 조용히 반대로 들어가는 사고를 막는다 — 이 프로젝트의 '조용한 무효' 패턴)
    plugin = "mock_components/GenericSystem" if fake else "rbpodo_hardware/RBPodoHardwareInterface"
    if plugin not in urdf_xml:
        raise RuntimeError(
            f"통합 URDF 에 기대한 하드웨어 플러그인이 없습니다: {plugin}\n"
            "  팔 모델이 rb5_850e(벤더 ros2_control 블록 보유)인지 확인하세요.")
    mode = "mock(하드웨어 없음)" if fake else \
        f"제어박스 {cfg('robot_ip')} / OperationMode=" + \
        ("Simulation" if cfg("cb_simulation").lower() in ("true", "1") else "🔴 Real")
    print(f"[real_robot] 하드웨어 = {plugin}\n[real_robot] 모드 = {mode}")

    nodes = [
        # ① 통합 URDF 발행. controller_manager 도 /robot_description 토픽으로 받아 간다.
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             output="both", parameters=[{"robot_description": urdf_xml,
                                         "use_sim_time": False}]),

        # ② ros2_control. joint_state_broadcaster 출력을 바로 /joint_states 로 내보내지 않고
        #    중간 토픽으로 뺀다 — 팔 6축만 들어 있어 그대로 쓰면 손가락 TF 가 사라진다(③).
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers], output="both",
             remappings=[("~/robot_description", "/robot_description"),
                         ("joint_states", "arm/joint_states")]),

        # ③ 팔 상태 + URDF 나머지 관절(그리퍼 손가락 등) 기본값을 합쳐 /joint_states 발행
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             parameters=[{"source_list": ["arm/joint_states"], "rate": 30,
                          "use_sim_time": False}], output="log"),

        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
             output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_controller", "-c", "/controller_manager"],
             output="screen"),
    ]

    rviz_cfg = os.path.join(share, "config", "rda_robot.rviz")
    if os.path.exists(rviz_cfg):
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          arguments=["-d", rviz_cfg],
                          condition=IfCondition(LaunchConfiguration("rviz"))))
    else:
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          condition=IfCondition(LaunchConfiguration("rviz"))))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="10.0.2.7",
                              description="레인보우 제어박스 IP(무선). 벤더 기본 10.0.2.7"),
        DeclareLaunchArgument("cb_simulation", default_value="true",
                              description="true=제어박스 Simulation 모드(팔 정지) / "
                                          "false=🔴 Real(실제 구동). ⚠ 어느 쪽이든 제어박스 접속 필요"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false",
                              description="true=mock 하드웨어(제어박스 불필요, 배선 검증용)"),
        DeclareLaunchArgument("mounts", default_value="",
                              description="mounts.yaml 경로(생략 시 설치본)"),
        DeclareLaunchArgument("controllers", default_value="",
                              description="컨트롤러 yaml(생략 시 controllers_real.yaml)"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("preflight", default_value="true",
                              description="실기 모드에서 제어박스 접속 가능 여부를 먼저 확인 "
                                          "(false 로 끄면 접속 실패 시 조용히 무한 대기)"),
        OpaqueFunction(function=_setup),
    ])
