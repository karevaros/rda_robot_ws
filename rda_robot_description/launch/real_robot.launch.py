"""실기(Rainbow RB5 제어박스) 연동 bringup — 7주차 실기 적용 준비.

    ros2 launch rda_robot_description real_robot.launch.py \
        use_fake_hardware:=true                  # ① 하드웨어 없이 배선만 검증
    ros2 launch rda_robot_description real_robot.launch.py \
        robot_ip:=10.0.2.7 cb_simulation:=true   # ② 제어박스 연결·팔은 안 움직임
        # 무선 링크면 servo_t2:=0.1 을 함께 준다(0.03 은 지터에 끊긴다 — HW_PARAMS)
        # 실기 기본: speed_bar 0.2 · max_joint_velocity 0.5 rad/s(하드웨어 write 에서 자름 — 속도바는 servo_j 에 안 먹는다)
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

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
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


#: 벤더 패치로 생긴 하드웨어 파라미터 — (최소, 최대, 설명). 최대 None = 상한 없음.
HW_PARAMS = {
    # 🔴 (2026-10-29 실측) 벤더 servo_j 유지시간 t2=0.03s 는 무선 링크에서 모자라다 —
    #    송신 간격이 10ms 로 일정해도 도착 지터로 서보가 끝나 13~16초 만에
    #    "Unexpected robot state changed" 로 하드웨어가 내려간다. 0.1 이면 60초 끊김 0.
    "servo_t2": (0.02, 0.2, "servo_j 유지시간[s]"),
    # 생성자가 접속 직후 1.0(100%)을 건다. 펜던트·move_j 계열에만 먹고
    # 🔴 servo_j 스트림엔 안 먹는다(5%·20%·100% 모두 같은 궤적, 실측) → 보조 수단일 뿐.
    "speed_bar": (0.0, 1.0, "제어박스 속도바"),
    # 실제 속도 상한 — 로봇으로 나가는 마지막 단계(write)에서 한 주기 이동량을 자른다.
    "max_joint_velocity": (0.0, None, "관절 속도 상한[rad/s], 0=끔"),
    # 🔴 (실측) 그리퍼 gripper_macro 도 같은 명령 채널(eval)로 나가 servo 모드를 끝낸다 →
    #    송신 44ms 뒤 "Unexpected robot state changed" 로 하드웨어 사망. eval 직후 이 시간 동안은
    #    robot_state 1 복귀를 허용하고 다음 servo_j 로 서보를 다시 연다. 빈 값 = 패치 기본 1.0s.
    "script_grace": (0.0, 10.0, "eval(그리퍼 등) 직후 상태변화 유예[s]"),
}


def _inject_hw_params(urdf_xml, values):
    """벤더 하드웨어 블록에 패치 파라미터(<param>)를 넣는다.

    ⚠ 벤더 원본은 모르는 파라미터를 **조용히 무시**한다 → 설치된 플러그인에 해당 패치가
    들어 있는지 먼저 확인하고, 없으면 멈춘다(이 프로젝트의 '조용한 무효' 패턴)."""
    lib = os.path.join(get_package_prefix("rbpodo_hardware"), "lib", "librbpodo_hardware.so")
    with open(lib, "rb") as f:
        blob = f.read()

    tag = "<plugin>rbpodo_hardware/RBPodoHardwareInterface</plugin>"
    if urdf_xml.count(tag) != 1:
        raise RuntimeError("통합 URDF 에서 rbpodo 하드웨어 블록을 하나로 특정하지 못했습니다.")

    lines = []
    for name, raw in values.items():
        lo, hi, desc = HW_PARAMS[name]
        try:
            v = float(raw)
        except ValueError:
            raise RuntimeError(f"{name} 는 숫자여야 합니다: {raw!r}")
        if v < lo or (hi is not None and v > hi):
            raise RuntimeError(f"{name}({desc}) 범위 밖: {v} (허용 {lo}~{hi if hi is not None else '∞'})")
        if name.encode() not in blob:
            raise RuntimeError(
                f"설치된 rbpodo_hardware 에 '{name}' 패치가 없습니다 — 값이 조용히 무시됩니다.\n"
                "  bash src/docs/scripts/setup_rbpodo_sdk.sh 로 패치·재빌드하세요.")
        print(f"[real_robot] {name} = {v}  ({desc})")
        lines.append(f'\n      <param name="{name}">{v}</param>')
    return urdf_xml.replace(tag, tag + "".join(lines))


#: 손목 카메라(D405) — 통합 URDF 의 sensor1 슬롯 프레임 이름(`sensor1_camera_*`)과 같게 둔다.
#  드라이버는 `<camera_name>_link/_depth_optical_frame/…` 로 frame_id 를 짓는다 → 이름이 같으면 연결된다.
#  🔴 publish_tf:=false — 드라이버도 link→depth/color→optical 4개 변환을 같은 이름으로 내므로
#     URDF(robot_state_publisher)와 **같은 연결을 두 곳에서 발행**하게 된다. 차이는 link→color
#     0.24°·0.01mm(장치 보정값, 2026-10-29 실측)뿐이라 URDF 쪽을 쓴다(Gazebo 와 같은 프레임 구조).
CAMERA_NAME = "sensor1_camera"

#: 이 launch 가 띄우는 노드 — 같은 이름이 이미 떠 있으면 시작하지 않는다.
OWN_NODES = ("robot_state_publisher", "controller_manager", "joint_state_publisher",
             "gripper_adapter", "cb_safety_publisher", "safety_monitor", CAMERA_NAME)


def _check_no_stale_nodes():
    """앞 실행의 노드가 남아 있으면 멈춘다.

    🔴 왜 필요한가 (2026-10-29 실측 — 리얼 모드 첫 구동 직전): 앞 시험의 `ros2 launch` 부모만
    죽고 자식(robot_state_publisher 등)이 고아로 남아 있었다. 새 controller_manager 는
    `/robot_description` 토픽(TRANSIENT_LOCAL)을 구독하므로 **남아 있던 옛 URDF 를 받아**
    cb_simulation:=false 로 띄웠는데도 `mode: Simulation`·`max_joint_velocity 0.010` 으로 접속했다.
    이번엔 안전한 쪽(Simulation)으로 틀렸지만, 반대 방향이면 시뮬 설정을 믿고 실기가 움직인다."""
    try:
        out = subprocess.check_output(["ros2", "node", "list", "--no-daemon", "--spin-time", "1"],
                                      text=True, stderr=subprocess.DEVNULL, timeout=20)
    except (subprocess.SubprocessError, FileNotFoundError):
        print("[real_robot] ⚠ 기존 노드 확인 실패 — 남은 노드가 없는지 직접 확인하세요(ros2 node list).")
        return
    stale = sorted({l.strip() for l in out.splitlines()
                    if l.strip().rsplit("/", 1)[-1] in OWN_NODES})
    if stale:
        raise RuntimeError(
            "앞 실행의 노드가 남아 있습니다 — 새 하드웨어가 옛 설정(/robot_description)을 받을 수 있습니다:\n  "
            + "\n  ".join(stale)
            + "\n  정리: pkill -TERM -f '[r]obot_state_publisher|[r]os2_control_node|[j]oint_state_publisher"
              "|[g]ripper_adapter|[c]b_safety_publisher|[s]afety_monitor'"
              "\n  (확인을 끄려면 preflight:=false — 권장하지 않음)")


def _setup(context, *args, **kwargs):
    cfg = lambda n: LaunchConfiguration(n).perform(context)

    share = get_package_share_directory(PKG)
    mounts = cfg("mounts") or os.path.join(share, "config", "mounts.yaml")
    controllers = cfg("controllers") or os.path.join(share, "config", "controllers_real.yaml")

    fake = cfg("use_fake_hardware").lower() in ("true", "1")
    if cfg("preflight").lower() in ("true", "1"):
        _check_no_stale_nodes()
        if not fake:
            _preflight(cfg("robot_ip"))

    urdf_xml = compose_urdf(mounts, {
        "robot_ip": cfg("robot_ip"),
        "cb_simulation": cfg("cb_simulation"),
        "use_fake_hardware": cfg("use_fake_hardware"),
    })

    if not fake:
        hw = {k: cfg(k) for k in HW_PARAMS if cfg(k)}
        if hw:
            urdf_xml = _inject_hw_params(urdf_xml, hw)

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

        # ③ 팔 상태 + 그리퍼 어댑터 상태 + URDF 나머지 관절 기본값을 합쳐 /joint_states 발행
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             parameters=[{"source_list": ["arm/joint_states",
                                          "gripper_controller/joint_states"],
                          "rate": 30, "use_sim_time": False}], output="log"),

        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
             output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=["arm_controller", "-c", "/controller_manager"],
             output="screen"),

        # ④ 그리퍼 어댑터 — 실기 RG2 는 ros2_control 관절이 아니라 제어박스 명령 소관.
        #    시뮬의 gripper_controller 와 **같은 이름의 액션 서버**를 흉내내므로
        #    수확 시퀀스(pregrasp_demo)는 코드 변경 없이 그대로 동작한다.
        #    ⚠ 명령 문자열은 미검증 추정값이다(gripper_adapter.py 상단 주석 참조).
        Node(package="rda_robot_bringup", executable="gripper_adapter.py",
             name="gripper_adapter", namespace="gripper_controller", output="screen",
             parameters=[{"dry_run": LaunchConfiguration("gripper_dry_run"),
                          "use_sim_time": False}],
             condition=IfCondition(LaunchConfiguration("gripper"))),
    ]

    # ⑤ 실행 감시 — 발행 노드와 감시 노드는 **한 단위**다.
    #    감시만 켜면 상태가 안 들어와 결함으로 잡히고(require_state), 발행만 켜면
    #    아무도 반응하지 않는다. 그래서 인자 하나(safety)로 둘을 함께 켠다.
    #    기본 auto(실기면 켬) — cb_safety_publisher 는 데이터 채널에 **두 번째 연결**을 연다.
    #       2026-10-29 실기(rb850_old)에서 동시 접속 확인(교대 70초×6회 끊김·지연 0) 후 켰다.
    #       fake 에선 제어박스가 없어 상태가 안 들어오므로(require_state → 결함) 끈다.
    safety = cfg("safety").lower()
    if safety == "auto":
        safety = "false" if fake else "true"
    if safety in ("true", "1"):
        if fake:
            print("[real_robot] ⚠ safety:=true 인데 use_fake_hardware 입니다 — "
                  "제어박스가 없어 안전 상태가 들어오지 않습니다(감시는 결함으로 판정).")
        nodes += [
            Node(package="rda_robot_bringup", executable="cb_safety_publisher",
                 name="cb_safety_publisher", output="screen",
                 parameters=[{"robot_ip": cfg("robot_ip"), "use_sim_time": False}],
                 remappings=[("~/safety_state", "/safety_monitor/safety_state")]),
            Node(package="rda_robot_bringup", executable="safety_monitor.py",
                 name="safety_monitor", output="screen",
                 parameters=[{"use_sim_time": False}]),
        ]
    else:
        print("[real_robot] ⚠ 실행 감시가 꺼져 있습니다(safety:=false) — "
              "충돌·E-stop 시 자동 정지하지 않습니다.")

    if cfg("camera").lower() in ("true", "1"):
        rs_args = {"camera_name": CAMERA_NAME, "camera_namespace": "camera",
                   "publish_tf": "false", "pointcloud.enable": "true",
                   "align_depth.enable": "true"}
        if cfg("camera_serial"):
            rs_args["serial_no"] = f"'{cfg('camera_serial')}'"
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")),
            launch_arguments=rs_args.items()))
        print(f"[real_robot] 손목 카메라 = realsense2_camera '{CAMERA_NAME}' "
              f"(TF 는 URDF 가 낸다 · 토픽 /camera/{CAMERA_NAME}/…)")

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
        DeclareLaunchArgument("gripper", default_value="true",
                              description="그리퍼 어댑터 실행(제어박스 명령으로 RG2 제어)"),
        DeclareLaunchArgument("gripper_dry_run", default_value="false",
                              description="true=그리퍼 명령을 보내지 않고 로그만 "
                                          "(use_fake_hardware 로 배선 볼 때 함께 켠다)"),
        DeclareLaunchArgument("safety", default_value="auto",
                              description="실행 감시(제어박스 충돌·E-stop·ROS 하드웨어 비활성 → 즉시 정지). "
                                          "auto=실기면 켬·fake 면 끔. 데이터 채널 동시 접속은 "
                                          "2026-10-29 실기에서 확인(교대 6회 끊김 0)"),
        DeclareLaunchArgument("servo_t2", default_value="",
                              description="servo_j 유지시간[s] 0.02~0.2(빈 값=SDK 기본 0.03). "
                                          "무선 링크면 0.1 — 0.03 은 지터에 끊긴다(2026-10-29 실측). "
                                          "벤더 패치 필요(setup_rbpodo_sdk.sh)"),
        DeclareLaunchArgument("speed_bar", default_value="0.2",
                              description="제어박스 속도바 0~1(빈 값=벤더 1.0). ⚠ servo_j 스트림엔 "
                                          "안 먹는다(실측) — 펜던트·move_j 계열용 보조 수단"),
        DeclareLaunchArgument("max_joint_velocity", default_value="0.5",
                              description="관절 속도 상한[rad/s](0=끔 · RB5 제원 3.14). 로봇으로 "
                                          "나가는 명령을 하드웨어 write 에서 자른다. 벤더 패치 필요"),
        DeclareLaunchArgument("script_grace", default_value="",
                              description="그리퍼 등 스크립트 직후 서보 끊김 유예[s](빈 값=패치 기본 1.0)"),
        DeclareLaunchArgument("camera", default_value="false",
                              description="손목 RealSense D405 드라이버를 함께 띄운다(realsense2_camera, "
                                          "벤더 소스 빌드 필요) — 프레임은 URDF sensor1_camera_* 사용"),
        DeclareLaunchArgument("camera_serial", default_value="",
                              description="카메라 시리얼(여러 대일 때). 비우면 첫 장치"),
        DeclareLaunchArgument("preflight", default_value="true",
                              description="실기 모드에서 제어박스 접속 가능 여부를 먼저 확인 "
                                          "(false 로 끄면 접속 실패 시 조용히 무한 대기)"),
        OpaqueFunction(function=_setup),
    ])
