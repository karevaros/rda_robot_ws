# rda_robot_ws

ROS2 Humble 기반 **농업 로봇 통합 제어** 워크스페이스 (기계연 교육파견).
4개 파트(모바일·로봇팔·엔드이펙터·센서)를 로드 → 통합 URDF → 결합 조정 → 장애물·경로 → (예정)통합제어.

## 패키지

| 패키지 | 설명 |
|--------|------|
| `rda_robot_description` | 모델 라이브러리(`config/models/`), 파트 편입물(RG2), mesh, 결합설정(`config/mounts.yaml`), 표시 launch/rviz |
| `rda_robot_assembler` | **RDA 로봇 어셈블러** — 조립 GUI + **통합 URDF 컴포저**(`compose_urdf`) + `mesh2urdf` |
| `rda_robot_bringup` | 자충돌 모니터, 장애물 발행(`obstacle_publisher.py`), 집기 데모(`pregrasp_demo.py`), Gazebo 월드 생성(`gen_gazebo_world.py`), 열매 인지(`fruit_detector.py`) |
| `rda_robot_moveit_config` | MoveIt2 설정(SRDF/ACM/OMPL/3D센서) + `moveit_demo`·`pregrasp_demo`·`perception_demo` launch |
| `rda_robot_msgs` | (예정) 메시지 정의 |

## 동작 구조

```
조립기 GUI ──저장──▶ config/mounts.yaml ──┐
                                          │   같은 모델 정의(part_registry)를 공유
config/models/<슬롯>/*.yaml ──────────────┤   → 앱 화면과 RViz 형상이 일치
                                          ▼
                            compose_urdf (Python 컴포저)
                                          │
          ┌───────────────┬───────────────┴───────┬───────────────────┐
          ▼               ▼                       ▼                   ▼
  rda_robot_display  moveit_demo          pregrasp_demo        perception_demo
   (RViz2 표시)     (MoveIt2 경로계획)   (열매 집기 데모)   (Gazebo 센싱→옥토맵)
```

장애물·작물은 `rda_robot_description/config/obstacles.yaml` **하나가 단일 진실원**이다 —
RViz/MoveIt planning scene(`obstacle_publisher.py`)과 Gazebo 월드(`gen_gazebo_world.py`)가
같은 파일에서 나오므로 두 화면의 좌표가 어긋나지 않는다.

**모델 라이브러리 26종** — 베이스 12(Scout 2.0·Husky·Jackal·RB-Theron 등) · 팔 6(RB5-850e·UR5e·UR10e·UF850·xArm6·RB10-1300e) · 엔드이펙터 6(RG2·RG6·Robotiq 2F-85/140·Franka Hand·Allegro V5) · 센서 2(D405·D435i). 전부 permissive 라이선스.
`config/models/<슬롯>/` 에 파일을 넣으면 **조립기와 통합 URDF 양쪽에 자동 반영**된다(재빌드·코드편집 불필요). → [`config/models/README.md`](rda_robot_description/config/models/README.md)

## 기본 파트 모델 (2주차 확정)

| 파트 | 모델 | 리포 | 라이선스 |
|------|------|------|----------|
| ⓐ 모바일 | Agilex Scout 2.0 | `agilexrobotics/scout_ros2` | Apache-2.0 |
| ⓑ 로봇팔 | Rainbow RB5-850e | `RainbowRobotics/rbpodo_ros2` | Apache-2.0 |
| ⓒ 엔드이펙터 | OnRobot RG2 | `AndrejOrsula/ur5_rg2_ign`에서 추출·편입 | BSD |
| ⓓ 센서 | RealSense D405(eye-in-hand) + D435i(eye-to-hand) | `IntelRealSense/realsense-ros` | Apache-2.0 |

> 벤더 리포는 `src/vendor/`(gitignore)로 clone, 산출물만 우리 패키지에 편입.
> **새 PC 에선 먼저** `bash src/docs/scripts/setup_vendor_models.sh` — 안 돌리면 드롭다운엔 뜨는데 로드가 실패한다.
> 파트 규약 비교·통합 규격: [`docs/파트규약-비교.md`](docs/파트규약-비교.md)

---

## 주요 실행 명령어

### 0) 환경 소싱 (새 터미널마다)
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

### 1) 조립기 GUI — 모델 선택 + 결합 위치/각도 맞추기
```bash
ros2 run rda_robot_assembler assembler
# 또는
~/robot_ws/src/rda_robot_assembler/run.sh
```
좌(슬롯·모델 선택) · 중앙(3D 뷰) · 우(부착 프레임 / XYZ / RPY / 초기 포즈). `Ctrl+S` 로 저장 →
`rda_robot_description/config/mounts.yaml`. 자세한 사용법: [`rda_robot_assembler/README.md`](rda_robot_assembler/README.md)

### 2) 저장한 결과를 RViz2 에서 보기
```bash
ros2 launch rda_robot_description rda_robot_display.launch.py
```
`mounts.yaml` 을 읽어 컴포저로 통합 URDF 를 만들고 RViz2 에 표시한다. **앱 저장 후 이 명령만
다시 실행하면 반영**(colcon 재빌드 불필요). 함께 뜨는 것:

| 노드 | 역할 |
|------|------|
| `robot_state_publisher` | 통합 URDF → TF |
| `joint_state_publisher_gui` | 관절 슬라이더(시작 포즈 = `mounts.yaml` 의 `initial_pose`) |
| `self_collision_monitor` | 움직이면 자충돌 감지 → **충돌 링크의 실제 mesh 를 빨강 오버레이** |
| `obstacle_publisher` | `config/obstacles.yaml` 의 장애물 마커 |
| `static_transform_publisher` | `world→base_link` (z 오프셋을 URDF 에서 자동 유도) |

옵션:
```bash
... rda_robot_display.launch.py collision:=false      # 자충돌 모니터 끄기
... rda_robot_display.launch.py obstacles:=false      # 장애물 끄기
... rda_robot_display.launch.py mounts_file:=/경로/mounts.yaml
```
> **RViz Fixed Frame 은 `world`**. 바닥 z=0, `base_link` 는 바닥이 아니라 z=+0.23479 에 있다
> (Scout 바퀴 최저점 기준). 이 값은 상수가 아니라 URDF 에서 매번 유도한다.

### 3) MoveIt2 — 경로 계획
```bash
ros2 launch rda_robot_moveit_config moveit_demo.launch.py
```
장애물 planning scene + OMPL 경로계획. RViz 의 **MotionPlanning** 패널에서 목표 포즈를 끌어
`Plan` 을 누르면 궤적이 나온다.
> ⚠ **실행(Execute)은 아직 불가** — 컨트롤러가 없어 `allow_trajectory_execution=false`(6주차에 연결).
> ⚠ **팔/그리퍼를 바꾸면 SRDF 재생성 필수** (아래 6번). 안 하면 MoveIt 이
> `Group state 'home' ... group 'arm' does not exist` 로 거부한다.

### 4) 집기 데모 — pre-grasp 자세 추정 + 줄기 회피 접근 (5주차)
```bash
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py
```
로봇 위치(`mounts.yaml` 의 `base_placement`)에서 **도달 가능한 실제 토마토를 자동 선택**해
`home → pre-grasp(OMPL) → 접근 → 파지 → 후퇴` 를 RViz 에서 재생한다. 접근 구간은
**① 직선 Cartesian(뚫리면) → ② OMPL 우회(주 줄기 회피) → ③ 폴백** 순으로 계획하고,
목표 열매가 매달린 화방대만 ACM 에서 충돌 허용한다(주 줄기·다른 화방대·거터는 장애물 유지).

| 인자 | 뜻 |
|------|-----|
| `target_source:=perception` | 목표를 **카메라 인지 결과**(`/detected_fruits`)에서 가져옴(기본 `yaml`) |
| `scan_all:=true` | 데모 대신 **전체 열매 도달 리포트** 출력 후 종료 |
| `diag_straight:=true` | 선택 열매의 접근각별 직선 Cartesian fraction 진단 |
| `target:="[x,y,z]" use_yaml_target:=false` | 좌표를 직접 지정 |
| `base_x:` `base_y:` `base_yaw:` | 로봇 위치 임시 덮어쓰기(기본 `auto`=저장값) |
| `rviz:=false` | 헤드리스 |

> ⚠ 궤적은 `/joint_states` 재생일 뿐 **실제 execute 는 6주차**(컨트롤러 연결 후).
> ⚠ 이 팔(reach 0.93m)로는 고설 토마토 74개 중 **최전열 최하단 3~4개만** 닿는다(실측).

### 5) Gazebo 시뮬 + 센싱 옥토맵 (perception)
```bash
# ⓐ 시뮬만 — 온실 월드 + 로봇 + depth 카메라
ros2 launch rda_robot_description gazebo_sim.launch.py
#    world:=empty (빈 월드) · gui:=false (헤드리스)

# ⓑ 시뮬 + MoveIt — 카메라가 본 것이 planning scene 장애물이 된다
ros2 launch rda_robot_moveit_config perception_demo.launch.py
#    obstacles:=true (명명 장애물 병행/비교) · octomap_resolution:=0.02 · rviz:=false · gui:=false
```
D435i(eye-to-hand)와 D405(eye-in-hand)의 depth 클라우드를 MoveIt `PointCloudOctomapUpdater`
2개가 받아 **하나의 옥토맵 = 충돌객체**로 만든다. `obstacles.yaml` 로 손수 넣은 명명 객체가
**하나도 없는 상태에서도** 계획이 막힌다(= 센싱만으로 장애물이 성립).

| 토픽 | 내용 |
|------|------|
| `/d435i/depth/points` · `/d405/depth/points` | 포인트클라우드(각 640×480) |
| `/d435i/depth/image_raw` · `/d405/depth/image_raw` | 컬러 이미지(RGB, 클라우드와 1:1 정렬) |
| `/monitored_planning_scene` | 옥토맵이 실린 planning scene |
| `/detected_fruits` | **인지된 열매**(MarkerArray, world 좌표 구 — 중심·지름) |

같이 뜨는 `fruit_detector` 가 클라우드의 빨간 영역을 3D 구로 만들어 `/detected_fruits`
로 낸다(`detect:=false` 로 끌 수 있음). 한 화방의 열매는 중심간격 6cm·반경 3.5cm 로
**서로 파고들어** 한 덩어리로 보이므로, 반경 사전지식 RANSAC + 반경고정 최소제곱으로
개별 열매를 분리한다. 실측 정확도: 중심오차 중앙값 **1.6cm** · 반경오차 **+0.1cm** ·
센서에 보이는 열매 기준 재현율 **92%**(25개 중 23개).

그 결과를 집기 데모의 목표로 바로 쓸 수 있다:
```bash
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py target_source:=perception
#   yaml 이름표(fruit_r0_p3_t0_f2) 대신 센싱 결과(det_13 …)를 목표로 삼는다.
```

확인:
```bash
ros2 topic hz /d435i/depth/points
ros2 topic echo /monitored_planning_scene --once --field world.octomap.octomap.id   # OcTree
```
> **선행 설치**: `sudo apt install ros-humble-moveit-ros-perception`
> (PointCloudOctomapUpdater 플러그인. 기본 moveit 설치엔 **없다**.) + Gazebo Classic 11 + `gazebo_ros_pkgs`.
> ⚠ 시뮬 노드는 **전부 `use_sim_time=true`** 여야 한다 — 빠지면 TF 시각이 어긋나 업데이터가
> 클라우드를 통째로 버리고 **옥토맵이 조용히 빈 채로** 남는다.
> ⚠ 로봇은 Gazebo 에서 `<static>true</static>` → **팔이 움직이지 않는다**(eye-in-hand 시점 고정).
> 그래서 `pregrasp_demo`(/joint_states 재생)와 이 시뮬을 **동시에 돌리면 안 된다** — TF 상으로만
> 팔이 움직여 클라우드가 엉뚱한 자세로 투영되고 옥토맵이 오염된다. 실제 관절 구동은 6주차.
> ⚠ 종료는 `gzserver` 만 죽이지 말고 **`ros2 launch` 부모 PID** 를 kill 할 것 — 남은
> `robot_state_publisher` 가 옛 URDF 를 latched 로 계속 발행해 다음 실행이 엉뚱하게 스폰된다.

### 6) 통합 URDF 생성/검증 · SRDF 재생성
```bash
DESC=~/robot_ws/src/rda_robot_description
ros2 run rda_robot_assembler compose_urdf --mounts $DESC/config/mounts.yaml -o /tmp/rda_robot.urdf
check_urdf /tmp/rda_robot.urdf

# 팔/그리퍼 교체 시 (SRDF/ACM 은 링크명·형상에 묶여 있음)
python3 src/docs/scripts/gen_srdf.py /tmp/rda_robot.urdf \
        src/rda_robot_moveit_config/config/rda_robot.srdf 5000
```

### 7) 회귀 테스트
```bash
python3 src/docs/scripts/test_models.py            # 조립기 모델 로드      (28/28)
python3 src/docs/scripts/test_integrated_urdf.py   # 통합 URDF 전 모델 조립 (29/29)
```
> 두 개는 **다른 것을 본다.** `test_models.py` 통과 = 조립기가 읽을 수 있다는 뜻일 뿐,
> 통합 URDF 반영을 보장하지 않는다. 실제로 그 착각으로 21종이 하루 동안 조용히 누락됐다.

### 8) 빌드 / git
```bash
# 특정 패키지
cd ~/robot_ws && colcon build --packages-select rda_robot_description --symlink-install

# 우리 패키지 전체 (벤더는 건드리지 않는다 — 아래 주의 참조)
cd ~/robot_ws && colcon build --symlink-install --packages-select \
    rda_robot_msgs rda_robot_description rda_robot_bringup \
    rda_robot_assembler rda_robot_moveit_config

cd ~/robot_ws/src && git add -A && git commit -m "메시지" && git push origin main
```

> **⚠ 인자 없는 `colcon build` 를 돌리지 말 것.** 이 워크스페이스는 `src/vendor/` 에 벤더
> 리포가 들어 있고, **전체 빌드는 원래 실패한다** — `xarm_sdk` 가 git submodule 을 요구하는데
> `setup_vendor_models.sh` 는 submodule 을 받지 않는다(우리는 `xarm_description` 만 쓰고
> SDK 는 안 쓰므로 문제되지 않는다). 한 패키지가 실패하면 **의존 패키지 21개가 연쇄 중단**된다.
>
> **더 나쁜 것: 벤더 패키지에 `--symlink-install` 을 섞으면 build 캐시가 깨진다.** 벤더는
> symlink 없이 빌드돼 있어서, 전체를 `--symlink-install` 로 돌리면 "심링크 자리에 디렉터리가
> 있다"로 실패하고 **CMake 캐시에 그 설정이 남아 이후 평범한 빌드까지 실패**한다.
> 복구: `rm -rf build/<패키지>` 후 재빌드. (2026-07-16 실제로 겪음)
>
> launch 는 설치 share 를 읽으므로 `rda_robot_description` 은 **`--symlink-install`** 로 빌드할 것.

---

## 최초 1회 — 조립기 파이썬 의존 (rosdep 밖)
```bash
python3 -m pip install --user pyvista pyvistaqt trimesh yourdfpy pycollada python-fcl
bash src/docs/scripts/setup_vendor_models.sh    # vendor/ 는 gitignore → 새 PC 필수
# 표시 패키지(설치됨): ros-humble-{urdf-launch,joint-state-publisher,joint-state-publisher-gui}

# perception(5번) 을 쓸 때만
sudo apt install ros-humble-moveit-ros-perception   # 옥토맵 업데이터 플러그인(기본 moveit 에 없음)
sudo apt install gazebo ros-humble-gazebo-ros-pkgs  # Gazebo Classic 11
```

## 알아둘 것
- **링크 이름**: 내장 Scout/RB5/RG2 는 `base_link`·`link0`~`link6`·`tcp`·`rg2_hand` 를 그대로 쓴다
  (gen_srdf·kinematics·moveit 이 의존하는 계약). 그 외 모델은 슬롯 접두사가 붙는다
  (`arm_base_link`·`sensor1_camera_link` …) — 모델끼리 `base_link` 충돌이 흔하기 때문.
- **결합값(mount)은 전부 추정치** — 도면/실측 확보 전까지 통합 로봇의 형상은 확정이 아니다.
  도구와 파이프라인은 동작하지만 그 위의 수치(도달권·경로)는 잠정값이다.
- **MoveIt 은 벤더 SRDF(`rbpodo.srdf`)를 쓰지 말 것** — `Never` 6쌍 중 4쌍이 실제로 충돌한다
  (`docs/기구학-분석.md` 6장). 우리 SRDF 는 `gen_srdf.py` 가 URDF 에서 직접 생성한다.
- **상판 적재 약 22~23kg**(URDF 실측) → Dingo(20kg)·TurtleBot4(9kg)는 참고용, 실사용 불가.
- **옥토맵 프레임은 `base_link` 로 강제된다** — `octomap_frame:=world` 를 줘도 PlanningSceneMonitor 가
  로봇 모델 프레임으로 덮어쓴다. 베이스가 정지 상태라 지금은 무해하지만, **주행이 붙으면 재검토 대상**
  (지도가 로봇을 따라 움직이게 된다).
- **자기 몸 필터 잔여**: Scout 베이스는 collision 이 박스뿐인데 visual mesh(범퍼·펜더)가 더 넓어,
  카메라가 본 범퍼 일부가 복셀로 남는다. 로봇 충돌형상 밖이라 계획엔 영향 없다
  (`padding_offset` 을 더 키우면 거터 복셀까지 지워지므로 0.05 로 둔다).
- **`/check_state_validity` 에 없는 관절 이름을 넣으면 move_group 이 abort 한다**(MoveIt 이 예외를
  안 잡음). 이 팔의 관절명은 `base·shoulder·elbow·wrist1·wrist2·wrist3`.

## 팁
- GUI(조립기/RViz)는 **본인 터미널에서 직접** 실행하면 계속 떠 있음.
- 굳이 백그라운드로 detach: `setsid <명령> < /dev/null &`
- 노드 정리 시 `pkill -f` 가 안 먹을 때: `ps -eo pid,cmd | grep <패턴> | awk '{print $1}' | xargs kill -9`
