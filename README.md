# rda_robot_ws — 농업(온실 토마토) 로봇 통합 제어 워크스페이스

ROS2 Humble 기반. **모바일 베이스 + 로봇팔 + 엔드이펙터 + 뎁스 카메라**를 하나의 로봇으로
조립하고, 온실 환경을 세우고, 카메라로 토마토를 찾아 집으러 가는 것까지 다룬다.
(한국기계연구원 교육파견 프로젝트)

이 문서는 **위에서부터 순서대로 복사·붙여넣기** 하면 그대로 돌아가도록 썼다.

무엇을 실행해 볼 수 있나:

| # | 실행 | 보이는 것 |
|---|------|-----------|
| 3-1 | 통합 로봇 표시 | 조립된 로봇과 온실·작물이 RViz2 에 뜨고, 관절을 움직이면 자충돌을 빨갛게 표시 |
| 3-2 | 조립기 GUI | 베이스/팔/그리퍼/센서를 드롭다운으로 갈아끼우고 결합 위치를 마우스로 조정 |
| 3-3 | MoveIt 경로계획 | 온실 장애물을 피하는 궤적 계획 |
| 3-4 | 집기 데모 | 도달 가능한 토마토를 골라 `접근 → 파지 → 후퇴` 를 애니메이션으로 재생 |
| 3-5 | Gazebo + 센싱 | 시뮬 카메라가 본 포인트클라우드가 **그대로 충돌 장애물(옥토맵)** 이 되고, 빨간 열매를 3D 로 인지 |
| 3-6 | 인지 → 집기 | yaml 좌표가 아니라 **카메라가 찾은 열매**를 목표로 집기 |

---

## 1. 요구 환경

- **Ubuntu 22.04** + **ROS2 Humble** (`ros-humble-desktop`)
- GPU 권장(RViz/Gazebo 렌더링). 개발 환경은 RTX 4060 / 드라이버 535 에서 검증했다.
- 디스크 약 3GB(벤더 모델 포함)

ROS2 가 아직 없다면 [공식 설치 문서](https://docs.ros.org/en/humble/Installation.html)를 먼저 따르고 오면 된다.

---

## 2. 설치 — 순서대로 복사해서 붙여넣기

### 2-1. 필요한 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip git \
  ros-humble-moveit ros-humble-moveit-ros-perception \
  ros-humble-urdf-launch ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  gazebo ros-humble-gazebo-ros-pkgs \
  python3-scipy python3-opencv
```

각각 왜 필요한지:

| 패키지 | 쓰이는 곳 |
|--------|-----------|
| `moveit` | 경로계획(3-3), 집기 데모(3-4) |
| `moveit-ros-perception` | **옥토맵 업데이터 플러그인** — 기본 moveit 설치엔 들어 있지 않다. 없으면 3-5 의 센싱 장애물이 아예 안 생긴다 |
| `joint-state-publisher(-gui)` | RViz 관절 슬라이더 |
| `gazebo` + `gazebo-ros-pkgs` | 시뮬레이션과 시뮬 카메라(3-5) |
| `python3-scipy`, `python3-opencv` | 열매 인지(빨강 세그멘테이션·군집화) |

### 2-2. 저장소 clone

이 저장소는 **워크스페이스의 `src` 폴더 자체**다. 그래서 `~/robot_ws/src` 로 받는다.

```bash
mkdir -p ~/robot_ws
git clone git@github.com:karevaros/rda_robot_ws.git ~/robot_ws/src
# HTTPS 를 쓴다면: git clone https://github.com/karevaros/rda_robot_ws.git ~/robot_ws/src
```

> 이 저장소는 **비공개**다. 접근 권한이 있는 계정(SSH 키 등록 또는 HTTPS 인증)으로 받아야 한다.

### 2-3. 벤더 모델 받기 (필수)

로봇 모델(Scout·RealSense·UR·xArm·Robotiq·Clearpath …)은 원저작 저장소에서 받아 쓴다.
그 폴더(`src/vendor/`)는 이 저장소에 포함돼 있지 않으니 **아래 스크립트를 반드시 한 번 돌려야
한다.** 안 돌리면 조립기 드롭다운에 이름은 보이는데 **로드가 실패**하고, 기본 로봇조차 안 뜬다.

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
bash src/docs/scripts/setup_vendor_models.sh     # clone + 벤더 description 패키지 빌드 (수 분)
```

이 스크립트가 받아오는 것(모두 **공개 저장소**이고 각 원저작자의 라이선스를 따른다):

| 받는 저장소 | 브랜치 | 라이선스 | 우리가 쓰는 것 |
|-------------|--------|----------|----------------|
| [agilexrobotics/scout_ros2](https://github.com/agilexrobotics/scout_ros2) | `humble` | Apache-2.0 | `scout_description` — **기본 모바일 베이스** |
| [RainbowRobotics/rbpodo_ros2](https://github.com/RainbowRobotics/rbpodo_ros2) | `main` | Apache-2.0 | `rbpodo_description` — **기본 로봇팔(RB5-850e)** |
| [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) | `ros2-master` | Apache-2.0 | `realsense2_description` — **기본 센서(D405/D435i)** |
| [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | `humble` | BSD-3 | UR5e·UR10e |
| [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) | `humble` | BSD-3 | xArm6·UF850 |
| [ABC-iRobotics/onrobot-ros2](https://github.com/ABC-iRobotics/onrobot-ros2) | `main` | MIT | OnRobot RG6 |
| [PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper) | `humble` | BSD-3 | Robotiq 2F-85/140 |
| [frankaemika/franka_description](https://github.com/frankaemika/franka_description) | `humble` | Apache-2.0 | Franka Hand |
| [Wonikrobotics-git/allegro_hand_ros2_v5](https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5) | `master-4finger` | BSD-2 | Allegro Hand V5 |
| [clearpathrobotics/clearpath_common](https://github.com/clearpathrobotics/clearpath_common) | `humble` | BSD-3 | Husky·Jackal·Ridgeback·Dingo |
| [RobotnikAutomation/robotnik_description](https://github.com/RobotnikAutomation/robotnik_description) · [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors) | `humble-devel` | BSD-3 | RB-Theron·RB-Kairos·RB-Vogui·RB-Summit |
| [turtlebot/turtlebot4](https://github.com/turtlebot/turtlebot4) · [iRobotEducation/create3_sim](https://github.com/iRobotEducation/create3_sim) | `humble` | Apache-2.0 / BSD-3 | TurtleBot4 |

받은 것은 `src/vendor/` 에 그대로 두고(원본 수정 없음), 우리 저장소에는 포함하지 않는다
(`.gitignore`). 즉 **벤더 코드는 각자의 저장소에서 각자의 라이선스로 받아 쓰는 구조**다.

### 2-4. 조립기용 파이썬 패키지

```bash
python3 -m pip install --user pyvista pyvistaqt trimesh yourdfpy pycollada python-fcl
```
조립기 GUI 의 3D 뷰와 실시간 자충돌 검사에 쓰인다(rosdep 으로는 안 깔린다).

### 2-5. 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
colcon build --symlink-install --packages-select \
  rda_robot_msgs rda_robot_description rda_robot_bringup \
  rda_robot_assembler rda_robot_moveit_config
```

> ⚠ **`--packages-select` 없이 그냥 `colcon build` 를 돌리지 말 것.** `src/vendor/` 의
> `xarm_sdk` 가 git submodule 을 요구해 **전체 빌드는 원래 실패**하고, 실패 하나가 의존
> 패키지 21개를 연쇄 중단시킨다(우리는 `xarm_description` 만 쓰므로 SDK 는 무관하다).
>
> ⚠ 벤더 패키지에 `--symlink-install` 을 **섞지 말 것.** 캐시가 깨져 이후 평범한 빌드까지
> 실패한다. 복구는 `rm -rf build/<패키지>` 후 재빌드.

### 2-6. 환경 소싱 — **새 터미널을 열 때마다**

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

매번 치기 귀찮으면 한 번만:
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/robot_ws/install/setup.bash' >> ~/.bashrc
```

### 2-7. 설치 확인

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
cd ~/robot_ws
python3 src/docs/scripts/test_models.py            # 조립기 모델 로드      → 28/28
python3 src/docs/scripts/test_integrated_urdf.py   # 통합 URDF 전 모델 조립 → 29/29
```

둘 다 통과하면 설치 끝이다. 두 테스트는 **서로 다른 것을 본다** — 앞은 "조립기가 모델을 읽을
수 있나", 뒤는 "그 모델이 실제로 통합 URDF 로 조립되나". 앞만 통과하고 뒤가 조용히 실패한
적이 있어서 둘 다 돌린다.

---

## 3. 실행해 보기 — 위에서부터 순서대로

아래 블록은 각각 **새 터미널에 그대로 붙여넣으면** 돌아간다(소싱 줄 포함).
GUI 창이 뜨므로 원격 접속이라면 X 포워딩이 필요하다. 종료는 `Ctrl+C`.

### 3-1. 통합 로봇을 RViz2 에 띄우기

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 launch rda_robot_description rda_robot_display.launch.py
```

조립 결과(`config/mounts.yaml`)를 읽어 통합 URDF 를 만들고 RViz2 에 표시한다. 같이 뜨는 것:

| 노드 | 역할 |
|------|------|
| `robot_state_publisher` | 통합 URDF → TF |
| `joint_state_publisher_gui` | 관절 슬라이더 (시작 자세 = `mounts.yaml` 의 `initial_pose`) |
| `self_collision_monitor` | 자기 몸끼리 부딪히면 **그 링크의 실제 mesh 를 빨갛게** 덮어 표시 |
| `obstacle_publisher` | `config/obstacles.yaml` 의 온실 구조·작물 |

슬라이더로 팔을 접어 보면 자충돌 표시를 확인할 수 있다.

```bash
# 옵션
ros2 launch rda_robot_description rda_robot_display.launch.py collision:=false   # 자충돌 모니터 끄기
ros2 launch rda_robot_description rda_robot_display.launch.py obstacles:=false   # 온실/작물 끄기
```

> RViz 의 Fixed Frame 은 `world` 다. 바닥이 z=0 이고 `base_link` 는 바닥이 아니라 z=+0.235 에
> 있다(Scout 바퀴 최저점 기준). 이 값은 상수로 박은 게 아니라 URDF 에서 매번 유도한다.

### 3-2. 조립기 GUI — 파트 갈아끼우기

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 run rda_robot_assembler assembler
```

왼쪽에서 슬롯(베이스·팔·엔드이펙터·센서1/2)별 모델을 고르고, 오른쪽에서 부착 프레임과
XYZ/RPY 를 조절하면 가운데 3D 뷰가 즉시 바뀐다. 파트끼리 겹치면 빨갛게 표시된다.
`Ctrl+S` 로 저장하면 `rda_robot_description/config/mounts.yaml` 에 기록되고,
**3-1 을 다시 실행하면 그 형상이 그대로 반영**된다(재빌드 불필요).

- 모델 라이브러리 **26종**: 베이스 12(Scout 2.0·Husky·Jackal·RB-Theron 등) · 팔 6(RB5-850e·UR5e·UR10e·UF850·xArm6·RB10-1300e) · 엔드이펙터 6(RG2·RG6·Robotiq 2F-85/140·Franka Hand·Allegro V5) · 센서 2(D405·D435i). 전부 permissive 라이선스.
- 새 모델은 `rda_robot_description/config/models/<슬롯>/` 에 파일을 떨구면 끝이다 → [`config/models/README.md`](rda_robot_description/config/models/README.md). 코드 수정도 재빌드도 필요 없다.
- 자세한 사용법 → [`rda_robot_assembler/README.md`](rda_robot_assembler/README.md)

> ⚠ **팔이나 그리퍼를 바꿨다면 SRDF 를 다시 만들어야 한다**(→ 6-2). 안 하면 MoveIt 이
> `Group state 'home' ... group 'arm' does not exist` 로 거부한다.

### 3-3. MoveIt2 — 온실 장애물을 피하는 경로계획

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 launch rda_robot_moveit_config moveit_demo.launch.py
```

RViz 의 **MotionPlanning** 패널에서 목표 자세를 마우스로 끌고 `Plan` 을 누르면 온실 구조와
작물을 피하는 궤적이 나온다.

> ⚠ **`Execute` 는 아직 동작하지 않는다** — 실제 컨트롤러가 없어 `allow_trajectory_execution=false`
> 로 명시해 두었다(6주차 통합제어에서 연결). 계획까지가 현재 범위다.

### 3-4. 집기 데모 — 토마토를 골라 접근·파지·후퇴

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py
```

지금 로봇 위치에서 **도달 가능한 토마토를 자동으로 골라**
`home → pre-grasp(열매를 바라보는 자세) → 접근 → 파지 → 후퇴 → home` 을 반복 재생한다.

접근 구간은 이 순서로 계획한다:

1. **직선 Cartesian** — 열매 앞이 뚫려 있으면 곧게 들어간다(접근각을 넓게 훑어 직선이
   통하는 각도를 먼저 찾는다).
2. **OMPL 우회** — 주 줄기가 막고 있으면 돌아 들어간다.
3. 목표 열매가 매달린 **화방대만** 충돌 예외로 둔다(수확 대상이라 스치는 게 정상).
   주 줄기·다른 화방대·거터는 장애물 그대로 → 진짜 회피다.

자주 쓰는 인자:

```bash
# 이 위치에서 어떤 열매가 닿는지만 보고 종료
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py scan_all:=true

# 특정 좌표를 목표로
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py use_yaml_target:=false target:="[0.86,0.18,0.98]"

# 로봇 위치를 임시로 옮겨 보기 (기본 auto = mounts.yaml 저장값)
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py base_x:=0.20

# 접근각별 직선 성공률 진단 / 헤드리스
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py diag_straight:=true
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py rviz:=false
```

> ⚠ 계획한 궤적을 `/joint_states` 로 흘려 **재생**하는 것이지 실제 구동이 아니다(6주차).
> ⚠ 이 팔(reach 0.93m)로는 고설 재배 토마토 74개 중 **최전열 최하단 3~5개**만 닿는다(실측).
> 더 닿게 하려면 조립기에서 로봇을 작물 쪽으로 옮겨 저장하면 된다.

### 3-5. Gazebo 시뮬 + 센싱 — 카메라가 본 것이 장애물이 된다

여기서부터는 좌표를 손으로 적어 넣은 장애물이 아니라 **카메라가 실제로 본 것**으로 돈다.

```bash
# ⓐ 시뮬만 먼저 — 온실 월드 + 로봇 + depth 카메라
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 launch rda_robot_description gazebo_sim.launch.py
```

```bash
# ⓑ 본편 — 시뮬 + MoveIt + 열매 인지 (RViz 에 옥토맵과 인지된 열매가 함께 보인다)
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 launch rda_robot_moveit_config perception_demo.launch.py
```

돌아가는 구조:

- D435i(전역, 베이스 부착)와 D405(손끝, 그리퍼 부착)가 포인트클라우드를 낸다.
- MoveIt 의 `PointCloudOctomapUpdater` 2개가 그 클라우드를 받아 **하나의 옥토맵**을 만든다
  → 그게 곧 충돌 장애물이다. `obstacles.yaml` 로 넣은 명명 객체가 **하나도 없는 상태에서도**
  계획이 막힌다(= 센싱만으로 장애물이 성립).
- `fruit_detector` 가 클라우드의 빨간 영역을 3D 구로 만들어 `/detected_fruits` 로 낸다.

| 토픽 | 내용 |
|------|------|
| `/d435i/depth/points`, `/d405/depth/points` | 포인트클라우드(각 640×480) |
| `/d435i/depth/image_raw`, `/d405/depth/image_raw` | 컬러 이미지(클라우드와 픽셀 1:1 정렬) |
| `/monitored_planning_scene` | 옥토맵이 실린 planning scene |
| `/detected_fruits` | 인지된 열매(MarkerArray — 중심·지름, world 좌표) |

확인 명령(다른 터미널에서):

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 topic hz /d435i/depth/points                                                  # 센서 입력
ros2 topic echo /monitored_planning_scene --once --field world.octomap.octomap.id   # 'OcTree' 면 옥토맵 생성됨
ros2 topic echo /detected_fruits --once | head -40                                  # 인지된 열매
```

옵션:

```bash
ros2 launch rda_robot_moveit_config perception_demo.launch.py gui:=false         # Gazebo 창 끄기
ros2 launch rda_robot_moveit_config perception_demo.launch.py rviz:=false        # 헤드리스
ros2 launch rda_robot_moveit_config perception_demo.launch.py detect:=false      # 열매 인지 끄기
ros2 launch rda_robot_moveit_config perception_demo.launch.py obstacles:=true    # 설계값 장애물도 함께(비교용)
ros2 launch rda_robot_moveit_config perception_demo.launch.py octomap_resolution:=0.02  # 복셀 더 잘게
ros2 launch rda_robot_description gazebo_sim.launch.py world:=empty               # 빈 월드
```

**열매 인지 정확도(실측, Gazebo 온실):** 중심오차 중앙값 **1.6cm** · 반경오차 **+0.1cm**(정답
3.5cm) · 센서에 실제로 보이는 열매 기준 재현율 **92%**(25개 중 23개). 전체 74개 중 나머지는
고정된 한 시점에서 가려지거나 화각 밖이다.

> 한 화방의 열매는 중심간격 6cm 인데 반경이 3.5cm — **서로 파고들어 한 덩어리로 보인다.**
> 덩어리 무게중심을 집으면 3cm 어긋나 파지에 실패하므로, 반경 사전지식 RANSAC + 반경고정
> 최소제곱으로 열매를 하나씩 분리한다.

> ⚠ 종료 후 남은 프로세스가 있으면 `ros2 launch` **부모 PID** 를 kill 할 것. `gzserver` 만
> 죽이면 `robot_state_publisher` 가 살아남아 옛 URDF 를 계속 발행하고, 다음 실행이 엉뚱한
> 로봇을 스폰한다.
> ⚠ 지금 로봇은 Gazebo 에서 `static` 이라 **팔이 물리적으로 움직이지 않는다**(손끝 카메라
> 시점 고정). 그래서 3-4 집기 데모와 이 시뮬을 **동시에 돌리면 안 된다** — TF 상으로만 팔이
> 움직여 클라우드가 엉뚱한 자세로 투영되고 옥토맵이 오염된다. 실제 관절 구동은 6주차.

### 3-6. 인지 결과로 집기 — 센싱과 계획을 잇기

3-5 ⓑ 를 띄워 둔 상태에서, 다른 터미널에:

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py target_source:=perception
```

`obstacles.yaml` 의 이름표(`fruit_r0_p3_t0_f2`)가 아니라 **카메라가 찾은 열매**(`det_13` …)를
목표로 삼는다. 실환경에는 이름표가 없으니 이쪽이 최종 형태다.

```bash
# 인지된 열매 중 어떤 게 닿는지만 확인
ros2 launch rda_robot_moveit_config pregrasp_demo.launch.py target_source:=perception scan_all:=true
```

> ⚠ 위 3-5 의 주의와 같은 이유로, 실제 로봇 관절이 도는 6주차 전까지 이 조합은 **각각
> 따로 확인**하는 용도다(시뮬 렌더는 고정, 데모는 TF 만 움직임).

---

## 4. 프로젝트 구조

| 패키지 | 내용 |
|--------|------|
| `rda_robot_description` | 모델 라이브러리(`config/models/`), mesh, 결합설정(`config/mounts.yaml`), 온실·작물 정의(`config/obstacles.yaml`), 표시·Gazebo launch |
| `rda_robot_assembler` | 조립 GUI + **통합 URDF 컴포저**(`compose_urdf`) + `mesh2urdf` |
| `rda_robot_bringup` | 자충돌 모니터, 장애물 발행, 집기 데모, Gazebo 월드 생성, **열매 인지**(`fruit_detector.py`) |
| `rda_robot_moveit_config` | MoveIt2 설정(SRDF/ACM/OMPL/3D센서) + `moveit_demo`·`pregrasp_demo`·`perception_demo` launch |
| `rda_robot_msgs` | (예정) 메시지 정의 |

데이터 흐름:

```
조립기 GUI ──저장──▶ config/mounts.yaml ──┐
                                          │  같은 모델 정의를 공유
config/models/<슬롯>/*.yaml ──────────────┤  → 앱 화면과 RViz/Gazebo 형상이 일치
                                          ▼
                            compose_urdf (Python 컴포저)
                                          │
          ┌───────────────┬───────────────┴───────┬───────────────────┐
          ▼               ▼                       ▼                   ▼
  rda_robot_display  moveit_demo          pregrasp_demo        perception_demo
    (3-1 표시)      (3-3 경로계획)         (3-4 집기)          (3-5 센싱·인지)
```

온실 구조와 작물은 `rda_robot_description/config/obstacles.yaml` **하나가 단일 진실원**이다.
RViz/MoveIt 의 planning scene(`obstacle_publisher.py`)과 Gazebo 월드(`gen_gazebo_world.py`)가
같은 파일에서 나오므로 두 화면의 좌표가 어긋나지 않는다.

**기본 파트 모델**

| 파트 | 모델 | 출처 | 라이선스 | 어디에 있나 |
|------|------|------|----------|-------------|
| 모바일 | Agilex Scout 2.0 | [agilexrobotics/scout_ros2](https://github.com/agilexrobotics/scout_ros2) | Apache-2.0 | `src/vendor/` (2-3 에서 clone) |
| 로봇팔 | Rainbow RB5-850e | [RainbowRobotics/rbpodo_ros2](https://github.com/RainbowRobotics/rbpodo_ros2) | Apache-2.0 | `src/vendor/` (2-3 에서 clone) |
| 엔드이펙터 | OnRobot RG2 | [AndrejOrsula/ur5_rg2_ign](https://github.com/AndrejOrsula/ur5_rg2_ign) 에서 추출·편입 | BSD | **이 저장소 안**(`rda_robot_description`) |
| 센서 | RealSense D405(손끝) + D435i(전역) | [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) | Apache-2.0 | `src/vendor/` (2-3 에서 clone) |

나머지 22종의 출처·라이선스는 [2-3 표](#2-3-벤더-모델-받기-필수)와
[`config/models/README.md`](rda_robot_description/config/models/README.md) 에 있다.

파트 규약 비교 → [`docs/파트규약-비교.md`](docs/파트규약-비교.md) ·
기구학 분석 → [`docs/기구학-분석.md`](docs/기구학-분석.md)

---

## 5. 온실·작물 바꾸기

`rda_robot_description/config/obstacles.yaml` 하나만 고치면 RViz 와 Gazebo 양쪽에 반영된다.
실제 구조는 이렇게 생겼다(템플릿 + 배치를 코드가 펼친다 — 열매를 손으로 나열하지 않는다):

```yaml
crops:
  template:
    stem:  {radius: 0.006, height: 2.2}      # 줄기
    truss:                                   # 화방(열매 뭉치)
      first_z: 0.10                          # 첫 화방 높이
      spacing: 0.25                          # 화방 수직간격
      count: 3                               # 주당 화방 수
      fruits_per_truss: 4
      fruit_radius: 0.035                    # 대과 토마토 Ø70mm
  span: {y_min: -1.2, y_max: 1.2, spacing: 0.4}   # 줄 길이와 주간 간격
  rows:
    - {x: 0.83,  gutter_top: 0.92}                       # 앞줄: 줄기 + 열매
    - {x: -0.83, gutter_top: 0.92, fruits_per_truss: 0}  # 뒷줄: 줄기만
```

수치를 슬라이더로 만지며 맞추고 싶다면:

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
ros2 run rda_robot_bringup crop_tuner.py
```

---

## 6. 개발자 참고

### 6-1. 통합 URDF 만들어 검사하기

```bash
source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash
DESC=~/robot_ws/src/rda_robot_description
ros2 run rda_robot_assembler compose_urdf --mounts $DESC/config/mounts.yaml -o /tmp/rda_robot.urdf
check_urdf /tmp/rda_robot.urdf
```

### 6-2. 팔/그리퍼를 바꿨을 때 — SRDF 재생성 (필수)

SRDF/ACM 은 링크 이름과 형상에 묶여 있어 자동으로 따라오지 않는다.

```bash
cd ~/robot_ws
python3 src/docs/scripts/gen_srdf.py /tmp/rda_robot.urdf \
        src/rda_robot_moveit_config/config/rda_robot.srdf 5000
```

> 벤더가 준 SRDF(`rbpodo.srdf`)는 쓰지 말 것 — `Never`(절대 충돌 안 함)로 적힌 6쌍 중 4쌍이
> 실제로는 충돌한다(`docs/기구학-분석.md` 6장). 우리 SRDF 는 URDF 에서 직접 생성한다.

### 6-3. 회귀 테스트

```bash
cd ~/robot_ws
python3 src/docs/scripts/test_models.py            # 조립기 모델 로드      → 28/28
python3 src/docs/scripts/test_integrated_urdf.py   # 통합 URDF 전 모델 조립 → 29/29
```

### 6-4. 다시 빌드 / 커밋

```bash
# 고친 패키지만
cd ~/robot_ws && colcon build --symlink-install --packages-select rda_robot_description

# 우리 패키지 전체
cd ~/robot_ws && colcon build --symlink-install --packages-select \
    rda_robot_msgs rda_robot_description rda_robot_bringup \
    rda_robot_assembler rda_robot_moveit_config

cd ~/robot_ws/src && git add -A && git commit -m "메시지" && git push origin main
```

> launch 는 설치된 share 를 읽으므로 `rda_robot_description` 은 반드시 `--symlink-install`
> 로 빌드할 것. 안 그러면 소스를 고쳐도 반영되지 않는다.

---

## 7. 문제가 생기면

| 증상 | 원인·해결 |
|------|-----------|
| 조립기 드롭다운에 모델은 보이는데 **로드 실패** | `src/vendor/` 가 없다 → **2-3** 실행 |
| `colcon build` 가 21개 패키지에서 연쇄 실패 | 인자 없이 전체를 빌드했다 → `--packages-select` 로 우리 패키지만(**2-5**) |
| 벤더를 빌드한 뒤 평범한 빌드까지 실패 | 벤더에 `--symlink-install` 을 섞었다 → `rm -rf build/<패키지>` 후 재빌드 |
| 소스를 고쳤는데 launch 에 반영이 안 됨 | `--symlink-install` 없이 빌드했다 → **6-4** |
| MoveIt 이 `group 'arm' does not exist` | 팔/그리퍼를 바꾸고 SRDF 를 안 만들었다 → **6-2** |
| RViz 에서 `Execute` 가 안 됨 | 의도된 동작. 컨트롤러 미연결(6주차 예정) |
| Gazebo 를 다시 켰더니 **엉뚱한 로봇**이 뜸 | 이전 `robot_state_publisher` 가 살아 옛 URDF 를 발행 중 → `ros2 launch` 부모 PID 를 kill |
| 옥토맵이 계속 **비어 있음** | 노드의 `use_sim_time` 이 빠졌다 → TF 시각 불일치로 클라우드가 통째로 버려진다 |
| 열매가 **하나도 검출되지 않음**(에러도 없음) | 클라우드 `rgb` 바이트 순서 문제. Gazebo 는 PCL 관례의 역순이라 기본값이 `bgr` 이다. 실기 카메라라면 `-p rgb_order:=rgb` |
| 인지는 되는데 **집으러 가지 않음** | 그 열매가 팔 도달권 밖일 수 있다 → `scan_all:=true` 로 도달 목록 확인 |

알아두면 좋은 것:

- **링크 이름 계약**: 내장 Scout/RB5/RG2 는 `base_link`·`link0`~`link6`·`tcp`·`rg2_hand` 를
  그대로 쓴다(`gen_srdf`·kinematics·MoveIt 이 의존). 그 외 모델은 슬롯 접두사가 붙는다
  (`arm_base_link`·`sensor1_camera_link` …) — 모델끼리 `base_link` 이름이 겹치기 때문이다.
- **결합값(mount)은 아직 추정치다.** 도면·실측을 반영하기 전까지 통합 로봇의 형상은 확정이
  아니고, 그 위에서 잰 도달권·경로 수치도 잠정값이다.
- **상판 적재 약 22~23kg**(URDF 실측) → Dingo(20kg)·TurtleBot4(9kg)는 참고용이지 실사용 불가.
- **옥토맵 프레임은 `base_link` 로 강제된다** — `octomap_frame` 을 지정해도 MoveIt 이 로봇
  모델 프레임으로 덮어쓴다. 베이스가 정지 상태라 지금은 무해하지만 주행이 붙으면 재검토 대상.

---

## 8. 라이선스와 출처

이 저장소에 담긴 것과 **밖에서 받아 오는 것**을 구분해 둔다.

**우리가 쓴 것** — `rda_robot_*` 5개 패키지의 코드·설정·문서. 사내/파견 과제 산출물이며
저장소는 비공개다(Apache-2.0 로 선언한 패키지가 있으나 전체 배포 정책은 미정).

**외부에서 받아 오는 로봇 모델** — `src/vendor/` 로 clone 해서 쓰고 저장소에는 넣지 않는다.
저장소 목록·브랜치·라이선스는 위 [2-3](#2-3-벤더-모델-받기-필수) 표에 있다. 원본은 수정하지
않으며, 각 저장소의 라이선스(Apache-2.0 / BSD-2·3 / MIT)를 그대로 따른다.

**저장소에 포함된 외부 산출물(추출·편입)**

| 대상 | 원출처 | 라이선스 | 처리 |
|------|--------|----------|------|
| OnRobot RG2 URDF·mesh | [AndrejOrsula/ur5_rg2_ign](https://github.com/AndrejOrsula/ur5_rg2_ign) | BSD | `ur5_rg2.urdf` 에서 RG2 부분만 추출해 `rda_robot_description` 에 편입(mimic 조인트 버그 수정). 출처는 `urdf/parts/endeffector/onrobot_rg2_macro.xacro` 머리말에 명시 |

**참고 데이터**

| 대상 | 출처 | 처리 |
|------|------|------|
| 작물 파라미터 근거(줄기 두께·화방당 열매 수 등) | AI-Hub 「지능형 스마트팜 통합 데이터(토마토)」 (dataSetSn=534) | 원본은 재배포 제약·대용량이라 **저장소에 없다**(`.gitignore`). 라벨 62,301개를 집계한 **파생 통계만** [`docs/crop_ref/AIHUB_통계.md`](docs/crop_ref/AIHUB_통계.md) 에 남겼다 |
| 온실 치수(줄 간격 0.83m·거터 상면 0.92m 등) | 대상 온실 STEP 도면 실측 | 도면 원본·추출 이미지는 저장소에 넣지 않고(`.gitignore`), 분석 스크립트와 결과 수치만 커밋 |

**의도적으로 쓰지 않은 모델**(라이선스·호환성 문제) — 사유는
[`docs/scripts/setup_vendor_models.sh`](docs/scripts/setup_vendor_models.sh) 말미에 정리해 두었다.
AgileX 저상형(Tracer·Scout Mini·Bunker)은 mesh 가 담긴 저장소에 LICENSE 파일이 없고,
MiR100 은 humble 브랜치인데도 description 이 catkin 이며, UR20/UR30 은 코드는 BSD-3 지만
mesh 에 별도 제한이 걸려 있다.

---

## 9. 진행 현황

| 주차 | 내용 | 상태 |
|------|------|------|
| 1 | 환경 구축(ROS2·워크스페이스·패키지 스캐폴드) | ✅ |
| 2 | 4개 파트 개별 로드(모바일·팔·엔드이펙터·센서) | ✅ |
| 3 | 통합 모델 — 통합 URDF·조립 GUI·모델 라이브러리 26종 | ✅ |
| 4 | 온실 환경 + 기구학 분석 + MoveIt 셋업 | ✅ |
| 5 | 경로 생성 — pre-grasp 자세 추정, 줄기 회피 접근, 집기 데모 | ✅ |
| — | Gazebo 시뮬 + 센싱 옥토맵 + 열매 인지 | ✅ |
| 6 | 통합 제어 — ros2_control 로 **실제 궤적 실행** | 예정 |
| 7 | 가상환경 컨버팅 | 예정 |
