# rda_robot_ws

ROS2 Humble 기반 **농업 로봇 통합 제어** 워크스페이스 (기계연 교육파견).
4개 파트(모바일·로봇팔·엔드이펙터·센서)를 로드 → 통합 URDF → 결합 조정 → 장애물·경로 → (예정)통합제어.

## 패키지

| 패키지 | 설명 |
|--------|------|
| `rda_robot_description` | 모델 라이브러리(`config/models/`), 파트 편입물(RG2), mesh, 결합설정(`config/mounts.yaml`), 표시 launch/rviz |
| `rda_robot_assembler` | **RDA 로봇 어셈블러** — 조립 GUI + **통합 URDF 컴포저**(`compose_urdf`) + `mesh2urdf` |
| `rda_robot_bringup` | 자충돌 모니터, 장애물 발행 |
| `rda_robot_moveit_config` | MoveIt2 설정(SRDF/ACM/OMPL) + `moveit_demo.launch.py` |
| `rda_robot_msgs` | (예정) 메시지 정의 |

## 동작 구조

```
조립기 GUI ──저장──▶ config/mounts.yaml ──┐
                                          │   같은 모델 정의(part_registry)를 공유
config/models/<슬롯>/*.yaml ──────────────┤   → 앱 화면과 RViz 형상이 일치
                                          ▼
                            compose_urdf (Python 컴포저)
                                          │
                          ┌───────────────┴───────────────┐
                          ▼                               ▼
              rda_robot_display.launch.py       moveit_demo.launch.py
                    (RViz2 표시)                   (MoveIt2 경로계획)
```

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
> ⚠ **팔/그리퍼를 바꾸면 SRDF 재생성 필수** (아래 4번). 안 하면 MoveIt 이
> `Group state 'home' ... group 'arm' does not exist` 로 거부한다.

### 4) 통합 URDF 생성/검증 · SRDF 재생성
```bash
DESC=~/robot_ws/src/rda_robot_description
ros2 run rda_robot_assembler compose_urdf --mounts $DESC/config/mounts.yaml -o /tmp/rda_robot.urdf
check_urdf /tmp/rda_robot.urdf

# 팔/그리퍼 교체 시 (SRDF/ACM 은 링크명·형상에 묶여 있음)
python3 src/docs/scripts/gen_srdf.py /tmp/rda_robot.urdf \
        src/rda_robot_moveit_config/config/rda_robot.srdf 5000
```

### 5) 회귀 테스트
```bash
python3 src/docs/scripts/test_models.py            # 조립기 모델 로드      (28/28)
python3 src/docs/scripts/test_integrated_urdf.py   # 통합 URDF 전 모델 조립 (29/29)
```
> 두 개는 **다른 것을 본다.** `test_models.py` 통과 = 조립기가 읽을 수 있다는 뜻일 뿐,
> 통합 URDF 반영을 보장하지 않는다. 실제로 그 착각으로 21종이 하루 동안 조용히 누락됐다.

### 6) 빌드 / git
```bash
cd ~/robot_ws && colcon build --packages-select rda_robot_description --symlink-install
cd ~/robot_ws && colcon build --symlink-install                          # 전체
cd ~/robot_ws/src && git add -A && git commit -m "메시지" && git push origin main
```
> launch 는 설치 share 를 읽으므로 `rda_robot_description` 은 **`--symlink-install`** 로 빌드할 것.

---

## 최초 1회 — 조립기 파이썬 의존 (rosdep 밖)
```bash
python3 -m pip install --user pyvista pyvistaqt trimesh yourdfpy pycollada python-fcl
bash src/docs/scripts/setup_vendor_models.sh    # vendor/ 는 gitignore → 새 PC 필수
# 표시 패키지(설치됨): ros-humble-{urdf-launch,joint-state-publisher,joint-state-publisher-gui}
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

## 팁
- GUI(조립기/RViz)는 **본인 터미널에서 직접** 실행하면 계속 떠 있음.
- 굳이 백그라운드로 detach: `setsid <명령> < /dev/null &`
- 노드 정리 시 `pkill -f` 가 안 먹을 때: `ps -eo pid,cmd | grep <패턴> | awk '{print $1}' | xargs kill -9`
