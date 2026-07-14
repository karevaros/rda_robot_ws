# rda_robot_ws

ROS2 Humble 기반 **농업 로봇 통합 제어** 워크스페이스 (기계연 교육파견).
4개 파트(모바일·로봇팔·엔드이펙터·센서)를 로드 → 통합 URDF → 결합 조정 → (예정)장애물·경로·통합제어.

## 패키지

| 패키지 | 설명 |
|--------|------|
| `rda_robot_description` | 통합 URDF/xacro, 파트 편입물(RG2), mesh, 결합설정(`config/mounts.yaml`), 표시 launch/rviz |
| `rda_robot_bringup` | (예정) 실행/브링업 |
| `rda_robot_msgs` | (예정) 메시지 정의 |
| `rda_robot_assembler` | 파트 결합 인터랙티브 조립 GUI (PyQt5 + pyvista 3D) |

## 확정 파트 모델 (2주차)

| 파트 | 모델 | 리포 | 라이선스 |
|------|------|------|----------|
| ⓐ 모바일 | Agilex Scout 2.0 | `agilexrobotics/scout_ros2` | Apache-2.0 |
| ⓑ 로봇팔 | Rainbow RB5-850e | `RainbowRobotics/rbpodo_ros2` | Apache-2.0 |
| ⓒ 엔드이펙터 | OnRobot RG2 | `AndrejOrsula/ur5_rg2_ign`에서 추출·편입 | BSD |
| ⓓ 센서 | RealSense D405(eye-in-hand) + D435i(eye-to-hand) | `IntelRealSense/realsense-ros` | Apache-2.0 |

> 벤더 리포는 `src/vendor/`(gitignore)로 clone, 산출물만 우리 패키지에 편입.
> 파트 규약 비교·통합 규격: [`docs/파트규약-비교.md`](docs/파트규약-비교.md)

---

## 주요 실행 명령어

### 0) 환경 소싱 (새 터미널마다)
```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash
```

### 1) 파트 조립기 GUI — 결합 위치/각도 맞추기
```bash
~/robot_ws/src/rda_robot_assembler/run.sh
# 또는
ros2 run rda_robot_assembler assembler
```
좌(파트 슬롯) · 중앙(3D 뷰) · 우(부모TF / 거리 XYZ / 각도 RPY). 조정 후 **"mounts.yaml 저장"** →
`rda_robot_description/config/mounts.yaml`.

### 2) 저장한 결과를 RViz2 에서 보기
```bash
ros2 launch rda_robot_description rda_robot_display.launch.py
```
`config/mounts.yaml`을 읽어 통합 로봇을 RViz2에 표시(+ 관절 슬라이더).
앱 저장 후 이 명령만 다시 실행하면 반영 (**colcon 재빌드 불필요**).
다른 파일: `mounts_file:=/경로/mounts.yaml`.

### 3) 통합 URDF 수동 생성/검증
```bash
DESC=~/robot_ws/src/rda_robot_description
xacro $DESC/urdf/rda_robot.urdf.xacro mounts_file:=$DESC/config/mounts.yaml > /tmp/rda_robot.urdf
check_urdf /tmp/rda_robot.urdf
```

### 4) 빌드 / git
```bash
cd ~/robot_ws && colcon build --packages-select rda_robot_description   # 특정 패키지
cd ~/robot_ws && colcon build                                           # 전체
cd ~/robot_ws/src && git add -A && git commit -m "메시지" && git push origin main
```

---

## 최초 1회 — 조립기 파이썬 의존 (rosdep 밖)
```bash
python3 -m pip install --user pyvista pyvistaqt trimesh yourdfpy pycollada
# 표시 패키지(설치됨): ros-humble-{urdf-launch,joint-state-publisher,joint-state-publisher-gui}
```

## 팁
- GUI(조립기/RViz)는 **본인 터미널에서 직접** 실행하면 계속 떠 있음.
- 굳이 백그라운드로 detach: `setsid <명령> < /dev/null &`
