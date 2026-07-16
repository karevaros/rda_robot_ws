# RDA 로봇 어셈블러 (`rda_robot_assembler`)

파트(베이스·로봇팔·엔드이펙터·센서1·센서2)를 3D로 보며 **모델을 고르고 결합 위치/각도를
맞추는** 인터랙티브 조립 GUI. 결과를 `mounts.yaml` 로 저장하면 **통합 URDF 에 그대로
반영**된다(RViz2·MoveIt2).

이 패키지는 실행파일 3종을 제공한다:

| 명령 | 역할 |
|------|------|
| `ros2 run rda_robot_assembler assembler` | 조립 GUI |
| `ros2 run rda_robot_assembler compose_urdf` | **통합 URDF 컴포저** — `mounts.yaml` → 단일 URDF |
| `ros2 run rda_robot_assembler mesh2urdf` | 외부 CAD/메시 → 강체 URDF 변환 |

> 앱 표시 이름은 **RDA 로봇 어셈블러**. ROS 패키지명·실행 명령은 `rda_robot_assembler` 로 유지한다.

## 조립기와 통합 URDF 가 같은 정의를 공유한다

컴포저는 조립기와 **같은 모델 레지스트리(`part_registry`)·같은 모델 폴더**를 읽는다.
따라서 앱 화면에서 본 형상이 곧 RViz/MoveIt 형상이다.

```
config/models/<슬롯>/*.yaml ─┬─▶ 조립기 GUI ──▶ mounts.yaml ──▶ compose_urdf ──▶ 통합 URDF
     (정본 = 소스 트리)      └────────────────────────────────────┘
```

> 이전에는 통합 URDF 가 별도 xacro 였고 슬롯별 `xacro:if` 분기가 있는 모델만 반영했다.
> 분기가 없으면 **에러 없이 그 슬롯이 통째로 빠졌고**(26종 중 21종이 그 상태), 팔의 tcp 도
> 벤더 인자를 빠뜨려 96.7mm 어긋나 있었다 — 즉 **앱과 RViz 형상이 서로 달랐다.**
> 컴포저가 같은 소스를 쓰면서 이 부류가 구조적으로 사라졌다. (2026-07-16)

## 화면 구성
- **메뉴/툴바(상단)**: 파일(저장·불러오기·기본값) / 보기(축·뷰 프리셋) / 도구(자충돌·모델)
- **좌**: 파트 슬롯 — 그룹을 **클릭하면 선택**(주황 테두리), 모델 선택·`3D에 표시` 토글
- **중앙**: pyvista 내장 3D 뷰(부착 프레임 축 표시)
- **우**: 결합 설정 — 붙일 파트 / 부착 프레임 / X·Y·Z(m) / Roll·Pitch·Yaw(°), 라이브 반영
  + 관절 초기 포즈(시작 자세) 슬라이더
- **상태바(하단)**: 좌=상태·오류 메시지, 우=자충돌 상태
- 좌/우 패널은 **드래그로 폭 조절**(스플리터). 제목의 `*` = 저장 안 된 변경.

## 단축키
| 키 | 동작 | 키 | 동작 |
|----|------|----|------|
| `Ctrl+S` | mounts.yaml 저장 | `1` | 등각 보기 |
| `Ctrl+O` | 불러오기 | `2` | 앞에서 보기 |
| `A` | 부착 프레임 축 표시 | `3` | 옆에서 보기 |
| `C` | 자충돌 검사 | `4` | 위에서 보기 |
| `F5` | 모델 새로고침 | `0` | 뷰 맞춤 |
| `Ctrl+M` | 모델 폴더 열기 | | |

## 실행
```bash
# 의존(pip, rosdep 밖): 최초 1회
python3 -m pip install --user pyvista pyvistaqt trimesh yourdfpy pycollada python-fcl
# 실행
ros2 run rda_robot_assembler assembler
# 또는
~/robot_ws/src/rda_robot_assembler/run.sh
```
- PyQt5 / vtk / numpy / scipy / yaml 은 시스템에 이미 존재.

## 결과 반영 — 앱 → RViz
저장 위치: `rda_robot_description/config/mounts.yaml` (`Ctrl+S`)

```bash
# 1) 바로 RViz 로 확인 — 재빌드 불필요
ros2 launch rda_robot_description rda_robot_display.launch.py

# 2) 통합 URDF 만 뽑아 검증
ros2 run rda_robot_assembler compose_urdf \
    --mounts ~/robot_ws/src/rda_robot_description/config/mounts.yaml -o /tmp/rda_robot.urdf
check_urdf /tmp/rda_robot.urdf

# 3) MoveIt 으로 경로계획까지
ros2 launch rda_robot_moveit_config moveit_demo.launch.py
```
> **팔/그리퍼를 바꿨다면** SRDF/ACM 재생성이 필요하다(링크명·형상에 묶임):
> ```bash
> python3 src/docs/scripts/gen_srdf.py /tmp/rda_robot.urdf \
>         src/rda_robot_moveit_config/config/rda_robot.srdf 5000
> ```

## 모델 선택 — 26종
슬롯별 드롭다운에서 고르고 저장하면 통합 형상이 바뀐다. 베이스 12 · 팔 6 · 엔드이펙터 6 ·
센서 2. `config/models/<슬롯>/` 에 파일을 넣으면 자동 등록(`F5` 새로고침) —
**코드 편집도 재빌드도 필요 없다.** 스키마·목록: `rda_robot_description/config/models/README.md`

**링크 이름 규칙**: 통합 URDF 에서 모델 링크에는 기본으로 슬롯 접두사가 붙는다
(`arm_base_link`·`sensor1_camera_link` …). 서로 다른 모델이 흔히 같은 `base_link` 를 써서
충돌하기 때문. 내장 Scout/RB5/RG2 는 하위 도구(gen_srdf·kinematics·moveit)가 의존하는
`base_link`·`link0`·`tcp`·`rg2_hand` 를 보존하려고 `prefix: ""` 로 예외 처리돼 있다.

## 회귀 테스트
```bash
python3 src/docs/scripts/test_models.py            # 조립기가 모델을 로드하는가   (28/28)
python3 src/docs/scripts/test_integrated_urdf.py   # 통합 URDF 에 반영되는가      (29/29)
```
> 두 개는 **다른 것을 본다.** 조립기 로드 통과가 통합 URDF 반영을 뜻하지 않는다 —
> 그 착각으로 21종이 하루 동안 조용히 누락됐다.

## 자충돌 검사
파트끼리 실제로 mesh 가 겹치면 3D에서 **빨강**으로 표시된다. 관절로 연결된 인접 링크처럼
설계상 항상 닿는 쌍은 무시한다. 모델 자체가 겹쳐 있어 계속 경고가 뜨면
**도구 ▸ 현재 겹침 무시**로 지금 상태를 정상으로 등록하면 이후 새로 생긴 겹침만 경고한다.
(등록 내용은 세션 내에서만 유지 — yaml 에 저장되지 않음.)

- **충돌 시 표시**: 충돌 파트는 빨강. 이때 **조작 중인 파트는 불투명**으로 두고 **충돌
  상대 파트만 반투명**으로 낮춰, 상대에 파묻힌 파트가 들여다보이게 한다.
- **마운트 접합쌍은 무시하지 않는다**(2026-07-15 수정). 예전엔 무조건 무시라 팔 마운트를
  플랫폼 안으로 100mm 가까이 파묻어도 경고가 없었다. 지금은 플랜지가 상판에 얹힌 정상
  결합은 mesh 가 안 겹쳐 조용하고, 진짜로 파고들면 경고한다.
  (침투깊이 임계값 방식은 FCL 깊이값이 실제 침투량과 무관하게 널뛰어 폐기했다.)

## 3D 뷰 격자
- **XY·XZ·YZ 3평면**에 격자를 그린다. 각 평면마다 **주 1m**(진한 선) + **보조 100mm**(옅은 선).
- 격자는 **지금 조작 중인 파트의 베이스**(부모에 붙는 지점 = anchor 프레임)를 지나가게
  따라 움직인다. 슬롯을 바꾸거나 마운트 XYZ/RPY 를 만지면 격자·눈금 라벨·카메라 범위가
  즉시 그 파트 기준으로 이동한다. → 결합값을 그 파트 기준으로 바로 읽을 수 있다.
- 크기·간격은 `app.py` 의 `GRID_SPAN`(기본 2m)·`GRID_MAJOR`·`GRID_MINOR`,
  선 색은 `GRID_MAJOR_COLOR`·`GRID_MINOR_COLOR` 로 조정한다.

## 모델 추가 — 외부 3D 파일에서 변환
`mesh2urdf` 로 외부 CAD/메시를 **강체 1링크 URDF**(관성 자동 계산)로 만들어 모델 폴더에 등록한다.
```bash
ros2 run rda_robot_assembler mesh2urdf bracket.stl --slot sensor1
ros2 run rda_robot_assembler mesh2urdf part.step --slot endeffector --label "툴 X" --density 7850
```
- 입력: STL·OBJ·DAE·GLB·PLY·3MF (바로) / STEP·IGES (`sudo apt install freecad` 필요)
- 단위(mm→m)·관성·충돌메시(볼록껍질)를 자동 처리. 변환 후 앱에서 `F5`(모델 새로고침).
- **관절 있는 파트는 불가** — 메시에 관절 정보가 없다. SolidWorks/Fusion360/Onshape 전용
  URDF 익스포터를 쓸 것. 자세한 표·옵션은 `rda_robot_description/config/models/README.md`.

## 자충돌 검사는 2단계다
| 단계 | 어디서 | 무엇을 | 언제 |
|------|--------|--------|------|
| 1단계 | **조립기**(`collision.py`) | 결합값이 파트를 파묻는지 — 정지 상태 배치 검사 | 조립 중 실시간 |
| 2단계 | **RViz**(`self_collision_monitor.py`) | 관절을 움직였을 때 링크끼리 겹치는지 | `rda_robot_display.launch.py` |
| 3단계 | **MoveIt**(SRDF/ACM) | 경로계획이 충돌을 회피 | `moveit_demo.launch.py` |

## 알려진 사항
- 결합값(mount)은 도면 확정 전 **육안 정렬용 추정치** — 정밀값은 실측 반영 필요.
  통합 로봇의 형상 자체가 아직 확정이 아니다.
- 파트 렌더·충돌 검사는 visual mesh 기준(collision mesh 아님 — 보수적).
- 컴포저의 re-root 는 안전조건을 검사한다: anchor 위쪽 조인트가 **fixed** 이고 **다른 자식이
  없어야** 제거한다. 아니면 조용히 형상이 바뀌는 대신 **명확히 실패**한다.
- 새 PC 에선 `bash src/docs/scripts/setup_vendor_models.sh` 를 먼저 — `vendor/` 가 gitignore 라
  안 돌리면 드롭다운엔 뜨는데 **로드가 실패**한다.
