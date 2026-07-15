# RDA 로봇 어셈블러 (`rda_robot_assembler`)

파트(베이스·로봇팔·엔드이펙터·센서1·센서2)를 3D로 보며 **결합 위치/각도**를 맞추는
인터랙티브 조립 GUI. 설정 결과를 `mounts.yaml` 로 저장하면 `rda_robot.urdf.xacro` 가
읽어 통합 URDF에 반영된다.

> 앱 표시 이름은 **RDA 로봇 어셈블러**. ROS 패키지명(`rda_robot_assembler`)과
> 실행 명령(`ros2 run rda_robot_assembler assembler`)은 그대로 유지한다.

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
~/robot_ws/src/rda_robot_assembler/run.sh
# 또는 colcon build 후:
ros2 run rda_robot_assembler assembler
```
- PyQt5 / vtk / numpy / scipy / yaml 은 시스템에 이미 존재.

## 결과 반영
저장 위치: `rda_robot_description/config/mounts.yaml`
```bash
xacro $(ros2 pkg prefix rda_robot_description)/share/rda_robot_description/urdf/rda_robot.urdf.xacro > /tmp/rda_robot.urdf
check_urdf /tmp/rda_robot.urdf
```

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

## 알려진 사항
- 센서 슬롯은 d405/d435i 전환 가능. 그 외 슬롯은 `config/models/<슬롯>/` 폴더에
  파일을 넣으면 드롭다운에 추가된다(`config/models/README.md` 참고).
- 결합값(mount)은 도면 확정 전 육안 정렬용 — 정밀값은 실측 반영 필요.
- 파트 렌더·충돌 검사는 visual mesh 기준(collision mesh 아님 — 보수적).
