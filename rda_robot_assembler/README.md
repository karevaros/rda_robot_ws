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
파트끼리 겹치면 3D에서 **빨강**으로 표시된다. 인접 링크·마운트 접합처럼 설계상 항상
닿는 쌍은 자동으로 무시한다. 모델 자체가 겹쳐 있어 계속 경고가 뜨면
**도구 ▸ 현재 겹침 무시**로 지금 상태를 정상으로 등록하면 이후 새로 생긴 겹침만 경고한다.
(등록 내용은 세션 내에서만 유지 — yaml 에 저장되지 않음.)

## 알려진 사항
- 센서 슬롯은 d405/d435i 전환 가능. 그 외 슬롯은 `config/models/<슬롯>/` 폴더에
  파일을 넣으면 드롭다운에 추가된다(`config/models/README.md` 참고).
- 결합값(mount)은 도면 확정 전 육안 정렬용 — 정밀값은 실측 반영 필요.
- 파트 렌더·충돌 검사는 visual mesh 기준(collision mesh 아님 — 보수적).
