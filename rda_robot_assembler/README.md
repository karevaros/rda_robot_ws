# rda_robot_assembler

파트(베이스·로봇팔·엔드이펙터·센서1·센서2)를 3D로 보며 **결합 위치/각도**를 맞추는
인터랙티브 조립 GUI. 설정 결과를 `mounts.yaml` 로 저장하면 `rda_robot.urdf.xacro` 가
읽어 통합 URDF에 반영된다.

## 구성
- 좌: 파트 슬롯(모델 선택·표시 토글·결합 설정 진입)
- 중앙: pyvista 내장 3D 뷰(부착 프레임 축 표시)
- 우: 결합 설정 — 부모 파트 / 부모 TF(프레임) / X·Y·Z(m) / Roll·Pitch·Yaw(°), 라이브 반영
- 하단: `mounts.yaml` 저장 / 불러오기 / 초안값 초기화

## 실행
```bash
# 의존(pip, rosdep 밖): 최초 1회
python3 -m pip install --user pyvista pyvistaqt trimesh yourdfpy pycollada
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

## 알려진 사항
- 센서 슬롯은 d405/d435i 전환 가능(그 외 슬롯은 확정 모델 고정).
- 결합값(mount)은 도면 확정 전 육안 정렬용 — 정밀값은 실측 반영 필요.
- 파트 렌더는 visual mesh(FK zero-pose) 기준.
