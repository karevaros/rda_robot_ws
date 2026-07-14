# 모델 드롭 폴더 — 코드 편집 없이 조립기에 모델 추가

이 폴더의 **슬롯 하위 폴더**(`base/ arm/ endeffector/ sensor1/ sensor2/`)에
파일을 넣으면, 조립기(`rda_robot_assembler`)가 자동으로 인식해 해당 슬롯의
드롭다운 목록에 추가합니다. **Python 코드(`part_registry.py`) 편집 불필요.**

앱 실행 중이면 왼쪽 **`🔄 모델 새로고침`** 버튼으로 재시작 없이 다시 스캔합니다.

## 두 가지 넣는 방법

### 1) 무설정 드롭 — URDF/xacro 파일만
`arm/` 에 `my_arm.urdf` 또는 `my_arm.xacro` 를 그냥 넣습니다.
- 라벨 = 파일명, `anchor` = 루트 링크(자동).
- mesh 는 `package://<빌드된 패키지>/...` 참조를 권장(colcon build 되어 있어야 함).
  URDF 옆에 상대경로로 둔 mesh 도 인식합니다(`meshes/foo.stl` 등).

### 2) yaml 디스크립터 — 세밀 설정
`anchor`(부착 프레임)나 `args`, 예쁜 라벨이 필요하면 `.yaml` 을 넣습니다.

```yaml
# endeffector/my_gripper.yaml
label: "제조사 그리퍼 X"     # 생략 시 파일명
# ── 소스: 둘 중 하나 ──
file: my_gripper.urdf        # 같은 폴더 파일(또는 절대경로)
# 또는 ↓ 빌드된 패키지 참조
# pkg: mygripper_description
# xacro: urdf/my_gripper.urdf.xacro
args: {}                     # (선택) xacro 인자, 예: {use_nominal_extrinsics: "true"}
anchor: gripper_base         # (선택) 부모에 붙는 프레임, 생략 시 루트 링크
```
같은 이름의 `.urdf`/`.xacro` 가 함께 있으면 yaml 설정이 우선합니다.

## anchor 정하는 법
`anchor` = 이 파트가 **부모에 맞닿는 링크**(mount joint 의 child).
- 베이스/팔: 보통 루트 링크가 접점 → **생략(=None)**.
- 그리퍼: 자체 `world` 루트를 갖는 모델이 많음 → **손바닥 링크명**을 지정.
- 애매하면 `check_urdf my_model.urdf` 로 트리를 보고 부모에 닿는 링크를 고릅니다.

## 폴더 위치 바꾸기
환경변수로 다른 폴더를 쓸 수 있습니다:
```bash
export RDA_MODELS_DIR=~/my_models   # 하위에 base/ arm/ ... 를 두면 됨
```

## 통합(최종) 로봇에 반영
조립기는 **설계·정렬 도구**입니다. 여기서 고른 모델·결합값은 `mounts.yaml` 로
저장되지만, 최종 통합 URDF(`urdf/rda_robot.urdf.xacro`)의 base/arm/그리퍼는
아직 별도 include 로 되어 있어, 실제 교체 시 그 파일도 함께 수정해야 합니다.
(센서는 mounts.yaml 로 자동 스위칭됨.)
