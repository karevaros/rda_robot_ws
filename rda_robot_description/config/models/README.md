# 모델 드롭 폴더 — 코드 편집 없이 조립기에 모델 추가

## 기본 제공 모델 라이브러리 (2026-07-15 추가, 총 28종)

> **새 PC 에서는 먼저** `bash docs/scripts/setup_vendor_models.sh` **를 실행**해야 한다.
> 아래 모델들이 `pkg:` 로 참조하는 description 패키지는 `vendor/` 에 clone 되는데
> `vendor/` 는 `.gitignore` 라서, 안 돌리면 드롭다운엔 뜨는데 로드가 실패한다.
> 검증: `python3 docs/scripts/test_models.py` (28종 전부 로드 + anchor 확인)

| 슬롯 | 모델 | 라이선스 |
|------|------|----------|
| **base** | Scout 2.0(기본) · box_base(테스트) | Apache-2.0 |
| | Robotnik **RB-Theron**(저상 305mm/200kg) · RB-Summit · **RB-Kairos**(팔 탑재 전제) · RB-Vogui | BSD-3 |
| | Clearpath **Ridgeback**(옴니/100kg) · Jackal · Husky | BSD-3 |
| | Clearpath Dingo-D / Dingo-O ⚠ payload 20kg · TurtleBot4 ⚠ payload 9kg | BSD-3 / Apache-2.0 |
| **arm** | RB5-850e(기본) · **RB10-1300e**(10kg/1300mm) | Apache-2.0 |
| | **UR5e**(5kg/850mm) · **UR10e**(12.5kg/1300mm) | BSD-3 |
| | **UF850**(5kg/850mm) · **xArm6**(5kg/700mm) | BSD-3 |
| **endeffector** | OnRobot RG2(기본, 편입본) · **OnRobot RG6**(스트로크 160mm) | BSD / MIT |
| | **Robotiq 2F-140**(140mm) · **Robotiq 2F-85** | BSD-3 |
| | **Franka Hand** · **Allegro Hand V5**(4지 16DOF) | Apache-2.0 / BSD-2 |
| **sensor1/2** | RealSense D405 · D435i | Apache-2.0 |

**⚠ 적재 제약:** 현재 팔+그리퍼+센서 = **약 22~23kg**(URDF 실측). Dingo(20kg)·TurtleBot4(9kg)는
이 하중을 못 버틴다 — 구성/참고용으로만 넣어두었다.

**과일 수확 관점:** 연한 과일에 이상적인 소프트/적응형 그리퍼는 ROS2 URDF 가 사실상 없다.
현실적 최선은 스트로크가 넓은 **Robotiq 2F-140** 또는 **OnRobot RG6**.

**등록하지 않은 것과 이유**는 `docs/scripts/setup_vendor_models.sh` 하단 주석 참조
(MiR100=humble 브랜치인데 catkin · AgileX Tracer/Scout Mini/Bunker=mesh 라이선스 없음 ·
OnRobot RG2(ABC)=상류 트리 버그 · UR20/30=mesh 별도 제한 라이선스).

**통합 URDF 반영은 자동이다(2026-07-16~).** 폴더에 드롭하면 조립기 드롭다운과 통합 URDF
양쪽에 바로 들어간다 — Python 컴포저가 조립기와 **같은 모델 정의**를 읽기 때문. 재빌드도
xacro 편집도 필요 없다. 단 **팔/그리퍼를 바꾸면 `gen_srdf.py` 재실행은 여전히 필요**하다
(SRDF/ACM 은 링크 이름·형상에 묶여 있음).

> 이전에는 `rda_robot.urdf.xacro` 에 슬롯별 `xacro:if` 분기를 손으로 추가해야 했고,
> 안 하면 **에러 없이 그 슬롯이 통째로 빠졌다**(26종 중 21종이 그 상태였다).

---


이 폴더의 **슬롯 하위 폴더**(`base/ arm/ endeffector/ sensor1/ sensor2/`)에
파일을 넣으면, 조립기(`rda_robot_assembler`)가 자동으로 인식해 해당 슬롯의
드롭다운 목록에 추가합니다. **Python 코드(`part_registry.py`) 편집 불필요.**

앱 실행 중이면 왼쪽 **`🔄 모델 새로고침`**(`F5`) 버튼으로 재시작 없이 다시 스캔합니다.

> **바로가기:** 조립기 왼쪽 **`📂 모델 폴더 열기`**(`Ctrl+M`, 도구 메뉴에도 있음)를 누르면
> 이 폴더가 파일 관리자로 열립니다. **폴더 열기 → 파일 드롭 → `F5`** 순서로 쓰면 됩니다.

## 외부 3D 파일(CAD)에서 모델 만들기

> **핵심:** 3D 파일에는 **형상만** 있고 **관절 정보가 없다.** 그래서 경로가 갈린다.

| 입력 | 방법 | 관절 |
|------|------|------|
| **STL·OBJ·DAE·GLB·PLY·3MF** | `mesh2urdf` (아래) — 바로 됨 | ❌ 강체만 |
| **STEP·IGES** | `sudo apt install freecad` 후 `mesh2urdf` (자동 테셀레이션) | ❌ 강체만 |
| **SolidWorks** | [sw_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter) (Windows/SolidWorks 내부) | ✅ 보존 |
| **Fusion360** | [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) 스크립트 | ✅ 보존 |
| **Onshape** | [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) | ✅ 보존 |

**관절이 있는 파트**는 STL/STEP 으로 뽑으면 관절이 사라진다 → 위 전용 익스포터를 쓰거나,
부품별로 각각 `mesh2urdf` 한 뒤 xacro 로 joint 를 직접 엮어야 한다.

### `mesh2urdf` — 강체 파트 자동 변환
메시를 링크 1개 URDF 로 만들고 **관성(inertia)까지 계산**해 이 폴더에 바로 등록한다.
```bash
# 기본 (단위·관성·충돌메시 자동)
ros2 run rda_robot_assembler mesh2urdf bracket.stl --slot sensor1

# 이름·라벨·재질·원점 지정
ros2 run rda_robot_assembler mesh2urdf part.step --slot endeffector \
    --name tool_x --label "커스텀 툴 X" --density 7850 --origin bottom
```
자동 처리:
- **단위**: CAD 는 보통 mm → 최대 치수가 10m 를 넘으면 mm 로 보고 ×0.001. `--scale 0.001` 로 강제 가능.
- **관성**: `--density`(기본 2700 알루미늄 / 강철 7850 / 플라스틱 1200)로 계산.
  메시가 닫혀있지 않으면(watertight=False) 볼록껍질로 근사하고 **경고**를 띄운다.
- **충돌 메시**: 기본 볼록껍질(`--collision hull`) — 빠르지만 **오목한 파트는 부풀어 오른다.**
  실측(L자+구멍 브래킷): `hull` 부피가 원본의 **260%** → 실제로 안 닿는데 충돌로 잡힌다.
  1.5배를 넘으면 변환기가 **경고**하니, 그때는 `--collision same` 을 쓸 것.

  | 모드 | 면수 | 부피(원본 대비) | 쓸 때 |
  |------|------|------------------|-------|
  | `hull`(기본) | 16 | 260% | 볼록한 파트, 속도 우선 |
  | `simplify` | 1030 | 87% | 오목 + 면수 줄이고 싶을 때 |
  | `same` | 1036 | 100% | **오목한 파트 — 가장 정확** |

  `simplify` 는 `pip install --user fast_simplification` 필요. 단 **CAD 테셀레이션은 감쇠가
  잘 안 된다**(위 예: 목표 300면인데 1030면에서 멈춤 — 라이브러리 한계). 목표에 못 미치면
  경고를 띄우며, 그 경우 `same` 과 실익 차이가 거의 없다.
- **원점**: `--origin keep`(기본, 원본 유지) / `center` / `bottom`(바닥을 z=0 — 위에 얹는 파트에 편함) / `com`.

생성물: `<슬롯>/<name>.urdf` + `<name>.yaml` + `<슬롯>/meshes/<name>_{visual,collision}.stl`
→ 조립기에서 **`🔄 모델 새로고침`** 하면 드롭다운에 뜬다.

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
조립기에서 고른 모델·결합값은 `mounts.yaml` 로 저장되고, **모든 슬롯이 그대로 통합
URDF 에 반영**됩니다. launch 가 Python 컴포저를 호출해 조립기와 같은 정의로 조립하므로
앱 화면과 RViz 형상이 일치합니다.

```bash
ros2 run rda_robot_assembler compose_urdf --mounts <mounts.yaml> -o /tmp/rda_robot.urdf
check_urdf /tmp/rda_robot.urdf
python3 src/docs/scripts/test_integrated_urdf.py     # 전 모델 조립 회귀
```

**링크 이름 규칙:** 모델 링크에는 기본으로 슬롯 접두사가 붙습니다(`arm_`, `sensor1_` …).
서로 다른 모델이 흔히 같은 이름(`base_link`)을 써서 충돌하기 때문입니다. 내장
Scout/RB5/RG2 는 하위 도구(gen_srdf·kinematics·moveit)가 의존하는 `base_link`·`link0`·
`tcp`·`rg2_hand` 를 보존하려고 `prefix: ""` 로 예외 처리돼 있습니다. 모델 yaml 에서
`prefix:` 로 직접 지정할 수도 있습니다.
