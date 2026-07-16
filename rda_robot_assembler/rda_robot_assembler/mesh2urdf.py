#!/usr/bin/env python3
"""외부 3D 파일(메시/STEP) → 강체 1링크 URDF 변환기.

조립기의 폴더 드롭 등록(`config/models/<슬롯>/`)에 바로 떨어지므로,
변환 후 조립기에서 '🔄 모델 새로고침' 하면 드롭다운에 뜬다.

■ 할 수 있는 것 / 없는 것
  · 관절 없는 **강체 파트**(브래킷·플레이트·센서 케이스·고정 툴) → 완전 자동.
  · **관절 있는 파트**는 불가 — 메시 파일에 관절 정보가 없기 때문.
    SolidWorks(sw_urdf_exporter)·Fusion360(fusion2urdf)·Onshape(onshape-to-robot)
    같은 전용 익스포터를 쓰거나, 부품별 메시를 각각 변환해 xacro 로 관절을 엮어야 한다.

■ 입력 포맷
  · 지금 바로: STL / OBJ / DAE / GLB·GLTF / PLY / 3MF / OFF  (trimesh)
  · STEP / IGES: FreeCAD 필요 (`sudo apt install freecad`) → 테셀레이션 후 위 경로로.

■ 자동 처리
  · 단위: CAD 는 보통 mm, URDF 는 m → 크기로 mm 추정 시 0.001 배 (`--scale` 로 강제 가능)
  · 관성(inertia): 밀도(`--density`, 기본 알루미늄 2700)로 trimesh 질량특성 계산.
    비watertight 메시는 볼록껍질로 대체 계산하고 경고한다.
  · 충돌 메시: 기본 볼록껍질(convex hull) — 원본 그대로 쓰면 면수가 많아 자충돌 검사가 느려진다.

사용:
  ros2 run rda_robot_assembler mesh2urdf bracket.stl --slot sensor1
  ros2 run rda_robot_assembler mesh2urdf part.step --slot endeffector --name tool_x \
      --label "커스텀 툴 X" --origin bottom --density 7850
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import trimesh
import yaml

from rda_robot_assembler import paths as _paths

SLOTS = ["base", "arm", "endeffector", "sensor1", "sensor2"]
CAD_EXT = {".step", ".stp", ".iges", ".igs"}
# 모델 폴더 경로는 paths.py 가 단일 정본으로 해석한다(하드코딩 금지).

# 단위 추정: 최대 치수가 이 값(m)을 넘으면 mm 로 그려진 것으로 본다.
# 로봇 파트가 10m 를 넘을 일은 없고, mm 로 그린 1m 파트는 1000 이 된다.
MM_HEURISTIC_M = 10.0


def models_dir():
    """모델 드롭 폴더(정본=소스 트리). 해석 규칙·근거는 paths.py 참조."""
    return _paths.models_dir()


def step_to_mesh(src, out_stl, deviation=0.1):
    """STEP/IGES → STL (FreeCAD 헤드리스 테셀레이션)."""
    exe = shutil.which("freecadcmd") or shutil.which("FreeCADCmd")
    if not exe:
        raise RuntimeError(
            f"STEP/IGES 를 읽으려면 FreeCAD 가 필요합니다: {src}\n"
            "  설치:  sudo apt install freecad\n"
            "  (설치 후 다시 실행하세요. 또는 원본 CAD 에서 STL 로 export 해도 됩니다.)")
    script = (
        "import Part, Mesh\n"
        f"s = Part.Shape()\n"
        f"s.read({src!r})\n"
        f"m = Mesh.Mesh(s.tessellate({deviation}))\n"
        f"m.write({out_stl!r})\n"
    )
    with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as f:
        f.write(script)
        script_path = f.name
    try:
        r = subprocess.run([exe, script_path], capture_output=True, text=True, timeout=600)
        if not os.path.exists(out_stl):
            raise RuntimeError(f"FreeCAD 변환 실패:\n{r.stdout[-500:]}\n{r.stderr[-500:]}")
    finally:
        os.unlink(script_path)
    return out_stl


def load_mesh(path):
    """입력 파일 → 단일 trimesh (STEP 은 FreeCAD 경유)."""
    path = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    ext = os.path.splitext(path)[1].lower()
    if ext in CAD_EXT:
        tmp = os.path.join(tempfile.mkdtemp(), "tess.stl")
        print(f"[STEP] FreeCAD 로 테셀레이션: {os.path.basename(path)}")
        path = step_to_mesh(path, tmp)
    m = trimesh.load(path, force="mesh")
    if not isinstance(m, trimesh.Trimesh) or m.faces is None or len(m.faces) == 0:
        raise RuntimeError(f"삼각면이 있는 메시를 읽지 못했습니다: {path}")
    return m


def resolve_scale(mesh, scale):
    """단위 스케일 결정. scale=None 이면 크기로 mm 여부 추정."""
    if scale is not None:
        return float(scale), f"사용자 지정 ×{scale}"
    ext = float(np.max(mesh.extents))
    if ext > MM_HEURISTIC_M:
        return 0.001, f"자동 추정: 최대치수 {ext:.1f} > {MM_HEURISTIC_M:.0f} → mm 로 보고 ×0.001"
    return 1.0, f"자동 추정: 최대치수 {ext:.3f} m → 이미 m 단위, ×1"


def apply_origin(mesh, mode):
    """링크 원점 기준 정렬. 반환: 적용된 이동량(설명용)."""
    lo, hi = mesh.bounds
    if mode == "keep":
        return np.zeros(3)
    if mode == "center":          # 바운딩박스 중심을 원점으로
        t = -(lo + hi) / 2.0
    elif mode == "bottom":        # XY 중심 + 바닥(min z)을 원점으로 → 위에 얹기 좋음
        t = np.array([-(lo[0] + hi[0]) / 2.0, -(lo[1] + hi[1]) / 2.0, -lo[2]])
    elif mode == "com":           # 무게중심을 원점으로
        t = -mesh.center_mass
    else:
        raise ValueError(mode)
    mesh.apply_translation(t)
    return t


def mass_properties(mesh, density):
    """질량·무게중심·관성텐서. 비watertight 면 볼록껍질로 대체."""
    src = mesh
    warn = None
    if not mesh.is_watertight:
        try:
            src = mesh.convex_hull
            warn = ("메시가 닫혀있지 않아(watertight=False) 볼록껍질로 질량특성을 계산했습니다 "
                    "— 관성값은 근사입니다.")
        except Exception:
            warn = "메시가 닫혀있지 않고 볼록껍질도 실패 — 관성은 단위행렬 근사."
            return 0.1, np.zeros(3), np.eye(3) * 1e-3, warn
    src = src.copy()
    src.density = float(density)
    m = float(src.mass)
    com = np.asarray(src.center_mass, dtype=float)
    I = np.asarray(src.moment_inertia, dtype=float)
    if not np.isfinite(m) or m <= 0:
        m = 0.1
        I = np.eye(3) * 1e-3
        warn = (warn or "") + " 질량이 비정상이라 0.1kg 로 대체."
    return m, com, I, warn


def make_collision(mesh, mode, max_faces):
    """충돌 메시 생성.

    simplify 는 실패 시 조용히 볼록껍질로 떨어지면 안 된다 — 오목 형상을 지키려고
    고른 모드인데 껍질이 나오면 거짓말이 된다(실제로 겪음: fast_simplification
    미설치 → except 로 hull 반환 → hull 과 부피가 똑같이 나옴). 명확히 실패시킨다.
    """
    if mode == "hull":
        return mesh.convex_hull
    if mode == "same":
        return mesh
    if mode == "simplify":
        if len(mesh.faces) <= max_faces:
            return mesh
        try:
            return mesh.simplify_quadric_decimation(face_count=max_faces)
        except ModuleNotFoundError as e:
            raise RuntimeError(
                f"--collision simplify 에는 fast_simplification 패키지가 필요합니다 ({e}).\n"
                "  설치:  python3 -m pip install --user fast_simplification\n"
                "  (또는 --collision hull / same 을 쓰세요.)") from e
    raise ValueError(mode)


URDF_TMPL = """<?xml version="1.0"?>
<!-- rda_robot_assembler/mesh2urdf 가 자동 생성. 원본: {src}
     강체 1링크(관절 없음). 관절이 필요하면 전용 CAD 익스포터를 쓰세요. -->
<robot name="{name}">
  <link name="{link}">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{vis}"/>
      </geometry>
      <material name="{name}_material">
        <color rgba="0.75 0.75 0.78 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{col}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="0 0 0"/>
      <mass value="{mass:.6f}"/>
      <inertia ixx="{ixx:.8f}" ixy="{ixy:.8f}" ixz="{ixz:.8f}" iyy="{iyy:.8f}" iyz="{iyz:.8f}" izz="{izz:.8f}"/>
    </inertial>
  </link>
</robot>
"""


def convert(src, slot, name=None, label=None, scale=None, density=2700.0,
            origin="keep", collision="hull", max_faces=2000, out_dir=None):
    if slot not in SLOTS:
        raise ValueError(f"슬롯은 {SLOTS} 중 하나여야 합니다: {slot}")
    name = name or os.path.splitext(os.path.basename(src))[0]
    name = "".join(c if (c.isalnum() or c == "_") else "_" for c in name)
    label = label or name

    mesh = load_mesh(src)
    n_faces0 = len(mesh.faces)

    s, why = resolve_scale(mesh, scale)
    if s != 1.0:
        mesh.apply_scale(s)
    print(f"[단위] {why}")

    t = apply_origin(mesh, origin)
    print(f"[원점] mode={origin}, 이동 {np.round(t, 4).tolist()}")

    mass, com, I, warn = mass_properties(mesh, density)
    if warn:
        print(f"[경고] {warn}")
    print(f"[관성] 밀도 {density} kg/m³ → 질량 {mass:.3f} kg, 무게중심 {np.round(com, 4).tolist()}")

    col = make_collision(mesh, collision, max_faces)
    print(f"[충돌] mode={collision}: 면 {n_faces0} → {len(col.faces)}")
    # 기본값(hull)이 이 파트에 안 맞으면 알려준다 — 오목 파트는 껍질이 크게 부풀어
    # 실제로 안 닿는데 충돌로 잡힌다(실측: L자+구멍 브래킷 = 원본의 2.6배).
    if collision == "hull" and mesh.is_watertight and mesh.volume > 0:
        r = float(col.volume) / float(mesh.volume)
        if r > 1.5:
            print(f"[경고] 볼록껍질 부피가 원본의 {r:.1f}배 — 오목한 형상입니다. "
                  f"충돌이 실제보다 크게 잡힙니다 → '--collision same' 을 고려하세요.")
    # 감쇠가 목표에 못 미치면 조용히 넘어가지 않는다(CAD 테셀레이션은 잘 안 줄어든다).
    if collision == "simplify" and len(col.faces) > max_faces * 1.5:
        print(f"[경고] 목표 {max_faces}면을 못 맞췄습니다(실제 {len(col.faces)}면). "
              f"CAD 테셀레이션은 감쇠가 막히는 경우가 많습니다 "
              f"— '--collision same'(면 {n_faces0})과 큰 차이가 없습니다.")

    root = out_dir or models_dir()
    sd = os.path.join(root, slot)
    md = os.path.join(sd, "meshes")
    os.makedirs(md, exist_ok=True)

    vis_rel = f"meshes/{name}_visual.stl"
    col_rel = f"meshes/{name}_collision.stl"
    mesh.export(os.path.join(sd, vis_rel))
    col.export(os.path.join(sd, col_rel))

    link = f"{name}_link"
    urdf_path = os.path.join(sd, f"{name}.urdf")
    with open(urdf_path, "w") as f:
        f.write(URDF_TMPL.format(
            src=os.path.basename(src), name=name, link=link, vis=vis_rel, col=col_rel,
            cx=com[0], cy=com[1], cz=com[2], mass=mass,
            ixx=I[0, 0], ixy=I[0, 1], ixz=I[0, 2], iyy=I[1, 1], iyz=I[1, 2], izz=I[2, 2]))

    yaml_path = os.path.join(sd, f"{name}.yaml")
    with open(yaml_path, "w") as f:
        yaml.safe_dump({"label": label, "file": f"{name}.urdf", "anchor": link},
                       f, allow_unicode=True, sort_keys=False)

    ext = np.round(mesh.extents, 4).tolist()
    print(f"\n✅ 변환 완료 — 크기 {ext} m")
    print(f"   URDF : {urdf_path}")
    print(f"   yaml : {yaml_path}")
    print(f"   mesh : {os.path.join(sd, vis_rel)}  (+ collision)")
    print(f"\n조립기에서 '🔄 모델 새로고침' → [{slot}] 드롭다운에 '{label}' 로 뜹니다.")
    return urdf_path


def main(argv=None):
    p = argparse.ArgumentParser(
        description="외부 3D 파일 → 강체 1링크 URDF (조립기 모델 폴더에 등록)",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("mesh", help="입력 3D 파일 (stl/obj/dae/glb/ply/3mf, step/iges는 FreeCAD 필요)")
    p.add_argument("--slot", required=True, choices=SLOTS, help="등록할 슬롯")
    p.add_argument("--name", help="모델 id (기본: 파일명)")
    p.add_argument("--label", help="드롭다운 표시 이름 (기본: name)")
    p.add_argument("--scale", type=float, help="단위 배율 강제 (mm→m 이면 0.001). 생략 시 자동 추정")
    p.add_argument("--density", type=float, default=2700.0,
                   help="밀도 kg/m³ (기본 2700=알루미늄, 강철 7850, 플라스틱 1200)")
    p.add_argument("--origin", default="keep", choices=["keep", "center", "bottom", "com"],
                   help="링크 원점 기준 (기본 keep=원본 유지)")
    p.add_argument("--collision", default="hull", choices=["hull", "simplify", "same"],
                   help="충돌 메시 (기본 hull=볼록껍질)")
    p.add_argument("--max-faces", type=int, default=2000, help="--collision simplify 목표 면수")
    p.add_argument("--out-dir", help="모델 폴더 루트 (기본 RDA_MODELS_DIR 또는 description/config/models)")
    a = p.parse_args(argv)
    try:
        convert(a.mesh, a.slot, a.name, a.label, a.scale, a.density,
                a.origin, a.collision, a.max_faces, a.out_dir)
    except Exception as e:
        print(f"\n❌ 변환 실패: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
