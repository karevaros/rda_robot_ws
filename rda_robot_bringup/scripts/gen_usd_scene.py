#!/usr/bin/env python3
"""통합 URDF + obstacles.yaml → USD 씬 (Isaac Sim 로드용) — 7주차 M4

**이 PC 에는 Isaac Sim 을 설치하지 않는다**(사용자 결정 2026-08-14). 대신 Isaac 이
그대로 열 수 있는 USD 파일만 만들어 넘긴다. 그래서 이 스크립트는 Isaac 런타임에
의존하지 않고 **Pixar USD(pip `usd-core`)만으로** 씬을 직접 작성한다.

  rda_robot.usd    로봇 — UsdPhysics 아티큘레이션(강체·관절·질량/관성·충돌)
  greenhouse.usd   온실 — obstacles.yaml 을 그대로 primitive 정적 충돌체로
  rda_scene.usd    위 둘을 참조 + PhysicsScene + 조명. **Isaac 에서 여는 파일.**

단일 진실원 유지: 형상·좌표는 `gen_gazebo_world.py` 와 **같은 소스**(통합 URDF ·
obstacles.yaml + expand_crops)에서 나온다. Gazebo 월드와 USD 씬이 자동으로 정합된다.

## 링크 → 강체 변환 규칙 (fixed 관절 병합)
URDF 44링크 중 질량이 있는 건 16개뿐이고, 나머지는 센서 광학 프레임 같은 표식이다.
USD/PhysX 에서 **강체 prim 안에 강체 prim 을 두는 건 무효**이므로 fixed 관절로 이어진
링크들을 하나의 **클러스터=강체 1개**로 합치고(질량·관성은 평행축 정리로 합성),
합쳐진 링크는 강체 밑의 **빈 Xform(프레임 표식)** 으로 남겨 좌표는 잃지 않는다.
가동 관절(revolute/prismatic)은 클러스터 사이를 잇는다.

## 알려진 한계 (README 에도 그대로 적는다)
  · `rg2_finger_joint2` 의 mimic 은 USD 표준에 없다 → 관절 2개를 독립으로 내보낸다.
    Isaac 에서 mimic joint 로 묶거나 컨트롤러가 같은 값을 주어야 한다.
  · 드라이브 게인(stiffness/damping)은 **플레이스홀더**. maxForce 만 URDF effort
    (2026-07-30 유도값)에서 온다.
  · 관절 속도 한계는 UsdPhysics 표준 속성이 아니라 생략(Isaac 측에서 설정).
  · 시각 재질은 mesh 파일의 대표색만 옮긴다(텍스처 미지원).

사용:
  ros2 run rda_robot_bringup gen_usd_scene.py --out ~/robot_ws/export/isaac
  (URDF 를 안 주면 mounts.yaml 로 통합 URDF 를 그 자리에서 조립한다)
"""
import argparse
import importlib.util
import math
import os
import re
import shutil
import subprocess
import sys

import numpy as np
import yaml

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, Vt

_here = os.path.dirname(os.path.abspath(__file__))


def _import_sibling(mod_name, file_name):
    p = os.path.join(_here, file_name)
    spec = importlib.util.spec_from_file_location(mod_name, p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ─────────────────────────────────────────────────────────────── 기하 유틸
def _rpy_to_R(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(rpy)
    T[:3, 3] = xyz
    return T


def _quat(R):
    """회전행렬(열벡터 규약) → (w, x, y, z). USD Quat 과 같은 의미."""
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def _gfq(q):
    return Gf.Quatf(float(q[0]), Gf.Vec3f(float(q[1]), float(q[2]), float(q[3])))


def _set_xform(prim, T, scale=None):
    """Xform prim 에 4x4 변환을 translate+orient(+scale) 로 기록(USD 표준 TRS 순서)."""
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in T[:3, 3]]))
    x.AddOrientOp().Set(_gfq(_quat(T[:3, :3])))
    if scale is not None:
        x.AddScaleOp().Set(Gf.Vec3f(*[float(s) for s in scale]))


def _sane(name):
    """USD prim 이름 규칙(영숫자·_, 숫자로 시작 금지)."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", str(name))
    return s if not s[0].isdigit() else "_" + s


# ─────────────────────────────────────────────────────────────── URDF 파싱
def parse_urdf(path):
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()

    def _geoms(elem, tag):
        out = []
        for g in elem.findall(tag):
            o = g.find("origin")
            xyz = [float(v) for v in (o.get("xyz").split() if o is not None and o.get("xyz") else [0, 0, 0])]
            rpy = [float(v) for v in (o.get("rpy").split() if o is not None and o.get("rpy") else [0, 0, 0])]
            ge = g.find("geometry")
            if ge is None or len(ge) == 0:
                continue
            shp = ge[0]
            d = dict(T=_tf(xyz, rpy), kind=shp.tag)
            if shp.tag == "mesh":
                d["uri"] = shp.get("filename")
                sc = shp.get("scale")
                d["scale"] = [float(v) for v in sc.split()] if sc else [1.0, 1.0, 1.0]
            elif shp.tag == "box":
                d["size"] = [float(v) for v in shp.get("size").split()]
            elif shp.tag == "cylinder":
                d["radius"] = float(shp.get("radius"))
                d["length"] = float(shp.get("length"))
            elif shp.tag == "sphere":
                d["radius"] = float(shp.get("radius"))
            else:
                continue
            mat = g.find("material")
            if mat is not None and mat.find("color") is not None:
                d["rgba"] = [float(v) for v in mat.find("color").get("rgba").split()]
            out.append(d)
        return out

    links = {}
    for le in root.findall("link"):
        n = le.get("name")
        d = dict(visual=_geoms(le, "visual"), collision=_geoms(le, "collision"), inertial=None)
        ie = le.find("inertial")
        if ie is not None and ie.find("mass") is not None:
            o = ie.find("origin")
            xyz = [float(v) for v in (o.get("xyz").split() if o is not None and o.get("xyz") else [0, 0, 0])]
            rpy = [float(v) for v in (o.get("rpy").split() if o is not None and o.get("rpy") else [0, 0, 0])]
            it = ie.find("inertia")
            I = np.zeros((3, 3))
            if it is not None:
                ixx, iyy, izz = (float(it.get(k, 0.0)) for k in ("ixx", "iyy", "izz"))
                ixy, ixz, iyz = (float(it.get(k, 0.0)) for k in ("ixy", "ixz", "iyz"))
                I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
            d["inertial"] = dict(mass=float(ie.find("mass").get("value")),
                                 com=np.array(xyz), R=_rpy_to_R(rpy), I=I)
        links[n] = d

    joints = {}
    for je in root.findall("joint"):
        o = je.find("origin")
        xyz = [float(v) for v in (o.get("xyz").split() if o is not None and o.get("xyz") else [0, 0, 0])]
        rpy = [float(v) for v in (o.get("rpy").split() if o is not None and o.get("rpy") else [0, 0, 0])]
        ax = je.find("axis")
        axis = [float(v) for v in (ax.get("xyz").split() if ax is not None and ax.get("xyz") else [1, 0, 0])]
        lim = je.find("limit")
        mim = je.find("mimic")
        joints[je.get("name")] = dict(
            type=je.get("type"), parent=je.find("parent").get("link"),
            child=je.find("child").get("link"), T=_tf(xyz, rpy), axis=np.array(axis),
            lower=float(lim.get("lower", 0.0)) if lim is not None else None,
            upper=float(lim.get("upper", 0.0)) if lim is not None else None,
            effort=float(lim.get("effort", 0.0)) if lim is not None else 0.0,
            mimic=(mim.get("joint") if mim is not None else None))
    return links, joints


def link_world_tf(links, joints):
    """관절값 0 에서 각 링크의 world(=root 링크) 변환."""
    child_of = {j["child"]: (n, j) for n, j in joints.items()}
    roots = [n for n in links if n not in child_of]
    if len(roots) != 1:
        raise SystemExit(f"[gen_usd_scene] 루트 링크가 1개가 아니다: {roots}")
    W = {roots[0]: np.eye(4)}

    def _walk(name):
        for jn, j in joints.items():
            if j["parent"] == name:
                W[j["child"]] = W[name] @ j["T"]
                _walk(j["child"])
    _walk(roots[0])
    missing = set(links) - set(W)
    if missing:
        raise SystemExit(f"[gen_usd_scene] 트리에 안 붙은 링크: {sorted(missing)}")
    return roots[0], W


def clusters(links, joints):
    """fixed 관절로 연결된 링크 묶음 → {대표링크: [링크들]}, {링크: 대표}."""
    parent = {n: n for n in links}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    order = list(links)                       # 대표는 트리에서 위쪽(먼저 나온) 링크
    rank = {n: i for i, n in enumerate(order)}
    for j in joints.values():
        if j["type"] != "fixed":
            continue
        a, b = find(j["parent"]), find(j["child"])
        if a == b:
            continue
        hi, lo = (a, b) if rank[a] < rank[b] else (b, a)
        parent[lo] = hi
    groups = {}
    for n in links:
        groups.setdefault(find(n), []).append(n)
    return groups, {n: find(n) for n in links}


def cluster_inertial(members, links, W, T_root_cluster):
    """클러스터 질량·질량중심·관성(클러스터 프레임 기준). 없으면 None."""
    Minv = np.linalg.inv(T_root_cluster)
    tot, coms, Is = 0.0, [], []
    for n in members:
        ine = links[n]["inertial"]
        if not ine:
            continue
        T = Minv @ W[n]                        # 링크 → 클러스터 프레임
        R = T[:3, :3]
        c = (T @ np.append(ine["com"], 1.0))[:3]
        Rl = R @ ine["R"]                      # 관성 텐서의 축까지 회전
        I = Rl @ ine["I"] @ Rl.T
        tot += ine["mass"]
        coms.append((ine["mass"], c))
        Is.append((ine["mass"], c, I))
    if tot <= 0.0:
        return None
    C = sum(m * c for m, c in coms) / tot
    Itot = np.zeros((3, 3))
    for m, c, I in Is:                          # 평행축 정리로 클러스터 질량중심 기준 합성
        d = c - C
        Itot += I + m * (float(d @ d) * np.eye(3) - np.outer(d, d))
    evals, evecs = np.linalg.eigh(Itot)
    if np.linalg.det(evecs) < 0:                # 오른손 좌표계 보장
        evecs[:, 0] *= -1.0
    return dict(mass=tot, com=C, diag=np.clip(evals, 0.0, None), axes=_quat(evecs))


# ─────────────────────────────────────────────────────────────── mesh
def resolve_pkg(uri, extra_paths):
    """package://pkg/rel → 실제 경로. ament 인덱스 → 소스트리 순으로 찾는다."""
    m = re.match(r"package://([^/]+)/(.+)", uri)
    if not m:
        return uri if os.path.exists(uri) else None
    pkg, rel = m.group(1), m.group(2)
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory(pkg), rel)
        if os.path.exists(p):
            return p
    except Exception:                            # noqa: BLE001 — ament 없이도 동작해야 한다
        pass
    for base in extra_paths:
        p = os.path.join(base, pkg, rel)
        if os.path.exists(p):
            return p
    return None


def write_mesh(stage, path, mesh_file, scale, rgba):
    import trimesh
    m = trimesh.load(mesh_file, force="mesh", process=False)
    v = np.asarray(m.vertices, dtype=np.float64) * np.asarray(scale, dtype=np.float64)
    f = np.asarray(m.faces, dtype=np.int32)
    g = UsdGeom.Mesh.Define(stage, path)
    g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)))
    g.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(f), 3, dtype=np.int32)))
    g.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(f.reshape(-1).astype(np.int32)))
    g.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    if len(v):
        g.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(c) for c in v.min(0)]),
                                          Gf.Vec3f(*[float(c) for c in v.max(0)])]))
    col = rgba[:3] if rgba else _mesh_color(m)
    g.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(c) for c in col])]))
    return g


def _mesh_color(m):
    """mesh 파일의 대표색(재질 baseColor → 정점색 평균 → 회색)."""
    try:
        vis = getattr(m, "visual", None)
        mat = getattr(vis, "material", None)
        for attr in ("baseColorFactor", "main_color", "diffuse"):
            c = getattr(mat, attr, None)
            if c is not None and len(c) >= 3:
                c = np.asarray(c, dtype=float)
                return (c[:3] / 255.0) if c.max() > 1.0 else c[:3]
        vc = getattr(vis, "vertex_colors", None)
        if vc is not None and len(vc):
            return np.asarray(vc, dtype=float).mean(0)[:3] / 255.0
    except Exception:                            # noqa: BLE001 — 색은 부가정보
        pass
    return np.array([0.6, 0.6, 0.6])


def write_shape(stage, path, g, rgba):
    """box/cylinder/sphere primitive 를 USD 로. → (prim, scale|None)

    box 는 USD 에 크기 속성이 하나뿐(정육면체)이라 size=1 로 두고 scale 로 변 길이를 준다.
    scale 은 호출부가 translate·orient 뒤에 붙여야 해서(TRS 순서) 여기서 걸지 않고 돌려준다.
    """
    scale = None
    if g["kind"] == "box":
        p = UsdGeom.Cube.Define(stage, path)
        p.CreateSizeAttr(1.0)
        scale = [float(s) for s in g["size"]]
    elif g["kind"] == "cylinder":
        p = UsdGeom.Cylinder.Define(stage, path)
        p.CreateRadiusAttr(float(g["radius"]))
        p.CreateHeightAttr(float(g["length"]))
        p.CreateAxisAttr(UsdGeom.Tokens.z)
    elif g["kind"] == "sphere":
        p = UsdGeom.Sphere.Define(stage, path)
        p.CreateRadiusAttr(float(g["radius"]))
    else:
        return None, None
    if rgba:
        p.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(c) for c in rgba[:3]])]))
    return p, scale


# ─────────────────────────────────────────────────────────────── 로봇 스테이지
_AXIS_TOKEN = {(1, 0, 0): "X", (0, 1, 0): "Y", (0, 0, 1): "Z"}


def _axis_align(axis):
    """URDF 축 → (USD axis 토큰, 관절 프레임에 덧붙일 회전 R). 축이 ±기본축이면 회전 없음."""
    a = np.asarray(axis, dtype=float)
    n = np.linalg.norm(a)
    if n < 1e-9:
        raise ValueError("관절 축이 0 벡터")
    a = a / n
    key = tuple(int(round(v)) for v in np.abs(a))
    if key in _AXIS_TOKEN and np.allclose(np.abs(a), key, atol=1e-6):
        tok = _AXIS_TOKEN[key]
        if float(a[list(key).index(1)]) > 0:
            return tok, np.eye(3)
        # 음의 방향: 축을 뒤집는 회전(직교하는 축 기준 180°)을 관절 프레임에 넣는다
        perp = np.array([1.0, 0.0, 0.0]) if tok != "X" else np.array([0.0, 1.0, 0.0])
        return tok, _axis_angle(perp, math.pi)
    # 임의 축: X 를 그 축으로 보내는 회전
    x = np.array([1.0, 0.0, 0.0])
    v = np.cross(x, a)
    c = float(x @ a)
    if np.linalg.norm(v) < 1e-9:
        return "X", (np.eye(3) if c > 0 else _axis_angle(np.array([0.0, 0.0, 1.0]), math.pi))
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return "X", np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def _axis_angle(axis, ang):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def build_robot(out_file, urdf_path, mesh_search, fix_base=True, robot_name="rda_robot"):
    links, joints = parse_urdf(urdf_path)
    root_link, W = link_world_tf(links, joints)
    groups, rep_of = clusters(links, joints)

    stage = Usd.Stage.CreateNew(out_file)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    if hasattr(UsdPhysics, "SetStageKilogramsPerUnit"):
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    root_path = Sdf.Path(f"/{_sane(robot_name)}")
    robot = UsdGeom.Xform.Define(stage, root_path)
    stage.SetDefaultPrim(robot.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

    body_path, stats = {}, dict(bodies=0, meshes=0, shapes=0, frames=0, mass=0.0, missing=[])
    for rep, members in groups.items():
        bp = root_path.AppendChild(_sane(rep))
        body = UsdGeom.Xform.Define(stage, bp)
        _set_xform(body.GetPrim(), W[rep])
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        body_path[rep] = bp
        stats["bodies"] += 1

        inr = cluster_inertial(members, links, W, W[rep])
        if inr:
            m = UsdPhysics.MassAPI.Apply(body.GetPrim())
            m.CreateMassAttr(float(inr["mass"]))
            m.CreateCenterOfMassAttr(Gf.Vec3f(*[float(v) for v in inr["com"]]))
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*[float(v) for v in inr["diag"]]))
            m.CreatePrincipalAxesAttr(_gfq(inr["axes"]))
            stats["mass"] += inr["mass"]

        Minv = np.linalg.inv(W[rep])
        for ln in members:
            T_body_link = Minv @ W[ln]
            frame = UsdGeom.Xform.Define(stage, bp.AppendChild(_sane(ln) + "_frame"))
            _set_xform(frame.GetPrim(), T_body_link)   # 링크 프레임 표식(TF 대응)
            stats["frames"] += 1
            for tag, purpose in (("visual", None), ("collision", UsdGeom.Tokens.guide)):
                for i, g in enumerate(links[ln][tag]):
                    gp = frame.GetPath().AppendChild(f"{tag}_{i}")
                    if g["kind"] == "mesh":
                        f = resolve_pkg(g["uri"], mesh_search)
                        if not f:
                            stats["missing"].append(g["uri"])
                            continue
                        prim = write_mesh(stage, gp, f, g["scale"], g.get("rgba"))
                        scale = None                            # mesh 는 정점에 스케일을 이미 반영
                        stats["meshes"] += 1
                    else:
                        prim, scale = write_shape(stage, gp, g, g.get("rgba"))
                        if prim is None:
                            continue
                        stats["shapes"] += 1
                    _set_xform(prim, g["T"], scale)
                    if purpose is not None:
                        UsdGeom.Imageable(prim).CreatePurposeAttr(purpose)
                        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
                        if g["kind"] == "mesh":
                            UsdPhysics.MeshCollisionAPI.Apply(prim.GetPrim()) \
                                .CreateApproximationAttr(UsdPhysics.Tokens.convexHull)

    jroot = root_path.AppendChild("joints")
    UsdGeom.Scope.Define(stage, jroot)
    njoint = 0
    for jn, j in joints.items():
        if j["type"] == "fixed":
            continue                                  # 클러스터 안으로 이미 병합됨
        pb, cb = rep_of[j["parent"]], rep_of[j["child"]]
        tok, R_align = _axis_align(j["axis"])
        T_jf = np.eye(4)
        T_jf[:3, :3] = R_align
        # 관절 프레임 = 자식 링크 프레임(URDF 규약)에 축 정렬 회전을 덧붙인 것
        T0 = np.linalg.inv(W[pb]) @ W[j["child"]] @ T_jf
        T1 = np.linalg.inv(W[cb]) @ W[j["child"]] @ T_jf
        cls = UsdPhysics.PrismaticJoint if j["type"] == "prismatic" else UsdPhysics.RevoluteJoint
        jp = cls.Define(stage, jroot.AppendChild(_sane(jn)))
        jp.CreateBody0Rel().SetTargets([body_path[pb]])
        jp.CreateBody1Rel().SetTargets([body_path[cb]])
        jp.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in T0[:3, 3]]))
        jp.CreateLocalRot0Attr(_gfq(_quat(T0[:3, :3])))
        jp.CreateLocalPos1Attr(Gf.Vec3f(*[float(v) for v in T1[:3, 3]]))
        jp.CreateLocalRot1Attr(_gfq(_quat(T1[:3, :3])))
        jp.CreateAxisAttr(tok)
        if j["type"] != "continuous" and j["lower"] is not None:
            k = (180.0 / math.pi) if j["type"] != "prismatic" else 1.0   # 회전 한계는 도(deg)
            jp.CreateLowerLimitAttr(float(j["lower"] * k))
            jp.CreateUpperLimitAttr(float(j["upper"] * k))
        dtype = "linear" if j["type"] == "prismatic" else "angular"
        drv = UsdPhysics.DriveAPI.Apply(jp.GetPrim(), dtype)
        drv.CreateTypeAttr(UsdPhysics.Tokens.force)
        drv.CreateMaxForceAttr(float(j["effort"]))    # URDF effort(2026-07-30 유도값)
        drv.CreateStiffnessAttr(1e4)                  # ⚠ 플레이스홀더
        drv.CreateDampingAttr(1e3)                    # ⚠ 플레이스홀더
        drv.CreateTargetPositionAttr(0.0)
        njoint += 1

    if fix_base:                                       # 베이스를 월드에 고정(수확 시 주차 상태)
        fj = UsdPhysics.FixedJoint.Define(stage, jroot.AppendChild("world_fix"))
        fj.CreateBody1Rel().SetTargets([body_path[rep_of[root_link]]])

    stage.GetRootLayer().Save()
    stats["joints"] = njoint
    stats["root_link"] = root_link
    return stats


# ─────────────────────────────────────────────────────────────── 온실 스테이지
def build_greenhouse(out_file, obstacles_yaml, include_targets=True):
    OB = _import_sibling("_obstacle_publisher", "obstacle_publisher.py")
    spec = yaml.safe_load(open(obstacles_yaml)) or {}
    try:
        OB.expand_crops(spec)
    except Exception as e:                             # noqa: BLE001
        sys.stderr.write(f"[gen_usd_scene] expand_crops 경고: {e}\n")

    stage = Usd.Stage.CreateNew(out_file)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/greenhouse")
    stage.SetDefaultPrim(root.GetPrim())

    n = 0
    for o in spec.get("obstacles", []):
        if o.get("kind") == "keepout":                 # 가상 벽 — 실체 없음(Gazebo 월드와 동일 규칙)
            continue
        if o.get("type") not in ("box", "cylinder", "sphere"):
            continue
        if not include_targets and o.get("kind") == "target":
            continue
        pose = o.get("pose") or {}
        T = _tf([float(v) for v in pose.get("xyz", [0, 0, 0])],
                [float(v) for v in pose.get("rpy", [0, 0, 0])])
        g = dict(kind=o["type"])
        if o["type"] == "box":
            g["size"] = o["size"]
        elif o["type"] == "cylinder":
            g["radius"], g["length"] = o["radius"], o["height"]
        else:
            g["radius"] = o["radius"]
        p, scale = write_shape(stage, root.GetPath().AppendChild(_sane(o["name"])), g, o.get("color"))
        _set_xform(p, T, scale)
        UsdPhysics.CollisionAPI.Apply(p.GetPrim())     # 정적 충돌체(RigidBody 없음)
        n += 1
    stage.GetRootLayer().Save()
    return n


def build_scene(out_file, robot_file, world_file, robot_name="rda_robot"):
    stage = Usd.Stage.CreateNew(out_file)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    sc = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    sc.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    sc.CreateGravityMagnitudeAttr(9.81)

    for name, f in (("greenhouse", world_file), (_sane(robot_name), robot_file)):
        p = UsdGeom.Xform.Define(stage, f"/World/{name}")
        p.GetPrim().GetReferences().AddReference("./" + os.path.basename(f))

    UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(1000.0)
    stage.GetRootLayer().Save()


# ─────────────────────────────────────────────────────────────── 산출물 부속
ISAAC_SCRIPT = '''#!/usr/bin/env python3
"""Isaac Sim 에서 rda_scene.usd 를 열고 ROS2 Bridge 를 붙이는 스크립트.

⚠ 이 파일은 **Isaac 이 설치된 PC 에서 실행**한다(생성 PC 에는 Isaac 이 없어 미검증).
  Isaac Sim 4.x/5.x 의 standalone python 으로:
      ./python.sh isaac_load_scene.py --usd /경로/rda_scene.usd
"""
import argparse

from isaacsim import SimulationApp

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True)
ap.add_argument("--headless", action="store_true")
a = ap.parse_args()

app = SimulationApp({"headless": a.headless})

from omni.isaac.core import World                    # noqa: E402
from omni.isaac.core.utils.stage import open_stage   # noqa: E402
from omni.isaac.core.utils.extensions import enable_extension  # noqa: E402

enable_extension("omni.isaac.ros2_bridge")           # ROS2 Bridge
open_stage(a.usd)

world = World(stage_units_in_meters=1.0)
world.reset()
print("[isaac] 로드 완료 — /World/rda_robot 아티큘레이션 확인 후 Play")
while app.is_running():
    world.step(render=not a.headless)
app.close()
'''


def write_readme(out_dir, stats, nprims, args):
    p = os.path.join(out_dir, "README.md")
    open(p, "w").write(f"""# Isaac Sim 로드용 USD 번들 (RDA 농업로봇)

이 PC 에는 Isaac Sim 을 설치하지 않고 **Isaac 이 열 수 있는 파일만** 생성했다.
생성기 = `rda_robot_bringup/scripts/gen_usd_scene.py` (Pixar USD `usd-core` 만 사용).

## 파일
| 파일 | 내용 |
|---|---|
| `rda_scene.usd` | **여는 파일** — 로봇+온실 참조 · PhysicsScene(중력 −Z 9.81) · DomeLight |
| `rda_robot.usd` | 로봇 아티큘레이션 — 강체 {stats['bodies']}개 · 가동관절 {stats['joints']}개 · 질량 합 {stats['mass']:.3f} kg |
| `greenhouse.usd` | 온실/작물 정적 충돌체 {nprims}개 (obstacles.yaml 단일 진실원) |
| `isaac_load_scene.py` | Isaac standalone 실행 예시(ROS2 Bridge 활성화) |

## Isaac 에서 여는 법
1. Isaac Sim 실행 → `File > Open` → `rda_scene.usd`
   (또는 `./python.sh isaac_load_scene.py --usd <경로>/rda_scene.usd`)
2. `/World/rda_robot` 이 **ArticulationRoot** 로 잡히는지 확인 → Play.
3. ROS2 로 움직이려면 `omni.isaac.ros2_bridge` 를 켜고 Articulation 에
   ROS2 JointState / Subscribe JointState OmniGraph 노드를 연결한다.

## 좌표·단위
* Z-up · 1 unit = 1 m · 질량 kg. URDF/Gazebo 월드와 **같은 좌표계**(world z=0 이 바닥).
* 로봇 베이스는 world 원점에서 +X 를 보고 서 있고, `world_fix` FixedJoint 로 고정돼 있다
  (주행이 필요하면 이 관절을 지우면 된다: `--no-fix-base` 로 재생성해도 된다).

## 알려진 한계 (Isaac 쪽에서 손봐야 하는 것)
1. **fixed 관절 병합** — URDF 44링크 중 질량이 있는 16개만 강체가 되고, 나머지는 강체 밑
   `*_frame` **빈 Xform**(좌표 표식)으로 남는다. 강체 안의 강체는 PhysX 에서 무효라서 그렇다.
2. **`rg2_finger_joint2` 의 mimic 은 USD 표준에 없다** — 관절 2개가 독립으로 나간다.
   Isaac 의 mimic joint 로 묶거나 컨트롤러가 같은 값을 주어야 한다.
3. **드라이브 게인은 플레이스홀더**(stiffness 1e4 / damping 1e3). `maxForce` 만 URDF effort
   (2026-07-30 URDF 질량·관성에서 유도한 값: base 64 / shoulder 314 / elbow 121 / wrist 22 Nm)에서 왔다.
   **이 값은 제조사 정격이 아니다** — 물리 시뮬에서 팔이 처지지 않을 하한×안전계수다.
4. **관절 속도 한계**는 UsdPhysics 표준 속성이 아니라 빠져 있다(Isaac 측 설정).
5. **시각 재질은 대표색만** — 텍스처·PBR 은 옮기지 않았다.
6. 충돌 mesh 는 전부 `convexHull` 근사다(오목 형상은 Isaac 에서 SDF/convexDecomposition 로 바꿀 것).

## 재생성
```bash
ros2 run rda_robot_bringup gen_usd_scene.py --out <dir>
python3 src/docs/scripts/test_usd_export.py      # 로컬 검증(Isaac 불필요)
```
""")
    return p


# ─────────────────────────────────────────────────────────────── main
def compose_urdf(dst):
    """mounts.yaml → 통합 URDF(어셈블러 컴포저). 실패 시 예외."""
    src = os.path.abspath(os.path.join(_here, "..", ".."))
    mounts = os.path.join(src, "rda_robot_description", "config", "mounts.yaml")
    if not os.path.exists(mounts):
        from ament_index_python.packages import get_package_share_directory
        mounts = os.path.join(get_package_share_directory("rda_robot_description"),
                              "config", "mounts.yaml")
    out = subprocess.run(["ros2", "run", "rda_robot_assembler", "compose_urdf",
                          "--mounts", mounts], capture_output=True, text=True)
    if out.returncode != 0 or not out.stdout.strip():
        raise SystemExit(f"[gen_usd_scene] 통합 URDF 조립 실패:\n{out.stderr}")
    open(dst, "w").write(out.stdout)
    if out.stderr.strip():
        sys.stderr.write(out.stderr)
    return dst


def main():
    ap = argparse.ArgumentParser(description="통합 URDF + obstacles.yaml → Isaac 로드용 USD")
    ap.add_argument("--out", default=os.path.expanduser("~/robot_ws/export/isaac"))
    ap.add_argument("--urdf", default=None, help="생략하면 mounts.yaml 로 즉석 조립")
    ap.add_argument("--obstacles", default=None)
    ap.add_argument("--no-fix-base", action="store_true", help="베이스 world 고정 관절 생략")
    ap.add_argument("--no-targets", action="store_true", help="열매(kind:target) 제외")
    ap.add_argument("--mesh-path", action="append", default=[],
                    help="package:// 해석용 추가 검색 경로")
    a = ap.parse_args()

    out_dir = os.path.abspath(os.path.expanduser(a.out))
    if os.path.isdir(out_dir):
        shutil.rmtree(out_dir)                       # 같은 파일명 덮어쓰기(작업 규칙 3)
    os.makedirs(out_dir, exist_ok=True)

    urdf = a.urdf or compose_urdf(os.path.join(out_dir, "rda_robot.urdf"))
    obstacles = a.obstacles or os.path.join(
        _here, "..", "..", "rda_robot_description", "config", "obstacles.yaml")
    if not os.path.exists(obstacles):
        from ament_index_python.packages import get_package_share_directory
        obstacles = os.path.join(get_package_share_directory("rda_robot_description"),
                                 "config", "obstacles.yaml")

    robot_f = os.path.join(out_dir, "rda_robot.usd")
    world_f = os.path.join(out_dir, "greenhouse.usd")
    scene_f = os.path.join(out_dir, "rda_scene.usd")

    stats = build_robot(robot_f, urdf, a.mesh_path, fix_base=not a.no_fix_base)
    nprims = build_greenhouse(world_f, obstacles, include_targets=not a.no_targets)
    build_scene(scene_f, robot_f, world_f)
    write_readme(out_dir, stats, nprims, a)
    isaac_py = os.path.join(out_dir, "isaac_load_scene.py")
    open(isaac_py, "w").write(ISAAC_SCRIPT)
    os.chmod(isaac_py, 0o755)

    sys.stderr.write(
        f"[gen_usd_scene] 로봇: 강체 {stats['bodies']} · 관절 {stats['joints']} · "
        f"프레임 {stats['frames']} · mesh {stats['meshes']} · shape {stats['shapes']} · "
        f"질량 {stats['mass']:.3f}kg (루트 {stats['root_link']})\n"
        f"[gen_usd_scene] 온실: primitive {nprims}\n"
        f"[gen_usd_scene] 출력: {scene_f}\n")
    if stats["missing"]:
        sys.stderr.write(f"[gen_usd_scene] ⚠ mesh 를 못 찾음 {len(stats['missing'])}건: "
                         f"{sorted(set(stats['missing']))[:3]} …\n")


if __name__ == "__main__":
    main()
