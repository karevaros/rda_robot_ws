#!/usr/bin/env python3
"""USD 내보내기 검증 — Isaac 없이 로컬에서 (7주차 M4)

`gen_usd_scene.py` 가 만든 USD 가 **URDF/obstacles.yaml 과 같은 로봇·같은 장면인지**를
Isaac 을 켜지 않고 확인한다. 파일이 열리는지만 보는 게 아니라 수치를 대조한다.

  1. 스테이지 규약      Z-up · 1unit=1m · defaultPrim · ArticulationRoot
  2. 강체·질량          강체 개수 · 질량 합 = URDF 질량 합
  3. 링크 프레임 FK     44개 링크 프레임의 world 변환 = URDF FK(관절 0)  ← 좌표·쿼터니언 규약 검증
  4. 관절 프레임 정합   body0·body1 이 가리키는 관절 프레임이 한 점에서 만나는가
                        (T_world(body0)·localT0 == T_world(body1)·localT1)
  5. 관절 한계·축       USD(도) = URDF(라디안) · 축 토큰 = URDF axis
  6. 온실               primitive 개수·자세·크기 = Gazebo 월드(SDF)와 동일
  7. 씬 합성            rda_scene.usd 에서 참조가 풀려 로봇·온실 prim 이 보이는가
  8. USD 규격 준수      UsdUtils.ComplianceChecker

사용: python3 src/docs/scripts/test_usd_export.py [--keep] [--out DIR]
"""
import argparse
import importlib.util
import math
import os
import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdUtils

_SRC = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
_SCRIPTS = os.path.join(_SRC, "rda_robot_bringup", "scripts")

_ok, _fail = [], []


def check(name, cond, detail=""):
    (_ok if cond else _fail).append(name)
    print(f"{'✅' if cond else '❌'} {name}{('  ' + detail) if detail else ''}")
    return cond


def _import(mod_name, path):
    spec = importlib.util.spec_from_file_location(mod_name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _m4(gf):
    """Gf.Matrix4d(행벡터 규약) → numpy 4x4(열벡터 규약)."""
    return np.array([[gf[r][c] for r in range(4)] for c in range(4)])


def world_tf(prim):
    return _m4(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))


def local_tf(pos, rot):
    q = np.array([rot.GetReal(), *rot.GetImaginary()])
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [pos[0], pos[1], pos[2]]
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None, help="검증할 번들(생략하면 임시 폴더에 새로 생성)")
    ap.add_argument("--keep", action="store_true")
    a = ap.parse_args()

    G = _import("_gen_usd_scene", os.path.join(_SCRIPTS, "gen_usd_scene.py"))

    out = a.out or os.path.join(tempfile.gettempdir(), "rda_usd_check")
    if not a.out:
        print(f"[생성] {out}")
        r = subprocess.run([sys.executable, os.path.join(_SCRIPTS, "gen_usd_scene.py"),
                            "--out", out], capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout, r.stderr)
            raise SystemExit("생성 실패")
        sys.stderr.write(r.stderr)

    urdf = os.path.join(out, "rda_robot.urdf")
    robot_usd = os.path.join(out, "rda_robot.usd")
    world_usd = os.path.join(out, "greenhouse.usd")
    scene_usd = os.path.join(out, "rda_scene.usd")

    # ── 기준값: URDF 를 직접 파싱해 얻는다(생성기와 같은 파서를 쓰되 결과를 따로 계산) ──
    links, joints = G.parse_urdf(urdf)
    root_link, W = G.link_world_tf(links, joints)
    urdf_mass = sum(l["inertial"]["mass"] for l in links.values() if l["inertial"])
    movable = {n: j for n, j in joints.items() if j["type"] != "fixed"}

    stage = Usd.Stage.Open(robot_usd)

    # 1. 스테이지 규약
    check("Z-up · 1 unit = 1 m",
          UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z
          and abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) < 1e-9,
          f"up={UsdGeom.GetStageUpAxis(stage)} mpu={UsdGeom.GetStageMetersPerUnit(stage)}")
    dp = stage.GetDefaultPrim()
    check("defaultPrim 이 ArticulationRoot",
          bool(dp) and dp.HasAPI(UsdPhysics.ArticulationRootAPI), str(dp.GetPath()))

    # 2. 강체·질량
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    usd_mass = sum(UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0.0
                   for p in bodies if p.HasAPI(UsdPhysics.MassAPI))
    n_expect = 1 + len(movable)          # 루트 클러스터 + 가동관절마다 클러스터 하나
    check("강체 개수 = 1 + 가동관절 수", len(bodies) == n_expect,
          f"{len(bodies)} (기대 {n_expect})")
    # UsdPhysics 의 mass 는 스키마상 float(32bit) — 상대오차 1e-6 이면 규격상 최선이다
    check("질량 합 = URDF 질량 합", abs(usd_mass - urdf_mass) <= 1e-6 * max(1.0, urdf_mass),
          f"USD {usd_mass:.6f} kg vs URDF {urdf_mass:.6f} kg")
    check("강체 안에 강체 없음(PhysX 규칙)",
          all(not any(anc.HasAPI(UsdPhysics.RigidBodyAPI)
                      for anc in _ancestors(stage, p)) for p in bodies))

    # 3. 링크 프레임 FK 대조 (44개 전부)
    worst, worst_n = 0.0, None
    for ln in links:
        prim = _find_frame(stage, ln)
        if prim is None:
            check(f"링크 프레임 존재: {ln}", False)
            continue
        e = np.abs(world_tf(prim) - W[ln]).max()
        if e > worst:
            worst, worst_n = e, ln
    check(f"링크 프레임 {len(links)}개 world 변환 = URDF FK", worst < 1e-5,
          f"최대 오차 {worst:.2e} m ({worst_n})")

    # 4·5. 관절 프레임 정합 · 한계 · 축
    jworst, lim_bad, axis_bad = 0.0, [], []
    for jn, j in movable.items():
        jp = stage.GetPrimAtPath(f"{dp.GetPath()}/joints/{G._sane(jn)}")
        if not jp:
            check(f"관절 prim 존재: {jn}", False)
            continue
        pj = UsdPhysics.Joint(jp)
        b0 = stage.GetPrimAtPath(pj.GetBody0Rel().GetTargets()[0])
        b1 = stage.GetPrimAtPath(pj.GetBody1Rel().GetTargets()[0])
        T0 = world_tf(b0) @ local_tf(pj.GetLocalPos0Attr().Get(), pj.GetLocalRot0Attr().Get())
        T1 = world_tf(b1) @ local_tf(pj.GetLocalPos1Attr().Get(), pj.GetLocalRot1Attr().Get())
        jworst = max(jworst, float(np.abs(T0 - T1).max()))
        rj = UsdPhysics.RevoluteJoint(jp)
        lo, hi = rj.GetLowerLimitAttr().Get(), rj.GetUpperLimitAttr().Get()
        if abs(lo - math.degrees(j["lower"])) > 1e-3 or abs(hi - math.degrees(j["upper"])) > 1e-3:
            lim_bad.append(f"{jn}: USD({lo:.2f},{hi:.2f})° vs URDF({math.degrees(j['lower']):.2f},"
                           f"{math.degrees(j['upper']):.2f})°")
        # 축: USD 토큰 축을 관절 프레임으로 되돌리면 URDF 축(자식 링크 프레임 기준)과 같아야 한다
        tok = rj.GetAxisAttr().Get()
        e = {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}[tok]
        R_jf = (np.linalg.inv(W[j["child"]]) @ T1)[:3, :3]
        got = R_jf @ np.array(e, dtype=float)
        want = j["axis"] / np.linalg.norm(j["axis"])
        if np.abs(got - want).max() > 1e-6:
            axis_bad.append(f"{jn}: {got.round(3)} vs {want.round(3)}")
    check("관절 프레임이 body0·body1 양쪽에서 일치", jworst < 1e-5, f"최대 오차 {jworst:.2e}")
    check("관절 한계(도) = URDF(라디안)", not lim_bad, "; ".join(lim_bad))
    check("관절 축 = URDF axis", not axis_bad, "; ".join(axis_bad))
    check("가동관절 개수", len([p for p in stage.Traverse()
                                if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint)])
          == len(movable), f"{len(movable)}개")
    drv = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.DriveAPI, "angular")]
    eff = {jn: j["effort"] for jn, j in movable.items()}
    dbad = [p.GetName() for p in drv
            if abs((UsdPhysics.DriveAPI(p, "angular").GetMaxForceAttr().Get() or -1)
                   - eff.get(p.GetName(), -2)) > 1e-6]
    check("드라이브 maxForce = URDF effort", len(drv) == len(movable) and not dbad, str(dbad))

    # 충돌체
    cols = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    urdf_cols = sum(len(l["collision"]) for l in links.values())
    check("충돌 prim 개수 = URDF collision 개수", len(cols) == urdf_cols,
          f"{len(cols)} vs {urdf_cols}")

    # 6. 온실 — Gazebo SDF 월드와 대조(같은 소스에서 나왔으므로 개수·자세가 같아야 한다)
    gz = _import("_gen_gazebo_world", os.path.join(_SCRIPTS, "gen_gazebo_world.py"))
    obstacles = os.path.join(_SRC, "rda_robot_description", "config", "obstacles.yaml")
    sdf = ET.fromstring(gz.build_world(obstacles))
    sdf_models = {m.get("name"): m for m in sdf.iter("model")}
    wstage = Usd.Stage.Open(world_usd)
    prims = [p for p in wstage.Traverse() if p.GetPath().pathString.count("/") == 2
             and p.IsA(UsdGeom.Gprim)]
    check("온실 primitive 개수 = Gazebo 월드 모델 수", len(prims) == len(sdf_models),
          f"USD {len(prims)} vs SDF {len(sdf_models)}")
    pworst, pworst_n = 0.0, None
    for p in prims:
        m = sdf_models.get(p.GetName())
        if m is None:
            check(f"SDF 에 없는 prim: {p.GetName()}", False)
            continue
        pose = [float(v) for v in m.find("pose").text.split()]
        T = G._tf(pose[:3], pose[3:])
        M = world_tf(p)
        # box 는 scale 로 변 길이를 주므로 world 변환에 스케일이 섞여 있다 → 열을 정규화해 회전만 본다
        R = M[:3, :3] / np.maximum(np.linalg.norm(M[:3, :3], axis=0), 1e-12)
        e = max(float(np.abs(M[:3, 3] - T[:3, 3]).max()), float(np.abs(R - T[:3, :3]).max()))
        if e > pworst:
            pworst, pworst_n = e, p.GetName()
    check("온실 primitive 자세 = SDF pose", pworst < 1e-6, f"최대 오차 {pworst:.2e} ({pworst_n})")
    check("온실 primitive 가 전부 정적 충돌체",
          all(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims)
          and not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims))
    # 크기 표본 대조(구/원기둥/박스 각 1개)
    size_bad = []
    TOL = 1e-6                                 # scale 은 Vec3f(32bit) — 0.24 같은 값에 반올림이 있다
    for p in prims:
        m = sdf_models[p.GetName()]
        geo = m.find(".//geometry")[0]
        if geo.tag == "sphere":
            r = float(geo.find("radius").text)
            if abs(UsdGeom.Sphere(p).GetRadiusAttr().Get() - r) > TOL:
                size_bad.append(p.GetName())
        elif geo.tag == "cylinder":
            r, l = float(geo.find("radius").text), float(geo.find("length").text)
            c = UsdGeom.Cylinder(p)
            if abs(c.GetRadiusAttr().Get() - r) > TOL or abs(c.GetHeightAttr().Get() - l) > TOL:
                size_bad.append(p.GetName())
        elif geo.tag == "box":
            s = [float(v) for v in geo.find("size").text.split()]
            sc = [v for v in UsdGeom.Xformable(p).GetOrderedXformOps()[-1].Get()]
            if max(abs(a - b) for a, b in zip(s, sc)) > TOL:
                size_bad.append(p.GetName())
    check("온실 primitive 치수 = SDF geometry", not size_bad, str(size_bad[:5]))

    # 7. 씬 합성 — 참조가 풀려야 로봇/온실 prim 이 보인다
    sstage = Usd.Stage.Open(scene_usd)
    tcp = [p for p in sstage.Traverse() if p.GetName() == "tcp_frame"]
    ghouse = [p for p in sstage.Traverse() if p.GetName().startswith("gutter")]
    check("rda_scene.usd 에서 참조 해석", bool(tcp) and bool(ghouse),
          f"tcp_frame {len(tcp)} · gutter* {len(ghouse)}")
    ps = [p for p in sstage.Traverse() if p.IsA(UsdPhysics.Scene)]
    g = UsdPhysics.Scene(ps[0]) if ps else None
    check("PhysicsScene 중력 −Z 9.81", bool(ps)
          and abs(g.GetGravityMagnitudeAttr().Get() - 9.81) < 1e-6
          and list(g.GetGravityDirectionAttr().Get()) == [0.0, 0.0, -1.0])
    if tcp:                                   # 합성 스테이지에서도 tcp 위치가 URDF 와 같은가
        # 🔴 합성 스테이지에는 로봇의 **월드 배치**(mounts.yaml base_placement + 지면 오프셋)가
        #    걸려 있다. 그걸 빼고 비교하면 '로봇만 보면 맞고 온실과의 상대 배치는 틀린' 상태가
        #    통과한다(2026-08-16 Unity 에서 실제로 그랬다 — 사용자가 화면에서 발견).
        px, py, pz, yaw = G.base_placement(urdf)
        T_wb = np.eye(4)
        T_wb[:3, :3] = np.array([[np.cos(yaw), -np.sin(yaw), 0.0],
                                 [np.sin(yaw),  np.cos(yaw), 0.0],
                                 [0.0,          0.0,         1.0]])
        T_wb[:3, 3] = [px, py, pz]
        e = float(np.abs(world_tf(tcp[0]) - T_wb @ W["tcp"]).max())
        check("합성 스테이지 tcp 위치 = URDF FK(월드 배치 반영)", e < 1e-5,
              f"{np.round(world_tf(tcp[0])[:3, 3], 4).tolist()} (오차 {e:.2e})")

    # 8. USD 무결성 — pip `usd-core` 에는 usdchecker/ComplianceChecker 가 없어 직접 훑는다
    comp = list(sstage.GetCompositionErrors()) if hasattr(sstage, "GetCompositionErrors") else []
    check("합성(composition) 오류 없음", not comp, str(comp[:2]))
    deps = UsdUtils.ComputeAllDependencies(scene_usd)
    missing_dep = [str(x) for x in (deps[1] if len(deps) > 1 else []) if not os.path.exists(str(x))]
    check("참조 파일이 전부 존재", not missing_dep, str(missing_dep))
    bad_mesh, bad_tf = [], []
    for p in sstage.Traverse():
        if p.IsA(UsdGeom.Mesh):
            mm = UsdGeom.Mesh(p)
            pts = mm.GetPointsAttr().Get()
            idx = mm.GetFaceVertexIndicesAttr().Get()
            cnt = mm.GetFaceVertexCountsAttr().Get()
            if not pts or not idx or not cnt or max(idx) >= len(pts) or min(idx) < 0:
                bad_mesh.append(p.GetName())
        if p.IsA(UsdGeom.Xformable):
            M = world_tf(p)
            if not np.isfinite(M).all():
                bad_tf.append(p.GetName())
    check("mesh 정점·인덱스 정합", not bad_mesh, str(bad_mesh[:5]))
    check("모든 변환이 유한값", not bad_tf, str(bad_tf[:5]))

    print(f"\n통과 {len(_ok)} / 실패 {len(_fail)}  (총 {len(_ok) + len(_fail)})")
    if _fail:
        print("실패:", ", ".join(_fail))
    if not a.out and not a.keep:
        import shutil
        shutil.rmtree(out, ignore_errors=True)
    sys.exit(1 if _fail else 0)


def _ancestors(stage, prim):
    p = prim.GetParent()
    while p and p.GetPath() != Sdf_root():
        yield p
        p = p.GetParent()


def Sdf_root():
    from pxr import Sdf
    return Sdf.Path.absoluteRootPath


def _find_frame(stage, link_name):
    from pxr import Sdf                                   # noqa: F401
    want = re.sub(r"[^A-Za-z0-9_]", "_", link_name) + "_frame"
    for p in stage.Traverse():
        if p.GetName() == want:
            return p
    return None


if __name__ == "__main__":
    main()
