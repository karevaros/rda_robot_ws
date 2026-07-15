#!/usr/bin/env python3
"""장애물 정합성 검사 — 시작 자세 로봇과 겹치는 장애물 / 도달 가능성."""
import os
import sys

import numpy as np
import trimesh
import yaml
import yourdfpy
from ament_index_python.packages import get_package_share_directory


def res(fname):
    f = fname
    if f.startswith("package://"):
        rest = f[len("package://"):]
        pkg, _, tail = rest.partition("/")
        try:
            return os.path.join(get_package_share_directory(pkg), tail)
        except Exception:
            return f
    return f


urdf, ocfg = sys.argv[1], sys.argv[2]
r = yourdfpy.URDF.load(urdf, filename_handler=res, load_collision_meshes=True)
r.update_cfg({j: 0.0 for j in r.actuated_joint_names})  # 시작 자세(initial_pose 전부 0)

spec = yaml.safe_load(open(ocfg))

# world -> base_link 오프셋: launch 와 동일하게 URDF 에서 유도(base_footprint 를 z=0 으로)
import xml.etree.ElementTree as ET
BZ = 0.0
for j in ET.parse(urdf).getroot().findall("joint"):
    if j.find("child").get("link") == "base_footprint":
        BZ = -float((j.find("origin").get("xyz")).split()[2])
print(f"world -> base_link z = {BZ:.5f} (base_footprint 를 바닥 z=0 에)")
W = np.eye(4)
W[2, 3] = BZ

# 로봇 전체 mesh (world 기준)
scene = r.scene
robot_meshes = []
for name, geom in scene.geometry.items():
    T = W @ scene.graph.get(name)[0]
    m = geom.copy()
    m.apply_transform(T)
    robot_meshes.append((name, m))
robot = trimesh.util.concatenate([m for _, m in robot_meshes])
print(f"로봇 시작자세 mesh: {len(robot.vertices)} verts, AABB z[{robot.bounds[0][2]:.3f},{robot.bounds[1][2]:.3f}]")


def prim_mesh(o):
    t = o["type"]
    pz = o.get("pose") or {}
    xyz = np.array(pz.get("xyz", [0, 0, 0]), float)
    rpy = np.array(pz.get("rpy", [0, 0, 0]), float)
    if t == "box":
        m = trimesh.creation.box(extents=[float(v) for v in o["size"]])
    elif t == "cylinder":
        m = trimesh.creation.cylinder(radius=float(o["radius"]), height=float(o["height"]))
    else:
        m = trimesh.creation.icosphere(radius=float(o["radius"]))
    T = trimesh.transformations.euler_matrix(*rpy)
    T[:3, 3] = xyz
    m.apply_transform(T)
    return m


mgr = trimesh.collision.CollisionManager()
for name, m in robot_meshes:
    mgr.add_object(f"robot::{name}", m)

print("\n=== 시작 자세 로봇 ↔ 장애물 간섭 검사 ===")
bad = []
for o in spec["obstacles"]:
    om = prim_mesh(o)
    hit, names = mgr.in_collision_single(om, return_names=True)
    kind = o.get("kind", "obstacle")
    if hit:
        links = sorted({n.split("::")[1].split(".dae")[0] for n in names})
        print(f"  ⚠ {o['name']:<16} [{kind}] 겹침 → {links[:6]}")
        bad.append((o["name"], kind, links))
    else:
        print(f"  ✅ {o['name']:<16} [{kind}] 간섭 없음")

# 도달 가능성: tcp 최대반경 1.102, 팔 베이스 link0 (world 기준)
base = r.get_transform("link0")[:3, 3] + np.array([0, 0, BZ])
print(f"\n=== 도달권 검사 (팔 베이스 link0 world={np.round(base,3)}, 최대반경 1.102 m) ===")
for o in spec["obstacles"]:
    if o.get("kind") != "obstacle":
        continue
    om = prim_mesh(o)
    d = np.linalg.norm(om.bounds.mean(0) - base)
    near = np.linalg.norm(np.clip(base, om.bounds[0], om.bounds[1]) - base)
    tag = "도달가능" if near <= 1.102 else "❌ 도달 불가"
    print(f"  {o['name']:<16} 중심거리 {d:.3f} m · 최근접 {near:.3f} m → {tag}")

print(f"\n간섭 {len(bad)}건")
