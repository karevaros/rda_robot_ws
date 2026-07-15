#!/usr/bin/env python3
"""Scout 플랫폼 실측 — 상판 높이/치수를 AABB 가 아니라 실제 형상으로."""
import os
import sys

import numpy as np
import trimesh
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


r = yourdfpy.URDF.load(sys.argv[1], filename_handler=res, load_collision_meshes=True)
r.update_cfg({j: 0.0 for j in r.actuated_joint_names})

# base_link 에 붙은 geometry 만 (바퀴 제외하고 본체)
scene = r.scene
base_fk = r.get_transform("base_link")
verts = []
for name, geom in scene.geometry.items():
    try:
        T = scene.graph.get(name)[0]
    except Exception:
        continue
    v = trimesh.transform_points(np.asarray(geom.vertices), T)
    verts.append((name, v))

# base_link 프레임 기준 전체
print("=== base_link 프레임 기준 geometry AABB ===")
for name, v in verts:
    lo, hi = v.min(0), v.max(0)
    print(f"{name:<42} z[{lo[2]:+.4f},{hi[2]:+.4f}] x[{lo[0]:+.3f},{hi[0]:+.3f}] y[{lo[1]:+.3f},{hi[1]:+.3f}]")

# ⚠ base_link 자체 geometry 만 (link0.. 팔은 제외 — 안 그러면 팔 끝(1.27)이 잡힘)
base_only = [(n, v) for n, v in verts if n.startswith("base_link")]
wheels = [(n, v) for n, v in verts if n.startswith("wheel")]
print(f"\n[base_link 본체 geometry {len(base_only)}개 · 바퀴 {len(wheels)}개 만 사용]")
allv = np.vstack([v for _, v in base_only])
for R in (0.10, 0.15, 0.20, 0.25):
    m = (np.abs(allv[:, 0]) < R) & (np.abs(allv[:, 1]) < R)
    if m.any():
        print(f"\n중심 |x|,|y|<{R}m 영역 최고 z = {allv[m][:,2].max():.4f} m  (정점 {m.sum()})")

wv = np.vstack([v for _, v in wheels])
print(f"\n바퀴 포함 전체 폭 Y = {np.vstack([allv,wv])[:,1].ptp():.3f} m")

# 본체 치수 (base_link geometry 전체)
body = allv
print(f"\n본체(z>0.02) X 범위 [{body[:,0].min():+.3f},{body[:,0].max():+.3f}] → 길이 {body[:,0].ptp():.3f} m")
print(f"본체(z>0.02) Y 범위 [{body[:,1].min():+.3f},{body[:,1].max():+.3f}] → 폭   {body[:,1].ptp():.3f} m")
print(f"본체(z>0.02) Z 범위 [{body[:,2].min():+.3f},{body[:,2].max():+.3f}] → 최고 {body[:,2].max():.3f} m")
print(f"\n전체 AABB Z 최고 = {allv[:,2].max():.4f} m")
