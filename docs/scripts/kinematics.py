#!/usr/bin/env python3
"""통합 URDF에서 기구학 파라미터 추출 (링크 파라미터 · 관절 제한 · 작업영역)."""
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

URDF = sys.argv[1]

tree = ET.parse(URDF)
root = tree.getroot()


def fl(s, n=3):
    v = [float(x) for x in (s or "0 0 0").split()]
    return v + [0.0] * (n - len(v))


links = {l.get("name") for l in root.findall("link")}
joints = []
for j in root.findall("joint"):
    o = j.find("origin")
    a = j.find("axis")
    lim = j.find("limit")
    mim = j.find("mimic")
    joints.append(
        dict(
            name=j.get("name"),
            type=j.get("type"),
            parent=j.find("parent").get("link"),
            child=j.find("child").get("link"),
            xyz=fl(o.get("xyz") if o is not None else None),
            rpy=fl(o.get("rpy") if o is not None else None),
            axis=fl(a.get("xyz") if a is not None else None) if a is not None else None,
            lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
            upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
            vel=float(lim.get("velocity")) if lim is not None and lim.get("velocity") else None,
            eff=float(lim.get("effort")) if lim is not None and lim.get("effort") else None,
            mimic=mim.get("joint") if mim is not None else None,
        )
    )

print(f"# 링크 {len(links)} · 관절 {len(joints)}")
actuated = [j for j in joints if j["type"] in ("revolute", "prismatic", "continuous") and not j["mimic"]]
mimics = [j for j in joints if j["mimic"]]
fixed = [j for j in joints if j["type"] == "fixed"]
print(f"# 가동관절 {len(actuated)} · mimic {len(mimics)} · fixed {len(fixed)}\n")

print("## 가동 관절 (링크 파라미터 · 이동 제한)")
hdr = f"{'joint':<22}{'type':<11}{'parent':<12}{'child':<12}{'axis':<10}{'origin xyz (m)':<26}{'origin rpy (deg)':<24}{'lower(deg)':<11}{'upper(deg)':<11}{'range':<9}{'vel(deg/s)':<11}{'eff(Nm)'}"
print(hdr)
print("-" * len(hdr))
for j in actuated:
    ax = "".join("XYZ"[i] if abs(j["axis"][i]) > 0.5 else "" for i in range(3))
    sign = "-" if any(v < -0.5 for v in j["axis"]) else "+"
    lo = math.degrees(j["lower"]) if j["lower"] is not None else None
    up = math.degrees(j["upper"]) if j["upper"] is not None else None
    rng = f"{up-lo:.0f}" if lo is not None and up is not None else "-"
    los = f"{lo:.1f}" if lo is not None else "-"
    ups = f"{up:.1f}" if up is not None else "-"
    vels = f"{math.degrees(j['vel']):.1f}" if j["vel"] else "-"
    effs = f"{j['eff']:.0f}" if j["eff"] else "-"
    print(
        f"{j['name']:<22}{j['type']:<11}{j['parent']:<12}{j['child']:<12}{sign+ax:<10}"
        f"{str([round(v,4) for v in j['xyz']]):<26}"
        f"{str([round(math.degrees(v),1) for v in j['rpy']]):<24}"
        f"{los:<11}{ups:<11}{rng:<9}{vels:<11}{effs}"
    )

print("\n## mimic 관절")
for j in mimics:
    print(f"  {j['name']:<22} → mimic({j['mimic']})  type={j['type']}")

# ---- 팔 체인 링크 길이 (link0..link6/tcp) ----
print("\n## 팔 체인 (연속 관절 원점 = 링크 오프셋)")
by_child = {j["child"]: j for j in joints}
chain, cur = [], "tcp"
while cur in by_child:
    j = by_child[cur]
    chain.append(j)
    cur = j["parent"]
chain.reverse()
total = 0.0
for j in chain:
    d = np.linalg.norm(j["xyz"])
    total += d
    print(f"  {j['parent']:<12}→{j['child']:<12} d={d:.4f} m  xyz={[round(v,4) for v in j['xyz']]}")
print(f"  체인 원점거리 합 = {total:.4f} m  (경로 길이, 직선 reach 아님)")


# ---- FK 로 작업영역(reach) 산출 ----
def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_mat(*rpy)
    T[:3, 3] = xyz
    return T


def axis_rot(axis, q):
    a = np.array(axis, float)
    a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    T = np.eye(4)
    T[:3, :3] = R
    return T


def fk(cfg, target="tcp"):
    """base_link 기준 target 프레임 위치."""
    T = np.eye(4)
    ch, cur = [], target
    while cur in by_child:
        j = by_child[cur]
        ch.append(j)
        cur = j["parent"]
    ch.reverse()
    for j in ch:
        T = T @ tf(j["xyz"], j["rpy"])
        if j["type"] in ("revolute", "continuous") and not j["mimic"]:
            T = T @ axis_rot(j["axis"], cfg.get(j["name"], 0.0))
    return T


arm = [j for j in actuated if j["name"] in {c["name"] for c in chain}]
print(f"\n## 작업영역 (base_link 기준, tcp, 팔 {len(arm)}축 몬테카를로)")
rng = np.random.default_rng(0)
pts = []
N = 200000
for _ in range(N):
    cfg = {j["name"]: rng.uniform(j["lower"], j["upper"]) for j in arm}
    pts.append(fk(cfg)[:3, 3])
pts = np.array(pts)
base = fk({}, "link0")[:3, 3]
r = np.linalg.norm(pts - base, axis=1)
print(f"  팔 베이스(link0) 원점 = {[round(v,4) for v in base]} (base_link 기준)")
print(f"  tcp 반경 r: min={r.min():.3f} max={r.max():.3f} m   (최대 reach ≈ {r.max():.3f} m)")
for i, ax in enumerate("XYZ"):
    print(f"  {ax}: [{pts[:,i].min():+.3f}, {pts[:,i].max():+.3f}] m")
print(f"  홈포즈(전관절 0) tcp = {[round(v,4) for v in fk({})[:3,3]]}")
print(f"  샘플 {N}, 바닥 아래(z<0) 비율 = {(pts[:,2]<0).mean()*100:.1f}%")
