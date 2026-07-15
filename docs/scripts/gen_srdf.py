#!/usr/bin/env python3
"""통합 URDF → SRDF 생성 (플래닝 그룹 + ACM 자동 산출).

MoveIt Setup Assistant 의 collision matrix 계산을 스크립트로 재현한다.
GUI 대신 스크립트인 이유: 통합 URDF 가 mounts.yaml(조립기)로 바뀌므로
모델을 갈아끼울 때마다 재생성해야 한다. GUI 1회 산출물은 곧 낡는다.

ACM 분류(Setup Assistant 와 동일 규칙):
  Adjacent  : 관절로 직접 연결된 링크쌍 → 항상 접촉, 무시
  Always    : 무작위 자세 전부에서 충돌 → 모델 자체 겹침, 무시
  Never     : 무작위 자세 어디서도 충돌 안 함 → 검사 불필요, 무시
  Default   : 시작 자세에서 이미 충돌 → 무시
  (그 외 = 실제로 충돌 가능 → 검사 대상으로 남긴다)

사용:
  python3 gen_srdf.py <urdf> <출력.srdf> [샘플수]
"""
import os
import sys
from collections import defaultdict

import numpy as np
import trimesh
import yourdfpy
from ament_index_python.packages import get_package_share_directory

N_SAMPLES = int(sys.argv[3]) if len(sys.argv) > 3 else 10000
ALWAYS_FRAC = 0.95      # 이 비율 이상 충돌 → Always


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


urdf_path, out_path = sys.argv[1], sys.argv[2]
# build_collision_scene_graph=True 가 있어야 collision_scene 이 만들어진다
# (기본 False → collision_scene 이 None). force_collision_mesh=False 로 해야
# collision geometry 가 없는 링크를 visual 로 대체하지 않는다.
robot = yourdfpy.URDF.load(
    urdf_path, filename_handler=res,
    load_collision_meshes=True, build_collision_scene_graph=True,
    force_collision_mesh=False,
)

# ── 링크별 충돌 mesh 를 링크 프레임에 baking ──────────────────────────────
# ⚠ visual 이 아니라 collision_scene 을 쓴다. MoveIt 은 URDF 의 collision
#    geometry 로 검사하므로, ACM 을 visual 로 산출하면 MoveIt 이 실제로 보는
#    형상과 어긋난다(collision mesh 가 더 두꺼우면 "Never" 로 끈 쌍이 실제로는 충돌).
robot.update_cfg({j: 0.0 for j in robot.actuated_joint_names})
scene = robot.collision_scene
geom_of_link = defaultdict(list)
parents = scene.graph.transforms.parents
for gname in scene.geometry:
    node = gname
    while node in parents and node not in robot.link_map:
        node = parents[node]
    if node in robot.link_map:
        geom_of_link[node].append(gname)

link_mesh = {}
for link, gnames in geom_of_link.items():
    inv = np.linalg.inv(robot.get_transform(link))
    parts = []
    for g in gnames:
        T = scene.graph.get(g)[0]
        m = scene.geometry[g].copy()
        if len(m.faces) == 0:
            continue
        m.apply_transform(inv @ T)      # 링크 프레임 기준으로 baking
        parts.append(m)
    if parts:
        link_mesh[link] = trimesh.util.concatenate(parts)

links = sorted(link_mesh)
print(f"충돌 대상 링크 {len(links)}개")

mgr = trimesh.collision.CollisionManager()
for l in links:
    mgr.add_object(l, link_mesh[l])

# ── Adjacent: 관절로 직접 연결된 쌍 ───────────────────────────────────────
adjacent = set()
for jn, j in robot.joint_map.items():
    p, c = j.parent, j.child
    if p in link_mesh and c in link_mesh:
        adjacent.add(frozenset((p, c)))
# fixed joint 로 이어진 링크는 상위 실체 링크로 승격돼 인접성이 끊길 수 있어
# 링크 트리상 2-hop(같은 부모를 공유)도 인접 취급하지 않는다(Setup Assistant 동일).

def set_cfg(cfg):
    robot.update_cfg(cfg)
    for l in links:
        mgr.set_transform(l, np.array(robot.get_transform(l), float))


def collide_pairs():
    _, names = mgr.in_collision_internal(return_names=True)
    return {frozenset(p) for p in names}


# ── 시작 자세(Default) ────────────────────────────────────────────────────
zero = {j: 0.0 for j in robot.actuated_joint_names}
set_cfg(zero)
default_hits = collide_pairs() - adjacent
print(f"시작자세 충돌쌍(Default) {len(default_hits)}")

# ── 무작위 샘플링 ─────────────────────────────────────────────────────────
lims = {}
for j in robot.actuated_joint_names:
    jj = robot.joint_map[j]
    lo = jj.limit.lower if jj.limit and jj.limit.lower is not None else -np.pi
    hi = jj.limit.upper if jj.limit and jj.limit.upper is not None else np.pi
    lims[j] = (lo, hi)

rng = np.random.default_rng(0)
count = defaultdict(int)
for i in range(N_SAMPLES):
    set_cfg({j: rng.uniform(*lims[j]) for j in robot.actuated_joint_names})
    for p in collide_pairs():
        count[p] += 1
    if (i + 1) % 2000 == 0:
        print(f"  샘플 {i+1}/{N_SAMPLES}")

all_pairs = {frozenset((a, b)) for i, a in enumerate(links) for b in links[i + 1:]}
rows = []          # (link1, link2, reason)
kept = []
for pr in sorted(all_pairs, key=lambda s: sorted(s)):
    a, b = sorted(pr)
    n = count.get(pr, 0)
    if pr in adjacent:
        rows.append((a, b, "Adjacent"))
    elif n >= N_SAMPLES * ALWAYS_FRAC:
        rows.append((a, b, "Always"))
    elif n == 0:
        rows.append((a, b, "Never"))
    elif pr in default_hits:
        rows.append((a, b, "Default"))
    else:
        kept.append((a, b, n))

print(f"\nACM: 무시 {len(rows)}쌍 / 검사유지 {len(kept)}쌍 (전체 {len(all_pairs)})")
print("검사 유지되는(=실제 충돌 가능) 쌍 상위:")
for a, b, n in sorted(kept, key=lambda t: -t[2])[:12]:
    print(f"  {a:<22}{b:<22}{n/N_SAMPLES*100:5.1f}% 샘플에서 충돌")

# ── SRDF 출력 ─────────────────────────────────────────────────────────────
ARM_CHAIN = ("link0", "tcp")
GRIPPER_JOINT = "rg2_finger_joint1"
EE_PARENT = "rg2_hand"

L = []
L.append('<?xml version="1.0" encoding="UTF-8"?>')
L.append("<!-- 자동 생성: docs/scripts/gen_srdf.py — 직접 편집하지 말 것.")
L.append(f"     통합 URDF({os.path.basename(urdf_path)}) 기준, 무작위 {N_SAMPLES} 샘플로 ACM 산출.")
L.append("     모델(mounts.yaml)을 바꾸면 반드시 재생성. -->")
L.append('<robot name="rda_robot">')
L.append("    <!-- 플래닝 그룹 -->")
L.append('    <group name="arm">')
L.append(f'        <chain base_link="{ARM_CHAIN[0]}" tip_link="{ARM_CHAIN[1]}"/>')
L.append("    </group>")
L.append('    <group name="gripper">')
L.append(f'        <joint name="{GRIPPER_JOINT}"/>')
L.append("    </group>")
L.append('    <end_effector name="rg2" parent_link="tcp" group="gripper" parent_group="arm"/>')
L.append("")
L.append("    <!-- 이름있는 자세 -->")
L.append('    <group_state name="home" group="arm">')
for j in ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"):
    L.append(f'        <joint name="{j}" value="0"/>')
L.append("    </group_state>")
L.append('    <group_state name="open" group="gripper">')
L.append(f'        <joint name="{GRIPPER_JOINT}" value="0"/>')
L.append("    </group_state>")
L.append('    <group_state name="close" group="gripper">')
lo, hi = lims[GRIPPER_JOINT]
L.append(f'        <joint name="{GRIPPER_JOINT}" value="{hi:.4f}"/>')
L.append("    </group_state>")
L.append("")
L.append("    <!-- 충돌 무시 쌍 (Adjacent/Always/Never/Default) -->")
for a, b, why in rows:
    L.append(f'    <disable_collisions link1="{a}" link2="{b}" reason="{why}"/>')
L.append("</robot>")

with open(out_path, "w") as f:
    f.write("\n".join(L) + "\n")
print(f"\n생성: {out_path}")
