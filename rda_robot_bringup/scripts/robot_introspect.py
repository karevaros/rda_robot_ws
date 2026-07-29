#!/usr/bin/env python3
"""로봇 모델 introspection — URDF/SRDF 에서 그룹 관절·접근축을 자동 유도.

pregrasp 알고리즘을 **모델 불문**으로 만들기 위한 순수 파서(ROS 무관, xml+numpy).
  · group_joints(srdf, urdf, "arm")     → 팔 구동 관절 순서열(chain link0→tcp 자동 추적)
  · group_joints(srdf, urdf, "gripper") → 그리퍼 구동 관절(+ mimic 포함 시 grip_playback_joints)
  · detect_approach_axis(...)           → 그리퍼 손끝이 tcp 로컬 어느 축으로 뻗는지 FK 로 감지
                                          (RG2=−Y, 다른 그리퍼면 자동으로 그 축)

이 값들만 있으면 gaze pose·궤적 재생이 팔/그리퍼 종류에 의존하지 않는다.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def _tf(xyz, rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


def parse_urdf(urdf_xml):
    """→ (joints, child_to_joint). joints[name]={type,parent,child,T,axis,mimic}."""
    root = ET.fromstring(urdf_xml)
    joints, child_to_joint = {}, {}
    for j in root.findall("joint"):
        name, typ = j.get("name"), j.get("type")
        pe, ce = j.find("parent"), j.find("child")
        if pe is None or ce is None:
            continue
        parent, child = pe.get("link"), ce.get("link")
        o = j.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if o is not None:
            if o.get("xyz"):
                xyz = [float(v) for v in o.get("xyz").split()]
            if o.get("rpy"):
                rpy = [float(v) for v in o.get("rpy").split()]
        mim = j.find("mimic")
        # 관절 회전/이동 축(URDF 기본 = X). 관절값을 넣은 FK(자세별 TCP 위치)에 필요.
        ae = j.find("axis")
        axis = [1.0, 0.0, 0.0]
        if ae is not None and ae.get("xyz"):
            axis = [float(v) for v in ae.get("xyz").split()]
        joints[name] = dict(type=typ, parent=parent, child=child,
                            T=_tf(xyz, rpy), axis=axis,
                            mimic=(mim.get("joint") if mim is not None else None))
        child_to_joint[child] = name
    return joints, child_to_joint


def _chain(joints, child_to_joint, base_link, tip_link):
    """base_link → tip_link 사이 관절 이름을 순서대로(부모→자식)."""
    path, link, guard = [], tip_link, 0
    while link != base_link and guard < 10000:
        jn = child_to_joint.get(link)
        if jn is None:
            return []                       # base 에 못 닿음
        path.append(jn)
        link = joints[jn]["parent"]
        guard += 1
    path.reverse()
    return path


_ACTUATED = ("revolute", "prismatic", "continuous")


def group_joints(srdf_xml, joints, child_to_joint, group_name, actuated_only=True):
    """SRDF 그룹의 관절 이름. chain/joint/link 정의 모두 지원."""
    root = ET.fromstring(srdf_xml)
    grp = next((g for g in root.findall("group") if g.get("name") == group_name), None)
    if grp is None:
        return []
    out = []
    ch = grp.find("chain")
    if ch is not None:
        out = _chain(joints, child_to_joint, ch.get("base_link"), ch.get("tip_link"))
    else:
        for je in grp.findall("joint"):
            out.append(je.get("name"))
        for le in grp.findall("link"):
            jn = child_to_joint.get(le.get("name"))
            if jn and jn not in out:
                out.append(jn)
    if actuated_only:
        out = [n for n in out
               if joints.get(n, {}).get("type") in _ACTUATED
               and not joints[n].get("mimic")]
    return out


def mimics_of(joints, drivers):
    """drivers 를 mimic 하는 관절들(그리퍼 재생 시 함께 발행)."""
    dv = set(drivers)
    return [n for n, j in joints.items() if j.get("mimic") in dv]


def compose_tf(joints, child_to_joint, from_link, to_link):
    """cfg=0 에서 to_link 원점을 from_link 프레임으로(고정/영자세 관절 origin 합성)."""
    T, link, guard = np.eye(4), to_link, 0
    while link != from_link and guard < 10000:
        jn = child_to_joint.get(link)
        if jn is None:
            return None
        T = joints[jn]["T"] @ T
        link = joints[jn]["parent"]
        guard += 1
    return T if link == from_link else None


def fk_chain(joints, child_to_joint, base_link, tip_link):
    """base_link→tip_link 관절 순서열(고정관절 포함). fk_pos 에 넘길 체인."""
    return _chain(joints, child_to_joint, base_link, tip_link)


def fk_pos(joints, chain, q):
    """관절값 q(name→rad/m)를 넣어 tip 원점을 base 프레임에서 계산 → np.array([x,y,z]).

    ROS 서비스(/compute_fk) 없이 로컬로 푼다(웨이포인트 수백 개의 TCP 경로길이를
    재는 데 서비스 왕복은 너무 느리다). revolute/continuous=축 회전, prismatic=축 이동.
    """
    T = np.eye(4)
    for jn in chain:
        j = joints.get(jn)
        if j is None:
            continue
        T = T @ j["T"]
        typ = j.get("type")
        if typ in ("revolute", "continuous"):
            T = T @ _axis_rot(j.get("axis", [1.0, 0.0, 0.0]), float(q.get(jn, 0.0)))
        elif typ == "prismatic":
            D = np.eye(4)
            ax = np.asarray(j.get("axis", [1.0, 0.0, 0.0]), float)
            D[:3, 3] = ax * float(q.get(jn, 0.0))
            T = T @ D
    return T[:3, 3].copy()


def _axis_rot(axis, ang):
    """축-각 회전(4x4). Rodrigues."""
    a = np.asarray(axis, float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return np.eye(4)
    a = a / n
    c, s = math.cos(ang), math.sin(ang)
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + s * K + (1.0 - c) * (K @ K)
    return T


def detect_approach_axis(srdf_xml, joints, child_to_joint, tcp_link):
    """그리퍼 손끝(그리퍼 그룹 관절의 자식 링크)들의 tcp 프레임상 평균 방향 = 접근축.

    RG2 는 −Y 로 나온다. 다른 그리퍼면 그 그리퍼가 뻗는 축이 자동으로 잡힌다.
    반환: 단위벡터 리스트 [x,y,z] (tcp 로컬) 또는 None(감지 실패 → 호출측이 기본값 사용)."""
    grip = group_joints(srdf_xml, joints, child_to_joint, "gripper", actuated_only=False)
    grip = list(grip) + mimics_of(joints, grip)
    finger_links = [joints[n]["child"] for n in grip if n in joints]
    pts = []
    for fl in finger_links:
        T = compose_tf(joints, child_to_joint, tcp_link, fl)
        if T is not None:
            pts.append(T[:3, 3])
    if not pts:
        return None
    mid = np.mean(pts, axis=0)
    n = float(np.linalg.norm(mid))
    return (mid / n).tolist() if n > 1e-6 else None


def link_geoms(urdf_xml):
    """링크별 기하 목록 → {link: [(kind, data, T_link_geom), ...]}.

    kind='mesh'(data=filename) / 'box'(size) / 'cylinder'(r,h) / 'sphere'(r).
    그리퍼가 접근축으로 얼마나 뻗는지(=손끝 깊이) 재는 데 쓴다.

    ⚠ collision 우선, **없으면 visual 로 폴백**한다 — control 모드 URDF 는 로봇 링크의
      collision 을 제거하므로(Gazebo 물리 접촉 회피) collision 만 보면 손끝 깊이를 못 잰다
      (실측: 손가락 깊이가 링크 원점뿐인 10.5~10.5cm 로 나와 자동유도가 거부됐다)."""
    root = ET.fromstring(urdf_xml)
    out = {}
    for ln in root.findall("link"):
        items = []
        for col in (ln.findall("collision") or []) or ln.findall("visual"):
            g = col.find("geometry")
            if g is None:
                continue
            o = col.find("origin")
            xyz = [float(v) for v in (o.get("xyz").split()
                                      if o is not None and o.get("xyz") else [0, 0, 0])]
            rpy = [float(v) for v in (o.get("rpy").split()
                                      if o is not None and o.get("rpy") else [0, 0, 0])]
            T = _tf(xyz, rpy)
            for kind, attr in (("mesh", "filename"), ("box", "size"),
                               ("cylinder", None), ("sphere", None)):
                e = g.find(kind)
                if e is None:
                    continue
                if kind == "mesh":
                    items.append(("mesh", e.get("filename"), T))
                elif kind == "box":
                    items.append(("box", [float(v) for v in e.get("size").split()], T))
                elif kind == "cylinder":
                    items.append(("cylinder",
                                  [float(e.get("radius")), float(e.get("length"))], T))
                else:
                    items.append(("sphere", [float(e.get("radius"))], T))
        if items:
            out[ln.get("name")] = items
    return out


def _movable_children_below(joints, child_to_joint, root_link):
    """root_link 아래(자손)에서 **움직이는 관절의 자식 링크** 목록.
    SRDF 가 낡아 그리퍼 관절 이름이 안 맞을 때의 폴백(손가락 후보)."""
    kids = {}
    for n, j in joints.items():
        kids.setdefault(j["parent"], []).append(n)
    out, stack = [], [root_link]
    seen = set()
    while stack:
        link = stack.pop()
        if link in seen:
            continue
        seen.add(link)
        for n in kids.get(link, []):
            j = joints[n]
            if j["type"] in ("revolute", "prismatic", "continuous"):
                out.append(j["child"])
            stack.append(j["child"])
    return out


def gripper_span(urdf_xml, srdf_xml, tcp_link, approach_axis, gripper_group="gripper",
                 mesh_resolver=None):
    """그리퍼 손가락이 **접근축 방향으로 차지하는 깊이 구간** (near, far)[m] — tcp 기준.

    파지 지점을 상수(`grasp_offset`)로 두면 그리퍼를 바꿨을 때 조용히 틀린다
    (예: 손가락 5cm 짜리로 바꾸면 열매가 손끝 너머에 놓여 빈손으로 닫힌다).
    → 손가락 링크의 collision 기하를 tcp 프레임으로 옮겨 접근축에 투영해 실제 깊이를 잰다.
    RG2 실측: (0.091, 0.213) m.

    `mesh_resolver(filename) -> 로컬 경로` 를 주면 mesh 도 반영한다(없으면 mesh 는 건너뛰고
    링크 원점만 쓴다 — 그 경우 손끝 깊이를 과소평가하므로 호출측이 알아서 판단할 것).
    반환: (near, far) 또는 None(그리퍼 관절이 없는 등 감지 실패)."""
    joints, c2j = parse_urdf(urdf_xml)
    grip = group_joints(srdf_xml, joints, c2j, gripper_group, actuated_only=False)
    grip = list(grip) + mimics_of(joints, grip)
    finger_links = [joints[n]["child"] for n in grip if n in joints]
    if not finger_links:
        # SRDF 가 다른 그리퍼 기준으로 남아 있으면(스왑 후 gen_srdf 미실행) 이름이 안 맞는다.
        # → URDF 만으로 폴백: **tcp 아래로 매달린 움직이는 관절**의 자식 링크를 손가락으로 본다.
        finger_links = _movable_children_below(joints, c2j, tcp_link)
    if not finger_links:
        return None
    a = np.asarray(approach_axis, float)
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        return None
    a = a / n
    geoms = link_geoms(urdf_xml)
    lo, hi = float("inf"), float("-inf")
    for fl in finger_links:
        T = compose_tf(joints, c2j, tcp_link, fl)
        if T is None:
            continue
        d0 = float(T[:3, 3] @ a)
        lo, hi = min(lo, d0), max(hi, d0)          # 링크 원점(기하 없어도 최소한 이건 반영)
        for kind, data, Tg in geoms.get(fl, []):
            pts = None
            if kind == "mesh" and mesh_resolver is not None:
                try:
                    import trimesh
                    path = mesh_resolver(data)
                    if path:
                        pts = np.asarray(trimesh.load(path, force="mesh").vertices, float)
                except Exception:                   # noqa: BLE001 — 기하 없으면 원점만 사용
                    pts = None
            elif kind == "box":
                h = np.array(data) / 2.0
                pts = np.array([[sx * h[0], sy * h[1], sz * h[2]]
                                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            elif kind == "cylinder":
                r, h = data
                pts = np.array([[sx * r, sy * r, sz * h / 2.0]
                                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            elif kind == "sphere":
                r = data[0]
                pts = np.array([[sx * r, sy * r, sz * r]
                                for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            if pts is None or not len(pts):
                continue
            W = (T @ Tg)                            # tcp → 기하 프레임
            P = (W[:3, :3] @ pts.T).T + W[:3, 3]
            d = P @ a
            lo, hi = min(lo, float(d.min())), max(hi, float(d.max()))
    if not np.isfinite(lo) or not np.isfinite(hi):
        return None
    return (lo, hi)


def playback_joints(srdf_xml, urdf_xml, arm_group="arm", gripper_group="gripper"):
    """재생/계획에 필요한 관절 이름을 SRDF+URDF 에서 자동 유도.

    반환: dict(arm=[...구동...], gripper_drivers=[...], gripper_all=[...+mimic...])."""
    joints, c2j = parse_urdf(urdf_xml)
    arm = group_joints(srdf_xml, joints, c2j, arm_group, actuated_only=True)
    gdrv = group_joints(srdf_xml, joints, c2j, gripper_group, actuated_only=True)
    gall = list(gdrv) + mimics_of(joints, gdrv)
    return dict(arm=arm, gripper_drivers=gdrv, gripper_all=gall,
                joints=joints, child_to_joint=c2j)
