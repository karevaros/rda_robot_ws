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
    """→ (joints, child_to_joint). joints[name]={type,parent,child,T,mimic}."""
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
        joints[name] = dict(type=typ, parent=parent, child=child,
                            T=_tf(xyz, rpy),
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


def playback_joints(srdf_xml, urdf_xml, arm_group="arm", gripper_group="gripper"):
    """재생/계획에 필요한 관절 이름을 SRDF+URDF 에서 자동 유도.

    반환: dict(arm=[...구동...], gripper_drivers=[...], gripper_all=[...+mimic...])."""
    joints, c2j = parse_urdf(urdf_xml)
    arm = group_joints(srdf_xml, joints, c2j, arm_group, actuated_only=True)
    gdrv = group_joints(srdf_xml, joints, c2j, gripper_group, actuated_only=True)
    gall = list(gdrv) + mimics_of(joints, gdrv)
    return dict(arm=arm, gripper_drivers=gdrv, gripper_all=gall,
                joints=joints, child_to_joint=c2j)
