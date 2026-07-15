#!/usr/bin/env python3
"""URDF effort limit(10 Nm) 타당성 검증: 팔 자중+페이로드 정적 토크 실계산."""
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

root = ET.parse(sys.argv[1]).getroot()

ARM_LINKS = ["link1", "link2", "link3", "link4", "link5", "link6"]
PAYLOAD = 5.0  # kg, RB5 정격
G = 9.81

# 링크 질량·COM
mass, com = {}, {}
for l in root.findall("link"):
    i = l.find("inertial")
    if i is None:
        continue
    m = float(i.find("mass").get("value"))
    o = i.find("origin")
    c = [float(x) for x in (o.get("xyz") if o is not None else "0 0 0").split()]
    mass[l.get("name")] = m
    com[l.get("name")] = np.array(c)

joints = {}
for j in root.findall("joint"):
    o = j.find("origin")
    joints[j.get("name")] = dict(
        parent=j.find("parent").get("link"),
        child=j.find("child").get("link"),
        xyz=np.array([float(x) for x in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]),
        type=j.get("type"),
    )

by_child = {v["child"]: (k, v) for k, v in joints.items()}


def pos_in_base(link):
    """zero-config 에서 base_link 기준 링크 원점 (모든 origin rpy=0 확인됨)."""
    p = np.zeros(3)
    cur = link
    while cur in by_child:
        _, j = by_child[cur]
        p = p + j["xyz"]
        cur = j["parent"]
    return p


arm_mass = sum(mass.get(l, 0.0) for l in ARM_LINKS)
print(f"URDF 팔 링크 질량 합(link1..6) = {arm_mass:.2f} kg   (제원 전체 22 kg)")
print(f"페이로드 = {PAYLOAD} kg\n")

# shoulder 관절 위치 (최악: 팔을 수평으로 완전히 뻗은 자세)
# zero-config 는 수직이므로, 수평 뻗음의 모멘트암 = 각 질량의 shoulder 로부터의 체인 거리
sh = pos_in_base("link2")  # shoulder joint child
print("최악 자세(팔 수평 완전 신장) shoulder 정적 토크:")
tot = 0.0
for l in ARM_LINKS[1:]:
    p = pos_in_base(l) + com.get(l, np.zeros(3))
    # 수직 체인 거리 -> 수평 뻗었을 때 모멘트암
    arm_len = abs(p[2] - sh[2]) + abs(p[1]) + abs(p[0])
    t = mass.get(l, 0.0) * G * arm_len
    tot += t
    print(f"  {l:<8} m={mass.get(l,0):.2f}kg  모멘트암={arm_len:.3f}m  → {t:.1f} Nm")

tcp = pos_in_base("tcp")
pay_arm = abs(tcp[2] - sh[2])
t_pay = PAYLOAD * G * pay_arm
tot += t_pay
print(f"  {'payload':<8} m={PAYLOAD:.2f}kg  모멘트암={pay_arm:.3f}m  → {t_pay:.1f} Nm")
print(f"\n  shoulder 필요 정적 토크 합계 ≈ {tot:.1f} Nm")
print(f"  URDF effort limit           = 10 Nm")
print(f"  → 부족 배율 ≈ {tot/10:.1f}배  (페이로드 단독만도 {t_pay:.1f} Nm > 10 Nm)")
