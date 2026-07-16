#!/usr/bin/env python3
"""URDF effort limit(10 Nm) 타당성 검증: 팔 자중+페이로드 정적 토크 실계산.

사용:  torque_check.py <urdf> [링크접두사] [페이로드kg]
  링크접두사: RB5 는 무접두("") — 통합 URDF 의 링크명 계약. 그 외 팔은 슬롯 접두사가
              붙는다(예: arm__rb10_1300e → "arm_"). compose_urdf 의 prefix 규칙 참조.
  페이로드  : 기본 5.0 kg (RB5-850e 정격). 팔을 바꾸면 그 팔의 정격으로 줄 것.
"""
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

root = ET.parse(sys.argv[1]).getroot()
PREFIX = sys.argv[2] if len(sys.argv) > 2 else ""
PAYLOAD = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0  # kg, RB5-850e 정격

ARM_LINKS = [PREFIX + n for n in
             ["link1", "link2", "link3", "link4", "link5", "link6"]]
TCP = PREFIX + "tcp"
SHOULDER = PREFIX + "link2"
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
sh = pos_in_base(SHOULDER)  # shoulder joint child
print("최악 자세(팔 수평 완전 신장) shoulder 정적 토크:")
tot = 0.0
for l in ARM_LINKS[1:]:
    p = pos_in_base(l) + com.get(l, np.zeros(3))
    # 수직 체인 거리 -> 수평 뻗었을 때 모멘트암
    arm_len = abs(p[2] - sh[2]) + abs(p[1]) + abs(p[0])
    t = mass.get(l, 0.0) * G * arm_len
    tot += t
    print(f"  {l:<8} m={mass.get(l,0):.2f}kg  모멘트암={arm_len:.3f}m  → {t:.1f} Nm")

tcp = pos_in_base(TCP)
# 링크와 같은 식을 쓴다. 예전엔 payload 만 abs(tcp[2]-sh[2]) 로 z 차이만 봐서 손목의
# 측방 오프셋을 통째로 빠뜨렸다(RB5 는 tcp 가 link6 에서 y 로 96.7mm 나가므로 과소평가).
pay_arm = abs(tcp[2] - sh[2]) + abs(tcp[1]) + abs(tcp[0])
t_pay = PAYLOAD * G * pay_arm
tot += t_pay
print(f"  {'payload':<8} m={PAYLOAD:.2f}kg  모멘트암={pay_arm:.3f}m  → {t_pay:.1f} Nm")
print(f"\n  shoulder 필요 정적 토크 합계 ≈ {tot:.1f} Nm")
print(f"  URDF effort limit           = 10 Nm")
print(f"  → 부족 배율 ≈ {tot/10:.1f}배  (페이로드 단독만도 {t_pay:.1f} Nm > 10 Nm)")
