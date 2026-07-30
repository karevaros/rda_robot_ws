#!/usr/bin/env python3
"""관절 effort(토크) 한계를 URDF 질량·관성에서 유도한다 — 플레이스홀더 10 Nm 교체용.

사용:
    python3 docs/scripts/joint_effort_derive.py <urdf> [--payload 5.0] [--sf 2.0]
                                                [--prefix ""] [--yaml <출력경로>]

■ 왜 유도하나 (2026-07-30 조사)
  벤더 URDF 는 6축 전부 `effort: 10` 이다(rbpodo_description/robots/rb5_850e/joint.yaml).
  자기 팔조차 들 수 없는 값이라 명백한 플레이스홀더다.
  그런데 **레인보우 공식 제원에 축별 토크가 없다** — `manual/appendix/system_specifications`
  에는 페이로드 5kg · 리치 927.7mm · 중량 22kg · 관절 최대 180°/s 까지만 있다.
  ⇒ '데이터시트 값으로 교체' 가 불가능하다. 그래서 **URDF 자체의 질량·관성에서
  요구 토크를 계산**하고, 안전계수를 곱한 값을 쓴다.

■ 이 값의 성격 (🔴 반드시 같이 읽을 것)
  **제조사 정격이 아니다.** '이 팔이 자기 무게와 정격 페이로드를 들고 규정 가속도로
  움직이려면 최소 이만큼은 필요하다' 는 **요구 하한 × 안전계수**다.
  · 쓸 수 있는 곳: 물리 시뮬(Gazebo·Isaac)에서 팔이 처지지 않게 하는 것.
    실제로 6주차에 Gazebo 에서 팔이 처져 **중력·충돌을 끄는 우회**를 했는데,
    원인의 하나가 이 값이다(10 Nm 로는 자기 팔을 못 든다).
  · 쓸 수 없는 곳: 토크 기반 제어·안전 정격 주장·충돌력 계산.
    그건 제조사 값이 있어야 한다(레인보우에 문의 필요).
  · 방향성: 위치제어(우리 사용법)에서 **너무 큰 값은 무해**하고(한계가 안 걸림)
    **너무 작은 값은 팔이 주저앉는다.** 그래서 넉넉한 쪽으로 잡는다.

■ 계산
  각 회전관절 j, 자세 q 에서
    τ_grav = Σ_{j 하류 링크 i} [ (COM_i − p_j) × (m_i·g) ] · â_j        (g = −z·9.81)
    I_axis = Σ_i [ â_j·(R_i I_i R_iᵀ)·â_j + m_i·d_i⊥² ]  (d⊥ = COM 과 관절축의 수직거리)
    τ_req  = |τ_grav| + I_axis · α_max
  를 자세를 훑어 최대값을 취한다. 페이로드는 tcp 위치의 점질량으로 넣는다.
  ⚠ α_max 는 `joint_limits.yaml` 의 추정치다(제조사 미공개) → 가속 항도 추정 성분을 갖는다.
  ⚠ 관성 텐서는 **실제 값으로 보인다**(질량합 20.84kg vs 제원 22kg · 텐서 비균일) —
    이 계산이 성립하는 근거. 만약 팔을 바꿔 텐서가 1e-4 같은 더미면 가속 항은 무의미하다.
"""
import argparse
import itertools
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np

G = np.array([0.0, 0.0, -9.81])

#: joint_limits.yaml 의 가속도 추정치(rad/s²). 없는 관절은 기본 3.0.
ALPHA_MAX = {"base": 3.0, "shoulder": 3.0, "elbow": 3.0,
             "wrist1": 3.0, "wrist2": 5.0, "wrist3": 5.0}


def _tf(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp,     cp * sr,                cp * cr]])
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, xyz
    return T


def _axis_rot(axis, ang):
    a = np.asarray(axis, float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(4)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * (K @ K)
    return T


def parse(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links = {}
    for l in root.findall("link"):
        i = l.find("inertial")
        if i is None:
            continue
        o = i.find("origin")
        xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
        rpy = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
        a = i.find("inertia").attrib
        I = np.array([[float(a["ixx"]), float(a["ixy"]), float(a["ixz"])],
                      [float(a["ixy"]), float(a["iyy"]), float(a["iyz"])],
                      [float(a["ixz"]), float(a["iyz"]), float(a["izz"])]])
        links[l.get("name")] = dict(m=float(i.find("mass").get("value")),
                                    T_com=_tf(xyz, rpy), I=I)
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
        rpy = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
        ae = j.find("axis")
        axis = [float(v) for v in ae.get("xyz").split()] if ae is not None else [1.0, 0, 0]
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            T=_tf(xyz, rpy), axis=axis, type=j.get("type"))
    return root, links, joints


def frames(joints, children, root_link, q):
    """root_link 아래 **트리 전체**의 링크 프레임 + 관절 프레임을 계산.

    체인만 훑으면 그리퍼·센서처럼 곁가지로 달린 질량이 빠진다 — 실측에서 그 때문에
    wrist3 요구토크가 0 으로 나왔다(tcp 가 wrist3 축 위에 있어 페이로드 점질량의
    수직거리가 0). 손가락·카메라는 축에서 벗어나 있으므로 트리를 다 봐야 맞다."""
    L = {root_link: np.eye(4)}
    J = {}
    stack = [root_link]
    while stack:
        parent = stack.pop()
        for jn in children.get(parent, []):
            j = joints[jn]
            Tj = L[parent] @ j["T"]
            J[jn] = Tj.copy()              # 관절 프레임(회전 전) = 축 원점·방향
            T = Tj
            if j["type"] in ("revolute", "continuous"):
                T = T @ _axis_rot(j["axis"], float(q.get(jn, 0.0)))
            L[j["child"]] = T
            stack.append(j["child"])
    return L, J


def distal_links(joints, children, jn):
    """관절 jn 하류의 모든 링크(곁가지 포함)."""
    out, stack = [], [joints[jn]["child"]]
    while stack:
        l = stack.pop()
        out.append(l)
        for cj in children.get(l, []):
            stack.append(joints[cj]["child"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("urdf")
    ap.add_argument("--payload", type=float, default=5.0, help="정격 페이로드 kg (RB5-850E=5)")
    ap.add_argument("--sf", type=float, default=2.0, help="안전계수")
    ap.add_argument("--prefix", default="", help="팔 링크 접두사(통합 URDF 의 RB5 는 빈 값)")
    ap.add_argument("--step-deg", type=float, default=30.0, help="자세 훑기 간격")
    ap.add_argument("--yaml", help="결과를 이 경로에 yaml 로 저장")
    a = ap.parse_args()

    P = a.prefix
    _, links, joints = parse(a.urdf)
    order = [P + n for n in ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]]
    missing = [j for j in order if j not in joints]
    if missing:
        sys.exit(f"error: URDF 에 팔 관절이 없습니다: {missing}\n"
                 f"  --prefix 를 확인하세요(통합 URDF 의 RB5 는 접두사 없음).")

    children = {}
    for jn, j in joints.items():
        children.setdefault(j["parent"], []).append(jn)
    tcp = P + "tcp"
    if tcp not in [j["child"] for j in joints.values()]:
        sys.exit(f"error: {tcp} 링크로 가는 관절을 찾지 못했습니다.")

    root = joints[order[0]]["parent"]                 # 팔 베이스(link0)
    distal = {jn: distal_links(joints, children, jn) for jn in order}

    chain_mass = sum(links[l]["m"] for l in distal[order[0]] if l in links)
    ee_mass = sum(links[l]["m"] for l in distal[order[5]] if l in links)
    print(f"팔 하류 총질량 = {chain_mass:.2f} kg (그 중 wrist3 하류 = {ee_mass:.2f} kg, "
          f"그리퍼·센서 포함)  ·  페이로드 {a.payload} kg  ·  안전계수 ×{a.sf}")

    # ── 자세 훑기: 중력 토크는 base 회전에 불변(중력이 수직) → base=0 고정.
    #    지배적인 shoulder·elbow·wrist1 을 촘촘히, wrist2·wrist3 은 거칠게.
    st = math.radians(a.step_deg)
    n_fine = int(round(2 * math.pi / st))
    fine = [-math.pi + k * st for k in range(n_fine)]
    coarse = [-math.pi + k * (math.pi / 2) for k in range(4)]
    grid = list(itertools.product(fine, fine, fine, coarse, [0.0, math.pi]))

    peak = {j: 0.0 for j in order}          # 요구 토크(중력+가속) 최대
    peak_grav = {j: 0.0 for j in order}      # 중력 성분만의 최대(별도 추적)
    peak_q = {j: None for j in order}

    for sh, el, w1, w2, w3 in grid:
        q = {order[0]: 0.0, order[1]: sh, order[2]: el,
             order[3]: w1, order[4]: w2, order[5]: w3}
        L, J = frames(joints, children, root, q)
        c_tcp = L[tcp][:3, 3]

        for jn in order:
            Tj = J[jn]
            pj = Tj[:3, 3]
            aj = Tj[:3, :3] @ np.asarray(joints[jn]["axis"], float)
            aj = aj / np.linalg.norm(aj)

            tau_g, I_ax = 0.0, 0.0
            for l in distal[jn]:
                if l not in links:
                    continue
                m, Il = links[l]["m"], links[l]["I"]
                Tc = L[l] @ links[l]["T_com"]
                c, Rc = Tc[:3, 3], Tc[:3, :3]
                tau_g += np.dot(np.cross(c - pj, m * G), aj)
                d = (c - pj) - np.dot(c - pj, aj) * aj      # 축까지 수직 벡터
                I_ax += float(aj @ (Rc @ Il @ Rc.T) @ aj) + m * float(d @ d)
            # 페이로드 = tcp 위치의 점질량(정격 페이로드는 툴+공작물을 뜻한다)
            tau_g += np.dot(np.cross(c_tcp - pj, a.payload * G), aj)
            d = (c_tcp - pj) - np.dot(c_tcp - pj, aj) * aj
            I_ax += a.payload * float(d @ d)

            tau = abs(tau_g) + I_ax * ALPHA_MAX.get(jn[len(P):], 3.0)
            peak_grav[jn] = max(peak_grav[jn], abs(tau_g))
            if tau > peak[jn]:
                peak[jn], peak_q[jn] = tau, (sh, el, w1, w2, w3)

    print(f"\n자세 {len(grid)}개 훑음({a.step_deg:.0f}° 간격) — 관절별 최악 요구 토크")
    print(f"{'관절':<9}{'중력max':>9}{'요구max':>9}{'×SF':>8}   최악자세(shoulder,elbow,wrist1°)")
    out = {}
    for jn in order:
        val = peak[jn] * a.sf
        out[jn] = val
        q = peak_q[jn]
        pose = f"({math.degrees(q[0]):.0f}, {math.degrees(q[1]):.0f}, {math.degrees(q[2]):.0f})"
        print(f"{jn:<9}{peak_grav[jn]:>8.1f}N{peak[jn]:>8.1f}N{val:>7.1f}N   {pose}")

    # ── 유도 불가 구간을 바닥값으로 채운다 (🔴 여기만 규칙에 따른 선택이다)
    #   wrist3 는 **툴 롤 축**이라 중력이 원리적으로 걸리지 않는다(대칭 그리퍼의 COM 이
    #   축 위에 있고, 정격 페이로드도 tcp=축 위의 점질량으로 잡았다) → 요구 ≈ 0 이 나온다.
    #   0 을 effort 로 쓸 수는 없고, 중력에서 유도할 방법도 없다.
    #   규칙: **손목 3축 중 최대 요구값을 바닥값으로 삼는다.** 협동로봇 손목은 보통 같은
    #   액추에이터 모듈을 쓰고, 직렬 손목에서 하류 관절이 상류보다 약한 건 부자연스럽다.
    #   (위치제어에서 값이 큰 쪽은 무해하고 작은 쪽은 주저앉는다 — 스크립트 상단 참조)
    floor = max(out[j] for j in order[3:])
    raised = [j for j in order if out[j] < floor]
    for j in raised:
        out[j] = floor
    if raised:
        print(f"\n바닥값 {floor:.0f} Nm 적용(손목 최대 요구값) → {', '.join(raised)}")
        print("  근거: 툴 롤 축은 중력에서 유도 불가 · 손목은 통상 동일 모듈 · "
              "위치제어에선 큰 값이 무해")
    out = {j: round(v) for j, v in out.items()}

    print(f"\n현재 URDF 값 = 10 Nm (전축 동일) → 최대 부족 배율 "
          f"≈ {max(out.values())/10:.0f}배")

    if a.yaml:
        with open(a.yaml, "w") as f:
            f.write("# 자동 생성 — docs/scripts/joint_effort_derive.py\n")
            f.write(f"# URDF 질량·관성에서 유도한 **요구 하한 × 안전계수 {a.sf}**.\n")
            f.write("# 🔴 제조사 정격이 아니다(레인보우 공식 제원에 축별 토크가 없다).\n")
            f.write("#    쓸 수 있는 곳 = 물리 시뮬에서 팔이 처지지 않게 하기.\n")
            f.write("#    쓸 수 없는 곳 = 토크 기반 제어·안전 정격 주장·충돌력 계산.\n")
            f.write(f"# 조건: 페이로드 {a.payload}kg · 자세 {len(grid)}개({a.step_deg:.0f}°) · "
                    f"가속도는 joint_limits.yaml 추정치\n")
            f.write("effort:\n")
            for jn in order:
                f.write(f"  {jn}: {out[jn]:.0f}\n")
        print(f"\n저장: {a.yaml}")


if __name__ == "__main__":
    main()
