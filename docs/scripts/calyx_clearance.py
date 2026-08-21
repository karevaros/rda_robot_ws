#!/usr/bin/env python3
"""꽃받침(calyx) 여유 측정 — "꽃받침을 모델에 넣어야 하는가" 를 숫자로 가른다.

묻는 것: 파지 자세에서 **열매 꼭지**(꽃받침이 붙는 자리 = 열매 +z 극점)와 그리퍼 표면
사이가 얼마나 떨어져 있나. 그 거리보다 작은 꽃받침은 **어느 방향으로 뻗든** 닿지 않는다.

왜 이 방식인가: 꽃받침 치수를 사진에서 잴 수 없었다(스케일 기준 없음 · 자동 세그가
배경 초록에 오염돼 비율이 0.02~0.34 로 튐). 그래서 *크기를 가정해 넣어 보는* 대신
**닿기 시작하는 문턱**을 재고, 현실 치수가 그 아래인지로 판단한다.

보수적으로 잡은 것
  · 그리퍼 관절 **0 rad**(가장 닫힌 상태) 로 잰다. 실제 운용은 0.35~1.18 rad 로 더
    벌어져 있어 여유가 더 크다.
  · 꼭지 **점**에서 그리퍼 표면까지의 **최단거리** — 꽃받침이 어느 방향으로 뻗어도
    이 값 안이면 안 닿는다(충분조건).

사용: python3 docs/scripts/calyx_clearance.py [urdf경로]
      (생략 시 정본 mounts 로 즉석 합성)
"""
import importlib.util
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

SRC = os.path.expanduser("~/robot_ws/src")
APPROACH_AXIS = [0.0, -1.0, 0.0]     # tcp 기준 접근축(RB5+RG2 실측)
FRUIT_R = 0.035
GRASP_OFF = 0.1672                   # 파지거리 — gripper_pad_center + 손바닥 여유(실측)
CLUSTER_GAP_RATIO = 0.85             # obstacles.yaml crops.template.truss.cluster_gap


def _load(path, name):
    s = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(s)
    sys.modules[name] = m
    s.loader.exec_module(m)
    return m


def _quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def gripper_points(urdf, RI):
    """그리퍼 표면점(tcp 프레임). 손·손가락 mesh 정점."""
    import trimesh
    joints, c2j = RI.parse_urdf(urdf)
    geoms = RI.link_geoms(urdf)

    def resolve(fn):
        p = fn.replace("package://rda_robot_description",
                       os.path.join(SRC, "rda_robot_description"))
        return p if os.path.exists(p) else None

    out = {}
    for ln in ("rg2_hand", "rg2_leftfinger", "rg2_rightfinger"):
        T = RI.compose_tf(joints, c2j, "tcp", ln)
        if T is None:
            continue
        acc = []
        for kind, data, Tg in geoms.get(ln, []):
            if kind != "mesh":
                continue
            p = resolve(data)
            if not p:
                continue
            V = np.asarray(trimesh.load(p, force="mesh").vertices, float)
            W = T @ Tg
            acc.append((W[:3, :3] @ V.T).T + W[:3, 3])
        if acc:
            out[ln] = np.vstack(acc)
    return out


def main():
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else None
    if urdf_path is None:
        fd, urdf_path = tempfile.mkstemp(suffix=".urdf")
        os.close(fd)
        subprocess.run(["ros2", "run", "rda_robot_assembler", "compose_urdf",
                        "-o", urdf_path], check=True, stderr=subprocess.DEVNULL)
    urdf = open(urdf_path).read()
    RI = _load(os.path.join(SRC, "rda_robot_bringup/scripts/robot_introspect.py"), "ri")
    PG = _load(os.path.join(SRC, "rda_robot_bringup/scripts/pregrasp_pose.py"), "pg")
    OP = _load(os.path.join(SRC, "rda_robot_bringup/scripts/obstacle_publisher.py"), "op")

    pts = gripper_points(urdf, RI)
    G = np.vstack(list(pts.values()))
    a0 = np.array([1.0, 0.0, 0.0])          # 거터 법선(정면)
    gap = CLUSTER_GAP_RATIO * 2 * FRUIT_R
    offs = OP._cluster_offsets(4, gap)

    print(f"열매 r={FRUIT_R*100:.1f}cm · 파지거리 {GRASP_OFF*100:.2f}cm · "
          f"클러스터 간격 {gap*100:.2f}cm · 그리퍼 관절 0(가장 닫힘, 보수적)")
    print("\n[A] 목표 열매 자신의 꼭지 — 접근각 θ 별")
    print(f"{'θ':>5s} | {'손가락':>8s} {'손바닥':>8s} | 문턱(이보다 큰 꽃받침이면 접촉)")
    for th in (0, 15, 30, -15, -30):
        a = PG.approach_dir(a0, 0.0, math.radians(th))
        Rw = _quat_to_R(PG.mat_to_quat(
            PG.gaze_rotation(a, 0.0, APPROACH_AXIS)))
        t = -a * GRASP_OFF
        pole = np.array([0.0, 0.0, FRUIT_R])
        d = {ln: float(np.min(np.linalg.norm((Rw @ P.T).T + t - pole, axis=1)))
             for ln, P in pts.items()}
        fing = min(d["rg2_leftfinger"], d["rg2_rightfinger"])
        print(f"{th:+5.0f} | {fing*100:7.2f}cm {d['rg2_hand']*100:7.2f}cm | "
              f"**{min(fing, d['rg2_hand'])*100:.1f}cm**")

    print("\n[B] 같은 화방 이웃 열매의 꼭지 (목표=f0 를 잡는 자세에서)")
    for th in (0, 30):
        a = PG.approach_dir(a0, 0.0, math.radians(th))
        Rw = _quat_to_R(PG.mat_to_quat(PG.gaze_rotation(a, 0.0, APPROACH_AXIS)))
        Gw = (Rw @ G.T).T + (-a * GRASP_OFF)
        row = []
        for i, o in enumerate(offs):
            pole = np.array(o, float) + np.array([0, 0, FRUIT_R])
            row.append(f"f{i}={float(np.min(np.linalg.norm(Gw - pole, axis=1)))*100:.2f}cm")
        print(f"  θ={th:+3.0f}° : " + " · ".join(row) + "   (f0=목표)")
    print("\n⚠ [B] 는 새 문제가 아니다 — 이 모델의 열매는 애초에 서로 겹쳐 있고"
          f"(중심 {gap*100:.2f}cm < 반지름 합 {2*FRUIT_R*100:.1f}cm),")
    print("   열매를 충돌객체로 켜면 '손에 든 열매 ↔ 이웃 열매' 접촉이 이미 잡힌다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
