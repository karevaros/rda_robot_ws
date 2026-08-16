#!/usr/bin/env python3
"""Unity 내보내기 검증 — Unity 없이 할 수 있는 데까지 (7주차 M4)

`gen_unity_assets.py` 산출물을 검사한다. Unity 가 설치돼 있지 않아도 **대부분은 지금 검증된다**:

  1. 프로젝트 뼈대     manifest.json(URDF Importer) · ProjectVersion · 에디터 스크립트
  2. mesh 완결성       URDF 의 package:// 참조가 전부 Assets 안에 있고 원본과 바이트 동일
  3. 온실 데이터       greenhouse.json 개수·자세·치수 = Gazebo 월드(SDF)
  4. 좌표 변환 규약    C# 에 넣은 FLU→RUF 식(위치 (x,y,z)→(−y,z,x), 회전 (x,y,z,w)→(−y,z,x,−w))이
                       **서로 정합인지** 수치로 확인 — Unity 를 못 켜니 식 자체를 검증한다
  5. (선택) 배치 리포트 `rda_unity_report.json` 이 있으면 프레임 위치를 URDF FK 와 대조

사용: python3 src/docs/scripts/test_unity_export.py [--project ~/robot_ws/export/unity]
"""
import argparse
import filecmp
import importlib.util
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

_SRC = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
_SCRIPTS = os.path.join(_SRC, "rda_robot_bringup", "scripts")

_ok, _fail = [], []


def check(name, cond, detail=""):
    (_ok if cond else _fail).append(name)
    print(f"{'✅' if cond else '❌'} {name}{('  ' + detail) if detail else ''}")
    return cond


def _import(mod_name, path):
    spec = importlib.util.spec_from_file_location(mod_name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ── C# 에 넣은 변환식을 파이썬으로 그대로 옮긴 것(같은 식이어야 한다) ──
def ros_to_unity_pos(v):
    return np.array([-v[1], v[2], v[0]])


def _rpy_matrix(r, p, y):
    """URDF rpy(고정축 XYZ, = Rz·Ry·Rx) → 3×3 회전행렬."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def ros_to_unity_quat(q):
    """q = (x, y, z, w) → Unity (x, y, z, w)."""
    x, y, z, w = q
    return np.array([-y, z, x, -w])


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", default=os.path.expanduser("~/robot_ws/export/unity"))
    a = ap.parse_args()
    P = os.path.abspath(os.path.expanduser(a.project))
    G = _import("_gen_usd_scene", os.path.join(_SCRIPTS, "gen_usd_scene.py"))

    # 1. 프로젝트 뼈대
    man = os.path.join(P, "Packages", "manifest.json")
    check("Packages/manifest.json 존재", os.path.exists(man))
    if os.path.exists(man):
        dep = json.load(open(man)).get("dependencies", {})
        check("URDF Importer 의존성 명시",
              "com.unity.robotics.urdf-importer" in dep,
              dep.get("com.unity.robotics.urdf-importer", ""))
    pv = os.path.join(P, "ProjectSettings", "ProjectVersion.txt")
    check("ProjectVersion.txt 존재", os.path.exists(pv),
          open(pv).read().strip() if os.path.exists(pv) else "")
    for f in ("GreenhouseBuilder.cs", "RdaBatch.cs"):
        check(f"에디터 스크립트 {f}", os.path.exists(os.path.join(P, "Assets", "Editor", f)))

    # 2. mesh 완결성 — URDF Importer 는 package:// 를 URDF 폴더 기준으로 푼다
    urdf_path = os.path.join(P, "Assets", "RdaRobot", "rda_robot.urdf")
    check("Assets 안에 통합 URDF", os.path.exists(urdf_path))
    urdf = open(urdf_path).read()
    uris = sorted(set(re.findall(r'filename="(package://[^"]+)"', urdf)))
    bad, diff = [], []
    for uri in uris:
        m = re.match(r"package://([^/]+)/(.+)", uri)
        dst = os.path.join(P, "Assets", "RdaRobot", m.group(1), m.group(2))
        if not os.path.exists(dst):
            bad.append(uri)
            continue
        src = G.resolve_pkg(uri, [])
        if src and not filecmp.cmp(src, dst, shallow=False):
            diff.append(uri)
    check(f"mesh {len(uris)}개가 Assets 안에 존재", not bad, str(bad[:3]))
    check("복사한 mesh 가 원본과 동일", not diff, str(diff[:3]))

    # 3. 온실 데이터 = Gazebo 월드(SDF)
    gz = _import("_gen_gazebo_world", os.path.join(_SCRIPTS, "gen_gazebo_world.py"))
    sdf = ET.fromstring(gz.build_world(os.path.join(_SRC, "rda_robot_description", "config",
                                                    "obstacles.yaml")))
    sdf_models = {m.get("name"): m for m in sdf.iter("model")}
    items = json.load(open(os.path.join(P, "Assets", "RdaRobot", "greenhouse.json")))["items"]
    check("온실 항목 수 = Gazebo 월드 모델 수", len(items) == len(sdf_models),
          f"json {len(items)} vs SDF {len(sdf_models)}")
    pbad, sbad = [], []
    for it in items:
        m = sdf_models.get(it["name"])
        if m is None:
            pbad.append(it["name"])
            continue
        pose = [float(v) for v in m.find("pose").text.split()]
        if (max(abs(x - y) for x, y in zip(it["xyz"], pose[:3])) > 1e-9
                or max(abs(x - y) for x, y in zip(it["rpy"], pose[3:])) > 1e-9):
            pbad.append(it["name"])
        geo = m.find(".//geometry")[0]
        if geo.tag == "box":
            s = [float(v) for v in geo.find("size").text.split()]
            if max(abs(x - y) for x, y in zip(it["size"], s)) > 1e-9:
                sbad.append(it["name"])
        elif geo.tag == "cylinder":
            if (abs(it["radius"] - float(geo.find("radius").text)) > 1e-9
                    or abs(it["height"] - float(geo.find("length").text)) > 1e-9):
                sbad.append(it["name"])
        elif geo.tag == "sphere":
            if abs(it["radius"] - float(geo.find("radius").text)) > 1e-9:
                sbad.append(it["name"])
    check("온실 자세 = SDF pose", not pbad, str(pbad[:3]))
    check("온실 치수 = SDF geometry", not sbad, str(sbad[:3]))

    # 4. 좌표 변환 규약 — 위치식과 회전식이 서로 맞는지(Unity 를 못 켜니 식을 검증한다)
    rng = np.random.default_rng(7)
    worst = 0.0
    for _ in range(2000):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)                      # (x,y,z,w)
        v = rng.normal(size=3)
        rotated_ros = quat_to_R(q) @ v              # ROS 에서 회전
        lhs = ros_to_unity_pos(rotated_ros)         # 회전 뒤 Unity 로
        rhs = quat_to_R(ros_to_unity_quat(q)) @ ros_to_unity_pos(v)   # Unity 로 옮긴 뒤 회전
        worst = max(worst, float(np.abs(lhs - rhs).max()))
    check("FLU→RUF 위치식·회전식 정합(무작위 2000회)", worst < 1e-9, f"최대 오차 {worst:.2e}")
    M = np.array([[0, -1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)
    check("좌표 변환이 손잡이 뒤집기(det = −1)", abs(np.linalg.det(M) + 1.0) < 1e-12,
          f"det = {np.linalg.det(M):+.0f}")
    cs = open(os.path.join(P, "Assets", "Editor", "GreenhouseBuilder.cs")).read()
    check("C# 변환식이 위 검증식과 같은 문자열",
          "new Vector3(-y, z, x)" in cs and "new Quaternion(-q.y, q.z, q.x, -q.w)" in cs)

    # 5. (선택) 배치모드 리포트 대조 — Unity 를 돌린 뒤에만 존재
    rep = os.path.join(P, "rda_unity_report.json")
    if not os.path.exists(rep):
        print("ℹ 배치모드 리포트 없음 — Unity 설치·라이선스 후 RdaBatch.RunAll 을 돌리면 "
              "프레임 위치까지 대조한다(현재 미검증 항목).")
    else:
        r = json.load(open(rep))
        links, joints = G.parse_urdf(urdf_path)
        _, W = G.link_world_tf(links, joints)
        movable = [n for n, j in joints.items() if j["type"] != "fixed"]
        check("Unity 관절(revolute) 수 = URDF 가동관절 수",
              r.get("revolute") == len(movable), f"{r.get('revolute')} vs {len(movable)}")
        check("Unity 온실 오브젝트 수", r.get("greenhouse") == len(items),
              f"{r.get('greenhouse')} vs {len(items)}")
        fbad = []
        for name, pos in (r.get("frames") or {}).items():
            if name not in W:
                continue
            want = ros_to_unity_pos(W[name][:3, 3])
            if np.abs(np.array(pos) - want).max() > 1e-3:
                fbad.append(f"{name}: {np.round(pos,4).tolist()} vs {np.round(want,4).tolist()}")
        check("Unity 프레임 위치 = URDF FK(좌표 변환 적용)", not fbad, "; ".join(fbad))

        # 🔴 온실 '형상' 검사 — 개수만 세면 원기둥이 눕거나 상자가 돌아가도 통과한다.
        #    실제로 그런 결함이 있었다(2026-08-16, 사용자가 화면에서 발견):
        #    좌표 변환이 ROS 로컬 +Z 를 이미 Unity +Y 로 보내는데 GreenhouseBuilder 가
        #    원기둥에 Quaternion.Euler(90,0,0) 를 더 곱해 **레일이 거터와 직각으로** 깔렸다
        #    (rail_L0 축 기대 [1,0,0] ↔ 실측 [0,0,1]). 그래서 축·치수까지 여기서 본다.
        samples = {s["name"]: s for s in (r.get("greenhouse_samples") or [])}
        if not samples:
            print("ℹ 리포트에 greenhouse_samples 가 없다 — 생성기를 갱신하고 RunAll 을 다시 돌릴 것")
        else:
            byname = {it["name"]: it for it in items}
            pbad, abad, sbad = [], [], []
            for nm, s in samples.items():
                it = byname.get(nm)
                if it is None:
                    continue
                want_p = ros_to_unity_pos(np.asarray(it["xyz"], float))
                if np.abs(np.asarray(s["pos"], float) - want_p).max() > 1e-3:
                    pbad.append(nm)
                b = np.asarray(s["bounds"], float)
                if it["type"] == "cylinder":
                    # 원기둥 축 = 오브젝트 로컬 +Y(transform.up). ROS 로컬 +Z 를 rpy 로 돌린 뒤 변환.
                    ax = _rpy_matrix(*it["rpy"]) @ np.array([0.0, 0.0, 1.0])
                    want_a = ros_to_unity_pos(ax)
                    got_a = np.asarray(s["axis"], float)
                    na, ng = np.linalg.norm(want_a), np.linalg.norm(got_a)
                    if na > 0 and ng > 0 and abs(float(np.dot(want_a / na, got_a / ng))) < 0.999:
                        abad.append(f"{nm}: 기대 {np.round(want_a,3).tolist()} vs {np.round(got_a,3).tolist()}")
                    # world AABB 의 최대변 = 원기둥 길이(축이 좌표축에 정렬된 경우)
                    if abs(float(b.max()) - float(it["height"])) > 1e-2:
                        sbad.append(f"{nm}: 길이 {b.max():.3f} vs {it['height']:.3f}")
                elif it["type"] == "box":
                    want_b = np.abs(ros_to_unity_pos(np.asarray(it["size"], float)))
                    if np.abs(np.sort(b) - np.sort(want_b)).max() > 1e-2:
                        sbad.append(f"{nm}: {np.round(b,3).tolist()} vs {np.round(want_b,3).tolist()}")
            check("Unity 온실 위치 = obstacles.yaml", not pbad, f"{len(pbad)}개 불일치: {pbad[:3]}")
            check("Unity 온실 원기둥 축 = obstacles.yaml", not abad,
                  f"{len(abad)}개 불일치: {abad[:2]}")
            check("Unity 온실 치수 = obstacles.yaml", not sbad, f"{len(sbad)}개 불일치: {sbad[:3]}")

    print(f"\n통과 {len(_ok)} / 실패 {len(_fail)}  (총 {len(_ok) + len(_fail)})")
    if _fail:
        print("실패:", ", ".join(_fail))
    sys.exit(1 if _fail else 0)


if __name__ == "__main__":
    main()
