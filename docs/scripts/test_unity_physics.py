#!/usr/bin/env python3
"""Unity 물리 거동 시험 — **중력에서 팔이 처지는가** (RDA 7주차 M4 후속, 2026-08-18 신설)

■ 왜 필요했나
  3종 중 어디에서도 **중력에서의 팔 거동을 확인한 적이 없다.**
    · Gazebo   : control 모드에서 로봇 링크 중력 OFF · collision 제거(팔 처짐·밀림 회피)
    · Unity    : RdaRosBridge 는 **미러**라 중력 OFF · 콜라이더 OFF
    · Isaac    : 프로젝트 범위 밖(2026-08-16 사용자 결정)
  ⇒ 2026-07-30 에 URDF 질량·관성에서 유도한 effort 값
     (base 64 / shoulder 314 / elbow 121 / wrist 22 Nm)이 **실제로 팔을 버티는지** 미검증이었다.
     이 스크립트가 그 한 가지에 답한다.

■ 무엇을 재나
  Unity 물리 플레이어(`Build/physics/RdaPhysicsPlayer`)를 자세·forceLimit 을 바꿔 가며 돌려
    ① 유도 effort 로 목표 자세를 유지하는가(드리프트·tcp 이동)
    ② **종전 플레이스홀더 10 Nm 로는 무너지는가**(= 이 검사가 실효성이 있는가 · 결함 주입 검사)
    ③ forceLimit 을 더 올려도 결과가 같은가(= effort 상한이 병목이 **아님**을 보이는 대조)

■ 무엇을 재지 않나 (보고서에 반드시 함께 적을 것)
  · 접촉·충돌 응답 — 로봇 콜라이더는 끈 채로 잰다(자기충돌 밀림과 중력 처짐이 섞이면 못 가른다).
  · 드라이브 게인의 타당성 — stiffness/damping 은 플레이스홀더다. 잔여 처짐은 게인 사안이다.
  · **실기 정확도** — PhysX 시뮬레이터 안의 값이지 로봇의 실제 처짐이 아니다.

선행: 물리 플레이어가 빌드돼 있을 것
  ros2 run rda_robot_bringup gen_unity_assets.py --out ~/robot_ws/export/unity
  ~/unity/Editor/Unity -batchmode -nographics -quit -projectPath <p> -executeMethod RdaBatch.BuildPhysicsScene
  ~/unity/Editor/Unity -batchmode -nographics -quit -projectPath <p> -executeMethod RdaBatch.BuildPhysicsPlayer

사용: python3 docs/scripts/test_unity_physics.py [--project <dir>] [--settle 6]
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

_PASS, _FAIL = [], []

# 자세 — ROS 관절 이름 = 라디안. 중력 모멘트가 큰 순서로.
POSES = {
    "home(전부 0)": "base=0,shoulder=0,elbow=0,wrist1=0,wrist2=0,wrist3=0",
    "수평 전개(최악)": "base=0,shoulder=-1.5708,elbow=0,wrist1=0,wrist2=0,wrist3=0",
    "수평+팔꿈치 굽힘": "base=0,shoulder=-1.5708,elbow=1.2,wrist1=0,wrist2=0,wrist3=0",
    "수확 유사": ("base=0.15,shoulder=-0.55,elbow=0.75,"
                  "wrist1=-0.30,wrist2=0.40,wrist3=-0.20"),
}

# 판정 기준 — 근거는 아래 주석. 이 값들은 '합격선'이지 정밀도 주장이 아니다.
HOLD_RAD = 0.15      # 유도 effort 로 목표에서 이 이상 밀리면 '버티지 못한다'(8.6°)
COLLAPSE_RAD = 0.30  # 플레이스홀더 10Nm 로 이 이상 밀려야 '검사가 결함을 잡는다'(17.2°)


def check(name, cond, detail=""):
    (_PASS if cond else _FAIL).append(name)
    print(f"{'✅' if cond else '❌'} {name}  {detail}")
    return cond


def run(player, pose, settle, force_limit=None, stiffness=None, damping=None):
    """물리 플레이어 1회 실행 → 리포트 dict."""
    fd, path = tempfile.mkstemp(suffix=".json", prefix="rda_phys_")
    os.close(fd)
    cmd = [player, "-batchmode", "-nographics", "-logFile", "/dev/null",
           "--settle", str(settle), "--pose", pose, "--report", path]
    if force_limit is not None:
        cmd += ["--force-limit", str(force_limit)]
    if stiffness is not None:
        cmd += ["--stiffness", str(stiffness)]
    if damping is not None:
        cmd += ["--damping", str(damping)]
    subprocess.run(cmd, timeout=300, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    with open(path) as f:
        d = json.load(f)
    os.unlink(path)
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--project", default=os.path.expanduser("~/robot_ws/export/unity"))
    ap.add_argument("--settle", type=float, default=6.0)
    a = ap.parse_args()

    player = os.path.join(a.project, "Build", "physics", "RdaPhysicsPlayer")
    if not os.path.exists(player):
        print(f"❌ 물리 플레이어가 없다: {player}\n"
              "   먼저 RdaBatch.BuildPhysicsScene → RdaBatch.BuildPhysicsPlayer")
        return 1

    # ── ① 유도 effort 로 자세를 유지하는가
    derived = {}
    for label, pose in POSES.items():
        d = run(player, pose, a.settle)
        derived[label] = d
        drift, drop = d["max_drift_rad"], d["tcp_drop_m"]
        check(f"유도 effort 로 자세 유지 — {label}", drift < HOLD_RAD,
              f"최대 드리프트 {drift:.5f} rad (기준 {HOLD_RAD}) · tcp 이동 {drop * 1000:.1f} mm")

    # 유도 effort 가 실제로 들어갔는지(= 남의 값을 쓰지 않았는지) 확인.
    j = {x["name"]: x for x in derived["수평 전개(최악)"]["joints"]}
    want = {"base": 64.0, "shoulder": 314.0, "elbow": 121.0,
            "wrist1": 22.0, "wrist2": 22.0, "wrist3": 22.0}
    bad = [n for n, v in want.items() if abs(j[n]["force_limit_after_set"] - v) > 1e-6]
    check("forceLimit = URDF effort(유도값)", not bad,
          f"불일치 {bad}" if bad else "base 64 · shoulder 314 · elbow 121 · wrist 22 Nm")

    # ⚠ Unity 임포터가 넣어 준 값은 신뢰하지 않는다 — 관측만 남긴다.
    imported = sorted({x["force_limit_imported"] for x in derived["수평 전개(최악)"]["joints"]})
    print(f"   ℹ Unity URDF Importer 가 넣은 forceLimit(관측): {imported} "
          "— 이 값은 실행마다 달랐다(2026-08-18). 그래서 URDF 에서 직접 넣는다.")

    # ── ② 플레이스홀더 10 Nm 로는 무너지는가 (= 이 검사가 결함을 잡는가)
    worst = POSES["수평 전개(최악)"]
    ph = run(player, worst, a.settle, force_limit=10)
    check("검사 실효성 — 플레이스홀더 10 Nm 면 무너진다", ph["max_drift_rad"] > COLLAPSE_RAD,
          f"최대 드리프트 {ph['max_drift_rad']:.5f} rad (기준 >{COLLAPSE_RAD}) · "
          f"tcp 이동 {ph['tcp_drop_m'] * 1000:.0f} mm")

    # ── ③ effort 상한이 병목이 아님을 보이는 대조
    hi = run(player, worst, a.settle, force_limit=100000)
    base = derived["수평 전개(최악)"]
    same = abs(hi["max_drift_rad"] - base["max_drift_rad"]) < 1e-4
    check("effort 상한은 병목이 아니다(forceLimit 10^5 로 올려도 동일)", same,
          f"유도값 {base['max_drift_rad']:.5f} rad ↔ 10^5 {hi['max_drift_rad']:.5f} rad")

    # ── ④ 남은 처짐은 드라이브 게인 쪽임을 보인다(원인 귀속)
    stiff = run(player, worst, a.settle, stiffness=10000000, damping=100000)
    check("잔여 처짐은 게인 사안(stiffness 10^7 이면 줄어든다)",
          stiff["tcp_drop_m"] < base["tcp_drop_m"] * 0.5,
          f"tcp 이동 {base['tcp_drop_m'] * 1000:.1f} mm → {stiff['tcp_drop_m'] * 1000:.1f} mm")

    print("\n[측정 요약 — 근거대장에 옮길 값]")
    print(f"  물리 조건: 중력 ON · 로봇 콜라이더 OFF · fixed dt {base['fixed_dt']} s · "
          f"stiffness {base['stiffness']:.0f} / damping {base['damping']:.0f} · settle {a.settle} s")
    for label, d in derived.items():
        print(f"  {label:18s} 최대 드리프트 {d['max_drift_rad']:.5f} rad · "
              f"tcp 이동 {d['tcp_drop_m'] * 1000:6.1f} mm")
    print(f"  대조(10 Nm)        최대 드리프트 {ph['max_drift_rad']:.5f} rad · "
          f"tcp 이동 {ph['tcp_drop_m'] * 1000:6.1f} mm")

    print(f"\n통과 {len(_PASS)} / 실패 {len(_FAIL)}  (총 {len(_PASS) + len(_FAIL)})")
    if _FAIL:
        print("실패:", ", ".join(_FAIL))
    return 1 if _FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
