#!/usr/bin/env python3
"""통합 URDF 회귀 테스트 — 모델 조합별로 컴포저가 단일 트리를 만드는지 검증.

■ 왜 필요한가 (이 테스트가 없어서 생긴 일)
`test_models.py` 는 **조립기 로드**만 검증한다. 그걸 28/28 통과했다고 "모델 확장 완료"로
보고했으나, 통합 URDF 는 슬롯별 `xacro:if` 분기가 있는 7종만 반영하고 나머지 21종은
조용히 슬롯을 통째로 누락시켰다 — 그래서 display/moveit launch 가 둘 다 기동 불가였다.
**검증 범위(조립기 로드)를 실제 요구사항(통합 URDF 반영)으로 착각한 것**이다.
이 테스트는 그 간극을 메운다.

■ 검사 항목
  1. 기본 조합이 조립되고 링크명 계약(base_link·link0·tcp·rg2_hand)이 보존되는가
  2. 등록된 모든 모델이 자기 슬롯에서 조립되는가 — **최소 구성으로 격리 검사**
  3. 결과가 URDF 규격상 단일 트리인가 (`check_urdf`)

■ 왜 '최소 구성'인가
기본 조합에서 한 슬롯만 바꾸면 자식 슬롯이 연쇄로 깨진다. mounts.yaml 의 parent_frame 은
모델마다 다른 이름(tcp·rg2_hand…)이라, 팔을 바꾸면 그리퍼 부착점도 다시 골라야 하기
때문이다 — 이건 컴포저 결함이 아니라 성립하지 않는 조합이다. 그래서 검사 대상 모델마다
**부모가 내장 모델(prefix 없음)인 최소 구성**만 세워 그 모델 자체의 조립 가능성만 본다.

사용:  python3 src/docs/scripts/test_integrated_urdf.py [--slot arm] [-v]
"""
import argparse
import copy
import os
import shutil
import subprocess
import sys
import tempfile

import yaml

from rda_robot_assembler import part_registry as reg
from rda_robot_assembler import composer

# 링크명 계약 — gen_srdf.py·kinematics.py·moveit_demo.launch.py·self_collision_monitor.py 가
# 이 이름들에 의존한다. 기본 조합에서 반드시 살아 있어야 한다.
CONTRACT_LINKS = ["base_link", "link0", "link6", "tcp", "rg2_hand"]

DEFAULT_COMBO = {
    "base": "scout_v2",
    "arm": "rb5_850e",
    "endeffector": "onrobot_rg2",
    "sensor1": "d405",
    "sensor2": "d435i",
}


def base_mounts():
    """정본 mounts.yaml 을 읽어 기본 조합으로 정규화한 dict 반환."""
    path = os.path.join(os.path.dirname(reg.models_dir()), "mounts.yaml")
    with open(path) as f:
        y = yaml.safe_load(f) or {}
    y.setdefault("models", {})
    y["models"].update(DEFAULT_COMBO)
    for slot, mid in DEFAULT_COMBO.items():
        if slot in (y.get("mounts") or {}):
            y["mounts"][slot]["model"] = mid
    return y


def _mount(parent_slot, parent_frame):
    return {"parent_slot": parent_slot, "parent_frame": parent_frame,
            "xyz": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}


def minimal_combo(slot, mid):
    """검사 대상 모델 1종만 얹은 최소 구성.

    부모는 항상 prefix 없는 내장 모델이라 parent_frame 이름이 흔들리지 않는다.
    """
    y = {"models": {}, "mounts": {}}
    if slot == "base":
        y["models"]["base"] = mid                       # 베이스 단독
    elif slot == "arm":
        y["models"].update(base="scout_v2", arm=mid)
        y["mounts"]["arm"] = _mount("base", "base_link")
    elif slot == "endeffector":
        y["models"].update(base="scout_v2", arm="rb5_850e", endeffector=mid)
        y["mounts"]["arm"] = _mount("base", "base_link")
        y["mounts"]["endeffector"] = _mount("arm", "tcp")
    else:                                                # sensor1 / sensor2
        y["models"].update(base="scout_v2")
        y["models"][slot] = mid
        y["mounts"][slot] = _mount("base", "base_link")
    return y


def compose_combo(y, tmpdir, tag):
    """조합 dict → (성공?, 메시지, urdf 경로)"""
    p = os.path.join(tmpdir, f"{tag}.yaml")
    with open(p, "w") as f:
        yaml.safe_dump(y, f)
    try:
        xml = composer.compose(p)
    except composer.ComposeError as e:
        return False, str(e).split("\n")[0], None
    except Exception as e:
        return False, f"{type(e).__name__}: {e}", None
    out = os.path.join(tmpdir, f"{tag}.urdf")
    with open(out, "w") as f:
        f.write(xml)
    return True, "", out


def check_urdf(path):
    exe = shutil.which("check_urdf")
    if not exe:
        return True, "(check_urdf 없음 — 건너뜀)"
    r = subprocess.run([exe, path], capture_output=True, text=True)
    if r.returncode != 0:
        line = (r.stderr or r.stdout).strip().split("\n")[0]
        return False, line[:120]
    return True, ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slot", help="이 슬롯만 검사")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args()

    models, slot_add = reg.load_external_models()
    tmp = tempfile.mkdtemp(prefix="rda_urdf_test_")
    ok_n = fail_n = 0
    failures = []

    # 1) 기본 조합 + 링크명 계약
    y = base_mounts()
    good, msg, path = compose_combo(y, tmp, "default")
    if good:
        good, msg = check_urdf(path)
    if good:
        with open(path) as f:
            xml = f.read()
        missing = [l for l in CONTRACT_LINKS if f'<link name="{l}"' not in xml]
        if missing:
            good, msg = False, f"링크명 계약 위반 — 누락: {', '.join(missing)}"
    print(f"{'✅' if good else '❌'} 기본조합       scout_v2+rb5_850e+onrobot_rg2+d405+d435i"
          f"{'' if good else '  → ' + msg}")
    if good:
        ok_n += 1
    else:
        fail_n += 1
        failures.append(("기본조합", msg))

    # 2) 슬롯별 모델 전수 — 최소 구성으로 격리 검사
    for slot in reg.SLOTS:
        if a.slot and slot != a.slot:
            continue
        cands = list(reg.BUILTIN_SLOT_MODELS.get(slot, [])) + list(slot_add.get(slot, []))
        for mid in cands:
            y = minimal_combo(slot, mid)
            tag = f"{slot}__{mid}".replace("/", "_")
            good, msg, path = compose_combo(y, tmp, tag)
            if good:
                good, msg = check_urdf(path)
            if good:
                ok_n += 1
                if a.verbose:
                    print(f"✅ {slot:12s} {mid}")
            else:
                fail_n += 1
                failures.append((f"{slot}/{mid}", msg))
                print(f"❌ {slot:12s} {mid}\n      → {msg}")

    shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n조립 성공 {ok_n} / 실패 {fail_n}  (총 {ok_n + fail_n})")
    if failures:
        print("\n실패 목록:")
        for name, m in failures:
            print(f"  · {name}: {m}")
    return 1 if fail_n else 0


if __name__ == "__main__":
    sys.exit(main())
