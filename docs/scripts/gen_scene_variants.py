#!/usr/bin/env python3
"""정본 장면(`obstacles.yaml`)에서 **파생 장면**을 만든다.

왜 스크립트인가: 파생 장면을 손으로 복사해 두면 정본이 바뀔 때 조용히 낡는다.
정본을 한 곳에 두고 **차이만 코드로** 적어 두면, 정본이 바뀌었을 때 다시 돌리기만 하면 된다.

만드는 것
  · obstacles_real_truss.yaml   — 현실 화방 높이(first_z 0.35). 스탠드 구성용.
  · obstacles_detail.yaml       — 상세 화방(화방대 위치 교정 + 소과경) + 기본 높이
  · obstacles_detail_real.yaml  — 상세 화방 + 현실 화방 높이

🔴 **정본은 읽기만 한다.** 파생 파일에는 "생성물" 표시를 붙인다.

상세 형상의 근거(2026-08-21, `tomatopick/` 사진 5장 대조):
  · 화방대가 주 줄기에서 나와 **아래로 처지며** 열매를 단다        (9aJK·e77cc)
  · 각 열매는 **소과경**으로 화방대 축에 붙는다(= 사진의 '갈래')   (sub3-3·9aJK)
⚠ **잎은 넣지 않는다**(2026-08-21 사용자 지시). 사진에는 잎이 무성하지만 모델에는 담지
   않기로 했다 — 이 결정은 회피 비교 해석에 영향이 있다(`구성프리셋-선정규칙.md` 4장:
   통로 쪽 캐노피 깊이가 없어 정면 직선 진입이 막히지 않는다).
⚠ 사진에는 스케일 기준이 없다 → **치수는 통상값(추정)** 이고 사진에서 잰 값이 아니다.
   사진이 정한 것은 '무엇이 어디에 붙어 있나'(구조)뿐이다.

사용: python3 docs/scripts/gen_scene_variants.py [--check]
      --check 면 파일을 쓰지 않고 **최신인지만** 확인한다(CI/회귀용).
"""
import argparse
import os
import re
import sys


CFG = os.path.expanduser("~/robot_ws/src/rda_robot_description/config")
SRC = os.path.join(CFG, "obstacles.yaml")

BANNER = (
    "# ⚠ 생성물 — 손으로 고치지 말 것.\n"
    "#   정본 = obstacles.yaml · 생성기 = docs/scripts/gen_scene_variants.py\n"
    "#   정본을 고쳤으면 생성기를 다시 돌릴 것.\n"
)

#: 상세 화방 형상 — 값은 **통상값(추정)**, 사진은 구조만 정했다.
#  키는 정본 obstacles.yaml 에 **중립 기본값 + @tune 마커**로 이미 들어 있다.
#  여기서는 그 줄의 **숫자만** 바꾼다(주석·마커 보존).
DETAIL = {
    # 화방대 축을 **열매 윗면(+z 표면)** 높이로. 0=클러스터 중심(종전, 열매 관통).
    "rachis_z_frac": 1.0,
    # 소과경(화방대 → 각 열매 윗면). 사진상 화방대(4mm)보다 확실히 가늘다.
    "pedicel_radius": 0.002,
    # 처짐·마디는 켜지 않는다 — 축을 윗면에 얹으면 처지는 순간 열매를 파고든다
    #   (실측: droop 3cm 에서 1.3cm 침투). 마디 1개면 이름도 종전 `rachis_<키>` 유지.
    "rachis_droop": 0.0,
    "rachis_segments": 1,
}

VARIANTS = {
    "obstacles_real_truss.yaml": {"first_z": 0.35},
    "obstacles_detail.yaml": dict(DETAIL),
    "obstacles_detail_real.yaml": dict(DETAIL, first_z=0.35),
}

# `key: value` 줄에서 앞부분과 숫자를 분리 (crop_tuner 와 같은 규칙 — 주석은 건드리지 않는다)
VAL_RE = re.compile(r"^(\s*([A-Za-z_][\w]*):\s*)(-?[\d.]+)")


def build(src, overrides):
    """정본을 **줄 단위로** 읽어 지정한 키의 숫자만 바꾼다.

    🔴 yaml.safe_dump 로 다시 쓰면 **주석과 `@tune` 마커가 전부 날아간다** —
    그러면 파생 장면은 튜너(crop_tuner)로 조절할 수 없다. 줄 단위 치환이면
    정본의 주석·근거·마커가 파생에도 그대로 남는다."""
    out, seen = [], set()
    for ln in open(src):
        m = VAL_RE.match(ln)
        if m and m.group(2) in overrides:
            key = m.group(2)
            v = overrides[key]
            # 원본 줄의 표기(정수/소수)를 따라간다 — 파일 스타일을 흔들지 않는다
            txt = (f"{float(v):g}" if "." in m.group(3)
                   else (str(int(round(v))) if float(v).is_integer() else f"{float(v):g}"))
            if "." in m.group(3) and "." not in txt:
                txt += ".0"
            ln = ln[:m.start(3)] + txt + ln[m.end(3):]
            seen.add(key)
        out.append(ln)
    missing = set(overrides) - seen
    if missing:
        raise SystemExit(
            f"error: 정본에 없는 키 {sorted(missing)} — obstacles.yaml 에 "
            f"중립 기본값으로 먼저 추가할 것(그래야 튜너에도 뜬다)")
    return BANNER + "".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="쓰지 않고 최신인지만 확인(다르면 exit 1)")
    a = ap.parse_args()
    stale = []
    for name, kw in VARIANTS.items():
        path = os.path.join(CFG, name)
        text = build(SRC, kw)
        cur = open(path).read() if os.path.exists(path) else None
        if a.check:
            if cur != text:
                stale.append(name)
            continue
        with open(path, "w") as f:
            f.write(text)
        print(f"{'갱신' if cur != text else '동일'}  {path}")
    if a.check:
        if stale:
            print("낡음 ❌ " + ", ".join(stale)
                  + "\n  → python3 docs/scripts/gen_scene_variants.py 로 다시 생성")
            return 1
        print(f"최신 ✅ ({len(VARIANTS)}개)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
