#!/usr/bin/env python3
"""정본 장면(`obstacles.yaml`)에서 **파생 장면**을 만든다.

왜 스크립트인가: 파생 장면을 손으로 복사해 두면 정본이 바뀔 때 조용히 낡는다.
정본을 한 곳에 두고 **차이만 코드로** 적어 두면, 정본이 바뀌었을 때 다시 돌리기만 하면 된다.

만드는 것
  · obstacles_real_truss.yaml   — 현실 화방 높이(first_z 0.35). 스탠드 구성용.
  · obstacles_detail.yaml       — 상세 작물(잎·소과경·화방대 곡선) + 기본 높이
  · obstacles_detail_real.yaml  — 상세 작물 + 현실 화방 높이

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
import sys

import yaml

CFG = os.path.expanduser("~/robot_ws/src/rda_robot_description/config")
SRC = os.path.join(CFG, "obstacles.yaml")

BANNER = (
    "# ⚠ 생성물 — 손으로 고치지 말 것.\n"
    "#   정본 = obstacles.yaml · 생성기 = docs/scripts/gen_scene_variants.py\n"
    "#   정본을 고쳤으면 생성기를 다시 돌릴 것.\n"
)

#: 상세 작물 형상 — 값은 **통상값(추정)**, 사진은 구조만 정했다.
DETAIL_TRUSS = {
    "rachis_on_top": True,       # 🔴 화방대 축을 **열매 윗면(+z 표면)** 높이에 둔다
                                 #    (사용자 지시 2026-08-21). 종전에는 축이 클러스터
                                 #    **중심**으로 가 열매 속을 관통했다.
    "rachis_droop": 0.0,         # 🔴 처짐 없음(수평).
    #   왜 0 인가 — 축을 열매 윗면에 얹으면 **처짐이 원리적으로 성립하지 않는다.**
    #   축의 양 끝(줄기 부착점·클러스터 끝)이 이미 최상단 열매 윗면 높이라, 3cm 만 처져도
    #   축 중간이 그 열매 속으로 1.3cm 파고든다(2026-08-21 실측: 축 z 1.0067 ↔ 윗면 1.0200).
    #   사진의 '아래로 휜 화방대' 를 담으려면 축을 윗면보다 더 위로 띄우고 소과경을 길게
    #   줘야 하는데, 그건 "윗면에 위치" 라는 지시와 다르다 → 지시를 따른다.
    "rachis_segments": 1,        # 직선이므로 마디 1개. 이름도 종전 `rachis_<키>` 유지
                                 #   (여러 마디면 `_s0/_s1…` 로 갈려 이름 계약이 흔들린다)
    "pedicel_radius": 0.002,     # 소과경 굵기 — 사진상 화방대(4mm)보다 확실히 가늘다
    "pedicel_min_len": 0.008,    # 이보다 짧으면 자루 없이 축에 붙은 것으로 본다
}
VARIANTS = {
    "obstacles_real_truss.yaml": dict(first_z=0.35, detail=False),
    "obstacles_detail.yaml": dict(first_z=None, detail=True),
    "obstacles_detail_real.yaml": dict(first_z=0.35, detail=True),
}


def build(src, first_z=None, detail=False):
    d = yaml.safe_load(open(src)) or {}
    tr = d["crops"]["template"]["truss"]
    if first_z is not None:
        tr["first_z"] = float(first_z)
    if detail:
        tr.update(DETAIL_TRUSS)
    return d


def render(d):
    return BANNER + yaml.safe_dump(d, allow_unicode=True, sort_keys=False, width=100)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="쓰지 않고 최신인지만 확인(다르면 exit 1)")
    a = ap.parse_args()
    stale = []
    for name, kw in VARIANTS.items():
        path = os.path.join(CFG, name)
        text = render(build(SRC, **kw))
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
