#!/usr/bin/env python3
"""모델 라이브러리 검증 — 조립기의 실제 로더로 전 모델을 로드해 본다.

폴더 드롭 등록(part_registry)과 실제 로드(urdf_loader)를 모두 통과하는지 확인.
yaml 만 있고 로드가 안 되면 조립기 드롭다운에는 뜨는데 고르면 터진다.

⚠ anchor 조용한 폴백 검사 (핵심)
  urdf_loader.LoadedPart 는 지정한 anchor 가 실제 프레임에 없으면
  **경고 없이 root 로 되돌린다**(`if self.anchor not in self.frames: self.anchor = self.root`).
  그러면 파트가 조용히 엉뚱한 지점에 붙는다. 그래서 yaml 이 선언한 anchor 와
  로드 후 part.anchor 가 같은지 대조한다 — 다르면 폴백이 작동한 것 = 실패.

실행: python3 docs/scripts/test_models.py
"""
import sys

from rda_robot_assembler import urdf_loader
from rda_robot_assembler.part_registry import SLOTS, reload_models

MODELS, SLOT_MODELS = reload_models()

ok = fail = 0
rows = []
for slot in SLOTS:
    for mid in SLOT_MODELS.get(slot, []):
        m = MODELS.get(mid)
        if m is None:
            rows.append(("❌", slot, mid, "", "", "MODELS 에 없음"))
            fail += 1
            continue
        want_anchor = m.get("anchor")
        try:
            p = urdf_loader.load_part(mid, m)
        except Exception as e:
            rows.append(("❌", slot, mid, "", "", f"{type(e).__name__}: {e}"[:66]))
            fail += 1
            continue
        nl = len(getattr(p, "link_meshes", {}) or {})
        nf = len(getattr(p, "frames", {}) or {})
        note = ""
        st = "✅"
        # 조용한 anchor 폴백 탐지
        if want_anchor and p.anchor != want_anchor:
            st, note = "❌", f"anchor '{want_anchor}' 미존재 → root('{p.anchor}') 로 조용히 폴백"
        elif not nl:
            st, note = "❌", "링크 mesh 0개(렌더 불가)"
        if st == "✅":
            ok += 1
        else:
            fail += 1
        rows.append((st, slot, mid, f"링크{nl:>3}", f"프레임{nf:>3}", note or f"anchor={p.anchor}"))

w = max((len(r[2]) for r in rows), default=10)
for st, slot, mid, a, b, note in rows:
    print(f"{st} {slot:<12} {mid:<{w}} {a:<7} {b:<8} {note}")
print(f"\n로드 성공 {ok} / 실패 {fail}  (총 {ok + fail})")
sys.exit(0 if fail == 0 else 1)
