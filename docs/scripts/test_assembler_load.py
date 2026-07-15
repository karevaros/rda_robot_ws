#!/usr/bin/env python3
"""조립기 '불러오기'가 모델 선택까지 복원하는지 검증 (헤드리스).

버그: _load() 가 mounts/initial_pose 만 읽고 models 를 안 읽어서
      위치만 복원되고 모델은 그대로였다.
※ 실제 mounts.yaml 은 건드리지 않는다 — _mounts_path 를 temp 로 monkeypatch.
"""
import os
import sys
import tempfile

import yaml
from PyQt5 import QtWidgets

from rda_robot_assembler import app as A
from rda_robot_assembler import part_registry as reg

TMP = tempfile.mkdtemp(prefix="rda_load_test_")
MP = os.path.join(TMP, "mounts.yaml")
A.Assembler._mounts_path = lambda self: MP   # 실파일 보호

qapp = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
w = A.Assembler()

fails = []


def chk(cond, msg):
    print(("✅ " if cond else "❌ ") + msg)
    if not cond:
        fails.append(msg)


# 기본 모델 확인
print(f"기본 모델: {w.models}")
base_default = w.models["base"]
arm_default = w.models["arm"]

# 다른 모델로 바꿀 후보 찾기
arm_alt = next((m for m in reg.SLOT_MODELS["arm"] if m != arm_default), None)
base_alt = next((m for m in reg.SLOT_MODELS["base"] if m != base_default), None)
chk(arm_alt is not None and base_alt is not None, f"교체 후보 존재 (arm={arm_alt}, base={base_alt})")

# ── 1) 모델을 바꿔서 저장 ────────────────────────────────────────────────
for slot, mid in (("arm", arm_alt), ("base", base_alt)):
    combo = w.slot_widgets[slot]["combo"]
    combo.setCurrentIndex(combo.findData(mid))   # 시그널 → _on_model_changed
w.mounts["arm"].xyz = [0.11, 0.22, 0.33]
w._save()

saved = yaml.safe_load(open(MP))
chk(saved.get("models", {}).get("arm") == arm_alt,
    f"저장 파일의 models.arm == {arm_alt}")
chk(saved["mounts"]["arm"]["xyz"] == [0.11, 0.22, 0.33], "저장 파일의 위치 반영")

# ── 2) 모델을 되돌려 놓고(=파일과 다른 상태) 불러오기 ────────────────────
for slot, mid in (("arm", arm_default), ("base", base_default)):
    combo = w.slot_widgets[slot]["combo"]
    combo.setCurrentIndex(combo.findData(mid))
w.mounts["arm"].xyz = [0.0, 0.0, 0.0]
chk(w.models["arm"] == arm_default, "불러오기 직전: 모델이 파일과 다른 상태로 돌려놓음")

w._load()

# ── 3) 핵심 검증: 모델 + 위치 둘 다 복원됐는가 ──────────────────────────
chk(w.models["arm"] == arm_alt, f"[핵심] 불러오기 후 models['arm'] == {arm_alt} (모델 복원)")
chk(w.models["base"] == base_alt, f"[핵심] 불러오기 후 models['base'] == {base_alt} (모델 복원)")
chk(w.slot_widgets["arm"]["combo"].currentData() == arm_alt,
    "[핵심] 콤보 표시값도 복원 (화면 == 내부상태)")
chk([round(v, 3) for v in w.mounts["arm"].xyz] == [0.11, 0.22, 0.33], "위치도 복원(기존 동작 유지)")
chk(w.loaded.get("arm") is not None and w.loaded["arm"].model_id == arm_alt,
    f"[핵심] 파트가 실제로 재로드됨 (loaded['arm'].model_id == {arm_alt})")
chk(w._dirty is False, "불러오기 직후 dirty=False")

# ── 4) 등록 안 된 모델은 조용히 넘기지 않고 경고 ────────────────────────
d = yaml.safe_load(open(MP))
d["models"]["arm"] = "존재하지_않는_모델_xyz"
yaml.safe_dump(d, open(MP, "w"), allow_unicode=True)
w._load()
txt = w.lbl_status.text()
chk("존재하지_않는_모델_xyz" in txt and "건너뜀" in txt,
    f"[핵심] 미등록 모델을 경고로 알림 (조용한 무시 아님): {txt[:60]}...")
chk("c04a5e" in w.lbl_status.styleSheet(), "경고가 빨강으로 표시")

# 실파일 무변경 확인
real = os.path.expanduser("~/robot_ws/src/rda_robot_description/config/mounts.yaml")
chk(w._mounts_path() != real, f"실제 mounts.yaml 무변경(temp 사용: {MP})")

print(f"\n{'전체 통과 ✅' if not fails else f'실패 {len(fails)}건 ❌'}")
sys.exit(1 if fails else 0)
