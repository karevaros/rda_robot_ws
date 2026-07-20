#!/usr/bin/env python3
"""RDA 로봇 어셈블러 (PyQt5 + pyvista 내장 3D).

좌: 파트 슬롯 선택(모델/표시)  |  중앙: 3D 뷰  |  우: 결합 설정(부착 프레임/거리/각도)
결과: mounts.yaml 저장 → rda_robot.urdf.xacro 가 읽어 통합.
"""
import os
import subprocess
import sys
import math
import numpy as np
import yaml

from PyQt5 import QtWidgets, QtCore, QtGui
from pyvistaqt import QtInteractor
import pyvista as pv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rda_robot_assembler import part_registry as reg
from rda_robot_assembler import urdf_loader as ul
from rda_robot_assembler.assembly import Mount, compute_placements
from rda_robot_assembler import collision as col

# 앱 표시 이름(사람이 읽는 이름). ROS 패키지명·실행 명령어는 rda_robot_assembler 유지.
APP_NAME = "RDA 로봇 어셈블러"

# 슬롯별 표시 색(파트 구분용)
SLOT_COLORS = {
    "base": "#8899aa",
    "arm": "#e08a3c",
    "endeffector": "#4c9f70",
    "sensor1": "#c04a5e",
    "sensor2": "#8a5fb0",
}

DEG = 180.0 / math.pi

# 3D 뷰 외곽 경계 박스/바닥 격자 크기. GRID_SPAN(m) = 축당 전체 폭. 눈금 1m 간격.
GRID_SPAN = 2.0  # 외곽 격자를 키우려면 이 값만 조정(예: 4.0)
_H = GRID_SPAN / 2.0
# [xmin,xmax,ymin,ymax,zmin,zmax] — XY 중앙정렬, Z 는 지면(0)부터
GRID_BOUNDS = (-_H, _H, -_H, _H, 0.0, GRID_SPAN)
# 1m 간격 눈금 개수(span/1 + 1)
_NLAB = int(round(GRID_SPAN)) + 1

# 격자: 주격자 1m + 보조격자 100mm(치수 감각용)
GRID_MAJOR = 1.0    # m
GRID_MINOR = 0.1    # m (=100mm)
# 평면이 3장이라 선이 진하면 로봇을 가린다 → 밝은 회색으로 물러나게.
GRID_MAJOR_COLOR = "#8a8a8a"
GRID_MINOR_COLOR = "#d6d6d6"
# 격자를 그릴 평면 3종 — 조작 중인 파트의 베이스를 지나가게 배치한다.
#   XY(바닥) / XZ / YZ  ← direction = 각 평면의 법선
GRID_PLANES = (("xy", (0, 0, 1)), ("xz", (0, 1, 0)), ("yz", (1, 0, 0)))

# 자충돌 시 표시: 충돌 파트는 빨강. 단 '지금 조작 중인 파트'가 상대 파트에 파묻혀
# 안 보이는 걸 막기 위해, 충돌 상대 파트만 반투명으로 낮춘다.
COLLISION_COLOR = "#ff2020"
COLLISION_OPACITY = 0.30   # 충돌 중인 '비활성' 파트 불투명도


def default_mounts():
    """초안 기본 결합값(비교표 초안 기준)."""
    return {
        "arm": Mount("base", "base_link", [0.0, 0.0, 0.25], [0, 0, 0]),
        "endeffector": Mount("arm", "tcp", [0.0, 0.0, 0.0], [0, 0, 0]),
        "sensor1": Mount("endeffector", "rg2_hand", [0.02, 0.0, 0.03], [0, 0, 0]),
        "sensor2": Mount("base", "base_link", [-0.25, 0.0, 0.6], [0, 0.3, 0]),
    }


class SlotBox(QtWidgets.QGroupBox):
    """클릭하면 해당 슬롯이 선택되는 그룹박스."""
    clicked = QtCore.pyqtSignal()

    def mousePressEvent(self, ev):
        self.clicked.emit()
        super().mousePressEvent(ev)


class MountEditor(QtWidgets.QWidget):
    """우측 결합 설정 위젯(활성 슬롯 1개)."""
    changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._block = False
        form = QtWidgets.QFormLayout(self)
        self.title = QtWidgets.QLabel("—")
        f = self.title.font(); f.setBold(True); f.setPointSize(f.pointSize() + 1)
        self.title.setFont(f)
        form.addRow(self.title)

        self.parent_slot = QtWidgets.QComboBox()
        self.parent_frame = QtWidgets.QComboBox()
        self.parent_slot.setToolTip("이 파트를 어느 파트에 붙일지 선택")
        self.parent_frame.setToolTip("부모 파트의 어느 링크(프레임)에 붙일지 선택")
        form.addRow("붙일 파트", self.parent_slot)
        form.addRow("부착 프레임", self.parent_frame)

        self.sp = {}
        for key, label, lo, hi, step, suffix in [
            ("x", "X (앞/뒤)", -3, 3, 0.005, " m"),
            ("y", "Y (좌/우)", -3, 3, 0.005, " m"),
            ("z", "Z (위/아래)", -3, 3, 0.005, " m"),
            ("roll", "Roll (X축 회전)", -180, 180, 1.0, " °"),
            ("pitch", "Pitch (Y축 회전)", -180, 180, 1.0, " °"),
            ("yaw", "Yaw (Z축 회전)", -180, 180, 1.0, " °"),
        ]:
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi); s.setSingleStep(step); s.setDecimals(3 if suffix == " m" else 1)
            s.setSuffix(suffix)
            self.sp[key] = s
            form.addRow(label, s)

        self.parent_slot.currentIndexChanged.connect(self._on_parent_slot)
        for w in [self.parent_frame]:
            w.currentIndexChanged.connect(self._emit)
        for s in self.sp.values():
            s.valueChanged.connect(self._emit)

        self._slots_provider = None  # () -> list of (slot, label)
        self._frames_provider = None  # (slot) -> list of frame names
        self.active = None

    def set_providers(self, slots_provider, frames_provider):
        self._slots_provider = slots_provider
        self._frames_provider = frames_provider

    def load(self, slot, mount):
        """활성 슬롯 mount 값을 위젯에 반영."""
        self._block = True
        self.active = slot
        self.title.setText(f"결합 설정 · {reg.SLOT_LABELS.get(slot, slot)}")
        # 부모 파트 후보
        self.parent_slot.clear()
        cands = self._slots_provider() if self._slots_provider else []
        for s, label in cands:
            if s == slot:
                continue
            self.parent_slot.addItem(label, s)
        # 현재 부모 선택
        idx = max(0, self.parent_slot.findData(mount.parent_slot))
        self.parent_slot.setCurrentIndex(idx)
        self._reload_frames(select=mount.parent_frame)
        # xyz/rpy
        for k, v in zip(["x", "y", "z"], mount.xyz):
            self.sp[k].setValue(v)
        for k, v in zip(["roll", "pitch", "yaw"], mount.rpy):
            self.sp[k].setValue(v * DEG)
        self._block = False

    def _reload_frames(self, select=None):
        """부모 파트의 링크 목록으로 콤보 재구성.

        select 가 그 목록에 없으면(= 부모 모델을 바꿔서 예전 프레임명이 남은 경우)
        '(없음)' 항목으로 **그대로 보여준다**. 예전엔 findData 가 -1 이라 선택이
        안 되고 콤보만 0번 항목을 보여줘서, 화면은 멀쩡한데 실제 mount 값은
        옛 프레임인 채로 남는 거짓 표시가 됐다.
        """
        self.parent_frame.blockSignals(True)
        self.parent_frame.clear()
        ps = self.parent_slot.currentData()
        frames = self._frames_provider(ps) if (self._frames_provider and ps) else []
        for fr in frames:
            self.parent_frame.addItem(fr, fr)
        if select is not None:
            i = self.parent_frame.findData(select)
            if i < 0:
                self.parent_frame.insertItem(0, f"{select}  ⚠ (부모에 없음)", select)
                i = 0
            self.parent_frame.setCurrentIndex(i)
        self.parent_frame.blockSignals(False)

    def _on_parent_slot(self):
        if self._block:
            return
        self._reload_frames()
        self._emit()

    def _emit(self):
        if not self._block:
            self.changed.emit()

    def current_mount(self):
        return Mount(
            self.parent_slot.currentData(),
            self.parent_frame.currentData(),
            [self.sp["x"].value(), self.sp["y"].value(), self.sp["z"].value()],
            [self.sp["roll"].value() / DEG, self.sp["pitch"].value() / DEG, self.sp["yaw"].value() / DEG],
        )


class JointPoseEditor(QtWidgets.QWidget):
    """활성 슬롯의 가동관절 초기 포즈(슬라이더+°). 관절 없으면 숨김."""
    changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._block = False
        self.v = QtWidgets.QVBoxLayout(self)
        self.v.setContentsMargins(0, 0, 0, 0)
        self.title = QtWidgets.QLabel("관절 초기 포즈 (시작 자세)")
        f = self.title.font(); f.setBold(True)
        self.title.setFont(f)
        self.v.addWidget(self.title)
        self.form_host = QtWidgets.QWidget()
        self.form = QtWidgets.QFormLayout(self.form_host)
        self.form.setContentsMargins(0, 0, 0, 0)
        self.v.addWidget(self.form_host)
        self.rows = {}   # joint -> (slider, spin)
        self.active = None

    def load(self, slot, part):
        """part.actuated / joint_limits / joint_pose 로 행 재구성."""
        self._block = True
        self.active = slot
        # 기존 행 제거
        while self.form.rowCount():
            self.form.removeRow(0)
        self.rows = {}
        acts = getattr(part, "actuated", []) if part else []
        self.setVisible(bool(acts))
        for jn in acts:
            lo, hi = part.joint_limits.get(jn, (-math.pi, math.pi))
            val = part.joint_pose.get(jn, 0.0)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(int(math.degrees(lo)), int(math.degrees(hi)))
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(math.degrees(lo), math.degrees(hi))
            spin.setDecimals(1); spin.setSingleStep(1.0); spin.setSuffix(" °")
            spin.setValue(math.degrees(val)); slider.setValue(int(math.degrees(val)))
            slider.valueChanged.connect(lambda d, s=spin: (s.blockSignals(True), s.setValue(d), s.blockSignals(False), self._emit()))
            spin.valueChanged.connect(lambda d, sl=slider: (sl.blockSignals(True), sl.setValue(int(d)), sl.blockSignals(False), self._emit()))
            row = QtWidgets.QWidget(); hl = QtWidgets.QHBoxLayout(row)
            hl.setContentsMargins(0, 0, 0, 0)
            hl.addWidget(slider, 1); hl.addWidget(spin)
            self.form.addRow(jn, row)
            self.rows[jn] = (slider, spin)
        self._block = False

    def _emit(self):
        if not self._block:
            self.changed.emit()

    def current_pose(self):
        """joint -> rad"""
        return {jn: math.radians(spin.value()) for jn, (sl, spin) in self.rows.items()}


class Assembler(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.resize(1500, 900)

        self.models = {s: reg.default_model(s) for s in reg.SLOTS}
        self.enabled = {s: True for s in reg.SLOTS}
        self.loaded = {}          # slot -> LoadedPart
        self.actors = {}          # slot -> list[actor]
        self.mounts = default_mounts()
        self.joint_pose = {}      # slot -> {joint: rad}
        self.active_slot = "arm"
        self.collider = col.CollisionChecker()   # 자충돌 검사기
        self._dirty = False       # 저장 후 변경 여부(제목 * 표시)
        self._grid_origin = None  # 현재 격자 기준점(파트 베이스) — None 이면 재생성
        self._bounds = GRID_BOUNDS   # 현재 격자 경계(카메라 맞춤용)
        self.env_spec = None      # 현재 배경 정의(dict) — None 이면 배경 없음
        self.env_actors = []      # 배경 액터들(비선택·표시전용)
        self.base_offset = np.eye(4)   # 배경 기준 로봇 베이스 배치(world→base)
        self.base_place = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}  # UI 값

        self._build_ui()
        self._reload_all_parts()
        self.editor.set_providers(self._slot_choices, self._frames_of)
        self._select_slot("arm")
        self._refresh_view(full=True)
        # 초기(기본) 배치의 겹침을 기준으로 보정 → 이후 이탈만 경고
        self._calibrate_baseline(initial=True)
        self._set_dirty(False)
        self._set_status("파트 슬롯을 클릭해 결합 설정을 편집하세요.")
        # 시작 시 배경 지정(선택): RDA_ASSEMBLER_ENV=<라벨|파일명 일부>. 없으면 '없음'.
        self._preselect_environment(os.environ.get("RDA_ASSEMBLER_ENV"))
        # 시작 시 배경 기준 로봇 배치(선택): RDA_ASSEMBLER_BASE="x,y,z,yaw°".
        self._preset_base_place(os.environ.get("RDA_ASSEMBLER_BASE"))

    def _preset_base_place(self, spec):
        if not spec:
            return
        try:
            vals = [float(v) for v in spec.replace(" ", "").split(",")]
        except ValueError:
            self._set_status(f"RDA_ASSEMBLER_BASE 형식 오류: {spec!r} (기대: x,y,z,yaw)")
            return
        for key, v in zip(("x", "y", "z", "yaw"), vals):
            self.base_place[key] = v
            sp = self.base_spins.get(key)
            if sp is not None:
                sp.blockSignals(True); sp.setValue(v); sp.blockSignals(False)
        self.base_offset = self._base_matrix()
        self._update_transforms()
        self.plotter.render()

    def _preselect_environment(self, want):
        if not want:
            return
        want = want.lower()
        for i in range(self.env_combo.count()):
            data = self.env_combo.itemData(i) or ""
            text = self.env_combo.itemText(i)
            if want in text.lower() or want in os.path.basename(str(data)).lower():
                self.env_combo.setCurrentIndex(i)
                return
        self._set_status(f"배경 '{want}' 을(를) 찾지 못했습니다.")

    # ---------- UI ----------
    def _build_ui(self):
        self._build_actions()
        self._build_menu()
        self._build_toolbar()

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        splitter.addWidget(self._build_left())
        splitter.addWidget(self._build_center())
        splitter.addWidget(self._build_right())
        splitter.setStretchFactor(0, 0)   # 좌: 고정 성향
        splitter.setStretchFactor(1, 1)   # 중앙 3D 뷰가 여유공간 차지
        splitter.setStretchFactor(2, 0)   # 우: 고정 성향
        splitter.setSizes([300, 880, 320])
        self.splitter = splitter
        self.setCentralWidget(splitter)

        self._build_statusbar()
        self._update_title()

    def _build_actions(self):
        """메뉴/툴바가 공유하는 액션(체크 상태의 단일 소스)."""
        def act(text, slot=None, shortcut=None, tip=None, checkable=False, checked=False):
            a = QtWidgets.QAction(text, self)
            if shortcut:
                a.setShortcut(shortcut)
            if tip:
                a.setToolTip(tip); a.setStatusTip(tip)
            if checkable:
                a.setCheckable(True); a.setChecked(checked)
            if slot:
                a.triggered.connect(slot)
            return a

        # 파일
        self.act_save = act("저장", self._save, QtGui.QKeySequence.Save,
                            "현재 결합값·초기 포즈를 mounts.yaml 로 저장")
        self.act_load = act("불러오기", self._load, QtGui.QKeySequence.Open,
                            "mounts.yaml 에서 결합값·초기 포즈를 다시 읽음")
        self.act_reset = act("기본값으로 되돌리기", self._reset, None,
                             "결합값을 초안 기본값으로 되돌림(저장 전까지 파일은 그대로)")
        self.act_quit = act("종료", self.close, QtGui.QKeySequence.Quit)

        # 보기
        self.act_axes = act("부착 프레임 축", lambda: self._refresh_view(), "A",
                            "선택한 파트가 붙는 부모 프레임의 XYZ 축을 3D에 표시",
                            checkable=True, checked=True)
        self.act_iso = act("등각 보기", lambda: self._set_view("iso"), "1")
        self.act_front = act("앞에서 보기", lambda: self._set_view("front"), "2")
        self.act_side = act("옆에서 보기", lambda: self._set_view("side"), "3")
        self.act_top = act("위에서 보기", lambda: self._set_view("top"), "4")
        self.act_fit = act("뷰 맞춤", self._fit_view, "0", "카메라를 기본 위치로 되돌림")

        # 도구
        self.act_collision = act("자충돌 검사", self._on_collision_toggle, "C",
                                 "파트끼리 겹치면 3D에서 빨강으로 표시",
                                 checkable=True, checked=True)
        self.act_calib = act("현재 겹침 무시", lambda: self._calibrate_baseline(), None,
                             "지금 겹쳐 있는 쌍을 정상으로 등록 → 이후 새로 생긴 겹침만 경고")
        self.act_calib_clear = act("무시 목록 비우기", self._clear_baseline, None,
                                   "등록해 둔 겹침 무시를 모두 해제")
        self.act_refresh = act("모델 새로고침", self._refresh_models, "F5",
                               "config/models/<슬롯>/ 폴더를 다시 스캔해 드롭다운 갱신")
        self.act_open_models = act("모델 폴더 열기", self._open_models_dir, "Ctrl+M",
                                   f"파일 관리자로 모델 폴더 열기:\n{reg.models_dir()}")

    def _build_menu(self):
        mb = self.menuBar()
        m = mb.addMenu("파일(&F)")
        m.addAction(self.act_save); m.addAction(self.act_load)
        m.addSeparator(); m.addAction(self.act_reset)
        m.addSeparator(); m.addAction(self.act_quit)

        m = mb.addMenu("보기(&V)")
        m.addAction(self.act_axes)
        m.addSeparator()
        m.addAction(self.act_iso); m.addAction(self.act_front)
        m.addAction(self.act_side); m.addAction(self.act_top)
        m.addSeparator(); m.addAction(self.act_fit)

        m = mb.addMenu("도구(&T)")
        m.addAction(self.act_collision)
        m.addAction(self.act_calib); m.addAction(self.act_calib_clear)
        m.addSeparator()
        m.addAction(self.act_refresh); m.addAction(self.act_open_models)

    def _build_toolbar(self):
        tb = QtWidgets.QToolBar("기본 도구모음")
        tb.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        tb.setMovable(False)
        tb.addAction(self.act_save)
        tb.addSeparator()
        tb.addAction(self.act_collision)
        tb.addAction(self.act_calib)
        tb.addAction(self.act_calib_clear)
        tb.addSeparator()
        tb.addAction(self.act_axes)
        tb.addSeparator()
        tb.addWidget(QtWidgets.QLabel(" 보기: "))
        tb.addAction(self.act_iso); tb.addAction(self.act_front)
        tb.addAction(self.act_side); tb.addAction(self.act_top)
        tb.addAction(self.act_fit)
        tb.addSeparator()
        # 배경(환경) 선택 — config/environments/*.yaml 을 스캔해 채운다.
        tb.addWidget(QtWidgets.QLabel(" 배경: "))
        self.env_combo = QtWidgets.QComboBox()
        self.env_combo.setToolTip(
            "3D 뷰에 표시할 배경(온실 등). config/environments/ 에 yaml 을 넣으면 자동 등록됩니다.")
        self._populate_environments()
        self.env_combo.currentIndexChanged.connect(self._on_env_changed)
        tb.addWidget(self.env_combo)
        self.addToolBar(QtCore.Qt.TopToolBarArea, tb)

    def _build_left(self):
        left = QtWidgets.QWidget()
        left.setMinimumWidth(240)
        lv = QtWidgets.QVBoxLayout(left)
        lv.addWidget(QtWidgets.QLabel("<b>파트 슬롯</b> <span style='color:gray;'>— 클릭해 선택</span>"))
        self.slot_widgets = {}
        for slot in reg.SLOTS:
            box = SlotBox(reg.SLOT_LABELS[slot])
            box.clicked.connect(lambda s=slot: self._select_slot(s))
            v = QtWidgets.QVBoxLayout(box)
            combo = QtWidgets.QComboBox()
            for mid in reg.SLOT_MODELS[slot]:
                combo.addItem(reg.MODELS[mid]["label"], mid)
            combo.currentIndexChanged.connect(lambda _i, s=slot: self._on_model_changed(s))
            v.addWidget(combo)
            row = QtWidgets.QHBoxLayout()
            chk = QtWidgets.QCheckBox("3D에 표시")
            chk.setChecked(True)
            chk.stateChanged.connect(lambda _s, sl=slot: self._on_enable_changed(sl))
            row.addWidget(chk)
            row.addStretch(1)
            if slot == "base":
                tag = QtWidgets.QLabel("<span style='color:gray;'>루트 · 부모 없음</span>")
                row.addWidget(tag)
            v.addLayout(row)
            lv.addWidget(box)
            self.slot_widgets[slot] = {"combo": combo, "chk": chk, "box": box}
        btn_refresh = QtWidgets.QPushButton("🔄 모델 새로고침")
        btn_refresh.setToolTip(self.act_refresh.toolTip())
        btn_refresh.clicked.connect(self._refresh_models)
        lv.addWidget(btn_refresh)
        self.btn_open_models = QtWidgets.QPushButton("📂 모델 폴더 열기")
        self.btn_open_models.setToolTip(self.act_open_models.toolTip())
        self.btn_open_models.clicked.connect(self._open_models_dir)
        lv.addWidget(self.btn_open_models)
        lv.addWidget(self._build_base_placement())
        lv.addStretch(1)
        return left

    def _build_base_placement(self):
        """배경(환경) 기준 로봇 베이스 위치/방향 컨트롤."""
        box = QtWidgets.QGroupBox("배경 기준 로봇 위치")
        box.setToolTip("배경(온실) 안에서 로봇 베이스를 옮깁니다. 배경이 없어도 로봇만 이동합니다.")
        form = QtWidgets.QFormLayout(box)
        form.setContentsMargins(8, 4, 8, 8)
        self.base_spins = {}
        specs = [("x", "X", -10.0, 10.0, 0.05, " m", 3),
                 ("y", "Y", -10.0, 10.0, 0.05, " m", 3),
                 ("z", "Z", -2.0, 3.0, 0.05, " m", 3),
                 ("yaw", "Yaw", -180.0, 180.0, 5.0, " °", 1)]
        for key, label, lo, hi, step, suffix, dec in specs:
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(lo, hi); sp.setSingleStep(step); sp.setDecimals(dec)
            sp.setSuffix(suffix); sp.setValue(self.base_place[key])
            sp.valueChanged.connect(lambda _v, k=key: self._on_base_place_changed(k))
            form.addRow(label, sp)
            self.base_spins[key] = sp
        btn = QtWidgets.QPushButton("원점으로")
        btn.setToolTip("로봇 베이스를 배경 원점(0,0,0·0°)으로 되돌립니다.")
        btn.clicked.connect(self._reset_base_place)
        form.addRow(btn)
        return box

    def _base_matrix(self):
        """base_place(x,y,z,yaw°) → 4x4 (world→base). Z 축 yaw 회전 + 평행이동."""
        p = self.base_place
        a = p["yaw"] * math.pi / 180.0
        c, s = math.cos(a), math.sin(a)
        M = np.eye(4)
        M[0, 0], M[0, 1] = c, -s
        M[1, 0], M[1, 1] = s, c
        M[0, 3], M[1, 3], M[2, 3] = p["x"], p["y"], p["z"]
        return M

    def _on_base_place_changed(self, key):
        self.base_place[key] = self.base_spins[key].value()
        self.base_offset = self._base_matrix()
        self._update_transforms()
        self.plotter.render()
        self._set_dirty()

    def _reset_base_place(self):
        for k, sp in self.base_spins.items():
            sp.blockSignals(True); sp.setValue(0.0); sp.blockSignals(False)
            self.base_place[k] = 0.0
        self.base_offset = np.eye(4)
        self._update_transforms()
        self.plotter.render()
        self._set_dirty()

    def _build_center(self):
        mid = QtWidgets.QWidget()
        mid.setMinimumWidth(400)
        mv = QtWidgets.QVBoxLayout(mid)
        mv.setContentsMargins(0, 0, 0, 0)
        self.plotter = QtInteractor(mid)
        # 반투명 파트가 겹칠 때 뒤쪽이 제대로 비쳐 보이게(순서 무관 투명 렌더)
        try:
            self.plotter.enable_depth_peeling(10)
        except Exception:
            pass
        mv.addWidget(self.plotter.interactor)
        return mid

    def _build_right(self):
        right = QtWidgets.QWidget()
        right.setMinimumWidth(260)
        rv = QtWidgets.QVBoxLayout(right)
        self.editor = MountEditor()
        self.editor.changed.connect(self._on_mount_changed)
        rv.addWidget(self.editor)
        line = QtWidgets.QFrame(); line.setFrameShape(QtWidgets.QFrame.HLine)
        rv.addWidget(line)
        self.jpose = JointPoseEditor()
        self.jpose.changed.connect(self._on_jpose_changed)
        rv.addWidget(self.jpose)
        rv.addStretch(1)
        return right

    def _build_statusbar(self):
        sb = self.statusBar()
        self.lbl_status = QtWidgets.QLabel("")
        sb.addWidget(self.lbl_status, 1)
        self.lbl_collision = QtWidgets.QLabel("자충돌: —")
        sb.addPermanentWidget(self.lbl_collision)

    def _update_title(self):
        star = "*" if self._dirty else ""
        self.setWindowTitle(f"{APP_NAME} — mounts.yaml{star}")

    def _set_dirty(self, dirty=True):
        self._dirty = dirty
        self._update_title()

    # ---------- 데이터/상태 ----------
    def _slot_choices(self):
        return [(s, reg.SLOT_LABELS[s]) for s in reg.SLOTS
                if s in self.loaded and self.enabled[s]]

    def _frames_of(self, slot):
        p = self.loaded.get(slot)
        return p.link_names if p else []

    def _reload_all_parts(self):
        for slot in reg.SLOTS:
            self._reload_part(slot, refresh=False)

    def _reload_part(self, slot, refresh=True):
        mid = self.models[slot]
        try:
            self.loaded[slot] = ul.load_part(mid, reg.MODELS[mid])
            self._set_status("")
        except Exception as e:
            self.loaded.pop(slot, None)
            self._set_status(f"[{slot}] 로드 실패: {e}", error=True)
        self._sync_collider(slot)
        if refresh:
            self._refresh_view(full=True)

    def _sync_collider(self, slot):
        """충돌객체를 loaded&enabled 상태와 일치시킴(포즈 반영 포함)."""
        part = self.loaded.get(slot)
        if part is not None and self.enabled.get(slot):
            self.collider.rebuild_part(slot, part)
        else:
            self.collider.drop_part(slot)

    def _set_status(self, msg, error=False):
        self.lbl_status.setText(msg)
        self.lbl_status.setStyleSheet("color:#c04a5e; font-weight:bold;" if error else "color:gray;")

    # ---------- 이벤트 ----------
    def _refresh_models(self):
        """모델 폴더 재스캔 → 슬롯 드롭다운 갱신(현재 선택 유지)."""
        reg.reload_models()
        for slot in reg.SLOTS:
            combo = self.slot_widgets[slot]["combo"]
            cur = combo.currentData()
            combo.blockSignals(True)
            combo.clear()
            for mid in reg.SLOT_MODELS.get(slot, []):
                combo.addItem(reg.MODELS[mid]["label"], mid)
            idx = combo.findData(cur)
            combo.setCurrentIndex(idx if idx >= 0 else 0)
            combo.blockSignals(False)
            self.models[slot] = combo.currentData()
        n = sum(len(v) for v in reg.SLOT_MODELS.values())
        self._set_status(f"모델 새로고침 완료 (총 {n}개)")

    def _open_models_dir(self):
        """모델 드롭 폴더를 파일 관리자로 연다(여기에 urdf/xacro/yaml 을 넣으면 F5 로 등록)."""
        d = reg.models_dir()
        try:
            os.makedirs(d, exist_ok=True)   # RDA_MODELS_DIR 로 옮긴 경우 없을 수 있음
        except Exception as e:
            self._set_status(f"모델 폴더를 만들 수 없습니다: {e}", error=True)
            return
        if QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(d)):
            self._set_status(f"모델 폴더 열기: {d}")
            return
        # Qt 가 실패하면(데스크톱 환경에 따라 다름) xdg-open 으로 재시도.
        # 조용히 아무 일도 안 일어나면 안 되므로 결과를 반드시 알린다.
        try:
            subprocess.Popen(["xdg-open", d], start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._set_status(f"모델 폴더 열기: {d}")
        except Exception as e:
            self._set_status(f"폴더를 열지 못했습니다({e}) — 경로: {d}", error=True)

    def _on_model_changed(self, slot):
        self.models[slot] = self.slot_widgets[slot]["combo"].currentData()
        self._reload_part(slot)
        if self.active_slot == slot:
            self._select_slot(slot)
        self._set_dirty()

    def _on_enable_changed(self, slot):
        self.enabled[slot] = self.slot_widgets[slot]["chk"].isChecked()
        self._sync_collider(slot)
        self._refresh_view(full=True)
        self._set_dirty()

    def _select_slot(self, slot):
        if slot == "base":
            self._set_status("베이스는 루트 파트라 결합 설정이 없습니다(모델·표시만 변경 가능).")
            return
        self.active_slot = slot
        self.editor.load(slot, self.mounts.get(slot, Mount()))
        self.jpose.load(slot, self.loaded.get(slot))
        for s, w in self.slot_widgets.items():
            w["box"].setStyleSheet("QGroupBox{font-weight:bold;border:2px solid #e08a3c;margin-top:6px;}"
                                   if s == slot else "")
        self._refresh_view()

    def _on_mount_changed(self):
        if self.active_slot:
            self.mounts[self.active_slot] = self.editor.current_mount()
            self._refresh_view()
            self._set_dirty()

    def _on_jpose_changed(self):
        slot = self.active_slot
        if slot and slot in self.loaded:
            self.joint_pose[slot] = self.jpose.current_pose()
            self._apply_joint_pose(slot)
            self._set_dirty()

    def _apply_joint_pose(self, slot):
        """관절 포즈를 파트에 반영 → 해당 파트 액터 재생성 → 배치 갱신."""
        part = self.loaded.get(slot)
        if part is None:
            return
        part.set_joint_pose(self.joint_pose.get(slot, {}))
        self._sync_collider(slot)   # 바뀐 포즈로 충돌객체 재빌드
        for a in self.actors.get(slot, []):
            try:
                self.plotter.remove_actor(a, render=False)
            except Exception:
                pass
        self._add_part_actors(slot)
        self._update_transforms()
        self.plotter.render()

    # ---------- 자충돌 ----------
    def _report_collision(self, active, pairs):
        if not getattr(self, "lbl_collision", None):
            return
        if not active:
            self.lbl_collision.setText("자충돌 검사: 꺼짐")
            self.lbl_collision.setStyleSheet("color:gray;")
        elif not pairs:
            self.lbl_collision.setText("자충돌: 없음 ✓")
            self.lbl_collision.setStyleSheet("color:#2e7d32;")
        else:
            def lbl(s):
                return reg.SLOT_LABELS.get(s, s)
            txt = ", ".join(f"{lbl(a)}(자체)" if a == b else f"{lbl(a)}↔{lbl(b)}"
                            for a, b in sorted(pairs))
            self.lbl_collision.setText(f"⚠ 자충돌 {len(pairs)}건: {txt}")
            self.lbl_collision.setStyleSheet("color:#c62828; font-weight:bold;")

    def _on_collision_toggle(self):
        self.collider.enabled = self.act_collision.isChecked()
        self._refresh_view()

    def _calibrate_baseline(self, initial=False):
        """현재 배치의 겹침을 기준(무시)으로 등록."""
        placed = {s: self.loaded[s] for s in self.loaded if self.enabled.get(s)}
        world = compute_placements(placed, self.mounts)
        for s in world:
            self.collider.set_world(s, world[s])
        n = self.collider.calibrate({s: self.loaded[s] for s in world}, self.mounts)
        self._refresh_view()
        if not initial:
            self._set_status(f"현재 겹침 {n}쌍을 정상으로 등록했습니다. 이후 새로 생긴 겹침만 경고합니다.")

    def _clear_baseline(self):
        self.collider.clear_calibration()
        self._refresh_view()
        self._set_status("겹침 무시 목록을 비웠습니다.")

    # ---------- 렌더링 ----------
    def _refresh_view(self, full=False):
        if full:
            self.plotter.clear()
            self.actors = {}
            self.env_actors = []       # clear() 가 배경 액터도 지웠다
            self._grid_origin = None   # clear() 가 격자도 지웠으니 재생성 강제
            self.plotter.add_axes()
            for slot in reg.SLOTS:
                if not self.enabled.get(slot) or slot not in self.loaded:
                    continue
                self._add_part_actors(slot)
            self._render_environment()   # 배경을 파트 위에 다시 얹는다
        self._update_transforms()   # 여기서 격자도 파트 베이스로 따라 옮겨짐
        if full:
            self._fit_view()
        self.plotter.render()

    # ---------- 배경(환경) ----------
    def _populate_environments(self):
        """config/environments/*.yaml 을 스캔해 배경 드롭다운을 채운다('없음' + 파일들)."""
        self.env_combo.blockSignals(True)
        self.env_combo.clear()
        self.env_combo.addItem("없음", None)
        try:
            from rda_robot_assembler import paths as _paths
            d = _paths.environments_dir()
            files = sorted(f for f in os.listdir(d)
                           if f.lower().endswith((".yaml", ".yml")))
        except Exception as e:
            files, d = [], None
            print(f"[assembler] 배경 폴더 스캔 실패: {e}", file=sys.stderr)
        for f in files:
            path = os.path.join(d, f)
            label = f
            try:
                with open(path) as fh:
                    spec = yaml.safe_load(fh) or {}
                label = spec.get("label") or os.path.splitext(f)[0]
            except Exception:
                pass
            self.env_combo.addItem(label, path)
        self.env_combo.blockSignals(False)

    def _on_env_changed(self, _idx=None):
        """드롭다운 선택 변경 → 배경 정의를 로드하고 다시 그린다."""
        path = self.env_combo.currentData()
        if path is None:
            self.env_spec = None
        else:
            try:
                with open(path) as fh:
                    self.env_spec = yaml.safe_load(fh) or {}
            except Exception as e:
                self.env_spec = None
                self._set_status(f"배경 로드 실패: {e}")
        # 배경만 다시 그린다(파트 재빌드 없이): 기존 배경 액터 제거 후 재렌더.
        self._render_environment()
        # 카메라: 배경이 있으면 로봇+인접 배경이 보이게, 없으면 격자 기준.
        try:
            self.plotter.view_isometric()
            self.plotter.reset_camera(
                bounds=self._env_fit_bounds() if self.env_spec else list(self._bounds))
        except Exception:
            pass
        self.plotter.render()
        name = self.env_combo.currentText()
        self._set_status(f"배경: {name}" if self.env_spec else "배경을 껐습니다.")

    def _clear_environment(self):
        for a in self.env_actors:
            try:
                self.plotter.remove_actor(a, render=False)
            except Exception:
                pass
        self.env_actors = []

    def _render_environment(self):
        """self.env_spec(있으면)의 primitive 들을 표시전용 메시로 그린다."""
        self._clear_environment()
        self._env_bounds = None
        if not self.env_spec:
            return
        items = self.env_spec.get("items") or self.env_spec.get("obstacles") or []
        bx = [1e9, -1e9, 1e9, -1e9, 1e9, -1e9]
        for i, it in enumerate(items):
            mesh, color, opacity = self._env_mesh(it)
            if mesh is None:
                continue
            try:
                a = self.plotter.add_mesh(
                    mesh, color=color, opacity=opacity, pickable=False,
                    name=f"_env_{i}", specular=0.2, smooth_shading=True)
                self.env_actors.append(a)
                b = mesh.bounds
                for k, op in ((0, min), (1, max), (2, min), (3, max), (4, min), (5, max)):
                    bx[k] = op(bx[k], b[k])
            except Exception as e:
                print(f"[assembler] 배경 항목 렌더 실패({it.get('name','?')}): {e}",
                      file=sys.stderr)
        if self.env_actors:
            self._env_bounds = bx

    def _env_fit_bounds(self):
        """배경이 있을 때 카메라 맞춤용 경계. 긴 배경(예: 6m 거터)은 원점 ±2.5m 로
        클램프해 로봇이 구석에 밀리지 않게 한다. 로봇 주변도 최소 포함."""
        if not getattr(self, "_env_bounds", None):
            return list(self._bounds)
        C = 2.5   # 원점 기준 최대 반경(m)
        e = self._env_bounds
        merged = [
            max(e[0], -C), min(e[1], C),
            max(e[2], -C), min(e[3], C),
            min(e[4], 0.0), min(max(e[5], 1.5), 3.0),
        ]
        # 로봇 근처(±0.6)도 항상 포함
        return [min(merged[0], -0.6), max(merged[1], 0.6),
                min(merged[2], -0.6), max(merged[3], 0.6),
                merged[4], merged[5]]

    def _env_mesh(self, it):
        """배경 항목 dict → (pv.PolyData, color[str/tuple], opacity). 실패 시 (None,..)."""
        t = it.get("type")
        pz = it.get("pose") or {}
        xyz = [float(v) for v in pz.get("xyz", [0, 0, 0])]
        rpy = [float(v) for v in pz.get("rpy", [0, 0, 0])]
        rgba = it.get("color", [0.6, 0.6, 0.6, 0.6])
        color = tuple(float(v) for v in rgba[:3])
        opacity = float(rgba[3]) if len(rgba) > 3 else 0.6
        try:
            if t == "box":
                sx, sy, sz = (float(v) for v in it["size"])
                mesh = pv.Cube(x_length=sx, y_length=sy, z_length=sz)
            elif t == "cylinder":
                mesh = pv.Cylinder(radius=float(it["radius"]),
                                   height=float(it["height"]),
                                   direction=(0, 0, 1), resolution=24)
            elif t == "sphere":
                mesh = pv.Sphere(radius=float(it["radius"]))
            else:
                return None, color, opacity
        except (KeyError, TypeError, ValueError) as e:
            print(f"[assembler] 배경 항목 형상 오류({it.get('name','?')}): {e}",
                  file=sys.stderr)
            return None, color, opacity
        # 자세 적용: 회전(rpy, ZYX) 후 평행이동. pyvista 는 도 단위이므로 환산.
        if any(rpy):
            mesh = mesh.rotate_x(rpy[0] * DEG, inplace=False)
            mesh = mesh.rotate_y(rpy[1] * DEG, inplace=False)
            mesh = mesh.rotate_z(rpy[2] * DEG, inplace=False)
        mesh = mesh.translate(xyz, inplace=False)
        return mesh, color, opacity

    # ---------- 격자 ----------
    def _grid_anchor(self, world):
        """격자 기준점 = 조작 중인 파트가 부모에 붙는 지점(= 그 파트의 베이스).

        anchor 프레임을 쓰는 이유: 파트 root 원점은 모델에 따라 실제 부착점과
        멀 수 있다(예: RG2 는 root 가 world, 부착은 rg2_hand).
        """
        part = self.loaded.get(self.active_slot)
        W = world.get(self.active_slot)
        if part is None or W is None:
            return np.zeros(3)
        A = np.asarray(W, dtype=float) @ part.frames.get(part.anchor, np.eye(4))
        return A[:3, 3]

    def _update_grids(self, world):
        """활성 파트 베이스가 움직였으면 격자를 그 위치로 다시 그린다."""
        o = self._grid_anchor(world)
        if self._grid_origin is not None and np.allclose(o, self._grid_origin, atol=1e-6):
            return
        self._grid_origin = o
        self._draw_grids(o)

    def _draw_grids(self, origin):
        """파트 베이스를 지나는 XY·XZ·YZ 3평면 격자(주 1m + 보조 100mm) + 눈금 박스."""
        ox, oy, oz = (float(v) for v in origin)
        h = GRID_SPAN / 2.0
        self._bounds = (ox - h, ox + h, oy - h, oy + h, oz - h, oz + h)
        # X/Y/Z 눈금·라벨 박스(기준점 기준으로 이동)
        try:
            self.plotter.remove_bounds_axes()
        except Exception:
            pass
        try:
            self.plotter.show_grid(
                bounds=self._bounds, color="gray",
                n_xlabels=_NLAB, n_ylabels=_NLAB, n_zlabels=_NLAB,
                xtitle="X (m)", ytitle="Y (m)", ztitle="Z (m)",
            )
        except Exception:
            pass
        for axis, normal in GRID_PLANES:
            for kind, step, color, width in (
                ("minor", GRID_MINOR, GRID_MINOR_COLOR, 1),
                ("major", GRID_MAJOR, GRID_MAJOR_COLOR, 2),
            ):
                try:
                    n = max(1, int(round(GRID_SPAN / step)))
                    plane = pv.Plane(center=(ox, oy, oz), direction=normal,
                                     i_size=GRID_SPAN, j_size=GRID_SPAN,
                                     i_resolution=n, j_resolution=n)
                    self.plotter.add_mesh(plane, style="wireframe", color=color,
                                          line_width=width, pickable=False,
                                          name=f"_grid_{axis}_{kind}")
                except Exception:
                    pass

    def _set_view(self, which):
        """뷰 프리셋 — 카메라 방향만 바꾸고 화면 범위는 현재 격자 기준 유지."""
        try:
            {"iso": self.plotter.view_isometric,
             "front": self.plotter.view_yz,   # +X 에서 바라봄
             "side": self.plotter.view_xz,    # -Y 에서 바라봄
             "top": self.plotter.view_xy}[which]()
            self.plotter.reset_camera(bounds=list(self._bounds))
        except Exception:
            pass
        self.plotter.render()

    def _fit_view(self):
        """카메라를 현재 격자 영역에 맞춤(자동 scene fit 대신)."""
        try:
            self.plotter.reset_camera(bounds=list(self._bounds))
        except TypeError:
            # 구버전 호환: 경계 상자 기준 수동 설정
            self.plotter.reset_camera()
        try:
            self.plotter.view_isometric()
            self.plotter.reset_camera(bounds=list(self._bounds))
        except Exception:
            pass

    def _add_part_actors(self, slot):
        part = self.loaded[slot]
        color = SLOT_COLORS.get(slot, "#cccccc")
        lst = []
        for geom, T in part.mesh_instances:
            try:
                # T(root→mesh)를 정점에 baking → mesh 좌표가 root 프레임 기준(작은 값).
                # (일부 vendor mesh 는 로컬 정점이 원점에서 수 m 떨어져 있어 baking 필수)
                mesh = pv.wrap(geom).transform(np.asarray(T, dtype=float), inplace=False)
            except Exception:
                continue
            actor = self.plotter.add_mesh(mesh, color=color, opacity=1.0,
                                          smooth_shading=True)
            lst.append(actor)
        self.actors[slot] = lst

    def _update_transforms(self):
        world = compute_placements(
            {s: self.loaded[s] for s in self.loaded if self.enabled.get(s)},
            self.mounts,
        )
        # 배경 기준 로봇 베이스 배치: 전 슬롯에 오프셋을 곱해 로봇 전체를 옮긴다
        # (배경은 고정, 로봇만 이동 = 온실 안 로봇 위치 변경).
        if not np.allclose(self.base_offset, np.eye(4)):
            world = {s: self.base_offset @ W for s, W in world.items()}
        # 격자를 조작 중인 파트 베이스로 이동
        self._update_grids(world)
        # 미배치 슬롯 경고
        missing = [s for s in self.loaded if self.enabled.get(s) and s not in world]
        for slot, lst in self.actors.items():
            W = world.get(slot)
            visible = W is not None
            for actor in lst:
                actor.SetVisibility(visible)
                if visible:
                    actor.user_matrix = W
        # 부착 프레임 축
        self.plotter.remove_actor("_attach_axis", render=False)
        if self.act_axes.isChecked() and self.active_slot in self.mounts:
            mnt = self.mounts[self.active_slot]
            ps = mnt.parent_slot
            if ps in world:
                Fw = world[ps] @ self.loaded[ps].frames.get(mnt.parent_frame, np.eye(4))
                self._draw_axis(Fw, "_attach_axis")
        # 부모에 존재하지 않는 부착 프레임 경고.
        # assembly.compute_placements 는 frames.get(name, eye(4)) 라 없는 프레임을
        # 조용히 원점으로 대체한다 — 모델을 바꾸면 예전 프레임명(base_link 등)이
        # 그대로 남아 파트가 소리 없이 엉뚱한 데 놓인다. 여기서 눈에 띄게 알린다.
        bad_frame = []
        for slot, mnt in self.mounts.items():
            if not self.enabled.get(slot) or slot not in self.loaded or mnt is None:
                continue
            parent = self.loaded.get(mnt.parent_slot)
            if parent is not None and mnt.parent_frame not in parent.link_names:
                bad_frame.append(f"{reg.SLOT_LABELS.get(slot, slot)}→'{mnt.parent_frame}'")

        warns = []
        if missing:
            warns.append("미배치(부모 확인 필요): " + ", ".join(missing))
        if bad_frame:
            warns.append("부착 프레임이 부모에 없음(원점으로 대체됨): " + ", ".join(bad_frame))
        if warns:
            self._set_status(" / ".join(warns), error=True)
        elif self.lbl_status.text().startswith(("미배치", "부착 프레임")):
            self._set_status("")

        # ---- 자충돌 검사 & 하이라이트(충돌 파트=빨강) ----
        colliding_slots, pairs = set(), set()
        active = getattr(self, "collider", None) is not None and self.collider.enabled
        if active:
            for slot in world:
                self.collider.set_world(slot, world[slot])
            loaded_placed = {s: self.loaded[s] for s in world}
            for pr in self.collider.check(loaded_placed, self.mounts):
                a, b = tuple(pr)
                sa, sb = a.split("::")[0], b.split("::")[0]
                colliding_slots.update((sa, sb))
                pairs.add(tuple(sorted((sa, sb))))
        self._report_collision(active, pairs)
        for slot, lst in self.actors.items():
            hit = slot in colliding_slots
            c = COLLISION_COLOR if hit else SLOT_COLORS.get(slot, "#cccccc")
            # 조작 중인 파트는 항상 불투명 — 충돌 상대만 반투명으로 비켜줘서
            # 파묻힌 활성 파트가 들여다보이게 한다.
            op = COLLISION_OPACITY if (hit and slot != self.active_slot) else 1.0
            for actor in lst:
                try:
                    actor.prop.color = c
                    actor.prop.opacity = op
                except Exception:
                    pass

    def _draw_axis(self, T, name, length=0.15):
        # 간단한 3색 축 마커
        origin = T[:3, 3]
        try:
            self.plotter.remove_actor(name, render=False)
        except Exception:
            pass
        arrows = []
        for i, col in enumerate([(1, 0, 0), (0, 1, 0), (0, 0, 1)]):
            d = T[:3, i]
            arrows.append(pv.Arrow(start=origin, direction=d, scale=length))
        merged = arrows[0].merge(arrows[1]).merge(arrows[2])
        self.plotter.add_mesh(merged, color="yellow", name=name)

    # ---------- 파일 ----------
    def _mounts_path(self):
        # 소스 위치에 저장(재빌드 없이 xacro·launch 가 읽도록)
        return os.path.expanduser("~/robot_ws/src/rda_robot_description/config/mounts.yaml")

    def _save(self):
        data = {}
        for slot, mnt in self.mounts.items():
            if not self.enabled.get(slot):
                continue
            data[slot] = {
                "model": self.models[slot],
                "parent_slot": mnt.parent_slot,
                "parent_frame": mnt.parent_frame,
                "xyz": [round(float(v), 5) for v in mnt.xyz],
                "rpy": [round(float(v), 6) for v in mnt.rpy],
            }
        # 관절 초기 포즈(flat: joint->rad) — 로드된 모든 가동관절 현재값
        ipose = {}
        for slot, part in self.loaded.items():
            if not self.enabled.get(slot):
                continue
            for jn, v in getattr(part, "joint_pose", {}).items():
                ipose[jn] = round(float(v), 6)
        # 배경 기준 로봇 베이스 배치(있을 때만 기록 — 원점이면 생략해 파일을 깔끔히).
        out = {"mounts": data, "models": self.models, "initial_pose": ipose}
        if any(abs(self.base_place[k]) > 1e-9 for k in ("x", "y", "z", "yaw")):
            out["base_placement"] = {
                "x": round(self.base_place["x"], 5),
                "y": round(self.base_place["y"], 5),
                "z": round(self.base_place["z"], 5),
                "yaw_deg": round(self.base_place["yaw"], 3),
            }
        path = self._mounts_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False)
        self._set_dirty(False)
        self._set_status(f"저장됨: {path}")
        self.statusBar().showMessage("mounts.yaml 저장 완료", 4000)

    def _load(self):
        path = self._mounts_path()
        if not os.path.exists(path):
            QtWidgets.QMessageBox.warning(self, "불러오기", f"파일 없음:\n{path}")
            return
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        mp = d.get("mounts", {})

        # ── 모델 선택 복원 ────────────────────────────────────────────────
        # 저장은 models 를 쓰는데(_save) 불러오기가 이걸 안 읽어서, 예전엔
        # 결합 위치만 돌아오고 모델은 그대로였다. 반드시 mount/포즈보다 **먼저**
        # 복원해야 한다 — 파트를 다시 로드해야 initial_pose 의 관절이 존재한다.
        sel = dict(d.get("models") or {})
        for slot, m in mp.items():          # 구버전 파일 호환(mounts.<slot>.model 만 있는 경우)
            if m.get("model"):
                sel.setdefault(slot, m["model"])
        missing = []
        for slot in reg.SLOTS:
            mid = sel.get(slot)
            if not mid:
                continue
            combo = self.slot_widgets[slot]["combo"]
            idx = combo.findData(mid)
            if idx < 0:
                # 등록되지 않은 모델(폴더에서 빠졌거나 이름이 바뀜).
                # 조용히 무시하면 화면 모델과 파일 내용이 어긋난 채로 남는다 → 경고.
                missing.append(f"{slot}={mid}")
                continue
            self.models[slot] = mid
            combo.blockSignals(True)        # 슬롯마다 _on_model_changed 가 도는 것 방지
            combo.setCurrentIndex(idx)
            combo.blockSignals(False)
        self._reload_all_parts()

        for slot, m in mp.items():
            self.mounts[slot] = Mount(m.get("parent_slot"), m.get("parent_frame"),
                                      m.get("xyz", [0, 0, 0]), m.get("rpy", [0, 0, 0]))
        # 관절 초기 포즈 배분·반영
        ip = d.get("initial_pose", {}) or {}
        for slot, part in self.loaded.items():
            pose = {jn: float(ip[jn]) for jn in getattr(part, "actuated", []) if jn in ip}
            if pose:
                self.joint_pose[slot] = pose
                part.set_joint_pose(pose)
        # 배경 기준 로봇 베이스 배치 복원(없으면 원점).
        bp = d.get("base_placement") or {}
        vals = {"x": float(bp.get("x", 0.0)), "y": float(bp.get("y", 0.0)),
                "z": float(bp.get("z", 0.0)), "yaw": float(bp.get("yaw_deg", 0.0))}
        for k, v in vals.items():
            self.base_place[k] = v
            sp = self.base_spins.get(k)
            if sp is not None:
                sp.blockSignals(True); sp.setValue(v); sp.blockSignals(False)
        self.base_offset = self._base_matrix()

        self._select_slot(self.active_slot)
        self._refresh_view(full=True)
        self._set_dirty(False)
        if missing:
            self._set_status(
                f"불러옴: {path} — ⚠ 등록되지 않은 모델은 건너뜀({', '.join(missing)}). "
                "모델 폴더를 확인하고 F5(모델 새로고침) 후 다시 불러오세요.", error=True)
        else:
            self._set_status(f"불러옴: {path}")

    def _reset(self):
        self.mounts = default_mounts()
        self._select_slot(self.active_slot)
        self._refresh_view()
        self._set_dirty()
        self._set_status("결합값을 기본값으로 되돌렸습니다(저장 전까지 파일은 그대로).")

    def closeEvent(self, ev):
        """저장하지 않은 변경이 있으면 확인."""
        if not self._dirty:
            return super().closeEvent(ev)
        r = QtWidgets.QMessageBox.question(
            self, "종료", "저장하지 않은 변경이 있습니다. 저장할까요?",
            QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard | QtWidgets.QMessageBox.Cancel,
            QtWidgets.QMessageBox.Save)
        if r == QtWidgets.QMessageBox.Save:
            self._save(); ev.accept()
        elif r == QtWidgets.QMessageBox.Discard:
            ev.accept()
        else:
            ev.ignore()


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    w = Assembler()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
