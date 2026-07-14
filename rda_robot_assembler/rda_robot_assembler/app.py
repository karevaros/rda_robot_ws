#!/usr/bin/env python3
"""rda_robot 파트 조립기 (PyQt5 + pyvista 내장 3D).

좌: 파트 슬롯 선택(모델/표시)  |  중앙: 3D 뷰  |  우: 결합 설정(부모프레임/거리/각도)
결과: mounts.yaml 저장 → rda_robot.urdf.xacro 가 읽어 통합.
"""
import os
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


def default_mounts():
    """초안 기본 결합값(비교표 초안 기준)."""
    return {
        "arm": Mount("base", "base_link", [0.0, 0.0, 0.25], [0, 0, 0]),
        "endeffector": Mount("arm", "tcp", [0.0, 0.0, 0.0], [0, 0, 0]),
        "sensor1": Mount("endeffector", "rg2_hand", [0.02, 0.0, 0.03], [0, 0, 0]),
        "sensor2": Mount("base", "base_link", [-0.25, 0.0, 0.6], [0, 0.3, 0]),
    }


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
        form.addRow("부모 파트", self.parent_slot)
        form.addRow("부모 TF(프레임)", self.parent_frame)

        self.sp = {}
        for key, lo, hi, step, suffix in [
            ("x", -3, 3, 0.005, " m"), ("y", -3, 3, 0.005, " m"), ("z", -3, 3, 0.005, " m"),
            ("roll", -180, 180, 1.0, " °"), ("pitch", -180, 180, 1.0, " °"), ("yaw", -180, 180, 1.0, " °"),
        ]:
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi); s.setSingleStep(step); s.setDecimals(3 if suffix == " m" else 1)
            s.setSuffix(suffix)
            self.sp[key] = s
            form.addRow(key.upper() if len(key) == 1 else key.capitalize(), s)

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
        self.parent_frame.blockSignals(True)
        self.parent_frame.clear()
        ps = self.parent_slot.currentData()
        frames = self._frames_provider(ps) if (self._frames_provider and ps) else []
        for fr in frames:
            self.parent_frame.addItem(fr, fr)
        if select is not None:
            i = self.parent_frame.findData(select)
            if i >= 0:
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
        self.title = QtWidgets.QLabel("관절 초기 포즈")
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
        self.setWindowTitle("rda_robot 파트 조립기")
        self.resize(1500, 900)

        self.models = {s: reg.default_model(s) for s in reg.SLOTS}
        self.enabled = {s: True for s in reg.SLOTS}
        self.loaded = {}          # slot -> LoadedPart
        self.actors = {}          # slot -> list[actor]
        self.mounts = default_mounts()
        self.joint_pose = {}      # slot -> {joint: rad}
        self.active_slot = "arm"

        self._build_ui()
        self._reload_all_parts()
        self.editor.set_providers(self._slot_choices, self._frames_of)
        self._select_slot("arm")
        self._refresh_view(full=True)

    # ---------- UI ----------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        h = QtWidgets.QHBoxLayout(central)

        # 좌: 슬롯 패널
        left = QtWidgets.QWidget(); left.setFixedWidth(300)
        lv = QtWidgets.QVBoxLayout(left)
        lv.addWidget(QtWidgets.QLabel("<b>파트 슬롯</b>"))
        self.slot_widgets = {}
        for slot in reg.SLOTS:
            box = QtWidgets.QGroupBox(reg.SLOT_LABELS[slot])
            box.setCheckable(False)
            v = QtWidgets.QVBoxLayout(box)
            combo = QtWidgets.QComboBox()
            for mid in reg.SLOT_MODELS[slot]:
                combo.addItem(reg.MODELS[mid]["label"], mid)
            combo.currentIndexChanged.connect(lambda _i, s=slot: self._on_model_changed(s))
            v.addWidget(combo)
            row = QtWidgets.QHBoxLayout()
            chk = QtWidgets.QCheckBox("표시")
            chk.setChecked(True)
            chk.stateChanged.connect(lambda _s, sl=slot: self._on_enable_changed(sl))
            edit_btn = QtWidgets.QPushButton("결합 설정 ▸")
            edit_btn.clicked.connect(lambda _c, sl=slot: self._select_slot(sl))
            if slot == "base":
                edit_btn.setEnabled(False)
                edit_btn.setText("(루트)")
            row.addWidget(chk); row.addWidget(edit_btn)
            v.addLayout(row)
            lv.addWidget(box)
            self.slot_widgets[slot] = {"combo": combo, "chk": chk, "box": box, "btn": edit_btn}
        lv.addStretch(1)
        h.addWidget(left)

        # 중앙: 3D 뷰
        mid = QtWidgets.QWidget()
        mv = QtWidgets.QVBoxLayout(mid)
        self.plotter = QtInteractor(mid)
        mv.addWidget(self.plotter.interactor)
        # 뷰 툴바
        tb = QtWidgets.QHBoxLayout()
        self.chk_axes = QtWidgets.QCheckBox("부착 프레임 축 표시"); self.chk_axes.setChecked(True)
        self.chk_axes.stateChanged.connect(lambda _s: self._refresh_view())
        btn_reset_view = QtWidgets.QPushButton("뷰 리셋")
        btn_reset_view.clicked.connect(self._fit_view)
        tb.addWidget(self.chk_axes); tb.addStretch(1); tb.addWidget(btn_reset_view)
        mv.addLayout(tb)
        h.addWidget(mid, 1)

        # 우: 설정 + 파일 버튼
        right = QtWidgets.QWidget(); right.setFixedWidth(320)
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
        self.lbl_status = QtWidgets.QLabel("")
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setStyleSheet("color:#c04a5e;")
        rv.addWidget(self.lbl_status)
        for text, fn in [("mounts.yaml 저장", self._save), ("불러오기", self._load), ("초안값 초기화", self._reset)]:
            b = QtWidgets.QPushButton(text); b.clicked.connect(fn); rv.addWidget(b)
        h.addWidget(right)

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
            self._set_status(f"[{slot}] 로드 실패: {e}")
        if refresh:
            self._refresh_view(full=True)

    def _set_status(self, msg):
        self.lbl_status.setText(msg)

    # ---------- 이벤트 ----------
    def _on_model_changed(self, slot):
        self.models[slot] = self.slot_widgets[slot]["combo"].currentData()
        self._reload_part(slot)
        if self.active_slot == slot:
            self._select_slot(slot)

    def _on_enable_changed(self, slot):
        self.enabled[slot] = self.slot_widgets[slot]["chk"].isChecked()
        self._refresh_view(full=True)

    def _select_slot(self, slot):
        if slot == "base":
            return
        self.active_slot = slot
        self.editor.load(slot, self.mounts.get(slot, Mount()))
        self.jpose.load(slot, self.loaded.get(slot))
        for s, w in self.slot_widgets.items():
            w["box"].setStyleSheet("QGroupBox{font-weight:bold;border:2px solid #e08a3c;margin-top:6px;}"
                                   if s == slot else "")

    def _on_mount_changed(self):
        if self.active_slot:
            self.mounts[self.active_slot] = self.editor.current_mount()
            self._refresh_view()

    def _on_jpose_changed(self):
        slot = self.active_slot
        if slot and slot in self.loaded:
            self.joint_pose[slot] = self.jpose.current_pose()
            self._apply_joint_pose(slot)

    def _apply_joint_pose(self, slot):
        """관절 포즈를 파트에 반영 → 해당 파트 액터 재생성 → 배치 갱신."""
        part = self.loaded.get(slot)
        if part is None:
            return
        part.set_joint_pose(self.joint_pose.get(slot, {}))
        for a in self.actors.get(slot, []):
            try:
                self.plotter.remove_actor(a, render=False)
            except Exception:
                pass
        self._add_part_actors(slot)
        self._update_transforms()
        self.plotter.render()

    # ---------- 렌더링 ----------
    def _refresh_view(self, full=False):
        if full:
            self.plotter.clear()
            self.actors = {}
            self.plotter.add_axes()
            # 외곽 경계 박스 + 눈금 1m 간격(GRID_SPAN 기준)
            try:
                self.plotter.show_grid(
                    bounds=GRID_BOUNDS, color="gray",
                    n_xlabels=_NLAB, n_ylabels=_NLAB, n_zlabels=_NLAB,
                    xtitle="X (m)", ytitle="Y (m)", ztitle="Z (m)",
                )
            except Exception:
                pass
            # 바닥 1m 셀 격자면(GRID_SPAN x GRID_SPAN) — 크기 감각용 레퍼런스
            try:
                ncell = max(1, int(round(GRID_SPAN)))
                floor = pv.Plane(center=(0, 0, 0), direction=(0, 0, 1),
                                 i_size=GRID_SPAN, j_size=GRID_SPAN,
                                 i_resolution=ncell, j_resolution=ncell)
                self.plotter.add_mesh(floor, style="wireframe", color="dimgray",
                                      line_width=1, name="_floor_grid", pickable=False)
            except Exception:
                pass
            for slot in reg.SLOTS:
                if not self.enabled.get(slot) or slot not in self.loaded:
                    continue
                self._add_part_actors(slot)
        self._update_transforms()
        if full:
            self._fit_view()
        self.plotter.render()

    def _fit_view(self):
        """카메라를 고정 4m 영역에 맞춤(자동 scene fit 대신)."""
        try:
            self.plotter.reset_camera(bounds=list(GRID_BOUNDS))
        except TypeError:
            # 구버전 호환: 경계 상자 기준 수동 설정
            self.plotter.reset_camera()
        try:
            self.plotter.view_isometric()
            self.plotter.reset_camera(bounds=list(GRID_BOUNDS))
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
        if self.chk_axes.isChecked() and self.active_slot in self.mounts:
            mnt = self.mounts[self.active_slot]
            ps = mnt.parent_slot
            if ps in world:
                Fw = world[ps] @ self.loaded[ps].frames.get(mnt.parent_frame, np.eye(4))
                self._draw_axis(Fw, "_attach_axis")
        if missing:
            self._set_status("미배치(부모 확인 필요): " + ", ".join(missing))
        elif self.lbl_status.text().startswith("미배치"):
            self._set_status("")

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
        try:
            from ament_index_python.packages import get_package_share_directory
            # 소스 경로에 저장(재빌드 없이 xacro 가 읽도록 share 도 갱신)
        except Exception:
            pass
        # 소스 위치 우선
        src = os.path.expanduser("~/robot_ws/src/rda_robot_description/config/mounts.yaml")
        return src

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
        path = self._mounts_path()
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            yaml.safe_dump({"mounts": data, "models": self.models,
                            "initial_pose": ipose}, f,
                           allow_unicode=True, sort_keys=False)
        QtWidgets.QMessageBox.information(self, "저장", f"저장됨:\n{path}")

    def _load(self):
        path = self._mounts_path()
        if not os.path.exists(path):
            QtWidgets.QMessageBox.warning(self, "불러오기", f"파일 없음:\n{path}")
            return
        with open(path) as f:
            d = yaml.safe_load(f) or {}
        mp = d.get("mounts", {})
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
        self._select_slot(self.active_slot)
        self._refresh_view(full=True)

    def _reset(self):
        self.mounts = default_mounts()
        self._select_slot(self.active_slot)
        self._refresh_view()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = Assembler()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
