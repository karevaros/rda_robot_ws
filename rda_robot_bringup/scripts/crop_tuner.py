#!/usr/bin/env python3
"""작물 파라미터 튜너 GUI (PyQt5) — Joint State Publisher 처럼 슬라이더/수치로 조절.

동작: obstacles.yaml 에서 `@tune=<id>|<min>|<max>` 마커가 붙은 줄을 찾아 각각
슬라이더+스핀박스를 만든다. 값을 바꾸면 **그 줄의 숫자만 교체**해 저장하고,
obstacle_publisher 의 라이브 리로드가 ~1초 내 RViz 에 자동 반영한다.
yaml 의 주석·구조는 그대로 보존된다(줄 단위 교체 + 원자적 저장).

실행:  ros2 run rda_robot_bringup crop_tuner.py [obstacles.yaml경로]
       (경로 생략 시 소스 트리의 config/obstacles.yaml — launch 가 읽는 그 파일)
"""
import os
import re
import sys

from PyQt5 import QtWidgets, QtCore

# 한 줄에서  key: value  와  @tune=<id>|<min>|<max>|<기본값>(선택)  를 각각 뽑는다.
TUNE_RE = re.compile(r"@tune=([A-Za-z0-9_]+)\|(-?[\d.]+)\|(-?[\d.]+)(?:\|(-?[\d.]+))?")
VAL_RE = re.compile(r"^(\s*[A-Za-z_][\w]*:\s*)(-?[\d.]+)")

LABELS = {
    "stem_radius": "줄기 반경 (m)",
    "stem_height": "줄기 높이 (m)",
    "stem_lean_y": "줄기 곡선 편차 (m)",
    "stem_curve_k": "줄기 곡률",
    "stem_segments": "줄기 마디 수",
    "truss_first_z": "첫 화방 높이 (m)",
    "truss_spacing": "화방 수직간격 (m)",
    "truss_count": "주당 화방 수",
    "fruits_per_truss": "화방당 열매 수",
    "fruit_radius": "열매 반경 (m)",
    "rachis_out": "클러스터 거리 (m)",
    "cluster_gap": "뭉침 정도 (작을수록 촘촘)",
    "plant_spacing": "주간 간격 (m)",
}
SLIDER_STEPS = 1000     # 실수 슬라이더 해상도


def default_path():
    return os.path.expanduser(
        "~/robot_ws/src/rda_robot_description/config/obstacles.yaml")


def parse_tunables(path):
    """(id, min, max, value, is_int, default) 목록을 파일 등장 순서대로 반환."""
    out = []
    with open(path) as f:
        for ln in f:
            mt = TUNE_RE.search(ln)
            mv = VAL_RE.match(ln)
            if not (mt and mv):
                continue
            tid, smin, smax, sdef = mt.group(1), mt.group(2), mt.group(3), mt.group(4)
            sval = mv.group(2)
            is_int = "." not in smin and "." not in smax
            default = float(sdef) if sdef is not None else float(sval)
            out.append((tid, float(smin), float(smax), float(sval), is_int, default))
    return out


def write_values(path, updates):
    """updates={id: (value, is_int)} 의 값으로 해당 @tune 줄의 숫자만 교체 후 원자적 저장."""
    with open(path) as f:
        lines = f.readlines()
    new = []
    for ln in lines:
        mt = TUNE_RE.search(ln)
        if mt and mt.group(1) in updates:
            value, is_int = updates[mt.group(1)]
            txt = str(int(round(value))) if is_int else f"{value:g}"
            ln = VAL_RE.sub(lambda m: m.group(1) + txt, ln, count=1)
        new.append(ln)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.writelines(new)
    os.replace(tmp, path)      # 원자적 — 리로더가 반쪽 파일을 읽지 않게


class TunerRow(QtCore.QObject):
    """슬라이더 ↔ 스핀박스 동기화 한 줄."""
    changed = QtCore.pyqtSignal()

    def __init__(self, tid, vmin, vmax, value, is_int):
        super().__init__()
        self._ready = False      # 구성 중 setValue 가 changed 를 흘리지 않게
        self.id, self.min, self.max, self.is_int = tid, vmin, vmax, is_int
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        if is_int:
            self.spin = QtWidgets.QSpinBox()
            self.spin.setRange(int(vmin), int(vmax))
            self.slider.setRange(int(vmin), int(vmax))
            self.spin.setValue(int(round(value)))
            self.slider.setValue(int(round(value)))
        else:
            self.spin = QtWidgets.QDoubleSpinBox()
            self.spin.setRange(vmin, vmax)
            self.spin.setDecimals(3)
            self.spin.setSingleStep(max(0.001, (vmax - vmin) / 100.0))
            self.spin.setValue(value)
            self.slider.setRange(0, SLIDER_STEPS)
            self.slider.setValue(self._to_slider(value))
        self.slider.valueChanged.connect(self._from_slider)
        self.spin.valueChanged.connect(self._from_spin)
        self._ready = True

    def _to_slider(self, v):
        if self.is_int:
            return int(round(v))
        return int(round((v - self.min) / (self.max - self.min) * SLIDER_STEPS))

    def _from_slider(self, s):
        if not self._ready:
            return
        v = s if self.is_int else self.min + s / SLIDER_STEPS * (self.max - self.min)
        self.spin.blockSignals(True)
        self.spin.setValue(int(v) if self.is_int else v)
        self.spin.blockSignals(False)
        self.changed.emit()

    def _from_spin(self, v):
        if not self._ready:
            return
        self.slider.blockSignals(True)
        self.slider.setValue(self._to_slider(v))
        self.slider.blockSignals(False)
        self.changed.emit()

    def value(self):
        return self.spin.value()


class CropTuner(QtWidgets.QWidget):
    def __init__(self, path):
        super().__init__()
        self.path = path
        self.setWindowTitle("작물 파라미터 튜너")
        self.resize(430, 640)
        self.rows = {}
        self._pending = {}
        self._ready = False      # build 중/직후 스퓨리어스 변경으로 파일 쓰지 않게
        # 슬라이더 드래그 중 파일을 매 픽셀 쓰지 않도록 짧게 모았다가 저장(디바운스).
        self._timer = QtCore.QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(120)
        self._timer.timeout.connect(self._flush)

        outer = QtWidgets.QVBoxLayout(self)
        info = QtWidgets.QLabel(
            "슬라이더/숫자를 바꾸면 obstacles.yaml 에 저장되고 RViz 가 ~1초 내 반영합니다.")
        info.setWordWrap(True)
        info.setStyleSheet("color:#555;")
        outer.addWidget(info)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        host = QtWidgets.QWidget()
        self.form = QtWidgets.QFormLayout(host)
        scroll.setWidget(host)
        outer.addWidget(scroll, 1)

        btns = QtWidgets.QHBoxLayout()
        b_reset = QtWidgets.QPushButton("기본값으로 초기화")
        b_reset.setToolTip("모든 값을 마커의 기본값으로 되돌립니다.")
        b_reset.clicked.connect(self.reset_defaults)
        btns.addWidget(b_reset)
        b_reload = QtWidgets.QPushButton("파일에서 다시 읽기")
        b_reload.clicked.connect(self.reload)
        btns.addWidget(b_reload)
        btns.addStretch(1)
        outer.addLayout(btns)

        self.status = QtWidgets.QLabel("")
        self.status.setStyleSheet("color:#2e7d32;")
        outer.addWidget(self.status)

        self.build()

    def build(self):
        # 기존 폼 비우기
        while self.form.rowCount():
            self.form.removeRow(0)
        self.rows.clear()
        try:
            tunables = parse_tunables(self.path)
        except OSError as e:
            self.status.setText(f"파일을 읽지 못함: {e}")
            self.status.setStyleSheet("color:#c62828;")
            return
        if not tunables:
            self.form.addRow(QtWidgets.QLabel(
                "@tune 마커가 없습니다. obstacles.yaml 을 확인하세요."))
            return
        self.defaults = {}
        for tid, vmin, vmax, value, is_int, default in tunables:
            self.defaults[tid] = (default, is_int)
            row = TunerRow(tid, vmin, vmax, value, is_int)
            row.changed.connect(lambda t=tid: self._on_change(t))
            self.rows[tid] = row
            cell = QtWidgets.QWidget()
            h = QtWidgets.QHBoxLayout(cell)
            h.setContentsMargins(0, 0, 0, 0)
            h.addWidget(row.slider, 1)
            h.addWidget(row.spin)
            self.form.addRow(LABELS.get(tid, tid), cell)
        # 구성이 끝난 다음 틱부터 사용자 변경을 받는다(초기화 신호 무시).
        self._ready = False
        QtCore.QTimer.singleShot(250, lambda: setattr(self, "_ready", True))

    def reload(self):
        self.build()
        self.status.setText("파일에서 현재 값을 다시 읽었습니다.")

    def reset_defaults(self):
        if not getattr(self, "defaults", None):
            return
        try:
            write_values(self.path, dict(self.defaults))
        except OSError as e:
            self.status.setText(f"초기화 실패: {e}")
            self.status.setStyleSheet("color:#c62828;")
            return
        self.build()          # 슬라이더를 기본값으로 다시 읽는다
        self.status.setText("기본값으로 초기화했습니다.")
        self.status.setStyleSheet("color:#2e7d32;")

    def _on_change(self, tid):
        if not self._ready:
            return
        row = self.rows[tid]
        self._pending[tid] = (row.value(), row.is_int)
        self._timer.start()

    def _flush(self):
        if not self._pending:
            return
        try:
            write_values(self.path, dict(self._pending))
            names = ", ".join(self._pending)
            self.status.setText(f"저장: {names}")
            self.status.setStyleSheet("color:#2e7d32;")
        except OSError as e:
            self.status.setText(f"저장 실패: {e}")
            self.status.setStyleSheet("color:#c62828;")
        self._pending.clear()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else default_path()
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        print(f"[crop_tuner] 파일 없음: {path}", file=sys.stderr)
        sys.exit(1)
    app = QtWidgets.QApplication(sys.argv)
    w = CropTuner(path)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
