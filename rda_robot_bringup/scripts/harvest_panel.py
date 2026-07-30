#!/usr/bin/env python3
"""수확 실행 제어·모니터링 패널 (PyQt5) — 6주차 'GUI 실행 패널'.

    ros2 run rda_robot_bringup harvest_panel.py

`harvest_operator.py`(터미널 조작기)의 GUI 판이면서, 조작기에 없던 **모니터링**을 더한다.
조작기는 타깃 목록과 명령 전송만 했다 — 지금 무엇이 연결돼 있는지, 안전 상태가 어떤지,
팔이 어디 있는지는 볼 수 없었다.

■ 왜 어셈블러 GUI 안에 넣지 않았나
  `rda_robot_assembler` 는 **rclpy 를 전혀 쓰지 않는 오프라인 모델링 도구**다(ROS 그래프
  없이도 떠야 한다 — 모델을 조립하는 데 로봇이 필요하지 않다). 거기에 ROS 를 넣으면
  어셈블러가 ROS 그래프에 묶이고, '모델링'과 '실기 조작'이라는 성격이 다른 두 관심사가
  한 프로세스에 섞인다. ⇒ 패널은 **독립 노드**로 두고, 어셈블러에는 이 패널을 여는
  버튼만 붙였다(사용자 입장에선 통합, 코드 상으론 분리).

■ 무엇을 보고 무엇을 보내나
  본다  : 연결 상태(move_group·컨트롤러) · 안전 상태(`/safety_monitor/abort`) ·
          수확 타깃 목록(`/harvest_targets`) · 관절 상태(`/joint_states`)
  보낸다: 수확 명령(`/harvest_cmd` — 번호/이름/home/list) · **정지**(궤적 goal 취소) ·
          안전 해제(`/safety_monitor/reset`)

■ 🔴 '정지' 는 조작기에 없던 기능이다
  인터랙티브 모드에는 stop 명령이 없다(명령을 받으면 한 사이클을 끝까지 수행한다).
  그래서 여기서는 데모 노드를 거치지 않고 **컨트롤러의 액션 goal 을 직접 취소**한다
  — 안전 감시(safety_monitor)가 결함 때 쓰는 것과 같은 경로다.
  ⚠ 이건 '지금 가는 궤적을 멈추는 것' 이지 비상정지가 아니다. 실기의 하드웨어 E-stop 을
    대체하지 않는다.
"""
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from action_msgs.srv import CancelGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from PyQt5 import QtCore, QtGui, QtWidgets

APP_NAME = "RDA 수확 패널"
#: 상태 색 — 초록=정상 / 노랑=대기 / 빨강=문제
OK, WAIT, BAD = "#2e7d32", "#f9a825", "#c62828"


class PanelNode(Node):
    """ROS 쪽. Qt 위젯을 직접 만지지 않고 값만 들고 있는다(타이머가 읽어 간다)."""

    def __init__(self):
        super().__init__("harvest_panel")
        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.declare_parameter("arm_controller", "arm_controller")
        self.declare_parameter("gripper_controller", "gripper_controller")
        self.arm_ctrl = str(self.get_parameter("arm_controller").value)
        self.grip_ctrl = str(self.get_parameter("gripper_controller").value)

        self.targets_text = ""
        self.joints = {}
        self.abort = False
        self.abort_reason = ""

        self.create_subscription(String, "harvest_targets", self._on_targets, latched)
        self.create_subscription(JointState, "joint_states", self._on_js, 10)
        self.create_subscription(Bool, "/safety_monitor/abort", self._on_abort, latched)
        self.create_subscription(String, "/safety_monitor/abort_reason",
                                 self._on_reason, latched)
        self.cmd_pub = self.create_publisher(String, "harvest_cmd", 10)
        self._reset_cli = self.create_client(Trigger, "/safety_monitor/reset")
        self._cancel = {
            n: self.create_client(
                CancelGoal, f"/{n}/follow_joint_trajectory/_action/cancel_goal")
            for n in (self.arm_ctrl, self.grip_ctrl)}

    # ── 수신 ────────────────────────────────────────────────────────────
    def _on_targets(self, m):
        self.targets_text = m.data

    def _on_js(self, m):
        for n, p in zip(m.name, m.position):
            self.joints[n] = float(p)

    def _on_abort(self, m):
        self.abort = bool(m.data)

    def _on_reason(self, m):
        self.abort_reason = m.data

    # ── 상태 조회 ───────────────────────────────────────────────────────
    def peers(self):
        """무엇이 떠 있는지 판단한다.

        ⚠ 팔 컨트롤러를 액션의 `.../status` 토픽으로 찾으면 **영원히 못 찾는다** —
        액션 내부 토픽은 `/_action/` 이 붙은 **숨김 토픽**이라 그래프 조회에 안 잡힌다
        (실측: 컨트롤러가 멀쩡히 도는데 X 로 표시되면서 정지 버튼은 정상 동작했다).
        ⇒ 취소 서비스의 준비 상태로 본다 — 어차피 정지에 쓰는 바로 그 경로다."""
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        arm_cli = self._cancel.get(self.arm_ctrl)
        return {
            "move_group": "move_group" in names,
            "demo": "pregrasp_demo" in names,
            "arm": bool(arm_cli is not None and arm_cli.service_is_ready()),
            "safety": "safety_monitor" in names,
            "targets": bool(self.targets_text),
        }

    # ── 송신 ────────────────────────────────────────────────────────────
    def send_cmd(self, text):
        m = String()
        m.data = text
        self.cmd_pub.publish(m)

    def stop(self):
        """진행 중 궤적 goal 을 전부 취소. 빈 goal_info = 그 서버의 모든 goal."""
        sent = []
        for name, cli in self._cancel.items():
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())
                sent.append(name)
        return sent

    def reset_safety(self):
        if not self._reset_cli.service_is_ready():
            return False
        self._reset_cli.call_async(Trigger.Request())
        return True


class Panel(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle(APP_NAME)
        self.resize(680, 640)

        root = QtWidgets.QVBoxLayout(self)

        # ── 안전 배너: 결함이면 화면 맨 위에서 빨갛게 — 놓칠 수 없게 ──
        self.banner = QtWidgets.QLabel()
        self.banner.setAlignment(QtCore.Qt.AlignCenter)
        self.banner.setMinimumHeight(38)
        f = self.banner.font()
        f.setBold(True)
        f.setPointSize(f.pointSize() + 1)
        self.banner.setFont(f)
        root.addWidget(self.banner)

        # ── 연결 상태 ──
        gb = QtWidgets.QGroupBox("연결 상태")
        hl = QtWidgets.QHBoxLayout(gb)
        self.dots = {}
        for key, label in (("move_group", "MoveIt"), ("demo", "수확 노드"),
                           ("arm", "팔 컨트롤러"), ("safety", "안전 감시"),
                           ("targets", "타깃 목록")):
            w = QtWidgets.QLabel(f"● {label}")
            self.dots[key] = w
            hl.addWidget(w)
        hl.addStretch(1)
        root.addWidget(gb)

        # ── 타깃 목록 + 명령 ──
        gb2 = QtWidgets.QGroupBox("수확 타깃")
        v2 = QtWidgets.QVBoxLayout(gb2)
        self.targets = QtWidgets.QListWidget()
        self.targets.itemDoubleClicked.connect(lambda _: self._harvest())
        v2.addWidget(self.targets)
        h2 = QtWidgets.QHBoxLayout()
        b_h = QtWidgets.QPushButton("선택 수확")
        b_h.clicked.connect(self._harvest)
        b_l = QtWidgets.QPushButton("목록 갱신")
        b_l.clicked.connect(lambda: self.node.send_cmd("l"))
        b_home = QtWidgets.QPushButton("home 복귀")
        b_home.clicked.connect(lambda: self.node.send_cmd("h"))
        for b in (b_h, b_l, b_home):
            h2.addWidget(b)
        h2.addStretch(1)
        v2.addLayout(h2)
        root.addWidget(gb2, 1)

        # ── 정지 / 안전 해제 ──
        h3 = QtWidgets.QHBoxLayout()
        self.b_stop = QtWidgets.QPushButton("■ 정지 (궤적 취소)")
        self.b_stop.setStyleSheet(
            f"QPushButton{{background:{BAD};color:white;font-weight:bold;padding:8px}}")
        self.b_stop.clicked.connect(self._stop)
        self.b_reset = QtWidgets.QPushButton("안전 해제")
        self.b_reset.clicked.connect(self._reset)
        h3.addWidget(self.b_stop, 2)
        h3.addWidget(self.b_reset, 1)
        root.addLayout(h3)

        # ── 관절 상태 ──
        gb3 = QtWidgets.QGroupBox("관절 상태 (rad)")
        v3 = QtWidgets.QVBoxLayout(gb3)
        self.joints = QtWidgets.QLabel("(수신 대기…)")
        self.joints.setFont(QtGui.QFont("monospace"))
        v3.addWidget(self.joints)
        root.addWidget(gb3)

        # ── 로그 ──
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(110)
        root.addWidget(self.log)

        # rclpy 를 Qt 타이머로 돌린다. 별도 스레드를 쓰지 않는 이유 — 이 패널은
        # 가벼운 구독 몇 개뿐이라 spin_once(0) 로 충분하고, 스레드를 쓰면 Qt 위젯을
        # ROS 콜백에서 만지게 되는 사고를 부른다(Qt 는 GUI 스레드 밖 접근을 허용하지 않는다).
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)
        self._last_targets = None
        self._say("패널 시작 — 수확 노드가 interactive 모드로 떠 있어야 목록이 보입니다.")

    # ── 주기 갱신 ───────────────────────────────────────────────────────
    def _tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

        p = self.node.peers()
        for k, w in self.dots.items():
            w.setStyleSheet(f"color:{OK if p[k] else BAD}")

        if self.node.abort:
            self.banner.setText(f"🔴 정지 상태 — {self.node.abort_reason or '결함'}"
                                "   (원인 제거 후 '안전 해제')")
            self.banner.setStyleSheet(f"background:{BAD};color:white")
        elif not p["demo"]:
            self.banner.setText("수확 노드 없음 — pregrasp_demo 를 interactive:=true 로 띄우세요")
            self.banner.setStyleSheet(f"background:{WAIT};color:black")
        else:
            self.banner.setText("정상")
            self.banner.setStyleSheet(f"background:{OK};color:white")

        # 타깃 목록은 바뀔 때만 다시 그린다(매 100ms 재구성하면 선택이 풀린다)
        t = self.node.targets_text
        if t != self._last_targets:
            self._last_targets = t
            keep = self.targets.currentRow()
            self.targets.clear()
            items = [ln.strip() for ln in t.splitlines() if ln.strip().startswith("[")]
            if not items and t:
                # 줄바꿈이 사라진 형태로 와도 목록이 조용히 비지 않게 한다
                # (한 줄로 뭉쳐 오면 위 파싱이 0건이 되는데, 화면만 보면 '타깃 없음'과
                #  구분이 안 된다 — 실측으로 겪은 혼동).
                items = [m.strip() for m in re.findall(r"\[\d+\][^\[]*", t)]
            for s in items:
                self.targets.addItem(s)
            if 0 <= keep < self.targets.count():
                self.targets.setCurrentRow(keep)

        if self.node.joints:
            self.joints.setText("  ".join(
                f"{n}={v:+.3f}" for n, v in list(self.node.joints.items())[:8]))

    # ── 동작 ────────────────────────────────────────────────────────────
    def _harvest(self):
        it = self.targets.currentItem()
        if it is None:
            self._say("타깃을 먼저 고르세요.")
            return
        # 목록 줄은 "[3] det_13  (0.83,0.13,0.98)  r=0.037" 형태 → 대괄호 안 번호를 보낸다
        text = it.text()
        num = text[1:text.index("]")] if "]" in text else text
        self.node.send_cmd(num)
        self._say(f"수확 명령 전송: [{num}] {text}")

    def _stop(self):
        sent = self.node.stop()
        if sent:
            self._say(f"정지 — 궤적 취소 요청: {', '.join(sent)}")
        else:
            self._say("정지 실패 — 컨트롤러 취소 서비스가 없습니다(실행 중이 아닐 수 있음).")

    def _reset(self):
        if self.node.reset_safety():
            self._say("안전 해제 요청 — 원인이 제거됐는지 확인하세요.")
        else:
            self._say("안전 해제 실패 — safety_monitor 가 떠 있지 않습니다.")

    def _say(self, s):
        self.log.appendPlainText(s)


def main():
    rclpy.init()
    node = PanelNode()
    app = QtWidgets.QApplication(sys.argv)
    w = Panel(node)
    w.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
