#!/usr/bin/env python3
"""수확 조작기 — 실제 로봇 조작하듯 사용자가 타깃을 골라 명령하는 키보드 인터페이스.

pregrasp_demo 를 interactive:=true 로 띄운 뒤, **다른 터미널**에서 이 스크립트를 실행한다:
  ros2 run rda_robot_bringup harvest_operator.py

동작:
  · pregrasp_demo 가 발행하는 /harvest_targets(수확 타깃 목록)를 화면에 보여준다.
  · 사용자가 번호를 입력하면 /harvest_cmd 로 보내 → 데모가 그 열매를 전체 수확(home→접근→파지→후퇴).
  · l=목록 갱신, h=home 복귀, q=종료.

⚠ 이 스크립트는 stdin(키보드)이 필요하므로 launch 에 넣지 않고 사용자 터미널에서 직접 실행한다.
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String


class HarvestOperator(Node):
    def __init__(self):
        super().__init__("harvest_operator")
        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, "harvest_cmd", 10)
        self.latest = "(타깃 목록 대기중… pregrasp_demo 가 interactive:=true 로 떴는지 확인)"
        self.create_subscription(String, "harvest_targets", self._cb, latched)

    def _cb(self, msg):
        self.latest = msg.data

    def send(self, s):
        m = String()
        m.data = s
        self.pub.publish(m)


def main():
    rclpy.init()
    node = HarvestOperator()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    print("=" * 60)
    print(" 🍅 수확 조작기 — 타깃 번호를 입력해 수확을 명령하세요.")
    print("    (l=목록 갱신, h=home 복귀, q=종료)")
    print("=" * 60)
    try:
        while rclpy.ok():
            print("\n" + node.latest)
            try:
                s = input("타깃 번호> ").strip()
            except EOFError:
                break
            if s.lower() in ("q", "quit", "exit"):
                break
            if not s:
                continue
            node.send(s)
            if s.lower() not in ("l", "list", "refresh"):
                print(f"  → 명령 전송: '{s}' (로봇 동작 중… 데모 터미널 로그 참고)")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
