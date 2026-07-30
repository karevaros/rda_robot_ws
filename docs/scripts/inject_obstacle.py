#!/usr/bin/env python3
"""실행 중 장애물 주입기 — 재계획 트리거의 **발화 경로**를 검증하는 진단 도구.

    python3 docs/scripts/inject_obstacle.py [--size 0.4 0.6 0.5] [--at 0.6 0.0 1.0]
                                            [--delay 1.0] [--remove-after 0]

■ 왜 필요한가 (2026-07-30)
  재계획 트리거(`replan`)를 만든 뒤 센싱 장면에서 수확런을 두 번(harvest_linear·direct)
  돌렸는데 **한 번도 발화하지 않았다.** 그건 좋은 소식이지만(거짓 트리거 없음), 정작
  '진짜로 막혔을 때 제대로 끊고 다시 계획하는지' 는 검증되지 않은 채 남았다.
  자연 발생을 기다릴 수 없으므로 **실행 중에 장애물을 직접 넣어** 발화시킨다.

■ 동작
  `/joint_states` 를 보다가 팔이 움직이기 시작하면(관절 변화 > `--thresh`) `--delay` 초 뒤
  planning scene 에 상자 하나를 ADD 한다. 그 뒤 데모 로그에서 확인할 것:
    "실행 중 장면 변화 감지 — 남은 N점 중 M점이 막혔다 … 접촉: …"
    "실제 자세 동기화 — 가정했던 자세와 최대 X rad 차이"
    "[…] 재계획 1/2 — 현재 자세에서 다시 계획한다"

■ ⚠ 진단 전용이다
  planning scene 에만 넣으므로 Gazebo 물리와는 무관하다(로봇이 이 상자를 실제로 치지는
  않는다). '계획이 이 공간을 피해야 한다' 고 MoveIt 에 알리는 것뿐이다.
  실기에서는 절대 쓰지 말 것 — 계획이 갑자기 막히는 상황을 인위적으로 만드는 도구다.
"""
import argparse
import sys

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

OBJ_ID = "replan_test_block"


class Injector(Node):
    def __init__(self, a):
        super().__init__("inject_obstacle")
        self.a = a
        self.cli = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.get_cli = self.create_client(GetPlanningScene, "get_planning_scene")
        self.start = None
        self.fired_at = None
        self.create_subscription(JointState, "joint_states", self._on_js, 10)

    def _on_js(self, m):
        cur = {n: float(p) for n, p in zip(m.name, m.position)}
        if self.start is None:
            self.start = cur
            self.get_logger().info("기준 자세 기록 — 팔이 움직이기를 기다립니다…")
            return
        if self.fired_at is not None:
            return
        moved = max((abs(cur[n] - self.start[n]) for n in cur if n in self.start),
                    default=0.0)
        if moved > self.a.thresh:
            self.get_logger().info(
                f"움직임 감지({moved:.3f}rad > {self.a.thresh}) → {self.a.delay}s 뒤 주입")
            self.fired_at = self.get_clock().now().nanoseconds + int(self.a.delay * 1e9)

    def _scene(self, op):
        co = CollisionObject()
        co.header.frame_id = self.a.frame
        co.id = OBJ_ID
        co.operation = op
        if op == CollisionObject.ADD:
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = list(self.a.size)
            p = Pose()
            p.position.x, p.position.y, p.position.z = self.a.at
            p.orientation.w = 1.0
            co.primitives.append(box)
            co.primitive_poses.append(p)
        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request(scene=ps)
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        # 🔴 응답을 믿지 말고 **재조회로 확인**한다.
        #   실측(2026-07-30): 상자는 실제로 들어갔는데 이 응답이 10초 안에 오지 않아
        #   '주입 실패' 로 보고했다 — 데모 로그에는 그 상자와의 접촉이 찍히고 있었다.
        #   (이 프로젝트가 Stage 5 에서 이미 기록해 둔 교훈인데 여기서 다시 밟았다.)
        want = (op == CollisionObject.ADD)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._in_scene() == want:
                return True
        return False

    def _in_scene(self):
        """장면에 상자가 실제로 있는지 재조회(응답 대신 상태를 본다)."""
        if not self.get_cli.service_is_ready():
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None:
            return None
        return OBJ_ID in [o.id for o in res.scene.world.collision_objects]

    def run(self):
        if not self.cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                "apply_planning_scene 서비스가 없습니다 — move_group 이 떠 있는지 확인.")
            return 1
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.fired_at is None:
                continue
            if self.get_clock().now().nanoseconds < self.fired_at:
                continue
            break
        ok = self._scene(CollisionObject.ADD)
        self.get_logger().info(
            f"장애물 주입 {'성공' if ok else '실패'} — {self.a.frame} "
            f"@{tuple(self.a.at)} 크기 {tuple(self.a.size)}")
        if not ok:
            return 1
        if self.a.remove_after > 0:
            end = self.get_clock().now().nanoseconds + int(self.a.remove_after * 1e9)
            while rclpy.ok() and self.get_clock().now().nanoseconds < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            self._scene(CollisionObject.REMOVE)
            self.get_logger().info("장애물 제거 — 장면을 원래대로 되돌렸습니다.")
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frame", default="world")
    ap.add_argument("--size", nargs=3, type=float, default=[0.4, 0.8, 0.6],
                    help="상자 크기 x y z (m)")
    ap.add_argument("--at", nargs=3, type=float, default=[0.55, 0.0, 1.05],
                    help="상자 중심 위치 (frame 기준). 기본값은 팔 앞쪽 작업공간")
    ap.add_argument("--thresh", type=float, default=0.05, help="움직임 감지 임계 rad")
    ap.add_argument("--delay", type=float, default=1.0, help="움직임 감지 후 주입까지 s")
    ap.add_argument("--remove-after", type=float, default=0.0,
                    help=">0 이면 그 초 뒤 상자를 제거(0=남겨 둠)")
    a = ap.parse_args()

    rclpy.init()
    node = Injector(a)
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
