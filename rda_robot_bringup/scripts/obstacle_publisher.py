#!/usr/bin/env python3
"""장애물 환경 퍼블리셔 — obstacles.yaml 을 RViz 마커 + MoveIt 플래닝 씬으로 발행.

`config/obstacles.yaml`(단일 진실원)을 읽어 두 곳에 동시에 반영한다:
  · RViz 표시   → /obstacle_markers (MarkerArray, 래치)
  · MoveIt 충돌 → /collision_object (CollisionObject) — planning scene 이 있을 때만

좌표계: yaml 의 `frame`(기본 world). world → base_link 고정 TF 는 launch 가 발행.

파라미터:
  obstacles_file (str)  yaml 경로. 비우면 rda_robot_description/config/obstacles.yaml
  marker_topic (str, 'obstacle_markers')
  publish_collision (bool, True)   MoveIt CollisionObject 발행 여부
  period (float, 1.0)              재발행 주기[s] (0 이면 1회만)
"""
import os
import sys

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray

try:
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    HAVE_MOVEIT = True
except ImportError:      # MoveIt 미설치 환경에서도 마커는 뜨게
    HAVE_MOVEIT = False

# yaml type -> (Marker.type, SolidPrimitive.type)
SHAPES = {
    "box": (Marker.CUBE, "BOX"),
    "cylinder": (Marker.CYLINDER, "CYLINDER"),
    "sphere": (Marker.SPHERE, "SPHERE"),
}


def quat_from_rpy(r, p, y):
    import math
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def default_yaml():
    return os.path.join(
        get_package_share_directory("rda_robot_description"), "config", "obstacles.yaml"
    )


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__("obstacle_publisher")
        self.declare_parameter("obstacles_file", "")
        self.declare_parameter("marker_topic", "obstacle_markers")
        self.declare_parameter("publish_collision", True)
        self.declare_parameter("period", 1.0)

        path = self.get_parameter("obstacles_file").value or default_yaml()
        self.path = path
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.mpub = self.create_publisher(
            MarkerArray, self.get_parameter("marker_topic").value, latched
        )
        self.want_collision = bool(self.get_parameter("publish_collision").value)
        self.cpub = None
        if self.want_collision and HAVE_MOVEIT:
            self.cpub = self.create_publisher(CollisionObject, "collision_object", latched)
        elif self.want_collision and not HAVE_MOVEIT:
            self.get_logger().warn(
                "moveit_msgs 를 import 할 수 없어 CollisionObject 발행을 건너뜁니다 "
                "(RViz 마커만 발행). MoveIt 설치 후 다시 실행하세요."
            )

        self.spec = self.load()
        self.publish_all()
        p = float(self.get_parameter("period").value)
        if p > 0:
            self.create_timer(p, self.publish_all)

    def load(self):
        if not os.path.exists(self.path):
            raise FileNotFoundError(f"장애물 정의 파일 없음: {self.path}")
        with open(self.path) as f:
            spec = yaml.safe_load(f) or {}
        obs = spec.get("obstacles") or []
        if not obs:
            self.get_logger().warn(f"{self.path} 에 obstacles 항목이 비어 있습니다.")
        # 조용한 오작동 방지: 형상별 필수 키를 로드 시점에 검사
        for o in obs:
            name, t = o.get("name"), o.get("type")
            if not name:
                raise ValueError(f"name 없는 장애물 항목: {o}")
            if t not in SHAPES:
                raise ValueError(f"[{name}] 알 수 없는 type={t!r} (지원: {list(SHAPES)})")
            need = {"box": ["size"], "cylinder": ["radius", "height"], "sphere": ["radius"]}[t]
            miss = [k for k in need if o.get(k) is None]
            if miss:
                raise ValueError(f"[{name}] type={t} 에 필수 키 누락: {miss}")
            if t == "box" and len(o["size"]) != 3:
                raise ValueError(f"[{name}] box size 는 [x,y,z] 3개여야 함: {o['size']}")
        self.get_logger().info(
            f"장애물 {len(obs)}개 로드: {self.path} (frame={spec.get('frame','world')})"
        )
        return spec

    def _pose(self, o):
        pose = Pose()
        pz = o.get("pose") or {}
        xyz = pz.get("xyz", [0.0, 0.0, 0.0])
        rpy = pz.get("rpy", [0.0, 0.0, 0.0])
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz]
        qx, qy, qz, qw = quat_from_rpy(*[float(v) for v in rpy])
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw
        return pose

    def _dims(self, o):
        t = o["type"]
        if t == "box":
            return [float(v) for v in o["size"]]
        if t == "cylinder":
            return [float(o["height"]), float(o["radius"])]     # SolidPrimitive 순서
        return [float(o["radius"])]

    def publish_all(self):
        frame = self.spec.get("frame", "world")
        arr = MarkerArray()
        for i, o in enumerate(self.spec.get("obstacles", [])):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "obstacles"
            m.id = i
            m.type = SHAPES[o["type"]][0]
            m.action = Marker.ADD
            m.pose = self._pose(o)
            t = o["type"]
            if t == "box":
                m.scale.x, m.scale.y, m.scale.z = self._dims(o)
            elif t == "cylinder":
                r, h = float(o["radius"]), float(o["height"])
                m.scale.x = m.scale.y = 2.0 * r
                m.scale.z = h
            else:
                d = 2.0 * float(o["radius"])
                m.scale.x = m.scale.y = m.scale.z = d
            c = o.get("color", [0.5, 0.5, 0.5, 0.8])
            m.color.r, m.color.g, m.color.b, m.color.a = [float(v) for v in c]
            arr.markers.append(m)

            # 이름표
            tm = Marker()
            tm.header = m.header
            tm.ns = "obstacle_labels"
            tm.id = i
            tm.type = Marker.TEXT_VIEW_FACING
            tm.action = Marker.ADD
            tm.pose = self._pose(o)
            tm.pose.position.z += 0.08
            tm.scale.z = 0.06
            tm.color.r = tm.color.g = tm.color.b = tm.color.a = 1.0
            kind = o.get("kind", "obstacle")
            tm.text = o["name"] if kind == "obstacle" else f"{o['name']} [제한]"
            arr.markers.append(tm)

        arr.markers.extend(self._workspace_markers(frame, len(arr.markers)))
        self.mpub.publish(arr)

    def _workspace_markers(self, frame, base_id):
        """작업 공간 경계를 와이어프레임으로 표시(면으로 그리면 안이 안 보임)."""
        ws = self.spec.get("workspace")
        if not ws:
            return []
        c = [float(v) for v in ws["center"]]
        s = [float(v) for v in ws["size"]]
        lo = [c[i] - s[i] / 2 for i in range(3)]
        hi = [c[i] + s[i] / 2 for i in range(3)]
        corners = [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        # 한 좌표만 다른 꼭짓점끼리 잇는다 = 박스 12모서리
        edges = [(a, b) for i, a in enumerate(corners) for b in corners[i + 1:]
                 if sum(1 for k in range(3) if abs(a[k] - b[k]) > 1e-9) == 1]
        from geometry_msgs.msg import Point
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "workspace"
        m.id = base_id
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.006
        m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.85, 0.85, 0.75
        for a, b in edges:
            for p in (a, b):
                pt = Point()
                pt.x, pt.y, pt.z = p
                m.points.append(pt)
        return [m]

        if self.cpub is not None:
            for o in self.spec.get("obstacles", []):
                co = CollisionObject()
                co.header.frame_id = frame
                co.header.stamp = self.get_clock().now().to_msg()
                co.id = o["name"]
                sp = SolidPrimitive()
                sp.type = getattr(SolidPrimitive, SHAPES[o["type"]][1])
                sp.dimensions = self._dims(o)
                co.primitives.append(sp)
                co.primitive_poses.append(self._pose(o))
                co.operation = CollisionObject.ADD
                self.cpub.publish(co)


def main():
    rclpy.init()
    try:
        node = ObstaclePublisher()
    except Exception as e:                       # 설정 오류는 조용히 넘기지 않는다
        print(f"[obstacle_publisher] 설정 오류: {e}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
