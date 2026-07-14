#!/usr/bin/env python3
"""자충돌 모니터 노드 — 기본 RViz 에서 움직임에 따른 자충돌 감지.

/robot_description(래치)로 통합 URDF 를 받아 링크별 충돌 mesh 를 구축하고,
/joint_states 를 구독해 매번 FK → self-collision 검사(trimesh + python-fcl).
충돌 링크를 RViz 에 빨강 AABB 박스 + 상태 텍스트 마커로 표시한다.

무시(allowed)되는 쌍:
  · 관절로 연결된 인접 링크(설계상 항상 접촉)
  · 시작 자세에서 이미 겹쳐 있는 쌍(auto_baseline) — 모델 자체 겹침 보정
    → 서비스 '~/recalibrate_baseline' 로 현재 자세 기준 재보정 가능

파라미터:
  marker_topic (str, 'self_collision_markers')
  auto_baseline (bool, True)
  min_period (float, 0.1)  검사 최소 간격[s]
  text_z (float, 1.6)      상태 텍스트 높이[m]
"""
import os
import tempfile
import numpy as np
import trimesh
import yourdfpy
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


def pkg_resolver(fname):
    """package://PKG/rest -> <share>/rest (yourdfpy filename_handler)."""
    if fname.startswith("package://"):
        rest = fname[len("package://"):]
        pkg, _, tail = rest.partition("/")
        try:
            return os.path.join(get_package_share_directory(pkg), tail)
        except Exception:
            return fname
    return fname


class SelfCollisionMonitor(Node):
    def __init__(self):
        super().__init__("self_collision_monitor")
        self.declare_parameter("marker_topic", "self_collision_markers")
        self.declare_parameter("auto_baseline", True)
        self.declare_parameter("min_period", 0.1)
        self.declare_parameter("text_z", 1.6)

        self.robot = None
        self.fk = None
        self.mgr = None
        self.linkmesh = {}
        self.link_aabb = {}     # link -> (center(3), extent(3)) in link frame
        self.adj = set()        # frozenset(a,b) 인접 링크
        self.allowed = set()    # frozenset(a,b) 기준 보정
        self.base_frame = "base_link"
        self._baselined = False
        self._pending_calib = False
        self._last = 0.0
        self._prev_pairs = set()

        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "robot_description", self._on_desc, latched)
        self.create_subscription(JointState, "joint_states", self._on_js, 10)
        topic = self.get_parameter("marker_topic").value
        self.pub = self.create_publisher(MarkerArray, topic, 1)
        self.create_service(Trigger, "~/recalibrate_baseline", self._on_recalib)
        self.get_logger().info("self_collision_monitor 시작 — /robot_description 대기")

    # ---------- 모델 구축 ----------
    def _on_desc(self, msg):
        try:
            self._build(msg.data)
        except Exception as e:
            self.get_logger().error(f"URDF 로드/충돌모델 구축 실패: {e}")

    def _build(self, urdf_xml):
        path = os.path.join(tempfile.gettempdir(), "rda_selfcol.urdf")
        with open(path, "w") as f:
            f.write(urdf_xml)
        robot = yourdfpy.URDF.load(
            path, filename_handler=pkg_resolver,
            load_meshes=True, build_scene_graph=True)
        base = robot.scene.graph.base_frame

        def fk(link):
            return np.asarray(
                robot.scene.graph.get(frame_to=link, frame_from=base)[0], float)

        # zero 자세에서 링크프레임 mesh baking
        try:
            robot.update_cfg({j: 0.0 for j in robot.actuated_joint_names})
        except Exception:
            pass
        linkmesh = {}
        for node in robot.scene.graph.nodes_geometry:
            T, gname = robot.scene.graph[node]
            geom = robot.scene.geometry.get(gname)
            if geom is None or getattr(geom, "faces", None) is None:
                continue
            link = robot.scene.graph.transforms.parents.get(node) or base
            Tbl = fk(link)
            m = geom.copy()
            m.apply_transform(np.linalg.inv(Tbl) @ np.asarray(T, float))
            linkmesh.setdefault(link, []).append(m)

        mgr = trimesh.collision.CollisionManager()
        aabb = {}
        for link, ms in linkmesh.items():
            comb = ms[0] if len(ms) == 1 else trimesh.util.concatenate(ms)
            mgr.add_object(link, comb)
            lo, hi = comb.bounds
            aabb[link] = ((lo + hi) / 2.0, np.maximum(hi - lo, 1e-3))

        self.robot, self.fk, self.base_frame = robot, fk, base
        self.mgr, self.linkmesh, self.link_aabb = mgr, linkmesh, aabb
        self.adj = {frozenset((j.parent, j.child))
                    for j in robot.joint_map.values() if j.parent and j.child}
        self.allowed = set()
        self._baselined = False
        self.get_logger().info(
            f"충돌모델 구축: 링크 {len(linkmesh)}개, 인접무시 {len(self.adj)}쌍, base={base}")

    # ---------- 검사 ----------
    def _on_js(self, msg):
        if self.robot is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last < float(self.get_parameter("min_period").value):
            return
        self._last = now

        acts = set(self.robot.actuated_joint_names)
        cfg = {n: p for n, p in zip(msg.name, msg.position) if n in acts}
        if cfg:
            try:
                self.robot.update_cfg(cfg)
            except Exception:
                return
        Ws = {}
        for link in self.linkmesh:
            W = self.fk(link)
            Ws[link] = W
            self.mgr.set_transform(link, W)

        hit, names = self.mgr.in_collision_internal(return_names=True)
        pairs = {frozenset(p) for p in names} if hit else set()
        pairs -= self.adj

        # 기준 보정(시작 자세 또는 서비스 요청)
        if self._pending_calib or (not self._baselined and
                                   self.get_parameter("auto_baseline").value):
            self.allowed = set(pairs)
            self._baselined = True
            self._pending_calib = False
            self.get_logger().info(f"기준 보정: 현재 자세 겹침 {len(self.allowed)}쌍 무시 등록")

        real = pairs - self.allowed
        if real and real != self._prev_pairs:
            desc = ", ".join(sorted("↔".join(sorted(p)) for p in real))
            self.get_logger().warn(f"⚠ 자충돌 {len(real)}쌍: {desc}")
        self._prev_pairs = real
        self._publish(real, Ws)

    def _on_recalib(self, req, resp):
        self._pending_calib = True
        resp.success = True
        resp.message = "다음 검사에서 현재 자세를 기준으로 재보정합니다."
        return resp

    # ---------- 마커 ----------
    def _publish(self, pairs, Ws):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        colliding = set()
        for p in pairs:
            colliding |= set(p)
        for i, link in enumerate(sorted(colliding)):
            c, ext = self.link_aabb[link]
            W = Ws[link]
            pos = W @ np.array([c[0], c[1], c[2], 1.0])
            q = Rotation.from_matrix(np.array(W[:3, :3], dtype=float)).as_quat()  # xyzw
            m = Marker()
            m.header.frame_id = self.base_frame
            m.ns = "self_collision"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.x = float(q[0])
            m.pose.orientation.y = float(q[1])
            m.pose.orientation.z = float(q[2])
            m.pose.orientation.w = float(q[3])
            m.scale.x = float(ext[0])
            m.scale.y = float(ext[1])
            m.scale.z = float(ext[2])
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.1, 0.1, 0.45
            arr.markers.append(m)

        t = Marker()
        t.header.frame_id = self.base_frame
        t.ns = "self_collision_text"
        t.id = 1000
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.z = float(self.get_parameter("text_z").value)
        t.pose.orientation.w = 1.0
        t.scale.z = 0.12
        if pairs:
            body = "\n".join(sorted("↔".join(sorted(p)) for p in pairs))
            t.text = f"SELF-COLLISION ({len(pairs)})\n{body}"
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 0.25, 0.25, 1.0
        else:
            t.text = "self-collision: OK"
            t.color.r, t.color.g, t.color.b, t.color.a = 0.2, 0.9, 0.35, 1.0
        arr.markers.append(t)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = SelfCollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
