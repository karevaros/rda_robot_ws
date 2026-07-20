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

import math

# yaml type -> (Marker.type, SolidPrimitive.type)
SHAPES = {
    "box": (Marker.CUBE, "BOX"),
    "cylinder": (Marker.CYLINDER, "CYLINDER"),
    "sphere": (Marker.SPHERE, "SPHERE"),
}

# kind 종류
#   obstacle : 실제 물체. 회피 대상 → 마커 + CollisionObject.
#   keepout  : 가상 이동제한(실체 없음). 회피 대상 → 마커 + CollisionObject.
#   target   : 집기 목표(토마토). **회피 대상이 아님** → 마커만(플래닝 목표를 장애물로
#              세면 "목표 접근 = 충돌"이 된다). 잡을 때 개별로 collision 에 추가하는 건 6주차.
COLLISION_KINDS = ("obstacle", "keepout")


def _cluster_offsets(n, s):
    """열매 n개(3~4)를 뭉친 bunch 로 배치할 상대 오프셋(줄기 기준 로컬).
    거의 맞닿는 구들의 사면체 패킹 — 1자가 아니라 뭉쳐 보이게. s=열매 간격."""
    base = [
        (0.0,        0.0,        0.0),
        (s,          0.0,       -0.10 * s),
        (0.5 * s,    0.87 * s,  -0.10 * s),
        (0.5 * s,    0.29 * s,  -0.82 * s),   # 4번째는 아래 apex
    ]
    return base[:max(1, min(n, 4))]


def _seg_cylinder(name, p1, p2, radius, color):
    """두 점 p1→p2 를 잇는 원통 item. cylinder 기본축(Z)을 방향벡터로 회전.
    rpy=(0, θ, φ): θ=acos(dz/len), φ=atan2(dy,dx) → Rz(φ)Ry(θ)·ez = 방향."""
    dx, dy, dz = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
    ln = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-9
    theta = math.acos(max(-1.0, min(1.0, dz / ln)))
    phi = math.atan2(dy, dx)
    mid = [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2]
    return {"name": name, "kind": "obstacle", "type": "cylinder",
            "pose": {"xyz": mid, "rpy": [0.0, theta, phi]},
            "radius": radius, "height": ln, "color": color, "nolabel": True}


def expand_crops(spec):
    """spec['crops'](파라메트릭 템플릿+배치) → 개체(줄기·화방·열매)로 펼쳐 obstacles 에 append.

    형태(사진 참고, docs/crop_ref): 고설 유인 토마토.
      · 줄기 = **로그 곡선으로 +Y 방향 상승**(거터에서 위로 휘어 오름) → 짧은 원통 여러 마디.
      · 화방 = 줄기에서 **한쪽(통로쪽)으로만** 나와 아래로 드룹, 열매 **3~4개**.
    근거 수치: 줄기 Ø11.5mm·(열매수 참고 AIHub) · 대과 Ø~7cm(통상) · 줄피치1.66m·거터0.92m(STEP).
    줄기·화방대=obstacle(회피), 열매=target(집기 목표).
    """
    cr = spec.get("crops")
    if not cr:
        return 0
    tpl = cr.get("template", {})
    stem = tpl.get("stem", {})
    tr = tpl.get("truss", {})
    span = cr.get("span", {})
    rows = cr.get("rows", [])
    y0 = float(span.get("y_min", -2.0))
    y1 = float(span.get("y_max", 2.0))
    dy = float(span.get("spacing", 0.4))
    n_plants = max(1, int(round((y1 - y0) / dy)) + 1)

    s_r = float(stem.get("radius", 0.006))
    s_h = float(stem.get("height", 2.2))
    lean_y = float(stem.get("lean_y", 0.30))       # +Y 최대 편차
    curve_k = float(stem.get("curve_k", 4.0))      # 로그 곡률
    n_seg = int(stem.get("segments", 6))
    first_z = float(tr.get("first_z", 0.35))
    t_gap = float(tr.get("spacing", 0.25))
    t_cnt = int(tr.get("count", 3))
    t_maxz = tr.get("max_z")
    base_nf = int(tr.get("fruits_per_truss", 4))   # 화방당 3~4
    f_r = float(tr.get("fruit_radius", 0.035))
    rachis_out = float(tr.get("rachis_out", 0.06))     # 줄기→클러스터 중심 옆거리(+Y)
    cluster_gap = float(tr.get("cluster_gap", 0.85))   # 열매 간격(지름 배수, <1 겹침=뭉침)
    rachis_r = float(tr.get("rachis_radius", 0.004))
    stem_color = tpl.get("stem_color", [0.30, 0.50, 0.20, 0.95])
    rachis_color = tpl.get("rachis_color", [0.35, 0.55, 0.25, 0.95])
    fruit_color = tpl.get("fruit_color", [0.90, 0.20, 0.13, 0.97])
    ln1k = math.log(1.0 + curve_k) or 1e-9

    def stem_pt(x, y, gtop, t):
        """줄기 곡선 위 점: 높이 t(0~1), +Y 로 로그 편차."""
        z = gtop + t * s_h
        yy = y + lean_y * math.log(1.0 + curve_k * t) / ln1k
        return [x, yy, z]

    items = spec.setdefault("obstacles", [])
    added = 0
    for ri, row in enumerate(rows):
        x = float(row["x"])
        gtop = float(row.get("gutter_top", 0.92))
        row_has_fruit = int(row.get("fruits_per_truss", base_nf)) > 0
        # 화방(열매)이 달리는 쪽(Y): 통로에서 줄기를 바라볼 때 '왼쪽' = +Y(앞줄 x>0 기준).
        #  근거: 앞줄은 통로에서 +X 를 바라봄 → 왼쪽 = +Y. 'left'/'right'/'+y'/'-y' 로 고정 가능.
        side_spec = str(row.get("truss_side", "left"))
        if side_spec == "+y":
            side_y = 1.0
        elif side_spec == "-y":
            side_y = -1.0
        elif side_spec == "right":
            side_y = -1.0 if x >= 0 else 1.0
        else:  # left(기본)
            side_y = 1.0 if x >= 0 else -1.0
        for pi in range(n_plants):
            y = y0 + pi * dy
            # 줄기: 로그 곡선을 짧은 원통 마디로
            pts = [stem_pt(x, y, gtop, k / n_seg) for k in range(n_seg + 1)]
            for si in range(n_seg):
                items.append(_seg_cylinder(f"stem_r{ri}_p{pi}_s{si}",
                                           pts[si], pts[si + 1], s_r, stem_color))
                added += 1
            if not row_has_fruit:
                continue
            # 화방: 줄기 왼쪽(+Y)으로 나와 아래로 드룹하는 '뭉친' 열매 클러스터 3~4개
            gap = cluster_gap * 2.0 * f_r
            for ti in range(t_cnt):
                tz = gtop + first_z + ti * t_gap
                if t_maxz is not None and tz > float(t_maxz):
                    break
                nf = base_nf if (pi + ti) % 2 == 0 else max(3, base_nf - 1)
                t_at = min(1.0, (tz - gtop) / s_h)
                ax, ay, _ = stem_pt(x, y, gtop, t_at)         # 줄기 위 부착점
                # 클러스터 중심: 왼쪽(+side_y)으로 rachis_out, 살짝 아래
                cy = ay + side_y * (rachis_out + f_r)
                cz = tz - f_r
                for fi, (ox, oy, oz) in enumerate(_cluster_offsets(nf, gap)):
                    items.append({
                        "name": f"fruit_r{ri}_p{pi}_t{ti}_f{fi}", "kind": "target",
                        "type": "sphere",
                        "pose": {"xyz": [ax + ox, cy + oy, cz + oz]},
                        "radius": f_r, "color": fruit_color, "nolabel": True,
                    })
                    added += 1
                # 화방대(줄기→클러스터): 얇은 원통
                items.append(_seg_cylinder(f"rachis_r{ri}_p{pi}_t{ti}",
                                           [ax, ay, tz], [ax, cy, cz], rachis_r, rachis_color))
                added += 1
    return added


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
        self.spec = self.load()      # QoS depth 산정에 장애물 수가 필요 → 먼저 로드

        # 마커는 MarkerArray 1개로 전부 보내므로 depth=1 로 충분
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
            # depth 는 장애물 수보다 크게 잡는다(마커와 달리 장애물 1개당 메시지 1개).
            # KEEP_LAST(depth=1) 이면 연달아 보낸 앞 메시지가 덮일 수 있어 위험하다
            # — RELIABLE 도 "최신 depth 개"를 보장할 뿐 전부를 보장하진 않는다.
            depth = max(20, len(self.spec.get("obstacles") or []) * 2)
            self.cpub = self.create_publisher(
                CollisionObject, "collision_object",
                QoSProfile(depth=depth,
                           reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           history=QoSHistoryPolicy.KEEP_LAST))
        elif self.want_collision and not HAVE_MOVEIT:
            self.get_logger().warn(
                "moveit_msgs 를 import 할 수 없어 CollisionObject 발행을 건너뜁니다 "
                "(RViz 마커만 발행). MoveIt 설치 후 다시 실행하세요."
            )

        self.publish_all()
        p = float(self.get_parameter("period").value)
        if p > 0:
            self.create_timer(p, self.publish_all)

    def load(self):
        if not os.path.exists(self.path):
            raise FileNotFoundError(f"장애물 정의 파일 없음: {self.path}")
        with open(self.path) as f:
            spec = yaml.safe_load(f) or {}
        n_crop = expand_crops(spec)      # 파라메트릭 작물 → obstacles 로 전개
        if n_crop:
            self.get_logger().info(f"작물 파라메트릭 전개: {n_crop}개 개체 생성(줄기·열매).")
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
        try:
            self._src_mtime = os.path.getmtime(self.path)
        except OSError:
            self._src_mtime = None
        return spec

    def _maybe_reload(self):
        """yaml 파일이 바뀌었으면 다시 읽어 즉시 반영(재기동 없이 파라미터 튜닝).

        편집 중 저장돼 파싱이 깨지면(불완전 저장) 이전 것을 유지하고 경고만 낸다.
        """
        try:
            m = os.path.getmtime(self.path)
        except OSError:
            return False
        if m == getattr(self, "_src_mtime", None):
            return False
        try:
            new_spec = self.load()          # load 가 _src_mtime 을 갱신
        except Exception as e:
            self._src_mtime = m             # 같은 깨진 상태로 매틱 재시도하지 않도록
            self.get_logger().warn(f"장애물 파일 재로딩 실패(편집 중?): {e}")
            return False
        self.spec = new_spec
        self.get_logger().info("장애물 파일 변경 감지 → 다시 발행합니다.")
        return True

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
        reloaded = self._maybe_reload()
        frame = self.spec.get("frame", "world")
        # 리로드로 개체 수가 줄면 옛 마커가 남으므로 먼저 전부 지운다(DELETEALL).
        if reloaded:
            clr = MarkerArray()
            for ns in ("obstacles", "obstacle_labels", "workspace"):
                dm = Marker()
                dm.header.frame_id = frame
                dm.ns = ns
                dm.action = Marker.DELETEALL
                clr.markers.append(dm)
            self.mpub.publish(clr)
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

            # 이름표 — 작물처럼 수백 개면 라벨이 화면을 덮으므로 nolabel 은 건너뜀
            if o.get("nolabel"):
                continue
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
            suffix = {"keepout": " [제한]", "target": " [목표]"}.get(kind, "")
            tm.text = f"{o['name']}{suffix}"
            arr.markers.append(tm)

        arr.markers.extend(self._workspace_markers(frame, len(arr.markers)))
        self.mpub.publish(arr)

        # MoveIt planning scene 로도 같은 장애물을 보낸다(단일 진실원 유지).
        # target(집기 목표)은 회피 대상이 아니라 제외 — 목표를 장애물로 세면 플래닝 실패.
        if self.cpub is not None:
            cur_ids = {o["name"] for o in self.spec.get("obstacles", [])
                       if o.get("kind") != "target"}
            # 리로드로 사라진 충돌객체는 planning scene 에서 REMOVE.
            if reloaded:
                for gone in getattr(self, "_prev_ids", set()) - cur_ids:
                    rm = CollisionObject()
                    rm.header.frame_id = frame
                    rm.id = gone
                    rm.operation = CollisionObject.REMOVE
                    self.cpub.publish(rm)
            self._prev_ids = cur_ids
            n_pub = 0
            for o in self.spec.get("obstacles", []):
                if o.get("kind") == "target":
                    continue
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
                n_pub += 1
            self._pub_count = getattr(self, "_pub_count", 0) + 1
            if self._pub_count <= 3 or self._pub_count % 30 == 0:
                self.get_logger().info(
                    f"CollisionObject {n_pub}개 발행 (#{self._pub_count}, frame={frame})")

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
