#!/usr/bin/env python3
"""열매 인지 노드 (perception Stage 4) — 카메라가 본 빨간 토마토를 3D 타깃으로 낸다.

입력  : depth 카메라의 **organized PointCloud2**(`/d435i/depth/points`, `/d405/depth/points`).
        Gazebo/RealSense 클라우드는 640×480 격자에 xyz + rgb 가 같이 실려 있어
        **컬러 이미지와 깊이가 이미 1:1 정렬**돼 있다 → 역투영 계산이 따로 필요 없다.
출력  : `/detected_fruits` (visualization_msgs/MarkerArray, world 좌표 SPHERE)
        marker.scale = 지름, ns='fruit', id=추적번호 → 그대로 집기 타깃으로 쓴다.

파이프라인
  ① 빨강 세그멘테이션 : HSV 임계(붉은색은 H 가 0/180 양끝으로 갈라져 두 구간을 OR).
  ② 2D 연결성분        : organized 격자에서 라벨링(scipy.ndimage).
  ③ 3D 분리            : 한 덩어리로 붙은 인접 열매를 단일연결 군집화로 쪼갬(cluster_gap).
  ④ 중심·반경 추정     : ★ 카메라는 열매의 **앞면(반구)만** 본다 → 보이는 점들의 무게중심은
                         중심보다 카메라 쪽으로 치우친다. 반구 껍질의 무게중심은 중심에서
                         r/2 만큼 앞이므로, 시선방향으로 r/2 를 **밀어서** 보정한다.
                         r 은 시선축에 수직인 퍼짐(95퍼센타일)으로 추정.
  ⑤ 좌표변환·추적      : TF 로 world 좌표화 → 프레임/카메라 간 같은 열매를 병합(EMA),
                         `min_hits` 회 이상 관측된 것만 발행(순간 노이즈 제거).

⚠ 이 노드는 **이름표를 모른다.** obstacles.yaml 의 `fruit_r0_p3_t0_f2` 같은 이름은 실환경
   센싱엔 존재하지 않는다 → 접근 시 '목표 화방대만 ACM 제외' 하던 방식은 Stage 5 에서
   **열매 주변 구(sphere) 영역 허용**으로 바꿔야 한다.

실행:
  ros2 run rda_robot_bringup fruit_detector.py --ros-args -p use_sim_time:=true
  # perception_demo.launch.py 가 detect:=true(기본) 로 함께 띄운다.
"""
import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


def cloud_to_arrays(msg, rgb_order="bgr"):
    """organized PointCloud2 → (xyz[H,W,3] float32, rgb[H,W,3] uint8).

    필드 오프셋을 메시지에서 읽는다(플러그인마다 point_step·offset 이 다르다).
    무효점(NaN)은 그대로 두고 호출측에서 마스킹한다.

    ⚠ `rgb` 필드(float32 에 uint32 를 담는 관례)의 **바이트 순서는 발행자마다 다르다.**
      PCL 관례는 0x00RRGGBB(상위=R)지만 Gazebo Classic 의 `libgazebo_ros_camera` 는
      반대로 담는다(실측: 같은 장면에서 클라우드 평균색이 컬러 이미지의 정반대). 잘못
      맞추면 **빨강이 파랑으로 읽혀 검출 0개**가 된다 — 조용한 실패라 파라미터로 뺐다."""
    off = {f.name: f.offset for f in msg.fields}
    for need in ("x", "y", "z", "rgb"):
        if need not in off:
            return None, None
    h, w, ps = msg.height, msg.width, msg.point_step
    if h <= 1:            # unorganized 면 2D 연결성분을 못 쓴다
        return None, None
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, ps)
    xyz = np.stack([buf[:, :, off[k]:off[k] + 4].copy().view(np.float32)[:, :, 0]
                    for k in ("x", "y", "z")], axis=-1)
    packed = buf[:, :, off["rgb"]:off["rgb"] + 4].copy().view(np.uint32)[:, :, 0]
    hi, mid, lo = (packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF
    ch = (hi, mid, lo) if rgb_order == "rgb" else (lo, mid, hi)
    return xyz, np.stack(ch, axis=-1).astype(np.uint8)


class FruitDetector(Node):
    def __init__(self):
        super().__init__("fruit_detector")
        self.declare_parameter("cloud_topics",
                               ["/d435i/depth/points", "/d405/depth/points"])
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("output_topic", "detected_fruits")
        self.declare_parameter("period", 0.5)        # 처리 주기[s] (센서 15Hz 를 다 쓰지 않는다)
        self.declare_parameter("max_range", 3.0)     # 이보다 먼 점은 버림
        # rgb 필드 바이트 순서: Gazebo Classic=bgr, PCL/RealSense 관례=rgb.
        self.declare_parameter("rgb_order", "bgr")
        # 빨강 HSV 임계 — OpenCV H 는 0~179. 붉은색은 0 과 180 양끝에 걸쳐 두 구간을 쓴다.
        self.declare_parameter("hue_width", 12)      # 0±hue_width, 180∓hue_width
        self.declare_parameter("sat_min", 90)
        self.declare_parameter("val_min", 40)
        self.declare_parameter("min_points", 25)     # 이보다 작은 덩어리는 노이즈
        self.declare_parameter("cluster_gap", 0.03)  # 붙어 보이는 열매를 쪼개는 3D 간격[m]
        self.declare_parameter("radius_min", 0.012)
        self.declare_parameter("radius_max", 0.070)
        # ④-a 겹친 열매 분리(반경 사전지식 RANSAC)
        self.declare_parameter("split_by_prior", True)
        self.declare_parameter("sphere_prior", 0.035)   # 열매 반경 사전값[m] (대과 토마토)
        self.declare_parameter("ransac_tol", 0.012)     # 구면 인라이어 허용오차[m]
        self.declare_parameter("min_inliers", 25)       # 열매 하나로 인정할 최소 점수
        self.declare_parameter("max_per_cluster", 6)    # 한 덩어리에서 뽑을 최대 열매 수
        self.declare_parameter("merge_dist", 0.04)   # 같은 열매로 볼 world 거리[m]
        self.declare_parameter("min_hits", 2)        # 이만큼 관측돼야 발행(깜빡임 제거)
        self.declare_parameter("forget_sec", 10.0)   # 이 시간 못 보면 트랙 삭제

        gp = self.get_parameter
        self.world = gp("world_frame").value
        self.max_range = float(gp("max_range").value)

        import tf2_ros
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

        latched = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, gp("output_topic").value, latched)

        sensor_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.last_t = {}
        self.tracks = []          # [{p:np.array(3), r:float, hits:int, t:float}]
        self.next_id = 0
        for topic in gp("cloud_topics").value:
            self.create_subscription(PointCloud2, topic,
                                     lambda m, t=topic: self.on_cloud(m, t), sensor_qos)
            self.get_logger().info(f"클라우드 구독: {topic}")

    # ---------- ① 빨강 마스크 ----------
    def red_mask(self, rgb):
        import cv2
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        hw = int(self.get_parameter("hue_width").value)
        hue_ok = (h <= hw) | (h >= 180 - hw)
        return hue_ok & (s >= int(self.get_parameter("sat_min").value)) \
                      & (v >= int(self.get_parameter("val_min").value))

    # ---------- ③ 붙은 덩어리 3D 분리 ----------
    def split_3d(self, pts):
        """단일연결 군집화로 cluster_gap 보다 멀리 떨어진 부분을 나눈다.
        (2D 로는 앞뒤로 겹친 두 열매가 한 덩어리로 보인다.)"""
        gap = float(self.get_parameter("cluster_gap").value)
        if len(pts) < 8:
            return [pts]
        from scipy.cluster.hierarchy import fcluster, linkage
        sub = pts if len(pts) <= 400 else pts[np.random.choice(len(pts), 400, replace=False)]
        lab = fcluster(linkage(sub, method="single"), t=gap, criterion="distance")
        cents = np.array([sub[lab == k].mean(axis=0) for k in np.unique(lab)])
        if len(cents) == 1:
            return [pts]
        # 전체 점을 가장 가까운 군집 중심에 배정(서브샘플 라벨을 전 점으로 확장)
        d = np.linalg.norm(pts[:, None, :] - cents[None, :, :], axis=2)
        who = d.argmin(axis=1)
        return [pts[who == k] for k in range(len(cents))]

    # ---------- ④-a 화방 안에서 개별 열매 분리 (반경 사전지식 RANSAC) ----------
    def fit_spheres_prior(self, pts):
        """겹쳐 붙은 열매 덩어리에서 **열매 하나씩** 중심을 뽑는다. [(center, r), ...]

        왜 필요한가: 한 화방의 열매는 중심간격 6cm 인데 반경이 3.5cm — **서로 파고들어**
        한 덩어리로 보인다. 3D 간격 군집화(cluster_gap)로는 절대 안 쪼개진다. 덩어리
        무게중심을 집으면 실제 열매에서 3cm 어긋나 파지에 실패한다.

        방법: 구면 위의 점 p 에서 중심은 표면법선 방향으로 r 만큼 안쪽 —
        **보이는 앞면 중앙부에서는 법선 ≈ 시선방향** 이므로 `c = p + r·(p/|p|)` 가
        후보가 된다. 모든 점에서 후보를 만들어 인라이어(|‖q−c‖−r| < tol)가 가장 많은
        후보를 채택하고, 그 점들을 빼고 반복(greedy RANSAC)."""
        r = float(self.get_parameter("sphere_prior").value)
        tol = float(self.get_parameter("ransac_tol").value)
        need = int(self.get_parameter("min_inliers").value)
        kmax = int(self.get_parameter("max_per_cluster").value)
        out, rest = [], pts
        rng = np.random.default_rng(0)
        while len(rest) >= need and len(out) < kmax:
            u = rest / np.maximum(np.linalg.norm(rest, axis=1, keepdims=True), 1e-9)
            idx = (rng.choice(len(rest), 80, replace=False) if len(rest) > 80
                   else np.arange(len(rest)))
            cand = rest[idx] + r * u[idx]                       # 후보 중심들
            d = np.abs(np.linalg.norm(rest[None, :, :] - cand[:, None, :], axis=2) - r)
            score = (d < tol).sum(axis=1)
            b = int(score.argmax())
            if score[b] < need:
                break
            inl = d[b] < tol
            c = (rest[inl] + r * u[inl]).mean(axis=0)           # 인라이어로 1차 중심
            # ★ 반경 고정 구면 최소제곱(Gauss-Newton 5회): c ← mean(p − r·(p−c)/‖p−c‖).
            #   1차 추정은 '법선 ≈ 시선' 가정이라 캡 가장자리에서 어긋난다 → 실제 곡면에
            #   맞춰 다시 당긴다. 이 보정만으로 중심오차가 눈에 띄게 줄었다(2.4cm → 아래 검증).
            q = rest[inl]
            for _ in range(5):
                v = q - c
                nv = np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-9)
                c = (q - r * v / nv).mean(axis=0)
            rr = float(np.median(np.linalg.norm(q - c, axis=1)))
            out.append((c, rr))
            rest = rest[~inl]
        return out

    # ---------- ④ 중심·반경 ----------
    def fit_sphere(self, pts):
        """보이는 앞면(반구)만으로 중심·반경 추정. 반환 (center, radius) 또는 None."""
        c = pts.mean(axis=0)
        n = np.linalg.norm(c)
        if n < 1e-6:
            return None
        u = c / n                                   # 카메라 원점 → 열매 시선방향
        d = pts - c
        lat = np.linalg.norm(d - np.outer(d @ u, u), axis=1)   # 시선축에 수직인 거리
        r = float(np.percentile(lat, 95))                       # 이상점에 둔감하게
        rmin = float(self.get_parameter("radius_min").value)
        rmax = float(self.get_parameter("radius_max").value)
        if not (rmin <= r <= rmax):
            return None
        # ★ 앞면만 보이므로 무게중심은 중심보다 카메라 쪽 — 반구 껍질 기준 r/2 뒤로 민다.
        return c + u * (r * 0.5), r

    # ---------- 메인 ----------
    def on_cloud(self, msg, topic):
        now = self.get_clock().now().nanoseconds * 1e-9
        period = float(self.get_parameter("period").value)
        if now - self.last_t.get(topic, 0.0) < period:
            return
        self.last_t[topic] = now

        xyz, rgb = cloud_to_arrays(msg, str(self.get_parameter("rgb_order").value).lower())
        if xyz is None:
            self.get_logger().warn(f"{topic}: organized xyz+rgb 클라우드가 아님 — 건너뜀",
                                   throttle_duration_sec=10.0)
            return

        finite = np.isfinite(xyz).all(axis=2)
        # 무효점을 0 으로 눕히고 거리 계산(NaN/inf 를 큰 수로 바꾸면 float32 오버플로 경고).
        safe = np.where(finite[..., None], xyz, 0.0).astype(np.float64)
        valid = finite & (np.linalg.norm(safe, axis=2) < self.max_range)
        mask = self.red_mask(rgb) & valid
        if not mask.any():
            self._publish(now)
            return

        from scipy import ndimage
        lab, n = ndimage.label(mask)
        minpts = int(self.get_parameter("min_points").value)
        found = []
        for k in range(1, n + 1):
            sel = lab == k
            if sel.sum() < minpts:
                continue
            prior = bool(self.get_parameter("split_by_prior").value)
            for part in self.split_3d(xyz[sel]):
                if len(part) < minpts:
                    continue
                if prior:
                    # 겹친 열매를 하나씩 분리(권장). 하나도 못 뽑으면 덩어리 통째로 폴백.
                    got = self.fit_spheres_prior(part)
                    if got:
                        found.extend(got)
                        continue
                fit = self.fit_sphere(part)
                if fit:
                    found.append(fit)
        if not found:
            self._publish(now)
            return

        # ---------- ⑤ world 변환 + 추적 병합 ----------
        try:
            tf = self.tfbuf.lookup_transform(self.world, msg.header.frame_id,
                                             rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF {self.world}←{msg.header.frame_id} 실패: {e}",
                                   throttle_duration_sec=5.0)
            return
        R, t = self._tf_mat(tf)
        merge = float(self.get_parameter("merge_dist").value)
        for c, r in found:
            p = R @ c + t
            hit = None
            for tr in self.tracks:
                if np.linalg.norm(tr["p"] - p) < merge:
                    hit = tr
                    break
            if hit is None:
                self.tracks.append({"id": self.next_id, "p": p, "r": r, "hits": 1, "t": now})
                self.next_id += 1
            else:                                   # EMA — 여러 시점/카메라 관측을 융합
                a = 0.4
                hit["p"] = (1 - a) * hit["p"] + a * p
                hit["r"] = (1 - a) * hit["r"] + a * r
                hit["hits"] += 1
                hit["t"] = now
        self._publish(now)

    @staticmethod
    def _tf_mat(tf):
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        tr = tf.transform.translation
        return R, np.array([tr.x, tr.y, tr.z])

    def _publish(self, now):
        forget = float(self.get_parameter("forget_sec").value)
        self.tracks = [t for t in self.tracks if now - t["t"] <= forget]
        minh = int(self.get_parameter("min_hits").value)
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        shown = [t for t in self.tracks if t["hits"] >= minh]
        for tr in shown:
            m = Marker()
            m.header.frame_id = self.world
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "fruit"
            m.id = int(tr["id"])
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = [float(v) for v in tr["p"]]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = float(2 * tr["r"])   # 지름
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.25, 0.1, 0.95
            arr.markers.append(m)
        self.pub.publish(arr)
        if shown:
            self.get_logger().info(f"인지 열매 {len(shown)}개 발행 "
                                   f"(추적 {len(self.tracks)}개)", throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = FruitDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
