"""손목 카메라 TF 정합 점검: /tf_static 발행자·sensor1 연결 중복·base_link→광학 프레임·포인트클라우드를 로봇 기준으로 옮긴 높이 분포.

    ros2 launch rda_robot_description real_robot.launch.py use_fake_hardware:=true gripper:=false camera:=true
    python3 camera_tf_check.py

2026-10-29 실측(D405 · fake): 발행자 robot_state_publisher 만 · 연결 11·중복 0 · base_link→color_optical (0.238,0.009,1.263) ·
클라우드 frame_id sensor1_camera_depth_optical_frame · 높이 5/50/95% 0.42/1.48/2.87 m(사무실).
"""
import time, numpy as np, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from collections import Counter
rclpy.init(); n = rclpy.create_node("camtf")
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
edges = Counter(); pubs = {}
def cb(m):
    for t in m.transforms:
        if "sensor1" in t.child_frame_id: edges[(t.header.frame_id, t.child_frame_id)] += 1
n.create_subscription(TFMessage, "/tf_static", cb, QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
cloud = [None]
n.create_subscription(PointCloud2, "/camera/sensor1_camera/depth/color/points", lambda m: cloud.__setitem__(0, m), qos_profile_sensor_data)
t = time.time()
while time.time() - t < 6 and (cloud[0] is None or time.time() - t < 3): rclpy.spin_once(n, timeout_sec=0.05)
print("/tf_static 발행자:", [ (i.node_name) for i in n.get_publishers_info_by_topic("/tf_static")])
dup = {k: v for k, v in edges.items() if v > 1}
print("sensor1 연결 %d개 · 중복 %d개" % (len(edges), len(dup)), dup if dup else "")
tr = buf.lookup_transform("base_link", "sensor1_camera_color_optical_frame", rclpy.time.Time(), rclpy.duration.Duration(seconds=2)).transform.translation
print("base_link → color_optical: (%.3f, %.3f, %.3f) m" % (tr.x, tr.y, tr.z))
m = cloud[0]; print("클라우드 frame_id:", m.header.frame_id if m else None)
pts = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(m, field_names=("x","y","z"), skip_nans=True)][::50])
T = buf.lookup_transform("world" if buf.can_transform("world", m.header.frame_id, rclpy.time.Time()) else "base_link", m.header.frame_id, rclpy.time.Time())
q = T.transform.rotation; tt = T.transform.translation
x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
W = pts @ R.T + np.array([tt.x, tt.y, tt.z])
print("기준 %s · 점 %d개(1/50) · z 분위수 5/50/95%%: %.2f / %.2f / %.2f m · 카메라→점 거리 중앙 %.2f m" % (T.header.frame_id, len(W), *np.percentile(W[:,2], [5,50,95]), np.median(np.linalg.norm(pts, axis=1))))
