"""RealSense SDK 단독 점검(ROS 없이): 깊이·컬러 30프레임을 받아 fps·깊이 유효율·범위를 보고 PNG 를 남긴다.

    python3 realsense_sdk_check.py      # 현재 폴더에 d405_color.png · d405_depth.png

2026-10-29 실측(D405 · 시리얼 352122273557 · FW 5.15.1.55 · USB 3.2): 29.0 fps · 깊이 유효 87% · 0.11~6.55 m.
⚠ ROS 토픽 속도를 rclpy 로 이미지째 구독해 재면 4~14 Hz 로 나온다 — 측정자가 병목이다. 발행 속도는
  camera_info(작은 메시지)로 잴 것(같은 조건 29.5~30.5 Hz).
"""
import time, numpy as np, pyrealsense2 as rs
p = rs.pipeline(); c = rs.config()
c.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
c.enable_stream(rs.stream.color, 848, 480, rs.format.rgb8, 30)
prof = p.start(c)
dev = prof.get_device(); print("장치:", dev.get_info(rs.camera_info.name), "· USB", dev.get_info(rs.camera_info.usb_type_descriptor))
scale = dev.first_depth_sensor().get_depth_scale()
for _ in range(15): p.wait_for_frames()   # 자동노출 안정화
t = time.time(); n = 0
for _ in range(30):
    f = p.wait_for_frames(); n += 1
d = np.asanyarray(f.get_depth_frame().get_data()) * scale
col = np.asanyarray(f.get_color_frame().get_data())
dt = time.time() - t
valid = d[d > 0]
print("프레임 %d개 %.1f fps · 깊이 유효 %.0f%% · 중앙 %.3f m · 범위 %.3f~%.3f m" % (n, n/dt, 100*valid.size/d.size, np.median(valid) if valid.size else 0, valid.min() if valid.size else 0, valid.max() if valid.size else 0))
print("컬러 %dx%d · 평균 밝기 %.0f" % (col.shape[1], col.shape[0], col.mean()))
import PIL.Image as I
I.fromarray(col).save("d405_color.png")
dv = np.clip(d / 0.5, 0, 1); I.fromarray((255 * (1 - dv) * (d > 0)).astype(np.uint8)).save("d405_depth.png")
p.stop()
