#!/usr/bin/env python3
"""obstacles.yaml (+expand_crops) → Gazebo Classic SDF 월드 (5주차 perception · Stage 2)

온실 구조(거터·레일)와 작물(줄기·화방대·열매)을 **Gazebo 모델(visual+collision)** 로
세워, D435i 카메라가 실제로 그것을 보고 포인트클라우드를 얻게 한다. 좌표·형상은
obstacles.yaml + obstacle_publisher.expand_crops() 를 그대로 재사용 → RViz/MoveIt 장면과
Gazebo 장면이 **단일 진실원으로 자동 정합**.

  · box(거터) · cylinder(레일·줄기·화방대) · sphere(열매) 를 SDF geometry 로 변환.
  · 모두 static(고정). 색상 = obstacles.yaml 의 rgba.
  · kind:keepout(가상 벽/ground_plane)은 제외 — 실체 없음(+ Gazebo 기본 ground plane 사용).
  · **열매(kind:target)도 포함** — 카메라가 봐야 인지(Stage 4)가 가능하므로.

사용: ros2 run rda_robot_bringup gen_gazebo_world.py [obstacles.yaml] > greenhouse.world
"""
import os
import sys

import yaml

# 형제 스크립트 expand_crops 재사용
_here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _here)
from obstacle_publisher import expand_crops  # noqa: E402


def _geometry_sdf(o):
    t = o["type"]
    if t == "box":
        sx, sy, sz = (float(v) for v in o["size"])
        return f"<box><size>{sx} {sy} {sz}</size></box>"
    if t == "cylinder":
        return (f"<cylinder><radius>{float(o['radius'])}</radius>"
                f"<length>{float(o['height'])}</length></cylinder>")
    if t == "sphere":
        return f"<sphere><radius>{float(o['radius'])}</radius></sphere>"
    raise ValueError(f"알 수 없는 type: {t}")


def _model_sdf(o):
    pose = o.get("pose") or {}
    x, y, z = (float(v) for v in pose.get("xyz", [0, 0, 0]))
    r, p, yaw = (float(v) for v in pose.get("rpy", [0, 0, 0]))
    col = o.get("color", [0.6, 0.6, 0.6, 1.0])
    cr, cg, cb = col[0], col[1], col[2]
    ca = col[3] if len(col) > 3 else 1.0
    geom = _geometry_sdf(o)
    name = o["name"]
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} {r} {p} {yaw}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material>
            <ambient>{cr} {cg} {cb} {ca}</ambient>
            <diffuse>{cr} {cg} {cb} {ca}</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>{geom}</geometry>
        </collision>
      </link>
    </model>"""


def build_world(obstacles_yaml):
    spec = yaml.safe_load(open(obstacles_yaml)) or {}
    try:
        expand_crops(spec)
    except Exception as e:      # noqa: BLE001
        sys.stderr.write(f"[gen_gazebo_world] expand_crops 경고: {e}\n")
    models = []
    for o in spec.get("obstacles", []):
        if o.get("kind") == "keepout":       # 가상 벽/바닥 — 실체 없음
            continue
        if o.get("type") not in ("box", "cylinder", "sphere"):
            continue
        models.append(_model_sdf(o))
    body = "".join(models)
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="greenhouse">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.8 0.9 1</background></scene>
{body}
  </world>
</sdf>
"""


def main():
    default = os.path.join(
        _here, "..", "..", "rda_robot_description", "config", "obstacles.yaml")
    path = sys.argv[1] if len(sys.argv) > 1 else default
    if not os.path.exists(path):
        # 설치 트리 폴백
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("rda_robot_description"),
                            "config", "obstacles.yaml")
    sys.stdout.write(build_world(path))


if __name__ == "__main__":
    main()
