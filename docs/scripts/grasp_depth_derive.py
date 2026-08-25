#!/usr/bin/env python3
"""파지 깊이(grasp_offset) 유도 — 보고서 수치의 재현 스크립트.

묻는 것: **열매를 그리퍼의 어디로 물어야 하는가**. 종전에는 손가락이 접근축으로 차지하는
구간의 비율(`grasp_depth`, 기본 0.33)로 정했는데, 그 비율은 그리퍼가 바뀌면 같은 값이
다른 곳을 뜻한다. 2026-08-21 사용자가 화면을 보고 지적해 mesh 실측으로 바꾼 것을,
데모를 띄우지 않고 되짚을 수 있게 뽑아 둔다.

세 값을 낸다.
  ① `gripper_span`      — 손가락이 접근축으로 차지하는 전체 구간 (뿌리 구동부 ~ 날 끝)
  ② `gripper_pad_center`— 손 앞으로 나온 구간의 **파지면 면적중심** · palm 앞면 깊이
  ③ 채택 파지거리       — `goff = max(② , palm앞면 + 열매반지름 + 여유)`

🔴 왜 ③ 의 하한이 필요한가: ② 만 맞추면 파지면 중심은 맞지만 **손바닥이 열매를 파고든다**
(RG2 실측 1.0cm). 반대로 종전 0.33(13.1cm)은 열매를 **집게 구동부**에 박는 값이었다.

사용: python3 docs/scripts/grasp_depth_derive.py [urdf경로]
      (생략 시 정본 mounts 로 즉석 합성 — `ros2 run rda_robot_assembler compose_urdf`)
"""
import importlib.util
import os
import subprocess
import sys
import tempfile

SRC = os.path.expanduser("~/robot_ws/src")
APPROACH_AXIS = [0.0, -1.0, 0.0]     # tcp 기준 접근축(RB5+RG2 실측)
FRUIT_R = 0.035                      # obstacles.yaml crops.template.truss.fruit_r
PALM_CLEARANCE = 0.005               # pregrasp_demo 파라미터 palm_clearance 기본값
LEGACY_DEPTH = 0.33                  # 종전 grasp_depth 상수(비교용)


def _load(path, name):
    s = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(s)
    sys.modules[name] = m
    s.loader.exec_module(m)
    return m


def _resolve(uri):
    """package:// → 소스 트리 경로(설치본이 아니라 정본을 본다)."""
    if not uri:
        return None
    p = uri.replace("package://rda_robot_description",
                    os.path.join(SRC, "rda_robot_description"))
    return p if os.path.exists(p) else None


def main():
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else None
    if urdf_path is None:
        fd, urdf_path = tempfile.mkstemp(suffix=".urdf")
        os.close(fd)
        subprocess.run(["ros2", "run", "rda_robot_assembler", "compose_urdf",
                        "-o", urdf_path], check=True, stderr=subprocess.DEVNULL)
    urdf = open(urdf_path).read()
    srdf = open(os.path.join(
        SRC, "rda_robot_moveit_config/config/rda_robot.srdf")).read()
    RI = _load(os.path.join(SRC, "rda_robot_bringup/scripts/robot_introspect.py"), "ri")

    # 🔴 mesh_resolver 를 빼면 링크 **원점만** 써서 구간이 10.50~10.50cm 로 무너진다
    #    (에러 없이 그럴듯한 값이 나온다 — 2026-08-25 이 스크립트를 쓰며 실제로 밟았다).
    span = RI.gripper_span(urdf, srdf, "tcp", APPROACH_AXIS, mesh_resolver=_resolve)
    ca = RI.gripper_closing_axis(urdf, srdf, "tcp", APPROACH_AXIS)
    pad = RI.gripper_pad_center(urdf, srdf, "tcp", APPROACH_AXIS,
                                ca or [1.0, 0.0, 0.0], mesh_resolver=_resolve)
    if span is None or pad is None:
        print("유도 실패 — mesh 를 못 읽었다(collision 없는 URDF?)")
        return 1
    pc, palm = float(pad[0]), float(pad[1])
    need = palm + FRUIT_R + PALM_CLEARANCE
    goff = max(pc, need)
    legacy = span[0] + LEGACY_DEPTH * (span[1] - span[0])
    frac = (goff - span[0]) / max(1e-9, span[1] - span[0])

    print(f"닫힘축(tcp 로컬)        : {tuple(round(float(v), 3) for v in (ca or []))}")
    print(f"① 손가락 패드 구간      : {span[0]*100:.2f} ~ {span[1]*100:.2f} cm")
    print(f"② 파지면 면적중심       : {pc*100:.2f} cm   · palm 앞면 {palm*100:.2f} cm")
    print(f"③ 손바닥 하한           : palm {palm*100:.2f} + 열매 {FRUIT_R*100:.1f}"
          f" + 여유 {PALM_CLEARANCE*100:.1f} = {need*100:.2f} cm")
    print(f"⇒ 채택 파지거리 goff    : {goff*100:.2f} cm"
          f"  (패드 구간의 {frac:.3f} 지점"
          f" · {'손바닥 하한이 지배' if goff > pc + 1e-9 else '면적중심이 지배'})")
    print()
    print(f"[대조] 종전 grasp_depth={LEGACY_DEPTH} ⇒ {legacy*100:.2f} cm — 열매 앞면이"
          f" {(legacy - FRUIT_R)*100:.2f} cm 에 놓인다"
          f" ⇒ 집게 구동부(패드 시작 {span[0]*100:.2f} cm) 안쪽 = 박힌다")
    print(f"[대조] 노출구간 면적중심만 맞추면 {pc*100:.2f} cm ⇒ 손바닥 앞면 {palm*100:.2f} cm 이"
          f" 열매 앞면 {(pc - FRUIT_R)*100:.2f} cm 를 {(palm - (pc - FRUIT_R))*100:.2f} cm 파고든다")
    print("        ⚠ 기록의 '1.0cm 침투' 는 **패드 전체** 면적중심(15.23cm) 기준이다 —"
          " 위 값은 **손 앞 노출구간** 중심(pad_center 반환값) 기준이라 서로 다른 수치다.")
    print(f"[차이] 채택값은 종전보다 {(goff - legacy)*100:.2f} cm 덜 접근한다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
