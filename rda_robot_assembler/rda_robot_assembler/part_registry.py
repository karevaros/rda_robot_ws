"""파트 슬롯 & 모델 레지스트리.

슬롯: base / arm / endeffector / sensor1 / sensor2
각 모델은 xacro 소스(패키지 상대경로) + xacro args + anchor(부모에 붙는 프레임)로 정의.
anchor 가 root 와 다르면(예: RG2 는 world 루트지만 rg2_hand 로 붙음) 로더가 오프셋 보정.
"""

# 슬롯 순서(왼쪽 탭 순서). base 는 루트(부모 없음).
SLOTS = ["base", "arm", "endeffector", "sensor1", "sensor2"]

SLOT_LABELS = {
    "base": "① 베이스(모바일)",
    "arm": "② 로봇팔",
    "endeffector": "③ 엔드이펙터",
    "sensor1": "④ 센서 1",
    "sensor2": "⑤ 센서 2",
}

# 모델 레지스트리: id -> dict
#   pkg          : ament 패키지명 (mesh package:// 및 xacro 위치 기준)
#   xacro        : 패키지 share 기준 상대 경로
#   args         : xacro 인자 dict
#   anchor       : 부모에 부착되는 프레임(=mount joint 의 child). None 이면 root 사용.
#   macro        : 통합 xacro 출력 시 사용할 매크로 정보(내보내기용). None 이면 include 방식.
MODELS = {
    "scout_v2": {
        "label": "Agilex Scout 2.0",
        "pkg": "scout_description",
        "xacro": "urdf/scout_v2.xacro",
        "args": {},
        "anchor": None,  # base_link
    },
    "rb5_850e": {
        "label": "Rainbow RB5-850e",
        "pkg": "rbpodo_description",
        "xacro": "robots/rb5_850e.urdf.xacro",
        "args": {},
        "anchor": None,  # link0
    },
    "onrobot_rg2": {
        "label": "OnRobot RG2",
        "pkg": "rda_robot_description",
        "xacro": "urdf/parts/endeffector/onrobot_rg2_standalone.urdf.xacro",
        "args": {},
        "anchor": "rg2_hand",  # world 루트지만 rg2_hand 로 부착
    },
    "d405": {
        "label": "RealSense D405 (eye-in-hand)",
        "pkg": "realsense2_description",
        "xacro": "urdf/test_d405_camera.urdf.xacro",
        "args": {"use_nominal_extrinsics": "true"},
        "anchor": None,  # base_link
    },
    "d435i": {
        "label": "RealSense D435i (eye-to-hand)",
        "pkg": "realsense2_description",
        "xacro": "urdf/test_d435i_camera.urdf.xacro",
        "args": {"use_nominal_extrinsics": "true"},
        "anchor": None,  # base_link
    },
}

# 슬롯별 선택 가능한 모델 후보(첫 번째가 기본값).
SLOT_MODELS = {
    "base": ["scout_v2"],
    "arm": ["rb5_850e"],
    "endeffector": ["onrobot_rg2"],
    "sensor1": ["d405", "d435i"],
    "sensor2": ["d435i", "d405"],
}

# 통합 xacro 내보내기: 슬롯 -> 매크로 호출 정보.
# (rda_robot.urdf.xacro 가 mounts.yaml 을 읽어 parent/xyz/rpy 를 채운다)
EXPORT_MACRO = {
    "arm": {
        "kind": "rb_6dof",
        "include": "$(find rbpodo_description)/robots/rb_6dof.xacro",
    },
    "endeffector": {
        "kind": "onrobot_rg2",
        "include": "$(find rda_robot_description)/urdf/parts/endeffector/onrobot_rg2_macro.xacro",
    },
    "sensor1": {
        "kind": "sensor_d405",
        "include": "$(find realsense2_description)/urdf/_d405.urdf.xacro",
    },
    "sensor2": {
        "kind": "sensor_d435i",
        "include": "$(find realsense2_description)/urdf/_d435i.urdf.xacro",
    },
}


def default_model(slot):
    return SLOT_MODELS[slot][0]
