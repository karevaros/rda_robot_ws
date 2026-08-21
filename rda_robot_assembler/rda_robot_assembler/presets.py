"""로봇 구성 프리셋 해석 — `presets.yaml` 의 이름 하나로 mounts·srdf·scene 을 함께 고른다.

왜 한 곳에 모으나: 구성이 바뀌면 **셋이 같이** 바뀌어야 한다. 특히 SRDF/ACM 은 링크
이름·형상에 묶여 있어서, 스탠드처럼 링크가 늘어난 구성에 정본 SRDF 를 그대로 쓰면
새 링크의 자충돌 쌍이 없어 **계획이 조용히 막힌다**. launch 마다 세 인자를 손으로
맞추게 두면 언젠가 하나를 빠뜨린다 → 이름 하나로 묶어 그 실수를 없앤다.

`paths.models_dir()` 과 같은 원칙으로 **소스 트리**를 정본으로 본다.
"""
import os

import yaml

from . import paths

_PRESET_FILE = "presets.yaml"


def config_dir():
    """`rda_robot_description/config` 절대경로 (models 폴더의 부모)."""
    return os.path.dirname(paths.models_dir())


def preset_path():
    return os.path.join(config_dir(), _PRESET_FILE)


def _load():
    p = preset_path()
    if not os.path.exists(p):
        return {"default": "base", "presets": {}}
    return yaml.safe_load(open(p)) or {"default": "base", "presets": {}}


def names():
    """등록된 프리셋 이름 목록."""
    return list((_load().get("presets") or {}).keys())


def default_name():
    return str(_load().get("default") or "base")


def _srdf_dir():
    """SRDF 는 moveit_config 패키지에 있다. config_dir 에서 형제 패키지를 찾아간다."""
    src = os.path.dirname(os.path.dirname(config_dir()))     # …/src
    return os.path.join(src, "rda_robot_moveit_config", "config")


def resolve(name=None):
    """프리셋 이름 → dict(name, label, mounts, srdf, scene, note) — 경로는 **절대경로**.

    이름이 없거나 빈 값이면 `default` 프리셋. 등록되지 않은 이름이면 KeyError.
    파일이 실제로 있는지도 확인한다 — 없는 경로를 조용히 넘기면 launch 가
    엉뚱한 폴백으로 뜬다."""
    d = _load()
    presets = d.get("presets") or {}
    nm = str(name or d.get("default") or "base").strip()
    if nm not in presets:
        raise KeyError(
            f"알 수 없는 로봇 구성 프리셋 '{nm}'. 가능: {sorted(presets)} "
            f"({preset_path()})")
    e = dict(presets[nm])
    cfg, sd = config_dir(), _srdf_dir()
    out = {"name": nm, "label": e.get("label", nm), "note": (e.get("note") or "").strip()}
    for key, base in (("mounts", cfg), ("scene", cfg), ("srdf", sd)):
        v = e.get(key)
        if not v:
            out[key] = None
            continue
        p = v if os.path.isabs(v) else os.path.join(base, v)
        if not os.path.exists(p):
            raise FileNotFoundError(
                f"프리셋 '{nm}' 의 {key} 파일이 없습니다: {p}\n"
                f"  {preset_path()} 를 확인하세요.")
        out[key] = p
    return out


def describe(name=None):
    e = resolve(name)
    return (f"로봇 구성 = {e['name']} ({e['label']})\n"
            f"  mounts : {e['mounts']}\n"
            f"  srdf   : {e['srdf']}\n"
            f"  scene  : {e['scene']}"
            + (f"\n  note   : {e['note'].splitlines()[0]}" if e["note"] else ""))
