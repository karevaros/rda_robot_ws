"""통합 URDF Python 컴포저.

mounts.yaml + 모델 정의(part_registry) 를 읽어 슬롯별 URDF 를 조립해 단일 URDF 를 만든다.
`rda_robot.urdf.xacro` 의 슬롯별 `xacro:if` 분기를 대체한다 — 분기 없는 모델(26종 중 21종)이
조용히 누락되던 문제의 근본 해결.

■ 왜 xacro 가 아니라 파이썬인가 (2026-07-15 타당성 실측)
  1. 표준 URDF 모델(UR·Robotiq 등)은 자체 `world` 루트 + `world→base_link` 조인트를 만든다.
     여기에 마운트 조인트를 더하면 루트가 둘(`Two root links found`) → URDF 규격 위반.
     **xacro 안에서는 이미 만들어진 링크/조인트를 지울 수단이 없다.** ← 결정타
  2. xacro 표현식은 화이트리스트라 `os`/`__import__` 로 경로를 해석할 수 없다.
  3. `xacro:include` 는 모델별 args 를 못 넘긴다($(arg) 는 전역).
  파이썬은 셋 다 우회 가능하고, 조립기와 **같은 모델 정의**를 그대로 공유한다.

■ 조립 절차
  1. mounts.yaml → 슬롯별 모델 id + mount(parent_slot·parent_frame·xyz·rpy)
  2. 모델별 `xacro_to_urdf()` 실행 → URDF XML (조립기와 동일 경로·동일 args)
  3. **re-root**: anchor 위쪽 조상(world 등)을 제거해 anchor 를 루트로 만든다
  4. **prefix**: 모델 yaml 의 `prefix:` 로 링크/조인트/머티리얼 이름 충돌 회피
  5. 병합 + 슬롯별 mount 조인트(parent=부모 슬롯의 parent_frame, child=anchor)
  6. 이름 충돌·루트 다중 검사 → 문제가 있으면 **조용히 넘어가지 않고 즉시 실패**

■ 링크명 계약(하위 산출물이 의존 — 바꾸지 말 것)
  base_link(베이스) · link0·tcp(팔) · rg2_hand(그리퍼)
  → 이 모델들은 prefix 없이 조립한다. gen_srdf.py·kinematics.py·moveit_demo.launch.py 참조.
  센서 링크명은 prefix 가 붙어 기존 xacro 판(d405_link 등)과 다르다. 코드 의존 없음(문서만).
"""
import argparse
import copy
import os
import sys
import xml.etree.ElementTree as ET

import yaml

from rda_robot_assembler import part_registry as reg
from rda_robot_assembler import urdf_loader as ul


class ComposeError(RuntimeError):
    """조립 실패. 메시지에 원인·조치를 담아 사용자에게 그대로 보인다."""


# ---------------------------------------------------------------- 모델 해석

def all_models():
    """내장 + 폴더 드롭 모델 전체. 조립기와 같은 정의·같은 폴더(paths.models_dir)."""
    ext, _ = reg.load_external_models()
    m = dict(reg.BUILTIN_MODELS)
    m.update(ext)
    return m


def _model_or_fail(model_id, slot, models):
    if model_id not in models:
        known = sorted(k for k in models if k.startswith(f"{slot}__")) or \
            reg.BUILTIN_SLOT_MODELS.get(slot, [])
        raise ComposeError(
            f"{slot} 슬롯: 모델 '{model_id}' 를 찾을 수 없습니다.\n"
            f"  이 슬롯에서 쓸 수 있는 모델: {', '.join(known) or '(없음)'}\n"
            f"  모델 폴더: {reg.models_dir()}\n"
            "  조치: mounts.yaml 의 모델 id 를 확인하거나, 모델 파일을 위 폴더에 넣으세요.")
    return models[model_id]


# ---------------------------------------------------------------- URDF 유틸

def _links(robot):
    return {l.get("name"): l for l in robot.findall("link")}


def _joints(robot):
    return robot.findall("joint")


def _child_to_joint(robot):
    out = {}
    for j in _joints(robot):
        c = j.find("child")
        if c is not None and c.get("link"):
            out[c.get("link")] = j
    return out


def _parent_to_children(robot):
    out = {}
    for j in _joints(robot):
        p, c = j.find("parent"), j.find("child")
        if p is not None and c is not None:
            out.setdefault(p.get("link"), []).append(c.get("link"))
    return out


def find_root(robot, what):
    """부모가 없는 링크 = 루트. 정확히 1개여야 한다."""
    links = set(_links(robot))
    children = set(_child_to_joint(robot))
    roots = sorted(links - children)
    if len(roots) != 1:
        raise ComposeError(
            f"{what}: 루트 링크가 {len(roots)}개입니다 ({', '.join(roots) or '없음'}).\n"
            "  URDF 는 루트가 정확히 1개여야 합니다. 모델 URDF 자체를 확인하세요.")
    return roots[0]


def reroot_at_anchor(robot, anchor, what):
    """anchor 의 조상(루트 쪽) 링크·조인트를 제거해 anchor 를 루트로 만든다.

    표준 URDF 모델이 자체적으로 만드는 `world` 루트를 걷어내는 단계.
    조립기의 `T_anchor_root` 오프셋 보정과 같은 의미다 — anchor 를 부착점에 정렬하므로
    조상 조인트의 변환은 버린다.

    안전 조건(어기면 조용히 형상이 바뀌므로 즉시 실패):
      · 조상 조인트가 전부 fixed 여야 한다(가동관절이면 DOF 손실)
      · 조상 링크에 anchor 계통 말고 다른 자식이 없어야 한다(서브트리 유실)
    """
    links = _links(robot)
    if anchor not in links:
        raise ComposeError(
            f"{what}: anchor '{anchor}' 링크가 모델 URDF 에 없습니다.\n"
            f"  이 모델의 링크: {', '.join(sorted(links)[:12])}"
            f"{' ...' if len(links) > 12 else ''}\n"
            "  조치: 모델 yaml 의 anchor 값을 실제 링크명으로 고치세요.")

    c2j = _child_to_joint(robot)
    p2c = _parent_to_children(robot)

    chain_links, chain_joints = [], []
    cur = anchor
    while cur in c2j:
        j = c2j[cur]
        parent = j.find("parent").get("link")
        if j.get("type") != "fixed":
            raise ComposeError(
                f"{what}: anchor '{anchor}' 위쪽 조인트 '{j.get('name')}' 가 "
                f"fixed 가 아닙니다(type={j.get('type')}).\n"
                "  이 조인트를 제거하면 관절 자유도가 사라집니다.\n"
                "  조치: anchor 를 이 조인트보다 위(루트 쪽) 링크로 지정하세요.")
        siblings = [c for c in p2c.get(parent, []) if c != cur]
        if siblings:
            raise ComposeError(
                f"{what}: anchor '{anchor}' 의 조상 링크 '{parent}' 에 다른 자식이 "
                f"있습니다({', '.join(siblings)}).\n"
                "  조상을 제거하면 그 서브트리가 유실됩니다.\n"
                "  조치: anchor 를 루트로 지정하거나 모델 URDF 를 확인하세요.")
        chain_joints.append(j)
        chain_links.append(parent)
        cur = parent

    for j in chain_joints:
        robot.remove(j)
    for ln in chain_links:
        robot.remove(links[ln])
    return chain_links


# ---------------------------------------------------------------- prefix

def _rename_refs(robot, ren):
    """ren: {old: new} 로 링크/조인트/머티리얼 이름과 그 참조를 전부 갱신."""
    for el in robot.iter():
        tag = el.tag
        if tag in ("link", "joint", "material") and el.get("name") in ren:
            el.set("name", ren[el.get("name")])
        if tag in ("parent", "child") and el.get("link") in ren:
            el.set("link", ren[el.get("link")])
        if tag == "mimic" and el.get("joint") in ren:
            el.set("joint", ren[el.get("joint")])
        if tag == "gazebo" and el.get("reference") in ren:
            el.set("reference", ren[el.get("reference")])


DEFAULT_PREFIX = "{slot}_"


def resolve_prefix(model, slot):
    """모델의 prefix 결정. 기본은 슬롯 기반 자동 prefix.

    기본값을 '{slot}_' 로 둔 이유: 서로 다른 모델이 같은 링크명(base_link 등)을 쓰는 게
    흔해서, 기본이 '무 prefix' 면 조합할 때마다 충돌로 실패한다. 슬롯 기반이면 같은 모델을
    두 슬롯에 써도(sensor1·sensor2 둘 다 d405) 안전하다.

    prefix 를 명시하면(빈 문자열 포함) 그 값을 그대로 쓴다 — 링크명 계약이 걸린 내장
    모델(scout_v2·rb5_850e·onrobot_rg2)이 `prefix: ""` 로 base_link·link0·tcp·rg2_hand 를
    보존한다. 값 안의 `{slot}` 은 슬롯명으로 치환된다.
    """
    p = model.get("prefix", DEFAULT_PREFIX)
    if p is None:
        p = ""
    return p.format(slot=slot)


def apply_prefix(robot, prefix):
    """링크·조인트·머티리얼 이름에 prefix 를 붙이고 모든 참조를 갱신."""
    if not prefix:
        return
    ren = {}
    for tag in ("link", "joint", "material"):
        for el in robot.findall(tag):
            n = el.get("name")
            if n and not n.startswith(prefix):
                ren[n] = prefix + n
    # <material> 은 link/visual 안에도 참조로 나타난다(이름만 있는 형태)
    _rename_refs(robot, ren)


# ---------------------------------------------------------------- 조립

def _origin_el(xyz, rpy):
    e = ET.Element("origin")
    e.set("xyz", " ".join(str(float(v)) for v in xyz))
    e.set("rpy", " ".join(str(float(v)) for v in rpy))
    return e


def _prefixed(slot_prefix, name):
    return (slot_prefix + name) if slot_prefix else name


def compose(mounts_path, robot_name="rda_robot"):
    """mounts.yaml 경로 → 통합 URDF XML 문자열."""
    with open(mounts_path) as f:
        y = yaml.safe_load(f) or {}
    sel = y.get("models") or {}
    mounts = y.get("mounts") or {}
    models = all_models()

    # 슬롯별 모델 id: models.<slot> 우선, 없으면 mounts.<slot>.model
    chosen = {}
    for slot in reg.SLOTS:
        mid = sel.get(slot) or (mounts.get(slot) or {}).get("model")
        if mid:
            chosen[slot] = mid
    if "base" not in chosen:
        raise ComposeError(
            "base 슬롯 모델이 지정되지 않았습니다.\n"
            f"  {mounts_path} 의 models.base 를 설정하세요.")

    # 1) 슬롯별 URDF 생성 + re-root + prefix
    parts = {}      # slot -> dict(root, anchor, prefix, robot)
    for slot in reg.SLOTS:
        if slot not in chosen:
            continue
        mid = chosen[slot]
        model = _model_or_fail(mid, slot, models)
        what = f"{slot} 슬롯 모델 '{mid}'"
        try:
            urdf_path = ul.xacro_to_urdf(model)
        except Exception as e:
            raise ComposeError(f"{what}: URDF 생성 실패.\n  {e}")
        robot = ET.parse(urdf_path).getroot()

        root = find_root(robot, what)
        anchor = model.get("anchor") or root
        reroot_at_anchor(robot, anchor, what)
        # re-root 후 anchor 가 실제 루트인지 재확인(안전망)
        new_root = find_root(robot, what + " (re-root 후)")
        if new_root != anchor:
            raise ComposeError(
                f"{what}: re-root 후 루트가 anchor 와 다릅니다 "
                f"(루트={new_root}, anchor={anchor}).")

        prefix = resolve_prefix(model, slot)
        apply_prefix(robot, prefix)
        parts[slot] = {"anchor": _prefixed(prefix, anchor), "prefix": prefix,
                       "robot": robot, "model_id": mid}

    # 2) 이름 충돌 검사 — prefix 를 안 붙였을 때 조용히 덮이는 걸 막는다
    seen = {}   # name -> slot
    for slot, p in parts.items():
        for tag in ("link", "joint"):
            for el in p["robot"].findall(tag):
                n = el.get("name")
                key = (tag, n)
                if key in seen:
                    raise ComposeError(
                        f"{tag} 이름 충돌: '{n}' 이 {seen[key]} 슬롯과 {slot} 슬롯 양쪽에 "
                        "있습니다.\n"
                        "  조치: 둘 중 한 모델의 yaml 에 prefix 를 지정하세요.\n"
                        f"    예)  prefix: {slot}_\n"
                        f"  모델 폴더: {reg.models_dir()}")
                seen[key] = slot

    # 3) 병합
    out = ET.Element("robot", {"name": robot_name})
    mats = {}   # 최상위 material 은 이름 기준 dedupe(내용이 다르면 실패)
    for slot in reg.SLOTS:
        if slot not in parts:
            continue
        out.append(ET.Comment(f" ===== {slot}: {parts[slot]['model_id']} ===== "))
        for el in list(parts[slot]["robot"]):
            if el.tag == "material" and el.get("name"):
                n = el.get("name")
                blob = ET.tostring(el)
                if n in mats:
                    if mats[n] != blob:
                        raise ComposeError(
                            f"material '{n}' 이 슬롯마다 다르게 정의돼 있습니다.\n"
                            "  조치: 해당 모델 yaml 에 prefix 를 지정하세요.")
                    continue        # 같은 정의면 하나만 남긴다
                mats[n] = blob
            out.append(copy.deepcopy(el))

    # 4) mount 조인트
    for slot in reg.SLOTS:
        if slot not in parts or slot == "base":
            continue
        m = mounts.get(slot) or {}
        ps = m.get("parent_slot")
        pf = m.get("parent_frame")
        if not ps or not pf:
            raise ComposeError(
                f"{slot} 슬롯: mounts.{slot} 에 parent_slot/parent_frame 이 없습니다.\n"
                "  조립기에서 부모·부착 프레임을 지정한 뒤 저장하세요.")
        if ps not in parts:
            raise ComposeError(
                f"{slot} 슬롯의 부모 슬롯 '{ps}' 에 모델이 없습니다.\n"
                f"  {mounts_path} 의 models.{ps} 를 설정하거나 부모를 바꾸세요.")
        parent_link = _prefixed(parts[ps]["prefix"], pf)
        if parent_link not in {e.get("name") for e in parts[ps]["robot"].findall("link")}:
            avail = sorted(e.get("name") for e in parts[ps]["robot"].findall("link"))
            raise ComposeError(
                f"{slot} 슬롯: 부착 프레임 '{pf}' 이 부모({ps}={parts[ps]['model_id']}) 에 "
                "없습니다.\n"
                f"  부모의 링크: {', '.join(avail[:12])}{' ...' if len(avail) > 12 else ''}\n"
                "  조치: 조립기에서 부착 프레임을 다시 고르고 저장하세요.")
        j = ET.SubElement(out, "joint", {"name": f"{slot}_mount_joint", "type": "fixed"})
        ET.SubElement(j, "parent", {"link": parent_link})
        ET.SubElement(j, "child", {"link": parts[slot]["anchor"]})
        j.append(_origin_el(m.get("xyz") or [0, 0, 0], m.get("rpy") or [0, 0, 0]))

    # 5) 최종 검사 — 단일 트리인가
    find_root(out, "통합 URDF")

    ET.indent(out, space="  ")
    return ET.tostring(out, encoding="unicode")


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="mounts.yaml → 통합 URDF (rda_robot.urdf.xacro 대체)")
    ap.add_argument("--mounts", help="mounts.yaml 경로 (생략 시 정본 소스의 config/mounts.yaml)")
    ap.add_argument("-o", "--out", help="출력 파일 (생략 시 stdout)")
    ap.add_argument("--name", default="rda_robot", help="robot name")
    a = ap.parse_args(argv)

    mounts = a.mounts
    if not mounts:
        mounts = os.path.join(os.path.dirname(reg.models_dir()), "mounts.yaml")
    try:
        xml = compose(mounts, a.name)
    except ComposeError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if a.out:
        with open(a.out, "w") as f:
            f.write(xml)
    else:
        sys.stdout.write(xml)
    return 0


if __name__ == "__main__":
    sys.exit(main())
