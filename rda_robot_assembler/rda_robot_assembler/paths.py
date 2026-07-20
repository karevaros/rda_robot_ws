"""모델 폴더 경로 해석 (단일 정본).

■ 정본 = 소스 트리의 `rda_robot_description/config/models` (2026-07-16 확정)

조립기와 통합 URDF 컴포저가 **같은 폴더**를 읽어야 폴더 드롭이 재빌드 없이 즉시
반영된다. 한쪽이 설치본(share)을 읽으면, 드롭 후 `colcon build` 를 빼먹었을 때
조립기만 새 모델을 보고 컴포저는 못 보는 **조용한 불일치**가 생긴다.

실제로 그렇게 됐다: `027ba15` 가 모델 21종을 소스에 추가하고 재빌드를 하지 않아
share 에는 반영되지 않았고, `test_models.py`(조립기=소스) 는 28/28 통과하는데
통합 URDF(share) 는 그 모델을 모르는 상태가 하루 동안 유지됐다.

launch 도 같은 원칙이다 — `rda_robot_display.launch.py` 는 이미 mounts.yaml 을
소스에서 읽는다("앱 저장 후 colcon 재빌드 없이 반영").

해석 순서:
  1. 환경변수 `RDA_MODELS_DIR`      — 명시 지정이 최우선
  2. 소스 트리 자동 탐색            — 이 파일의 realpath 에서 위로 올라가며 탐색
                                       (symlink-install 이면 설치본에서도 소스로 풀린다)
  3. 설치 share (폴백)              — 소스를 못 찾은 경우. 폴더 드롭이 재빌드 전까지
                                       반영되지 않으므로 경고를 낸다.
  4. 실패                           — 조용히 빈 목록을 주지 않고 명확히 실패한다.
"""
import os
import sys

# <워크스페이스>/src 아래에서 찾을 상대 경로
_MODELS_SUBPATH = os.path.join("rda_robot_description", "config", "models")

_cached = None


def _from_env():
    d = os.environ.get("RDA_MODELS_DIR")
    if not d:
        return None
    d = os.path.expanduser(d)
    if not os.path.isdir(d):
        raise RuntimeError(
            f"RDA_MODELS_DIR 이 가리키는 폴더가 없습니다: {d}\n"
            "  환경변수를 고치거나, 해제해서 자동 탐색을 쓰세요.")
    return d


def _from_source_tree():
    """이 파일의 realpath 에서 위로 올라가며 `*/rda_robot_description/config/models` 탐색.

    realpath 를 쓰는 이유: colcon --symlink-install 이면 설치본의 __file__ 이
    build/ 아래를 가리켜도 realpath 는 소스로 풀린다(실측 확인).
    """
    here = os.path.realpath(__file__)
    d = os.path.dirname(here)
    while True:
        cand = os.path.join(d, _MODELS_SUBPATH)
        if os.path.isdir(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:          # 루트 도달
            return None
        d = parent


def _from_share():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("rda_robot_description")
    except Exception:
        return None
    cand = os.path.join(share, "config", "models")
    return cand if os.path.isdir(cand) else None


def environments_dir():
    """배경(환경) 정의 폴더 = config/environments (models 와 같은 config 아래).

    배경은 선택 기능이라 없어도 실패하지 않는다 — 폴더가 없으면 만들어 준다
    (드롭 폴더로 쓰라고). 해석 근거는 models_dir 과 동일(소스 정본).
    """
    cand = os.path.join(os.path.dirname(models_dir()), "environments")
    if not os.path.isdir(cand):
        try:
            os.makedirs(cand, exist_ok=True)
        except OSError:
            pass
    return cand


def models_dir():
    """모델 드롭 폴더(정본). 못 찾으면 RuntimeError."""
    global _cached
    if _cached:
        return _cached

    d = _from_env()
    if d is None:
        d = _from_source_tree()
    if d is None:
        d = _from_share()
        if d is not None:
            print(
                "[rda] 경고: 소스 트리의 config/models 를 찾지 못해 설치본(share)을 씁니다:\n"
                f"        {d}\n"
                "        이 경우 폴더에 새 모델을 넣어도 colcon build 전까지 반영되지 않습니다.\n"
                "        소스에서 실행하거나 RDA_MODELS_DIR 로 직접 지정하세요.",
                file=sys.stderr)
    if d is None:
        raise RuntimeError(
            "모델 폴더(config/models)를 찾지 못했습니다.\n"
            "  기대 위치: <워크스페이스>/src/rda_robot_description/config/models\n"
            "  조치: 워크스페이스 안에서 실행하거나, RDA_MODELS_DIR 로 직접 지정하세요.\n"
            "        예)  export RDA_MODELS_DIR=~/robot_ws/src/rda_robot_description/config/models")

    _cached = d
    return d
