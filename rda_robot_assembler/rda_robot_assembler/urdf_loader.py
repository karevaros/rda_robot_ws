"""파트 URDF 로드: xacro→urdf 생성, yourdfpy 로드(package:// 리졸브),
프레임 FK(부착점) / anchor 변환 / 렌더용 mesh 인스턴스 추출.
"""
import os
import subprocess
import tempfile
import numpy as np
from ament_index_python.packages import get_package_share_directory

import yourdfpy

_CACHE_DIR = os.path.join(tempfile.gettempdir(), "rda_assembler_cache")
os.makedirs(_CACHE_DIR, exist_ok=True)


def _pkg_resolver(fname):
    """package://PKG/rest -> <share>/rest"""
    if fname.startswith("package://"):
        rest = fname[len("package://"):]
        pkg, _, tail = rest.partition("/")
        try:
            return os.path.join(get_package_share_directory(pkg), tail)
        except Exception:
            return fname
    return fname


def xacro_to_urdf(model):
    """model dict -> URDF 파일 경로(캐시)."""
    share = get_package_share_directory(model["pkg"])
    xacro_path = os.path.join(share, model["xacro"])
    args = [f"{k}:={v}" for k, v in model.get("args", {}).items()]
    key = model["pkg"] + "__" + os.path.basename(model["xacro"]) + "__" + "_".join(args)
    key = key.replace("/", "_").replace(":", "")
    out = os.path.join(_CACHE_DIR, key + ".urdf")
    # 캐시가 소스보다 최신이면 재사용
    if os.path.exists(out) and os.path.getmtime(out) >= os.path.getmtime(xacro_path):
        return out
    res = subprocess.run(["xacro", xacro_path] + args, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"xacro 실패 ({model['pkg']}):\n{res.stderr[-800:]}")
    with open(out, "w") as f:
        f.write(res.stdout)
    return out


class LoadedPart:
    """로드된 파트 1개의 렌더/FK 데이터."""

    def __init__(self, model_id, model):
        self.model_id = model_id
        self.model = model
        urdf_path = xacro_to_urdf(model)
        self.robot = yourdfpy.URDF.load(
            urdf_path, filename_handler=_pkg_resolver,
            load_meshes=True, build_scene_graph=True,
        )
        self.root = self.robot.base_link
        self.scene = self.robot.scene
        self.base_frame = self.scene.graph.base_frame

        # 링크 이름(부착점 드롭다운용, 순서 보존)
        self.link_names = list(self.robot.link_map.keys())

        # 가동(actuated) 관절 + limit — 초기 포즈 UI 용
        self.actuated = list(getattr(self.robot, "actuated_joint_names", []))
        self.joint_limits = {}
        for jn in self.actuated:
            j = self.robot.joint_map.get(jn)
            lo, hi = -3.14159, 3.14159
            if j is not None and getattr(j, "limit", None) is not None:
                if j.limit.lower is not None:
                    lo = float(j.limit.lower)
                if j.limit.upper is not None:
                    hi = float(j.limit.upper)
            self.joint_limits[jn] = (lo, hi)
        self.joint_pose = {jn: 0.0 for jn in self.actuated}

        # anchor(부모에 붙는 프레임) 이름 — 관절과 무관(루트측)하므로 고정
        anchor = model.get("anchor") or self.root
        self.anchor = anchor

        # 인접 링크쌍(관절로 연결 = 설계상 접촉) — 자충돌 검사 시 무시용
        self.adjacent_links = set()
        for j in self.robot.joint_map.values():
            pa, ch = getattr(j, "parent", None), getattr(j, "child", None)
            if pa and ch:
                self.adjacent_links.add(frozenset((pa, ch)))

        self._extract()

    def _extract(self):
        """현재 관절 포즈 기준으로 프레임 FK / anchor 변환 / mesh 인스턴스 재계산."""
        self.frames = {}
        for name in self.scene.graph.nodes:
            try:
                T = self.scene.graph.get(frame_to=name, frame_from=self.base_frame)[0]
                self.frames[name] = np.asarray(T, dtype=float)
            except Exception:
                pass
        if self.anchor not in self.frames:
            self.anchor = self.root
        self.T_root_anchor = self.frames.get(self.anchor, np.eye(4))
        self.T_anchor_root = np.linalg.inv(self.T_root_anchor)

        # 렌더용 mesh 인스턴스 + 충돌검사용 링크별 mesh(면 있는 것만)
        self.mesh_instances = []
        self.link_meshes = {}   # link_name -> list[(geom, T_root_mesh)]
        graph = self.scene.graph
        for node in graph.nodes_geometry:
            T, gname = graph[node]
            geom = self.scene.geometry.get(gname)
            if geom is None or not hasattr(geom, "vertices"):
                continue
            T = np.asarray(T, dtype=float)
            self.mesh_instances.append((geom, T))
            # 충돌객체는 삼각면이 있어야 함(FCL). 링크는 geometry 노드의 부모 프레임.
            if getattr(geom, "faces", None) is None:
                continue
            try:
                link = graph.transforms.parents.get(node) or self.root
            except Exception:
                link = self.root
            self.link_meshes.setdefault(link, []).append((geom, T))

    def set_joint_pose(self, pose):
        """pose: dict {joint_name: rad}. 가동관절만 반영 후 FK/mesh 재계산."""
        if not self.actuated:
            return
        cfg = {}
        for jn in self.actuated:
            v = pose.get(jn, self.joint_pose.get(jn, 0.0))
            self.joint_pose[jn] = float(v)
            cfg[jn] = float(v)
        try:
            self.robot.update_cfg(cfg)
        except Exception:
            # dict 미지원 버전 대비: 순서 배열로
            arr = np.array([cfg[j] for j in self.actuated], dtype=float)
            self.robot.update_cfg(arr)
        self._extract()

    def frame_world(self, part_world, frame):
        """이 파트가 part_world 에 놓였을 때 특정 프레임의 world 변환."""
        return part_world @ self.frames.get(frame, np.eye(4))


def load_part(model_id, model):
    return LoadedPart(model_id, model)
