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

        # 프레임(링크) FK: name -> 4x4 (root->frame)
        self.frames = {}
        for name in self.scene.graph.nodes:
            try:
                T = self.scene.graph.get(frame_to=name, frame_from=self.base_frame)[0]
                self.frames[name] = np.asarray(T, dtype=float)
            except Exception:
                pass
        # 링크 이름은 순서 보존(부착점 드롭다운용)
        self.link_names = list(self.robot.link_map.keys())

        # anchor(부모에 붙는 프레임) 변환 root->anchor
        anchor = model.get("anchor") or self.root
        self.anchor = anchor if anchor in self.frames else self.root
        self.T_root_anchor = self.frames.get(self.anchor, np.eye(4))
        self.T_anchor_root = np.linalg.inv(self.T_root_anchor)

        # 렌더용 mesh 인스턴스: (trimesh_geom, T_root_mesh)
        self.mesh_instances = []
        for node in self.scene.graph.nodes_geometry:
            T, gname = self.scene.graph[node]
            geom = self.scene.geometry.get(gname)
            if geom is None or not hasattr(geom, "vertices"):
                continue
            self.mesh_instances.append((geom, np.asarray(T, dtype=float)))

    def frame_world(self, part_world, frame):
        """이 파트가 part_world 에 놓였을 때 특정 프레임의 world 변환."""
        return part_world @ self.frames.get(frame, np.eye(4))


def load_part(model_id, model):
    return LoadedPart(model_id, model)
