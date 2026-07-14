"""조립 배치 계산.

각 슬롯의 mount 설정:
    Mount(parent_slot, parent_frame, xyz(3), rpy(3))
base 슬롯은 부모 없음(루트).

part_world(base)  = I
part_world(slot)  = part_world(parent_slot)
                    @ frames[parent_frame]        # 부모 파트 내 부착 프레임
                    @ M(xyz, rpy)                  # 사용자 설정 오프셋
                    @ inv(T_root_anchor)           # anchor 를 부착점에 정렬
"""
import numpy as np
from scipy.spatial.transform import Rotation


def mat(xyz, rpy):
    """URDF 규약(고정축 XYZ = Rz*Ry*Rx) 4x4 변환."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


class Mount:
    __slots__ = ("parent_slot", "parent_frame", "xyz", "rpy")

    def __init__(self, parent_slot=None, parent_frame=None, xyz=(0, 0, 0), rpy=(0, 0, 0)):
        self.parent_slot = parent_slot
        self.parent_frame = parent_frame
        self.xyz = list(xyz)
        self.rpy = list(rpy)


def compute_placements(loaded, mounts):
    """loaded: dict slot->LoadedPart, mounts: dict slot->Mount.
    반환: dict slot->4x4 world 변환. 미배치/순환은 건너뜀.
    """
    world = {}
    # base 는 항상 루트
    if "base" in loaded:
        world["base"] = np.eye(4)

    # 나머지는 부모가 배치될 때까지 반복(단순 트리 → 최대 슬롯수 회 반복)
    pending = [s for s in loaded if s != "base"]
    for _ in range(len(pending) + 1):
        progressed = False
        for slot in list(pending):
            mnt = mounts.get(slot)
            if mnt is None or mnt.parent_slot is None:
                continue
            ps = mnt.parent_slot
            if ps not in world:
                continue  # 부모 아직 미배치
            parent_part = loaded.get(ps)
            if parent_part is None:
                continue
            T_parent_frame = parent_part.frames.get(mnt.parent_frame, np.eye(4))
            M = mat(mnt.xyz, mnt.rpy)
            world[slot] = world[ps] @ T_parent_frame @ M @ loaded[slot].T_anchor_root
            pending.remove(slot)
            progressed = True
        if not progressed:
            break
    return world
