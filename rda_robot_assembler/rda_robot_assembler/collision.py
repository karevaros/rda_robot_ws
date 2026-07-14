"""자충돌(self-collision) 검사 — trimesh + python-fcl.

각 파트의 링크를 개별 충돌객체로 등록(파트 root 프레임에 baking)하고,
파트 world 변환만 set_transform 으로 갱신해 빠르게 재검사한다.
관절 포즈가 바뀐 파트는 rebuild_part 로 현재 포즈를 다시 baking 한다.

무시(allowed collision)되는 쌍:
  ① 같은 파트에서 관절로 연결된 인접 링크(설계상 항상 접촉)
  ② 파트 마운트 접합쌍(자식 anchor 링크 ↔ 부모 parent_frame 링크)
  ③ 사용자가 '기준 보정'으로 등록한 현재 충돌쌍(모델 자체 겹침 보정)

객체 이름 규약: "<slot>::<link>"
"""
import numpy as np
import trimesh


class CollisionChecker:
    def __init__(self):
        self.mgr = trimesh.collision.CollisionManager()
        self.slot_links = {}    # slot -> list[link] (등록된 객체)
        self.allowed = set()    # frozenset({nameA, nameB}) 사용자 기준 보정
        self.enabled = True

    @staticmethod
    def _name(slot, link):
        return f"{slot}::{link}"

    def rebuild_part(self, slot, part):
        """파트의 링크별 mesh 를 현재 포즈로 baking 해 충돌객체로 (재)등록."""
        self.drop_part(slot)
        links = []
        for link, insts in getattr(part, "link_meshes", {}).items():
            meshes = []
            for geom, T in insts:
                try:
                    m = geom.copy()
                    m.apply_transform(np.asarray(T, dtype=float))
                    meshes.append(m)
                except Exception:
                    continue
            if not meshes:
                continue
            combined = meshes[0] if len(meshes) == 1 else trimesh.util.concatenate(meshes)
            try:
                self.mgr.add_object(self._name(slot, link), combined)
                links.append(link)
            except Exception:
                pass
        self.slot_links[slot] = links

    def drop_part(self, slot):
        for link in self.slot_links.pop(slot, []):
            try:
                self.mgr.remove_object(self._name(slot, link))
            except Exception:
                pass

    def set_world(self, slot, W):
        """파트 world 변환 갱신(모든 링크 객체에 동일 W = root→world)."""
        W = np.asarray(W, dtype=float)
        for link in self.slot_links.get(slot, []):
            try:
                self.mgr.set_transform(self._name(slot, link), W)
            except Exception:
                pass

    def _structural_allowed(self, loaded, mounts):
        """관절 인접쌍 + 마운트 접합쌍(항상 무시)."""
        allow = set()
        for slot, part in loaded.items():
            for pair in getattr(part, "adjacent_links", set()):
                a, b = tuple(pair)
                allow.add(frozenset((self._name(slot, a), self._name(slot, b))))
        for slot, mnt in mounts.items():
            if slot not in loaded or mnt is None or mnt.parent_slot not in loaded:
                continue
            child_root = getattr(loaded[slot], "anchor", None) \
                or getattr(loaded[slot], "root", None)
            allow.add(frozenset((self._name(slot, child_root),
                                 self._name(mnt.parent_slot, mnt.parent_frame))))
        return allow

    def _colliding_names(self):
        """FCL 내부 충돌쌍을 frozenset 집합으로 정규화(버전별 tuple/frozenset 대응)."""
        hit, names = self.mgr.in_collision_internal(return_names=True)
        if not hit:
            return set()
        return {frozenset(p) for p in names}

    def check(self, loaded, mounts):
        """무시쌍 제외한 실제 충돌 집합 반환: set of frozenset(nameA, nameB)."""
        if not self.enabled:
            return set()
        allow = self._structural_allowed(loaded, mounts) | self.allowed
        return {p for p in self._colliding_names() if p not in allow}

    def calibrate(self, loaded, mounts):
        """현재 충돌(구조적 무시 제외)을 기준으로 등록해 이후 무시. 등록 건수 반환."""
        allow = self._structural_allowed(loaded, mounts)
        new = {p for p in self._colliding_names() if p not in allow}
        self.allowed |= new
        return len(new)

    def clear_calibration(self):
        self.allowed = set()
