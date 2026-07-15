"""자충돌(self-collision) 검사 — trimesh + python-fcl.

각 파트의 링크를 개별 충돌객체로 등록(파트 root 프레임에 baking)하고,
파트 world 변환만 set_transform 으로 갱신해 빠르게 재검사한다.
관절 포즈가 바뀐 파트는 rebuild_part 로 현재 포즈를 다시 baking 한다.

무시(allowed collision)되는 쌍:
  ① 같은 파트에서 관절로 연결된 인접 링크(설계상 항상 접촉)
  ② 사용자가 '현재 겹침 무시'로 등록한 충돌쌍(모델 자체 겹침 보정)

■ 마운트 접합쌍을 왜 무시하지 않는가 (2026-07-15 수정)
  예전엔 '자식 anchor 링크 ↔ 부모 parent_frame 링크'를 무조건 무시했다. 그 탓에
  팔 마운트를 모바일 플랫폼 안으로 100mm 가까이 파묻어도 경고가 전혀 뜨지 않는
  사각지대가 있었다(사용자 신고).
  침투깊이 임계값으로 완화해 보려 했으나, FCL 의 mesh-mesh 침투깊이는 접촉한
  삼각형 하나의 값이라 실제 침투량과 무관하게 널뛴다(실측: 실제 8mm 침투 →
  48.8mm 보고 / 실제 58mm 침투 → 21mm 보고). 임계값 방식은 신뢰할 수 없어 폐기.
  대신 **실제 mesh 교차 자체를 신호로 사용**한다. 플랜지가 상판에 '얹힌' 정상
  결합은 삼각형이 교차하지 않아 조용하고, 진짜로 파고들면 교차하므로 보고된다.
  설계상 원래 겹치는 모델은 시작 시 자동 기준보정(②)이 걸러 준다.

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

    def _adjacent_allowed(self, loaded):
        """관절로 연결된 인접 링크쌍 — 설계상 항상 접촉하므로 무조건 무시."""
        allow = set()
        for slot, part in loaded.items():
            for pair in getattr(part, "adjacent_links", set()):
                a, b = tuple(pair)
                allow.add(frozenset((self._name(slot, a), self._name(slot, b))))
        return allow

    def _structural_allowed(self, loaded, mounts=None):
        """구조적으로 항상 무시하는 쌍 = 관절 인접쌍."""
        return self._adjacent_allowed(loaded)

    def _colliding_names(self):
        """FCL 내부 충돌쌍을 frozenset 집합으로 정규화(버전별 tuple/frozenset 대응)."""
        hit, names = self.mgr.in_collision_internal(return_names=True)
        if not hit:
            return set()
        return {frozenset(p) for p in names}

    def _report(self, loaded, extra_allowed):
        """무시규칙 적용 후 남는 충돌쌍."""
        allow = self._adjacent_allowed(loaded) | extra_allowed
        return {p for p in self._colliding_names() if p not in allow}

    def check(self, loaded, mounts=None):
        """무시쌍 제외한 실제 충돌 집합 반환: set of frozenset(nameA, nameB)."""
        if not self.enabled:
            return set()
        return self._report(loaded, self.allowed)

    def calibrate(self, loaded, mounts=None):
        """현재 충돌을 기준으로 등록해 이후 무시. 등록 건수 반환."""
        new = self._report(loaded, set())
        self.allowed |= new
        return len(new)

    def clear_calibration(self):
        self.allowed = set()
