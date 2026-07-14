#!/usr/bin/env python3
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
from rda_robot_assembler import part_registry as reg
from rda_robot_assembler import urdf_loader as ul

for slot in reg.SLOTS:
    mid = reg.default_model(slot)
    m = reg.MODELS[mid]
    try:
        p = ul.load_part(mid, m)
        # anchor 위치
        ap = p.T_root_anchor[:3, 3]
        print(f"✅ {slot:11s} {mid:12s} root={p.root:10s} anchor={p.anchor:12s} "
              f"anchorpos={np.round(ap,3)} #frames={len(p.frames)} #meshinst={len(p.mesh_instances)}")
        # 부착점 후보 몇 개
        print(f"     links: {p.link_names[:8]}{'...' if len(p.link_names)>8 else ''}")
    except Exception as e:
        import traceback; traceback.print_exc()
        print(f"❌ {slot} {mid}: {e}")
