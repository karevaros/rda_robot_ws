#!/usr/bin/env python3
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import numpy as np
from rda_robot_assembler import part_registry as reg
from rda_robot_assembler import urdf_loader as ul
from rda_robot_assembler.assembly import Mount, compute_placements

loaded = {s: ul.load_part(reg.default_model(s), reg.MODELS[reg.default_model(s)]) for s in reg.SLOTS}

mounts = {
    "arm":         Mount("base", "base_link", [0, 0, 0.25], [0, 0, 0]),
    "endeffector": Mount("arm", "tcp", [0, 0, 0], [0, 0, 0]),
    "sensor1":     Mount("endeffector", "rg2_hand", [0.02, 0, 0.03], [0, 0, 0]),
    "sensor2":     Mount("base", "base_link", [-0.25, 0, 0.6], [0, 0.3, 0]),
}
world = compute_placements(loaded, mounts)
print("배치된 슬롯:", list(world.keys()))
for slot in reg.SLOTS:
    if slot not in world:
        print(f"  ❌ {slot} 미배치"); continue
    # anchor 의 world 위치 = part_world @ T_root_anchor
    aw = world[slot] @ loaded[slot].T_root_anchor
    print(f"  {slot:11s} part_world_pos={np.round(world[slot][:3,3],3)}  anchor_world_pos={np.round(aw[:3,3],3)}")
