#!/usr/bin/env python3
# bboxes.csv -> 온실 단면도(X-Z, Y 방향에서 본 것) + 평면도(X-Y)
import csv, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

here = os.path.dirname(__file__)
rows = []
with open(os.path.join(here, "bboxes.csv")) as f:
    for r in csv.DictReader(f):
        rows.append({k: float(v) for k, v in r.items()})

def classify(r):
    xl, yl, zl, zmin = r["xlen"], r["ylen"], r["zlen"], r["zmin"]
    if xl > 15000 and yl > 30000:            # 외곽 셸
        return ("envelope", "#cccccc", 0.15)
    if abs(zl-120) < 5 and zmin > 700:        # 고설 거터/베드 (Z~0.8)
        return ("bed_high", "#2a7", 0.9)
    if abs(zl-50) < 5 and zmin < 400:         # 지면 부재 (Z~0.2)
        return ("ground", "#48c", 0.9)
    if zmin > 4000:                           # 지붕 구조
        return ("roof", "#e94", 0.8)
    return ("other", "#999", 0.6)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

# --- 단면도 X-Z ---
for r in rows:
    cat, col, a = classify(r)
    ax1.add_patch(Rectangle((r["xmin"]/1000, r["zmin"]/1000),
                            r["xlen"]/1000, max(r["zlen"]/1000, 0.03),
                            facecolor=col, edgecolor="k", lw=0.4, alpha=a))
ax1.set_xlim(-16, 3); ax1.set_ylim(-0.3, 6.5)
ax1.set_xlabel("X (m, 폭)"); ax1.set_ylabel("Z (m, 높이)")
ax1.set_title("단면도 (X-Z) — 옆에서 본 온실")
ax1.axhline(0, color="brown", lw=1); ax1.grid(alpha=0.3)
ax1.set_aspect("equal")

# --- 평면도 X-Y ---
for r in rows:
    cat, col, a = classify(r)
    ax2.add_patch(Rectangle((r["xmin"]/1000, r["ymin"]/1000),
                            r["xlen"]/1000, r["ylen"]/1000,
                            facecolor=col, edgecolor="k", lw=0.4, alpha=a))
ax2.set_xlim(-16, 3); ax2.set_ylim(-21, 21)
ax2.set_xlabel("X (m, 폭)"); ax2.set_ylabel("Y (m, 길이)")
ax2.set_title("평면도 (X-Y) — 위에서 본 온실")
ax2.grid(alpha=0.3); ax2.set_aspect("equal")

from matplotlib.patches import Patch
leg = [Patch(facecolor="#cccccc", label="외곽 셸 18x40x6.2m"),
       Patch(facecolor="#2a7", label="고설 부재 Z~0.8-0.92m (8개)"),
       Patch(facecolor="#48c", label="지면 부재 Z~0.2m (10개)"),
       Patch(facecolor="#e94", label="지붕 구조 Z~5-6m")]
ax1.legend(handles=leg, loc="upper left", fontsize=8)

plt.tight_layout()
out = os.path.join(here, "greenhouse_layout.png")
plt.savefig(out, dpi=110)
print("saved", out)
