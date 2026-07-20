# 2차: 부재 유형별 위치 추출 → 줄 간격/통로/높이 산출
import FreeCAD, Part, os
SRC = os.path.join(os.path.dirname(__file__), "chamdan2greenhause.stp")
shape = Part.Shape(); shape.read(SRC)

def key(s):
    bb=s.BoundBox
    return (round(bb.XLength,0), round(bb.YLength,0), round(bb.ZLength,0))

groups={}
for s in shape.Solids:
    groups.setdefault(key(s), []).append(s)

def report(k, label):
    if k not in groups:
        print(f"\n[{label}] 해당 유형 없음 {k}"); return
    solids=groups[k]
    print(f"\n=== [{label}] {k} mm × {len(solids)}개 ===")
    rows=[]
    for s in solids:
        bb=s.BoundBox
        rows.append((round(bb.Center.x,1), round(bb.Center.y,1),
                     round(bb.ZMin,1), round(bb.ZMax,1)))
    rows.sort()
    for cx,cy,zmin,zmax in rows:
        print(f"   중심X={cx/1000:+.3f} m  중심Y={cy/1000:+.3f} m  Z[{zmin/1000:.3f}, {zmax/1000:.3f}] m")
    # X 간격
    xs=sorted(set(r[0] for r in rows))
    if len(xs)>1:
        gaps=[round((xs[i+1]-xs[i])/1000,3) for i in range(len(xs)-1)]
        print(f"   → X 중심 간격들(m): {gaps}")

report((579.0,35079.0,50.0), "재배 거터(베드)")
report((240.0,34050.0,120.0), "지면 파이프레일")
report((240.0,160.0,1020.0), "지지 기둥")
