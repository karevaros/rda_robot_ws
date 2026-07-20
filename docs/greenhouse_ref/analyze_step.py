# FreeCAD 헤드리스: STEP 온실 파일 정확 실측 (배치 변환 반영)
# 실행: freecadcmd analyze_step.py
import FreeCAD, Part, os

SRC = os.path.join(os.path.dirname(__file__), "chamdan2greenhause.stp")
print("파일:", SRC)

shape = Part.Shape()
shape.read(SRC)

def bb_line(name, bb):
    print(f"  [{name}] X {bb.XLength/1000:.3f} m  Y {bb.YLength/1000:.3f} m  Z {bb.ZLength/1000:.3f} m"
          f"  | 원점 X[{bb.XMin/1000:.2f},{bb.XMax/1000:.2f}] Y[{bb.YMin/1000:.2f},{bb.YMax/1000:.2f}] Z[{bb.ZMin/1000:.2f},{bb.ZMax/1000:.2f}] m")

print("\n=== 전체 바운딩박스 (배치 반영) ===")
bb_line("ALL", shape.BoundBox)

# 최상위 솔리드/컴파운드 분해
solids = shape.Solids
print(f"\n=== 솔리드 개수: {len(solids)} ===")

# 각 솔리드의 바운딩박스 수집 → 크기 분포 파악
import collections
sizes = []
for i, s in enumerate(solids):
    bb = s.BoundBox
    sizes.append((round(bb.XLength,1), round(bb.YLength,1), round(bb.ZLength,1),
                  round(bb.XMin,1), round(bb.YMin,1), round(bb.ZMin,1), s.Volume))

# 유형별 그룹핑 (X,Y,Z 치수 반올림 기준)
groups = collections.Counter((sx,sy,sz) for sx,sy,sz,_,_,_,_ in sizes)
print("\n=== 솔리드 치수 유형별 개수 (mm, 상위 25) ===")
for (sx,sy,sz), cnt in groups.most_common(25):
    print(f"  {cnt:4d} 개  |  {sx:8.1f} x {sy:8.1f} x {sz:8.1f} mm")
