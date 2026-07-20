import FreeCAD, Import, os
SRC = os.path.join(os.path.dirname(__file__), "chamdan2greenhause.stp")
doc = FreeCAD.newDocument("gh")
Import.insert(SRC, "gh")
print("문서 객체 수:", len(doc.Objects))
for o in doc.Objects:
    shp = getattr(o, "Shape", None)
    if shp is None or not hasattr(shp,"Solids") or len(shp.Solids)==0:
        continue
    b = shp.BoundBox; n=len(shp.Solids)
    print(f"[{o.Label}] 솔리드 {n} | X{b.XLength/1000:.2f} Y{b.YLength/1000:.2f} Z{b.ZLength/1000:.2f}m Z[{b.ZMin/1000:.3f},{b.ZMax/1000:.3f}]")
    for s in shp.Solids[:2]:
        sb=s.BoundBox
        print(f"    · {sb.XLength:.0f}x{sb.YLength:.0f}x{sb.ZLength:.0f}mm Z[{sb.ZMin/1000:.3f},{sb.ZMax/1000:.3f}]")
