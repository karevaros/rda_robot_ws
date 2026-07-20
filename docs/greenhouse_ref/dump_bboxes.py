# 모든 솔리드 바운딩박스 → CSV (부품명 매핑 포함 시도)
import FreeCAD, Part, os, csv
SRC = os.path.join(os.path.dirname(__file__), "chamdan2greenhause.stp")
shape = Part.Shape(); shape.read(SRC)
out = os.path.join(os.path.dirname(__file__), "bboxes.csv")
with open(out,"w",newline="") as f:
    w=csv.writer(f); w.writerow(["idx","xmin","ymin","zmin","xmax","ymax","zmax","xlen","ylen","zlen"])
    for i,s in enumerate(shape.Solids):
        b=s.BoundBox
        w.writerow([i,round(b.XMin,1),round(b.YMin,1),round(b.ZMin,1),
                    round(b.XMax,1),round(b.YMax,1),round(b.ZMax,1),
                    round(b.XLength,1),round(b.YLength,1),round(b.ZLength,1)])
print("wrote", out, "solids:", len(shape.Solids))
