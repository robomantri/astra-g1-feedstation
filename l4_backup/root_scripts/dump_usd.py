"""Dump the full contents of the ASTRA scan USDC so we know exactly what it holds."""
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True).app

from pxr import Usd, UsdGeom, UsdShade, Gf

P = "/root/coordex/source/coordex/coordex/assets/office.usdc"
stage = Usd.Stage.Open(P)

prims = list(stage.Traverse())
print("=" * 70)
print(f"FILE: {P}")
print(f"defaultPrim   : {stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else None}")
print(f"upAxis        : {UsdGeom.GetStageUpAxis(stage)}")
print(f"metersPerUnit : {UsdGeom.GetStageMetersPerUnit(stage)}")
print(f"total prims   : {len(prims)}")
print("=" * 70)

from collections import Counter
print("\n--- PRIM TYPES ---")
for t, n in Counter(p.GetTypeName() for p in prims).most_common():
    print(f"  {n:6d}  {t}")

print("\n--- GEOMETRY ---")
total_pts = total_faces = 0
meshes = []
for p in prims:
    if p.IsA(UsdGeom.Mesh):
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        fvc = m.GetFaceVertexCountsAttr().Get()
        np_, nf = len(pts) if pts else 0, len(fvc) if fvc else 0
        total_pts += np_; total_faces += nf
        meshes.append((p.GetPath(), np_, nf))
    elif p.IsA(UsdGeom.Points):
        pc = UsdGeom.Points(p).GetPointsAttr().Get()
        print(f"  POINTCLOUD {p.GetPath()}  {len(pc) if pc else 0} points")
print(f"  Mesh prims: {len(meshes)}   total points: {total_pts}   total faces: {total_faces}")
for path, np_, nf in meshes[:15]:
    print(f"    {str(path)[:60]:60s} pts={np_:8d} faces={nf:8d}")
if len(meshes) > 15:
    print(f"    ... and {len(meshes)-15} more")

print("\n--- PRIMVARS on first mesh (colour data?) ---")
if meshes:
    mp = stage.GetPrimAtPath(meshes[0][0])
    api = UsdGeom.PrimvarsAPI(mp)
    for pv in api.GetPrimvars():
        val = pv.Get()
        n = len(val) if hasattr(val, "__len__") else "scalar"
        print(f"    {pv.GetName():30s} interp={pv.GetInterpolation():12s} count={n}")

print("\n--- MATERIALS / TEXTURES ---")
mats = [p for p in prims if p.IsA(UsdShade.Material)]
print(f"  materials: {len(mats)}")
for p in prims:
    if p.IsA(UsdShade.Shader):
        sh = UsdShade.Shader(p)
        for i in sh.GetInputs():
            v = i.Get()
            if v is not None and ("file" in i.GetBaseName().lower() or "asset" in str(type(v)).lower()):
                print(f"    {p.GetPath()} :: {i.GetBaseName()} = {v}")

print("\n--- EXTERNAL REFERENCES / PAYLOADS ---")
ext = 0
for p in prims:
    for r in p.GetMetadata("references").prependedItems if p.GetMetadata("references") else []:
        print(f"    REF  {p.GetPath()} -> {r.assetPath}"); ext += 1
    for pl in p.GetMetadata("payload").prependedItems if p.GetMetadata("payload") else []:
        print(f"    PAYL {p.GetPath()} -> {pl.assetPath}"); ext += 1
print(f"  external refs/payloads: {ext}")
print(f"  layer external deps: {stage.GetRootLayer().GetExternalReferences()}")

print("\n--- WORLD BOUNDING BOX ---")
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
rng = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
mn, mx = rng.GetMin(), rng.GetMax()
print(f"  min  ({mn[0]:9.3f}, {mn[1]:9.3f}, {mn[2]:9.3f})")
print(f"  max  ({mx[0]:9.3f}, {mx[1]:9.3f}, {mx[2]:9.3f})")
print(f"  size ({mx[0]-mn[0]:9.3f}, {mx[1]-mn[1]:9.3f}, {mx[2]-mn[2]:9.3f})")
app.close()
