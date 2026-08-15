"""Dump every mesh in office.usdc with its displayColor and bound-material diffuse.

Run with `python -u` -- Isaac Sim hard-exits and discards a buffered stdout.
"""
import os

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

from isaaclab.app import AppLauncher

app = AppLauncher({"headless": True}).app

from pxr import Usd, UsdGeom, UsdShade, Gf

USD = "/root/coordex/source/coordex/coordex/assets/office.usdc"
stage = Usd.Stage.Open(USD)

print("=" * 78)
print("UP AXIS:", UsdGeom.GetStageUpAxis(stage), " METERS/UNIT:", UsdGeom.GetStageMetersPerUnit(stage))
print("DEFAULT PRIM:", stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else None)
print("=" * 78)

# ---- unresolved asset paths (the thing that silently makes everything white) ----
print("\n--- ASSET PATH REFERENCES ---")
for prim in stage.Traverse():
    for attr in prim.GetAttributes():
        if attr.GetTypeName() == "asset":
            v = attr.Get()
            if v:
                p = v.resolvedPath or "<<UNRESOLVED>>"
                print(f"  {prim.GetPath()}.{attr.GetName()}")
                print(f"      raw={v.path}  resolved={p}")

# ---- materials ----
print("\n--- MATERIALS ---")
mats = {}
for prim in stage.Traverse():
    if prim.GetTypeName() == "Material":
        mat = UsdShade.Material(prim)
        info = {"path": str(prim.GetPath()), "diffuse": None, "tex": None,
                "rough": None, "metal": None}
        for child in Usd.PrimRange(prim):
            if child.GetTypeName() != "Shader":
                continue
            sh = UsdShade.Shader(child)
            sid = sh.GetShaderId() if hasattr(sh, "GetShaderId") else None
            for inp in sh.GetInputs():
                n = inp.GetBaseName()
                val = inp.Get()
                if n in ("diffuseColor", "baseColor") and val is not None:
                    info["diffuse"] = tuple(round(float(c), 4) for c in val)
                if n == "roughness" and val is not None:
                    info["rough"] = round(float(val), 3)
                if n == "metallic" and val is not None:
                    info["metal"] = round(float(val), 3)
                if n == "file" and val is not None:
                    info["tex"] = val.path
                # connected diffuse -> texture
                if n in ("diffuseColor", "baseColor") and inp.HasConnectedSource():
                    src = inp.GetConnectedSource()
                    info["diffuse"] = f"<connected:{src[0].GetPath().name}>"
        mats[str(prim.GetPath())] = info
        print(f"  {info['path']}")
        print(f"      diffuse={info['diffuse']} rough={info['rough']} "
              f"metal={info['metal']} tex={info['tex']}")

# ---- meshes ----
print("\n--- MESHES (name | tris | displayColor | bound material diffuse | size m) ---")
rows = []
for prim in stage.Traverse():
    if prim.GetTypeName() != "Mesh":
        continue
    mesh = UsdGeom.Mesh(prim)
    counts = mesh.GetFaceVertexCountsAttr().Get() or []
    pts = mesh.GetPointsAttr().Get() or []

    dc = mesh.GetDisplayColorAttr().Get()
    dc_s = None
    if dc is not None and len(dc):
        uniq = {tuple(round(float(c), 3) for c in x) for x in dc}
        dc_s = f"{sorted(uniq)[:3]}{'...' if len(uniq) > 3 else ''} (n={len(dc)})"

    binding = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    mdiff = None
    mpath = None
    if binding:
        mpath = str(binding.GetPath())
        mdiff = mats.get(mpath, {}).get("diffuse")

    # local bbox
    dims = None
    if pts:
        xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
        dims = (round(max(xs) - min(xs), 2), round(max(ys) - min(ys), 2), round(max(zs) - min(zs), 2))

    rows.append((len(counts), prim.GetName(), dc_s, mpath, mdiff, dims))

rows.sort(reverse=True)
print(f"  total meshes: {len(rows)}")
for tris, name, dc_s, mpath, mdiff, dims in rows:
    print(f"  [{tris:5d} tris] {name}")
    print(f"        size={dims}  displayColor={dc_s}")
    print(f"        mat={mpath}  matDiffuse={mdiff}")

print("\nDONE")
app.close()
