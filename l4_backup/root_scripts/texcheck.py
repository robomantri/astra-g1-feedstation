"""Definitive check: is there ANY loadable texture in office.usdc, and does
Material17's Image_Texture actually point at a file?"""
import os

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
from isaaclab.app import AppLauncher

app = AppLauncher({"headless": True}).app

from pxr import Usd, UsdGeom, UsdShade, Sdf

USD = "/root/coordex/source/coordex/coordex/assets/office.usdc"
stage = Usd.Stage.Open(USD)

print("=" * 78)
print("EVERY Shader PRIM IN THE FILE")
print("=" * 78)
for prim in stage.Traverse():
    if prim.GetTypeName() != "Shader":
        continue
    sh = UsdShade.Shader(prim)
    sid = sh.GetIdAttr().Get() if sh.GetIdAttr() else None
    print(f"\n  {prim.GetPath()}   info:id={sid}")
    for inp in sh.GetInputs():
        name = inp.GetBaseName()
        attr = inp.GetAttr()
        authored = attr.HasAuthoredValue()
        val = attr.Get()
        conn = inp.GetConnectedSource()
        conn_s = f" <- {conn[0].GetPath()}.{conn[1]}" if conn else ""
        print(f"      inputs:{name:16s} type={attr.GetTypeName()} "
              f"authored={authored} value={val!r}{conn_s}")
    for outp in sh.GetOutputs():
        print(f"      outputs:{outp.GetBaseName()}")

print("\n" + "=" * 78)
print("EVERY ASSET-TYPED ATTRIBUTE (authored or not), whole stage")
print("=" * 78)
found_any = False
for prim in stage.Traverse():
    for attr in prim.GetAttributes():
        if attr.GetTypeName() == Sdf.ValueTypeNames.Asset:
            found_any = True
            v = attr.Get()
            print(f"  {prim.GetPath()}.{attr.GetName()}")
            print(f"      authored={attr.HasAuthoredValue()}  value={v!r}")
            if v:
                print(f"      raw='{v.path}'  resolved='{v.resolvedPath}'  "
                      f"EXISTS={bool(v.resolvedPath) and os.path.exists(v.resolvedPath)}")
if not found_any:
    print("  (none)")

print("\n" + "=" * 78)
print("MATERIAL17 SUBTREE, RAW")
print("=" * 78)
m17 = stage.GetPrimAtPath("/root/_materials/Material17")
print(f"  exists={bool(m17)}")
if m17:
    for p in Usd.PrimRange(m17):
        print(f"    {p.GetPath()}  type={p.GetTypeName()}")
        for a in p.GetAttributes():
            if a.HasAuthoredValue():
                print(f"        {a.GetName()} = {a.Get()!r}")

print("\n" + "=" * 78)
print("WHICH MESHES BIND MATERIAL17 / DO MESHES HAVE UVs?")
print("=" * 78)
bound17 = []
uv_meshes = 0
total = 0
uv_names = set()
for prim in stage.Traverse():
    if prim.GetTypeName() != "Mesh":
        continue
    total += 1
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if mat and "Material17" in str(mat.GetPath()):
        bound17.append(str(prim.GetPath()))
    pv = UsdGeom.PrimvarsAPI(prim)
    for v in pv.GetPrimvars():
        n = v.GetPrimvarName()
        if v.GetTypeName() in (Sdf.ValueTypeNames.TexCoord2fArray,
                               Sdf.ValueTypeNames.Float2Array):
            uv_names.add(n)
            uv_meshes += 1
            break
print(f"  meshes total                 : {total}")
print(f"  meshes bound to Material17   : {len(bound17)}  {bound17[:5]}")
print(f"  meshes carrying UV primvars  : {uv_meshes}")
print(f"  UV primvar names seen        : {sorted(uv_names)}")

print("\nDONE")
app.close()
