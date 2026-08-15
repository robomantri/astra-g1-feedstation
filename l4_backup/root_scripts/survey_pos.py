"""World-space bbox of the scan's significant meshes, so props can be placed
against real coordinates instead of guesses."""
import os

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
from isaaclab.app import AppLauncher

app = AppLauncher({"headless": True}).app

import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext, SimulationCfg
from pxr import Usd, UsdGeom, Gf
import omni.usd

OFFICE_USD = "/root/coordex/source/coordex/coordex/assets/office.usdc"
OFFICE_POS = (51.5357 - 4.0, -30.5250 - 2.0, 2.65648 + 0.001)

sim = SimulationContext(SimulationCfg(dt=1 / 60.0, device="cuda:0"))
cfg = sim_utils.UsdFileCfg(usd_path=OFFICE_USD)
cfg.func("/World/office", cfg, translation=OFFICE_POS)
stage = omni.usd.get_context().get_stage()
sim.reset()

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

MATNAME = {}
for prim in stage.Traverse():
    if prim.GetTypeName() == "Material":
        from pxr import UsdShade
        for c in Usd.PrimRange(prim):
            if c.GetTypeName() == "Shader":
                v = UsdShade.Shader(c).GetInput("diffuseColor")
                if v and v.Get() is not None:
                    MATNAME[str(prim.GetPath())] = tuple(round(float(x), 2) for x in v.Get())

from pxr import UsdShade

rows = []
allmin = [1e9] * 3
allmax = [-1e9] * 3
for prim in stage.Traverse():
    if prim.GetTypeName() != "Mesh":
        continue
    if not str(prim.GetPath()).startswith("/World/office"):
        continue
    bb = cache.ComputeWorldBound(prim)
    r = bb.ComputeAlignedRange()
    if r.IsEmpty():
        continue
    mn, mx = r.GetMin(), r.GetMax()
    for i in range(3):
        allmin[i] = min(allmin[i], mn[i])
        allmax[i] = max(allmax[i], mx[i])
    size = [mx[i] - mn[i] for i in range(3)]
    vol = size[0] * size[1] * size[2]
    mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    mpath = str(mat.GetPath()) if mat else None
    rows.append((vol, prim.GetName(), [round(v, 2) for v in mn], [round(v, 2) for v in mx],
                 [round(v, 2) for v in size], MATNAME.get(mpath)))

print("\n=== SCAN WORLD EXTENT ===")
print(f"  min = {[round(v,3) for v in allmin]}")
print(f"  max = {[round(v,3) for v in allmax]}")
print(f"  size= {[round(allmax[i]-allmin[i],3) for i in range(3)]}")

rows.sort(reverse=True)
print("\n=== TOP 25 MESHES BY VOLUME (world coords) ===")
for vol, name, mn, mx, size, col in rows[:25]:
    print(f"  {name:12s} vol={vol:8.2f} size={size} min={mn} max={mx} diffuse={col}")

print("\n=== YELLOW (rail) MESHES ===")
for vol, name, mn, mx, size, col in rows:
    if col and col[0] > 0.9 and col[1] > 0.9 and col[2] < 0.2:
        print(f"  {name:12s} size={size} min={mn} max={mx}")

print("\nDONE")
app.close()
