"""Print a coordinate map of the ASTRA scene: every notable object's position and size.

  ~/isaacsim6_venv/bin/python ~/Desktop/h12/scripts/where.py
"""
from pxr import Usd, UsdGeom

SCENE = "/home/kasper/Desktop/h12/astra_station_sim.usd"

stage = Usd.Stage.Open(SCENE)
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])


def box(path):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return None
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return None if r.IsEmpty() else r


print(f"{'object':30s} {'centre (x,y,z)':>26s}   {'size (w,d,h)':>22s}")
print("-" * 84)
for path in [
    "/World/Station",
    "/World/GroundPlane",
    "/World/WallPanels/panel_left",
    "/World/WallPanels/panel_right",
    "/World/G1",
    "/World/Bins",
]:
    r = box(path)
    if not r:
        continue
    c, s = r.GetMidpoint(), r.GetSize()
    name = path.split("/World/")[-1]
    print(f"{name:30s} ({c[0]:7.2f},{c[1]:7.2f},{c[2]:6.2f})   ({s[0]:6.2f},{s[1]:6.2f},{s[2]:6.2f})")

print()
print("individual bins:")
bins = stage.GetPrimAtPath("/World/Bins")
if bins:
    for b in bins.GetChildren():
        r = box(str(b.GetPath()))
        if r:
            c = r.GetMidpoint()
            print(f"  {b.GetName():14s} ({c[0]:6.2f},{c[1]:6.2f},{c[2]:5.2f})")

print()
print("station meshes bigger than 1.5m (landmarks you can aim at):")
big = []
for prim in stage.Traverse():
    if prim.GetTypeName() != "Mesh":
        continue
    if not str(prim.GetPath()).startswith("/World/Station"):
        continue
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        continue
    s = r.GetSize()
    if max(s) > 1.5:
        c = r.GetMidpoint()
        big.append((-max(s), prim.GetName(), c, s))
big.sort()
for _, name, c, s in big[:12]:
    print(f"  {name:12s} centre=({c[0]:6.2f},{c[1]:6.2f},{c[2]:5.2f})  size=({s[0]:5.2f},{s[1]:5.2f},{s[2]:5.2f})")

print()
print("frame: origin = room corner at floor level; X 0..11.08, Y 0..7.97 (0 = back wall), Z 0 = floor")
