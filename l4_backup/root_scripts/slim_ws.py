from pxr import Usd
import shutil
SRC = "/root/astra_workspace/astra_workspace.usd"
OUT = "/root/astra_workspace/astra_workspace_1bay.usd"
shutil.copy(SRC, OUT)
s = Usd.Stage.Open(OUT)

# Collect every path AND name up front -- RemovePrim expires the prim handle,
# so reading .GetName() afterwards raises "expired prim".
targets = []
for name in ("Station2", "Station3"):
    p = s.GetPrimAtPath(f"/World/{name}")
    if p and p.IsValid():
        targets.append((str(p.GetPath()), name))
wp = s.GetPrimAtPath("/World/WallPanels")
if wp and wp.IsValid():
    for ch in wp.GetChildren():
        if ch.GetName().endswith(("_Station2", "_Station3")):
            targets.append((str(ch.GetPath()), ch.GetName()))

for path, _name in targets:
    s.RemovePrim(path)
s.GetRootLayer().Save()

chk = Usd.Stage.Open(OUT)
print("removed:", [n for _p, n in targets])
print("/World now:", [c.GetName() for c in chk.GetPrimAtPath("/World").GetChildren()])
print("panels now:", [c.GetName() for c in chk.GetPrimAtPath("/World/WallPanels").GetChildren()])
