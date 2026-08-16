"""Render a thumbnail per crate/conveyor asset, for picking.

One stage is reused for every asset: the item goes in at /World/Item, the
camera is fitted to its bounding box, a frame is captured, then the prim is
removed. Reloading the stage per asset would mean rebuilding the render
product each time, which is where this kind of script usually breaks.

  OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim6_venv/bin/python scripts/asset_contact_sheet.py
"""
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO, "asset_thumbs")
os.makedirs(OUT, exist_ok=True)

A = os.path.join(REPO, "assets")
ASSETS = [
    ("ConveyorBelt_A03", f"{A}/Isaac/Props/Conveyors/ConveyorBelt_A03.usd", "conveyor"),
    ("ConveyorBelt_A05", f"{A}/Isaac/Props/Conveyors/ConveyorBelt_A05.usd", "conveyor"),
    ("ConveyorBelt_A06", f"{A}/Isaac/Props/Conveyors/ConveyorBelt_A06.usd", "conveyor"),
    ("ConveyorBelt_A08", f"{A}/Isaac/Props/Conveyors/ConveyorBelt_A08.usd", "conveyor"),
    ("ConveyorBelt_A32", f"{A}/Isaac/Props/Conveyors/ConveyorBelt_A32.usd", "conveyor"),
    ("small_KLT", f"{A}/Isaac/Props/KLT_Bin/small_KLT.usd", "bin"),
    ("small_KLT_visual", f"{A}/Isaac/Props/KLT_Bin/small_KLT_visual.usd", "bin"),
    ("small_KLT_visual_collision",
     f"{A}/Isaac/Props/KLT_Bin/small_KLT_visual_collision.usd", "bin"),
    ("container_h20",
     f"{A}/Isaac/Props/PackingTable/props/container_h20/container_h20.usd", "container"),
    ("SM_Crate_A07_Yellow_01",
     f"{A}/Isaac/Props/PackingTable/props/SM_Crate_A07_Yellow_01/"
     "SM_Crate_A07_Yellow_01.usd", "crate"),
    ("euro_crate_600x400x120", f"{A}/crates/euro_crate_600x400x120.usd", "crate (ours)"),
    ("euro_crate_600x400x120_red", f"{A}/crates/euro_crate_600x400x120_red.usd",
     "crate (ours)"),
]

world = World(stage_units_in_meters=1.0)
stage = world.stage

dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Dome"))
dome.CreateIntensityAttr(1200.0)
key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Key"))
key.CreateIntensityAttr(2500.0)
UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-40.0, 0.0, 35.0))

# A render product on the interactive persp camera renders at whatever size the
# headless viewport happens to be (320x240 here, below DLSS's minimum) and the
# annotator never yields a frame. A dedicated camera prim driven by the
# replicator orchestrator renders at exactly the size asked for.
W, H = 640, 480
cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/ThumbCam"))
cam.CreateFocalLengthAttr(24.0)
cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(20.955 * H / W)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
cam_xf = UsdGeom.Xformable(cam.GetPrim())
cam_xf.ClearXformOpOrder()
cam_op = cam_xf.AddTransformOp()

rp = rep.create.render_product("/World/ThumbCam", (W, H))
annot = rep.AnnotatorRegistry.get_annotator("rgb")
annot.attach(rp)


def look_at(eye, target, up=Gf.Vec3d(0, 0, 1)):
    """Camera transform (inverse view matrix) for a look-at."""
    m = Gf.Matrix4d()
    m.SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), up)
    return m.GetInverse()

import imageio.v2 as imageio  # noqa: E402

bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
report = []

for name, path, kind in ASSETS:
    size = os.path.getsize(path) if os.path.exists(path) else -1
    if size <= 0:
        report.append((name, kind, "MISSING/EMPTY", None))
        print(f"[skip] {name}: {'absent' if size < 0 else '0 bytes'}")
        continue

    add_reference_to_stage(usd_path=path, prim_path="/World/Item")
    for _ in range(8):
        world.step(render=True)

    bbc.Clear()
    rng = bbc.ComputeWorldBound(stage.GetPrimAtPath("/World/Item")).ComputeAlignedRange()
    if rng.IsEmpty():
        report.append((name, kind, "no geometry", None))
        stage.RemovePrim("/World/Item")
        continue
    mn, mx = np.array(rng.GetMin()), np.array(rng.GetMax())
    ctr = (mn + mx) / 2.0
    span = float(np.max(mx - mn)) or 1.0
    d = span * 2.1
    cam_op.Set(look_at([ctr[0] + d * 0.72, ctr[1] - d * 0.72, ctr[2] + d * 0.5],
                       [float(ctr[0]), float(ctr[1]), float(ctr[2])]))

    # The annotator yields an empty buffer for the first frames, so keep
    # stepping the orchestrator until it is actually an image.
    img = None
    for _ in range(30):
        rep.orchestrator.step(rt_subframes=8, pause_timeline=False)
        raw = np.asarray(annot.get_data())
        if raw.ndim == 3 and raw.shape[0] > 1 and raw.shape[2] >= 3:
            img = raw[..., :3]
            break
    if img is None:
        report.append((name, kind, "no frame captured", None))
        print(f"[skip] {name}: annotator gave {np.asarray(annot.get_data()).shape}")
        stage.RemovePrim("/World/Item")
        continue
    f = os.path.join(OUT, f"{name}.png")
    imageio.imwrite(f, img)

    dims = (mx - mn)
    report.append((name, kind, f"{dims[0]:.2f} x {dims[1]:.2f} x {dims[2]:.2f} m", f))
    print(f"[ok]   {name}  {dims[0]:.2f}x{dims[1]:.2f}x{dims[2]:.2f} m")
    stage.RemovePrim("/World/Item")

with open(os.path.join(OUT, "inventory.txt"), "w") as fh:
    for name, kind, info, f in report:
        fh.write(f"{name}\t{kind}\t{info}\t{f or ''}\n")
print(f"\nwrote {sum(1 for r in report if r[3])} thumbnails to {OUT}")
simulation_app.close()
