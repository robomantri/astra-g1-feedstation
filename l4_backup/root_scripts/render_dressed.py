"""Render the ASTRA scan + added dressing, composed like the reference render."""
import os

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
from isaaclab.app import AppLauncher

app = AppLauncher({"headless": True, "enable_cameras": True}).app

import torch
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.sensors import Camera, CameraCfg
from pxr import UsdGeom
import omni.usd
import imageio.v3 as iio

ASSETS = "/root/coordex/source/coordex/coordex/assets"
OFFICE_POS = (51.5357 - 4.0, -30.5250 - 2.0, 2.65648 + 0.001)
OUT = "/root/dressed_renders"
os.makedirs(OUT, exist_ok=True)

sim = SimulationContext(SimulationCfg(dt=1 / 60.0, device="cuda:0"))

# scan
c = sim_utils.UsdFileCfg(usd_path=f"{ASSETS}/office.usdc")
c.func("/World/office", c, translation=OFFICE_POS)
# dressing (already authored in world coordinates)
d = sim_utils.UsdFileCfg(usd_path=f"{ASSETS}/dressing.usda")
d.func("/World/dressing", d, translation=(0.0, 0.0, 0.0))

stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.GetTypeName() in ("DomeLight", "DistantLight", "SphereLight", "RectLight"):
        if str(prim.GetPath()).startswith("/World/office"):
            UsdGeom.Imageable(prim).MakeInvisible()
            a = prim.GetAttribute("inputs:intensity")
            if a:
                a.Set(0.0)

# lighting -- 2200/900/45000 gave mean luma 215, 1100/320/18000 still washed the
# reds to pink. The scan's materials are near-white and roughness 1.0, so they
# blow out early; keep the key low and let the fills do the shaping.
key = sim_utils.DistantLightCfg(color=(1.0, 0.97, 0.92), intensity=420.0, angle=2.5)
key.func("/World/key", key, translation=(0, 0, 6), orientation=(0.86, 0.28, -0.33, 0.0))
dome = sim_utils.DomeLightCfg(color=(0.48, 0.53, 0.62), intensity=110.0)
dome.func("/World/dome", dome)
f1 = sim_utils.SphereLightCfg(color=(1.0, 0.97, 0.93), intensity=6000.0, radius=0.8)
f1.func("/World/fill1", f1, translation=(1.5, 1.5, 3.1))
f2 = sim_utils.SphereLightCfg(color=(1.0, 0.97, 0.93), intensity=5000.0, radius=0.8)
f2.func("/World/fill2", f2, translation=(4.8, 3.6, 3.1))

cam_cfg = CameraCfg(
    prim_path="/World/cam", height=1080, width=1920, data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=19.0, focus_distance=7.0, horizontal_aperture=24.0,
        clipping_range=(0.05, 120.0)),
)
cam = Camera(cam_cfg)
sim.reset()

# the reference is an elevated ~30-degree 3/4 view; get above the guard rail so
# the bins and conveyor are not occluded by it
SHOTS = [
    ("A_reference_like", (-1.8, -1.5, 2.95), (3.2, 3.4, 0.70)),
    ("B_hopper_front",   (0.2, -1.8, 3.05), (4.3, 3.6, 0.60)),
    ("C_conveyor_side",  (1.2, -1.7, 2.70), (4.6, 3.2, 0.80)),
    ("D_west_wide",      (-2.8, 0.2, 2.90), (3.8, 3.8, 0.70)),
    ("E_high_wide",      (-2.2, -1.8, 3.30), (3.2, 3.2, 0.55)),
    ("F_robot_area",     (-2.6, -1.6, 1.90), (1.6, 1.2, 0.75)),
]

for name, eye, look in SHOTS:
    cam.set_world_poses_from_view(
        torch.tensor([eye], dtype=torch.float32, device=sim.device),
        torch.tensor([look], dtype=torch.float32, device=sim.device),
    )
    for _ in range(40):
        sim.step()
        cam.update(sim.get_physics_dt())
    rgb = cam.data.output["rgb"][0, ..., :3].detach().cpu().numpy().astype(np.uint8)
    iio.imwrite(f"{OUT}/{name}.png", rgb)
    print(f"  wrote {OUT}/{name}.png  mean_luma={rgb.mean():.1f}")

print("DONE")
app.close()
