"""Render the ASTRA SWEETS scan from several angles with its real materials.

The USD ships its own `env_light` dome bound to color_0C0C0C.exr = RGB(12,12,12),
which is effectively black -- it lights nothing. We disable it and light the room
ourselves, otherwise the 15 UsdPreviewSurface materials (yellow rails, green floor
markings, red extinguisher) never show.
"""
import os

os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"

from isaaclab.app import AppLauncher

app = AppLauncher({"headless": True, "enable_cameras": True}).app

import torch
import numpy as np
import isaaclab.sim as sim_utils
from isaaclab.sim import SimulationContext, SimulationCfg
from isaaclab.sensors import Camera, CameraCfg
from pxr import Usd, UsdGeom, UsdLux, Gf
import omni.usd
import imageio.v3 as iio

OFFICE_USD = "/root/coordex/source/coordex/coordex/assets/office.usdc"
OFFICE_POS = (51.5357 - 4.0, -30.5250 - 2.0, 2.65648 + 0.001)
OUT = "/root/scan_renders"
os.makedirs(OUT, exist_ok=True)

sim = SimulationContext(SimulationCfg(dt=1 / 60.0, device="cuda:0"))

# ---- office scan ----
cfg = sim_utils.UsdFileCfg(usd_path=OFFICE_USD)
cfg.func("/World/office", cfg, translation=OFFICE_POS)

stage = omni.usd.get_context().get_stage()

# ---- kill the near-black dome the scan ships with ----
for prim in stage.Traverse():
    if prim.GetTypeName() in ("DomeLight", "DistantLight", "SphereLight", "RectLight"):
        if str(prim.GetPath()).startswith("/World/office"):
            print(f"  disabling scan light: {prim.GetPath()} ({prim.GetTypeName()})")
            UsdGeom.Imageable(prim).MakeInvisible()
            intens = prim.GetAttribute("inputs:intensity")
            if intens:
                intens.Set(0.0)

# ---- our own lighting ----
key = sim_utils.DistantLightCfg(color=(1.0, 0.97, 0.92), intensity=2200.0, angle=1.5)
key.func("/World/key", key, translation=(0, 0, 6), orientation=(0.85, 0.30, -0.35, 0.0))

dome = sim_utils.DomeLightCfg(color=(0.55, 0.58, 0.65), intensity=900.0)
dome.func("/World/dome", dome)

# soft ceiling fill so the interior is not lit from one side only
fill = sim_utils.SphereLightCfg(color=(1.0, 0.98, 0.95), intensity=45000.0, radius=0.6)
fill.func("/World/fill", fill, translation=(1.5, 2.0, 3.2))

# ---- camera ----
cam_cfg = CameraCfg(
    prim_path="/World/cam",
    height=900,
    width=1600,
    data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(
        focal_length=18.0, focus_distance=6.0, horizontal_aperture=24.0, clipping_range=(0.05, 100.0)
    ),
)
cam = Camera(cam_cfg)

sim.reset()

# room in world coords is X[-4, 7.08]  Y[-2, 5.97]  Z[0, 3.41]
SHOTS = [
    ("01_sw_high", (-3.2, -1.2, 2.9), (2.5, 2.5, 1.0)),
    ("02_ref_3quarter", (-2.5, -1.5, 2.6), (2.0, 2.0, 0.9)),
    ("03_se_corner", (6.2, -1.4, 2.9), (0.5, 2.5, 1.0)),
    ("04_nw_corner", (-3.2, 5.2, 2.9), (2.5, 1.0, 1.0)),
    ("05_wide_centre", (-1.0, -1.6, 3.25), (3.0, 3.0, 0.8)),
    ("06_hopper_close", (0.5, -1.0, 1.9), (3.2, 2.6, 1.0)),
]

for name, eye, look in SHOTS:
    cam.set_world_poses_from_view(
        torch.tensor([eye], dtype=torch.float32, device=sim.device),
        torch.tensor([look], dtype=torch.float32, device=sim.device),
    )
    # let the RTX denoiser settle before grabbing the frame
    for _ in range(35):
        sim.step()
        cam.update(sim.get_physics_dt())

    rgb = cam.data.output["rgb"][0, ..., :3].detach().cpu().numpy().astype(np.uint8)
    path = f"{OUT}/{name}.png"
    iio.imwrite(path, rgb)
    print(f"  wrote {path}  mean_luma={rgb.mean():.1f}")

print("DONE")
app.close()
