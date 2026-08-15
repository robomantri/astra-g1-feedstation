"""Measure the scan's real geometry extent and shoot an overview of the whole room.

Two questions:
  1. Where is the geometry actually located? (BBoxCache over the pseudo-root
     includes lights/cameras, which can skew it -- measure meshes only.)
  2. What is actually in this file? Render the whole room and look.
"""
import numpy as np
import torch
from isaaclab.app import AppLauncher

app = AppLauncher(headless=True, enable_cameras=True).app

import isaaclab.sim as sim_utils
from isaaclab.sensors import Camera, CameraCfg
from pxr import Usd, UsdGeom, Gf

USD = "/root/coordex/source/coordex/coordex/assets/office.usdc"

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 60, device="cpu"))

# --- Spawn the scan untranslated so measurements are in its own frame.
cfg = sim_utils.UsdFileCfg(usd_path=USD)
cfg.func("/World/office", cfg, translation=(0.0, 0.0, 0.0))

sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.95)).func(
    "/World/dome", sim_utils.DomeLightCfg(intensity=2500.0, color=(0.9, 0.9, 0.95))
)
sim_utils.DistantLightCfg(intensity=2000.0).func("/World/key", sim_utils.DistantLightCfg(intensity=2000.0))

stage = sim.stage
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

# --- Mesh-only extent.
mins = np.array([1e9, 1e9, 1e9])
maxs = np.array([-1e9, -1e9, -1e9])
mesh_info = []
for prim in stage.Traverse():
    if not prim.IsA(UsdGeom.Mesh):
        continue
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        continue
    lo, hi = np.array(r.GetMin()), np.array(r.GetMax())
    mins = np.minimum(mins, lo)
    maxs = np.maximum(maxs, hi)
    mesh_info.append((str(prim.GetPath()), lo, hi, float(np.prod(hi - lo))))

print("=" * 72)
print("MESH-ONLY EXTENT (scan's own frame)")
print(f"  min  ({mins[0]:8.3f}, {mins[1]:8.3f}, {mins[2]:8.3f})")
print(f"  max  ({maxs[0]:8.3f}, {maxs[1]:8.3f}, {maxs[2]:8.3f})")
print(f"  size ({maxs[0]-mins[0]:8.3f}, {maxs[1]-mins[1]:8.3f}, {maxs[2]-mins[2]:8.3f})")
ctr = (mins + maxs) / 2
print(f"  centre ({ctr[0]:8.3f}, {ctr[1]:8.3f}, {ctr[2]:8.3f})")
print(f"  floor z = {mins[2]:.3f}")

print("\n--- 12 LARGEST MESHES (bbox volume) ---")
for path, lo, hi, vol in sorted(mesh_info, key=lambda t: -t[3])[:12]:
    c = (lo + hi) / 2
    print(f"  vol={vol:9.1f}  centre=({c[0]:7.2f},{c[1]:7.2f},{c[2]:6.2f})  "
          f"size=({hi[0]-lo[0]:5.2f},{hi[1]-lo[1]:5.2f},{hi[2]-lo[2]:5.2f})  {path[-42:]}")

# --- Overview camera: pull back along the room diagonal, look at the centre.
span = float(np.linalg.norm(maxs[:2] - mins[:2]))
eye = Gf.Vec3d(float(ctr[0] + span * 0.42), float(ctr[1] - span * 0.42), float(mins[2] + span * 0.38))
cam_cfg = CameraCfg(
    prim_path="/World/survey_cam",
    height=1080,
    width=1920,
    data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.1, 500.0)),
)
cam = Camera(cam_cfg)
sim.reset()
cam.set_world_poses_from_view(
    eyes=torch.tensor([[eye[0], eye[1], eye[2]]], dtype=torch.float32, device=cam.device),
    targets=torch.tensor([[float(ctr[0]), float(ctr[1]), float(mins[2] + 1.5)]],
                         dtype=torch.float32, device=cam.device),
)
print(f"\ncamera eye    ({eye[0]:.2f}, {eye[1]:.2f}, {eye[2]:.2f})")
print(f"camera target ({ctr[0]:.2f}, {ctr[1]:.2f}, {mins[2]+1.5:.2f})")

for _ in range(30):
    sim.step()
    cam.update(sim.get_physics_dt())

import imageio.v3 as iio
rgb = cam.data.output["rgb"][0].cpu().numpy()
if rgb.shape[-1] == 4:
    rgb = rgb[..., :3]
iio.imwrite("/root/scene_overview.png", rgb.astype(np.uint8))
print("\nwrote /root/scene_overview.png")
app.close()
