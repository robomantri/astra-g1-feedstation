"""Step 1 of the OmniContact -> Isaac Sim port.

Goal: get the G1 MJCF into Isaac Sim and report the joint ordering, so we can
build the isaac<->lab permutation the ONNX policy needs.

The policy's obs uses "lab" (IsaacLab) joint order. The MuJoCo deploy code maps
mujoco->lab via a hardcoded mj2lab index list. Isaac Sim will report its own
order, so we rebuild the permutation BY NAME rather than trusting any list.
"""
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.kit.commands  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

MJCF = "/root/OmniContact_sim2sim/g1_description/g1_29dof.xml"
OUT = "/root/g1_29dof_from_mjcf.usd"

# --- import MJCF -----------------------------------------------------------
status, plan = False, None
for cmd in ("MJCFCreateImportConfig", "MJCFCreateAsset"):
    pass

import_cfg = None
try:
    _, import_cfg = omni.kit.commands.execute("MJCFCreateImportConfig")
    import_cfg.set_fix_base(False)
    import_cfg.set_import_inertia_tensor(True)
    import_cfg.set_make_default_prim(True)
    print("[step1] got MJCF import config")
except Exception as e:
    print(f"[step1] MJCFCreateImportConfig failed: {e}")

try:
    omni.kit.commands.execute(
        "MJCFCreateAsset",
        mjcf_path=MJCF,
        import_config=import_cfg,
        prim_path="/World/G1",
        dest_path=OUT,
    )
    print(f"[step1] imported -> {OUT}")
except Exception as e:
    print(f"[step1] MJCFCreateAsset failed: {e}")
    simulation_app.close()
    raise SystemExit(1)

# --- report joint order ----------------------------------------------------
stage = Usd.Stage.Open(OUT) if OUT else omni.usd.get_context().get_stage()
joints = []
for prim in stage.Traverse():
    if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
        joints.append(prim.GetName())
print(f"[step1] found {len(joints)} joints")
for i, n in enumerate(joints):
    print(f"   {i:2d}  {n}")

simulation_app.close()
