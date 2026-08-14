"""Load the ASTRA SWEETS E57 laser scan (12.06M coloured points) into Isaac Sim.

Isaac Sim reads E57 natively via omni.usd.fileformat.e57, so the file can be
referenced straight onto a prim -- no conversion step.
"""

import asyncio
import math

import carb
import omni.kit.app
import omni.usd

E57_PATH = (
    "/home/kasper/Desktop/h12/office/v.20260801/E57/"
    "AstraSweets_RCP_Inkapstation.e57"
)


def _light_scene(stage):
    """A bare stage has no lighting; the scan renders black without one."""
    from pxr import Sdf, UsdLux

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/scan_dome"))
    dome.GetIntensityAttr().Set(1000.0)
    carb.log_warn("[scan] dome light added at intensity 1000")


def _frame(stage, prim_path):
    from pxr import Gf, Usd, UsdGeom

    from omni.kit.viewport.utility import get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(prim_path)).ComputeAlignedRange()
    if rng.IsEmpty():
        carb.log_warn("[scan] bbox empty, leaving camera alone")
        return
    center, size = rng.GetMidpoint(), rng.GetSize()
    carb.log_warn(f"[scan] bbox center={center} size={size}")

    # A laser scan only captures surfaces the scanner could see, i.e. the INSIDE
    # of the room. Viewed from outside you just get the blank back faces of the
    # walls, so put the camera inside, near one corner at standing height,
    # looking across at the opposite corner.
    eye = Gf.Vec3d(
        center[0] - size[0] * 0.40,
        center[1] - size[1] * 0.38,
        center[2] + size[2] * 0.30,
    )
    look = Gf.Vec3d(
        center[0] + size[0] * 0.30,
        center[1] + size[1] * 0.28,
        center[2] - size[2] * 0.10,
    )

    state = ViewportCameraState("/OmniverseKit_Persp", get_active_viewport())
    state.set_position_world(eye, True)
    state.set_target_world(look, True)
    carb.log_warn(f"[scan] camera inside room at {eye} looking at {look}")


async def _run():
    app = omni.kit.app.get_app()
    for _ in range(20):
        await app.next_update_async()

    ctx = omni.usd.get_context()

    # The e57 fileformat plugin writes its content to an absolute /data3D path,
    # so it cannot be referenced onto a prim -- it has to be opened as the stage.
    carb.log_warn(f"[scan] opening {E57_PATH}")
    ok, err = await ctx.open_stage_async(E57_PATH)
    if not ok:
        carb.log_error(f"[scan] open failed: {err}")
        return
    carb.log_warn("[scan] stage opened")

    stage = ctx.get_stage()

    # 12M points takes a while to stream in; give it real time before framing
    for _ in range(240):
        await app.next_update_async()

    kinds = {}
    for p in stage.Traverse():
        kinds[p.GetTypeName()] = kinds.get(p.GetTypeName(), 0) + 1
    carb.log_warn(f"[scan] prim types after load: {kinds}")

    _light_scene(stage)
    for _ in range(30):
        await app.next_update_async()

    # frame whatever the plugin actually created (usually /data3D)
    target = "/data3D" if stage.GetPrimAtPath("/data3D") else str(
        stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else "/"
    )
    carb.log_warn(f"[scan] framing {target}")
    _frame(stage, target)
    carb.log_warn("[scan] done")


asyncio.ensure_future(_run())
