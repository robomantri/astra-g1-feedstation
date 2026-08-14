import asyncio
import math

import carb
import omni.kit.app
import omni.usd

USD_PATH = (
    "/home/kasper/Desktop/h12/office/v.20260801/USD_&_STL/"
    "20260087 - ASTRA SWEETS - Robot handling of bins at feed stations - "
    "Cloud scan and 3D model_usd_v.usdc"
)


def _place_camera(stage):
    """Point the perspective camera at the model's bounding box.

    frame_viewport_prims() silently no-ops here, so compute the box and set the
    camera explicitly instead.
    """
    from pxr import Gf, Usd, UsdGeom

    from omni.kit.viewport.utility import get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])

    # Bound the MESHES only. Measuring /root pulls in the Camera and Light prims,
    # which blow the box up to 56x38m and throw the camera far outside the room.
    # The real content is a single ~11 x 8 x 3.4m room (confirmed against the STL).
    room = Gf.Range3d()
    for prim in stage.Traverse():
        if prim.GetTypeName() == "Mesh":
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not box.IsEmpty():
                room.UnionWith(box)

    if room.IsEmpty():
        room = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    center, size = room.GetMidpoint(), room.GetSize()
    carb.log_warn(f"[autoopen] room center={center} size={size}")

    # sit back ~0.8x the bbox diagonal along a -Y/+Z three-quarter view (stage is Z-up)
    diagonal = math.sqrt(sum(v * v for v in size))
    offset = Gf.Vec3d(0.5, -1.0, 0.55).GetNormalized() * (diagonal * 0.8)
    eye = Gf.Vec3d(center[0], center[1], center[2]) + offset

    state = ViewportCameraState("/OmniverseKit_Persp", get_active_viewport())
    state.set_position_world(eye, True)
    state.set_target_world(Gf.Vec3d(center[0], center[1], center[2]), True)
    carb.log_warn(f"[autoopen] camera placed at {eye} looking at {center}")


def _fix_lighting(stage):
    """The export ships an unlit scene: a DomeLight at intensity 1.0 pointed at a
    near-black EXR, plus a 10cm sphere light for a 56x38m floor. Nothing renders.

    Only touches the in-memory stage — the .usdc on disk is never written.
    """
    from pxr import UsdLux

    dome = UsdLux.DomeLight(stage.GetPrimAtPath("/root/env_light"))
    if dome:
        dome.GetIntensityAttr().Set(1000.0)
        # drop the black environment map so the plain white color is used
        dome.GetTextureFileAttr().Set("")
        carb.log_warn("[autoopen] dome light raised to 1000, black EXR cleared")

    sphere = UsdLux.SphereLight(stage.GetPrimAtPath("/root/Light/Light"))
    if sphere:
        sphere.GetIntensityAttr().Set(50000.0)
        carb.log_warn("[autoopen] sphere light raised to 50000")


async def _open_when_ready():
    # let the app finish standing up its viewport/renderer before touching the stage
    for _ in range(20):
        await omni.kit.app.get_app().next_update_async()

    ctx = omni.usd.get_context()
    carb.log_warn(f"[autoopen] opening {USD_PATH}")
    ok, err = await ctx.open_stage_async(USD_PATH)
    if not ok:
        carb.log_error(f"[autoopen] failed to open stage: {err}")
        return
    carb.log_warn("[autoopen] stage opened OK")

    # the scan sits ~30m off origin, so the default camera misses it entirely
    for _ in range(60):
        await omni.kit.app.get_app().next_update_async()

    try:
        _fix_lighting(ctx.get_stage())
    except Exception as exc:
        carb.log_warn(f"[autoopen] could not fix lighting: {exc}")

    try:
        _place_camera(ctx.get_stage())
    except Exception as exc:  # framing is a convenience, never fail the load over it
        carb.log_warn(f"[autoopen] could not place camera: {exc}")


asyncio.ensure_future(_open_when_ready())
