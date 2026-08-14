"""Render the conveyor + blue crates from the saved scene."""
import asyncio
import carb
import omni.kit.app
import omni.usd

SCENE = "/home/kasper/Desktop/h12/astra_station_sim.usd"
SHOT = "/home/kasper/Desktop/h12/astra_conveyor_view.png"


async def _run():
    app = omni.kit.app.get_app()
    for _ in range(20):
        await app.next_update_async()
    ctx = omni.usd.get_context()
    ok, err = await ctx.open_stage_async(SCENE)
    if not ok:
        carb.log_error(f"[view] {err}")
        return
    for _ in range(140):
        await app.next_update_async()

    from pxr import Gf, Usd, UsdGeom
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    stage = ctx.get_stage()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    r = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/Conveyor")).ComputeAlignedRange()
    c = r.GetMidpoint()
    carb.log_warn(f"[view] conveyor centre {c}")

    # approach from the +Y side (open floor) so the station does not occlude it
    eye = Gf.Vec3d(c[0] + 1.15, c[1] + 2.45, 1.95)
    st = ViewportCameraState("/OmniverseKit_Persp", get_active_viewport())
    st.set_position_world(eye, True)
    st.set_target_world(Gf.Vec3d(c[0], c[1], 0.80), True)

    vp = get_active_viewport()
    vp.resolution = (1920, 1080)
    for _ in range(70):
        await app.next_update_async()
    capture_viewport_to_file(vp, SHOT)
    for _ in range(70):
        await app.next_update_async()
    carb.log_warn(f"[view] captured -> {SHOT}")


asyncio.ensure_future(_run())
