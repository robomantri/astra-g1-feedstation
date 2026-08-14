"""Open the assembled ASTRA scene and render a view of the G1 at the station.

  OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim6_venv/bin/isaacsim isaacsim.exp.full \
    --exec ~/render_astra_view.py
"""
import asyncio
import carb
import omni.kit.app
import omni.usd

SCENE = "/home/kasper/Desktop/h12/astra_station_sim.usd"
SHOT = "/home/kasper/Desktop/h12/astra_g1_view.png"


async def _run():
    app = omni.kit.app.get_app()
    for _ in range(20):
        await app.next_update_async()

    ctx = omni.usd.get_context()
    ok, err = await ctx.open_stage_async(SCENE)
    if not ok:
        carb.log_error(f"[view] open failed: {err}")
        return
    for _ in range(120):
        await app.next_update_async()

    from pxr import Gf, Usd, UsdGeom
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    stage = ctx.get_stage()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    g1 = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/G1")).ComputeAlignedRange()
    c = g1.GetMidpoint()
    carb.log_warn(f"[view] G1 z {g1.GetMin()[2]:.3f}..{g1.GetMax()[2]:.3f}")

    # stand off ~3.5m, slightly above waist height, looking at the robot
    eye = Gf.Vec3d(c[0] + 2.2, c[1] + 2.6, 1.35)
    st = ViewportCameraState("/OmniverseKit_Persp", get_active_viewport())
    st.set_position_world(eye, True)
    st.set_target_world(Gf.Vec3d(c[0], c[1], 0.65), True)

    vp = get_active_viewport()
    vp.resolution = (1920, 1080)
    for _ in range(60):
        await app.next_update_async()
    capture_viewport_to_file(vp, SHOT)
    for _ in range(60):
        await app.next_update_async()
    carb.log_warn(f"[view] captured -> {SHOT}")


asyncio.ensure_future(_run())
