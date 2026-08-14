"""Recolour the ConveyorBelt_A08 frame to safety yellow and render proof shots.

  OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim6_venv/bin/isaacsim isaacsim.exp.full \
    --exec /home/kasper/Desktop/h12/scripts/conveyor_yellow_test.py
"""
import asyncio
import carb
import omni.kit.app
import omni.usd

ASSET = "/home/kasper/Desktop/h12/assets/Isaac/Props/Conveyors/ConveyorBelt_A08.usd"
SHOT_BEFORE = "/home/kasper/Desktop/h12/scratch_conveyor_before.png"
SHOT_AFTER = "/home/kasper/Desktop/h12/scratch_conveyor_yellow.png"


# ---------------------------------------------------------------- the snippet
SAFETY_YELLOW = (0.98, 0.75, 0.02)

FRAME_MATERIALS = (
    "MetalPainted_Blue_Glossy_A",   # main frame / side rails / legs shell (59.9% of body)
    "MetalPainted_Gray_Glossy_A",   # grey painted details on the frame
    "Metal_Glossy_A",               # support legs
    "Metal_Rough_A",                # rough metal frame details
    "Steel_A",                      # bolts + fixing bodies
)


def recolour_conveyor_frame(stage, asset_prim_path, rgb=SAFETY_YELLOW,
                            material_names=FRAME_MATERIALS):
    """Override the diffuse tint of the conveyor's frame materials to `rgb`.

    `asset_prim_path` is the prim the ConveyorBelt_A08.usd reference sits on,
    e.g. "/World/Conveyor/asset".  Returns the list of shader prim paths touched.
    """
    from pxr import Sdf, UsdShade

    root = stage.GetPrimAtPath(asset_prim_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"no prim at {asset_prim_path}")

    # instanceable prims cannot carry overrides on their descendants
    if root.IsInstanceable():
        root.SetInstanceable(False)

    looks = stage.GetPrimAtPath(f"{asset_prim_path}/Looks")
    if not looks or not looks.IsValid():
        raise RuntimeError(f"no Looks scope under {asset_prim_path}")

    touched = []
    for name in material_names:
        mat_prim = stage.GetPrimAtPath(f"{looks.GetPath()}/{name}")
        if not mat_prim or not mat_prim.IsValid():
            carb.log_warn(f"[yellow] material {name} not found, skipping")
            continue
        if mat_prim.IsInstanceable():
            mat_prim.SetInstanceable(False)
        mat = UsdShade.Material(mat_prim)
        # the MDL surface shader is the material's single Shader child
        shaders = [c for c in mat_prim.GetChildren() if c.IsA(UsdShade.Shader)]
        src = mat.ComputeSurfaceSource("mdl")[0]
        if src:
            shaders = [src.GetPrim()]
        for sh_prim in shaders:
            sh = UsdShade.Shader(sh_prim)
            sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(rgb)
            # OmniPBR ignores diffuse_color_constant when a diffuse_texture is
            # bound, but set it too for the untextured library materials.
            sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(rgb)
            touched.append(sh_prim.GetPath().pathString)
    return touched
# ------------------------------------------------------------ end of snippet


async def _wait(n):
    app = omni.kit.app.get_app()
    for _ in range(n):
        await app.next_update_async()


async def _run():
    await _wait(20)

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    ctx = omni.usd.get_context()
    ok, err = await ctx.new_stage_async()
    if not ok:
        carb.log_error(f"[yellow] new stage failed: {err}")
        return
    await _wait(20)

    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    # lighting
    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(900.0)
    dist = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    dist.CreateIntensityAttr(2500.0)
    dist.CreateAngleAttr(1.5)
    UsdGeom.Xformable(dist).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 130.0))

    # floor
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    fx = UsdGeom.Xformable(floor)
    fx.AddTranslateOp().Set(Gf.Vec3d(-1.36, 0.0, -0.05))
    fx.AddScaleOp().Set(Gf.Vec3f(14.0, 14.0, 0.1))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.30, 0.30, 0.32)])

    # reference the conveyor onto a CHILD of a wrapper xform
    wrapper = UsdGeom.Xform.Define(stage, "/World/Conveyor")
    asset_prim = stage.DefinePrim("/World/Conveyor/asset")
    asset_prim.GetReferences().AddReference(ASSET)
    await _wait(120)

    # ---- report what actually loaded -------------------------------------
    rng = Usd.PrimRange(asset_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
    meshes = [p for p in rng if p.IsA(UsdGeom.Mesh)]
    carb.log_warn(f"[yellow] referenced {ASSET}")
    carb.log_warn(f"[yellow] asset prim valid={asset_prim.IsValid()} "
                  f"instanceable={asset_prim.IsInstanceable()} meshes={len(meshes)}")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    rr = cache.ComputeWorldBound(asset_prim).ComputeAlignedRange()
    carb.log_warn(f"[yellow] bbox min={rr.GetMin()} max={rr.GetMax()} size={rr.GetSize()}")

    body = stage.GetPrimAtPath("/World/Conveyor/asset/SM_ConveyorBelt_A08_02")
    ss = UsdGeom.Subset(stage.GetPrimAtPath(
        "/World/Conveyor/asset/SM_ConveyorBelt_A08_02/M_ConveyorBelt_A01_Body"))
    bm = UsdShade.MaterialBindingAPI(ss.GetPrim()).ComputeBoundMaterial()[0]
    carb.log_warn(f"[yellow] body subset bound material = {bm.GetPath() if bm else None}")

    # ---- camera (explicit USD camera == deterministic framing) -----------
    c = rr.GetMidpoint()
    eye = Gf.Vec3d(c[0] + 3.2, c[1] - 4.4, 2.6)
    tgt = Gf.Vec3d(c[0], c[1], 0.5)
    cam = UsdGeom.Camera.Define(stage, "/World/ShotCam")
    cam.CreateFocalLengthAttr(24.0)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateVerticalApertureAttr(20.955 * 1000.0 / 1600.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
    m = Gf.Matrix4d().SetLookAt(eye, tgt, Gf.Vec3d(0, 0, 1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(m)

    vp = get_active_viewport()
    vp.resolution = (1600, 1000)
    vp.camera_path = "/World/ShotCam"
    await _wait(150)
    carb.log_warn(f"[yellow] viewport cam={vp.camera_path} res={vp.resolution} eye={eye} tgt={tgt}")

    capture_viewport_to_file(vp, SHOT_BEFORE)
    await _wait(90)
    carb.log_warn(f"[yellow] captured BEFORE -> {SHOT_BEFORE}")

    # ---- apply the recolour ---------------------------------------------
    try:
        touched = recolour_conveyor_frame(stage, "/World/Conveyor/asset")
        carb.log_warn(f"[yellow] recoloured {len(touched)} shaders: {touched}")
    except Exception as exc:  # noqa: BLE001
        carb.log_error(f"[yellow] recolour FAILED: {exc}")
        return

    # read back
    for p in touched:
        a = stage.GetPrimAtPath(p).GetAttribute("inputs:diffuse_tint")
        carb.log_warn(f"[yellow] readback {p} diffuse_tint={a.Get()} authored={a.IsAuthored()}")

    await _wait(200)
    capture_viewport_to_file(vp, SHOT_AFTER)
    await _wait(120)
    carb.log_warn(f"[yellow] captured AFTER -> {SHOT_AFTER}")
    carb.log_warn("[yellow] DONE")


asyncio.ensure_future(_run())
