"""Build a sim-ready ASTRA SWEETS feed-station scene in Isaac Sim.

Base geometry is the delivered ArchiCAD model (the station machine, railings,
walls). On top of that this adds a physics scene, static colliders for the
station, a Unitree G1, a conveyor, and Euro crates as dynamic rigid bodies.

Run with:
  OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim6_venv/bin/isaacsim isaacsim.exp.full \
    --exec ~/build_astra_scene.py

Writes the assembled stage to OUT_PATH so it can be reopened without rebuilding.
"""

import asyncio

import carb
import omni.kit.app
import omni.usd

OFFICE_USD = (
    "/home/kasper/Desktop/h12/office/v.20260801/USD_&_STL/"
    "20260087 - ASTRA SWEETS - Robot handling of bins at feed stations - "
    "Cloud scan and 3D model_usd_v.usdc"
)
OUT_PATH = "/home/kasper/Desktop/h12/astra_station_sim.usd"
SHOT_PATH = "/home/kasper/Desktop/h12/astra_scene_render.png"

# Assets mirrored locally from the Isaac 6.0 S3 library -- referencing the remote
# URLs blocks the build for minutes and re-fetches on every load.
ASSETS = "/home/kasper/Desktop/h12/assets"
G1_USD = f"{ASSETS}/Isaac/Robots/Unitree/G1/g1.usd"
CONVEYOR_USD = f"{ASSETS}/Isaac/Props/Conveyors/ConveyorBelt_A08.usd"
# A08 measures 2.72 x 1.15 x 1.17 m, Z-up, metres, and already sits on z=0.
CONVEYOR_POS = (3.90, 5.10, 0.0)   # where the loose bins used to be
CONVEYOR_YAW = 0.0                 # degrees about Z; belt runs along +X

# Supplier's own 600x400x120 Euro crate, converted from STEP (see make_crate_usd.py).
# Centred in x/y with its base at z=0, so translate.z is simply the surface it rests on.
CRATE_USD = f"{ASSETS}/crates/euro_crate_600x400x120.usd"
CRATE_USD_RED = f"{ASSETS}/crates/euro_crate_600x400x120_red.usd"
CRATE_W, CRATE_D, CRATE_H = 0.600, 0.400, 0.120
# 36-crate pile: 3 columns (x) x 2 rows (y) x 6 layers (z)
PILE_ORIGIN = (8.80, 6.90)      # from (7.30, 5.00): +1.5 X, +1.5 Y, then +0.4 Y
PILE_COLS, PILE_ROWS, PILE_LAYERS = 3, 2, 6
PILE_YAW = 90.0                 # crates turned so their long side runs along Y
N_PILES = 3                     # three piles side by side
# clear gap BEFORE each pile (index 0 unused, the first pile sits at PILE_ORIGIN)
PILE_GAPS = (0.0, 0.5, 0.31)    # gap before each pile (0.06 + 0.25 between 1 and 2)
PILE_STEP_SIGN = -1             # extend in -X: +X would run past the wall at 11.08
PILE_CRATE_USD = (None, None, "red")   # per pile: None = grey, "red" = red variant
# A08's roller bed tops out at z=0.7693 in the asset's own frame; the uprights carry on
# up to 1.166, so the overall bbox height is NOT the surface to place things on.
CONVEYOR_ROLLER_TOP = 0.7693
CONVEYOR_ROLLER_X = (-2.682, -0.007)   # roller run, conveyor-local
# G1 stands at the discharge end of the belt, on the far side of the tipper trough
# (the trough occupies y 3.32..4.10; the robot used to be at y 2.23, the near side).
G1_POS = (4.65, 5.10)
G1_YAW = 195.0                  # face back down the belt
N_CRATES_ON_BELT = 2


WALL_TEXTURE = f"{ASSETS}/textures/astra_wall.png"
FLOOR_TEXTURE = f"{ASSETS}/textures/floor_tile.png"
FLOOR_TILE_M = 1.0  # one texture tile per metre
# Meshes identified as walls in the ArchiCAD model (thin slab, >2m on both other axes)
WALL_MESHES = ["Mesh_017", "Mesh_004"]
WALL_STRETCH = True  # one copy of the artwork stretched to fill each wall
WALL_REPEATS = 1.0   # only used when WALL_STRETCH is False
SIDE_WALL_MESHES = ()   # Mesh_004 moved to concrete; no branded side walls left
IMAGE_ASPECT = 2.637  # astra_wall.png is 1640x622


def _planar_uvs(mesh, repeats, room_centre):
    """Author faceVarying 'st' by projecting onto the wall's two large axes.

    The ArchiCAD export ships no UVs at all (259 of 270 meshes have none), so a
    texture cannot bind until we generate them. These walls are flat slabs, so a
    planar projection down the thin axis is exact -- no unwrapping needed.
    """
    from pxr import Gf, Sdf, UsdGeom, Vt

    pts = mesh.GetPointsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not idx:
        return False

    lo = [min(pt[i] for pt in pts) for i in range(3)]
    hi = [max(pt[i] for pt in pts) for i in range(3)]
    span = [hi[i] - lo[i] for i in range(3)]
    thin = span.index(min(span))
    u_ax, v_ax = [a for a in range(3) if a != thin]
    # keep the image upright: V follows the world-vertical axis where possible
    if v_ax != 2 and u_ax == 2:
        u_ax, v_ax = v_ax, u_ax

    # Which way does U have to run so text is not mirrored?
    # The wall is seen from inside the room, i.e. the viewer looks along
    #   d = -inward_normal
    # with up = +Z, so screen-right is r = d x up. U must increase along r.
    #   thin=Y -> r = (-inward, 0, 0)      thin=X -> r = (0, +inward, 0)
    inward = 1.0 if room_centre[thin] > (lo[thin] + hi[thin]) * 0.5 else -1.0
    u_sign = -inward if thin == 1 else (inward if thin == 0 else 1.0)

    su = span[u_ax] or 1.0
    sv = span[v_ax] or 1.0

    if WALL_STRETCH:
        # one copy of the artwork covering the whole wall; it takes the wall's
        # aspect rather than its own, which is what "stretch to fit" means
        rep_u = rep_v = 1.0
    else:
        # Tile `repeats` times across, then derive the vertical count so each tile
        # keeps the artwork's own aspect and nothing looks stretched:
        #   tile width  = su / repeats
        #   tile height = tile width / IMAGE_ASPECT
        #   rep_v       = sv / tile height
        rep_u = repeats
        rep_v = repeats * IMAGE_ASPECT * sv / su

    uvs = []
    for i in idx:
        pt = pts[i]
        fu = (pt[u_ax] - lo[u_ax]) / su
        if u_sign < 0:
            fu = 1.0 - fu
        uvs.append(
            Gf.Vec2f(
                float(fu * rep_u),
                float((pt[v_ax] - lo[v_ax]) / sv * rep_v),
            )
        )

    pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    )
    pv.Set(Vt.Vec2fArray(uvs))
    return True


def _tex_material(stage, name, tex_path, roughness=0.65):
    """UsdPreviewSurface with a repeating texture."""
    from pxr import Gf, Sdf, UsdShade

    root = f"/World/Looks/{name}"
    mat = UsdShade.Material.Define(stage, Sdf.Path(root))

    reader = UsdShade.Shader.Define(stage, Sdf.Path(root + "/stReader"))
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")

    tex = UsdShade.Shader.Define(stage, Sdf.Path(root + "/tex"))
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex_path)
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result"
    )
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    surf = UsdShade.Shader.Define(stage, Sdf.Path(root + "/surface"))
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb"
    )
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    return mat


# The back wall (Mesh_017, x -50.859..-40.695 local) is interrupted by a column
# cluster at x ~ -44.2, so the branding reads as two separate panels -- exactly like
# the reference render. Mesh_017 is a single 8-point box, so a UV seam mid-face is
# impossible; instead lay two flat quads just in front of it, each mapped 0..1.
WALL_LOCAL = dict(xmin=-50.8587, xmax=-40.6947, yface=30.5989, zmin=-2.43007, zmax=0.683928)
PILLAR_L, PILLAR_R = -44.28, -44.14  # gap the column cluster occupies


def _wall_panels(stage, offset, material, suffix="", dx=0.0):
    """Two textured quads in front of the back wall, split at the column."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    w = WALL_LOCAL
    y = w["yface"] + float(offset[1]) + 0.012   # 12mm proud, avoids z-fighting
    z0 = w["zmin"] + float(offset[2])
    z1 = w["zmax"] + float(offset[2])
    segments = {
        "panel_left": (w["xmin"], PILLAR_L),
        "panel_right": (PILLAR_R, w["xmax"]),
    }

    UsdGeom.Xform.Define(stage, Sdf.Path("/World/WallPanels"))
    for name, (lx, hx) in segments.items():
        x0 = lx + float(offset[0]) + dx
        x1 = hx + float(offset[0]) + dx
        override = PANEL_RIGHT_EDGE.get(f"{name}{suffix}")
        if override is not None:
            x1 = float(override)
        mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(f"/World/WallPanels/{name}{suffix}"))
        # wound so the normal faces +Y, i.e. into the room
        mesh.CreatePointsAttr(
            [(x1, y, z0), (x0, y, z0), (x0, y, z1), (x1, y, z1)]
        )
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateNormalsAttr([(0, 1, 0)] * 4)
        mesh.CreateDoubleSidedAttr(True)
        # U runs -X because the room side is viewed looking along -Y, where screen
        # right is -X; without this the artwork comes out mirrored.
        pv = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        )
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        carb.log_warn(f"[build] {name}{suffix}: x {x0:.2f}..{x1:.2f} ({x1 - x0:.2f}m wide)")


SAFETY_YELLOW = (0.98, 0.75, 0.02)

# Parts of the tipper/feed machine to render as brushed metal rather than flat
# ArchiCAD colour. Paths are into the referenced station.
METAL_MESHES = (
    "VisualSceneNode1/Geometry141/Mesh_117",
    "VisualSceneNode1/Geometry133/Mesh_109",
    "VisualSceneNode1/Geometry135/Mesh_111",
    "VisualSceneNode1/Geometry94/Mesh_073",
    "VisualSceneNode1/Geometry194/Mesh_150",
    "VisualSceneNode1/Geometry123/Mesh_099",
    "VisualSceneNode1/Geometry124/Mesh_100",
    "VisualSceneNode1/Geometry122/Mesh_098",
    "VisualSceneNode1/Geometry121/Mesh_097",
    "VisualSceneNode1/Geometry93/Mesh_072",
    "VisualSceneNode1/Geometry92/Mesh_071",
    "VisualSceneNode1/Geometry116/Mesh_092",
    "VisualSceneNode1/Geometry100/Mesh_079",
    "VisualSceneNode1/Geometry106/Mesh_085",
    "VisualSceneNode1/Geometry90/Mesh_069",
    "VisualSceneNode1/Geometry126/Mesh_102",
)
# Two copies of the workspace, joined end to end. The side walls sit at the +X end
# (x ~10.5..10.9), so the free side is -X and the copy extends that way. The copy's
# own side wall would land right on the join, so it is dropped.
STATION_COPIES = (
    {"name": "Station",  "dx_rooms": 0.0,  "drop": ()},
    # everything at the copy's +X end is dropped so its back wall butts straight
    # up against the original's: side wall, two columns, a pier and the ceiling beam
    # (all sit at x -0.49..-0.10, i.e. right on the join at x=0)
    {
        "name": "Station2",
        "dx_rooms": -1.0,
        "drop": ("Mesh_004", "Mesh_003", "Mesh_018", "Mesh_002", "Mesh_016", "Mesh_189"),
        "clear_join": True,     # sweep this copy's +X end so it butts up to its neighbour
    },
    {
        "name": "Station3",
        "dx_rooms": -2.0,
        "drop": ("Mesh_004", "Mesh_003", "Mesh_018", "Mesh_002", "Mesh_016", "Mesh_189"),
        "clear_join": True,
    },
)
# deactivated on every copy
REMOVE_MESHES = ("Mesh_001",)

# Clear out the copy's join end entirely. These 20 are named explicitly; everything
# else whose centre falls inside their combined strip is swept too ("and stuff near
# it"), except room-spanning geometry -- the 11x8m floor slab and the 10.16m back
# wall merely pass through the strip and must survive.
STATION2_CLEAR = (
    "Mesh_032", "Mesh_020", "Mesh_188", "Mesh_187", "Mesh_023", "Mesh_022",
    "Mesh_028", "Mesh_029", "Mesh_030", "Mesh_027", "Mesh_204", "Mesh_214",
    "Mesh_009", "Mesh_015", "Mesh_014", "Mesh_010", "Mesh_011", "Mesh_224",
    "Mesh_012", "Mesh_013",
)
CLEAR_PAD = 0.30        # metres of slack around the strip
CLEAR_MAX_FOOTPRINT = 5.0   # a mesh wider/deeper than this spans the room: keep it

# Floor trim at the joins. These skirting strips are only ~6cm tall so they are not
# walls, but a walking robot would still trip on them. The join sweep misses them
# because it tests a prim's CENTRE and these strips run far along the wall, so clear
# them by OVERLAP with a band around each join instead.
JOIN_TRIM_MAX_H = 0.10      # metres; anything this low is trim, not structure
JOIN_TRIM_BAND = 0.75       # metres either side of a join

# Widen a concrete pillar about its own centre, then run the copy's right-hand
# branded panel across to meet its new face, closing the gap over the join.
# relative subpath -> extra metres in X, applied to EVERY copy so each bay's
# right-hand panel has a widened pillar in its neighbour to attach to
WIDEN_PILLARS = {
    "VisualSceneNode1/Geometry7/Mesh_006": 0.15,
}
# panel name -> world x its right edge should reach (None = leave as authored)
PANEL_RIGHT_EDGE = {}   # "panel_right_<Name>" -> world x; filled in after widening

CONCRETE_TEXTURE_NAME = "concrete.png"
CONCRETE_TILE_M = 1.4      # metres per texture tile, world scale
# Structural concrete. 5 of these are columns; Mesh_189 is a ceiling beam and
# Mesh_003 a thin pier -- all cast concrete, so they share the material.
CONCRETE_MESHES = (
    "VisualSceneNode1/Geometry6/Mesh_005",
    "VisualSceneNode1/Geometry7/Mesh_006",
    "VisualSceneNode1/Geometry2/Mesh_001",
    "VisualSceneNode1/Geometry17/Mesh_016",
    "VisualSceneNode1/Geometry19/Mesh_018",
    "VisualSceneNode1/Geometry263/Mesh_189",
    "VisualSceneNode1/Geometry4/Mesh_003",
    "VisualSceneNode1/Geometry5/Mesh_004",   # side wall
)
METAL_RGB = (0.56, 0.57, 0.58)
METAL_ROUGHNESS = 0.28

# Lighting. The first pass at 1200/2500 blew the whites out; the supplier's
# reference render is much softer, so these are dialled back and the key light is
# widened (a bigger angular size = softer shadow edges).
DOME_INTENSITY = 320.0
KEY_INTENSITY = 900.0
KEY_ANGLE = 6.0        # degrees of angular size; 1.0 gives hard, crisp shadows
# The A08 frame is painted with a material *named* Blue; it covers 38.6% of the model
# (rails, uprights, cross-braces, feet). The rest are small frame details.
CONVEYOR_FRAME_MATERIALS = (
    "MetalPainted_Blue_Glossy_A",
    "MetalPainted_Gray_Glossy_A",
    "Metal_Glossy_A",
    "Metal_Rough_A",
    "Steel_A",
)


def _recolour_conveyor_frame(stage, asset_prim_path, rgb=SAFETY_YELLOW):
    """Tint the conveyor's frame materials.

    These are MDL/OmniPBR shaders and the asset already drives their colour through
    `diffuse_tint`, so overriding that multiplies the painted-metal albedo and keeps
    the normal/roughness detail instead of flattening it to a solid colour.
    """
    from pxr import Sdf, UsdShade

    root = stage.GetPrimAtPath(asset_prim_path)
    if not root or not root.IsValid():
        carb.log_warn(f"[build] no conveyor prim at {asset_prim_path}")
        return 0
    if root.IsInstanceable():          # overrides cannot reach into instance proxies
        root.SetInstanceable(False)

    looks = stage.GetPrimAtPath(f"{asset_prim_path}/Looks")
    if not looks or not looks.IsValid():
        carb.log_warn("[build] conveyor has no Looks scope")
        return 0

    n = 0
    for name in CONVEYOR_FRAME_MATERIALS:
        mat_prim = stage.GetPrimAtPath(f"{looks.GetPath()}/{name}")
        if not mat_prim or not mat_prim.IsValid():
            continue
        if mat_prim.IsInstanceable():
            mat_prim.SetInstanceable(False)
        mat = UsdShade.Material(mat_prim)
        shaders = [c for c in mat_prim.GetChildren() if c.IsA(UsdShade.Shader)]
        src = mat.ComputeSurfaceSource("mdl")[0]
        if src:
            shaders = [src.GetPrim()]
        for sh_prim in shaders:
            sh = UsdShade.Shader(sh_prim)
            sh.CreateInput("diffuse_tint", Sdf.ValueTypeNames.Color3f).Set(rgb)
            sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(rgb)
            n += 1
    return n


def _metal_material(stage):
    """Brushed stainless: a plain UsdPreviewSurface, metallic with low roughness.

    No texture, so it needs no UVs -- which matters here because the ArchiCAD
    meshes have none.
    """
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, Sdf.Path("/World/Looks/Metal"))
    surf = UsdShade.Shader.Define(stage, Sdf.Path("/World/Looks/Metal/surface"))
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*METAL_RGB))
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(1.0)
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(METAL_ROUGHNESS)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    return mat


def _worldscale_uvs(mesh, tile_m):
    """faceVarying 'st' in metres / tile_m, projected down the mesh's thin axis.

    Unlike the stretch-to-fit mapping used for the branded panels, this keeps the
    texture the same physical size on every prim, so a 3m column and a 0.13m beam
    show concrete at the same grain. The ArchiCAD meshes carry no UVs at all, so
    something has to be authored before any texture can bind.
    """
    from pxr import Gf, Sdf, UsdGeom, Vt

    pts = mesh.GetPointsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not idx:
        return False
    lo = [min(pt[i] for pt in pts) for i in range(3)]
    hi = [max(pt[i] for pt in pts) for i in range(3)]
    span = [hi[i] - lo[i] for i in range(3)]
    thin = span.index(min(span))
    u_ax, v_ax = [a for a in range(3) if a != thin]
    if v_ax != 2 and u_ax == 2:      # keep V vertical where possible
        u_ax, v_ax = v_ax, u_ax

    uvs = [
        Gf.Vec2f(
            float((pts[i][u_ax] - lo[u_ax]) / tile_m),
            float((pts[i][v_ax] - lo[v_ax]) / tile_m),
        )
        for i in idx
    ]
    UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    ).Set(Vt.Vec2fArray(uvs))
    return True


def _widen_in_x(stage, path, extra_m):
    """Scale a prim in X about its own centre so its width grows by `extra_m`.

    Uses XformCommonAPI with a pivot; safe here because these ArchiCAD meshes carry
    no xformOps of their own (unlike the referenced Isaac assets, where the
    quaternion `orient` op makes XformCommonAPI silently no-op).
    Returns the new world-space x range, or None.
    """
    from pxr import Gf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        carb.log_warn(f"[build] widen target missing: {path}")
        return None
    pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
    if not pts:
        return None
    lo = min(pt[0] for pt in pts)
    hi = max(pt[0] for pt in pts)
    width = hi - lo
    if width <= 0:
        return None
    factor = (width + extra_m) / width
    cy = (min(pt[1] for pt in pts) + max(pt[1] for pt in pts)) / 2.0
    cz = (min(pt[2] for pt in pts) + max(pt[2] for pt in pts)) / 2.0

    api = UsdGeom.XformCommonAPI(prim)
    api.SetPivot(Gf.Vec3f(float((lo + hi) / 2.0), float(cy), float(cz)))
    api.SetScale(Gf.Vec3f(float(factor), 1.0, 1.0))

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    carb.log_warn(
        f"[build] widened {path.split('/')[-1]} by {extra_m:.2f}m "
        f"(x{factor:.3f}) -> world x {r.GetMin()[0]:.3f}..{r.GetMax()[0]:.3f}"
    )
    return (r.GetMin()[0], r.GetMax()[0])


def _add_asset(stage, path, asset, translate):
    """Reference `asset` under `path` and place it at `translate`.

    The reference goes on a CHILD prim so our translate never collides with the
    asset's own xformOpOrder. Placing the referencing prim directly does not work:
    the G1 uses [translate, orient, scale], AddTranslateOp() then raises, and
    XformCommonAPI.SetTranslate() silently no-ops because it cannot handle the
    quaternion 'orient' op.
    """
    from pxr import Gf, Sdf, UsdGeom

    wrapper = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    wrapper.AddTranslateOp().Set(Gf.Vec3d(*translate))
    inner = stage.DefinePrim(Sdf.Path(path + "/asset"), "Xform")
    inner.GetReferences().AddReference(asset)
    return inner


def _station_offset(stage, path):
    """Shift so the room's min corner sits at the origin with the floor at z=0.

    Bound the meshes only -- the Camera and Light prims in this file inflate the
    /root bbox to 56x38m and would give a nonsense offset.
    """
    from pxr import Gf, Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    room = Gf.Range3d()
    # keep a named reference -- Usd.Stage.Open(...).Traverse() lets the stage be
    # collected mid-iteration and the prims expire
    src = Usd.Stage.Open(path)
    for prim in src.Traverse():
        if prim.GetTypeName() == "Mesh":
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not box.IsEmpty():
                room.UnionWith(box)
    mn, size = room.GetMin(), room.GetSize()

    # Align z to the TOP of the floor slab, not the model's absolute minimum.
    # The slab is 0.28m thick, so using the minimum puts world z=0 at the slab's
    # underside and everything placed on "the floor" sinks 0.276m into it.
    floor_top = None
    for prim in src.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        if box.IsEmpty():
            continue
        bs = box.GetSize()
        if bs[0] > 3.0 and bs[1] > 2.0 and bs[2] < 0.6:      # broad, flat -> floor slab
            top = box.GetMax()[2]
            if floor_top is None or top > floor_top:
                floor_top = top
    if floor_top is None:
        floor_top = mn[2]
    carb.log_warn(
        f"[build] station min={mn} size={size} floor_top(local)={floor_top:.4f}"
    )
    return Gf.Vec3d(-mn[0], -mn[1], -floor_top), size


async def _run():
    app = omni.kit.app.get_app()
    for _ in range(20):
        await app.next_update_async()

    ctx = omni.usd.get_context()
    await ctx.new_stage_async()
    for _ in range(10):
        await app.next_update_async()

    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())

    # ---- physics -------------------------------------------------------
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)
    carb.log_warn("[build] physics scene created")

    # ---- floor ---------------------------------------------------------
    ground = UsdGeom.Xform.Define(stage, Sdf.Path("/World/GroundPlane"))
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/GroundPlane/mesh"))
    s = 30.0
    zg = -0.01  # 1cm under the station floor slab, avoids z-fighting
    plane.CreatePointsAttr([(-s, -s, zg), (s, -s, zg), (s, s, zg), (-s, s, zg)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    plane.CreateNormalsAttr([(0, 0, 1)] * 4)
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    # tile the floor texture once per metre so it reads as a real tiled floor
    # rather than the flat grey the default displayColor gives
    from pxr import Vt

    reps = (2 * s) / FLOOR_TILE_M
    UsdGeom.PrimvarsAPI(plane.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
    ).Set(
        Vt.Vec2fArray(
            [Gf.Vec2f(0, 0), Gf.Vec2f(reps, 0), Gf.Vec2f(reps, reps), Gf.Vec2f(0, reps)]
        )
    )
    del ground
    carb.log_warn("[build] ground plane + collider + tiled floor material")

    # ---- lighting ------------------------------------------------------
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
    dome.GetIntensityAttr().Set(DOME_INTENSITY)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
    key.GetIntensityAttr().Set(KEY_INTENSITY)
    key.GetAngleAttr().Set(KEY_ANGLE)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))
    carb.log_warn("[build] lighting")

    # ---- the station (delivered ArchiCAD geometry), one prim per copy -----
    offset, size = _station_offset(stage, OFFICE_USD)
    room_w = size[0]
    station_roots = []
    for spec in STATION_COPIES:
        root = f"/World/{spec['name']}"
        dx = spec["dx_rooms"] * room_w
        prim = stage.DefinePrim(Sdf.Path(root), "Xform")
        prim.GetReferences().AddReference(OFFICE_USD)
        UsdGeom.Xformable(prim).AddTranslateOp().Set(
            Gf.Vec3d(offset[0] + dx, offset[1], offset[2])
        )
        station_roots.append((root, dx, spec))
        carb.log_warn(f"[build] {spec['name']} at dx={dx:+.2f} (x {dx:.2f}..{dx + room_w:.2f})")
    for _ in range(40):
        await app.next_update_async()

    # deactivate unwanted meshes: REMOVE_MESHES everywhere, plus per-copy drops
    n_off = 0
    for root, dx, spec in station_roots:
        drops = set(REMOVE_MESHES) | set(spec["drop"])
        for prim in stage.Traverse():
            if prim.GetTypeName() != "Mesh":
                continue
            if not str(prim.GetPath()).startswith(root + "/"):
                continue
            if prim.GetName() in drops:
                prim.SetActive(False)
                n_off += 1
    carb.log_warn(f"[build] deactivated {n_off} meshes (removed / side wall on copy)")

    # ---- sweep the join end of every copy that asks for it ----------------
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    for root, _dx, spec in station_roots:
        if not spec.get("clear_join"):
            continue
        prefix = root + "/"
        strip = Gf.Range3d()
        named = 0
        for prim in stage.Traverse():
            if prim.GetTypeName() != "Mesh":
                continue
            if not str(prim.GetPath()).startswith(prefix):
                continue
            if prim.GetName() in STATION2_CLEAR:
                r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
                if not r.IsEmpty():
                    strip.UnionWith(r)
                prim.SetActive(False)
                named += 1

        swept = kept = 0
        if strip.IsEmpty():
            continue
        lo, hi = strip.GetMin(), strip.GetMax()
        for prim in stage.Traverse():
            if prim.GetTypeName() != "Mesh" or not prim.IsActive():
                continue
            if not str(prim.GetPath()).startswith(prefix):
                continue
            r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            ctr, sz = r.GetMidpoint(), r.GetSize()
            inside = all(
                lo[i] - CLEAR_PAD <= ctr[i] <= hi[i] + CLEAR_PAD for i in range(3)
            )
            if not inside:
                continue
            if max(sz[0], sz[1]) >= CLEAR_MAX_FOOTPRINT:
                kept += 1          # floor slab / back wall passing through
                continue
            prim.SetActive(False)
            swept += 1
        carb.log_warn(
            f"[build] {spec['name']} join end: {named} named + {swept} swept, "
            f"{kept} room-spanning kept "
            f"(strip x {lo[0]:.2f}..{hi[0]:.2f})"
        )

    # ---- clear low floor trim spanning each join --------------------------
    joins = []
    for i in range(1, len(station_roots)):
        joins.append(station_roots[i - 1][1])   # the +X edge of copy i is at its dx
    n_trim = 0
    for jx in joins:
        for prim in stage.Traverse():
            if prim.GetTypeName() != "Mesh" or not prim.IsActive():
                continue
            if not str(prim.GetPath()).startswith("/World/Station"):
                continue
            r = bb.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            mn, mx = r.GetMin(), r.GetMax()
            if mx[2] - mn[2] > JOIN_TRIM_MAX_H:
                continue                        # too tall to be trim
            if mx[0] < jx - JOIN_TRIM_BAND or mn[0] > jx + JOIN_TRIM_BAND:
                continue                        # does not reach the join
            if max(mx[0] - mn[0], mx[1] - mn[1]) >= CLEAR_MAX_FOOTPRINT:
                continue                        # the floor slab itself
            prim.SetActive(False)
            n_trim += 1
    carb.log_warn(
        f"[build] cleared {n_trim} floor-trim strips across {len(joins)} join(s) "
        f"at x={[round(j, 2) for j in joins]}"
    )

    # static triangle-mesh colliders on every remaining station mesh
    n = 0
    for root, _dx, _spec in station_roots:
        for prim in stage.Traverse():
            if prim.GetTypeName() == "Mesh" and str(prim.GetPath()).startswith(root + "/"):
                UsdPhysics.CollisionAPI.Apply(prim)
                mca = UsdPhysics.MeshCollisionAPI.Apply(prim)
                mca.CreateApproximationAttr().Set(UsdPhysics.Tokens.none)
                n += 1
    carb.log_warn(f"[build] {len(station_roots)} station copies, colliders on {n} meshes")

    # ---- Astra branding on the walls ------------------------------------
    from pxr import UsdShade

    wall_mat = _tex_material(stage, "AstraWall", WALL_TEXTURE)

    # ---- concrete on the structure ---------------------------------------
    concrete_mat = _tex_material(
        stage, "Concrete", f"{ASSETS}/textures/{CONCRETE_TEXTURE_NAME}", roughness=0.85
    )
    n_conc = n_conc_missing = 0
    for root, _dx, _spec in station_roots:
        for sub in CONCRETE_MESHES:
            prim = stage.GetPrimAtPath(f"{root}/{sub}")
            if not prim or not prim.IsValid():
                n_conc_missing += 1
                continue
            if not prim.IsActive():
                continue                      # dropped on this copy
            if _worldscale_uvs(UsdGeom.Mesh(prim), CONCRETE_TILE_M):
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(concrete_mat)
                n_conc += 1
    carb.log_warn(f"[build] concrete bound to {n_conc} meshes ({n_conc_missing} missing)")

    # ---- brushed metal on the machine ------------------------------------
    metal_mat = _metal_material(stage)
    n_metal = n_missing = 0
    for root, _dx, _spec in station_roots:
        for sub in METAL_MESHES:
            prim = stage.GetPrimAtPath(f"{root}/{sub}")
            if not prim or not prim.IsValid():
                n_missing += 1
                continue
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(metal_mat)
            n_metal += 1
    carb.log_warn(f"[build] metal bound to {n_metal} meshes ({n_missing} missing)")

    floor_mat = _tex_material(stage, "Floor", FLOOR_TEXTURE, roughness=0.45)
    UsdShade.MaterialBindingAPI.Apply(
        stage.GetPrimAtPath("/World/GroundPlane/mesh")
    ).Bind(floor_mat)
    carb.log_warn("[build] floor material bound")

    # Widen the pillar in every bay, remembering each bay's resulting left face.
    pillar_face = {}          # dx of the bay -> world x of its widened pillar face
    for root, dx, spec in station_roots:
        for sub, extra in WIDEN_PILLARS.items():
            rng = _widen_in_x(stage, f"{root}/{sub}", extra)
            if rng:
                pillar_face[round(dx, 3)] = rng[0]

    # Each clear_join copy's right panel runs out to the pillar in the bay on its
    # +X side -- NOT to a single global value, or bay 3 would stretch across bay 2.
    for root, dx, spec in station_roots:
        if not spec.get("clear_join"):
            continue
        neighbour = round(dx + room_w, 3)
        if neighbour in pillar_face:
            PANEL_RIGHT_EDGE[f"panel_right_{spec['name']}"] = pillar_face[neighbour]
            carb.log_warn(
                f"[build] {spec['name']} right panel -> pillar face "
                f"x={pillar_face[neighbour]:.3f} (bay at dx={neighbour:+.2f})"
            )

    # back wall -> two panels split at the column, for each copy
    for root, dx, spec in station_roots:
        _wall_panels(stage, offset, wall_mat, suffix=f"_{spec['name']}", dx=dx)

    # remaining flat walls (e.g. Mesh_004) still get a single stretched copy
    done = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "Mesh":
            continue
        if prim.GetName() not in SIDE_WALL_MESHES:
            continue
        local_centre = (
            -offset[0] + size[0] * 0.5,
            -offset[1] + size[1] * 0.5,
            -offset[2] + size[2] * 0.5,
        )
        if _planar_uvs(UsdGeom.Mesh(prim), WALL_REPEATS, local_centre):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(wall_mat)
            done += 1
    carb.log_warn(f"[build] side walls textured: {done}")

    # ---- Unitree G1 -----------------------------------------------------
    robot = _add_asset(stage, "/World/G1", G1_USD, (G1_POS[0], G1_POS[1], 0.80))
    if G1_YAW:
        UsdGeom.Xformable(stage.GetPrimAtPath("/World/G1")).AddRotateZOp().Set(G1_YAW)
    # the G1 ships a Physics variant set; pick the articulated one so it simulates
    vset = robot.GetVariantSets().GetVariantSet("Physics")
    if vset:
        avail = vset.GetVariantNames()
        for want in ("PhysX", "SimplifiedPhysX"):
            if want in avail:
                vset.SetVariantSelection(want)
                carb.log_warn(f"[build] G1 Physics variant -> {want} (of {avail})")
                break
    for _ in range(20):
        await app.next_update_async()
    # The G1's origin is at its pelvis, so a nominal z guess buries the feet.
    # Measure the loaded bbox and lift it so the lowest point rests on the floor.
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    g1_range = bb.ComputeWorldBound(stage.GetPrimAtPath("/World/G1")).ComputeAlignedRange()
    if not g1_range.IsEmpty():
        drop = g1_range.GetMin()[2] - 0.02          # want feet 2cm above the floor
        xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/G1"))
        op = xf.GetOrderedXformOps()[0]
        cur = op.Get()
        op.Set(Gf.Vec3d(cur[0], cur[1], cur[2] - drop))
        carb.log_warn(
            f"[build] G1 feet were at z={g1_range.GetMin()[2]:.3f}, lifted by {-drop:.3f}"
        )
    carb.log_warn("[build] G1 referenced")

    # ---- conveyor -------------------------------------------------------
    conveyor = _add_asset(stage, "/World/Conveyor", CONVEYOR_USD, CONVEYOR_POS)
    if CONVEYOR_YAW:
        UsdGeom.Xformable(
            stage.GetPrimAtPath("/World/Conveyor")
        ).AddRotateZOp().Set(CONVEYOR_YAW)
    for _ in range(30):
        await app.next_update_async()
    cbb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    crange = cbb.ComputeWorldBound(stage.GetPrimAtPath("/World/Conveyor")).ComputeAlignedRange()
    if not crange.IsEmpty():
        carb.log_warn(
            f"[build] conveyor bbox x {crange.GetMin()[0]:.2f}..{crange.GetMax()[0]:.2f} "
            f"y {crange.GetMin()[1]:.2f}..{crange.GetMax()[1]:.2f} "
            f"z {crange.GetMin()[2]:.2f}..{crange.GetMax()[2]:.2f}"
        )
    del conveyor
    n_tint = _recolour_conveyor_frame(stage, "/World/Conveyor/asset")
    carb.log_warn(f"[build] conveyor A08 placed, frame tinted on {n_tint} shaders")

    bins = UsdGeom.Xform.Define(stage, Sdf.Path("/World/Bins"))
    del bins
    # ---- crates riding on the conveyor ----------------------------------
    rx0 = CONVEYOR_POS[0] + CONVEYOR_ROLLER_X[0]
    rx1 = CONVEYOR_POS[0] + CONVEYOR_ROLLER_X[1]
    run = rx1 - rx0
    crate_z = CONVEYOR_POS[2] + CONVEYOR_ROLLER_TOP + 0.002   # 2mm settling gap
    for i in range(N_CRATES_ON_BELT):
        # evenly spaced along the roller run: centres at (i+1)/(n+1) of the way
        cx = rx0 + run * (i + 1) / (N_CRATES_ON_BELT + 1)
        _add_asset(
            stage,
            f"/World/Crates/crate_{i:02d}",
            CRATE_USD,
            (cx, CONVEYOR_POS[1], crate_z),
        )
        carb.log_warn(f"[build] crate_{i:02d} at x={cx:.3f} y={CONVEYOR_POS[1]:.2f} z={crate_z:.4f}")
    carb.log_warn(
        f"[build] {N_CRATES_ON_BELT} crates on the belt "
        f"(roller run x {rx0:.2f}..{rx1:.2f})"
    )

    # ---- 36-crate pile: 3 cols x 2 rows x 6 layers ----------------------
    px, py = PILE_ORIGIN
    # With a 90 deg yaw the crate's long side (0.600) runs along Y and its short
    # side (0.400) along X, so the column/row pitches swap accordingly.
    turned = abs(PILE_YAW % 180.0 - 90.0) < 1e-6
    pitch_x = CRATE_D if turned else CRATE_W
    pitch_y = CRATE_W if turned else CRATE_D
    footprint_x = PILE_COLS * pitch_x
    n_pile = 0
    # walk outwards, accumulating each pile's own gap so they need not be even
    offset = 0.0
    for pile in range(N_PILES):
        if pile:
            offset += footprint_x + PILE_GAPS[pile]
        cx_centre = px + PILE_STEP_SIGN * offset
        crate_usd = CRATE_USD_RED if PILE_CRATE_USD[pile] == "red" else CRATE_USD
        x0 = cx_centre - (PILE_COLS - 1) * pitch_x / 2.0
        y0 = py - (PILE_ROWS - 1) * pitch_y / 2.0
        for layer in range(PILE_LAYERS):
            # 1mm per layer keeps the convex hulls from starting interpenetrated
            z = layer * (CRATE_H + 0.001)
            for cx in range(PILE_COLS):
                for ry in range(PILE_ROWS):
                    path = f"/World/CratePile/p{pile}_c{layer}_{cx}{ry}"
                    _add_asset(
                        stage,
                        path,
                        crate_usd,
                        (x0 + cx * pitch_x, y0 + ry * pitch_y, z),
                    )
                    if PILE_YAW:
                        # op order [translate, rotateZ] -> rotate about the crate's
                        # own origin first, then move into place (USD row-vector)
                        UsdGeom.Xformable(
                            stage.GetPrimAtPath(path)
                        ).AddRotateZOp().Set(PILE_YAW)
                    n_pile += 1
        carb.log_warn(
            f"[build]   pile {pile}: centre x={cx_centre:.2f} "
            f"spans {cx_centre - footprint_x / 2:.2f}..{cx_centre + footprint_x / 2:.2f} "
            f"({PILE_CRATE_USD[pile] or 'grey'}, gap before {PILE_GAPS[pile]:.2f})"
        )
    carb.log_warn(
        f"[build] {N_PILES} piles, {n_pile} crates total, "
        f"{PILE_COLS}x{PILE_ROWS} footprint "
        f"({PILE_COLS * pitch_x:.2f} x {PILE_ROWS * pitch_y:.2f} m), yaw {PILE_YAW:g} deg, "
        f"{PILE_LAYERS} layers, top at z={(PILE_LAYERS - 1) * (CRATE_H + 0.001) + CRATE_H:.3f}"
    )

    for _ in range(90):
        await app.next_update_async()

    counts = {}
    for pr in stage.Traverse():
        counts[pr.GetTypeName()] = counts.get(pr.GetTypeName(), 0) + 1
    carb.log_warn(f"[build] prim types: {counts}")

    stage.GetRootLayer().Export(OUT_PATH)
    carb.log_warn(f"[build] saved -> {OUT_PATH}")

    # ---- camera ---------------------------------------------------------
    from omni.kit.viewport.utility import get_active_viewport
    from omni.kit.viewport.utility.camera_state import ViewportCameraState

    # Mesh_017 (the branded back wall) sits at y~0 after the origin shift, so stand
    # inside the room and look back at it with the station in the foreground.
    # both workspaces: the copy extends to -11.08, so frame the whole run
    centre = Gf.Vec3d(4.4, 4.2, 0.9)
    eye = Gf.Vec3d(7.6, 8.5, 2.6)
    st = ViewportCameraState("/OmniverseKit_Persp", get_active_viewport())
    st.set_position_world(eye, True)
    st.set_target_world(centre, True)
    # give the viewport frames to actually apply the camera before we finish
    for _ in range(30):
        await app.next_update_async()
    carb.log_warn(f"[build] camera eye={eye} target={centre}")

    # Render the viewport straight to a file. Screen-scraping the window with xwd
    # picks up whatever overlaps it; this renders regardless of window state.
    from omni.kit.viewport.utility import capture_viewport_to_file

    vp = get_active_viewport()
    vp.resolution = (1920, 1080)
    for _ in range(60):
        await app.next_update_async()
    capture_viewport_to_file(vp, SHOT_PATH)
    for _ in range(60):
        await app.next_update_async()
    carb.log_warn(f"[build] captured -> {SHOT_PATH}")
    carb.log_warn("[build] done")


asyncio.ensure_future(_run())
