"""OmniContact G1 moving the ASTRA euro crate from a table onto the conveyor,
in Isaac Sim.

Scene is in the ASTRA workspace frame (as built by build_astra_scene.py):
  conveyor      x 1.18..3.90, y 4.52..5.68, roller top z = 0.77
  crates on it  z 0.77..0.89   (600x400x120 euro crate)
  crate pile    x 4.99..9.40, y 6.30..7.50
So y = 5.10 is the clear working lane down the middle of the station.

  table   (5.60, 5.10) top z 0.75   <- crate starts here
  goal    (2.60, 5.10)      z 0.83  <- conveyor roller top + half crate
  G1      (7.00, 5.10)              <- open floor east of the table
"""
import argparse
import os
import sys

cli = argparse.ArgumentParser()
cli.add_argument("--max-steps", type=int, default=3000)
cli.add_argument("--record", default="/root/oc_astra.mp4")
cli.add_argument("--record-every", type=int, default=10)
cli.add_argument("--half-dims", nargs=3, type=float, default=None)
cli.add_argument("--loop", action="store_true", help="respawn the box and repeat")
cli.add_argument("--plain-box", action="store_true",
                 help="use a plain cuboid instead of the euro crate USD")
A = cli.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402

_DIAG = open("/root/oc_astra_diag.txt", "w", buffering=1)


def log(*a):
    m = " ".join(str(x) for x in a)
    _DIAG.write(m + "\n")
    _DIAG.flush()


import omni.usd  # noqa: E402
import omni.kit.commands  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdLux  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, GroundPlane  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

OC_ROOT = os.environ.get("OC_ROOT", "/root/omnicontact_g1")
sys.path.insert(0, OC_ROOT)
sys.path.insert(0, os.path.join(OC_ROOT, "deploy_omnicontact"))
os.chdir(OC_ROOT)

ASTRA_USD = "/root/astra_workspace/astra_workspace.usd"
CRATE_USD = "/root/astra_workspace/assets/crates/euro_crate_600x400x120.usd"

# CFgen plans its reference path assuming the robot STARTS AT THE ORIGIN.
# Spawning the G1 elsewhere makes it follow a path meant for a different start
# and it walks off and falls. So: keep the robot at (0,0) and SHIFT THE ASTRA
# WORKSPACE around it (same trick used for the coordex OFFICE_POS).
#
# workspace frame -> world:  world = workspace + ASTRA_OFFSET
#   conveyor  ws x 1.18..3.90, y 4.52..5.68  ->  world x 2.90..5.62, y -0.58..0.58
#   tipper    ws x 3.93..7.11, y 3.32..4.10  ->  world x 5.65..8.83, y -1.78..-1.00
#   pile      ws x 4.99..9.40, y 6.30..7.50  ->  world x 6.71..11.12, y 1.20..2.40
# so the robot's lane (y~0, x 0..2.6) is clear of all of it.
ASTRA_OFFSET = (1.72, -4.75, 0.0)   # +0.35 in y: conveyor now sits between camera and robot
LOOP = A.loop
SWAP_AFTER_PLACE = False   # see notes: every post-carry policy swap falls

CRATE_HALF = tuple(A.half_dims) if A.half_dims else (0.30, 0.20, 0.06)
# Work the pile side: piles are at world x 6.71..11.12, so the robot starts
# out among them and carries in -x to the conveyor. Same relative geometry as
# the proven run (robot 1.5 m behind the box, box 1.8 m from the goal), rotated
# 180 degrees. Lane y = 0.35 clears the piles (y >= 1.20) by 0.85 m.
LANE_Y = 0.35
CRATE_XY = (7.00, LANE_Y)                  # 1.5 m ahead of the spawn
TABLE_TOP = 0.35                           # low stand the box starts on
CRATE_Z = CRATE_HALF[2]                    # box on the floor (known-good)
CONVEYOR_TOP = 0.77                        # roller top, workspace z
GOAL = (5.00, LANE_Y, CONVEYOR_TOP + CRATE_HALF[2])  # ON the deck; it overshoots ~0.28 m,
                                                     # so aim in from the x=5.62 edge
G1_XY = (8.50, LANE_Y)                     # out among the crate piles
G1_YAW = 3.14159265                        # facing -x, toward the conveyor
                                           # start quat is antipodal (w~0) and the robot spins out
TABLE_XY = CRATE_XY

MJ_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


def build_runner(task="carrybox", init=None, goal=None):
    """Build one of the repo's runners. `loco` takes pelvis positions, so it can
    be used to walk the robot home after the carry finishes."""
    from omnicontact_runner_args import parse_args
    from run_skill_omnicontact import OmniContactRunner
    if task == "carrybox":
        init = init or (TABLE_XY[0], TABLE_XY[1], CRATE_Z)
        goal = goal or GOAL
    argv = sys.argv
    sys.argv = [
        "run_skill_omnicontact.py",
        "--reference-source", "CFgen", "--policy", "policy.onnx",
        "--task", task,
        "--box-half-dims", str(CRATE_HALF[0]), str(CRATE_HALF[1]), str(CRATE_HALF[2]),
        "--init-pos", *[str(v) for v in init],
        "--goal-pos", *[str(v) for v in goal],
        "--headless",
    ]
    try:
        rargs = parse_args()
    finally:
        sys.argv = argv
    r = OmniContactRunner(rargs)
    # The reference plan is built from the runner's OWN internal MuJoCo base
    # pose (default keyframe = the origin), never from Isaac -- see
    # omnicontact_runner_reset._prepare_episode -> _sync_state_cmd_from_mj,
    # which reads self.d.qpos[:3]. Spawn the Isaac robot anywhere else and it
    # starts off its own reference, lunges to close the gap and falls. Put the
    # MuJoCo base where the Isaac robot actually spawns. For carrybox,
    # _reset_env_cfgen only moves the object, so this survives.
    import mujoco
    r.d.qpos[0], r.d.qpos[1] = float(G1_XY[0]), float(G1_XY[1])
    _h = G1_YAW / 2.0
    r.d.qpos[3:7] = [np.cos(_h), 0.0, 0.0, np.sin(_h)]
    mujoco.mj_forward(r.m, r.d)
    r._prepare_episode()
    _rb = getattr(r.policy, "ref_base_pos", None)
    _rq = getattr(r.policy, "ref_base_quat", None)
    log(f"[plan] mj qpos base={np.round(r.d.qpos[:3],2)} quat={np.round(r.d.qpos[3:7],3)}")
    log(f"[plan] state_cmd base={np.round(r.state_cmd.base_pos,2)} quat={np.round(r.state_cmd.base_quat,3)}")
    if _rb is not None:
        log(f"[plan] ref_base_pos[0]={np.round(np.asarray(_rb)[0],2)} [-1]={np.round(np.asarray(_rb)[-1],2)}")
    if _rq is not None:
        log(f"[plan] ref_base_quat[0]={np.round(np.asarray(_rq)[0],3)}")
    return r


def make_carton(stage, path, half, mass, tex):
    """A textured box mesh: 24 verts, per-face UVs, rigid body."""
    from pxr import Sdf, UsdShade, Vt, Gf
    hx, hy, hz = half
    # 6 faces x 4 corners, wound CCW so normals face outward
    faces = [
        [(+hx,-hy,-hz), (+hx,+hy,-hz), (+hx,+hy,+hz), (+hx,-hy,+hz)],   # +X
        [(-hx,+hy,-hz), (-hx,-hy,-hz), (-hx,-hy,+hz), (-hx,+hy,+hz)],   # -X
        [(+hx,+hy,-hz), (-hx,+hy,-hz), (-hx,+hy,+hz), (+hx,+hy,+hz)],   # +Y
        [(-hx,-hy,-hz), (+hx,-hy,-hz), (+hx,-hy,+hz), (-hx,-hy,+hz)],   # -Y
        [(-hx,-hy,+hz), (+hx,-hy,+hz), (+hx,+hy,+hz), (-hx,+hy,+hz)],   # +Z
        [(-hx,+hy,-hz), (+hx,+hy,-hz), (+hx,-hy,-hz), (-hx,-hy,-hz)],   # -Z
    ]
    pts, counts, idx, uvs = [], [], [], []
    for f in faces:
        b = len(pts)
        pts.extend([Gf.Vec3f(*v) for v in f])
        counts.append(4)
        idx.extend([b, b + 1, b + 2, b + 3])
        uvs.extend([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateSubdivisionSchemeAttr("none")
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.faceVarying).Set(Vt.Vec2fArray(uvs))

    # kraft-cardboard material
    mat = UsdShade.Material.Define(stage, Sdf.Path(path + "/mat"))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + "/mat/surf"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.92)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    st = UsdShade.Shader.Define(stage, Sdf.Path(path + "/mat/st"))
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tx = UsdShade.Shader.Define(stage, Sdf.Path(path + "/mat/tex"))
    tx.CreateIdAttr("UsdUVTexture")
    tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex)
    tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st.CreateOutput("result", Sdf.ValueTypeNames.Float2))
    tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
    mat.CreateSurfaceOutput().ConnectToSource(
        sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

    # physics
    pr = mesh.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(pr)
    UsdPhysics.CollisionAPI.Apply(pr)
    UsdPhysics.MeshCollisionAPI.Apply(pr).CreateApproximationAttr().Set("convexHull")
    UsdPhysics.MassAPI.Apply(pr).CreateMassAttr(mass)
    return pr

def metalize(stage, root_path, rgb=(0.68, 0.70, 0.73), rough=0.28, metal=0.95):
    """Bind one metallic material to every mesh under root_path."""
    from pxr import Sdf, UsdShade
    mp = root_path + "/_metal"
    mat = UsdShade.Material.Define(stage, Sdf.Path(mp))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(mp + "/surf"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    mat.CreateSurfaceOutput().ConnectToSource(
        sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    # The MJCF importer does not nest geometry under the robot xform -- it puts
    # it in top-level /meshes, /visuals scopes. So sweep the whole stage and take
    # every mesh that is not the workspace, the crate or the ground.
    skip = ("/World/Astra", "/World/crate", "/World/ground", "/World/table",
            "/World/CratesVis", "/World/_crate")
    n = 0
    for pr in stage.Traverse():
        if not pr.IsA(UsdGeom.Mesh):
            continue
        pth = pr.GetPath().pathString
        if any(pth.startswith(k) for k in skip):
            continue
        UsdShade.MaterialBindingAPI.Apply(pr).Bind(mat)
        n += 1
    return n

def plastic_mat(stage, path, rgb, rough=0.34):
    """Moulded-plastic look: non-metallic, semi-glossy."""
    from pxr import Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + "/s"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.4)
    mat.CreateSurfaceOutput().ConnectToSource(
        sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def pile_dx(c, grp="pile"):
    """Per-crate x nudge.

    conveyor: the two crates on the deck sit at world x 3.83 and 4.72, right
    where the G1 sets the box down (it lands at x 5.28 and was riding up on
    them, settling at z 0.96 rather than 0.92). Slide them 0.5 m down the belt
    to clear the pile-side end.

    pile: red is pulled toward the greys; the middle grey stack is pulled
    toward the far one so grey-grey : grey-red lands at 2:1 (0.41 / 0.20).
    Layout x is red 5.19..5.99, middle grey 6.7..7.5, far grey 8.4..9.2 -- note
    the conveyor crates are at 2.11/3.00 and would otherwise match the
    "< 8.0" middle-grey test, which is why the group has to be passed in."""
    if grp == "conveyor":
        return -0.50
    if c["red"]:
        return 0.20
    return 0.0933 if c["t"][0] < 8.0 else 0.0


def place_crates(stage, offset):
    """Re-place the workspace crates as direct references (the nested ones do not draw)."""
    import json
    from pxr import Sdf, UsdShade
    lay = json.load(open("/root/astra_demo/crate_layout.json"))
    asset = "/root/astra_workspace/assets/crates/euro_crate_600x400x120.usd"

    # the originals report correct bounds but never render -- switch them off so
    # we do not end up with invisible duplicates in the physics/bbox picture
    for grp in ("/World/Astra/Crates", "/World/Astra/CratePile"):
        gp = stage.GetPrimAtPath(grp)
        if gp and gp.IsValid():
            gp.SetActive(False)

    grey = plastic_mat(stage, "/World/_crate_grey", (0.58, 0.59, 0.62))
    red = plastic_mat(stage, "/World/_crate_red", (0.55, 0.08, 0.08))

    root = UsdGeom.Scope.Define(stage, Sdf.Path("/World/CratesVis"))
    n = 0
    for grp in ("conveyor", "pile"):
        for c in lay[grp]:
            _dx = pile_dx(c, grp)
            pth = f"/World/CratesVis/{grp}_{c['n']}"
            add_reference_to_stage(usd_path=asset, prim_path=pth)
            pr = stage.GetPrimAtPath(pth)
            xf = UsdGeom.Xformable(pr)
            xf.AddTranslateOp().Set(Gf.Vec3d(c["t"][0] + offset[0] + _dx,
                                             c["t"][1] + offset[1],
                                             c["t"][2] + offset[2]))
            q = c["q"]
            xf.AddOrientOp().Set(Gf.Quatf(q[0], Gf.Vec3f(q[1], q[2], q[3])))
            # strongerThanDescendants: otherwise the crate asset's own mesh-level
            # material wins and every pile comes out grey
            UsdShade.MaterialBindingAPI.Apply(pr).Bind(
                red if c["red"] else grey,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            n += 1
    return n

def fill_candy(stage, offset, seed=11):
    """Red candy pellets in the top-layer crates of each pile."""
    import json, math, random
    from pxr import Sdf, UsdShade, Vt

    lay = json.load(open("/root/astra_demo/crate_layout.json"))
    pile = lay["pile"]
    if not pile:
        return 0, 0
    top_z = max(c["t"][2] for c in pile)
    tops = [c for c in pile if abs(c["t"][2] - top_z) < 1e-3]

    rng = random.Random(seed)
    R = 0.007                       # pellet radius (~14 mm granules)
    # crate interior, local frame: outer extent is (-0.3,-0.2,0)..(0.3,0.2,0.12)
    ix, iy = 0.265, 0.165           # inset from the walls
    z0, z1 = 0.072, 0.107           # thin visible top layer

    pts, idx = [], []
    for c in tops:
        # piles are rotated 90 deg about z, so swap the local axes
        q = c["q"]
        rot90 = abs(abs(q[0]) - math.sqrt(0.5)) < 1e-2
        ax, ay = (iy, ix) if rot90 else (ix, iy)
        n = 1100
        # blue sweets in the red crates, red sweets in the grey ones
        base = 3 if c["red"] else 0
        for _ in range(n):
            idx.append(base + rng.randrange(3))
            pts.append((
                c["t"][0] + offset[0] + pile_dx(c) + rng.uniform(-ax, ax),
                c["t"][1] + offset[1] + rng.uniform(-ay, ay),
                c["t"][2] + offset[2] + rng.uniform(z0, z1),
            ))

    root = Sdf.Path("/World/Candy")
    pi = UsdGeom.PointInstancer.Define(stage, root)

    # three red-ish prototypes so the fill is not flat
    # first three are red sweets (grey crates), last three blue (red crates)
    protos, shades = [], [(0.72, 0.06, 0.08), (0.62, 0.10, 0.12), (0.80, 0.14, 0.10),
                          (0.10, 0.24, 0.72), (0.14, 0.34, 0.80), (0.08, 0.18, 0.60)]
    for i, rgb in enumerate(shades):
        sp = UsdGeom.Sphere.Define(stage, root.AppendChild(f"proto{i}"))
        sp.CreateRadiusAttr(R)
        sp.CreateExtentAttr(Vt.Vec3fArray([(-R, -R, -R), (R, R, R)]))
        mat = UsdShade.Material.Define(stage, root.AppendChild(f"m{i}"))
        sh = UsdShade.Shader.Define(stage, root.AppendChild(f"m{i}").AppendChild("s"))
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.12)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(
            sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
        UsdShade.MaterialBindingAPI.Apply(sp.GetPrim()).Bind(mat)
        protos.append(sp.GetPath())

    pi.CreatePrototypesRel().SetTargets(protos)
    pi.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    pi.CreateProtoIndicesAttr(Vt.IntArray(idx))
    return len(tops), len(pts)

def fill_hopper(stage, seed=13):
    """Mixed red/orange/blue pellet layer in the tipper trough.

    World-coord geometry probed with OC_PROBE2 (valid for ASTRA_OFFSET
    (1.72, -4.75)): compartments x 5.84..7.18 and 7.34..8.68, interior
    y -1.16..-0.84, open top tilted from z 1.03 (y -0.72) to 1.47 (y -1.30).
    The layer follows that tilt ~7 cm below the rim so it reads as full.
    """
    import random
    from pxr import Sdf, UsdShade, Vt

    rng = random.Random(seed)
    R = 0.0095
    root = Sdf.Path("/World/HopperCandy")
    pi = UsdGeom.PointInstancer.Define(stage, root)

    shades = [(0.72, 0.06, 0.08), (0.80, 0.14, 0.10),   # red
              (0.93, 0.42, 0.04), (0.85, 0.33, 0.07),   # orange
              (0.10, 0.24, 0.72), (0.14, 0.34, 0.80)]   # blue
    protos = []
    for i, rgb in enumerate(shades):
        sp = UsdGeom.Sphere.Define(stage, root.AppendChild(f"proto{i}"))
        sp.CreateRadiusAttr(R)
        sp.CreateExtentAttr(Vt.Vec3fArray([(-R, -R, -R), (R, R, R)]))
        mat = UsdShade.Material.Define(stage, root.AppendChild(f"m{i}"))
        sh = UsdShade.Shader.Define(stage, root.AppendChild(f"m{i}").AppendChild("s"))
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.12)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(
            sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
        UsdShade.MaterialBindingAPI.Apply(sp.GetPrim()).Bind(mat)
        protos.append(sp.GetPath())

    pts, idx = [], []
    for x0, x1 in ((5.84, 7.18), (7.34, 8.68)):
        for _ in range(5200):
            x = rng.uniform(x0, x1)
            y = rng.uniform(-1.16, -0.84)
            z = 1.03 - 0.759 * (y + 0.72) - 0.07 + rng.uniform(-0.025, 0.02)
            pts.append((x, y, z))
            idx.append(rng.randrange(len(shades)))

    pi.CreatePrototypesRel().SetTargets(protos)
    pi.CreatePositionsAttr(Vt.Vec3fArray([Gf.Vec3f(*q) for q in pts]))
    pi.CreateProtoIndicesAttr(Vt.IntArray(idx))
    return len(pts)


def main():
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0,
                  rendering_dt=1.0 / 50.0)
    stage = omni.usd.get_context().get_stage()
    world.scene.add(GroundPlane(prim_path="/World/ground", size=80.0))
    gp = stage.GetPrimAtPath("/World/ground")
    if gp and gp.IsValid():
        UsdGeom.Imageable(gp).CreateVisibilityAttr("invisible")   # collision stays
    # NO extra lights. astra_workspace.usd already ships a tuned rig
    # (DomeLight 320, KeyLight distant 900 @ 6 deg) matched to the supplier
    # reference render at 0.0% blown highlights. Adding a second dome+key on
    # top was over-lighting the scene ~3x and washing it out.

    # ---- ASTRA workspace (visual + its own collision geometry) ----
    add_reference_to_stage(usd_path=ASTRA_USD, prim_path="/World/Astra")
    UsdGeom.Xformable(stage.GetPrimAtPath("/World/Astra")).AddTranslateOp().Set(
        Gf.Vec3d(*ASTRA_OFFSET))
    # Make the workspace VISUAL-ONLY. Its collision meshes (tipper machine at
    # x 3.93..7.11, y 3.32..4.10, guard rails, floor slab) were tripping the G1:
    # it drifted into the machine footprint and fell every run. The policy walks
    # on the flat GroundPlane instead; ASTRA supplies the look of the station.
    n_off = n_keep = 0
    astra_root = stage.GetPrimAtPath("/World/Astra")
    for pp in Usd.PrimRange(astra_root):
        if not pp.HasAPI(UsdPhysics.CollisionAPI):
            continue
        # Keep the conveyor solid -- the box is placed ON its deck, so it needs
        # something to land on. Everything else stays visual-only.
        if "/Conveyor" in pp.GetPath().pathString:
            n_keep += 1
            continue
        UsdPhysics.CollisionAPI(pp).CreateCollisionEnabledAttr(False)
        n_off += 1
    log(f"[astra] collision: disabled {n_off}, kept {n_keep} (conveyor)")
    # ---- dark upper wall above the 3.11 m panels: low camera angles were
    # seeing void through the strip above the back wall (33-118 black rows).
    # A horizontal ceiling is wrong here -- it blocks the dome AND the 6-deg
    # key and crushes the scene to luma ~11. A vertical slab at the wall
    # plane blocks only the sightline, not the light.
    _uw = UsdGeom.Cube.Define(stage, "/World/upperwall")
    _ux = UsdGeom.Xformable(_uw.GetPrim())
    _ux.AddTranslateOp().Set(Gf.Vec3d(7.0, -4.85, 5.0))
    _ux.AddScaleOp().Set(Gf.Vec3f(35.0, 0.05, 2.1))
    UsdGeom.Gprim(_uw.GetPrim()).CreateDisplayColorAttr([Gf.Vec3f(0.13, 0.13, 0.14)])
    log("[astra] upper wall added (z 2.9..7.1 at y -4.85)")

    # ---- realism pass (reference image) ----
    from pxr import UsdShade as _USm, Sdf as _Sdf
    # concrete texture on every material literally named "Concrete" (the
    # pillar meshes carry primvars:st, so a UV texture maps directly)
    _ncc = 0
    for _pr in Usd.PrimRange(stage.GetPrimAtPath("/World/Astra")):
        if _pr.GetTypeName() != "Material" or _pr.GetName() != "Concrete":
            continue
        _mp = _pr.GetPath()
        _surf = None
        for _c in Usd.PrimRange(_pr):
            if _c.IsA(_USm.Shader) and _USm.Shader(_c).GetIdAttr().Get() == "UsdPreviewSurface":
                _surf = _USm.Shader(_c)
                break
        if _surf is None:
            continue
        _rd = _USm.Shader.Define(stage, _mp.AppendChild("stReader"))
        _rd.CreateIdAttr("UsdPrimvarReader_float2")
        _rd.CreateInput("varname", _Sdf.ValueTypeNames.Token).Set("st")
        _tx = _USm.Shader.Define(stage, _mp.AppendChild("concTex"))
        _tx.CreateIdAttr("UsdUVTexture")
        _tx.CreateInput("file", _Sdf.ValueTypeNames.Asset).Set(
            "/root/astra_workspace/assets/textures/concrete.png")
        _tx.CreateInput("wrapS", _Sdf.ValueTypeNames.Token).Set("repeat")
        _tx.CreateInput("wrapT", _Sdf.ValueTypeNames.Token).Set("repeat")
        _tx.CreateInput("st", _Sdf.ValueTypeNames.Float2).ConnectToSource(
            _rd.CreateOutput("result", _Sdf.ValueTypeNames.Float2))
        _surf.CreateInput("diffuseColor", _Sdf.ValueTypeNames.Color3f).ConnectToSource(
            _tx.CreateOutput("rgb", _Sdf.ValueTypeNames.Float3))
        _surf.CreateInput("roughness", _Sdf.ValueTypeNames.Float).Set(0.85)
        _ncc += 1
    log(f"[astra] concrete texture bound into {_ncc} materials")

    # glossier floor: reflections of railings/banners like the reference
    _fs = _USm.Shader(stage.GetPrimAtPath("/World/Astra/Looks/Floor/surface"))
    if _fs:
        _fs.CreateInput("roughness", _Sdf.ValueTypeNames.Float).Set(0.22)
        log("[astra] floor roughness -> 0.22")

    # moodier balance, tunable: OC_DOME / OC_KEY
    _dome = float(os.environ.get("OC_DOME", "170"))
    _key = float(os.environ.get("OC_KEY", "1000"))
    stage.GetPrimAtPath("/World/Astra/DomeLight").GetAttribute("inputs:intensity").Set(_dome)
    stage.GetPrimAtPath("/World/Astra/KeyLight").GetAttribute("inputs:intensity").Set(_key)
    log(f"[astra] lights: dome {_dome} key {_key}")

    # match the guard-rail yellow to the conveyor's safety amber
    from pxr import UsdShade as _US
    _CONV_YELLOW = Gf.Vec3f(0.980, 0.750, 0.020)
    _ny = 0
    for _pr in stage.Traverse():
        if not _pr.IsA(_US.Shader):
            continue
        if "/_materials/Material5/" not in _pr.GetPath().pathString:
            continue
        _sh = _US.Shader(_pr)
        _i = _sh.GetInput("diffuseColor")
        if _i is not None:
            _i.Set(_CONV_YELLOW)
            _ny += 1
    log(f"[astra] retinted {_ny} rail shaders to conveyor yellow")

    _nc = place_crates(stage, ASTRA_OFFSET)
    log(f"[astra] placed {_nc} plastic euro crates (direct refs)")
    # The piles are scenery -- nothing is ever placed on them, and leaving them
    # solid puts collision geometry right where the G1 works the pile side.
    _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                            ["default", "render", "proxy", "guide"])
    _cv = stage.GetPrimAtPath("/World/CratesVis")
    _rng = _bc.ComputeWorldBound(_cv).ComputeAlignedRange()
    if not _rng.IsEmpty():
        _mn, _mx = _rng.GetMin(), _rng.GetMax()
        log(f"[astra] crate piles world bounds x {_mn[0]:.2f}..{_mx[0]:.2f} "
            f"y {_mn[1]:.2f}..{_mx[1]:.2f} z {_mn[2]:.2f}..{_mx[2]:.2f}")
    # Do NOT disable collision on these: they are rigid bodies resting on the
    # floor, so removing their colliders drops them through it and leaves the
    # candy PointInstancer hanging in mid-air. They are far from the robot lane
    # anyway -- the pile-side collapse was the body/world velocity-frame bug.
    _ncr, _npel = fill_candy(stage, ASTRA_OFFSET)
    log(f"[astra] candy: {_npel} pellets across {_ncr} top crates")
    _nh = fill_hopper(stage)
    log(f"[astra] hopper: {_nh} mixed pellets in the trough")

    # ---- G1 ----
    _, icfg = omni.kit.commands.execute("MJCFCreateImportConfig")
    icfg.set_fix_base(False)
    icfg.set_import_inertia_tensor(True)
    icfg.set_make_default_prim(False)
    omni.kit.commands.execute(
        "MJCFCreateAsset", mjcf_path=f"{OC_ROOT}/g1_description/g1_29dof.xml",
        import_config=icfg, prim_path="/World/G1")
    for pp in list(stage.Traverse()):
        if pp.GetPath().pathString.endswith("worldBody") and \
                pp.HasAPI(UsdPhysics.ArticulationRootAPI):
            pp.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    roots = [pp.GetPath().pathString for pp in stage.Traverse()
             if pp.HasAPI(UsdPhysics.ArticulationRootAPI)
             and pp.GetPath().pathString.startswith("/World/G1")]
    log(f"[astra] G1 articulation root: {roots}")
    _hy = G1_YAW / 2.0
    robot = world.scene.add(SingleArticulation(
        prim_path=roots[0], name="g1",
        position=np.array([G1_XY[0], G1_XY[1], 0.793]),
        orientation=np.array([np.cos(_hy), 0.0, 0.0, np.sin(_hy)])))

    # ---- table the crate starts on ----
    if False:
        world.scene.add(FixedCuboid(
            prim_path="/World/table", name="table",
            position=np.array([TABLE_XY[0], TABLE_XY[1], TABLE_TOP / 2.0]),
            scale=np.array([0.50, 0.70, TABLE_TOP]),
            color=np.array([0.55, 0.55, 0.58])))

    # ---- the euro crate ----
    if A.plain_box:
        make_carton(stage, "/World/crate", CRATE_HALF, 1.5,
                    "/root/astra_workspace/assets/textures/cardboard.png")
        crate = world.scene.add(SingleRigidPrim(
            prim_path="/World/crate", name="crate",
            position=np.array([TABLE_XY[0], TABLE_XY[1], CRATE_Z])))
        log("[astra] cardboard carton created")
    else:
        add_reference_to_stage(usd_path=CRATE_USD, prim_path="/World/crate")
        cp = stage.GetPrimAtPath("/World/crate")
        # the asset is geometry only -- make it a proper dynamic rigid body
        UsdPhysics.RigidBodyAPI.Apply(cp)
        UsdPhysics.CollisionAPI.Apply(cp)
        mass = UsdPhysics.MassAPI.Apply(cp)
        mass.CreateMassAttr(1.5)
        for child in cp.GetChildren():
            if child.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(child)
                mc = UsdPhysics.MeshCollisionAPI.Apply(child)
                mc.CreateApproximationAttr().Set("convexHull")
        crate = world.scene.add(SingleRigidPrim(
            prim_path="/World/crate", name="crate",
            position=np.array([TABLE_XY[0], TABLE_XY[1], CRATE_Z])))
        log("[astra] euro crate USD added as rigid body")

    world.reset()

    # after reset the MJCF geometry is fully realised, so binding sticks
    _nm = metalize(stage, "/World/G1")
    log(f"[astra] metallic finish bound to {_nm} meshes")

    if os.environ.get("OC_PROBE") == "1":
        _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                ["default", "render", "proxy", "guide"])
        sx, sy = G1_XY
        hits = []
        for _pp in stage.Traverse():
            if not _pp.HasAPI(UsdPhysics.CollisionAPI):
                continue
            _en = UsdPhysics.CollisionAPI(_pp).GetCollisionEnabledAttr()
            if _en and not _en.Get():
                continue
            _r = _bc.ComputeWorldBound(_pp).ComputeAlignedRange()
            if _r.IsEmpty():
                continue
            _mn, _mx = _r.GetMin(), _r.GetMax()
            dx = max(_mn[0] - sx, 0.0, sx - _mx[0])
            dy = max(_mn[1] - sy, 0.0, sy - _mx[1])
            d = (dx * dx + dy * dy) ** 0.5
            if d < 2.0:
                hits.append((d, _pp.GetPath().pathString, _mn, _mx))
        hits.sort()
        log(f"[probe] {len(hits)} collision-enabled prims within 2 m of spawn {G1_XY}")
        for d, pth, mn, mx in hits[:15]:
            log(f"[probe]  d={d:.2f}  x {mn[0]:.2f}..{mx[0]:.2f} "
                f"y {mn[1]:.2f}..{mx[1]:.2f} z {mn[2]:.2f}..{mx[2]:.2f}  {pth}")
        simulation_app.close()
        return

    if os.environ.get("OC_PROBE2") == "1":
        from pxr import UsdShade as _USh
        _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                ["default", "render", "proxy", "guide"])
        L = []
        L.append("== /World/Astra children ==")
        for _ch in stage.GetPrimAtPath("/World/Astra").GetChildren():
            _r = _bc.ComputeWorldBound(_ch).ComputeAlignedRange()
            if _r.IsEmpty():
                L.append(f"  {_ch.GetName()}  (empty bbox)")
                continue
            _mn, _mx = _r.GetMin(), _r.GetMax()
            L.append(f"  {_ch.GetName():24s} x {_mn[0]:7.2f}..{_mx[0]:7.2f} "
                     f"y {_mn[1]:6.2f}..{_mx[1]:6.2f} z {_mn[2]:5.2f}..{_mx[2]:5.2f}")
        L.append("== VERTICAL SLABS (z-range>2, xy footprint<2, meshes anywhere) ==")
        for _pp in Usd.PrimRange(stage.GetPrimAtPath("/World/Astra")):
            if not _pp.IsA(UsdGeom.Mesh):
                continue
            _r = _bc.ComputeWorldBound(_pp).ComputeAlignedRange()
            if _r.IsEmpty():
                continue
            _mn, _mx = _r.GetMin(), _r.GetMax()
            if (_mx[2] - _mn[2]) > 2.0 and (_mx[0] - _mn[0]) < 2.0 and (_mx[1] - _mn[1]) < 2.0:
                _st = UsdGeom.PrimvarsAPI(_pp).HasPrimvar("st")
                _mat = _USh.MaterialBindingAPI(_pp).ComputeBoundMaterial()[0]
                L.append(f"  st={_st} x {_mn[0]:6.2f}..{_mx[0]:6.2f} y {_mn[1]:6.2f}..{_mx[1]:6.2f} "
                         f"z {_mn[2]:.2f}..{_mx[2]:.2f} mat={_mat.GetPath().name if _mat else None} "
                         f" {_pp.GetPath()}")
        L.append("== MACHINE-REGION MESHES (x 4..9, y -5..1, top z>0.9) ==")
        for _pp in Usd.PrimRange(stage.GetPrimAtPath("/World/Astra")):
            if not _pp.IsA(UsdGeom.Mesh):
                continue
            _r = _bc.ComputeWorldBound(_pp).ComputeAlignedRange()
            if _r.IsEmpty():
                continue
            _mn, _mx = _r.GetMin(), _r.GetMax()
            cx, cy = (_mn[0]+_mx[0])/2, (_mn[1]+_mx[1])/2
            if 4.0 < cx < 9.0 and -5.0 < cy < 1.0 and _mx[2] > 0.9:
                L.append(f"  x {_mn[0]:.2f}..{_mx[0]:.2f} y {_mn[1]:.2f}..{_mx[1]:.2f} "
                         f"z {_mn[2]:.2f}..{_mx[2]:.2f}  {_pp.GetPath()}")
        L.append("== Floor shader inputs ==")
        _fm = stage.GetPrimAtPath("/World/Astra/Looks/Floor")
        for _pp in Usd.PrimRange(_fm):
            if _pp.IsA(_USh.Shader):
                _sh = _USh.Shader(_pp)
                ins = {i.GetBaseName(): i.Get() for i in _sh.GetInputs()}
                L.append(f"  {_pp.GetPath()}  id={_sh.GetIdAttr().Get()}  {ins}")
        open("/root/probe2_out.txt", "w").write("\n".join(str(x) for x in L))
        simulation_app.close()
        return

    dof = list(robot.dof_names)
    isaac2mj = np.array([dof.index(j) for j in MJ_JOINTS], dtype=np.int32)
    log(f"[astra] {len(dof)} dofs mapped")

    runner = build_runner("carrybox")
    policy, sc, po = runner.policy, runner.state_cmd, runner.policy_output
    log(f"[astra] policy ready | task={policy.task} "
        f"init={TABLE_XY}+z{CRATE_Z} goal={GOAL}")

    writer = None
    if A.record:
        import imageio
        from isaacsim.core.utils.viewports import set_camera_view
        import omni.replicator.core as rep
        import carb
        _st = carb.settings.get_settings()
        if os.environ.get("OC_PATHTRACE", "") == "1":
            spp = int(os.environ.get("OC_SPP", "64"))
            _st.set("/rtx/rendermode", "PathTracing")
            _st.set("/rtx/pathtracing/spp", 1)
            _st.set("/rtx/pathtracing/totalSpp", spp)
            _st.set("/rtx/pathtracing/maxBounces", 6)
            _st.set("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 6)
            _st.set("/rtx/pathtracing/optixDenoiser/enabled", True)
            log(f"[astra] PATH TRACING enabled (totalSpp={spp})")
        else:
            # still nicer than bare defaults for the real-time path
            _st.set("/rtx/reflections/enabled", True)
            _st.set("/rtx/ambientOcclusion/enabled", True)
            _st.set("/rtx/directLighting/sampledLighting/enabled", True)
        import carb
        _st = carb.settings.get_settings()
        if os.environ.get("OC_PATHTRACE", "") == "1":
            spp = int(os.environ.get("OC_SPP", "64"))
            _st.set("/rtx/rendermode", "PathTracing")
            _st.set("/rtx/pathtracing/spp", 1)
            _st.set("/rtx/pathtracing/totalSpp", spp)
            _st.set("/rtx/pathtracing/maxBounces", 6)
            _st.set("/rtx/pathtracing/maxSpecularAndTransmissionBounces", 6)
            _st.set("/rtx/pathtracing/optixDenoiser/enabled", True)
            log(f"[astra] PATH TRACING enabled (totalSpp={spp})")
        else:
            # still nicer than bare defaults for the real-time path
            _st.set("/rtx/reflections/enabled", True)
            _st.set("/rtx/ambientOcclusion/enabled", True)
            _st.set("/rtx/directLighting/sampledLighting/enabled", True)
        # Aiming any higher than this tilts the top of the frame off the building
        # wall into empty space, which renders as a black band (OC_CAMSWEEP=1
        # measures it: tgt z 0.50 -> 0 rows, 0.85 -> 59, 1.30 -> 127). The dead
        # floor at the bottom is cropped off in run_astra_demo.sh instead.
        set_camera_view(eye=[7.20, 6.30, 3.40], target=[6.90, -0.90, 1.00])
        _w, _h = (int(v) for v in os.environ.get("OC_RES", "1280x720").split("x"))
        rp = rep.create.render_product("/OmniverseKit_Persp", (_w, _h))
        log(f"[astra] render {_w}x{_h}")
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach(rp)
        writer = imageio.get_writer(A.record, fps=30)

        if os.environ.get("OC_CAMSWEEP") == "1":
            cands = [
                ("C0", [7.90, 8.40, 4.20], [8.00, 1.00, 0.60]),   # current
                ("C1", [7.00, 6.90, 3.70], [6.90, -0.60, 0.95]),
                ("C2", [7.20, 6.30, 3.40], [6.90, -0.90, 1.00]),
                ("C3", [6.70, 5.90, 3.10], [6.80, -1.20, 1.05]),
                ("C4", [7.60, 6.60, 3.00], [6.90, -1.00, 1.10]),
                ("C5", [7.40, 5.60, 2.40], [6.80, -1.40, 1.15]),
            ]
            for nm, eye, tgt in cands:
                set_camera_view(eye=eye, target=tgt)
                for _ in range(12):
                    world.step(render=True)
                img = np.asarray(annot.get_data())[..., :3]
                imageio.imwrite(f"/root/cam_{nm}.png", img)
                g = img.mean(axis=2)
                dark = g.mean(axis=1) < 12
                top = int(np.argmax(~dark)) if dark.any() else 0
                _lum = g.mean()
                _blown = float((g > 247).mean()) * 100
                _crush = float((g < 8).mean()) * 100
                log(f"[cam] {nm} eye={eye} tgt={tgt} -> black_top={top} "
                    f"luma={_lum:.1f} blown={_blown:.2f}% crushed={_crush:.2f}%")
            simulation_app.close()
            return


    gains_set = False
    frames = 0
    phase = "carry"
    stand_off = False
    loco = None
    pose_until = 0
    from policy.loco_mode.LocoMode import LocoMode
    from policy.defaultpose.DefaultPose import DefaultPose
    cycle = 0
    from policy.omnicontact.CFgen_reference import plan_cfgen_reference
    cycle = 0
    from policy.omnicontact.CFgen_reference import plan_cfgen_reference
    hold_action = None          # last carry action, held across the switch
    blend_i, BLEND_N = 0, 50    # policy ticks to cross-fade (50*4 steps = 1 s)
    still = 0
    prev_box = None
    HOME = (0.0, 0.0)
    for step in range(A.max_steps):
        q = robot.get_joint_positions()
        dq = robot.get_joint_velocities()
        sc.q[:] = q[isaac2mj]
        sc.dq[:] = dq[isaac2mj]
        bpos, bquat = robot.get_world_pose()
        sc.base_pos[:] = bpos
        sc.base_quat[:] = bquat
        w, x, y, z = bquat
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        # Isaac reports base velocities in the WORLD frame; the policy was
        # written against MuJoCo qvel[3:6]/[0:3], which are in the BODY frame.
        # These coincide at yaw~0, so feeding world velocities looked fine for
        # every run along +x -- but at yaw 170-180 the x/y components are
        # sign-flipped and the robot collapses within a second.
        sc.ang_vel[:] = R.T @ robot.get_angular_velocity()
        sc.lin_vel[:] = R.T @ robot.get_linear_velocity()
        sc.gravity_ori[:] = R.T @ np.array([0.0, 0.0, -1.0])

        opos, oquat = crate.get_world_pose()
        sc.obj_pos[:] = opos
        sc.obj_quat[:] = oquat
        sc.carry_box_pos[:] = opos
        sc.carry_box_quat[:] = oquat

        # The stand only has to hold the box until it is picked up. It sits at
        # (1.5, 0), right on the path to the conveyor at (3.3, 0), and the robot
        # trips over it mid-carry. Once the box is up, drop its collision.
        if phase == "carry" and opos[2] > 0.60 and not stand_off:
            tp = stage.GetPrimAtPath("/World/table")
            if tp and tp.IsValid():
                for q in Usd.PrimRange(tp):
                    if q.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
                stand_off = True
                log(f"[astra] box lifted -- stand collision off @{step}")

        # --- once the box is placed and has stopped moving, walk home ---
        if phase == "carry" and step > 600:
            if prev_box is not None and np.linalg.norm(opos - prev_box) < 2e-4:
                still += 1
            else:
                still = 0
            prev_box = opos.copy()
            placed = (opos[2] > 0.5 and
                      np.linalg.norm(opos[:2] - np.array(GOAL[:2])) < 0.6)
            if SWAP_AFTER_PLACE and placed and still > 250:
                log(f"[astra] placed at {np.round(opos,2)} -- clearing collisions, "
                    f"hot-swapping to LocoMode")
                # the robot is boxed in: conveyor edge at x=2.90, box at z=0.92
                # right beside it. Free it so it can turn and walk back.
                # freeze the box where it landed. Disabling its collision would
                # drop it through the (now non-solid) conveyor and it free-falls.
                pr = stage.GetPrimAtPath("/World/crate")
                if pr and pr.IsValid():
                    rb = UsdPhysics.RigidBodyAPI(pr)
                    if rb:
                        rb.CreateKinematicEnabledAttr(True)
                for q in Usd.PrimRange(stage.GetPrimAtPath("/World/Astra")):
                    if "/Conveyor" in q.GetPath().pathString and \
                            q.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
                # documented FSM order: settle to DefaultPose first, then walk
                hold_action = np.asarray(po.actions, dtype=np.float32).copy()
                dp = DefaultPose(sc, po)
                dp.enter()
                policy = dp
                blend_i = 0
                pose_until = 10**9   # stay in DefaultPose: LocoMode handover is unstable
                gains_set = False
                phase = "pose"

            if False:   # let the robot settle into a neutral stance first
                cycle += 1
                log(f"[astra] cycle {cycle}: placed at {np.round(opos,2)} "
                    f"-- respawning box on the stand")
                # box reappears where it started
                crate.set_world_pose(
                    position=np.array([TABLE_XY[0], TABLE_XY[1], CRATE_Z]),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0]))
                crate.set_linear_velocity(np.zeros(3))
                crate.set_angular_velocity(np.zeros(3))
                # stand must be solid again to hold it
                tp = stage.GetPrimAtPath("/World/table")
                if tp and tp.IsValid():
                    for q in Usd.PrimRange(tp):
                        if q.HasAPI(UsdPhysics.CollisionAPI):
                            UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(True)
                stand_off = False
                # re-plan a fresh carry FROM THE ROBOT'S CURRENT POSE, in place
                sc.obj_pos[:] = [TABLE_XY[0], TABLE_XY[1], CRATE_Z]
                sc.obj_quat[:] = [1.0, 0.0, 0.0, 0.0]
                sc.carry_box_pos[:] = sc.obj_pos
                sc.carry_box_quat[:] = sc.obj_quat
                policy.goal_pos = np.array(GOAL, dtype=np.float32)
                plan_cfgen_reference(policy, policy._get_fk_info())
                policy.counter_step = 0
                still, prev_box = 0, None

        if phase == "pose" and step >= pose_until:
            log(f"[astra] default pose settled @{step} -- entering LocoMode")
            hold_action = np.asarray(po.actions, dtype=np.float32).copy()
            loco = LocoMode(sc, po)
            loco.enter()
            policy = loco
            gains_set = False
            blend_i = 0
            phase = "home"

        if phase == "home":
            # steer LocoMode back to the origin
            d = np.array([0.0, 0.0]) - bpos[:2]
            dist = float(np.linalg.norm(d))
            wq = bquat
            yaw = np.arctan2(2 * (wq[0] * wq[3] + wq[1] * wq[2]),
                             1 - 2 * (wq[2] ** 2 + wq[3] ** 2))
            desired = np.arctan2(d[1], d[0])
            err = (desired - yaw + np.pi) % (2 * np.pi) - np.pi
            sc.vel_cmd[0] = float(np.clip(dist, 0.0, 0.45)) if abs(err) < 0.6 else 0.0
            sc.vel_cmd[1] = 0.0
            sc.vel_cmd[2] = float(np.clip(err, -0.6, 0.6))
            if dist < 0.35:
                sc.vel_cmd[:] = 0.0

        if step % 4 == 0:
            try:
                policy.run()
            except Exception as e:
                log(f"[astra] policy.run() failed @{step}: {type(e).__name__}: {e}")
                break
            if not gains_set:
                kp = np.zeros(len(dof), dtype=np.float32)
                kd = np.zeros(len(dof), dtype=np.float32)
                kp[isaac2mj] = np.asarray(po.kps, dtype=np.float32)
                kd[isaac2mj] = np.asarray(po.kds, dtype=np.float32)
                if kp.max() > 0:
                    robot.get_articulation_controller().set_gains(kps=kp, kds=kd)
                    gains_set = True
                    log("[astra] drive gains applied")

        act = np.asarray(po.actions, dtype=np.float32)
        if hold_action is not None and blend_i < BLEND_N:
            if step % 4 == 0:
                blend_i += 1
            w = blend_i / float(BLEND_N)
            act = (1.0 - w) * hold_action + w * act
        tgt = np.zeros(len(dof), dtype=np.float32)
        tgt[isaac2mj] = act
        robot.apply_action(ArticulationAction(joint_positions=tgt))

        _capture = writer is not None and step % A.record_every == 0
        world.step(render=_capture)
        if _capture and os.environ.get("OC_PATHTRACE", "") == "1":
            # let the path tracer converge on this frame before grabbing it
            for _ in range(int(os.environ.get("OC_ACCUM", "8"))):
                simulation_app.update()
        if writer and step % A.record_every == 0:
            d = annot.get_data()
            if d is not None and len(d):
                writer.append_data(np.asarray(d)[:, :, :3])
                frames += 1
        if step % 300 == 0:
            _yaw = np.degrees(np.arctan2(2*(bquat[0]*bquat[3]+bquat[1]*bquat[2]),
                                        1-2*(bquat[2]**2+bquat[3]**2)))
            log(f"[astra] step {step:4d} [{phase}] g1={np.round(bpos,2)} "
                f"yaw={_yaw:6.1f} crate={np.round(opos,2)}")

    if writer:
        writer.close()
        log(f"[astra] wrote {frames} frames")
    log("[astra] done")
    simulation_app.close()


main()
