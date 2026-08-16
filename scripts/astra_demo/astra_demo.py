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
import math
import os
import sys

# ---- path resolution -------------------------------------------------------
# Nothing below may hardcode /root: this file has to run both from a git clone
# on a fresh machine and from the original layout on the L4 box. Each path is
# env var -> vendored copy inside the repo -> legacy /root.
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))   # <repo> when run from a clone


def _pick(env, *candidates):
    v = os.environ.get(env)
    if v:
        return v
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return candidates[-1]


OC_ROOT = _pick("OC_ROOT",
                os.path.join(_REPO, "vendor", "omnicontact_g1"),
                "/root/omnicontact_g1")
ASTRA_WS = _pick("ASTRA_WS",
                 os.path.join(_REPO, "vendor", "astra_workspace"),
                 "/root/astra_workspace")
# writable place for the video, diagnostics and probe dumps
OUT_DIR = os.environ.get("OC_OUT_DIR") or (_HERE if os.access(_HERE, os.W_OK) else "/tmp")

ASTRA_USD = os.path.join(ASTRA_WS, "astra_workspace.usd")
CRATE_USD = os.path.join(ASTRA_WS, "assets", "crates", "euro_crate_600x400x120.usd")
# SM_Crate_A07_Yellow_01 does NOT render in this stage -- verified as a direct
# reference, via its inner SubUSD, and as a visual skin (hiding the collider
# left the pallet visibly empty). Its physics works, so it silently gives an
# invisible crate. It is also authored in centimetres, and SingleRigidPrim
# overwrites the unit-conversion xform op, which makes it 100x too large.
# Deliberately not vendored; use the euro crate below instead.
SM_CRATE = os.path.join(ASTRA_WS, "assets", "Isaac", "Props", "PackingTable",
                        "props", "SM_Crate_A07_Yellow_01", "SubUSDs",
                        "SM_Crate_A07_01_1.usd")
# the long side of the belt instead of its end.
CONV_ROT = float(os.environ.get("OC_CONV_ROT", "90"))
# Pivot for the prim ops, calibrated by measurement: the conveyor sits under
# transforms whose composition order is not obvious, so the pivot that lands
# the belt back on its own centre was solved from three probe renders --
#   centre(px,py) = (5.62,-1.01) + px*(-1,1) + py*(-1,-1)
# and (4.26, 0.35) needs (1.36, 0.00).
CONV_CX = float(os.environ.get("OC_CONV_PX", "1.36"))
CONV_CY = float(os.environ.get("OC_CONV_PY", "0.00"))
# The deck crates are placed from LAYOUT coordinates, so they rotate about the
# conveyor centre expressed in that frame instead.
CONV_CTR_WS = (2.54, 5.10)

# Links the real G1 paints black, read from Unitree's g1_29dof_rev_1_0.urdf
# (material "dark", rgba 0.2). Matched against the prim path, so the trailing
# _link is dropped where the MJCF names differ. The rubber hands are black on
# the hardware and every hand link is dark in the inspire-hand URDF, so they
# are in the set even though this URDF variant leaves them white.
DARK_LINKS = ("pelvis", "hip_pitch", "ankle_roll", "logo", "head",
              "hand", "palm")

PILE_PALLET_H = 0.144      # euro pallet under each crate pile
TEX_DIR = os.path.join(ASTRA_WS, "assets", "textures")
LAYOUT_JSON = os.path.join(_HERE, "crate_layout.json")

cli = argparse.ArgumentParser()
cli.add_argument("--max-steps", type=int, default=3000)
cli.add_argument("--record", default=os.path.join(OUT_DIR, "oc_astra.mp4"))
cli.add_argument("--record-every", type=int, default=10)
cli.add_argument("--half-dims", nargs=3, type=float, default=None)
cli.add_argument("--loop", action="store_true", help="respawn the box and repeat")
cli.add_argument("--skin-height", type=float, default=None,
                 help="visual crate height (m); collider keeps its own")
cli.add_argument("--crate-skin", default=None,
                 help='visual-only crate mesh over the physics box; "sm" selects SM_Crate_A07')
cli.add_argument("--crate-yaw", type=float, default=0.0,
                 help="spawn yaw of the picked crate, degrees")
cli.add_argument("--crate-usd", default=None,
                 help='crate asset to pick; "sm" selects SM_Crate_A07_Yellow_01')
cli.add_argument("--plain-box", action="store_true",
                 help="use a plain cuboid instead of the euro crate USD")
A = cli.parse_args()

# CRATE_USD is resolved above, before argparse exists, so apply the
# --crate-usd override here instead.
if A.crate_usd:
    CRATE_USD = SM_CRATE if A.crate_usd == "sm" else A.crate_usd

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402

_DIAG = open(os.path.join(OUT_DIR, "oc_astra_diag.txt"), "w", buffering=1)


def log(*a):
    m = " ".join(str(x) for x in a)
    _DIAG.write(m + "\n")
    _DIAG.flush()


import omni.usd  # noqa: E402
import omni.kit.commands  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdLux  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, GroundPlane  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

sys.path.insert(0, OC_ROOT)
sys.path.insert(0, os.path.join(OC_ROOT, "deploy_omnicontact"))
os.chdir(OC_ROOT)


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
CRATE_XY = (9.00, LANE_Y)                  # on the pallet, 2.0 m from the robot
TABLE_TOP = 0.35                           # low stand the box starts on
# The grasp reference puts BOTH hands at the object CENTRE height
# (CFgen_meta2_carrybox._grasp_targets: contact_center = obj_pos). The policy
# was trained on a 0.30 m cube resting on the floor, i.e. a centre at z 0.15.
# A 0.17 m crate laid on the floor has its centre at 0.085 and the hands are
# asked to meet at floor level, so the grasp never lands. Keep the centre at
# the trained height and put a pallet under anything shorter.
# Height the crate centre is presented at. 0.15 matches the trained cube on
# the floor; raising it puts the crate on a stand so the robot can reach in
# standing upright instead of crouched, which is what limits forward reach.
GRASP_CENTRE_Z = float(os.environ.get("OC_GRASP_Z", "0.15"))
CRATE_Z = max(CRATE_HALF[2], GRASP_CENTRE_Z)
PALLET_H = CRATE_Z - CRATE_HALF[2]         # 0 for the 0.30 cube, 0.065 for SM
CONVEYOR_TOP = 0.77                        # roller top, workspace z
# Rotated, the deck is only 1.16 m wide in x (3.68..4.84) and the crate lands
# 0.25-0.75 m short of the goal depending on where the robot stops. Aiming at
# 3.75 puts it mid-deck across that whole range; 4.15 let it land on the edge
# and topple off.
GOAL = (3.75, LANE_Y, CONVEYOR_TOP + CRATE_HALF[2])  # ON the deck; it overshoots ~0.28 m,
                                                     # so aim in from the x=5.62 edge
G1_XY = (7.00, LANE_Y)                     # between the table and the conveyor
G1_YAW = 0.0                               # facing +x, toward the table
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


def skin_crate(stage, prim_path, asset, target_full, visual_h=None):
    """Scale a crate mesh onto the physics box as a visual-only child.

    No collision is applied to the skin -- the box underneath keeps its convex
    hull, so the grasp the policy sees is exactly the one it was trained on.
    """
    from pxr import Sdf, UsdShade
    skin = prim_path + "/skin"
    add_reference_to_stage(usd_path=asset, prim_path=skin)
    pr = stage.GetPrimAtPath(skin)
    xf = UsdGeom.Xformable(pr)
    xf.ClearXformOpOrder()                       # measure the raw asset first
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           ["default", "render", "proxy", "guide"])
    r = bb.ComputeLocalBound(pr).ComputeAlignedRange()
    mn, mx = np.array(r.GetMin()), np.array(r.GetMax())
    size = np.maximum(mx - mn, 1e-6)
    # Footprint fills the collider; height may be shorter than it.
    tf = np.array(target_full, dtype=float)
    sc = tf / size
    if visual_h:
        sc[2] = float(visual_h) / size[2]
    ctr = (mn + mx) / 2.0
    # ops are [translate, scale] -> p * S + T. x/y centre on the box origin;
    # z is BOTTOM-aligned to the collider's underside, so a shorter visual
    # still sits flush on whatever the collider is resting on instead of
    # floating half-way up it.
    t = -ctr * sc
    t[2] = -tf[2] / 2.0 - mn[2] * sc[2]
    xf.AddTranslateOp().Set(Gf.Vec3d(*t))
    xf.AddScaleOp().Set(Gf.Vec3f(*sc))
    # VISUAL ONLY. The crate asset ships CollisionAPI/RigidBodyAPI of its own,
    # which would silently add a second collider to the body and change the
    # grasp physics -- the robot toppled mid-carry until these were switched
    # off. The hidden carton child is the only collider.
    for _pp in Usd.PrimRange(pr):
        if _pp.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(_pp).CreateCollisionEnabledAttr(False)
        if _pp.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(_pp).CreateRigidBodyEnabledAttr(False)

    # the collider box is hidden with visibility, not opacity -- see the
    # crate construction in main(); RTX draws opacity-0 UsdPreviewSurface solid
    return size, sc


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

def shade_g1(stage, shell_rgb=None, dark_rgb=(0.045, 0.045, 0.05)):
    """Shade the G1 like the real robot: glossy grey plastic shell, black
    head and hands.

    Head/hand meshes are matched on prim path -- the MJCF link names carry
    through, so head_link, *_rubber_hand, *_hand_*_link and *_palm_link all
    hit. The wrist_pitch/roll/yaw links deliberately do not, since those are
    grey on the real robot.
    """
    from pxr import UsdShade
    # 0.79 photographed as white; 0.52 keeps the plastic sheen but reads as
    # grey. OC_G1_GREY overrides without editing.
    if shell_rgb is None:
        _g = float(os.environ.get("OC_G1_GREY", "0.52"))
        shell_rgb = (_g, _g * 1.01, _g * 1.04)
    shell = plastic_mat(stage, "/World/_g1_shell", shell_rgb, rough=0.15)
    # matte: high roughness and low specular, so it reads as the real robot's
    # black rather than piano lacquer
    black = plastic_mat(stage, "/World/_g1_black", dark_rgb, rough=0.80,
                        spec=0.12)
    # The MJCF importer does not nest geometry under the robot xform -- it puts
    # it in top-level /meshes, /visuals scopes. So sweep the whole stage and take
    # every mesh that is not the workspace, the crate or the ground.
    skip = ("/World/Astra", "/World/crate", "/World/ground", "/World/table",
            "/World/CratesVis", "/World/_crate")
    n_shell = n_dark = 0
    for pr in stage.Traverse():
        if not pr.IsA(UsdGeom.Mesh):
            continue
        pth = pr.GetPath().pathString
        if any(pth.startswith(k) for k in skip):
            continue
        low = pth.lower()
        is_dark = any(k in low for k in DARK_LINKS)
        UsdShade.MaterialBindingAPI.Apply(pr).Bind(
            black if is_dark else shell,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        n_dark += is_dark
        n_shell += not is_dark
    return n_shell, n_dark

def plastic_mat(stage, path, rgb, rough=0.34, spec=0.4):
    """Moulded-plastic look: non-metallic, semi-glossy."""
    from pxr import Sdf, UsdShade
    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + "/s"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(spec)
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
    # Equal 0.30 m gaps. Base world footprints are red 6.71..7.91,
    # mid grey 8.22..9.42, far grey 9.92..11.12 -- gaps of 0.31 and 0.50.
    if c["red"]:
        return 0.21
    return 0.20 if c["t"][0] < 8.0 else 0.0


def place_crates(stage, offset):
    """Re-place the workspace crates as direct references (the nested ones do not draw)."""
    import json, math
    from pxr import Sdf, UsdShade
    lay = json.load(open(LAYOUT_JSON))
    # the piles always use the euro crate, even when the robot picks something
    # else, so this must not follow --crate-usd
    asset = os.path.join(ASTRA_WS, "assets", "crates", "euro_crate_600x400x120.usd")

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
    # The two crates that ship on the deck sit exactly where the robot now
    # places, once the belt is turned 90 degrees. Drop them.
    _groups = ("pile",) if os.environ.get("OC_DECK_CRATES", "0") != "1" \
        else ("conveyor", "pile")
    for grp in _groups:
        for c in lay[grp]:
            _x = c["t"][0] + pile_dx(c, grp)
            _y = c["t"][1]
            _q = Gf.Quatf(c["q"][0], Gf.Vec3f(c["q"][1], c["q"][2], c["q"][3]))
            if grp == "conveyor" and abs(CONV_ROT) > 1e-6:
                # these sit ON the belt, so they have to turn with it: rotate
                # the position about the conveyor centre and spin the crate by
                # the same angle
                _a = math.radians(CONV_ROT)
                _px, _py = CONV_CTR_WS
                _vx, _vy = _x - _px, _y - _py
                _x = _px + _vx * math.cos(_a) - _vy * math.sin(_a)
                _y = _py + _vx * math.sin(_a) + _vy * math.cos(_a)
                _q = Gf.Quatf(math.cos(_a / 2),
                              Gf.Vec3f(0.0, 0.0, math.sin(_a / 2))) * _q
            pth = f"/World/CratesVis/{grp}_{c['n']}"
            add_reference_to_stage(usd_path=asset, prim_path=pth)
            pr = stage.GetPrimAtPath(pth)
            xf = UsdGeom.Xformable(pr)
            _zl = PILE_PALLET_H if grp == "pile" else 0.0
            xf.AddTranslateOp().Set(Gf.Vec3d(_x + offset[0], _y + offset[1],
                                             c["t"][2] + offset[2] + _zl))
            xf.AddOrientOp().Set(_q)
            # strongerThanDescendants: otherwise the crate asset's own mesh-level
            # material wins and every pile comes out grey
            UsdShade.MaterialBindingAPI.Apply(pr).Bind(
                red if c["red"] else grey,
                bindingStrength=UsdShade.Tokens.strongerThanDescendants)
            # The piles are scenery: make them STATIC. Left dynamic and lifted
            # onto pallets, 108 crates free-fall 0.144 m on frame 1 and the
            # solver disturbance knocks the robot over during the grasp.
            # Disabling collision instead would drop them through the floor.
            for _q in Usd.PrimRange(pr):
                if _q.HasAPI(UsdPhysics.RigidBodyAPI):
                    UsdPhysics.RigidBodyAPI(_q).CreateRigidBodyEnabledAttr(False)
            n += 1
    return n

def fill_candy(stage, offset, seed=11):
    """Red candy pellets in the top-layer crates of each pile."""
    import json, math, random
    from pxr import Sdf, UsdShade, Vt

    lay = json.load(open(LAYOUT_JSON))
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
                c["t"][2] + offset[2] + PILE_PALLET_H + rng.uniform(z0, z1),
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
    # ---- rotate the conveyor 90 degrees about its own centre --------------
    # Appended, not prepended: USD applies xformOpOrder left to right on a row
    # vector, so M = existing * T(-c) * R * T(+c) puts the rotation in the
    # parent frame, after the asset's own placement.
    _cvp = stage.GetPrimAtPath("/World/Astra/Conveyor")
    if _cvp and _cvp.IsValid() and abs(CONV_ROT) > 1e-6:
        _cv = UsdGeom.Xformable(_cvp)
        _cv.AddTranslateOp(opSuffix="rotPivotNeg").Set(Gf.Vec3d(-CONV_CX, -CONV_CY, 0.0))
        _cv.AddRotateZOp(opSuffix="rot90").Set(CONV_ROT)
        _cv.AddTranslateOp(opSuffix="rotPivotPos").Set(Gf.Vec3d(CONV_CX, CONV_CY, 0.0))
        log(f"[astra] conveyor rotated {CONV_ROT:g} deg about ws "
            f"({CONV_CX}, {CONV_CY})")
        _bbz = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                 ["default", "render", "proxy", "guide"])
        _rz = _bbz.ComputeWorldBound(_cvp).ComputeAlignedRange()
        if not _rz.IsEmpty():
            _m0, _m1 = _rz.GetMin(), _rz.GetMax()
            log(f"[astra] conveyor world bbox x {_m0[0]:.2f}..{_m1[0]:.2f} "
                f"y {_m0[1]:.2f}..{_m1[1]:.2f} z {_m0[2]:.2f}..{_m1[2]:.2f}")

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
            os.path.join(TEX_DIR, "concrete.png"))
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
        # A crate already on the belt, at the far end -- the robot places at
    # ~(4.41, 0.32), so y 1.30 keeps it well clear while staying on the deck
    # (which spans y -1.01..1.71 once the conveyor is turned 90 degrees).
    _dk = "/World/CratesVis/deck_far"
    add_reference_to_stage(
        usd_path=os.path.join(ASTRA_WS, "assets", "crates",
                              "euro_crate_600x400x120.usd"),
        prim_path=_dk)
    _dkp = stage.GetPrimAtPath(_dk)
    _dkx = UsdGeom.Xformable(_dkp)
    _dkx.AddTranslateOp().Set(Gf.Vec3d(4.26, 1.30, 0.77))
    # Match the crate the robot places: same size and same orientation. The
    # placed crate ends up yawed ~175 deg, so its 0.40 long axis lies along
    # world x -- this one therefore gets 0 deg, not the 90 it had. Scale is the
    # skin scale, 0.60x0.40x0.12 -> 0.40x0.267x0.13.
    _dkx.AddRotateZOp().Set(0.0)
    _dkx.AddScaleOp().Set(Gf.Vec3f(0.6667, 0.6675, 1.0833))
    for _pp in Usd.PrimRange(_dkp):
        if _pp.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(_pp).CreateRigidBodyEnabledAttr(False)
    from pxr import UsdShade as _USd
    _dgrey = plastic_mat(stage, "/World/_deck_crate_grey", (0.55, 0.56, 0.59),
                         rough=0.28)
    _USd.MaterialBindingAPI.Apply(_dkp).Bind(
        _dgrey, bindingStrength=_USd.Tokens.strongerThanDescendants)
    log("[astra] deck crate: 0.40x0.267x0.13, yaw 0, matching the placed crate")

    log(f"[astra] crate piles world bounds x {_mn[0]:.2f}..{_mx[0]:.2f} "
            f"y {_mn[1]:.2f}..{_mx[1]:.2f} z {_mn[2]:.2f}..{_mx[2]:.2f}")
    # Do NOT disable collision on these: they are rigid bodies resting on the
    # floor, so removing their colliders drops them through it and leaves the
    # candy PointInstancer hanging in mid-air. They are far from the robot lane
    # anyway -- the pile-side collapse was the body/world velocity-frame bug.
    _ncr, _npel = fill_candy(stage, ASTRA_OFFSET)
    log(f"[astra] candy: {_npel} pellets across {_ncr} top crates")
    # ---- pallet + painted floor box under each pile ----------------------
    # Footprints are 1.20 x 1.20 at y 1.55..2.75; after the spacing fix the
    # three sit at x 6.92..8.12, 8.42..9.62, 9.92..11.12.
    from pxr import UsdShade
    _pal = plastic_mat(stage, "/World/_pile_pallet", (0.42, 0.31, 0.20),
                       rough=0.85, spec=0.1)
    _mark = plastic_mat(stage, "/World/_floor_mark", (0.95, 0.76, 0.05),
                        rough=0.55, spec=0.2)
    for _i, _cx in enumerate((7.52, 9.02, 10.52)):
        _cy = 2.15
        _pp = UsdGeom.Cube.Define(stage, f"/World/PileBase/pallet_{_i}")
        _px = UsdGeom.Xformable(_pp.GetPrim())
        _px.AddTranslateOp().Set(Gf.Vec3d(_cx, _cy, PILE_PALLET_H / 2.0))
        # Flush with the 1.20 pile footprint. At 1.28 the pallets nearly
        # touched (pile spacing is 1.50) and read as one continuous slab from
        # above.
        # UsdGeom.Cube spans -1..1 (size 2), so scales here are HALF-extents.
        # Setting 1.20 made 2.4 m pallets that overlapped into one giant slab.
        _px.AddScaleOp().Set(Gf.Vec3f(0.60, 0.60, PILE_PALLET_H / 2.0))
        UsdShade.MaterialBindingAPI.Apply(_pp.GetPrim()).Bind(
            _pal, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        # painted outline on the floor: four flat strips, not a filled slab
        # Outline must clear the pallet (half 0.60) and its neighbour: pile
        # centres are 1.50 apart, so +-0.68 leaves 0.05 between adjacent boxes.
        _oh, _w = 0.68, 0.07
        for _j, (_ox, _oy, _sx, _sy) in enumerate((
                (0.0, +_oh, 2 * _oh + _w, _w), (0.0, -_oh, 2 * _oh + _w, _w),
                (+_oh, 0.0, _w, 2 * _oh + _w), (-_oh, 0.0, _w, 2 * _oh + _w))):
            _mp = UsdGeom.Cube.Define(stage, f"/World/PileBase/mark_{_i}_{_j}")
            _mx = UsdGeom.Xformable(_mp.GetPrim())
            _mx.AddTranslateOp().Set(Gf.Vec3d(_cx + _ox, _cy + _oy, 0.004))
            _mx.AddScaleOp().Set(Gf.Vec3f(_sx / 2.0, _sy / 2.0, 0.004))
            UsdShade.MaterialBindingAPI.Apply(_mp.GetPrim()).Bind(
                _mark, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
    log("[astra] 3 pile pallets + painted floor boxes")

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

    # ---- pallet under the crate, so its centre reaches the grasp height ----
    if PALLET_H > 0.01:
        world.scene.add(FixedCuboid(
            prim_path="/World/table", name="table",
            # Above 0.5 m the stand becomes a FLOATING slab: only the top
            # 0.12 m exists, so the robot can step right up to the crate
            # without its knees fouling table legs, and the slab reads as a
            # suspended platform. Below that, a solid table/pallet as before.
            # Pallet CENTRED under the crate, and that is load-bearing: the
            # grasp itself slides the crate ~0.2 m toward the robot before the
            # lift, so the 0.20 near-side lip is exactly what absorbs it -- the
            # crate ends up ON the edge as it is picked. Every near-edge start
            # tested (flush, 6 cm lip, 6 cm lip + high friction) either slid
            # off, tripped the robot, or killed the grasp.
            position=np.array([TABLE_XY[0],
                               TABLE_XY[1],
                               PALLET_H - 0.06 if PALLET_H > 0.5
                               else PALLET_H / 2.0]),
            scale=np.array([0.70, 0.60, 0.12]) if PALLET_H > 0.5
            else np.array([0.90, 0.70, PALLET_H]) if PALLET_H > 0.20
            else np.array([0.80, 0.60, PALLET_H]),   # euro-pallet footprint
            color=np.array([0.50, 0.51, 0.54]) if PALLET_H > 0.5
            else np.array([0.55, 0.55, 0.58]) if PALLET_H > 0.20
            else np.array([0.42, 0.31, 0.20])))
        log(f"[astra] pallet {PALLET_H:.3f} m under the crate "
            f"(centre -> {CRATE_Z:.3f})")

    # ---- the euro crate ----
    if A.plain_box:
        if A.crate_skin:
            # collider and skin as siblings under one rigid body, so the
            # collider can be hidden without hiding the skin
            _root = UsdGeom.Xform.Define(stage, Sdf.Path("/World/crate"))
            _bp = make_carton(stage, "/World/crate/box", CRATE_HALF, 1.5,
                              os.path.join(TEX_DIR, "cardboard.png"))
            _bp.RemoveAPI(UsdPhysics.RigidBodyAPI)     # body belongs on the parent
            UsdGeom.Imageable(_bp).CreateVisibilityAttr("invisible")
            UsdPhysics.RigidBodyAPI.Apply(_root.GetPrim())
            UsdPhysics.MassAPI.Apply(_root.GetPrim()).CreateMassAttr(1.5)
        else:
            make_carton(stage, "/World/crate", CRATE_HALF, 1.5,
                        os.path.join(TEX_DIR, "cardboard.png"))
        crate = world.scene.add(SingleRigidPrim(
            prim_path="/World/crate", name="crate",
            position=np.array([TABLE_XY[0], TABLE_XY[1], CRATE_Z])))
        CRATE_OFF = np.zeros(3)   # make_carton builds it centred on the origin
        if A.crate_skin:
            _sk = SM_CRATE if A.crate_skin == "sm" else A.crate_skin
            _sz, _sc = skin_crate(stage, "/World/crate", _sk,
                                  [2 * h for h in CRATE_HALF],
                                  visual_h=A.skin_height)
            log(f"[astra] crate skin {os.path.basename(_sk)} "
                f"{np.round(_sz, 3)} -> scale {np.round(_sc, 3)}")
        else:
            log("[astra] cardboard carton created")
    else:
        add_reference_to_stage(usd_path=CRATE_USD, prim_path="/World/crate")
        cp = stage.GetPrimAtPath("/World/crate")
        # the asset is geometry only -- make it a proper dynamic rigid body
        UsdPhysics.RigidBodyAPI.Apply(cp)
        UsdPhysics.CollisionAPI.Apply(cp)
        mass = UsdPhysics.MassAPI.Apply(cp)
        mass.CreateMassAttr(1.5)
        # Recurse: GetChildren() only sees direct children, and assets like
        # SM_Crate_A07 nest their meshes under SubUSDs, so a direct-children
        # sweep gives the crate NO colliders and the hands pass through it.
        _ncol = 0
        for child in Usd.PrimRange(cp):
            if child.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(child)
                mc = UsdPhysics.MeshCollisionAPI.Apply(child)
                mc.CreateApproximationAttr().Set("convexHull")
                _ncol += 1
        log(f"[astra] crate colliders: {_ncol} meshes")
        # These assets put their origin at the BASE, so the geometric centre
        # sits half a height higher. Measure it rather than assume: the policy
        # is fed the centre, and being a half-height low makes every grasp miss.
        _bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                ["default", "render", "proxy", "guide"])
        _r = _bb.ComputeWorldBound(cp).ComputeAlignedRange()
        _mn, _mx = _r.GetMin(), _r.GetMax()
        CRATE_OFF = np.array([(_mn[0] + _mx[0]) / 2.0,
                              (_mn[1] + _mx[1]) / 2.0,
                              (_mn[2] + _mx[2]) / 2.0])
        _cy = math.radians(A.crate_yaw) / 2.0
        crate = world.scene.add(SingleRigidPrim(
            prim_path="/World/crate", name="crate",
            position=np.array([TABLE_XY[0] - CRATE_OFF[0],
                               TABLE_XY[1] - CRATE_OFF[1],
                               CRATE_Z - CRATE_OFF[2]]),
            orientation=np.array([math.cos(_cy), 0.0, 0.0, math.sin(_cy)])))
        _vm = sum(1 for _q in Usd.PrimRange(cp) if _q.IsA(UsdGeom.Mesh))
        _vr = _bb.ComputeWorldBound(cp).ComputeAlignedRange()
        log(f"[astra] crate prim: {_vm} meshes, bbox {np.round(_vr.GetMin(),3)}"
            f"..{np.round(_vr.GetMax(),3)}")
        log(f"[astra] crate USD added as rigid body; origin->centre offset "
            f"{np.round(CRATE_OFF, 4)}")

    world.reset()

    # after reset the MJCF geometry is fully realised, so binding sticks
    # The skin asset ships an MDL material (Plastic_Yellow_A.mdl) that does not
    # resolve here and renders near-black. Bind our own yellow -- and do it
    # AFTER world.reset(), because the reference is not composed before that,
    # so an earlier sweep finds no meshes at all.
    if A.crate_skin:
        from pxr import UsdShade as _USy
        _yl = plastic_mat(stage, "/World/_crate_skin",
                          (0.55, 0.56, 0.59), rough=0.28)
        _nsk = 0
        _skp = stage.GetPrimAtPath("/World/crate/skin")
        if _skp and _skp.IsValid():
            for _pp in Usd.PrimRange(_skp):
                if _pp.IsA(UsdGeom.Mesh):
                    _USy.MaterialBindingAPI.Apply(_pp).Bind(
                        _yl, bindingStrength=_USy.Tokens.strongerThanDescendants)
                    _nsk += 1
        log(f"[astra] crate skin: yellow bound to {_nsk} meshes")

    _ns, _nd = shade_g1(stage)
    log(f"[astra] G1 shaded: {_ns} shell meshes grey, {_nd} head/hand meshes black")

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
        open(os.path.join(OUT_DIR, "probe2_out.txt"), "w").write("\n".join(str(x) for x in L))
        simulation_app.close()
        return

    dof = list(robot.dof_names)
    isaac2mj = np.array([dof.index(j) for j in MJ_JOINTS], dtype=np.int32)
    log(f"[astra] {len(dof)} dofs mapped")

    # Walking speed. plan_cfgen_reference() builds the generator as
    # cfgen_cls(pad=30), so the pace comes from CfGenCarryBox's default
    # step_size_linear -- the spacing between reference waypoints, in metres.
    # Smaller spacing means more frames to cover the same ground, i.e. a slower
    # walk with the gait actually legible. Overriding __defaults__ keeps the
    # vendored source untouched; the signature is (pad, linear, angular).
    _walk = float(os.environ.get("OC_WALK", "0.016"))
    if abs(_walk - 0.016) > 1e-6:
        from policy.omnicontact.CFgen_meta2_carrybox import CfGenCarryBox as _CGC
        _d = list(_CGC.__init__.__defaults__)
        _d[1] = _walk
        _CGC.__init__.__defaults__ = tuple(_d)
        log(f"[astra] walk step {_walk:.4f} m (default 0.016) -> "
            f"{0.016 / _walk:.2f}x slower")

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
        # OC_CAM="ex,ey,ez,tx,ty,tz" overrides the framing, for verification
        # renders of parts of the set the demo camera does not cover.
        _cam = os.environ.get("OC_CAM")
        if _cam:
            _v = [float(x) for x in _cam.split(",")]
            set_camera_view(eye=_v[:3], target=_v[3:6])
            log(f"[astra] camera override {_v}")
        else:
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
                imageio.imwrite(os.path.join(OUT_DIR, f"cam_{nm}.png"), img)
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
    # --- carry weld ---------------------------------------------------------
    welded = False
    weld_off = None      # crate centre in the robot's yaw frame, at grasp time
    weld_dyaw = 0.0      # crate yaw relative to robot yaw, at grasp time
    weld_stall = 0
    weld_done = False    # weld once per carry, never re-grab after release
    prev_bxy = None
    WELD = os.environ.get("OC_WELD", "1") == "1"
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

        _praw, oquat = crate.get_world_pose()
        # prim origin -> geometric centre, rotated into world
        _w, _x, _y, _z = oquat
        _Rc = np.array([
            [1 - 2 * (_y * _y + _z * _z), 2 * (_x * _y - _w * _z), 2 * (_x * _z + _w * _y)],
            [2 * (_x * _y + _w * _z), 1 - 2 * (_x * _x + _z * _z), 2 * (_y * _z - _w * _x)],
            [2 * (_x * _z - _w * _y), 2 * (_y * _z + _w * _x), 1 - 2 * (_x * _x + _y * _y)]])
        opos = _praw + _Rc @ CRATE_OFF
        sc.obj_pos[:] = opos
        sc.obj_quat[:] = oquat
        sc.carry_box_pos[:] = opos
        sc.carry_box_quat[:] = oquat

        # The stand only has to hold the box until it is picked up -- it sits on
        # the path to the conveyor and the robot trips over it mid-carry, so its
        # collision is dropped once the box is clearly off it. The threshold has
        # to be RELATIVE to where the crate started: an absolute 0.60 fires on
        # frame 1 for a crate presented at table height, deleting the stand out
        # from under it.
        # --- keep the carried crate upright ----------------------------
        # The policy squeezes the crate between the palms and nothing resists
        # roll, so it tumbles in transit and lands upside down. Cancel roll and
        # pitch each step, keeping the yaw physics produced, and leave position
        # alone so the policy still does the carrying and the placing.
        #
        # Deliberately NOT a kinematic weld: making the crate kinematic renders
        # it immovable to the solver, the grip reaction throws the robot, and
        # it falls mid-carry.
        # Relative to where the crate started, not absolute: a crate presented
        # on a 0.65 m table is already above any fixed threshold, so an
        # absolute 0.45 clamped its pose from frame 0 while it still sat there.
        # +0.04, not +0.15: the wobble the pick shows lives in the first
        # 15 cm of the lift, before the hold used to engage. Just above the
        # resting height, physics jitter cannot reach it but a real lift
        # trips it within a step or two.
        if WELD and phase == "carry" and opos[2] > CRATE_Z + 0.04:
            _cy = math.atan2(
                2.0 * (oquat[0] * oquat[3] + oquat[1] * oquat[2]),
                1.0 - 2.0 * (oquat[2] ** 2 + oquat[3] ** 2))
            crate.set_world_pose(
                position=_praw,
                orientation=np.array([math.cos(_cy / 2), 0.0, 0.0,
                                      math.sin(_cy / 2)]))
            _av = np.asarray(crate.get_angular_velocity(), dtype=float)
            crate.set_angular_velocity(np.array([0.0, 0.0, _av[2] * 0.2]))
            if not welded:
                welded = True
                log(f"[astra] crate held upright from step {step}")


        if phase == "carry" and opos[2] > CRATE_Z + 0.25 and not stand_off:
            tp = stage.GetPrimAtPath("/World/table")
            if tp and tp.IsValid():
                for q in Usd.PrimRange(tp):
                    if q.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
                stand_off = True
                log(f"[astra] box lifted @{step} -- table collision off so the "
                    f"robot can turn with the crate without fouling it")

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
                    position=np.array([TABLE_XY[0] - CRATE_OFF[0],
                                       TABLE_XY[1] - CRATE_OFF[1],
                                       CRATE_Z - CRATE_OFF[2]]),
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
