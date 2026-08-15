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
ASTRA_OFFSET = (1.72, -5.10, 0.0)
LOOP = A.loop
SWAP_AFTER_PLACE = False   # see notes: every post-carry policy swap falls

CRATE_HALF = tuple(A.half_dims) if A.half_dims else (0.30, 0.20, 0.06)
CRATE_XY = (1.50, 0.00)                    # 1.5 m ahead of the robot (trained geometry)
TABLE_TOP = 0.35                           # low stand the box starts on
CRATE_Z = CRATE_HALF[2]                    # box on the floor (known-good)
CONVEYOR_TOP = 0.77                        # roller top, workspace z
GOAL = (3.30, 0.00, CONVEYOR_TOP + CRATE_HALF[2])   # ON the conveyor deck
G1_XY = (0.00, 0.00)
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
    r._prepare_episode()
    return r


def main():
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0,
                  rendering_dt=1.0 / 50.0)
    stage = omni.usd.get_context().get_stage()
    world.scene.add(GroundPlane(prim_path="/World/ground", size=80.0))
    UsdLux.DistantLight.Define(stage, "/World/keylight").CreateIntensityAttr(1800.0)
    dome = UsdLux.DomeLight.Define(stage, "/World/dome")
    dome.CreateIntensityAttr(500.0)

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
    robot = world.scene.add(SingleArticulation(
        prim_path=roots[0], name="g1",
        position=np.array([G1_XY[0], G1_XY[1], 0.793])))

    # ---- table the crate starts on ----
    if False:
        world.scene.add(FixedCuboid(
            prim_path="/World/table", name="table",
            position=np.array([TABLE_XY[0], TABLE_XY[1], TABLE_TOP / 2.0]),
            scale=np.array([0.50, 0.70, TABLE_TOP]),
            color=np.array([0.55, 0.55, 0.58])))

    # ---- the euro crate ----
    if A.plain_box:
        crate = world.scene.add(DynamicCuboid(
            prim_path="/World/crate", name="crate",
            position=np.array([TABLE_XY[0], TABLE_XY[1], CRATE_Z]),
            scale=np.array([CRATE_HALF[0] * 2, CRATE_HALF[1] * 2, CRATE_HALF[2] * 2]),
            mass=1.5, color=np.array([0.55, 0.55, 0.57])))
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
        set_camera_view(eye=[0.1, 6.2, 2.5], target=[2.4, -1.2, 0.75])
        rp = rep.create.render_product("/OmniverseKit_Persp", (1280, 720))
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach(rp)
        writer = imageio.get_writer(A.record, fps=30)

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
        sc.ang_vel[:] = robot.get_angular_velocity()
        sc.lin_vel[:] = robot.get_linear_velocity()
        w, x, y, z = bquat
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
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

        world.step(render=(writer is not None and step % A.record_every == 0))
        if writer and step % A.record_every == 0:
            d = annot.get_data()
            if d is not None and len(d):
                writer.append_data(np.asarray(d)[:, :, :3])
                frames += 1
        if step % 300 == 0:
            log(f"[astra] step {step:4d} [{phase}] g1={np.round(bpos,2)} crate={np.round(opos,2)}")

    if writer:
        writer.close()
        log(f"[astra] wrote {frames} frames")
    log("[astra] done")
    simulation_app.close()


main()
