"""OmniContact G1 carry policy running inside Isaac Sim.

Design: the policy (CFgen reference generator + ONNX net) is reused COMPLETELY
UNCHANGED. The only new code is an adapter that

  1. reads robot + object state from Isaac Sim,
  2. writes it into the repo's plain-numpy StateAndCmd struct in MUJOCO joint
     order (so every downstream index in the policy stays valid),
  3. takes PolicyOutput.actions (also mujoco order) back into Isaac Sim order
     and applies them as PD position targets.

Joint order is resolved BY NAME, never by a hardcoded list, because Isaac Sim's
articulation order is its own thing.

FK for the observation still uses the repo's MujocoKinematics, which is a pure
kinematics helper (q + base pose -> body poses); no MuJoCo simulation involved.
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="carrybox")
parser.add_argument("--policy", default="policy.onnx")
parser.add_argument("--init-pos", nargs=2, type=float, default=[1.0, 0.0])
parser.add_argument("--goal-pos", nargs=2, type=float, default=[2.5, 0.5])
parser.add_argument("--max-steps", type=int, default=2000)
parser.add_argument("--record", default="/root/oc_isaac.mp4")
parser.add_argument("--record-every", type=int, default=8)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402

_DIAG = open("/root/oc_isaac_diag.txt", "w", buffering=1)
def log(*a):
    msg = " ".join(str(x) for x in a)
    _DIAG.write(msg + "\n"); _DIAG.flush()
    print(msg, flush=True)

import omni.usd  # noqa: E402
import omni.kit.commands  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, GroundPlane  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402

OC_ROOT = "/root/OmniContact_sim2sim"
sys.path.insert(0, OC_ROOT)
os.chdir(OC_ROOT)

G1_USD = "/root/g1_29dof_from_mjcf.usd"

# MuJoCo ACTUATED joint order (excludes floating_base_joint) -- this is the order
# StateAndCmd.q must be in for the policy's mj2lab indexing to be correct.
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


def quat_wxyz(q):
    return np.asarray(q, dtype=np.float32)


def main():
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0,
                  rendering_dt=1.0 / 200.0)
    world.scene.add(GroundPlane(prim_path="/World/ground", size=50.0))

    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DistantLight.Define(stage, "/World/light")
    light.CreateIntensityAttr(2500.0)

    # Import the MJCF straight into the LIVE stage. Going via a file produces a
    # stub USD with configuration/* sublayers whose physics layer does not
    # compose through a reference (articulation roots come back empty).
    _, icfg = omni.kit.commands.execute("MJCFCreateImportConfig")
    icfg.set_fix_base(False)
    icfg.set_import_inertia_tensor(True)
    icfg.set_make_default_prim(False)
    omni.kit.commands.execute(
        "MJCFCreateAsset",
        mjcf_path="/root/OmniContact_sim2sim/g1_description/g1_29dof.xml",
        import_config=icfg,
        prim_path="/World/G1",
    )
    log("[isaac] MJCF imported into live stage")
    all_paths = [pp.GetPath().pathString for pp in stage.Traverse()]
    log(f"[isaac] stage prims: {len(all_paths)}; sample: {all_paths[:12]}")
    # The MJCF importer leaves TWO ArticulationRootAPI prims: the real robot root
    # at .../pelvis/pelvis and a spurious one on worldBody. Two roots make the
    # physics view come back None ("NoneType has no attribute is_homogeneous").
    # Discover articulation roots rather than hardcoding: the MJCF importer path
    # layout changes once the asset is referenced under a new prim path.
    roots = [pp.GetPath().pathString for pp in stage.Traverse()
             if pp.HasAPI(UsdPhysics.ArticulationRootAPI)]
    log(f"[isaac] articulation roots found: {roots}")
    # drop any spurious worldBody root -- two roots make the physics view None
    for r in roots:
        if r.endswith("worldBody"):
            pr = stage.GetPrimAtPath(r)
            pr.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            log(f"[isaac] removed spurious ArticulationRootAPI at {r}")
    roots = [r for r in roots if not r.endswith("worldBody")]
    if not roots:
        log("[isaac] FATAL: no articulation root found"); simulation_app.close(); return
    root_path = roots[0]
    log(f"[isaac] using articulation root: {root_path}")
    robot = world.scene.add(SingleArticulation(prim_path=root_path,
                                               name="g1",
                                               position=np.array([0.0, 0.0, 0.80])))

    box = world.scene.add(DynamicCuboid(
        prim_path="/World/box", name="box",
        position=np.array([args.init_pos[0], args.init_pos[1], 0.15]),
        scale=np.array([0.30, 0.30, 0.30]), mass=1.0,
        color=np.array([0.85, 0.35, 0.12])))

    world.reset()

    dof_names = list(robot.dof_names)
    log(f"[isaac] articulation dofs = {len(dof_names)}")
    for i, n in enumerate(dof_names):
        log(f"    isaac[{i:2d}] {n}")

    missing = [j for j in MJ_JOINTS if j not in dof_names]
    if missing:
        log(f"[isaac] FATAL: joints missing from articulation: {missing}")
        simulation_app.close()
        return

    # permutations, resolved by name
    isaac2mj = np.array([dof_names.index(j) for j in MJ_JOINTS], dtype=np.int32)
    mj2isaac = np.argsort(isaac2mj)
    log(f"[isaac] built isaac<->mj permutation ({len(isaac2mj)} joints)")

    # --- policy, reused unchanged -----------------------------------------
    from common.ctrlcomp import StateAndCmd, PolicyOutput
    from policy.omnicontact.OmniContact import OmniContact
    import yaml

    with open(os.path.join(OC_ROOT, "policy/omnicontact/config/OmniContact.yaml")) as f:
        cfg = yaml.safe_load(f)

    state_cmd = StateAndCmd(num_joints=29)
    policy_output = PolicyOutput(num_joints=29)
    policy = OmniContact(state_cmd, policy_output, cfg)
    log("[isaac] OmniContact policy constructed")

    for attr, val in (("task", args.task),
                      ("goal_pos", np.array([*args.goal_pos, 0.0], dtype=np.float32))):
        if hasattr(policy, attr):
            setattr(policy, attr, val)
    if hasattr(policy, "enter"):
        policy.enter()

    writer = None
    if args.record:
        import imageio
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=[3.5, -3.5, 2.2], target=[1.5, 0.0, 0.6])
        import omni.replicator.core as rep
        rp = rep.create.render_product("/OmniverseKit_Persp", (1280, 720))
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach(rp)
        writer = imageio.get_writer(args.record, fps=30)
        log(f"[isaac] recording -> {args.record}")

    decim = 4
    for step in range(args.max_steps):
        # ---- Isaac Sim state -> StateAndCmd (mujoco joint order) ----
        q_isaac = robot.get_joint_positions()
        dq_isaac = robot.get_joint_velocities()
        state_cmd.q[:] = q_isaac[isaac2mj]
        state_cmd.dq[:] = dq_isaac[isaac2mj]

        base_pos, base_quat = robot.get_world_pose()          # quat is wxyz
        state_cmd.base_pos[:] = base_pos
        state_cmd.base_quat[:] = quat_wxyz(base_quat)

        lin_vel = robot.get_linear_velocity()
        ang_vel = robot.get_angular_velocity()
        state_cmd.lin_vel[:] = lin_vel
        state_cmd.ang_vel[:] = ang_vel
        # projected gravity in base frame
        w, x, y, z = base_quat
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        state_cmd.gravity_ori[:] = R.T @ np.array([0.0, 0.0, -1.0])

        bp, bq = box.get_world_pose()
        state_cmd.obj_pos[:] = bp
        state_cmd.obj_quat[:] = quat_wxyz(bq)
        state_cmd.carry_box_pos[:] = bp
        state_cmd.carry_box_quat[:] = quat_wxyz(bq)

        # ---- policy tick ----
        if step % decim == 0:
            policy.run()

        # ---- PolicyOutput (mujoco order) -> Isaac Sim ----
        tgt_mj = np.asarray(policy_output.actions, dtype=np.float32)
        tgt_isaac = np.zeros(len(dof_names), dtype=np.float32)
        tgt_isaac[isaac2mj] = tgt_mj
        robot.set_joint_position_targets(tgt_isaac)

        world.step(render=bool(writer) and step % args.record_every == 0)

        if writer and step % args.record_every == 0:
            data = annot.get_data()
            if data is not None and len(data):
                writer.append_data(np.asarray(data)[:, :, :3])

        if step % 200 == 0:
            log(f"[isaac] step {step:4d} base={np.round(base_pos,2)} "
                  f"box={np.round(bp,2)}")

    if writer:
        writer.close()
        log("[isaac] recording closed")
    simulation_app.close()


main()
