"""OmniContact G1 carry policy driven by Isaac Sim physics.

Key idea: build the repo's OWN OmniContactRunner (which loads config, resolves
the ONNX path, and applies all the per-task policy setup correctly), then ignore
its MuJoCo model and drive `runner.policy` from Isaac Sim state instead.

Everything policy-side -- CFgen reference generation, the 1244-dim observation,
the ONNX net -- is reused byte-for-byte. The adapter only:
  Isaac Sim state -> StateAndCmd (mujoco joint order) -> policy.run()
  -> PolicyOutput.actions (mujoco order) -> Isaac Sim joint targets

Isaac Sim's articulation order was verified to be exactly the policy's "lab"
order, and the permutation is still resolved by NAME so it cannot silently drift.
"""
import argparse
import os
import sys

cli = argparse.ArgumentParser()
cli.add_argument("--task", default="carrybox")
cli.add_argument("--policy", default="policy.onnx")
cli.add_argument("--init-pos", nargs=2, type=float, default=[1.0, 0.0])
cli.add_argument("--goal-pos", nargs=2, type=float, default=[2.5, 0.5])
cli.add_argument("--max-steps", type=int, default=2400)
cli.add_argument("--record", default="/root/oc_isaac.mp4")
cli.add_argument("--record-every", type=int, default=10)
A = cli.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402

_DIAG = open("/root/oc_isaac_diag.txt", "w", buffering=1)


def log(*a):
    m = " ".join(str(x) for x in a)
    _DIAG.write(m + "\n")
    _DIAG.flush()


import omni.usd  # noqa: E402
import omni.kit.commands  # noqa: E402
from pxr import UsdLux, UsdPhysics  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import DynamicCuboid, GroundPlane  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402

OC_ROOT = "/root/OmniContact_sim2sim"
sys.path.insert(0, OC_ROOT)
sys.path.insert(0, os.path.join(OC_ROOT, "deploy_omnicontact"))
os.chdir(OC_ROOT)

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


def build_runner():
    """Construct the repo's runner so the policy gets its real task config."""
    from omnicontact_runner_args import parse_args
    from run_skill_omnicontact import OmniContactRunner

    argv = sys.argv
    sys.argv = [
        "run_skill_omnicontact.py",
        "--reference-source", "CFgen",
        "--policy", A.policy,
        "--task", A.task,
        "--init-pos", str(A.init_pos[0]), str(A.init_pos[1]),
        "--goal-pos", str(A.goal_pos[0]), str(A.goal_pos[1]),
        "--headless",
    ]
    try:
        rargs = parse_args()
    finally:
        sys.argv = argv
    runner = OmniContactRunner(rargs)
    runner._prepare_episode()
    return runner


def main():
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 200.0,
                  rendering_dt=1.0 / 50.0)
    world.scene.add(GroundPlane(prim_path="/World/ground", size=50.0))
    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, "/World/light").CreateIntensityAttr(3000.0)

    _, icfg = omni.kit.commands.execute("MJCFCreateImportConfig")
    icfg.set_fix_base(False)
    icfg.set_import_inertia_tensor(True)
    icfg.set_make_default_prim(False)
    omni.kit.commands.execute(
        "MJCFCreateAsset",
        mjcf_path=f"{OC_ROOT}/g1_description/g1_29dof.xml",
        import_config=icfg, prim_path="/World/G1")

    for pp in list(stage.Traverse()):
        p = pp.GetPath().pathString
        if p.endswith("worldBody") and pp.HasAPI(UsdPhysics.ArticulationRootAPI):
            pp.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    roots = [pp.GetPath().pathString for pp in stage.Traverse()
             if pp.HasAPI(UsdPhysics.ArticulationRootAPI)]
    log(f"[isaac] articulation root: {roots}")

    robot = world.scene.add(SingleArticulation(
        prim_path=roots[0], name="g1", position=np.array([0.0, 0.0, 0.793])))
    box = world.scene.add(DynamicCuboid(
        prim_path="/World/box", name="box",
        position=np.array([A.init_pos[0], A.init_pos[1], 0.15]),
        scale=np.array([0.30, 0.30, 0.30]), mass=1.0,
        color=np.array([0.85, 0.35, 0.12])))
    world.reset()

    dof = list(robot.dof_names)
    isaac2mj = np.array([dof.index(j) for j in MJ_JOINTS], dtype=np.int32)
    log(f"[isaac] {len(dof)} dofs, permutation built")

    runner = build_runner()
    policy = runner.policy
    sc = runner.state_cmd
    po = runner.policy_output
    log(f"[isaac] runner built | task={policy.task} onnx ok")

    writer = None
    if A.record:
        import imageio
        from isaacsim.core.utils.viewports import set_camera_view
        import omni.replicator.core as rep
        set_camera_view(eye=[3.2, -3.2, 2.0], target=[1.6, 0.2, 0.5])
        rp = rep.create.render_product("/OmniverseKit_Persp", (1280, 720))
        annot = rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach(rp)
        writer = imageio.get_writer(A.record, fps=30)
        log(f"[isaac] recording -> {A.record}")

    decim = 4
    frames = 0
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

        opos, oquat = box.get_world_pose()
        for a, v in (("obj_pos", opos), ("carry_box_pos", opos)):
            getattr(sc, a)[:] = v
        for a, v in (("obj_quat", oquat), ("carry_box_quat", oquat)):
            getattr(sc, a)[:] = v

        if step % decim == 0:
            try:
                policy.run()
                # The MJCF import leaves joint drives with no usable stiffness, so
                # position targets are ignored and the robot ragdolls. The policy
                # publishes the gains it was trained with -- push them into the
                # articulation once (mujoco order -> isaac order).
                if not getattr(main, "_gains_set", False):
                    kp = np.zeros(len(dof), dtype=np.float32)
                    kd = np.zeros(len(dof), dtype=np.float32)
                    kp[isaac2mj] = np.asarray(po.kps, dtype=np.float32)
                    kd[isaac2mj] = np.asarray(po.kds, dtype=np.float32)
                    if kp.max() > 0:
                        robot.get_articulation_controller().set_gains(
                            kps=kp, kds=kd)
                        log(f"[isaac] drive gains set: kp[{kp.min():.0f}..{kp.max():.0f}] "
                            f"kd[{kd.min():.1f}..{kd.max():.1f}]")
                        main._gains_set = True
            except Exception as e:
                log(f"[isaac] policy.run() failed at step {step}: {type(e).__name__}: {e}")
                break

        tgt = np.zeros(len(dof), dtype=np.float32)
        tgt[isaac2mj] = np.asarray(po.actions, dtype=np.float32)
        robot.apply_action(ArticulationAction(joint_positions=tgt))

        world.step(render=(writer is not None and step % A.record_every == 0))
        if writer and step % A.record_every == 0:
            d = annot.get_data()
            if d is not None and len(d):
                writer.append_data(np.asarray(d)[:, :, :3])
                frames += 1

        if step % 300 == 0:
            log(f"[isaac] step {step:4d} base={np.round(bpos,2)} box={np.round(opos,2)}")

    if writer:
        writer.close()
        log(f"[isaac] wrote {frames} frames -> {A.record}")
    log("[isaac] done")
    simulation_app.close()


main()
