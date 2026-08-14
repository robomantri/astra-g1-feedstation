# H1-2 Office Pick-and-Carry — Research & Context

## Goal

Simulate a humanoid robot in the scanned office scene (`office.zip`), have it pick up
a cube from the table, and walk while carrying it. The original request was to
follow the approach in a VnRobo blog post ("Unitree H1-2: Enhanced Locomotion with
New Hardware") — i.e. extend Isaac Lab's H1 locomotion policy to the H1-2 (H1 body +
dexterous 5-finger hands + upgraded arm/leg actuators), extend the observation/reward
to account for the added ~5kg of arm/hand mass, and use a hierarchical
locomotion+arm controller (frozen legs while training the arm, then co-train) to get
walk + reach/grasp behavior.

We have Isaac Sim 5.1.0 / Isaac Lab 4.5.22 installed locally (conda env `env_g1` at
`~/miniconda3/envs/env_g1`), on an RTX 5070 Ti Laptop GPU (12GB, Blackwell
architecture — this matters, see Findings below).

## Scene assets (`office.zip`, extracted to `office/v.20260801/`)

- `Image_1.jpg`, `Image_2.jpg`, `Image_3.jpg` — reference photos of the point-cloud
  scan (a warehouse/production bay: concrete floor, yellow safety railings, a metal
  feed-station machine, roller door, fire extinguisher).
- `USD_&_STL.zip` → `usd_stl/` — the actual environment geometry:
  - `...Cloud scan and 3D model_stl_v.20260730.stl` — scanned mesh (STL)
  - `...Cloud scan and 3D model_usd_v.usdc` — same scene as USD, with `textures/`
    (an `.exr` color texture) — **this is the USD to use as the office layout in
    Isaac Sim**.
- `AstraSweets_RCP_Inkapstation.zip` — Matterport/RealityCapture-style capture
  project (`.rcp`, `.rcs` support files) — the raw scan project, not directly needed
  for Isaac Sim once the USD/STL export exists.
- Scene name suggests this is a real facility scan: "AstraSweets — Robot handling of
  bins at feed stations."

No cube asset was provided — a cube will need to be added to the scene (simple
primitive, placed on the table/machine surface visible in the scan).

## Findings: what already exists vs. what doesn't

### H1-2 specifically
- **Isaac Lab has no native H1-2 support.** Confirmed via open, unresolved GitHub
  issue `isaac-sim/IsaacLab#2324` — no `ArticulationCfg`, no reward config, no
  checkpoint upstream. This is why the VnRobo blog hand-writes its own
  `H1_2RobotCfg` — it's original integration work, not a reuse of an existing library
  component.
- **Isaac Lab *does* natively support plain H1** (no hands), including a mechanism
  to fetch a published pretrained walking checkpoint
  (`get_published_pretrained_checkpoint()`, task `Isaac-Velocity-Flat-H1-v0`) with
  zero training required. H1 and H1-2 share the same leg/height geometry: H1-2 only
  adds ~5kg of arm/hand mass on top.
- **`unitreerobotics/unitree_sim_isaaclab`** (official Unitree repo, Isaac Sim
  4.5/5.0/5.1 — matches our install) has real, working H1-2 stationary grasp tasks
  with an Inspire dexterous hand:
  - `Isaac-PickPlace-Cylinder-H12-27dof-Inspire-Joint`
  - `Isaac-PickPlace-RedBlock-H12-27dof-Inspire-Joint` ← closest match to "pick up
    the cube"
  - `Isaac-Stack-RgyBlock-H12-27dof-Inspire-Joint`
  - Control is via DDS commands (same protocol as the real robot), driven by
    `send_commands_8bit.py` / `send_commands_keyboard.py` — no autonomous
    "just play a policy" mode found.
  - **No H1-2 "Wholebody" (mobile/walking) task exists in this repo** — Wholebody
    tasks (`Isaac-Move-Cylinder-G129-*-Wholebody`) are G1-only.
- **`correlllab/h12_loco_manipulation`** — real H1-2, real sim-to-real result
  (a genuine video of the physical H1-2 squatting while the upper body moves), with
  a pretrained checkpoint (`policies/` folder). BUT:
  - Built on **Isaac Gym Preview 4**, confirmed **broken on Blackwell GPUs**
    (RTX 50-series) — fatal `SM_120` kernel errors in `libPhysXGpu_64.so`. Not
    usable on our hardware without re-compiling PhysX for Blackwell, which NVIDIA
    itself hasn't done for this deprecated product.
  - Even if it ran, the task is **velocity/height command tracking + arm-pose
    tracking**, not object grasping — the arms track a commanded pose, they don't
    pick anything up. No box/cube demo exists in this repo.
- **`qlOoOlp/unitree_H12_can_act`** (Hugging Face) — small imitation-learning (ACT)
  policy, 150 real teleop demos of H1-2 picking up a can. Vision-conditioned on
  their specific camera rig; stationary only; wouldn't transfer to our scanned scene
  without collecting new demonstrations.

### G1 (different robot — smaller/lighter Unitree humanoid)
Found several *actually working* loco-manipulation policies here, none of which
exist for H1-2:

| Repo | What it does | Stack | Checkpoint | Notes |
|---|---|---|---|---|
| `Skevinci/CoorDex` | `CoorDex-WalkGrab-Wuji-v0` — G1 walks up, grasps, keeps moving. Real-hardware demo (non-stop bottle grasp-and-carry, cube pick-and-turn). | Isaac Lab 2.2.0 / Isaac Sim 5.0 | ✅ included (`ckpts/locomanip/`) | Closest task-shape match to our goal; uses a 20-DoF Wuji hand, not G1's stock gripper. |
| `NVlabs/GR00T-WholeBodyControl` (GEAR-SONIC) | VLA-prompted pick-and-place ("pick up the cup"), bimanual handoffs during locomotion. | Isaac Lab 2.3.2, also MuJoCo | ✅ 3 checkpoints on HF (`download_from_hf.py`) | NVIDIA's own; likely best integration with our exact Isaac Sim 5.1 stack; heavier (VLA inference). |
| `luckyrobots/g1-manipulation-challenge` | Walker + reacher ONNX policies run concurrently ("legs keep walking while the arm reaches"), cylinder pick-place table-to-table. | MuJoCo + ONNX, CPU-only | ✅ included | Lightest/fastest to try, no GPU needed; framed as an open challenge (combining walk+reach cleanly is left to the user). |
| `InternRobotics/Homie` (HOMIE) | G1 walks/squats robustly while a **human teleoperates** the arms via an exoskeleton. | Custom RL + real deploy code | ✅ included | Manipulation is teleop-driven, not autonomous — doesn't decide to grasp on its own. |
| `LeCAR-Lab/BFM-Zero` | Foundation model, zero-shot goal-reaching/motion-tracking, real G1. | Custom (FB latent model) | ✅ included | General-purpose motion generalist; grasping a specific object isn't its demonstrated strength. |
| OmniContact | 40-min continuous box-carrying with failure/drop recovery — closest *concept* to our goal. | Mocap dataset + G1 trajectories (HF: `lightcone02/OmniContact-Dataset`) | Dataset yes; a ready-to-deploy inference checkpoint not clearly published | Promising but appears training-data-centric rather than drop-in runnable today. |

## Open decision

**No existing open-source policy does "H1-2 walks up, picks up a cube, walks away
carrying it" end to end.** Every path found has at least one hard blocker for our
exact ask (wrong robot, no checkpoint, broken on our GPU, or manipulation isn't
autonomous). The live decision is which tradeoff to accept:

1. **Stay on H1-2, hybrid**: reuse Isaac Lab's native pretrained H1 legs (zero
   training) + bolt on H1-2 arms/hands for a scripted/IK grasp (referencing
   `unitree_sim_isaaclab`'s real grasp code). Locomotion is frozen during the grasp,
   matching the blog's own hierarchical pattern — just skipping the leg retrain
   since H1/H1-2 legs are the same.
2. **Stay on H1-2, faithful to the blog**: hand-write the `H1_2RobotCfg` + reward
   (largely lifted from the blog) and actually run a short Isaac Lab training job
   for the legs (est. 1–3 hours on this GPU for flat-ground velocity tracking,
   based on typical H1-class PPO convergence), then layer grasping the same way.
3. **Switch to G1, reuse a real working policy**: drop the scanned scene + cube into
   CoorDex's `WalkGrab` task or NVIDIA's GEAR-SONIC, both of which are genuine,
   checkpointed, walk-and-grasp policies today, at the cost of not being the H1-2
   robot originally requested.

Not yet decided. Next concrete step under consideration: examine CoorDex's exact G1
USD/config expectations to determine what integrating our scanned scene + cube would
require, as a way of costing out option 3 against options 1/2.
