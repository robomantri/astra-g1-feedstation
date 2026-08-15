# Humanoid box / crate manipulation — what we tried

Goal: get a **Unitree G1 to pick up a box or crate with both hands**, ideally in
the ASTRA SWEETS feed-station scene.

Nothing here was moved — everything in `/root` is untouched. This folder is
copies + symlinks. `repos/` points at the three live checkouts.

---

## Scoreboard

| # | Approach | Robot | Object | Result |
|---|----------|-------|--------|--------|
| 1–2 | coordex WalkPickTurn (ASTRA scene) | G1 | 4 cm cube | ✅ **works** — 8/8 picks after bug fix |
| 3 | coordex + kinematic weld | G1 | 600×400×120 crate | ⚠️ works visually, grip is faked |
| 4 | coordex + weld | G1 | 300×200×150 box | ⚠️ same, looks more natural |
| 5–6 | **ResMimic** | G1 | box / suitcase | ❌ **dead end** — all motions single-arm |
| 7 | **InterMimic** SMPL-X teacher | SMPL-X | large table | ✅ **real bimanual lift**, reward 76–85 |
| 8 | InterMimic G1 | G1 | largebox 471×459×408 | ❌ reaches only, reward 0.0 |
| 9 | InterMimic G1 | G1 | smallbox 385×436×235 | ❌ reaches only, reward 0.0 |
| 10 | **InterMimic G1 + weld** | G1 | smallbox | ✅ **best G1 result** — real bimanual approach, welded grip |

---

## 1–2. coordex WalkPickTurn — ASTRA scene (`repos/coordex`)

The G1 walks to a table and picks a 4 cm cube, inside the ASTRA workspace.

**The bug that broke it:** `target_table_pose` / `target_region_pose` in
`walkpickturn_env_cfg.py` are `virtual_pose_in_robot_frame` terms holding
**hardcoded WORLD coordinates**. That helper adds only `env_origins`, never the
robot spawn. Relocating the G1 to the ASTRA conveyor left those two observations
out of distribution all episode: the robot walked up, stalled 0.21 m short, and
timed out every time. Fix = add `SPAWN_DELTA` to both. **0/8 → 8/8 successes.**

Diagnostics: `logs/probe_orig.log` (pre-ASTRA baseline, success at step 403),
`logs/probe2.log` (broken), `logs/probe_fix.log` (fixed).

## 3–4. Kinematic weld (`repos/coordex`, `scripts/rsl_rl/play_weld.py`)

No policy anywhere grasps a 600 mm crate, so: keep the trained walk/reach, and
once the right palm is within `ATTACH_DIST`, drive the object's pose from the
**midpoint of both palms** every frame. Object size/mass/colour are env vars
(`BOX_SIZE`, `BOX_MASS`, `BOX_RGB`).

Honest: only the *grip* is synthetic — but the two-handed look here is partly
incidental, since the underlying policy is a one-hand cube pinch.

## 5–6. ResMimic — dead end (`repos/ResMimic`)

Recommended on the strength of its paper/videos ("carries 4.5 kg boxes"), then
measured. **All four shipped motions are single-arm:**

| motion | L wrist min | R wrist min | frames both hands |
|---|---|---|---|
| carry | 0.100 m | 0.260 m | **0** |
| kneel | 0.880 | 0.850 | 0 |
| squat | 0.201 | 0.382 | 0 |
| chair | 0.506 | 0.507 | 0 |

Its "carrying" is whole-body/torso bracing, not gripping. It also ships only the
base motion-tracking prior (`base_policy.pt`, 3.5 MB) — the task residual is not
released. **Lesson: measure the shipped data before trusting any claim.**

Gotcha found: the `carry` motion needs `num_actors = 3`, or no support plate
spawns and the object falls at reset.

## 7–10. InterMimic (`repos/InterMimic`) — the good one

Bimanual **verified by measurement**: on all 17 `largetable` sequences both
wrists are within 0.35 m of the object for **52–88 % of frames**.

- **`07` SMPL-X teacher** (`checkpoints/smplx_teachers/sub2.pth`) — walks up,
  grips with both hands, **lifts the table and carries it**. Reward 76–85.
  This is what success looks like, just not on a G1.
- **`08`/`09` G1** (`checkpoints/g1/sub8.pth`, the only G1 weight released) —
  correct bimanual reach, never completes the lift. Reward 0.0 on both large and
  small box, so **object size is not the constraint**. The authors call the G1
  integration "a small demo".
  - Note: reward is a *product* `rb * ro * rig * rcg`, so 0.0 can mean one term
    (e.g. contact matching on G1 body IDs) collapsed — not necessarily total
    failure.
- **`10` G1 + weld** — weld-on-contact added to `intermimic_g1.py`
  (`G1_WELD=1`, `G1_WELD_DIST=0.35`). Both hands arrive, weld fires
  (logged at 0.15–0.33 m), box lifts and travels with the hands. **Best G1
  result: real trained bimanual approach, only the finger grip synthetic.**

`sub8` objects: largebox 471×459×408, smallbox 385×436×235,
plasticbox 555×278×385 (closest to the 600×400 euro crate), suitcase 410×532×413.

---

## Setup notes (hard-won)

- IsaacGym Preview 4 downloads from `developer.nvidia.com/isaac-gym-preview-4`
  with **no login** and runs GPU PhysX fine on the L4.
- InterMimic needs `export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"`
  or it fails with `libpython3.8.so.1.0: cannot open shared object file`.
- Neither repo ships headless video. Patched both: keep `graphics_device_id`
  when recording (headless normally forces `-1`, so `create_camera_sensor`
  returns -1), capture inside the existing per-step render hook, cap frames,
  then `os._exit(0)` — IsaacGym teardown segfaults and would leave the mp4
  without a moov atom.
- ResMimic's `play_residual.py` writes **the same filename for every env**, so
  `num_envs > 1` produces a corrupt mp4. Patched to add the env index.
- `pkill -f <pattern>` over SSH kills your own connection when the pattern
  appears in your command line. Kill by explicit PID (`scripts/stop_im.sh`).

## Training

InterMimic training = `intermimic.run` **without** `--test`, plus
`--resume 1 --checkpoint checkpoints/g1/sub8.pth` to fine-tune from the released
G1 weights. There is **no `train_g1.sh`** — only `test_g1.sh`.
Smoke test hit `assert(batch_size % minibatch_size == 0)`: the cfg wants
`horizon_length 32`, `minibatch_size 16384`, `numEnvs 2048`, so `--num_envs`
must keep `num_envs * 32` divisible by 16384 (i.e. multiples of 512).
See `logs/g1_train_smoke.log`.
