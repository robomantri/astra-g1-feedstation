# ASTRA SWEETS — Inkapstation (feed station) simulation

Isaac Sim environment for **project 20260087 — "Robot handling of bins at feed stations"**.
A laser scan and CAD model of the ASTRA SWEETS packing station, assembled into a
physics-enabled Isaac Sim scene with a Unitree G1.

---

## G1 pick-and-place demo

The headline deliverable: a Unitree G1 walks to a crate on a pallet, picks it up
two-handed, turns, carries it to the conveyor and places it flat on the rollers
— then stays standing. Driven by the released
[OmniContact](https://github.com/Ingrid789/OmniContact_sim2sim) `carrybox` ONNX
policy, reused byte-for-byte; only the simulator adapter is ours.

Runs on the L4 box (`ssh l4`), one command, ~10 min:

```bash
bash /root/astra_demo/run_astra_demo.sh            # -> astra_g1_conveyor_wallside.mp4
```

Local copies live in `scripts/astra_demo/`; the output is
`results/astra_g1_conveyor_wallside.mp4`.

| | |
|---|---|
| spawn | (7.00, 0.35), yaw 0° — between the pallet and the conveyor |
| pick | crate on a 0.144 m pallet at (9.00, 0.35), centre z 0.244 |
| place | conveyor deck at (4.50, 0.42, **0.87**) = deck top 0.77 + half crate 0.10 |

The conveyor is rotated 90° about its own centre, so the robot loads the long
side of the belt rather than its end. After grasping it turns ~180° and carries
the crate across; the reference generator plans that turn itself.

Measured limits, all established by bisection rather than assumed:

- **Grasp height ceiling 0.41 m** (crate centre). 0.40 and 0.41 lift; 0.42 and
  above never make contact.
- **Crate 0.40 × 0.267 × 0.20 m.** 0.40 length is the reach limit — the hands
  target the object's mid-plane and the trained max half-length is 0.20. The
  0.20 height stops it tumbling between the palms; at 0.113 it landed on its
  end. Anything 0.60 long (a real euro crate, `SM_Crate_A07`) cannot be picked
  in any orientation.
- **Walk 1.6× slower** than default (`OC_WALK=0.010` vs 0.016), so the gait
  reads on camera.

Tunable without editing code: `OC_DOME` / `OC_KEY` (light intensities),
`OC_RES`, `OC_SPP`, `OC_ACCUM`, `OC_PATHTRACE`, and `OC_CAMSWEEP=1` /
`OC_PROBE=1` / `OC_PROBE2=1` for single-boot camera and scene probes.

### Two constraints worth knowing

Both cost a lot of debugging; both are commented at their site in
`scripts/astra_demo/astra_demo.py`.

1. **The reference plan is built from the runner's own MuJoCo state, not from
   Isaac.** `_prepare_episode()` → `_sync_state_cmd_from_mj()` reads
   `self.d.qpos[:3]`, which sits at the origin. Spawn the Isaac robot anywhere
   else and it starts off its own reference, lunges to close the gap and falls.
   Fix: write the spawn pose into `r.d.qpos` *before* the plan is generated.

2. **Isaac reports base velocities in the world frame; the policy expects the
   body frame** (it was written against MuJoCo `qvel`). These coincide at
   yaw ≈ 0, so it looked correct for every run along +x — at yaw 170–180° the
   x/y components sign-flip and the robot collapses within a second. Feed
   `R.T @ get_angular_velocity()`, same as `gravity_ori`.

Every post-carry policy swap falls (7 variants tried) — only the carry policy
keeps the robot upright indefinitely.

### Why it is rendered offline, not run live

The simulation cannot keep up with wall-clock time, and the reason is
**physics, not graphics**.

Physics steps at 1/200 s, so realtime needs **200 steps/s**. Measured on the
L4 with rendering switched off entirely:

| configuration | steps/s | vs realtime |
|---|---|---|
| no rendering at all | ~65 | **3.1x too slow** |
| 1080p RTX render | ~9 | 22x too slow |

The gap exists *before a single pixel is drawn*, so resolution is not the
lever — 720p, 480p and fully headless all sit behind the same 3.1x wall.

The obvious suspect was scene complexity, and it was wrong. Stripping the 110
pile crates and 30,200 candy pellets made it **slower** (~41 steps/s), not
faster: they are static, visual-only geometry that costs nothing in the step
loop. The time goes to core Isaac stepping and the articulation solver.

The one real lever is `physics_dt`. Going 1/200 -> 1/100 halves the step
budget outright, but the OmniContact policy was trained on 50 Hz control
derived from a 200 Hz sim, so changing that ratio changes what the policy
sees. It is a stability trade, not a free win, and it is untested here.

Live *viewing* is a separate question and is feasible: Isaac Sim's WebRTC
livestream extension (`omni.kit.livestream.webrtc`) is installed and listens
on port 49100. It would stream fine — the robot would simply move at about a
third speed.

Two honest limits on the above: these are single runs on a shared VM, so treat
them as +-30%; and "not possible" is an inference from the two measurements,
not the result of attempting a live render.

**Footnote:** the exported video already plays **1.5x faster than simulated
time**. One frame per 10 steps is 20 fps of sim time, encoded at 30 fps, so
22.0 s of simulation becomes a 14.5 s video. Encode with `-r 20` instead of
`-r 30` if you ever want playback to match sim time.

---

## Quick start

All commands assume Isaac Sim 6.0 at `~/isaacsim6_venv`.

```bash
ISAAC="$HOME/isaacsim6_venv/bin/isaacsim isaacsim.exp.full"

# 1. Build (or rebuild) the sim-ready scene -> astra_station_sim.usd + a render
OMNI_KIT_ACCEPT_EULA=YES $ISAAC --exec ~/Desktop/h12/scripts/build_astra_scene.py

# 2. Reopen the built scene and render a view of the G1
OMNI_KIT_ACCEPT_EULA=YES $ISAAC --exec ~/Desktop/h12/scripts/render_astra_view.py

# 3. The photoreal laser scan (12M coloured points) on its own
OMNI_KIT_ACCEPT_EULA=YES $ISAAC --enable omni.usd.fileformat.e57 \
  --exec ~/Desktop/h12/scripts/isaacsim6_open_scan.py

# 4. The raw CAD model on its own (lit + framed)
OMNI_KIT_ACCEPT_EULA=YES $ISAAC --exec ~/Desktop/h12/scripts/isaacsim6_open_astra.py
```

Do **not** set `__VK_LAYER_NV_optimus=NVIDIA_only`. See [Environment](#environment).

---

## What is here

```
h12/
├── astra_station_sim.usd        the assembled sim scene (build output)
├── astra_scene_render.png       render from the last build
├── scripts/
│   ├── build_astra_scene.py     assembles the scene; the main entry point
│   ├── make_crate_usd.py        STEP -> USD crate variants (grey + red)
│   ├── where.py                 prints a coordinate map of the scene
│   ├── recolour.py              reusable material recolour helper
│   ├── render_astra_view.py     reopen the scene, render the G1
│   ├── view_conveyor.py         render the conveyor
│   ├── view_pile.py             render the crate piles
│   ├── isaacsim6_open_scan.py   loads the E57 point cloud
│   └── isaacsim6_open_astra.py  loads the bare CAD model, lit and framed
├── assets/
│   ├── Isaac/                   local mirror of Isaac 6.0 assets
│   │   ├── Robots/Unitree/G1/
│   │   ├── Props/KLT_Bin/
│   │   └── Props/Conveyors/     ConveyorBelt_A08 + Material Library + Textures
│   ├── crates/                  euro_crate_600x400x120{,_red}.usd
│   └── textures/                astra_wall.png, concrete.png, floor_tile.png
├── crate_src/                   supplier's STEP crate (600x400x120)
├── office/v.20260801/           the supplier delivery
└── recap_to_e57_prompt.txt      prompt used to get the E57 exported on Windows
```

---

## The scene

`astra_station_sim.usd` contains:

| Component | Detail |
|---|---|
| Station x2 | Two copies of the ArchiCAD bay joined end to end: `Station` x 0..11.08, `Station2` x -11.08..0 |
| Colliders | 441 static triangle-mesh colliders |
| Physics | `UsdPhysics.Scene`, gravity 9.81 m/s^2 down |
| Floor | Ground plane at z=-0.01 + tiled material; station floor slab is the walkable surface at z=0 |
| Unitree G1 | PhysX variant, 43 revolute + 2 fixed joints, at (4.65, 5.10), feet at z=0.02 |
| Conveyor | ConveyorBelt_A08, frame tinted safety yellow, x 1.18..3.90, roller bed at z=0.769 |
| Crates on belt | 2 Euro crates, evenly spaced on the roller run |
| Crate piles | 3 piles x (3 cols x 2 rows x 6 layers) = 108 crates; the far pile is red |
| Astra branding | Two stretched panels per bay, split at the column cluster |
| Brushed metal | 32 machine meshes (16 per copy) |
| Concrete | 9 structural meshes: columns, beam, pier, side wall |
| Lighting | Dome (1200) + distant key light (2500) |

Each bay is **11.078 x 7.974 x 3.407 m**; the two together span x -11.08..11.08.

Layout constants live at the top of `build_astra_scene.py` -- `STATION_COPIES`,
`CONVEYOR_POS`, `G1_POS`, `PILE_ORIGIN`, `PILE_GAPS`, `PILE_CRATE_USD`,
`METAL_MESHES`, `CONCRETE_MESHES`, `REMOVE_MESHES`, `STATION2_CLEAR`.

---

## Source data

The supplier delivery is `office/v.20260801/` — 18 files.

| File | What it is |
|---|---|
| `*.rcs` (254 MB) | 13.2M-point colour laser scan, Autodesk `ADOCT` (proprietary) |
| `E57/*.e57` (323 MB) | **12,062,324 points**, RGB + normals + intensity — use this |
| `*_usd_v.usdc` | ArchiCAD model, 270 meshes, flat material colours |
| `*_stl.stl` | Same geometry, 8,186 triangles, mm units |
| `Image_1/2/3.jpg` | Screenshots *of the point cloud* — not textures |

### Two things to know about the delivery

**There are no texture maps.** `textures/` contained a single 465-byte 1×1 grey pixel
and `images/` was empty — confirmed identical on the remote build box, so nothing was
lost in transit. The USD references exactly one asset (that grey pixel) and has no
external references or payloads. **259 of its 270 meshes have no UVs at all**, so any
texture needs UVs generated first. The supplier's photorealistic reference render was
made elsewhere with assets that were never delivered.

**The point count differs from the metadata.** The `.rcp` XML says 13,226,607 points;
the E57 has 12,062,324. This is correct, not a bad export — the project defines a
LimitBox of 11.099 × 8.067 × 3.214 m and ReCap clips to it, matching the exported
bounds within 2 cm.

---

## Environment

- **Isaac Sim 6.0.0.0** in a Python 3.12 venv at `~/isaacsim6_venv` (22 GB)
- GPU: RTX 5070 Ti Laptop (Blackwell, sm_120); torch 2.10.0+cu128 verified on device

This machine is an **Optimus laptop** — the AMD Radeon 610M drives the display and the
NVIDIA GPU is render-offload only. Two consequences:

- Isaac Sim **5.1 segfaulted on every launch** in `librtx.scenedb.plugin.so`, even with
  a blank stage. Upgrading to 6.0 fixed it (verified: 6.0 runs with no special env vars).
- **Never set `__VK_LAYER_NV_optimus=NVIDIA_only`.** It hides the AMD device that owns
  the X display, so Isaac Sim renders on the NVIDIA GPU but presents nothing — the
  window becomes a transparent shell. Symptom: correct title, high VRAM, see-through window.

Launch plainly with `OMNI_KIT_ACCEPT_EULA=YES`.

---

## Gotchas worth knowing

These each cost real debugging time.

**Ground level — use the floor slab's top, not the model minimum.** The station model
includes its own 0.28 m thick floor slab. Offsetting by `-bbox.min.z` puts world z=0 at
the slab's *underside*, so anything placed on "the floor" sinks 0.276 m — the G1 ends up
buried to mid-shin. Detect the slab (broad footprint, <0.6 m thick) and use its max z.

**The CAD model opens looking empty.** It ships unlit: the DomeLight is at intensity 1.0
pointed at a near-black EXR, plus a 10 cm sphere light for an 11 m room. Also,
`ComputeWorldBound` on `/root` returns 56 × 38 m because it includes the Camera and Light
prims — frame the *meshes* instead, or the camera lands far outside the room.

**Don't trust `BBoxCache` on the G1.** Its geometry sits behind `instanceable` prims, so
`stage.Traverse()` finds zero Mesh prims under it and the bounds come from an authored
`extentsHint`. The G1's origin is at the pelvis, ~0.792 m above the soles.

**Referenced assets already have xformOps.** `AddTranslateOp()` raises on them, and
`XformCommonAPI.SetTranslate()` *silently does nothing* when the prim uses a quaternion
`orient` op (the G1 does). Reference the asset onto a child prim and transform a wrapper.

**`Usd.Stage.Open(path).Traverse()`** — the stage is a temporary, gets collected
mid-iteration, and raises `Invalid range starting with expired prim`. Bind it to a name.

**The E57 cannot be referenced onto a prim.** The plugin writes to an absolute `/data3D`
path; `AddReference()` fails. Open it as the stage. Also put the camera *inside* the room
— a scan only captures surfaces the scanner saw, so from outside you get blank back faces.

**STEP CAD converts via trimesh + cascadio, not Kit.** Kit's CAD converter
(`omni.kit.converter.cad`, HOOPS) needs a GUI confirm dialog, so it is not scriptable
headless. `pip install cascadio` gives trimesh an OCCT STEP reader; see
`scripts/make_crate_usd.py`. The supplier's crate came in already in metres.

**Do not author vertex normals on a generated mesh.** Hydra ignores the `interpolation`
metadata on the `normals` attribute and reads it as faceVarying, so vertex-count normals
trip `corrupted data in primvar 'normal'`. Omit them and let the renderer generate face
normals -- which is the correct crisp look for a CAD part anyway.

**Two texture mappings, for different jobs.** `_planar_uvs()` stretches 0..1 across a
prim (right for a logo on a wall). `_worldscale_uvs()` maps metres/tile so the grain
stays the same physical size on every prim (right for concrete). Using the wrong one is
very obvious.

**Sweeping geometry by proximity must exclude room-spanning prims.** A naive
box-intersection query around the join end catches the 11x8m floor slab and the 10.16m
back wall, because they merely pass through. Test the *centre* against the strip and
skip anything with a footprint over ~5 m.

**Kill stale Isaac Sim before launching.** A leftover instance holding the GPU makes the
next launch die during startup with no `[build]` markers. Kill by explicit PID -- never
`pkill -f`/`pgrep -f` with a pattern that also matches your own shell command.

**Screenshots:** don't scrape the window with `xwd`, it captures whatever overlaps it.
Use `capture_viewport_to_file()` from `omni.kit.viewport.utility`.

**Wall branding.** The back wall (`Mesh_017`, 10.16 m) is interrupted by a column cluster
at local x ≈ −44.2, so the branding reads as two panels — 6.58 m and 3.45 m, matching the
reference render. Mesh_017 is a single 8-point box so a UV seam mid-face is impossible;
two quads sit 12 mm proud of the wall instead, each mapped 0→1 (stretched, not tiled).
U runs along −X: the room side is viewed looking along −Y where screen-right is −X, and
without that flip the artwork comes out mirrored. Note the mesh points are authored in
the station's *original* frame (x ≈ −51…−40), so any inward/outward test must use that
frame rather than the shifted one.

---

## How to change things

Everything below is a constant at the top of `scripts/build_astra_scene.py`. Edit, rerun
the build, done. **Kill any running Isaac Sim first** or the new one dies at startup.

### Add another workspace bay

One entry in `STATION_COPIES`. Bays are one room-width apart, and `dx_rooms` is measured
in room widths (11.078 m), extending in -X because the side walls sit at the +X end:

```python
STATION_COPIES = (
    {"name": "Station",  "dx_rooms":  0.0, "drop": ()},
    {"name": "Station2", "dx_rooms": -1.0, "drop": (...), "clear_join": True},
    {"name": "Station3", "dx_rooms": -2.0,                 # <-- new bay
     "drop": ("Mesh_004", "Mesh_003", "Mesh_018", "Mesh_002", "Mesh_016", "Mesh_189"),
     "clear_join": True},
)
```

| key | what it does |
|---|---|
| `name` | prim name under `/World`; also suffixes its wall panels |
| `dx_rooms` | offset in room widths; negative extends into open floor |
| `drop` | mesh names deactivated on this copy only (the end-wall structure) |
| `clear_join` | also sweep everything near that end so the bay butts up to its neighbour |

Everything else follows automatically: colliders, the join sweep, the Astra panels,
pillar widening, floor-trim clearance, metal and concrete all iterate the copies
(`METAL_MESHES` / `CONCRETE_MESHES` / `WIDEN_PILLARS` are *relative* subpaths, so they
apply to every bay).

Verified: adding `Station3` was exactly the 6-line entry above and nothing else --
3 bays, 608 colliders, 48 metal, 11 concrete, both joins swept and trimmed, each bay's
right panel 4.28 m attaching to its own neighbour's pillar.

What does **not** follow: the conveyor, G1 and crate piles are placed at absolute
coordinates and stay in the first bay. Duplicate those blocks if you want each bay kitted out.

### Move something

```python
CONVEYOR_POS = (3.90, 5.10, 0.0)     # belt runs along +X from here
G1_POS, G1_YAW = (4.65, 5.10), 195.0
PILE_ORIGIN = (8.80, 6.90)           # first pile centre
```

### Crate piles

```python
N_PILES = 3
PILE_GAPS = (0.0, 0.5, 0.31)         # clear gap BEFORE each pile, in metres
PILE_CRATE_USD = (None, None, "red") # None = grey
PILE_COLS, PILE_ROWS, PILE_LAYERS = 3, 2, 6
PILE_YAW = 90.0                      # 90 = crate long side along Y
PILE_STEP_SIGN = -1                  # -1 extends into the room; +1 hits the wall at 11.08
```

### Give meshes a material

Append the prim's **relative** subpath (everything after `/World/<Station>/`):

```python
METAL_MESHES    = ("VisualSceneNode1/Geometry141/Mesh_117", ...)
CONCRETE_MESHES = ("VisualSceneNode1/Geometry6/Mesh_005", ...)
```

Remember the ArchiCAD meshes have **no UVs**. Metal is textureless so it just works;
concrete needs UVs, which `_worldscale_uvs()` generates automatically.

### Delete meshes

```python
REMOVE_MESHES  = ("Mesh_001",)   # every copy
STATION2_CLEAR = (...)           # names seeding the join-end sweep
```

### Floor trim at the joins

Skirting strips are only ~6 cm tall, so they are not walls -- but a walking robot would
trip on them. They are cleared by *overlap* with a band around each join (the main sweep
misses them because it tests a prim's centre, and these strips run far along the wall):

```python
JOIN_TRIM_MAX_H = 0.10      # anything this low is trim, not structure
JOIN_TRIM_BAND  = 0.75      # metres either side of a join
```

### Widen a pillar / re-attach a panel

```python
WIDEN_PILLARS = {".../Geometry7/Mesh_006": 0.15}   # extra metres in X, about its centre
```
Applied to **every** bay, and each `clear_join` copy's right panel is then run out to the
pillar in the bay on its +X side. Note it must be the *neighbour's* pillar -- using one
global value makes the third bay's panel stretch 15 m across the second.

### Find coordinates

```bash
~/isaacsim6_venv/bin/python ~/Desktop/h12/scripts/where.py
```
Prints every object's centre and size plus station landmarks. In the GUI: select a prim,
read Property -> Transform -> Translate. Frame: origin = room corner, floor at z=0,
X along the branded wall, Y into the room.

### New crate colours

Add to `VARIANTS` in `scripts/make_crate_usd.py`, rerun it, then point `CRATE_USD` /
`CRATE_USD_RED` (or `PILE_CRATE_USD`) at the new file.

---

## Status and next steps

Done:

- Isaac Sim 6.0 working on this GPU
- Laser scan loading as 12.06M coloured points at ~119 FPS
- Two workspace bays joined end to end, join structure cleared on the copy
- Astra branding, tiled floor, brushed metal on the machine, concrete on the structure
- Yellow conveyor with crates on the belt; 108-crate piles from the supplier's own STEP crate
- G1 standing at the conveyor discharge end, feet on the floor

Not done:

- **No robot controller.** The G1 is articulated but unactuated, so under physics it will
  collapse -- it needs a control policy to stand or walk.
- **The second bay is empty.** Conveyor, G1 and piles are placed at absolute coordinates
  in the original bay only; they do not follow `STATION_COPIES`.
- Candy fill in the crates.
- Point cloud and CAD are still in different coordinate frames and not combined in one
  scene (the scan is registered to the `.rcp` origin, the CAD is shifted to the room corner).
