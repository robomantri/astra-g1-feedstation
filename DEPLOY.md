# Deploying the G1 pick-and-place demo on a new machine

Everything the demo needs is in this repo **except Isaac Sim itself**, which is
~23 GB and has to be installed from NVIDIA.

```
git clone https://github.com/robomantri/astra-g1-feedstation.git
cd astra-g1-feedstation
./scripts/astra_demo/run_astra_demo.sh
```

If that errors with `cannot import isaacsim`, do the one-time setup below.

---

## What is in the repo

| | size | what |
|---|---|---|
| `scripts/astra_demo/` | 2 MB | the demo: adapter, run script, crate layout, textures |
| `vendor/omnicontact_g1/` | 84 MB | OmniContact source **and the ONNX policy weights** |
| `vendor/astra_workspace/` | 197 MB | the ASTRA scene USD, crate assets, NVIDIA conveyor asset |
| `results/` | 32 MB | the rendered videos |
| `l4_backup/` | 53 MB | archive of the working tree from the dev box |

`vendor/omnicontact_g1/policy/omnicontact/model/policy.onnx` (9.7 MB) is the
weight file the demo actually runs. It is committed — nothing is downloaded at
run time.

## What is NOT in the repo, and why

- **Isaac Sim 5.0 + IsaacLab 2.2.0** (~23 GB installed). Far too large to
  version, and it is a per-machine install. See below.
- **OmniContact's `data/`** (73 MB of NPZ motion clips). Verified unnecessary:
  the demo ran to completion with that directory moved aside. It is only used
  by the `NPZmotion` reference source; we use `CFgen`, which generates the
  reference procedurally.
- **The rest of the NVIDIA Isaac asset mirror.** Only
  `assets/Isaac/Props/Conveyors` is vendored, because the scene USD references
  `Isaac/Props/.../ConveyorBelt_A08.usd`. The other ~400 MB of the mirror is
  unused.

## One-time setup

Isaac Sim 5.0 via pip, into a fresh Python 3.11 environment:

```bash
conda create -n coordex python=3.11 -y
conda activate coordex
pip install isaacsim[all,extscache]==5.0.0 --extra-index-url https://pypi.nvidia.com
pip install onnxruntime imageio imageio-ffmpeg mujoco
```

`onnxruntime` runs the policy, `mujoco` is imported by the OmniContact runner
for its internal state, and `imageio-ffmpeg` supplies the ffmpeg binary used to
re-encode the video.

Requires an RTX GPU. The reference runs were on an L4 (24 GB); a render takes
about 5 minutes at 1080p.

## Pointing the script at things

Auto-detection order for every path: **environment variable → vendored copy in
this repo → the legacy `/root` layout on the original dev box**. So the script
works unmodified from a clone *and* on the old machine.

| variable | default | meaning |
|---|---|---|
| `ISAAC_PYTHON` | — | python that can `import isaacsim`; wins over conda |
| `CONDA_SH` / `CONDA_ENV` | auto-detected / `coordex` | conda env to activate |
| `OC_ROOT` | `vendor/omnicontact_g1` | OmniContact checkout + weights |
| `ASTRA_WS` | `vendor/astra_workspace` | scene USD and assets |
| `OC_OUT_DIR` | `scripts/astra_demo` | where video, `run.log`, diagnostics land |
| `OC_RES` | `1920x1080` | render resolution |
| `OC_DOME` / `OC_KEY` | `170` / `1000` | light intensities |
| `OC_PATHTRACE` | `0` | `1` enables path tracing (measured worse — see README) |

Examples:

```bash
ISAAC_PYTHON=~/venvs/isaac/bin/python ./scripts/astra_demo/run_astra_demo.sh
OC_RES=1280x720 ./scripts/astra_demo/run_astra_demo.sh /tmp/quick.mp4
```

Diagnostic modes, each a single Isaac boot rather than a full render:

```bash
OC_CAMSWEEP=1 ...   # render one frame per candidate camera, report luma/black rows
OC_PROBE=1 ...      # list collision-enabled prims near the robot spawn
OC_PROBE2=1 ...     # dump lights, machine-region meshes, floor material
```

## Verifying it worked

A good run ends with the box resting at deck height:

```
[astra] step 2700 [carry] g1=[5.7  0.38 0.78] yaw=-178.3 crate=[5.27 0.3  0.92]
[demo] DONE -> .../astra_g1_conveyor_wallside.mp4
```

`crate=[..., 0.92]` is the check that matters: 0.77 deck top + 0.15 half box.
If the crate z reads ~0.15 the robot never picked it up; if it reads ~0.96 it
is sitting on top of the deck crates rather than the rollers.

The output is 1920×1088 (x264 pads the 1080 render to a macroblock boundary),
296 frames, ~9.9 s, H.264 Constrained Baseline.

## Third-party contents

`vendor/omnicontact_g1` is a subset of
[Ingrid789/OmniContact_sim2sim](https://github.com/Ingrid789/OmniContact_sim2sim),
vendored so deployment needs no network. The upstream repo carried no top-level
licence file at the time it was cloned — check upstream before any external
distribution. `vendor/astra_workspace/assets/Isaac` is NVIDIA sample content
redistributed under the Omniverse licence terms. The Astra branding and scan
data are the client's.
