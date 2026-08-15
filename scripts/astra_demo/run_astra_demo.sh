#!/usr/bin/env bash
#
# ASTRA feed-station demo — Unitree G1 picks a box off the floor beside the
# crate piles and places it on the conveyor, shot against the Astra branded
# walls.
#
# Produces a directly playable astra_g1_conveyor_wallside.mp4.
#
#   ./run_astra_demo.sh [output.mp4]
#
# Everything is real: OmniContact's released ONNX policy driving a G1
# articulation in Isaac Sim, genuine contact physics, no scripted grasp.
#
# Self-contained: the policy weights, robot description and scene assets are
# vendored under <repo>/vendor, so a fresh clone runs with no edits. The only
# external requirement is a Python with Isaac Sim importable — see DEPLOY.md.
#
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"

# --- paths: env var -> vendored copy in the repo -> legacy /root layout ------
pick() {  # pick VAR_VALUE CANDIDATE...
  local v="$1"; shift
  if [ -n "$v" ]; then echo "$v"; return; fi
  for c in "$@"; do [ -e "$c" ] && { echo "$c"; return; }; done
  echo "$1"                      # report the first candidate in the error
}

export OC_ROOT="$(pick "${OC_ROOT:-}"  "$REPO/vendor/omnicontact_g1"  /root/omnicontact_g1)"
export ASTRA_WS="$(pick "${ASTRA_WS:-}" "$REPO/vendor/astra_workspace" /root/astra_workspace)"
ASTRA_USD="$ASTRA_WS/astra_workspace.usd"
DEMO="$HERE/astra_demo.py"

# where the video, run log and diagnostics land
export OC_OUT_DIR="${OC_OUT_DIR:-$HERE}"
OUT="${1:-$OC_OUT_DIR/astra_g1_conveyor_wallside.mp4}"
RAW="${OUT%.mp4}_raw.mp4"

for p in "$OC_ROOT" "$ASTRA_USD" "$DEMO"; do
  [ -e "$p" ] || { echo "MISSING: $p" >&2; exit 1; }
done
POLICY="$OC_ROOT/policy/omnicontact/model/policy.onnx"
[ -f "$POLICY" ] || { echo "MISSING policy weights: $POLICY" >&2; exit 1; }

# --- python with Isaac Sim ---------------------------------------------------
# ISAAC_PYTHON wins; else activate a conda env; else trust python on PATH.
if [ -n "${ISAAC_PYTHON:-}" ]; then
  PY="$ISAAC_PYTHON"
else
  CONDA_SH="${CONDA_SH:-}"
  for c in "$CONDA_SH" "$HOME/miniforge3/etc/profile.d/conda.sh" \
           "$HOME/miniconda3/etc/profile.d/conda.sh" \
           /root/miniforge3/etc/profile.d/conda.sh; do
    [ -n "$c" ] && [ -f "$c" ] && { CONDA_SH="$c"; break; }
  done
  if [ -n "$CONDA_SH" ] && [ -f "$CONDA_SH" ]; then
    # shellcheck disable=SC1090
    source "$CONDA_SH"
    conda activate "${CONDA_ENV:-coordex}"
  fi
  PY=python
fi

"$PY" -c 'import isaacsim' 2>/dev/null || {
  cat >&2 <<EOF
[demo] ERROR: '$PY' cannot import isaacsim.

Isaac Sim is NOT vendored (it is ~23 GB) and must be installed separately.
See DEPLOY.md. Then either activate that environment before running, or:

    ISAAC_PYTHON=/path/to/python ./run_astra_demo.sh
    CONDA_SH=/path/to/conda.sh CONDA_ENV=myenv ./run_astra_demo.sh
EOF
  exit 1
}

export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y
# Real-time RTX at 1080p: deterministic, so no sampling noise.
# OC_PATHTRACE=1 is available but measured 3x blurrier and 28x more
# flickery at affordable sample counts -- needs ~1h/render to beat this.
export OC_PATHTRACE="${OC_PATHTRACE:-0}"
export OC_SPP="${OC_SPP:-48}"
export OC_ACCUM="${OC_ACCUM:-6}"
export OC_RES="${OC_RES:-1920x1080}"

echo "[demo] policy : $POLICY"
echo "[demo] scene  : $ASTRA_USD"
echo "[demo] output : $OUT"
echo "[demo] running (~5 min: 1080p, 3000 sim steps)…"

RUNLOG="$OC_OUT_DIR/run.log"
rm -f "$RAW" "$OUT"
"$PY" "$DEMO" \
  --max-steps 3000 \
  --plain-box --half-dims 0.15 0.15 0.15 \
  --record "$RAW" \
  > "$RUNLOG" 2>&1 || {
    echo "[demo] FAILED — see $RUNLOG" >&2
    tail -20 "$RUNLOG" >&2
    exit 1
  }

[ -f "$RAW" ] || { echo "[demo] ERROR: no video produced" >&2; exit 1; }

# --- trajectory ---------------------------------------------------------------
echo
echo "[demo] trajectory:"
grep -aE "^\[astra\] (step|box|collision)" "$OC_OUT_DIR/oc_astra_diag.txt" | sed 's/^/    /'

# --- re-encode ----------------------------------------------------------------
# Isaac Sim/imageio writes H.264 High profile, which many players and inline
# previews refuse. Constrained Baseline + yuv420p + faststart plays anywhere.
# select=gte(n,2) drops the RTX lighting-accumulation warmup: frames 0-1 come
# out at luma ~91 against the ~134 the rest of the clip holds.
FFMPEG="$("$PY" -c 'import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())')"
"$FFMPEG" -y -loglevel error -i "$RAW" \
  -vf "select=gte(n\\,2),setpts=PTS-STARTPTS" -c:v libx264 -profile:v baseline -level 3.1 \
  -pix_fmt yuv420p -movflags +faststart -r 30 -crf 18 "$OUT"
rm -f "$RAW"

echo
echo "[demo] DONE -> $OUT  ($(du -h "$OUT" | cut -f1), Constrained Baseline, plays anywhere)"
