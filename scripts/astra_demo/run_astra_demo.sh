#!/usr/bin/env bash
#
# ASTRA feed-station demo — Unitree G1 picks a box off the floor and places it
# on the conveyor, shot with the Astra branded walls as the backdrop.
#
# Produces a directly playable astra_g1_conveyor_wallside.mp4.
#
#   ./run_astra_demo.sh [output.mp4]
#
# Everything is real: OmniContact's released ONNX policy driving a G1
# articulation in Isaac Sim, genuine contact physics, no scripted grasp.
#
set -euo pipefail

OUT="${1:-/root/astra_demo/astra_g1_conveyor_wallside.mp4}"
RAW="${OUT%.mp4}_raw.mp4"

# --- paths -------------------------------------------------------------------
CONDA_SH=/root/miniforge3/etc/profile.d/conda.sh   # Isaac Sim 5.0 + IsaacLab env
export OC_ROOT=/root/omnicontact_g1                # OmniContact repo (ONNX policies)
ASTRA_USD=/root/astra_workspace/astra_workspace.usd
DEMO=/root/astra_demo/astra_demo.py

for p in "$CONDA_SH" "$OC_ROOT" "$ASTRA_USD" "$DEMO"; do
  [ -e "$p" ] || { echo "MISSING: $p" >&2; exit 1; }
done

# --- environment -------------------------------------------------------------
# shellcheck disable=SC1090
source "$CONDA_SH"
conda activate coordex
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y
# Real-time RTX at 1080p: deterministic, so no sampling noise.
# OC_PATHTRACE=1 is available but measured 3x blurrier and 28x more
# flickery at affordable sample counts -- needs ~1h/render to beat this.
export OC_PATHTRACE="${OC_PATHTRACE:-0}"
export OC_SPP="${OC_SPP:-48}"
export OC_ACCUM="${OC_ACCUM:-6}"
export OC_RES="${OC_RES:-1920x1080}"

echo "[demo] policy : $OC_ROOT/policy/omnicontact/model/policy.onnx"
echo "[demo] scene  : $ASTRA_USD"
echo "[demo] output : $OUT"
echo "[demo] running (~5 min: 1080p, 3000 sim steps)…"

cd /root
rm -f "$RAW" "$OUT"
python "$DEMO" \
  --max-steps 3000 \
  --plain-box --half-dims 0.15 0.15 0.15 \
  --record "$RAW" \
  > /root/astra_demo/run.log 2>&1 || {
    echo "[demo] FAILED — see /root/astra_demo/run.log" >&2
    tail -20 /root/astra_demo/run.log >&2
    exit 1
  }

[ -f "$RAW" ] || { echo "[demo] ERROR: no video produced" >&2; exit 1; }

# --- trajectory ---------------------------------------------------------------
echo
echo "[demo] trajectory:"
grep -aE "^\[astra\] (step|box|collision)" /root/oc_astra_diag.txt | sed 's/^/    /'

# --- re-encode ----------------------------------------------------------------
# Isaac Sim/imageio writes H.264 High profile, which many players and inline
# previews refuse. Constrained Baseline + yuv420p + faststart plays anywhere.
FFMPEG="$(python -c 'import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())')"
"$FFMPEG" -y -loglevel error -i "$RAW" \
  -c:v libx264 -profile:v baseline -level 3.1 \
  -pix_fmt yuv420p -movflags +faststart -r 30 -crf 18 "$OUT"
rm -f "$RAW"

echo
echo "[demo] DONE -> $OUT  ($(du -h "$OUT" | cut -f1), Constrained Baseline, plays anywhere)"
