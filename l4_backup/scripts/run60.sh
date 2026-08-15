#!/usr/bin/env bash
# Run the H1 demo in a fresh Isaac Sim 6.0.1 container. Usage: run60.sh <stage>
set -euo pipefail
S="${1:-walk}"
D=/root/docker/isaac-sim
exec docker run --rm --gpus all --network=host \
  -e ACCEPT_EULA=Y -e OMNI_KIT_ACCEPT_EULA=YES -e PRIVACY_CONSENT=Y -e OMNI_KIT_ALLOW_ROOT=1 \
  -v "$D/cache/kit:/isaac-sim/kit/cache" \
  -v "$D/cache/ov:/root/.cache/ov" \
  -v "$D/cache/glcache:/root/.cache/nvidia/GLCache" \
  -v "$D/cache/computecache:/root/.nv/ComputeCache" \
  -v "$D/logs:/root/.nvidia-omniverse/logs" \
  -v "$D/data:/root/.local/share/ov/data" \
  -v /root/f2:/workspace/f2 \
  --entrypoint bash my-isaaclab:6.0.1 \
  -c "cd /workspace/f2 && mkdir -p out && /isaac-sim/python.sh h1_pick_demo.py --stage $S"
