#!/usr/bin/env bash
# CoorDex stack: Isaac Sim 5.0.0 + Isaac Lab 2.2.0 + coordex, python 3.11
set -x
CONDA=/root/miniforge3
source "$CONDA/etc/profile.d/conda.sh"

conda env list | grep -q "^coordex " || conda create -n coordex python=3.11 -y
conda activate coordex

echo ">>> torch 2.7.0 cu128"
pip install -q torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128 2>&1 | tail -3

echo ">>> isaacsim 5.0.0 (~20GB)"
for a in 1 2 3; do
  pip install "isaacsim[all,extscache]==5.0.0" --extra-index-url https://pypi.nvidia.com && break
  echo ">>> attempt $a failed, purging"; pip cache purge || true
done

echo ">>> build deps"
DEBIAN_FRONTEND=noninteractive apt-get install -y -qq cmake build-essential 2>&1 | tail -2

echo ">>> IsaacLab v2.2.0"
rm -rf /root/IsaacLab
git clone --quiet https://github.com/isaac-sim/IsaacLab.git /root/IsaacLab
cd /root/IsaacLab && git checkout -q v2.2.0 && ./isaaclab.sh --install 2>&1 | tail -15

echo ">>> coordex"
rm -rf /root/coordex
git clone --quiet https://github.com/Skevinci/coordex.git /root/coordex
cd /root/coordex && python -m pip install -e source/coordex 2>&1 | tail -3

echo ">>> verify"
python -c "import isaacsim, isaaclab, coordex; print(\"IMPORTS OK\")" 2>&1 | tail -3
echo SETUP_DONE
