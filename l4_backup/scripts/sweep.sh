#!/bin/bash
# Sweep crate size, measure lift each time. No rendering -> fast.
source /root/miniforge3/etc/profile.d/conda.sh; conda activate coordex
export OMNI_KIT_ACCEPT_EULA=YES
cd /root/coordex
CFG=/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py

for SPEC in "0.04 0.04 0.04 0.2" "0.05 0.05 0.05 0.22" "0.06 0.05 0.05 0.25" "0.075 0.055 0.05 0.25"; do
  set -- $SPEC
  cp $CFG.precrate $CFG
  python3 /root/crate_swap.py $1 $2 $3 $4 >/dev/null
  echo "############ CRATE ${1}x${2}x${3} m, ${4} kg ############"
  timeout 600 python -u /root/coordex/scripts/rsl_rl/play_probe.py --task CoorDex-WalkPickTurn-Wuji-v0 \
    --headless --num_envs 1 --max_steps 600  2>&1 | grep -E "\[PROBE\] (RESULT|LIFT|cube initial)"
done
echo "############ SWEEP COMPLETE ############"
