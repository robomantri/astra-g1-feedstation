#!/usr/bin/env bash
source /root/miniforge3/etc/profile.d/conda.sh; conda activate coordex; cd /root/coordex
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y PRIVACY_CONSENT=Y
for S in 0.04,0.04,0.04 0.06,0.06,0.06 0.08,0.08,0.08 0.10,0.10,0.10 0.12,0.08,0.06 0.16,0.11,0.08; do
  T=$(echo $S | tr , x)
  BOX_SIZE=$S timeout 900 python scripts/rsl_rl/play_probe2.py \
      --task CoorDex-WalkPickTurn-Wuji-v0 --num_envs 1 --headless --max_steps 1000 \
      > /root/box_$T.log 2>&1
  N=$(grep -c "success" /root/box_$T.log)
  echo "SIZE $S -> $(grep -o "ended at step [0-9]*: \[.[a-z_]*" /root/box_$T.log | sed "s/ended at step //" | tr "\n" " ")"
done
echo SWEEP_DONE
