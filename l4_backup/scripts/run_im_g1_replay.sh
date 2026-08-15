#!/usr/bin/env bash
source /root/ResMimic/thirdparty/miniconda3/bin/activate intermimic-gym
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd /root/InterMimic
export PYTHONPATH="/root/InterMimic/isaacgym/src:/root/InterMimic:$PYTHONPATH"
export INTERMIMIC_RECORD=/root/im_g1_replay.mp4
export INTERMIMIC_RECORD_FPS=30
export INTERMIMIC_RECORD_MAXFRAMES=600
exec python -m intermimic.run --task InterMimicG1 \
  --cfg_env isaacgym/src/intermimic/data/cfg/omomo_g1_29dof_with_hand.yaml \
  --cfg_train isaacgym/src/intermimic/data/cfg/train/rlg/omomo_g1_29dof_with_hand.yaml \
  --test --play_dataset --num_envs 4 --headless
