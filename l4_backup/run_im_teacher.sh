#!/usr/bin/env bash
source /root/ResMimic/thirdparty/miniconda3/bin/activate intermimic-gym
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd /root/InterMimic
export PYTHONPATH="/root/InterMimic/isaacgym/src:/root/InterMimic:$PYTHONPATH"
export INTERMIMIC_RECORD=/root/im_teacher_sub2.mp4
export INTERMIMIC_RECORD_FPS=30
export INTERMIMIC_RECORD_MAXFRAMES=600
exec python -m intermimic.run --task InterMimic \
  --cfg_env isaacgym/src/intermimic/data/cfg/omomo_test.yaml \
  --cfg_train isaacgym/src/intermimic/data/cfg/train/rlg/omomo.yaml \
  --test --checkpoint checkpoints/smplx_teachers/sub2.pth --num_envs 4 --headless
