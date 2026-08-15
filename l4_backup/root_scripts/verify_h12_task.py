"""Instantiate Isaac-Velocity-Flat-H1-2-v0 and confirm every term resolves.

Catches the failure modes that only surface at env construction: a joint regex
matching nothing, a body name typo in a contact sensor, a reward term whose
params do not match its function signature.
"""
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--task", default="Isaac-Velocity-Flat-H1-2-v0")
parser.add_argument("--num_envs", type=int, default=4)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
app = AppLauncher(args).app

import gymnasium as gym
import torch

import isaaclab_tasks  # noqa: F401  (registers the envs)
from isaaclab_tasks.utils import parse_env_cfg

cfg = parse_env_cfg(args.task, device="cuda:0", num_envs=args.num_envs)
env = gym.make(args.task, cfg=cfg)
uenv = env.unwrapped

print("\n" + "=" * 68)
print(f"TASK OK: {args.task}")
print("=" * 68)
robot = uenv.scene["robot"]
print(f"joints  : {robot.num_joints}")
print(f"bodies  : {robot.num_bodies}")
print(f"obs dim : {uenv.observation_manager.group_obs_dim}")
print(f"act dim : {uenv.action_manager.total_action_dim}")
print(f"dt      : sim {uenv.step_dt / uenv.cfg.decimation:.6f}s  "
      f"policy {uenv.step_dt:.6f}s ({1/uenv.step_dt:.1f} Hz)  decim {uenv.cfg.decimation}")

print("\n--- ACTUATOR GROUPS (regex resolution) ---")
for name, act in robot.actuators.items():
    print(f"  {name:8s} {act.num_joints:2d} joints: {act.joint_names}")

print("\n--- REWARD TERMS ---")
for name, term in zip(uenv.reward_manager.active_terms, uenv.reward_manager._term_cfgs):
    print(f"  {name:28s} weight={term.weight:+.5g}")

print("\n--- TERMINATION TERMS ---")
print(" ", uenv.reward_manager.active_terms and uenv.termination_manager.active_terms)

print("\n--- SMOKE STEP (50 steps, zero action) ---")
env.reset()
act = torch.zeros(uenv.action_space.shape, device=uenv.device)
rew_sum = torch.zeros(args.num_envs, device=uenv.device)
for i in range(50):
    _, rew, _, _, _ = env.step(act)
    rew_sum += rew
print(f"  mean return over 50 steps: {rew_sum.mean().item():.3f}")
print(f"  base height after 50 steps: {robot.data.root_pos_w[:, 2].mean().item():.3f} m")
print("\nALL CHECKS PASSED")

env.close()
app.close()
