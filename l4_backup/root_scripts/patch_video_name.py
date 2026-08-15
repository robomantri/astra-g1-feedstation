"""Fix two bugs in ResMimic's play_residual.py video recording.

1. The per-env loop builds the SAME filename every iteration, so with num_envs=2
   two imageio writers write to one file concurrently and produce a corrupt mp4
   ("Invalid NAL unit size"). Add the env index.
2. set_play_cfg() hardcodes num_envs = 2, ignoring --num_envs. Respect the flag.
"""
import sys

p = "/root/ResMimic/legged_gym/legged_gym/scripts/play_residual.py"
s = open(p).read()

old_name = '            video_name = args.proj_name + "-" + args.exptid +".mp4"'
new_name = '            video_name = args.proj_name + "-" + args.exptid + f"-env{i}.mp4"'

old_n = "    env_cfg.env.num_envs = 2#2 if not args.num_envs else args.num_envs"
new_n = "    env_cfg.env.num_envs = args.num_envs if args.num_envs else 2"

changed = 0
if old_name in s:
    s = s.replace(old_name, new_name)
    changed += 1
elif new_name in s:
    print("video name already patched")
else:
    print("FAIL: video name line not found")
    sys.exit(1)

if old_n in s:
    s = s.replace(old_n, new_n)
    changed += 1
elif new_n in s:
    print("num_envs already patched")
else:
    print("FAIL: num_envs line not found")
    sys.exit(1)

open(p, "w").write(s)
print(f"patched OK ({changed} changes)")
