"""Generate play_probe.py: play_locomanip.py that logs crate lift height.

Renders nothing, so a 10 s episode runs in seconds instead of minutes. The
number we care about is max(cube_z) - initial cube_z: CoorDex's own success
criterion for WalkPickTurn is a lift of > 0.10 m (paper Appendix B.4).
"""
SRC = "/root/coordex/scripts/rsl_rl/play_locomanip.py"
DST = "/root/coordex/scripts/rsl_rl/play_probe.py"

s = open(SRC).read()

old = """    obs, _ = env.get_observations()
    step = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
        step += 1"""

new = """    obs, _ = env.get_observations()
    step = 0
    _scene = env.unwrapped.scene
    _cube = _scene["cube"]
    _z0 = float(_cube.data.root_pos_w[0, 2].item())
    _zmax = _z0
    _zhist = []
    print(f"[PROBE] cube initial z = {_z0:.4f} m")
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
        _z = float(_cube.data.root_pos_w[0, 2].item())
        _zmax = max(_zmax, _z)
        _zhist.append(_z)
        step += 1"""

assert old in s, "rollout loop not found"
s = s.replace(old, new, 1)

# Report right before the app shuts down.
old_tail = """        if rollout_steps is not None and step >= rollout_steps:"""
new_tail = """        if step % 100 == 0:
            print(f"[PROBE] step {step:4d}  z={_z:.4f}  lift={_z - _z0:+.4f}")
        if rollout_steps is not None and step >= rollout_steps:
            _lift = _zmax - _z0
            print("=" * 60)
            print(f"[PROBE] RESULT initial_z={_z0:.4f} max_z={_zmax:.4f} "
                  f"final_z={_zhist[-1]:.4f}")
            print(f"[PROBE] LIFT = {_lift:.4f} m   "
                  f"{'PICKED UP' if _lift > 0.10 else 'FAILED (CoorDex threshold 0.10 m)'}")
            print("=" * 60)"""

assert old_tail in s, "rollout tail not found"
s = s.replace(old_tail, new_tail, 1)

open(DST, "w").write(s)
import ast
ast.parse(open(DST).read())
print(f"wrote {DST}, syntax OK")
