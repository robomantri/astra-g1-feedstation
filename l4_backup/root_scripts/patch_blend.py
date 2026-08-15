"""Smooth the carry -> loco hand-over.

Swapping policies mid-episode makes the robot fall: the incoming policy's
observation history buffer is all zeros, so its first action is a large jump
from the pose the robot is actually in.

Fix: on switch, keep holding the last carry action while the loco policy ticks
(filling its 5-frame history), then cross-fade over ~1 s.
"""
import sys

p = "/root/oc_astra.py"
s = open(p).read()
todo = []

todo.append((
    '    phase = "carry"\n    stand_off = False',
    '    phase = "carry"\n    stand_off = False\n'
    '    hold_action = None          # last carry action, held across the switch\n'
    '    blend_i, BLEND_N = 0, 50    # policy ticks to cross-fade (50*4 steps = 1 s)',
    "blend state"))

todo.append((
    '                policy, sc, po = r2.policy, r2.state_cmd, r2.policy_output\n'
    '                phase = "home"',
    '                policy, sc, po = r2.policy, r2.state_cmd, r2.policy_output\n'
    '                hold_action = np.asarray(po.actions, dtype=np.float32).copy()\n'
    '                blend_i = 0\n'
    '                phase = "home"',
    "capture hold action"))

todo.append((
    '        tgt = np.zeros(len(dof), dtype=np.float32)\n'
    '        tgt[isaac2mj] = np.asarray(po.actions, dtype=np.float32)',
    '        act = np.asarray(po.actions, dtype=np.float32)\n'
    '        if hold_action is not None and blend_i < BLEND_N:\n'
    '            if step % 4 == 0:\n'
    '                blend_i += 1\n'
    '            w = blend_i / float(BLEND_N)\n'
    '            act = (1.0 - w) * hold_action + w * act\n'
    '        tgt = np.zeros(len(dof), dtype=np.float32)\n'
    '        tgt[isaac2mj] = act',
    "cross-fade"))

n = 0
for a, b, label in todo:
    if s.count(a) != 1:
        print(f"MISS ({s.count(a)}x): {label}")
        continue
    s = s.replace(a, b, 1)
    n += 1
    print(f"ok: {label}")

open(p, "w").write(s)
print(f"applied {n}/{len(todo)}")
sys.exit(0 if n == len(todo) else 1)
