"""Follow the repo's documented FSM order: OmniContact -> DefaultPose -> LocoMode.

Going straight from the carry policy into LocoMode still falls (survives ~600
steps). The README's hot-switch chain is
    Passive -> DefaultPose -> LocoMode -> OmniContact
i.e. the robot is brought to a stable default pose BEFORE locomotion is entered.
So insert a DefaultPose stage between placing and walking home.
"""
import sys

p = "/root/oc_astra.py"
s = open(p).read()
todo = []

todo.append((
    "    loco = None\n    from policy.loco_mode.LocoMode import LocoMode",
    "    loco = None\n    pose_until = 0\n"
    "    from policy.loco_mode.LocoMode import LocoMode\n"
    "    from policy.defaultpose.DefaultPose import DefaultPose",
    "DefaultPose import"))

# enter DefaultPose first, not LocoMode
todo.append((
    """                # velocity-driven walker: no reference to re-init
                loco = LocoMode(sc, po)
                loco.enter()
                policy = loco
                hold_action = np.asarray(po.actions, dtype=np.float32).copy()
                blend_i = 0
                phase = "home\"""",
    """                # documented FSM order: settle to DefaultPose first, then walk
                hold_action = np.asarray(po.actions, dtype=np.float32).copy()
                dp = DefaultPose(sc, po)
                dp.enter()
                policy = dp
                blend_i = 0
                pose_until = step + 500          # ~2.5 s to settle
                phase = "pose\"""",
    "enter DefaultPose"))

# after settling, hand DefaultPose -> LocoMode
todo.append((
    '        if phase == "home":',
    '''        if phase == "pose" and step >= pose_until:
            log(f"[astra] default pose settled @{step} -- entering LocoMode")
            hold_action = np.asarray(po.actions, dtype=np.float32).copy()
            loco = LocoMode(sc, po)
            loco.enter()
            policy = loco
            blend_i = 0
            phase = "home"

        if phase == "home":''',
    "DefaultPose -> LocoMode"))

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
