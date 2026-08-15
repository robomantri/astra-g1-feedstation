"""Hot-swap to LocoMode after placing, per the repo's FSM pattern.

Two changes, both from the user's suggestion:

1. LocoMode instead of the CFgen `loco` task. LocoMode is a VELOCITY-COMMAND
   walker (state_cmd.vel_cmd) with enter()/run()/exit() -- there is no reference
   trajectory to re-initialise, which is what made every previous hand-over
   fall. We steer it home with a simple heading controller.

2. Turn off collision between the G1 and (conveyor + placed box) at the moment
   of the swap. The robot finishes at x~2.68 with the conveyor edge at x=2.90
   and the box at chest height (z=0.92) right beside it, so turning around it
   was fouling both.
"""
import sys

p = "/root/oc_astra.py"
s = open(p).read()
todo = []

todo.append((
    "    phase = \"carry\"\n    stand_off = False",
    "    phase = \"carry\"\n    stand_off = False\n"
    "    loco = None\n"
    "    from policy.loco_mode.LocoMode import LocoMode",
    "LocoMode import"))

old = '''            if LOOP and placed and still > 700:'''
new = '''            if placed and still > 250:
                log(f"[astra] placed at {np.round(opos,2)} -- clearing collisions, "
                    f"hot-swapping to LocoMode")
                # the robot is boxed in: conveyor edge at x=2.90, box at z=0.92
                # right beside it. Free it so it can turn and walk back.
                for path in ("/World/crate",):
                    pr = stage.GetPrimAtPath(path)
                    if pr and pr.IsValid():
                        for q in Usd.PrimRange(pr):
                            if q.HasAPI(UsdPhysics.CollisionAPI):
                                UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
                for q in Usd.PrimRange(stage.GetPrimAtPath("/World/Astra")):
                    if "/Conveyor" in q.GetPath().pathString and \\
                            q.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
                # velocity-driven walker: no reference to re-init
                loco = LocoMode(sc, po)
                loco.enter()
                policy = loco
                hold_action = np.asarray(po.actions, dtype=np.float32).copy()
                blend_i = 0
                phase = "home"

            if False:'''
todo.append((old, new, "LocoMode swap + collision clear"))

# drive LocoMode home with a heading controller
todo.append((
    "        if step % 4 == 0:\n            try:\n                policy.run()",
    '''        if phase == "home":
            # steer LocoMode back to the origin
            d = np.array([0.0, 0.0]) - bpos[:2]
            dist = float(np.linalg.norm(d))
            wq = bquat
            yaw = np.arctan2(2 * (wq[0] * wq[3] + wq[1] * wq[2]),
                             1 - 2 * (wq[2] ** 2 + wq[3] ** 2))
            desired = np.arctan2(d[1], d[0])
            err = (desired - yaw + np.pi) % (2 * np.pi) - np.pi
            sc.vel_cmd[0] = float(np.clip(dist, 0.0, 0.45)) if abs(err) < 0.6 else 0.0
            sc.vel_cmd[1] = 0.0
            sc.vel_cmd[2] = float(np.clip(err, -0.6, 0.6))
            if dist < 0.35:
                sc.vel_cmd[:] = 0.0

        if step % 4 == 0:
            try:
                policy.run()''',
    "home steering"))

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
