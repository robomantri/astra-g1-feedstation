"""Continuous loop: place box -> box reappears on the stand -> fetch it again.

Why this beats the walk-home approach: CFgen seeds its plan from the robot's
ACTUAL pelvis pose (CFgen_reference.py:165 `pelvis_pos=fk_info["pelvis"]["pos"]`),
so it can re-plan a fresh carry from wherever the robot is standing. And
plan_cfgen_reference() rewrites the reference IN PLACE on the live policy --
same ONNX session, same 5-frame observation history -- so there is no policy
hand-over and none of the discontinuity that made the robot fall.

Loop per cycle:
  1. box settles on the conveyor
  2. teleport it back onto the stand (and re-enable the stand's collision)
  3. plan_cfgen_reference(...) -> new carry plan from current pose
  4. policy.counter_step = 0
"""
import sys

p = "/root/oc_astra.py"
s = open(p).read()
todo = []

todo.append((
    '    phase = "carry"\n    stand_off = False',
    '    phase = "carry"\n    stand_off = False\n'
    '    cycle = 0\n'
    '    from policy.omnicontact.CFgen_reference import plan_cfgen_reference',
    "loop state + import"))

# replace the whole walk-home block with a respawn-and-replan block
old_home = '''            if placed and still > 150:
                log(f"[astra] box placed at {np.round(opos,2)} -- walking home")
                runner2 = build_runner(
                    task="loco",
                    init=(float(bpos[0]), float(bpos[1])),
                    goal=HOME)
                hold_action = np.asarray(po.actions, dtype=np.float32).copy()
                policy, sc, po = runner2.policy, runner2.state_cmd, runner2.policy_output
                blend_i = 0
                phase = "home"'''
new_home = '''            if placed and still > 150:
                cycle += 1
                log(f"[astra] cycle {cycle}: placed at {np.round(opos,2)} "
                    f"-- respawning box on the stand")
                # box reappears where it started
                crate.set_world_pose(
                    position=np.array([TABLE_XY[0], TABLE_XY[1], CRATE_Z]),
                    orientation=np.array([1.0, 0.0, 0.0, 0.0]))
                crate.set_linear_velocity(np.zeros(3))
                crate.set_angular_velocity(np.zeros(3))
                # stand must be solid again to hold it
                tp = stage.GetPrimAtPath("/World/table")
                if tp and tp.IsValid():
                    for q in Usd.PrimRange(tp):
                        if q.HasAPI(UsdPhysics.CollisionAPI):
                            UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(True)
                stand_off = False
                # re-plan a fresh carry FROM THE ROBOT'S CURRENT POSE, in place
                sc.obj_pos[:] = [TABLE_XY[0], TABLE_XY[1], CRATE_Z]
                sc.obj_quat[:] = [1.0, 0.0, 0.0, 0.0]
                sc.carry_box_pos[:] = sc.obj_pos
                sc.carry_box_quat[:] = sc.obj_quat
                policy.goal_pos = np.array(GOAL, dtype=np.float32)
                plan_cfgen_reference(policy, policy._get_fk_info())
                policy.counter_step = 0
                still, prev_box = 0, None'''
todo.append((old_home, new_home, "respawn + replan"))

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
