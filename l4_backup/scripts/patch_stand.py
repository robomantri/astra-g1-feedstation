"""Stand collision is only needed until the box is lifted.

At a 0.25 m stand the G1 successfully picks the box up, then trips over the
stand while walking to the conveyor and falls. The stand sits at (1.5, 0),
directly on the path to the goal at (3.3, 0). Once the box is up the stand is
pure scenery, so drop its collision then.
"""
import sys

p = "/root/oc_astra.py"
s = open(p).read()
n = 0

a = "TABLE_TOP = 0.25                           # low pallet stand"
b = "TABLE_TOP = 0.35                           # low stand the box starts on"
if a in s:
    s = s.replace(a, b); n += 1

a = "scale=np.array([0.70, 0.70, TABLE_TOP]),"
b = "scale=np.array([0.50, 0.70, TABLE_TOP]),"
if a in s:
    s = s.replace(a, b); n += 1

a = '    phase, still, prev_box = "carry", 0, None'
b = '    phase, still, prev_box, stand_off = "carry", 0, None, False'
if a in s:
    s = s.replace(a, b); n += 1

a = "        # once the box is on the conveyor and has stopped moving, walk home"
b = '''        # the stand has done its job once the box is up; stop it tripping the
        # robot on the way to the conveyor
        if phase == "carry" and opos[2] > 0.60 and not stand_off:
            tp = stage.GetPrimAtPath("/World/table")
            if tp and tp.IsValid():
                for q in Usd.PrimRange(tp):
                    if q.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
                stand_off = True
                log(f"[astra] box lifted -- stand collision off @{step}")

        # once the box is on the conveyor and has stopped moving, walk home'''
if a in s:
    s = s.replace(a, b, 1); n += 1

open(p, "w").write(s)
print(f"applied {n}/4 changes")
if n != 4:
    print("WARNING: not all changes applied")
    sys.exit(1)
