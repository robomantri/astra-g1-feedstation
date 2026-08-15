import sys

p = "/root/oc_astra.py"
s = open(p).read()
todo = []

# 1. narrower stand in the travel direction
a = "            scale=np.array([0.80, 0.70, TABLE_TOP]),"
b = "            scale=np.array([0.50, 0.70, TABLE_TOP]),"
todo.append((a, b, "stand width"))

# 2. flag
a = '    phase = "carry"'
b = '    phase = "carry"\n    stand_off = False'
todo.append((a, b, "stand_off flag"))

# 3. drop stand collision once the box is lifted, so the robot does not trip
#    over it while walking to the conveyor
a = "        # --- once the box is placed and has stopped moving, walk home ---"
b = '''        # The stand only has to hold the box until it is picked up. It sits at
        # (1.5, 0), right on the path to the conveyor at (3.3, 0), and the robot
        # trips over it mid-carry. Once the box is up, drop its collision.
        if phase == "carry" and opos[2] > 0.60 and not stand_off:
            tp = stage.GetPrimAtPath("/World/table")
            if tp and tp.IsValid():
                for q in Usd.PrimRange(tp):
                    if q.HasAPI(UsdPhysics.CollisionAPI):
                        UsdPhysics.CollisionAPI(q).CreateCollisionEnabledAttr(False)
                stand_off = True
                log(f"[astra] box lifted -- stand collision off @{step}")

        # --- once the box is placed and has stopped moving, walk home ---'''
todo.append((a, b, "stand collision off"))

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
