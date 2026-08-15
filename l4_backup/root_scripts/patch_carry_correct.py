"""Correct the g1_hoi config for the CARRY motion.

Evidence for this pairing (measured, not guessed):
  support plate top   = 0.507 + 0.01/2 = 0.512 m   (static box, num_actors==3 only)
  suitcase half-height= 0.1653 m  -> resting centre z = 0.6775
  carry.npz object z0 = 0.677                       <-- 0.5 mm match
  box half-height     = 0.1150 m  -> resting centre z = 0.6270  (5 cm too low)

So carry.npz was recorded with the SUITCASE on the support plate. Two things must
hold together or the object simply falls at reset:
  1. object = suitcase (matches the reference trajectory height)
  2. num_actors = 3     (otherwise no support plate is ever spawned)
"""
import sys

p = "/root/ResMimic/legged_gym/legged_gym/envs/g1/g1_hoi_config.py"
s = open(p).read()

pairs = [
    # back to the object the carry reference was recorded with
    ("object_urdf_file = 'box/box.urdf'", "object_urdf_file = 'suitcase/suitcase.urdf'"),
    ("object_obj_file = 'box/box.obj'", "object_obj_file = 'suitcase/suitcase.obj'"),
    # spawn the support plate the object rests on
    ("num_actors = 2", "num_actors = 3"),
]

for old, new in pairs:
    if new in s and old not in s:
        print(f"already set: {new}")
        continue
    if s.count(old) != 1:
        print(f"FAIL: expected 1 occurrence of {old!r}, found {s.count(old)}")
        sys.exit(1)
    s = s.replace(old, new)

open(p, "w").write(s)
print("patched OK")
for line in s.splitlines():
    t = line.strip()
    if t.startswith(("object_urdf_file", "object_obj_file", "num_actors",
                     "motion_file", "object_motion_file")):
        print("   ", t)
