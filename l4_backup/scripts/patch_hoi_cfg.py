"""Switch ResMimic's g1_hoi task from suitcase+kneel to box+carry.

carry.npz starts the object at z=0.677 (table height) and lifts it 0.277 m --
that is the tabletop pick-and-carry we want. kneel.npz starts at z=0.012
(floor pickup, needs a kneel).
"""
import sys

p = "/root/ResMimic/legged_gym/legged_gym/envs/g1/g1_hoi_config.py"
s = open(p).read()
orig = s

pairs = [
    ("object_urdf_file = 'suitcase/suitcase.urdf'", "object_urdf_file = 'box/box.urdf'"),
    ("object_obj_file = 'suitcase/suitcase.obj'", "object_obj_file = 'box/box.obj'"),
    ('motion_file = f"{LEGGED_GYM_ROOT_DIR}/assets/motions/kneel.pkl"',
     'motion_file = f"{LEGGED_GYM_ROOT_DIR}/assets/motions/carry.pkl"'),
    ('object_motion_file = f"{LEGGED_GYM_ROOT_DIR}/assets/motions/kneel.npz"',
     'object_motion_file = f"{LEGGED_GYM_ROOT_DIR}/assets/motions/carry.npz"'),
]

for old, new in pairs:
    n = s.count(old)
    if n != 1:
        print(f"FAIL: expected 1 occurrence of {old!r}, found {n}")
        sys.exit(1)
    s = s.replace(old, new)

open(p, "w").write(s)
print("patched OK")
for line in s.splitlines():
    t = line.strip()
    if t.startswith(("object_urdf_file", "object_obj_file", "motion_file", "object_motion_file")):
        print("   ", t)
