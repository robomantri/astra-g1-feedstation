"""Restore the known-good single carry and reshoot from the far side.

- floor pick -> conveyor place (the configuration that worked cleanly)
- no stand, no respawn loop
- camera moved to +y looking toward -y so the Astra branded wall panels
  (world y = -5.01, x -19.8..12.6, up to z 3.11) form the backdrop
"""
import sys

p = "/root/oc_astra.py"
s = open(p).read()
todo = []

# box back on the floor
todo.append((
    "CRATE_Z = TABLE_TOP + CRATE_HALF[2]        # box starts ON the table",
    "CRATE_Z = CRATE_HALF[2]                    # box on the floor (known-good)",
    "floor pick"))

# no stand
todo.append(("    if True:\n        world.scene.add(FixedCuboid(",
             "    if False:\n        world.scene.add(FixedCuboid(",
             "stand off"))

# no respawn loop -- make it opt-in
todo.append(("            if placed and still > 700:",
             "            if LOOP and placed and still > 700:",
             "loop opt-in"))
todo.append(('cli.add_argument("--plain-box", action="store_true",',
             'cli.add_argument("--loop", action="store_true", help="respawn the box and repeat")\n'
             'cli.add_argument("--plain-box", action="store_true",',
             "loop flag"))

# camera: far side, walls behind the action
todo.append((
    "set_camera_view(eye=[-1.6, -4.2, 2.3], target=[2.2, 0.1, 0.6])",
    "set_camera_view(eye=[0.1, 6.2, 2.5], target=[2.4, -1.2, 0.75])",
    "camera to far side"))

n = 0
for a, b, label in todo:
    if s.count(a) != 1:
        print(f"MISS ({s.count(a)}x): {label}")
        continue
    s = s.replace(a, b, 1)
    n += 1
    print(f"ok: {label}")

# LOOP global
if "LOOP =" not in s:
    s = s.replace("ASTRA_OFFSET = (1.72, -5.10, 0.0)",
                  "ASTRA_OFFSET = (1.72, -5.10, 0.0)\nLOOP = A.loop", 1)
    print("ok: LOOP global")
    n += 1

open(p, "w").write(s)
print(f"applied {n}/{len(todo)+1}")
sys.exit(0 if n == len(todo) + 1 else 1)
