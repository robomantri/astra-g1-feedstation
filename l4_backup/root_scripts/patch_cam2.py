import shutil
F = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
shutil.copy(F, F + ".precam2")
s = open(F).read()
old = '''        self.viewer.eye = (-2.2 + SPAWN_DELTA[0], -1.4 + SPAWN_DELTA[1], 2.1)
        self.viewer.lookat = (1.2 + SPAWN_DELTA[0], 0.9 + SPAWN_DELTA[1], 0.8)'''
new = '''        # Reverse angle: from the +X side looking back past the robot toward the
        # conveyor. Room is x -4.0..7.08, y -2.0..5.97, so (5.55, 3.50) is inside
        # the walls, clear of the tipper (x -0.07..3.11, y 1.32..2.10) and below
        # the crate piles (y 4.30..5.50).
        self.viewer.eye = (4.90 + SPAWN_DELTA[0], 0.40 + SPAWN_DELTA[1], 1.90)
        self.viewer.lookat = (0.40 + SPAWN_DELTA[0], 0.10 + SPAWN_DELTA[1], 0.85)'''
assert old in s, "camera block not found"
open(F, "w").write(s.replace(old, new, 1))
import ast; ast.parse(open(F).read())
print("reverse-angle camera set; syntax OK")
