F = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
s = open(F).read()
old = '''        self.viewer.origin_type = "world"
        self.viewer.eye = (-2.2, -1.4, 2.1)
        self.viewer.lookat = (1.2, 0.9, 0.8)'''
new = '''        self.viewer.origin_type = "world"
        # Shift the shot by the same delta as the robot/table/cube, otherwise the
        # camera keeps pointing at the robot's old (0,0) spawn and the robot walks
        # out of frame entirely.
        self.viewer.eye = (-2.2 + SPAWN_DELTA[0], -1.4 + SPAWN_DELTA[1], 2.1)
        self.viewer.lookat = (1.2 + SPAWN_DELTA[0], 0.9 + SPAWN_DELTA[1], 0.8)'''
assert old in s, "camera block not found"
open(F, "w").write(s.replace(old, new, 1))
import ast; ast.parse(open(F).read())
print("camera shifted by SPAWN_DELTA; syntax OK")
