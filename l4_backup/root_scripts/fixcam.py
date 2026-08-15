"""Switch the demo to a fixed world camera.

Tracking the robot (origin_type="asset_root") swings the eye around during the
180-degree turn stage, which pushed it through a wall and blanked the last
~1.5 s of every episode. A static 3/4 shot avoids that entirely and matches the
composition of the reference render.

Room in world coords after OFFICE_POS: x -4.0..7.08, y -2.0..5.97, z 0..3.4.
Robot walks (0,0) -> (1.5,0); hopper sits near (1.5, 2.0).
"""
import ast
import re
import shutil

P = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
s = open(P).read()
shutil.copy(P, P + ".prefixcam")

lines = s.split("\n")
out, skipped = [], 0
for ln in lines:
    st = ln.strip()
    if st.startswith("self.viewer.") or st.startswith("# Room spans world x") or (
        st.startswith("#") and skipped and st.startswith("#") and "camera is an offset" in st.lower()
    ):
        skipped += 1
        continue
    out.append(ln)
s = "\n".join(out)

block = '''        # viewer settings -- fixed world camera (not robot-tracking).
        # asset_root tracking swung the eye through the wall during the
        # 180-degree turn stage, blanking the tail of every episode.
        # Room is world x -4.0..7.08, y -2.0..5.97, z 0..3.4; the hopper
        # sits near (1.5, 2.0) so this shot keeps it behind the robot.
        self.viewer.origin_type = "world"
        self.viewer.eye = (-2.2, -1.4, 2.1)
        self.viewer.lookat = (1.2, 0.9, 0.8)
'''

anchor = "        # viewer settings\n"
if anchor in s:
    s = s.replace(anchor, block, 1)
else:
    # Fall back: insert before episode_length_s assignment.
    m = re.search(r"^(\s*)self\.episode_length_s\s*=", s, re.M)
    s = s[: m.start()] + block + s[m.start():]

open(P, "w").write(s)
ast.parse(open(P).read())
print("fixed camera applied -- syntax OK")
for ln in open(P):
    if "viewer" in ln:
        print("   ", ln.rstrip())
