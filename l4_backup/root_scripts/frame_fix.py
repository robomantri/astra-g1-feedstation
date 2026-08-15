"""Pull the camera back to include the hopper, and stop clipping the scan to white.

The scan's room centre lands at world (1.54, 1.99) once OFFICE_POS is applied,
and the hopper/feed station sits right about there. The robot spawns at (0, 0)
and works out to the table at x=1.5 -- so the hopper is only ~2 m away in +y,
but the camera was parked at a (1.5, 1.5, 1.5) offset from the robot, which
crops to the robot's immediate surroundings and pushes the hopper out of frame.

Camera goes to the -y side, further out and higher, looking back at the robot,
so the hopper and the railing line sit behind it -- roughly the 3/4 elevated
composition of the reference render.

Lighting comes down hard. These meshes carry flat near-white materials with no
textures, so at the previous levels every surface clipped to pure white and all
the shape information was lost.
"""
import ast
import shutil

P = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
s = open(P).read()
shutil.copy(P, P + ".preframe")
n = 0


def rep(old, new):
    global s, n
    assert old in s, "MISS: " + old[:70]
    s = s.replace(old, new, 1)
    n += 1


# --- Camera: wide 3/4 view that takes in the hopper behind the robot.
rep(
    "        self.viewer.eye = (1.5, 1.5, 1.5)",
    "        self.viewer.eye = (-1.6, -3.8, 2.3)",
)

# --- Key light: was clipping every surface to white.
rep(
    """            color=(0.95, 0.93, 0.88), intensity=1800.0),""",
    """            color=(0.95, 0.93, 0.88), intensity=750.0),""",
)

# --- Ambient fill: enough to keep shadowed sides readable, not enough to flatten.
rep(
    """            color=(0.45, 0.47, 0.52), intensity=1500.0),""",
    """            color=(0.38, 0.40, 0.46), intensity=500.0),""",
)

open(P, "w").write(s)
ast.parse(open(P).read())
print(f"applied {n} edits -- syntax OK")
