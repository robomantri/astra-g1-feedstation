"""Add a frame cap so the recorded mp4 is always finalised.

rl_games' player has no games_num configured here, so --test loops for a very
large number of episodes. Killing it mid-run leaves the imageio writer unclosed
and the mp4 unreadable (no moov atom). Cap the frame count, close the writer,
and exit cleanly.

INTERMIMIC_RECORD_MAXFRAMES (default 900 = 30 s @ 30 fps).
"""
import sys

p = "/root/InterMimic/isaacgym/src/intermimic/env/tasks/base_task.py"
s = open(p).read()

if "INTERMIMIC_RECORD_MAXFRAMES" in s:
    print("already patched")
    sys.exit(0)

old = """        h, w = img.shape
        self._im_writer.append_data(img.reshape(h, w // 4, 4)[:, :, :3])"""

new = """        h, w = img.shape
        self._im_writer.append_data(img.reshape(h, w // 4, 4)[:, :, :3])

        self._im_frames = getattr(self, "_im_frames", 0) + 1
        cap = int(float(os.environ.get("INTERMIMIC_RECORD_MAXFRAMES", "900")))
        if self._im_frames >= cap:
            self._im_writer.close()
            print(f"[record] wrote {self._im_frames} frames -> {self._im_record_path}")
            sys.stdout.flush()
            os._exit(0)  # hard exit: IsaacGym teardown segfaults and would kill the file"""

if s.count(old) != 1:
    print(f"FAIL: expected 1 occurrence, found {s.count(old)}")
    sys.exit(1)

open(p, "w").write(s.replace(old, new))
print("patched OK")
