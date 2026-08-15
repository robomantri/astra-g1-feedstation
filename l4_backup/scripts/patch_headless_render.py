"""Allow headless camera-sensor rendering.

base_task sets graphics_device_id = -1 whenever headless, which disables ALL
rendering -- so create_camera_sensor() returns -1 and render_record() fails with
"could not find camera with handle -1". IsaacGym supports offscreen camera
sensors on a headless box (no viewer, but a real graphics device), which is
exactly what --record_video needs. Keep the graphics device when the config asks
for video.
"""
import sys

p = "/root/ResMimic/legged_gym/legged_gym/envs/base/base_task.py"
s = open(p).read()

old = """        self.graphics_device_id = self.sim_device_id
        if self.headless == True:
            self.graphics_device_id = -1"""

new = """        self.graphics_device_id = self.sim_device_id
        if self.headless == True and not getattr(cfg.env, "record_video", False):
            # Headless normally disables rendering entirely. Keep the graphics
            # device when recording: camera sensors render offscreen without a
            # viewer, and -1 would make create_camera_sensor() return -1.
            self.graphics_device_id = -1"""

if new in s:
    print("already patched")
    sys.exit(0)
if s.count(old) != 1:
    print(f"FAIL: expected 1 occurrence, found {s.count(old)}")
    sys.exit(1)

open(p, "w").write(s.replace(old, new))
print("patched OK")
