"""Add headless video recording to InterMimic.

InterMimic has no --record-video for policy testing. Two changes to base_task.py,
both gated behind the INTERMIMIC_RECORD env var so default behaviour is untouched:

1. __init__: headless normally forces graphics_device_id = -1, which disables ALL
   rendering so create_camera_sensor() would return -1. Keep the graphics device
   when recording (offscreen camera sensors need no viewer).
2. render(): currently no-ops headless because self.viewer is None. When
   recording, lazily create a camera on first call (envs exist by then) and
   append a frame each step. The existing rl_games test loop already calls
   render() every step, so recording comes for free.

INTERMIMIC_RECORD = output mp4 path. Optional INTERMIMIC_RECORD_FPS (default 30).
"""
import sys

p = "/root/InterMimic/isaacgym/src/intermimic/env/tasks/base_task.py"
s = open(p).read()

if "INTERMIMIC_RECORD" in s:
    print("already patched")
    sys.exit(0)

old_init = """        self.graphics_device_id = self.device_id
        if enable_camera_sensors == False and self.headless == True:
            self.graphics_device_id = -1"""
new_init = """        self.graphics_device_id = self.device_id
        self._im_record_path = os.environ.get("INTERMIMIC_RECORD", "")
        if enable_camera_sensors == False and self.headless == True and not self._im_record_path:
            self.graphics_device_id = -1"""

old_render = """    def render(self, sync_frame_time=False):
        if self.viewer:"""
new_render = """    def _im_record_frame(self):
        # Lazy camera creation: self.envs only exists after _create_envs().
        if not getattr(self, "_im_writer", None):
            import imageio
            props = gymapi.CameraProperties()
            props.width, props.height = 1280, 720
            self._im_cam = self.gym.create_camera_sensor(self.envs[0], props)
            if self._im_cam == -1:
                print("[record] create_camera_sensor failed; disabling recording")
                self._im_record_path = ""
                return
            fps = int(float(os.environ.get("INTERMIMIC_RECORD_FPS", "30")))
            self._im_writer = imageio.get_writer(self._im_record_path, fps=fps)
            print(f"[record] writing {self._im_record_path}")

        # Track the humanoid so a walking/carrying robot stays in frame.
        try:
            root = self._humanoid_root_states[0, :3].cpu().numpy()
        except Exception:
            root = np.zeros(3)
        eye = gymapi.Vec3(root[0] + 2.2, root[1] - 2.2, root[2] + 1.2)
        tgt = gymapi.Vec3(root[0], root[1], root[2])
        self.gym.set_camera_location(self._im_cam, self.envs[0], eye, tgt)

        self.gym.step_graphics(self.sim)
        self.gym.render_all_camera_sensors(self.sim)
        img = self.gym.get_camera_image(self.sim, self.envs[0], self._im_cam,
                                        gymapi.IMAGE_COLOR)
        h, w = img.shape
        self._im_writer.append_data(img.reshape(h, w // 4, 4)[:, :, :3])

    def render(self, sync_frame_time=False):
        if getattr(self, "_im_record_path", ""):
            if self.device != 'cpu':
                self.gym.fetch_results(self.sim, True)
            self._im_record_frame()
        if self.viewer:"""

for old, new, label in ((old_init, new_init, "init"), (old_render, new_render, "render")):
    if s.count(old) != 1:
        print(f"FAIL: {label}: expected 1 occurrence, found {s.count(old)}")
        sys.exit(1)
    s = s.replace(old, new)

# make sure os / np are importable in this module
if "\nimport os" not in s:
    s = s.replace("import sys", "import sys\nimport os", 1)
if "import numpy as np" not in s:
    s = s.replace("import sys", "import sys\nimport numpy as np", 1)

open(p, "w").write(s)
print("patched OK")
