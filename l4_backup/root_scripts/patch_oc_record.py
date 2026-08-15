"""Add offscreen video recording to OmniContact's headless runner.

The headless loop renders nothing. This adds a mujoco.Renderer that captures a
frame every N sim steps and writes an mp4, gated on the OC_RECORD env var so
default behaviour is unchanged. Needs MUJOCO_GL=egl on a headless box.

OC_RECORD        output mp4 path
OC_RECORD_EVERY  capture every N sim steps (default 10 -> 0.005*10 = 20 fps sim time)
OC_RECORD_FPS    output fps (default 30)
"""
import sys

p = "/root/OmniContact_sim2sim/deploy_omnicontact/run_skill_omnicontact.py"
s = open(p).read()

if "OC_RECORD" in s:
    print("already patched")
    sys.exit(0)

old = """                tau = self._pd_control()
                self.d.ctrl[:] = tau
                mujoco.mj_step(self.m, self.d)
                self._update_visualization()
                self.sim_counter += 1"""

new = """                tau = self._pd_control()
                self.d.ctrl[:] = tau
                mujoco.mj_step(self.m, self.d)
                self._update_visualization()
                self._oc_record_tick()
                self.sim_counter += 1
            self._oc_record_close()"""

if s.count(old) != 1:
    print(f"FAIL: expected 1 headless loop tail, found {s.count(old)}")
    sys.exit(1)
s = s.replace(old, new)

# Append the recorder methods to the class that defines run().
anchor = "    def run(self):"
methods = '''    def _oc_record_tick(self):
        import os
        path = os.environ.get("OC_RECORD", "")
        if not path:
            return
        every = int(os.environ.get("OC_RECORD_EVERY", "10"))
        if self.sim_counter % every != 0:
            return
        if not hasattr(self, "_oc_writer"):
            import imageio
            self._oc_renderer = mujoco.Renderer(self.m, height=720, width=1280)
            self._oc_cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(self._oc_cam)
            self._oc_cam.distance = 4.5
            self._oc_cam.azimuth = 120.0
            self._oc_cam.elevation = -18.0
            fps = int(os.environ.get("OC_RECORD_FPS", "30"))
            self._oc_writer = imageio.get_writer(path, fps=fps)
            print(f"[oc-record] writing {path}")
        # keep the robot centred as it walks
        self._oc_cam.lookat[:] = self.d.qpos[0:3]
        self._oc_renderer.update_scene(self.d, camera=self._oc_cam)
        self._oc_writer.append_data(self._oc_renderer.render())

    def _oc_record_close(self):
        w = getattr(self, "_oc_writer", None)
        if w is not None:
            w.close()
            del self._oc_writer
            print("[oc-record] closed")

    def run(self):'''

if s.count(anchor) != 1:
    print(f"FAIL: expected 1 'def run', found {s.count(anchor)}")
    sys.exit(1)
s = s.replace(anchor, methods)

open(p, "w").write(s)
print("patched OK")
