"""Bimanual weld for InterMimic's G1 task.

The G1 policy performs a correct trained BIMANUAL REACH down onto the box but
never completes the grip, so the object stays on the floor. This welds the
object to the hands once both are close: from then on the object's root pose is
driven from the midpoint of the two hand links each step, so it rises with the
robot as the reference motion stands back up.

Only the grip is synthetic -- approach, posture and two-handed geometry are the
real learned behaviour.

Enabled with G1_WELD=1. G1_WELD_DIST sets the trigger radius (default 0.35 m).
"""
import sys

p = "/root/InterMimic/isaacgym/src/intermimic/env/tasks/intermimic_g1.py"
s = open(p).read()

if "G1_WELD" in s:
    print("already patched")
    sys.exit(0)

old = """    def _compute_reward(self, actions):
        super()._compute_reward(actions)
        return"""

# _compute_reward runs every step after physics, so it is a valid weld hook.
new = '''    def _compute_reward(self, actions):
        super()._compute_reward(actions)
        self._g1_weld_step()
        return

    def _g1_weld_step(self):
        import os
        if os.environ.get("G1_WELD", "") != "1":
            return
        thr = float(os.environ.get("G1_WELD_DIST", "0.35"))

        # Resolve hand body ids once.
        if not hasattr(self, "_weld_ids"):
            names = ["left_wrist_yaw_link", "right_wrist_yaw_link"]
            try:
                ids = self._build_key_body_ids_tensor(names)
            except Exception as e:
                print(f"[weld] could not resolve hand bodies: {e}")
                self._weld_ids = None
                return
            self._weld_ids = ids
            self._weld_on = torch.zeros(self.num_envs, dtype=torch.bool,
                                        device=self.device)
            print(f"[weld] enabled thr={thr} hand_ids={ids.tolist()}")
        if self._weld_ids is None:
            return

        lh = self._rigid_body_pos[:, self._weld_ids[0], :]
        rh = self._rigid_body_pos[:, self._weld_ids[1], :]
        obj = self._target_states[:, 0:3]

        dl = torch.norm(lh - obj, dim=-1)
        dr = torch.norm(rh - obj, dim=-1)
        newly = (~self._weld_on) & (dl < thr) & (dr < thr)
        if bool(newly.any()):
            self._weld_on |= newly
            print(f"[weld] attached envs={newly.nonzero().flatten().tolist()} "
                  f"dl={dl.min().item():.3f} dr={dr.min().item():.3f}")

        # Episode restarts drop the weld.
        self._weld_on &= (self.progress_buf > 2)

        if bool(self._weld_on.any()):
            mid = 0.5 * (lh + rh)
            idx = self._weld_on.nonzero().flatten()
            self._target_states[idx, 0:3] = mid[idx]
            self._target_states[idx, 7:13] = 0.0   # zero lin+ang velocity
            tar_ids = self._tar_actor_ids[idx].contiguous()
            self.gym.set_actor_root_state_tensor_indexed(
                self.sim, gymtorch.unwrap_tensor(self._root_states),
                gymtorch.unwrap_tensor(tar_ids), len(tar_ids))
        return'''

if s.count(old) != 1:
    print(f"FAIL: expected 1 _compute_reward, found {s.count(old)}")
    sys.exit(1)

s = s.replace(old, new)

# ensure torch / gymtorch available in this module
if "import torch" not in s:
    s = "import torch\n" + s
if "gymtorch" not in s.split("def ")[0]:
    s = "from isaacgym import gymtorch\n" + s

open(p, "w").write(s)
print("patched OK")
