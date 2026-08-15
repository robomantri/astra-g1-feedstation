"""Spawn the handless H1-2 USD and dump its true articulation layout."""
from isaaclab.app import AppLauncher
app = AppLauncher(headless=True).app

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg

USD = "/root/h12/unitree_model/H1-2/h1_2_handless/h1_2_handless.usd"

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.005, device="cpu"))
cfg = ArticulationCfg(
    prim_path="/World/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=USD, activate_contact_sensors=True),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 1.05)),
    actuators={},
)
robot = Articulation(cfg)
sim.reset()

print("\n##### H1-2 HANDLESS ARTICULATION #####")
print("num_joints :", robot.num_joints)
print("num_bodies :", robot.num_bodies)
print("\n--- JOINT ORDER (PhysX) ---")
for i, n in enumerate(robot.joint_names):
    print(f"  [{i:2d}] {n}")
print("\n--- BODY NAMES ---")
for i, n in enumerate(robot.body_names):
    print(f"  [{i:2d}] {n}")
lim = robot.data.joint_pos_limits[0]
print("\n--- JOINT LIMITS (rad) ---")
for i, n in enumerate(robot.joint_names):
    print(f"  {n:34s} [{lim[i,0]:+.3f}, {lim[i,1]:+.3f}]")
print("\n--- TOTAL MASS ---", float(robot.data.default_mass[0].sum()), "kg")
app.close()
