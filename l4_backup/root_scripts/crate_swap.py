"""Turn the WalkPickTurn grasp target from a bare cube into a crate/bin.

Usage:  python3 crate_swap.py <sx> <sy> <sz> <mass>

Why this is parameterised rather than just set to a real bin size: the policy
checkpoint (walkpickturn_30k.pt) is a *residual* trained against a 0.04 m,
0.2 kg cube with the WUJI hand. The hand prior decodes finger targets tuned to
that aperture. Grow the object too far and the fingers close on air or jam, and
the pick fails outright -- so the size has to be walked up empirically and
checked in a render each time, not assumed.

The reference image shows a two-handed carry of a full EUR stacking crate
(600x400x220 mm). That is a different manipulation task from what this
checkpoint knows how to do; this makes the object read as a bin at whatever
size the trained grasp actually tolerates.
"""
import ast
import shutil
import sys

P = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"

sx, sy, sz, mass = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])

s = open(P).read()
shutil.copy(P, P + ".precrate")

old = """    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.04, 0.04, 0.04),
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
                disable_gravity=False,
                max_depenetration_velocity=1.0,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.2),
        ),"""

new = f"""    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="{{ENV_REGEX_NS}}/Cube",
        spawn=sim_utils.CuboidCfg(
            # Crate/bin proportions rather than a cube, sized to what the
            # trained WUJI grasp still holds (checkpoint was trained on a
            # 0.04 m cube -- see crate_swap.py).
            size=({sx}, {sy}, {sz}),
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
                disable_gravity=False,
                max_depenetration_velocity=1.0,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass={mass}),
            # Grey-blue moulded-plastic look of a EUR stacking crate, so it
            # reads as a bin on camera instead of an untextured white block.
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.30, 0.35, 0.45),
                roughness=0.6,
                metallic=0.0,
            ),
        ),"""

assert old in s, "cube block not found -- config may already be patched"
s = s.replace(old, new, 1)
open(P, "w").write(s)
ast.parse(open(P).read())
print(f"crate set to {sx} x {sy} x {sz} m, {mass} kg -- syntax OK")
