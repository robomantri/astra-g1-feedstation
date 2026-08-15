"""Tame the blown-out exposure so the scan's detail is visible."""
P = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
s = open(P).read()
import shutil, ast
shutil.copy(P, P + ".prelight")

old_key = """        spawn=sim_utils.DistantLightCfg(
            color=(0.75, 0.75, 0.75), intensity=3000.0),"""
new_key = """        spawn=sim_utils.DistantLightCfg(
            # Was 3000 with a white-ish tint, which clipped the untextured scan
            # mesh to pure white. Lower key + warmer tone keeps the walls and
            # floor readable instead of a flat blowout.
            color=(0.95, 0.93, 0.88), intensity=1800.0),"""
assert old_key in s, "key light block not found"
s = s.replace(old_key, new_key, 1)

old_dome = """        spawn=sim_utils.DomeLightCfg(
            color=(0.13, 0.13, 0.13), intensity=1000.0),"""
new_dome = """        spawn=sim_utils.DomeLightCfg(
            # Brighter, slightly cool ambient fill so shadowed sides of the
            # scan geometry (rails, pillars, the feed station) pick up shape
            # rather than going black.
            color=(0.45, 0.47, 0.52), intensity=1500.0),"""
assert old_dome in s, "dome light block not found"
s = s.replace(old_dome, new_dome, 1)

open(P, "w").write(s)
ast.parse(open(P).read())
print("lighting patched, syntax OK")
