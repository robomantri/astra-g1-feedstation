#!/usr/bin/env python3
"""Drop the ASTRA SWEETS scan into the CoorDex WalkPickTurn scene."""
P = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
s = open(P).read()
n = 0


def rep(old, new):
    global s, n
    assert old in s, "MISS: " + old[:60]
    s = s.replace(old, new, 1)
    n += 1


rep(
    "from coordex.robots.g1_wuji import G1_WUJI_ACTION_SCALE, G1_WUJI_CFG, WUJI_RH_ACTIVE_JOINT_NAMES",
    "from coordex.robots.g1_wuji import G1_WUJI_ACTION_SCALE, G1_WUJI_CFG, WUJI_RH_ACTIVE_JOINT_NAMES\n"
    "from coordex.assets import ASSET_DIR as COORDEX_ASSET_DIR\n"
    "\n"
    "# ASTRA SWEETS factory scan, dropped in as a visual backdrop.\n"
    "#\n"
    "# The scan is authored ~50 m from its own origin; (51.5357, -30.5250, 2.65648)\n"
    "# is the shift that puts the room's min corner at (0,0,0) with the floor at z=0.\n"
    "# The robot spawns at (0,0) and works out to x=1.5, so subtract a further\n"
    "# (4.0, 2.0) to land that workspace in the middle of the room's open walk lane\n"
    "# instead of in a corner. Raised 1 mm so it does not z-fight the plane terrain,\n"
    "# which stays as the physical floor -- the scan is scenery, not collision.\n"
    "OFFICE_POS = (51.5357 - 4.0, -30.5250 - 2.0, 2.65648 + 0.001)",
)

rep(
    '    light = AssetBaseCfg(\n        prim_path="/World/light",',
    '    office = AssetBaseCfg(\n'
    '        prim_path="/World/office",\n'
    '        spawn=sim_utils.UsdFileCfg(usd_path=f"{COORDEX_ASSET_DIR}/office.usdc"),\n'
    '        init_state=AssetBaseCfg.InitialStateCfg(pos=OFFICE_POS),\n'
    '    )\n'
    '\n'
    '    light = AssetBaseCfg(\n        prim_path="/World/light",',
)

open(P, "w").write(s)
print(f"applied {n} edits")

import ast
ast.parse(open(P).read())
print("syntax OK")
