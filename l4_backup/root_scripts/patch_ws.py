import shutil, re, sys
F = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
shutil.copy(F, F + ".preastra")
s = open(F).read()
n = 0
def rep(old, new):
    global s, n
    assert old in s, "MISS: " + old[:70]
    s = s.replace(old, new, 1); n += 1

# 1. our workspace USD is already origin-shifted, so drop the +51/-30 raw-scan offset;
#    keep only the (-4,-2) workspace centring and the 1mm z lift
rep('OFFICE_POS = (51.5357 - 4.0, -30.5250 - 2.0, 2.65648 + 0.001)',
    'OFFICE_POS = (-4.0, -2.0, 0.001)\n'
    '# Robot spawn: (4.65, 5.10) in the workspace\'s own frame, i.e. beside the\n'
    '# conveyor discharge. OFFICE_POS shifts the room by (-4,-2), so subtract that.\n'
    'ROBOT_SPAWN = (4.65 - 4.0, 5.10 - 2.0, 0.76)\n'
    '# Table and cube move by the SAME delta as the robot. The policy learned a fixed\n'
    '# robot->table geometry; moving the robot alone would put the cube out of reach.\n'
    'SPAWN_DELTA = (ROBOT_SPAWN[0], ROBOT_SPAWN[1])')

# 2. point at the dressed workspace instead of the bare scan
rep('spawn=sim_utils.UsdFileCfg(usd_path=f"{COORDEX_ASSET_DIR}/office.usdc")',
    'spawn=sim_utils.UsdFileCfg(\n'
    '            usd_path="/root/astra_workspace/astra_workspace.usd")')

# 3/4. table + cube, shifted by the same delta
rep('pos=(1.5, 0.0, 0.635)',
    'pos=(1.5 + SPAWN_DELTA[0], 0.0 + SPAWN_DELTA[1], 0.635)')
rep('pos=(1.3, 0.0, 0.675)',
    'pos=(1.3 + SPAWN_DELTA[0], 0.0 + SPAWN_DELTA[1], 0.675)')

# 5. robot spawn -- patch here, NOT in g1_wuji.py which other tasks share
rep('self.scene.robot = G1_WUJI_CFG.replace(\n            prim_path="{ENV_REGEX_NS}/Robot")',
    'self.scene.robot = G1_WUJI_CFG.replace(\n'
    '            prim_path="{ENV_REGEX_NS}/Robot",\n'
    '            init_state=G1_WUJI_CFG.init_state.replace(pos=ROBOT_SPAWN))')

open(F, "w").write(s)
import ast; ast.parse(s)
print(f"applied {n} edits, syntax OK")
print("backup at", F + ".preastra")
