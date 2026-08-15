import shutil
F = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
shutil.copy(F, F + ".preslim")
s = open(F).read(); n = 0
def rep(o, x):
    global s, n
    assert o in s, "MISS: " + o[:70]
    s = s.replace(o, x, 1); n += 1

rep('usd_path="/root/astra_workspace/astra_workspace.usd")',
    'usd_path="/root/astra_workspace/astra_workspace_1bay.usd")')

rep('''    dressing = AssetBaseCfg(
        prim_path="/World/dressing",
        spawn=sim_utils.UsdFileCfg(usd_path=f"{COORDEX_ASSET_DIR}/dressing.usda"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )
''',
'''    # dressing.usda disabled: it dressed the BARE office scan with props, and the
    # workspace USD already brings its own crates, conveyor and piles. Kept as None
    # rather than deleted so it is one line to restore.
    dressing = None
''')
open(F, "w").write(s)
import ast; ast.parse(s)
print(f"applied {n} edits, syntax OK; backup {F}.preslim")
