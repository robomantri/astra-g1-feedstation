import re, shutil
F = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
shutil.copy(F, F + ".predressremove")
s = open(F).read()
old = '''    dressing = AssetBaseCfg(
        prim_path="/World/dressing",
        spawn=sim_utils.UsdFileCfg(usd_path=f"{COORDEX_ASSET_DIR}/dressing.usda"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )
'''
new = '''    # dressing.usda removed: it dressed the BARE office scan with props. The
    # workspace USD now brings its own crates, conveyor and piles, so this was
    # duplicating them. Set to None rather than deleted so it is easy to restore.
    dressing = None
'''
assert old in s, "dressing block not found"
s = s.replace(old, new, 1)
open(F, "w").write(s)
import ast; ast.parse(s)
print("dressing disabled; syntax OK")
