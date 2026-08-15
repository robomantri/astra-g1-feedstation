"""Add the dressing asset + the lighting rig that produced the good stills
into the CoorDex walkpickturn env config."""
import re
import shutil

P = "/root/coordex/source/coordex/coordex/tasks/locomanip/walkpickturn_env_cfg.py"
shutil.copy(P, P + ".predress")
src = open(P).read()

# ---- 1. dressing asset, right after the office block -------------------------
office_block = """    office = AssetBaseCfg(
        prim_path="/World/office",
        spawn=sim_utils.UsdFileCfg(usd_path=f"{COORDEX_ASSET_DIR}/office.usdc"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=OFFICE_POS),
    )
"""
assert office_block in src, "office block not found"

dressing_block = office_block + """
    # The scan is 8k triangles: room shell, hopper, guard rails, bollards, floor
    # markings. The branded walls, EUR candy bins, conveyor and cart from the
    # reference render are not in it -- they are authored in dressing.usda,
    # already in world coordinates, so it spawns at the origin untranslated.
    dressing = AssetBaseCfg(
        prim_path="/World/dressing",
        spawn=sim_utils.UsdFileCfg(usd_path=f"{COORDEX_ASSET_DIR}/dressing.usda"),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, 0.0)),
    )
"""
src = src.replace(office_block, dressing_block, 1)

# ---- 2. key light -----------------------------------------------------------
old_key = """        spawn=sim_utils.DistantLightCfg(
            # Was 3000 with a white-ish tint, which clipped the untextured scan
            # mesh to pure white. Lower key + warmer tone keeps the walls and
            # floor readable instead of a flat blowout.
            color=(0.95, 0.93, 0.88), intensity=750.0),
    )"""
new_key = """        spawn=sim_utils.DistantLightCfg(
            # Was 3000, then 750. The scan's materials are near-white at
            # roughness 1.0 so they blow out early; at 750 the added red wall
            # panels still washed to pink. 420 with the fills below holds them.
            color=(1.0, 0.97, 0.92), intensity=420.0, angle=2.5),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.0, 0.0, 6.0), rot=(0.86, 0.28, -0.33, 0.0)),
    )"""
assert old_key in src, "key light block not found"
src = src.replace(old_key, new_key, 1)

# ---- 3. dome light ----------------------------------------------------------
old_dome = """        spawn=sim_utils.DomeLightCfg(
            # Brighter, slightly cool ambient fill so shadowed sides of the
            # scan geometry (rails, pillars, the feed station) pick up shape
            # rather than going black.
            color=(0.38, 0.40, 0.46), intensity=500.0),
    )"""
new_dome = """        spawn=sim_utils.DomeLightCfg(
            # Ambient only -- shaping now comes from the two sphere fills, so
            # this stays low or it lifts the blacks and flattens everything.
            color=(0.48, 0.53, 0.62), intensity=110.0),
    )

    # Practical fills over the feed station and the conveyor/bin stacks. Without
    # these the north half of the room falls off to flat shadow, because the
    # scan's own env_light is bound to color_0C0C0C.exr -- RGB(12,12,12), which
    # lights nothing at all.
    fill_hopper = AssetBaseCfg(
        prim_path="/World/fillHopper",
        spawn=sim_utils.SphereLightCfg(
            color=(1.0, 0.97, 0.93), intensity=6000.0, radius=0.8),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(1.5, 1.5, 3.1)),
    )
    fill_line = AssetBaseCfg(
        prim_path="/World/fillLine",
        spawn=sim_utils.SphereLightCfg(
            color=(1.0, 0.97, 0.93), intensity=5000.0, radius=0.8),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(4.8, 3.6, 3.1)),
    )"""
assert old_dome in src, "dome light block not found"
src = src.replace(old_dome, new_dome, 1)

open(P, "w").write(src)
print("patched", P)
for tag in ("dressing.usda", "intensity=420.0", "intensity=110.0", "fillHopper", "fillLine"):
    print(f"  {tag}: {'OK' if tag in src else 'MISSING'}")
