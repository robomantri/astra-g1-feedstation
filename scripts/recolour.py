"""Recolour the materials of a referenced USD asset, in place, on a live stage.

Handles both shader flavours that show up in Isaac Sim / Omniverse assets:

  * ``UsdPreviewSurface``   -> ``inputs:diffuseColor``
  * ``MDL`` (OmniPBR, OmniSurface, ...) -> ``inputs:diffuse_color_constant`` &co

...and in both cases it *neutralises the albedo/diffuse texture first*, because a
bound texture always wins over the constant colour and the recolour would
silently do nothing.

Usage::

    from recolour import recolour
    n = recolour(stage, "/World/Bins", (0.55, 0.55, 0.57))
    n = recolour(stage, "/World/Conveyor", (0.2, 0.5, 0.9), match="belt", roughness=0.3)

Standalone (no Isaac Sim needed) as long as ``pxr`` is importable.
"""

from __future__ import annotations

import re

from pxr import Sdf, Usd, UsdGeom, UsdShade

__all__ = ["recolour", "de_instance", "describe_materials"]


# --- names we know about ----------------------------------------------------
# MDL / OmniPBR-family diffuse constants.  From OmniPBR_ClearCoat.mdl:
#     diffuse = tex::texture_isvalid(diffuse_texture) ? desaturated_base
#                                                     : diffuse_color_constant;
#     tinted  = diffuse * diffuse_tint;
# ...so the constant only counts once the texture is invalid, and diffuse_tint
# multiplies on top of it (which is why we force the tint to white rather than
# to the requested colour -- setting both would square it).
_MDL_DIFFUSE_INPUTS = (
    "diffuse_color_constant",   # OmniPBR / OmniPBR_ClearCoat / most Isaac assets
    "base_color_constant",      # some OmniSurface variants
    "albedo_color",             # AperturePBR / older Omni materials
    "diffuseColor",             # MDL wrappers that mirror the preview-surface name
)
_MDL_NEUTRAL_WHITE_INPUTS = ("diffuse_tint",)  # multiplier -> force to 1,1,1
# MDL albedo/diffuse texture inputs that must be killed for the colour to show.
_MDL_DIFFUSE_TEXTURES = (
    "diffuse_texture",
    "albedo_map",
    "diffuse_map",
    "base_color_texture",
    "albedo_texture",
)
_MDL_METALLIC_INPUTS = ("metallic_constant", "metalness_constant", "metallic")
_MDL_METALLIC_TEXTURES = ("metallic_texture", "metallic_map")
_MDL_ROUGHNESS_INPUTS = ("reflection_roughness_constant", "roughness_constant", "roughness")
_MDL_ROUGHNESS_TEXTURES = (
    "reflectionroughness_texture",
    "reflection_roughness_texture",
    "roughness_texture",
    "roughness_map",
)

_PREVIEW_IDS = ("UsdPreviewSurface",)


# --- small helpers ----------------------------------------------------------
def _compile(match):
    """Return a predicate(name)->bool. ``match`` is a case-insensitive regex;
    if it is not valid regex syntax it is treated as a plain substring."""
    if match is None:
        return lambda _name: True
    try:
        rx = re.compile(match, re.IGNORECASE)
        return lambda name: rx.search(name) is not None
    except re.error:
        low = match.lower()
        return lambda name: low in name.lower()


def _disconnect(inp):
    """Drop any incoming connection on a UsdShade.Input (API differs by USD version)."""
    try:
        if not inp.HasConnectedSource():
            return
    except Exception:
        pass
    for meth in ("ClearSources", "DisconnectSource"):
        fn = getattr(inp, meth, None)
        if fn is None:
            continue
        try:
            fn()
            return
        except Exception:
            continue
    # last resort: clear the connection list on the raw attribute
    try:
        inp.GetAttr().ClearConnections()
    except Exception:
        pass


def _set_input(shader, name, value, sdf_type):
    """Set inputs:<name> on a shader, creating it if needed, disconnecting first.
    Returns True if a value was authored."""
    inp = shader.GetInput(name)
    if not inp:
        inp = shader.CreateInput(name, sdf_type)
    _disconnect(inp)
    try:
        inp.Set(value)
        return True
    except Exception:
        return False


def _kill_texture(shader, name):
    """Neutralise an existing asset-valued texture input (empty path + no connection).

    Only touches inputs that actually exist -- creating empty texture inputs on
    shaders that never had one is harmless but noisy.  Returns True if it did
    something.
    """
    inp = shader.GetInput(name)
    if not inp:
        return False
    _disconnect(inp)
    try:
        inp.Set(Sdf.AssetPath(""))
        return True
    except Exception:
        # asset-path set failed (odd type) -> block the opinion instead
        try:
            inp.GetAttr().Block()
            return True
        except Exception:
            return False


def _is_mdl(prim):
    if prim.HasAttribute("info:mdl:sourceAsset"):
        return True
    for name in prim.GetAuthoredPropertyNames():
        if name.startswith("info:mdl:"):
            return True
    return False


def _shaders_of(material):
    """Every Shader prim that can plausibly drive this material's surface:
    the resolved surface source for each render context, plus every Shader
    descendant of the Material prim (catches unconnected/odd networks)."""
    found, seen = [], set()

    def add(prim):
        if prim and prim.IsValid() and prim.GetPath() not in seen:
            seen.add(prim.GetPath())
            found.append(UsdShade.Shader(prim))

    for ctx in (["mdl"], ["glslfx"], []):
        try:
            src = material.ComputeSurfaceSource(ctx)
            src = src[0] if isinstance(src, tuple) else src
            if src:
                add(src.GetPrim())
        except Exception:
            pass
    for prim in Usd.PrimRange(material.GetPrim(), Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if prim.GetTypeName() == "Shader":
            add(prim)
    return found


# --- instancing -------------------------------------------------------------
def de_instance(stage, root_path):
    """Turn off ``instanceable`` on every prim at/under ``root_path``.

    Prims reached through an instance proxy are READ-ONLY: authoring to them
    raises / silently no-ops, so any material edit inside an instanced asset is
    lost.  De-instancing (which is what the Isaac Sim UI's
    "Instanceable -> off" toggle does) makes the subtree real and editable.
    Cost: the copies stop sharing a prototype, so memory/draw-call sharing is
    lost -- fine for a handful of props, avoid for thousands.

    WARNING (observed on Isaac Sim 6.0 / RTX): de-instancing a subtree that the
    renderer has *already* uploaded makes Hydra tear the instancer down mid-
    frame, and that tore the GPU down with it here (VK_ERROR_DEVICE_LOST +
    pagefault).  Recolour right after you build/reference the asset, before the
    viewport has settled, rather than on a scene that has been rendering for a
    while.

    Returns the number of prims de-instanced.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return 0
    n = 0
    # de-instancing can expose *nested* instanceable prims, so loop to fixpoint
    for _ in range(16):
        hits = []
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if prim.IsInstanceable() and not prim.IsInstanceProxy():
                hits.append(prim)
        if not hits:
            break
        for prim in hits:
            prim.SetInstanceable(False)
            n += 1
    return n


# --- the main event ---------------------------------------------------------
def recolour(stage, root_path, colour, match=None, metallic=None, roughness=None,
             display_color=False):
    """Recolour every material used by the subtree at ``root_path``.

    stage      : Usd.Stage (live, e.g. omni.usd.get_context().get_stage())
    root_path  : str/Sdf.Path, e.g. "/World/Conveyor"
    colour     : (r, g, b) linear floats 0..1
    match      : optional case-insensitive regex/substring tested against the
                 MATERIAL PRIM NAME.  None -> every material found.
    metallic   : optional float, also written (and its texture neutralised)
    roughness  : optional float, likewise
    display_color : also stamp primvars:displayColor on every gprim.  OFF by
                 default: it is only useful in non-RTX draw modes and writing it
                 into a live Kit stage makes Fabric grumble about missing
                 displayColor:indices.

    Returns the number of materials actually changed.

    What it does per material, for BOTH shader flavours:
      * UsdPreviewSurface : disconnect inputs:diffuseColor (a connected texture
        beats the constant, so this is mandatory) then set the colour.
      * MDL / OmniPBR     : blank inputs:diffuse_texture (and friends) so
        tex::texture_isvalid() goes false and the constant is used, then set
        inputs:diffuse_color_constant.

    Instancing: prims behind an ``instanceable`` prim are not editable and
    plain stage.Traverse() does not even see them.  This function traverses
    with Usd.TraverseInstanceProxies() AND calls de_instance() first, which is
    the approach that actually works -- see de_instance.__doc__.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise ValueError(f"recolour: no prim at {root_path}")

    # 1. make the subtree editable (no-op if nothing is instanced)
    de_instance(stage, root_path)

    wanted = _compile(match)
    col = Gf_Vec3f(colour)

    # 2. collect materials: every Material prim in the subtree, plus anything
    #    bound from outside it (assets often keep /World/Looks separate).
    materials, seen = [], set()

    def consider(prim):
        if not prim or not prim.IsValid() or prim.GetPath() in seen:
            return
        if not prim.IsA(UsdShade.Material):
            return
        seen.add(prim.GetPath())
        materials.append(UsdShade.Material(prim))

    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        consider(prim)
        if prim.IsA(UsdGeom.Imageable) and prim.HasAPI(UsdShade.MaterialBindingAPI):
            try:
                bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
                if bound:
                    consider(bound.GetPrim())
            except Exception:
                pass

    # 3. edit
    changed = 0
    for material in materials:
        mprim = material.GetPrim()
        if not wanted(mprim.GetName()):
            continue
        if mprim.IsInstanceProxy():
            # de_instance() should have prevented this; if it still happens the
            # subtree is instanced from a layer we cannot edit -> skip loudly.
            _warn(f"recolour: {mprim.GetPath()} is an instance proxy, not editable -- skipped")
            continue

        touched = False
        for shader in _shaders_of(material):
            sprim = shader.GetPrim()
            if _is_mdl(sprim):
                killed = False
                for tex in _MDL_DIFFUSE_TEXTURES:
                    killed |= _kill_texture(shader, tex)
                for name in _MDL_DIFFUSE_INPUTS:
                    # write the primary one always; the others only if present
                    if name == _MDL_DIFFUSE_INPUTS[0] or shader.GetInput(name):
                        touched |= _set_input(shader, name, col, Sdf.ValueTypeNames.Color3f)
                for name in _MDL_NEUTRAL_WHITE_INPUTS:
                    if shader.GetInput(name):
                        _set_input(shader, name, Gf_Vec3f((1, 1, 1)), Sdf.ValueTypeNames.Color3f)
                if killed:
                    # Belt and braces: if some renderer still resolves the old
                    # texture, desaturation=1 at least guarantees the original
                    # hue (magenta, here) cannot survive.  It is a no-op once
                    # the texture really is invalid -- see the MDL snippet above.
                    _set_input(shader, "albedo_desaturation", 1.0, Sdf.ValueTypeNames.Float)
                if metallic is not None:
                    for tex in _MDL_METALLIC_TEXTURES:
                        _kill_texture(shader, tex)
                    _set_input(shader, _MDL_METALLIC_INPUTS[0], float(metallic), Sdf.ValueTypeNames.Float)
                if roughness is not None:
                    for tex in _MDL_ROUGHNESS_TEXTURES:
                        _kill_texture(shader, tex)
                    _set_input(shader, _MDL_ROUGHNESS_INPUTS[0], float(roughness), Sdf.ValueTypeNames.Float)
            else:
                sid = shader.GetShaderId() or ""
                is_preview = sid in _PREVIEW_IDS or bool(shader.GetInput("diffuseColor"))
                if not is_preview and not shader.GetInput("diffuseColor"):
                    continue
                touched |= _set_input(shader, "diffuseColor", col, Sdf.ValueTypeNames.Color3f)
                if metallic is not None:
                    _set_input(shader, "metallic", float(metallic), Sdf.ValueTypeNames.Float)
                if roughness is not None:
                    _set_input(shader, "roughness", float(roughness), Sdf.ValueTypeNames.Float)

        if touched:
            changed += 1

    # 4. optional displayColor stamp (see docstring -- off by default)
    if display_color:
        for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if prim.IsInstanceProxy() or not prim.IsA(UsdGeom.Gprim):
                continue
            try:
                UsdGeom.Gprim(prim).CreateDisplayColorAttr().Set([col])
            except Exception:
                pass

    return changed


# --- inspection helper (handy when an asset does not recolour) ---------------
def describe_materials(stage, root_path):
    """Print every material/shader under root_path and its diffuse-ish inputs."""
    root = stage.GetPrimAtPath(root_path)
    if not root:
        print(f"no prim at {root_path}")
        return
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not prim.IsA(UsdShade.Material):
            continue
        print(f"MATERIAL {prim.GetPath()}  proxy={prim.IsInstanceProxy()}")
        for shader in _shaders_of(UsdShade.Material(prim)):
            sp = shader.GetPrim()
            src = sp.GetAttribute("info:mdl:sourceAsset")
            print(f"  SHADER {sp.GetPath()}  mdl={_is_mdl(sp)}  "
                  f"src={src.Get() if src else shader.GetShaderId()}")
            for inp in shader.GetInputs():
                print(f"    {inp.GetFullName():40s} = {inp.Get()!r}"
                      f"{'  <-CONNECTED' if inp.HasConnectedSource() else ''}")


# --- tiny shims so this file works inside and outside Kit -------------------
def Gf_Vec3f(colour):
    from pxr import Gf
    if isinstance(colour, Gf.Vec3f):
        return colour
    r, g, b = colour
    return Gf.Vec3f(float(r), float(g), float(b))


def _warn(msg):
    try:
        import carb
        carb.log_warn(msg)
    except Exception:
        print(msg)
