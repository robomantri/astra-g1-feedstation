"""Convert the supplier's STEP crate (600x400x120) into a USD with a blue material.

  ~/isaacsim6_venv/bin/python ~/Desktop/h12/scripts/make_crate_usd.py

STEP is read via trimesh + cascadio (OCCT). Kit's own CAD converter needs a GUI
confirm dialog, so it is not scriptable headless.
"""
import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

STEP = "/home/kasper/Desktop/h12/crate_src/PBT plastic crate 600 x 400 x120.stp"
OUT_DIR = "/home/kasper/Desktop/h12/assets/crates"
# one USD per colour variant
VARIANTS = {
    "euro_crate_600x400x120": (0.55, 0.55, 0.57),        # grey
    "euro_crate_600x400x120_red": (0.62, 0.09, 0.09),    # red
}

scene = trimesh.load(STEP)
mesh = scene.to_mesh() if hasattr(scene, "to_mesh") else scene
mesh.merge_vertices()

# Centre in X/Y so a translate value is the crate's centre, but keep the base at
# z=0 so a stack layer's z is simply the height below it.
lo, hi = mesh.bounds
mesh.apply_translation([-(lo[0] + hi[0]) / 2.0, -(lo[1] + hi[1]) / 2.0, -lo[2]])
size = mesh.extents
print(f"mesh: {len(mesh.vertices)} verts, {len(mesh.faces)} tris, size {size}")

for name, rgb in VARIANTS.items():
    out = f"{OUT_DIR}/{name}.usd"
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, Sdf.Path("/Crate"))
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")

    gprim = UsdGeom.Mesh.Define(stage, Sdf.Path("/Crate/geom"))
    gprim.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in mesh.vertices])
    gprim.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
    gprim.CreateFaceVertexIndicesAttr([int(i) for i in mesh.faces.reshape(-1)])
    gprim.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    gprim.CreateExtentAttr([Gf.Vec3f(*map(float, mesh.bounds[0])),
                            Gf.Vec3f(*map(float, mesh.bounds[1]))])
    # No normals authored on purpose. Hydra ignores the `interpolation` metadata on the
    # `normals` attribute and reads it as faceVarying, so vertex-count normals trip
    # "corrupted data in primvar 'normal'". With subdivisionScheme=none the renderer
    # generates face normals, which is the correct crisp look for a CAD part anyway
    # (trimesh's merged vertex normals would round off the sharp crate edges).

    mat = UsdShade.Material.Define(stage, Sdf.Path("/Crate/Looks/Crate"))
    shader = UsdShade.Shader.Define(stage, Sdf.Path("/Crate/Looks/Crate/surface"))
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.42)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(gprim.GetPrim()).Bind(mat)

    # physics: dynamic body with a convex-hull collider (the mesh is not watertight,
    # and convexDecomposition on 5.8k tris is slow for little gain on a box shape)
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(gprim.GetPrim()).CreateApproximationAttr().Set(
        UsdPhysics.Tokens.convexHull
    )
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr().Set(1.2)

    stage.GetRootLayer().Save()
    print("wrote", out, "rgb", rgb)

print(f"size = {size[0]:.4f} x {size[1]:.4f} x {size[2]:.4f} m, base at z=0, centred in x/y")
