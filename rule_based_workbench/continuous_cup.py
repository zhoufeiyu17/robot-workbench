"""Isolated continuous cup asset and native-profile adapter; never edits the base."""
from pathlib import Path
import hashlib
import json
import math
import os
import shutil

CUP = 'objects/anyverse_lidded_paper_cup_9cm_13cm/v1'
SECTORS = 32
SOFT_SHAPES = ('MiddleBody',)


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def body_bands(radii):
    if len(radii) != 3 or any(not math.isfinite(r) or r <= 0 for r in radii):
        raise ValueError('three measured positive body radii required')
    # One body collider avoids internal cap manifolds at the old 35/70mm cuts.
    # Middle/upper-body interaction labels are derived from contact height.
    rings = [(0., .0347), (.01, .0347), (.1, radii[2])]
    names = ('LowerBody', 'MiddleBody')
    return [dict(shape_path='/Cup/Collisions/'+name, z_min_m=z0, z_max_m=z1,
                 radius_m=max(r0,r1), radius_start_m=r0, radius_end_m=r1)
            for name, ((z0,r0),(z1,r1)) in zip(names,zip(rings,rings[1:]))]


def measured_radii(stage):
    import numpy as np
    from pxr import Usd, UsdGeom, Gf
    cache = UsdGeom.XformCache()
    points = []
    for prim in Usd.PrimRange(stage.GetPrimAtPath('/Cup/Visual')):
        if prim.IsA(UsdGeom.Mesh):
            transform = cache.GetLocalToWorldTransform(prim)
            points.extend(tuple(transform.Transform(Gf.Vec3d(*v)))
                          for v in UsdGeom.Mesh(prim).GetPointsAttr().Get())
    points = np.asarray(points)
    if len(points) < 100 or not np.isfinite(points).all():
        raise ValueError('source visual mesh is missing or nonfinite')
    radius = np.linalg.norm(points[:,:2],axis=1)
    radii = []
    for z in (.035,.07,.1):
        selected = np.abs(points[:,2]-z) <= .0026
        if not selected.any():
            raise ValueError('source mesh has no body section at '+str(z))
        radii.append(float(radius[selected].max()))
    return radii, dict(vertices=len(points), xyz_min_m=points.min(0).tolist(),
                      xyz_max_m=points.max(0).tolist(), body_sample_z_m=[.035,.07,.1],
                      body_outer_radius_m=radii, radial_sampling_half_window_m=.0026,
                      axis='unchanged source Cup Z axis', bottom_radius_preserved_m=.0347)


def copy_immutable(source, destination, omitted=()):
    source,destination=Path(source),Path(destination)
    def ignore(folder,names):
        relative=Path(folder).relative_to(source)
        return [name for name in names if name=='__pycache__' or str(relative/name) in omitted]
    # Files remain immutable hard links. Every changed file is omitted or replaced
    # atomically in this newly owned tree; no linked source file is edited.
    shutil.copytree(source,destination,copy_function=os.link,ignore=ignore)


def prepare_native(source,destination):
    copy_immutable(source,destination)
    root=Path(destination)/'source/robot_sim/robot_sim'
    cup=root/'asset_cfg/objects/cup.py'
    replacement=Path(__file__).with_name('native_overrides')/'cup.py'
    temporary=cup.with_suffix('.new');temporary.write_bytes(replacement.read_bytes());temporary.replace(cup)
    changes={
        root/'tasks/manager_based/coffee_make/relative_geometry.py': [
            ('self.cup_bands = [(b, self.fcl.Cylinder(b.radius_m, b.z_max_m - b.z_min_m)) for b in cup.bands]',
             'self.cup_bands = [(b, self._cup_shape(b)) for b in cup.bands]'),
            ('    def audit(', '''    def _cup_shape(self, band):
        if band.radius_start_m is None:
            return self.fcl.Cylinder(band.radius_m, band.z_max_m-band.z_min_m)
        vertices,faces=band.mesh()
        import numpy as np
        polygons=np.asarray([v for face in faces for v in (3,*face)],dtype=np.int32)
        return self.fcl.Convex(np.asarray(vertices,dtype=float),len(faces),polygons)

    def audit(''')],
        root/'tasks/manager_based/coffee_make/interaction_evaluation.py': [
            ('min(b.z_min_m * axis[2], b.z_max_m * axis[2]) - b.radius_m * radial_projection for b in self.cup.bands',
             'min(b.z_min_m*axis[2]-b.radius_at(b.z_min_m)*radial_projection, b.z_max_m*axis[2]-b.radius_at(b.z_max_m)*radial_projection) for b in self.cup.bands'),
            ('or abs(normal[2]) > math.sin(a.tilt_rad)',
             'or abs(normal[2]) > math.sin(self.cup.sidewall_normal_limit_rad(local[2]) if any(b.radius_start_m is not None for b in self.cup.bands) else a.tilt_rad)')]
    }
    identities={str(cup.relative_to(destination)):sha(cup)}
    for path,edits in changes.items():
        text=path.read_text()
        for old,new in edits:
            if text.count(old)!=1:
                raise ValueError('native adapter context differs: '+str(path)+' / '+old[:50])
            text=text.replace(old,new)
        temporary=path.with_suffix('.new');temporary.write_text(text);temporary.replace(path)
        identities[str(path.relative_to(destination))]=sha(path)
    return identities


def prepare_source_wrapped_fingers(source,destination):
    """Use native PhysX shrink-wrap; retain CAD, pads, gains and all other defaults."""
    copy_immutable(source,destination)
    path=Path(destination)/'source/robot_sim/robot_sim/asset_cfg/robots/ac_one_finger_contact.py'
    text=path.read_text()
    edits=[('    pads: tuple[ACOneFingerPadSpec, ...]\n',
            '    pads: tuple[ACOneFingerPadSpec, ...]\n    body_shrink_wrap: bool = True\n'),
           ('            mesh_api.GetApproximationAttr().Set(specification.body_collision_approximation)\n',
            '            mesh_api.GetApproximationAttr().Set(specification.body_collision_approximation)\n'
            '            from pxr import PhysxSchema\n'
            '            PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim).CreateShrinkWrapAttr(specification.body_shrink_wrap)\n')]
    for old,new in edits:
        if text.count(old)!=1:raise ValueError('finger cooking adapter source context differs')
        text=text.replace(old,new)
    compile(text,str(path),'exec')
    temporary=path.with_suffix('.new');temporary.write_text(text);temporary.replace(path)
    return dict(native_path=str(destination),source_native_path=str(source),body_shrink_wrap=True,
                default_resolution_and_hull_budget_unchanged=True,pads_and_drives_unchanged=True,
                native_override_sha256={str(path.relative_to(destination)):sha(path)})


def prepare_assets(source,destination):
    import yaml
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
    source,destination=Path(source),Path(destination)
    excluded=tuple(CUP+'/'+name for name in ('asset.yaml','sim/cup.usda','interaction/coffee_manipulation.yaml'))
    original=Usd.Stage.Open(str(source/CUP/'sim/cup.usda'))
    radii,visual=measured_radii(original)
    copy_immutable(source,destination,excluded)
    stage=Usd.Stage.CreateNew(str(destination/CUP/'sim/cup.usda'))
    UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    root=UsdGeom.Xform.Define(stage,'/Cup').GetPrim();stage.SetDefaultPrim(root)
    UsdPhysics.RigidBodyAPI.Apply(root).CreateRigidBodyEnabledAttr(True)
    UsdPhysics.MassAPI.Apply(root).CreateMassAttr(.02)
    Sdf.CopySpec(original.GetRootLayer(),'/Cup/Visual',stage.GetRootLayer(),'/Cup/Visual')
    materials={}
    for name in ('Hard','SoftBody'):
        material=UsdShade.Material.Define(stage,'/Cup/PhysicsMaterials/'+name)
        api=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(.6);api.CreateDynamicFrictionAttr(.5);api.CreateRestitutionAttr(.05)
        materials[name]=material
    UsdGeom.Xform.Define(stage,'/Cup/Collisions')
    # Runtime profile class is already available when the adapter is invoked.
    from robot_sim.asset_cfg.objects.cup import CupBand
    bands=body_bands(radii)+[dict(shape_path='/Cup/Collisions/'+name,z_min_m=lo,z_max_m=hi,radius_m=r)
                           for name,lo,hi,r in (('Rim',.1,.115,.045),('LidTop',.115,.13,.0395))]
    for row in bands:
        band=CupBand(**row);prim_path=band.shape_path
        if band.radius_start_m is None:
            mesh=UsdGeom.Cylinder.Define(stage,prim_path);mesh.CreateAxisAttr('Z')
            mesh.CreateRadiusAttr(band.radius_m);mesh.CreateHeightAttr(band.z_max_m-band.z_min_m)
        else:
            vertices,faces=band.mesh(SECTORS);mesh=UsdGeom.Mesh.Define(stage,prim_path)
            mesh.CreatePointsAttr(vertices);mesh.CreateFaceVertexCountsAttr([3]*len(faces))
            mesh.CreateFaceVertexIndicesAttr([v for f in faces for v in f]);mesh.CreateSubdivisionSchemeAttr('none')
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        mesh.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0,0,(band.z_min_m+band.z_max_m)/2))
        mesh.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        name=prim_path.rsplit('/',1)[-1];material=materials['SoftBody' if name in SOFT_SHAPES else 'Hard']
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
    stage.GetRootLayer().Save()
    manifest=yaml.safe_load((source/CUP/'asset.yaml').read_text())
    profile=yaml.safe_load((source/CUP/'interaction/coffee_manipulation.yaml').read_text())
    profile.update(schema_version=2,bands=bands,provenance='continuous outer body from unchanged source visual mesh; original hard bottom/rim/lid retained')
    profile['dependencies'][CUP+'/sim/cup.usda']=sha(destination/CUP/'sim/cup.usda')
    (destination/CUP/'interaction/coffee_manipulation.yaml').write_text(yaml.safe_dump(profile,sort_keys=False))
    manifest['integrity']['entrypoint_sha256']=profile['dependencies'][CUP+'/sim/cup.usda']
    manifest['integrity']['interaction_profile_sha256']=sha(destination/CUP/'interaction/coffee_manipulation.yaml')
    manifest['simulation']['collision_proxy']='continuous_body_convex_frusta_hard_bottom_10mm'
    (destination/CUP/'asset.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False))
    return dict(source_visual_sha256=sha(source/CUP/'source/cup_visual.usdc'),visual=visual,bands=bands,
                profile_sha256=manifest['integrity']['interaction_profile_sha256'],
                hard_bottom_height_m=.01,soft_shapes=SOFT_SHAPES,initial_contact='hard reference; preset changes only SoftBody gains',
                sidewall_normal_tolerance_rad=.02,sectors=SECTORS,
                contact_height_semantics=dict(MiddleBody=[.035,.07],UpperBody=[.07,.1]),
                sampled_body_radial_overhang_m=[(.0347+(radii[2]-.0347)*(z-.01)/.09)-r
                                                for z,r in zip((.035,.07,.1),radii)])
