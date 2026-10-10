"""Force-based cup contact presets and fixed-time jaw measurements."""
import math


def bind_preset_identity(identity,preset):
    """Freeze the simulation-only diagnostic material alongside the base physics."""
    import hashlib,json
    if (set(preset)!={'preset_id','stiffness_n_m','damping_n_s_m','diagnostic_claw_compression'}
            or preset['preset_id'] not in ('hard','firm','medium','soft')
            or preset['diagnostic_claw_compression'] is not True
            or not all(math.isfinite(preset[k]) for k in ('stiffness_n_m','damping_n_s_m'))
            or not 0<=preset['stiffness_n_m']<=200000 or not 0<=preset['damping_n_s_m']<=1000):
        raise ValueError('bounded explicit simulation diagnostic preset required')
    identity['cup_contact_preset']=dict(preset)
    return hashlib.sha256(json.dumps(identity,sort_keys=True,allow_nan=False).encode()).hexdigest()


def bind_plan_preset(plan,preset):
    """Existing publish binding, shared by native scene-derived base/retimed plans."""
    identity=plan['native_identity']
    identity['physical_configuration_sha256']=bind_preset_identity(identity['physical_configuration'],preset)
    plan['cup_contact_preset']=dict(preset)
    plan['preset_qualification']='simulation diagnostic candidate; default not selected'


def configure_preset(runtime,stage,preset):
    """Publish the declared identity only after applying its actual material."""
    identity=dict(runtime.physical_identity);digest=bind_preset_identity(identity,preset)
    material=configure(stage,runtime.root,preset['stiffness_n_m'],preset['damping_n_s_m'])
    runtime.physical_identity.update(identity);runtime.physical_sha256=digest
    return material


def stiffness_seeds(reference_force_n):
    if not math.isfinite(reference_force_n) or not .5 <= reference_force_n <= 100.:
        raise ValueError('valid bilateral hard-reference force required')
    return [dict(preset_id=name,contact_stiffness_n_m=reference_force_n/depth,
                 exploratory_deflection_m=depth,reference_force_n=reference_force_n)
            for name,depth in (('firm',.0005),('medium',.0015),('soft',.003))]


def configure(stage,root,stiffness_n_m,damping_n_s_m):
    """Only the named body material changes; force spring, never acceleration."""
    from pxr import UsdShade,PhysxSchema
    if (not all(math.isfinite(v) for v in (stiffness_n_m,damping_n_s_m))
            or not 0 <= stiffness_n_m <= 200000 or not 0 <= damping_n_s_m <= 1000):
        raise ValueError('bounded finite force-spring gains required')
    cup=stage.GetPrimAtPath(root+'/Cup')
    bindings={}
    expected={'LowerBody':'Hard','MiddleBody':'SoftBody','Rim':'Hard','LidTop':'Hard'}
    for name,target in expected.items():
        prim=stage.GetPrimAtPath(root+'/Cup/Collisions/'+name)
        material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
        if material.GetPath().name!=target:
            raise ValueError('continuous cup material binding differs: '+name)
        bindings[name]=str(material.GetPath())
    material=stage.GetPrimAtPath(root+'/Cup/PhysicsMaterials/SoftBody')
    api=PhysxSchema.PhysxMaterialAPI.Apply(material)
    api.CreateCompliantContactStiffnessAttr(stiffness_n_m)
    api.CreateCompliantContactDampingAttr(damping_n_s_m if stiffness_n_m else 0.)
    api.CreateCompliantContactAccelerationSpringAttr(False)
    for path in (root+'/Cup/PhysicsMaterials/Hard','/World/PhysicsMaterials/G2RubberFingerPad'):
        prim=stage.GetPrimAtPath(path)
        if not prim.IsValid():raise ValueError('hard reference material missing: '+path)
        value=prim.GetAttribute('physxMaterial:compliantContactStiffness').Get()
        if value not in (None,0.):raise ValueError('both contact sides must not be compliant')
    return dict(schema_version='cup_force_contact_preset.v1',stiffness_n_m=stiffness_n_m,
                damping_n_s_m=damping_n_s_m if stiffness_n_m else 0.,acceleration_spring=False,
                soft_body_z_m=[.01,.1],shape_bindings=bindings,
                scope='cup body contacts including metal and machine; diagnostic grip compression observed, external3mm guard retained',
                geometry_deformation=False,active_physics_verification_required=True)


def contact_forces(contacts,root,dt):
    if not math.isfinite(dt) or dt<=0:raise ValueError('positive contact timestep required')
    forces={side:dict(pad_n=0.,metal_n=0.,max_pad_penetration_m=0.,max_metal_penetration_m=0.,max_other_metal_penetration_m=0.) for side in ('left_link7','left_link8')}
    for contact in contacts:
        if root+'/Cup' not in contact.actors:continue
        ci=contact.actors.index(root+'/Cup');fi=1-ci;name=contact.actors[fi].rsplit('/',1)[-1]
        if name not in forces:continue
        value=math.sqrt(sum(v*v for v in contact.impulse))/dt
        if contact.shapes[fi]==contact.actors[fi]+'/RealFingerRubberPad':
            forces[name]['pad_n']+=value
            forces[name]['max_pad_penetration_m']=max(forces[name]['max_pad_penetration_m'],max(0.,-contact.separation_m))
        elif contact.shapes[fi].startswith(contact.actors[fi]+'/collisions/'):
            forces[name]['metal_n']+=value
            forces[name]['max_metal_penetration_m']=max(forces[name]['max_metal_penetration_m'],max(0.,-contact.separation_m))
            if contact.shapes[ci] not in {root+'/Cup/Collisions/MiddleBody',root+'/Cup/Collisions/UpperBody'}:
                forces[name]['max_other_metal_penetration_m']=max(forces[name]['max_other_metal_penetration_m'],max(0.,-contact.separation_m))
    return forces


def diagnostic_summary(rows):
    """Include failed attempts; never turn observed compression into acceptance."""
    phases={r['phase'] for r in rows}
    return dict(maximum_cup_tilt_deg=max((math.degrees(r['cup_tilt_rad']) for r in rows),default=None),
                maximum_tilt_by_phase_deg={p:max(math.degrees(r['cup_tilt_rad']) for r in rows if r['phase']==p) for p in sorted(phases)},
                maximum_observed_pad_compression_m=max((v['max_pad_penetration_m'] for r in rows for v in r['forces'].values()),default=0.),
                maximum_observed_metal_compression_m=max((v['max_metal_penetration_m'] for r in rows for v in r['forces'].values()),default=0.),
                peak_metal_force_n=max((v['metal_n'] for r in rows for v in r['forces'].values()),default=0.))


def gap_summary(rows,*,metal_offset_mm=.008):
    """0.6s hold endpoint, last0.2s; no invented longer settling period."""
    import numpy as np
    if not rows:raise ValueError('measured hold samples required')
    end=rows[-1]['hold_time_s']
    if abs(end-.6)>1/240+1e-9:raise ValueError('0.6s hold endpoint required')
    selected=[r for r in rows if r['hold_time_s']>=end-.2-1e-9]
    if len(selected)<6 or selected[-1]['hold_time_s']-selected[0]['hold_time_s']<.19:
        raise ValueError('complete last0.2s observation window required')
    times=np.array([r['hold_time_s'] for r in selected]);metal=np.array([sum(r['jaw_positions_m'])*1000+metal_offset_mm for r in selected])
    if not np.isfinite(metal).all():raise ValueError('nonfinite jaw sample')
    slope=float(np.polyfit(times-times[0],metal,1)[0]);span=float(np.ptp(metal))
    return dict(endpoint_hold_time_s=end,samples=len(selected),metal_gap_mean_mm=float(metal.mean()),
                nominal_rubber_gap_mean_mm=float(metal.mean()-3),metal_gap_range_mm=[float(metal.min()),float(metal.max())],
                nominal_rubber_gap_range_mm=[float(metal.min()-3),float(metal.max()-3)],
                gap_peak_to_peak_mm=span,gap_slope_mm_s=slope,gap_stable=span<=.5 and abs(slope)<=.5,
                physical_grasp_success_not_implied=True,reference='same metal surfaces along opening axis; rubber gap is nominal metal minus3mm')
