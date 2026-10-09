"""Standard-inspired bilateral compression fixture; not material certification."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time


def seed_stiffness(force_n, total_mm=9.5):
    if not math.isfinite(force_n) or force_n <= 0 or not math.isfinite(total_mm) or total_mm <= 0:
        raise ValueError('positive force and total compression required')
    return 2000 * force_n / total_mm


def main(a):
    a.output.mkdir(parents=True, exist_ok=False)
    from isaaclab.app import AppLauncher
    app = AppLauncher(headless=True, enable_cameras=False, device='cuda:0').app
    result = dict(passed=False, hardware_access=False, material_certification=False, presets=[])
    started = time.perf_counter()
    try:
        import torch
        import isaaclab.sim as sim
        from isaaclab.assets import RigidObject, RigidObjectCfg
        from isaaclab.sensors import ContactSensor, ContactSensorCfg
        from pxr import UsdPhysics, UsdShade, PhysxSchema
        from continuous_cup import CUP
        from robot_sim.asset_cfg.objects.cup import load_cup_profile
        stage = sim.get_current_stage()
        context = sim.SimulationContext(sim.SimulationCfg(dt=1/240, device='cuda:0',
            physx=sim.PhysxCfg(solver_type=1)))
        sim.GroundPlaneCfg().func('/World/Ground', sim.GroundPlaneCfg())
        cup = RigidObject(RigidObjectCfg(prim_path='/World/Cup', spawn=sim.UsdFileCfg(
            usd_path=str(a.assets / CUP / 'sim/cup.usda'), activate_contact_sensors=True,
            rigid_props=sim.RigidBodyPropertiesCfg(solver_position_iteration_count=32,
                solver_velocity_iteration_count=8)), init_state=RigidObjectCfg.InitialStateCfg(pos=(0.,0.,0.))))
        profile = load_cup_profile(CUP, a.assets)
        # Exclude lid from the fixture height, retaining the existing cup asset.
        height = next(b.z_max_m for b in profile.bands if b.shape_path.endswith('/Rim'))
        z = height * 2/3
        radius = next(b.radius_at(z) for b in profile.bands if b.z_min_m <= z <= b.z_max_m)
        heads = []
        for name, sign in (('L', -1), ('R', 1)):
            heads.append(RigidObject(RigidObjectCfg(prim_path='/World/Head'+name,
                spawn=sim.SphereCfg(radius=.005, activate_contact_sensors=True,
                    rigid_props=sim.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                    collision_props=sim.CollisionPropertiesCfg(contact_offset=.0001, rest_offset=0.),
                    physics_material=sim.RigidBodyMaterialCfg(static_friction=0., dynamic_friction=0.)),
                init_state=RigidObjectCfg.InitialStateCfg(pos=(sign*(radius+.006),0.,z)))))
        sensor = ContactSensor(ContactSensorCfg(prim_path='/World/Head.*',
            filter_prim_paths_expr=['/World/Cup'], update_period=0.))
        material = stage.GetPrimAtPath('/World/Cup/PhysicsMaterials/SoftBody')
        api = PhysxSchema.PhysxMaterialAPI.Apply(material)
        api.CreateCompliantContactAccelerationSpringAttr(False)
        api.CreateCompliantContactDampingAttr(10.)
        context.reset()
        poses = [h.data.default_root_state[:, :7].clone() for h in heads]
        initial = cup.data.default_root_state.clone()
        def place(compression):
            for i, (h, sign) in enumerate(zip(heads, (-1,1))):
                poses[i][:,0] = sign*(radius+.005-compression/2)
                h.write_root_pose_to_sim(poses[i])
        def measure(k, tag):
            api.CreateCompliantContactStiffnessAttr(float(k))
            cup.write_root_state_to_sim(initial); cup.reset(); sensor.reset()
            place(-.002)
            for _ in range(120):
                context.step(render=False); cup.update(context.get_physics_dt())
            rows=[]
            # 50mm/min relative speed, exactly9.5mm total reduction.
            steps=2736
            for i in range(steps+1):
                displacement=.0095*i/steps
                place(displacement)
                context.step(render=False);cup.update(context.get_physics_dt())
                sensor.update(context.get_physics_dt(),force_recompute=True)
                forces=sensor.data.force_matrix_w[0,:,0,:]
                values=forces[:,0].abs().cpu().tolist()
                if len(values)!=2 or not all(math.isfinite(v) for v in values):
                    raise ValueError('two finite measured head forces required')
                if i%8==0 or i==steps:
                    rows.append(dict(total_compression_mm=displacement*1000,
                        side_force_n=values, cup_pose=cup.data.root_state_w[0,:7].cpu().tolist()))
            (a.output/(tag+'.json')).write_text(json.dumps(rows))
            # Sensor order is immaterial; both head forces must match the target.
            peak=[max(r['side_force_n'][j] for r in rows) for j in range(2)]
            value=sum(peak)/2
            if value<=0:raise ValueError('fixture has no measured bilateral force')
            return dict(stiffness_n_m=k, peak_side_force_n=peak, peak_mean_force_n=value,
                        endpoint_side_force_n=rows[-1]['side_force_n'], curve=tag+'.json')
        for name, force in (('soft',3.),('medium',5.),('firm',8.)):
            k=seed_stiffness(force); attempts=[]
            for iteration in range(4):
                measured=measure(k,f'{name}-{iteration}');attempts.append(measured)
                print(name, iteration, measured, flush=True)
                if all(abs(v-force)<=max(.15,.05*force) for v in measured['peak_side_force_n']):break
                k*=force/measured['peak_mean_force_n']
                if not 1<=k<=200000:raise ValueError('calibration exceeds material parameter bounds')
            passed=all(abs(v-force)<=max(.15,.05*force) for v in measured['peak_side_force_n'])
            result['presets'].append(dict(preset_id=name,target_peak_force_n=force,
                stiffness_n_m=measured['stiffness_n_m'],damping_n_s_m=10.,passed=passed,attempts=attempts))
        result.update(passed=all(p['passed'] for p in result['presets']),
            fixture=dict(total_compression_mm=9.5,relative_speed_mm_min=50.,head_radius_mm=5.,
                cup_height_without_lid_mm=height*1000,load_height_mm=z*1000,physics_hz=240,
                damping_n_s_m=10.,existing_rigid_cup_with_compliant_contacts=True,
                lid_geometry_retained=True,thin_shell_deformation=False),
            profile_sha256=profile.profile_sha256,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    except Exception as e:
        import traceback
        result.update(error=str(e),traceback=traceback.format_exc())
    finally:
        result['seconds']=time.perf_counter()-started
        (a.output/'result.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result),flush=True)
        if 'context' in locals():context.stop()
        app.close(wait_for_replicator=False)
    return 0 if result['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--assets',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    raise SystemExit(main(p.parse_args()))
