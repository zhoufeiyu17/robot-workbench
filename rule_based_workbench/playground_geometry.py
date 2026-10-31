"""Plan against exported native geometry without starting Kit or a renderer."""
import argparse
import hashlib
import json
from real_contract import is_device_source
from pathlib import Path
import sys
import time
import types

from integration import _digest
from playground_pipeline import save


def configure_native(native_code):
    # The native package __init__ registers Isaac environments. Numerical modules
    # need only their package paths; no simulator, CUDA or robot SDK is imported.
    root=Path(native_code)/'source/robot_sim/robot_sim'
    packages=('', 'asset_cfg', 'asset_cfg.objects', 'asset_cfg.robots', 'specs', 'policies',
              'tasks', 'tasks.manager_based', 'tasks.manager_based.coffee_make')
    for suffix in packages:
        name='robot_sim'+('.'+suffix if suffix else '')
        module=types.ModuleType(name);module.__path__=[str(root.joinpath(*suffix.split('.'))) if suffix else str(root)]
        sys.modules[name]=module


def load(path, source, native_code, assets, *, maintenance_source=None):
    from maintenance_geometry import validate_export
    data=validate_export(path,source,maintenance_source)
    if source['scene'].get('asset_set') is not None:
        from scene_assets import load as load_scene_assets, check_geometry
        selection=load_scene_assets(assets,expected_digest=source['scene']['asset_set']['asset_set_digest'])
        if selection!=source['scene']['asset_set']:
            raise ValueError('exported planning scene differs from prepared asset selection')
        check_geometry(selection,data['observation']['scene']['profile_sha256'],data['cup']['profile_sha256'])
    configure_native(native_code)
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.asset_cfg.objects.cup import CupBand, CupInteractionProfile
    from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
    from robot_sim.tasks.manager_based.coffee_make.scene_instance import (
        MachineInstanceSpec,ResolvedPrimitive,ResolvedSite,ResolvedButton,ResolvedMachineScene)
    from robot_sim.tasks.manager_based.coffee_make.interaction_evaluation import PhysicalSample
    from robot_sim.tasks.manager_based.coffee_make.episode_planner import EpisodePlanner
    from robot_sim.tasks.manager_based.coffee_make.local_geometry import HandPart
    from robot_sim.tasks.manager_based.coffee_make.m1_robot_geometry import M1RobotKinematics
    from robot_sim.tasks.manager_based.coffee_make.a1_support_contact import load_support_contact
    from robot_sim.policies.rule_coffee import RuleObservation
    from real_contract import validate_assisted_cup_geometry
    validate_assisted_cup_geometry(source,data['cup'])
    urdf=ac_one_asset_spec(assets).urdf_path
    if data['source_digest']!=_digest(source) or data['urdf_sha256']!=hashlib.sha256(urdf.read_bytes()).hexdigest():
        raise ValueError('exported geometry source or robot differs')
    def transforms(value):
        return {k:RigidTransform(**v) if isinstance(v,dict) and 'orientation_wxyz' in v else v for k,v in value.items()}
    def primitive(value):return ResolvedPrimitive(**transforms(value))
    scene=transforms(data['observation']['scene'])
    scene.update(instance=MachineInstanceSpec(**transforms(scene['instance'])),
        shapes=tuple(map(primitive,scene['shapes'])),sites=tuple(ResolvedSite(**v) for v in scene['sites']),
        button_contacts=tuple(ResolvedButton(**v) for v in scene['button_contacts']))
    observation=RuleObservation(**{**data['observation'],'scene':ResolvedMachineScene(**scene),
        'sample':PhysicalSample(**transforms(data['observation']['sample']))})
    planner=EpisodePlanner.__new__(EpisodePlanner)
    planner.gripper_mode='distance'
    planner.open_position_m=data['open_position_m'];planner.parts=tuple(HandPart(**v) for v in data['parts'])
    planner.cup=CupInteractionProfile(**{**data['cup'],'bands':tuple(CupBand(**b) for b in data['cup']['bands'])})
    if source['scene'].get('asset_set') is not None:
        from scene_assets import derive_grasp_command
        from gripper_distance_plan import requested_metal_gap_mm
        from robot_sim.asset_cfg.robots.ac_one_finger_contact import ac_one_finger_contact_spec
        planner.scene_asset_set=source['scene']['asset_set']
        expected=derive_grasp_command(planner.cup,pad_thickness_m=ac_one_finger_contact_spec().rubber_thickness_m)
        if expected!=source.get('grasp_command'):
            raise ValueError('selected grasp seed differs from the loaded cup/pad geometry')
        planner.requested_metal_gap_mm=requested_metal_gap_mm(source)
    if source.get('adapter')=='operator_assisted_rgbd':
        from gripper_distance_plan import requested_metal_gap_mm
        planner.operator_assisted_cup_geometry=source['cup_geometry']
        planner.operator_assisted_cup_envelope=Path(assets)/'operator-assisted-cup-envelope.json'
        planner.requested_metal_gap_mm=requested_metal_gap_mm(source)
    planner.table=primitive(data['table']);planner.obstacles=tuple(map(primitive,data['obstacles']))
    planner.fk=M1RobotKinematics(urdf);planner.support_contact=load_support_contact(urdf)
    planner.identity=data['planner_identity'];planner.geo=None
    if planner.support_contact.identity!=planner.identity['support_contact']:
        raise ValueError('support contact policy differs from exported geometry')
    if data['native_identity']['planner_identity']!=planner.identity or not planner.identity.get('chassis_collision'):
        raise ValueError('native planner identity or chassis coverage differs')
    planner.contact_tracking_reserve_m=planner.CUP_GRASP_RESERVE_M
    planner.outlet_tolerance_m=None;planner.hover_clearances_m=(.005,)
    from robot_sim.tasks.manager_based.coffee_make.initial_robot_state import joint_positions
    if is_device_source(source):
        initial_model=source['initial_state']['model']
    elif source.get('schema_version')=='dual_gpu_pipeline.v1' and source.get('lane')=='simulated-real':
        initial_model=data['simulation_initial_state']
    else:
        raise ValueError('recognized physical or simulated source required for geometry loading')
    planner.home_state=joint_positions(initial_model)
    measured=initial_model['left_arm_joints']+initial_model['right_arm_joints']
    if max(abs(a-b) for a,b in zip(observation.sample.arm_positions,measured))>.005:
        raise ValueError('exported planning start differs from measured model state by over 0.005 rad')
    for side,start,names in (('left',0,tuple(f'left_joint{i}' for i in range(1,7))),
                             ('right',6,tuple(f'right_joint1{i}' for i in range(1,7)))):
        planner.home_state.update(zip(names,observation.sample.arm_positions[start:start+6]))
    planner.geometry(observation)
    return planner,observation,data


def main(args):
    from local_execution import register_auxiliary
    register_auxiliary(args,__file__)
    started=time.monotonic();args.output.mkdir(parents=True,exist_ok=False)
    requested_backend=getattr(args,'backend','native')
    backend='curobo_world_broadphase_native_exact' if requested_backend=='curobo' else 'native'
    try:
        source=json.loads(args.source.read_text())
        from playground_plan import build, retime
        from curobo_evidence import backend_identity, stamp_plan
        # The existing cached-route entry also reuses its checked command path.
        # This avoids loading meshes or repeating any audit just to change speed.
        base_path=args.initial.with_name('base-plan.json') if args.initial else None
        cached=bool(base_path and base_path.is_file())
        if cached:
            base=json.loads(base_path.read_text())
            identity=base.get('planning_backend',base.get('planning_validation',{}).get('backend',dict(backend='native',gpu_ik=False)))
            backend=identity['backend']
            ready=route_ready=time.monotonic()
            if requested_backend=='motiongen':configure_native(args.native_code)
        else:
            if args.initial and requested_backend=='curobo':
                raise ValueError('cached base-plan.json required for CPU-only speed adjustment; rebuild the scene to plan again')
            planner,obs,data=load(args.input,source,args.native_code,args.assets)
            if requested_backend=='curobo':
                from curobo_collision import CuroboWorldBroadphase
                from curobo_whole_geometry import CuroboWholeGeometry
                from curobo_relative_geometry import install, initial as accelerated_initial
                engine=CuroboWorldBroadphase(data)
                if engine.object_ids!=tuple(row[0] for row in planner.geo.whole.world):
                    raise ValueError('GPU/native world ordering differs')
                def callback(spheres,clearance):
                    return engine.query(spheres[None,None],clearance=clearance)[0,0].cpu().numpy()
                planner.geo.whole=CuroboWholeGeometry(planner.geo.whole,callback)
                install(planner.geo)
            identity=backend_identity(requested_backend,relative_geometry=requested_backend=='curobo')
            ready=time.monotonic()
            if args.initial:
                initial=json.loads(args.initial.read_text())
                if initial.get('planning_input_sha256')!=hashlib.sha256(args.input.read_bytes()).hexdigest():
                    raise ValueError('cached route belongs to another native geometry export')
            else:
                solve = accelerated_initial if requested_backend=='curobo' else lambda p,o,e: p.initial(o,e)
                initial=solve(planner,obs,data['native_identity']['acceptance']['edge_margin_m'])
                initial['planning_input_sha256']=hashlib.sha256(args.input.read_bytes()).hexdigest()
            save(args.output/'initial-plan.json',initial)
            if initial.get('passed') is not True:raise ValueError('no collision-free complete route; see initial-plan.json')
            route_ready=time.monotonic()
            base=build(planner,obs,initial,source,data['native_identity'],speed_scale=None,
                **({'environment':'fake'} if getattr(args,'environment','physical')=='fake' else {}))
            base=stamp_plan(base,identity)
        save(args.output/'base-plan.json',base)
        checked=time.monotonic()
        plan=retime(base,source,args.speed_scale,**({'environment':'fake'} if getattr(args,'environment','physical')=='fake' else {}))
        if getattr(args,'execution_json',None):
            from local_execution import validate,compute_resource,stamp_plan
            execution=validate({'execution':json.loads(args.execution_json)})
            stamp_plan(plan,compute_resource(execution))
        from frozen_executor import validate_plan_evidence
        validate_plan_evidence(plan,source)
        save(args.output/'plan.json',plan)
        save(args.output/'result.json',dict(status='geometric-plan-ready',timeline_digest=plan['timeline_digest'],
            backend=backend,backend_identity=identity,gpu_ik=False,cached_retime=cached,
            requested_backend=requested_backend,operation='cached_retime' if cached else 'plan',
            gpu_used=requested_backend=='curobo' and not cached,
            simulation=plan['simulation'],timings_seconds=dict(load=ready-started,route=route_ready-ready,
            path_build_and_audit=0. if cached else checked-route_ready,speed_interpolation=time.monotonic()-checked,
            total=time.monotonic()-started)))
    except Exception as error:
        save(args.output/'result.json',dict(status='failed',error=str(error),backend=backend,gpu_ik=False,elapsed_s=time.monotonic()-started))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('input','source','native-code','assets','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--initial',type=Path)
    parser.add_argument('--speed-scale',type=float,default=.3)
    parser.add_argument('--backend',choices=('native','curobo','motiongen'),default='native')
    parser.add_argument('--environment',choices=('physical','fake'),default='physical')
    parser.add_argument('--execution-json')
    main(parser.parse_args())
