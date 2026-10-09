"""Bounded real MotionGen integration smoke; no replay or executable artifact."""
import argparse,json,time,traceback
from pathlib import Path


def main(args):
    args.output.mkdir(exist_ok=False);began=time.perf_counter()
    result=dict(passed=False,hardware_access=False,executable_plan=False,scope='MotionGen integration smoke only')
    try:
        from playground_geometry import load
        planner,obs,data=load(args.input,json.loads(args.source.read_text()),args.native_code,args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        from curobo_robot_model import build_conservative_robot_model,fixed_world_mesh_descriptors
        from motiongen_plan import world_config
        from motiongen_motion import MotionGenerator
        state=dict(planner.home_state,left_joint7=planner.open_position_m,left_joint8=planner.open_position_m)
        right=(state['right_joint17']+state['right_joint18'])/2;state.update(right_joint17=right,right_joint18=right)
        certificate_path=args.output.parent/'mount-domain-v1/result.json'
        certificate=json.loads(certificate_path.read_text()) if certificate_path.exists() else None
        config,proxy=build_conservative_robot_model(data,ac_one_asset_spec(args.assets).urdf_path,planner.fk.movable,state,ignored_link_pairs=planner.geo.whole.ignores,domain_certificate=certificate)
        result['certified_domain_pairs']=proxy['certified_domain_pair_count']
        fixed=fixed_world_mesh_descriptors(data,proxy,planner.fk,state)
        world,_,_=world_config(planner,obs.sample.world_from_cup,fixed)
        result['left_spheres']=proxy['left_sphere_count'];result['phase']='constructor'
        (args.output/'progress.json').write_text(json.dumps(result));print('ENTER constructor',flush=True)
        engine=MotionGenerator(config,world)
        result.update(initialization_seconds=engine.initialization_seconds,installed_costs=len(engine.self_collision_partition['costs']),phase='warmup')
        (args.output/'progress.json').write_text(json.dumps(result));print('ENTER warmup',flush=True)
        engine.warmup()
        result.update(warmup_seconds=engine.warmup_seconds,phase='same_start_joint_query')
        (args.output/'progress.json').write_text(json.dumps(result));print('ENTER query',flush=True)
        start=config['kinematics']['cspace']['retract_config']
        try:
            _,receipt=engine.plan_joint(start,start,name='stationary-home-diagnostic')
            result['query']=receipt
        except ValueError as error:
            result.update(query_error=str(error),queries=engine.receipts)
        result.update(passed=True,phase='complete',query_success=bool(result.get('query',{}).get('success')),
                      note='initialization success is not path feasibility or scene acceptance')
    except Exception as error:result.update(passed=False,error=str(error),traceback=traceback.format_exc())
    result['seconds']=time.perf_counter()-began
    (args.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','source','native-code','assets','output'):p.add_argument('--'+name,type=Path,required=True)
    raise SystemExit(main(p.parse_args()))
