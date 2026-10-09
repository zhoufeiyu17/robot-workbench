"""GPU start-world pair diagnostics for the conservative model; no actions."""
import argparse,json,time,traceback
from pathlib import Path
from copy import deepcopy


def main(args):
    args.output.mkdir(exist_ok=False);start=time.perf_counter();result=dict(passed=False,hardware_access=False,executable_plan=False)
    try:
        import numpy as np,torch
        from playground_geometry import load
        planner,obs,data=load(args.input,json.loads(args.source.read_text()),args.native_code,args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        from curobo_robot_model import build_conservative_robot_model,fixed_world_mesh_descriptors
        from motiongen_plan import world_config
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import RobotConfig
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        from curobo.geom.types import WorldConfig
        from curobo.geom.sdf.world import CollisionQueryBuffer,WorldCollisionConfig
        from curobo.geom.sdf.world_mesh import WorldMeshCollision
        state=dict(planner.home_state,left_joint7=planner.open_position_m,left_joint8=planner.open_position_m)
        right=(state['right_joint17']+state['right_joint18'])/2;state.update(right_joint17=right,right_joint18=right)
        certificate_path=args.output.parent/'mount-domain-v1/result.json'
        certificate=json.loads(certificate_path.read_text()) if certificate_path.exists() else None
        raw,report=build_conservative_robot_model(data,ac_one_asset_spec(args.assets).urdf_path,planner.fk.movable,state,ignored_link_pairs=planner.geo.whole.ignores,domain_certificate=certificate)
        ta=TensorDeviceType(device=torch.device('cuda:0'));model=CudaRobotModel(RobotConfig.from_dict(deepcopy(raw),ta).kinematics)
        q=ta.to_device([raw['kinematics']['cspace']['retract_config']]);spheres=model.get_state(q).get_link_spheres().detach().unsqueeze(1)
        fixed=fixed_world_mesh_descriptors(data,report,planner.fk,state);world,_,_=world_config(planner,obs.sample.world_from_cup,fixed)
        worlds=[];names=[]
        for shape in world.cuboid or []:worlds.append(WorldConfig(cuboid=[shape]));names.append(shape.name)
        for shape in world.mesh or []:worlds.append(WorldConfig(mesh=[shape]));names.append(shape.name)
        checker=WorldMeshCollision(WorldCollisionConfig(tensor_args=ta,world_model=worlds,cache={'mesh':1,'obb':1}))
        query=spheres.repeat(len(worlds),1,1,1);buffer=CollisionQueryBuffer.initialize_from_shape(query.shape,ta,checker.collision_types)
        costs=checker.get_sphere_distance(query,buffer,ta.to_device([1.]),ta.to_device([0.]),env_query_idx=torch.arange(len(worlds),device=ta.device,dtype=torch.int32))
        torch.cuda.synchronize();costs=costs.detach().cpu().numpy()[:,0]
        labels={}
        for name,item in report['part_map'].items():
            if item['representation']=='robot_spheres':
                for i in model.kinematics_config.get_sphere_index_from_link_name(item['alias']).cpu().tolist():labels[int(i)]=name
        fixed_names={p['name']:p['source_part_name'] for p in fixed}
        rows=[]
        for env,row in enumerate(costs):
            for part in sorted({labels.get(int(i),'attached_object') for i in np.flatnonzero(row>0)}):
                ids=[i for i in np.flatnonzero(row>0) if labels.get(int(i),'attached_object')==part]
                rows.append(dict(robot_part=part,world_part=fixed_names.get(names[env],names[env]),cost=float(row[ids].max()),sphere_count=len(ids)))
        # Verify model construction coverage without turning it into an online check.
        coverage=[];rng=np.random.default_rng(4)
        for part in data['parts']:
            item=report['part_map'][part['name']]
            if item['representation']!='robot_spheres':continue
            original=np.asarray(part['vertices']);cells=raw['kinematics']['collision_spheres'][item['alias']]
            centers=np.asarray([v['center'] for v in cells]);radii=np.asarray([v['radius'] for v in cells])
            indices=rng.integers(0,len(original),size=(100,4));weights=rng.dirichlet(np.ones(4),100)
            samples=np.concatenate((original,(original[indices]*weights[...,None]).sum(1)))
            gap=max(float((np.linalg.norm(samples[i:i+128,None]-centers[None],axis=-1)-radii).min(1).max()) for i in range(0,len(samples),128))
            assert gap<=1e-7,(part['name'],gap)
            coverage.append(dict(part=part['name'],max_sample_gap_m=gap))
        result.update(passed=True,left_spheres=report['left_sphere_count'],all_spheres=report['allocated_spheres'],
            start_world_risks=sorted(rows,key=lambda x:-x['cost']),coverage_sample_checks=coverage,
            scope='proxy risk localization and offline model checks, not a path validation')
        (args.output/'model-report.json').write_text(json.dumps(report,indent=2))
    except Exception as error:result.update(error=str(error),traceback=traceback.format_exc())
    result['seconds']=time.perf_counter()-start;(args.output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='coverage_sample_checks'},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('input','source','native-code','assets','output'):p.add_argument('--'+n,type=Path,required=True)
    raise SystemExit(main(p.parse_args()))
