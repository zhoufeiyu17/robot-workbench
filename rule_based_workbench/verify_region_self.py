"""Actual-model region self query: GPU tensor oracle, CUDA graph and live radii.

No MotionGen plan is published and no robot/simulator action is executed.
"""
import argparse,json,time,traceback,statistics
from copy import deepcopy
from pathlib import Path


def main(args):
    args.output.mkdir(exist_ok=False);began=time.perf_counter()
    result=dict(passed=False,hardware_access=False,executable_plan=False)
    try:
        from playground_geometry import load
        planner,obs,data=load(args.input,json.loads(args.source.read_text()),args.native_code,args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        from curobo_robot_model import build_conservative_robot_model
        state=dict(planner.home_state,left_joint7=planner.open_position_m,left_joint8=planner.open_position_m)
        right=(state['right_joint17']+state['right_joint18'])/2;state.update(right_joint17=right,right_joint18=right)
        certificate_path=args.output.parent/'mount-domain-v1/result.json'
        certificate=json.loads(certificate_path.read_text()) if certificate_path.exists() else None
        raw,report=build_conservative_robot_model(data,ac_one_asset_spec(args.assets).urdf_path,planner.fk.movable,state,ignored_link_pairs=planner.geo.whole.ignores,domain_certificate=certificate)
        (args.output/'robot-config.json').write_text(json.dumps(raw,indent=2));(args.output/'model-report.json').write_text(json.dumps(report,indent=2))
        import torch,curobo
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import RobotConfig
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        from curobo.rollout.cost.self_collision_cost import SelfCollisionCost,SelfCollisionCostConfig
        from region_self_collision import install_cost
        from motiongen_self_collision import enabled_pairs
        assert curobo.__version__=='0.7.8'
        ta=TensorDeviceType(device=torch.device('cuda:0'))
        robot=RobotConfig.from_dict(deepcopy(raw),ta);model=CudaRobotModel(robot.kinematics)
        cfg=model.get_self_collision_config();n,pair_rows=enabled_pairs(cfg)
        pairs=torch.tensor(pair_rows,device=ta.device,dtype=torch.long)
        links=model.kinematics_config.link_sphere_idx_map.detach().cpu().reshape(-1).tolist()
        cost=SelfCollisionCost(SelfCollisionCostConfig(weight=1.,tensor_args=ta,self_collision_kin_config=cfg,return_loss=True))
        result.update(model_left_spheres=report['left_sphere_count'],all_spheres=n,region=install_cost(cost,links))
        q=ta.to_device([raw['kinematics']['cspace']['retract_config']])
        near=model.get_state(q).get_link_spheres().detach().clone().unsqueeze(1)
        groups=torch.tensor(links,device=ta.device,dtype=torch.float32)
        far=near.clone();far[...,0]+=groups*1000.
        attached=far.clone();cup_ids=model.kinematics_config.get_sphere_index_from_link_name('attached_object').long()
        metal_ids=model.kinematics_config.get_sphere_index_from_link_name(next(p['alias'] for p in report['part_map'].values() if p['link']=='left_link3' and p['representation']=='robot_spheres')).long()
        attached[...,cup_ids[0],:3]=attached[...,metal_ids[0],:3]+ta.to_device([.005,0,0]);attached[...,cup_ids[0],3]=.01
        changed=deepcopy(raw)
        changed['kinematics']['lock_joints'].update(left_joint7=.032496,left_joint8=.032496)
        other=RobotConfig.from_dict(deepcopy(changed),ta)
        model.update_kinematics_config(other.kinematics.kinematics_config)
        closed=model.get_state(q).get_link_spheres().detach().clone().unsqueeze(1)
        cases=[('home',near),('far',far),('attachment_enabled',attached),('far_after',far),('closed_jaws',closed)]

        def oracle(spheres):
            radius=spheres.detach()[...,3]+cfg.offset
            a,b=pairs[:,0],pairs[:,1]
            raw_distance=radius[...,a]+radius[...,b]-torch.linalg.vector_norm(spheres[...,a,:3]-spheres[...,b,:3],dim=-1)
            valid=(radius[...,a]>0)&(radius[...,b]>0)&(raw_distance>0)
            return torch.where(valid,raw_distance,0.).max(dim=-1).values

        checks=[]
        for name,spheres in cases:
            expected_input=spheres.clone().requires_grad_(True);expected=oracle(expected_input)
            reference_gradient=torch.autograd.grad(expected.sum(),expected_input)[0]
            actual_input=spheres.clone().requires_grad_(True);actual=cost(actual_input)
            gradient=torch.autograd.grad(actual.sum(),actual_input)[0]
            torch.cuda.synchronize()
            error=float((actual-expected).abs().max());ge=float((gradient-reference_gradient).abs().max())
            stats=cost._region_stats.cpu().tolist()
            if stats[2]:raise ValueError(f'{name}: region capacity exceeded {stats}')
            assert torch.isfinite(gradient).all() and error<2e-5 and ge<2e-4,(name,error,ge)
            checks.append(dict(case=name,collision_cost=float(actual.max()),value_error=error,sphere_gradient_error=ge,stats=stats))
        result['eager_checks']=checks
        cost.return_loss=False
        check=closed.clone().requires_grad_(True);expected=oracle(check)
        reference=torch.autograd.grad(expected.sum(),check)[0]
        query=closed.clone().requires_grad_(True);value=cost(query)
        actual=torch.autograd.grad((value*3.).sum(),query)[0]
        assert torch.allclose(actual,reference,atol=2e-4,rtol=0)
        result['native_ignore_outer_multiplier_preserved']=True
        static=near.clone().requires_grad_(True)
        stream=torch.cuda.Stream();stream.wait_stream(torch.cuda.current_stream())
        with torch.cuda.stream(stream):
            for _ in range(3):torch.autograd.grad(cost(static).sum(),static)
        torch.cuda.current_stream().wait_stream(stream)
        graph=torch.cuda.CUDAGraph()
        with torch.cuda.graph(graph):
            value=cost(static);gradient=torch.autograd.grad((value*3.).sum(),static)[0]
        graph_checks=[]
        for name,spheres in cases:
            with torch.no_grad():static.copy_(spheres)
            graph.replay();torch.cuda.synchronize()
            expected_input=spheres.clone().requires_grad_(True);expected=oracle(expected_input)
            reference_gradient=torch.autograd.grad(expected.sum(),expected_input)[0]
            error=float((value-expected).abs().max());ge=float((gradient-reference_gradient).abs().max())
            assert torch.isfinite(gradient).all() and error<2e-5 and ge<2e-4,(name,error,ge)
            assert cost._region_stats[2].item()==0
            graph_checks.append(dict(case=name,value_error=error,sphere_gradient_error=ge))
        # Stress capacity explicitly; fail closed without evaluating coincident
        # truncated geometry or requiring a device-to-host decision in a graph.
        crowded=near.clone();crowded[...,:3]=0.
        with torch.no_grad():static.copy_(crowded)
        graph.replay();torch.cuda.synchronize()
        assert cost._region_stats[2].item()==1 and value.min().item()>=1000 and torch.isfinite(gradient).all()
        times=[]
        with torch.no_grad():static.copy_(closed)
        graph.replay();torch.cuda.synchronize()
        for _ in range(10):
            start=time.perf_counter();graph.replay();torch.cuda.synchronize();times.append((time.perf_counter()-start)*1000)
        result.update(passed=True,cuda_graph_checks=graph_checks,overflow_rejected=True,
            graph_single_state_ms=statistics.median(times),gpu_peak_memory_mib=torch.cuda.max_memory_allocated()/2**20)
        # Offline diagnostic: distinguish the worst proxy home contact from
        # source-part geometry; never called in the online planning path.
        home=near.detach();rad=home[...,3]+cfg.offset
        a,b=pairs[:,0],pairs[:,1]
        distance=rad[...,a]+rad[...,b]-torch.linalg.vector_norm(home[...,a,:3]-home[...,b,:3],dim=-1)
        distance=torch.where((rad[...,a]>0)&(rad[...,b]>0),distance,-1e6)
        i,j=pair_rows[int(distance.reshape(-1).argmax())]
        names={}
        for name,item in report['part_map'].items():
            if item['representation']=='robot_spheres':
                for index in model.kinematics_config.get_sphere_index_from_link_name(item['alias']).cpu().tolist():names[int(index)]=name
        active_rows=torch.nonzero(distance.reshape(-1)>0).reshape(-1).cpu().tolist()
        result['home_proxy_pairs']=sorted({tuple(sorted((names[pair_rows[k][0]],names[pair_rows[k][1]]))) for k in active_rows})
        import fcl,trimesh,numpy as np
        home_state=dict(state,**dict(zip(model.joint_names,q[0].cpu().tolist())))
        frames=planner.fk.forward(home_state);objects=[]
        for index in (i,j):
            part=next(p for p in data['parts'] if p['name']==names[index])
            hull=trimesh.convex.convex_hull(np.asarray(part['vertices']))
            faces=np.column_stack((np.full(len(hull.faces),3),hull.faces)).reshape(-1)
            frame=frames[part['link']]
            objects.append(fcl.CollisionObject(fcl.Convex(hull.vertices,len(hull.faces),faces),fcl.Transform(frame[:3,:3],frame[:3,3])))
        result['offline_home_witness']=dict(parts=[names[i],names[j]],proxy_cost_m=float(distance.max()),
            source_part_convex_distance_m=float(fcl.distance(*objects)),not_an_online_check=True)
    except Exception as error:result.update(passed=False,error=str(error),traceback=traceback.format_exc())
    result['seconds']=time.perf_counter()-began;(args.output/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','source','native-code','assets','output'):p.add_argument('--'+name,type=Path,required=True)
    raise SystemExit(main(p.parse_args()))
