"""Bounded cuRobo density comparison; frozen inputs, no planning or robot actions.

Only finger backing changes. Uses official FK/world/self kernels, including the
existing 0.7.8 stale-output clearing fix, never the custom partition kernel.
"""
import argparse
from copy import deepcopy
import hashlib,json,statistics,time,traceback
from pathlib import Path


def main(args):
    root=args.output;root.mkdir(exist_ok=False)
    report=dict(passed=False,scope='GPU collision/gradient benchmark, NOT full planning latency',
                hardware_access=False,executable_plan=False,variants=[])
    began=time.perf_counter()
    try:
        import numpy as np
        from playground_geometry import load
        source=json.loads(args.source.read_text())
        planner,observation,data=load(args.input,source,args.native_code,args.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
        from robot_sim.tasks.manager_based.coffee_make.curobo_collision_cache import refresh_slow_self_collision_outputs
        from curobo_robot_model import build_robot_model,fixed_world_mesh_descriptors,digest
        from conservative_spheres import cover
        from motiongen_plan import world_config
        from isaac_lane import native_target
        import torch,curobo
        from curobo.types.base import TensorDeviceType
        from curobo.types.robot import RobotConfig
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        from curobo.geom.sdf.world import CollisionQueryBuffer,WorldCollisionConfig
        from curobo.geom.sdf.world_mesh import WorldMeshCollision
        from curobo.rollout.cost.self_collision_cost import SelfCollisionCost,SelfCollisionCostConfig
        assert curobo.__version__=='0.7.8' and torch.cuda.is_available()
        ta=TensorDeviceType(device=torch.device('cuda:0'))
        urdf=ac_one_asset_spec(args.assets).urdf_path
        state=dict(planner.home_state,left_joint7=planner.open_position_m,left_joint8=planner.open_position_m)
        right=(state['right_joint17']+state['right_joint18'])/2
        state.update(right_joint17=right,right_joint18=right)
        trajectory=json.loads(args.trajectory.read_text());spec=ac_one_dual_arm_zero_home_action_spec()
        selected=np.linspace(0,len(trajectory['native_actions'])-1,384).astype(int).tolist()
        joint_samples=[native_target(trajectory['native_actions'][i],spec,trajectory['gripper_opening_mm'],distance=True)['left_arm_joints'] for i in selected]
        report.update(curobo_version=curobo.__version__,gpu=torch.cuda.get_device_name(0),
            input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
            trajectory_sha256=hashlib.sha256(args.trajectory.read_bytes()).hexdigest(),
            joint_samples_sha256=digest(joint_samples),sample_indices=selected,
            workload='historical model-space arm samples; both variants use same fixed open left jaw and same world; not replay',
            common_locked_state={k:v for k,v in state.items() if not k.startswith('left_joint') or k in ('left_joint7','left_joint8')},
            custom_self_partition=False)
        fixed_world=None;common_geometry=None;body_hash=None
        for back in args.backing:
            row=dict(backing_m=back,status='running');report['variants'].append(row)
            (root/'progress.json').write_text(json.dumps(report,indent=2))
            stamp=time.perf_counter()
            retained={name for name,link in data['planner_identity']['chassis_collision']['base_arm_exemptions'] if link=='left_link1'}
            parts=[p for p in data['parts'] if p['link'].startswith('left_') or p['name'] in retained]
            iterator=iter(parts);details=[]
            def fitter(vertices,faces,count,radius):
                part=next(iterator);assert np.array_equal(vertices,np.asarray(part['vertices']))
                finger=part['link'] in ('left_link7','left_link8')
                options=dict(face_overhang_m=.002,protected_normal=[0,-1 if part['link']=='left_link7' else 1,0],backing_m=back) if finger else {}
                centers,radii,proof=cover(vertices,faces=faces,**options)
                details.append(dict(name=part['name'],link=part['link'],count=len(centers),**proof))
                return centers,radii,proof
            config,model_report=build_robot_model(data,urdf,planner.fk.movable,state,
                ignored_link_pairs=planner.geo.whole.ignores,fit=fitter)
            assert all(p['whole_solid_covered'] for p in details)
            # The fitter hook is used for this isolated comparison, not promoted
            # as a production model or as the prior adaptive-fit quality claim.
            static=fixed_world_mesh_descriptors(data,model_report,planner.fk,state)
            fixed_hash=digest(static)
            unchanged=digest({p['alias']:config['kinematics']['collision_spheres'][p['alias']]
                for p in model_report['part_map'].values() if p['representation']=='robot_spheres' and p['link'] not in ('left_link7','left_link8')})
            if common_geometry is None:common_geometry,body_hash=fixed_hash,unchanged
            assert (fixed_hash,unchanged)==(common_geometry,body_hash)
            row.update(model_prepare_seconds=time.perf_counter()-stamp,left_spheres=sum(p['count'] for p in details if p['link'].startswith('left')),
                allocated_spheres=model_report['allocated_spheres'],enabled_pairs=model_report['enabled_sphere_pairs'],
                pad_reports=[p for p in details if 'RealFingerRubberPad' in p['name']],
                max_finger_back_box_overhang_mm=1000*max(p['backing_face_overhang_m'] for p in details if p['link'] in ('left_link7','left_link8')),
                max_finger_front_box_overhang_mm=1000*max(p['protected_face_overhang_m'] for p in details if p['link'] in ('left_link7','left_link8')),
                fixed_geometry_sha256=fixed_hash,unchanged_body_spheres_sha256=unchanged)
            if row['allocated_spheres']>=1024:raise ValueError('official 0.7.8 self kernel sphere limit reached; no custom fallback')
            stamp=time.perf_counter()
            robot=RobotConfig.from_dict(deepcopy(config),ta);model=CudaRobotModel(robot.kinematics)
            self_config=model.get_self_collision_config()
            cost=SelfCollisionCost(SelfCollisionCostConfig(weight=1.,tensor_args=ta,self_collision_kin_config=self_config,return_loss=True))
            refresh_slow_self_collision_outputs(cost)
            if fixed_world is None:
                world,cup_names,desc=world_config(planner,observation.sample.world_from_cup,static)
                fixed_world=WorldMeshCollision(WorldCollisionConfig(tensor_args=ta,world_model=world,
                    cache={'mesh':len(world.mesh or []),'obb':len(world.cuboid or [])}))
            torch.cuda.synchronize();row['gpu_model_prepare_seconds']=time.perf_counter()-stamp
            row['self_kernel']='official_fast' if self_config.experimental_kernel else 'official_dense'
            row['benchmarks']=[]
            for batch,horizon in ((1,1),(12,32)):
                q=ta.to_device(joint_samples[:batch*horizon]).requires_grad_(True)
                n=row['allocated_spheres'];shape=(batch,horizon,n,4)
                buffer=CollisionQueryBuffer.initialize_from_shape(shape,ta,fixed_world.collision_types)
                weight,activation=ta.to_device([1.]),ta.to_device([.005])
                def query(which):
                    spheres=model.get_state(q).get_link_spheres().reshape(shape)
                    values=[]
                    if which in ('world','combined'):values.append(fixed_world.get_sphere_distance(spheres,buffer,weight,activation).sum())
                    if which in ('self','combined'):values.append(cost(spheres).sum())
                    grad=torch.autograd.grad(sum(values),q)[0]
                    return grad
                for which in ('world','self','combined'):
                    start=time.perf_counter();grad=query(which);torch.cuda.synchronize()
                    cold=time.perf_counter()-start
                    assert torch.isfinite(grad).all().item(),which
                    if cold>10:raise ValueError('single probe exceeded 10s; stop before repeated heavy queries')
                    walls=[];device=[]
                    for _ in range(5):
                        first,last=torch.cuda.Event(enable_timing=True),torch.cuda.Event(enable_timing=True)
                        torch.cuda.synchronize();start=time.perf_counter();first.record()
                        grad=query(which);last.record();torch.cuda.synchronize()
                        walls.append((time.perf_counter()-start)*1000);device.append(first.elapsed_time(last))
                    assert torch.isfinite(grad).all().item()
                    row['benchmarks'].append(dict(batch=batch,horizon=horizon,operation=which,
                        first_call_ms=cold*1000,wall_median_ms=statistics.median(walls),gpu_median_ms=statistics.median(device),wall_samples_ms=walls,gpu_samples_ms=device))
            row['status']='complete';row['gpu_peak_memory_mib']=torch.cuda.max_memory_allocated()/2**20
            if back==args.backing[-1]:
                # Diagnostic only: one enclosing parent sphere per material part.
                # Measure whether an official coarse self query can reject work;
                # do not implement a new hierarchy or change MotionGen here.
                coarse_config=deepcopy(config)
                for alias,values in coarse_config['kinematics']['collision_spheres'].items():
                    if not values or any(v['radius']<=0 for v in values):continue
                    points=np.asarray([v['center'] for v in values]);radii=np.asarray([v['radius'] for v in values])
                    center=points.mean(0);radius=float((np.linalg.norm(points-center,axis=1)+radii).max())+1e-6
                    coarse_config['kinematics']['collision_spheres'][alias]=[dict(center=center.tolist(),radius=radius)]
                coarse_robot=RobotConfig.from_dict(deepcopy(coarse_config),ta)
                coarse_model=CudaRobotModel(coarse_robot.kinematics)
                coarse_self=coarse_model.get_self_collision_config()
                coarse_cost=SelfCollisionCost(SelfCollisionCostConfig(weight=1.,tensor_args=ta,self_collision_kin_config=coarse_self,return_loss=True))
                refresh_slow_self_collision_outputs(coarse_cost)
                qc=ta.to_device(joint_samples).requires_grad_(True)
                coarse_n=coarse_model.get_state(qc).get_link_spheres().shape[-2]
                def coarse_query():
                    values=coarse_cost(coarse_model.get_state(qc).get_link_spheres().reshape(12,32,coarse_n,4))
                    grad=torch.autograd.grad(values.sum(),qc)[0]
                    return values.detach().clone(),grad
                coarse_value,grad=coarse_query();torch.cuda.synchronize();assert torch.isfinite(grad).all()
                times=[]
                for _ in range(5):
                    torch.cuda.synchronize();start=time.perf_counter();coarse_value,grad=coarse_query();torch.cuda.synchronize();times.append((time.perf_counter()-start)*1000)
                fine_value=cost(model.get_state(q).get_link_spheres().reshape(12,32,n,4)).detach().clone()
                torch.cuda.synchronize()
                labels={}
                for part,item in model_report['part_map'].items():
                    if item['representation']=='robot_spheres':
                        for index in model.kinematics_config.get_sphere_index_from_link_name(item['alias']).cpu().tolist():labels[int(index)]=part
                vectors=cost._out_vec.detach().cpu().numpy().reshape(384,n,4)
                witnesses=[]
                for index in (0,95,191,383):
                    ids=np.flatnonzero(np.linalg.norm(vectors[index,:,:3],axis=1)>1e-8).tolist()
                    witnesses.append(dict(sample=index,parts=sorted({labels.get(i,'attached_object') for i in ids})))
                missed=((fine_value>0)&(coarse_value<=0)).sum().item()
                assert missed==0,'coarse parent missed fine collision in common workload'
                report['hierarchy_probe']=dict(scope='self-collision broad rejection opportunity only; no adaptive hierarchy implemented',
                    parent_spheres=coarse_n,official_fast=bool(coarse_self.experimental_kernel),
                    coarse_fk_gradient_ms=statistics.median(times),samples=384,
                    coarse_positive_states=int((coarse_value>0).sum().item()),fine_positive_states=int((fine_value>0).sum().item()),
                    coarse_clear_trajectories=int((coarse_value<=0).all(dim=1).sum().item()),trajectories=12,
                    fine_collision_misses=missed,parents_enclose_all_child_spheres=True,fine_maximum_witnesses=witnesses)
                parent_spheres=coarse_model.get_state(qc).get_link_spheres().detach().reshape(384,coarse_n,4)
                pairs=coarse_self.thread_location[:coarse_self.thread_max].reshape(-1,2).long()
                pairs=pairs[(pairs>=0).all(dim=1)]
                effective=parent_spheres[...,3]+coarse_self.offset.reshape(1,-1)
                enabled=(effective[0,pairs[:,0]]>0)&(effective[0,pairs[:,1]]>0)
                pairs=pairs[enabled]
                child_count=torch.zeros(coarse_n,device=ta.device)
                for alias,values in config['kinematics']['collision_spheres'].items():
                    ids=coarse_model.kinematics_config.get_sphere_index_from_link_name(alias).long()
                    child_count[ids]=len(values)/len(ids)
                weights=child_count[pairs[:,0]]*child_count[pairs[:,1]]
                gap=torch.linalg.vector_norm(parent_spheres[:,pairs[:,0],:3]-parent_spheres[:,pairs[:,1],:3],dim=-1)-effective[:,pairs[:,0]]-effective[:,pairs[:,1]]
                regions=[]
                for margin in (0.,.005):
                    risk=gap<=margin
                    regions.append(dict(activation_margin_m=margin,parent_pair_count=len(pairs),
                        fine_pair_count=float(weights.sum().item()),mean_risk_parent_pairs=float(risk.float().sum(1).mean().item()),
                        mean_risk_fine_pair_fraction=float(((risk*weights).sum(1)/weights.sum()).mean().item()),
                        batch_union_fine_pair_fraction=float((risk.any(0)*weights).sum().item()/weights.sum().item())))
                report['hierarchy_probe']['region_opportunity']=regions
                # Bounded feasibility probe using the official fast kernel's
                # pair table. This is not installed into MotionGen rollouts.
                from curobo.cuda_robot_model.types import SelfCollisionKinematicsConfig
                fine_pairs=torch.nonzero(torch.triu(self_config.collision_matrix==1,diagonal=1)).long()
                parent_of=torch.empty(n,dtype=torch.long,device=ta.device)
                for alias in config['kinematics']['collision_spheres']:
                    ids=model.kinematics_config.get_sphere_index_from_link_name(alias).long()
                    parents=coarse_model.kinematics_config.get_sphere_index_from_link_name(alias).long()
                    parent_of[ids]=parents[0] if len(parents)==1 else parents
                padded=512;capacity=16384
                locations=torch.full((capacity*2,),-1,dtype=torch.int16,device=ta.device)
                fast_config=SelfCollisionKinematicsConfig(offset=torch.zeros(padded,device=ta.device),
                    collision_matrix=torch.zeros(2,dtype=torch.uint8,device=ta.device),thread_location=locations,
                    thread_max=capacity*2,checks_per_thread=32,experimental_kernel=True)
                selected_cost=SelfCollisionCost(SelfCollisionCostConfig(weight=1.,tensor_args=ta,self_collision_kin_config=fast_config,return_loss=True))
                active_sizes=[]
                def selected_query():
                    # Coarse parents enclose every child. Union across this
                    # batch is conservative; unselected pairs cannot collide.
                    coarse=coarse_model.get_state(qc.detach()).get_link_spheres().detach().reshape(384,coarse_n,4)
                    radii=coarse[...,3]+coarse_self.offset.reshape(1,-1)
                    separation=torch.linalg.vector_norm(coarse[:,pairs[:,0],:3]-coarse[:,pairs[:,1],:3],dim=-1)-radii[:,pairs[:,0]]-radii[:,pairs[:,1]]
                    candidates=(separation<=.005).any(0)
                    matrix=torch.zeros((coarse_n,coarse_n),dtype=torch.bool,device=ta.device)
                    matrix[pairs[:,0],pairs[:,1]]=candidates;matrix[pairs[:,1],pairs[:,0]]=candidates
                    selected=fine_pairs[matrix[parent_of[fine_pairs[:,0]],parent_of[fine_pairs[:,1]]]]
                    if len(selected)>capacity:raise ValueError('risk subset exceeds official pair capacity; probe refuses truncation')
                    ids,local_pairs=torch.unique(selected.flatten(),sorted=True,return_inverse=True)
                    if len(ids)>padded:raise ValueError('risk subset exceeds official fast-kernel resource envelope; probe refuses truncation')
                    active_sizes.append(len(ids))
                    locations.fill_(-1);locations[:local_pairs.numel()].copy_(local_pairs.to(torch.int16))
                    fast_config.offset.zero_();fast_config.offset[:len(ids)].copy_(self_config.offset.index_select(0,ids))
                    spheres=model.get_state(q).get_link_spheres().reshape(12,32,n,4).index_select(2,ids)
                    padding=torch.zeros((12,32,padded-len(ids),4),device=ta.device);padding[...,3]=-10.
                    value=selected_cost(torch.cat((spheres,padding),dim=2))
                    grad=torch.autograd.grad(value.sum(),q)[0]
                    return value.detach().clone(),grad.detach().clone(),len(selected)
                reference=cost(model.get_state(q).get_link_spheres().reshape(12,32,n,4))
                reference_grad=torch.autograd.grad(reference.sum(),q)[0].detach().clone();reference=reference.detach().clone()
                value,gradient,selected_count=selected_query();torch.cuda.synchronize()
                value_error=float((reference-value).abs().max().item());gradient_error=float((reference_grad-gradient).abs().max().item())
                assert torch.isfinite(gradient).all().item() and value_error<1e-5 and gradient_error<1e-4,(value_error,gradient_error)
                times=[]
                for _ in range(5):
                    torch.cuda.synchronize();start=time.perf_counter();selected_query();torch.cuda.synchronize();times.append((time.perf_counter()-start)*1000)
                report['hierarchy_probe']['official_subset_probe']=dict(selected_pairs=selected_count,
                    selected_spheres=max(active_sizes),padded_spheres=padded,
                    wall_median_ms=statistics.median(times),wall_samples_ms=times,value_error=value_error,joint_gradient_error=gradient_error,
                    includes='coarse FK, region distances, batch union, pair compaction, fine FK, official self query and backward',
                    caveats='fixed open jaws, attachment disabled, one workload; dynamic pair compaction not CUDA-graph validated; over-capacity fails not truncates',
                    installed_in_planner=False)
            (root/f'variant-{round(back*1000):02d}.json').write_text(json.dumps(row,indent=2))
            print(json.dumps({k:v for k,v in row.items() if k!='pad_reports'}),flush=True)
            del cost,model,robot,buffer,q;torch.cuda.empty_cache()
        report['passed']=True
    except Exception as error:report.update(error=str(error),traceback=traceback.format_exc())
    report['seconds']=time.perf_counter()-began
    (root/'result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='variants'}),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('input','source','native-code','assets','trajectory','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--backing',type=float,nargs='+',default=[.01,.005,.002,0.])
    raise SystemExit(main(p.parse_args()))
