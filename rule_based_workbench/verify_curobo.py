"""Offline parity and timing against frozen native geometry; no robot services."""
import argparse
import json
import time
from pathlib import Path

from benchmark_planning import write_new


def verify(args):
    import numpy as np
    from playground_geometry import load
    from curobo_whole_geometry import CuroboWholeGeometry
    source=json.loads(args.source.read_text())
    planner,obs,data=load(args.input,source,args.native_code,args.assets)
    whole=planner.geo.whole
    rng=np.random.default_rng(52)
    start=dict(planner.home_state)
    states=[start]
    names=tuple(f'left_joint{i}' for i in range(1,7))
    # Random configurations deliberately include forbidden states; they are never commanded.
    for _ in range(40):
        states.append({**start,**dict(zip(names,rng.uniform([-.3,0,0,-.5,-.3,-.4],[.4,1,2,.5,.3,.4])))})
    extra_index=None
    if args.extra_state:
        extra=json.loads(args.extra_state.read_text())
        extra_index=len(states)
        states.append(extra['positions'])
    if args.initial_plan:
        initial=json.loads(args.initial_plan.read_text())
        selected=initial.get('selected') or initial['rejected_candidates'][0]
        states.append(selected['closing']['terminal_state'])
        states.extend({**selected['lift']['terminal_state'],**dict(zip(names,row['q']))}
                      for row in selected['lift']['checks'])
    frames=[{n:planner.fk.pose(t) for n,t in planner.fk.forward(s).items()} for s in states]
    options=dict(margin=.0001,world_margin_paths=planner.geo.machine_shape_paths,world_margin=.003)
    native_begin=time.perf_counter()
    reference=[whole.audit(f,**options) for f in frames]
    if extra_index is not None:
        assert not reference[extra_index]['free'], 'recorded mast collision must remain rejected'
        assert any(all(token in str(pair) for token in extra['expected_pair_fragments'])
                   for pair in reference[extra_index]['violations']), reference[extra_index]
    native_seconds=time.perf_counter()-native_begin
    # Measure the untouched reference before the candidate installs per-mesh bounds.
    warm_native=[]
    for _ in range(3):
        tick=time.perf_counter()
        for frame in frames:whole.audit(frame,**options)
        warm_native.append(time.perf_counter()-tick)
    callback=None
    if args.backend=='curobo':
        from curobo_collision import CuroboWorldBroadphase
        engine=CuroboWorldBroadphase(data)
        assert engine.object_ids==tuple(v[0] for v in whole.world)
        def callback(spheres,clearance):
            return engine.query(spheres[None,None],clearance)[0,0].cpu().numpy()
    candidate=CuroboWholeGeometry(whole,callback)
    candidate.audit(frames[0],**options)
    started=time.perf_counter()
    actual=[candidate.audit(f,**options) for f in frames]
    seconds=time.perf_counter()-started
    mismatches=[]
    for index,(a,b) in enumerate(zip(reference,actual)):
        # Ordering/global nearest distance may change, but forbidden pair identities may not.
        expected={tuple(pair[:2]) for pair in a['violations']}
        found={tuple(pair[:2]) for pair in b['violations']}
        if a['free']!=b['free'] or expected!=found:
            mismatches.append(dict(index=index,expected=sorted(expected),actual=sorted(found)))
    assert not mismatches, mismatches
    warm_candidate=[]
    for _ in range(3):
        tick=time.perf_counter()
        for frame in frames:candidate.audit(frame,**options)
        warm_candidate.append(time.perf_counter()-tick)
    result=dict(passed=True,backend=args.backend,samples=len(states),
                reference_free=sum(r['free'] for r in reference),
                candidate_free=sum(r['free'] for r in actual),
                collision_pair_mismatches=mismatches,native_seconds=native_seconds,
                candidate_seconds=seconds,scope='whole geometry; not a full motion execution certificate')
    result.update(warm_native_median_seconds=float(np.median(warm_native)),
                  warm_candidate_median_seconds=float(np.median(warm_candidate)))
    if args.fast_cup_audit:
        from curobo_relative_geometry import install
        from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
        geo=planner.geo
        original=geo.audit
        geo.whole=candidate
        cases=[(frame,obs.sample.world_from_cup,dict(initial_table_contact=True)) for frame in frames]
        if args.initial_plan:
            closing=selected['closing']
            samples=closing['samples']
            for index in (0,len(samples)//2,len(samples)-1):
                sample=samples[index]
                state={**closing['terminal_state'],**sample['jaws']}
                sample_frames={n:planner.fk.pose(t) for n,t in planner.fk.forward(state).items()}
                cases.append((sample_frames,RigidTransform(**sample['world_from_cup']),
                              dict(grasp_contact=True,initial_table_contact=True)))
            placements=selected['placement'].get('rejected_candidates',[])
            for rejected in placements:
                cases.append((frames[0],RigidTransform(**rejected['candidate']['world_from_cup']),
                              dict(support_contact=True)))
        reference_cup=[original(frame,cup,**options) for frame,cup,options in cases]
        install(geo)
        actual_cup=[geo.audit(frame,cup,**options) for frame,cup,options in cases]
        for i,(a,b) in enumerate(zip(reference_cup,actual_cup)):
            assert a.free==b.free and {tuple(p[:2]) for p in a.violations}=={tuple(p[:2]) for p in b.violations}, (
                'cup geometry mismatch',i,a,b)
        result.update(cup_cases=len(cases),cup_pair_mismatches=[],cup_audit_stats=geo._candidate_audit_stats)
    if args.kinematics:
        import torch
        from curobo_planning import A1Kinematics,LEFT_JOINTS
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        gpu=A1Kinematics(ac_one_asset_spec(args.assets).urdf_path,planner.fk.movable,start,
                         use_cuda_graph=args.cuda_graphs)
        # Same lock state for every FK comparison, including all four jaws and right arm.
        q=np.array([[s[n] for n in LEFT_JOINTS] for s in states[:41]])
        gpu_states=gpu.batch_fk(q)
        torch.cuda.synchronize()
        positions=gpu_states.links_position.detach().cpu().numpy()
        quats=gpu_states.links_quaternion.detach().cpu().numpy()
        position_errors=[];rotation_errors=[]
        for i,f in enumerate(frames[:41]):
            for j,name in enumerate(gpu_states.link_names):
                position_errors.append(float(np.linalg.norm(positions[i,j]-f[name].position)))
                # Quaternion chord error avoids acos amplification for near-equal rotations.
                rotation_errors.append(float(min(np.linalg.norm(quats[i,j]-f[name].orientation_wxyz),
                                                 np.linalg.norm(quats[i,j]+f[name].orientation_wxyz))))
        assert max(position_errors)<2e-5,(max(position_errors),'FK position')
        assert max(rotation_errors)<2e-5,(max(rotation_errors),'FK quaternion')
        fk_times=[]
        for _ in range(10):
            t=time.perf_counter();gpu.batch_fk(q);torch.cuda.synchronize();fk_times.append(time.perf_counter()-t)
        target=[frames[i]['left_link6'] for i in range(1,9)]
        perturbation=np.random.default_rng(20260928).uniform(-.02,.02,(len(target),len(LEFT_JOINTS)))
        seeds=q[1:9]+perturbation
        target_rotations=[np.column_stack([p.rotate(tuple(float(i==j) for i in range(3)))
                                           for j in range(3)]) for p in target]
        ik_times=[];ik_residuals=[];ik_position_errors=[];ik_rotation_errors=[]
        # First solve includes IK warmup/graph capture; all three warm solves use
        # the same reachable goals and perturbed seeds, never the known answers.
        for call in range(4):
            torch.cuda.synchronize()
            t=time.perf_counter()
            solved=gpu.batch_ik([p.position for p in target],[p.orientation_wxyz for p in target],seeds=seeds)
            torch.cuda.synchronize()
            ik_times.append(time.perf_counter()-t)
            if not solved.success.all().item():
                raise AssertionError(f'IK call {call} failed: success={solved.success.detach().cpu().tolist()}, '
                                     f'position={solved.position_error.detach().cpu().tolist()}, '
                                     f'rotation={solved.rotation_error.detach().cpu().tolist()}')
            solutions=solved.solution.detach().cpu().numpy().reshape(len(target),-1)
            if solutions.shape!=(len(target),len(LEFT_JOINTS)):
                raise AssertionError(f'IK call {call} returned unexpected solution shape {solutions.shape}')
            for index,(solution,goal,rotation) in enumerate(zip(solutions,target,target_rotations)):
                current=planner.fk.forward({**start,**dict(zip(LEFT_JOINTS,solution))})['left_link6']
                position_error=current[:3,3]-goal.position
                rotation_error=planner.fk.Rotation.from_matrix(rotation.T@current[:3,:3]).as_rotvec()
                residual=float(np.linalg.norm(np.r_[position_error,rotation_error]))
                if not np.isfinite(residual) or residual>1e-5:
                    raise AssertionError(f'IK call {call} target {index} native FK residual={residual}; limit=1e-5')
                ik_residuals.append(residual)
                ik_position_errors.append(float(np.linalg.norm(position_error)))
                ik_rotation_errors.append(float(np.linalg.norm(rotation_error)))
        result['kinematics']=dict(samples=len(q),links=len(gpu_states.link_names),
            max_position_error_m=max(position_errors),max_quaternion_chord_error=max(rotation_errors),
            warm_fk_median_ms=float(np.median(fk_times))*1000,
            use_cuda_graph=args.cuda_graphs,ik_seed_rng=20260928,
            ik_seed_perturbation_max_rad=float(np.abs(perturbation).max()),
            ik_batch_seconds=ik_times[0],ik_cold_ms=ik_times[0]*1000,
            warm_ik_median_ms=float(np.median(ik_times[1:]))*1000,
            warm_ik_ms=[v*1000 for v in ik_times[1:]],ik_targets=len(target),
            ik_successes=int(solved.success.sum().item()),ik_native_fk_checks=len(ik_residuals),
            max_ik_native_residual=max(ik_residuals),
            max_ik_position_error_m=max(ik_position_errors),
            max_ik_rotation_error_rad=max(ik_rotation_errors))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('input','source','native-code','assets','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--initial-plan',type=Path)
    parser.add_argument('--extra-state',type=Path,help='Recorded diagnostic joint state with expected collision pair fragments.')
    parser.add_argument('--backend',choices=('batch-cpu','curobo'),default='batch-cpu')
    parser.add_argument('--kinematics',action='store_true')
    parser.add_argument('--fast-cup-audit',action='store_true')
    parser.add_argument('--cuda-graphs',action='store_true')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    try:
        result=verify(args)
    except Exception as error:
        write_new(args.output/'verification.json',dict(passed=False,error=str(error)))
        raise
    write_new(args.output/'verification.json',result)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
