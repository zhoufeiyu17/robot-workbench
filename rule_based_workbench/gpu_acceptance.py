"""GPU acceptance of fixed-arm symmetric jaw sweeps; no mesh narrow phase."""
import math
import time
from a1_asset_derivation import certificate_path


def jaw_sweep(engine, planner, q, start, end, *, centering=None):
    """Translate native finger spheres, then use the installed native GPU costs.

    50um samples are inflated by 25um to cover the continuous prismatic sweep.
    This checks commanded geometry only up to rigid contact, not elastic preload.
    """
    if not all(math.isfinite(v) for v in (*q,start,end)) or len(q)!=6:
        raise ValueError('finite six-joint pose and jaw endpoints required')
    upper=planner.open_position_m
    if not 0<=min(start,end)<=max(start,end)<=upper+1e-9:
        raise ValueError('jaw sweep outside native limits')
    torch=engine.torch;began=time.perf_counter();model=engine.generator.kinematics
    locked=engine.config['kinematics']['lock_joints'];state={**locked,**dict(zip([f'left_joint{i}' for i in range(1,7)],q))}
    frames=planner.fk.forward(state)
    count=max(1,math.ceil(abs(end-start)/.00005))+1
    positions=torch.linspace(start,end,count,device=engine.tensor_args.device)
    spheres=model.get_state(engine.tensor_args.to_device([q])).get_link_spheres().unsqueeze(1).repeat(1,count,1,1)
    for joint,link in (('left_joint7','left_link7'),('left_joint8','left_link8')):
        shifted=planner.fk.forward(dict(state,**{joint:state[joint]+.001}))
        axis=(shifted[link][:3,3]-frames[link][:3,3])/.001
        if abs(float(sum(axis*axis))-1.)>1e-8:raise ValueError('unit native prismatic axis required')
        ids=[]
        for alias,entry in engine.config['kinematics']['extra_links'].items():
            if entry['parent_link_name']==link and alias in engine.config['kinematics']['collision_link_names']:
                ids.extend(model.kinematics_config.get_sphere_index_from_link_name(alias).cpu().tolist())
        if not ids:raise ValueError('jaw collision spheres missing')
        delta=(positions-state[joint])[:,None,None]*engine.tensor_args.to_device(axis)[None,None,:]
        indices=torch.tensor(ids,device=spheres.device,dtype=torch.long)
        moved=spheres.index_select(2,indices)
        moved[...,:3]+=delta[None]
        moved[...,3]+=.000025+1e-6
        spheres.index_copy_(2,indices,moved)
    displacement=spheres.new_zeros((count,3))
    if centering is not None:
        vector=engine.tensor_args.to_device(centering['displacement_base_m'])
        span=float(torch.linalg.vector_norm(vector))
        if not 0<span<=.01:raise ValueError('bounded horizontal centering displacement required')
        moved=(centering['contact_start_m']-positions).clamp(0.,span)
        displacement=moved[:,None]*(vector/span)[None]
        ids=model.kinematics_config.get_sphere_index_from_link_name('world_cup')
        cup=spheres.index_select(2,ids)
        cup[...,:3]+=displacement[None,:,None,:]
        cup[...,3]+=.000026
        spheres.index_copy_(2,ids,cup)
        cup_world=engine.cup_contact.stationary_world_cost(cup).detach()
    else:
        cup_world=spheres.new_zeros((1,count))
    rollout=engine.generator.rollout_fn
    self_value=rollout.robot_self_collision_constraint(spheres).detach()
    environment=rollout.primitive_collision_constraint._cup_contact_original(spheres).detach()
    interaction=[]
    for indices,cost in rollout.primitive_collision_constraint._cup_contact_children:
        hand=spheres.index_select(2,indices).clone()
        hand[...,:3]-=displacement[None,:,None,:]
        if centering is not None:hand[...,3]+=.000025
        interaction.append(cost(hand).detach())
    claw,wrist=interaction
    bound=spheres.new_full((1,count),float('inf'))
    if bool((claw>0).any()):
        import json
        from pathlib import Path
        from dataclasses import asdict
        from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
        from cup_contact import sweep_contact_bound
        from motiongen_motion import normalized_fk_quaternion
        prepared=json.loads(certificate_path('a1_grasp_contact.json',engine.config['kinematics']['urdf_path']).read_text())
        if (prepared['cup_geometry_sha256']!=engine.cup_contact.geometry_digest
                or prepared['urdf_sha256']!=engine.cup_contact.model_report['kinematics']['urdf_sha256']
                or prepared['source_parts_sha256']!=engine.cup_contact.model_report['source_parts_sha256']):
            raise ValueError('jaw contact source identity differs')
        fk=engine.generator.compute_kinematics(engine._start(q)).ee_pose
        tool=RigidTransform(tuple(fk.position[0].tolist()),normalized_fk_quaternion(fk.quaternion[0].tolist()))
        first=engine.cup_contact.first_cup_pose
        cup=RigidTransform(tuple(first[:3]),tuple(first[3:])).compose(RigidTransform((0.,0.,-prepared['first_band_center_z_m'])))
        values=[]
        for gap,shift in zip(positions.cpu().tolist(),displacement.cpu().tolist()):
            translated=RigidTransform(tuple(a+b for a,b in zip(cup.position,shift)),cup.orientation_wxyz)
            values.append(sweep_contact_bound(prepared,asdict(tool.inverse().compose(translated)),[gap,gap])+.000051)
        bound=engine.tensor_args.to_device(values)[None]
    arrays=(self_value,environment,claw,wrist,cup_world)
    if any(not torch.isfinite(v).all() for v in arrays):raise ValueError('nonfinite GPU jaw sweep')
    claw_ok=(claw<=0)|(bound<=.003)
    result=dict(passed=bool((self_value<=0).all() and (environment<=0).all() and (wrist<=0).all() and (cup_world<=0).all() and claw_ok.all()),
        samples=count,self_max=float(self_value.max()),environment_max=float(environment.max()),
        cup_claw_max=float(claw.max()),cup_wrist_max=float(wrist.max()),cup_world_max=float(cup_world.max()),
        unresolved_claw_samples=int((~claw_ok).sum()),
        source_bound_used_samples=int(((claw>0)&claw_ok).sum()),
        maximum_jaw_step_m=.00005,continuous_translation_padding_m=.000026,
        start_m=start,end_m=end,wall_seconds=time.perf_counter()-began,
        scope='native GPU self/world; original cup interaction with source contact displacement bound; support checked separately')
    return result


def fixed_world_check(engine, planner, exported, cup_pose):
    """Static right arm versus external world; cup/fixed-robot is checked by CupContact.

    Base/table installation exclusion is unchanged. This does not certify
    fixed robot self pairs or the separate local support contacts.
    """
    import numpy as np
    from conservative_spheres import cover
    from motiongen_plan import world_config
    from curobo.geom.sdf.world import WorldCollisionConfig
    from curobo.geom.sdf.world_mesh import WorldMeshCollision
    from curobo.rollout.cost.primitive_collision_cost import PrimitiveCollisionCost,PrimitiveCollisionCostConfig
    started=time.perf_counter()
    world,_,_=world_config(planner,cup_pose)
    world.mesh=[mesh for mesh in world.mesh or [] if not mesh.name.startswith('motiongen_cup_band_')]
    checker=WorldMeshCollision(WorldCollisionConfig(tensor_args=engine.tensor_args,world_model=world,
        cache={'mesh':max(1,len(world.mesh)),'obb':max(1,len(world.cuboid or []))}))
    frames=planner.fk.forward({**planner.home_state,**engine.config['kinematics']['lock_joints']})
    rows=[];parts=[]
    for part in exported['parts']:
        if not part['link'].startswith('right_'):continue
        centers,radii,proof=cover(part['vertices'],faces=part['faces'],clip_boundary=True)
        if not proof['whole_solid_covered']:raise ValueError('fixed robot cover is incomplete')
        frame=frames[part['link']]
        rows.extend(np.column_stack((centers@frame[:3,:3].T+frame[:3,3],radii)).tolist())
        parts.append(dict(name=part['name'],spheres=len(radii)))
    if not rows:raise ValueError('missing static right arm')
    cost=PrimitiveCollisionCost(PrimitiveCollisionCostConfig(weight=1.,tensor_args=engine.tensor_args,
        world_coll_checker=checker,activation_distance=.0001,return_loss=True))
    value=cost(engine.tensor_args.to_device(rows)[None,None]).detach()
    if not engine.torch.isfinite(value).all():raise ValueError('nonfinite static world query')
    return dict(passed=bool((value<=0).all()),constraint_max=float(value.max()),parts=parts,
        wall_seconds=time.perf_counter()-started,scope='frozen right arm versus external world excluding cup; no fixed self-pair claim')


def boxes_separated(rotation, translation, half_a, half_b):
    """Conservative 15-axis OBB separation; works on NumPy or Torch batches."""
    r=abs(rotation)+1e-7
    clear=(abs(translation)>(half_a+(r*half_b[...,None,:]).sum(-1)+1e-6)).any(-1)
    projected=(translation[..., :,None]*rotation).sum(-2)
    clear|=(abs(projected)>(half_b+(r*half_a[..., :,None]).sum(-2)+1e-6)).any(-1)
    for i in range(3):
        for j in range(3):
            a,b=(i+1)%3,(i+2)%3;c,d=(j+1)%3,(j+2)%3
            distance=abs(translation[...,b]*rotation[...,a,j]-translation[...,a]*rotation[...,b,j])
            radius=half_a[...,a]*r[...,b,j]+half_a[...,b]*r[...,a,j]+half_b[...,c]*r[...,i,d]+half_b[...,d]*r[...,i,c]
            clear|=distance>radius+1e-6
    return clear


def same_support_source(a,b):
    # The sidecar location changes in immutable asset clones; its SHA and all
    # geometry, algorithm and policy fields must still match exactly.
    return {k:v for k,v in a.items() if k!='sidecar_path'}=={k:v for k,v in b.items() if k!='sidecar_path'}


class SupportBounds:
    """Native GPU FK plus conservative component boxes for the local support policy."""
    def __init__(self, engine, planner, urdf, *, active_joints=None):
        import json
        from pathlib import Path
        import numpy as np
        from curobo_planning import robot_config
        from curobo.types.robot import RobotConfig
        from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
        self.torch=engine.torch;self.args=engine.tensor_args
        self.support=planner.support_contact
        self.certificate=json.loads(certificate_path('a1_support_domain.json',urdf).read_text())
        if not same_support_source(self.certificate['support_identity'],self.support.identity):
            raise ValueError('support domain source identity differs')
        state={**planner.home_state,**engine.config['kinematics']['lock_joints']}
        config,identity=robot_config(urdf,planner.fk.movable,state)
        if active_joints is not None:
            active=list(active_joints)
            if len(set(active))!=len(active) or any(n not in identity['joints'] for n in active):
                raise ValueError('unique native active joints required')
            config['kinematics']['lock_joints']={n:v for n,v in state.items() if n not in active}
            config['kinematics']['cspace'].update(joint_names=active,
                retract_config=[max(identity['joints'][n]['lower'],min(identity['joints'][n]['upper'],state[n])) for n in active],
                null_space_weight=[1.]*len(active),cspace_distance_weight=[1.]*len(active))
        self.order=tuple(config['kinematics']['cspace']['joint_names'])
        self.indices={n:i for i,n in enumerate(self.order)}
        if self.certificate['urdf_sha256']!=identity['urdf_sha256']:raise ValueError('support URDF differs')
        expected={n:[j['lower'],j['upper']] for n,j in identity['joints'].items()}
        for proof in self.certificate['checks']:
            if (proof.get('reason')!='continuous_interval_separation' or proof.get('joint_domain')!=expected
                    or not math.isfinite(proof.get('minimum_separation_lower_bound_m',float('nan')))
                    or proof['minimum_separation_lower_bound_m']<=1e-7):
                raise ValueError('incomplete support domain certificate')
        self.fixed={n:v for n,v in state.items() if n not in self.indices}
        self.model=CudaRobotModel(RobotConfig.from_dict(config,self.args).kinematics)
        if tuple(self.model.joint_names)!=self.order:raise ValueError('support GPU joint ordering differs')
        self.boxes={};self.axes={};self.selected={}
        for name,mesh in self.support.meshes.items():
            rows=[];orientations=[]
            for index,ids in enumerate(mesh.components):
                points=mesh.triangles[ids].reshape(-1,3);origin=points.mean(0)
                _,axes=np.linalg.eigh((points-origin).T@(points-origin))
                local=(points-origin)@axes;lo,hi=local.min(0),local.max(0)
                rows.append(np.r_[origin+axes@((lo+hi)/2),(hi-lo)/2]);orientations.append(axes)
                if name in self.support.faces and frozenset(map(int,ids))==self.support.faces[name]:self.selected[name]=index
            self.boxes[name]=self.args.to_device(np.asarray(rows))
            self.axes[name]=self.args.to_device(np.asarray(orientations))
        if any(n not in self.selected for pair in self.support.pairs if pair[1].endswith('5') for n in pair):
            raise ValueError('support component identity missing')
        self.near_domains={}
        path=certificate_path('a1_support_near.json',urdf)
        if path.exists():
            asset=json.loads(path.read_text())
            if not same_support_source(asset['support_identity'],self.support.identity) or asset['urdf_sha256']!=identity['urdf_sha256']:
                raise ValueError('near-support source identity differs')
            for proof in asset['checks']:
                keys=proof['relative_joints'];pair=proof['pair'];indices=proof['component_pair']
                if tuple(pair) not in self.support.pairs or not pair[1].endswith('5'):
                    raise ValueError('unexpected near-support pair')
                wanted=[0.,0.,.0012+1e-9] if indices==[self.selected[n] for n in pair] else [0.,0.,0.]
                if proof.get('shift_m')!=wanted:raise ValueError('support component translation policy differs')
                offset=0 if pair[0].startswith('left_') else 10
                side='left' if offset==0 else 'right'
                if keys!=[f'{side}_joint{offset+i}' for i in (3,4,5)]:raise ValueError('incomplete relative support domain')
                cells=[proof] if proof.get('certified') else proof.get('certified_cells',[])
                for cell in cells:
                    lower=cell.get('minimum_separation_lower_bound_m',float('nan'))
                    if not math.isfinite(lower) or lower<=1e-7:raise ValueError('unproven support cell')
                    if any(not expected[n][0]<=cell['joint_domain'][n][0]<=cell['joint_domain'][n][1]<=expected[n][1] for n in keys):
                        raise ValueError('support cell outside original joint limits')
                if cells:
                    tensor=self.torch.tensor([[cell['joint_domain'][n] for n in keys] for cell in cells],device=self.args.device,dtype=self.torch.float64)
                    self.near_domains[(*pair,*indices)]=(keys,tensor)


    def check(self,q):
        torch=self.torch;started=time.perf_counter()
        values=self.args.to_device(q)
        if values.ndim!=2 or values.shape[1]!=len(self.order) or not torch.isfinite(values).all():raise ValueError('finite support path required')
        poses=self.model.get_state(values).link_poses
        checks=[]
        for a,b in self.support.pairs:
            if b.endswith('3'):
                proof=next(row for row in self.certificate['checks'] if row['pair']==[a,b])
                if proof.get('certified') is not True:raise ValueError('uncertified support pair')
                # Common ancestors cancel; check every relative coordinate of the offline proof.
                ok=torch.ones(len(values),device=values.device,dtype=torch.bool)
                for name in proof['relative_joints']:
                    lo,hi=proof['joint_domain'][name]
                    x=values[:,self.indices[name]] if name in self.indices else values.new_full((len(values),),self.fixed[name])
                    ok&=(x>=lo)&(x<=hi)
                checks.append(dict(pair=[a,b],passed=bool(ok.all()),method='offline continuous source-surface domain / GPU bound check'))
                continue
            relative=torch.linalg.inv(poses[a].get_matrix())@poses[b].get_matrix()
            aa,bb=self.boxes[a],self.boxes[b]
            center=torch.einsum('nij,kj->nki',relative[:,:3,:3],bb[:,:3])+relative[:,:3,3][:,None]
            axes_a,axes_b=self.axes[a],self.axes[b]
            rotation=torch.einsum('aki,nkl,blj->nabij',axes_a,relative[:,:3,:3],axes_b)
            def separated(shift):
                translation=torch.einsum('aki,nabk->nabi',axes_a,center[:,None]-aa[None,:,None,:3]+shift)
                return boxes_separated(rotation,translation,aa[None,:,None,3:],bb[None,None,:,3:])
            clear=separated(0.)
            ai,bi=self.selected[a],self.selected[b]
            shift=values.new_tensor([0.,0.,.0012+1e-9])
            clear[:,ai,bi]=separated(shift)[:,ai,bi]
            for (left,right,ci,cj),(keys,domain) in self.near_domains.items():
                if (left,right)!=(a,b):continue
                coordinates=torch.stack([values[:,self.indices[n]].double() if n in self.indices else
                    torch.full((len(values),),self.fixed[n],device=values.device,dtype=torch.float64) for n in keys],dim=-1)
                inside=((coordinates[:,None]>=domain[None,:,:,0])&(coordinates[:,None]<=domain[None,:,:,1])).all(-1).any(-1)
                clear[:,ci,cj]|=inside
            okay=clear.all(dim=(1,2))
            checks.append(dict(pair=[a,b],passed=bool(okay.all()),unresolved_samples=int((~okay).sum()),
                component_counts=[len(aa),len(bb)],selected_components=[ai,bi],
                unresolved_pairs=(~clear).any(0).nonzero().cpu().tolist(),
                unresolved_joint_min=values[~okay].min(0).values.cpu().tolist() if (~okay).any() else None,
                unresolved_joint_max=values[~okay].max(0).values.cpu().tolist() if (~okay).any() else None,method='conservative oriented component boxes / shifted support component only'))
        return dict(passed=all(row['passed'] for row in checks),checks=checks,wall_seconds=time.perf_counter()-started)


def bind_runtime_support_gpu(runtime, urdf):
    """Reuse the accepted support policy on both arms' measured joints at each substep."""
    import torch
    from types import SimpleNamespace
    from curobo.types.base import TensorDeviceType
    checker=runtime.zero_collision
    if checker.support_contact is None:raise ValueError('runtime support identity missing')
    robot=runtime.env.scene['robot']
    state=dict(zip(robot.joint_names,robot.data.joint_pos[0].tolist(),strict=True))
    active=tuple(f'{side}_joint{i+offset}' for side,offset in (('left',0),('right',10)) for i in range(1,7))
    engine=SimpleNamespace(torch=torch,tensor_args=TensorDeviceType(device=torch.device('cuda:0')),
        config={'kinematics':{'lock_joints':state}})
    planner=SimpleNamespace(home_state=state,support_contact=checker.support_contact,fk=SimpleNamespace(movable=tuple(state)))
    bounds=SupportBounds(engine,planner,urdf,active_joints=active)
    current={};original=checker.check
    def support_check(frames):
        result=bounds.check([[current[n] for n in active]])
        return [dict(pair=tuple(row['pair']),ok=row['passed'],contact=None,over_limit=None,outside_support=None,
            limit_m=.0012,metric='separation_along_link2_positive_z',gpu_evidence=row) for row in result['checks']]
    def check(positions):
        current.clear();current.update(positions)
        return original(positions)
    checker.support_contact.check=support_check
    checker.check=check
    return dict(backend='existing GPU SupportBounds',active_joints=list(active),support_identity=checker.support_contact.identity,
                cpu_support_triangle_checks=False,remaining_convex_self_checks_unchanged=True)


def fixed_self_check(engine, planner, exported):
    """Conservative OBB acceptance of the native fixed self-pair list."""
    import numpy as np
    began=time.perf_counter();whole=planner.geo.whole
    parts={p['name']:p for p in exported['parts']}
    state={**planner.home_state,**engine.config['kinematics']['lock_joints']};frames=planner.fk.forward(state)
    selected={};centers=[];axes=[];halves=[];names=[]
    for index,(link,name,*_) in enumerate(whole.parts):
        if link.startswith('left_'):continue
        points=np.asarray(parts[name]['vertices']);origin=points.mean(0)
        _,basis=np.linalg.eigh((points-origin).T@(points-origin));local=(points-origin)@basis
        lo,hi=local.min(0),local.max(0);frame=frames[link]
        selected[index]=len(centers);centers.append(frame[:3,:3]@(origin+basis@((lo+hi)/2))+frame[:3,3]);axes.append(frame[:3,:3]@basis);halves.append((hi-lo)/2);names.append(name)
    pairs=[]
    for a,b in whole._self_indices:
        if a not in selected or b not in selected:continue
        if frozenset((whole.parts[a][0],whole.parts[b][0])) in planner.support_contact.delegated_pairs:continue
        pairs.append((selected[a],selected[b]))
    if not pairs:raise ValueError('empty fixed self-pair coverage')
    torch=engine.torch;args=engine.tensor_args
    c,r,h=(args.to_device(np.asarray(x)) for x in (centers,axes,halves))
    ids=torch.tensor(pairs,device=c.device,dtype=torch.long);a,b=ids[:,0],ids[:,1]
    ra=r[a].transpose(-1,-2);relative=ra@r[b];translation=(ra@(c[b]-c[a])[...,None]).squeeze(-1)
    clear=boxes_separated(relative,translation,h[a],h[b])
    import json
    from pathlib import Path
    certificate_file=certificate_path('a1_fixed_domain.json',engine.config['kinematics']['urdf_path'])
    certified=[]
    if certificate_file.exists():
        certificate=json.loads(certificate_file.read_text())
        report=engine.cup_contact.model_report
        if (certificate.get('urdf_sha256')!=exported['urdf_sha256']
                or certificate.get('source_parts_sha256')!=report['source_parts_sha256']
                or certificate.get('required_clearance_m')!=.0001):
            raise ValueError('fixed-domain source identity/clearance differs')
        proofs={frozenset(row['parts']):row for row in certificate['checks'] if row.get('certified')}
        for index in (~clear).nonzero().reshape(-1).cpu().tolist():
            pair=[names[pairs[index][0]],names[pairs[index][1]]];proof=proofs.get(frozenset(pair))
            if proof is None:continue
            lower=proof.get('minimum_separation_lower_bound_m',float('nan'))
            if proof.get('reason')!='continuous_interval_separation' or not math.isfinite(lower) or lower<=1e-7:
                raise ValueError('invalid fixed-domain separation proof')
            keys=proof['relative_joints'];limits=proof['joint_domain']
            q=args.to_device([state[n] for n in keys]);lo=args.to_device([limits[n][0] for n in keys]);hi=args.to_device([limits[n][1] for n in keys])
            if bool(((q>=lo)&(q<=hi)).all()):clear[index]=True;certified.append(pair)
    bad=(~clear).nonzero().reshape(-1).cpu().tolist()
    return dict(passed=not bad,pair_count=len(pairs),domain_certified_pairs=certified,unresolved_pairs=[[names[pairs[i][0]],names[pairs[i][1]]] for i in bad],
        wall_seconds=time.perf_counter()-began,scope='native fixed self-pair list; local support pairs separately checked')


def task_check(engine, planner, stages, closing):
    """GPU FK task constraints on the native path; no retimed collision recheck.

    Cup orientation here is derived from a frozen rigid tool/cup relation.
    Physical cup tilt belongs to native replay acceptance, not this FK check.
    """
    import numpy as np
    from itertools import product
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from gripper_distance_plan import HOLDING_STAGES
    torch=engine.torch;args=engine.tensor_args;started=time.perf_counter()
    def matrix(pose):
        result=np.eye(4);result[:3,:3]=np.column_stack([pose.rotate(tuple(float(i==j) for i in range(3))) for j in range(3)]);result[:3,3]=pose.position
        return result
    reference=planner.fk.forward(closing['terminal_state'])
    mount=matrix(planner.geo.mount);goal=mount@reference['left_link6']
    held=matrix(RigidTransform(**closing['world_from_cup']));relation=np.linalg.inv(goal)@held
    outward=np.asarray(planner.geo.scene.world_from_support.rotate((0.,-1.,0.)));outward[2]=0.;outward/=np.linalg.norm(outward)
    machine=[]
    for shape in planner.geo.scene.shapes:
        corners=np.array([shape.world_from_shape.transform_point(tuple(sign*d/2 for sign,d in zip(signs,shape.dimensions_m))) for signs in product((-1,1),repeat=3)])
        machine.append([float((corners@outward).max()),float(corners[:,2].min()),float(corners[:,2].max())])
    obstacles=args.to_device(machine);direction=args.to_device(outward)
    bodies=[];cup_bodies=[]
    for part in planner.parts:
        if part.link in ('left_link6','left_link7','left_link8'):
            relative=np.linalg.inv(reference['left_link6'])@reference[part.link]
            points=np.asarray(part.vertices)@relative[:3,:3].T+relative[:3,3]
            bodies.append(args.to_device(points))
    for band in planner.cup.bands:
        points=np.array([(sx*band.radius_m,sy*band.radius_m,z) for sx,sy in product((-1,1),repeat=2) for z in (band.z_min_m,band.z_max_m)])
        cup_bodies.append(args.to_device(points@relation[:3,:3].T+relation[:3,3]))
    tray_front=max(float(np.dot(p,outward)) for p in planner.geo.scene.support_polygon_world_m)
    tray_exit=math.inf
    maximum=0.;minimum=math.inf;cup_minimum=math.inf;held_count=cross_count=0;cross_positions=[]
    for stage in stages:
        if stage['name'] not in HOLDING_STAGES:continue
        q=args.to_device([r['left_arm_joints'] for r in stage['rows']])
        poses=engine.generator.kinematics.get_state(q).ee_pose.get_matrix()
        world=args.to_device(mount)[None]@poses
        delta=args.to_device(goal[:3,:3]).T[None]@world[:,:3,:3]
        angle=torch.acos(((delta.diagonal(dim1=-2,dim2=-1).sum(-1)-1)/2).clamp(-1.,1.))
        maximum=max(maximum,float(angle.max()));held_count+=len(q)
        if stage.get('substage')=='cross':
            cross_positions.append(world[:,:3,3])
            for is_cup,points in [(False,p) for p in bodies]+[(True,p) for p in cup_bodies]:
                positions=torch.einsum('nij,kj->nki',world[:,:3,:3],points)+world[:,:3,3][:,None]
                near=(positions@direction).min(-1).values;low=positions[:,:,2].min(-1).values;high=positions[:,:,2].max(-1).values
                distance=torch.maximum(torch.maximum(low[:,None]-obstacles[None,:,2],obstacles[None,:,1]-high[:,None]),near[:,None]-obstacles[None,:,0])
                if is_cup:
                    cup_minimum=min(cup_minimum,float(distance.min()))
                    tray_exit=min(tray_exit,float(near.min())-tray_front)
                else:minimum=min(minimum,float(distance.min()))
            cross_count+=len(q)
    if not held_count or not cross_count or not all(math.isfinite(v) for v in (minimum,cup_minimum)):raise ValueError('missing held/cross task coverage')
    from cup_contact import ALLOWANCE_M
    from motiongen_motion import segment_errors,TASK_POSITION_TOLERANCE_M
    positions=torch.cat(cross_positions)
    cross_error=float(segment_errors(positions,positions[0],positions[-1]).max())
    passed=(maximum<=math.radians(.5)+1e-6 and minimum>=.017-1e-6
            and cup_minimum>=-ALLOWANCE_M-1e-6 and tray_exit>=-1e-6
            and cross_error<=TASK_POSITION_TOLERANCE_M+1e-7)
    return dict(passed=passed,max_ee_orientation_error_rad=maximum,max_cup_orientation_error_rad=maximum,
        orientation_error_scope='planned_tool_and_frozen_rigid_attachment',physical_cup_tilt_observed=False,
        orientation_tolerance_rad=math.radians(.5),maximum_cross_line_error_m=cross_error,min_cross_clearance_m=minimum,cross_clearance_required_m=.017,
        cross_clearance_scope='original hand geometry only; separate cup allowance',min_cross_cup_clearance_m=cup_minimum,cup_environment_allowance_m=ALLOWANCE_M,
        checked_held_samples=held_count,checked_cross_samples=cross_count,wall_seconds=time.perf_counter()-started,
        min_original_cup_tray_exit_clearance_m=tray_exit,tray_exit_required=True,
        backend='native GPU FK and original geometry projections',retimed_collision_audit=False)


def closing_geometry(engine, planner, state, cup):
    """Reuse the pinned nominal contact geometry at the current observed pose."""
    import json
    from pathlib import Path
    from dataclasses import asdict
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from motiongen_motion import task_pose_error
    prepared=json.loads(certificate_path('a1_grasp_contact.json',engine.config['kinematics']['urdf_path']).read_text())
    tool=planner.geo.mount.compose(planner.fk.pose(planner.fk.forward(state)['left_link6']))
    actual=cup.inverse().compose(tool);expected=prepared['original_cup_from_tool']
    _,_,valid=task_pose_error(actual.position,actual.orientation_wxyz,expected['position'],expected['orientation_wxyz'])
    if not valid:raise ValueError('current grasp is outside the prepared nominal contact pose')
    gap=prepared['terminal_jaw_position_m']
    shift=RigidTransform(**prepared['original_cup_from_held_cup'])
    held=cup.compose(shift)
    vector=planner.geo.mount.inverse().rotate(cup.rotate(shift.position))
    q=[state[f'left_joint{i}'] for i in range(1,7)]
    sweep=jaw_sweep(engine,planner,q,state['left_joint7'],gap,centering=dict(
        displacement_base_m=vector,contact_start_m=max(prepared['contact_geometry_stops_m'].values())))
    if not sweep['passed']:raise ValueError('GPU closing/centering acceptance failed: '+str(sweep))
    return dict(passed=True,terminal_state=dict(state,left_joint7=gap,left_joint8=gap),world_from_cup=asdict(held),
        initial_world_from_cup=asdict(cup),gpu_sweep=sweep,contact_geometry_stops_m=prepared['contact_geometry_stops_m'],
        scope='pinned nominal contact geometry with bounded current grasp pose and GPU symmetric centering sweep; no physical force claim')
