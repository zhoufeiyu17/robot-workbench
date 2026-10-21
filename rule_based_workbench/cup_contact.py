"""Pair-scoped 3mm claw/cup allowance using cuRobo 0.7.8 native mesh queries.

The original world checks all robot spheres against every non-cup obstacle.
Separate native cup costs keep wrist/arms strict and permit bounded claw contact.
Negative eta shifts the native activation boundary without making small sphere
radii negative (negative radii would disable collision checking entirely).
"""
from dataclasses import fields
from copy import deepcopy

CLAW_LINKS = frozenset(('left_link7', 'left_link8'))
ALLOWANCE_M = .003
NUMERICAL_EPS_M = 1e-7


def permits_claw_contact(link, depths):
    import math
    values=tuple(depths)
    return link in CLAW_LINKS and bool(values) and all(math.isfinite(v) and 0<=v<=ALLOWANCE_M+NUMERICAL_EPS_M for v in values)


def cup_meshes(world):
    meshes=[mesh for mesh in world.mesh or [] if mesh.name.startswith('motiongen_cup_band_')]
    if not meshes or len({mesh.name for mesh in meshes})!=len(meshes):
        raise ValueError('unique explicit cup-band meshes required for scoped contact')
    return meshes


def cup_geometry_digest(world):
    from curobo_robot_model import digest
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    meshes=cup_meshes(world)
    first=RigidTransform(tuple(meshes[0].pose[:3]),tuple(meshes[0].pose[3:])).inverse()
    rows=[]
    for mesh in meshes:
        pose=first.compose(RigidTransform(tuple(mesh.pose[:3]),tuple(mesh.pose[3:])))
        rows.append(dict(name=mesh.name,vertices=[[float(x) for x in row] for row in mesh.vertices],
            faces=[[int(x) for x in row] for row in mesh.faces],scale=mesh.scale,
            first_from_band=[round(float(v),9)+0. for v in (*pose.position,*pose.orientation_wxyz)]))
    return digest(rows)


def fixed_contact_bound(prepared,tool_from_cup,jaws):
    """Lipschitz bound for a prepared fixed claw/cup contact; no mesh query."""
    import math
    reference=prepared['tool_from_held_cup']
    a,b=reference['orientation_wxyz'],tool_from_cup['orientation_wxyz']
    numbers=[*a,*b,*reference['position'],*tool_from_cup['position'],*jaws,
        prepared['terminal_jaw_position_m'],prepared['maximum_claw_penetration_m'],
        prepared['cup_radius_from_origin_m'],prepared['numerical_contact_reserve_m']]
    if (len(a)!=4 or len(b)!=4 or len(reference['position'])!=3 or len(tool_from_cup['position'])!=3 or len(jaws)!=2
            or not all(math.isfinite(v) for v in numbers)):
        raise ValueError('finite fixed-contact pose and two jaws required')
    if not (0<=prepared['maximum_claw_penetration_m']<=ALLOWANCE_M and prepared['cup_radius_from_origin_m']>0
            and prepared['numerical_contact_reserve_m']>=1e-5 and prepared['terminal_jaw_position_m']>=0):
        raise ValueError('invalid fixed-contact geometric bound')
    na,nb=math.sqrt(sum(v*v for v in a)),math.sqrt(sum(v*v for v in b))
    if abs(na-1)>1e-4 or abs(nb-1)>1e-4:raise ValueError('unit fixed-contact rotations required')
    angle=2*math.acos(min(1.,abs(sum(x*y for x,y in zip(a,b)))/(na*nb)))
    displacement=math.dist(reference['position'],tool_from_cup['position'])+2*prepared['cup_radius_from_origin_m']*math.sin(angle/2)
    jaw=max(abs(v-prepared['terminal_jaw_position_m']) for v in jaws)
    return prepared['maximum_claw_penetration_m']+prepared['numerical_contact_reserve_m']+displacement+jaw


def sweep_contact_bound(prepared,tool_from_cup,jaws):
    """Use the nearest offline source sample, retaining the3mm bound online."""
    samples=prepared.get('jaw_sweep_source_samples')
    if not samples:return fixed_contact_bound(prepared,tool_from_cup,jaws)
    if not 1<=len(samples)<=500:raise ValueError('bounded source jaw samples required')
    sample=min(samples,key=lambda row:max(abs(q-row['jaw_position_m']) for q in jaws))
    reference={**prepared,'tool_from_held_cup':sample['tool_from_cup'],
               'terminal_jaw_position_m':sample['jaw_position_m'],
               'maximum_claw_penetration_m':sample['maximum_claw_penetration_m']}
    return fixed_contact_bound(reference,tool_from_cup,jaws)


def fixed_wrist_clearance(prepared,tool_from_cup,jaws):
    import math
    minimum=prepared.get('minimum_wrist_clearance_m',float('nan'))
    if (prepared.get('wrist_source_separation_proven') is not True
            or not math.isfinite(minimum) or minimum<=1e-5):
        raise ValueError('positive prepared wrist/cup separation required')
    # The existing bound also includes jaw displacement, conservatively, though
    # the wrist itself does not move as the jaws close.
    displacement=fixed_contact_bound(prepared,tool_from_cup,jaws)-prepared['maximum_claw_penetration_m']
    return minimum-displacement


def split_indices(config, model):
    selected=[]
    for alias,link in config['kinematics']['extra_links'].items():
        if link['parent_link_name'] in CLAW_LINKS and alias in config['kinematics']['collision_link_names']:
            selected.extend(model.kinematics_config.get_sphere_index_from_link_name(alias).cpu().tolist())
    if not selected or len(set(selected))!=len(selected):raise ValueError('unique claw sphere indices required')
    count=model.kinematics_config.total_spheres
    return sorted(selected),[i for i in range(count) if i not in set(selected)]


def install_cost(cost, checker, claw_indices, strict_indices):
    import torch
    from curobo.rollout.cost.primitive_collision_cost import PrimitiveCollisionCost,PrimitiveCollisionCostConfig
    original=cost.forward
    children=[]
    for indices,allowed in ((claw_indices,True),(strict_indices,False)):
        settings={field.name:getattr(cost,field.name) for field in fields(PrimitiveCollisionCostConfig)}
        settings.update(world_coll_checker=checker)
        if allowed:settings['activation_distance']=-(ALLOWANCE_M+NUMERICAL_EPS_M)
        child=PrimitiveCollisionCost(PrimitiveCollisionCostConfig(**settings))
        child.weight=cost.weight
        children.append((torch.tensor(indices,device=cost.tensor_args.device,dtype=torch.long),child))
    def forward(spheres, env_query_idx=None):
        value=original(spheres,env_query_idx)
        for indices,child in children:value=value+child(spheres.index_select(2,indices),env_query_idx)
        return value
    cost.forward=forward
    cost._cup_contact_children=children
    cost._cup_contact_original=original


class CupContact:
    def allow_claw_contact(self, allowed):
        """Reuse captured native buffers; released-hand paths must avoid the cup."""
        if type(allowed) is not bool:raise ValueError('explicit claw contact phase required')
        seen=set()
        for rollout in [self.generator.rollout_fn,*self.generator.get_all_rollout_instances()]:
            for name in ('primitive_collision_cost','primitive_collision_constraint'):
                cost=getattr(rollout,name,None)
                if cost is not None and id(cost) not in seen and hasattr(cost,'_cup_contact_children'):
                    seen.add(id(cost))
                    activation=cost._cup_contact_children[0][1].activation_distance
                    if allowed: activation.fill_(-(ALLOWANCE_M+NUMERICAL_EPS_M))
                    # Restore native cost warning distance; hard constraints remain zero.
                    else: activation.copy_(cost.activation_distance)
        if not seen:raise ValueError('native claw/cup cost missing')
        self.report['current_claw_allowance_m']=ALLOWANCE_M if allowed else 0.

    def _bind_inward_envelope(self,world,prepared=None):
        from cup_envelope import prepare,validate_prepared
        selected=prepare(world) if prepared is None else prepared
        validate_prepared(selected,cup_geometry_digest(world))
        self.inward_envelope=deepcopy(selected)

    def __init__(self, generator, config, world, model_report=None, *, prepared_cup_envelope=None):
        from curobo.geom.sdf.world import WorldCollisionConfig
        from curobo.geom.sdf.world_mesh import WorldMeshCollision
        from curobo.geom.types import WorldConfig
        self.model_report=model_report
        self.generator=generator
        self.inward_proxy=bool(model_report and model_report.get('cup_proxy_mode')=='inward_surface_3mm')
        if self.inward_proxy:self._bind_inward_envelope(world,prepared_cup_envelope)
        meshes=cup_meshes(world)
        self.names=tuple(mesh.name for mesh in meshes)
        self.main=generator.world_coll_checker
        self.checker=WorldMeshCollision(WorldCollisionConfig(tensor_args=generator.tensor_args,
            world_model=WorldConfig(mesh=meshes),cache={'mesh':len(meshes),'obb':1}))
        self.claw_indices,self.strict_indices=split_indices(config,generator.kinematics)
        self.wrist_indices=[]
        for alias,link in config['kinematics']['extra_links'].items():
            if link['parent_link_name']=='left_link6' and alias in config['kinematics']['collision_link_names']:
                self.wrist_indices.extend(generator.kinematics.kinematics_config.get_sphere_index_from_link_name(alias).cpu().tolist())
        if not self.wrist_indices:raise ValueError('fixed wrist coverage required')
        from cup_envelope import cylinder_sagitta
        self.wrist_mesh_padding=max(cylinder_sagitta(mesh) for mesh in meshes)
        if self.inward_proxy:
            import torch
            from curobo.rollout.cost.primitive_collision_cost import PrimitiveCollisionCost,PrimitiveCollisionCostConfig
            model=generator.kinematics.kinematics_config
            fixed=set(model.get_sphere_index_from_link_name('world_cup').cpu().tolist())
            moving=torch.tensor([i for i in range(model.total_spheres) if i not in fixed],device=generator.tensor_args.device,dtype=torch.long)
            self.stationary_world_cost=PrimitiveCollisionCost(PrimitiveCollisionCostConfig(
                weight=1.,tensor_args=generator.tensor_args,world_coll_checker=self.main,
                activation_distance=0.,return_loss=True))
            installed=set()
            for rollout in [generator.rollout_fn,*generator.get_all_rollout_instances()]:
                if getattr(rollout,'cuda_graph_instance',False):raise ValueError('install fixed-world filter before warmup')
                for name in ('primitive_collision_cost','primitive_collision_constraint'):
                    cost=getattr(rollout,name,None)
                    if cost is not None and id(cost) not in installed:
                        installed.add(id(cost));original=cost.forward
                        def forward(spheres,env_query_idx=None,original=original):
                            return original(spheres.index_select(2,moving),env_query_idx)
                        cost.forward=forward
                        install_cost(cost,self.checker,self.claw_indices,self.wrist_indices)
        seen=set()
        for rollout in ([] if self.inward_proxy else [generator.rollout_fn,*generator.get_all_rollout_instances()]):
            if getattr(rollout,'cuda_graph_instance',False):raise ValueError('install contact costs before warmup')
            for name in ('primitive_collision_cost','primitive_collision_constraint'):
                cost=getattr(rollout,name,None)
                if cost is not None and id(cost) not in seen:
                    seen.add(id(cost));install_cost(cost,self.checker,self.claw_indices,self.strict_indices)
        if not seen and not self.inward_proxy:raise ValueError('no native world collision cost installed')
        self.report=dict(allowance_m=ALLOWANCE_M,numerical_epsilon_m=NUMERICAL_EPS_M,
            claw_links=sorted(CLAW_LINKS),claw_spheres=len(self.claw_indices),installed_costs=len(seen),
            backend='native_mesh_negative_activation_boundary',wrist_and_arm_cup_allowance_m=0.,
            claw_other_obstacle_allowance_m=0.)
        if self.inward_proxy:self.report.update(backend='native_self_and_world_inward_cup_proxy',external_query_allowance_m=0.,external_geometry_omission_m=.006,minimum_surface_inset_m=.003,external_pair_scope='all',installed_interaction_costs=len(installed),hand_interaction_geometry='original unshrunk cup')
        self.held_spheres=None;self.held_locks=None
        self.update(world)

    def update(self,world):
        from curobo.types.math import Pose
        meshes=cup_meshes(world)
        if tuple(mesh.name for mesh in meshes)!=self.names:raise ValueError('cup contact topology changed')
        geometry=cup_geometry_digest(world)
        if hasattr(self,'geometry_digest') and geometry!=self.geometry_digest:
            raise ValueError('cup geometry changed; recreate the scoped collision checker')
        if self.inward_proxy:
            from cup_envelope import validate_prepared
            validate_prepared(self.inward_envelope,geometry)
        self.geometry_digest=geometry;self.first_cup_pose=list(meshes[0].pose)
        self.active=True;self.held_spheres=None;self.held_locks=None
        for mesh in meshes:
            self.main.enable_obstacle(mesh.name,False)
            self.checker.update_obstacle_pose(mesh.name,Pose.from_list(mesh.pose,tensor_args=self.checker.tensor_args))
            self.checker.enable_obstacle(mesh.name,True)
        if self.inward_proxy:
            from cup_envelope import transformed_spheres
            from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
            prepared=self.inward_envelope
            first=RigidTransform(tuple(self.first_cup_pose[:3]),tuple(self.first_cup_pose[3:]))
            rows=transformed_spheres(prepared,self.geometry_digest,first,self.model_report['cup_active_spheres'])
            self.world_proxy_spheres=self.generator.tensor_args.to_device(rows)
            # This cup is fixed relative to the base while arms move. Validate
            # against static world once; its constant soft cost must not drive
            # the arm optimizer. Carried cup spheres remain in every query.
            value=self.stationary_world_cost(self.world_proxy_spheres[None,None]).detach()
            import torch
            if not torch.isfinite(value).all().item() or value.max().item()>0:
                raise ValueError('observed/placed inward cup collides with the static world')
            self.restore_world_proxy()

    def restore_world_proxy(self):
        if self.inward_proxy and self.active:
            self.generator.attach_spheres_to_robot(sphere_tensor=self.world_proxy_spheres,link_name='world_cup')

    def set_active(self,active):
        self.active=bool(active)
        for name in self.names:self.checker.enable_obstacle(name,bool(active))
        if self.inward_proxy:
            if active:self.restore_world_proxy()
            else:self.generator.detach_spheres_from_robot(link_name='world_cup')

    def certify_held(self,engine,q,prepared=None):
        import torch
        from curobo.rollout.cost.primitive_collision_cost import PrimitiveCollisionCost,PrimitiveCollisionCostConfig
        selected=getattr(engine,'grasp_contact_certificate',None)
        if selected is None or prepared!=selected:
            raise ValueError('held contact requires this request selected-pose source certificate')
        if (self.model_report is None or prepared.get('urdf_sha256')!=self.model_report['kinematics']['urdf_sha256']
                or prepared.get('source_parts_sha256')!=self.model_report['source_parts_sha256']
                or prepared.get('cup_geometry_sha256')!=self.geometry_digest
                or prepared.get('claw_cup_allowance_m')!=ALLOWANCE_M or prepared.get('passed') is not True):
            raise ValueError('selected fixed-contact geometry identity differs')
        if not self.active:raise ValueError('held-contact certification requires the current cup world pose')
        local=engine.generator.kinematics.kinematics_config.get_link_spheres('attached_object')
        if not (local[:,3]>0).any().item():raise ValueError('attach the cup before certifying its fixed relative contact')
        query=engine.generator.kinematics.get_state(engine.tensor_args.to_device([q])).get_link_spheres().unsqueeze(1)
        # Wrist and held cup are rigid relative to left_link6. Prove this pair
        # once with the original cup mesh rather than two outward sphere covers.
        wrist=PrimitiveCollisionCost(PrimitiveCollisionCostConfig(weight=1.,tensor_args=engine.tensor_args,
            world_coll_checker=self.checker,activation_distance=self.wrist_mesh_padding+NUMERICAL_EPS_M,return_loss=True))
        wrist_value=wrist(query.index_select(2,torch.tensor(self.wrist_indices,device=query.device,dtype=torch.long))).detach()
        if not torch.isfinite(wrist_value).all().item():raise ValueError('nonfinite wrist/cup GPU query')
        wrist_evidence=dict(method='native cup mesh / conservative wrist spheres',constraint_max=float(wrist_value.max()))
        indices=torch.tensor(self.claw_indices,device=query.device,dtype=torch.long)
        cost=PrimitiveCollisionCost(PrimitiveCollisionCostConfig(weight=1.,tensor_args=engine.tensor_args,
            world_coll_checker=self.checker,activation_distance=-(ALLOWANCE_M+NUMERICAL_EPS_M),return_loss=True))
        value=cost(query.index_select(2,indices)).detach()
        if not torch.isfinite(value).all().item():raise ValueError('nonfinite held claw/cup contact')
        evidence=dict(method='conservative GPU claw spheres',constraint_max=float(value.max()))
        if value.max().item()>0 or wrist_value.max().item()>0:
            if prepared is None or self.model_report is None:raise ValueError('held claw/cup contact exceeds 3mm at attachment')
            if (prepared.get('urdf_sha256')!=self.model_report['kinematics']['urdf_sha256']
                    or prepared.get('source_parts_sha256')!=self.model_report['source_parts_sha256']
                    or prepared.get('cup_geometry_sha256')!=self.geometry_digest
                    or prepared.get('claw_cup_allowance_m')!=ALLOWANCE_M
                    or prepared.get('passed') is not True):
                raise ValueError('prepared fixed-contact geometry identity differs')
            from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
            from dataclasses import asdict
            first=RigidTransform(tuple(self.first_cup_pose[:3]),tuple(self.first_cup_pose[3:]))
            cup=first.compose(RigidTransform((0.,0.,-prepared['first_band_center_z_m'])))
            fk=engine.generator.compute_kinematics(engine._start(q)).ee_pose
            from motiongen_motion import normalized_fk_quaternion
            tool=RigidTransform(tuple(fk.position[0].cpu().tolist()),normalized_fk_quaternion(fk.quaternion[0].cpu().tolist()))
            relation=asdict(tool.inverse().compose(cup))
            bound=fixed_contact_bound(prepared,relation,[engine.config['kinematics']['lock_joints'][n] for n in ('left_joint7','left_joint8')])
            if bound>ALLOWANCE_M:raise ValueError('prepared fixed claw/cup bound exceeds 3mm')
            if wrist_value.max().item()>0:
                lower=fixed_wrist_clearance(prepared,relation,[engine.config['kinematics']['lock_joints'][n] for n in ('left_joint7','left_joint8')])
                if lower<=0:raise ValueError('prepared fixed wrist/cup separation exhausted')
                wrist_evidence=dict(method='offline source separation plus rigid displacement bound',minimum_clearance_bound_m=lower)
            evidence=dict(method='prepared source geometry plus rigid displacement bound',maximum_penetration_bound_m=bound)
        self.held_spheres=local.detach().clone();self.held_locks=dict(engine.config['kinematics']['lock_joints'])
        self.set_active(False)
        return dict(passed=True,allowance_m=ALLOWANCE_M,evidence=evidence,wrist_cup=dict(passed=True,allowance_m=0.,evidence=wrist_evidence),scope='fixed claw/cup relative transform, invalidated by attachment or locked-joint changes')

    def assert_ready(self,engine):
        import torch
        local=engine.generator.kinematics.kinematics_config.get_link_spheres('attached_object')
        attached=(local[:,3]>0).any().item()
        if self.inward_proxy:
            world=self.generator.kinematics.kinematics_config.get_link_spheres('world_cup')
            world_active=(world[:,3]>0).any().item()
            if world_active==attached:raise ValueError('exactly one cup proxy must be active')
            if world_active and not torch.equal(world,self.world_proxy_spheres):raise ValueError('world cup proxy differs from the current observation')
        if attached:
            if (self.active or self.held_spheres is None or not torch.equal(local,self.held_spheres)
                    or self.held_locks!=engine.config['kinematics']['lock_joints']):
                raise ValueError('attached cup lacks a current 3mm claw-contact certificate')
        elif not self.active:
            raise ValueError('restore cup world collision after detaching before planning')

    def assert_unattached(self,engine):
        local=engine.generator.kinematics.kinematics_config.get_link_spheres('attached_object')
        if (local[:,3]>0).any().item():raise ValueError('detach cup before changing world or locked jaws')
