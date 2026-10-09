"""Actual cuRobo MotionGen calls for a frozen robot/world; no CPU planner fallback."""
from copy import deepcopy
import math
import time
from functools import wraps

from curobo_planning import LEFT_JOINTS
from integration import _digest

RUNNING_POSE_WEIGHT = 100.
TASK_POSITION_TOLERANCE_M = .002
TASK_ORIENTATION_TOLERANCE_RAD = math.radians(.5)


def cuda_graph_counter(torch):
    """Count native API calls in this planner process; never change graph inputs."""
    graph=torch.cuda.CUDAGraph
    if not hasattr(graph,'_workbench_calls'):
        calls=dict(captures=0,replays=0)
        for method,key in (('capture_begin','captures'),('replay','replays')):
            original=getattr(graph,method)
            @wraps(original)
            def observed(self,*args,_original=original,_key=key,**kwargs):
                calls[_key]+=1
                return _original(self,*args,**kwargs)
            setattr(graph,method,observed)
        graph._workbench_calls=calls
    return graph._workbench_calls


def finite_vector(value, size, name):
    values = tuple(value)
    if len(values) != size or any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
        raise ValueError(f'{name} requires {size} finite numbers')
    return list(values)


def normalized_fk_quaternion(value):
    values=finite_vector(value,4,'native FK quaternion')
    norm=math.sqrt(sum(v*v for v in values))
    if abs(norm-1.)>1e-4:
        raise ValueError('native FK quaternion is not a unit rotation')
    return tuple(v/norm for v in values)


def checked_pose(pose):
    position = finite_vector(pose['position'], 3, 'position')
    quaternion = finite_vector(pose['orientation_wxyz'], 4, 'orientation_wxyz')
    if abs(sum(v*v for v in quaternion)-1.) > 1e-6:
        raise ValueError('unit wxyz goal quaternion required')
    return position, quaternion


def task_pose_error(position, quaternion, target_position, target_quaternion):
    position = finite_vector(position, 3, 'solved position')
    quaternion = finite_vector(quaternion, 4, 'solved quaternion')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if abs(norm-1.) > 1e-4:
        raise ValueError('solved FK quaternion must be unit length')
    distance = math.dist(position, target_position)
    dot = min(1., abs(sum(a*b for a,b in zip(quaternion,target_quaternion)))/norm)
    angle = 2*math.acos(dot)
    return distance, angle, distance <= TASK_POSITION_TOLERANCE_M+1e-7 and angle <= TASK_ORIENTATION_TOLERANCE_RAD+1e-6


def sync_locked_joint_metadata(model, positions):
    """0.7.8 updates locked transforms but leaves get_full_js metadata stale."""
    state = model.lock_jointstate
    if state is None or set(state.joint_names) != set(positions):
        raise ValueError('locked joint metadata differs from the requested fixed state')
    values = [positions[name] for name in state.joint_names]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('finite locked joint metadata required')
    state.position.copy_(state.position.new_tensor(values).reshape_as(state.position))


def linear_hold_weights(hold_orientation, linear_approach=False, linear_axis=None):
    if linear_axis is not None and type(linear_axis) is not int:raise ValueError('linear axis must be an integer')
    if linear_approach:
        if linear_axis is not None and linear_axis!=0:raise ValueError('approach uses tool X')
        linear_axis=0
    if linear_axis is not None and (type(linear_axis) is not int or not 0<=linear_axis<3 or not hold_orientation):
        raise ValueError('linear motion requires a held orientation and X/Y/Z axis')
    weights=[1.,1.,1.,0.,0.,0.]
    if linear_axis is not None:
        weights[3:]=[1.,1.,1.];weights[3+linear_axis]=0.
    return linear_axis,weights


def segment_errors(positions, origin, goal):
    direction=goal-origin
    squared=(direction*direction).sum(-1)
    if squared<=1e-16:return ((positions-origin)**2).sum(-1)**.5
    fraction=(((positions-origin)*direction).sum(-1)/squared).clip(0.,1.)
    return ((positions-origin-fraction[...,None]*direction)**2).sum(-1)**.5


def discrete_motion(path, velocity_limit, acceleration_limit, dt=1/30):
    """The native executor uses target differences, including zero-speed joins."""
    if len(path)<2 or not all(math.isfinite(v) and v>0 for v in (velocity_limit,acceleration_limit,dt)):
        raise ValueError('finite positive motion limits and a complete path required')
    rows=[finite_vector(row,6,'control joint path') for row in path]
    velocities=[[0.]*6]+[[(b-a)/dt for a,b in zip(left,right)] for left,right in zip(rows,rows[1:])]+[[0.]*6]
    velocity=max(abs(v) for row in velocities for v in row)
    acceleration=max(abs(b-a)/dt for left,right in zip(velocities,velocities[1:]) for a,b in zip(left,right))
    return dict(passed=velocity<=velocity_limit+1e-9 and acceleration<=acceleration_limit+1e-9,
        max_velocity_rad_s=velocity,max_acceleration_rad_s2=acceleration,
        velocity_limit_rad_s=velocity_limit,acceleration_limit_rad_s2=acceleration_limit,
        interpolation_dt=dt,zero_velocity_joins=True)


def continuous_path(path, velocity_limit, acceleration_limit, *, positions=None, tcp_speed=None, frontloaded=False, duration_s=None):
    """One quintic clock over the joined joint polyline; no internal stops."""
    from bisect import bisect_right
    if positions is not None and (len(positions)!=len(path) or tcp_speed is None or not math.isfinite(tcp_speed) or tcp_speed<=0):
        raise ValueError('TCP clock requires one position per target and a positive speed')
    points, arc, xyz = [], [], []
    for i,row in enumerate(path):
        row = finite_vector(row, 6, 'joined path')
        distance = math.dist(points[-1], row) if points else 0.
        if points and distance <= 1e-12:
            continue
        if positions is not None:
            point=finite_vector(positions[i],3,'TCP position')
            distance=max(math.dist(xyz[-1],point),1e-12) if xyz else 0.
            xyz.append(point)
        points.append(row); arc.append((arc[-1] if arc else 0.)+distance)
    if len(points)<2:
        raise ValueError('continuous cross requires nonstationary geometry')
    intervals=max(2,math.ceil(arc[-1]/tcp_speed*30*1.875)) if positions is not None else max(2,len(path)-1)
    if duration_s is not None:
        if not math.isfinite(duration_s) or duration_s<=0:raise ValueError('positive starting duration required')
        intervals=max(2,math.ceil(duration_s*30))
    for _ in range(10):
        result, coordinates = [], []
        for i in range(intervals+1):
            t=i/intervals
            s=(6*t*t-8*t**3+3*t**4 if frontloaded else 10*t**3-15*t**4+6*t**5)*arc[-1]
            index=min(len(points)-2,max(0,bisect_right(arc,s)-1))
            alpha=min(1.,max(0.,(s-arc[index])/(arc[index+1]-arc[index])))
            result.append([a+(b-a)*alpha for a,b in zip(points[index],points[index+1])])
            coordinates.append(index+alpha)
        result[0],result[-1]=points[0],points[-1]
        check=discrete_motion(result,velocity_limit,acceleration_limit)
        if check['passed']:
            return result, dict(method='TCP_arc_frontloaded_clock' if frontloaded else ('TCP_arc_quintic_clock' if positions is not None else 'joined_joint_polyline_arc_length_quintic_clock'),
                source_path_sha256=_digest(path),
                distinct_source_vertices=len(points),source_vertex_coordinates=coordinates,
                original_ticks=len(path),ticks=len(result),duration_s=intervals/30,
                internal_stop_commands=False,control_validation=check,
                final_gpu_path_query_required=True)
        factor=max(check['max_velocity_rad_s']/velocity_limit,
                   math.sqrt(check['max_acceleration_rad_s2']/acceleration_limit),1.01)
        intervals=math.ceil(intervals*factor*1.02)
    raise ValueError('continuous cross cannot satisfy original motion limits')


class MotionGenerator:
    """One GPU solver, reused for the stages of one simulation planning request."""

    def __init__(self, robot_config, world_config, *, use_cuda_graph=True, model_report=None):
        import torch
        import curobo
        from curobo.types.base import TensorDeviceType
        from curobo.geom.sdf.world import CollisionCheckerType
        from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig
        from curobo.util_file import get_task_configs_path, join_path, load_yaml
        if curobo.__version__ != '0.7.8':
            raise ValueError('MotionGenerator requires cuRobo 0.7.8')
        if model_report is not None:
            from curobo_robot_model import digest
            if model_report['robot_config_sha256']!=digest(robot_config):raise ValueError('model report/config identity differs')
        self.config = deepcopy(robot_config)
        self.motion_control_limits=None if model_report is None else model_report.get('motion_control_limits')
        # These are pose-goal links, not collision links. MotionGen.warmup binds
        # every listed link to a pose, and 0.7.8 Goal.copy_ retains them on None.
        # Keep the active EE and fixed right tips, never the moving intermediate
        # left links. The right tips also prevent the parser pruning lock_joints.
        self.config['kinematics']['link_names'] = [self.config['kinematics']['ee_link'], 'right_link17', 'right_link18']
        self.tensor_args = TensorDeviceType(device=torch.device('cuda:0'))
        self.torch = torch
        self.cuda_graph_calls=cuda_graph_counter(torch)
        self.receipts = []
        started = time.perf_counter()
        gradient_ik = deepcopy(load_yaml(join_path(get_task_configs_path(), 'gradient_ik_autotune.yml')))
        # A1's supported home legitimately sits at its joint lower bounds.
        # Keep the URDF hard bounds; remove only IK's soft inward repulsion.
        gradient_ik['cost']['bound_cfg']['activation_distance'] = [0.]
        config = MotionGenConfig.load_from_robot_config(
            deepcopy(self.config), world_config, tensor_args=self.tensor_args,
            gradient_ik_file=gradient_ik,
            num_ik_seeds=32, num_graph_seeds=2, num_trajopt_seeds=2,
            finetune_trajopt_iters=100,
            interpolation_dt=1/30, interpolation_steps=512, trajopt_tsteps=64,
            # Allow slower motion for the control profile without changing the
            # 30Hz output clock or relaxing velocity/acceleration limits.
            trajopt_dt=.15, maximum_trajectory_dt=.5,
            velocity_scale=self.config['kinematics']['cspace'].get('velocity_scale'),
            position_threshold=.002, rotation_threshold=.05, cspace_threshold=.005,
            use_cuda_graph=use_cuda_graph, collision_checker_type=CollisionCheckerType.MESH,
            self_collision_check=True, self_collision_opt=True,
            evaluate_interpolated_trajectory=True, sync_cuda_time=True,
            collision_activation_distance=.005, high_precision=False, store_debug_in_result=True)
        self.generator = MotionGen(config)
        seen_pose_costs=set()
        for rollout in self.generator.get_all_pose_rollout_instances():
            cost=getattr(rollout,'goal_cost',None)
            if cost is not None and id(cost) not in seen_pose_costs:
                seen_pose_costs.add(id(cost));cost.run_weight=RUNNING_POSE_WEIGHT
                cost.update_run_weight(run_weight=RUNNING_POSE_WEIGHT)
        self._locked_models = {tuple(sorted(self.config['kinematics']['lock_joints'].items())):
                               deepcopy(self.generator.kinematics.kinematics_config)}
        self.locked_joint_updates = []
        self.initialization_breakdown = dict(native_setup_seconds=time.perf_counter()-started)
        section = time.perf_counter()
        from robot_sim.tasks.manager_based.coffee_make.curobo_collision_cache import configure_zero_home_collision_cache
        # 0.7.8's large-sphere self kernel does not zero a previous collision.
        # Install the existing native fix before warmup captures CUDA graphs.
        self.collision_cache_refresh = configure_zero_home_collision_cache(self.generator)
        self.initialization_breakdown['native_collision_refresh_seconds'] = time.perf_counter()-section
        section = time.perf_counter()
        from region_self_collision import install_motiongen
        self.self_collision_partition = install_motiongen(self.generator)
        self.initialization_breakdown['region_install_seconds'] = time.perf_counter()-section
        section = time.perf_counter()
        from cup_contact import CupContact
        self.cup_contact = CupContact(self.generator,self.config,world_config,model_report)
        if tuple(self.generator.joint_names) != LEFT_JOINTS:
            raise ValueError('MotionGen active joint order differs from the six left model joints')
        torch.cuda.synchronize()
        self.initialization_breakdown['cup_contact_and_sync_seconds'] = time.perf_counter()-section
        self.initialization_seconds = time.perf_counter()-started
        self.warmup_seconds = None

    def warmup(self):
        started = time.perf_counter()
        self.generator.warmup(enable_graph=True, warmup_js_trajopt=True, n_goalset=1)
        self.torch.cuda.synchronize()
        self.warmup_seconds = time.perf_counter()-started

    def _reset_region_counters(self):
        seen=set()
        for rollout in [self.generator.rollout_fn,*self.generator.get_all_rollout_instances()]:
            for name in ('robot_self_collision_cost','robot_self_collision_constraint'):
                cost=getattr(rollout,name,None)
                if cost is not None and id(cost) not in seen and hasattr(cost,'_region_counters'):
                    seen.add(id(cost));cost._region_counters.zero_();cost._region_stats.zero_()

    def _start(self, positions):
        from curobo.types.state import JointState
        values = finite_vector(positions, 6, 'start joint positions')
        return JointState.from_position(self.tensor_args.to_device([values]), joint_names=list(LEFT_JOINTS))

    def plan_pose(self, start, goals, *, name, hold_orientation=False, linear_approach=False, linear_axis=None, project_to_goal_frame=True):
        """Choose a goal and generate the complete joint path on the GPU."""
        from curobo.types.math import Pose
        from curobo.rollout.cost.pose_cost import PoseCostMetric
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        if not goals:
            raise ValueError('at least one base-frame pose goal required')
        self.cup_contact.assert_ready(self)
        checked = [checked_pose(goal) for goal in goals]
        positions = self.tensor_args.to_device([value[0] for value in checked])
        quaternions = self.tensor_args.to_device([value[1] for value in checked])
        linear_axis,weights=linear_hold_weights(hold_orientation,linear_approach,linear_axis)
        metric = (PoseCostMetric(hold_partial_pose=True, project_to_goal_frame=project_to_goal_frame,
                  hold_vec_weight=self.tensor_args.to_device(weights))
                  if hold_orientation else PoseCostMetric(release_partial_pose=True))
        config = MotionGenPlanConfig(max_attempts=4, timeout=5., enable_graph_attempt=2,
                                     pose_cost_metric=metric, enable_finetune_trajopt=True)
        state = self._start(start)
        if hold_orientation:
            # Keep the task's angle limit in radians; the native setter below
            # uses its own quaternion-distance units for start validation.
            quaternion=self.generator.compute_kinematics(state).ee_pose.quaternion[0].detach().cpu().tolist()
            if any(not task_pose_error([0.,0.,0.],quaternion,[0.,0.,0.],goal[1])[2] for goal in checked):
                raise ValueError('held orientation differs from start by more than 0.5deg')
        if len(goals)>1:
            raise ValueError('this rule chain uses one pose goal per stage')
        # The rule generator emits one grasp and tests placements sequentially.
        # Keep goalset mode without padding each solve to seven duplicate goals.
        method = 'plan_goalset'
        target = Pose(position=positions.unsqueeze(0),quaternion=quaternions.unsqueeze(0))
        if hold_orientation:
            # 0.7.8's start validation needs a 2-D Pose even in goalset mode.
            # Use its public metric setter, then keep the normal goalset solver.
            metric_goal=Pose(position=positions,quaternion=quaternions)
            if not self.generator.update_pose_cost_metric(metric,state,metric_goal):
                raise ValueError('start pose is outside the native held-pose constraint')
            config.pose_cost_metric=None
        self._reset_region_counters()
        started = time.perf_counter()
        try:
            result = getattr(self.generator, method)(state, target, config)
            self.torch.cuda.synchronize()
        finally:
            if hold_orientation:
                self.generator.update_pose_cost_metric(PoseCostMetric.reset_metric())
        path, receipt = self._result(name, method, result, time.perf_counter()-started)
        # Library rotation error is NOT radians. Check the actual solved FK in
        # task units, independently of the library's quaternion cost threshold.
        end = self.generator.kinematics.get_state(self.tensor_args.to_device([path[-1]]))
        goal_index = receipt.get('goalset_index', [0])[0]
        goal_position, goal_quaternion = checked[goal_index]
        position_error, angle_error, passed = task_pose_error(end.ee_position[0].detach().cpu().tolist(),
            end.ee_quaternion[0].detach().cpu().tolist(), goal_position, goal_quaternion)
        receipt.update(task_position_error_m=position_error, task_orientation_error_rad=angle_error)
        if hold_orientation:
            # Validate the actual native path's task semantics, not only its end.
            # Later user speed interpolation does not rerun collision checks.
            path_state=self.generator.kinematics.get_state(self.tensor_args.to_device(path))
            quaternions=path_state.ee_quaternion.double()
            reference=self.tensor_args.to_device(goal_quaternion).double()
            dots=(quaternions*reference).sum(-1).abs()/(self.torch.linalg.vector_norm(quaternions,dim=-1)*self.torch.linalg.vector_norm(reference))
            maximum=float((2*self.torch.acos(dots.clamp(0.,1.))).max())
            receipt['maximum_path_orientation_error_rad']=maximum
            if not math.isfinite(maximum) or maximum>TASK_ORIENTATION_TOLERANCE_RAD+1e-6:
                receipt['success']=False
                raise ValueError(f'MotionGen {name} exceeds 0.5deg along the held-orientation path')
            if linear_axis is not None:
                if linear_axis==0 and project_to_goal_frame:
                    local=metric_goal.compute_local_pose(Pose(position=path_state.ee_position,quaternion=path_state.ee_quaternion))
                    errors=self.torch.linalg.vector_norm(local.position[...,1:],dim=-1)
                    reference='tool-X goal-frame line'
                else:
                    errors=segment_errors(path_state.ee_position,path_state.ee_position[0],self.tensor_args.to_device(goal_position))
                    reference='actual start-goal segment'
                transverse=float(errors.max())
                receipt['maximum_linear_transverse_error_m']=transverse
                receipt['linear_axis_goal_frame']=linear_axis
                receipt['linear_acceptance_reference']=reference
                if linear_approach:receipt['maximum_approach_transverse_error_m']=transverse
                if not math.isfinite(transverse) or transverse>TASK_POSITION_TOLERANCE_M+1e-7:
                    receipt['success']=False
                    receipt['unaccepted_path_joints']=path
                    raise ValueError(f'MotionGen {name} exceeds the straight-motion 2mm corridor')
        if not passed:
            receipt['success'] = False
            raise ValueError(f'MotionGen {name} exceeds accepted 2mm/0.5deg pose tolerance')
        return path, receipt

    def plan_joint(self, start, goal, *, name):
        """Use MotionGen's joint-space search/trajectory optimization for return-home."""
        from curobo.wrap.reacher.motion_gen import MotionGenPlanConfig
        self.cup_contact.assert_ready(self)
        state, target = self._start(start), self._start(goal)
        self._reset_region_counters()
        started = time.perf_counter()
        result = self.generator.plan_single_js(state, target,
            MotionGenPlanConfig(max_attempts=4, timeout=5., enable_graph_attempt=2))
        self.torch.cuda.synchronize()
        return self._result(name, 'plan_single_js', result, time.perf_counter()-started)

    def validate_continuous_path(self, path):
        """The joined cross has a new sampling clock: query its final GPU path."""
        began=time.perf_counter(); maximum_self=maximum_world=0.
        for offset in range(0,len(path),256):
            q=self.tensor_args.to_device(path[offset:offset+256])
            spheres=self.generator.kinematics.get_state(q).get_link_spheres().unsqueeze(0)
            rollout=self.generator.rollout_fn
            values=[rollout.robot_self_collision_constraint(spheres),
                    rollout.primitive_collision_constraint(spheres)]
            if any(not self.torch.isfinite(v).all() for v in values):
                raise ValueError('nonfinite continuous cross GPU constraint')
            maximum_self=max(maximum_self,float(values[0].max()))
            maximum_world=max(maximum_world,float(values[1].max()))
        receipt=dict(passed=maximum_self<=0. and maximum_world<=0.,
            self_max=maximum_self,world_max=maximum_world,samples=len(path),
            path_sha256=_digest(path),
            backend='existing native GPU hard constraints; attached cup and full world',
            wall_seconds=time.perf_counter()-began)
        if not receipt['passed']:
            raise ValueError('joined cross GPU path rejected: '+str(receipt))
        return receipt

    def _result(self, name, method, result, seconds):
        success = bool(result.success is not None and result.success.all().item())
        receipt = dict(stage=name, method='MotionGen.'+method, success=success,
                       status=str(result.status), wall_seconds=seconds,
                       solve_seconds=float(result.solve_time), total_seconds=float(result.total_time))
        native_attempts = getattr(result, 'attempts', None)
        receipt.update(native_attempts=native_attempts,
            # 0.7.8 plan_single_js stores the last zero-based loop index;
            # pose methods store the number of attempted iterations.
            attempts=None if native_attempts is None else int(native_attempts)+(method=='plan_single_js'),
            native_trajopt_attempts=getattr(result, 'trajopt_attempts', None),
            used_graph=bool(getattr(result, 'used_graph', False)),
            pose_finetune_iteration_budget=None if method=='plan_single_js' else 100,
            timing_scope='native cumulative components; may be nested, do not sum with wall_seconds')
        for key in ('ik_time', 'graph_time', 'trajopt_time', 'finetune_time'):
            value = getattr(result, key, None)
            if value is not None: receipt[key+'_seconds'] = float(value)
        for key in ('position_error', 'rotation_error', 'cspace_error'):
            value = getattr(result, key, None)
            if value is not None:
                receipt[key] = value.detach().cpu().reshape(-1).tolist()
        ik = (result.debug_info or {}).get('ik_result')
        if ik is not None:
            receipt['ik'] = {key:getattr(ik,key).detach().cpu().reshape(-1).tolist()
                             for key in ('success','position_error','rotation_error')}
        for label in ('trajopt_result','finetune_trajopt_result'):
            stage=(result.debug_info or {}).get(label)
            if stage is not None:
                row={}
                raw=getattr(stage,'solution',None)
                if raw is not None and getattr(raw,'position',None) is not None:
                    row['optimizer_knot_positions']=raw.position.detach().cpu().tolist()
                for key in ('success','smooth_label','smooth_error','optimized_dt'):
                    value=getattr(stage,key,None)
                    if value is not None:row[key]=value.detach().cpu().reshape(-1).tolist()
                metrics=getattr(stage,'metrics',None)
                if metrics is not None:
                    for key in ('feasible','constraint'):
                        value=getattr(metrics,key,None)
                        if value is not None:
                            flat=value.detach().float().reshape(-1)
                            row[key]=dict(minimum=float(flat.min()),maximum=float(flat.max()),nonzero=int((flat!=0).sum()),count=flat.numel())
                receipt[label]=row
        regions=[];seen=set()
        for rollout in [self.generator.rollout_fn,*self.generator.get_all_rollout_instances()]:
            for key in ('robot_self_collision_cost','robot_self_collision_constraint'):
                cost=getattr(rollout,key,None)
                if cost is not None and id(cost) not in seen and hasattr(cost,'_region_stats'):
                    seen.add(id(cost));regions.append(dict(role=key,last=cost._region_stats.detach().cpu().tolist(),
                        query_counters=cost._region_counters.detach().cpu().tolist()))
        receipt['self_region_queries']=regions
        receipt['self_region_counter_scope']='this native invocation; excludes warmup and failure diagnostics'
        self.receipts.append(receipt)
        if not success:
            try:
                if ik is not None:
                    # Failure-only diagnostics, never an alternate acceptance test.
                    from motiongen_self_collision import enabled_pairs
                    q=ik.solution.detach().reshape(-1,6)[:1]
                    spheres=self.generator.kinematics.get_state(q).get_link_spheres().unsqueeze(1)
                    rollout=self.generator.rollout_fn
                    receipt['failed_ik_constraints']=dict(
                        self=rollout.robot_self_collision_constraint(spheres).detach().cpu().tolist(),
                        world=rollout.primitive_collision_constraint(spheres).detach().cpu().tolist())
                    cost=rollout.primitive_collision_constraint
                    if hasattr(cost,'_cup_contact_children'):
                        receipt['failed_ik_cup_parts']=[]
                        for role,(indices,query) in zip(('claw','wrist'),cost._cup_contact_children):
                            enabled=set(indices.cpu().tolist())
                            for alias in self.config['kinematics']['collision_link_names']:
                                ids=self.generator.kinematics.kinematics_config.get_sphere_index_from_link_name(alias)
                                if not len(ids) or not set(ids.cpu().tolist())<=enabled:continue
                                value=float(query(spheres.index_select(2,ids)).detach().max())
                                if value>0:
                                    receipt['failed_ik_cup_parts'].append(dict(role=role,alias=alias,
                                        link=self.config['kinematics']['extra_links'][alias]['parent_link_name'],cost=value))
                    native=rollout.robot_self_collision_constraint.self_collision_kin_config
                    _,pairs=enabled_pairs(native)
                    pair=self.torch.tensor(pairs,device=spheres.device,dtype=self.torch.long)
                    xyz=spheres[0,0,:,:3];radius=spheres[0,0,:,3]+native.offset
                    depth=radius[pair[:,0]]+radius[pair[:,1]]-self.torch.linalg.vector_norm(xyz[pair[:,0]]-xyz[pair[:,1]],dim=-1)
                    depth=self.torch.where((radius[pair[:,0]]>0)&(radius[pair[:,1]]>0),depth,-1.)
                    ids=self.torch.topk(depth,min(12,len(pairs))).indices.cpu().tolist()
                    aliases={}
                    for alias in self.config['kinematics']['collision_link_names']:
                        for i in self.generator.kinematics.kinematics_config.get_sphere_index_from_link_name(alias).cpu().tolist():aliases[i]=alias
                    receipt['proxy_overlap_diagnostic']=[dict(aliases=[aliases[int(pair[i,j])] for j in (0,1)],
                        links=[self.config['kinematics']['extra_links'][aliases[int(pair[i,j])]]['parent_link_name'] for j in (0,1)],
                        sphere_indices=pair[i].cpu().tolist(),proxy_overlap_m=float(depth[i])) for i in ids if float(depth[i])>0]
                    receipt['failed_ik_joints']=q.cpu().tolist()
                for label in ('trajopt_result','finetune_trajopt_result'):
                    failed_stage=(result.debug_info or {}).get(label)
                    solution=getattr(failed_stage,'interpolated_solution',None)
                    if solution is None:continue
                    positions=solution.position[:1]
                    if positions.ndim!=3:continue
                    last_step=getattr(failed_stage,'path_buffer_last_tstep',None)
                    length=positions.shape[1] if last_step is None else int(last_step[0])
                    positions=positions[:,:int(length)]
                    if positions.shape[1]<3:continue
                    receipt[label]['failed_path_joints']=positions[0].detach().cpu().tolist()
                    samples=self.generator.kinematics.get_state(positions.reshape(-1,6)).get_link_spheres().reshape(1,positions.shape[1],-1,4)
                    rollout=self.generator.rollout_fn
                    self_value=rollout.robot_self_collision_constraint(samples).detach()
                    world_value=rollout.primitive_collision_constraint(samples).detach()
                    velocity=positions.diff(dim=1)*30;acceleration=velocity.diff(dim=1)*30
                    receipt[label]['failed_path_diagnostic']=dict(samples=positions.shape[1],
                        self_max=float(self_value.max()),world_max=float(world_value.max()),
                        world_nonzero=int((world_value>0).sum()),self_nonzero=int((self_value>0).sum()),
                        finite_difference_velocity_max_rad_s=float(velocity.abs().max()),
                        finite_difference_acceleration_max_rad_s2=float(acceleration.abs().max()))
            except Exception as diagnostic_error:
                receipt['failure_diagnostic_error']=f'{type(diagnostic_error).__name__}: {diagnostic_error}'
            raise ValueError(f'MotionGen {name} failed: {result.status}')
        plan = result.get_interpolated_plan()
        positions = plan.get_ordered_joint_state(list(LEFT_JOINTS)).position.detach().cpu()
        if positions.ndim != 2 or positions.shape[1] != 6 or len(positions) < 2 or not self.torch.isfinite(positions).all():
            raise ValueError('MotionGen returned an invalid joint path')
        dt = float(result.interpolation_dt)
        if abs(dt-1/30) > 1e-8:
            raise ValueError('MotionGen interpolation clock differs from 30 Hz')
        receipt.update(ticks=len(positions), interpolation_dt=dt)
        if result.goalset_index is not None:
            receipt['goalset_index'] = result.goalset_index.detach().cpu().reshape(-1).tolist()
        return positions.tolist(), receipt

    def update_world(self, world):
        self.cup_contact.assert_unattached(self)
        self.generator.update_world(world)
        self.cup_contact.update(world)

    def update_locked_joints(self, values):
        expected = self.config['kinematics']['lock_joints']
        if set(values) != set(expected) or not all(math.isfinite(v) for v in values.values()):
            raise ValueError('complete finite locked joint state required')
        if any(values[name] != expected[name] for name in expected if name.startswith('right_')):
            raise ValueError('inactive right arm/jaws are frozen in the planning world')
        if abs(values['left_joint7']-values['left_joint8'])>1e-9:
            raise ValueError('mechanically linked jaws require symmetric model positions')
        if values==expected:return
        self.cup_contact.assert_unattached(self)
        from curobo.types.robot import RobotConfig
        started=time.perf_counter()
        key=tuple(sorted(values.items()))
        cached=key in self._locked_models
        if not cached:
            # Only the initial opening and latest contact geometry are retained.
            if len(self._locked_models)>=2:
                self._locked_models.pop(next(reversed(self._locked_models)))
            config=deepcopy(self.config)
            config['kinematics']['lock_joints']=dict(values)
            self._locked_models[key]=RobotConfig.from_dict(config,self.tensor_args).kinematics.kinematics_config
        # Reuse the native tensor-copy path. Rebuilding URDF/self-pair tables on
        # every return to the same open/closed geometry is initialization work.
        self.generator.kinematics.update_kinematics_config(self._locked_models[key])
        sync_locked_joint_metadata(self.generator.kinematics, values)
        self.cup_contact.restore_world_proxy()
        self.torch.cuda.synchronize()
        self.locked_joint_updates.append(dict(cache_hit=cached,wall_seconds=time.perf_counter()-started))
        self.config['kinematics']['lock_joints'] = dict(values)
