"""Local table-placement IK for explicit maintenance; never a collision certificate."""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import inspect
import math
from pathlib import Path

from integration import _digest
from joint_space import convert_state, model_joint_limits


class NativeReturnKinematics:
    """Reuse the installed model and native CPU FK/IK, without starting a solver service."""
    def __init__(self, urdf_path, source, world_from_robot):
        from robot_sim.tasks.manager_based.coffee_make.m1_robot_geometry import M1RobotKinematics
        from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
        from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
        from curobo_planning import robot_config
        if not isinstance(urdf_path,(str,Path)) or not str(urdf_path):
            raise ValueError('installed return IK URDF path required')
        path=Path(urdf_path).resolve();sha=hashlib.sha256(path.read_bytes()).hexdigest()
        if sha!=source['calibration_profile']['model_binding']['urdf_sha256']:
            raise ValueError('return IK URDF differs from installed calibration')
        context=source.get('replay_context') or source.get('maintenance_context',{}).get('current_scene',{})
        if world_from_robot!=context.get('world_from_robot'):
            raise ValueError('return IK world/base transform is not sealed in the current source')
        self.fk=M1RobotKinematics(path);self.spec=ac_one_dual_arm_zero_home_action_spec()
        self.transform=RigidTransform(**world_from_robot);self.pose_type=RigidTransform
        _,native=robot_config(path,self.fk.movable,self.positions(source['initial_state']['model']))
        limits=model_joint_limits(self.spec,source)
        self.limits={side: [(max(lo,native['joints'][name]['lower']),min(hi,native['joints'][name]['upper']))
            for name,(lo,hi) in zip(self.names(side),limits[side+'_arm_joints'])] for side in ('left','right')}
        self.identity=dict(urdf_path=str(path),urdf_sha256=sha,world_from_robot=deepcopy(world_from_robot),
            implementation='M1RobotKinematics.inverse',implementation_sha256=hashlib.sha256(
                Path(inspect.getfile(M1RobotKinematics)).read_bytes()).hexdigest(),collision_checked=False)

    @staticmethod
    def names(side):return tuple(f'left_joint{i}' if side=='left' else f'right_joint{10+i}' for i in range(1,7))

    def positions(self,target):
        result={name:q for side in ('left','right') for name,q in zip(self.names(side),target[side+'_arm_joints'])}
        result.update(left_joint7=target['left_gripper']/2000,left_joint8=target['left_gripper']/2000,
            right_joint17=target['right_gripper']/2000,right_joint18=target['right_gripper']/2000)
        return result

    def pose(self,target):
        return asdict(self.transform.compose(self.fk.pose(self.fk._forward_link(self.positions(target),'left_link6'))))

    def solve(self,goal,target,seed):
        relative=self.transform.inverse().compose(self.pose_type(**goal))
        q,_=self.fk.inverse(relative,seed=seed,positions=self.positions(target))
        return list(q)

    def validate(self,target,goal):
        for side in ('left','right'):
            if any(not lo-1e-12<=q<=hi+1e-12 for q,(lo,hi) in zip(target[side+'_arm_joints'],self.limits[side])):
                raise ValueError('offset IK exceeds calibrated model/Owner joint bounds')
        actual=self.pose(target)
        distance=math.dist(actual['position'],goal['position'])
        dot=min(1.,abs(sum(a*b for a,b in zip(actual['orientation_wxyz'],goal['orientation_wxyz']))))
        angle=2*math.acos(dot)
        if distance>1e-5 or angle>1e-5:
            raise ValueError('offset fixed IK solution differs from its requested Cartesian pose')
        return distance,angle


def offset_request(source,parent_source,return_goal=None,offset_xy_mm=None):
    context=source['replay_context'];historical=parent_source['scene']['environment_from_cup']
    origin=deepcopy(context.get('return_origin',historical));offset=deepcopy(context.get('offset_xy_mm',[0.,0.]))
    from cup_return_replay import _pose_values
    _pose_values(origin)
    if (not isinstance(offset,list) or len(offset)!=2
            or any(type(v) not in (int,float) or not math.isfinite(v) or abs(v)>50. for v in offset)):
        raise ValueError('return offset requires two finite values within +/-50 mm from the fixed origin')
    if offset_xy_mm is not None and offset_xy_mm!=offset:
        raise ValueError('requested offset differs from the sealed replay context')
    if (abs(origin['position'][2]-historical['position'][2])>1e-9
            or abs(abs(sum(a*b for a,b in zip(origin['orientation_wxyz'],historical['orientation_wxyz'])))-1.)>1e-9):
        raise ValueError('table return changes XY only; preserve the parent support height and upright orientation')
    goal=deepcopy(origin);goal['position'][:2]=[q+d/1000 for q,d in zip(origin['position'][:2],offset)]
    if return_goal is not None and return_goal!=goal:
        raise ValueError('return goal differs from fixed origin plus the sealed user offset')
    delta=[goal['position'][i]-historical['position'][i] for i in range(2)]
    exact=all(abs(d)<=1e-10 for d in delta)
    return dict(return_origin=origin,offset_xy_mm=offset,return_goal=goal,effective_delta_m=delta,
        exact_parent_reversal=exact,coordinate_frame='world XY; original height and orientation retained')


def _local_stages(base):
    names=[row['name'] for row in base['stages']]
    required=['lower','support_settle','open','release_hold','retreat']
    start=names.index('lower')
    if names[start:start+len(required)]!=required or any(names.count(name)!=1 for name in required):
        raise ValueError('offset return requires one continuous table lower/release/retreat sequence')
    return set(range(start,start+len(required)))


def _air_goals(start,end,speed):
    """Smooth horizontal alignment at a previously planned clearance height."""
    if abs(start['position'][2]-end['position'][2])>1e-9:
        raise ValueError('offset alignment must retain its hover height')
    intervals=max(6,math.ceil(1.875*math.dist(start['position'],end['position'])/speed*30))
    q0=list(start['orientation_wxyz']);q1=list(end['orientation_wxyz'])
    if sum(a*b for a,b in zip(q0,q1))<0:q1=[-v for v in q1]
    goals=[]
    for i in range(intervals+1):
        t=i/intervals;u=10*t**3-15*t**4+6*t**5
        q=[a+(b-a)*u for a,b in zip(q0,q1)];norm=math.sqrt(sum(v*v for v in q))
        goals.append(dict(position=[a+(b-a)*u for a,b in zip(start['position'],end['position'])],
                          orientation_wxyz=[v/norm for v in q]))
    return goals


def apply_offset(base,source,request,kinematics,*,evidence=None,environment='fake'):
    """Solve once at publication; reconstruction verifies fixed solutions using FK only."""
    from motiongen_plan import encode_stage,enforce_stage_motion_limits
    from physical_motiongen import reported_target
    if not isinstance(kinematics,NativeReturnKinematics):
        raise ValueError('offset return requires the installed native CPU kinematics')
    if environment not in ('fake','physical'):
        raise ValueError('explicit fake or physical offset environment required')
    if evidence is not None and evidence.get('kinematics')!=kinematics.identity:
        raise ValueError('offset FK model or implementation differs from publication')
    local=_local_stages(base);starts=base['stages'];models=[];raw_solutions=[];cache={};ik_cache={}
    error_m=error_rad=0.;seed=None;cursor=0
    original=[]
    for i,stage in enumerate(starts):
        end=starts[i+1]['start_tick'] if i+1<len(starts) else len(base['ticks'])
        original.append([convert_state(t['target'],source['calibration_profile'],to_reported=False)
                         for t in base['ticks'][stage['start_tick']:end]])
    lower=min(local);retreat=max(local)
    acceptance=base.get('native_identity',{}).get('acceptance',{})
    speed,window,clearance=(acceptance.get(key) for key in ('landing_speed_m_s','stable_window_s','hand_clearance_m'))
    if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in (speed,window,clearance)):
        raise ValueError('offset motion requires the existing native speed, stability and clearance bounds')
    hover_start=kinematics.pose(original[lower][0]);original_place=kinematics.pose(original[lower+1][0])
    place=deepcopy(original_place)
    place['position']=list(place['position']);place['orientation_wxyz']=list(place['orientation_wxyz'])
    place['position'][:2]=[p+d for p,d in zip(place['position'][:2],request['effective_delta_m'])]
    if request.get('placement_tool_pose') is not None:
        if environment!='physical':raise ValueError('observed held-cup placement belongs to physical maintenance')
        from cup_return_replay import _pose_values
        _pose_values(request['placement_tool_pose'])
        place=deepcopy(request['placement_tool_pose'])
    displacement=[b-a for a,b in zip(original_place['position'],place['position'])]
    hover=deepcopy(place);hover['position'][2]=hover_start['position'][2]
    hover_height=hover['position'][2]-place['position'][2]
    retreat_end=kinematics.pose(original[retreat][-1]);retreat_height=retreat_end['position'][2]-place['position'][2]
    if min(hover_height,retreat_height)<clearance:
        raise ValueError('original return path does not provide the native clearance for horizontal alignment')

    def solved(index,name,substage,rows,goals,fixed=None):
        nonlocal cursor,seed,error_m,error_rad
        fixed=fixed or {}
        for i,(row,goal) in enumerate(zip(rows,goals,strict=True)):
            key=(*goal['position'],*goal['orientation_wxyz'])
            if evidence is None:
                if i in fixed:q=fixed[i]
                elif key in ik_cache:q=ik_cache[key]
                else:q=kinematics.solve(goal,row,seed if seed is not None else row['left_arm_joints'])
            else:
                try:q=evidence['model_joint_solutions'][cursor]
                except (KeyError,IndexError,TypeError):raise ValueError('offset solution rows are incomplete')
            if not isinstance(q,list) or len(q)!=6 or any(type(v) not in (float,int) or not math.isfinite(v) for v in q):
                raise ValueError('six finite offset model joints required')
            if i in fixed and q!=fixed[i]:raise ValueError('offset must retain its connection to the original trajectory')
            ik_cache[key]=q;seed=list(q);row['left_arm_joints']=list(q);raw_solutions.append(list(q));cursor+=1
            distance,angle=kinematics.validate(row,goal);error_m=max(error_m,distance);error_rad=max(error_rad,angle)
        encoded=encode_stage(dict(name=name,substage=substage,rows=rows),kinematics.spec,
            base['gripper_opening_mm'],cache,environment=environment,source=source)
        encoded['offset_goals']=deepcopy(goals)
        if any(abs(c['delta_rad'])>1e-12 for item in encoded.get('position_corrections',[]) for c in item['changes']):
            raise ValueError('offset IK must solve inside command limits without projecting the path')
        models.append((index,True,encoded))

    for index,stage in enumerate(starts):
        rows=deepcopy(original[index])
        if index==lower:
            goals=_air_goals(hover_start,hover,speed)
            solved(index,'transport','return_hover_alignment',[deepcopy(rows[0]) for _ in goals],goals,{0:rows[0]['left_arm_joints']})
            count=math.ceil(2*window*30)
            solved(index,'lift_hold','return_hover_settle',[deepcopy(rows[0]) for _ in range(count)],[deepcopy(hover) for _ in range(count)])
            goals=[]
            for row in rows:
                goal=deepcopy(place);height=kinematics.pose(row)['position'][2]
                if request.get('placement_tool_pose') is not None:
                    fraction=(height-original_place['position'][2])/(hover_start['position'][2]-original_place['position'][2])
                    height=place['position'][2]+fraction*(hover_start['position'][2]-place['position'][2])
                goal['position'][2]=height;goals.append(goal)
            solved(index,stage['name'],'return_vertical_lower',rows,goals)
        elif index in local:
            goals=[]
            for row in rows:
                if index==retreat:
                    goal=kinematics.pose(row);goal['position']=list(goal['position'])
                    goal['position']=[p+d for p,d in zip(goal['position'],displacement)]
                    if request.get('placement_tool_pose') is not None:
                        goal['orientation_wxyz']=deepcopy(place['orientation_wxyz'])
                        fraction=(kinematics.pose(row)['position'][2]-original_place['position'][2])/(retreat_end['position'][2]-original_place['position'][2])
                        goal['position'][2]=place['position'][2]+fraction*(retreat_end['position'][2]-place['position'][2])
                else:goal=deepcopy(place)
                goals.append(goal)
            solved(index,stage['name'],stage['substage'],rows,goals)
            if index==retreat:
                shifted=deepcopy(retreat_end);shifted['position']=list(shifted['position'])
                shifted['position']=[p+d for p,d in zip(shifted['position'],displacement)]
                if request.get('placement_tool_pose') is not None:
                    shifted['orientation_wxyz']=deepcopy(place['orientation_wxyz'])
                shifted['position'][2]=retreat_end['position'][2]
                goals=_air_goals(shifted,retreat_end,speed)
                solved(index,'retreat','return_unshift_at_clearance',[deepcopy(original[index][-1]) for _ in goals],goals,
                       {len(goals)-1:original[index][-1]['left_arm_joints']})
        else:
            begin=stage['start_tick'];end=starts[index+1]['start_tick'] if index+1<len(starts) else len(base['ticks'])
            models.append((index,False,dict(name=stage['name'],substage=stage['substage'],rows=rows,
                native_actions=deepcopy(base['native_actions'][begin:end]))))
    if evidence is not None and len(evidence['model_joint_solutions'])!=cursor:
        raise ValueError('extra offset solution rows are not permitted')
    # Only the changed moving stages may slow down; original path sections,
    # stationary release and jaw targets retain their existing clock.
    retiming=[]
    for index,(parent,changed,stage) in enumerate(models):
        if not changed or stage['name'] not in ('transport','lower','retreat'):continue
        stages,adjustments=enforce_stage_motion_limits([stage],base['control'],kinematics.spec,base['gripper_opening_mm'])
        if environment=='physical':
            times=adjustments[0]['source_sample_times'] if adjustments else list(range(len(stage['rows'])))
            goals=stage['offset_goals']
            for row,t in zip(stages[0]['rows'],times,strict=True):
                a=min(int(t),len(goals)-1);b=min(a+1,len(goals)-1);alpha=t-a
                qa,qb=goals[a]['orientation_wxyz'],goals[b]['orientation_wxyz']
                if sum(x*y for x,y in zip(qa,qb))<0:qb=[-v for v in qb]
                q=[x+(y-x)*alpha for x,y in zip(qa,qb)];norm=math.sqrt(sum(v*v for v in q))
                goal=dict(position=[x+(y-x)*alpha for x,y in zip(goals[a]['position'],goals[b]['position'])],
                          orientation_wxyz=[v/norm for v in q])
                distance,angle=kinematics.validate(row,goal)
                error_m=max(error_m,distance);error_rad=max(error_rad,angle)
        models[index]=(parent,changed,stages[0]);retiming.extend(adjustments)
    result=deepcopy(base);ticks=[];actions=[];new_stages=[]
    for index,(parent,changed,stage) in enumerate(models):
        original_stage=starts[parent];begin=original_stage['start_tick']
        new_stages.append(dict(original_stage,name=stage['name'],substage=stage['substage'],
            base_reverse_stage_index=parent,start_tick=len(ticks),start_s=len(ticks)/30))
        for i,row in enumerate(stage['rows']):
            target=(reported_target(row,source['calibration_profile'],source) if changed
                    else deepcopy(base['ticks'][begin+i]['target']))
            ticks.append(dict(tick=len(ticks),time_s=len(ticks)/30,target=target))
        actions.extend(stage['native_actions'])
    result.update(ticks=ticks,native_actions=actions,stages=new_stages,timeline_digest=_digest(ticks),
        native_substeps=[base['control']['substeps']]*len(ticks),duration_s=len(ticks)/30)
    from control_motion import validate_control
    from joint_space import contract
    result['joint_space']=contract(source['calibration_profile'],actions)
    result['control_validation']=validate_control(result)
    proof=dict(schema_version='cup_return_local_offset.v2',environment=environment,kinematics=deepcopy(kinematics.identity),
        model_joint_solutions=raw_solutions,local_stage_indices=[i for i,(_,changed,_) in enumerate(models) if changed],
        effective_delta_m=request['effective_delta_m'],
        blend='horizontal hover alignment; stable hold; fixed XY/orientation lowering; full-offset retreat; horizontal unshift at clearance',
        hover_clearance_m=hover_height,unshift_clearance_m=retreat_height,
        hover_settle_s=math.ceil(2*window*30)/30,air_alignment_peak_speed_m_s=speed,
        maximum_fk_position_error_m=error_m,maximum_fk_orientation_error_rad=error_rad,
        fk_error_scope=('fixed Cartesian IK and every emitted row after optional uniform joint-polyline resampling'
                        if environment=='physical' else 'fixed Cartesian IK samples before optional uniform joint-polyline resampling'),
        retiming=retiming,method='native CPU IK once; fixed solutions checked by FK; existing uniform stage retiming',
        collision_check=dict(status='not_run',reason='local_offset_is_not_covered_by_parent_collision_proof'))
    if request.get('placement_tool_pose') is not None:
        proof.update(placement_tool_pose=deepcopy(place),effective_tool_displacement_m=displacement)
    if evidence is not None and evidence!=proof:
        raise ValueError('offset derivation evidence differs from fixed FK and timing reconstruction')
    return result,proof
