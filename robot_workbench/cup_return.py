"""Explicit fake maintenance replay; historical observation is never recaptured."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from uuid import uuid4


def _read_pin(item):
    raw=Path(item['path']).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=item['sha256']:raise ValueError('场景恢复来源文件已变化：'+item['path'])
    return raw


def _save(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    staged=path.with_name(path.name+'.tmp')
    staged.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
    staged.replace(path)


def installed_pair(device):
    preparation=getattr(device,'collection_preparation',None)
    if preparation:return preparation.snapshot().get('installed') or {}
    flow=device.workflows.get('calibration')
    return (flow.app.installed_pair() or {}) if flow is not None and hasattr(flow,'app') else {}


def configure(device,root,maintenance):
    """Keep the original anchor and latest successful parent across service restarts."""
    state_source=maintenance.get('recovery_state_source')
    state=json.loads(_read_pin(state_source)) if state_source else {}
    if state and (state.get('schema')!='scene_recovery_state.v1' or state.get('robot_id')!=device.id):
        raise ValueError('场景恢复记录不属于当前 fake 设备')
    device.cup_return_root=Path(root)
    device.scene_recovery_state=state
    device.cup_return_inputs=deepcopy(state.get('latest_parent') or maintenance.get('cup_return_parent'))
    device.scene_recovery_error=None
    if not state and device.cup_return_inputs:
        source=json.loads(_read_pin(device.cup_return_inputs['source']))
        state.update(schema='scene_recovery_state.v1',robot_id=device.id,
            origin=deepcopy(source['scene']['environment_from_cup']),latest_parent=deepcopy(device.cup_return_inputs))
    if state:_save(Path(root)/'scene-recovery-state.json',state)


def remember_success(device,runner,receipt,reference):
    """Keep small endpoint state here; promote large accepted inputs in a CPU child."""
    qualified=(receipt.get('motion_outcome')=='completed' and receipt.get('task_qualified')
        and receipt.get('task_outcome')=='success')
    plan_ref=getattr(runner,'payload',{}).get('plan_ref') if runner is not None else None
    if not qualified or plan_ref is None:
        return _remember_success_local(device,runner,receipt,reference)
    if runner.execution_instance_id!=receipt.get('execution_instance_id'):
        raise ValueError('场景恢复来源的执行身份不一致')
    from .recording_process import _environment
    executable=os.environ.get('WORKBENCH_CPU_PYTHON') or sys.executable
    if not Path(executable).is_absolute():raise ValueError('absolute recovery CPU interpreter required')
    preparation=getattr(device,'collection_preparation',None)
    request=dict(robot_id=device.id,root=str(device.cup_return_root),plan_ref=plan_ref,
        execution_instance_id=runner.execution_instance_id,
        native_receipt={key:reference[key] for key in ('receipt_path','receipt_sha256')},
        state=getattr(device,'scene_recovery_state',{}),inputs=getattr(device,'cup_return_inputs',None),
        preparation_output=preparation.snapshot().get('output') if preparation else None)
    raw=json.dumps(request,allow_nan=False)
    if len(raw.encode())>256*1024:raise ValueError('recovery request exceeds bounded metadata envelope')
    root=Path(device.cup_return_root);root.mkdir(parents=True,exist_ok=True)
    with (root/('recovery-worker-'+uuid4().hex+'.log')).open('xb') as log:
        result=subprocess.run([executable,'-u','-B','-m',__name__,'--remember'],input=raw,
            stdout=subprocess.PIPE,stderr=log,text=True,env=_environment(),timeout=120)
    if result.returncode or len(result.stdout.encode())>256*1024:
        raise ValueError('场景恢复记录未完成，请查看 '+log.name)
    value=json.loads(result.stdout);state=value['state']
    if (state.get('schema')!='scene_recovery_state.v1' or state.get('robot_id')!=device.id
            or state.get('last_completed',{}).get('execution_instance_id')!=runner.execution_instance_id):
        raise ValueError('场景恢复记录的设备或执行身份不一致')
    device.scene_recovery_state=state;device.cup_return_inputs=value['inputs'];device.scene_recovery_error=None


def _remember_success_local(device,runner,receipt,reference):
    """Persist the accepted execution's own immutable inputs, even if recording failed."""
    from cup_return_replay import is_replay
    from integration import _digest
    endpoint=receipt.get('endpoint_observation') or {}
    state=deepcopy(getattr(device,'scene_recovery_state',{}))
    if (state and endpoint.get('status')=='observed' and endpoint.get('environment_from_cup')
            and receipt.get('actual_environment_from_machine_root')):
        if getattr(runner,'execution_instance_id',None)!=receipt.get('execution_instance_id'):
            raise ValueError('场景终点的执行身份不一致')
        # Preserve actual scene separately from successful-parent qualification.
        state['last_observed_scene']=dict(execution_instance_id=receipt['execution_instance_id'],
            cup=deepcopy(endpoint['environment_from_cup']),machine=deepcopy(receipt['actual_environment_from_machine_root']),
            receipt=deepcopy(reference),task_outcome=receipt['task_outcome'],physics_step=endpoint['physics_step'])
        _save(Path(device.cup_return_root)/'scene-recovery-state.json',state)
        device.scene_recovery_state=state
    if (receipt.get('motion_outcome')!='completed' or not receipt.get('task_qualified')
            or receipt.get('task_outcome')!='success'):return
    if getattr(runner,'execution_instance_id',None)!=receipt.get('execution_instance_id'):
        raise ValueError('场景恢复来源的执行身份不一致')
    payload=runner.payload;plan,source=payload['plan'],payload['source']
    if (receipt['parent_timeline_digest']!=plan['timeline_digest'] or receipt['source_digest']!=_digest(source)):
        raise ValueError('场景恢复来源与实际成功轨迹不一致')
    if not is_replay(plan):
        binding=deepcopy(runner.recovery_parent_binding)
        if binding.get('source_digest')!=_digest(source):raise ValueError('场景恢复来源与标定绑定不一致')
        machine=receipt['actual_environment_from_machine_root']
        root=Path(device.cup_return_root)/'recovery-parents'/receipt['execution_instance_id']
        root.mkdir(parents=True,exist_ok=False)
        manifest={}
        for name,value in (('plan',plan),('source',source),('binding',binding),('base',dict(initial_suite=dict(
                episodes=[dict(environment_from_machine_root=machine)])))):
            path=root/(name+'.json');_save(path,value)
            manifest[name]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        manifest['native_receipt']=dict(path=reference['receipt_path'],sha256=reference['receipt_sha256'])
        preparation=device.collection_preparation.snapshot()
        model=Path(preparation['output'])/'calibration/model/robot.urdf'
        sha=hashlib.sha256(model.read_bytes()).hexdigest()
        if sha!=source['calibration_profile']['model_binding']['urdf_sha256']:
            raise ValueError('场景恢复模型与本次标定不一致')
        manifest['urdf']=dict(path=str(model),sha256=sha)
        state.update(schema='scene_recovery_state.v1',robot_id=device.id,latest_parent=manifest)
        state.setdefault('origin',deepcopy(source['scene']['environment_from_cup']))
        device.cup_return_inputs=deepcopy(manifest)
    if not state:return
    state['last_completed']=dict(execution_instance_id=receipt['execution_instance_id'],
        cup=deepcopy(receipt['last_observation']['sample']['world_from_cup']),
        machine=deepcopy(receipt['actual_environment_from_machine_root']),
        receipt=deepcopy(reference),task_kind=receipt.get('task_kind','collection'))
    _save(Path(device.cup_return_root)/'scene-recovery-state.json',state)
    device.scene_recovery_state=state
    device.scene_recovery_error=None


def _remember_worker(request):
    from types import SimpleNamespace
    from .control_plan import load_artifact
    reference=request['native_receipt'];plan_ref=request['plan_ref']
    if plan_ref.get('schema')!='collection_plan_ref.v1' or plan_ref.get('environment')!='fake':
        raise ValueError('validated fake recovery plan reference required')
    receipt=json.loads(_read_pin(dict(path=reference['receipt_path'],sha256=reference['receipt_sha256'])))
    if (receipt.get('execution_instance_id')!=request['execution_instance_id']
            or receipt.get('motion_outcome')!='completed' or receipt.get('task_qualified') is not True
            or receipt.get('task_outcome')!='success'
            or receipt.get('runtime_nonce') is not None
                and receipt['runtime_nonce']!=plan_ref.get('origin',{}).get('runtime_nonce')):
        raise ValueError('verified native receipt differs from accepted recovery execution')
    device=SimpleNamespace(id=request['robot_id'],cup_return_root=Path(request['root']),
        scene_recovery_state=request['state'],cup_return_inputs=request['inputs'],
        collection_preparation=SimpleNamespace(snapshot=lambda:dict(output=request['preparation_output'])))
    runner=SimpleNamespace(execution_instance_id=request['execution_instance_id'],
        payload={key:load_artifact(plan_ref[key]) for key in ('plan','source')},
        recovery_parent_binding=load_artifact(plan_ref['binding']) if plan_ref.get('binding') else None)
    _remember_success_local(device,runner,receipt,reference)
    value=json.dumps(dict(state=device.scene_recovery_state,inputs=device.cup_return_inputs),allow_nan=False)
    if len(value.encode())>256*1024:raise ValueError('recovery result exceeds bounded metadata envelope')
    return value


def prepare(device,offset_xy_mm=None):
    if device.environment=='physical':
        from .physical_cup_return import prepare as prepare_physical
        return prepare_physical(device,offset_xy_mm)
    from cup_return_replay import build_replay_source,build_return_plan
    from integration import _digest
    from real_observation import read_initial_sample,seal_initial_state
    from real_contract import validate_real_plan
    if device.environment!='fake':raise ValueError('取回回放目前仅支持完整 fake')
    manifest=getattr(device,'cup_return_inputs',None)
    if not manifest:raise ValueError('没有已绑定的成功取放轨迹可用于取回')
    if ((getattr(device,'scene_recovery_state',{}) or {}).get('last_completed') or {}).get('task_kind') in ('cup_return','scene_initialization'):
        raise ValueError('场景已恢复；请先完成下一轮放杯，再恢复场景')
    parent={}
    for name in ('plan','source','binding','native_receipt','base'):
        raw=_read_pin(manifest[name])
        parent[name]=json.loads(raw)
    runtime=device.runtime
    state=device.snapshot('collection')
    if runtime is None or state.get('holder')!='collection' or not state['allowed_operations']['preview']['allowed']:
        raise ValueError('请等待当前动作结束，并将控制权交接给数采')
    if any(w.snapshot().get('busy') for w in device.workflows.values()):raise ValueError('当前业务仍在执行')
    binding_path=Path(runtime.owner.hardware_binding)
    installed=(device.collection_preparation.snapshot().get('installed') or {})
    if not installed.get('verified') or any(installed.get(k)!=parent['binding'].get(k) for k in ('head_id','wrist_id')):
        raise ValueError('原轨迹的标定组尚未核验为当前安装组')
    active=Path(installed['path'])
    for name,expected in parent['binding']['active_sha256'].items():
        if Path(name).name!=name or hashlib.sha256((active/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('当前安装文件与原轨迹不一致：'+name)
    current=json.loads(binding_path.read_text()) if binding_path.exists() else deepcopy(parent['binding'])
    for key in ('robot_id','camera_serials','calibration_profile_digest','head_id','wrist_id','model_binding'):
        if current.get(key)!=parent['binding'].get(key):raise ValueError('当前标定与原轨迹不一致：'+key)
    before=read_initial_sample(lambda:device.snapshot('collection'),environment='fake')
    scene=device.fake_scene_snapshot()
    after=read_initial_sample(lambda:device.snapshot('collection'),environment='fake')
    deadline=time.monotonic()+1.
    while after[1]['source_time_ns']<=before[1]['source_time_ns'] and time.monotonic()<deadline:
        time.sleep(.01)
        after=read_initial_sample(lambda:device.snapshot('collection'),environment='fake')
    initial=seal_initial_state(before,after,parent['source']['calibration_profile'])
    context=dict(schema_version='cup_return_context.v1',clock=initial['clock'],
        parent_execution_instance_id=parent['native_receipt']['execution_instance_id'],
        physical_configuration_sha256=scene['physical_configuration_sha256'],
        current_cup_pose=scene['cup'],current_machine_pose=scene['environment_from_machine_root'],
        parent_fixture_machine_pose=parent['base']['initial_suite']['episodes'][0]['environment_from_machine_root'],
        parent_fixture_sha256=manifest['base']['sha256'],pose_basis='parent_plant_fixture')
    origin=deepcopy((getattr(device,'scene_recovery_state',{}) or {}).get('origin') or parent['source']['scene']['environment_from_cup'])
    offset=[0.,0.] if offset_xy_mm is None else offset_xy_mm
    if (not isinstance(offset,list) or len(offset)!=2 or any(type(v) not in (int,float)
            or not math.isfinite(v) or abs(v)>50 for v in offset)):
        raise ValueError('前后、左右偏移须为 ±50 mm 内的有限数值')
    context.update(return_origin=origin,offset_xy_mm=offset,world_from_robot=scene.get('world_from_robot'))
    directory=Path(device.cup_return_root)/('cup-return-'+uuid4().hex)
    source=build_replay_source(parent['source'],initial,context,run_id=directory.name)
    goal=deepcopy(origin)
    for i in range(2):goal['position'][i]+=offset[i]/1000
    kinematics=None
    if any(abs(a-b)>1e-9 for a,b in zip(goal['position'],parent['source']['scene']['environment_from_cup']['position'])):
        from cup_return_offset import NativeReturnKinematics
        if 'urdf' not in manifest:raise ValueError('当前父轨迹缺少已核验的标定模型，请先完成一次正常数采准备')
        _read_pin(manifest['urdf'])
        kinematics=NativeReturnKinematics(manifest['urdf']['path'],source,context['world_from_robot'])
    plan=build_return_plan(parent['plan'],parent['source'],parent['native_receipt'],source,
        return_goal=goal,offset_xy_mm=offset,kinematics=kinematics)
    # Keep historical image hashes in the parent proof, not in this live feedback binding.
    binding={k:deepcopy(parent['binding'][k]) for k in ('active_sha256','calibration_accepted','calibration_origins',
        'calibration_profile_digest','calibration_snapshot_id','camera_serials','head_id','wrist_id','model_binding','robot_id')}
    binding.update(schema_version='fake_current_replay_binding.v1',observation_scope='historical_parent_only',
        environment='fake',simulation_only=True,evidence_source='fake_current_replay_context',
        source_digest=_digest(source),initial_state=initial,clock=initial['clock'],
        historical_parent_binding_sha256=manifest['binding']['sha256'])
    validate_real_plan(plan,source,binding,environment='fake')
    return publish(device,directory,plan,source,binding,current,parent['plan'],offset,origin,goal,state,scene=scene)


def publish(device,directory,plan,source,binding,current,parent_plan,offset,origin,goal,state,scene=None):
    """Publish one checked maintenance artifact to the same idle Owner."""
    from integration import _digest
    directory.mkdir(parents=True,exist_ok=False)
    for name,value in (('plan',plan),('source',source),('binding',binding),('previous-binding',current)):
        with (directory/(name+'.json')).open('x') as stream:json.dump(value,stream,ensure_ascii=False,allow_nan=False)
    if scene is not None:_save(directory/'scene.json',scene)
    (directory/'binding-current.json').write_text(json.dumps(binding,allow_nan=False))
    metadata=dict(timeline_digest=plan['timeline_digest'],duration_s=plan['duration_s'],
        parent_plan_id=parent_plan['run_id'],parent_speed_scale=parent_plan.get('normal_recipe',{}).get('requested_speed_scale',1.),
        source_digest=_digest(source),offset_xy_mm=offset,origin_position_m=origin['position'],
        goal_position_m=goal['position'],metal_gap_mm=plan['gripper_distance_control']['closed_metal_gap_mm'])
    return activate_prepared(device,directory,metadata,current,state)


def activate_prepared(device,directory,metadata,current,state):
    """Activate only small metadata and file references in the live SDK process."""
    runtime=device.runtime;binding_path=Path(runtime.owner.hardware_binding)
    staged=directory/'binding-current.json'
    with runtime.lock:
        latest=device.snapshot('collection')
        if (device.runtime is not runtime or latest['command_epoch']!=state['command_epoch']
                or latest['runtime_nonce']!=state['runtime_nonce']
                or not latest['allowed_operations']['preview']['allowed']):
            raise ValueError('准备取回期间控制状态已变化，请重新生成')
        staged.replace(binding_path)
    from .control_plan import ArtifactPreview
    try:preview=device.submit('collection',ArtifactPreview(plan_path=directory/'plan.json',source_path=directory/'source.json'))
    except Exception:
        with runtime.lock:
            if json.loads(binding_path.read_text()).get('source_digest')==metadata['source_digest']:
                _save(binding_path,current)
        raise
    speed=metadata['parent_speed_scale'];offset=metadata['offset_xy_mm']
    task={k:metadata[k] for k in ('timeline_digest','duration_s','parent_plan_id','parent_speed_scale','source_digest',
        'offset_xy_mm','origin_position_m','goal_position_m')}
    task.update(directory=str(directory),submitted=False,environment=device.environment,media_id=directory.name,
        return_policy=metadata.get('return_policy'),
        binding_path=str(binding_path),plan_sha256=[p['sha256'] for p in preview.get('plans',[])],
        runtime_nonce=preview['runtime_nonce'],command_epoch=preview['command_epoch'])
    device.cup_return_task=task
    preview.update(purpose='cup_return',speed_scale=speed,offset_xy_mm=offset,
        origin_position_m=metadata['origin_position_m'],goal_position_m=metadata['goal_position_m'])
    physical=device.environment=='physical';unit='秒' if physical else '个仿真秒'
    gap=metadata['metal_gap_mm']
    collision=('直接复用本轮已完成轨迹，未重新检测、IK或碰撞检查。' if task['return_policy']=='completed-parent-reverse-no-replanning-v1'
        else '当前场景路径检查已通过。') if physical else '未重新检查碰撞。'
    return dict(preview=preview,summary=f"恢复场景：从咖啡机取杯，放到固定原起点前后 {offset[0]:+g} mm、左右 {offset[1]:+g} mm；{metadata['duration_s']:.2f} {unit}。"
        f'抓杯金属间距{gap:g} mm；{collision}保留位置与动态限值。只执行维护动作，不录制训练数据。')


def clear(device,*,maintenance_actor=None):
    task=getattr(device,'cup_return_task',None)
    if not task:return
    state=device.snapshot('collection')
    if task.get('submitted') and view(device,state,{}).get('busy'):
        raise ValueError('场景恢复确认已提交，请等待本次动作结束')
    if maintenance_actor is not None:
        runtime=device.runtime
        if runtime is None or maintenance_actor.action!='ops_prepare_collection':
            raise ValueError('清理恢复绑定需要当前数采准备维护身份')
        runtime.admit_business('collection',maintenance_actor=maintenance_actor)
    elif not state['allowed_operations']['preview']['allowed']:raise ValueError('请先等待取回动作结束')
    path=Path(task['binding_path'])
    current=json.loads(path.read_text())
    if current.get('source_digest')==task['source_digest']:
        staged=Path(task['directory'])/'restore-binding.json'
        staged.write_bytes((Path(task['directory'])/'previous-binding.json').read_bytes())
        staged.replace(path)
    device.cup_return_task=None


def view(device,runtime,value):
    task=getattr(device,'cup_return_task',None)
    if not task:return value
    physical=task.get('environment')=='physical'
    result=deepcopy(value);frozen=runtime.get('frozen') or {}
    submitted=task['submitted']
    context_changed=(runtime.get('runtime_nonce') is not None and type(runtime.get('command_epoch')) is int
        and (runtime['runtime_nonce'],runtime['command_epoch'])!=(task['runtime_nonce'],task['command_epoch']))
    matching=not context_changed and frozen.get('timeline_digest')==task['timeline_digest']
    if submitted:
        accepted=runtime.get('execution_calibration') or {}
        matching=matching and accepted.get('accepted_monotonic_s',0)>=task.get('submitted_monotonic_s',float('inf'))
    status='awaiting_review';motion='未执行';native={}
    message='取回计划已准备，请审阅后确认执行。'
    if submitted:
        status='running';motion='running';message='正在沿原轨迹取杯并放回起始位置。'
        if matching:
            motion=frozen.get('status','unknown')
            candidate=runtime.get('native_task') or {}
            if (candidate.get('execution_instance_id') and candidate['execution_instance_id']==runtime.get('execution_instance_id')
                    and candidate.get('execution_timeline_digest')==frozen.get('execution_timeline_digest')):native=candidate
            if motion in ('completed','failed','stopped'):
                status='completed' if motion=='completed' else 'failed'
                message=('取回动作完成；实际杯位待相机观测确认。' if physical else '取回动作完成，正在核验杯位。') if status=='completed' else str(frozen.get('reason') or '取回动作未完成')
                if native.get('task_qualified'):message='杯子已放到指定恢复位置，机械臂已回位。'
                elif native.get('task_outcome') in ('failed','unverifiable'):
                    status='failed'
                    failure=native.get('first_failure') or {}
                    reason=failure.get('reason') or native.get('message') or '未提供原生失败原因'
                    stage=(failure.get('applied_context') or {}).get('stage')
                    phase={'lower':'下降放杯','open':'松爪','lift':'取杯抬升','transport':'持杯搬运','retreat':'撤手'}.get(stage,stage)
                    message='动作已结束，过程验收未通过：'+(phase+'阶段，' if phase else '')+str(reason)+'。'
                    speed,limit=failure.get('contact_speed_m_s'),failure.get('contact_speed_limit_m_s')
                    if isinstance(speed,(int,float)) and isinstance(limit,(int,float)) and speed>limit:
                        message+=f' 指爪与杯子相对速度 {speed*1000:.2f} mm/s，当前门槛 {limit*1000:.2f} mm/s。'
        rejection=runtime.get('last_confirm_rejection') or {}
        if context_changed:
            status='superseded';motion='context_superseded'
            message='控制上下文已变化，本取回任务不再执行；原任务结果以执行记录为准。'
        elif (task.get('plan_sha256') and rejection.get('plan_sha256')==task['plan_sha256']
                and rejection.get('command_epoch')==task['command_epoch'] and rejection.get('motion_started') is False
                and rejection.get('monotonic_s',0)>=task.get('submitted_monotonic_s',float('inf'))):
            status='failed';motion='未执行';message='执行确认已过期或撤销，动作未启动；请重新生成取回计划。'
        elif not matching and (runtime.get('recoverable_fault') or runtime.get('state')=='FAULT'):
            status='failed';message=str(runtime.get('reason') or '取回动作未启动，请查看控制故障')
    result.update(id=task['directory'],status=status,busy=status=='running',
        stage_id='collection.return_review' if not submitted else 'collection.return_execute' if status=='running' else 'collection.return_complete',
        title='取回杯子',message=message,failure=None,request=None,progress=None,speed_scale=task['parent_speed_scale'],
        primary_action=dict(id='execute',label='审阅后执行取回',allowed=not submitted),
        outcomes=dict(motion=motion,data='维护动作，过程录像独立保存' if physical else '未请求录制',
            simulation=('复用本轮已完成轨迹；未重新检测或碰撞检查' if task.get('return_policy')=='completed-parent-reverse-no-replanning-v1'
                else '当前路径检查通过') if physical else '未重新碰撞检查',task=native.get('task_outcome','待核验')))
    if status=='failed':
        result['failure']=dict(message=message,recovery='先核对当前杯位与首次失败阶段；过程失败不代表动作未执行，不要直接重复取杯。')
    collection=result.setdefault('collection',{})
    collection.setdefault('actions',{name:dict(id=action,label=label,allowed=False) for name,action,label in (
        ('prepare','collection_prepare','初始化与预热'),('plan','collection_plan','观测与规划'),('execute','execute','审阅后执行取回'))})
    collection.update(return_task=dict(status=status,message=message,duration_s=task['duration_s'],
        parent_speed_scale=task['parent_speed_scale'],parent_plan_id=task['parent_plan_id'],
        **{key:deepcopy(task[key]) for key in ('offset_xy_mm','origin_position_m','goal_position_m') if key in task}),
        recording={},errors={},native_task=native,simulation=None,timing={})
    if task.get('media_id'):
        result['trajectory']=dict(id=task['media_id'],timeline_digest=task['timeline_digest'],
            duration_s=task['duration_s'],speed_scale=task['parent_speed_scale'],
            url='/api/result-media?environment='+('physical' if physical else 'fake')+'&business=collection&result_id='+task['media_id'])
    result['details']=[dict(label='父轨迹',value=task['parent_plan_id']),dict(label='取回计划时长',value=f"{task['duration_s']:.2f} "+('秒' if physical else '个仿真秒'))]
    return result


if __name__=='__main__':
    if sys.argv[1:]!=['--remember']:raise SystemExit('private recovery worker only')
    raw=sys.stdin.buffer.read(256*1024+1)
    if len(raw)>256*1024:raise SystemExit('bounded recovery metadata required')
    print(_remember_worker(json.loads(raw)))
