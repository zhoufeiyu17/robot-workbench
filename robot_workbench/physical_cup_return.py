"""Publish externally checked physical recovery plans; never plan inside the SDK Owner."""
import json
import os
import subprocess
import sys
from pathlib import Path
import time
from uuid import uuid4

from .cup_return import _read_pin, installed_pair, activate_prepared


def prepare(device,offset_xy_mm=None):
    from joint_space import state_near
    from real_observation import read_initial_sample
    path=getattr(device,'physical_cup_return_candidate',None)
    if not path or not Path(path).is_file():raise ValueError('当前真机还原计划尚未完成观测与路径检查')
    candidate=json.loads(Path(path).read_text())
    if candidate.get('schema')!='physical_cup_return_candidate.v1':raise ValueError('真机还原候选格式不匹配')
    directory=Path(device.cup_return_root)/('physical-return-'+uuid4().hex)
    directory.parent.mkdir(parents=True,exist_ok=True)
    # JSON decoding/validation can hold the GIL longer than the live 100ms budget.
    from .recording_process import _environment
    executable=os.environ.get('WORKBENCH_CPU_PYTHON') or sys.executable
    with directory.with_suffix('.log').open('xb') as log:
        result=subprocess.run([executable,'-B','-m',__name__,'--stage',str(path),str(directory)],
            stdout=subprocess.PIPE,stderr=log,text=True,timeout=120,env=_environment())
    if result.returncode:raise ValueError('取回计划隔离校验未完成，未下发动作；详细原因已保存到诊断记录。')
    if len(result.stdout.encode())>256*1024:raise ValueError('还原计划元数据超出限制')
    prepared=json.loads(result.stdout);metadata=prepared['metadata'];binding=prepared['binding']
    if any(prepared.get(k)!=candidate.get(k)for k in ('runtime_nonce','command_epoch')):
        raise ValueError('准备取回期间候选身份变化')
    offset=metadata['offset_xy_mm']
    if offset!=([0.,0.] if offset_xy_mm is None else offset_xy_mm):
        raise ValueError('偏移与已检查的真机还原计划不同，请先生成对应路径')
    consumed=Path(path).parent/('submitted-'+metadata['source_digest']+'.json')
    if consumed.exists():raise ValueError('本还原计划已提交过执行；先核对现场，不能重复执行旧计划')
    state=device.snapshot('collection');runtime=device.runtime
    if (runtime is None or state.get('holder')!='collection' or not state['allowed_operations']['preview']['allowed']
            or any(flow.snapshot().get('busy') for flow in device.workflows.values())):
        raise ValueError('请等待动作结束，并将控制权交接给数采')
    if any(candidate.get(k)!=state.get(k) for k in ('runtime_nonce','command_epoch')):
        raise ValueError('真机连接或控制权已变化，请用新反馈重新生成还原计划')
    measured,evidence=read_initial_sample(lambda:device.snapshot('collection'),environment='physical')
    initial=prepared['initial_state']
    if (evidence.get('pid')!=initial['owner_samples'][-1]['pid']
            or evidence.get('disabled_grippers',[])!=initial.get('disabled_grippers',[])
            or not state_near(measured,initial['planned_reported'],disabled_grippers=initial.get('disabled_grippers',[]))):
        raise ValueError('当前真机初态与还原计划不同，请重新观测和生成路径')
    current=json.loads(Path(runtime.owner.hardware_binding).read_text())
    installed=installed_pair(device)
    if not installed.get('verified'):raise ValueError('当前标定安装组未就绪')
    for key in ('robot_id','camera_serials','calibration_profile_digest','head_id','wrist_id','model_binding','active_sha256'):
        if current.get(key)!=binding.get(key):raise ValueError('还原计划与当前标定不同：'+key)
    if any(installed.get(k)!=binding.get(k) for k in ('head_id','wrist_id')):
        raise ValueError('还原计划与当前安装组不同')
    import hashlib
    for name,sha in binding['active_sha256'].items():
        if Path(name).name!=name or hashlib.sha256((Path(installed['path'])/name).read_bytes()).hexdigest()!=sha:
            raise ValueError('当前安装文件变化：'+name)
    (directory/'previous-binding.json').write_text(json.dumps(current,allow_nan=False))
    result=activate_prepared(device,directory,metadata,current,state)
    device.cup_return_task['consumption_path']=str(consumed)
    return result


def mark_submitted(device,review_id):
    """Persist before submitting; an ambiguous control response never authorizes replay."""
    task=device.cup_return_task
    if task.get('environment')!='physical':return
    with Path(task['consumption_path']).open('x') as stream:
        json.dump(dict(schema='physical_cup_return_submission.v1',review_id=review_id,
            source_digest=task['source_digest'],timeline_digest=task['timeline_digest'],
            submitted_wall_ns=time.time_ns(),outcome='pending'),stream)


def stage_candidate(path,directory):
    """Private CPU worker; never imports or calls the device SDK."""
    from integration import _digest
    from physical_cup_maintenance import validate_source,validate_evidence
    candidate=json.loads(Path(path).read_text())
    if candidate.get('schema')!='physical_cup_return_candidate.v1':raise ValueError('真机还原候选格式不匹配')
    raw={name:_read_pin(candidate[name]) for name in ('source','plan','binding')}
    source,plan,binding=(json.loads(raw[name]) for name in ('source','plan','binding'))
    validate_source(source,binding);validate_evidence(plan,source)
    from .control_plan import MAX_INPUT
    from physical_cup_maintenance import execution_package
    raw['plan'],proof_files=execution_package(plan,Path(directory).absolute())
    if len(raw['plan'])>MAX_INPUT:
        raise ValueError('取回计划超过执行文件大小上限；请拆分证明材料，不得放宽控制入口限制')
    context=source['maintenance_context'];parent=plan['maintenance_derivation']['parent_plan']
    metadata=dict(timeline_digest=plan['timeline_digest'],duration_s=plan['duration_s'],
        parent_plan_id=parent['run_id'],parent_speed_scale=parent.get('normal_recipe',{}).get('requested_speed_scale',1.),
        source_digest=_digest(source),offset_xy_mm=context['offset_world_xy_mm'],
        origin_position_m=context['return_origin']['position'],goal_position_m=context['return_goal']['position'],
        metal_gap_mm=plan['gripper_distance_control']['closed_metal_gap_mm'])
    directory=Path(directory);directory.mkdir(exist_ok=False)
    for name,value in proof_files.items():(directory/name).write_bytes(value)
    for name,value in raw.items():(directory/(name+'.json')).write_bytes(value)
    (directory/'binding-current.json').write_bytes(raw['binding'])
    return dict(metadata=metadata,binding=binding,initial_state=source['initial_state'],
        runtime_nonce=candidate['runtime_nonce'],command_epoch=candidate['command_epoch'])


if __name__=='__main__':
    if len(sys.argv)!=4 or sys.argv[1]!='--stage':raise SystemExit('private staging worker required')
    print(json.dumps(stage_candidate(sys.argv[2],sys.argv[3]),allow_nan=False))
