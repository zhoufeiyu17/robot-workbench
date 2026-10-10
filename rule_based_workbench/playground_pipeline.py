"""Three playground jobs; reuse acquisition, pinned DSW transport and local raw exporter."""
from copy import deepcopy
from pathlib import Path
from uuid import uuid4
import hashlib
import json
import re
import shlex
import shutil
import subprocess
import time

from integration import _digest
from run_pipeline import load_config, python_command, ssh, transport, lane, bounded_command, AFFINITY_EXEC, isaac_environment
from detector_service import gpu_check_command
from real_contract import namespace, validate_namespace, owner_matches_environment
from launch_environment import HOST_PYTHON,clean_command

NATIVE_STAGE_STATUS = {'playground-geometry-inputs':'geometry-inputs-ready',
                       'playground-first-frame':'first-frame-ready',
                       'playground-plan':'geometric-plan-ready','playground-replay':'completed'}

def stage_label(name):
    labels = {'initialize': '初始化本轮目录', 'shared-storage': '校验共享存储',
              'physical-capture-import': '导入真机观测', 'detect': '检测杯子与咖啡机',
              'physical-detection-bind': '绑定有效检测窗口', 'bridge': '构造检测场景',
              'physical-model-bind': '绑定标定模型', 'playground-geometry-inputs': '构造简化场景与渲染首帧',
              'playground-plan': 'CPU 轨迹规划与碰撞检查'}
    return '远端环境预检 · ' + name.removeprefix('preflight-') if name.startswith('preflight-') else labels.get(name, name)


def save(path,value):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2))
    temporary.replace(path)


def verify_l3_derivation(sim, saved, plan, source, timeline, *, spec=None):
    """Verify association and adjusted targets separately; never trust replay limits."""
    if spec is None:
        from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
        spec=ac_one_dual_arm_zero_home_action_spec()
    from playground_sim import validate_l3_replay_derivation
    derivation=validate_l3_replay_derivation(plan,source,timeline,spec,saved)['l3_derivation']
    expected=dict(execution_timeline_digest=timeline['timeline_digest'],
        l3_derivation_digest=derivation['derivation_digest'],
        l3_commands_digest=derivation['derived_commands_digest'],
        replay_matches_shared_commands=derivation['replay_matches_shared_commands'])
    if any(sim.get(key)!=value for key,value in expected.items()):
        raise ValueError('L3 result differs from independently verified command derivation')
    return dict(passed=True,scope='L3 auxiliary command derivation; no task qualification',**expected)


def prepare_environment(config,progress=None,cancel=None):
    cfg,contract,_=load_config(Path(config['pipeline_config']))
    checks=[]
    # Only administrator-configured, task-owned user units; no HTTP-supplied commands.
    units=config.get('preview_units',[])
    if units:
        if (not isinstance(units,list) or any(not isinstance(u,str) or
                not u.startswith('collection-preview-') or not u.endswith('.service') or '/' in u for u in units)):
            raise ValueError('only task-owned collection preview units are allowed')
        subprocess.run(['systemctl','--user','start',*units],check=True,timeout=30,capture_output=True)
    for side in ('simulated-real','dsw-scene'):
        if cancel is not None and cancel():raise InterruptedError('环境准备已中断')
        if progress: progress('检查 '+side+' 与候选代码', next_label='检查仿真 DSW' if side=='simulated-real' else '等待检测器、控制与三路相机就绪')
        script='import os,pathlib,sys; assert all(pathlib.Path(p).exists() and os.access(p,os.R_OK) for p in sys.argv[1:]); print("ready")'
        paths=[cfg['workbench_code']+'/playground_sim.py',cfg['native_code'],cfg['assets'],'/isaac-sim/python.sh']
        backend=cfg.get('planning_backend','native')
        gpu_planning = side=='dsw-scene' and backend in ('curobo','motiongen')
        if gpu_planning:
            paths += [cfg['curobo_root']+'/'+name for name in ('python/curobo/__init__.py','deps','cuda-overlay/lib64')]
            names=('motiongen_plan.py','motiongen_motion.py','physical_motiongen.py','control_motion.py','curobo_worker.py') if backend=='motiongen' else ('curobo_collision.py','curobo_whole_geometry.py','curobo_relative_geometry.py')
            paths += [cfg['workbench_code']+'/'+name for name in names]
        from cancellable_process import run
        run(transport(cfg,side,python_command(script,*paths),timeout=25),check=True,timeout=25,capture_output=True,cancel=cancel)
        channel='本机' if cfg.get('execution') else 'SSH'
        checks.append(dict(key=side,label=('检测 DSW' if side=='simulated-real' else '仿真 DSW')+' · 连接与资产预检',
                           status='ready',message=(channel+'、候选代码、资产与 cuRobo 依赖可读；GPU 规划在规划步骤初始化，尚未常驻预热'
                               if gpu_planning else channel+'、候选代码与资产可用；Isaac 在规划步骤初始化，尚未常驻预热')))
    return checks


def native_stage(cfg, mode, output, *, speed=.3, plan_path=None, execution_timeline_path=None, timeout_s=600):
    root=cfg['remote_run_root']; wb=cfg['workbench_code']
    args=['/isaac-sim/python.sh','-u','-B',wb+'/playground_sim.py','--mode',mode,
          '--source',root+'/detection-bridge/source.json','--suite',root+'/detection-bridge/suite.json',
          '--assets',cfg['assets'],'--hardware-binding',root+'/detection-bridge/hardware-binding.json',
          '--output',output,'--gpu',str(lane(cfg,'dsw-scene')['gpu']),'--instance',lane(cfg,'dsw-scene')['instance_id'],'--run-id',Path(root).name,
          '--finger-envelope',cfg['finger_envelope'],'--finger-envelope-sha256',cfg['finger_envelope_sha256'],
          '--speed-scale',str(speed)]
    if plan_path: args+=['--plan',plan_path]
    if execution_timeline_path:args+=['--execution-timeline',execution_timeline_path]
    if cfg.get('environment')=='fake':
        args+=['--environment','fake']
        if cfg.get('cup_contact_preset') is not None:args+=['--cup-contact-preset',json.dumps(cfg['cup_contact_preset'])]
    if cfg.get('execution'):args+=['--execution-json',json.dumps(cfg['execution'])]
    environment=isaac_environment(cfg,'dsw-scene')
    seconds=timeout_s
    command=gpu_check_command(cfg,'dsw-scene')+' && '+shlex.join(clean_command([HOST_PYTHON,'-I','-B','-c',AFFINITY_EXEC,*args],environment))
    return dict(name='playground-'+mode,side='dsw-scene',timeout_s=seconds,
                result_path=output+'/result.json',
                **({'execution_config':{'execution':cfg['execution'],'workbench_code':cfg['workbench_code']},
                    'terminate_grace_s':25,'failure_result':output+'/result.json'} if cfg.get('execution') else {}),
                argv=transport(cfg,'dsw-scene',shlex.join(['timeout','-k','30',str(seconds),'bash','-c',command]),timeout=seconds,
                    **({'failure_result':output+'/result.json'} if cfg.get('execution') else {})))


def geometry_stage(cfg,output,*,speed=.3,initial=None):
    root=cfg['remote_run_root'];wb=cfg['workbench_code']
    backend=cfg.get('planning_backend','native');gpu=backend in ('curobo','motiongen') and not initial
    args=['/isaac-sim/python.sh','-u','-B',wb+'/playground_geometry.py',
          '--input',root+'/playground-scene/planning-input.json','--source',root+'/detection-bridge/source.json',
          '--native-code',cfg['native_code'],'--assets',cfg['assets'],'--output',output,'--speed-scale',str(speed),
          '--backend',backend]
    if initial:args+=['--initial',initial]
    expected='geometric-plan-ready'
    if backend=='motiongen' and not initial:
        expected='motiongen-plan-ready'
        binding=root+'/detection-bridge/hardware-binding.json'
        if cfg.get('planner_service'):
            from planner_service import private_spec_arguments
            private=private_spec_arguments(cfg)
            request=dict(input=root+'/playground-scene/planning-input.json',source=root+'/detection-bridge/source.json',
                         output=output,speed_scale=speed,physical_binding=binding)
            if cfg.get('environment')=='fake':request['environment']='fake'
            args=['/isaac-sim/python.sh','-u','-B',wb+'/curobo_worker.py','--request',json.dumps(request),
                  '--service-spec-file',private['spec_path'],'--spec-sha256',private['spec_sha256']]
        else:
            args=['/isaac-sim/python.sh','-u','-B',wb+'/motiongen_plan.py',
                  '--input',root+'/playground-scene/planning-input.json','--source',root+'/detection-bridge/source.json',
                  '--physical-binding',binding,'--native-code',cfg['native_code'],'--assets',cfg['assets'],
                  '--output',output,'--speed-scale',str(speed)]
    if cfg.get('environment')=='fake' and not (backend=='motiongen' and not initial and cfg.get('planner_service')):
        args+=['--environment','fake']
    if cfg.get('execution') and not (backend=='motiongen' and not initial and cfg.get('planner_service')):
        args+=['--execution-json',json.dumps(cfg['execution'])]
    environment={'CUDA_VISIBLE_DEVICES':str(lane(cfg,'dsw-scene')['gpu']) if gpu else '', 'PYTHONNOUSERSITE':'1','PYTHONDONTWRITEBYTECODE':'1',
        'OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','PYTHONPATH':cfg['deps']+':'+wb+':'+str(Path(wb).parent)}
    if cfg.get('execution'):environment['ROBOT_WORKBENCH_SESSION_ID']=cfg['execution']['session_id']
    if gpu:
        install=cfg['curobo_root'];cache=root+'/cache/curobo'
        environment.update(PYTHONPATH=install+'/python:'+install+'/deps:'+environment['PYTHONPATH'],
            CUDA_HOME=install+'/cuda-overlay',LD_LIBRARY_PATH=install+'/cuda-overlay/lib64',
            TORCH_EXTENSIONS_DIR=cache+'/torch',WARP_CACHE_PATH=cache+'/warp',XDG_CACHE_HOME=cache,
            MAX_JOBS='4',TORCH_CUDA_ARCH_LIST='12.0')
    command=shlex.join(clean_command([HOST_PYTHON,'-I','-B','-c',AFFINITY_EXEC,*args],environment))
    if gpu:command=gpu_check_command(cfg,'dsw-scene')+' && '+command
    return dict(name='playground-plan',side='dsw-scene',timeout_s=600,
                expected_native_status=expected,
                label=('GPU 碰撞检查与规则轨迹规划' if gpu else '复用已验证轨迹并调整速度'
                       if backend=='curobo' and initial else stage_label('playground-plan')),
                result_path=output+'/result.json',
                **({'execution_config':{'execution':cfg['execution'],'workbench_code':cfg['workbench_code']}} if cfg.get('execution') else {}),
                argv=transport(cfg,'dsw-scene',shlex.join(['timeout','-k','30','600','bash','-c',command]),timeout=600))


def fetch(remote, local, *, limit=64*1024*1024, cancel=None, execution_config=None):
    local=Path(local); local.parent.mkdir(parents=True,exist_ok=True)
    script='import pathlib,sys; p=pathlib.Path(sys.argv[1]); assert p.is_file() and p.stat().st_size<=int(sys.argv[2]); sys.stdout.buffer.write(p.read_bytes())'
    temporary=local.with_suffix(local.suffix+'.partial')
    try:
        with temporary.open('wb') as stream:
            from cancellable_process import run
            run(transport(execution_config or {},'dsw-scene',python_command(script,remote,limit),timeout=120),stdout=stream,check=True,timeout=120,cancel=cancel)
        temporary.replace(local)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def run_stages(stages,output,contract,progress=None,cancel=None):
    output=Path(output); (output/'logs').mkdir(parents=True,exist_ok=True)
    deadline=time.monotonic()+contract['resources']['max_wall_time_minutes']*60
    receipts=[]
    for index,stage in enumerate(stages):
        if cancel is not None and cancel():raise InterruptedError('当前阶段已中断')
        label=stage.get('label',stage_label(stage['name']))
        following=stages[index+1].get('label',stage_label(stages[index+1]['name'])) if index+1<len(stages) else '读取并校验规划产物'
        rows=[dict(label=r['label'],status=r['status'],elapsed_s=r['actual_s']) for r in receipts]
        if progress: progress(label,stage_status='running',next_label=following,
                              stages=rows+[dict(label=label,status='running',elapsed_s=0)])
        started=time.monotonic(); argv,seconds=bounded_command(stage,deadline-time.monotonic())
        receipt=dict(stage=stage['name'],label=label,status='running')
        failure_announced=False
        def poll_failure():
            nonlocal failure_announced
            if failure_announced:return
            from owned_process import read_failure_result
            failure=read_failure_result(stage['failure_result'])
            if failure is not None:
                failure_announced=True
                detail=failure.get('error') or '原生阶段报告失败'
                progress('阶段失败，正在清理本轮计算进程：'+str(detail),stage_status='failed',
                    next_label='等待本轮计算进程退出',error=str(detail),
                    stages=rows+[dict(label=label,status='failed',elapsed_s=time.monotonic()-started,error=str(detail))])
        try:
            destination=output/stage['output'] if stage.get('output') else output/'logs'/(stage['name']+'.log')
            destination.parent.mkdir(parents=True,exist_ok=True)
            with destination.open('wb') as stream:
                from cancellable_process import run
                result=run(argv,stdout=stream,stderr=subprocess.STDOUT if not stage.get('output') else subprocess.PIPE,
                           timeout=seconds+40,cancel=cancel,
                           **({'terminate_grace_s':stage['terminate_grace_s']} if 'terminate_grace_s' in stage else {}),
                           **({'on_poll':poll_failure} if progress and stage.get('failure_result') else {}))
            receipt['exit_code']=result.returncode
            if stage['name'] in NATIVE_STAGE_STATUS:
                remote_result=stage.get('result_path')
                if not remote_result:
                    raise RuntimeError('native stage result path missing: '+stage['name'])
                local_result=output/'logs'/(stage['name']+'.result.json')
                try:
                    fetch(remote_result,local_result,limit=4*1024*1024,cancel=cancel,execution_config=stage.get('execution_config'))
                    outcome=json.loads(local_result.read_text())
                except InterruptedError:raise
                except Exception as error:
                    raise RuntimeError(f'{stage["name"]} exited {result.returncode} but native result is missing or unreadable: {remote_result}') from error
                receipt.update(native_status=outcome.get('status') if isinstance(outcome,dict) else None,
                               native_result_path=str(local_result))
                if isinstance(outcome,dict) and outcome.get('error'):receipt['native_error']=outcome['error']
                expected=stage.get('expected_native_status',NATIVE_STAGE_STATUS[stage['name']])
                if not isinstance(outcome,dict) or outcome.get('status')!=expected:
                    detail=outcome.get('error',outcome.get('status')) if isinstance(outcome,dict) else 'not an object'
                    raise RuntimeError(f'{stage["name"]} native result did not complete: {detail}; see {local_result}')
            if result.returncode: raise RuntimeError(f'{stage["name"]} failed ({result.returncode}); see {destination}')
            receipt['status']='completed'
        except Exception as error:
            receipt.update(status='failed',error=str(error)); raise
        finally:
            receipt['actual_s']=time.monotonic()-started; receipts.append(receipt)
            save(output/'timings.json',receipts)
            if progress: progress(label,stage_status=receipt['status'],next_label=following if receipt['status']=='completed' else '检查失败原因',
                                  stages=[dict(label=r['label'],status=r['status'],elapsed_s=r['actual_s']) for r in receipts])
    return receipts


def plan(config,output,bridge,speed_scale,progress=None,cancel=None):
    from real_acquisition import prepare_inputs
    from real_pipeline import preparation_stages
    from real_contract import validate_real_plan
    from frozen_executor import validate_plan_evidence
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    cfg,contract,_=load_config(Path(config['pipeline_config']))
    active,_,_=load_config(Path(bridge.config['preparation']['pipeline_config']))
    environment=namespace(bridge.config.get('environment','physical'))['environment']
    if any(value.get('environment','physical')!=environment for value in (cfg,active)):
        raise ValueError('workbench and planning/acquisition device environments differ')
    service,active_service=cfg.get('detector_service'),active.get('detector_service')
    if bool(service)!=bool(active_service): raise ValueError('candidate and active detector startup differ')
    if service:
        ownership={'root','socket','owner_token'}
        if {k:v for k,v in service.items() if k not in ownership}!={k:v for k,v in active_service.items() if k not in ownership}:
            raise ValueError('candidate and active detector identity/resources differ')
        contract['filesystem']['writable_roots']=[active_service['root'] if p==service['root'] else p
                                                for p in contract['filesystem']['writable_roots']]
        cfg['detector_service']=deepcopy(active_service)
        contract['detector_service']=deepcopy(active_service)
    if cfg.get('planning_backend')=='motiongen' and cfg.get('planner_service')!=active.get('planner_service'):
        raise ValueError('candidate 与当前工作台规划进程身份不一致；尚未开始采集观测')
    previous=cfg['remote_run_root']; cfg['remote_run_root']=previous+'-playground-'+uuid4().hex
    contract['filesystem']['writable_roots']=[cfg['remote_run_root'] if p==previous else p for p in contract['filesystem']['writable_roots']]
    cfg.update(render_replay=False,contract_path=str(output/'contract.json'))
    save(output/'config.json',cfg)
    if 'config_sha256' in contract.get('provenance',{}):
        contract['provenance']['config_sha256']={'config.json':hashlib.sha256((output/'config.json').read_bytes()).hexdigest()}
    save(output/'contract.json',contract)
    acquisition=deepcopy(bridge.config['preparation'])
    acquisition['environment']=environment
    acquisition['pipeline_config']=str(output/'config.json')
    if progress: progress('采集观测并冻结实测起点',next_label='导入观测并检测杯子与咖啡机')
    prepared=prepare_inputs(acquisition,output,read_snapshot=lambda:bridge.camera('base_0','snapshot.zip')[0],
                            read_owner=bridge.owner,cancel=cancel)
    cfg,contract,_=load_config(Path(prepared['pipeline_config']))
    # No previous full rollout, no duplicate geometry solve, and no replay before execution.
    stages=preparation_stages(cfg,contract,prepared['capture_dir'],prepared['physical_binding'],scene_only=True)
    remote=cfg['remote_run_root']+'/playground-plan'
    stages.append(native_stage(cfg,'geometry-inputs',cfg['remote_run_root']+'/playground-scene'))
    stages.append(geometry_stage(cfg,remote,speed=speed_scale))
    run_stages(stages,output,contract,progress,cancel)
    if progress: progress('读取并校验规划产物',next_label='核对首帧后点击 action 真机执行')
    fetch(remote+'/result.json',output/'native-result.json',cancel=cancel,execution_config=cfg)
    native_result=json.loads((output/'native-result.json').read_text())
    if native_result.get('status') not in ('geometric-plan-ready','motiongen-plan-ready'):
        raise ValueError('原生几何规划失败：'+str(native_result.get('error','missing planning receipt')))
    for remote_name,local_name in [('detection-bridge/source.json','source.json'),
                                 ('detection-bridge/hardware-binding.json','hardware-binding.json'),
                                 ('playground-plan/plan.json','plan.json'),('playground-scene/first-frame.png','first-frame.png')]:
        fetch(cfg['remote_run_root']+'/'+remote_name,output/local_name,cancel=cancel,execution_config=cfg)
    source=json.loads((output/'source.json').read_text()); result=json.loads((output/'plan.json').read_text())
    binding=json.loads((output/'hardware-binding.json').read_text())
    validate_real_plan(result,source,binding,environment=environment); validate_plan_evidence(result,source)
    return dict(validated=True,source_path=str(output/'source.json'),plan_path=str(output/'plan.json'),
                hardware_binding=str(output/'hardware-binding.json'),first_frame_path=str(output/'first-frame.png'),
                config_path=prepared['pipeline_config'],environment=environment,speed_scale=speed_scale,duration_s=result['duration_s'],timeline_digest=result['timeline_digest'],
                summary=f'{len(result["ticks"])} 个目标，{result["duration_s"]:.2f} 秒；几何检查通过，动态仿真尚未执行')


def restore_plan(config,directory,bridge):
    """Restore a checked pre-action checkpoint, never resume physical commands."""
    from real_contract import validate_real_plan
    from frozen_executor import validate_plan_evidence
    from real_recording import verify_raw
    directory=Path(directory).resolve()
    checkpoint=directory/'prepared-plan.json'
    if not checkpoint.is_file():return None
    saved=json.loads(checkpoint.read_text())
    if saved.get('schema_version')!='playground_prepared_plan.v1':
        raise ValueError('未知 Playground 阶段检查点版本')
    if saved.get('invalidated_by_interrupt') is True:return None
    prepared=deepcopy(saved['plan'])
    def artifact(value):
        path=Path(value).resolve()
        if not path.is_relative_to(directory/'jobs') or not path.is_file():
            raise ValueError('恢复计划的文件缺失或不在任务目录')
        return path
    source,plan_value,binding=[json.loads(artifact(prepared[key]).read_text())
                             for key in ('source_path','plan_path','hardware_binding')]
    environment=bridge.config.get('environment','physical')
    validate_real_plan(plan_value,source,binding,environment=environment)
    validate_plan_evidence(plan_value,source)
    if prepared['timeline_digest']!=plan_value['timeline_digest']:
        raise ValueError('检查点轨迹与已保存计划不一致')
    artifact(prepared['first_frame_path'])
    load_config(artifact(prepared['config_path']))
    installed=json.loads(Path(bridge.config['trusted_binding_path']).read_text())
    if any(installed.get(key)!=binding.get(key) for key in
           ('robot_id','camera_serials','calibration_profile_digest')):
        raise ValueError('真机或标定版本已改变，需要重新规划')
    if saved.get('execution_pending'):
        # An interrupted execution is recoverable only with a sealed zero-command
        # receipt. Unknown/mid-motion outcomes cannot become a fresh action button.
        execution=Path(saved['execution_output']).resolve()
        if not execution.is_relative_to(directory/'jobs'):raise ValueError('执行记录目录无效')
        phase=execution/'execution-phase.json'
        if phase.is_file() and json.loads(phase.read_text())==dict(phase='pre_action',timeline_digest=prepared['timeline_digest']):
            saved['execution_pending']=False
            save(checkpoint,saved)
            prepared['validated']=True
            return prepared
        receipt=execution/'recording-directory.json'
        if not receipt.is_file():return None
        raw=Path(json.loads(receipt.read_text())['directory']).resolve()
        if not raw.is_relative_to(Path(bridge.config['local_recording_root']).resolve()):
            raise ValueError('录制回执不在当前真机录制目录')
        if not (raw/'manifest.json').is_file():return None
        manifest,_,recorded_plan=verify_raw(raw,environment=environment)
        if recorded_plan['timeline_digest']!=prepared['timeline_digest']:
            raise ValueError('恢复回执属于另一条轨迹')
        if type(manifest.get('command_count')) is not int or manifest['command_count']!=0:return None
        saved['execution_pending']=False
        save(checkpoint,saved)
    prepared['validated']=True
    return prepared


def publish_binding(bridge,binding):
    """Same trusted local binding boundary as the main workbench, without moving motors."""
    path=Path(bridge.config['trusted_binding_path'])
    environment=bridge.config.get('environment','physical')
    validate_namespace(binding,environment)
    if environment=='fake' and '/calibration/hardware/active' in str(path.resolve()):
        raise ValueError('fake binding must use its isolated installation path')
    if not path.is_absolute() or path.is_symlink(): raise ValueError('trusted binding path invalid')
    old=json.loads(path.read_text()); owner=bridge.owner()
    validate_namespace(old,environment)
    if not owner_matches_environment(owner,environment) or owner.get('holder','collection')!='collection':
        raise ValueError('current device environment and collection control permission required')
    if any(old.get(k)!=binding.get(k) for k in ('robot_id','camera_serials')):
        raise ValueError('robot/camera binding changed')
    if owner.get('stationary') is not True or owner.get('automatic_enabled') is not False or owner.get('state') not in ('READY','STOPPED_DAMPING_HOLD','MANUAL_TAKEOVER'):
        raise ValueError('stationary healthy control state required; no automatic fault recovery')
    save(path,binding)


def controller_failure(owner,manifest):
    frozen=owner.get('frozen') or {}
    if frozen.get('status')=='completed':return None
    outcome=motion_outcome(owner) or manifest.get('controller_outcome')
    if not ((outcome is not None and outcome!='completed') or frozen.get('status')=='failed'):
        return None
    return (manifest.get('controller_reason') or frozen.get('reason') or owner.get('reason')
            or (owner.get('motion') or {}).get('reason'))


def motion_outcome(owner):
    frozen=(owner.get('frozen') or {}).get('status')
    if frozen in ('completed','failed','cancelled'):
        return {'completed':'completed','failed':'failed','cancelled':'stopped'}[frozen]
    return {'FAULT':'failed','STOPPED':'stopped'}.get((owner.get('motion') or {}).get('status'))


def resident_wait_budget(config,contract):
    """An absolute wall budget; simulated trajectory duration is not a wall ETA."""
    deadline=config['resident_deadline_wall_ns']
    if type(deadline) is not int or deadline!=contract['resources'].get('resident_deadline_wall_ns'):
        raise ValueError('resident deadline differs from the frozen execution contract')
    seconds=min(contract['resources']['max_wall_time_minutes']*60,(deadline-time.time_ns())/1e9-45)
    if seconds<=0:raise TimeoutError('resident lifetime cannot cover execution and 45 s shutdown reserve')
    return seconds


def resident_wait_progress(owner,started,budget):
    frozen=owner.get('frozen') or {};total=(owner.get('execution_timeline') or {}).get('command_count')
    sent=frozen.get('send_count');elapsed=max(0.,time.monotonic()-started)
    qualified=type(sent) is int and type(total) is int and 0<=sent<=total and total>0
    return dict(clock='wall',budget_s=budget,elapsed_s=elapsed,remaining_s=max(0.,budget-elapsed),
                commands_sent=sent if type(sent) is int else None,total_commands=total,
                percent=100*sent/total if qualified else None)


def resident_execution_identity(lane,confirmation,before,deadline,*,cancel=None):
    """Bind the queued confirm to its subsequent actual Owner acceptance."""
    reviewed=confirmation.get('plan_sha256')
    if confirmation.get('queued')!='confirm' or not isinstance(reviewed,str) or not re.fullmatch('[0-9a-f]{64}',reviewed):
        raise ValueError('current queued confirmation receipt required')
    previous=before.get('execution_instance_id') or (before.get('execution_calibration') or {}).get('execution_instance_id')
    deadline=min(deadline,time.monotonic()+15)
    while time.monotonic()<deadline:
        if cancel is not None and cancel():raise InterruptedError('等待本次 Owner 接受执行已中断')
        owner=lane.owner();accepted=owner.get('execution_calibration') or {};instance=owner.get('execution_instance_id')
        if instance and instance!=previous:
            if (not isinstance(instance,str) or not re.fullmatch('[0-9a-f]{32}',instance)
                    or accepted.get('execution_instance_id')!=instance or not accepted.get('plan_sha256')
                    or accepted['plan_sha256'][0]!=reviewed
                    or type(owner.get('command_epoch')) is not int
                    or accepted.get('command_epoch')!=owner['command_epoch']
                    or not isinstance(owner.get('runtime_nonce'),str) or not owner['runtime_nonce']):
                raise ValueError('Owner acceptance differs from this queued review')
            return dict(execution_instance_id=instance,command_epoch=owner['command_epoch'],runtime_nonce=owner['runtime_nonce'])
        if owner.get('connected') is not True:raise ValueError('Owner disconnected before accepting this execution')
        time.sleep(.05)
    raise TimeoutError('Owner did not accept this queued execution within its wall budget')


def resident_task_receipt(lane,source,plan,runtime,deadline,*,execution_identity=None,cancel=None,progress=None):
    """Only a current same-plant native receipt can qualify a local L2 task."""
    expected=(runtime or {}).get('timeline_digest');clock=source.get('clock',{}).get('source_clock_id')
    last={}
    def unverified(reason):
        return dict(task_outcome='unverified',native_task=dict(last,receipt_verified=False,message=reason))
    identity=execution_identity or {};instance=identity.get('execution_instance_id')
    if not expected or not clock or not isinstance(instance,str) or not re.fullmatch('[0-9a-f]{32}',instance):
        return unverified('本次接受执行的实例或同 plant 时钟身份缺失')
    while time.monotonic()<deadline:
        if cancel is not None and cancel():raise InterruptedError('同 plant 任务回执等待已中断')
        owner=lane.owner();last=deepcopy(owner.get('native_task') or {})
        if (any(owner.get(key)!=value for key,value in identity.items())
                or (owner.get('execution_calibration') or {}).get('execution_instance_id')!=instance):
            return unverified('Owner 执行实例已改变；拒绝将其他执行的回执用于本轮')
        same=(last.get('execution_instance_id')==instance and last.get('execution_timeline_digest')==expected
              and last.get('source_clock_id')==clock)
        if same and last.get('task_outcome') in ('success','failed','interrupted','unverifiable'):
            if not last.get('receipt_path') or not last.get('receipt_sha256'):
                return unverified(last.get('message') or '本次原生任务回执未落盘；不使用 L3 回放代替')
            try:
                root=Path(lane.config['native_task_root'])
                path=Path(last['receipt_path'])
                if (not root.is_absolute() or root.is_symlink() or not path.is_absolute() or path.is_symlink()
                        or path.resolve(strict=True)!=root.resolve(strict=True)/(instance+'.json')
                        or path.stat().st_size>8*1024*1024):
                    raise ValueError('native receipt outside the exact current execution task scope')
                data=path.read_bytes();receipt=json.loads(data)
                if hashlib.sha256(data).hexdigest()!=last['receipt_sha256']:
                    raise ValueError('native receipt SHA differs from the published Owner receipt')
                required=dict(schema_version='owner_physx_native_task.v1',environment='fake',simulation_only=True,
                    hardware_qualified=False,source_clock_id=clock,execution_timeline_digest=expected,
                    execution_instance_id=instance,
                    parent_timeline_digest=plan['timeline_digest'],source_digest=_digest(source),
                    task_outcome=last['task_outcome'],task_qualified=last['task_qualified'],runtime_nonce=last['runtime_nonce'],
                    command_epoch=identity['command_epoch'])
                if (any(receipt.get(key)!=value for key,value in required.items())
                        or type(owner.get('command_epoch')) is not int or last['runtime_nonce']!=owner.get('runtime_nonce')
                        or type(receipt.get('task_qualified')) is not bool
                        or receipt['task_qualified']!=(receipt['task_outcome']=='success')
                        or receipt['task_qualified'] and receipt.get('motion_outcome')!='completed'):
                    raise ValueError('native receipt identity or task qualification differs')
                return dict(task_outcome=receipt['task_outcome'],native_task=dict(last,receipt_verified=True,
                    scope='L2_same_plant',receipt_path=str(path.resolve())))
            except (KeyError,TypeError,ValueError,OSError) as error:
                return unverified(str(error))
        if progress:progress('等待同 plant 原生任务回执落盘；L3 回放不参与任务判定',native_task=last,task_outcome='unverified')
        if owner.get('connected') is not True:return unverified('等待任务回执时 Owner 断开')
        time.sleep(.1)
    return unverified('冻结墙钟预算内未取得本次同 plant 原生任务回执')


def execution_runtime(lane,plan,output,previous_path,*,cancel=None):
    """Wait for the new private compilation, then transport that exact shared sequence."""
    from execution_timeline import validate
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        if cancel is not None and cancel():raise InterruptedError('执行准备已中断')
        owner=lane.owner();meta=owner.get('execution_timeline') or {}
        if meta.get('parent_timeline_digest')==plan['timeline_digest'] and meta.get('path')!=previous_path:
            if not meta.get('path'):raise ValueError('执行序列未保存，仿真对照不可用；真机控制独立运行')
            original=Path(meta['path'])
            if not original.is_absolute() or original.is_symlink():raise ValueError('regular execution artifact required')
            path=original.resolve(strict=True)
            root=Path(lane.config['local_recording_root']).resolve().parent
            if not path.is_relative_to(root) or not path.name.startswith('execution-'):
                raise ValueError('execution artifact outside the owned control workspace')
            timeline=validate(json.loads(path.read_text()),plan)
            if timeline['timeline_digest']!=meta['timeline_digest']:raise ValueError('execution artifact identity differs')
            save(Path(output)/'execution-timeline.json',timeline)
            return timeline
        if owner.get('connected') is not True or (owner.get('frozen') or {}).get('status') in ('failed','cancelled'):
            raise ValueError(owner.get('reason') or 'control stopped while preparing execution')
        time.sleep(.05)
    raise TimeoutError('execution compilation result unavailable')


def prepare_remote_runtime(cfg,parent_path,timeline,*,cancel=None):
    """Send only initialization metadata; verify the same unchanged suffix remotely."""
    destination=cfg['remote_run_root']+'/playground-runtime-'+timeline['timeline_digest']+'/execution-timeline.json'
    script="""import json,pathlib,sys
sys.path[:0]=[sys.argv[1],sys.argv[1].rsplit("/",1)[0]]
from execution_timeline import prepare,initialize,validate
plan=json.loads(pathlib.Path(sys.argv[2]).read_text());init=json.loads(sys.argv[3]);p=pathlib.Path(sys.argv[4])
runtime=initialize(prepare(plan),init['actual_start'],init['command_limits'],environment=init['environment'])
assert runtime['timeline_digest']==sys.argv[5],'shared execution derivation differs'
if p.exists():
 assert validate(json.loads(p.read_text()),plan)['timeline_digest']==sys.argv[5]
else:
 p.parent.mkdir(parents=True,exist_ok=False)
 with p.open('x') as f:json.dump(runtime,f,allow_nan=False)
"""
    from cancellable_process import run
    run(transport(cfg,'dsw-scene',python_command(script,cfg['workbench_code'],parent_path,
        json.dumps(timeline['initialization']),destination,timeline['timeline_digest'])),
        check=True,timeout=30,capture_output=True,cancel=cancel)
    return destination



def prepare_remote_resume(cfg,plan,parent_path):
    """Rebuild only the checked initial-opening suffix beside its existing DSW parent."""
    resume=plan['execution_resume']
    destination=cfg['remote_run_root']+'/playground-resume-'+plan['timeline_digest']+'/plan.json'
    script="""import json,pathlib,sys
sys.path[:0]=[sys.argv[1],sys.argv[1].rsplit("/",1)[0]]
from half_speed_plan import resume_initial_open
from integration import _digest
wb,parent,source,binding,destination,gap,cut,expected,environment=sys.argv[1:]
p=pathlib.Path(destination)
if p.is_file():
    result=json.loads(p.read_text())
else:
    load=lambda name:json.loads(pathlib.Path(name).read_text())
    result=resume_initial_open(load(parent),load(source),load(binding),float(gap),minimum_tick=int(cut),environment=environment)
    assert _digest(result)==expected,'remote suffix differs from checked local plan'
    p.parent.mkdir(parents=True,exist_ok=True)
    import tempfile,os
    with tempfile.NamedTemporaryFile(mode='w',dir=p.parent,delete=False) as f:
        json.dump(result,f,separators=(',',':'));temporary=f.name
    os.replace(temporary,p)
assert _digest(result)==expected,'existing remote suffix differs'
print(result['timeline_digest'])
"""
    root=cfg['remote_run_root']
    response=subprocess.run(transport(cfg,'dsw-scene',python_command(script,cfg['workbench_code'],parent_path,
        root+'/detection-bridge/source.json',root+'/detection-bridge/hardware-binding.json',destination,
        resume['measured_metal_gap_mm'],resume['skipped_ticks'],_digest(plan),cfg.get('environment','physical'))),
        check=True,timeout=60,capture_output=True,text=True)
    if response.stdout.strip()!=plan['timeline_digest']:raise ValueError('DSW remaining timeline receipt differs')
    return destination

def execute(config,output,bridge,prepared,progress=None,cancel=None):
    from real_bridge import RealBridge
    from real_contract import validate_real_plan
    from frozen_executor import validate_plan_evidence
    environment=namespace(bridge.config.get('environment','physical'))['environment']
    from real_recording import export_playback,verify_raw
    from real_workflow import resolve_raw_source
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    save(output/'execution-phase.json',dict(phase='pre_action',timeline_digest=prepared['timeline_digest']))
    cfg,contract,_=load_config(Path(prepared['config_path']))
    if cfg.get('environment','physical')!=environment or prepared.get('environment',environment)!=environment:
        raise ValueError('prepared plan environment differs from this workbench')
    resident=environment=='fake' and (cfg.get('execution') or {}).get('transport')=='local'
    original=json.loads(Path(prepared['plan_path']).read_text())
    source=json.loads(Path(prepared['source_path']).read_text()); binding=json.loads(Path(prepared['hardware_binding']).read_text())
    remote_plan=cfg['remote_run_root']+'/playground-plan/plan.json'
    speed=prepared['speed_scale']
    if speed != original.get('execution_derivation',{}).get('speed_scale',1):
        # Reuse the frozen observed scene; never silently change speed on a checked timeline.
        adjusted=cfg['remote_run_root']+'/playground-speed-'+uuid4().hex
        run_stages([geometry_stage(cfg,adjusted,speed=speed,initial=cfg['remote_run_root']+'/playground-plan/initial-plan.json')],output,contract,progress,cancel)
        fetch(adjusted+'/result.json',output/'native-result.json',cancel=cancel,execution_config=cfg)
        native_result=json.loads((output/'native-result.json').read_text())
        if native_result.get('status')!='geometric-plan-ready':
            raise ValueError('速度调整后的原生几何规划失败：'+str(native_result.get('error','missing planning receipt')))
        fetch(adjusted+'/plan.json',output/'plan.json',cancel=cancel,execution_config=cfg); remote_plan=adjusted+'/plan.json'
    else: shutil.copyfile(prepared['plan_path'],output/'plan.json')
    plan_value=json.loads((output/'plan.json').read_text())
    if original.get('execution_resume') and speed != original.get('execution_derivation',{}).get('speed_scale',1):
        from half_speed_plan import resume_initial_open
        plan_value=resume_initial_open(plan_value,source,binding,original['execution_resume']['measured_metal_gap_mm'],environment=environment)
        save(output/'plan.json',plan_value)
    validate_real_plan(plan_value,source,binding,environment=environment); validate_plan_evidence(plan_value,source)
    # The checked parent is already on DSW. Derive its identical suffix there;
    # sending the multi-megabyte JSON over the robot uplink delayed action startup.
    if plan_value.get('execution_resume'):
        if progress: progress('复用 DSW 父轨迹并核验剩余指令',next_label='预览并启动真机与仿真')
        remote_plan=prepare_remote_resume(cfg,plan_value,remote_plan)
    simulation_scope=dict(scope='L3_auxiliary_independent_target',same_plant=False,task_success_evidence=False) if resident else {}
    save(output/'simulation-plan.json',dict(path=remote_plan,timeline_digest=plan_value['timeline_digest'],**simulation_scope))
    for name,value in [('source.json',source),('hardware-binding.json',binding)]: save(output/name,value)
    local_config={**bridge.config,'source_path':str(output/'source.json'),'plan_path':str(output/'plan.json'),
                  'hardware_binding':str(output/'hardware-binding.json'),'artifact_dir':str(output),'render_replay':False}
    local_config.pop('raw_source_dir',None)  # This run must resolve the owner's newly sealed recording.
    if not local_config.get('local_recording_root'):
        raise ValueError('local_recording_root required before execution; no raw-transfer dead end')
    lane=RealBridge(local_config)
    publish_binding(bridge,binding)
    preview=lane.control(dict(action='preview_frozen'))
    if preview.get('timeline_digest')!=plan_value['timeline_digest']: raise ValueError('preview timeline differs')
    before=lane.owner()
    previous=(before.get('recording') or {}).get('directory')
    previous_runtime=(before.get('execution_timeline') or {}).get('path')
    runtime_enabled='compiled_tick_count_before_initialization' in preview
    simulation=native_stage(cfg,'replay',cfg['remote_run_root']+'/playground-replay-'+output.name,plan_path=remote_plan,
                            timeout_s=contract['resources'].get('max_replay_time_seconds',600))
    log=(output/'simulation.log').open('wb')
    process=None; videos=[]; recording=None; manifest=None; errors={}; owner={};controller_outcome=None
    execution_started=None;execution_budget=None;execution_deadline=None;simulation_started=None
    task_outcome='unverified';native_task_evidence=None;execution_identity=None;confirmation_requested=False
    def resident_evidence():
        if not resident:return {}
        return dict(execution_scope='resident_same_plant',native_task=deepcopy(native_task_evidence or owner.get('native_task')),
                    execution_instance_id=(execution_identity or {}).get('execution_instance_id'),
                    simulation_scope=simulation_scope,
                    execution_wait=resident_wait_progress(owner,execution_started,execution_budget)
                    if execution_started is not None else None)
    def stop_resident():
        if resident:
            state=lane.owner()
            if execution_identity and state.get('execution_instance_id')!=execution_identity['execution_instance_id']:
                raise ValueError('Owner now has a different execution instance; it was not stopped')
            if state.get('automatic_enabled') or confirmation_requested and execution_identity is None:
                lane.control(dict(action='stop'))
    try:
        if cancel is not None and cancel():raise InterruptedError('执行准备已中断')
        if resident:
            execution_budget=resident_wait_budget(cfg,contract)
            execution_started=time.monotonic();execution_deadline=execution_started+execution_budget
        # Both lanes start from one click; there is deliberately no per-tick barrier.
        if not runtime_enabled:
            process=subprocess.Popen(simulation['argv'],stdout=log,stderr=subprocess.STDOUT);simulation_started=time.monotonic()
        save(output/'execution-phase.json',dict(phase='confirm_requested',timeline_digest=prepared['timeline_digest']))
        confirmation_requested=True
        confirmation=lane.control(dict(action='confirm',token=preview['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
        if resident:
            execution_identity=resident_execution_identity(lane,confirmation,before,execution_deadline,cancel=cancel)
            save(output/'execution-instance.json',execution_identity)
        runtime=None
        if runtime_enabled:
            try:
                runtime=execution_runtime(lane,plan_value,output,previous_runtime,cancel=cancel)
                remote_runtime=prepare_remote_runtime(cfg,remote_plan,runtime,cancel=cancel)
                simulation=native_stage(cfg,'replay',cfg['remote_run_root']+'/playground-replay-'+output.name,
                    plan_path=remote_plan,execution_timeline_path=remote_runtime,
                    timeout_s=contract['resources'].get('max_replay_time_seconds',600))
                process=subprocess.Popen(simulation['argv'],stdout=log,stderr=subprocess.STDOUT)
                simulation_started=time.monotonic()
            except InterruptedError:raise
            except Exception as error:errors['simulation_runtime']=str(error)
        if progress:
            if resident:progress('fake 同 plant 采集运行中；独立目标回放为 L3 辅助',stage_estimate_s=None,
                next_label='同 plant 录制封存与任务证据核验',estimate_s=None,
                estimate_source='按实际指令进度显示；冻结墙钟预算，不把仿真秒数当作等待时间',
                simulation=simulation_scope,**resident_evidence())
            else:progress('真机采集与仿真渲染独立运行',stage_estimate_s=plan_value['duration_s'],next_label='真机录制封存与视频导出',
                          estimate_s=plan_value['duration_s']+120,estimate_source='控制时间线 + 视频导出粗估 120 秒')
        deadline=execution_deadline if resident else time.monotonic()+(runtime or {}).get('duration_s',plan_value['duration_s'])+60
        while time.monotonic()<deadline:
            if cancel is not None and cancel():
                if not resident and lane.owner().get('automatic_enabled'):lane.control(dict(action='stop'))
                raise InterruptedError('fake 同 plant 采集已中断' if resident else '真机采集已中断')
            owner=lane.owner(); candidate=owner.get('recording') or {}
            if resident and progress:
                progress('fake 同 plant 采集运行中；独立目标回放为 L3 辅助',**resident_evidence())
            if candidate.get('directory') and candidate['directory']!=previous and not (output/'recording-directory.json').exists():
                save(output/'recording-directory.json',{'directory':candidate['directory']})
            terminal=motion_outcome(owner) is not None
            if candidate.get('done') and candidate.get('directory')!=previous and (terminal or not runtime_enabled):
                recording=candidate; break
            if runtime_enabled and terminal and (not candidate or candidate.get('directory')==previous):
                recording=dict(done=True,error=owner.get('recording_error') or '本轮没有正式录制结果');break
            if owner.get('connected') is not True: raise ValueError('control disconnected; no automatic recovery')
            if resident and owner.get('feedback_healthy') is False:
                raise ValueError(owner.get('reason') or 'resident feedback unhealthy; no automatic recovery')
            time.sleep(.25)
        if recording is None:
            raise TimeoutError('resident Owner/recording exceeded the frozen wall budget' if resident else 'physical recording did not seal')
        snapshot_manifest=recording.get('manifest') or {}
        controller_outcome=motion_outcome(owner) if runtime_enabled else snapshot_manifest.get('controller_outcome')
        controller_reason=controller_failure(owner,snapshot_manifest)
        if controller_reason:errors['controller']=controller_reason
        if recording.get('error'):errors['recording']=recording['error']
        if progress: progress('同 plant 录制已封存，核验可用帧' if resident else '真机录制已封存，核验可用帧',
                              next_label='等待 L3 辅助回放视频封存' if resident else '等待独立仿真视频封存',
                              errors=errors,recording=recording,physical_outcome=controller_outcome,task_outcome='unverified')
        raw=None
        try:
            raw=resolve_raw_source(local_config,recording)
            manifest,recorded_source,recorded_plan=verify_raw(raw,**({'environment':environment} if environment=='fake' else {}))
            if not runtime_enabled:controller_outcome=manifest.get('controller_outcome')
            if (_digest(recorded_source)!=_digest(source) or _digest(recorded_plan)!=_digest(plan_value)
                    or _digest(json.loads((raw/'binding.json').read_text()))!=_digest(binding)):
                raise ValueError('raw belongs to another run')
        except Exception as error:
            if not runtime_enabled:raise
            errors['recording']=str(error)
            manifest=dict(controller_outcome=controller_outcome,validation=dict(status='failed'),camera_frames={})
        controller_reason=controller_failure(owner,manifest)
        if controller_reason:errors['controller']=controller_reason
        else:errors.pop('controller',None)
        frame_counts=manifest.get('camera_frames',{})
        if all(type(frame_counts.get(camera)) is int and frame_counts[camera]>0
               for camera in ('base_0','left_wrist_0','right_wrist_0')):
            try:
                from cancellable_process import run
                export_playback(raw,None,output/'physical-video',runner=lambda argv,**kw:run(argv,cancel=cancel,**kw),
                                **({'environment':environment} if environment=='fake' else {}))
            except InterruptedError:
                raise
            except Exception as error:
                errors['physical_export']=str(error)
            for camera in ('base_0','left_wrist_0','right_wrist_0'):
                path=output/'physical-video'/f'source-{camera}.mp4'
                if not path.is_file():continue
                if 'physical_export' in errors:
                    # A failed batch export may already contain playable streams.
                    # Verify each independently rather than advertising partial files.
                    try:
                        probe=json.loads(subprocess.run(['ffprobe','-v','error','-count_frames','-show_streams','-of','json',str(path)],
                            check=True,capture_output=True,text=True,timeout=15).stdout)['streams']
                        if len(probe)!=1 or probe[0].get('codec_name')!='h264' or probe[0].get('pix_fmt')!='yuv420p' or int(probe[0].get('nb_read_frames',0))<=0:continue
                    except Exception:continue
                videos.append(dict(label=namespace(environment)['label']+' '+camera,path=str(path)))
        else:
            errors.setdefault('recording','真机录制没有可导出的完整三路帧；原始帧数 '+json.dumps(frame_counts,ensure_ascii=False))
        if resident:
            task=resident_task_receipt(lane,source,plan_value,runtime,execution_deadline,
                execution_identity=execution_identity,cancel=cancel,progress=progress)
            task_outcome=task['task_outcome'];native_task_evidence=task['native_task']
            save(output/'native-task-verification.json',task)
        if progress: progress('同 plant 视频已核验，等待 L3 辅助回放封存' if resident else '核验真机视频后等待独立仿真视频封存',next_label='独立回放本轮可用视频',
                              errors=errors,recording=recording,physical_outcome=controller_outcome,task_outcome=task_outcome,
                              **resident_evidence())
        deadline=(min((simulation_started or execution_started)+simulation['timeout_s'],execution_deadline)+40
                  if resident else time.monotonic()+simulation['timeout_s']+40)
        if process is None:
            return dict(status='failed',physical_outcome=controller_outcome,task_outcome=task_outcome,
                capture_outcome=manifest.get('validation',{}).get('status'),videos=videos,errors=errors,
                error='; '.join(f'{k}: {v}' for k,v in errors.items()),recording=recording,**resident_evidence())
        while process.poll() is None:
            if cancel is not None and cancel():raise InterruptedError('仿真回放已中断')
            if time.monotonic()>deadline:raise TimeoutError('仿真回放超时')
            time.sleep(.1)
        returncode=process.wait(timeout=0)
        remote=cfg['remote_run_root']+'/playground-replay-'+output.name
        fetch(remote+'/result.json',output/'simulation-result.json',cancel=cancel,execution_config=cfg)
        sim=json.loads((output/'simulation-result.json').read_text())
        if resident:
            sim.update(simulation_scope);save(output/'simulation-result.json',sim)
        if returncode!=0 or sim.get('status')!='completed':
            errors['simulation']=sim.get('error') or f'simulation status={sim.get("status")}, exit={returncode}'
        if sim.get('status')=='completed' and (sim.get('source_run_id')!=source['run_id']
                or sim.get('plan_run_id')!=plan_value['run_id'] or sim.get('timeline_digest')!=plan_value['timeline_digest']):
            raise ValueError('simulation replay belongs to another source or timeline')
        if runtime is not None and sim.get('status')=='completed':
            if sim.get('l3_derivation_ref')!='l3-initialization-derivation.json':
                raise ValueError('L3 replay lacks its explicit command derivation')
            derivation_path=output/'l3-initialization-derivation.json'
            fetch(remote+'/l3-initialization-derivation.json',derivation_path,cancel=cancel,execution_config=cfg)
            if hashlib.sha256(derivation_path.read_bytes()).hexdigest()!=sim.get('l3_derivation_sha256'):
                raise ValueError('L3 derivation artifact digest differs')
            proof=verify_l3_derivation(sim,json.loads(derivation_path.read_text()),plan_value,source,runtime)
            save(output/'l3-derivation-verification.json',proof)
            sim['derivation_verification']=proof;save(output/'simulation-result.json',sim)
        for camera in ('base_0','left_wrist_0','right_wrist_0'):
            path=output/f'sim-{camera}.mp4'
            fetch(remote+'/'+f'{camera}.mp4',path,cancel=cancel,execution_config=cfg)
            videos.append(dict(label=('L3 辅助目标回放 ' if resident else '仿真 ')+camera,path=str(path)))
        physical_ok=controller_outcome=='completed' and manifest.get('validation',{}).get('status')=='passed'
        if not physical_ok:
            errors.setdefault('physical',str(manifest.get('controller_outcome'))+'; '+json.dumps(manifest.get('validation',{}),ensure_ascii=False))
        complete=physical_ok and returncode==0 and sim.get('status')=='completed' and not errors
        return dict(status='completed' if complete else 'stopped' if controller_outcome=='stopped' else 'failed',
                    videos=videos,message=('同 plant 采集与 L3 辅助回放已保存；任务结果仅由同 plant 证据验收' if resident else '两侧视频已保存；任务抓放结果需验收') if complete else '执行未完整结束；已保留本轮可用视频',task_outcome=task_outcome,
                    physical_outcome=controller_outcome,capture_outcome=manifest.get('validation',{}).get('status'),simulation=sim,
                    errors=errors,error='; '.join(f'{key}: {value}' for key,value in errors.items()) if errors else None,
                    recording=recording,**resident_evidence())
    except InterruptedError:
        stop_resident()
        raise
    except Exception as error:
        if resident:
            try:stop_resident()
            except Exception as stop_error:errors['controller_stop']=str(stop_error)
        # Preserve already-exported videos on a simulation failure, never label raw complete.
        if videos or errors:
            errors.setdefault('postprocessing',str(error))
            return dict(status='failed',videos=videos,errors=errors,recording=recording,
                        physical_outcome=controller_outcome or (manifest or {}).get('controller_outcome'),
                        error='; '.join(f'{key}: {value}' for key,value in errors.items()),
                        message='部分视频已保存；执行或仿真未完整结束' if videos else '执行失败；本轮没有可用视频',task_outcome=task_outcome,**resident_evidence())
        raise
    finally:
        # Local SSH cleanup does not send robot release/stop/restart commands.
        if process is not None and process.poll() is None:
            process.terminate()
            # The local owned_process wrapper needs its full 20 s child-group cleanup.
            try: process.wait(timeout=30 if resident else 5)
            except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
        log.close()
