"""Trusted single-host deployment on the task-owned 734 GPU1; no SSH fallback."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

TARGET=dict(transport='local',instance_id='dsw-3lduujsvlem1a91wdl',gpu_index=1,
            gpu_uuid='GPU-ffc2b1a7-9e85-ec83-d289-f7d1b45ccd64',vulkan_gpu=1)


def validate(config, contract=None):
    value=config.get('execution')
    if value is None:
        return None
    fields={*TARGET,'session_id','plant','max_processes','min_free_memory_mib'}
    if (not isinstance(value,dict) or set(value)!=fields or any(value.get(k)!=v for k,v in TARGET.items())
            or any(type(value[k]) is not int for k in ('gpu_index','vulkan_gpu'))):
        raise ValueError('local execution must freeze exactly 734 GPU1/CUDA mask1/Vulkan1')
    if not isinstance(value['session_id'],str) or not value['session_id']:
        raise ValueError('local execution requires its owning Session')
    if (type(value['max_processes']) is not int or value['max_processes']!=4
            or type(value['min_free_memory_mib']) is not int or value['min_free_memory_mib']<=0):
        raise ValueError('explicit bounded local process/memory budgets required')
    plant=value['plant']
    required={'session_id','pid','process_start_ticks','scope','script','script_sha256','argv','max_memory_mib'}
    if (not isinstance(plant,dict) or set(plant)!=required or plant['session_id']!=value['session_id']
            or type(plant['pid']) is not int or plant['pid']<=0
            or not isinstance(plant['process_start_ticks'],str) or not plant['process_start_ticks'].isdigit()
            or type(plant['max_memory_mib']) is not int or plant['max_memory_mib']<=0
            or not isinstance(plant['argv'],list) or not plant['argv'] or any(not isinstance(v,str) for v in plant['argv'])
            or not isinstance(plant['script_sha256'],str) or not re.fullmatch('[0-9a-f]{64}',plant['script_sha256'])):
        raise ValueError('current Session plant PID/start/scope/code/budget evidence required')
    for key in ('scope','script'):
        path=Path(plant[key])
        if not path.is_absolute() or '..' in path.parts:
            raise ValueError('absolute owned plant paths required')
    if (Path(plant['script']).name!='serve_physx_device.py' or plant['script'] not in plant['argv']
            or not any(plant['argv'][i:i+2]==['--output',plant['scope']] for i in range(len(plant['argv'])-1))):
        raise ValueError('plant command must bind its exact owned output scope')
    if contract is not None:
        if (contract.get('execution')!=value or contract.get('identity',{}).get('session_id')!=value['session_id']
                or contract['resources'].get('max_workers_per_instance')!=value['max_processes']):
            raise ValueError('local execution differs from frozen Session/resource contract')
        from run_pipeline import below,pinned_files
        fs=contract['filesystem'];pins={k:v for group in pinned_files(contract) for k,v in group.items()}
        if (not any(below(plant['scope'],p) for p in fs['writable_roots'])
                or pins.get(plant['script'])!=plant['script_sha256']):
            raise ValueError('plant scope/code must be declared and pinned in the same contract')
    return value


def _process(pid, start, *, proc_root, expected_argv=None, required=(), session=None,expected_env=None):
    proc=Path(proc_root)/str(pid)
    if (proc/'stat').read_text().rsplit(')',1)[1].split()[19]!=str(start):
        raise ValueError('GPU process PID/start time changed')
    argv=[v.decode() for v in (proc/'cmdline').read_bytes().split(b'\0') if v]
    if expected_argv is not None and argv!=expected_argv or any(v not in argv for v in required):
        raise ValueError('GPU process command/scope differs')
    env=dict(v.split('=',1) for v in (proc/'environ').read_text().split('\0') if '=' in v)
    if env.get('CUDA_VISIBLE_DEVICES')!='1':raise ValueError('owned process CUDA mask differs from GPU1')
    if session is not None and env.get('ROBOT_WORKBENCH_SESSION_ID')!=session:
        raise ValueError('GPU worker belongs to another Session')
    if any(env.get(k)!=v for k,v in (expected_env or {}).items()):
        raise ValueError('detector runtime root/pinned identity changed; start a new private scope')


def detector_environment(config,contract=None):
    from run_pipeline import pinned_files,below
    root=config['detection_runtime_root']
    contract=contract or json.loads(Path(config['contract_path']).read_text())
    pins={k:v for group in pinned_files(contract) for k,v in group.items() if below(k,root)}
    if not pins:raise ValueError('detector runtime has no frozen pins')
    identity=hashlib.sha256(json.dumps(pins,sort_keys=True).encode()).hexdigest()
    return dict(ROBOT_WORKBENCH_DETECTOR_RUNTIME_ROOT=root,ROBOT_WORKBENCH_DETECTOR_RUNTIME_IDENTITY=identity)


def gpu_check(config, *, require=None, exclude=None, proc_root='/proc', query=subprocess.check_output):
    """Read-only admission: actual graphics+compute PIDs, never stop/discover owners."""
    execution=validate(config)
    if execution is None:raise ValueError('explicit local resource contract required')
    plant=execution['plant'];known={};required=set()
    if hashlib.sha256(Path(plant['script']).read_bytes()).hexdigest()!=plant['script_sha256']:
        raise ValueError('plant script changed after scope registration')
    _process(plant['pid'],plant['process_start_ticks'],proc_root=proc_root,expected_argv=plant['argv'])
    known[plant['pid']]=plant['max_memory_mib'];required.add(plant['pid'])
    for role in ('detector','planner'):
        if role==exclude:continue
        spec=config.get(role+'_service')
        if spec is None:
            if role==require:raise ValueError(role+' service is not configured')
            continue
        path=Path(spec['root'])/'status.json'
        if not path.is_file():
            if role==require:raise ValueError(role+' service is not ready')
            continue
        state=json.loads(path.read_text())
        pid=state.get('pid');start=state.get('process_start_ticks')
        if type(pid) is not int or pid<=0:raise ValueError('invalid '+role+' PID')
        proc=Path(proc_root)/str(pid)
        if not proc.exists():
            if role==require:raise ValueError(role+' service exited')
            continue
        if role=='planner':
            from planner_service import load_private_spec
            private=load_private_spec(Path(spec['allowed_output_root'])/'planner-spec.json',spec['spec_sha256'])
            if private.get('session_id')!=execution['session_id']:
                raise ValueError('planner private scope belongs to another Session')
            if any(state.get(k)!=v for k,v in {**private,'spec_sha256':spec['spec_sha256']}.items()):raise ValueError('planner private identity differs')
            tokens=[config['workbench_code']+'/curobo_worker.py','--service-spec-file',spec['allowed_output_root']+'/planner-spec.json']
            uses_gpu=state.get('model_initialized') is True
        else:
            if (any(state.get(k)!=spec[k] for k in ('owner_token','identity','socket','allowed_read_roots','allowed_output_root'))
                    or state.get('service_root')!=spec['root']):raise ValueError('detector private identity differs')
            if (type(state.get('physical_gpu')) is not int or state['physical_gpu']!=1
                    or type(state.get('logical_cuda_index')) is not int or state['logical_cuda_index']!=0):
                raise ValueError('local detector must declare physical GPU1 and logical CUDA0')
            tokens=[config['detection_script'],'--serve','--service-root',spec['root'],'--owner-token',spec['owner_token']]
            uses_gpu=state.get('status')=='ready'
        _process(pid,start,proc_root=proc_root,required=tokens,session=execution['session_id'],
                 expected_env=detector_environment(config) if role=='detector' else None)
        if role==require and state.get('status')!='ready':raise ValueError(role+' is not ready')
        if pid in known:raise ValueError('GPU process assigned to two scopes')
        known[pid]=spec['max_memory_mib']
        if uses_gpu:required.add(pid)
    xml=ET.fromstring(query(['nvidia-smi','-i','1','-q','-x'],text=True))
    cards=xml.findall('gpu')
    if len(cards)!=1 or cards[0].findtext('uuid')!=execution['gpu_uuid']:
        raise ValueError('actual GPU1 UUID differs from frozen resource')
    card=cards[0]
    def mib(text):
        match=re.fullmatch(r'([0-9]+) MiB',(text or '').strip())
        if not match:raise ValueError('GPU memory accounting unavailable')
        return int(match[1])
    free=mib(card.findtext('fb_memory_usage/free'));seen=set()
    rows=[]
    for row in card.findall('processes/process_info'):
        pid=int(row.findtext('pid'));memory=mib(row.findtext('used_memory'))
        if pid not in known:raise ValueError('selected GPU has an unowned process')
        if memory>known[pid]:raise ValueError('owned GPU process exceeds memory budget')
        seen.add(pid);rows.append(dict(pid=pid,memory_mib=memory,type=row.findtext('type')))
    if not required<=seen:raise ValueError('owned plant/initialized worker GPU context missing')
    if len(seen)>=execution['max_processes'] and require is None:
        raise ValueError('no declared GPU process slot available')
    minimum=max([execution['min_free_memory_mib'],*[s['min_free_memory_mib'] for s in
        (config.get('detector_service'),config.get('planner_service')) if s]])
    if len(seen)>execution['max_processes'] or free<minimum:
        raise ValueError('local GPU process/free-memory budget exceeded')
    return dict(local_owned=True,gpu=1,gpu_uuid=execution['gpu_uuid'],free_mib=free,processes=rows)


def check_command(config, *, require=None, exclude=None):
    from run_pipeline import python_command
    script=('import json,sys; sys.path[:0]=[sys.argv[1],sys.argv[1].rsplit("/",1)[0]]; '
            'from local_execution import gpu_check; print(json.dumps(gpu_check(json.loads(sys.argv[2]),require=sys.argv[3] or None,exclude=sys.argv[4] or None)))')
    return python_command(script,config['workbench_code'],json.dumps(config),require or '',exclude or '')


def stamp_plan(plan, resource):
    if resource!=TARGET:raise ValueError('unrecognized local compute resource')
    plan.update(instance_id=resource['instance_id'],gpu=resource['gpu_index'],
                compute_resource=dict(resource,logical_cuda_index=0))
    return plan
