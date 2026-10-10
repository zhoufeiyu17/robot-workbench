"""Current resident C installation -> isolated D configuration, without GPU startup.

prepare writes only a fresh task directory and returns an explicit conversion
command. finalize requires that command's actual converted asset receipt. Worker
objects are constructed stopped; the caller owns GPU admission and lifecycle.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import secrets
from types import SimpleNamespace
from urllib.parse import urlsplit


def read(path):return json.loads(Path(path).read_bytes())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')
    path.chmod(0o600)
    return str(path)


def inside(path,root,*,exists=True):
    path=Path(path)
    if not path.is_absolute() or '..' in path.parts or path.is_symlink() or not path.resolve().is_relative_to(root):
        raise ValueError('absolute current-run path required: '+str(path))
    if exists and not path.exists():raise FileNotFoundError(path)
    return path.resolve()


def loopback(url):
    parsed=urlsplit(url)
    if (parsed.scheme!='http' or parsed.hostname!='127.0.0.1' or not parsed.port
            or parsed.username or parsed.password or parsed.path not in ('','/') or parsed.query or parsed.fragment):
        raise ValueError('explicit current loopback service URL required')
    return url.rstrip('/')


def verify_inputs(setup):
    pins=dict(setup['input_sha256'])
    for group in setup.get('dependency_provenance',{}).values():pins.update(group)
    for path,digest in pins.items():
        if sha(path)!=digest:raise ValueError('frozen collection setup input changed: '+path)


def selected_dependency_pins(contract,pipeline):
    """Retain actual selected immutable dependencies, not an old workbench/run."""
    from run_pipeline import below
    roots=[pipeline[key] for key in ('native_code','detection_runtime_root','curobo_root','deps','assets') if pipeline.get(key)]
    roots += [str(Path(__file__).resolve().parents[1]),str(Path(pipeline['detection_script']).parent)]
    files={pipeline[key] for key in ('initial_suite','finger_envelope','detection_node_config','detection_python',
            'prompt_file','detection_cuda_home') if pipeline.get(key)}
    result={};excluded=[];seen={}
    code_root=contract.get('code',{}).get('root')
    for name in ('input_sha256','code_sha256','prompt_sha256'):
        result[name]={}
        for raw,digest in contract.get('provenance',{}).get(name,{}).items():
            path=Path(raw)
            if not path.is_absolute():
                if not code_root or not Path(code_root).is_absolute():
                    raise ValueError('relative dependency pin requires its explicit source code root')
                path=Path(code_root)/path
            if '..' in path.parts:raise ValueError('dependency pin cannot contain parent traversal')
            key=str(path)
            if key not in files and not any(below(key,root) for root in roots):
                excluded.append(key);continue
            if key in seen and seen[key]!=digest:raise ValueError('contradictory dependency pins: '+key)
            if sha(path)!=digest:raise ValueError('selected read-only dependency changed: '+key)
            seen[key]=digest;result[name][key]=digest
    return result,sorted(set(excluded))


def prepare_collection_setup(*,run_root,calibration_config,capture_root,expected_install,
        registration,profile_template,base_pipeline_config,execution,contract_template,
        detection_runtime_root,control_url,token_file,producer_ports,producer_directories,
        trusted_binding_path,local_recording_root,resident_deadline_wall_ns,output):
    """CPU-only. The template supplies interface fields, never installed calibration."""
    from robot_workbench.calibration_profile import build_fake_profile
    from local_execution import validate as validate_local
    if type(resident_deadline_wall_ns) is not int or resident_deadline_wall_ns<=0:
        raise ValueError('the original resident wall deadline is required')
    root=Path(run_root).resolve();output=inside(output,root,exists=False)
    config_path=inside(calibration_config,root);capture=inside(capture_root,root)
    registration_path=inside(registration,root);token=inside(token_file,root)
    trusted=inside(trusted_binding_path,root,exists=False)
    recording=inside(local_recording_root,root)
    if not recording.is_dir():raise ValueError('actual resident recording root required')
    if config_path.parent!=root:raise ValueError('calibration configuration must identify this resident run')
    if output.exists() or output==root:raise FileExistsError(output)
    c=read(config_path)
    if execution['plant']['scope']!=str(root):raise ValueError('execution plant scope differs from the resident run')
    validate_local({'execution':execution})
    if sha(execution['plant']['script'])!=execution['plant']['script_sha256']:
        raise ValueError('current resident plant code changed')
    if set(producer_ports)!={'head','left','right'} or set(producer_directories)!={'base_0','left_wrist_0','right_wrist_0'}:
        raise ValueError('all three resident producer ports and spool directories required')
    expected_ports=dict(head=c['hardware_camera_port'],**c['hardware_wrist_camera_ports'])
    if producer_ports!=expected_ports or len(set(producer_ports.values()))!=3:
        raise ValueError('D must use the same three C resident camera producers')
    for value in producer_ports.values():
        if type(value) is not int or not 0<value<65536:raise ValueError('valid loopback producer ports required')
    directories={key:str(inside(value,root)) for key,value in producer_directories.items()}
    if any(read(Path(value)/'recording-service.json').get('schema')!='rgbd_lease.v1' for value in directories.values()):
        raise ValueError('resident producer raw lease protocol missing')
    if len(token.read_text().strip())<32:raise ValueError('current collection gateway token missing')
    control_url=loopback(control_url)
    base=deepcopy(base_pipeline_config)
    # No historical observations, accepted plan, worker credentials or run scope.
    for key in ('observation_input_root','detection_input_root','planned_input_root','diagnostic_input_root',
                'replay_profile_stop_after_ticks','detector_service','planner_service','contract_path','execution','remote_run_root'):
        base.pop(key,None)
    if base.get('planning_backend')!='motiongen' or base.get('force_diagnostic'):
        raise ValueError('guarded MotionGen base policy required')
    wb=Path(__file__).resolve().parents[1]/'rule_based_workbench'
    base.update(workbench_code=str(wb),detection_runtime_root=str(Path(detection_runtime_root).resolve()),
                environment='fake',execution=deepcopy(execution),render_replay=False,record_raw=False,
                resident_deadline_wall_ns=resident_deadline_wall_ns)
    dependency_provenance,excluded_pins=selected_dependency_pins(contract_template,base)
    template=Path(profile_template).resolve()
    if not template.is_file():raise FileNotFoundError(template)
    template_value=read(template)
    if (template_value.get('source',{}).get('kind')!='interface_template'
            or template_value['source'].get('calibration_available') is not False):
        raise ValueError('explicit static interface template without historical calibration required')
    pins={str(path):sha(path) for path in (config_path,registration_path,template,Path(execution['plant']['script']))}
    output.mkdir(mode=0o700)
    bridge=build_fake_profile(config_path,capture,template_value,read(registration_path),output/'calibration',
                              expected_install=expected_install)
    bridge_receipt=read(output/'calibration'/'bridge.json')
    pins.update(bridge_receipt['input_sha256'])
    conversion=dict(argv=['/isaac-sim/python.sh','-u','-B',str(wb/'a1_isaac_assets.py'),
        '--bundle',str(output/'calibration'/'model'),'--base-assets',base['assets'],
        '--output',str(output/'native-assets'),'--gpu','1','--native-code',base['native_code']],
        env={'CUDA_VISIBLE_DEVICES':'1','ROBOT_WORKBENCH_SESSION_ID':execution['session_id']},
        gpu_execution_required=True,executed=False,
        purpose='new planning/model assets only; fixed observation plant is never replaced')
    from run_pipeline import isaac_environment
    conversion['env']=isaac_environment({**base,'planning_backend':'native','remote_run_root':str(output/'conversion')},'dsw-scene')
    if execution['max_processes']==5:conversion['argv']+=['--execution-json',json.dumps(execution)]
    pins[str(wb/'a1_isaac_assets.py')]=sha(wb/'a1_isaac_assets.py')
    from a1_asset_derivation import CERTIFICATES
    for name in ('a1_asset_derivation.py',*CERTIFICATES):pins[str(wb/name)]=sha(wb/name)
    setup=dict(schema_version='resident_collection_setup.v1',status='awaiting_native_asset_conversion',
        root=str(output),resident_root=str(root),calibration=bridge,calibration_clock=c['clock'],
        installed=deepcopy(expected_install),input_sha256=pins,pipeline=base,
        dependency_provenance=dependency_provenance,excluded_unconsumed_contract_pin_paths=excluded_pins,
        contract_template=deepcopy(contract_template),conversion=conversion,
        control_url=control_url,token_file=str(token),robot_id=c['robot_id'],
        camera_serials=deepcopy(c['hardware_camera_serials']),producer_ports=deepcopy(producer_ports),
        producer_directories=directories,trusted_binding_path=str(trusted),local_recording_root=str(recording),
        actual_plant_unchanged=True,runtime_allowed=False)
    verify_inputs(setup)
    path=write(output/'setup.json',setup)
    return dict(setup_path=path,status=setup['status'],conversion=conversion,runtime_allowed=False)


def finalize_collection_setup(setup_path):
    """CPU binding after explicit remote conversion; does not launch a worker."""
    from run_pipeline import load_config,PATH_KEYS,below
    from planner_service import startup_config
    setup=read(setup_path);verify_inputs(setup)
    if setup.get('schema_version')!='resident_collection_setup.v1' or setup.get('runtime_allowed') is not False:
        raise ValueError('an unactivated current-run setup receipt is required')
    output=Path(setup['root']);pipeline=deepcopy(setup['pipeline'])
    if not output.resolve().is_relative_to(Path(setup['resident_root']).resolve()):
        raise ValueError('setup output escaped the current resident scope')
    target=output/'native-assets';binding=read(target/'a1-binding.json')
    profile=read(output/'calibration'/'profile.json')
    if (binding.get('schema')!='a1_isaac_assets.v1' or binding.get('conversion_complete') is not True
            or binding.get('physics_validated') is not False or binding.get('runtime_allowed') is not False
            or binding.get('model_binding')!=profile['model_binding']
            or binding.get('bundle_seal')!=sha(output/'calibration'/'model'/'SHA256SUMS.json')
            or Path(binding.get('base_assets','')).resolve()!=Path(pipeline['assets']).resolve()):
        raise ValueError('current calibrated asset conversion receipt missing or mismatched')
    for key in ('urdf','usd'):
        inside(binding[key],target)
    if sha(binding['usd'])!=binding.get('usd_sha256') or sha(binding['urdf'])!=profile['model_binding']['urdf_sha256']:
        raise ValueError('converted planning URDF/USD digest differs')
    pipeline['assets']=str(target)
    jobs=output/'jobs';capture=output/'sealed-observations'
    jobs.mkdir(exist_ok=False);capture.mkdir(exist_ok=False)
    pipeline['remote_run_root']=str(jobs/'template')
    contract=deepcopy(setup['contract_template']);execution=pipeline['execution']
    contract.update(schema_version='anyverse.dsw-codex-experiment.v1',execution=deepcopy(execution))
    contract.setdefault('identity',{})['session_id']=execution['session_id']
    contract['code']=dict(project='robot-workbench',root=str(Path(__file__).resolve().parents[1]),
        source_identity='actual file SHA pins; selected immutable dependencies retained separately',
        repository_expected=False,preserve_existing_changes=True)
    resources=contract['resources']
    resources['resident_deadline_wall_ns']=pipeline['resident_deadline_wall_ns']
    if type(resources.get('service_lifetime_seconds')) is not int or resources['service_lifetime_seconds']<=0:
        raise ValueError('explicit bounded worker service_lifetime_seconds required')
    resources.update(lanes={execution['instance_id']:[execution['gpu_index']]},max_gpu_count_per_instance=1,
        max_workers_per_instance=execution['max_processes'],require_gpu_idle_check=True)
    inherited=deepcopy(setup['dependency_provenance'])
    code=inherited['code_sha256']
    code.update({str(path):sha(path) for path in sorted(Path(pipeline['workbench_code']).glob('*.py'))})
    code.update({str(path):sha(path) for path in sorted(Path(__file__).parent.glob('*.py'))})
    code[execution['plant']['script']]=execution['plant']['script_sha256']
    if execution['max_processes']==5:
        preview=execution['roles']['preview']
        code[preview['script']]=preview['script_sha256']
        inherited['input_sha256'][preview['config_path']]=preview['config_sha256']
    detector=Path(pipeline['detection_script'])
    for path in (detector,detector.with_name('isaac_detection_service.py'),
                 Path(pipeline['detection_runtime_root'])/'scripts'/'run_online_detection_episode.py'):
        code[str(path)]=sha(path)
    runtime_code=sorted((Path(pipeline['detection_runtime_root'])/'ros2'/'online_detection_ros').rglob('*.py'))
    if not runtime_code:raise ValueError('explicit read-only detector ROS package code is missing')
    code.update({str(path):sha(path) for path in runtime_code})
    pins={**inherited['input_sha256'],**setup['input_sha256']}
    pins.update({str(path):sha(path) for path in (Path(setup_path),output/'calibration'/'profile.json',
        output/'calibration'/'acquisition-binding.json',output/'calibration'/'model'/'SHA256SUMS.json',
        target/'a1-binding.json',Path(binding['urdf']),Path(binding['usd']),
        target/'robots/arx_ac_one/v1/d405_rig.json',Path(pipeline['initial_suite']),
        Path(pipeline['finger_envelope']),Path(pipeline['detection_node_config']))})
    for name in ('ac_one.usd','ac_one_no_handle.usd','ac_one_no_handle_d405.usd'):
        path=target/'robots/arx_ac_one/v1/sim'/name
        if path.resolve()!=Path(binding['usd']).resolve():raise ValueError('planning USD alias differs')
        pins[str(path)]=sha(path)
    for name in ('acone.urdf','acone_no_handle.urdf','acone_no_handle_d405.urdf'):
        path=Path(binding['urdf']).with_name(name)
        if path.resolve()!=Path(binding['urdf']).resolve():raise ValueError('planning URDF alias differs')
        pins[str(path)]=sha(path)
    for path in sorted((target/'robots/arx_ac_one/v1/sim').rglob('*.usd')):
        if not path.resolve().is_relative_to(target):raise ValueError('planning USD dependency escapes isolated assets')
        pins[str(path)]=sha(path)
    if binding.get('support_derivation'):
        from a1_asset_derivation import CERTIFICATES,PROOF
        parent=Path(binding['urdf']).parent
        for path in (parent/'a1_support_contact.json',parent/PROOF,*[parent/'derived-certificates'/name for name in CERTIFICATES]):
            pins[str(path)]=sha(path)
    from a1_asset_validation import validate_assets
    asset_validation=validate_assets(target,profile,pins)
    inputs=[pipeline[key] for key in (*PATH_KEYS,'curobo_root','prompt_file','detection_runtime_root') if pipeline.get(key)]
    candidates=set([str(capture),str(output/'calibration'),str(Path(setup_path)),
        *inputs,*[str(Path(path)) for path in pins],*code,*inherited['prompt_sha256'],
        str(Path(__file__).resolve().parents[1]),setup['pipeline']['assets'],str(detector.parent),
        str(Path(execution['plant']['script']).parent),'/isaac-sim','/workspace/isaaclab'])
    read_roots=[]
    for path in sorted(candidates,key=lambda item:(len(Path(item).parts),item)):
        if not any(below(path,parent) for parent in read_roots):read_roots.append(path)
    parent=Path('/tmp')/('rbw-detector-'+secrets.token_hex(12))
    detector_root=parent/'service'
    service=dict(root=str(detector_root),socket=str(detector_root/'d'),owner_token=secrets.token_hex(24),
        identity={key:'sha256:'+sha(path) for key,path in {'adapter_sha256':detector,
            'service_sha256':detector.with_name('isaac_detection_service.py'),
            'node_config_sha256':Path(pipeline['detection_node_config'])}.items()},
        # IPC requests read only the sealed capture/manifest and imported job
        # frames. Code/model/config file pins stay in the experiment contract.
        allowed_read_roots=[str(capture),str(jobs)],allowed_output_root=str(jobs),
        max_memory_mib=12288,min_free_memory_mib=execution['min_free_memory_mib'],
        max_lifetime_seconds=resources['service_lifetime_seconds'])
    pipeline['detector_service']=service;contract['detector_service']=deepcopy(service)
    contract['filesystem']=dict(readonly_roots=read_roots,writable_roots=[str(jobs/'template'),str(jobs),
        str(capture),str(parent),str(detector_root),execution['plant']['scope']],allow_overwrite=False,allow_delete=False)
    contract['provenance']=dict(input_sha256=pins,code_sha256=code,prompt_sha256=inherited['prompt_sha256'],config_sha256={})
    pipeline['contract_path']=str(output/'pipeline-contract.json')
    path=write(output/'pipeline-template.json',pipeline)
    contract['provenance']['config_sha256']={Path(path).name:sha(path)}
    write(output/'pipeline-contract.json',contract)
    load_config(Path(path))
    active=startup_config(path,output)
    cfg,_,_=load_config(active)
    roles={'base_0':'head','left_wrist_0':'left','right_wrist_0':'right'}
    preparation=dict(environment='fake',pipeline_config=str(active),acquisition=dict(
        profile_path=str(output/'calibration'/'profile.json'),
        acquisition_binding_path=str(output/'calibration'/'acquisition-binding.json'),
        measured_initial_state=True,transport=dict(endpoint=None,remote_root=str(capture))))
    config=dict(environment='fake',robot_id=setup['robot_id'],control_url=setup['control_url'],token_file=setup['token_file'],
        trusted_binding_path=setup['trusted_binding_path'],local_recording_root=setup['local_recording_root'],
        native_task_root=str(Path(setup['resident_root'])/'native-tasks'),
        cameras={name:dict(url='http://127.0.0.1:'+str(setup['producer_ports'][role]),serial=setup['camera_serials'][role],
            recording_directory=setup['producer_directories'][name]) for name,role in roles.items()},
        source_path=str(jobs/'not-yet-observed'/'source.json'),plan_path=str(jobs/'not-yet-planned'/'plan.json'),
        hardware_binding=str(output/'calibration'/'acquisition-binding.json'),preparation=preparation,
        playground=dict(enabled=True,pipeline_config=str(active)),hardware_access_enabled=True)
    bridge_path=write(output/'collection-config.json',config)
    result=dict(schema_version='resident_collection_configuration.v1',status='configured_workers_not_started',
        setup_path=str(setup_path),setup_sha256=sha(setup_path),pipeline_config=str(active),
        collection_config=bridge_path,playground_directory=str(output/'playground'),
        initial_trusted_binding_source=str(output/'calibration'/'acquisition-binding.json'),
        trusted_binding_path=setup['trusted_binding_path'],
        worker_state_directory=str(output/'workers'),detector_parent=str(parent),
        installed=setup['installed'],source_clock=setup['calibration_clock'],
        asset_binding_validation=asset_validation,
        actual_plant_unchanged=True,task_or_gpu_qualification=False,
        worker_start_required=['DetectorService.start','PlannerService.start'])
    verify_inputs(setup)
    parent.mkdir(mode=0o700,exist_ok=False)
    write(output/'configured.json',result)
    return result


def construct_collection(configured_path,device,application):
    """Attach existing business classes; all GPU services remain stopped."""
    from detector_service import DetectorService
    from planner_service import PlannerService
    from real_bridge import RealBridge
    from playground import Playground
    from robot_workbench.workflows import CollectionWorkflow
    from run_pipeline import load_config
    configured=read(configured_path);setup=read(configured['setup_path']);verify_inputs(setup)
    if sha(configured['setup_path'])!=configured['setup_sha256'] or device.environment!='fake':
        raise ValueError('current fake setup/device identity required')
    load_config(Path(configured['pipeline_config']))
    installed=read(configured['initial_trusted_binding_source']);trusted=read(configured['trusted_binding_path'])
    if any(installed.get(key)!=trusted.get(key) for key in ('robot_id','camera_serials','calibration_profile_digest')):
        raise ValueError('publish this installation to the resident Owner trusted binding before attachment')
    detector,planner=DetectorService(),PlannerService()
    def initialization():
        state=deepcopy(detector.status());other=planner.status();state['planner']=other
        if other['status'] in ('stopped','failed'):
            state.update(status=other['status'],message='当前规划服务尚未就绪',error=other.get('error'))
        elif other['status']=='starting' and state.get('status')=='ready':state['status']='starting'
        return state
    bridge=RealBridge(read(configured['collection_config']))
    playground=Playground(configured['playground_directory'],bridge,initialization,recover_planner=planner.recover)
    workflow=CollectionWorkflow(playground,device,application)
    return SimpleNamespace(bridge=bridge,playground=playground,workflow=workflow,detector=detector,planner=planner,
        pipeline_config=Path(configured['pipeline_config']),worker_state_directory=Path(configured['worker_state_directory']))
