"""Current resident C installation -> isolated D configuration, without GPU startup.

prepare writes only a fresh task directory and returns an explicit conversion
command. finalize requires that command's actual converted asset receipt. Worker
objects are constructed stopped; the caller owns GPU admission and lifecycle.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import secrets
import shutil
from types import SimpleNamespace
from urllib.parse import urlsplit


def read(path):return json.loads(Path(path).read_bytes())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _process_failure_reason(raw):
    """Expose a bounded diagnostic, never an encoded subprocess command."""
    import re
    text=raw.decode(errors='replace') if isinstance(raw,bytes) else str(raw or '')
    for line in reversed(text[-8192:].splitlines()):
        if re.match(r'^(?:[\w.]+)?(?:Error|Exception):\s',line) and 'CalledProcessError:' not in line:
            return line[:700]
    return '未收到具体错误说明，请查看本次诊断文件'


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


def validated_spool_directories(values,contract,run_root):
    """An external spool is only the exact camera tree admitted by this contract."""
    root=Path(run_root).resolve()
    declared=contract.get('camera_spool_root')
    if declared is not None:
        spool=Path(declared)
        if (not spool.is_absolute() or '..' in spool.parts or spool.is_symlink() or not spool.is_dir()
                or root.is_relative_to(spool.resolve())
                or spool.resolve() not in {Path(p).resolve() for p in contract.get('filesystem',{}).get('writable_roots',[]) if Path(p).is_absolute()}):
            raise ValueError('camera spool requires its exact declared writable root')
        root=spool.resolve()
    directories={key:str(inside(value,root)) for key,value in values.items()}
    if declared is not None and directories!={name:str(root/name) for name in ('base_0','left_wrist_0','right_wrist_0')}:
        raise ValueError('camera directories must be the three declared spool children')
    return directories


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


def connection_config(setup,pipeline_path,calibration_root,capture_root,jobs_root,*,endpoint=None):
    """Device-local addresses/credentials stay on the Owner host in both topologies."""
    calibration_root=Path(calibration_root);jobs_root=Path(jobs_root)
    roles={'base_0':'head','left_wrist_0':'left','right_wrist_0':'right'}
    config=dict(environment=setup['environment'],robot_id=setup['robot_id'],control_url=setup['control_url'],token_file=setup['token_file'],
        trusted_binding_path=setup['trusted_binding_path'],local_recording_root=setup['local_recording_root'],
        cameras={name:dict(url='http://127.0.0.1:'+str(setup['producer_ports'][role]),serial=setup['camera_serials'][role],
            recording_directory=setup['producer_directories'][name]) for name,role in roles.items()},
        source_path=str(jobs_root/'not-yet-observed/source.json'),plan_path=str(jobs_root/'not-yet-planned/plan.json'),
        hardware_binding=str(calibration_root/'acquisition-binding.json'),hardware_access_enabled=True,
        preparation=dict(environment=setup['environment'],pipeline_config=str(pipeline_path),acquisition=dict(
            profile_path=str(calibration_root/'profile.json'),acquisition_binding_path=str(calibration_root/'acquisition-binding.json'),
            measured_initial_state=True,transport=dict(endpoint=endpoint,remote_root=str(capture_root)))),
        playground=dict(enabled=True,pipeline_config=str(pipeline_path)))
    if setup['environment']=='fake':config['native_task_root']=str(Path(setup['resident_root'])/'native-tasks')
    return config


def verify_calibration_capsule(root,expected):
    from artifact_transport import verify,contained
    from integration import _digest,validate_profile
    from real_observation import validate_binding
    from model_adapt import verify as verify_model
    root=Path(root);verify(root,expected)
    descriptor=read(root/'device.json');profile=read(root/'calibration/profile.json')
    binding=read(root/'calibration/acquisition-binding.json');bridge=read(root/'calibration/bridge.json')
    environment=descriptor['environment'];validate_profile(profile);validate_binding(profile,binding,environment=environment)
    if (descriptor.get('schema')!='device_calibration_capsule.v1' or descriptor['robot_id']!=binding['robot_id']
            or descriptor['hardware_camera_serials']!=binding['camera_serials']
            or bridge['runtime_environment']!=environment or bridge['profile_digest']!=_digest(profile)
            or binding['calibration_profile_digest']!=_digest(profile)
            or any(descriptor['expected_install'][name+'_id']!=binding[name+'_id'] for name in ('head','wrist'))
            or descriptor['expected_install']['manifest_sha256']!=binding['active_sha256']['manifest.json']):
        raise ValueError('transported device/calibration identity differs')
    if set(descriptor['input_files'])!=set(bridge['input_sha256']):raise ValueError('calibration capsule omitted original evidence')
    for original,digest in bridge['input_sha256'].items():
        if expected['files'].get(descriptor['input_files'][original],{}).get('sha256')!=digest:raise ValueError('calibration evidence is not in sealed manifest')
        path=contained(root,descriptor['input_files'][original])
        if sha(path)!=digest:raise ValueError('transported calibration evidence changed')
    verify_model(root/'calibration/model')
    if read(root/'calibration/model/profile.json')!=profile:raise ValueError('transported calibrated model differs from profile')
    return descriptor


def import_calibration_capsule(root,expected,output):
    descriptor=verify_calibration_capsule(root,expected);root=Path(root);output=Path(output)
    shutil.copytree(root/'calibration',output)
    bridge=read(output/'bridge.json');write(output/'origin-bridge.json',bridge)
    bridge['input_sha256']={str(root/descriptor['input_files'][path]):digest for path,digest in bridge['input_sha256'].items()}
    bridge['input_sha256'].update({str(root/name):row['sha256'] for name,row in expected['files'].items()})
    bridge['transport_origin']=dict(descriptor_sha256=sha(root/'device.json'),origin_bridge_sha256=sha(output/'origin-bridge.json'))
    (output/'bridge.json').write_text(json.dumps(bridge,allow_nan=False,indent=2)+'\n')
    return dict(output=str(output),profile_path=str(output/'profile.json'),runtime_allowed=False,
        simulation_only=descriptor['environment']=='fake',transported=True)


def prepare_collection_setup(*,run_root,calibration_config,capture_root,expected_install,
        registration,profile_template,base_pipeline_config,execution,contract_template,
        detection_runtime_root,control_url,token_file,producer_ports,producer_directories,
        trusted_binding_path,local_recording_root,resident_deadline_wall_ns,output,diagnostic_source=None,
        prepared_calibration=None):
    """CPU-only. The template supplies interface fields, never installed calibration."""
    from robot_workbench.calibration_profile import build_device_profile
    from robot_workbench.sample_clock import clock_for_source
    from local_execution import validate as validate_local,owner_spec
    from launch_environment import persistent_service
    persistent=persistent_service(contract_template['resources'])
    if (persistent and resident_deadline_wall_ns is not None) or (not persistent and
            (type(resident_deadline_wall_ns) is not int or resident_deadline_wall_ns<=0)):
        raise ValueError('the original resident wall deadline is required')
    root=Path(run_root).resolve();output=inside(output,root,exists=False)
    compute_only=prepared_calibration is not None
    if compute_only:
        capsule=Path(prepared_calibration['root']);c=verify_calibration_capsule(capsule,prepared_calibration['manifest'])
        if c['expected_install']!=expected_install:raise ValueError('transported calibration installation differs')
        config_path=capture=registration_path=token=trusted=recording=None
    else:
        config_path=inside(calibration_config,root);c=read(config_path)
    environment=c.get('environment')
    if environment not in ('fake','physical'):raise ValueError('explicit current device environment required')
    if not compute_only and environment=='fake':capture=inside(capture_root,root)
    elif not compute_only:
        capture=Path(capture_root)
        if not capture.is_absolute() or '..' in capture.parts or capture.is_symlink() or not capture.is_dir() or capture.resolve()!=capture:
            raise ValueError('explicit absolute physical capture directory required')
    if not compute_only:
        registration_path=inside(registration,root);token=inside(token_file,root)
        trusted=inside(trusted_binding_path,root,exists=False)
        recording=inside(local_recording_root,root)
        if not recording.is_dir():raise ValueError('actual resident recording root required')
        if config_path.parent!=root:raise ValueError('calibration configuration must identify this resident run')
    if output.exists() or output==root:raise FileExistsError(output)
    if execution is None:raise ValueError('explicit local compute deployment is required for both device environments')
    validate_local({'execution':execution,'environment':environment})
    owner=owner_spec(execution)
    if owner['scope']!=str(root):raise ValueError('execution owner scope differs from the resident run')
    if sha(owner['script'])!=owner['script_sha256']:
        raise ValueError('current resident compute owner code changed')
    if environment=='physical' and (diagnostic_source is not None or contract_template.get('diagnostic_calibration_source')):
        raise ValueError('fake diagnostic calibration cannot authorize physical preparation')
    if not compute_only and (set(producer_ports)!={'head','left','right'} or set(producer_directories)!={'base_0','left_wrist_0','right_wrist_0'}):
        raise ValueError('all three resident producer ports and spool directories required')
    expected_ports=None if compute_only else dict(head=c['hardware_camera_port'],**c['hardware_wrist_camera_ports'])
    if not compute_only and (producer_ports!=expected_ports or len(set(producer_ports.values()))!=3):
        raise ValueError('D must use the same three C resident camera producers')
    for value in (producer_ports or {}).values():
        if type(value) is not int or not 0<value<65536:raise ValueError('valid loopback producer ports required')
    directories={} if compute_only else validated_spool_directories(producer_directories,contract_template,root)
    if any(read(Path(value)/'recording-service.json').get('schema')!='rgbd_lease.v1' for value in directories.values()):
        raise ValueError('resident producer raw lease protocol missing')
    if not compute_only:
        if len(token.read_text().strip())<32:raise ValueError('current collection gateway token missing')
        control_url=loopback(control_url)
    base=deepcopy(base_pipeline_config)
    # No historical observations, accepted plan, worker credentials or run scope.
    for key in ('observation_input_root','detection_input_root','planned_input_root','diagnostic_input_root',
                'replay_profile_stop_after_ticks','detector_service','planner_service','scene_service','contract_path','execution','remote_run_root'):
        base.pop(key,None)
    if base.get('planning_backend')!='motiongen' or base.get('force_diagnostic'):
        raise ValueError('guarded MotionGen base policy required')
    wb=Path(__file__).resolve().parents[1]/'rule_based_workbench'
    base.update(workbench_code=str(wb),detection_runtime_root=str(Path(detection_runtime_root).resolve()),
                environment=environment,execution=deepcopy(execution),render_replay=False,record_raw=False,
                resident_deadline_wall_ns=resident_deadline_wall_ns)
    contract_template=deepcopy(contract_template)
    diagnostic=contract_template.pop('diagnostic_calibration_source',None)
    admission_source=diagnostic_source if diagnostic_source is not None else diagnostic
    if isinstance(admission_source,dict):
        # Startup fixture equality is already admitted. D receives only the
        # builder's calibration/model inputs, never its truth/base admission pins.
        contract_template.pop('filesystem',None)  # finalize rebuilds D roots from its actual inputs.
        if contract_template.get('camera_spool_root') is not None:
            contract_template['filesystem']=dict(writable_roots=[str(Path(contract_template['camera_spool_root']).resolve())])
        admission={Path(path).resolve() for path in admission_source['input_sha256']}
        for group in contract_template.get('provenance',{}).values():
            if not isinstance(group,dict):continue
            for raw in list(group):
                path=Path(raw)
                if not path.is_absolute():
                    if not contract_template.get('code',{}).get('root'):continue
                    path=Path(contract_template['code']['root'])/path
                if path.resolve() in admission:group.pop(raw)
    dependency_provenance,excluded_pins=selected_dependency_pins(contract_template,base)
    if not compute_only:
        template=Path(profile_template).resolve()
        if not template.is_file():raise FileNotFoundError(template)
        template_value=read(template)
    if not compute_only and (template_value.get('source',{}).get('kind')!='interface_template'
            or template_value['source'].get('calibration_available') is not False):
        raise ValueError('explicit static interface template without historical calibration required')
    pins={str(Path(owner['script'])):sha(owner['script'])}
    if not compute_only:pins.update({str(path):sha(path) for path in (config_path,registration_path,template)})
    else:pins.update(prepared_calibration.get('input_pins',{}))
    output.mkdir(mode=0o700)
    if compute_only:
        bridge=import_calibration_capsule(capsule,prepared_calibration['manifest'],output/'calibration')
    else:
        bridge=build_device_profile(config_path,capture,template_value,read(registration_path),output/'calibration',
            expected_install=expected_install,diagnostic_source=diagnostic_source,environment=environment)
    bridge_receipt=read(output/'calibration'/'bridge.json')
    pins.update(bridge_receipt['input_sha256'])
    conversion=dict(argv=['/isaac-sim/python.sh','-u','-B',str(wb/'a1_isaac_assets.py'),
        '--bundle',str(output/'calibration'/'model'),'--base-assets',base['assets'],
        '--output',str(output/'native-assets'),'--gpu',str(execution['gpu_index']),'--native-code',base['native_code']],
        env={'CUDA_VISIBLE_DEVICES':str(execution['gpu_index']),'ROBOT_WORKBENCH_SESSION_ID':execution['session_id']},
        gpu_execution_required=True,executed=False,
        purpose='new planning/model assets only; the observed device backend is never replaced')
    from run_pipeline import isaac_environment
    conversion['env']=isaac_environment({**base,'planning_backend':'native','remote_run_root':str(output/'conversion')},'dsw-scene')
    if execution['max_processes'] in (5,6):conversion['argv']+=['--execution-json',json.dumps(execution)]
    pins[str(wb/'a1_isaac_assets.py')]=sha(wb/'a1_isaac_assets.py')
    from a1_asset_derivation import CERTIFICATES
    for name in ('a1_asset_derivation.py',*CERTIFICATES):pins[str(wb/name)]=sha(wb/name)
    setup=dict(schema_version='resident_collection_setup.v1',status='awaiting_native_asset_conversion',
        environment=environment,compute_only=compute_only,root=str(output),resident_root=str(root),calibration=bridge,calibration_clock=clock_for_source(c,environment),
        installed=deepcopy(expected_install),input_sha256=pins,pipeline=base,
        dependency_provenance=dependency_provenance,excluded_unconsumed_contract_pin_paths=excluded_pins,
        contract_template=deepcopy(contract_template),conversion=conversion,
        control_url=control_url,token_file=str(token),robot_id=c['robot_id'],
        camera_serials=deepcopy(c['hardware_camera_serials']),producer_ports=deepcopy(producer_ports),
        producer_directories=directories,trusted_binding_path=str(trusted),local_recording_root=str(recording),
        actual_device_configuration_unchanged=True,runtime_allowed=False)
    if environment=='fake':setup['actual_plant_unchanged']=True
    if compute_only:
        for key in ('control_url','token_file','producer_ports','producer_directories','trusted_binding_path','local_recording_root'):setup.pop(key,None)
    verify_inputs(setup)
    path=write(output/'setup.json',setup)
    return dict(setup_path=path,status=setup['status'],conversion=conversion,runtime_allowed=False)


def finalize_collection_setup(setup_path):
    """CPU binding after explicit remote conversion; does not launch a worker."""
    from run_pipeline import load_config,PATH_KEYS,below
    from local_execution import validate as validate_local,owner_spec
    from planner_service import startup_config
    setup=read(setup_path);verify_inputs(setup)
    if setup.get('schema_version')!='resident_collection_setup.v1' or setup.get('runtime_allowed') is not False:
        raise ValueError('an unactivated current-run setup receipt is required')
    output=Path(setup['root']);pipeline=deepcopy(setup['pipeline'])
    environment=pipeline.get('environment')
    if environment not in ('fake','physical') or setup.get('environment',environment)!=environment:
        raise ValueError('setup device environment differs from its pipeline')
    execution=validate_local(pipeline)
    if execution is None:raise ValueError('explicit local compute deployment required')
    owner=owner_spec(execution)
    if owner['scope']!=setup['resident_root']:raise ValueError('compute owner scope changed since preparation')
    if not output.resolve().is_relative_to(Path(setup['resident_root']).resolve()):
        raise ValueError('setup output escaped the current resident scope')
    if not setup.get('compute_only'):validated_spool_directories(setup['producer_directories'],setup['contract_template'],setup['resident_root'])
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
    # Worker lifetime is independent; each D job retains the existing two-hour cap.
    budget=resources.get('max_wall_time_minutes')
    if type(budget) in (int,float) and math.isfinite(budget):resources['max_wall_time_minutes']=min(budget,120)
    resources['resident_deadline_wall_ns']=pipeline['resident_deadline_wall_ns']
    from launch_environment import persistent_service
    if not persistent_service(resources) and (type(resources.get('service_lifetime_seconds')) is not int
            or resources['service_lifetime_seconds']<=0):
        raise ValueError('explicit bounded worker service_lifetime_seconds required')
    resources.update(lanes={execution['instance_id']:[execution['gpu_index']]},max_gpu_count_per_instance=1,
        max_workers_per_instance=execution['max_processes'],require_gpu_idle_check=True)
    inherited=deepcopy(setup['dependency_provenance'])
    code=inherited['code_sha256']
    code.update({str(path):sha(path) for path in sorted(Path(pipeline['workbench_code']).glob('*.py'))})
    code.update({str(path):sha(path) for path in sorted(Path(__file__).parent.glob('*.py'))})
    code[owner['script']]=owner['script_sha256']
    if execution['max_processes'] in (5,6):
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
        str(Path(owner['script']).parent),'/isaac-sim','/workspace/isaaclab'])
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
    if persistent_service(resources):service['persistent_service']=True
    pipeline['detector_service']=service;contract['detector_service']=deepcopy(service)
    contract['filesystem']=dict(readonly_roots=read_roots,writable_roots=[str(jobs/'template'),str(jobs),
        str(capture),str(parent),str(detector_root),owner['scope']],allow_overwrite=False,allow_delete=False)
    if contract.get('camera_spool_root') is not None:
        contract['filesystem']['writable_roots'].append(str(Path(contract['camera_spool_root']).resolve()))
    contract['provenance']=dict(input_sha256=pins,code_sha256=code,prompt_sha256=inherited['prompt_sha256'],config_sha256={})
    pipeline['contract_path']=str(output/'pipeline-contract.json')
    path=write(output/'pipeline-template.json',pipeline)
    contract['provenance']['config_sha256']={Path(path).name:sha(path)}
    write(output/'pipeline-contract.json',contract)
    load_config(Path(path))
    active=startup_config(path,output)
    if execution['max_processes']==6:
        from scene_service import startup_config as scene_startup
        active=scene_startup(active,output)
    cfg,_,_=load_config(active)
    bridge_path=None
    if not setup.get('compute_only'):
        bridge_path=write(output/'collection-config.json',connection_config(setup,active,output/'calibration',capture,jobs))
    result=dict(schema_version='resident_collection_configuration.v1',status='configured_workers_not_started',
        environment=environment,
        setup_path=str(setup_path),setup_sha256=sha(setup_path),pipeline_config=str(active),
        collection_config=bridge_path,playground_directory=str(output/'playground'),
        initial_trusted_binding_source=str(output/'calibration'/'acquisition-binding.json'),
        trusted_binding_path=setup.get('trusted_binding_path'),sealed_observation_root=str(capture),
        worker_state_directory=str(output/'workers'),detector_parent=str(parent),
        installed=setup['installed'],source_clock=setup['calibration_clock'],
        asset_binding_validation=asset_validation,
        actual_device_configuration_unchanged=True,task_or_gpu_qualification=False,
        worker_start_required=['DetectorService.start','PlannerService.start'])
    if environment=='fake':result['actual_plant_unchanged']=True
    verify_inputs(setup)
    parent.mkdir(mode=0o700,exist_ok=False)
    write(output/'configured.json',result)
    return result


def validate_collection_attachment(configured_path):
    """Pure preparation: validate the full contract, return only attachment inputs."""
    from run_pipeline import load_config
    pins={}
    def snapshot(path):
        raw=Path(path).read_bytes();pins[str(path)]=hashlib.sha256(raw).hexdigest()
        return json.loads(raw)
    configured=snapshot(configured_path);setup=snapshot(configured['setup_path']);verify_inputs(setup)
    raw_pipeline=snapshot(configured['pipeline_config'])
    contract=raw_pipeline.get('compute_transport',{}).get('contract_file') or raw_pipeline['contract_path']
    pins[contract]=sha(contract)
    environment=setup['pipeline'].get('environment')
    if (sha(configured['setup_path'])!=configured['setup_sha256'] or environment not in ('fake','physical')
            or configured.get('environment',environment)!=environment):
        raise ValueError('current declared setup/device environment required')
    pipeline,_,_=load_config(Path(configured['pipeline_config']))
    if pipeline.get('environment', 'physical')!=environment:
        raise ValueError('pipeline environment differs from the configured device')
    value=dict(configured=configured,environment=environment,has_scene=bool(pipeline.get('scene_service')),
        installed_binding=snapshot(configured['initial_trusted_binding_source']),bridge_config=snapshot(configured['collection_config']),
        attachment_sha256=pins)
    if any(sha(path)!=digest for path,digest in pins.items()):raise ValueError('collection attachment inputs changed during validation')
    return value


def construct_collection(configured_path,device,application,*,validated=None):
    """Attach shared business classes from verified inputs; GPU services stay stopped."""
    from detector_service import DetectorService
    from planner_service import PlannerService
    from real_bridge import RealBridge
    from playground import Playground
    from robot_workbench.workflows import CollectionWorkflow
    validated=validate_collection_attachment(configured_path) if validated is None else validated
    configured=validated['configured']
    # Rehash the few attachment files without decoding the large contract again.
    # Full source verification and contract parsing happened in the CPU worker.
    if (device.environment!=validated['environment'] or str(configured_path) not in validated['attachment_sha256']
            or any(sha(path)!=digest for path,digest in validated['attachment_sha256'].items())):
        raise ValueError('validated collection attachment inputs changed')
    installed=validated['installed_binding'];trusted=read(configured['trusted_binding_path'])
    if any(installed.get(key)!=trusted.get(key) for key in ('robot_id','camera_serials','calibration_profile_digest')):
        raise ValueError('publish this installation to the resident Owner trusted binding before attachment')
    detector,planner=DetectorService(),PlannerService()
    scene=None
    if validated['has_scene']:
        from scene_service import SceneService
        scene=SceneService()
    def initialization():
        state=deepcopy(detector.status());other=planner.status();state['planner']=other
        if other['status'] in ('stopped','failed'):
            state.update(status=other['status'],message='当前规划服务尚未就绪',error=other.get('error'))
        elif other['status']=='starting' and state.get('status')=='ready':state['status']='starting'
        if scene:
            state['scene']=scene.status()
            if state['scene']['status'] not in ('ready','busy'):
                state.update(status=state['scene']['status'],message='场景服务尚未就绪',error=state['scene'].get('error'))
        return state
    bridge=RealBridge(validated['bridge_config'])
    def recover(*,progress=None):
        planner.recover(progress=progress)
        if scene:scene.recover(progress=progress)
    playground=Playground(configured['playground_directory'],bridge,initialization,recover_planner=recover)
    playground.managed_lifecycle=True
    workflow=CollectionWorkflow(playground,device,application)
    return SimpleNamespace(bridge=bridge,playground=playground,workflow=workflow,detector=detector,planner=planner,scene=scene,
        pipeline_config=Path(configured['pipeline_config']),worker_state_directory=Path(configured['worker_state_directory']),
        configured_path=Path(configured_path),attachment_identity=deepcopy(validated['attachment_sha256']))


def compute_deployment(path,expected_sha256):
    """Administrator-supplied compute-host facts; never a browser action payload."""
    from local_execution import validate,owner_spec
    import re
    path=Path(path)
    if not path.is_absolute() or path.is_symlink() or not re.fullmatch('[0-9a-f]{64}',expected_sha256) or sha(path)!=expected_sha256:
        raise ValueError('pinned compute deployment file differs')
    value=read(path)
    required={'schema','python','workbench_root','run_root','input_root','execution','base_pipeline_config',
        'contract_template','detection_runtime_root','input_sha256'}
    if set(value)!=required or value['schema']!='workbench.compute_deployment.v1':raise ValueError('explicit compute deployment fields required')
    execution=validate({'execution':value['execution']})
    root=Path(value['run_root']).resolve()
    if (str(root)!=owner_spec(execution)['scope'] or not root.is_dir()
            or Path(value['workbench_root']).resolve()!=Path(__file__).resolve().parents[1]
            or not Path(value['python']).is_absolute()):raise ValueError('compute owner/code/runtime scope differs')
    inbox=inside(value['input_root'],root)
    if inbox==root or not inbox.is_dir():raise ValueError('predeployed compute input directory required')
    required_pins={owner_spec(execution)['script'],str(Path(value['workbench_root'])/'robot_workbench/collection_setup.py'),
        str(Path(value['workbench_root'])/'rule_based_workbench/run_pipeline.py')}
    if not required_pins<=set(value['input_sha256']):raise ValueError('compute deployment must pin owner and shared preparation/transport code')
    for name,expected in value['input_sha256'].items():
        if sha(name)!=expected:raise ValueError('compute deployment dependency changed: '+name)
    return value


def compute_operation(deployment_path,deployment_sha256,request,*,checker=None,converter=None):
    """Fixed remote entry: inspect or the existing shared prepare/conversion/finalize."""
    from artifact_transport import contained,manifest
    from local_execution import gpu_check
    import re
    value=compute_deployment(deployment_path,deployment_sha256)
    if (not isinstance(request,dict) or request.get('schema')!='remote_collection_request.v1'
            or request.get('operation') not in ('inspect','prepare') or not re.fullmatch('[0-9a-f]{32}',request.get('request_id',''))):
        raise ValueError('fixed compute operation and request identity required')
    allowed={'schema','operation','request_id'}|({'input_directory','manifest'} if request['operation']=='prepare' else set())
    if set(request)!=allowed:raise ValueError('unknown remote compute request field')
    gpu=(checker or gpu_check)({'execution':value['execution']})
    header=dict(schema='remote_collection_receipt.v1',request_id=request['request_id'],deployment_sha256=deployment_sha256,
        run_root=value['run_root'],input_root=value['input_root'],gpu_preflight=gpu)
    if request['operation']=='inspect':return header
    if request['input_directory']!='calibration-'+request['request_id']:raise ValueError('compute input directory differs from request identity')
    root=contained(value['input_root'],request['input_directory'])
    if read(root/'.transport.json')!=request['manifest'] or (root/'.transport.partial').exists():raise ValueError('completed sealed calibration transfer required')
    descriptor=verify_calibration_capsule(root,request['manifest'])
    output=Path(value['run_root'])/('compute-'+request['request_id'])
    if (output/'compute-receipt.json').exists():
        saved=read(output/'compute-receipt.json')
        if saved['request']!=request or saved['response']['deployment_sha256']!=deployment_sha256:raise ValueError('compute request identity already bound to different inputs')
        from artifact_transport import verify
        verify(output/'client-package',saved['response']['manifest'])
        return saved['response']
    prepared=prepare_collection_setup(run_root=value['run_root'],calibration_config=None,capture_root=None,
        expected_install=descriptor['expected_install'],registration=None,profile_template=None,
        base_pipeline_config=value['base_pipeline_config'],execution=value['execution'],contract_template=value['contract_template'],
        detection_runtime_root=value['detection_runtime_root'],control_url=None,token_file=None,producer_ports=None,producer_directories=None,
        trusted_binding_path=None,local_recording_root=None,resident_deadline_wall_ns=value['contract_template']['resources'].get('resident_deadline_wall_ns'),
        output=output,prepared_calibration=dict(root=str(root),manifest=request['manifest'],
            input_pins={str(deployment_path):deployment_sha256,**value['input_sha256']}))
    setup=read(prepared['setup_path'])
    header['gpu_preflight']=(checker or gpu_check)({'execution':value['execution']})
    if converter is None:
        import subprocess
        from launch_environment import clean_environment
        log_path=output/'conversion.log'
        try:
            with log_path.open('xb') as log:
                subprocess.run(setup['conversion']['argv'],env=clean_environment(setup['conversion']['env']),
                    stdout=log,stderr=subprocess.STDOUT,timeout=600,check=True)
        except subprocess.CalledProcessError as error:
            with log_path.open('rb') as log:
                log.seek(max(0,log_path.stat().st_size-8192));reason=_process_failure_reason(log.read())
            raise ValueError(f'数采资产转换失败：{reason}；诊断：{log_path}') from error
    else:converter(setup)
    configured=finalize_collection_setup(prepared['setup_path'])
    config_path=Path(configured['pipeline_config']);pipeline=read(config_path)
    package=output/'client-package';package.mkdir()
    for name,source in {'config.json':config_path,'contract.json':Path(pipeline['contract_path']),
            'profile.json':output/'calibration/profile.json','binding.json':output/'calibration/acquisition-binding.json',
            'planner-spec.json':config_path.parent/'planner-spec.json',
            **({'scene-spec.json':config_path.parent/'scene-spec.json'} if pipeline.get('scene_service') else {})}.items():
        shutil.copyfile(source,package/name);(package/name).chmod(0o600)
    write(package/'configured.json',configured)
    config_root=output/'client-configs';config_root.mkdir()
    response=dict(header,environment=descriptor['environment'],robot_id=descriptor['robot_id'],
        expected_install=descriptor['expected_install'],calibration_descriptor_sha256=sha(root/'device.json'),
        artifact_root=str(output),artifact_directory='client-package',config_root=str(config_root),manifest=manifest(package,[p.name for p in package.iterdir()]))
    write(output/'compute-receipt.json',dict(request=request,response=response))
    return response


_COMPUTE_BOOTSTRAP = r'''import hashlib,json,os,pathlib,sys
p=pathlib.Path(sys.argv[1]); raw=p.read_bytes()
assert hashlib.sha256(raw).hexdigest()==sys.argv[2],'compute deployment SHA differs'
d=json.loads(raw); root=pathlib.Path(d['workbench_root'])
assert root.is_absolute() and pathlib.Path(d['python']).is_absolute(),'absolute deployed compute code/runtime required'
module=root/'robot_workbench/collection_setup.py'
assert hashlib.sha256(module.read_bytes()).hexdigest()==d['input_sha256'][str(module)],'shared preparation code differs'
code="import json,sys;from pathlib import Path;d=json.loads(Path(sys.argv[1]).read_text());r=d['workbench_root'];sys.path[:0]=[r,r+'/rule_based_workbench',r+'/calibration/infra'];from robot_workbench.collection_setup import compute_operation;raw=sys.stdin.buffer.read(8*1024*1024+1);assert len(raw)<=8*1024*1024,'bounded compute request required';print(json.dumps(compute_operation(sys.argv[1],sys.argv[2],json.loads(raw)),allow_nan=False))"
os.execv(d['python'],[d['python'],'-B','-c',code,sys.argv[1],sys.argv[2]])'''


def remote_compute_call(compute,request,*,runner=None,cancel=None):
    import re
    import shlex
    import subprocess
    import tempfile
    from artifact_transport import ssh_command
    from cancellable_process import run
    if set(compute)!={'endpoint','deployment_path','deployment_sha256'}:raise ValueError('explicit pinned compute deployment and endpoint required')
    if not Path(compute['deployment_path']).is_absolute() or not re.fullmatch('[0-9a-f]{64}',compute['deployment_sha256']):raise ValueError('absolute pinned compute deployment path required')
    raw=json.dumps(request,allow_nan=False).encode()
    if len(raw)>8*1024*1024:raise ValueError('bounded compute request required')
    command=ssh_command(compute['endpoint'],shlex.join(['python3','-B','-c',_COMPUTE_BOOTSTRAP,compute['deployment_path'],compute['deployment_sha256']]))
    with tempfile.TemporaryFile() as stream:
        stream.write(raw);stream.seek(0)
        try:
            reply=(runner or run)(command,stdin=stream,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True,timeout=720,cancel=cancel)
        except subprocess.CalledProcessError as error:
            raise ValueError(f'远端数采准备失败（退出码 {error.returncode}）：{_process_failure_reason(error.stderr)}') from error
    if len(reply.stdout)>8*1024*1024:raise ValueError('bounded compute reply required')
    result=json.loads(reply.stdout)
    if (not isinstance(result,dict) or result.get('schema')!='remote_collection_receipt.v1'
            or result.get('request_id')!=request['request_id'] or result.get('deployment_sha256')!=compute['deployment_sha256']):
        raise ValueError('remote compute response identity differs')
    return result


def prepare_remote_collection(*,compute,run_root,calibration_config,capture_root,expected_install,registration,
        profile_template,control_url,token_file,producer_ports,producer_directories,trusted_binding_path,
        local_recording_root,output,still_current,transfer=None,runner=None,cancel=None,camera_spool_root=None):
    """CPU profile sealing and explicit transport; returns uninstalled shared D configuration."""
    from artifact_transport import manifest,transfer as send,validate_endpoint
    from robot_workbench.calibration_profile import build_device_profile
    from robot_workbench.sample_clock import clock_for_source
    from run_pipeline import load_config
    validate_endpoint(compute['endpoint'])
    if not callable(still_current):raise ValueError('current installation/request fence required')
    root=Path(run_root).resolve();output=inside(output,root,exists=False)
    c=read(inside(calibration_config,root));token=inside(token_file,root)
    if len(token.read_text().strip())<32:raise ValueError('local Owner token missing')
    if producer_ports!=dict(head=c['hardware_camera_port'],**c['hardware_wrist_camera_ports']):raise ValueError('same current C camera ports required')
    spool_contract=({} if camera_spool_root is None else dict(camera_spool_root=str(camera_spool_root),
        filesystem=dict(writable_roots=[str(camera_spool_root)])))
    directories=validated_spool_directories(producer_directories,spool_contract,root)
    if set(directories)!={'base_0','left_wrist_0','right_wrist_0'}:raise ValueError('all local common camera directories required')
    setup=dict(environment=c['environment'],robot_id=c['robot_id'],camera_serials=c['hardware_camera_serials'],resident_root=str(root),
        control_url=loopback(control_url),token_file=str(token),producer_ports=producer_ports,producer_directories=directories,
        trusted_binding_path=str(inside(trusted_binding_path,root,exists=False)),local_recording_root=str(inside(local_recording_root,root)))
    if not still_current():raise InterruptedError('remote preparation request already revoked')
    output.mkdir();capsule=output/'inputs';capsule.mkdir()
    template=read(profile_template)
    if template.get('source',{}).get('kind')!='interface_template' or template['source'].get('calibration_available') is not False:raise ValueError('static uncalibrated interface template required')
    build_device_profile(calibration_config,capture_root,template,read(registration),capsule/'calibration',expected_install=expected_install,environment=c['environment'])
    bridge=read(capsule/'calibration/bridge.json');evidence=capsule/'evidence';evidence.mkdir();paths={}
    forbidden={token.resolve(),Path(compute['endpoint']['key']).resolve()}
    for name,expected in bridge['input_sha256'].items():
        path=Path(name)
        if path.resolve() in forbidden or sha(path)!=expected:raise ValueError('calibration evidence changed or includes a private credential')
        target=evidence/expected
        if not target.exists():shutil.copyfile(path,target)
        paths[name]=str(target.relative_to(capsule))
    write(capsule/'device.json',dict(schema='device_calibration_capsule.v1',environment=c['environment'],robot_id=c['robot_id'],
        hardware_camera_serials=c['hardware_camera_serials'],clock=clock_for_source(c,c['environment']),expected_install=expected_install,input_files=paths))
    sealed=manifest(capsule,[str(p.relative_to(capsule)) for p in capsule.rglob('*') if p.is_file()])
    request=dict(schema='remote_collection_request.v1',operation='inspect',request_id=secrets.token_hex(16))
    inspected=remote_compute_call(compute,request,runner=runner,cancel=cancel)
    write(output/'compute-inspection.json',inspected)
    if not still_current():raise InterruptedError('remote preparation revoked before input upload')
    name='calibration-'+request['request_id']
    (transfer or send)(direction='upload',local_root=str(output),local_directory='inputs',remote_root=inspected['input_root'],
        remote_directory=name,expected=sealed,endpoint=compute['endpoint'],cancel=cancel)
    request.update(operation='prepare',input_directory=name,manifest=sealed)
    response=remote_compute_call(compute,request,runner=runner,cancel=cancel);write(output/'remote-compute-receipt.json',response)
    if (response.get('environment')!=c['environment'] or response.get('robot_id')!=c['robot_id'] or response.get('expected_install')!=expected_install
            or response.get('calibration_descriptor_sha256')!=sha(capsule/'device.json')
            or response.get('artifact_root')!=str(Path(inspected['run_root'])/('compute-'+request['request_id']))
            or response.get('artifact_directory')!='client-package'
            or response.get('config_root')!=str(Path(response['artifact_root'])/'client-configs')):raise ValueError('remote prepared artifacts belong to different device inputs')
    if not still_current() or any(sha(path)!=expected for path,expected in bridge['input_sha256'].items()):
        raise InterruptedError('installation/request changed while remote preparation ran; reply retained but not installed')
    (transfer or send)(direction='download',local_root=str(output),local_directory='compute',remote_root=response['artifact_root'],
        remote_directory=response['artifact_directory'],expected=response['manifest'],endpoint=compute['endpoint'],cancel=cancel)
    package=output/'compute';client=output/'client';client.mkdir()
    if read(package/'binding.json')!=read(capsule/'calibration/acquisition-binding.json') or read(package/'profile.json')!=read(capsule/'calibration/profile.json'):
        raise ValueError('compute result changed the installed calibration')
    pipeline=read(package/'config.json')
    pipeline['compute_transport']=dict(endpoint=deepcopy(compute['endpoint']),contract_file=str(package/'contract.json'),
        contract_sha256=sha(package/'contract.json'),config_root=response['config_root'])
    pipeline_path=write(client/'config.json',pipeline)
    for file in ('planner-spec.json','scene-spec.json'):
        if (package/file).exists():shutil.copyfile(package/file,client/file);(client/file).chmod(0o600)
    load_config(Path(pipeline_path))
    configured=read(package/'configured.json')
    config=connection_config(setup,pipeline_path,capsule/'calibration',configured['sealed_observation_root'],pipeline['remote_run_root'],endpoint=compute['endpoint'])
    connection_path=write(client/'collection-config.json',config)
    local_setup=dict(schema_version='remote_collection_client_setup.v1',pipeline=pipeline,input_sha256={**bridge['input_sha256'],
        **{str(capsule/name):row['sha256'] for name,row in sealed['files'].items()},
        **{str(path):sha(path) for path in [*package.iterdir(),*client.iterdir()] if path.is_file()}})
    local_setup_path=write(client/'setup.json',local_setup)
    result=dict(configured,setup_path=local_setup_path,setup_sha256=sha(local_setup_path),pipeline_config=pipeline_path,
        collection_config=connection_path,initial_trusted_binding_source=str(capsule/'calibration/acquisition-binding.json'),
        trusted_binding_path=setup['trusted_binding_path'],playground_directory=str(output/'playground'),worker_state_directory=str(output/'workers'),
        remote_compute_receipt=str(output/'remote-compute-receipt.json'))
    if not still_current() or any(sha(path)!=expected for path,expected in bridge['input_sha256'].items()):
        raise InterruptedError('remote reply no longer matches current installation; not published')
    write(client/'configured.json',result)
    return result


def prepare_remote_collection_process(*,still_current,cancel=None,**options):
    """Run all profile/transport/config work outside the live control interpreter."""
    configured=_collection_cpu('prepare',options,still_current=still_current,cancel=cancel)
    output=Path(options['output'])
    if (configured.get('setup_path')!=str(output/'client/setup.json')
            or configured.get('trusted_binding_path')!=str(options['trusted_binding_path'])):
        raise ValueError('CPU preparation result differs from its current-run configuration')
    return configured


def prepare_collection_attachment_process(configured_path,expected_install,*,still_current):
    return _collection_cpu('attachment',dict(configured_path=str(configured_path),expected_install=expected_install),
        still_current=still_current)


def load_collection_pipeline_process(path):
    """The unchanged config validator in a CPU child; no persistent private reply."""
    return tuple(_collection_cpu('pipeline',dict(configured_path=str(path),expected_install=None),still_current=lambda:True))


def _collection_cpu(operation,options,*,still_current,cancel=None):
    from contextlib import ExitStack
    import os
    import subprocess
    import sys
    import tempfile
    import time
    from launch_environment import clean_environment
    from owned_process import wait_owned
    def current():return still_current() and not (cancel is not None and cancel())
    if not current():raise InterruptedError('remote preparation request already revoked')
    if operation=='prepare':
        root=Path(options['run_root']).resolve();output=inside(options['output'],root,exists=False)
    elif operation in ('attachment','pipeline'):output=Path(options['configured_path']).parent
    else:raise ValueError('unknown CPU collection preparation operation')
    cpu_python=os.environ.get('WORKBENCH_CPU_PYTHON',sys.executable)
    if not Path(cpu_python).is_absolute():raise ValueError('absolute CPU preparation interpreter required')
    code=Path(__file__).resolve().parents[1]
    environment=clean_environment(dict(PYTHONPATH=os.pathsep.join(map(str,(code,code/'rule_based_workbench',code/'calibration/infra'))),
        CUDA_VISIBLE_DEVICES='',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',
        ROBOT_WORKBENCH_COLLECTION_CPU_WORKER='1'))
    for key in tuple(environment):
        if key.startswith('ROBOT_WORKBENCH_GUARD_'):environment.pop(key)
    request=json.dumps(dict(parent_pid=os.getpid(),operation=operation,options=options),default=str,allow_nan=False).encode()
    if len(request)>64*1024:raise ValueError('bounded collection preparation request required')
    log_path=output.with_name(output.name+'-'+operation+'-'+secrets.token_hex(4)+'-cpu.log')
    maximum=2*1024*1024 if operation=='pipeline' else 256*1024
    process=None
    with tempfile.TemporaryFile() as source,tempfile.TemporaryFile() as reply,ExitStack() as files:
        log=files.enter_context(tempfile.TemporaryFile() if operation=='pipeline' else log_path.open('x+b'))
        if operation!='pipeline':log_path.chmod(0o600)
        source.write(request);source.seek(0)
        try:
            process=subprocess.Popen([cpu_python,'-u','-B','-m','robot_workbench.collection_setup','--prepare-worker'],
                stdin=source,stdout=reply,stderr=log,env=environment,start_new_session=True,close_fds=True)
            deadline=time.monotonic()+1200
            while process.poll() is None:
                if not current():raise InterruptedError('remote preparation cancelled; late reply not installed')
                if time.monotonic()>=deadline:raise TimeoutError('CPU collection preparation exceeded 1200 seconds')
                try:process.wait(timeout=.05)
                except subprocess.TimeoutExpired:pass
            if not current():raise InterruptedError('late remote preparation was not installed')
            reply.seek(0);raw=reply.read(maximum+1)
            if len(raw)>maximum:raise ValueError('bounded collection preparation reply required')
            if process.returncode:
                log.flush()
                log.seek(max(0,os.fstat(log.fileno()).st_size-8192));reason=_process_failure_reason(log.read())
                raise ValueError('CPU collection preparation failed: '+reason+
                    ('; diagnostic: '+str(log_path) if operation!='pipeline' else ''))
            result=json.loads(raw)
            if (result.get('worker_pid')!=process.pid or result.get('parent_pid')!=os.getpid()
                    or result.get('operation')!=operation
                    or result.get('expected_install')!=options['expected_install']):
                raise ValueError('CPU preparation process or installation identity differs')
            path=output/'client/configured.json' if operation=='prepare' else Path(options['configured_path'])
            if operation=='pipeline':
                config,_,hashes=result['value']
                contract=config.get('compute_transport',{}).get('contract_file') or config['contract_path']
                if sha(path)!=hashes['config_sha256'] or sha(contract)!=hashes['contract_sha256']:
                    raise ValueError('pipeline inputs changed during CPU validation')
            elif path.stat().st_size>256*1024 or sha(path)!=result['configured_sha256']:
                raise ValueError('CPU preparation result differs from its current-run configuration')
            if not current():raise InterruptedError('late remote preparation was not installed')
            return result['value']
        finally:
            # Only this newly spawned PGID, including its SSH/transfer children.
            # Remote owned work remains governed by its original compute owner.
            if process is not None:wait_owned(process,timeout=.01,grace=5)


def _prepare_worker():
    import contextlib
    import os
    import signal
    import sys
    import threading
    import time
    def stop(*_):raise InterruptedError('CPU collection preparation cancelled')
    signal.signal(signal.SIGTERM,stop)
    raw=sys.stdin.buffer.read(64*1024+1)
    if len(raw)>64*1024:raise ValueError('bounded collection preparation request required')
    request=json.loads(raw);parent=request['parent_pid'];options=request['options'];operation=request['operation']
    if sys.platform.startswith('linux'):
        import ctypes
        libc=ctypes.CDLL(None,use_errno=True)
        if libc.prctl(1,signal.SIGTERM,0,0,0)!=0:raise OSError(ctypes.get_errno(),'preparation parent lifetime guard failed')
    if os.getppid()!=parent:raise InterruptedError('preparation parent exited before startup')
    def watch_parent():
        while os.getppid()==parent:time.sleep(.05)
        os.killpg(os.getpgrp(),signal.SIGTERM)
    threading.Thread(target=watch_parent,daemon=True).start()
    with contextlib.redirect_stdout(sys.stderr):
        if operation=='prepare':
            value=prepare_remote_collection(**options,still_current=lambda:os.getppid()==parent,
                cancel=lambda:os.getppid()!=parent)
            path=Path(options['output'])/'client/configured.json'
            receipt=Path(options['output'])/'preparation-worker.json'
        elif operation=='attachment':
            path=Path(options['configured_path']);value=validate_collection_attachment(path)
            receipt=path.parent/('attachment-worker-'+str(os.getpid())+'.json')
        elif operation=='pipeline':
            from run_pipeline import load_config
            path=Path(options['configured_path']);value=load_config(path);receipt=None
        else:raise ValueError('unknown CPU collection preparation operation')
    result=dict(worker_pid=os.getpid(),parent_pid=parent,expected_install=options['expected_install'],
        operation=operation,configured_sha256=sha(path),value=value)
    reply=json.dumps(result,allow_nan=False).encode()
    if len(reply)>(2*1024*1024 if operation=='pipeline' else 256*1024):raise ValueError('bounded collection preparation reply required')
    if receipt is not None:write(receipt,{key:value for key,value in result.items() if key!='value'})
    sys.stdout.buffer.write(reply);sys.stdout.buffer.flush()


if __name__=='__main__':
    import sys
    if sys.argv[1:]!=['--prepare-worker']:raise SystemExit('private collection preparation worker required')
    _prepare_worker()
