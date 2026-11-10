"""Common business assembly plus the physical input/output adapter boundary.

Profiles are trusted local deployment files, never browser-supplied commands.
Reading/preflighting them imports no CAN, RealSense or model runtime.
"""
from copy import deepcopy
import json
import hashlib
import math
import os
from pathlib import Path
import re
import secrets
import subprocess
import sys
import threading
import time
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
ROLES = {'head': 'base_0', 'left': 'left_wrist_0', 'right': 'right_wrist_0'}


def _paths():
    for path in (ROOT, ROOT/'calibration', ROOT/'calibration/infra', ROOT/'rule_based_workbench'):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))


def _inference_options(profile):
    """Validate inert model selection and explicit hardware gripper policy."""
    from .inference_model import BASE566_ID, model_spec
    config=profile['inference']
    if not isinstance(config,dict):raise ValueError('inference deployment configuration must be an object')
    if 'disabled_grippers' not in profile or profile['disabled_grippers'] not in ([],['right']):
        raise ValueError('inference requires explicit disabled_grippers: [] or ["right"]')
    model_id=config.get('model_id',BASE566_ID)
    bundle=config.get('model_bundle')
    if bundle is not None and not isinstance(bundle,dict):
        raise ValueError('inference model_bundle must be a trusted model descriptor object')
    model_spec(model_id,bundle)
    catalog=config.get('catalog_root')
    if type(config.get('recording_enabled',True)) is not bool:raise ValueError('recording_enabled must be boolean')
    if catalog is not None and (not isinstance(catalog,str) or not Path(catalog).is_absolute() or '..' in Path(catalog).parts):
        raise ValueError('absolute inference catalog_root required')
    options=dict(model_id=model_id,model_bundle=deepcopy(bundle),catalog_root=catalog,
                disabled_grippers=list(profile['disabled_grippers']),recording_enabled=config.get('recording_enabled',True),
                control_mapping_pending=config.get('control_mapping_pending'))
    backend=config.get('recording_backend','capture_session_v1')
    if backend not in (None,'capture_session_v1'):raise ValueError('known recording backend required')
    options['recording_backend']=backend
    if profile.get('data_plane') is not None:options['ros_plane']=deepcopy(profile['data_plane'])
    chunks=config.get('action_chunks',False);normal=config.get('normal_modes')
    if type(chunks)is not bool:raise ValueError('action_chunks must be boolean')
    if normal is not None and (not isinstance(normal,dict) or not chunks
            or config.get('recording_enabled',True) or normal.get('recording_required')is not False
            or profile.get('preserve_begin_jaws')is not True):
        raise ValueError('normal modes require explicit chunks, preserved begin jaws and workbench recording disabled')
    if chunks:options['action_chunks']=True
    if 'wam_stall_recovery_available'in config:
        if type(config['wam_stall_recovery_available'])is not bool:raise ValueError('explicit WAM trick availability required')
        options['wam_stall_recovery_available']=config['wam_stall_recovery_available']
    if config.get('reference_planning')is not None:
        if not chunks:raise ValueError('reference planning requires explicit action chunks')
        options['reference_planning']=deepcopy(config['reference_planning'])
    if config.get('finite_chunk_count')is not None:options['finite_chunk_count']=config['finite_chunk_count']
    if normal is not None:options['normal_modes']=deepcopy(normal)
    initial=config.get('normal_initial_mode','MODEL')
    if initial not in ('MODEL','HOLD')or initial=='HOLD'and normal is None:
        raise ValueError('initial HOLD requires explicitly configured normal modes')
    if 'normal_initial_mode'in config:options['normal_initial_mode']=initial
    kind,policy=config.get('normal_finish_kind'),config.get('normal_finish_policy')
    if kind is not None or policy is not None:
        if normal is None or kind!='maintenance_home_v1'or not isinstance(policy,dict):
            raise ValueError('workbench finish requires explicit maintenance_home_v1 policy and normal modes')
        options.update(normal_finish_kind=kind,normal_finish_policy=deepcopy(policy))
    if config.get('recording_context_policy','fixed')!='fixed':
        raise ValueError('per-episode recording context belongs to the collector Raw-gate integration')
    if 'task'in config:options['task']=config['task']
    if 'native_binding'in config:
        from .mixed366_control import load_binding
        load_binding(config['native_binding'],profile)
        if config.get('task')not in ('coffee','lid_handoff'):raise ValueError('native inference requires an explicit task')
    if 'reference_trajectory' in config:options['reference_trajectory']=deepcopy(config['reference_trajectory'])
    if 'prepared_reference_execution'in config:
        if type(config['prepared_reference_execution'])is not bool or config['prepared_reference_execution']and not profile.get('prepared_reference_enabled'):
            raise ValueError('prepared reference execution requires the enabled SDK adapter')
        options['prepared_reference_execution']=config['prepared_reference_execution']
    if 'reference_integral_feedback'in config:
        if type(config['reference_integral_feedback'])is not bool or config['reference_integral_feedback']and not config.get('prepared_reference_execution'):
            raise ValueError('reference integral requires qualified prepared execution')
        options['reference_integral_feedback']=config['reference_integral_feedback']
    return options


def _pedal_input_mode(profile):
    mode=profile.get('pedal_input_mode','legacy_stop')
    if mode not in ('legacy_stop','external_intents'):
        raise ValueError('pedal_input_mode must be legacy_stop or external_intents')
    if mode=='external_intents' and 'pedal_stop_key_code' in profile:
        raise ValueError('external_intents conflicts with pedal_stop_key_code; one pedal reader required')
    if 'pedal_stop_key_code'in profile and (type(profile['pedal_stop_key_code'])is not int
            or not 1<=profile['pedal_stop_key_code']<=0x2ff):
        raise ValueError('pedal_stop_key_code must be an explicit Linux key code in 1..767')
    return mode


def read_profile(path,*,require_ros=False):
    value = json.loads(Path(path).read_text())
    if not isinstance(value,dict):raise ValueError('physical-device profile must be an object')
    if require_ros:
        if value.get('data_plane')is None:raise ValueError('production workbench requires explicit ROS data_plane configuration')
        if (value.get('inference')or{}).get('native_rgb')is not None:
            raise ValueError('ROS workbench cannot retain the legacy native_rgb file source')
        if value.get('recording_backend','capture_session_v1')!='capture_session_v1':
            raise ValueError('ROS workbench requires the resident capture_session_v1 recorder')
    if value.get('data_plane') is not None:
        from .ros_data import plane_config
        plane_config(value['data_plane'])
    if 'triton_preview' in value:
        from .triton_preview import validate_config
        validate_config(value['triton_preview'])
    if (not isinstance(value, dict) or value.get('schema') != 'robot_workbench.physical_device.v1'
            or value.get('environment') != 'physical' or not isinstance(value.get('robot_id'), str)
            or not value['robot_id'].strip() or value['robot_id'].startswith('fake-')):
        raise ValueError('explicit physical-device schema and non-fake robot identity required')
    interfaces=value.get('can_interfaces')
    if (not isinstance(interfaces,dict)or set(interfaces)!={'left','right'}
            or any(not isinstance(name,str)or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,14}',name)
                   for name in interfaces.values())or interfaces['left']==interfaces['right']):
        raise ValueError('explicit distinct left/right CAN interface names required')
    if 'acquisition_blocked_reason' in value and (not isinstance(value['acquisition_blocked_reason'],str)
            or not 1 <= len(value['acquisition_blocked_reason']) <= 1024):
        raise ValueError('nonempty bounded acquisition_blocked_reason required')
    if 'exclusive_idle_units'in value:
        units=value['exclusive_idle_units']
        if (not isinstance(units,list)or not units
                or any(not isinstance(unit,str)or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]*\.service',unit)for unit in units)
                or len(set(units))!=len(units)):
            raise ValueError('explicit unique collection handoff unit names required')
    if value.get('initial_hold_mode','manual_gravity') not in ('manual_gravity','retained_hold'):
        raise ValueError('explicit initial hold mode required')
    _pedal_input_mode(value)
    if value.get('recording_backend','capture_session_v1')not in(None,'capture_session_v1'):
        raise ValueError('known common control recording backend required')
    if value.get('disabled_grippers',[]) not in ([],['left'],['right'],['left','right']):
        raise ValueError('disabled_grippers must list left and/or right explicitly')
    if type(value.get('prepared_reference_enabled',False))is not bool:
        raise ValueError('prepared_reference_enabled must be boolean')
    if type(value.get('raw_state_enabled',False))is not bool:
        raise ValueError('raw_state_enabled must be boolean')
    if type(value.get('preserve_begin_jaws',False))is not bool:
        raise ValueError('preserve_begin_jaws must be boolean')
    from .motion_reference import NATIVE_REFERENCE_APIS
    native_api=value.get('native_reference_api','legacy_recompute')
    if native_api not in NATIVE_REFERENCE_APIS:
        raise ValueError('known native_reference_api required')
    if native_api=='preserve_reference_v1'and(not value.get('prepared_reference_enabled')
            or not value.get('target_interpreter_profile')or not isinstance(value.get('sdk_quiesce'),dict)):
        raise ValueError('preserved reference requires enabled shared windows and a pinned SDK contract')
    if 'dm_joint_velocity_contract'in value:
        from .sdk_quiesce import dm_joint_velocity_wire_scale
        if not isinstance(value.get('sdk_quiesce'),dict):raise ValueError('DM velocity codec requires a pinned SDK')
        dm_joint_velocity_wire_scale(value['dm_joint_velocity_contract'],can_interfaces=value['can_interfaces'],robot_id=value['robot_id'])
    if 'native_gripper_calibration' in value:
        from .sdk_quiesce import load_gripper_calibration
        if not isinstance(value.get('sdk_quiesce'),dict):raise ValueError('gripper endpoints require pinned SDK')
        load_gripper_calibration(value['native_gripper_calibration'],robot_id=value['robot_id'])
        mapping=json.loads(Path(value['jaw_mapping_file']).read_text())
        if mapping.get('native_gripper_calibration')!=value['native_gripper_calibration']:
            raise ValueError('jaw mapping and native gripper calibration must share one binding')
    if 'dm_torque_contract'in value:
        from .sdk_quiesce import dm_torque_units
        if (not isinstance(value.get('sdk_quiesce'),dict)or native_api!='preserve_reference_v1'
                or value.get('ec_torque_contract')is not None or not value.get('dm_joint_velocity_contract')):
            raise ValueError('DM torque migration requires pinned SDK, velocity mapping, preserved references and legacy EC units')
        dm_torque_units(value['dm_torque_contract'],can_interfaces=value['can_interfaces'],robot_id=value['robot_id'])
    if 'ec_torque_contract' in value:
        from .sdk_quiesce import ec_torque_units,EC_J2_GRAVITY_SCALES,EC_J2_EQUIVALENT_UNITS
        if not isinstance(value.get('sdk_quiesce'),dict) or native_api!='preserve_reference_v1':
            raise ValueError('EC torque contract requires pinned SDK and preserved references')
        mode=ec_torque_units(value['ec_torque_contract'])
        if mode in EC_J2_GRAVITY_SCALES and mode!=EC_J2_EQUIVALENT_UNITS and 'sdk_gravity_calibration'not in value:
            raise ValueError('J2 gravity dose requires calibrated gravity binding')
    if 'sdk_gravity_calibration' in value:
        from .gravity_calibration import load_gravity_calibration
        from .sdk_quiesce import ec_torque_units,EC_J2_GRAVITY_SCALES
        if (not isinstance(value.get('sdk_quiesce'),dict)
                or ec_torque_units(value.get('ec_torque_contract')) not in(None,*EC_J2_GRAVITY_SCALES)):
            raise ValueError('gravity calibration requires pinned SDK and reviewed J2 EC units')
        load_gravity_calibration(value['sdk_gravity_calibration'])
    if value.get('control_owner_descriptor') is not None:
        path=Path(value['control_owner_descriptor'])
        if not path.is_absolute()or '..'in path.parts:raise ValueError('absolute private Owner descriptor required')
    if value.get('control_lifecycle','managed') not in ('managed','external'):
        raise ValueError('control_lifecycle must be managed or external')
    if value.get('control_lifecycle')=='external' and (not value.get('control_owner_descriptor')
            or not isinstance(value.get('control_consumer_id'),str) or not value['control_consumer_id']):
        raise ValueError('external control requires a dedicated consumer descriptor and identity')
    if 'target_interpreter_profile' in value:
        from .target_interpreter import PROFILES
        if (value['target_interpreter_profile'] not in PROFILES or not value.get('prepared_reference_enabled')
                or 'right_j4_diagnostic_kp' in value or value.get('left_j6_diagnostic')):
            raise ValueError('shared target interpreter requires one trusted motion profile and native windows')
    if 'motion_gain_profile' in value and (value['motion_gain_profile']!='normal_X5' or not value.get('target_interpreter_profile')):
        raise ValueError('explicit normal_X5 motion gains require the shared target interpreter')
    if 'reference_start_allowance' in value:
        policy=value['reference_start_allowance']
        if (value['robot_id']!='arx13' or value.get('motion_gain_profile')!='normal_X5'
                or not isinstance(policy,dict) or set(policy)-{'reference_id','maximum_error_rad','study_bindings'}
                or not {'reference_id','maximum_error_rad'}<=set(policy)
                or not isinstance(policy['reference_id'],str) or len(policy['reference_id'])!=64
                or any(c not in '0123456789abcdef' for c in policy['reference_id'])
                or type(policy['maximum_error_rad'])not in (int,float)
                or not .01<policy['maximum_error_rad']<=math.radians(2.)):
            raise ValueError('ARX13 default-PD study allowance requires one reference and at most 2 degrees')
        bindings=policy.get('study_bindings',[])
        if (not isinstance(bindings,list)or len(bindings)>12 or len(set(map(str,bindings)))!=len(bindings)
                or any(not isinstance(x,str)or len(x)!=64 or any(c not in '0123456789abcdef'for c in x)for x in bindings)):
            raise ValueError('bounded unique study input hashes required for transient start allowance')
    if type(value.get('left_j6_diagnostic',False))is not bool or value.get('left_j6_diagnostic')and (
            value.get('disabled_grippers')!=['right']or 'right_j4_diagnostic_kp'in value or 'right_j4_diagnostic_kd'in value):
        raise ValueError('left J6 diagnostic requires fixed 40/1 profile and disabled right jaw')
    if value.get('prepared_reference_enabled')and ('right_j4_diagnostic_kp'in value or value.get('left_j6_diagnostic')):
        raise ValueError('prepared reference cannot use a one-shot diagnostic gain profile')
    if 'right_j4_diagnostic_kp' in value and (
            type(value['right_j4_diagnostic_kp']) not in (int,float) or value['right_j4_diagnostic_kp'] not in (60.,80.)
            or value.get('disabled_grippers')!=['right']):
        raise ValueError('single-motion diagnostic requires reviewed right J4 kp60/80 and disabled right jaw')
    if 'right_j4_diagnostic_kd' in value and (
            type(value['right_j4_diagnostic_kd']) not in (int,float)
            or (value.get('right_j4_diagnostic_kp'),value['right_j4_diagnostic_kd']) not in ((60.,1.),(80.,1.),(80.,1.6))):
        raise ValueError('diagnostic kd requires the reviewed kp/kd pair: 60/1, 80/1 or 80/1.6')
    for key in ('legacy_source', 'storage_root', 'jaw_mapping_file'):
        if not isinstance(value.get(key), str) or not Path(value[key]).is_absolute():
            raise ValueError('absolute deployment path required: '+key)
    cameras = value.get('cameras')
    if not isinstance(cameras, dict) or set(cameras) != set(ROLES):
        raise ValueError('head/left/right camera roles required')
    serials = []
    for role, row in cameras.items():
        serial = row.get('serial')
        if (not isinstance(serial, str) or not serial.isascii() or not serial.isdecimal()
                or not isinstance(row.get('intrinsics'), str) or not Path(row['intrinsics']).is_absolute()):
            raise ValueError('decimal RealSense serial and absolute intrinsics path required: '+role)
        serials.append(serial)
    if len(set(serials)) != 3:
        raise ValueError('three distinct camera serials required')
    for key in ('calibration_config', 'collection_config', 'hardware_binding','camera_service_descriptor','physical_cup_return_candidate'):
        if key in value and (not isinstance(value[key], str) or not Path(value[key]).is_absolute()):
            raise ValueError('absolute deployment path required: '+key)
    setup = value.get('collection_setup')
    if setup is not None:
        if (not isinstance(setup,dict) or set(setup)-{'compute','registration','profile_template','asset_catalog'}
                or not {'compute','registration','profile_template'}<=set(setup)
                or not value.get('calibration_config') or value.get('collection_config')):
            raise ValueError('collection_setup requires current calibration and replaces legacy collection_config')
        for key in ('registration','profile_template'):
            if not isinstance(setup[key],str) or not Path(setup[key]).is_absolute():
                raise ValueError('absolute collection preparation input required: '+key)
        compute=setup['compute']
        if (not isinstance(compute,dict) or set(compute)!={'endpoint','deployment_path','deployment_sha256'}
                or not isinstance(compute['deployment_path'],str) or not Path(compute['deployment_path']).is_absolute()
                or not isinstance(compute['deployment_sha256'],str) or not re.fullmatch('[0-9a-f]{64}',compute['deployment_sha256'])):
            raise ValueError('explicit pinned remote compute deployment required')
        _paths()
        from artifact_transport import validate_endpoint
        validate_endpoint(compute['endpoint'])
        if setup.get('asset_catalog') is not None:
            from .scene_asset_catalog import SceneAssetCatalog
            SceneAssetCatalog(setup['asset_catalog'],setup)
    if 'calibration_output' in value and (not isinstance(value['calibration_output'],str)
            or not Path(value['calibration_output']).is_absolute()):
        raise ValueError('absolute durable calibration output required')
    histories=value.get('collection_history_catalogs',[])
    if (not isinstance(histories,list) or len(histories)>32 or any(not isinstance(path,str)
            or not Path(path).is_absolute() or '..' in Path(path).parts for path in histories)):
        raise ValueError('explicit bounded absolute collection history catalogs required')
    inference = value.get('inference')
    if inference is not None:
        options=_inference_options(value)
        inference['model_id']=options['model_id']
        from urllib.parse import urlsplit
        endpoint = urlsplit(inference.get('model_endpoint', ''))
        if (endpoint.scheme != 'http' or endpoint.hostname not in ('127.0.0.1', 'localhost')
                or endpoint.port is None or not 0<endpoint.port<65536
                or endpoint.username or endpoint.password or endpoint.path not in ('', '/')
                or endpoint.query or endpoint.fragment):
            raise ValueError('inference requires an explicitly deployed loopback model endpoint; no automatic SSH route')
        if setup is not None and {'profile_path','binding_path'} & set(inference):
            raise ValueError('current C installation profile cannot be mixed with a static inference profile')
        pending=inference.get('control_mapping_pending')
        if pending is not None and (not isinstance(pending,str) or not pending.strip() or len(pending)>1024):
            raise ValueError('explicit bounded pending model control mapping reason required')
        if pending and {'profile_path','binding_path'}&set(inference):
            raise ValueError('pending mapping cannot also claim an installed control profile')
        if 'native_binding'in inference and (pending or setup is not None or {'profile_path','binding_path'}&set(inference)):
            raise ValueError('native recorder coordinates cannot be mixed with simulation calibration or pending mapping')
        native_wam=str(inference.get('model_id','')).startswith('wam-')
        for key in (('model_token_file',) if setup is not None or pending or 'native_binding'in inference or native_wam else ('model_token_file','profile_path','binding_path')):
            if not isinstance(inference.get(key), str) or not Path(inference[key]).is_absolute():
                raise ValueError('absolute inference artifact path required: '+key)
    return value


def calibration_configuration(path):
    config=json.loads(Path(path).read_text())
    if not isinstance(config,dict):raise ValueError('calibration configuration must be an object')
    seed=config.get('wrist_seed_file')
    if seed is not None:
        if not isinstance(seed,str) or not seed:raise ValueError('calibration.wrist_seed_file must be a nonempty path')
        seed_path=ROOT/'calibration'/seed
        if not seed_path.is_file():
            raise ValueError('calibration.wrist_seed_file is missing from this deployment: '+str(seed_path))
        config['wrist_seed_file']=str(seed_path)
    return config


def preflight(profile, *, net_root='/sys/class/net'):
    """Read files/interface names only; no imports or connection to hardware."""
    checks = []
    def check(name, ok, detail):
        checks.append(dict(name=name, passed=bool(ok), detail=str(detail)))
    if profile.get('acquisition_blocked_reason'):
        check('device commissioning',False,profile['acquisition_blocked_reason'])
    for unit in profile.get('exclusive_idle_units',[]):
        try:
            result=subprocess.run(['systemctl','--user','show',unit,'--property=ActiveState,SubState,MainPID'],
                capture_output=True,text=True,timeout=3)
            state=dict(line.split('=',1)for line in result.stdout.splitlines()if '='in line)
            check('exclusive collection handoff',result.returncode==0 and state==dict(ActiveState='inactive',SubState='dead',MainPID='0'),unit+' '+str(state))
        except (OSError,subprocess.TimeoutExpired)as error:check('exclusive collection handoff',False,str(error))
    if 'sdk_quiesce' in profile:
        from .sdk_quiesce import validate_contract
        try:
            validate_contract(profile['sdk_quiesce'])
            check('SDK quiesce candidate',True,profile['sdk_quiesce']['library_build_id'])
        except (OSError,ValueError) as error:check('SDK quiesce candidate',False,error)
    source = Path(profile['legacy_source'])/'infra/arx_env'
    for name in ('coffee_safety.py', 'pedal_stop.py', 'backends/arx5_backend.py'):
        check('legacy '+name, (source/name).is_file(), source/name)
    check('jaw mapping', Path(profile['jaw_mapping_file']).is_file(), profile['jaw_mapping_file'])
    for role, name in profile['can_interfaces'].items():
        check(role+' CAN interface', (Path(net_root)/name).exists(), name)
    for role, row in profile['cameras'].items():
        try:
            intrinsic = json.loads(Path(row['intrinsics']).read_text())
            valid = (intrinsic.get('serial') == row['serial'] and intrinsic.get('width') == 640
                and intrinsic.get('height') == 480
                and all(type(intrinsic.get(k)) in (int, float) and math.isfinite(intrinsic[k]) for k in ('fx','fy','cx','cy'))
                and intrinsic['fx'] > 0 and intrinsic['fy'] > 0)
            check(role+' camera identity', valid, row['intrinsics'])
        except (OSError, ValueError, TypeError, KeyError) as exc:
            check(role+' camera identity', False, exc)
    for key in ('calibration_config', 'collection_config', 'hardware_binding'):
        if key in profile:
            check(key, Path(profile[key]).is_file(), profile[key])
    if profile.get('calibration_config'):
        try:
            config=calibration_configuration(profile['calibration_config'])
            if config.get('wrist_seed_file'):check('calibration wrist_seed_file',True,config['wrist_seed_file'])
        except (OSError,ValueError,TypeError) as error:check('calibration wrist_seed_file',False,error)
    for key in ('registration','profile_template'):
        path=(profile.get('collection_setup') or {}).get(key)
        if path:check('collection '+key,Path(path).is_file(),path)
    for key, path in (profile.get('inference') or {}).items():
        if key.endswith('_path') or key.endswith('_file'):
            check('inference '+key, Path(path).is_file(), path)
    return dict(schema='physical_device_preflight.v1', robot_id=profile['robot_id'],
        configuration_ready=all(row['passed'] for row in checks), hardware_verified=False,
        hardware_connected=False, cameras_started=False, models_started=False, checks=checks,
        businesses={name:bool(profile.get(key)) for name,key in
                    (('calibration','calibration_config'),('collection','collection_setup' if profile.get('collection_setup') else 'collection_config'),('inference','inference'))},
        inference=(dict(model_id=_inference_options(profile)['model_id'],
                        disabled_grippers=list(profile['disabled_grippers']),
                        catalog_root=profile['inference'].get('catalog_root'),
                        model_endpoint_checked=False,model_load_requires_explicit_action=True,
                        model_prepare_requires_device_connection=False) if profile.get('inference') else None),
        limitations=['CAN identity only; SDK, pedal and actual sensors are checked at explicit connect/start',
                     'remote compute deployment is pinned locally and contacted only by explicit preparation'])


def create_device(device_id, environment, *, runtime_factory=None, ref=None, safety=None,
                  backend_factory=None, lock_root=None, initialize_backend=None, drive=None,
                  label=None, owner_options=None):
    """The same Device lifecycle and Runtime for both sensor/actuator adapters.

    A main-thread simulator may supply runtime_factory; physical adapters supply
    the ordinary backend factory. Neither factory runs until explicit connect.
    """
    _paths()
    from .service import Device
    from .runtime import Runtime
    if environment not in ('fake', 'physical'):
        raise ValueError('explicit device environment required')
    if runtime_factory is not None and backend_factory is not None:
        raise ValueError('choose one acquisition factory')
    if runtime_factory is None:
        if backend_factory is None or ref is None or safety is None or lock_root is None:
            raise ValueError('runtime or explicit backend dependencies required')
        def runtime_factory():
            return Runtime(ref, safety, backend_factory, device_id=device_id, environment=environment,
                lock_root=lock_root, initialize_backend=initialize_backend, **(owner_options or {}))
    return Device(device_id, environment, runtime_factory, label=label, drive=drive)


def install_business_workflows(device, application, *, calibration=None, collection=None,
                               inference=None, camera_service=None):
    """Attach actual C Workbench / D Playground / infer components, never substitutes."""
    from .workflows import CalibrationWorkflow, CollectionWorkflow
    if calibration is not None:
        device.workflows['calibration'] = (calibration if isinstance(calibration, CalibrationWorkflow)
            else CalibrationWorkflow(calibration, device, application))
    if collection is not None:
        device.workflows['collection'] = (collection if isinstance(collection, CollectionWorkflow)
            else CollectionWorkflow(collection, device, application))
    if inference is not None:
        device.workflows['inference'] = inference
    if camera_service is not None:
        device.camera_service=camera_service
        from .bridges import CameraMedia
        for role, name in ROLES.items():
            def frame(name=name):
                value = camera_service.frames[name].snapshot(formats=['jpeg'])
                if not value['status'].get('ready'):
                    raise ValueError(value['status'].get('error') or '相机尚未提供新鲜图像')
                return value['jpeg'], value['status']
            device.media[role] = CameraMedia(role, role, frame, environment=device.environment)
    return device


def _remote_control_client(profile,hardware_binding=None):
    from .control_owner import ControlRuntimeClient
    client=ControlRuntimeClient(profile['control_owner_descriptor'],hardware_binding=hardware_binding,
        recording_backend=profile.get('recording_backend','capture_session_v1'))
    if client.device_id!=profile['robot_id']or client.environment!='physical':
        raise ValueError('control Owner belongs to another physical device')
    if profile.get('control_lifecycle')=='external':
        descriptor=client.channel.descriptor
        if (not descriptor.get('shared_consumers') or not profile.get('control_consumer_id')
                or descriptor.get('consumer_id')!=profile['control_consumer_id']):
            raise ValueError('external control requires the matching dedicated consumer capability; administrator descriptor forbidden')
    return client


def physical_runtime_factory(profile, *, recording_factory=None, event=None, hardware_binding=None,execution_root=None,local_owner=False):
    """Returns an inert callback. Shared Device owns the arms-supported approval."""
    pedal_mode=_pedal_input_mode(profile)
    if local_owner and profile.get('control_lifecycle')=='external':
        raise ValueError('an external consumer profile cannot construct the SDK Owner')
    execution_root=Path(execution_root) if execution_root is not None else Path(profile['storage_root'])/'execution'
    if profile.get('control_owner_descriptor')and not local_owner:
        def remote_acquire():
            client=_remote_control_client(profile,hardware_binding or profile.get('hardware_binding'))
            if profile.get('control_lifecycle')=='external':
                try:
                    client.attach();client.inputs=client.control_inputs();return client
                except BaseException:
                    client.detach();raise
            return client.acquire()
        return remote_acquire
    def acquire():
        _paths()
        from examples.scripts.debug import joint_head_real_owner as ref
        from real_control import controller_conflicts
        from .runtime import Runtime
        conflicts = controller_conflicts()
        if conflicts:
            raise ValueError('existing hardware controller processes require owner handoff: '+str(conflicts))
        report = preflight(profile)
        if not report['configuration_ready']:
            raise ValueError('physical deployment preflight failed: '+', '.join(row['name'] for row in report['checks'] if not row['passed']))
        # Prepare evidence storage before loading the SDK or constructing its Owner.
        # The control loop must never discover a missing/unwritable directory.
        from tempfile import NamedTemporaryFile
        try:
            execution_root.mkdir(mode=0o700, parents=True, exist_ok=True)
            with NamedTemporaryFile(mode='w', dir=execution_root, prefix='.execution-write-check-') as probe:
                probe.write('{}')
                probe.flush()
        except OSError as exc:
            raise ValueError(f'execution evidence directory unavailable: {execution_root}: {exc}') from exc
        safety = ref.legacy_modules(profile['legacy_source'])
        from .sdk_process import SDKProcess
        backend_factory=lambda:SDKProcess(profile)
        runtime = Runtime(ref, safety, backend_factory, device_id=profile['robot_id'], environment='physical',
            lock_root=Path(profile['storage_root'])/'device-locks', jaw_mapping_file=profile['jaw_mapping_file'],
            hardware_binding=hardware_binding or profile.get('hardware_binding'), recording_factory=recording_factory,
            execution_root=execution_root, event=event, data_plane=profile.get('data_plane'),
            **({'preserve_begin_jaws':True} if profile.get('preserve_begin_jaws')is True else {}))
        try:
            if pedal_mode=='legacy_stop':
                from arx_env.pedal_stop import PedalStopListener
                listener = PedalStopListener(key_code=profile.get('pedal_stop_key_code',5))
                runtime.owner.listener = listener
                listener.start()
            elif runtime.owner.listener is not None:
                raise ValueError('external_intents requires no legacy Owner pedal listener')
            runtime.owner.backend.connect()
            runtime.owner.backend.set_damping_hold(manual=profile.get('initial_hold_mode','manual_gravity')=='manual_gravity')
        except Exception as exc:
            # Keep the sole Owner/lock even when acquisition partially fails.
            # Explicit supported release is needed before another acquisition.
            runtime.initialization_error = str(exc)
            runtime.owner.control.state = runtime.owner.states.FAULT
            runtime.owner.control.reason = 'initialization failed: '+str(exc)
        return runtime
    return acquire


def _private_token(path):
    token = secrets.token_urlsafe(32)
    with os.fdopen(os.open(path, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600), 'w') as stream:
        stream.write(token)
    return token


def _collection(profile, device, application, gateway, tokens, ports, output):
    from real_bridge import RealBridge
    from playground import Playground
    from detector_service import DetectorService
    from planner_service import PlannerService
    from run_pipeline import load_config
    config = json.loads(Path(profile['collection_config']).read_text())
    if config.get('environment', 'physical') != 'physical' or config.get('robot_id') != device.id:
        raise ValueError('collection configuration differs from physical robot')
    config.update(control_url=f'http://127.0.0.1:{gateway.server_port}', token_file=str(tokens['collection']))
    config['cameras'] = {name:dict(config.get('cameras', {}).get(name, {}),
        serial=profile['cameras'][role]['serial'], url=f'http://127.0.0.1:{ports[role]}') for role,name in ROLES.items()}
    pipeline_path = Path(config['preparation']['pipeline_config'])
    pipeline, _, _ = load_config(pipeline_path)
    detector, planner = DetectorService(), PlannerService()
    scene = None
    if pipeline.get('scene_service'):
        from scene_service import SceneService
        scene = SceneService()
    def status():
        result = deepcopy(detector.status()); result['planner'] = planner.status()
        if scene: result['scene'] = scene.status()
        for name in ('planner','scene'):
            part = result.get(name)
            if part and part.get('status') not in ('ready','busy'):
                result.update(status=part['status'],message='请显式初始化与预热 '+name,error=part.get('error'))
        return result
    def prepare(*,still_current,launch):
        if not pipeline.get('execution'):
            raise ValueError('旧固定配置缺少明确的受控计算归属，不支持自动服务准备；请核对部署')
        steps=[]
        for name,manager in (('detector',detector),('planner',planner),('scene',scene)):
            if manager is None:continue
            started=False
            def scoped_launch(create):
                def owned_create():
                    nonlocal started
                    started=True;return create()
                return launch(owned_create)
            try:
                if not still_current():raise InterruptedError('旧配置运维准备已撤销')
                reused=manager.status().get('status')=='ready'
                if not reused:
                    if hasattr(manager,'config'):manager.recover(timeout=240,active=still_current,launch=scoped_launch)
                    else:
                        started=True;manager.start(pipeline_path,output/'workers',launch=scoped_launch)
                        deadline=time.monotonic()+240
                        while manager.status().get('status')!='ready':
                            state=manager.status()
                            if not still_current():raise InterruptedError('旧配置服务启动已撤销')
                            if state.get('status') in ('failed','unknown','stopped') or time.monotonic()>deadline:
                                raise ValueError(state.get('error') or '旧配置服务未就绪')
                            time.sleep(.05)
                if not still_current():raise InterruptedError('旧配置服务核验已撤销')
                proof=manager.probe()
                steps.append(dict(role=name,outcome='reused' if reused else 'recovered',evidence=proof,needs_warmup=proof.get('needs_warmup',False)))
            except Exception:
                if started:manager.close()
                raise
        return dict(compatibility_path='legacy_fixed_profile',single_role_restart_supported=False,service_steps=steps)
    playground = Playground(output/'playground', RealBridge(config), status)
    playground.operations_prepare_services=prepare
    install_business_workflows(device, application, collection=playground)
    return SimpleNamespace(playground=playground,detector=detector,planner=planner,scene=scene)


from .resident_collection import CollectionPreparation


class RemoteCollectionPreparation(CollectionPreparation):
    """Same preparation lifecycle; only the compute-location adapter differs."""
    def __init__(self,*,details,calibration,calibration_config,**kwargs):
        self.details=deepcopy(details);self.calibration=calibration
        self.calibration_config=Path(calibration_config)
        from .scene_asset_catalog import SceneAssetCatalog
        self.asset_catalog=(SceneAssetCatalog(details['asset_catalog'],details)if details.get('asset_catalog')else None)
        super().__init__(**kwargs)

    def snapshot(self):
        value=super().snapshot()
        if self.asset_catalog is not None:
            value['scene_assets']=self.asset_catalog.public(self.device.environment,
                requested=value.get('requested_asset_pair_id'),applied=value.get('applied_asset_pair_id'))
        return value

    def validate_asset_request(self,asset_pair_id=None,expected_catalog_digest=None):
        if self.asset_catalog is None:return super().validate_asset_request(asset_pair_id,expected_catalog_digest)
        if asset_pair_id is None and expected_catalog_digest is None:
            with self.lock:asset_pair_id=self.value.get('requested_asset_pair_id')
            if asset_pair_id is None:raise ValueError('请先选择与现场实物一致的咖啡机和杯子组合')
            expected_catalog_digest=self.asset_catalog.digest
        return asset_pair_id,self.asset_catalog.resolve(asset_pair_id,expected_catalog_digest)

    def _configure_asset_request(self,asset_pair_id,expected_catalog_digest):
        selection=self.validate_asset_request(asset_pair_id,expected_catalog_digest)
        if selection is None:return
        with self.lock:
            if self.busy or self.running:raise ValueError('数采资产正在准备，不能并发更换组合')
        self._require_idle()
        pair,details=selection
        with self.lock:
            if self.busy or self.running:raise ValueError('数采资产正在准备，不能并发更换组合')
            changed=pair!=self.value.get('requested_asset_pair_id')
            self.details=details
            if changed:
                self.request_generation+=1;self.value.update(requested_asset_pair_id=pair)
        if changed:self.application.invalidate_collection_preparation(self.device)

    def _reuse_parts(self,expected,current):
        with self.lock:changed=self.value.get('applied_asset_pair_id')!=self.value.get('requested_asset_pair_id')
        return False if changed else super()._reuse_parts(expected,current)

    def _status(self,status,**fields):
        if status=='ready' and self.asset_catalog is not None:
            fields['applied_asset_pair_id']=self.value.get('requested_asset_pair_id')
        return super()._status(status,**fields)

    def installation(self,snapshot):
        if not snapshot or snapshot.get('verified') is not True:
            raise ValueError('current verified physical calibration installation required')
        manifest=Path(self.calibration.config['calibration_root'])/'hardware/active/manifest.json'
        return dict(head_id=snapshot['head_id'],wrist_id=snapshot['wrist_id'],
            manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())

    def _prepare(self,snapshot):
        from .collection_setup import prepare_remote_collection_process
        from .control_owner import collection_recording_root
        runtime=self.device.runtime
        owner_reference=(dict(descriptor=str(runtime.descriptor),runtime_nonce=runtime.nonce)
            if getattr(runtime,'remote_control_owner',False)else None)
        recording_root=(collection_recording_root(Path(runtime.descriptor).parent)
            if owner_reference else self.output/'collection-raw')
        expected=self.installation(snapshot)
        current=self.request_fence(snapshot,expected)
        if not current():raise InterruptedError('remote preparation request or installation changed')
        if self._reuse_parts(expected,current):return
        if self.parts:
            self._require_idle();self.application.invalidate_collection_preparation(self.device);self._close_parts()
        destination=self.output/('collection-'+expected['manifest_sha256'][:16]+'-'+secrets.token_hex(8))
        self._status('preparing_profile',installed=deepcopy(snapshot),installation_identity=expected,output=str(destination),error=None)
        configured=prepare_remote_collection_process(compute=self.details['compute'],run_root=self.output,
            calibration_config=self.calibration_config,capture_root=self.calibration.output,expected_install=expected,
            registration=self.details['registration'],profile_template=self.details['profile_template'],
            control_url=self.control_url,token_file=self.token_file,producer_ports=self.ports,producer_directories=self.directories,
            trusted_binding_path=getattr(self,'binding_path',self.output/'collection-binding.json'),local_recording_root=recording_root,
            output=destination,still_current=current,cancel=lambda:not current(),
            camera_spool_root=None if getattr(runtime,'data_plane',None)else Path(self.directories['base_0']).parent,
            control_owner_reference=owner_reference,data_plane=getattr(runtime,'data_plane',None))
        if not current():raise InterruptedError('late remote preparation was not installed')
        configured_path=Path(configured['setup_path']).parent/'configured.json'
        self._activate_configured(configured,configured_path,expected,time.time_ns()+1200_000_000_000,still_current=current)


def _attach_remote_collection(profile,deployment,tokens):
    calibration=deployment.calibration
    if calibration is None:raise ValueError('remote collection preparation requires the shared calibration Workbench')
    (deployment.output/'collection-raw').mkdir(exist_ok=True)
    preparation=RemoteCollectionPreparation(details=profile['collection_setup'],calibration=calibration,
        calibration_config=deployment.output/'calibration-config.json',
        contract={'resources':dict(persistent_service=True,service_lifetime_seconds=None,resident_deadline_wall_ns=None)},
        output=deployment.output,base={},device=deployment.device,application=deployment.application,
        ports=deployment.camera_ports,directories=deployment.cameras.directories,
        control_url=f'http://127.0.0.1:{deployment.gateway.server_port}',token_file=tokens['collection'],
        deadline_wall_ns=None,installed_snapshot=calibration.installed_pair)
    deployment.device.collection_preparation=preparation
    runtime=deployment.device.runtime
    authoritative_binding=(runtime.owner.hardware_binding if getattr(runtime,'remote_control_owner',False)else None)
    preparation.binding_path=Path(authoritative_binding or profile.get('hardware_binding')or deployment.output/'collection-binding.json')
    deployment.collection=preparation
    flow=deployment.device.workflows['calibration'];original=flow.calibration_installed
    def installed(snapshot):
        # Common C invalidation finishes before publishing a new prepared identity.
        original(snapshot);preparation.notify(snapshot)
    flow.calibration_installed=installed;calibration.on_calibration_installed=installed
    preparation.notify(calibration.installed_pair())  # Notification never prepares or moves.
    return preparation


def build_physical_application(profile, *, output, start_cameras=False, event=None):
    """Start only CPU UI/camera backend; sensors require start_cameras explicitly."""
    _paths()
    if profile.get('native_gripper_calibration'):
        from arx_env.joint_head_jaw_mapping import configure_mapping
        configure_mapping(profile['jaw_mapping_file'])
    from .service import Application, Device
    from .bridges import start_control_gateway
    from .rgbd_service import CommonCameraService
    from .recording_process import capture_control
    from .sample_clock import clock_spec
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    intrinsics = {name:json.loads(Path(profile['cameras'][role]['intrinsics']).read_text()) for role,name in ROLES.items()}
    shared_descriptor=profile.get('camera_service_descriptor')
    if profile.get('data_plane')is not None:
        from .ros_inputs import RosCameraView
        cameras=RosCameraView(profile['data_plane'],profile['robot_id'],intrinsics,'physical')
    else:
        cameras = (CommonCameraService.connect(shared_descriptor) if shared_descriptor else
                   CommonCameraService.start(intrinsics, output/'camera-service', 'physical'))
    if cameras.descriptor['environment']!='physical' or cameras.descriptor['intrinsics']!=intrinsics:
        raise ValueError('shared physical camera identities differ from deployment profile')
    ports = cameras.start_servers()
    tokens = {name:output/(name+'-gateway-token') for name in ('calibration','collection')}
    secrets_by_business = {name:_private_token(path) for name,path in tokens.items()}
    binding_path=profile.get('hardware_binding')or(output/'collection-binding.json'if profile.get('collection_setup')else None)
    def recording(payload, on_fault):
        if profile.get('data_plane')is not None:
            return capture_control(output/'collection-raw',payload,cameras=None,environment='physical',clock=clock_spec(),
                source_time=device.runtime.owner.control_time,on_fault=on_fault,ros_runtime=device.runtime)
        if profile.get('recording_backend','capture_session_v1') is None:
            from .recording_process import ProcessRecordingSession
            return ProcessRecordingSession(output/'collection-raw'/secrets.token_hex(16),plan_ref=payload['plan_ref'],
                camera_channels=cameras.channels,storage_format='segments',execution_timeline=payload.get('execution_timeline'),
                environment='physical',execution_owner=payload.get('execution_owner'),on_fault=on_fault,
                persistent_service=True,max_wall_time_seconds=7200,**payload.get('recording_preparation',{}))
        native=(profile.get('inference')or{}).get('native_rgb')
        if native:
            from .prometheus_rgb import RGBClient
            source=RGBClient(**native)
        else:source=cameras
        return capture_control(output/'collection-raw',payload,cameras=source,environment='physical',clock=clock_spec(),
            source_time=device.runtime.owner.control_time,on_fault=on_fault)
    recording.supports_deferred_preparation = True
    recording.recording_backend=profile.get('recording_backend','capture_session_v1')
    runtime_factory = physical_runtime_factory(profile, recording_factory=recording,event=event,
        hardware_binding=binding_path,execution_root=output/'execution')
    device = create_device(profile['robot_id'], 'physical', runtime_factory=runtime_factory,
                           label=profile.get('label') or 'A1 真机 · 待显式连接')
    if profile.get('recording_backend','capture_session_v1')is not None:
        from .control_owner import collection_recording_root
        record_root=(collection_recording_root(Path(profile['control_owner_descriptor']).parent)
            if profile.get('control_owner_descriptor')else output/'collection-raw')
        device.configure_capture_results(record_root)
    device.control_owner_descriptor=profile.get('control_owner_descriptor')
    device.control_lifecycle=profile.get('control_lifecycle','managed')
    if profile.get('triton_preview'):
        from .triton_preview import TritonPreviewClient
        device.triton_preview = TritonPreviewClient(profile['triton_preview'])
    if profile.get('precision_study'):
        from .precision_study import PrecisionStudy
        device.precision_study=PrecisionStudy(device,profile['precision_study'])
    if profile.get('control_owner_descriptor'):
        existing=None
        try:
            existing=_remote_control_client(profile)
            if existing.nonce is not None:
                if device.control_lifecycle=='external':existing.attach()
                existing.inputs=existing.control_inputs();device.attach_existing_owner(existing)
        except (OSError,ValueError,ConnectionError,PermissionError)as error:
            device.error='独立Owner尚不可读；未接入或重启SDK：'+str(error)
        finally:
            if existing is not None and device.runtime is not existing:existing.detach()
    if profile.get('collection_history_catalogs'):
        from .collection_history import load_history,CollectionHistoryGroup
        try:
            device.collection_history=CollectionHistoryGroup([load_history(path,environment='physical',robot_id=profile['robot_id'])
                for path in profile['collection_history_catalogs']])
        except (OSError,ValueError,KeyError,TypeError) as error:
            device.collection_history_error=str(error)
    if profile.get('physical_cup_return_candidate'):
        device.physical_cup_return_candidate=profile['physical_cup_return_candidate']
        device.cup_return_root=Path(profile['storage_root'])/'physical-cup-returns'
    application = Application({'physical':device, 'fake':Device('fake-unconfigured','fake',None,label='Fake · 未装配仿真输入输出')})
    application.operations_managed = True
    gateway, _ = start_control_gateway(device, secrets_by_business)
    deployment = SimpleNamespace(application=application,device=device,cameras=cameras,gateway=gateway,
        camera_ports=ports,output=output,camera_processes=[],collection=None,calibration=None,inference=None)
    install_business_workflows(device, application, camera_service=cameras)
    try:
        _attach_physical_businesses(profile,deployment,tokens,start_cameras,event)
        return deployment
    except BaseException:
        close_deployment(deployment)
        raise


def _attach_physical_businesses(profile,deployment,tokens,start_cameras,event):
    device,application,cameras = deployment.device,deployment.application,deployment.cameras
    gateway,ports,output = deployment.gateway,deployment.camera_ports,deployment.output
    if profile.get('calibration_config'):
        from examples.scripts.debug.joint_head_workbench import Workbench
        config = calibration_configuration(profile['calibration_config'])
        if config.get('environment') != 'physical' or config.get('simulation_only') is True:
            raise ValueError('physical calibration configuration required, never relabel a fake config')
        config.setdefault('calibration_root',str(Path(profile['storage_root'])/'calibrations'))
        config.update(robot_id=device.id,hardware_control=dict(port=gateway.server_port,token_file=str(tokens['calibration'])),
            hardware_status_owner_port=gateway.server_port,hardware_camera_port=ports['head'],
            hardware_wrist_camera_ports={k:ports[k] for k in ('left','right')},
            hardware_camera_serials={k:v['serial'] for k,v in profile['cameras'].items()},
            hardware_wrist_camera_files={k:profile['cameras'][k]['intrinsics'] for k in ('left','right')},
            camera=profile['cameras']['head']['intrinsics'],jaw_mapping_file=profile['jaw_mapping_file'])
        (output/'calibration-config.json').write_text(json.dumps(config,allow_nan=False,indent=2)+'\n')
        deployment.calibration = Workbench(config, profile.get('calibration_output') or Path(profile['storage_root'])/'calibration-workbench')
        install_business_workflows(device, application, calibration=deployment.calibration)
        from .physical_preview import PhysicalPreview
        device.preview=PhysicalPreview(deployment.calibration)
        device.preview.start()
    if profile.get('collection_setup'):
        _attach_remote_collection(profile,deployment,tokens)
    if profile.get('collection_config'):
        deployment.collection = _collection(profile,device,application,gateway,tokens,ports,output)
    if profile.get('inference'):
        from .inference_workflow import InferenceWorkflow
        config = profile['inference']
        def profile_provider():
            if config.get('control_mapping_pending'):
                raise ValueError(config['control_mapping_pending'])
            if 'native_binding'in config:
                from .mixed366_control import load_binding
                return load_binding(config['native_binding'],profile)
            from .calibration_profile import load_device_profile
            installed_profile,binding=load_device_profile(config['profile_path'],environment='physical',robot_id=device.id)
            if json.loads(Path(config['binding_path']).read_text())!=binding:
                raise ValueError('inference deployment binding differs from shared installed-profile receipt')
            if deployment.calibration is not None:
                current=deployment.calibration.installed_pair() or {}
                manifest=Path(deployment.calibration.config['calibration_root'])/'hardware/active/manifest.json'
                if (current.get('verified') is not True or any(binding.get(key)!=current.get(key) for key in ('head_id','wrist_id'))
                        or binding.get('active_sha256',{}).get('manifest.json')!=hashlib.sha256(manifest.read_bytes()).hexdigest()):
                    raise ValueError('static inference profile differs from current C installation; prepare its updated shared profile')
            return installed_profile,binding
        if profile.get('collection_setup'):
            from .calibration_profile import installed_profile_provider
            details=profile['collection_setup']
            profile_provider=installed_profile_provider(config_path=output/'calibration-config.json',
                capture_root=deployment.calibration.output,registration=details['registration'],
                profile_template=details['profile_template'],output=output,
                installed_snapshot=deployment.calibration.installed_pair,environment='physical')
        installed_provider=profile_provider
        def selected_profile_provider():
            from .inference_model import model_family
            if model_family(deployment.inference.selected_model_id)in('wam','wam-mem'):
                from .wam import native_binding
                return native_binding(profile)
            return installed_provider()
        deployment.inference = InferenceWorkflow(device,Path(profile['storage_root'])/'inference',camera_service=cameras,
            model_endpoint=config['model_endpoint'],model_token_file=config['model_token_file'],profile_provider=selected_profile_provider,
            event=(lambda name,data:event(name,**data)) if event is not None else None,**_inference_options(profile))
        controls=config.get('operator_controls')
        if controls is not None:
            if (not isinstance(controls,dict)or not {'home_velocity_rad_s','timeout_s','focus_timeout_s'}<=controls.keys()
                    or any(type(controls[k])not in(int,float)or not 0<controls[k]<float('inf')
                           for k in ('home_velocity_rad_s','timeout_s','focus_timeout_s'))):
                raise ValueError('explicit positive operator Home/focus configuration required')
            if controls.get('pedal_source_descriptor')and (not Path(controls['pedal_source_descriptor']).is_absolute()
                    or type(controls.get('event_max_age_s'))not in(int,float)or not 0<controls['event_max_age_s']<float('inf')
                    or type(controls.get('double_press_window_s'))not in(int,float)
                    or not 0<controls['double_press_window_s']<=controls['event_max_age_s']):
                raise ValueError('absolute pedal source, finite event age and explicit double-press window required')
            deployment.inference.operator_controls=deepcopy(controls)
            if controls.get('pedal_source_descriptor'):
                application.install_operator_pedal(device,deployment.inference,controls['pedal_source_descriptor'])
        if config.get('native_rgb')and profile.get('data_plane')is None:
            from .prometheus_rgb import RGBClient
            rgb=config['native_rgb']
            if set(rgb)!={'directory','owner_path'}or any(not Path(p).is_absolute()for p in rgb.values()):
                raise ValueError('absolute existing raw RGB subscriber paths required')
            deployment.inference.rgb_source=RGBClient(**rgb)
            if deployment.inference.recording_backend:deployment.inference.cameras=deployment.inference.rgb_source
            from .bridges import CameraMedia
            from PIL import Image
            import io
            for role,camera in ROLES.items():
                old=device.media.get(role)
                if old is not None:old.closed.set()
                def frame(camera=camera):
                    value=deployment.inference.rgb_source.latest_rgb(camera)
                    buffer=io.BytesIO();Image.frombytes('RGB',(640,480),value['rgb']).save(buffer,format='JPEG',quality=85)
                    return buffer.getvalue(),dict(value['metadata'],frame_id=str(value['metadata']['frame_number']),ready=True)
                device.media[role]=CameraMedia(role,role,frame,environment='physical')
        install_business_workflows(device,application,inference=deployment.inference)
    if start_cameras and not profile.get('camera_service_descriptor'):
        from .rgbd_service import start_capture_sources
        deployment.camera_processes.extend(start_capture_sources(profile,cameras,output))


def close_deployment(deployment):
    """Only owned CPU/sensor resources; never disconnect an acquired robot implicitly."""
    runtime=deployment.device.runtime
    remote=bool(getattr(deployment.device,'control_owner_descriptor',None)or(runtime is not None and getattr(runtime,'remote_control_owner',False)))
    if runtime is not None and not remote:
        raise ValueError('explicit supported controller release required before stopping this service')
    if remote and runtime is not None:
        # Business exit detaches its view. The separately managed Owner retains
        # its finite execution and the SDK; no implicit Stop/release is sent.
        runtime.detach();deployment.device.closed.set()
        if deployment.device.thread is not None:deployment.device.thread.join(1)
        deployment.device.runtime=None;deployment.device.credentials={}
    if getattr(deployment.device,'capture_results',None)is not None:deployment.device.capture_results.close()
    for process in deployment.camera_processes:
        if process.poll() is None:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
    for media in deployment.device.media.values():media.closed.set()
    for workflow in deployment.device.workflows.values():
        if getattr(workflow,'closed',None) is not None:workflow.closed.set()
    if deployment.calibration is not None:
        if deployment.device.preview is not None:
            deployment.device.preview.close()
        deployment.calibration.monitor_stop.set()
    if deployment.inference is not None:
        deployment.inference.close()
    if remote:
        # Remote collection/model/camera workers have their own lifecycle. A
        # business-server reload is not authorization to stop their processes.
        closed=getattr(deployment.collection,'closed',None)
        if closed is not None:closed.set()
    elif isinstance(deployment.collection,CollectionPreparation):
        deployment.collection.close(lifecycle_admission=lambda action: action.startswith('stop:'))
    elif deployment.collection is not None:
        for name in ('detector','planner','scene'):
            worker=getattr(deployment.collection,name,None)
            if worker is not None:worker.close()
    deployment.gateway.shutdown();deployment.gateway.server_close()
    deployment.cameras.close()
