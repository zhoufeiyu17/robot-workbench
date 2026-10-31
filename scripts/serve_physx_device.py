#!/usr/bin/env python3
"""Bounded resident fake device: one plant, one clock, shared control and cameras."""
import argparse
import errno
from copy import deepcopy
from dataclasses import asdict,replace
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'calibration'), str(ROOT/'calibration/infra'), str(ROOT/'rule_based_workbench')]
from probe_owner_physx import preflight, save


def plant_joint_offsets(base):
    """Fixed plant-only truth, ordered left J1..J6 then right J1..J6."""
    values=base.get('plant_joint_offsets_rad',[0.]*12)
    if not isinstance(values,list) or len(values)!=12 or any(type(q) not in (int,float) for q in values):
        raise ValueError('plant_joint_offsets_rad must contain twelve finite numbers')
    try:offsets=tuple(float(q) for q in values)
    except OverflowError as error:raise ValueError('plant_joint_offsets_rad must contain twelve finite numbers') from error
    if not all(math.isfinite(q) for q in offsets):
        raise ValueError('plant_joint_offsets_rad must contain twelve finite numbers')
    return offsets


def install_notification(workflow,workbench,notify):
    """Synchronous invalidation must complete before background setup is notified."""
    original=workflow.calibration_installed
    def installed(snapshot):
        original(snapshot)
        notify(snapshot)
    workflow.calibration_installed=installed
    workbench.on_calibration_installed=installed
    return installed


def bootstrap_calibration(workbench,*,notify=None,runtime=None):
    """Publish the actual restored installation; never restore a motion approval."""
    from robot_workbench.workflows import CalibrationWorkflow
    snapshot=workbench.installed_pair()
    if not snapshot or snapshot.get('verified') is not True:
        raise ValueError('restored calibration must match this Workbench geometry and library')
    if runtime is not None:runtime.notify_calibration_installed(CalibrationWorkflow.install_identity(snapshot))
    if notify is not None:notify(snapshot)
    return snapshot


def inference_profile_provider(output, calibration, template_path, *, diagnostic_source=None):
    from robot_workbench.calibration_profile import installed_profile_provider
    return installed_profile_provider(config_path=output/'calibration-config.json',capture_root=output/'calibration-workbench',
        registration=output/'registration.json',profile_template=template_path,output=output,
        installed_snapshot=calibration.installed_pair,environment='fake',diagnostic_source=diagnostic_source)


def inference_options(setup):
    """Use the shared trusted model-deployment validation, preserving the fake default."""
    from robot_workbench.device_bootstrap import _inference_options
    from robot_workbench.inference_transport import MODEL_ID
    return _inference_options(dict(inference=dict(setup,model_id=setup.get('model_id',MODEL_ID)),disabled_grippers=[]))


def inference_source(root):
    """Pin only existing Infer settings/history and indexed playback files."""
    from robot_workbench.collection_history import _file, _pin
    from robot_workbench.inference_history import DATABASE, entries, checked_entry
    root=Path(root)
    if not root.is_absolute() or root.resolve()!=root or root.is_symlink() or not root.is_dir():
        raise ValueError('retained inference source requires a canonical absolute directory')
    files={};ids=set()
    for name in ('execution-settings.json','history.json',DATABASE):
        if not (root/name).exists() and not (root/name).is_symlink():continue
        path=_file(root,name)
        files[name]=_pin(path)
        if name=='execution-settings.json':
            if path.stat().st_size>32*1024*1024:raise ValueError('retained inference metadata exceeds 32 MiB')
            value=json.loads(path.read_bytes())
            if not isinstance(value,dict):raise ValueError('retained inference metadata must be an object')
            if (type(value.get('execute_steps')) is not int or not 1<=value['execute_steps']<=60
                    or type(value.get('revision')) is not int or value['revision']<0):
                raise ValueError('invalid retained inference execution settings')
    for entry in entries(root):
        identity,_,_,relative,_=checked_entry(entry)
        if identity in ids:raise ValueError('duplicate retained inference result ID')
        ids.add(identity)
        if relative is not None:files[relative]=_pin(_file(root,relative))
    return dict(schema='inference_retained_source.v1',root=str(root),files=files) if files else None


def validate_inference_source(source):
    if (not isinstance(source,dict) or set(source)!={'schema','root','files'}
            or source['schema']!='inference_retained_source.v1' or not isinstance(source['root'],str)
            or not isinstance(source['files'],dict) or not source['files']):
        raise ValueError('explicit pinned inference source required')
    if inference_source(source['root'])!=source:
        raise ValueError('retained inference file set, SHA or size changed')
    return deepcopy(source)


def restore_inference(source,output):
    """Restore historical metadata/playback only; no model or execution state."""
    from robot_workbench.collection_history import _file, _pin
    source=validate_inference_source(source);root=Path(source['root']);output=Path(output)
    if (not output.is_absolute() or output.resolve()!=output or output.exists()
            or output.is_symlink() or output.is_relative_to(root)):
        raise ValueError('inference restore destination must be a new independent directory')
    output.mkdir(parents=True);transports={}
    for relative,pin in source['files'].items():
        original=_file(root,relative);target=output/relative;target.parent.mkdir(parents=True,exist_ok=True)
        linked=False
        if relative.endswith('.mp4'):
            try:os.link(original,target,follow_symlinks=False);linked=True
            except OSError as error:
                if error.errno!=errno.EXDEV:raise
        if not linked:
            with original.open('rb') as src,target.open('xb') as dst:shutil.copyfileobj(src,dst)
        if _pin(_file(output,relative))!=pin or _pin(_file(root,relative))!=pin:
            raise ValueError('retained inference file changed during restore')
        transports[relative]='hardlink_readonly_playback' if linked else 'copy'
    validate_inference_source(source)
    receipt=dict(status='restored',source=source,files=transports,restored_wall_ns=time.time_ns(),
                 scope='historical Infer settings and playback; no new execution, model loading or raw export',
                 original_receipt_paths_preserved=True)
    with (output/'restore-receipt.json').open('x') as stream:json.dump(receipt,stream,ensure_ascii=False,allow_nan=False,indent=2)
    return receipt


def nominal_inputs(path):
    config = json.loads(path.read_text())
    if config.get('model_family') != 'ac_one' or config.get('simulation_only') is not True:
        raise ValueError('explicit nominal A1 fake config required')
    for key in ('urdf','mesh_manifest','scene_seed','candidate','camera_seed','jaw_mapping_file'):
        if not Path(config[key]).is_absolute() or not Path(config[key]).is_file():
            raise ValueError('absolute nominal input missing: '+key)
    asset_root = Path(config['urdf']).parent
    seal_path = asset_root/'SHA256SUMS.json'
    if hashlib.sha256(seal_path.read_bytes()).hexdigest() != config['asset_seal_sha256']:
        raise ValueError('nominal A1 asset seal differs')
    for relative, digest in json.loads(seal_path.read_text()).items():
        file = asset_root/relative
        if not file.resolve().is_relative_to(asset_root.resolve()) or hashlib.sha256(file.read_bytes()).hexdigest() != digest:
            raise ValueError('nominal A1 input differs: '+relative)
    return config


def printed_marker_fixture(candidate):
    """The printed face sits 0.1 mm outside its housing, not on the same plane.

    This is plant geometry only. The estimator keeps the sealed nominal mounts.
    """
    markers=[]
    for ident,link in ((24,'left_link6'),(30,'right_link16')):
        transform=deepcopy(candidate['T_link6_marker'][str(ident)])
        for axis in range(3):transform[axis][3]-=.0001*transform[axis][2]
        markers.append(dict(id=ident,link=link,size_m=.015,T_link_marker=transform,
                            print_surface_offset_m=.0001))
    return markers


def configure_calibration(nominal, output, intrinsics, ports, gateway_port, token_file, *, source_clock, auto_confirm=False,
                          resident_deadline_wall_ns=None,robot_id=None,persistent_service=False):
    from robot_workbench.physx_camera import CAMERAS
    keys=('urdf','mesh_manifest','scene_seed','candidate','camera_seed','jaw_mapping_file','model_family',
          'asset_geometry_id','asset_seal_sha256','parking_support_contact','wrist_nominal_mounts',
          'wrist_camera_housings','wrist_support_proxy','margin_px')
    config = {key:deepcopy(nominal[key]) for key in keys if key in nominal}
    camera_files={}
    for role,name in CAMERAS.items():
        path=output/('camera-'+role+'.json');save(path,intrinsics[name]);camera_files[role]=str(path)
    calibration_root=output/'calibrations'
    calibration_root.mkdir(exist_ok=False)
    if robot_id is not None and not robot_id.startswith('fake-'):raise ValueError('fake robot identity required')
    config.update(robot_id=robot_id or 'fake-physx-'+output.name, simulation_only=True, mode='simulation',
        environment='fake',robot_source='simulated',clock=deepcopy(source_clock),
        evidence_source='physx_articulation', calibration_root=str(calibration_root),
        camera=camera_files['head'], marker_size_m=.015, gap_m=dict(left=.08,right=.08),
        head_motion_seed=str(ROOT/'calibration/standard_head_motion_seed_a1_fake_v2.json'),
        wrist_seed_file='configs/wrist-motion-seed-a1-fake-v1.json',
        hardware_control=dict(port=gateway_port,token_file=str(token_file)),
        hardware_status_owner_port=gateway_port,hardware_camera_port=ports['head'],
        hardware_wrist_camera_ports={side:ports[side] for side in ('left','right')},
        hardware_wrist_camera_files={side:camera_files[side] for side in ('left','right')},
        hardware_camera_serials={role:intrinsics[name]['serial'] for role,name in CAMERAS.items()},
        wrist_simulation_cameras={side:dict(intrinsics[CAMERAS[side]],rectified=True) for side in ('left','right')},
        pipeline_auto_confirm=auto_confirm,bind_host='127.0.0.1')
    if persistent_service:
        if resident_deadline_wall_ns is not None:raise ValueError('persistent service cannot include a deadline')
        config.update(persistent_service=True,resident_deadline_wall_ns=None)
    elif resident_deadline_wall_ns is not None:
        if type(resident_deadline_wall_ns) is not int or resident_deadline_wall_ns<=0:
            raise ValueError('fixed positive resident calibration deadline required')
        config['resident_deadline_wall_ns']=resident_deadline_wall_ns
    save(output/'calibration-config.json',config)
    return config


def run(args, base, nominal):
    resident_deadline_wall_ns=args.resident_deadline_wall_ns
    deadline=None if resident_deadline_wall_ns is None else time.monotonic()+max(0.,(resident_deadline_wall_ns-time.time_ns())/1e9)
    os.environ['ROBOT_SIM_ASSET_ROOT']=base['pipeline_config']['assets']
    from isaaclab.app import AppLauncher
    app=AppLauncher(headless=True,enable_cameras=True,device='cuda:0',active_gpu=args.gpu,multi_gpu=False,
        kit_args=f'--/renderer/activeGpu={args.gpu} --/renderer/multiGpu/enabled=false '
        '--/renderer/multiGpu/autoEnable=false --/renderer/gpuEnumeration/glInterop/enabled=false').app
    env=producer=device=ui=gateway=collection_preparation=task_observer=preview_service=model_process=None
    done=threading.Event();requests=queue.Queue(maxsize=1)
    events=queue.Queue(maxsize=4096);logging_failed=threading.Event()
    writer_done=threading.Event()
    def event(kind,**fields):
        try:events.put_nowait(dict(kind=kind,wall_ns=time.time_ns(),**fields))
        except queue.Full:logging_failed.set()
    def write_events():
        try:
            with (args.output/'events.jsonl').open('x') as stream:
                while not writer_done.is_set() or not events.empty():
                    try:row=events.get(timeout=.1)
                    except queue.Empty:continue
                    if row['kind']=='_native_task_receipt':
                        from robot_workbench.resident_collection import write_task_receipt,publish_task_status
                        owner,receipt=row['owner'],row['receipt']
                        try:
                            result=write_task_receipt(args.output/'native-tasks',owner,receipt)
                            try:
                                from robot_workbench.cup_return import remember_success
                                remember_success(device,row.get('runner'),receipt,result)
                            except Exception as recovery_error:
                                result['scene_recovery_error']=str(recovery_error)
                                device.scene_recovery_error=str(recovery_error)
                            row=dict(kind='native_task_receipt_written',wall_ns=time.time_ns(),**result)
                        except Exception as error:
                            with owner.lock:
                                current=dict(getattr(owner,'native_task_status',{}))
                                if current.get('observer_id')==receipt.get('observer_id'):
                                    publish_task_status(owner,dict(current,task_outcome='unverifiable',task_qualified=False,
                                        receipt_path=None,receipt_sha256=None,message='任务证据落盘失败：'+str(error)))
                            row=dict(kind='native_task_receipt_failed',wall_ns=time.time_ns(),error=str(error))
                    stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush();events.task_done()
        except Exception:logging_failed.set()
    writer=threading.Thread(target=write_events,daemon=True,name='resident-evidence');writer.start()
    for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,lambda *_:done.set())
    try:
        from robot_sim.tasks.manager_based.coffee_make.env import CoffeeMakeEnv
        from robot_sim.tasks.manager_based.coffee_make.episode_scene import EpisodeScene
        from robot_sim.tasks.manager_based.coffee_make.generalization_env_cfg import make_generalization_env_cfg
        from robot_sim.tasks.manager_based.coffee_make.generalization_execution import ControlSpec
        from robot_sim.tasks.manager_based.coffee_make.interaction_evaluation import EpisodeAcceptanceSpec
        from isaac_lane import configure_observation_scene
        from examples.scripts.debug.joint_head_isaac_capture import create_markers
        from examples.scripts.debug import joint_head_real_owner as ref
        from examples.scripts.debug.joint_head_fake_hardware import FakeSafety
        from arx_env.joint_head_jaw_mapping import configure_mapping
        from robot_workbench.runtime import Runtime
        from robot_workbench.service import Application,Device,serve
        from robot_workbench.bridges import start_control_gateway,CameraMedia
        from robot_workbench.physx import IsaacPlant,PhysxBackend,SimulationClock
        from robot_workbench.physx_camera import CameraProducer,camera_intrinsics,CAMERAS
        from robot_workbench.sample_clock import clock_spec
        import isaaclab.sim as sim_utils
        configure_mapping(nominal['jaw_mapping_file'])
        episode=EpisodeScene.from_dict(base['initial_suite']['episodes'][0])
        cfg=make_generalization_env_cfg(control=ControlSpec(240,8,2.,5.,300.),
            acceptance=replace(EpisodeAcceptanceSpec.development(),open_position_m=82.992/2000),
            asset_root=Path(base['pipeline_config']['assets']),cameras=True,camera_rig='d405',
            physics_device='cuda:0',pad_torsional_radius_m=.0015,solver_profile='contact_32_8',
            episode=episode,continuous_motion=True)
        configure_observation_scene(cfg)
        env=CoffeeMakeEnv(cfg)
        preset=base['pipeline_config'].get('cup_contact_preset')
        if preset:
            from cup_presets import configure,bind_preset_identity
            bind_preset_identity(env._m2_runtime.physical_identity,preset)
            configure(sim_utils.get_current_stage(),env._m2_runtime.root,preset['stiffness_n_m'],preset['damping_n_s_m'])
        maintenance=(args.collection_spec or {}).get('maintenance_restore') or {}
        env.reset(options={'episode':episode})
        if maintenance.get('scene'):
            from robot_workbench.fake_scene import restore_episode_poses,normalize_restored_scene
            restored_scene,orientation_corrections=normalize_restored_scene(maintenance['scene'])
            restored=EpisodeScene.from_dict(restored_scene)
            restored,final_corrections=restore_episode_poses(env,restored,episode)
            orientation_corrections+=final_corrections
            env.sim.forward();env.scene.update(dt=env.physics_dt)
            save(args.output/'scene-restored.json',dict(status='restored',episode=asdict(restored),orientation_corrections=orientation_corrections,
                purpose='maintenance state restoration; not a completed return motion',
                parent=maintenance.get('cup_return_parent',{}).get('native_receipt')))
        import numpy as np
        from scipy.spatial.transform import Rotation
        robot=env.scene['robot'];base_index=robot.body_names.index('base_link')
        registration=np.eye(4)
        registration[:3,3]=robot.data.body_pos_w[0,base_index].cpu().numpy()
        quaternion=robot.data.body_quat_w[0,base_index].cpu().numpy()
        registration[:3,:3]=Rotation.from_quat(quaternion[[1,2,3,0]]).as_matrix()
        save(args.output/'registration.json',dict(length_unit='m',
            T_environment_calibration_world=registration.tolist(),
            source='fixed plant base_link mount; not an estimated camera transform'))
        candidate=json.loads(Path(nominal['candidate']).read_text())
        markers=printed_marker_fixture(candidate)
        create_markers(sim_utils.get_current_stage(),markers)
        for _ in range(4):env._m2_runtime._render()
        save(args.output/'truth-fixture.json',dict(markers=markers,
            marker_basis='sealed nominal fixture plus 0.1mm printed-face offset along marker -Z; independent of fitted/install outputs',
            nominal_candidate_sha256=hashlib.sha256(Path(nominal['candidate']).read_bytes()).hexdigest(),
            plant_base_case=base['case_id'],estimator_reads_this_file=False))
        plant=IsaacPlant(env);clock=SimulationClock()
        backend=PhysxBackend(plant,joint_offsets=args.plant_joint_offsets_rad,metal_gap_offset_mm=.008,event=event)
        source_clock=clock_spec('fake',source_clock_id=backend.source_clock_id)
        plant_fixture=dict(source_clock_id=backend.source_clock_id,joint_offsets_rad=list(backend.offsets),metal_gap_offset_mm=backend.gap_offset)
        save(args.output/'plant-fixture.json',plant_fixture)
        from robot_workbench.resident_collection import NativeTaskObserver,CollectionPreparation
        def task_receipt(owner,receipt):
            if logging_failed.is_set():raise RuntimeError('resident evidence writer unavailable')
            events.put_nowait(dict(kind='_native_task_receipt',owner=owner,receipt=receipt,runner=owner.runner))
        task_observer=NativeTaskObserver(env._m2_runtime,backend,plant_fixture,task_receipt)
        intrinsics=camera_intrinsics(env.cfg.d405_rig)
        def recording_guard(lease):
            owner=backend.owner
            recording=getattr(owner,'recording',None) if owner else None
            return recording is not None and recording.lease_active(lease)
        producer=CameraProducer(intrinsics,args.camera_spool_root,recording_guard=recording_guard)
        ports=producer.start_servers(dict(zip(CAMERAS,args.camera_ports)))
        from robot_workbench.recording_process import ProcessRecordingSession
        recording_root=args.output/'collection-raw'
        recording_root.mkdir(exist_ok=False)
        def recording_factory(payload,on_fault):
            return ProcessRecordingSession(recording_root/secrets.token_hex(16)/'raw',
                plan_ref=payload['plan_ref'],camera_channels=producer.channels,storage_format='segments',
                execution_timeline=payload.get('execution_timeline'),environment='fake',
                execution_owner=payload.get('execution_owner'),on_fault=on_fault,
                resident_deadline_wall_ns=resident_deadline_wall_ns,persistent_service=args.persistent,
                max_wall_time_seconds=min(args.collection_spec['resources']['max_wall_time_minutes']*60,7200) if args.persistent else None,
                **payload.get('recording_preparation',{}))
        recording_factory.supports_deferred_preparation=True
        token_files={};tokens={}
        for business in ('calibration','collection'):
            path=args.output/(business+'-gateway-token')
            token=secrets.token_urlsafe(32)
            with os.fdopen(os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600),'w') as stream:stream.write(token)
            tokens[business]=token;token_files[business]=str(path)
        def factory():
            request={'ready':threading.Event(),'cancelled':False,'lock':threading.Lock()}
            requests.put_nowait(request)
            if not request['ready'].wait(30):
                with request['lock']:
                    if not request['ready'].is_set():
                        request['cancelled']=True
                        raise TimeoutError('PhysX main thread did not acquire the controller')
            if 'error' in request:raise RuntimeError(request['error'])
            return request['runtime']
        robot_id=args.resume_source['origin_robot_id'] if args.resume_source else 'fake-physx-'+args.output.name
        from robot_workbench.device_bootstrap import create_device,install_business_workflows
        device=create_device(robot_id,'fake',runtime_factory=factory,label='A1 PhysX · 同源三相机',
                      drive=lambda runtime,closed:closed.wait())
        device.camera_service=producer.common
        device.camera_restart=producer.restart
        device.camera_start=producer.start
        device.camera_stop=producer.stop
        if args.collection_history is not None:device.collection_history=args.collection_history
        if args.resume_source:device.label='A1 PhysX · 日常开发（复用标定）'
        if args.diagnostic_source:
            from robot_workbench.calibration_profile import diagnostic_metadata
            device.label='A1 PhysX · 复用标定诊断（非空库验收）'
            device.diagnostic_calibration=diagnostic_metadata(args.diagnostic_source)
        application=Application({'fake':device,
            'physical':Device('physical-unconfigured','physical',None,label='真机 · 未配置')})
        application.operations_managed=True
        from robot_workbench.fake_scene import FakeScene
        scene=FakeScene(env,episode,device,application,event)
        def update_scene(payload=None,*,read_only=False):
            request={'ready':threading.Event(),'cancelled':False,'lock':threading.Lock(),
                     'scene_payload':deepcopy(payload),'scene_read_only':read_only}
            requests.put_nowait(request)
            if not request['ready'].wait(30):
                with request['lock']:
                    if not request['ready'].is_set():
                        request['cancelled']=True
                        raise TimeoutError('场景移动尚未完成；请核验诊断，不自动重试')
            if 'error' in request:raise RuntimeError(request['error'])
            return request['result']
        device.fake_scene_update=update_scene
        device.fake_scene_snapshot=lambda:update_scene(read_only=True)
        from robot_workbench.cup_return import configure as configure_recovery
        configure_recovery(device,args.output,maintenance)
        gateway,_=start_control_gateway(device,tokens,port=args.gateway_port)
        calibration_config=configure_calibration(nominal,args.output,intrinsics,ports,gateway.server_port,
            token_files['calibration'],source_clock=source_clock,auto_confirm=args.calibration_auto_confirm,
            resident_deadline_wall_ns=resident_deadline_wall_ns,robot_id=robot_id,persistent_service=args.persistent)
        if args.resume_source:
            from robot_workbench.calibration_profile import restore_calibration
            calibration_config['calibration_resume']=restore_calibration(args.resume_source,calibration_config,args.output)
            save(args.output/'calibration-config.json',calibration_config)
        from examples.scripts.debug.joint_head_workbench import Workbench
        from robot_workbench.workflows import CalibrationWorkflow
        calibration=Workbench(calibration_config,args.output/'calibration-workbench')
        if args.resume_source:bootstrap_calibration(calibration)
        device.workflows['calibration']=CalibrationWorkflow(calibration,device,application)
        mesh_workbench=calibration
        if args.diagnostic_source:
            from robot_workbench.calibration_profile import diagnostic_mesh_view
            mesh_workbench,device.diagnostic_mesh_candidates=diagnostic_mesh_view(calibration,args.diagnostic_source)
        if args.collection_spec is not None and args.collection_spec['resources']['max_workers'] in (5,6):
            from robot_workbench.preview_service import prepare_preview_roles,resident_execution,PreviewService,calibration_mesh_inputs
            from arx_env.joint_head_workbench_geometry import jaw_gaps
            prepare_preview_roles(args.collection_spec,args.output,calibration_config)
            def preview_admission():
                if collection_preparation and collection_preparation.parts:
                    from run_pipeline import load_config
                    return load_config(Path(collection_preparation.parts.pipeline_config))[0]
                return {'execution':resident_execution(args.collection_spec,args.output)}
            def mesh_inputs(camera,candidate):
                if getattr(device,'diagnostic_calibration',{}).get('invalidated'):
                    raise ValueError('复用标定诊断已失效，请启动新运行')
                return calibration_mesh_inputs(mesh_workbench,producer,camera,candidate)
            preview_service=PreviewService(execution=resident_execution(args.collection_spec,args.output),
                deadline_wall_ns=resident_deadline_wall_ns,deps=base['pipeline_config']['deps'],
                admission_config=preview_admission,
                validate_pose=lambda q,gaps:(calibration.geometry._q(q),jaw_gaps(gaps)),
                mesh_inputs=mesh_inputs)
            device.preview=preview_service
            preview_service.start()
        if args.collection_spec is not None:
            from robot_workbench.calibration_profile import diagnostic_snapshot
            collection_preparation=CollectionPreparation(contract=args.collection_spec,output=args.output,base=base,
                device=device,application=application,ports=ports,directories=producer.directories,
                control_url='http://127.0.0.1:'+str(gateway.server_port),token_file=token_files['collection'],
                deadline_wall_ns=resident_deadline_wall_ns,
                installed_snapshot=(lambda:diagnostic_snapshot(args.diagnostic_source)) if args.diagnostic_source else calibration.installed_pair,
                diagnostic_source=args.diagnostic_source)
            device.collection_preparation=collection_preparation
            install_notification(device.workflows['calibration'],calibration,collection_preparation.notify)
            if args.diagnostic_source:collection_preparation.notify(diagnostic_snapshot(args.diagnostic_source))
            if args.resume_source:bootstrap_calibration(calibration,notify=collection_preparation.notify)
        inference=None;support_sample=None
        inference_setup=(args.collection_spec or {}).get('inference_setup')
        if inference_setup:
            from robot_workbench.inference_workflow import InferenceWorkflow
            if 'model_manifest' in inference_setup:
                from robot_workbench.inference_transport import start_model_service
                model_process,endpoint=start_model_service(inference_setup,args.output/'model-service',token_files['calibration'])
                model_token=token_files['calibration']
            else:
                endpoint,model_token=inference_setup['model_endpoint'],inference_setup['model_token_file']
            inference=InferenceWorkflow(device,args.output/'inference',camera_service=producer.common,
                model_endpoint=endpoint,model_token_file=model_token,
                **inference_options(inference_setup),
                profile_provider=inference_profile_provider(args.output,calibration,
                    args.collection_spec['collection_setup']['profile_template'],diagnostic_source=args.diagnostic_source),
                support_provider=lambda now,feedback:deepcopy(support_sample),event=lambda name,data:event(name,**data))
        install_business_workflows(device,application,camera_service=producer.common,inference=inference)
        ui=serve(application,port=args.port,browser_ports=args.browser_ports)
        threading.Thread(target=ui.serve_forever,daemon=True,name='resident-workbench-http').start()
        save(args.output/'service.json',dict(pid=os.getpid(),environment='fake',clock=source_clock,
            resident_deadline_wall_ns=resident_deadline_wall_ns,persistent_service=args.persistent,
            source_clock_id=backend.source_clock_id,ui_port=ui.server_port,gateway_port=gateway.server_port,
            browser_ports=args.browser_ports,allowed_hosts=sorted(ui.allowed_hosts),
            camera_ports=ports,camera_directories=producer.directories,gateway_token_files=token_files,
            calibration_config=str(args.output/'calibration-config.json'),
            calibration_output=str(args.output/'calibration-workbench'),
            collection_binding=str(args.output/'collection-binding.json'),
            collection_contract=str(args.collection_contract) if args.collection_contract else None,
            collection_enabled=args.collection_spec is not None,
            inference_enabled=inference is not None,
            preview_enabled=preview_service is not None,
            full_pipeline_qualified=False,physical_hardware_connected=False,
            **({'calibration_resume':calibration_config['calibration_resume'],'fresh_C_completed':False} if args.resume_source else {}),
            **({'diagnostic_calibration':device.diagnostic_calibration,'fresh_C_completed':False} if args.diagnostic_source else {})))
        save(args.output/'collection-runtime.json',dict(environment='fake',clock=source_clock,
            robot_id=device.id,camera_serials=calibration_config['hardware_camera_serials'],
            owner_port=gateway.server_port,owner_token_file=token_files['collection'],
            camera_ports=ports,camera_directories=producer.directories,
            active_calibration_root=str(args.output/'calibrations/hardware/active'),
            binding_path=str(args.output/'collection-binding.json'),source_not_yet_acquired=True))
        class Idle:
            def tick(self):return True
        idle=Idle();last_status=0.
        def actor():
            runtime=device.runtime
            if runtime is None or not backend.is_connected:
                return dict(owner_available=False,owner_pid=None,command_epoch=None,runtime_nonce=None)
            owner=runtime.owner
            with owner.lock:
                return dict(owner_available=owner.feedback_healthy,owner_pid=os.getpid(),
                            command_epoch=owner.epoch,runtime_nonce=runtime.nonce)
        def release(runtime):
            if runtime.release_receipt and not runtime.release_receipt.get('sdk_close_returned'):
                raise RuntimeError('previous SDK release is unconfirmed; automatic retry forbidden')
            runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
            runtime.tick();runtime.close()
        def observe(method,owner,**options):
            try:getattr(task_observer,method)(owner,**options)
            except Exception as error:
                # Task observation is not a new motion fault/stop policy.
                event('native_task_monitor_error',error=str(error))
                if owner:
                    with owner.lock:
                        value=dict(getattr(owner,'native_task_status',{}))
                        value.update(task_outcome='unverifiable',task_qualified=False,message=str(error))
                        owner.native_task_status=value
                task_observer.active=None
        while not done.is_set() and (deadline is None or time.monotonic()<deadline):
            try:request=requests.get_nowait()
            except queue.Empty:request=None
            if request is not None:
                try:
                    if request['cancelled']:continue
                    if 'scene_payload' in request:
                        with request['lock']:
                            if request['cancelled']:continue
                            request['result']=scene.snapshot() if request.get('scene_read_only') else scene.apply(request['scene_payload'])
                        continue
                    runtime=Runtime(ref,FakeSafety,lambda:backend,device_id=device.id,environment='fake',
                        lock_root=args.output/'owner-lock',control_clock=clock,event=event,
                        jaw_mapping_file=nominal['jaw_mapping_file'],
                        hardware_binding=args.output/'collection-binding.json',execution_root=args.output,
                        recording_factory=recording_factory,
                        initialize_backend=lambda value:value.set_damping_hold(manual=True))
                    backend.owner=runtime.owner
                    settings_source=maintenance.get('execution_settings_source')
                    if settings_source and not runtime.execution_settings_path.exists():
                        raw=Path(settings_source['path']).read_bytes()
                        if hashlib.sha256(raw).hexdigest()!=settings_source['sha256']:
                            raise ValueError('retained execution settings changed')
                        runtime.restore_execution_settings(json.loads(raw))
                        save(runtime.execution_settings_path,runtime.export_execution_settings())
                    if args.diagnostic_source:
                        identity=hashlib.sha256(json.dumps(device.diagnostic_calibration,sort_keys=True).encode()).hexdigest()
                        runtime.notify_calibration_installed('diagnostic-calibration-'+identity)
                    if args.resume_source:bootstrap_calibration(calibration,runtime=runtime)
                    task_observer.attach(runtime.owner)
                    with request['lock']:
                        cancelled=request['cancelled']
                        if not cancelled:request['runtime']=runtime
                        request['ready'].set()
                    if cancelled:
                        release(runtime);backend.is_connected=True;backend.owner=None
                except Exception as error:request['error']=str(error)
                finally:request['ready'].set()
            current=device.runtime
            if current:current.owner.evidence_logging_healthy=not logging_failed.is_set()
            owner=current.owner if current else None
            if owner:observe('before_advance',owner)
            clock.advance(current or idle,backend,
                after_physics=lambda:observe('after_advance',owner,physics_advanced=True))
            observe('after_advance',owner,physics_advanced=False)
            if inference and owner and getattr(owner.runner,'is_inference',False) and clock.index%4==0:
                try:
                    from robot_workbench.physx_task import inference_tray_support
                    support_sample=inference_tray_support(env._m2_runtime,backend)
                except Exception as error:
                    support_sample=dict(error=str(error),evidence_source='physx_support_unavailable')
            if current and not backend.is_connected:
                current.close();device.closed.set()
                device.runtime=None;device.credentials={}
                backend.is_connected=True;backend.owner=None
                backend.active=deepcopy(backend.actual);backend.applied_sequences=dict(left=None,right=None)
            if clock.index % 5==0 and env._sim_step_counter % env.cfg.sim.render_interval==0:
                producer.collect(plant,backend,actor())
            if time.monotonic()-last_status>=2:
                save(args.output/'runtime-status.json',dict(pid=os.getpid(),clock=source_clock,
                    source_time_ns=backend.physics_step*1_000_000_000//240,physics_step=backend.physics_step,
                    cameras=producer.snapshot(),owner=actor(),contact=plant.contact_status,
                    evidence_logging_healthy=not logging_failed.is_set()))
                last_status=time.monotonic()
            if preview_service and clock.index%120==0:
                from arx_env.joint_head_jaw_mapping import feedback_gaps
                feedback=backend.read_calibration_state();q=feedback['qpos']
                preview_service.mirror_feedback(dict(q=q[:6]+q[7:13],jaw_gap_mm=feedback_gaps(q),
                    source_id=backend.source_clock_id,sequence=backend.physics_step,
                    source_age_s=max(0.,(time.time_ns()-feedback['sample_wall_ns'])/1e9)))
        save(args.output/'final.json',dict(status='bounded-service-ended',clock=source_clock,
            physics_step=backend.physics_step,cameras=producer.snapshot(),full_pipeline_qualified=False,
            physical_hardware_connected=False,teardown_verified=False))
    except BaseException as error:
        save(args.output/'final.json',dict(status='failed',error=str(error),traceback=traceback.format_exc(),
                                         full_pipeline_qualified=False,teardown_verified=False))
        raise
    finally:
        from isaac_lane import close_runtime
        done.set()
        if ui:ui.shutdown();ui.server_close()
        if device and device.runtime:
            try:
                runtime=device.runtime
                if runtime.release_receipt and not runtime.release_receipt.get('sdk_close_returned'):
                    raise RuntimeError('previous SDK release is unconfirmed; teardown will not retry it')
                runtime.owner.submit(dict(action='stop'));runtime.tick()
                if task_observer:task_observer.after_advance(runtime.owner,physics_advanced=False)
                runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'));runtime.tick()
                runtime.close();device.closed.set()
            except Exception as error:event('teardown_error',error=str(error))
        if collection_preparation:
            try:collection_preparation.close(lifecycle_admission=lambda action: action.startswith('stop:'))
            except Exception as error:event('teardown_error',error=str(error))
        if preview_service:
            try:preview_service.close()
            except Exception as error:event('teardown_error',error=str(error))
        if device:
            for workflow in list(device.workflows.values()):
                try:workflow.cancel()
                except Exception as error:event('teardown_error',error=str(error))
                if callable(getattr(workflow,'close',None)):workflow.close()
                elif hasattr(workflow,'closed'):workflow.closed.set()
            for media in device.media.values():media.closed.set()
        if gateway:gateway.shutdown();gateway.server_close()
        if model_process and model_process.poll() is None:
            model_process.terminate()
            try:model_process.wait(timeout=30)
            except subprocess.TimeoutExpired:event('teardown_error',error='owned model service did not exit; retain process identity for cleanup')
        if producer:producer.close()
        writer_done.set();writer.join(timeout=5)
        close_runtime(env,app)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,required=True)
    parser.add_argument('--nominal-config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gpu',type=int,required=True)
    parser.add_argument('--port',type=int,default=0)
    parser.add_argument('--gateway-port',type=int,default=0)
    parser.add_argument('--camera-ports',type=int,nargs=3,default=[0,0,0])
    lifetime=parser.add_mutually_exclusive_group()
    lifetime.add_argument('--duration-wall-s',type=float)
    lifetime.add_argument('--persistent',action='store_true',help='explicit persistent resource contract; no automatic service exit')
    parser.add_argument('--calibration-auto-confirm',action='store_true')
    parser.add_argument('--collection-contract',type=Path,
        help='explicit current resident GPU/resource contract; enables post-install background D preparation only')
    parser.add_argument('--calibration-resume-source',type=Path,
        help='pinned dev_resume source; independent editable calibration copy, never motion/task resume')
    parser.add_argument('--collection-history-source',type=Path,
        help='pinned completed fake recordings; read-only history, never execution or checkpoint resume')
    parser.add_argument('--collection-history-catalog',type=Path,action='append',default=[],
        help='retained local read-only history catalog; does not depend on original raw source')
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--deadline-wall-ns',type=int,help='optional absolute cap from the bounded fake manager')
    args=parser.parse_args();args.output=args.output.resolve()
    if args.persistent:
        if args.deadline_wall_ns is not None or not args.collection_contract:
            raise ValueError('persistent service requires a resource contract and no deadline')
    elif args.duration_wall_s is None:args.duration_wall_s=3600.
    if args.deadline_wall_ns is not None and args.deadline_wall_ns<=time.time_ns():
        raise ValueError('authorized fake session deadline expired')
    if (not args.persistent and not 0<args.duration_wall_s<=86400) or any(not 0<=p<=65535 for p in [args.port,args.gateway_port,*args.camera_ports]):
        raise ValueError('bounded duration and valid loopback ports required')
    if args.output.exists():raise FileExistsError(args.output)
    args.collection_spec=None
    args.diagnostic_source=None
    args.resume_source=None
    args.history_source=None;args.collection_history=None
    if args.collection_contract:
        from robot_workbench.resident_collection import load_collection_contract
        args.collection_contract=args.collection_contract.resolve()
        args.collection_spec=load_collection_contract(args.collection_contract,args.output,args.gpu,args.duration_wall_s)
        if (args.collection_spec.get('resources',{}).get('persistent_service') is True)!=args.persistent:
            raise ValueError('CLI lifetime differs from collection contract')
    if args.calibration_resume_source and (args.collection_spec or {}).get('diagnostic_calibration_source') is not None:
        raise ValueError('dev_resume and warm_diagnostic are mutually exclusive')
    from robot_workbench.service import loopback_browser_hosts
    args.browser_ports=list((args.collection_spec or {}).get('browser_ports',[]))
    args.camera_spool_root=Path((args.collection_spec or {}).get('camera_spool_root',args.output/'camera-spool')).resolve()
    browser_hosts=loopback_browser_hosts(args.browser_ports)
    base=json.loads(args.base.read_text())
    args.plant_joint_offsets_rad=plant_joint_offsets(base)
    nominal=nominal_inputs(args.nominal_config)
    binding=base['lanes']['source']['configuration']['cameras']['calibration_binding']
    if (nominal['asset_seal_sha256']!=binding['a1_sha256'] or
            nominal['asset_geometry_id']!=binding['geometry_id']):
        raise ValueError('nominal estimator assets differ from this frozen plant A1 identity')
    checked=preflight(base)
    checked['plant_joint_offsets_rad']=list(args.plant_joint_offsets_rad)
    checked.update(browser_ports=args.browser_ports,browser_hosts=sorted(browser_hosts),
        camera_spool_root=str(args.camera_spool_root),
        camera_spool_retention='same-process bounded memory RGB-D; committed segments persist, no per-frame disk spool')
    if args.collection_spec:
        retained_inference=(args.collection_spec.get('maintenance_restore') or {}).get('inference_source')
        if retained_inference is not None:
            validate_inference_source(retained_inference)
            checked['retained_inference']=dict(root=retained_inference['root'],files=len(retained_inference['files']),
                scope='historical settings/playback only; no new execution')
        if args.collection_spec.get('inference_setup'):
            inference_options(args.collection_spec['inference_setup'])  # Validate before importing Isaac or model runtimes.
        from detector_adapter.preflight_release import preflight_release
        checked['detector_release_preflight']=preflight_release(base['pipeline_config'],
            args.collection_spec['collection_setup']['detection_runtime_root'])
        diagnostic=args.collection_spec.get('diagnostic_calibration_source')
        if diagnostic is not None:
            from robot_workbench.calibration_profile import validate_diagnostic_source,diagnostic_metadata
            args.diagnostic_source=validate_diagnostic_source(diagnostic,base,nominal,
                printed_marker_fixture(json.loads(Path(nominal['candidate']).read_bytes())),args.output)
            checked.update(diagnostic_calibration=diagnostic_metadata(args.diagnostic_source),fresh_C_completed=False)
    if args.calibration_resume_source:
        from robot_workbench.calibration_profile import validate_resume_source
        args.resume_source=validate_resume_source(json.loads(args.calibration_resume_source.read_bytes()),base,nominal,
            printed_marker_fixture(json.loads(Path(nominal['candidate']).read_bytes())),args.output)
        checked.update(calibration_resume=dict(mode='dev_resume',source_run=args.resume_source['source_run'],
            robot_id=args.resume_source['origin_robot_id'],expected_install=args.resume_source['expected_install']),
            fresh_C_completed=False,empty_store_qualification=False)
    if args.collection_history_source:
        from robot_workbench.collection_history import validate_source
        robot_id=args.resume_source['origin_robot_id'] if args.resume_source else 'fake-physx-'+args.output.name
        args.history_source=validate_source(json.loads(args.collection_history_source.read_bytes()),robot_id=robot_id)
        checked['collection_history']=dict(source_run=args.history_source['spec']['source_run'],
            jobs=args.history_source['spec']['jobs'],videos=len(args.history_source['entries']),readonly=True)
    retained=[]
    if args.collection_history_catalog:
        from robot_workbench.collection_history import load_history
        robot_id=args.resume_source['origin_robot_id'] if args.resume_source else 'fake-physx-'+args.output.name
        for path in args.collection_history_catalog:
            if not path.is_absolute() or path.resolve()!=path or path.is_symlink():
                raise ValueError('retained history requires a canonical absolute catalog')
            history=load_history(path,environment='fake',robot_id=robot_id)
            if any(row['details']['robot_id']!=robot_id for row in history.results()):
                raise ValueError('retained history belongs to another virtual robot')
            retained.append(history)
        checked['retained_history_catalogs']=[str(path) for path in args.collection_history_catalog]
    if args.dry_run:
        print(json.dumps(dict(status='dry-run-passed',**checked)));return
    args.output.mkdir(parents=True)
    retained_inference=((args.collection_spec or {}).get('maintenance_restore') or {}).get('inference_source')
    if retained_inference is not None:restore_inference(retained_inference,args.output/'inference')
    if args.history_source is not None:
        from robot_workbench.collection_history import import_history
        args.collection_history=import_history(args.history_source,args.output/'collection-history')
    if retained:
        from robot_workbench.collection_history import CollectionHistoryGroup
        args.collection_history=CollectionHistoryGroup(retained+([args.collection_history] if args.collection_history else []))
    started_wall_ns=time.time_ns();args.resident_deadline_wall_ns=None if args.persistent else started_wall_ns+round(args.duration_wall_s*1e9)
    if args.deadline_wall_ns is not None:
        if args.deadline_wall_ns<=started_wall_ns:raise ValueError('authorized fake session deadline expired')
        args.resident_deadline_wall_ns=min(args.resident_deadline_wall_ns,args.deadline_wall_ns)
    save(args.output/'run.json',dict(pid=os.getpid(),started_wall_ns=started_wall_ns,duration_wall_s=args.duration_wall_s,
        resident_deadline_wall_ns=args.resident_deadline_wall_ns,persistent_service=args.persistent,
        base_sha256=hashlib.sha256(args.base.read_bytes()).hexdigest(),
        nominal_config_sha256=hashlib.sha256(args.nominal_config.read_bytes()).hexdigest(),**checked))
    run(args,base,nominal)


if __name__=='__main__':main()
