"""CPU-only full configuration wiring with real C store, builders and classes.

Synthetic USD bytes are explicitly not a GPU conversion or physics result.
No worker is started. --a1 is a sealed nominal A1 geometry input.
"""
import argparse
from copy import deepcopy
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
import warnings

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]
from test_fake_calibration_profile import fixture,write
from local_execution import TARGET,PHYSICAL_TARGET
from robot_workbench.collection_setup import prepare_collection_setup,finalize_collection_setup,construct_collection,read,sha,verify_inputs
from robot_workbench.service import Device,Application
from robot_workbench.bridges import start_control_gateway
from a1_isaac_assets import prepare as prepare_assets
from a1_asset_validation import validate_assets
from run_pipeline import load_config,pinned_files,transport
from planner_service import decode_private_spec,startup_config,validate_config as validate_planner_config
from launch_environment import clean_command
from real_acquisition import validate_config as validate_acquisition


def rejects(fn,kind=(ValueError,FileNotFoundError,FileExistsError)):
    try:fn()
    except kind:return
    raise AssertionError('invalid setup accepted')


def camera(serial):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            data=json.dumps({'ready':False,'serial':serial,'cpu_fixture_only':True}).encode()
            self.send_response(200);self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=lambda:server.serve_forever(poll_interval=.01),daemon=True);thread.start()
    return server,thread


def inputs(root):
    dependency=root/'readonly-dependencies';dependency.mkdir()
    directories={key:dependency/key for key in ('native','runtime','deps','curobo','cuda','assets','adapter','old-workbench')}
    for directory in directories.values():directory.mkdir()
    def text(path,content='# CPU-only unexecuted dependency fixture\n'):
        path.parent.mkdir(parents=True,exist_ok=True);path.write_text(content);return str(path)
    native=text(directories['native']/'source/robot_sim/module.py')
    entry=text(directories['runtime']/'scripts/run_online_detection_episode.py')
    node=text(directories['runtime']/'ros2/online_detection_ros/node.py')
    params=text(directories['runtime']/'ros2/online_detection_ros/config/params.yaml','cpu_fixture: true\n')
    dep=text(directories['deps']/'helper.py');curobo=text(directories['curobo']/'python/curobo/model.py')
    old=text(directories['old-workbench']/'solver.py')
    old_plan=text(dependency/'old-plan.json','{"historical":true}')
    detector=text(directories['adapter']/'isaac_rgbd_detection.py')
    text(directories['adapter']/'isaac_detection_service.py')
    suite=text(dependency/'initial-suite.json','{"episodes":[]}')
    envelope=text(dependency/'finger-envelope.json','{"cpu_fixture":true}')
    config=text(dependency/'detector-node.yaml','cpu_fixture: true\n')
    prompt=text(dependency/'prompt.json','{"cpu_fixture":true}')
    rig=dict(camera_proxy_size_m=[.05,.04,.03],device_calibrations={},cameras=[
        dict(device_calibration_ref=name,parent=parent) for name,parent in
        [('base_0','base_link'),('left_wrist_0','left_link6'),('right_wrist_0','right_link16')]])
    write(directories['assets']/'robots/arx_ac_one/v1/d405_rig.json',rig)
    base=dict(planning_backend='motiongen',assets=str(directories['assets']),native_code=str(directories['native']),
        workbench_code=str(directories['old-workbench']),deps=str(directories['deps']),curobo_root=str(directories['curobo']),
        initial_suite=suite,finger_envelope=envelope,finger_envelope_sha256=sha(envelope),detection_python=sys.executable,
        detection_script=detector,detection_node_config=config,detection_cuda_home=str(directories['cuda']),
        opening_mm=82.992,prompt_file=prompt,motion_profile='continuous-target',
        observation_input_root='/old/observation',detection_input_root='/old/detection',planned_input_root=old_plan,
        detector_service={'old':True},planner_service={'old':True},remote_run_root='/old/run',contract_path='/old/contract')
    provenance=dict(code_sha256={path:sha(path) for path in (native,entry,node,dep,curobo,old)},
        input_sha256={path:sha(path) for path in (params,old_plan)},prompt_sha256={prompt:sha(prompt)})
    contract=dict(code={'root':str(directories['old-workbench'])},resources=dict(max_wall_time_minutes=20,
        max_cpu_threads=4,service_lifetime_seconds=1100),provenance=provenance)
    return base,contract,str(directories['runtime']),dict(native=native,entry=entry,node=node,params=params,dep=dep,curobo=curobo,old=old,old_plan=old_plan,prompt=prompt)


def main(a1,roles=4,persistent=False):
    with tempfile.TemporaryDirectory() as temp,tempfile.TemporaryDirectory() as spool_temp:
        root=Path(temp).resolve();capture,active,cconfig,installed,registration=fixture(root,Path(a1).resolve())
        base,contract,runtime_root,external=inputs(root)
        if roles==5:contract['resources'].update(max_wall_time_minutes=245,service_lifetime_seconds=14400)
        if persistent:contract['resources'].update(persistent_service=True,service_lifetime_seconds=None,
            resident_deadline_wall_ns=None,max_wall_time_minutes=120)
        # These startup-only pins deliberately sit inside an otherwise accepted
        # dependency root, so the generic path allowlist alone cannot exclude them.
        admission_root=Path(base['deps'])/'diagnostic-origin';admission_root.mkdir()
        admission=[admission_root/name for name in ('truth-fixture.json','plant-fixture.json','run.json','base.json',
            'nominal-config.json','registration.json','opaque-admission.json')]
        for path in admission:write(path,{'fixture_admission_only':True})
        relative_admission=Path(contract['code']['root'])/'admission-relative.json'
        write(relative_admission,{'fixture_admission_only':True});admission.append(relative_admission)
        actual_calibration=active/'head.json'
        contract['diagnostic_calibration_source']=dict(mode='warm_diagnostic',source_run=str(admission_root),
            source_base_manifest=str(admission_root/'base.json'),expected_install=installed,
            input_sha256={str(path):sha(path) for path in (*admission,actual_calibration)})
        contract['provenance']['input_sha256'].update(contract['diagnostic_calibration_source']['input_sha256'])
        contract['provenance']['input_sha256']['admission-relative.json']=contract['provenance']['input_sha256'].pop(str(relative_admission))
        contract['filesystem']=dict(readonly_roots=[str(path) for path in admission],writable_roots=['/startup-only-root'])
        original_contract=deepcopy(contract)
        token=root/'collection-token';token.write_text('x'*48);token.chmod(0o600)
        device=Device('cpu-fake-device','fake',None);application=Application({'fake':device})
        gateway,gateway_thread=start_control_gateway(device,{'calibration':'y'*48,'collection':'x'*48},port=0)
        servers={role:camera(cconfig['hardware_camera_serials'][role]) for role in ('head','left','right')}
        parts=None;detector_parent=None;memory_producer=None
        try:
            ports={role:value[0].server_port for role,value in servers.items()}
            cconfig.update(hardware_camera_port=ports['head'],hardware_wrist_camera_ports={k:ports[k] for k in ('left','right')})
            write(root/'calibration-config.json',cconfig);write(root/'registration.json',registration)
            recording=root/'collection-raw';recording.mkdir();dirs={}
            for name in ('base_0','left_wrist_0','right_wrist_0'):
                directory=root/('spool-'+name);directory.mkdir();write(directory/'recording-service.json',{'schema':'rgbd_lease.v1'});dirs[name]=str(directory)
            if roles==4:
                # Exercise real producer registration, not a hand-written disk marker.
                from robot_workbench.physx_camera import CameraProducer
                roles_by_name={'base_0':'head','left_wrist_0':'left','right_wrist_0':'right'}
                memory_producer=CameraProducer({name:dict(serial=cconfig['hardware_camera_serials'][roles_by_name[name]],
                    width=640,height=480,fx=400.,fy=401.,cx=320.,cy=240.,coefficients=[0.]*5,
                    distortion_model='inverse_brown_conrady',rectified=False) for name in dirs},root/'memory-cameras',
                    recording_guard=lambda lease:True)
                dirs=memory_producer.directories
            script=ROOT/'scripts/serve_physx_device.py'
            execution=dict(TARGET,session_id='cpu-contract-fixture-only',max_processes=4,min_free_memory_mib=16384,
                plant=dict(session_id='cpu-contract-fixture-only',pid=123,process_start_ticks='456',scope=str(root),
                    script=str(script),script_sha256=sha(script),argv=['python',str(script),'--output',str(root),'--gpu','1'],max_memory_mib=12288))
            if persistent:execution['plant']['argv'].append('--persistent')
            if roles==5:
                from robot_workbench.preview_service import prepare_preview_roles
                preview_contract=dict(resources={'max_workers':5},preview_setup=dict(port=19012,max_memory_mib=12288,auxiliary_max_memory_mib=16384))
                if persistent:preview_contract['resources'].update(contract['resources'])
                execution.update(max_processes=5,roles=prepare_preview_roles(preview_contract,root,cconfig))
            template=ROOT/'configs/collection-profile-template.json'
            options=dict(run_root=root,calibration_config=root/'calibration-config.json',capture_root=capture,
                expected_install=installed,registration=root/'registration.json',profile_template=template,
                base_pipeline_config=base,execution=execution,contract_template=contract,detection_runtime_root=runtime_root,
                control_url='http://127.0.0.1:'+str(gateway.server_port),token_file=token,producer_ports=ports,
                producer_directories=dirs,trusted_binding_path=root/'collection-binding.json',local_recording_root=recording,
                resident_deadline_wall_ns=None if persistent else 3_000_000_000_000_000_000)
            before={str(path):sha(path) for path in (root/'calibration-config.json',active/'manifest.json',template)}
            rejects(lambda:prepare_collection_setup(**{**options,'producer_ports':{**ports,'head':19999}},output=root/'wrong-camera'))
            rejects(lambda:prepare_collection_setup(**{**options,'control_url':'http://10.0.0.1:739'},output=root/'old-service'))
            rejects(lambda:prepare_collection_setup(**{**options,'execution':{**execution,'gpu_index':0}},output=root/'wrong-gpu'))
            bad=deepcopy(installed);bad['manifest_sha256']='0'*64
            rejects(lambda:prepare_collection_setup(**{**options,'expected_install':bad},output=root/'wrong-install'))
            bad=deepcopy(installed);bad['head_id']='older-head'
            rejects(lambda:prepare_collection_setup(**{**options,'expected_install':bad},output=root/'wrong-head'))
            batch=capture/'batch-head/capture/dataset.json';saved_batch=batch.read_bytes()
            changed=read(batch);changed['clock']['source_clock_id']='previous-plant';write(batch,changed)
            rejects(lambda:prepare_collection_setup(**options,output=root/'old-clock'));batch.write_bytes(saved_batch)
            changed_contract=deepcopy(contract);changed_contract['provenance']['code_sha256'][external['native']]='0'*64
            rejects(lambda:prepare_collection_setup(**{**options,'contract_template':changed_contract},output=root/'bad-dependency'))
            fast=Path(spool_temp).resolve();fast_dirs={}
            for name in dirs:
                path=fast/name;path.mkdir();write(path/'recording-service.json',{'schema':'rgbd_lease.v1'});fast_dirs[name]=str(path)
            fast_contract=deepcopy(contract);fast_contract['camera_spool_root']=str(fast)
            rejects(lambda:prepare_collection_setup(**{**options,'producer_directories':fast_dirs},output=root/'undeclared-spool'))
            rejects(lambda:prepare_collection_setup(**{**options,'producer_directories':fast_dirs,'contract_template':fast_contract},output=root/'unwritable-spool'))
            fast_contract['filesystem']['writable_roots'].append(str(fast))
            rejects(lambda:prepare_collection_setup(**{**options,'producer_directories':{**fast_dirs,'base_0':dirs['base_0']},'contract_template':fast_contract},output=root/'mixed-spool'))
            if roles==5:options.update(producer_directories=fast_dirs,contract_template=fast_contract)
            result=prepare_collection_setup(**options,output=root/'collection-new')
            setup=read(result['setup_path']);verify_inputs(setup)
            assert contract==original_contract,'D preparation must not mutate the startup contract'
            assert 'diagnostic_calibration_source' not in setup['contract_template']
            if roles==5:
                assert setup['contract_template']['filesystem']=={'writable_roots':[str(fast)]}
                assert setup['producer_directories']==fast_dirs
            else:assert 'filesystem' not in setup['contract_template']
            assert 'admission-relative.json' not in setup['contract_template']['provenance']['input_sha256']
            assert all(str(path) not in json.dumps(setup['contract_template']) for path in admission)
            assert all(str(path) not in group for group in setup['dependency_provenance'].values() for path in admission)
            assert setup['input_sha256'][str(actual_calibration)]==sha(actual_calibration),'actual builder inputs must be restored'
            assert result['status']=='awaiting_native_asset_conversion' and not result['runtime_allowed']
            assert not setup['conversion']['executed'] and setup['actual_plant_unchanged']
            assert all(key not in setup['pipeline'] for key in ('observation_input_root','detection_input_root','planned_input_root',
                'detector_service','planner_service','remote_run_root','contract_path'))
            assert setup['pipeline']['environment']=='fake' and setup['pipeline']['execution']==execution
            assert setup['conversion']['argv'][setup['conversion']['argv'].index('--gpu')+1]=='1'
            if roles==5:assert json.loads(setup['conversion']['argv'][-1])==execution
            assert setup['conversion']['env']['CUDA_VISIBLE_DEVICES']=='1'
            assert read(root/'collection-new/calibration/profile.json')['source']['kind']=='installed_fake_calibration'
            assert read(template)['source']['calibration_available'] is False
            assert set(setup['excluded_unconsumed_contract_pin_paths'])=={external['old'],external['old_plan']}
            assert all(sha(path)==digest for path,digest in before.items())
            rejects(lambda:finalize_collection_setup(result['setup_path']))
            rejects(lambda:prepare_collection_setup(**options,output=root/'collection-new'))
            output=root/'collection-new';assets=output/'native-assets'
            with warnings.catch_warnings():
                # The identity optical pose in this CPU fixture has a valid but
                # nonunique Euler representation; no physical pose is inferred.
                warnings.filterwarnings('ignore',message='Gimbal lock detected.*',category=UserWarning)
                binding=prepare_assets(output/'calibration/model',base['assets'],assets)
            rejects(lambda:finalize_collection_setup(result['setup_path']))
            # These bytes exercise hashes and schema only, never USD or physics.
            Path(binding['usd']).write_bytes(b'CPU-only synthetic USD bytes; NOT a GPU or PhysX artifact.\n')
            binding.update(conversion_complete=True,usd_sha256=sha(binding['usd']),cpu_fixture_only=True)
            write(assets/'a1-binding.json',binding)
            native=Path(external['native']);saved=native.read_bytes();native.write_text('changed after preparation')
            rejects(lambda:finalize_collection_setup(result['setup_path']));native.write_bytes(saved)
            configured=finalize_collection_setup(result['setup_path']);detector_parent=Path(configured['detector_parent'])
            cfg,frozen,_=load_config(Path(configured['pipeline_config']))
            pins={key:value for group in pinned_files(frozen) for key,value in group.items()}
            assert 'diagnostic_calibration_source' not in frozen
            assert all(str(path) not in pins for path in admission),'truth/base admission cannot enter D input pins'
            assert all(str(path) not in frozen['filesystem']['readonly_roots'] for path in admission)
            assert '/startup-only-root' not in frozen['filesystem']['writable_roots']
            if roles==5:
                assert frozen['camera_spool_root']==str(fast)
                assert str(fast) in frozen['filesystem']['writable_roots']
                saved=Path(result['setup_path']).read_bytes()
                tampered=read(result['setup_path']);tampered['contract_template']['filesystem']['writable_roots']=[]
                write(Path(result['setup_path']),tampered)
                rejects(lambda:finalize_collection_setup(result['setup_path']))
                Path(result['setup_path']).write_bytes(saved)
            for key in ('native','entry','node','params','dep','curobo','prompt'):
                assert pins[external[key]]==sha(external[key]),key
            assert external['old'] not in pins and external['old_plan'] not in pins
            assert frozen['code']['root']==str(ROOT) and frozen['code']['root']!=contract['code']['root']
            assert cfg['execution']==execution and cfg['resident_deadline_wall_ns']==options['resident_deadline_wall_ns']
            assert frozen['resources']['resident_deadline_wall_ns']==cfg['resident_deadline_wall_ns']
            assert frozen['resources']['max_wall_time_minutes']==min(original_contract['resources']['max_wall_time_minutes'],120)
            assert frozen['resources']['service_lifetime_seconds']==original_contract['resources']['service_lifetime_seconds']
            assert cfg['detector_service']['max_lifetime_seconds']==original_contract['resources']['service_lifetime_seconds']
            assert cfg['planner_service']['max_lifetime_seconds']==original_contract['resources']['service_lifetime_seconds']
            if persistent:
                assert cfg['detector_service']['persistent_service'] is True
                assert cfg['planner_service']['persistent_service'] is True
            assert contract==original_contract,'D budget derivation must not mutate the resident startup contract'
            assert not configured['task_or_gpu_qualification']
            assert validate_assets(assets,read(output/'calibration/profile.json'),pins)['profile_id']==binding['profile_id']
            spec=cfg['planner_service'];private=Path(configured['pipeline_config']).parent/'planner-spec.json'
            decoded=decode_private_spec(private.read_bytes(),spec['spec_sha256'])
            assert decoded['session_id']==execution['session_id'] and 'owner_token' not in spec
            # Contract pins can authorize individual files. Worker IPC roots
            # must remain directories; file authority is never widened.
            exact_files=[path for path in frozen['filesystem']['readonly_roots'] if Path(path).is_file()]
            assert exact_files,'fixture must exercise the actual file-root failure'
            detector_roots=cfg['detector_service']['allowed_read_roots']
            assert detector_roots==[str(output/'sealed-observations'),str(output/'jobs')]
            assert all(Path(path).is_dir() and Path(path).resolve()==Path(path) for path in detector_roots)
            planner_reads=[path for path in spec['allowed_read_roots'] if path!=spec['allowed_output_root']]
            assert all(Path(path).is_dir() and Path(path).resolve()==Path(path) for path in planner_reads)
            assert not set(exact_files)&set(spec['allowed_read_roots'])
            assert all(any(Path(path).is_relative_to(Path(root)) for root in frozen['filesystem']['readonly_roots']) for path in planner_reads)
            assert not Path(spec['allowed_output_root']).exists(),'private jobs creation remains in original startup preflight'
            # Share/recovery load the same validated directory-only private scope.
            shared=startup_config(output/'pipeline-template.json',output/'shared',shared_config=configured['pipeline_config'])
            shared_cfg,shared_contract,_=load_config(shared)
            assert shared_cfg['planner_service']==spec
            validate_planner_config(shared_cfg,shared_contract)
            assert len(spec['socket'].encode())<104 and len(cfg['detector_service']['socket'].encode())<104
            assert transport(cfg,'dsw-scene','true')==clean_command(['bash','-c','true'])
            bridge=read(configured['collection_config']);validate_acquisition(bridge['preparation'])
            if roles==5:assert {name:row['recording_directory'] for name,row in bridge['cameras'].items()}==fast_dirs
            assert bridge['preparation']['acquisition']['transport']['endpoint'] is None
            write(root/'collection-binding.json',read(configured['initial_trusted_binding_source']))
            parts=construct_collection(output/'configured.json',device,application)
            state=parts.playground.snapshot()
            assert state['can_execute'] is False and state['environment']['ready'] is False
            assert parts.detector._process is None and parts.planner.process is None
            assert parts.detector.status()['status']=='stopped' and parts.planner.status()['status']=='stopped'
            assert device.runtime is None
            assert all(name not in sys.modules for name in ('isaaclab','pxr','torch'))
            assert all(sha(path)==digest for path,digest in before.items())
        finally:
            if memory_producer:memory_producer.close()
            if parts:
                parts.workflow.closed.set();parts.workflow.thread.join(3)
                parts.detector.close();parts.planner.close()
            for server,thread in servers.values():server.shutdown();server.server_close();thread.join(2)
            gateway.shutdown();gateway.server_close();gateway_thread.join(2)
            if detector_parent:detector_parent.rmdir()
    print('PASS CPU-only complete bootstrap: real C store/profile/asset-overlay builders; explicitly synthetic USD; retained external pins; real asset/load_config/private startup/bridge/Playground/Workflow composition; workers stopped and no GPU qualification')


class PhysicalCollectionSetupTests(unittest.TestCase):
    """Same C/profile/assets/D composition with physical-interface CPU fixtures."""
    def setUp(self):
        from test_device_profile import DeviceProfileTests
        from robot_workbench.rgbd_service import CommonCameraService
        self.fixture=DeviceProfileTests();self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        f=self.fixture;self.root=f.service
        # Empty geometry directory is enough for the real CPU overlay builder;
        # the synthetic URDF/USD never qualifies physics or actual observations.
        (f.root/'a1/meshes').mkdir()
        self.base,self.contract,self.runtime_root,_=inputs(f.root)
        self.contract['resources'].update(persistent_service=True,service_lifetime_seconds=None,
            resident_deadline_wall_ns=None,max_wall_time_minutes=120)
        self.token=self.root/'collection-token';self.token.write_text('x'*48)
        self.device=Device(f.config['robot_id'],'physical',None)
        self.application=Application({'physical':self.device})
        self.gateway,self.gateway_thread=start_control_gateway(self.device,{'calibration':'y'*48,'collection':'x'*48},port=0)
        self.addCleanup(lambda:(self.gateway.shutdown(),self.gateway.server_close(),self.gateway_thread.join(2)))
        self.servers={role:camera(serial) for role,serial in f.config['hardware_camera_serials'].items()}
        self.addCleanup(lambda:[(server.shutdown(),server.server_close(),thread.join(2)) for server,thread in self.servers.values()])
        ports={role:value[0].server_port for role,value in self.servers.items()}
        f.config.update(hardware_camera_port=ports['head'],hardware_wrist_camera_ports={role:ports[role] for role in ('left','right')})
        write(f.path,f.config);write(self.root/'registration.json',f.registration)
        intrinsics={name:dict(serial=f.config['hardware_camera_serials'][role],width=640,height=480,
            fx=400.,fy=401.,cx=320.,cy=240.,coefficients=[0.]*5,distortion_model='inverse_brown_conrady',rectified=False)
            for role,name in {'head':'base_0','left':'left_wrist_0','right':'right_wrist_0'}.items()}
        self.cameras=CommonCameraService.start(intrinsics,self.root/'camera-service','physical');self.addCleanup(self.cameras.close)
        recording=self.root/'collection-raw';recording.mkdir()
        script=ROOT/'scripts/serve_physical_device.py'
        self.execution=dict(TARGET,environment='physical',session_id='cpu-physical-contract-only',max_processes=4,min_free_memory_mib=16384,
            owner=dict(session_id='cpu-physical-contract-only',pid=123,process_start_ticks='456',scope=str(self.root),
                script=str(script),script_sha256=sha(script),argv=['python',str(script),'--output',str(self.root)],
                max_memory_mib=12288,gpu_context_required=False))
        self.options=dict(run_root=self.root,calibration_config=f.path,capture_root=f.capture,expected_install=f.expected,
            registration=self.root/'registration.json',profile_template=ROOT/'configs/collection-profile-template.json',
            base_pipeline_config=self.base,execution=self.execution,contract_template=self.contract,detection_runtime_root=self.runtime_root,
            control_url=f'http://127.0.0.1:{self.gateway.server_port}',token_file=self.token,producer_ports=ports,
            producer_directories=self.cameras.directories,trusted_binding_path=self.root/'collection-binding.json',
            local_recording_root=recording,resident_deadline_wall_ns=None)

    def prepare(self,name='collection'):
        return prepare_collection_setup(**self.options,output=self.root/name)

    def converted(self,result):
        output=Path(result['setup_path']).parent
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore',message='Gimbal lock detected.*',category=UserWarning)
            binding=prepare_assets(output/'calibration/model',self.base['assets'],output/'native-assets')
        Path(binding['usd']).write_bytes(b'CPU-only physical-interface USD fixture; never converted by Isaac.\n')
        binding.update(conversion_complete=True,usd_sha256=sha(binding['usd']),cpu_fixture_only=True)
        write(output/'native-assets/a1-binding.json',binding)
        return output

    def test_same_prepare_finalize_and_business_classes_without_a_plant_or_hardware_start(self):
        original=deepcopy(self.contract);result=self.prepare();setup=read(result['setup_path'])
        self.assertEqual(setup['environment'],'physical');self.assertNotIn('plant',setup['pipeline']['execution'])
        self.assertNotIn('actual_plant_unchanged',setup);self.assertFalse(result['conversion']['executed'])
        self.assertEqual(setup['calibration_clock'],{'environment':'physical','clock_domain':'host_wall','source_clock_id':None})
        profile=read(Path(result['setup_path']).parent/'calibration/profile.json')
        self.assertEqual(profile['source']['kind'],'installed_physical_calibration');self.assertFalse(profile['source']['simulation_only'])
        self.assertEqual(setup['producer_directories'],self.cameras.directories)
        output=self.converted(result);configured=finalize_collection_setup(result['setup_path'])
        self.addCleanup(lambda:Path(configured['detector_parent']).rmdir())
        pipeline,contract,_=load_config(Path(configured['pipeline_config']))
        self.assertEqual(pipeline['execution'],self.execution);self.assertEqual(pipeline['environment'],'physical')
        self.assertEqual(contract['resources']['lanes'],{TARGET['instance_id']:[1]})
        self.assertTrue(pipeline['planner_service']['persistent_service'])
        self.assertNotIn('root@47.116.111.83',str(transport(pipeline,'dsw-scene','true')))
        config=read(configured['collection_config'])
        self.assertEqual(config['preparation']['environment'],'physical');self.assertNotIn('native_task_root',config)
        self.assertFalse(configured['task_or_gpu_qualification']);self.assertNotIn('actual_plant_unchanged',configured)
        write(Path(configured['trusted_binding_path']),read(configured['initial_trusted_binding_source']))
        parts=construct_collection(output/'configured.json',self.device,self.application)
        def close():
            parts.workflow.closed.set();parts.workflow.thread.join(3);parts.detector.close();parts.planner.close()
            if parts.scene:parts.scene.close()
        self.addCleanup(close)
        self.assertIsNone(parts.detector._process);self.assertIsNone(parts.planner.process);self.assertIsNone(self.device.runtime)
        self.assertFalse(parts.playground.snapshot()['can_execute'])
        self.assertEqual(self.contract,original)
        self.assertTrue(all(name not in sys.modules for name in ('isaaclab','pxr','torch')))

    def test_739_physical_conversion_and_worker_identity_use_one_frozen_gpu_zero(self):
        self.execution.update(PHYSICAL_TARGET)
        result=self.prepare('collection739')
        conversion=result['conversion']
        self.assertEqual(conversion['argv'][conversion['argv'].index('--gpu')+1],'0')
        self.assertEqual(conversion['env']['CUDA_VISIBLE_DEVICES'],'0')
        self.converted(result);configured=finalize_collection_setup(result['setup_path'])
        self.addCleanup(lambda:Path(configured['detector_parent']).rmdir())
        pipeline,contract,_=load_config(Path(configured['pipeline_config']))
        self.assertEqual(pipeline['execution'],self.execution)
        self.assertEqual(contract['resources']['lanes'],{PHYSICAL_TARGET['instance_id']:[0]})
        self.assertEqual(pipeline['planner_service']['compute_resource'],PHYSICAL_TARGET)
        self.assertFalse(configured['task_or_gpu_qualification'])

    def test_selected_objects_survive_calibrated_asset_conversion_and_contract_pinning(self):
        from test_scene_assets import selection,seal
        source=Path(self.base['assets'])/'source-bundle-manifest.json'
        source.write_text(json.dumps('fixture source',sort_keys=True,allow_nan=False))
        selected=selection(source)
        selected['perception']['node_config_sha256']=sha(self.base['detection_node_config'])
        seal(selected)
        write(Path(self.base['assets'])/'scene-assets.json',selected)
        self.base['scene_asset_set']=selected
        result=self.prepare('selected-objects')
        output=self.converted(result)
        configured=finalize_collection_setup(result['setup_path'])
        self.addCleanup(lambda:Path(configured['detector_parent']).rmdir())
        pipeline,contract,_=load_config(Path(configured['pipeline_config']))
        self.assertEqual(pipeline['scene_asset_set'],selected)
        self.assertEqual(read(output/'native-assets/scene-assets.json'),selected)
        pins={k:v for group in pinned_files(contract) for k,v in group.items()}
        self.assertIn(str(output/'native-assets/scene-assets.json'),pins)
        self.assertEqual(pins[str(source)],selected['input_sha256'][str(source)])
        source.write_text('changed input')
        with self.assertRaisesRegex(ValueError,'dependency changed'):
            self.prepare('changed-selected-input')

    def test_missing_or_fake_deployment_and_stale_device_provenance_are_rejected(self):
        for name,changes in [('missing',{'execution':None}),('missing-owner',{'execution':{key:value for key,value in self.execution.items() if key!='owner'}}),
            ('diagnostic',{'diagnostic_source':{'mode':'warm_diagnostic'}}),
            ('wrong-camera',{'producer_ports':{**self.options['producer_ports'],'head':19999}})]:
            with self.subTest(name=name),self.assertRaises((ValueError,FileNotFoundError)):
                prepare_collection_setup(**{**self.options,**changes},output=self.root/name)
        wrong=deepcopy(self.execution);wrong.pop('environment');wrong['plant']=wrong.pop('owner');wrong['plant'].pop('gpu_context_required')
        with self.assertRaises(ValueError):prepare_collection_setup(**{**self.options,'execution':wrong},output=self.root/'fake-owner')
        result=self.prepare('tamper')
        path=Path(result['setup_path']);setup=read(path);setup['pipeline']['environment']='fake';write(path,setup)
        with self.assertRaisesRegex(ValueError,'environment'):finalize_collection_setup(path)
        capture=self.fixture.capture/'batch-head/capture/dataset.json';value=read(capture);value['clock']={'environment':'fake','clock_domain':'simulation','source_clock_id':'not-real'};write(capture,value)
        with self.assertRaises(ValueError):self.prepare('wrong-capture-clock')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--a1',type=Path,required=True)
    parser.add_argument('--roles',type=int,choices=(4,5),default=4)
    parser.add_argument('--persistent',action='store_true')
    args=parser.parse_args();main(args.a1,args.roles,args.persistent)
