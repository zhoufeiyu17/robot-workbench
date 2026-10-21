"""Two host directories and injected SSH/transfer; no remote calls, SDK or GPU."""
from copy import deepcopy
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch
import warnings

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration/infra')]
import test_collection_setup as setup_fixture
from test_collection_setup import prepare_assets,read,sha,write
from robot_workbench.collection_setup import (prepare_remote_collection,prepare_remote_collection_process,
    prepare_collection_attachment_process,compute_operation,construct_collection)
from artifact_transport import transfer,validate_endpoint
from run_pipeline import load_config,transport,bounded_command,write_pipeline_config
from playground_pipeline import native_stage
from local_execution import TARGET,PHYSICAL_TARGET


class RemoteCollectionSetupTests(unittest.TestCase):
    def test_remote_failure_exposes_cause_without_ssh_or_encoded_request(self):
        from robot_workbench.collection_setup import remote_compute_call,_process_failure_reason
        def failed(command,**kwargs):
            raise subprocess.CalledProcessError(1,command,stderr=b'Traceback:\nValueError: baseline certificate URDF differs: a1_conservative_refinement.json\n')
        with self.assertRaises(ValueError) as caught:
            remote_compute_call(self.compute,dict(schema='remote_collection_request.v1',operation='inspect',request_id='a'*32),runner=failed)
        self.assertIn('baseline certificate URDF differs',str(caught.exception))
        self.assertNotIn('ssh',str(caught.exception));self.assertNotIn(str(self.key),str(caught.exception))
        self.assertNotIn('private-command',_process_failure_reason(b'subprocess.CalledProcessError: private-command'))

    def setUp(self):
        self.h=setup_fixture.PhysicalCollectionSetupTests();self.h.setUp();self.addCleanup(self.h.doCleanups)
        self.remote=self.h.fixture.root/'compute-host';self.remote.mkdir();(self.remote/'inputs').mkdir()
        execution=deepcopy(self.h.execution);owner=execution['owner'];old=owner['scope']
        owner.update(scope=str(self.remote),pid=912,process_start_ticks='917')
        owner['argv']=[str(self.remote) if item==old else item for item in owner['argv']]
        self.deployment=dict(schema='workbench.compute_deployment.v1',python=sys.executable,workbench_root=str(ROOT),
            run_root=str(self.remote),input_root=str(self.remote/'inputs'),execution=execution,
            base_pipeline_config=self.h.base,contract_template=self.h.contract,detection_runtime_root=self.h.runtime_root,
            input_sha256={str(path):sha(path) for path in (Path(owner['script']),ROOT/'robot_workbench/collection_setup.py',ROOT/'rule_based_workbench/run_pipeline.py')})
        self.path=self.remote/'deployment.json';write(self.path,self.deployment)
        self.key=self.h.root/'private-key-path';self.key.write_text('PRIVATE SSH KEY MUST NOT TRANSFER')
        self.compute=dict(endpoint=dict(host='cpu-configured-734',port=734,key=str(self.key),
            known_hosts=str(self.h.root/'known-hosts'),host_key_alias='[cpu-configured-734]:734'),
            deployment_path=str(self.path),deployment_sha256=sha(self.path))
        self.requests=[];self.checks=[];self.uploads=[];self.current=True;self.revoke_on_reply=False
        self.options={key:self.h.options[key] for key in ('run_root','calibration_config','capture_root','expected_install','registration',
            'profile_template','control_url','token_file','producer_ports','producer_directories','trusted_binding_path','local_recording_root')}
        self.addCleanup(self.cleanup_remote)

    def cleanup_remote(self):
        for file in self.remote.rglob('configured.json'):
            if file.parent.name=='client-package':continue
            value=read(file)
            if value.get('detector_parent'):
                try:Path(value['detector_parent']).rmdir()
                except FileNotFoundError:pass

    def checker(self,config):
        self.checks.append(deepcopy(config))
        self.assertEqual(config['execution']['owner']['pid'],912)
        self.assertEqual(config['execution']['owner']['scope'],str(self.remote))
        return dict(gpu=config['execution']['gpu_index'],gpu_uuid=config['execution']['gpu_uuid'],cpu_injected_preflight_only=True)

    def convert(self,setup):
        output=Path(setup['root'])
        # The minimal CPU URDF has no meshes; file-only transport omits its empty directory.
        (output/'calibration/model/meshes').mkdir(exist_ok=True)
        with warnings.catch_warnings():
            warnings.filterwarnings('ignore',message='Gimbal lock detected.*')
            binding=prepare_assets(output/'calibration/model',setup['pipeline']['assets'],output/'native-assets')
        Path(binding['usd']).write_bytes(b'CPU synthetic USD, NOT a GPU qualification')
        binding.update(conversion_complete=True,usd_sha256=sha(binding['usd']),cpu_fixture_only=True)
        write(output/'native-assets/a1-binding.json',binding)

    def runner(self,argv,*,stdin,**kwargs):
        self.assertEqual(argv[0],'ssh');self.assertIn('StrictHostKeyChecking=yes',argv)
        self.assertIn('ForwardAgent=no',argv);self.assertEqual(argv[-2],self.compute['endpoint']['host'])
        self.assertNotIn(self.h.token.read_text(),str(argv))
        request=json.load(stdin);self.requests.append(request)
        result=compute_operation(self.compute['deployment_path'],self.compute['deployment_sha256'],request,
            checker=self.checker,converter=self.convert)
        if self.revoke_on_reply and request['operation']=='prepare':self.current=False
        return subprocess.CompletedProcess(argv,0,json.dumps(result).encode(),b'')

    def transfer(self,**kwargs):
        self.assertEqual(kwargs['endpoint'],self.compute['endpoint'])
        if kwargs['direction']=='upload':
            self.uploads.append(deepcopy(kwargs))
            folder=Path(kwargs['local_root'])/kwargs['local_directory']
            for name in kwargs['expected']['files']:
                raw=(folder/name).read_bytes()
                self.assertNotIn(self.h.token.read_bytes(),raw)
                self.assertNotIn(self.key.read_bytes(),raw)
        return transfer(**{**kwargs,'endpoint':None})

    def prepare(self,name='remote-collection'):
        return prepare_remote_collection(compute=self.compute,**self.options,output=self.h.root/name,
            still_current=lambda:self.current,runner=self.runner,transfer=self.transfer)

    def cpu_process(self,callback):
        popen=subprocess.Popen
        def child(command,**kwargs):
            if command==['ps','-axo','pgid=,stat=']:return popen(command,**kwargs)
            self.assertEqual(command[3:],[ '-m','robot_workbench.collection_setup','--prepare-worker'])
            return popen([*command[:3],str(Path(__file__).resolve()),'--prepare-worker'],**kwargs)
        with patch('subprocess.Popen',side_effect=child):
            return callback()

    def process_prepare(self,name,*,current=lambda:True):
        return self.cpu_process(lambda:prepare_remote_collection_process(compute=self.compute,**self.options,
            output=self.h.root/name,still_current=current))

    def test_real_cpu_process_keeps_control_interpreter_available_during_heavy_validation(self):
        from robot_workbench.runtime import Runtime
        from test_owner_clock import ClockBackend,Safety,calibration,jaw
        jaw.configure_mapping(None)
        runtime=Runtime(calibration,Safety,lambda:ClockBackend(time.monotonic),device_id='cpu-prepare-owner',
            environment='physical',lock_root=self.h.root/'cpu-owner',
            initialize_backend=lambda backend:backend.set_damping_hold(manual=True))
        samples=[];done=threading.Event();errors=[]
        def heartbeat():
            try:
                while not done.wait(.005):runtime.tick();samples.append(time.monotonic())
            except BaseException as error:errors.append(error)
        thread=threading.Thread(target=heartbeat);thread.start()
        try:result=self.process_prepare('cpu-heavy')
        finally:
            done.set();thread.join(2)
            runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'));runtime.tick();runtime.close()
        self.assertEqual(errors,[])
        self.assertFalse(any(event[0] in ('begin','send') for event in runtime.owner.backend.events))
        evidence=read(self.h.root/'cpu-heavy/cpu-load.json')
        worker=read(self.h.root/'cpu-heavy/preparation-worker.json')
        self.assertNotEqual(worker['worker_pid'],os.getpid())
        self.assertEqual(worker['parent_pid'],os.getpid())
        self.assertEqual(evidence['pid'],worker['worker_pid'])
        self.assertEqual(evidence['cuda'],'');self.assertEqual(evidence['threads'],'1')
        self.assertFalse(evidence['guard_environment']);self.assertFalse(evidence['sdk_loaded'])
        self.assertGreater(evidence['end']-evidence['start'],.1)
        during=[stamp for stamp in samples if evidence['start']<=stamp<=evidence['end']]
        self.assertGreater(len(during),10)
        max_gap=max(b-a for a,b in zip(during,during[1:]))
        self.assertLess(max_gap,.1)
        print('CPU collection isolation: '+json.dumps(dict(worker_cpu_s=evidence['end']-evidence['start'],
            runtime_tick_max_gap_s=max_gap,runtime_tick_samples=len(during),separate_worker=True)))
        self.assertFalse(Path(result['trusted_binding_path']).exists(),'CPU worker cannot publish binding')
        self.assertIsNone(self.h.device.runtime,'CPU preparation never connects hardware')

    def test_attachment_validation_runs_in_cpu_process_then_shared_constructor_uses_small_inputs(self):
        result=self.prepare('cpu-attachment');path=Path(result['setup_path']).parent/'configured.json'
        attachment=self.cpu_process(lambda:prepare_collection_attachment_process(path,self.options['expected_install'],
            still_current=lambda:True))
        evidence=read(self.h.root/'cpu-attachment/cpu-load.json')
        self.assertNotEqual(evidence['pid'],os.getpid())
        write(Path(result['trusted_binding_path']),attachment['installed_binding'])
        with patch('robot_workbench.collection_setup.verify_inputs',side_effect=AssertionError('full validation belongs in child')),\
                patch('run_pipeline.load_config',side_effect=AssertionError('contract parsing belongs in child')):
            parts=construct_collection(path,self.h.device,self.h.application,validated=attachment)
        parts.workflow.closed.set();parts.workflow.thread.join(3);parts.detector.close();parts.planner.close()
        self.assertIsNone(self.h.device.runtime)
        Path(result['collection_config']).write_bytes(Path(result['collection_config']).read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError,'attachment inputs changed'):
            construct_collection(path,self.h.device,self.h.application,validated=attachment)

    def test_attachment_child_still_checks_original_source_pins(self):
        result=self.prepare('attachment-pins');path=Path(result['setup_path']).parent/'configured.json'
        source=self.h.fixture.root/'installed/hardware/active/manifest.json';source.write_bytes(source.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError,'frozen collection setup input changed'):
            prepare_collection_attachment_process(path,self.options['expected_install'],still_current=lambda:True)
        self.assertFalse(Path(result['trusted_binding_path']).exists())

    def test_control_pipeline_validator_uses_private_cpu_ipc_and_preserves_exact_result(self):
        import run_pipeline
        result=self.prepare('pipeline-cpu');path=Path(result['pipeline_config'])
        expected=load_config(path)
        files={str(p) for p in self.h.root.rglob('*')}
        popen=subprocess.Popen;children=[]
        def launch(*args,**kwargs):
            child=popen(*args,**kwargs);children.append(child.pid);return child
        with patch.object(run_pipeline,'_control_config_validation',False),\
                patch.dict(os.environ,{'WORKBENCH_CPU_PYTHON':sys.executable}),patch('subprocess.Popen',side_effect=launch):
            self.assertEqual(load_config(path),expected)
            self.assertEqual(children,[],'standalone CPU calls must not spawn a second interpreter')
            run_pipeline.isolate_control_config_validation()
            actual=load_config(path)
        self.assertEqual(actual,expected)
        self.assertEqual(len(children),1,'the CPU validator must not recursively spawn')
        self.assertNotEqual(children[0],os.getpid())
        self.assertEqual({str(p) for p in self.h.root.rglob('*')},files,'private pipeline reply must not create artifacts')

    def test_cached_path_prefix_matches_pathlib_semantics(self):
        from pathlib import PurePosixPath
        from run_pipeline import below
        paths=['','.', '/', '//', '/a', '/a/b', '/ab', '/a//b/', '/a/./b','/a/../b',
            'a','a/b','../a','a/../b','a b/c','/中文/a',Path('/a/b')]
        for path in paths:
            for root in paths:
                self.assertEqual(below(path,root),PurePosixPath(path)==PurePosixPath(root) or
                    PurePosixPath(root) in PurePosixPath(path).parents,(path,root))

    def test_cpu_process_cancel_reaps_its_group_and_never_installs_late_result(self):
        output=self.h.root/'cpu-cancel';worker=[]
        def current():
            marker=output/'cpu-load-start.json'
            if marker.exists():
                worker.append(read(marker)['pid']);return False
            return True
        with self.assertRaisesRegex(InterruptedError,'not installed'):
            self.process_prepare('cpu-cancel',current=current)
        self.assertTrue(worker)
        from owned_process import group_alive
        self.assertFalse(group_alive(worker[0]))
        self.assertFalse((output/'client/configured.json').exists())
        self.assertFalse(Path(self.options['trusted_binding_path']).exists())

    def test_cpu_process_reply_is_rejected_when_parent_installation_fence_changes(self):
        output=self.h.root/'cpu-late'
        with self.assertRaisesRegex(InterruptedError,'not installed'):
            self.process_prepare('cpu-late',current=lambda:not (output/'preparation-worker.json').exists())
        self.assertFalse(Path(self.options['trusted_binding_path']).exists())

    def test_full_shared_remote_preparation_mirror_and_per_observation_config_transport(self):
        result=self.prepare();pipeline,contract,_=load_config(Path(result['pipeline_config']))
        self.assertEqual([row['operation'] for row in self.requests],['inspect','prepare'])
        self.assertGreaterEqual(len(self.checks),3)
        self.assertEqual(pipeline['compute_transport']['endpoint'],self.compute['endpoint'])
        self.assertTrue(pipeline['contract_path'].startswith(str(self.remote)))
        self.assertTrue(pipeline['compute_transport']['contract_file'].startswith(str(self.h.root)))
        with patch('planner_service.local_read_directories',side_effect=AssertionError('robot must not stat compute-host paths')):
            load_config(Path(result['pipeline_config']))
        bridge=read(result['collection_config'])
        self.assertEqual(bridge['control_url'],self.h.options['control_url'])
        self.assertEqual(bridge['token_file'],str(self.h.token))
        self.assertEqual(bridge['preparation']['acquisition']['transport']['endpoint'],self.compute['endpoint'])
        self.assertFalse(Path(result['trusted_binding_path']).exists(),'remote reply must not install a binding by itself')
        command=transport(pipeline,'dsw-scene','true')
        self.assertEqual(command[0],'ssh');self.assertEqual(command[-2],'cpu-configured-734')
        self.assertNotIn('739',command);self.assertNotIn('741',command)
        stage=native_stage(pipeline,'geometry-inputs',pipeline['remote_run_root']+'/cpu-scene')
        bounded,seconds=bounded_command(stage,100)
        self.assertEqual(seconds,55);wrapper=shlex.split(bounded[-1]);self.assertEqual(shlex.split(wrapper[-1])[:4],['timeout','-k','30','55'])
        self.assertEqual(stage['execution_config']['compute_transport'],pipeline['compute_transport'])
        local=self.h.root/'next-observation'
        path=write_pipeline_config(pipeline,contract,local,transfer=self.transfer)
        next_config,_,_=load_config(path)
        self.assertNotIn('compute_transport',read(Path(next_config['contract_path']).with_name('config.json')))
        load_config(Path(next_config['contract_path']).with_name('config.json'))
        write(Path(result['trusted_binding_path']),read(result['initial_trusted_binding_source']))
        parts=construct_collection(Path(result['setup_path']).parent/'configured.json',self.h.device,self.h.application)
        try:
            self.assertIsNone(parts.detector._process);self.assertIsNone(parts.planner.process);self.assertIsNone(self.h.device.runtime)
        finally:
            parts.workflow.closed.set();parts.workflow.thread.join(3);parts.detector.close();parts.planner.close()

    def test_late_reply_is_retained_but_never_published_or_installed(self):
        self.revoke_on_reply=True
        with self.assertRaisesRegex(InterruptedError,'not installed'):self.prepare('late')
        self.assertTrue((self.h.root/'late/remote-compute-receipt.json').is_file())
        self.assertFalse((self.h.root/'late/client/configured.json').exists())
        self.assertFalse(Path(self.h.options['trusted_binding_path']).exists())

    def test_739_physical_remote_preparation_keeps_both_worker_roles_on_same_frozen_endpoint(self):
        from detector_service import launch_command
        from run_pipeline import isaac_environment
        self.deployment['execution'].update(PHYSICAL_TARGET)
        write(self.path,self.deployment)
        self.compute['deployment_sha256']=sha(self.path)
        self.compute['endpoint'].update(host='cpu-configured-739',port=739,host_key_alias='[cpu-configured-739]:739')
        result=self.prepare('remote739');pipeline,contract,_=load_config(Path(result['pipeline_config']))
        self.assertEqual(pipeline['planner_service']['compute_resource'],PHYSICAL_TARGET)
        self.assertEqual(contract['resources']['lanes'],{PHYSICAL_TARGET['instance_id']:[0]})
        self.assertIn('CUDA_VISIBLE_DEVICES=0',launch_command(pipeline))
        for role in ('simulated-real','dsw-scene'):
            self.assertEqual(transport(pipeline,role,'true')[-2],'cpu-configured-739')
            self.assertEqual(isaac_environment(pipeline,role)['CUDA_VISIBLE_DEVICES'],'0')
        self.assertEqual([r['operation'] for r in self.requests],['inspect','prepare'])
        self.assertTrue(all(row['execution']['gpu_index']==0 for row in self.checks))

    def test_installation_sha_change_rejects_reply_even_if_request_fence_is_still_current(self):
        original=self.runner
        def changed(argv,**kwargs):
            result=original(argv,**kwargs)
            if self.requests[-1]['operation']=='prepare':
                manifest=self.h.fixture.root/'installed/hardware/active/manifest.json'
                manifest.write_bytes(manifest.read_bytes()+b' ')
            return result
        with self.assertRaisesRegex(InterruptedError,'installation/request changed'):
            prepare_remote_collection(compute=self.compute,**self.options,output=self.h.root/'changed-install',
                still_current=lambda:True,runner=changed,transfer=self.transfer)
        self.assertFalse((self.h.root/'changed-install/client/configured.json').exists())
        self.assertFalse(Path(self.h.options['trusted_binding_path']).exists())

    def test_mismatched_deployment_and_unknown_operation_reject_before_compute(self):
        self.compute['deployment_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'deployment'):self.prepare('bad-deployment')
        self.assertEqual(self.checks,[])
        self.compute['deployment_sha256']=sha(self.path)
        with self.assertRaises(ValueError):compute_operation(self.path,sha(self.path),
            dict(schema='remote_collection_request.v1',operation='shell',request_id='a'*32,command='anything'),checker=self.checker)
        self.assertEqual(self.checks,[])
        for value in ({**self.compute['endpoint'],'strict_host_key_checking':False},{**self.compute['endpoint'],'host':'-ProxyCommand=bad'},
                      {**self.compute['endpoint'],'key':'relative-key'}):
            with self.assertRaises(ValueError):validate_endpoint(value)

    def test_shared_camera_spool_requires_exact_explicit_three_children(self):
        spool=self.remote.parent/'common-camera-service';spool.mkdir()
        directories={name:str(spool/name) for name in ('base_0','left_wrist_0','right_wrist_0')}
        for path in directories.values():Path(path).mkdir()
        options={**self.options,'producer_directories':directories}
        with self.assertRaisesRegex(ValueError,'current-run path'):
            prepare_remote_collection(compute=self.compute,**options,output=self.h.root/'undeclared-spool',
                still_current=lambda:True,runner=self.runner,transfer=self.transfer)
        self.assertEqual(self.requests,[])
        mixed={**directories,'left_wrist_0':self.options['producer_directories']['left_wrist_0']}
        with self.assertRaises(ValueError):
            prepare_remote_collection(compute=self.compute,**{**options,'producer_directories':mixed},
                camera_spool_root=spool,output=self.h.root/'mixed-spool',still_current=lambda:True,
                runner=self.runner,transfer=self.transfer)
        self.assertEqual(self.requests,[])
        result=prepare_remote_collection(compute=self.compute,**options,camera_spool_root=spool,
            output=self.h.root/'shared-spool',still_current=lambda:True,runner=self.runner,transfer=self.transfer)
        connection=read(result['collection_config'])
        self.assertEqual({name:row['recording_directory'] for name,row in connection['cameras'].items()},directories)


def cpu_worker():
    """Exercise the production child entry with only network/GPU adapters replaced."""
    import robot_workbench.collection_setup as setup
    import run_pipeline
    original=setup.prepare_remote_collection;load=run_pipeline.load_config
    def runner(argv,*,stdin,**kwargs):
        request=json.load(stdin)
        # The sealed deployment path/SHA are encoded by the existing transport.
        command=shlex.split(argv[-1]);path,digest=command[-2:]
        result=compute_operation(path,digest,request,checker=lambda _:dict(cpu_fixture_only=True),
            converter=lambda value:RemoteCollectionSetupTests.convert(None,value))
        return subprocess.CompletedProcess(argv,0,json.dumps(result).encode(),b'')
    def local_transfer(**kwargs):return transfer(**{**kwargs,'endpoint':None})
    def heavy_load(path):
        if path.parent.name=='client' and not (path.parent.parent/'cpu-load-start.json').exists():
            output=path.parent.parent
            write(output/'cpu-load-start.json',dict(pid=os.getpid()))
            start=time.monotonic()
            json.dumps([0]*6_000_000)
            end=time.monotonic()
            write(output/'cpu-load.json',dict(pid=os.getpid(),start=start,end=end,cuda=os.environ.get('CUDA_VISIBLE_DEVICES'),
                threads=os.environ.get('OPENBLAS_NUM_THREADS'),sdk_loaded='arx_env.backends.arx5_backend' in sys.modules,
                guard_environment=[key for key in os.environ if key.startswith('ROBOT_WORKBENCH_GUARD_')]))
        return load(path)
    with patch.object(setup,'prepare_remote_collection',side_effect=lambda **kw:original(**kw,runner=runner,transfer=local_transfer)),\
            patch.object(run_pipeline,'load_config',side_effect=heavy_load):
        setup._prepare_worker()


if __name__=='__main__':
    if sys.argv[1:]==['--prepare-worker']:cpu_worker()
    else:unittest.main()
