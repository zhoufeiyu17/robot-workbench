"""CPU contract/lifecycle checks; never launches Isaac or changes a GPU contract."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch
from unittest.mock import Mock

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration/infra')]
from robot_workbench.preview_service import prepare_preview_roles,resident_execution,PreviewService
from local_execution import validate,register_role,_role_locks,TARGET,PHYSICAL_TARGET


class PreviewServiceTest(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name).resolve()
        self.contract=dict(identity={'session_id':'cpu-preview-session','instance':TARGET['instance_id']},
            resources=dict(max_workers=5,gpu_index=1,gpu_uuid=TARGET['gpu_uuid'],cuda_visible_devices=[1]),
            collection_setup=dict(plant_max_memory_mib=12288,min_free_memory_mib=16384),
            preview_setup=dict(port=19011,max_memory_mib=12288,auxiliary_max_memory_mib=16384))
        self.roles=prepare_preview_roles(self.contract,self.root,{'model_family':'ac_one','mesh_rendering_enabled':True})
        self.proc=self.root/'proc';current=self.proc/str(os.getpid());current.mkdir(parents=True)
        self.write_process(current,['python3',str(ROOT/'scripts/serve_physx_device.py'),'--output',str(self.root),'--gpu','1'])
        self.execution=resident_execution(self.contract,self.root,proc_root=self.proc)

    def write_process(self,path,argv):
        (path/'cmdline').write_bytes(b'\0'.join(v.encode() for v in argv)+b'\0')
        (path/'stat').write_text(str(os.getpid())+' (fixture) '+' '.join(['S']+['0']*18+['123']))

    def test_three_camera_inputs_follow_wrist_head_contract(self):
        import numpy as np
        from types import SimpleNamespace
        from robot_workbench.preview_service import calibration_mesh_inputs
        head=dict(id='head-a',joint_offsets_rad={'left':[0.]*6,'right':[0.]*6},camera_T_world_optical=np.eye(4).tolist())
        wrist=dict(id='wrist-a',left={'T_link6_camera':np.eye(4).tolist()},right={'T_link6_camera':np.eye(4).tolist()})
        workbench=SimpleNamespace(calibration_results=lambda:[{'id':'head-a','parameters':head}],wrist_results=lambda:[wrist],
            wrist_overlay_error=lambda row:None,wrist_contract=lambda row:{'head_snapshot':head},
            geometry=SimpleNamespace(frames=lambda q:{side:{'link6':np.eye(4)} for side in ('left','right')}))
        class Frame:
            def __init__(self,name):self.name=name
            def snapshot(self):return {'jpeg':b'jpeg-'+self.name.encode(),'status':{'ready':True,'data_valid':True,'frame_readback':{'qpos':[0.]*14}}}
        names=('base_0','left_wrist_0','right_wrist_0')
        producer=SimpleNamespace(frames={name:Frame(name) for name in names},intrinsics={name:{} for name in names})
        group=calibration_mesh_inputs(workbench,producer,'all','wrist-a')
        self.assertEqual([group[name][4]['id'] for name in ('head','left','right')],['head-a','wrist-a','wrist-a'])
        self.assertEqual(group['right'][4]['head_result_id'],'head-a')
        with self.assertRaises(ValueError):calibration_mesh_inputs(workbench,producer,'all','head-a')
        workbench.calibration_results=lambda:[]
        with self.assertRaises(ValueError):calibration_mesh_inputs(workbench,producer,'all','wrist-a')

    def test_contract_and_launch_freeze_fifth_role_without_starting_it(self):
        validate({'execution':self.execution})
        config=json.loads(Path(self.roles['preview']['config_path']).read_text())
        self.assertTrue(config['display_only']);self.assertFalse(config['mesh_rendering_enabled'])
        service=PreviewService(execution=self.execution,deadline_wall_ns=time.time_ns()+1_000_000_000_000,
            deps='/approved/deps',mesh_inputs=None,validate_pose=None)
        self.assertIsNone(service.process);self.assertEqual(service.state()['status'],'configured')
        argv=service.launch(600,Path(self.roles['preview']['root'])/'frames-one')
        self.assertIn('--display-only',argv);self.assertEqual(argv[argv.index('--render-gpu')+1],'1')
        self.assertEqual(json.loads(argv[argv.index('--execution-json')+1]),self.execution)
        four=deepcopy(self.contract);four['resources']['max_workers']=4
        with self.assertRaises(ValueError):prepare_preview_roles(four,self.root/'four',{})

    def test_persistent_renderer_has_no_expiry_but_keeps_owned_wrapper(self):
        self.contract['resources'].update(persistent_service=True,service_lifetime_seconds=None)
        (self.root/'persistent').mkdir()
        roles=prepare_preview_roles(self.contract,self.root/'persistent',{})
        execution=deepcopy(self.execution);execution['roles']=roles;execution['plant']['argv'].append('--persistent')
        service=PreviewService(execution=execution,deadline_wall_ns=None,deps='/approved/deps',mesh_inputs=None,validate_pose=None)
        argv=service.launch(None,Path(roles['preview']['root'])/'frames-one')
        self.assertIn('owned_process.py', ' '.join(argv));self.assertIn('--persistent',argv)
        self.assertNotIn('--timeout',argv);self.assertEqual(argv[argv.index('--duration-s')+1],'0')
        self.assertTrue(service.state()['recovery']['allowed'])
        service.close();self.assertFalse(service.state()['recovery']['allowed'])

    def test_fake739_resident_and_preview_bind_same_declared_gpu_zero(self):
        self.contract['identity']['instance']=PHYSICAL_TARGET['instance_id']
        self.contract['resources'].update(gpu_index=0,cuda_visible_devices=[0],gpu_uuid=PHYSICAL_TARGET['gpu_uuid'])
        self.write_process(self.proc/str(os.getpid()),
            ['python3',str(ROOT/'scripts/serve_physx_device.py'),'--output',str(self.root),'--gpu','0'])
        execution=resident_execution(self.contract,self.root,proc_root=self.proc)
        self.assertEqual(validate({'execution':execution}),execution)
        self.assertEqual(execution['gpu_uuid'],PHYSICAL_TARGET['gpu_uuid'])
        self.assertEqual(execution['gpu_index'],0);self.assertEqual(execution['vulkan_gpu'],0)
        service=PreviewService(execution=execution,deadline_wall_ns=time.time_ns()+1_000_000_000_000,
            deps='/approved/deps',mesh_inputs=None,validate_pose=None)
        argv=service.launch(600,Path(self.roles['preview']['root'])/'frames739')
        self.assertEqual(argv[argv.index('--render-gpu')+1],'0')
        self.assertEqual(json.loads(argv[argv.index('--execution-json')+1]),execution)

    def test_shared_model_contract_is_preserved_without_freezing_a_pid(self):
        from robot_workbench.inference_model import BASE566_ID,BASE566_PINS
        model=dict(service_receipt='/trusted/model.json',service_spec_sha256='a'*64,
            service_script='/trusted/service.py',service_script_sha256='b'*64,service_argv_sha256='c'*64,
            status_port=49379,worker_script='/trusted/worker.py',worker_script_sha256='d'*64,
            model_id=BASE566_ID,checkpoint_sha256=BASE566_PINS['checkpoint'],max_memory_mib=16000)
        self.contract['shared_model']=model
        execution=resident_execution(self.contract,self.root,proc_root=self.proc)
        self.assertEqual(execution['shared_model'],model);self.assertIsNot(execution['shared_model'],model)
        validate({'execution':execution})
        self.contract['shared_model']['pid']=123
        with self.assertRaisesRegex(ValueError,'ownership fields'):resident_execution(self.contract,self.root,proc_root=self.proc)

    def test_registration_publishes_actual_command_and_rejects_live_slot_reuse(self):
        spec=self.roles['preview'];argv=['python3',spec['script'],'--config',spec['config_path'],
            '--output',spec['root']+'/frames-one','--token-file',spec['token_file'],'--port',str(spec['port']),
            '--render-gpu','1','--duration-s','600','--display-only','--execution-json',json.dumps(self.execution)]
        self.write_process(self.proc/str(os.getpid()),argv)
        count=len(_role_locks)
        try:
            with patch.dict(os.environ,{'CUDA_VISIBLE_DEVICES':'1','ROBOT_WORKBENCH_SESSION_ID':'cpu-preview-session'}):
                update=register_role(self.execution,'preview',spec['script'],instance_id='a'*32,proc_root=self.proc)
                update('ready')
                receipt=json.loads(Path(spec['registration']).read_text())
                self.assertEqual(receipt['argv'],argv);self.assertEqual(receipt['pid'],os.getpid())
                self.assertEqual(receipt['renderer_instance_id'],'a'*32)
                with self.assertRaises(BlockingIOError):register_role(self.execution,'preview',spec['script'],instance_id='b'*32,proc_root=self.proc)
        finally:
            while len(_role_locks)>count:os.close(_role_locks.pop())

    def test_close_reaps_only_its_owned_cpu_wrapper(self):
        service=PreviewService(execution=self.execution,deadline_wall_ns=time.time_ns()+1_000_000_000_000,
            deps='/approved/deps',mesh_inputs=None,validate_pose=None)
        marker=self.root/'child.pid'
        code='import os,pathlib,time;pathlib.Path('+repr(str(marker))+').write_text(str(os.getpid()));time.sleep(60)'
        service.process=subprocess.Popen([sys.executable,str(ROOT/'rule_based_workbench/owned_process.py'),
            '--timeout','60','--',sys.executable,'-c',code])
        unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'])
        try:
            deadline=time.monotonic()+3
            while not marker.exists() and time.monotonic()<deadline:time.sleep(.01)
            self.assertTrue(marker.exists());service.close()
            state=subprocess.run(['ps','-o','stat=','-p',marker.read_text()],capture_output=True,text=True).stdout.strip()
            self.assertTrue(not state or state.startswith('Z'));self.assertIsNone(unrelated.poll())
            self.assertEqual(service.state()['status'],'stopped')
        finally:
            if service.process is not None:service.close()
            unrelated.terminate();unrelated.wait(3)

    def test_startup_publishes_adapter_bound_to_latest_context(self):
        service=PreviewService(execution=self.execution,deadline_wall_ns=time.time_ns()+1_000_000_000_000,
            deps='/approved/deps',mesh_inputs=None,validate_pose=None)
        service.bind_context('old-context');service.requested={'view':dict(yaw=.2,pitch=.4,zoom=1.)}
        entered=threading.Event();release=threading.Event();bound=[];started=[]
        adapter=Mock()
        adapter.bind_context.side_effect=lambda context:bound.append(context)
        def overview(**view):
            entered.set()
            if not release.wait(2):raise TimeoutError('controlled startup was not released')
        adapter.overview.side_effect=overview;adapter.start.side_effect=lambda:started.append(bound[-1])
        def process(command,**kwargs):
            output=command[command.index('--output')+1]
            registration=Path(service.spec['registration']);registration.parent.mkdir(parents=True,exist_ok=True)
            registration.write_text(json.dumps(dict(argv=[output],status='ready',renderer_instance_id='renderer-fixture')))
            return Mock(poll=Mock(return_value=None))
        client=Mock();client.request.return_value=dict(renderer_instance_id='renderer-fixture',capabilities={})
        with patch.object(service,'_stop_owned'),patch('local_execution.gpu_check'),patch('robot_workbench.preview_service.subprocess.Popen',side_effect=process),patch('robot_workbench.preview_service.RendererClient',return_value=client),patch('robot_workbench.preview_service.TargetPreview',return_value=adapter):
            service.start()
            try:
                self.assertTrue(entered.wait(1),service.error);self.assertIsNone(service.adapter)
                service.bind_context('new-context')
            finally:release.set();service.thread.join(2)
        self.assertFalse(service.thread.is_alive());self.assertEqual(service.status,'ready');self.assertIs(service.adapter,adapter)
        self.assertEqual(bound,['new-context']);self.assertEqual(started,['new-context'])
        if service.log:service.log.close()


if __name__=='__main__':unittest.main()
