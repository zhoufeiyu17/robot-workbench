"""CPU contract/lifecycle checks; never launches Isaac or changes a GPU contract."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from robot_workbench.preview_service import prepare_preview_roles,resident_execution,PreviewService
from local_execution import validate,register_role,_role_locks,TARGET


class PreviewServiceTest(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name).resolve()
        self.contract=dict(identity={'session_id':'cpu-preview-session'},resources={'max_workers':5},
            collection_setup=dict(plant_max_memory_mib=12288,min_free_memory_mib=16384),
            preview_setup=dict(port=19011,max_memory_mib=12288,auxiliary_max_memory_mib=16384))
        self.roles=prepare_preview_roles(self.contract,self.root,{'model_family':'ac_one','mesh_rendering_enabled':True})
        self.proc=self.root/'proc';current=self.proc/str(os.getpid());current.mkdir(parents=True)
        self.write_process(current,['python3',str(ROOT/'scripts/serve_physx_device.py'),'--output',str(self.root)])
        self.execution=resident_execution(self.contract,self.root,proc_root=self.proc)

    def write_process(self,path,argv):
        (path/'cmdline').write_bytes(b'\0'.join(v.encode() for v in argv)+b'\0')
        (path/'stat').write_text(str(os.getpid())+' (fixture) '+' '.join(['S']+['0']*18+['123']))

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


if __name__=='__main__':unittest.main()
