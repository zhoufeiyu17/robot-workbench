"""CPU /proc and model-status fixtures for one explicitly frozen shared GPU worker."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
import test_local_execution as fixture
from local_execution import PHYSICAL_TARGET,gpu_check,role_spec_digest,validate,validate_shared_model
from robot_workbench.inference_model import BASE566_ID,BASE566_PINS,BASE566_PROMPT,worker_identity


class SharedModelGpuTest(unittest.TestCase):
    process=fixture.LocalExecutionTest.process
    query=fixture.LocalExecutionTest.query
    save=fixture.LocalExecutionTest.save

    def test_gpu_admission_needs_only_stdlib(self):
        result=subprocess.run([sys.executable,'-S','-B',__file__,
            'SharedModelGpuTest.test_valid_worker_is_counted_and_both_service_and_worker_can_restart'],
            capture_output=True,text=True,timeout=20)
        self.assertEqual(result.returncode,0,result.stderr)

    def setUp(self):
        fixture.LocalExecutionTest.setUp(self)
        self.execution.update(PHYSICAL_TARGET)
        plant=self.execution['plant'];plant['argv'][plant['argv'].index('--gpu')+1]='0'
        self.process(101,plant['argv'],mask='0')
        self.script=self.root/'serve_inference_model.py';self.script.write_text('# CPU metadata fixture; never executed\n')
        self.worker=self.root/'base566_worker.py';self.worker.write_bytes((ROOT/'scripts/base566_worker.py').read_bytes())
        self.argv=[sys.executable,'-B',str(self.script),'--port','49379']
        self.receipt=self.root/'model.json'
        self.spec=dict(service_receipt=str(self.receipt),service_spec_sha256='d'*64,
            service_script=str(self.script),service_script_sha256=hashlib.sha256(self.script.read_bytes()).hexdigest(),
            service_argv_sha256=role_spec_digest(self.argv),status_port=49379,worker_script=str(self.worker),
            worker_script_sha256=hashlib.sha256(self.worker.read_bytes()).hexdigest(),model_id=BASE566_ID,
            checkpoint_sha256=BASE566_PINS['checkpoint'],max_memory_mib=16000)
        self.execution['shared_model']=self.spec;self.config['execution']=self.execution
        boot=self.proc/'sys/kernel/random';boot.mkdir(parents=True);(boot/'boot_id').write_text('cpu-test-boot')
        self.bundle=dict(schema='workbench.base566.v1',model_id=BASE566_ID,epoch=7,step=58982,prompt=BASE566_PROMPT,
            action_horizon=60,execution_steps=10,action_hz=30,rtc=False,python=sys.executable,model_source=str(self.root/'model-source'),
            artifacts={key:dict(path=str(self.root/(key+'.artifact')),sha256=digest) for key,digest in BASE566_PINS.items()})
        self.bundle['artifacts']['worker']['path']=str(self.worker)
        self.install(201,202)

    def stat(self,pid,*,parent,start='777',state='S',session=None):
        fields=[state,str(parent),str(pid),str(pid)]+['0']*15+[start]
        (self.proc/str(pid)/'stat').write_text(str(pid)+' (cpu fixture) '+' '.join(fields))

    def install(self,service,worker):
        self.process(service,self.argv,mask='0');self.stat(service,parent=99)
        worker_argv=[sys.executable,'-u','-B',str(self.worker),'--bundle-json',json.dumps(self.bundle)]
        self.process(worker,worker_argv,mask='0');self.stat(worker,parent=service)
        receipt=dict(schema='robot_workbench.managed_service.v1',spec_sha256=self.spec['service_spec_sha256'],phase='running',
            identity=dict(pid=service,start_ticks='777',pgid=service,sid=service,uid=os.getuid(),boot_id='cpu-test-boot',script=str(self.script)))
        self.receipt.write_text(json.dumps(receipt));self.receipt.chmod(0o600)
        self.status=dict(service_pid=service,worker_pid=worker,worker_exit_code=None,status='ready',loaded=True,
                         model_id=BASE566_ID,worker_identity=worker_identity(BASE566_ID,self.bundle))

    def check(self,pids=None,**kwargs):
        return gpu_check(self.config,proc_root=self.proc,query=self.query(
            pids or (101,self.status['worker_pid']),gpu=0,uuid=PHYSICAL_TARGET['gpu_uuid'],**kwargs))

    def test_valid_worker_is_counted_and_both_service_and_worker_can_restart(self):
        validate(self.config)
        with patch('local_execution._model_status',side_effect=lambda port:deepcopy(self.status)):
            result=self.check();self.assertEqual(result['shared_model']['worker_pid'],202)
            self.assertEqual(len(result['processes']),2)
            self.install(301,302)
            self.assertEqual(self.check()['shared_model']['service_pid'],301)
            with self.assertRaisesRegex(ValueError,'unowned process'):self.check((101,202,302))
            with self.assertRaisesRegex(ValueError,'memory budget'):self.check(memory=17000)
        self.assertEqual(self.config['execution']['max_processes'],4)

    def test_same_session_does_not_authorize_wrong_parent_manifest_or_gpu(self):
        with patch('local_execution._model_status',side_effect=lambda port:deepcopy(self.status)):
            self.stat(202,parent=999)
            with self.assertRaisesRegex(ValueError,'direct child'):self.check()
            self.stat(202,parent=201)
            argv=[v.decode() for v in (self.proc/'202/cmdline').read_bytes().split(b'\0') if v]
            self.process(202,argv,mask='1');self.stat(202,parent=201)
            with self.assertRaisesRegex(ValueError,'CUDA mask'):self.check()
            self.process(202,argv,mask='0',session='other-session');self.stat(202,parent=201)
            with self.assertRaisesRegex(ValueError,'another Session'):self.check()
            self.process(202,argv,mask='0');self.stat(202,parent=201)
            invalid=deepcopy(self.bundle);invalid['artifacts']['checkpoint']['sha256']='a'*64
            argv[-1]=json.dumps(invalid);self.process(202,argv,mask='0');self.stat(202,parent=201)
            with self.assertRaisesRegex(ValueError,'manifest differs'):self.check()

    def test_receipt_pid_reuse_http_identity_and_midcheck_worker_changes_are_rejected(self):
        with patch('local_execution._model_status',return_value=self.status):
            self.stat(201,parent=99,start='888')
            with self.assertRaisesRegex(ValueError,'PID/start'):self.check()
            self.stat(201,parent=99)
            value=json.loads(self.receipt.read_text());value['spec_sha256']='f'*64;self.receipt.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,'deployment differs'):self.check()
        self.install(201,202)
        with patch('local_execution._model_status',return_value=dict(self.status,service_pid=999)):
            with self.assertRaisesRegex(ValueError,'HTTP service PID'):self.check()
        calls=[]
        def changed(port):
            calls.append(port)
            if len(calls)==2:self.stat(202,parent=201,start='888')
            return deepcopy(self.status)
        with patch('local_execution._model_status',side_effect=changed):
            with self.assertRaisesRegex(ValueError,'changed during GPU admission'):self.check()

    def test_absent_or_cold_model_does_not_authorize_any_gpu_worker(self):
        self.receipt.unlink()
        with patch('local_execution._model_status') as query:
            with self.assertRaisesRegex(ValueError,'unowned process'):self.check()
            query.assert_not_called()
        self.install(201,202)
        with patch('local_execution._model_status',return_value=dict(self.status,loaded=False,worker_pid=None,status='not_loaded')):
            self.assertNotIn('shared_model',self.check((101,)))
            with self.assertRaisesRegex(ValueError,'unowned process'):self.check()
        with patch('local_execution._model_status',side_effect=ConnectionRefusedError('not yet listening')):
            self.assertNotIn('shared_model',self.check((101,)))
            with self.assertRaisesRegex(ValueError,'unowned process'):self.check()
        for changes in ({'model_id':'base566-epoch005-step42130'},{'status_port':0},{'max_memory_mib':True},
                        {'service_receipt':'relative'},{'worker_script_sha256':'invalid'},{'pid':202}):
            with self.assertRaises(ValueError):validate_shared_model(dict(self.spec,**changes))

    def test_regular_running_state_transitions_do_not_look_like_pid_reuse(self):
        calls=[]
        def schedule(port):
            calls.append(port)
            if len(calls)==2:self.stat(201,parent=99,state='R');self.stat(202,parent=201,state='R')
            return deepcopy(self.status)
        with patch('local_execution._model_status',side_effect=schedule):self.assertEqual(self.check()['shared_model']['worker_pid'],202)


if __name__=='__main__':unittest.main()
