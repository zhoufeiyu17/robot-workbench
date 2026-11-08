"""Large CPU plan compilation/Owner races. Synthetic evidence is not robot qualification."""
from copy import deepcopy
from http.client import HTTPConnection
import json
import gc
import os
from pathlib import Path
import sys
import threading
import time
import unittest
import weakref
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench')]
import test_unified_runtime as runtime_tests
from robot_workbench import control_plan
from robot_workbench.bridges import start_control_gateway
from robot_workbench.service import Device
from integration import _digest,SIM_PLANNER_ID
from joint_space import contract



class ControlPlanTests(unittest.TestCase):
    def setUp(self):
        self.harness=runtime_tests.UnifiedRuntimeTest();self.harness.setUp();self.addCleanup(self.harness.doCleanups)
        self.runtime,self.creds,self.command=self.harness.collection_case('isolated-control')
        self.directory=Path(self.runtime.owner.hardware_binding).parent
    def large(self,count=6000):
        command=deepcopy(self.command);plan=command['plan'];source=command['source']
        originals=plan['ticks'];actions=plan['native_actions']
        choices=[0]*(count-2)+[1,2]
        plan.update(planner_id=SIM_PLANNER_ID,
            ticks=[dict(tick=i,time_s=i/30,target=deepcopy(originals[index]['target'])) for i,index in enumerate(choices)],
            native_actions=[deepcopy(actions[index]) for index in choices],native_substeps=[8]*count,duration_s=count/30,
            stages=[dict(name='initial_hold',start_tick=0),dict(name='close_and_hold',start_tick=count-2),dict(name='home_hold',start_tick=count-1)])
        plan.pop('physical_gpu_conversion',None)
        plan['timeline_digest']=_digest(plan['ticks']);plan['joint_space']=contract(source['calibration_profile'],plan['native_actions'])
        plan['planning_validation']=dict(schema_version='geometric_path_validation.v1',passed=True,
            timeline_digest=plan['timeline_digest'],source_digest=_digest(source),max_sample_step_rad=.01,
            collision_coverage=['base_link','left','right'])
        return command
    def start_ticks(self):
        done=threading.Event();times=[];errors=[]
        def run():
            while not done.is_set():
                start=time.monotonic();times.append(start)
                try:
                    self.harness.sim+=.01;self.runtime.tick()
                except Exception as error:errors.append(error);return
                done.wait(max(0,.01-(time.monotonic()-start)))
        thread=threading.Thread(target=run,daemon=True);thread.start()
        self.addCleanup(lambda:(done.set(),thread.join(2)))
        return done,thread,times,errors

    def test_large_http_preview_and_initialization_never_decode_full_plan_in_owner_process(self):
        command=self.large();raw=json.dumps(command).encode();self.assertGreater(len(raw),1024*1024)
        target=deepcopy(command['plan']['ticks'][5000]['target']);del command;gc.collect()
        device=Device('isolated-control','fake',None);device.runtime=self.runtime;device.credentials=self.creds
        server,server_thread=start_control_gateway(device,{'calibration':'c'*48,'collection':'d'*48})
        self.addCleanup(lambda:(server.shutdown(),server.server_close(),server_thread.join(2)))
        done,ticker,times,errors=self.start_ticks();before=list(self.runtime.owner.backend.events)
        original=json.loads
        def bounded(value,*args,**kwargs):
            if isinstance(value,(bytes,str)) and len(value)>control_plan.MAX_METADATA:
                raise AssertionError('Owner process decoded a full plan')
            return original(value,*args,**kwargs)
        connection=HTTPConnection('127.0.0.1',server.server_port,timeout=20)
        try:
            with patch.object(json,'loads',new=bounded):
                connection.request('POST','/operator',raw,headers={'Authorization':'Bearer '+'d'*48,'Content-Type':'application/json'})
                response=connection.getresponse();response_raw=response.read()
                self.assertEqual(response.status,200,response_raw.decode())
                review=json.loads(response_raw);proxy=self.runtime.owner.pending[review['token']][0][0]
                payload,frozen,base=self.runtime.owner.frozen_payloads[proxy.sha256]
                self.assertEqual(set(payload),{'action','plan_ref'});self.assertIsInstance(frozen.ticks,control_plan.Rows)
                self.assertEqual(len(frozen.ticks),6000);self.assertNotIn('native_actions',frozen.plan)
                self.assertIs(base['commands'],frozen.ticks);self.assertEqual(frozen.ticks[5000]['target'],target)
                self.assertEqual(self.runtime.owner.backend.events,before,'preview issued an actuator command')
                # The real Owner/runner and CPU fake backend perform the normal
                # stationary gate and initialization; the first held pose is unchanged.
                self.harness.confirm(self.runtime,self.creds['collection'],review)
                deadline=time.monotonic()+4
                while time.monotonic()<deadline:
                    runner=self.runtime.owner.runner
                    if runner is not None and runner.last_sent_index>=0:break
                    if runner is not None and runner.status=='FAULT':self.fail(runner.reason)
                    time.sleep(.005)
                self.assertIsNotNone(runner);self.assertGreaterEqual(runner.last_sent_index,0)
                self.assertIsInstance(runner.runtime['commands'],control_plan.Rows)
                self.assertIn('execution_timeline_ref',runner.runtime)
                started=time.monotonic();self.runtime.submit(self.creds['collection'],{'action':'stop'})
                self.assertLess(time.monotonic()-started,.1)
                deadline=time.monotonic()+1
                while runner.status!='STOPPED' and time.monotonic()<deadline:time.sleep(.005)
                self.assertEqual(runner.status,'STOPPED')
        finally:connection.close();done.set();ticker.join(2)
        self.assertEqual(errors,[])
        self.assertIs(runner.frozen.guard,self.runtime.owner.control_guard)
        decisions=runner.frozen.snapshot()['guard_decisions']
        self.assertTrue(decisions)
        self.assertEqual({row['policy_hash']for row in decisions},{self.runtime.execution_calibration['control_policy']['policy_hash']})
        self.assertNotEqual(payload['plan_ref']['validation_worker_pid'],os.getpid())
        self.assertEqual(payload['plan_ref']['origin']['runtime_nonce'],self.runtime.nonce)
        self.assertEqual(control_plan.load_artifact(payload['plan_ref']['plan']),json.loads(raw)['plan'])
        actual=control_plan.load_artifact(runner.runtime['execution_timeline_ref'])
        self.assertEqual(len(actual['commands']),len(runner.runtime['commands']))
        self.assertEqual(actual['commands'][-1],runner.runtime['commands'][-1])
        gaps=[b-a for a,b in zip(times,times[1:])]
        self.assertGreater(len(gaps),10);self.assertLess(max(gaps),.1)
        print(json.dumps(dict(control_plan_cpu_check=dict(ticks=6000,input_bytes=len(raw),max_owner_gap_s=max(gaps),
            heartbeat_samples=len(times),worker_pid=payload['plan_ref']['validation_worker_pid'],owner_pid=os.getpid(),
            preview_and_initialization=True,robot_or_gpu_used=False,linux_immutability_qualification=sys.platform.startswith('linux')))))
        self.assertTrue(self.runtime.owner.feedback_healthy)

    def test_direct_python_input_is_bounded_before_json_serialization(self):
        value={'action':'preview_frozen','plan':{'ticks':[None]*513},'source':{}}
        with patch.object(json,'dumps',side_effect=AssertionError('large Python input must be rejected first')):
            with self.assertRaisesRegex(ValueError,'large Python plans'):self.runtime.submit(self.creds['collection'],value)
        self.assertFalse(self.runtime.owner.pending)

    def test_delayed_audit_references_survive_row_release_and_owner_handoff(self):
        import inspect
        review=self.runtime.submit(self.creds['collection'],self.command)
        proxy=self.runtime.owner.pending[review['token']][0][0]
        payload,frozen,base=self.runtime.owner.frozen_payloads[proxy.sha256]
        references=deepcopy(payload['plan_ref']);preview_rows=weakref.ref(frozen.ticks)
        runner_type=inspect.getclosurevars(self.runtime.collection.start_motion).nonlocals['Runner']
        runner=runner_type(self.runtime.owner,proxy,payload,frozen,base)
        with patch.object(self.runtime.owner,'runner',runner):
            timeline=runner.compile_runtime(self.runtime.owner.cache['calibration_feedback']['qpos'])
        execution_ref=deepcopy(timeline['execution_timeline_ref']);execution_rows=weakref.ref(timeline['commands'])
        del runner,timeline,frozen,base,payload
        self.runtime.handoff(self.creds['collection'],'calibration');gc.collect()
        self.assertIsNone(preview_rows());self.assertIsNone(execution_rows())
        self.assertEqual(control_plan.load_artifact(references['plan']),self.command['plan'])
        self.assertEqual(control_plan.load_artifact(references['source']),self.command['source'])
        self.assertEqual(control_plan.load_artifact(references['base_timeline'])['parent_timeline_digest'],references['timeline_digest'])
        self.assertEqual(control_plan.load_artifact(execution_ref)['parent_timeline_digest'],references['timeline_digest'])
        self.assertIsInstance(control_plan.load_artifact(references['binding']),dict)

    def test_invalid_sealed_stage_index_stops_executor_without_command_or_uncaught_tick(self):
        review=self.runtime.submit(self.creds['collection'],self.command)
        proxy=self.runtime.owner.pending[review['token']][0][0]
        _,frozen,base=self.runtime.owner.frozen_payloads[proxy.sha256]
        row=dict(base['commands'][0],stage='valid',substage='invalid')
        fd,metadata=control_plan._rows([row],'a'*32)
        metadata['names']=metadata['names'][:1]
        rows=control_plan.Rows(fd,metadata,'a'*32)
        with self.assertRaisesRegex(ValueError,'stage index'):rows[0]
        sent=[];stopped=[]
        frozen.ticks=rows;frozen.times=control_plan.Column(rows,'times')
        frozen.sink=sent.append;frozen.stop=stopped.append
        result=frozen.start(10.,dict(sample_monotonic_s=10.,state=row['target']))
        self.assertEqual(result['status'],'failed');self.assertIn('stage index',result['reason'])
        self.assertEqual(sent,[]);self.assertEqual(len(stopped),1)
        frozen.tick(10.01,dict(sample_monotonic_s=10.01,state=row['target']))
        self.assertEqual(sent,[]);self.assertEqual(len(stopped),1)

    def test_valid_native_task_preparation_runs_in_cpu_child_and_returns_bounded_policy(self):
        import test_physx_task
        from robot_workbench import physx_task
        from execution_timeline import prepare
        captured=[];original=physx_task.prepare_task
        def inputs(plan,source,fixture):
            captured.append((plan,source,fixture));return original(plan,source,fixture)
        with patch.object(test_physx_task,'prepare_task',side_effect=inputs):test_physx_task.fixture()
        plan,source,fixture=captured[0]
        root=self.directory/'native-task-compiler';root.mkdir()
        context=self.runtime.owner.plan_context(self.runtime.owner.cache['joint_limits'])
        context.update(current_state=plan['ticks'][0]['target'],physical_limits={
            'left_arm_joints':[[-2.,2.]]*6,'right_arm_joints':[[-2.,2.]]*6,'left_gripper':[0.,83.],'right_gripper':[0.,83.]},
            held_arms=[],native_task_fixture=fixture)
        reference=dict(plan=control_plan._save(root,'plan.json',plan),source=control_plan._save(root,'source.json',source),
            base_timeline=control_plan._save(root,'base.json',prepare(plan)),timeline_digest=plan['timeline_digest'],
            origin={key:context[key] for key in ('owner_pid','runtime_nonce','epoch')})
        with patch.object(physx_task,'prepare_isolated_task',side_effect=AssertionError('task decoded in Owner')):
            timeline=control_plan.initialize(reference,context,current=lambda:True)
        item=timeline['native_task'];self.assertIsNone(item['error'],item)
        self.assertNotEqual(item['preparation_worker_pid'],os.getpid())
        self.assertLess(len(json.dumps(item)),64*1024)
        self.assertNotIn('plan',item['prepared']);self.assertNotIn('ticks',item['prepared']['model_plan'])
        self.assertIsInstance(timeline['commands'],control_plan.Rows)

    def test_large_whitespace_or_key_order_cannot_change_operator_action(self):
        raw=b'{'+b' '*70000+b'"action":"stop"}'
        self.assertEqual(control_plan.preview_request(raw),{'action':'stop'})
        value={'source':self.command['source'],'plan':self.command['plan'],'action':'preview_frozen','padding':'x'*70000}
        classified=control_plan.preview_request(json.dumps(value).encode())
        self.assertIsInstance(classified,control_plan.RawPreview)
        with self.assertRaisesRegex(ValueError,'exactly action'):
            self.runtime.submit(self.creds['collection'],classified)

    def test_cpu_interpreter_uses_fixed_module_and_clean_runtime_environment(self):
        original=control_plan.subprocess.Popen;calls=[]
        def spawn(argv,**kwargs):calls.append((argv,kwargs));return original(argv,**kwargs)
        with patch.dict(os.environ,dict(WORKBENCH_CPU_PYTHON=sys.executable,PYTHONPATH='/forbidden/isaac',
                LD_LIBRARY_PATH='/forbidden/isaac',LD_PRELOAD='/forbidden/library',CUDA_VISIBLE_DEVICES='7')),\
                patch.object(control_plan.subprocess,'Popen',side_effect=spawn):
            self.runtime.submit(self.creds['collection'],self.command)
        self.assertEqual(len(calls),1);argv,kwargs=calls[0]
        self.assertEqual(argv[:5],[sys.executable,'-u','-B','-m','robot_workbench.control_plan'])
        env=kwargs['env'];self.assertEqual(env['CUDA_VISIBLE_DEVICES'],'')
        self.assertNotIn('LD_LIBRARY_PATH',env);self.assertNotIn('LD_PRELOAD',env)
        self.assertNotIn('/forbidden',env['PYTHONPATH'])
        self.assertEqual(set(env['PYTHONPATH'].split(os.pathsep)),{str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration/infra')})
        self.assertTrue(all(env[key]=='1' for key in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','NUMEXPR_NUM_THREADS')))

    def test_full_validation_rejects_tampered_geometry_and_artifact_content(self):
        command=self.large(1000);command['plan']['planning_validation']['passed']=False
        with self.assertRaises(ValueError):self.runtime.submit(self.creds['collection'],control_plan.RawPreview(json.dumps(command).encode()))
        self.assertFalse(self.runtime.owner.pending);self.assertFalse(self.runtime.owner.frozen_payloads)
        review=self.runtime.submit(self.creds['collection'],self.command);proxy=self.runtime.owner.pending[review['token']][0][0]
        reference=self.runtime.owner.frozen_payloads[proxy.sha256][0]['plan_ref']['plan']
        path=Path(reference['path']);path.chmod(0o600);path.write_text('{}')
        with self.assertRaisesRegex(ValueError,'SHA256'):control_plan.load_artifact(reference)

    def test_stop_or_handoff_revokes_inflight_compiler_and_old_review(self):
        for action in ('stop','handoff'):
            if action=='handoff':self.harness.advance(self.runtime,42)
            old=self.runtime.submit(self.creds['collection'],self.command)
            old_handles=set(self.runtime.owner.frozen_payloads)
            entered,release=threading.Event(),threading.Event();results=[]
            original=control_plan.subprocess.Popen
            def spawn(*args,**kwargs):
                process=original(*args,**kwargs);entered.set()
                if not release.wait(3):raise AssertionError('test compiler launch barrier timed out')
                return process
            def preview():
                try:results.append(self.runtime.submit(self.creds['collection'],control_plan.RawPreview(json.dumps(self.command).encode())))
                except Exception as error:results.append(error)
            with patch.object(control_plan.subprocess,'Popen',side_effect=spawn):
                thread=threading.Thread(target=preview);thread.start()
                try:
                    self.assertTrue(entered.wait(2));started=time.monotonic()
                    if action=='stop':self.runtime.submit(self.creds['collection'],{'action':'stop'})
                    else:self.runtime.handoff(self.creds['collection'],'calibration')
                    self.assertLess(time.monotonic()-started,.1)
                finally:release.set();thread.join(4)
            self.assertFalse(thread.is_alive());self.assertIsInstance(results[0],ValueError)
            self.assertFalse(self.runtime.owner.pending)
            self.assertLessEqual(set(self.runtime.owner.frozen_payloads),old_handles,'revoked compiler installed a handle')
            with self.assertRaises((ValueError,PermissionError)):
                self.harness.confirm(self.runtime,self.creds['collection'],old)

    def test_invalid_worker_result_and_unsealed_or_wrong_identity_rows_are_rejected(self):
        context=self.runtime.owner.plan_context(self.runtime.owner.cache['joint_limits'])
        message=dict(nonce='a'*32,pid=12345,parent_pid=os.getpid(),metadata=json.dumps(dict(origin=dict(
            owner_pid=context['owner_pid'],runtime_nonce=context['runtime_nonce'],epoch=context['epoch']+1))).encode())
        with self.assertRaisesRegex(ValueError,'another Owner or epoch'):
            control_plan._verified_metadata(message,context,'a'*32,12345)
        self.assertFalse(self.runtime.owner.pending)
        row=dict(time_s=0.,target=self.command['plan']['ticks'][0]['target'],source_tick=0,
            holding=False,unloading=False,stage='initial_hold',substage='initial_hold')
        fd,metadata=control_plan._rows([row],'a'*32)
        try:
            with self.assertRaises(ValueError):control_plan.Rows(os.dup(fd),metadata,'b'*32)
            rows=control_plan.Rows(os.dup(fd),metadata,'a'*32)
            with self.assertRaises(OSError):os.write(rows.fd,b'x')
            with self.assertRaises(TypeError):rows.memory[0]=0
            if sys.platform.startswith('linux'):
                self.assertEqual(control_plan.fcntl.fcntl(fd,control_plan.F_GET_SEALS)&control_plan.SEALS,control_plan.SEALS)
                for length in (os.fstat(fd).st_size-1,os.fstat(fd).st_size+1):
                    with self.assertRaises(OSError):os.ftruncate(fd,length)
            self.assertEqual(rows[0]['target'],row['target'])
        finally:os.close(fd)
        path=self.directory/'writable-array';path.write_bytes(b'x'*256)
        with self.assertRaises((ValueError,OSError)):
            control_plan.Rows(os.open(path,os.O_RDWR),dict(row_count=1,names=['a']),'a'*32)


if __name__=='__main__':unittest.main()
