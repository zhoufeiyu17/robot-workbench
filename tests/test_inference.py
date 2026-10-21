"""CPU policy scheduler checks. No model imports, GPU, simulator or hardware."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image, ImageOps

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.inference import InferenceLoop, StableTraySupport, audit_dynamics
from robot_workbench.inference_model import BASE379_PINS, CAMS, PROMPT, FrozenBaseClient, LegacyBaseAdapter, letterbox_rgb224, validate_bundle
from robot_workbench.inference_model import (BASE566_ID, BASE566_PINS, BASE566_PROMPT, ContinuousBaseAdapter,
    model_spec, worker_identity, validate_worker_identity)


def observation(t=0.):
    return dict(source_time=t, device_state14=[0.]*14, images={name:np.zeros((224,224,3),np.uint8) for name in CAMS},
                frame_ids={name:str(t) for name in CAMS}, task_success=True, cup_truth=[1,2,3])


def decode(row, _state):
    return dict(decoded_device14=list(row), final_device14=list(row), projections=[])


def support(t, supported=True):
    return dict(source_time=t, tray_supported=supported, stable=True, evidence_source='fixture_contacts')


def bundle():
    return dict(schema='workbench.base379.v1', epoch=10, step=29450, prompt=PROMPT,
        action_horizon=60, execution_steps=10, action_hz=30, rtc=False,
        artifacts={key:dict(path='/remote/'+key, sha256=value) for key,value in BASE379_PINS.items()},
        python='/remote/launcher', model_source='/remote/source', history_source='/remote/history')


def base566_bundle(model_id=BASE566_ID):
    spec = model_spec(model_id)
    return dict(schema=spec['schema'], model_id=model_id, epoch=spec['epoch'], step=spec['step'], prompt=BASE566_PROMPT,
        action_horizon=60, execution_steps=10, action_hz=30, rtc=False,
        artifacts={key:dict(path='/remote/'+key, sha256=value) for key,value in spec['pins'].items()},
        python='/remote/env/bin/python', model_source='/remote/source')


class InferenceTests(unittest.TestCase):
    def test_base566_continuous_jaw_and_explicit_disabled_right_model_slot(self):
        def to_device(value):
            q=np.asarray(value,dtype=float).copy();q[[6,13]]=q[[6,13]]*1000+.008;return q
        def to_model(value):
            q=np.asarray(value,dtype=float).copy();q[[6,13]]=(q[[6,13]]-.008)/1000;return q
        limits=[[-3.,3.]]*14;limits[6]=limits[13]=[0.,83.]
        adapter=ContinuousBaseAdapter(model_to_device=to_device,device_to_model=to_model,position_limits=limits,
            disabled_right_gripper_model_m=.082992)
        held=np.zeros(14);held[7:13]=.2
        raw=np.zeros(14);raw[6]=.051234;raw[7:]=1.
        result=adapter.decode(raw,held)
        self.assertAlmostEqual(result['final_device14'][6],51.242)
        self.assertAlmostEqual(result['final_device14'][13],83.)
        np.testing.assert_allclose(result['final_device14'][7:13],held[7:13])
        self.assertEqual(adapter.device_to_model(held)[13],.082992)
        self.assertEqual(adapter.device_to_model(result['final_device14'])[13],.082992)
        self.assertEqual(held[13],0.,'disabled raw feedback remains unmodified')
        raw[6]=.12
        self.assertEqual(adapter.decode(raw,held)['final_device14'][6],83.)
        with self.assertRaisesRegex(ValueError,'disabled right'):
            ContinuousBaseAdapter(model_to_device=to_device,device_to_model=to_model,position_limits=limits,
                disabled_right_gripper_model_m=float('nan'))

    def test_base566_family_binds_selected_checkpoint_and_rejects_changed_io(self):
        for name in (BASE566_ID,'base566-epoch005-step42130'):
            value=base566_bundle(name)
            self.assertEqual(validate_bundle(value),value)
            identity=worker_identity(name,value)
            self.assertEqual(identity['vision_variant'],'original_unpatched')
            self.assertFalse(identity['rtc_audit_outputs'])
            validate_worker_identity(identity,name,bundle=value)
            changed=dict(identity,checkpoint_sha256='0'*64)
            with self.assertRaisesRegex(ValueError,'identity'):validate_worker_identity(changed,name,bundle=value)
        value=base566_bundle();value.update(model_id='base566-epoch008-step67408',epoch=8,step=67408)
        value['artifacts']['checkpoint']['sha256']='a'*64;value['artifacts']['metadata']['sha256']='b'*64
        self.assertEqual(validate_bundle(value)['epoch'],8,'same-family trusted catalog needs no epoch code change')
        self.assertEqual(worker_identity(value['model_id'],value)['checkpoint_sha256'],'a'*64)
        for key,changed in (('prompt','another task'),('rtc',True),('model_id','other-epoch008-step67408')):
            invalid=deepcopy(value);invalid[key]=changed
            with self.assertRaises(ValueError):validate_bundle(invalid)
        invalid=deepcopy(value);invalid['artifacts']['stats']['sha256']='c'*64
        with self.assertRaisesRegex(ValueError,'stats'):validate_bundle(invalid)

    def test_base566_worker_is_inert_and_history_padding_is_exact(self):
        import importlib.util, hashlib
        path=Path(__file__).resolve().parents[1]/'scripts/base566_worker.py'
        self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),BASE566_PINS['worker'])
        spec=importlib.util.spec_from_file_location('base566_worker_cpu_test',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        images={key:np.zeros((224,224,3),np.uint8) for key in CAMS}
        history=np.arange(28,dtype=np.float32).reshape(2,14)
        row=module.sample(np.zeros(14),history,images)
        np.testing.assert_array_equal(row['history'][-2:],history)
        self.assertFalse(row['history_mask'][:-2].any());self.assertTrue(row['history_mask'][-2:].all())
        self.assertFalse(row['history'][:-2].any());self.assertEqual(row['prompt'],BASE566_PROMPT)
        with self.assertRaises(ValueError):module.sample(np.zeros(14),np.zeros((61,14)),images)
        with patch('subprocess.Popen',side_effect=AssertionError('constructor must be inert')):
            client=FrozenBaseClient(base566_bundle(),log_path='/not-opened',adapter=None)
            self.assertEqual(client.model_id,BASE566_ID)

    def loop(self, predict=None, **kwargs):
        events=[]
        predict = predict or (lambda request:dict(request_id=request['request_id'],action14=np.ones((60,14))*.01))
        value = InferenceLoop(predict=predict, decode=decode, device_to_model=lambda q:q,
                              event=lambda name,data:events.append((name,data)), **kwargs)
        value.start(epoch=1,source_time=0.)
        return value,events

    def await_result(self, loop):
        deadline=time.monotonic()+2.
        while loop._result.empty() and time.monotonic()<deadline:
            time.sleep(.001)
        self.assertFalse(loop._result.empty(), 'mock worker failed to return')

    def test_completed_only_history_and_no_task_truth_in_model(self):
        requests=[]
        def predict(request):
            requests.append(deepcopy(request))
            return dict(request_id=request['request_id'],action14=np.ones((60,14))*.01)
        loop,events=self.loop(predict)
        loop.tick(0.,epoch=1,observation=observation())
        self.await_result(loop)
        for i in range(10):
            now=i/30
            command=loop.tick(now,epoch=1)
            self.assertIsNotNone(command,loop.snapshot())
            self.assertEqual(len(loop.history),i)
            self.assertIsNone(loop.tick(now+.001,epoch=1),'send must not mark a command completed')
            loop.acknowledge(command['command_id'],applied_device14=[i*.02]*14,source_time=(i+1)/30,
                             feedback={'measured_device14':[.4]*14})
        loop.tick(10/30,epoch=1,observation=observation(10/30))
        self.await_result(loop)
        self.assertEqual(len(requests),2)
        self.assertEqual(requests[0]['applied_history'].shape,(0,14))
        np.testing.assert_allclose(requests[1]['applied_history'][:,0],np.arange(10)*.02)
        self.assertNotIn('task_success',requests[0]['observation'])
        self.assertNotIn('cup_truth',requests[0]['observation'])
        self.assertEqual([row[0] for row in events].count('command_applied'),10)
        self.assertEqual(loop.completed,10)
        loop.stop()

    def test_model_wait_never_blocks_deadline_and_old_result_cannot_resume(self):
        started,finish=threading.Event(),threading.Event()
        def predict(request):
            started.set();finish.wait(2.)
            return dict(request_id=request['request_id'],action14=np.zeros((60,14)))
        loop,events=self.loop(predict)
        loop.tick(0.,epoch=1,observation=observation())
        self.assertTrue(started.wait(1.))
        before=time.monotonic()
        self.assertIsNone(loop.tick(40.,epoch=1))
        self.assertLess(time.monotonic()-before,.1)
        self.assertEqual(loop.status,'completed')
        self.assertIsNone(loop.outcome['success'],'missing real verifier cannot invent task success')
        finish.set();self.await_result(loop)
        loop.tick(40.01,epoch=1)
        self.assertEqual(loop.discarded,1)
        self.assertEqual(loop.submitted,0)

    def test_handoff_revokes_inflight_and_restart_does_not_overlap_model_calls(self):
        finish=threading.Event();calls=[]
        def predict(request):
            calls.append(request);finish.wait(2.)
            return dict(request_id=request['request_id'],action14=np.zeros((60,14)))
        loop,events=self.loop(predict)
        loop.tick(0.,epoch=1,observation=observation())
        loop.tick(.1,epoch=2)
        self.assertEqual(loop.reason,'owner_epoch_changed')
        loop.start(epoch=2,source_time=.1)
        loop.tick(.1,epoch=2,observation=observation(.1))
        self.assertEqual(len(calls),1)
        finish.set();self.await_result(loop)
        loop.tick(.2,epoch=2)
        self.assertEqual(loop.discarded,1)
        self.assertEqual(loop.submitted,0)
        loop.tick(.2,epoch=2,observation=observation(.2))
        self.await_result(loop)
        self.assertEqual(len(calls),2)
        loop.stop()

    def test_malformed_and_wrong_identity_responses_are_rejected(self):
        for bad in ('nan','shape','identity','text'):
            def predict(request):
                values=np.zeros((60,14)) if bad!='shape' else np.zeros((2,14))
                if bad=='nan':values[0,0]=float('nan')
                if bad=='text':values=values.astype(str)
                return dict(request_id='other' if bad=='identity' else request['request_id'],action14=values)
            loop,_=self.loop(predict)
            loop.tick(0.,epoch=1,observation=observation())
            self.await_result(loop)
            self.assertIsNone(loop.tick(.01,epoch=1))
            self.assertEqual(loop.status,'stopped',bad)
            self.assertEqual(loop.submitted,0)

    def test_numeric_limits_are_record_only_and_collision_has_no_callback(self):
        def audit(actions, previous):
            return audit_dynamics(actions,[[0.]*14],position_limits=[[-.001,.001]]*14,
                                  velocity_limits=[.1]*14,acceleration_limits=[.1]*14)
        loop,events=self.loop(audit=audit)
        loop.tick(0.,epoch=1,observation=observation());self.await_result(loop)
        command=loop.tick(.01,epoch=1)
        self.assertIsNotNone(command)
        receipt=next(data for name,data in events if name=='prediction_received')
        self.assertEqual(receipt['audit']['policy'],'record_only')
        self.assertFalse(receipt['audit']['collision_checked'])
        self.assertTrue(receipt['audit']['problems'])
        loop.stop()

    def test_stale_prediction_is_discarded_and_clock_reverse_stops(self):
        loop,events=self.loop(maximum_observation_age_s=.1)
        loop.tick(0.,epoch=1,observation=observation());self.await_result(loop)
        self.assertIsNone(loop.tick(.2,epoch=1))
        self.assertEqual(loop.status,'running')
        self.assertEqual(loop.discarded,1)
        loop.tick(.1,epoch=1)
        self.assertEqual(loop.reason,'source_clock_reversed')

    def test_final_support_must_be_continuous_and_current_not_latched(self):
        judge=StableTraySupport(.2,.11)
        judge.update(support(0.));judge.update(support(.1));judge.update(support(.2))
        self.assertTrue(judge.result(.2)['success'])
        judge.update(support(.3,False))
        self.assertFalse(judge.result(.3)['success'])
        self.assertIsNone(judge.result(.5)['success'])
        judge.update(support(.5));judge.update(support(.5));judge.update(support(.6))
        self.assertFalse(judge.result(.6)['success'])
        judge.update(support(.7))
        self.assertTrue(judge.result(.7)['success'])
        loop,_=self.loop(support=StableTraySupport(.2,.11))
        loop.tick(39.8,epoch=1,support=support(39.8))
        loop.tick(39.9,epoch=1,support=support(39.9))
        loop.tick(40.,epoch=1,support=support(40.))
        self.assertTrue(loop.outcome['success'])

    def test_broken_acceptance_evidence_cannot_interrupt_control_or_claim_success(self):
        loop,_=self.loop(support=StableTraySupport(.2,.11))
        loop.tick(.1,epoch=1,support=dict(source_time=1.,tray_supported=True,stable=True,evidence_source='future'))
        self.assertEqual(loop.status,'running')
        self.assertIn('future',loop.support_error)
        loop.tick(40.,epoch=1)
        self.assertEqual(loop.status,'completed')
        self.assertIsNone(loop.outcome['success'])

    def test_audit_reports_initial_jump_and_second_difference_without_gating(self):
        report=audit_dynamics([[.5]*14,[.5]*14],[[0.]*14,[0.]*14],position_limits=[[-1.,1.]]*14,
                             velocity_limits=[2.]*14,acceleration_limits=[5.]*14)
        self.assertTrue(any(row['kind']=='velocity' and row['step']==0 for row in report['problems']))
        self.assertTrue(any(row['kind']=='acceleration' and row['step']==0 for row in report['problems']))

    def test_legacy_mapping_keeps_raw_prediction_conversion_and_projection_distinct(self):
        def convert(q):
            v=np.asarray(q).copy();v[[6,13]]*=1000;v[0]+=.05;return v
        inverse=lambda q:np.asarray(q)
        limits=[[-1.,1.]]*14
        limits[6]=limits[13]=[0.,83.]
        adapter=LegacyBaseAdapter(model_to_device=convert,device_to_model=inverse,position_limits=limits,
                                  closed_jaw_mm=65.,open_jaw_mm=83.,project_position_limits=True)
        raw=[0.]*14;raw[0]=2.;raw[6]=.044;raw[7]=.8;raw[13]=.088
        state=[0.]*14;state[13]=71.
        result=adapter.decode(raw,state)
        self.assertEqual(result['decoded_device14'][0],2.05)
        self.assertEqual(result['final_device14'][0],1.)
        self.assertEqual(result['decoded_device14'][6],44.)
        self.assertEqual(result['final_device14'][6],83.)
        self.assertEqual(result['final_device14'][7],0.)
        self.assertEqual(result['final_device14'][13],71.)
        self.assertEqual(len(result['projections']),1)
        self.assertEqual(result['conversions']['left_jaw']['legacy_semantic_m'],.088)
        self.assertEqual(raw[0],2.,'raw model output must remain untouched')
        adapter.project=False
        self.assertEqual(adapter.decode(raw,state)['final_device14'][0],2.05)

    def test_frozen_preprocessing_and_bundle_do_not_load_or_start_a_model(self):
        image=np.random.default_rng(7).integers(0,256,(480,640,3),dtype=np.uint8)
        expected=np.asarray(ImageOps.pad(Image.fromarray(image),(224,224),method=Image.Resampling.BILINEAR,color=(0,0,0)))
        np.testing.assert_array_equal(letterbox_rgb224(image),expected)
        self.assertTrue((letterbox_rgb224(image)[:28]==0).all())
        value=bundle()
        self.assertEqual(validate_bundle(value),value)
        broken=deepcopy(value);broken['artifacts']['stats']['sha256']='1'*64
        with self.assertRaisesRegex(ValueError,'stats'):validate_bundle(broken)
        broken=deepcopy(value);broken['rtc']=True
        with self.assertRaisesRegex(ValueError,'manifest'):validate_bundle(broken)
        with patch('subprocess.Popen',side_effect=AssertionError('constructor must be inert')):
            client=FrozenBaseClient(value,log_path='/not-opened',adapter=None)
            with self.assertRaisesRegex(ValueError,'preparation'):client.infer({})

    def test_shared_model_pipe_is_serial_across_old_and_new_episode_loops(self):
        entered,release=threading.Event(),threading.Event()
        active=[0];maximum=[0];results=[]
        def infer(*args,**kwargs):
            active[0]+=1;maximum[0]=max(maximum[0],active[0]);entered.set()
            release.wait(1.);active[0]-=1
            return np.zeros((60,14))
        client=FrozenBaseClient(bundle(),log_path='/not-opened',
            adapter=SimpleNamespace(model_input=lambda _:([0.]*14,{})))
        client.client=SimpleNamespace(infer=infer)
        def run(request_id):
            results.append(client.infer(dict(request_id=request_id,observation={},seed=0,
                applied_history=np.zeros((0,14)))))
        first=threading.Thread(target=run,args=('old',));second=threading.Thread(target=run,args=('new',))
        first.start();self.assertTrue(entered.wait(1.));second.start()
        time.sleep(.01);self.assertEqual(active[0],1)
        release.set();first.join(1.);second.join(1.)
        self.assertEqual(maximum[0],1)
        self.assertEqual([item['request_id'] for item in results],['old','new'])

    def test_frozen_shell_launcher_does_not_require_executable_artifact_mode(self):
        commands=[]
        class Client:
            identity={}
            def __init__(self,command,*args,**kwargs):commands.append(command)
            def close(self):pass
        module=SimpleNamespace(Pi05Process=Client)
        spec=SimpleNamespace(loader=SimpleNamespace(exec_module=lambda value:None))
        with patch('robot_workbench.inference_model.validate_bundle',side_effect=lambda value,**_:value), \
                patch('importlib.util.spec_from_file_location',return_value=spec), \
                patch('importlib.util.module_from_spec',return_value=module):
            client=FrozenBaseClient(bundle(),log_path='/not-opened',adapter=None)
            with self.assertRaisesRegex(ValueError,'worker identity'):client.prepare()
        self.assertEqual(commands[0][:3],['bash',bundle()['python'],bundle()['artifacts']['worker']['path']])

    def test_base566_launcher_uses_selected_manifest_and_checks_ready_identity(self):
        commands=[];value=base566_bundle('base566-epoch005-step42130')
        class Client:
            identity=worker_identity(value['model_id'],value)
            def __init__(self,command,*args,**kwargs):commands.append(command)
            def close(self):pass
        module=SimpleNamespace(Pi05Process=Client)
        spec=SimpleNamespace(loader=SimpleNamespace(exec_module=lambda value:None))
        with patch('robot_workbench.inference_model.validate_bundle',side_effect=lambda value,**_:value), \
                patch('importlib.util.spec_from_file_location',return_value=spec), \
                patch('importlib.util.module_from_spec',return_value=module):
            client=FrozenBaseClient(value,log_path='/not-opened',adapter=None)
            self.assertEqual(client.prepare()['step'],42130)
            self.assertEqual(commands[0][:4],[value['python'],'-u','-B',value['artifacts']['worker']['path']])
            import json
            self.assertEqual(json.loads(commands[0][-1]),value)


if __name__=='__main__':
    unittest.main()
