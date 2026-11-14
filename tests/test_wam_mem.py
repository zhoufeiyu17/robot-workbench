"""Temporal state lifecycle and existing HTTP/session transport; CPU only."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest.mock import Mock, patch
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench import wam,wam_mem as mem
from robot_workbench.inference_model import model_family,action_contract
from robot_workbench.inference_transport import ModelService,RemoteBaseClient,ModelDomainAdapter,serve_model


def registry():
    rows=[dict(path=role+'.fixture',bytes=1,sha256=hashlib.sha256(role.encode()).hexdigest())for role in mem.ROLES]
    return dict(model_id='wam-mem-fixture-step1',step=1,instruction='fixture instruction',files=rows,
        inference_capabilities={'denoise_steps':{'minimum':1,'maximum':10,'default':2}},
        artifacts={role:row['sha256']for role,row in zip(mem.ROLES,rows)},
        bundle_sha256=hashlib.sha256(json.dumps(sorted(rows,key=lambda r:r['path']),sort_keys=True,separators=(',',':')).encode()).hexdigest())


def request(identity='decision',stamp=100.,epoch=1,session='session'):
    return dict(request_id=identity,seed=7,applied_history=np.empty((0,14),np.float32),
        memory=dict(protocol=mem.MEMORY_PROTOCOL,session_id=session,epoch=epoch,clock_domain='source',start='continue',revision=0),
        memory_context=dict(revision=1,time_s=0.),
        observation=dict(source_time=stamp,state_reference=dict(stream_id='state',source_instance_id='fixture',
            source_clock_id='source',source_time_ns=round(stamp*1e9),sequence=round(stamp*100)),model_state14=np.zeros(14,np.float32),
            images={key:np.zeros((8,10,3),np.uint8)for key in wam.CAMS},
            memory_sampling=dict(schema='workbench.memory-activity.v2',run_id='run',phase='WAIT',
                model_steps=0,manual_steps=0,model_time_s=0.,nodes=[])))


def observed(identity,step,logical_time,stamp,*,kind='MODEL',revision=0,epoch=1):
    value=request(identity,stamp,epoch=epoch)
    value.update(memory_operation='observe',memory_sample=mem.observation_sample(kind,step,logical_time,stamp),
        memory_context=dict(revision=revision,time_s=logical_time))
    value['observation']['frame_ids']={'sample':stamp}
    return value


class Policy:
    def __init__(self):
        self.resets=0;self.times=[];self.observations=[];self.memory=None;self.executed=[];self.first_frame=None;self.acts=0
        self.config=SimpleNamespace(inference=SimpleNamespace(denoising_steps=2))
    def reset(self,*,control_frequency):
        assert control_frequency==30
        self.resets+=1;self.times=[];self.memory=None;self.executed=[];self.pending=None;self.first_frame=None
    def observe(self,rgb,timestamp):
        if self.times and timestamp<=self.times[-1]:
            raise ValueError('New observations must follow the committed memory time')
        self.observations.append((rgb,timestamp));self.pending=(rgb,timestamp)
    def _context(self,rgb,timestamp,instruction,proprio):
        if self.first_frame is None:self.first_frame=rgb['main'].copy()
        value=float(np.asarray(proprio).sum())+sum(float(image.sum())for image in rgb.values())+1
        return SimpleNamespace(memory=(self.memory or 0)*2+value)
    def act(self,instruction,proprio):
        rgb,timestamp=self.pending;self.pending=None;self.acts+=1
        self.memory=self._context(rgb,timestamp,instruction,proprio).memory;self.times.append(timestamp)
        actions=np.full((32,14),self.memory,np.float32)
        return {'actions':actions,'execute':actions[:getattr(getattr(self.config,'data',None),'execute_actions',32)]}


class Worker:
    def __init__(self):
        self.bundle=mem.descriptor(registry());self.model_id=self.bundle['model_id']
        self.policy=Policy();self.memory=mem.PhaseMemory(self.policy,'fixture instruction',lambda seed:None)
    def prepare(self):return mem.worker_identity(self.bundle)
    def infer(self,value):
        value=mem.worker_request(value)
        try:result=self.memory.infer(value)
        except mem.MemoryRequestRejected as error:return dict(request_id=value['request_id'],memory_rejected=str(error))
        if value['warmup']:self.memory.clear()
        return result
    def close(self):pass
    def can_reconfigure(self,bundle):return mem.sampling_change(self.bundle,bundle)
    def reconfigure(self,bundle):
        self.bundle=deepcopy(bundle);self.policy.config.inference.denoising_steps=bundle['inference_options']['denoise_steps']
        return mem.worker_identity(bundle)


class MemoryTests(unittest.TestCase):
    def test_progress_queues_node_before_a_fresh_live_observation(self):
        from robot_workbench.monitored_memory import MonitoredMemorySession
        from robot_workbench.inference_workflow import offer_memory_progress
        client=SimpleNamespace(model_bundle=mem.descriptor(registry()))
        flow=MonitoredMemorySession(client,ModelDomainAdapter(),'source')
        flow.prepare_io.acquire();flow.io.acquire()
        try:
            flow.sampler.bootstrap(request()['observation'])
            activity=dict(schema='workbench.memory-activity.v2',run_id='run',phase='WAIT',model_steps=32,manual_steps=0,
                          model_time_s=32/30.,nodes=[dict(kind='MODEL',step=32,model_time_s=32/30.,target_source_time=101.)])
            offer_memory_progress(flow,activity,1,'run')
            self.assertEqual(len(flow.pending),1)
            self.assertEqual(flow.pending[0]['metadata']['node']['target_source_time'],101.)
            self.assertIsNone(flow.pending[0]['observation'])
        finally:
            flow.pending.clear();flow.prepare_io.release();flow.io.release();flow.close()

    def test_readonly_policy_and_sampler_share_model_and_manual_stride(self):
        for stride in(16,32):
            client=SimpleNamespace(model_bundle={'training_execute_actions':stride},memory_revision=0)
            flow=mem.MemorySession(client,None,'source')
            try:
                policy=mem.sampling_policy(client.model_bundle)
                snapshot=flow.snapshot()
                self.assertEqual({key:snapshot[key]for key in policy},policy)
                self.assertEqual(policy['model_submission_stride'],stride)
                self.assertEqual(policy['manual_submission_stride'],stride)
                self.assertEqual(flow.sampler.strides,{'MODEL':stride,'TEACH':stride})
                self.assertEqual(policy['model_step_time_s'],1/30.)
                self.assertEqual(policy['manual_step_time_s'],.03)
            finally:flow.close()

    def test_manifest_training_execution_length_is_verified(self):
        with TemporaryDirectory() as directory:
            value=registry();value['root']=directory
            path=Path(directory)/'metadata/training-config.json'
            path.parent.mkdir();raw=json.dumps({'data':{'execute_actions':16}}).encode();path.write_bytes(raw)
            value['files'].append(dict(path='metadata/training-config.json',bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()))
            value['bundle_sha256']=hashlib.sha256(json.dumps(sorted(value['files'],key=lambda r:r['path']),sort_keys=True,separators=(',',':')).encode()).hexdigest()
            bundle=mem.descriptor(value)
            self.assertEqual(bundle['training_execute_actions'],16)
            bundle['training_execute_actions']=33
            with self.assertRaisesRegex(ValueError,'execution length'):mem.validate_descriptor(bundle)
            path.write_text('{}')
            with self.assertRaisesRegex(ValueError,'configuration differs'):mem.descriptor(value)

    def test_model_selection_preserves_independent_execution_length(self):
        from robot_workbench.inference_workflow import InferenceWorkflow
        flow=InferenceWorkflow.__new__(InferenceWorkflow)
        flow.device=SimpleNamespace(maintenance=True);flow.selected_model_id='prior';flow.generation=4
        flow.model_bundle={'training_execute_actions':16}
        flow._model_selection=Mock(return_value=('new',flow.model_bundle,'digest'))
        flow._prepare_model=Mock(return_value={'model_ready':True})
        flow.set_execution_steps=Mock(side_effect=AssertionError('model loading cannot change execution length'))
        for steps in (1,16,20,32):
            flow.execute_steps=steps
            result=flow.operations_prepare_model(model_id='new',still_current=lambda:True)
            self.assertTrue(result['model_ready'])
            self.assertEqual(flow.execute_steps,steps)
        flow.set_execution_steps.assert_not_called()
        flow.selected_model_id='new';flow.set_execution_steps.reset_mock()
        flow.operations_prepare_model(model_id='new',still_current=lambda:True)
        flow.set_execution_steps.assert_not_called()

    def test_explicit_commit_retry_frozen_query_and_clear(self):
        policy=Policy();memory=mem.PhaseMemory(policy,'fixture instruction',lambda seed:None)
        with self.assertRaisesRegex(mem.MemoryRequestRejected,'submit an observation'):memory.infer(request())
        initial=observed('initial',0,0.,100.,revision=0)
        result=memory.infer(initial)
        self.assertNotIn('action14',result);self.assertEqual(policy.times,[0.])
        self.assertEqual(policy.memory,1.)
        retry=memory.infer(dict(initial,request_id='retry'))
        self.assertEqual(retry['audit'],result['audit']);self.assertEqual(memory.observations,1)
        later=request('same-observation-later-time',100.)
        later['observation']=deepcopy(initial['observation']);later['memory_context']['time_s']=.1
        np.testing.assert_array_equal(memory.infer(later)['action14'],3.)
        self.assertEqual(policy.times,[0.]);self.assertEqual(policy.memory,1.)
        for index in range(3):
            value=request('query'+str(index),101.+index)
            value['memory_context']['time_s']=index/30.
            answer=memory.infer(value)
            np.testing.assert_array_equal(answer['action14'],3.)
            self.assertEqual(policy.memory,1.);self.assertEqual(policy.times,[0.])
        self.assertEqual(memory.calls,4)
        value=observed('manual',32,.96,104.,kind='TEACH',revision=1,epoch=2)
        value['observation']['model_state14'][0]=2
        answer=memory.infer(value)
        self.assertEqual(policy.memory,5.);self.assertEqual(policy.times,[0.,.96])
        self.assertEqual(answer['audit']['memory']['manual_observations'],1)
        old=request('old-snapshot',100.,epoch=2)
        old['observation']=deepcopy(initial['observation'])
        np.testing.assert_array_equal(memory.infer(old)['action14'],1.)
        invalid=deepcopy(value);invalid['observation']['model_state14'][0]=3
        with self.assertRaisesRegex(mem.MemoryRequestRejected,'different'):memory.infer(invalid)
        with self.assertRaisesRegex(mem.MemoryRequestRejected,'obsolete'):memory.infer(request('old-epoch',100.))
        clear=dict(request_id='clear',memory=dict(value['memory'],epoch=3),memory_operation='clear')
        self.assertTrue(memory.infer(clear)['audit']['memory']['empty'])
        with self.assertRaisesRegex(mem.MemoryRequestRejected,'obsolete'):memory.infer(value)
        restarted=observed('restarted',0,0.,110.,revision=0,epoch=4)
        restarted['memory']['revision']=1
        memory.infer(restarted)
        self.assertEqual(policy.times,[0.]);self.assertEqual(policy.memory,1.)

    def test_full_prediction_and_snapshot_expiry(self):
        for training_steps in (16,32):
            policy=Policy();policy.config.data=SimpleNamespace(execute_actions=training_steps)
            memory=mem.PhaseMemory(policy,'fixture instruction',lambda seed:None)
            for index in range(3):
                memory.infer(observed('node'+str(index),index*training_steps,index*training_steps/30.,100.+index,revision=index))
            self.assertEqual(len(memory.contexts),2)
            with self.assertRaisesRegex(mem.MemoryRequestRejected,'expired'):memory.infer(request())
            value=request('current',103.);value['memory_context']=dict(revision=3,time_s=3.)
            self.assertEqual(memory.infer(value)['action14'].shape,(32,14))
            self.assertEqual(memory.observations,3)

    def test_stationary_new_state_is_distinct_from_a_repeated_sample(self):
        policy=Policy();memory=mem.PhaseMemory(policy,'fixture instruction',lambda seed:None)
        initial=observed('initial',0,0.,100.)
        memory.infer(initial)
        second=observed('stationary',1,1/30.,100.,revision=1)
        second['observation']['state_reference'].update(source_time_ns=100_030_000_000,sequence=10003)
        memory.infer(second)
        self.assertEqual(memory.observations,2)
        duplicate=deepcopy(second);duplicate['request_id']='repeated-sample'
        duplicate['memory_sample']=mem.observation_sample('MODEL',2,2/30.,100.)
        duplicate['memory_context']=dict(revision=2,time_s=2/30.)
        with self.assertRaisesRegex(mem.MemoryRequestRejected,'already committed'):memory.infer(duplicate)
        self.assertEqual(memory.observations,2)

    def test_default_scheduler_is_independent_of_slowdown_and_execution_length(self):
        for stride in (16,32):
            for execute in (1,8,16,20,32):
                expected=None
                for scale in (1.,6.,25.):
                    clock=mem.LogicalMemoryClock();sampler=mem.LogicalNodeSampler(model_stride=stride)
                    value=request()['observation'];value['memory_sampling'].update(clock.snapshot())
                    sampler.bootstrap(value);nodes=[];now=100.
                    for index in range(0,160,execute):
                        count=min(execute,160-index)
                        points=[(step+1)/30.*scale for step in range(count)]
                        clock.start_segment(dict(start_source_s=now,source_point_times_s=points),now)
                        for point in points:
                            clock.advance(now+point)
                            nodes.extend(sampler.offer(dict(schema='workbench.memory-activity.v2',phase='MODEL_EXECUTING',**clock.snapshot())))
                        now+=points[-1]+.2
                        clock.advance(now)
                    signature=[(n['node']['step'],n['node']['model_time_s'])for n in nodes]
                    if expected is None:expected=signature
                    self.assertEqual(signature,expected)
                    self.assertEqual([x[0]for x in signature],list(range(stride,161,stride)))

    def test_manual_default_commits_after_sixteen_real_steps_across_hold(self):
        clock=mem.LogicalMemoryClock();sampler=mem.LogicalNodeSampler(model_stride=16)
        sampler.bootstrap(request()['observation'])
        def offer():
            return sampler.offer(dict(schema='workbench.memory-activity.v2',phase='TEACH',**clock.snapshot()))
        clock.set_mode('TEACH',100.);clock.advance(100.45)
        self.assertEqual(clock.steps['TEACH'],15);self.assertEqual(offer(),[])
        clock.set_mode('HOLD',100.45);clock.advance(200.)
        self.assertEqual(clock.steps['TEACH'],15);self.assertEqual(offer(),[])
        clock.set_mode('TEACH',200.);clock.advance(200.03)
        nodes=offer();self.assertEqual([n['node']['step']for n in nodes],[16])
        self.assertAlmostEqual(nodes[0]['node']['model_time_s'],.48)
        self.assertAlmostEqual(nodes[0]['node']['target_source_time'],200.03)
        clock.advance(200.51)
        self.assertEqual([n['node']['step']for n in offer()],[32])

    def test_manual_30ms_explicit_stride_wait_and_interrupted_prefix(self):
        for manual_stride in (16,32):
            clock=mem.LogicalMemoryClock();sampler=mem.LogicalNodeSampler(model_stride=16,manual_stride=manual_stride)
            value=request()['observation'];sampler.bootstrap(value)
            clock.start_segment(dict(start_source_s=100.,source_point_times_s=[(i+1)*.2 for i in range(32)]),100.)
            clock.set_mode('HOLD',101.05)
            self.assertEqual(clock.steps['MODEL'],5)
            clock.advance(110.);self.assertEqual(clock.steps['MODEL'],5)
            clock.set_mode('TEACH',110.);clock.advance(110.96)
            nodes=sampler.offer(dict(schema='workbench.memory-activity.v2',phase='MODEL_EXECUTING',**clock.snapshot()))
            self.assertEqual([n['node']['step']for n in nodes],list(range(manual_stride,33,manual_stride)))
            self.assertAlmostEqual(nodes[-1]['node']['model_time_s'],5/30.+.96)
            clock.set_mode('HOLD',111.);clock.advance(200.)
            self.assertEqual(clock.steps['TEACH'],33)
            clock.set_mode('TEACH',200.);clock.advance(200.02)
            self.assertEqual(clock.steps['TEACH'],34)
            self.assertEqual(sampler.strides['TEACH'],manual_stride)
            clock.set_mode('HOLD',201.)
            frozen=clock.model_time;clock.advance(300.)
            self.assertEqual(clock.model_time,frozen)

    def test_explicit_target_pairs_real_nearest_samples_and_reports_missing_history(self):
        from robot_workbench.observation_pairing import TimestampPairer,PairingPending
        pairer=TimestampPairer()
        history=[(1_000_000_000+i*10_000_000,{'index':i})for i in range(10)]
        images={name:history for name in wam.CAMS}
        rgb,state,diagnostic=pairer.select(images,history,target_ns=1_025_000_000)
        self.assertEqual(state,{'index':2})
        self.assertTrue(all(row=={'index':2}for row in rgb.values()))
        self.assertEqual(diagnostic['target_wall_ns'],1_025_000_000)
        with self.assertRaises(PairingPending):pairer.select(images,history,target_ns=1_100_000_000)
        with self.assertRaisesRegex(ValueError,'expired'):pairer.select(images,history,target_ns=999_000_000)
        sampler=mem.LogicalNodeSampler()
        value=request()['observation'];sampler.bootstrap(value)
        with self.assertRaisesRegex(ValueError,'missing'):
            sampler.offer(dict(value['memory_sampling'],model_steps=32,model_time_s=32/30.))

    def test_native_adapter_and_source_identity_without_weight_hash(self):
        limits=np.array([[-3,3]]*6+[[0,85]])
        adapter=mem.Adapter(np.tile(limits,(2,1)),85)
        raw=np.array([np.pi/2,0,0,0,0,0,40]*2,dtype=float)
        images=request()['observation']['images']
        state,pixels=adapter.model_input(dict(device_state14=raw,images=images))
        self.assertAlmostEqual(state[0],np.pi/2,places=6);self.assertAlmostEqual(state[6],.04,places=6)
        np.testing.assert_array_equal(pixels[wam.CAMS[0]],images[wam.CAMS[0]])
        bundle=mem.descriptor(registry());bundle['artifacts']['checkpoint']={
            'source_receipt_sha256':'a'*64,'path':'policy/weights.pt','bytes':100_000_000}
        mem.validate_descriptor(bundle)
        self.assertNotIn('sha256',mem.worker_identity(bundle)['checkpoint_identity'])
        self.assertEqual(model_family(bundle['model_id']),'wam-mem')
        self.assertFalse(wam.is_model_id(bundle['model_id']))
        self.assertEqual(action_contract(bundle['model_id'],bundle)['shape'],[32,14])
        bad=deepcopy(bundle);bad['artifacts']['checkpoint']['path']='../outside.pt'
        with self.assertRaises(ValueError):mem.validate_descriptor(bad)

    def test_http_explicit_interfaces_hot_parameters_and_isolated_lifecycle(self):
        from robot_workbench.monitored_memory import MonitoredMemorySession
        worker=Worker();variants=mem.variants(registry())
        service=ModelService(worker,catalog=lambda:{str(n):b for n,b in enumerate(variants)},
            client_factory=lambda _:self.fail('sampling must retain the worker'))
        with TemporaryDirectory()as directory:
            token=Path(directory)/'token';token.write_text('fixture-secret-'+40*'x')
            server=serve_model(service,token_file=token,port=0)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            def client(steps=2):return RemoteBaseClient('http://127.0.0.1:'+str(server.server_port),token_file=token,
                adapter=ModelDomainAdapter(),conversion_identity={'scope':'cpu-only'},model_id=worker.model_id,
                model_bundle=mem.descriptor(registry(),denoise_steps=steps))
            pin=None;flow=None
            try:
                base=client();base.prepare();pin=base.pin('explicit',auto_renew=False)
                with self.assertRaisesRegex(RuntimeError,'exclusive'):base.pin('other',auto_renew=False)
                flow=MonitoredMemorySession(pin,ModelDomainAdapter(),'source');pin.memory_flow=flow
                first=request('first',100.);first.update(epoch=1,clock_domain='source',valid_until_source_time=200.)
                first['observation']['feedback_sample_time_s']=99.995
                answer=flow.infer(first)
                self.assertEqual(answer['audit']['memory']['observations'],1)
                self.assertNotIn(pin.token,json.dumps(answer['audit']))
                resolved=threading.Event();entered=threading.Event();release=threading.Event()
                clock=mem.LogicalMemoryClock();clock.set_mode('TEACH',101.)
                def selected(stamp,epoch,run_id=None):
                    value=request('selected',stamp)['observation'];value.pop('images')
                    return dict(value,run_id='run')
                def resolve(value):
                    result=dict(value,images=request()['observation']['images'])
                    if value['source_time']>101.9:resolved.set()
                    return result
                flow.select=selected;flow.resolve=resolve
                act=Policy.act
                def waiting(policy,*args):
                    entered.set();release.wait(3);return act(policy,*args)
                with patch.object(Policy,'act',waiting),ThreadPoolExecutor(max_workers=1)as pool:
                    value=request('long-prediction',100.1)
                    value.update(epoch=1,clock_domain='source',valid_until_source_time=200.)
                    pending=pool.submit(flow.infer,value);self.assertTrue(entered.wait(1))
                    clock.advance(101.96)
                    sample=selected(101.96,2)
                    sample['memory_sampling']=dict(schema='workbench.memory-activity.v2',run_id='run',phase='TEACH',**clock.snapshot())
                    flow.offer(sample,2)
                    self.assertTrue(resolved.wait(1),'real bytes must resolve while model IO is blocked')
                    self.assertEqual(worker.memory.observations,1)
                    release.set();pending.result(3)
                deadline=time.monotonic()+2
                while worker.memory.observations<2 and time.monotonic()<deadline:time.sleep(.01)
                self.assertEqual(worker.memory.observations,2)
                self.assertEqual(worker.policy.times,[0.,.96])
                flow.clear('manual-check-clear',epoch=3)
                first['request_id']='fresh-after-manual';first['epoch']=3
                flow.infer(first);flow.mode='HOLD'
                before=(worker.policy.memory,list(worker.policy.times),pin.token,pin.memory_context_revision)
                for steps in(10,1,2):
                    flow.reconfigure(client(steps))
                    self.assertEqual((worker.policy.memory,list(worker.policy.times),pin.token,pin.memory_context_revision),before)
                    self.assertEqual(worker.policy.config.inference.denoising_steps,steps)
                arbitrary=request('rtc-prefix',101.)
                receipt=pin.observe_model_node(arbitrary,step=11,model_time_s=11/30.,target_source_time=101.,epoch=4,clock_domain='source')
                self.assertEqual(receipt['audit']['memory']['context_revision'],2)
                manual=request('manual',102.)
                pin.observe_manual_node(manual,step=16,model_time_s=11/30.+.48,target_source_time=102.,epoch=4,clock_domain='source')
                self.assertEqual(worker.memory.manual_observations,1)
                snapshot=deepcopy(worker.policy.memory)
                def predict(index):
                    value=request('rtc-query'+str(index),103.,epoch=4)
                    value['memory_context']=dict(revision=2,time_s=11/30.)
                    return pin.infer_chunk(value,epoch=4,clock_domain='source',valid_until_source_time=200.)
                # The worker serializes GPU work. Overlapping callers may get
                # busy, but an accepted request keeps its explicit context.
                with ThreadPoolExecutor(max_workers=2)as pool:
                    futures=[pool.submit(predict,i)for i in range(2)]
                    replies=[]
                    for future in futures:
                        try:replies.append(future.result())
                        except RuntimeError as error:self.assertIn('busy',str(error))
                self.assertTrue(replies)
                for reply in replies:self.assertEqual(reply['audit']['context_revision'],2)
                self.assertEqual(worker.policy.memory,snapshot)
                stale=request('stale',104.,epoch=4)
                stale['memory_context']=dict(revision=1,time_s=1.)
                with self.assertRaisesRegex(RuntimeError,'expired'):
                    pin.infer_chunk(stale,epoch=4,clock_domain='source',valid_until_source_time=200.)
                self.assertTrue(service.status()['loaded']);self.assertIsNone(service.status().get('error'))
                self.assertTrue(flow.control_event(10));self.assertTrue(flow.control_event(13))
                self.assertEqual(flow.snapshot()['control_event_gaps'],1)
                self.assertTrue(flow.snapshot()['available'])
                flow.clear('clear',epoch=4)
                self.assertEqual(pin.memory_context_revision,0);self.assertIsNone(worker.policy.memory)
                first['request_id']='after-clear';first['epoch']=5
                self.assertEqual(flow.infer(first)['audit']['memory']['observations'],1)
                entered=threading.Event();release=threading.Event();act=Policy.act
                def blocked(policy,*args):
                    entered.set();release.wait(3);return act(policy,*args)
                with patch.object(Policy,'act',blocked),ThreadPoolExecutor(max_workers=2)as pool:
                    pending=pool.submit(pin.infer_chunk,request('pending',110.),epoch=6,clock_domain='source',valid_until_source_time=200.)
                    self.assertTrue(entered.wait(1))
                    reset=pool.submit(pin.memory_clear,'concurrent-clear',epoch=7,clock_domain='source')
                    deadline=time.monotonic()+1
                    while not server.model_service._mutating and time.monotonic()<deadline:time.sleep(.005)
                    self.assertTrue(server.model_service._mutating);release.set()
                    with self.assertRaisesRegex(RuntimeError,'reset while'):pending.result(3)
                    self.assertTrue(reset.result(3)['audit']['memory']['empty'])
                self.assertIsNone(worker.policy.memory)
            finally:
                if flow:flow.close();pin.memory_flow=None
                if pin:pin.close()
                server.shutdown();server.server_close();thread.join(2)


if __name__=='__main__':unittest.main()
