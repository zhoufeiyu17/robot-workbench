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
        memory=dict(protocol=mem.MEMORY_PROTOCOL,session_id=session,epoch=epoch,clock_domain='source',start='fresh',revision=0),
        observation=dict(source_time=stamp,model_state14=np.zeros(14,np.float32),
            images={key:np.zeros((8,10,3),np.uint8)for key in wam.CAMS}))


class RealTrajectorySampling(unittest.TestCase):
    @staticmethod
    def observation(index, offset=0.):
        return dict(source_time=offset+index/30., model_state14=[float(index)]*14,
                    frame_ids={'fixture': index})

    def collect(self, phase, *, count=65):
        sampler=mem.RealTrajectorySampler();blocks=[]
        for index in range(count):
            value=self.observation(index)
            blocks.extend(sampler.offer(value,phase=phase,grid_time=value['source_time']))
        return sampler,blocks

    def test_model_and_teach_share_training_write_indices_and_gt_alignment(self):
        model,a=self.collect('MODEL_EXECUTING');teach,b=self.collect('TEACH')
        self.assertEqual([x['effective_index'] for x in a],[0,32,64])
        self.assertEqual([x['valid_actions'] for x in a],[0,32,32])
        for left,right in zip(a,b):
            self.assertEqual({k:v for k,v in left.items() if k!='mode'},
                             {k:v for k,v in right.items() if k!='mode'})
            if left['valid_actions']:
                np.testing.assert_array_equal(np.asarray(left['actions14'])-left['states14'],1.)
                self.assertTrue(all(a>b for a,b in zip(left['action_times'],left['state_times'])))
        self.assertEqual(model.samples,teach.samples)

    def test_wait_and_handover_preserve_effective_stride_and_true_source_gap(self):
        sampler=mem.RealTrajectorySampler();blocks=[]
        for index in range(20):
            value=self.observation(index)
            blocks.extend(sampler.offer(value,phase='MODEL_EXECUTING',grid_time=value['source_time']))
        for index in range(20,100):
            value=self.observation(index)
            self.assertEqual(sampler.offer(value,phase='WAIT',grid_time=value['source_time']),[])
        self.assertEqual(sampler.samples,20)
        for index in range(100,113):
            value=self.observation(index)
            blocks.extend(sampler.offer(value,phase='TEACH',grid_time=value['source_time']))
        self.assertEqual([x['effective_index']for x in blocks],[0,32])
        self.assertEqual(blocks[-1]['observation']['source_time'],112/30.)
        self.assertNotIn(([19.]*14,[100.]*14),list(zip(blocks[-1]['states14'],blocks[-1]['actions14'])))
        self.assertEqual(blocks[-1]['valid_actions'],31)

    def test_duplicate_and_missing_frames_never_generate_interpolated_gt(self):
        sampler=mem.RealTrajectorySampler(stride=4);blocks=[]
        for index in [0,1,2,2,6,7]:
            value=self.observation(index)
            blocks.extend(sampler.offer(value,phase='TEACH',grid_time=value['source_time']))
        self.assertEqual(blocks[-1]['effective_index'],4)
        self.assertEqual(blocks[-1]['states14'],[[0.]*14,[1.]*14,[6.]*14])
        self.assertEqual(blocks[-1]['actions14'],[[1.]*14,[2.]*14,[7.]*14])
        self.assertEqual(sampler.gaps,1)
        with self.assertRaisesRegex(ValueError,'regressed'):
            sampler.offer(self.observation(6),phase='TEACH',grid_time=6/30.)




class Policy:
    def __init__(self):
        self.resets=0;self.times=[];self.observations=[];self.memory=None;self.executed=[];self.first_frame=None;self.acts=0
        self.config=SimpleNamespace(inference=SimpleNamespace(denoising_steps=2))
    def reset(self,*,control_frequency):
        assert control_frequency==30
        self.resets+=1;self.times=[];self.memory=None;self.executed=[];self.pending=None;self.first_frame=None
    def observe(self,rgb,timestamp):
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
        value=mem.worker_request(value);result=self.memory.infer(value)
        if value['warmup']:self.memory.clear()
        return result
    def close(self):pass
    def can_reconfigure(self,bundle):return mem.sampling_change(self.bundle,bundle)
    def reconfigure(self,bundle):
        self.bundle=deepcopy(bundle);self.policy.config.inference.denoising_steps=bundle['inference_options']['denoise_steps']
        return mem.worker_identity(bundle)


class MemoryTests(unittest.TestCase):
    def test_sixteen_step_sampler_matches_teach_and_survives_clear(self):
        client=SimpleNamespace(model_bundle={'training_execute_actions':16},
            memory_clear=Mock(return_value={'audit':{'memory':{}}}))
        flow=mem.MemorySession(client,None,'source')
        try:
            for phase in ('MODEL_EXECUTING','TEACH'):
                flow.begin('fresh');blocks=[]
                for index in range(33):
                    obs=RealTrajectorySampling.observation(index)
                    blocks.extend(flow.sampler.offer(obs,phase=phase,grid_time=obs['source_time']))
                self.assertEqual([b['effective_index']for b in blocks],[0,16,32])
            flow.clear('clear',epoch=1)
            self.assertEqual(flow.sampler.stride,16)
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

    def test_sixteen_step_execution_preserves_full_prediction_and_memory(self):
        policy=Policy();policy.config.data=SimpleNamespace(execute_actions=16)
        memory=mem.PhaseMemory(policy,'fixture instruction',lambda seed:None)
        result=memory.infer(request())
        self.assertEqual(result['action14'].shape,(32,14))
        self.assertEqual(result['audit']['memory']['sampling_stride'],16)
        self.assertEqual(policy.times,[0.])
        self.assertEqual(memory.observations,1)

    def test_hot_sampling_keeps_exclusive_memory_lease_and_rejects_other_changes(self):
        from robot_workbench.inference_model import model_bundle_digest
        worker=Worker();variants=mem.variants(registry())
        service=ModelService(worker,catalog=lambda:{str(n):b for n,b in enumerate(variants)},
            client_factory=lambda _:self.fail('sampling must retain the worker'))
        with TemporaryDirectory()as directory:
            token=Path(directory)/'token';token.write_text('fixture-secret-'+40*'x')
            server=serve_model(service,token_file=token,port=0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            def client(steps):return RemoteBaseClient('http://127.0.0.1:'+str(server.server_port),token_file=token,
                adapter=ModelDomainAdapter(),conversion_identity={'scope':'cpu-only'},model_id=worker.model_id,
                model_bundle=mem.descriptor(registry(),denoise_steps=steps))
            pin=None;flow=None
            try:
                base=client(2);base.prepare();pin=base.pin('continuous-memory',auto_renew=False)
                first=request('first',100.);first.pop('memory')
                pin.infer_chunk(first,epoch=1,clock_domain='source',valid_until_source_time=110.)
                flow=mem.MemorySession(pin,ModelDomainAdapter(),'source');pin.memory_flow=flow
                state=(worker.policy.memory,worker.policy.first_frame.copy(),list(worker.policy.times),worker.memory.calls,worker.memory.observations)
                identity=(pin.token,pin.session_id,pin.memory_revision,worker.memory.phase)
                for steps in (10,1,2):
                    flow.reconfigure(client(steps))
                    self.assertEqual(worker.policy.config.inference.denoising_steps,steps)
                    self.assertEqual(worker.memory.calls,state[3]);self.assertEqual(worker.memory.observations,state[4])
                    self.assertEqual(worker.policy.memory,state[0]);np.testing.assert_array_equal(worker.policy.first_frame,state[1])
                    self.assertEqual(worker.policy.times,state[2]);self.assertEqual((pin.token,pin.session_id,pin.memory_revision,worker.memory.phase),identity)
                    self.assertEqual(pin.model_binding['bundle_digest'],model_bundle_digest(worker.model_id,pin.model_bundle))
                    pin.renew();self.assertTrue(pin.healthy())
                self.assertEqual(service.prepare_attempts,1)
                with self.assertRaisesRegex(RuntimeError,'pinned'):client(10).prepare()
                next_request=request('continued',101.);next_request.pop('memory');next_request['memory_start']='continue'
                result=pin.infer_chunk(next_request,epoch=2,clock_domain='source',valid_until_source_time=111.)
                self.assertEqual(result['audit']['memory']['observations'],1);self.assertFalse(result['audit']['memory']['reset_applied'])
                with self.assertRaises(ValueError):mem.descriptor(registry(),denoise_steps=11)
            finally:
                if flow:flow.close();pin.memory_flow=None
                if pin:pin.close()
                server.shutdown();server.server_close();thread.join(2)

    def test_phase_lifecycle_retry_and_original_input(self):
        policy=Policy();seeds=[];memory=mem.PhaseMemory(policy,'fixture instruction',seeds.append)
        a=request();first=memory.infer(a);again=memory.infer(deepcopy(a))
        np.testing.assert_array_equal(first['action14'],again['action14'])
        self.assertEqual((policy.resets,len(policy.times)),(1,1))
        b=memory.infer(request('next',101.))
        self.assertEqual(b['audit']['memory']['observations'],1)
        self.assertEqual(seeds,[7,8]);self.assertEqual(policy.times,[0.])
        self.assertEqual(b['audit']['memory']['source_time'],100.)
        self.assertEqual(b['audit']['memory']['query_source_time'],101.)
        self.assertEqual(b['audit']['memory']['origin_source_time'],100.)
        changed=deepcopy(a);changed['observation']['images'][wam.CAMS[0]][0,0,0]=1
        with self.assertRaisesRegex(ValueError,'different data'):memory.infer(changed)
        reset=memory.infer(request('decision',104.,epoch=2))
        self.assertTrue(reset['audit']['memory']['reset_applied']);self.assertEqual(policy.times,[0.])
        with self.assertRaisesRegex(ValueError,'obsolete'):memory.infer(a)
        same=memory.infer(request('same-observation-read',104.,epoch=2))
        self.assertEqual(same['audit']['memory']['observations'],1)
        with self.assertRaisesRegex(ValueError,'future memory'):memory.infer(request('late',103.,epoch=2))
        memory.infer(request('decision',200.,epoch=0,session='reacquired'))
        self.assertEqual(policy.resets,3)
        self.assertEqual(policy.first_frame.shape,(8,10,3))

    def test_manual_memory_is_observation_only_and_clear_cannot_be_repopulated(self):
        policy=Policy();memory=mem.PhaseMemory(policy,'fixture instruction',lambda seed:None)
        memory.infer(request('model',100.))
        a=request('manual',101.,epoch=2);a['memory']['start']='continue';a['memory_operation']='observe'
        a['observation']['model_state14'][0]=2
        a['memory_sample']=dict(states14=[],actions14=[],state_times=[],action_times=[],mode='TEACH',
            effective_index=32,action_semantics='materialized_next_observation',valid_actions=0,executed_action_condition=False)
        result=memory.infer(a)
        self.assertNotIn('action14',result);self.assertEqual(memory.calls,1)
        self.assertEqual(policy.memory,5.);self.assertEqual(policy.times,[0.,1.])
        resumed=request('resume',102.,epoch=3);resumed['memory']['start']='continue'
        result=memory.infer(resumed)
        self.assertFalse(result['audit']['memory']['reset_applied']);self.assertEqual(policy.memory,5.)
        np.testing.assert_array_equal(result['action14'],11.)
        self.assertEqual(result['audit']['memory']['manual_observations'],1)
        self.assertFalse(result['audit']['memory']['executed_action_context']);self.assertEqual(policy.executed,[])
        clear=dict(request_id='clear',memory=dict(resumed['memory'],epoch=4),memory_operation='clear')
        result=memory.infer(clear);self.assertEqual(result['audit']['memory']['revision'],1)
        self.assertIsNone(policy.memory);self.assertIsNone(policy.first_frame);self.assertEqual(policy.times,[])
        with self.assertRaisesRegex(ValueError,'obsolete'):memory.infer(a)
        old=request('old',103.,epoch=4);old['memory']['start']='continue'
        with self.assertRaisesRegex(ValueError,'revision'):memory.infer(old)
        restarted=request('new',104.,epoch=5);restarted['memory'].update(start='continue',revision=1)
        result=memory.infer(restarted);self.assertEqual(policy.memory,1.)
        self.assertEqual(result['audit']['memory']['origin_source_time'],104.)
        bad=request('wrong-clock',105.,epoch=6);bad['memory'].update(clock_domain='another',revision=1)
        with self.assertRaisesRegex(ValueError,'clock'):memory.infer(bad)
        self.assertEqual(policy.memory,1.)

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

    def test_http_exclusive_memory_pin_and_reacquisition(self):
        worker=Worker();service=ModelService(worker)
        with TemporaryDirectory()as directory:
            token=Path(directory)/'http-token';token.write_text('fixture-secret-'+40*'x')
            server=serve_model(service,token_file=token,port=0)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            client=RemoteBaseClient('http://127.0.0.1:'+str(server.server_port),token_file=token,
                adapter=ModelDomainAdapter(),conversion_identity={'scope':'cpu-only'},
                model_id=worker.model_id,model_bundle=worker.bundle)
            pin=None
            try:
                client.prepare();self.assertIsNone(worker.memory.phase)
                pin=client.pin('same-logical-session',auto_renew=False)
                with self.assertRaisesRegex(RuntimeError,'exclusive'):client.pin('other',auto_renew=False)
                a=request();a.pop('memory')
                first=pin.infer_chunk(a,epoch=1,clock_domain='source',valid_until_source_time=110.)
                self.assertNotIn(pin.token,json.dumps(first['audit']))
                self.assertEqual(first['audit']['memory']['observations'],1)
                old_session=worker.memory.phase[0]
                b=request('next',101.);b.pop('memory')
                pin.infer_chunk(b,epoch=1,clock_domain='source',valid_until_source_time=111.)
                self.assertEqual(worker.memory.calls,2)
                pin.close();pin=client.pin('same-logical-session',auto_renew=False)
                pin.infer_chunk(a,epoch=1,clock_domain='source',valid_until_source_time=110.)
                self.assertNotEqual(old_session,worker.memory.phase[0]);self.assertEqual(worker.memory.calls,1)
                image_fixture=request()['observation']['images']
                flow=mem.MemorySession(pin,ModelDomainAdapter(),'source',resolve=lambda obs:dict(obs,images=image_fixture));pin.memory_flow=flow
                self.assertTrue(flow.control_event(10))
                self.assertTrue(flow.control_event(13))
                self.assertFalse(flow.control_event(12))
                self.assertTrue(flow.snapshot()['available'])
                self.assertEqual(flow.snapshot()['last_control_gap'],dict(previous=10,received=13,missing=2))
                flow.sampler.bootstrap(a['observation'])
                def offered(stamp,phase='TEACH',epoch=2):
                    value=request('sample',stamp)['observation'];value.pop('images')
                    value['model_state14'][0]=2
                    value.update(run_id='run',sample_grid_time=stamp,
                        memory_sampling=dict(schema='workbench.memory-activity.v1',run_id='run',phase=phase))
                    flow.offer(value,epoch)
                for index in range(1,33):offered(100.+index/30.)
                self.assertEqual(flow.snapshot()['phase'],'TEACH','paired state recovers a missing mode audit event')
                deadline=time.monotonic()+2
                while worker.memory.manual_observations<1 and time.monotonic()<deadline:time.sleep(.01)
                self.assertEqual(worker.memory.manual_observations,1)
                self.assertEqual(worker.memory.observations,2)
                flow.mode_event(dict(command_epoch=3,mode='HOLD',completed=True))
                self.assertEqual(flow.snapshot()['control_epoch'],3)
                flow.begin('continue')
                continued=request('continued',102.);continued.update(epoch=4,clock_domain='source',valid_until_source_time=112.)
                answer=flow.infer(continued)
                self.assertEqual(answer['audit']['memory']['observations'],2)
                flow.mode_event(dict(command_epoch=4,mode='MODEL',completed=True))
                entered=threading.Event();release=threading.Event();act=Policy.act
                def blocked_decision(policy,*args):
                    entered.set();release.wait(3);return act(policy,*args)
                waiting=request('waiting-model',103.)
                waiting.update(epoch=4,clock_domain='source',valid_until_source_time=113.)
                with patch.object(Policy,'act',blocked_decision),ThreadPoolExecutor(max_workers=1)as pool:
                    pending=pool.submit(flow.infer,waiting)
                    self.assertTrue(entered.wait(1))
                    try:
                        count=flow.sampler.samples
                        offered(103.5,'WAIT',4)
                        self.assertEqual(flow.sampler.samples,count)
                        self.assertFalse(flow.pending)
                    finally:release.set()
                    answer=pending.result(3)
                self.assertEqual(answer['audit']['memory']['source_time'],100.+32/30.)
                self.assertEqual(answer['audit']['memory']['observations'],2)
                next_decision=request('after-wait',105.)
                next_decision.update(epoch=4,clock_domain='source',valid_until_source_time=115.)
                flow.infer(next_decision)
                np.testing.assert_allclose(worker.policy.times,[0.,32/30.],rtol=0,atol=1e-12)
                cleared=flow.clear('clear',5);self.assertTrue(cleared['audit']['memory']['empty'])
                self.assertEqual(pin.memory_revision,1)
                self.assertEqual(pin.memory_clear('clear',epoch=5,clock_domain='source'),cleared)
                self.assertIsNone(worker.policy.memory)
                offered(106.,'HOLD',5);time.sleep(.05);self.assertIsNone(worker.policy.memory)
                flow.close();pin.memory_flow=None
                # Clear is ordered after an in-flight GPU call; the old result is fenced.
                entered=threading.Event();release=threading.Event();act=Policy.act
                def blocked(policy,*args):
                    entered.set();release.wait(3);return act(policy,*args)
                with patch.object(Policy,'act',blocked),ThreadPoolExecutor(max_workers=2)as pool:
                    pending=pool.submit(pin.infer_chunk,request('pending',110.),epoch=6,clock_domain='source',valid_until_source_time=120.)
                    self.assertTrue(entered.wait(1))
                    reset=pool.submit(pin.memory_clear,'concurrent-clear',epoch=7,clock_domain='source')
                    deadline=time.monotonic()+1
                    while not server.model_service._mutating and time.monotonic()<deadline:time.sleep(.005)
                    self.assertTrue(server.model_service._mutating);release.set()
                    with self.assertRaisesRegex(RuntimeError,'reset while'):pending.result(3)
                    self.assertTrue(reset.result(3)['audit']['memory']['empty'])
                self.assertIsNone(worker.policy.memory);self.assertEqual(pin.memory_revision,2)

            finally:
                if pin is not None:pin.close()
                server.shutdown();server.server_close();thread.join(2);server.model_service.close()


if __name__=='__main__':unittest.main()
