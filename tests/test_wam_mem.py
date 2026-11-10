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
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench import wam,wam_mem as mem
from robot_workbench.inference_model import model_family,action_contract
from robot_workbench.inference_transport import ModelService,RemoteBaseClient,ModelDomainAdapter,serve_model


def registry():
    rows=[dict(path=role+'.fixture',bytes=1,sha256=hashlib.sha256(role.encode()).hexdigest())for role in mem.ROLES]
    return dict(model_id='wam-mem-fixture-step1',step=1,instruction='fixture instruction',files=rows,
        artifacts={role:row['sha256']for role,row in zip(mem.ROLES,rows)},
        bundle_sha256=hashlib.sha256(json.dumps(sorted(rows,key=lambda r:r['path']),sort_keys=True,separators=(',',':')).encode()).hexdigest())


def request(identity='decision',stamp=100.,epoch=1,session='session'):
    return dict(request_id=identity,seed=7,applied_history=np.empty((0,14),np.float32),
        memory=dict(protocol=mem.MEMORY_PROTOCOL,session_id=session,epoch=epoch,clock_domain='source',start='fresh',revision=0),
        observation=dict(source_time=stamp,model_state14=np.zeros(14,np.float32),
            images={key:np.zeros((8,10,3),np.uint8)for key in wam.CAMS}))


class Policy:
    def __init__(self):
        self.resets=0;self.times=[];self.observations=[];self.memory=None;self.executed=[];self.first_frame=None;self.acts=0
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
        return {'execute':np.full((32,14),self.memory,np.float32)}


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


class MemoryTests(unittest.TestCase):
    def test_phase_lifecycle_retry_and_original_input(self):
        policy=Policy();seeds=[];memory=mem.PhaseMemory(policy,'fixture instruction',seeds.append)
        a=request();first=memory.infer(a);again=memory.infer(deepcopy(a))
        np.testing.assert_array_equal(first['action14'],again['action14'])
        self.assertEqual((policy.resets,len(policy.times)),(1,1))
        b=memory.infer(request('next',101.))
        self.assertEqual(b['audit']['memory']['observations'],2)
        self.assertEqual(seeds,[7,8]);self.assertEqual(policy.times,[0.,1.])
        self.assertEqual(b['audit']['memory']['source_time'],101.)
        self.assertEqual(b['audit']['memory']['origin_source_time'],100.)
        changed=deepcopy(a);changed['observation']['images'][wam.CAMS[0]][0,0,0]=1
        with self.assertRaisesRegex(ValueError,'different data'):memory.infer(changed)
        reset=memory.infer(request('decision',104.,epoch=2))
        self.assertTrue(reset['audit']['memory']['reset_applied']);self.assertEqual(policy.times,[0.])
        with self.assertRaisesRegex(ValueError,'obsolete'):memory.infer(a)
        with self.assertRaisesRegex(ValueError,'strictly advance'):memory.infer(request('late',104.,epoch=2))
        memory.infer(request('decision',200.,epoch=0,session='reacquired'))
        self.assertEqual(policy.resets,3)
        self.assertEqual(set(policy.observations[-1][0]),{'main','left','right'})

    def test_manual_memory_is_observation_only_and_clear_cannot_be_repopulated(self):
        policy=Policy();memory=mem.PhaseMemory(policy,'fixture instruction',lambda seed:None)
        memory.infer(request('model',100.))
        a=request('manual',101.,epoch=2);a['memory']['start']='continue';a['memory_operation']='observe'
        a['observation']['model_state14'][0]=2
        result=memory.infer(a)
        self.assertNotIn('action14',result);self.assertEqual(policy.acts,1)
        self.assertEqual(policy.memory,5.);self.assertEqual(policy.times,[0.,1.])
        resumed=request('resume',102.,epoch=3);resumed['memory']['start']='continue'
        result=memory.infer(resumed)
        self.assertFalse(result['audit']['memory']['reset_applied']);self.assertEqual(policy.memory,11.)
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
                flow.mode_event(dict(command_epoch=2,mode='TEACH',completed=True))
                manual=request('manual',101.)['observation'];manual.pop('images');manual['model_state14'][0]=2
                flow.offer(manual,2)
                deadline=time.monotonic()+2
                while worker.memory.manual_observations<1 and time.monotonic()<deadline:time.sleep(.01)
                self.assertEqual(worker.memory.manual_observations,1)
                flow.mode_event(dict(command_epoch=3,mode='HOLD',completed=True))
                self.assertEqual(flow.snapshot()['control_epoch'],3)
                self.assertEqual(flow.snapshot()['phase_epoch'],2)
                flow.begin('continue')
                continued=request('continued',102.);continued.update(epoch=4,clock_domain='source',valid_until_source_time=112.)
                answer=flow.infer(continued)
                self.assertEqual(answer['audit']['memory']['observations'],3)
                cleared=flow.clear('clear',5);self.assertTrue(cleared['audit']['memory']['empty'])
                self.assertEqual(pin.memory_revision,1)
                self.assertEqual(pin.memory_clear('clear',epoch=5,clock_domain='source'),cleared)
                self.assertIsNone(worker.policy.memory)
                flow.offer(manual,2);time.sleep(.05);self.assertIsNone(worker.policy.memory)
                flow.close();pin.memory_flow=None
                # Clear is ordered after an in-flight GPU call; the old result is fenced.
                entered=threading.Event();release=threading.Event();act=worker.policy.act
                def blocked(*args):
                    entered.set();release.wait(3);return act(*args)
                worker.policy.act=blocked
                with ThreadPoolExecutor(max_workers=2)as pool:
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
