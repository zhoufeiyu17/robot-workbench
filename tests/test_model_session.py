"""Real loopback HTTP/NPZ with a CPU worker: shared pins, revocation and secrecy."""
from concurrent.futures import ThreadPoolExecutor
import json
import subprocess
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
from robot_workbench.model_session import PinnedModelClient,PinnedModelService,PROTOCOL,binding
from robot_workbench.inference_transport import ModelService,RemoteBaseClient,serve_model,PROTOCOL as WIRE
from test_inference_transport import StubModel,Adapter,request


class SessionTests(unittest.TestCase):
    def test_transport_and_lease_use_run_policy_without_forwarding_it_to_worker(self):
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        pin=self.pin(self.a,'cpu-policy')
        parameters=control_parameters(defaults('inference'),business='inference');parameters['measured']['limit_rad_s']=1.1
        policy=dict(revision=7,parameters=parameters,policy_hash=policy_hash(parameters))
        pin.bind_control_policy(policy)
        reply=pin.infer(request('policy-request'));audit=reply['audit']
        transport=audit['transport']['guard_decisions'][0];lease=audit['session_guard']
        self.assertEqual((transport['rule_id'],lease['rule_id']),('G09','G10'))
        self.assertEqual({row['policy_hash']for row in (transport,lease)},{policy['policy_hash']})
        self.assertEqual({row['revision']for row in (transport,lease)},{7})
        self.assertNotIn('policy_hash',self.model.requests[-1])
        self.assertNotIn(pin.token,json.dumps(audit))

    def test_expired_client_pin_keeps_decision_and_does_not_call_worker(self):
        pin=self.pin(self.a,'cpu-expired');before=len(self.model.requests)
        with patch('robot_workbench.model_session.time.monotonic',return_value=pin.renewed_at+pin.ttl):
            with self.assertRaisesRegex(ValueError,'not healthy')as caught:pin.infer(request('expired'))
        self.assertEqual(len(self.model.requests),before)
        self.assertEqual(caught.exception.guard_decision['rule_id'],'G10')
        self.assertEqual(caught.exception.guard_decision['verdict'],'reject')
        self.assertNotIn(pin.token,json.dumps(caught.exception.guard_decision))

    def setUp(self):
        self.temp=TemporaryDirectory();self.token_file=Path(self.temp.name)/'http-auth'
        self.token_file.write_text('cpu-only-'+40*'x');self.clock=[1.];self.pins=[]
        self.model=StubModel();self.raw=ModelService(self.model);self.start_server()
        self.a=self.client();self.b=self.client();self.a.prepare()

    def start_server(self,port=0):
        self.server=serve_model(self.raw,token_file=self.token_file,port=port)
        self.service=self.server.model_service;self.service.clock=lambda:self.clock[0]
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()

    def tearDown(self):
        if self.model.release is not None:self.model.release.set()
        for pin in self.pins:
            try:pin.close()
            except RuntimeError:pass
        self.server.shutdown();self.server.server_close();self.thread.join(2)
        self.service.close();self.temp.cleanup()

    def client(self):
        return RemoteBaseClient('http://127.0.0.1:'+str(self.server.server_port),token_file=self.token_file,
            adapter=Adapter(),conversion_identity={'device':'cpu-explicit'})

    def pin(self,client,session,*,renew=False):
        value=client.pin(session,auto_renew=renew);self.pins.append(value);return value

    def rpc(self,client,path,body):return json.loads(client._request(path,json.dumps(body).encode()))

    def test_plain_service_is_protected_and_current_public_diagnostics_still_work(self):
        self.assertIsInstance(self.service,PinnedModelService)
        self.assertEqual(self.a.status()['session_protocol'],PROTOCOL)
        self.assertEqual(self.a.infer(request())['request_id'],'request-1')
        self.rpc(self.a,'/unload',dict(protocol=WIRE,model_id=self.a.model_id,conversion_id=self.a.conversion_id,
            expected_bundle_digest=self.a.bundle_digest))
        self.assertFalse(self.a.status()['loaded'])

    def test_two_consumers_block_every_public_model_change_without_unloading(self):
        a=self.pin(self.a,'workbench');b=self.pin(self.b,'dagger')
        self.assertEqual(a.infer(request('a'))['request_id'],'a')
        self.assertEqual(b.infer(request('b'))['request_id'],'b')
        for path in ('/prepare','/unload','/v1/conversions','/v1/infer'):
            with self.subTest(path=path),self.assertRaisesRegex(RuntimeError,'pinned'):self.rpc(self.b,path,{})
        self.b.prepare(ready_only=True)
        with self.assertRaisesRegex(RuntimeError,'session'):self.b.infer(request('unbound'))
        a.close();self.assertEqual(self.b.status()['active_sessions'],1)
        self.assertEqual(b.infer(request())['request_id'],'request-1')
        b.close();self.assertEqual(self.a.status()['active_sessions'],0);self.assertEqual(self.model.closes,0)

    def test_second_listener_cannot_bypass_the_same_model_registry(self):
        a=self.pin(self.a,'first-listener')
        server=serve_model(self.raw,token_file=self.token_file)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            other=RemoteBaseClient('http://127.0.0.1:'+str(server.server_port),token_file=self.token_file,
                adapter=Adapter(),conversion_identity={'device':'cpu-second-listener'})
            self.assertIs(server.model_service,self.service)
            self.assertEqual(other.status()['active_sessions'],1)
            with self.assertRaisesRegex(RuntimeError,'pinned'):other.prepare()
        finally:server.shutdown();server.server_close();thread.join(2)

    def test_capability_never_reaches_worker_status_log_or_returned_audit(self):
        pin=self.pin(self.a,'workbench');capability=pin.token
        log=Path(self.temp.name)/'worker.log';self.model.log_path=str(log);original=self.model.infer
        def infer(value):
            log.write_text(value['request_id'])
            result=original(value);result['audit']['worker_request_id']=value['request_id'];return result
        self.model.infer=infer
        result=pin.infer(request('visible-id'))
        self.assertEqual(result['audit']['worker_request_id'],'visible-id')
        self.assertEqual(result['audit']['transport']['request_id'],'visible-id')
        self.assertNotIn(capability,json.dumps(result['audit']))
        self.assertNotIn(capability,json.dumps(self.a.status()))
        self.assertNotIn(capability,log.read_text())
        self.clock[0]=32.
        self.assertNotIn(capability,json.dumps(self.a.status()))
        with self.assertRaisesRegex(RuntimeError,'expired'):pin.infer(request('expired'))

    def test_expired_or_released_inflight_result_does_not_escape(self):
        for mode in ('expire','release'):
            with self.subTest(mode=mode):
                self.clock[0]=1.;a=self.pin(self.a,'source-'+mode);self.model.entered.clear();self.model.release=threading.Event()
                with ThreadPoolExecutor(max_workers=1)as pool:
                    pending=pool.submit(a.infer,request(mode));self.assertTrue(self.model.entered.wait(1))
                    if mode=='expire':self.clock[0]=32.
                    else:a.close()
                    self.model.release.set()
                    with self.assertRaisesRegex((RuntimeError,ValueError),'expired'):pending.result(timeout=2)
                self.model.release=None
                try:a.close()
                except RuntimeError:pass
                self.assertEqual(self.model.closes,0);self.assertTrue(self.raw.status()['loaded'])

    def test_renewal_remains_available_during_inference(self):
        a=self.pin(self.a,'workbench');b=self.pin(self.b,'dagger')
        self.model.entered.clear();self.model.release=threading.Event()
        with ThreadPoolExecutor(max_workers=1)as pool:
            pending=pool.submit(a.infer,request());self.assertTrue(self.model.entered.wait(1))
            self.clock[0]=20.;a.renew();b.renew();self.clock[0]=35.;self.model.release.set()
            self.assertEqual(pending.result(timeout=2)['request_id'],'request-1')
        self.model.release=None
        self.assertEqual(b.infer(request('next'))['request_id'],'next')

    def test_service_restart_rejects_old_capability_even_without_other_pins(self):
        a=self.pin(self.a,'old-service');port=self.server.server_port
        self.server.shutdown();self.server.server_close();self.thread.join(2);self.service.close()
        self.model=StubModel();self.raw=ModelService(self.model);self.start_server(port)
        fresh=self.client();fresh.prepare()
        count=len(self.model.requests)
        with self.assertRaisesRegex(RuntimeError,'expired'):a.infer(request('stale'))
        self.assertEqual(len(self.model.requests),count)

    def test_expired_release_is_idempotent_and_does_not_unload_a_new_pin(self):
        old=self.pin(self.a,'old');capability=old.token;self.clock[0]=32.
        new=self.pin(self.b,'new')
        old.close()
        receipt=self.rpc(self.a,'/unload',dict(session_protocol=PROTOCOL,operation='release',token=capability))
        self.assertTrue(receipt['already_released']);self.assertFalse(receipt['model_unloaded'])
        self.assertEqual(self.service.status()['active_sessions'],1)
        self.assertEqual(new.infer(request())['request_id'],'request-1');self.assertEqual(self.model.closes,0)

    def test_binding_change_and_queued_diagnostics_block_acquire_or_reuse(self):
        self.raw.active='queued-conversion'
        with self.assertRaisesRegex(RuntimeError,'busy'):self.a.pin('not-started',auto_renew=False)
        self.raw.active=None;a=self.pin(self.a,'same-model')
        self.raw.instance_id='replaced-instance'
        with self.assertRaisesRegex(ValueError,'changed')as caught:self.service._check(a.token)
        self.assertEqual(caught.exception.guard_decision['rule_id'],'G01')
        self.assertEqual(caught.exception.guard_decision['action'],'INVALIDATE_RUN')
        self.assertNotIn(a.token,json.dumps(caught.exception.guard_decision))
        with self.assertRaisesRegex(RuntimeError,'changed'):a.renew()
        with self.assertRaisesRegex(ValueError,'not healthy'):a.infer(request())

    def test_admission_gap_before_underlying_worker_is_already_busy(self):
        entered=threading.Event();release=threading.Event();original=self.raw.infer
        def delayed(payload):entered.set();release.wait(1);return original(payload)
        with patch.object(self.raw,'infer',side_effect=delayed),ThreadPoolExecutor(max_workers=1)as pool:
            pending=pool.submit(self.a.infer,request());self.assertTrue(entered.wait(1))
            with self.assertRaisesRegex(RuntimeError,'busy'):self.b.pin('race',auto_renew=False)
            with self.assertRaisesRegex(RuntimeError,'in flight'):self.rpc(self.b,'/unload',{})
            release.set();pending.result(timeout=2)

    def test_slow_model_loading_does_not_block_status_or_session_rejection(self):
        entered=threading.Event();release=threading.Event();original=self.model.prepare
        def slow():entered.set();release.wait(2);return original()
        self.raw._set(loaded=False,status='not_loaded')
        with patch.object(self.model,'prepare',side_effect=slow),ThreadPoolExecutor(max_workers=2)as pool:
            pending=pool.submit(self.a.prepare)
            try:
                self.assertTrue(entered.wait(1))
                status=pool.submit(self.b.status).result(timeout=.4)
                self.assertEqual(status['status'],'loading');self.assertTrue(status['busy'])
                with self.assertRaisesRegex(RuntimeError,'busy'):
                    self.rpc(self.b,'/prepare',dict(session_protocol=PROTOCOL,operation='acquire',session_id='racing',binding=binding(status)))
            finally:release.set()
            pending.result(timeout=2)

    def test_actual_diagnostic_route_is_guarded_and_pins_wait_for_queued_work(self):
        calls=[]
        def diagnostic(kind,body):self.raw.active='queued';calls.append(kind);return {'job_id':'cpu'}
        self.raw.tool_start=diagnostic
        self.raw.tool_catalog=lambda:dict(models=[dict(selectable=True)],sources=[dict(selectable=True)],busy=False)
        a=self.pin(self.a,'workbench')
        with self.assertRaisesRegex(RuntimeError,'pinned'):self.rpc(self.b,'/v1/infer',{})
        self.assertFalse(self.service.tool_catalog()['models'][0]['selectable']);self.assertEqual(calls,[])
        a.close()
        from urllib.request import Request,urlopen
        body=Request('http://127.0.0.1:'+str(self.server.server_port)+'/v1/conversions',data=b'{}',
            headers={'Authorization':'Bearer '+self.token_file.read_text(),'Content-Type':'application/json'})
        with urlopen(body,timeout=2)as response:
            self.assertEqual(response.status,202);self.assertEqual(json.load(response)['job_id'],'cpu')
        with self.assertRaisesRegex(RuntimeError,'busy'):self.b.pin('while-queued',auto_renew=False)
        for operation in (lambda:self.a.prepare(),lambda:self.rpc(self.a,'/unload',{}),lambda:self.b.infer(request())):
            with self.assertRaisesRegex(RuntimeError,'busy'):operation()

    def test_heartbeat_failure_latches_and_close_joins_without_worker_unload(self):
        self.service.ttl=.15;a=self.pin(self.a,'heartbeat',renew=True);initial=a.renewed_at
        deadline=time.monotonic()+1.
        while a.renewed_at==initial and time.monotonic()<deadline:time.sleep(.005)
        self.assertGreater(a.renewed_at,initial)
        self.raw.instance_id='changed';deadline=time.monotonic()+1.
        while a.error is None and time.monotonic()<deadline:time.sleep(.005)
        self.assertFalse(a.healthy());self.assertIsNotNone(a.error)
        a.close();self.assertFalse(a._thread.is_alive());self.assertEqual(self.model.closes,0)

    def test_chunk_freezes_observation_and_declares_unmapped_model_values(self):
        a=self.pin(self.a,'consumer');q=request('chunk');original=self.model.infer
        def mutate_caller(value):
            q['observation']['source_time']=999.;q['observation']['device_state14'][:]=999.
            q['request_id']='new-observation'
            return original(value)
        self.model.infer=mutate_caller
        chunk=a.infer_chunk(q,epoch=4,clock_domain='robot-boot-1',valid_until_source_time=3.)
        self.assertEqual(chunk['request_id'],'chunk');self.assertEqual(chunk['epoch'],4)
        self.assertEqual(chunk['observation']['source_time'],2.)
        self.assertEqual(chunk['validity']['valid_until_source_time'],3.)
        self.assertEqual(chunk['model_binding'],binding(self.a.status()))
        self.assertEqual(chunk['action_contract']['units'],(['rad']*6+['m'])*2)
        self.assertIsNone(chunk['action_contract']['sampling']['first_sample_offset_s'])
        self.assertFalse(chunk['action_contract']['device_mapping_applied'])
        self.assertFalse(chunk['execution_authorized'])
        np.testing.assert_array_equal(chunk['action14'],np.full((60,14),7,np.float32))
        np.testing.assert_allclose(self.model.requests[-1]['observation']['model_state14'],.1)
        self.assertNotIn(a.token,json.dumps({k:v for k,v in chunk.items()if k!='action14'}))

    def test_chunk_validation_never_calls_model_and_old_infer_needs_no_new_fields(self):
        a=self.pin(self.a,'consumer');count=len(self.model.requests)
        options=dict(epoch=0,clock_domain='robot',valid_until_source_time=3.)
        for change in ({'epoch':True},{'epoch':-1},{'clock_domain':''},
                       {'valid_until_source_time':float('nan')},{'valid_until_source_time':2.}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                a.infer_chunk(request(),**(options|change))
        a.action_contract=None  # A prior v1 server may still serve legacy infer.
        with self.assertRaisesRegex(ValueError,'contract'):a.infer_chunk(request(),**options)
        self.assertEqual(len(self.model.requests),count)
        self.assertEqual(set(a.infer(request())),{'request_id','action14','audit'})

    def test_manual_renew_failure_is_latched_and_network_time_does_not_extend_ttl(self):
        a=self.pin(self.a,'manual')
        clock=[123.];rpc=a._rpc
        def slow(*args,**kwargs):
            result=rpc(*args,**kwargs);clock[0]+=5.;return result
        with patch('robot_workbench.model_session.time.monotonic',side_effect=lambda:clock[0]),\
             patch.object(a,'_rpc',side_effect=slow):a.renew()
        self.assertEqual(a.renewed_at,123.)
        self.raw.instance_id='changed'
        with self.assertRaisesRegex(RuntimeError,'changed'):a.renew()
        self.assertFalse(a.healthy());self.assertIsNotNone(a.error)
        with self.assertRaisesRegex(ValueError,'explicitly acquire'):a.renew()

    def test_reacquisition_cannot_accept_response_from_previous_pin(self):
        a=self.pin(self.a,'reused-client')
        def late(value):
            a.close();a.acquire()
            return dict(request_id=value['request_id'],action14=np.zeros((60,14)),audit={})
        with patch.object(self.a,'infer',side_effect=late):
            with self.assertRaisesRegex(ValueError,'replaced'):a.infer(request('old-result'))
        self.assertTrue(a.healthy())

    def test_public_imports_and_cli_help_do_not_import_ui_control_or_model_frameworks(self):
        code='''
import importlib.abc, runpy, sys
class Deny(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('torch','triton','rclpy','arx_env','pyrealsense2') or fullname in (
            'robot_workbench.service','robot_workbench.inference_workflow','robot_workbench.control_owner',
            'robot_workbench.sdk_process','robot_workbench.mixed366_control'):
            raise AssertionError('forbidden import: '+fullname)
sys.meta_path.insert(0,Deny())
from robot_workbench.inference_transport import RemoteBaseClient, ModelDomainAdapter, ModelService
from robot_workbench.model_session import PinnedModelClient
from robot_workbench.mixed366 import Service
for script in ('serve_inference_model','serve_base566_model','serve_mixed366_model'):
    sys.argv=[script,'--help']
    try: runpy.run_path('scripts/'+script+'.py',run_name='__main__')
    except SystemExit as e: assert e.code == 0
'''
        result=subprocess.run([sys.executable,'-B','-c',code],cwd=ROOT,capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_model_contract_distinguishes_mixed_native_and_base_training_coordinates(self):
        from robot_workbench.inference_model import action_contract,MIXED366_ID,BASE566_ID
        from robot_workbench.mixed366 import descriptor
        mixed=action_contract(MIXED366_ID,descriptor(dict(bundle_sha256='a'*64,
            source_manifest_sha256='b'*64,runtime_sha256='c'*64)))
        base=action_contract(BASE566_ID)
        self.assertEqual(mixed['coordinate_domain'],'recorded_native_state14')
        self.assertEqual(mixed['jaw_semantics'],'recorder_endpoint_coordinate')
        self.assertEqual(mixed['history_steps'],0)
        self.assertEqual(base['jaw_semantics'],'continuous_sum_two_finger_travel')
        self.assertEqual(base['history_steps'],60)

    def test_mixed_chunk_public_model_domain_client_preserves_values_for_both_backends(self):
        from robot_workbench.inference_model import MIXED366_ID,worker_identity,CAMS
        from robot_workbench.inference_transport import ModelDomainAdapter
        from robot_workbench.mixed366 import descriptor
        for backend in ('torch','triton'):
            with self.subTest(backend=backend):
                bundle=descriptor(dict(bundle_sha256='a'*64,source_manifest_sha256='b'*64,
                    runtime_sha256='c'*64),backend,'d'*64 if backend=='triton'else None)
                model=StubModel();model.model_id=MIXED366_ID;model.bundle=bundle
                identity=worker_identity(MIXED366_ID,bundle)
                if backend=='triton':identity['packed_sha256']='e'*64
                model.prepare=lambda:identity
                raw=ModelService(model);server=serve_model(raw,token_file=self.token_file)
                thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start();pin=None
                try:
                    client=RemoteBaseClient('http://127.0.0.1:'+str(server.server_port),token_file=self.token_file,
                        adapter=ModelDomainAdapter(),conversion_identity={'domain':'recorded_native_state14'},
                        model_id=MIXED366_ID,model_bundle=bundle,task='lid_handoff')
                    client.prepare();pin=client.pin('model-domain',auto_renew=False)
                    q=dict(request_id='mixed-chunk',seed=7,applied_history=np.empty((0,14),np.float32),
                        observation=dict(source_time=2.,model_state14=np.arange(14,dtype=np.float32),
                            images={c:np.zeros((480,640,3),np.uint8)for c in CAMS}))
                    chunk=pin.infer_chunk(q,epoch=0,clock_domain='recording',valid_until_source_time=3.)
                    self.assertEqual(chunk['task'],'lid_handoff')
                    self.assertEqual(chunk['model_binding']['backend'],backend)
                    self.assertEqual(chunk['action_contract']['coordinate_domain'],'recorded_native_state14')
                    np.testing.assert_array_equal(model.requests[-1]['observation']['model_state14'],q['observation']['model_state14'])
                    np.testing.assert_array_equal(chunk['action14'],np.full((60,14),7,np.float32))
                finally:
                    if pin is not None:pin.close()
                    server.shutdown();server.server_close();thread.join(2);server.model_service.close()


if __name__=='__main__':unittest.main()
