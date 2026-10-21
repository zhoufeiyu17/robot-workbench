"""Maintenance races against the real CPU Owner; no SDK, camera process or weights."""
from copy import deepcopy
from http.client import HTTPConnection
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]
import test_unified_runtime as runtime_tests
from robot_workbench.service import Application,Device,serve
from robot_workbench.inference_workflow import InferenceWorkflow


class Flow:
    pause_observations=InferenceWorkflow.pause_observations
    resume_observations=InferenceWorkflow.resume_observations
    _start_cache=InferenceWorkflow._start_cache
    def __init__(self):
        self.generation=0;self.lock=threading.RLock();self.cache_stop=threading.Event()
        self.cache_thread=None;self.cache_paused=False;self.busy=False
        self.prepared={'fixture':True};self.cache={};self.support_cache={};self.cache_error=None
        self.model_identity={'model':'CPU fixture'};self.calls=[]
        self.model_client=SimpleNamespace(status=lambda:dict(status='ready',loaded=True,
            warmup=dict(completed=True,wall_s=.01,output_sha256='a'*64,input='CPU fixture')))
    def snapshot(self):return dict(busy=self.busy,primary_action=None,inference={'model_ready':True})
    def results(self):return []
    def invalidate(self):self.generation+=1
    cancel=invalidate
    def _revoke_device_review(self):self.calls.append('revoke')
    def operations_prepare_model(self,*,still_current,**kwargs):
        if not still_current():raise ValueError('maintenance revoked')
        self.calls.append(('prepare_model',kwargs));status=self.model_client.status()
        if not (status.get('warmup') or {}).get('completed'):raise ValueError('模型缺少预热证据')
        return dict(model_ready=True,model_status=status['status'],warmup=status['warmup'])
    def _cache_loop(self):self.cache_stop.wait()


class MaintenanceTests(unittest.TestCase):
    def setUp(self):
        self.harness=runtime_tests.UnifiedRuntimeTest();self.harness.setUp();self.addCleanup(self.harness.doCleanups)
        runtime,creds=self.harness.create();creds['inference']=runtime.register_business('inference')
        self.runtime,self.creds=runtime,creds
        self.device=Device('robot','fake',Mock(side_effect=AssertionError('maintenance must not connect SDK')))
        self.device.runtime=runtime;self.device.credentials=creds
        self.flow=Flow();self.device.workflows['inference']=self.flow
        self.addCleanup(self.flow.pause_observations)
        self.app=Application({'fake':self.device});self.number=0
    def action(self,action,**fields):
        self.number+=1
        return self.app.action(action,dict(environment='fake',business='calibration',
            request_id=fields.pop('request_id','request-'+str(self.number)),**fields))
    def begin(self,**fields):return self.action('ops_maintenance_begin',purpose=fields.pop('purpose','prepare_environment'),**fields)
    def end(self,token,**fields):return self.action('ops_maintenance_end',maintenance_token=token,**fields)
    def thread(self,function):
        results=[]
        def call():
            try:results.append(function())
            except Exception as error:results.append(error)
        thread=threading.Thread(target=call,daemon=True);thread.start()
        self.addCleanup(thread.join,2)
        return thread,results

    def test_collection_maintenance_uses_live_busy_in_both_view_cache_races(self):
        from robot_workbench.maintenance import busy_reason
        from robot_workbench.workflows import CollectionWorkflow
        flow=CollectionWorkflow.__new__(CollectionWorkflow)
        flow.lock=threading.Lock()
        state=dict(busy=False,warmup={'status':'ready'})
        flow.playground=SimpleNamespace(lock=threading.Lock(),state=state,
            initialization=lambda:{'status':'ready'})
        self.device.workflows['collection']=flow
        for cached,actual in ((True,False),(False,True)):
            with self.subTest(cached=cached,actual=actual):
                flow.value={'busy':cached};state['busy']=actual
                self.assertEqual(busy_reason(self.device),
                    '业务仍在执行或收尾，请先停止任务并等待完成' if actual else None)
        state['busy']=False;self.flow.busy=True
        self.assertIsNotNone(busy_reason(self.device),'other workflow guards still apply')

    def test_begin_is_idempotent_private_and_blocks_both_admission_layers(self):
        preview=self.harness.preview(self.runtime,self.creds['calibration'])
        receipt=self.begin(request_id='claim');token=receipt['token']
        self.assertEqual(receipt['status'],'ready');self.assertEqual(self.begin(request_id='claim'),receipt)
        self.assertTrue(self.device.maintenance);self.assertTrue(self.runtime.maintenance_active)
        self.assertFalse(self.runtime.owner.pending)
        public=self.app.state('fake','calibration')
        self.assertNotIn(token,str(public));self.assertNotIn('token',public['maintenance'])
        self.assertTrue(public['runtime']['allowed_operations']['stop']['allowed'])
        self.assertTrue(all(not gate['allowed'] for name,gate in public['runtime']['allowed_operations'].items() if name!='stop'))
        for action in ('connect','inference_run','execution_settings','manual_preview'):
            with self.subTest(action=action),self.assertRaisesRegex(ValueError,'维护'):self.action(action)
        with self.assertRaisesRegex(ValueError,'维护'):self.device.connect({})
        with self.assertRaisesRegex(ValueError,'维护'):self.harness.preview(self.runtime,self.creds['calibration'])
        with self.assertRaisesRegex(ValueError,'维护'):self.harness.confirm(self.runtime,self.creds['calibration'],preview)
        with self.assertRaisesRegex(ValueError,'维护'):self.runtime.request_release(self.creds['calibration'])
        before=deepcopy(self.runtime.owner.backend.events)
        self.assertEqual(self.end(token),{'ended':True,'motion_resumed':False,'target':receipt['target']})
        self.assertFalse(self.device.maintenance);self.assertFalse(self.runtime.maintenance_active)
        self.assertEqual(self.runtime.owner.backend.events,before)
        self.assertEqual(self.begin(request_id='claim')['status'],'ended')
        self.assertFalse(self.app.maintenance,'replaying an ended begin must not claim again')

    def test_service_ensure_keeps_collection_plan_but_revokes_execution_review(self):
        from robot_workbench.maintenance import ControlActor
        flow=SimpleNamespace(snapshot=lambda:dict(busy=False),invalidate=Mock())
        self.device.workflows['collection']=flow
        self.app.reviews[('fake','collection')]={'id':'old-review'}
        receipt=self.begin(purpose='prepare_collection')
        self.assertEqual(receipt['status'],'ready');flow.invalidate.assert_not_called()
        self.assertNotIn(('fake','collection'),self.app.reviews)
        actor=ControlActor(self.app,self.device,self.app.maintenance['fake'],'ops_prepare_collection')
        with self.assertRaisesRegex(ValueError,'未知数采生命周期'):actor.lifecycle_admission('warmup:collection')
        self.end(receipt['token'])

    def test_active_claim_can_be_recovered_after_request_cache_eviction(self):
        receipt=self.begin(request_id='claim');self.app.requests.clear()
        self.assertEqual(self.begin(request_id='claim'),receipt)
        with self.assertRaisesRegex(ValueError,'不同操作'):
            self.action('ops_maintenance_begin',request_id='claim',purpose='restart_cameras')
        self.end(receipt['token'])

    def test_cache_join_does_not_block_stop_state_or_duplicate_begin(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        def blocked_cache():entered.set();release.wait(2)
        self.flow._cache_loop=blocked_cache;self.flow._start_cache();self.assertTrue(entered.wait(1))
        thread,result=self.thread(lambda:self.begin(request_id='claim'))
        deadline=time.monotonic()+1
        while not self.flow.cache_stop.is_set() and time.monotonic()<deadline:time.sleep(.001)
        duplicate=self.begin(request_id='claim');self.assertEqual(duplicate['status'],'starting')
        started=time.monotonic();self.action('stop');self.assertLess(time.monotonic()-started,.2)
        self.assertTrue(self.runtime.owner.stop_requested)
        self.assertEqual(self.app.state('fake','calibration')['maintenance']['status'],'starting')
        with self.assertRaisesRegex(ValueError,'维护'):self.action('inference_run')
        release.set();thread.join(1);self.assertFalse(thread.is_alive());self.assertEqual(result[0]['token'],duplicate['token'])
        self.end(duplicate['token'])

    def test_stop_does_not_wait_for_workflow_invalidation(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        self.flow.invalidate=lambda:(entered.set(),release.wait(2))
        receipt=[];thread,result=self.thread(lambda:receipt.append(self.begin()))
        self.assertTrue(entered.wait(1))
        started=time.monotonic();self.action('stop');self.assertLess(time.monotonic()-started,.2)
        self.assertTrue(self.runtime.owner.stop_requested)
        release.set();thread.join(1);self.end(receipt[0]['token'])

    def test_running_request_and_async_job_prevent_claim(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        def blocked(*args,**kwargs):entered.set();release.wait(2);return {}
        with patch.object(self.app,'_action',side_effect=blocked):
            thread,result=self.thread(lambda:self.action('select_result'))
            self.assertTrue(entered.wait(1))
            with self.assertRaisesRegex(ValueError,'请求仍在处理'):self.begin()
            release.set();thread.join(1)
        self.assertEqual(self.app.action_calls['fake'],0)
        entered.clear();release.clear()
        with patch.object(self.app,'perform',side_effect=blocked):
            self.app.action('select_result',
                dict(environment='fake',business='inference',request_id='async'))
            self.assertTrue(entered.wait(1))
            with self.assertRaisesRegex(ValueError,'请求仍在处理'):self.begin()
            release.set()
            deadline=time.monotonic()+1
            while self.app.jobs[('fake','inference')]['status']=='running' and time.monotonic()<deadline:time.sleep(.001)
        self.assertFalse(self.app.maintenance)

    def test_queued_confirmation_and_connection_cleanup_prevent_claim(self):
        preview=self.harness.preview(self.runtime,self.creds['calibration'])
        self.harness.confirm(self.runtime,self.creds['calibration'],preview)
        with self.assertRaisesRegex(ValueError,'排队'):self.begin()
        self.assertFalse(self.device.maintenance);self.assertFalse(self.runtime.maintenance_active)
        self.runtime.submit(self.creds['calibration'],{'action':'stop'});self.harness.advance(self.runtime)
        self.device.runtime=None;self.device.thread=SimpleNamespace(is_alive=lambda:True)
        with self.assertRaisesRegex(ValueError,'收尾'):self.begin()
        self.device.runtime=self.runtime;self.device.thread=None

    def test_recording_writer_must_finish_before_claim(self):
        done=threading.Event();self.runtime.owner.recording=SimpleNamespace(done=done,
            thread=SimpleNamespace(is_alive=lambda:True))
        with self.assertRaisesRegex(ValueError,'录制'):self.begin()
        done.set()
        with self.assertRaisesRegex(ValueError,'录制'):self.begin()
        self.assertFalse(self.runtime.maintenance_active);self.assertFalse(self.device.maintenance)
        self.runtime.owner.recording=None

    def test_in_flight_sdk_call_rejects_begin_but_stop_is_independent(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        self.runtime.owner.backend.read_delay=lambda:(entered.set(),release.wait(2))
        tick,result=self.thread(lambda:self.harness.advance(self.runtime));self.assertTrue(entered.wait(1))
        begin,answer=self.thread(self.begin)
        started=time.monotonic();self.action('stop');self.assertLess(time.monotonic()-started,.2)
        begin.join(.5);self.assertFalse(begin.is_alive());self.assertIsInstance(answer[0],ValueError)
        self.assertFalse(self.device.maintenance);release.set();tick.join(1)
        self.runtime.owner.backend.read_delay=lambda:None

    def test_preview_compiling_before_maintenance_cannot_publish_after_end(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        original=self.runtime.ref.Owner.submit
        def delayed(owner,command):
            result=original(owner,command)
            if owner is not self.runtime.owner:entered.set();release.wait(2)
            return result
        with patch.object(self.runtime.ref.Owner,'submit',new=delayed):
            thread,result=self.thread(lambda:self.harness.preview(self.runtime,self.creds['calibration']))
            self.assertTrue(entered.wait(1));receipt=self.begin();self.end(receipt['token'])
            release.set();thread.join(1)
        self.assertIsInstance(result[0],ValueError);self.assertIn('revoked',str(result[0]));self.assertFalse(self.runtime.owner.pending)

    def test_rebind_preserves_camera_object_and_model_stop_stays_available(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        camera=SimpleNamespace(reconnect=lambda **kwargs:(self.assertEqual(kwargs,{'allow_current':True}),
            entered.set(),release.wait(2),{'rebound':True})[-1])
        self.device.camera_service=camera;identity=self.flow.model_identity;token=self.begin()['token']
        thread,result=self.thread(lambda:self.action('ops_rebind_cameras',maintenance_token=token,request_id='rebind'))
        self.assertTrue(entered.wait(1));self.action('stop')
        with self.assertRaisesRegex(ValueError,'尚未返回'):self.end(token)
        release.set();thread.join(1)
        self.assertIsInstance(result[0],ValueError);self.assertIn('Stop',str(result[0]));self.assertIs(self.device.camera_service,camera)
        self.assertIs(self.flow.model_identity,identity);self.assertIsNone(self.flow.prepared);self.assertIsNone(self.flow.cache)
        with self.assertRaisesRegex(ValueError,'Stop'):
            self.action('ops_rebind_cameras',maintenance_token=token,request_id='rebind')
        self.end(token);self.assertIsNone(self.flow.cache_thread)

    def test_same_service_rebind_verifies_real_cpu_frames_without_replacing_binding(self):
        import test_camera_rebind as camera_tests
        fixture=camera_tests.CameraReconnectTest();self.addCleanup(fixture.doCleanups)
        source,cameras,_=fixture.attached('fake');self.device.camera_service=cameras
        frames,channels=cameras.frames,cameras.channels;token=self.begin()['token']
        for _ in range(2):
            receipt=self.action('ops_rebind_cameras',maintenance_token=token)
            self.assertTrue(receipt['already_bound']);self.assertFalse(receipt['reconnected'])
            self.assertEqual(len(receipt['cameras']),3);self.assertEqual(receipt['generation'],0)
            self.assertTrue(all(row['rgb_bytes']==12 and row['depth_bytes']==8 for row in receipt['cameras'].values()))
        self.assertIs(self.device.camera_service,cameras);self.assertIs(cameras.frames,frames);self.assertIs(cameras.channels,channels)
        self.assertIsNone(source.process.poll());self.end(token)

    def test_failed_operation_and_failed_end_keep_gate_until_explicit_success(self):
        token=self.begin(request_id='failure-claim')['token'];camera=SimpleNamespace(reconnect=Mock(side_effect=ValueError('fixture failure')),
            reconnect_state=lambda:{'active_calls':0});self.device.camera_service=camera
        for _ in range(2):
            with self.assertRaisesRegex(ValueError,'fixture failure'):
                self.action('ops_rebind_cameras',maintenance_token=token,request_id='fail')
        self.assertEqual(camera.reconnect.call_count,1);self.assertTrue(self.runtime.maintenance_active)
        self.assertEqual(self.begin(request_id='failure-claim')['status'],'failed')
        with patch.object(self.flow,'resume_observations',side_effect=ValueError('still draining')):
            with self.assertRaisesRegex(ValueError,'still draining'):self.end(token)
        self.assertTrue(self.device.maintenance);self.assertEqual(self.app.state('fake','calibration')['maintenance']['status'],'failed')
        self.end(token);self.assertFalse(self.runtime.maintenance_active)

    def test_failed_cache_pause_returns_claim_for_explicit_end_only(self):
        with patch.object(self.flow,'pause_observations',side_effect=ValueError('cache stuck')):receipt=self.begin()
        self.assertEqual(receipt['status'],'failed');self.assertIn('cache stuck',receipt['error'])
        self.assertTrue(self.device.maintenance);self.assertTrue(self.runtime.maintenance_active)
        with self.assertRaisesRegex(ValueError,'尚未就绪'):self.action('ops_prepare_model',maintenance_token=receipt['token'])
        self.end(receipt['token']);self.assertFalse(self.device.maintenance)

    def test_camera_callbacks_and_model_prepare_are_explicit_and_parameters_are_strict(self):
        token=self.begin()['token'];self.device.camera_service=object()
        calls=[]
        self.device.camera_start=lambda progress:(calls.append('start'),progress('starting'),{'started':True})[-1]
        self.device.camera_restart=lambda progress:(calls.append('restart'),progress('restarting'),{'restarted':True})[-1]
        self.action('ops_start_cameras',maintenance_token=token)
        with self.assertRaisesRegex(ValueError,'作用域'):self.action('ops_restart_cameras',maintenance_token=token)
        result=self.action('ops_prepare_model',maintenance_token=token)
        self.assertEqual(result['model_status'],'ready');self.assertEqual(result['warmup']['output_sha256'],'a'*64)
        self.assertEqual(calls,['start']);self.assertIn(('prepare_model',{}),self.flow.calls)
        self.device.factory.assert_not_called()
        with self.assertRaisesRegex(ValueError,'未知字段'):self.action('ops_prepare_model',maintenance_token=token,extra=True)
        self.action('ops_prepare_model',maintenance_token=token,expected_model_id='cpu-model')
        self.assertIn(('prepare_model',{'expected_model_id':'cpu-model'}),self.flow.calls)
        with self.assertRaisesRegex(ValueError,'原模型ID'):self.action('ops_prepare_model',maintenance_token=token,expected_model_id=None)
        with self.assertRaisesRegex(ValueError,'身份已失效'):self.end('wrong')
        self.end(token)

    def test_supported_release_requires_valid_maintenance_token(self):
        token=self.begin()['token']
        with self.assertRaisesRegex(ValueError,'维护'):self.action('release')
        with self.assertRaisesRegex(ValueError,'作用域'):self.action('release',maintenance_token=token)
        self.assertIsNone(self.runtime.release_receipt)
        self.assertTrue(self.runtime.maintenance_active)
        self.end(token)

    def test_http_requires_csrf_and_exposes_only_public_maintenance_state(self):
        server=serve(self.app,port=0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        def request(method,path,value=None,csrf=True):
            connection=HTTPConnection('127.0.0.1',server.server_port,timeout=1)
            headers={'Content-Type':'application/json'}
            if csrf:headers['X-CSRF-Token']=self.app.csrf
            try:
                connection.request(method,path,body=json.dumps(value) if value is not None else None,headers=headers)
                response=connection.getresponse();return response.status,json.loads(response.read())
            finally:connection.close()
        body=dict(environment='fake',business='calibration',request_id='http-claim',purpose='prepare_model')
        try:
            self.assertEqual(request('POST','/api/actions/ops_maintenance_begin',body,csrf=False)[0],403)
            code,receipt=request('POST','/api/actions/ops_maintenance_begin',body)
            self.assertEqual(code,200);token=receipt['token']
            self.assertEqual(request('POST','/api/actions/ops_maintenance_begin',body),(200,receipt))
            code,state=request('GET','/api/state?environment=fake&business=calibration')
            self.assertEqual(code,200);self.assertNotIn(token,json.dumps(state))
            ordinary=dict(environment='fake',business='calibration',request_id='http-connect')
            self.assertEqual(request('POST','/api/actions/connect',ordinary)[0],409)
            self.assertEqual(request('POST','/api/actions/stop',dict(ordinary,request_id='http-stop'))[0],200)
            self.assertEqual(request('POST','/api/actions/ops_maintenance_end',
                dict(ordinary,request_id='http-end',maintenance_token=token))[0],200)
        finally:server.shutdown();server.server_close();thread.join(1)

    def test_model_warmup_works_while_disconnected_without_sdk_factory(self):
        self.device.runtime=None;self.device.credentials={}
        token=self.begin()['token']
        self.action('ops_prepare_model',maintenance_token=token)
        self.assertIn(('prepare_model',{}),self.flow.calls)
        self.device.factory.assert_not_called();self.assertIsNone(self.device.runtime)
        self.end(token)

    def test_model_prepare_without_verified_warmup_keeps_maintenance_failed(self):
        token=self.begin()['token'];self.flow.model_client.status=lambda:dict(status='ready',loaded=True)
        with self.assertRaisesRegex(ValueError,'预热证据'):self.action('ops_prepare_model',maintenance_token=token)
        self.assertEqual(self.app.maintenance['fake']['status'],'failed');self.assertTrue(self.runtime.maintenance_active)
        self.end(token)


if __name__=='__main__':unittest.main()
