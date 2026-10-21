"""Restricted operations against the real CPU Owner and loopback HTTP server."""
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

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]
from test_http_service import HTTPTest
from test_unified_runtime import UnifiedRuntimeTest, jaw
from robot_workbench.maintenance import target
from robot_workbench.management import hold_verified
from robot_workbench.service import Application,Device


class OperationFixture:
    def body(self,**fields):
        self.number+=1
        return dict(environment='fake',request_id='operation-'+str(self.number),
            expected_target=target(self.app,self.device),**fields)
    def operation(self,action,**fields):return self.app.maintenance_action(action,self.body(**fields))
    def begin(self,purpose):return self.operation('begin',purpose=purpose)
    def start(self,function):
        results=[]
        def run():
            try:results.append(function())
            except Exception as error:results.append(error)
        thread=threading.Thread(target=run,daemon=True);thread.start()
        self.addCleanup(thread.join,2)
        return thread,results


class ControlOperationsTests(OperationFixture,unittest.TestCase):
    def test_gain_diagnostics_are_cached_between_transitions(self):
        owner=self.runtime.owner;owner.local_gains_key=None
        feedback=deepcopy(owner.cache['calibration_feedback']);limits=deepcopy(owner.cache['joint_limits'])
        with patch.object(owner.backend._backend,'read_gains',wraps=owner.backend._backend.read_gains) as read:
            for _ in range(40):owner.cache_state(feedback,limits)
            self.assertEqual(read.call_count,1)
            owner.epoch+=1;owner.cache_state(feedback,limits)
            self.assertEqual(read.call_count,2,'a control transition must refresh actual gains')
        self.assertIsNotNone(owner.cache['local_gains_sample_wall_ns'])

    def test_recovery_waits_for_published_state_after_live_state_changes(self):
        from arx_env.joint_head_real_motion import MotionTrackingError
        from robot_workbench.management import WorkbenchManager
        owner=self.runtime.owner
        owner.motion_fault(MotionTrackingError('injected stale recovery publication'));self.harness.advance(self.runtime)
        window=self.begin('recover_control');entered=threading.Event();release=threading.Event()
        self.addCleanup(release.set);original=owner.cache_state
        def pause_publication(feedback,limits):
            if owner.control.state is owner.states.STOPPED and 'FAULT'in owner.cache['state']:
                entered.set();release.wait(1)
            return original(feedback,limits)
        owner.cache_state=pause_publication
        worker,result=self.start(lambda:self.operation('recover_control',maintenance_token=window['token']))
        deadline=time.monotonic()+1
        while owner.commands.empty()and time.monotonic()<deadline:time.sleep(.001)
        tick=threading.Thread(target=lambda:self.harness.advance(self.runtime));tick.start()
        self.assertTrue(entered.wait(1));worker.join(.03)
        self.assertTrue(worker.is_alive(),'recovery accepted the old FAULT cache')
        self.assertFalse(WorkbenchManager.control_healthy(self.runtime.fact_snapshot()))
        release.set();tick.join(1);self.harness.advance(self.runtime,2);worker.join(1)
        self.assertFalse(worker.is_alive());self.assertIsInstance(result[0],dict,str(result[0]))
        self.assertTrue(WorkbenchManager.control_healthy(result[0]['runtime']))
        self.assertEqual(result[0]['runtime']['state'],'STOPPED_DAMPING_HOLD')
        self.operation('end',maintenance_token=window['token'])

    def setUp(self):
        self.harness=UnifiedRuntimeTest();self.harness.setUp();self.addCleanup(self.harness.doCleanups)
        self.runtime,self.credentials=self.harness.create()
        self.device=Device('robot','fake',Mock(side_effect=AssertionError('must not auto-connect')))
        self.device.runtime=self.runtime;self.device.credentials=self.credentials
        self.app=Application({'fake':self.device});self.number=0

    def test_remote_business_observer_does_not_block_confirmed_protected_reconnect(self):
        self.test_connect_retires_only_the_current_confirmed_protected_runtime(remote_observer=True)

    def test_connect_retires_only_the_current_confirmed_protected_runtime(self,remote_observer=False):
        from robot_workbench.maintenance import ControlActor
        previous=self.runtime;backend=previous.owner.backend._backend
        backend.terminal_failure='CPU fixture owner heartbeat lost';backend.is_connected=False
        confirmed=[False]
        backend.protection_snapshot=lambda:dict(event=dict(id='cpu-fault'),worker_exit_confirmed=confirmed[0])
        disposed=[];backend.dispose=lambda:disposed.append(True)
        self.assertFalse(previous.tick())
        self.device.environment='physical';self.device.maintenance=True
        self.device.drive=lambda runtime,closed:closed.wait()
        application=SimpleNamespace(lock=threading.RLock(),maintenance={},server_instance_id='cpu-server')
        window=dict(active_action='connect_control',_runtime=previous,_stop_serial=0)
        application.maintenance['physical']=window
        actor=ControlActor(application,self.device,window,'connect_control')
        if remote_observer:
            previous.remote_control_owner=True;previous.detach=Mock()
            monitor=threading.Thread(target=self.device.closed.wait);monitor.start();self.device.thread=monitor
            self.addCleanup(self.device.closed.set);self.addCleanup(monitor.join,2)
        def fresh():
            runtime=self.harness.create()[0]
            runtime.clients.clear()  # Device.connect owns registration in this test.
            return runtime
        self.device.factory=Mock(side_effect=fresh)
        with self.assertRaisesRegex(ValueError,'支撑'):self.device.connect({},maintenance_actor=actor)
        with self.assertRaisesRegex(ValueError,'退出或录制收尾尚未确认'):
            self.device.connect({'arms_supported':True},maintenance_actor=actor)
        self.device.factory.assert_not_called();self.assertIsNotNone(previous.fd)
        confirmed[0]=True;self.device.stop_serial+=1
        with self.assertRaisesRegex(ValueError,'Stop代次'):
            self.device.connect({'arms_supported':True},maintenance_actor=actor)
        self.device.factory.assert_not_called();self.device.stop_serial=0
        with patch.object(backend,'close',side_effect=AssertionError('not a normal SDK close')):
            self.device.connect({'arms_supported':True},maintenance_actor=actor)
        self.device.closed.set();self.device.thread.join(1)
        self.assertEqual(disposed,[True]);self.assertIsNone(previous.fd)
        self.assertIsNot(self.device.runtime,previous);self.assertTrue(actor.current())
        self.assertTrue(self.device.runtime.prior_control_retirement['worker_exit_confirmed'])
        self.assertFalse(self.device.runtime.prior_control_retirement['sdk_close_confirmed'])
        self.assertFalse(self.device.runtime.prior_control_retirement['motion_resumed'])
        if remote_observer:self.assertFalse(monitor.is_alive());previous.detach.assert_called_once()
        with self.assertRaises(PermissionError):previous.submit(self.credentials['calibration'],{'action':'stop'})
        with self.assertRaisesRegex(ValueError,'设备已连接'):
            self.device.connect({'arms_supported':True},maintenance_actor=actor)
        self.device.factory.assert_called_once()

    def drive(self,thread,limit=700):
        deadline=time.monotonic()+1
        while self.runtime._hold_verification is None and thread.is_alive() and time.monotonic()<deadline:time.sleep(.001)
        for _ in range(limit):
            self.harness.advance(self.runtime)
            if self.runtime._hold_verification is None:break
        thread.join(1);self.assertFalse(thread.is_alive())

    def test_fixed_hold_uses_nonzero_raw_pose_mapping_once_and_original_stop(self):
        from test_collection_physx_protocol import protocol_fixture
        self.runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
        self.harness.advance(self.runtime);self.runtime.close()
        _,_,_,_,mapping=protocol_fixture(Path(self.harness.directory.name))
        jaw.configure_mapping(mapping)
        q=[.2,-.1,.3,.4,-.2,.1,jaw.feedback_sdk('left',35),-.3,.2,-.1,.5,.1,-.2,jaw.feedback_sdk('right',45)]
        runtime,credentials=self.harness.create(q=q,mapping=mapping)
        self.runtime=runtime;self.device.runtime=runtime;self.device.credentials=credentials
        backend=runtime.owner.backend._backend;backend.passive_jaws=('right',)
        original_begin=backend.begin
        def resampled_begin(side,measured):
            actual=list(measured);actual[0 if side=='left' else 7]+=.002
            backend.q=actual[:]
            return original_begin(side,actual)
        backend.begin=resampled_begin
        runtime.execution_calibration={'zero_offsets_rad':[.4]*12}
        window=self.begin('verify_hold');before=len(runtime.owner.backend.events)
        worker,result=self.start(lambda:self.operation('verify_hold',maintenance_token=window['token']))
        self.drive(worker)
        self.assertIsInstance(result[0],dict,str(result[0]));receipt=result[0]
        self.assertEqual(receipt['status'],'completed');self.assertTrue(receipt['verified']);self.assertEqual(receipt['duration_s'],5.)
        self.assertGreaterEqual(receipt['held_source_s'],5.);self.assertLess(receipt['held_source_s'],5.02)
        self.assertGreaterEqual(receipt['sample_count'],500)
        self.assertEqual(receipt['initial_feedback']['qpos'],q)
        actual=q[:];actual[0]+=.002;actual[7]+=.002
        self.assertEqual(receipt['begin_feedback_qpos'],actual);self.assertFalse(receipt['joint_offsets_applied'])
        expected=actual[:];expected[6]=jaw.command_sdk('left',35);expected[13]=jaw.command_sdk('right',45)
        for a,b in zip(receipt['fixed_target'],expected):self.assertAlmostEqual(a,b)
        self.assertEqual(receipt['disabled_grippers'],['right'])
        self.assertEqual(receipt['jaw_command_clipped'],{'left':False,'right':False})
        for value,want in zip(receipt['jaw_command_mm'],[35,45]):self.assertAlmostEqual(value,want)
        sends=[event for event in runtime.owner.backend.events[before:] if event[0]=='send']
        self.assertGreater(len(sends),900);self.assertEqual(receipt['sdk_send_count'],len(sends))
        self.assertTrue(all(row[2]==receipt['fixed_target'] for row in sends))
        self.assertTrue(receipt['stop_receipt']['stop_hold_return_confirmed'])
        self.assertTrue(hold_verified(receipt),'manager must accept the actual Runtime receipt, not only its HTTP mock')
        changes=[('target_source','fresh_measured_physical_feedback'),('status','failed'),('verified',False),
            ('held_source_s',4.99),('joint_offsets_applied',True),('requested_target',[0.]*14),
            ('sample_count',receipt['sample_count']+1),('sdk_send_count',0),('sdk_send_count',receipt['sdk_send_count']+1),
            ('sdk_returns',[]),('stop_receipt',{'stop_hold_return_confirmed':False}),
            ('jaw_command_mm',[84.,45.]),('jaw_command_clipped',{'left':True,'right':False})]
        for name,value in changes:
            with self.subTest(tampered=name,value=value):
                invalid=deepcopy(receipt);invalid[name]=value;self.assertFalse(hold_verified(invalid))
        self.assertFalse(hold_verified(None));self.assertFalse(hold_verified({}))
        self.assertEqual(runtime.owner.control.state,runtime.owner.states.STOPPED)
        self.assertEqual(runtime.owner.runner.disabled_grippers,('right',))
        self.assertEqual(runtime.owner.runner.error_settings,runtime.owner.motion_error_settings())
        self.assertTrue(runtime.maintenance_active);self.assertEqual(runtime.holder,'calibration')
        snapshot=runtime.fact_snapshot()['control_operation']
        self.assertNotIn('samples',snapshot);self.assertNotIn('sdk_returns',snapshot)
        self.assertEqual(snapshot['sample_count'],receipt['sample_count'])
        self.operation('end',maintenance_token=window['token'])

    def test_stop_preempts_hold_and_preserves_partial_sdk_and_feedback_evidence(self):
        window=self.begin('verify_hold')
        worker,result=self.start(lambda:self.operation('verify_hold',maintenance_token=window['token']))
        deadline=time.monotonic()+1
        while self.runtime._hold_verification is None and time.monotonic()<deadline:time.sleep(.001)
        self.harness.advance(self.runtime,80)
        self.assertEqual(self.runtime.control_operation['status'],'holding')
        self.assertTrue(self.app.environment_state('fake')['activity']['busy'])
        self.app.action_calls['fake']=1
        started=time.monotonic();self.operation('stop');self.assertLess(time.monotonic()-started,.2)
        count=sum(row[0]=='send' for row in self.runtime.owner.backend.events)
        self.app.action_calls['fake']=0;self.drive(worker)
        self.assertIsInstance(result[0],ValueError)
        receipt=self.runtime.control_operation
        self.assertEqual(receipt['status'],'failed');self.assertFalse(receipt['verified']);self.assertGreater(receipt['sample_count'],50)
        self.assertEqual(sum(row[0]=='send' for row in self.runtime.owner.backend.events),count)
        self.assertTrue(receipt['stop_receipt']['stop_hold_return_confirmed'])
        self.assertFalse(hold_verified(receipt),'a stopped partial hold must not qualify')
        with self.assertRaisesRegex(ValueError,'Stop'):self.operation('verify_hold',maintenance_token=window['token'])
        self.operation('end',maintenance_token=window['token'])

    def test_hold_rejects_arbitrary_target_duration_and_stale_feedback_without_begin(self):
        window=self.begin('verify_hold');events=list(self.runtime.owner.backend.events)
        for extra in ({'duration_s':1},{'target':[0.]*14},{'q_target':[0.]*14}):
            with self.assertRaisesRegex(ValueError,'未知字段'):
                self.operation('verify_hold',maintenance_token=window['token'],**extra)
        self.runtime.owner.cache['calibration_feedback']['sample_time_s']-=.11
        with self.assertRaisesRegex(ValueError,'fresh healthy stationary'):
            self.operation('verify_hold',maintenance_token=window['token'])
        self.assertEqual(events,self.runtime.owner.backend.events)
        self.operation('end',maintenance_token=window['token'])

    def test_collection_admission_keeps_maintenance_and_restricts_lifecycle_callbacks(self):
        calls=[]
        def prepare(*,still_current,lifecycle_admission):
            self.assertEqual(self.runtime.holder,'collection');self.assertTrue(self.runtime.maintenance_active)
            self.assertTrue(self.device.maintenance);self.assertTrue(still_current())
            self.assertTrue(lifecycle_admission('start:detector'))
            with self.assertRaises(ValueError):lifecycle_admission('warmup:collection')
            self.assertTrue(lifecycle_admission('start:collection'))
            with self.assertRaises(ValueError):lifecycle_admission('start:other')
            calls.append('prepared');return dict(ready=True)
        self.device.collection_preparation=SimpleNamespace(snapshot=lambda:dict(busy=False),operations_prepare=prepare)
        window=self.begin('prepare_collection');before=list(self.runtime.owner.backend.events)
        self.assertEqual(self.operation('prepare_collection',maintenance_token=window['token']),{'ready':True})
        self.assertEqual(self.runtime.owner.backend.events,before);self.assertEqual(calls,['prepared'])
        self.device.factory.assert_not_called();self.operation('end',maintenance_token=window['token'])
        def cancelled_prepare(*,still_current,lifecycle_admission):
            self.operation('stop')
            self.assertFalse(still_current());self.assertFalse(lifecycle_admission('start:collection'))
            with self.assertRaises(ValueError):lifecycle_admission('warmup:collection')
            self.assertTrue(lifecycle_admission('stop:detector'))
            self.assertTrue(lifecycle_admission('stop:planner'));self.assertTrue(lifecycle_admission('stop:scene'))
            return {'ready':False}
        self.device.collection_preparation.operations_prepare=cancelled_prepare
        window=self.begin('prepare_collection')
        with self.assertRaisesRegex(ValueError,'Stop'):self.operation('prepare_collection',maintenance_token=window['token'])
        self.harness.advance(self.runtime);self.operation('end',maintenance_token=window['token'])

    def test_recover_acknowledges_only_original_recoverable_fault_without_sdk_or_reconnect(self):
        from arx_env.joint_head_real_motion import MotionTrackingError
        self.runtime.owner.motion_fault(MotionTrackingError('tracking fixture'));self.harness.advance(self.runtime)
        window=self.begin('recover_control');before=list(self.runtime.owner.backend.events)
        worker,result=self.start(lambda:self.operation('recover_control',maintenance_token=window['token']))
        deadline=time.monotonic()+1
        while self.runtime.owner.commands.empty() and worker.is_alive() and time.monotonic()<deadline:time.sleep(.001)
        self.harness.advance(self.runtime,2);worker.join(1)
        self.assertFalse(worker.is_alive());self.assertIsInstance(result[0],dict,str(result[0]))
        self.assertTrue(result[0]['recovered']);self.assertFalse(result[0]['motion_resumed'])
        self.assertEqual(self.runtime.owner.backend.events,before);self.device.factory.assert_not_called()
        self.operation('end',maintenance_token=window['token'])
        self.runtime.owner.motion_fault(RuntimeError('native SDK failure'));self.harness.advance(self.runtime)
        window=self.begin('recover_control')
        with self.assertRaisesRegex(ValueError,'可确认恢复'):
            self.operation('recover_control',maintenance_token=window['token'])
        self.operation('end',maintenance_token=window['token'])

    def test_physical_support_confirmation_blocks_control_effects(self):
        self.device.environment='physical';self.app=Application({'physical':self.device})
        for action in ('release_control','verify_hold'):
            self.number+=1
            base=dict(environment='physical',expected_target=target(self.app,self.device))
            window=self.app.maintenance_action('begin',dict(base,request_id='begin-'+action,purpose=action))
            events=list(self.runtime.owner.backend.events)
            with self.assertRaisesRegex(ValueError,'支撑'):
                self.app.maintenance_action(action,dict(base,request_id=action,maintenance_token=window['token']))
            self.assertEqual(events,self.runtime.owner.backend.events)
            self.app.maintenance_action('end',dict(base,request_id='end-'+action,maintenance_token=window['token']))

    def test_stop_services_require_exact_scope_and_do_not_invoke_control_factory(self):
        calls=[]
        self.device.camera_service=SimpleNamespace(reconnect=Mock(side_effect=AssertionError('stopped camera cannot rebind')))
        self.device.camera_stop=lambda progress:(progress('stopping_camera_service'),calls.append('camera-stop'),{'stopped':True})[-1]
        window=self.begin('stop_cameras')
        self.assertEqual(self.operation('stop_cameras',maintenance_token=window['token']),{'stopped':True})
        self.operation('end',maintenance_token=window['token'])
        def invalidate_model(*,still_current):
            self.assertTrue(still_current());calls.append('model-invalidated');return {'invalidated':True}
        self.device.workflows['inference']=SimpleNamespace(snapshot=lambda:dict(busy=False),invalidate=lambda:None,
            operations_invalidate_model=invalidate_model)
        window=self.begin('stop_model')
        with self.assertRaisesRegex(ValueError,'作用域'):self.operation('prepare_model',maintenance_token=window['token'])
        self.assertEqual(self.operation('invalidate_model',maintenance_token=window['token']),{'invalidated':True})
        self.operation('end',maintenance_token=window['token'])
        self.assertEqual(calls,['camera-stop','model-invalidated']);self.device.factory.assert_not_called()


class ControlOperationsHTTPTests(OperationFixture,HTTPTest):
    def setUp(self):
        super().setUp();self.number=0;self.app.operations_managed=True
    def request(self,action,body,*,csrf=True,origin=None,host=None):
        connection=HTTPConnection('127.0.0.1',self.server.server_port,timeout=12)
        headers={'Content-Type':'application/json','Origin':origin or self.url}
        if csrf:headers['X-CSRF-Token']=self.app.csrf
        if host:headers['Host']=host
        try:
            connection.request('POST','/api/maintenance/'+action,json.dumps(body),headers)
            response=connection.getresponse();return response.status,json.loads(response.read())
        finally:connection.close()
    def test_http_identity_security_scope_and_managed_business_zero_effects(self):
        factory=self.device.factory;self.device.factory=Mock(wraps=factory)
        body=self.body(purpose='connect_control')
        for headers in ({'csrf':False},{'origin':'https://other.example'},{'host':'other.example'}):
            self.assertEqual(self.request('begin',body,**headers)[0],403)
        wrong=deepcopy(body);wrong['expected_target']['server_instance_id']='obsolete'
        self.assertEqual(self.request('begin',wrong)[0],409)
        for action in ('connect','release','recover'):
            with self.assertRaisesRegex(ValueError,'operation_required'):
                self.app.action(action,dict(environment='fake',business='calibration',request_id=action))
        from robot_workbench.bridges import calibration_gateway
        with self.assertRaisesRegex(ValueError,'operation_required'):
            calibration_gateway(self.device).acquire({'confirmation':'ARMS_SUPPORTED_CONNECT'})
        with self.assertRaisesRegex(ValueError,'operation_required'):self.device.connect({'arms_supported':True})
        for action in ('release','recover'):
            with self.assertRaisesRegex(ValueError,'operation_required'):self.device.submit('collection',{'action':action})
        self.device.factory.assert_not_called()
        code,window=self.request('begin',body);self.assertEqual(code,200)
        self.assertEqual(self.request('begin',body),(200,window))
        forbidden=self.body(maintenance_token=window['token'])
        self.assertEqual(self.request('verify_hold',forbidden)[0],409)
        self.assertEqual(self.request('end',self.body(maintenance_token='wrong'))[0],409)
        self.assertTrue(self.app.environment_state('fake')['operations_managed'])
        self.assertNotIn(window['token'],json.dumps(self.app.environment_state('fake')))
        self.assertEqual(self.request('end',self.body(maintenance_token=window['token']))[0],200)

    def test_connect_and_release_bind_only_successful_target_changes_and_keep_gates(self):
        window=self.begin('connect_control');old=deepcopy(window['target']);factory=self.device.factory
        def observed_factory():
            self.assertTrue(self.device.maintenance);return factory()
        self.device.factory=observed_factory
        code,result=self.request('connect_control',self.body(maintenance_token=window['token']))
        self.assertEqual(code,200,result);self.assertTrue(result['connected'])
        self.assertEqual(self.device.runtime.owner.timing_probe.thread_id,self.device.thread.ident)
        self.assertIsNotNone(self.device.runtime.owner.timing_probe.current,'shared Owner loop must run its timing probe')
        self.assertTrue(self.device.maintenance);self.assertTrue(self.device.runtime.maintenance_active)
        self.assertEqual(set(self.device.runtime.clients),{'calibration','collection','inference'})
        self.assertNotEqual(old,result['target'])
        stale=self.body(maintenance_token=window['token']);stale['expected_target']=old
        self.assertEqual(self.request('end',stale)[0],409)
        self.operation('end',maintenance_token=window['token'])
        before=list(self.device.runtime.owner.backend.events)
        for action in ('connect','release','recover'):
            with self.assertRaisesRegex(ValueError,'operation_required'):
                self.app.action(action,dict(environment='fake',business='calibration',request_id='connected-'+action))
        self.assertEqual(list(self.device.runtime.owner.backend.events),before)
        window=self.begin('release_control')
        code,result=self.request('release_control',self.body(maintenance_token=window['token']))
        self.assertEqual(code,200,result);self.assertIsNone(self.device.runtime)
        self.assertIsNone(result['target']['runtime_nonce']);self.assertTrue(result['release_receipt']['sdk_close_returned'])
        self.assertEqual(result['release_receipt']['status'],'released')
        self.operation('end',maintenance_token=window['token'])

    def test_hard_fault_recovery_window_releases_then_connects_a_fresh_runtime(self):
        window=self.begin('connect_control')
        code,result=self.request('connect_control',self.body(maintenance_token=window['token']))
        self.assertEqual(code,200,result)
        self.operation('end',maintenance_token=window['token'])
        previous=self.device.runtime
        with previous.owner.lock:
            previous.owner.control.fault('CPU nonrecoverable boundary fault')
            previous.owner.recoverable_fault=None
        window=self.begin('recover_control')
        code,released=self.request('release_control',self.body(maintenance_token=window['token']))
        self.assertEqual(code,200,released)
        self.assertTrue(released['release_receipt']['sdk_close_returned'])
        self.assertIsNone(self.device.runtime)
        code,connected=self.request('connect_control',self.body(maintenance_token=window['token']))
        self.assertEqual(code,200,connected)
        self.assertIsNot(self.device.runtime,previous)
        self.assertNotEqual(connected['target']['runtime_nonce'],previous.nonce)
        self.assertFalse(connected['runtime']['automatic_enabled'])
        self.operation('end',maintenance_token=window['token'])

    def test_stop_ignores_old_runtime_nonce_and_preempts_connection_factory(self):
        entered,release=threading.Event(),threading.Event();self.addCleanup(release.set)
        factory=self.device.factory
        def delayed():entered.set();release.wait(2);return factory()
        self.device.factory=delayed;window=self.begin('connect_control')
        worker,result=self.start(lambda:self.request('connect_control',self.body(maintenance_token=window['token'])))
        self.assertTrue(entered.wait(1));self.app.action_calls['fake']=1
        body=self.body();body['expected_target']['runtime_nonce']='obsolete'
        before=time.monotonic();self.assertEqual(self.request('stop',body)[0],200)
        self.assertLess(time.monotonic()-before,.2);self.app.action_calls['fake']=0
        release.set();worker.join(2);self.assertFalse(worker.is_alive());self.assertEqual(result[0][0],409)
        self.wait(lambda state:(state['runtime'].get('stop_receipt') or {}).get('stop_hold_return_confirmed'))
        self.assertTrue(self.device.maintenance);self.assertTrue(self.device.runtime.maintenance_active)
        self.assertFalse(any(row[0] in ('begin','send') for row in self.device.runtime.owner.backend.events))
        current=target(self.app,self.device)
        self.assertEqual(self.app.maintenance['fake']['target'],current)
        stale=self.body(maintenance_token=window['token']);stale['expected_target']=window['target']
        self.assertEqual(self.request('end',stale)[0],409)
        self.assertEqual(self.request('inspect',dict(stale,maintenance_token='wrong'))[0],409)
        code,inspection=self.request('inspect',stale)
        self.assertEqual(code,200);self.assertTrue(inspection['runtime_same']);self.assertIsNone(inspection['active_action'])
        self.assertEqual(inspection['target'],current)
        self.assertTrue(self.device.maintenance,'inspection must not end, resume or release')
        runtime=self.device.runtime
        try:
            self.device.runtime=None
            self.assertFalse(self.request('inspect',stale)[1]['runtime_same'])
        finally:self.device.runtime=runtime
        stale['expected_target']=inspection['target'];stale['request_id']='explicit-recovered-end'
        self.assertEqual(self.request('end',stale)[0],200)


def load_tests(loader,tests,pattern):
    return unittest.TestSuite(cls(name) for cls in (ControlOperationsTests,ControlOperationsHTTPTests)
        for name in cls.__dict__ if name.startswith('test_'))


if __name__=='__main__':unittest.main()
