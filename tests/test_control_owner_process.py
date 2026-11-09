"""Actual Runtime/target interpreter in an independent CPU process; no SDK/CAN."""
from pathlib import Path
from unittest.mock import patch
import gc,json,multiprocessing,os,sys,tempfile,threading,time,unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]
from robot_workbench.control_owner import ControlChannel,ControlRuntimeClient


def setUpModule():
    global recording_fixture
    from test_recording_process import start_recording_fixture
    recording_fixture=start_recording_fixture()


def tearDownModule():
    from test_recording_process import stop_recording_fixture
    stop_recording_fixture(recording_fixture)


class EventReadLockTest(unittest.TestCase):
    def test_fake_owner_clock_uses_published_simulation_time_without_wall_extrapolation(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from robot_workbench.control_owner import OwnerFacts
        client=SimpleNamespace(environment='fake',state={'control_time_s':7.5},ros_state=None)
        owner=OwnerFacts(client)
        with patch('time.monotonic',return_value=1000.):
            self.assertEqual(owner.control_time(),7.5)
            client.ros_state=SimpleNamespace(inputs=SimpleNamespace(latest=lambda _: (SimpleNamespace(source_time_ns=8_000_000_000),{})))
            self.assertEqual(owner.control_time(),8.)
            client.environment='physical';self.assertEqual(owner.control_time(),1000.)

    def test_record_backlog_cannot_starve_state_or_repeat_health_per_event(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        client=object.__new__(ControlRuntimeClient)
        client.inference_lock=threading.RLock();client.event_closed=threading.Event()
        client.event_cursor=0;client.inference_session='session';client.recording_error=None
        client.recording=object();runner=SimpleNamespace(value={});client.runners={'run':runner}
        client.inference_callbacks={'record':Mock()};client._recording_facts=Mock()
        def call(operation,**kwargs):
            if operation=='events':
                return dict(incomplete=False,latest=client.event_cursor+64,events=[
                    dict(sequence=client.event_cursor+i,event='owner_inference',value=dict(
                        session_id='session',kind='record',name='feedback',value={})) for i in range(1,33)])
            if operation=='inference_state':
                client.event_closed.set()
                return dict(execution_instance_id='run',snapshot={'mode':'HOLD'})
            raise AssertionError(operation)
        client._call=call
        thread=threading.Thread(target=client._events);thread.start()
        try:self.assertTrue(client.event_closed.wait(.5),'continuous records starved the Owner snapshot')
        finally:client.event_closed.set();thread.join(1)
        self.assertEqual(runner.value,{'mode':'HOLD'})
        self.assertEqual(client.inference_callbacks['record'].call_count,32)
        client._recording_facts.assert_called_once_with()

    def test_missing_recording_capability_refuses_attachment_and_stop_is_independent(self):
        from unittest.mock import Mock,patch
        from types import SimpleNamespace
        def refresh(client):
            client.nonce='cpu-owner';client.state={'connected':True,'control_owner':{'protocol_features':['capture_session_v1']}}
        channel=SimpleNamespace(descriptor={'device_id':'cpu','environment':'fake'})
        with patch('robot_workbench.control_owner.ControlChannel',return_value=channel),patch.object(
                ControlRuntimeClient,'refresh',refresh),patch.object(ControlRuntimeClient,'detach')as detach:
            with self.assertRaisesRegex(ValueError,'attachment refused before control'):
                ControlRuntimeClient('/unused',recording_backend='capture_session_v1')
            detach.assert_called_once()
        client=object.__new__(ControlRuntimeClient);client._call=Mock(return_value={'stopped':True})
        self.assertEqual(client.request_stop(),{'stopped':True});client._call.assert_called_once_with('stop')

    def test_old_owner_descriptor_is_rejected_before_socket_or_credentials(self):
        with tempfile.TemporaryDirectory()as folder:
            path=Path(folder)/'descriptor.json'
            path.write_text(json.dumps(dict(schema='workbench.control_owner.v1',environment='fake')));path.chmod(0o600)
            with self.assertRaisesRegex(ValueError,'protocol version mismatch'):ControlChannel(path)

    def test_new_client_refuses_old_owner_that_would_ignore_mode_configuration(self):
        from robot_workbench.control_owner import ControlRuntimeClient
        client=object.__new__(ControlRuntimeClient);client.state={'control_owner':{}}
        client._call=lambda *args,**kwargs:self.fail('must reject before remote configure')
        for config in ({'normal_modes':{}},{'normal_finish_policy':{}},{'action_chunk_binding':{}},
                       {'recording_context_policy':'per_episode_review'},{'recording_backend':'capture_session_v1'},
                       {'state_max_age_s':.05},{'max_camera_span_s':.05}):
            with self.assertRaisesRegex(ValueError,'protocol version mismatch'):
                client.configure_remote_inference(config,{},created=None,metric=None,record=None,predict=None)

    def test_business_drains_diagnostic_backlog_before_query_images_expire(self):
        from collections import OrderedDict
        from robot_workbench.inference_owner import digest
        import numpy as np
        client=object.__new__(ControlRuntimeClient)
        client.inference_lock=threading.RLock();client.lock=threading.RLock()
        client.event_closed=threading.Event();client.event_cursor=0;client.inference_session='new'
        client.recording=None;client.runners={};client.recording_error=None
        ids={'head':{'frame':1}}
        client.observations=OrderedDict([(digest(ids),{'images':{'raw':'original frame'}})])
        queried=threading.Event()
        def predict(request):
            self.assertEqual(request['observation']['images'],{'raw':'original frame'});queried.set()
            return dict(request_id='q',action14=np.zeros((60,14)))
        client.inference_callbacks=dict(predict=predict)
        rows=[dict(sequence=i,event='diagnostic',value={})for i in range(1,81)]
        rows.append(dict(sequence=81,event='owner_inference',value=dict(session_id='new',kind='query',request=dict(
            request_id='q',applied_history=[],observation={'frame_ids':ids}))))
        def call(operation,**args):
            if operation=='events':return dict(events=[r for r in rows if r['sequence']>args['after']][:1],latest=81,incomplete=False)
            if operation=='inference_state':return dict(execution_instance_id=None,snapshot=None)
            if operation=='inference_result':return {}
            raise AssertionError(operation)
        client._call=call
        thread=threading.Thread(target=client._events);thread.start()
        try:self.assertTrue(queried.wait(.4),'old 20ms-per-row draining loses the frame before dispatch')
        finally:client.event_closed.set();thread.join(1)
        self.assertIsNone(client.recording_error)

    def test_event_reads_snapshot_without_parent_encoding_and_preserve_cursors(self):
        from collections import deque
        from types import SimpleNamespace
        from unittest.mock import patch
        from robot_workbench.control_owner import ControlOwnerService,_encode_event_response,LIMIT
        service=object.__new__(ControlOwnerService);service.device=SimpleNamespace(runtime=None)
        service.event_lock=threading.Lock();service.events=deque(maxlen=8192);service.event_sequence=0
        for i in range(35):
            service.event('owner_inference'if i%2 else 'inference_guard_decision',sample=i)
        service.event('fault_diagnostic',payload='x'*(LIMIT//2+100))
        service.event('control_study_feedback',sample=36)
        with patch('robot_workbench.control_owner._encode',side_effect=AssertionError('parent encoded events')):
            first=service._read('events',dict(after=0))
            self.assertEqual(len(first['events']),32)
            self.assertTrue(service.event_lock.acquire(blocking=False));service.event_lock.release()
            second=service._read('events',dict(after=32))
            self.assertEqual([r['sequence']for r in second['events']],[33,34,35])
            large=service._read('events',dict(after=35))
            self.assertEqual([r['sequence']for r in large['events']],[36])
            self.assertEqual(service._read('events',dict(after=36))['events'][0]['sequence'],37)
        wire=_encode_event_response(dict(ok=True,request_id='cpu',result=large))
        self.assertGreater(len(wire),LIMIT//2);self.assertLess(len(wire),LIMIT)
        self.assertEqual(json.loads(wire)['result'],large)
        rows=[dict(sequence=i,event='control_study_feedback',value='x'*(LIMIT//3))for i in range(1,4)]
        trimmed=json.loads(_encode_event_response(dict(result=dict(events=rows,latest=3,incomplete=False))))['result']
        self.assertEqual([r['sequence']for r in trimmed['events']],[1]);self.assertEqual(trimmed['latest'],3)
        service.events.popleft()
        self.assertTrue(service._read('events',dict(after=0))['incomplete'])
        with self.assertRaisesRegex(ValueError,'bounded size'):
            _encode_event_response(dict(result=dict(events=[dict(payload='x'*LIMIT)])))


def _owner_process(root,ready,closing,heartbeat=None,tripped=None,fixture_mode=None):
    import robot_workbench.control_owner as transport
    from robot_workbench.runtime import Runtime
    from robot_workbench.service import Device
    from test_shared_target_routes import WindowBackend
    from test_owner_clock import calibration,Safety,jaw
    # Production is Linux-only. macOS CPU checks use an explicitly test-local
    # lifetime identity; Linux checks read the actual boot/start identity.
    if sys.platform!='linux':transport._identity=lambda:dict(pid=os.getpid(),start_ticks='cpu-test',boot_id='cpu-test')
    jaw.configure_mapping(None);device=Device('arx6'if fixture_mode=='ros_workflow_wam'else'isolated-cpu','fake',None)
    if fixture_mode=='ros_workflow_wam':
        from test_mixed366_control import native_mapping_fixture
        _,mapping=native_mapping_fixture(Path(root)/'native');jaw.configure_mapping(mapping['path'])
    service=transport.ControlOwnerService(device,Path(root)/'service')
    if fixture_mode=='slow_read_encoding':
        original_read=service._read;delay_lock=threading.Lock()
        def delayed_read(operation,arguments):
            flag=Path(root)/'delay-read'
            with delay_lock:
                if operation=='snapshot'and flag.exists():
                    delay=float(flag.read_text()or'.25');flag.unlink()
                    service.read_encoder.submit(time.sleep,delay)
                    (Path(root)/'read-delayed').touch()
            return original_read(operation,arguments)
        service._read=delayed_read
    if fixture_mode=='large_events':
        service.event('fault_diagnostic',trace=[[(i+j)/10000 for j in range(14)]for i in range(26000)])
    elif fixture_mode=='write_encoding_failure':
        original_encode=transport._encode
        def fail_once(value):
            if isinstance(value,dict)and value.get('ok')is True and 'stop'in value.get('result',{}):
                transport._encode=original_encode
                raise ValueError('CPU response encoding failure after Stop')
            return original_encode(value)
        transport._encode=fail_once
    simulation=[10.]
    clock=(lambda:simulation[0])if fixture_mode=='ros_workflow_sim'else time.monotonic
    def factory():
        gc.disable()  # Explicit CPU analogue of SDKProcess's connected GC policy.
        if fixture_mode=='ros_workflow_wam':
            from test_normal_modes import NormalBackend
            raw=NormalBackend(clock)
        else:
            raw=WindowBackend(clock);raw.target_interpreter_profile='x5_shared_precision_v1'
        if fixture_mode in('ros_workflow','ros_workflow_sim','ros_workflow_wam'):
            raw.q[6]=jaw.feedback_sdk('left',83.);raw.q[13]=jaw.feedback_sdk('right',83.)
        from collections import deque
        raw.packets=deque(maxlen=64)  # Test diagnostics must not retain every trajectory for a memory soak.
        if heartbeat is not None:
            read=raw.read_calibration_state
            def measured():
                heartbeat.value=time.monotonic()
                if tripped.value:raw.terminal_failure='SDK owner heartbeat lost';raw.window=None
                return read()
            raw.read_calibration_state=measured
        runtime=Runtime(calibration,Safety,lambda:raw,device_id=device.id,environment='fake',
            lock_root=Path(root)/'locks',initialize_backend=lambda b:b.set_damping_hold(manual=True),event=service.event,
            control_clock=clock,
            data_plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_process_state',domain_id=126)if fixture_mode in ('ros_state','ros_workflow','ros_workflow_sim','ros_workflow_wam')else None)
        if fixture_mode=='ros_workflow_wam':
            runtime.owner.preserve_begin_jaws=True
            read_feedback=raw.read_calibration_state
            def manual_feedback():
                value=read_feedback()
                drag=Path(root)/'manual-drag-q.json'
                if drag.exists()and all(not any(row['kp'])for row in raw.mode_gains.values()):
                    positions=json.loads(drag.read_text())
                    for index,position in enumerate(positions):
                        if position is not None:
                            raw.q[index]=value['qpos'][index]=position
                return value
            raw.read_calibration_state=manual_feedback
            from robot_workbench.recording_process import capture_control
            from robot_workbench.sample_clock import clock_spec
            def recording(payload,on_fault):
                return capture_control(Path(root)/'control-capture',payload,cameras=None,environment='fake',
                    clock=clock_spec(),source_time=clock,on_fault=on_fault,ros_runtime=runtime)
            recording.recording_backend='capture_session_v1';recording.supports_deferred_preparation=True
            runtime.owner.recording_factory=recording
        return runtime
    def drive(runtime,closed):
        runtime.owner.backend._backend.control_thread=threading.get_ident()
        while not closed.is_set():
            started=time.monotonic()
            if fixture_mode=='ros_workflow_sim':simulation[0]+=.01
            if not runtime.tick():break
            closed.wait(max(0.,.01-(time.monotonic()-started)))
    device.factory=factory;device.drive=drive
    serving=threading.Thread(target=service.serve_forever,daemon=True);serving.start()
    ready.put(str(service.descriptor_path));closing.wait(3600)
    if device.runtime is None:service.close()
    else:
        service.event_encoder.shutdown(wait=True,cancel_futures=True)
        service.read_encoder.shutdown(wait=True,cancel_futures=True)


def _guard_process(heartbeat,tripped,ready,closed):
    from robot_workbench.arx_output_guard import FeedbackLatch
    latch=FeedbackLatch(('cpu-left','cpu-right'),time.monotonic)
    latch.begin();latch.phase='active'
    ready.put(dict(armed=True))
    while not closed.is_set():
        # Explicit synthetic motor traffic; Owner progress is the separate input.
        now=time.monotonic();latch.seen={key:now for key in latch.expected}
        if heartbeat.value:
            latch.heartbeat=heartbeat.value
            if latch.check():
                tripped.value=True
                ready.put(dict(reason=latch.reason,age_s=now-heartbeat.value));return
        closed.wait(.002)


def _business_process(descriptor,ready,mode):
    channel=ControlChannel(descriptor);state=channel.call('snapshot',business='calibration');nonce=state['runtime_nonce']
    q=state['calibration_feedback']['qpos'][:];q[0]=.04
    review=channel.call('submit',expected_nonce=nonce,business='calibration',command=dict(action='preview',active_arm='left',q_active=q[:6],max_velocity=.03))
    channel.call('submit',expected_nonce=nonce,business='calibration',command=dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
    ready.put(dict(pid=os.getpid(),nonce=nonce))
    ready.close();ready.join_thread()
    if mode=='exit':os._exit(0)
    if mode=='gc':
        def slow_gc(phase,info):
            if phase=='start':
                old=sys.getswitchinterval();sys.setswitchinterval(1.)
                end=time.monotonic()+.35
                while time.monotonic()<end:pass
                sys.setswitchinterval(old)
        gc.callbacks.append(slow_gc)
        try:gc.collect()
        finally:gc.callbacks.remove(slow_gc)
    elif mode=='http':
        from http.server import BaseHTTPRequestHandler,HTTPServer
        import http.client
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                old=sys.getswitchinterval();sys.setswitchinterval(1.);end=time.monotonic()+.5
                while time.monotonic()<end:pass
                sys.setswitchinterval(old);self.send_response(200);self.end_headers();self.wfile.write(b'CPU blocking HTTP test')
        server=HTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.handle_request);thread.start()
        client=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=3)
        client.request('GET','/block');client.getresponse().read();client.close();thread.join();server.server_close()


class ControlOwnerProcessTests(unittest.TestCase):
    def test_control_policy_round_trip_uses_owner_revision_and_store(self):
        self.acquire()
        before=self.channel.call('control_policy_snapshot',business='calibration')
        result=self.channel.call('control_policy',business='calibration',payload=dict(
            expected_revision=before['revision'],changes={'arrival.jaw_mm':3.,'measured.limit_rad_s':1.1}))
        self.assertEqual(result['revision'],before['revision']+1)
        after=self.channel.call('control_policy_snapshot',business='calibration')
        self.assertEqual(after['policy_hash'],result['policy_hash'])
        self.assertEqual(after['parameters']['arrival']['jaw_mm'],3.)
        self.assertNotIn('policy_id',after)
        with self.assertRaises(ValueError):
            self.channel.call('control_policy',business='calibration',payload=dict(
                expected_revision=before['revision'],changes={'arrival.jaw_mm':4.}))
        self.assertFalse(self.channel.call('inspect')['automatic_enabled'])

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='wb-owner-',dir='/tmp');self.addCleanup(self.temp.cleanup)
        self.context=multiprocessing.get_context('spawn');self.ready=self.context.Queue();self.closing=self.context.Event()
        self.heartbeat=self.context.Value('d',0.);self.tripped=self.context.Value('b',False)
        fixture_mode=('large_events'if self._testMethodName.startswith('test_event_')else
                      'ros_workflow_wam'if self._testMethodName.startswith('test_wam32_')else
                      'ros_workflow_sim'if self._testMethodName=='test_ros_workflow_uses_simulation_clock_for_the_complete_run'else
                      'ros_workflow'if self._testMethodName=='test_ros_workflow_runs_through_independent_owner_and_resident_recorder'else
                      'ros_state'if self._testMethodName=='test_ros_state_projection_has_no_raw_feedback_on_control_rpc'else
                      'write_encoding_failure'if self._testMethodName=='test_successful_stop_encoding_failure_is_unknown'else
                      'slow_read_encoding'if self._testMethodName in ('test_read_encoder_delay_does_not_stall_tick_or_stop_and_preserves_sample_age','test_concurrent_state_readers_wait_outside_control_instead_of_failing')else None)
        self.process=self.context.Process(target=_owner_process,args=(self.temp.name,self.ready,self.closing,self.heartbeat,self.tripped,fixture_mode));self.process.start()
        self.addCleanup(self.cleanup)
        self.descriptor=self.ready.get(timeout=8);self.channel=ControlChannel(self.descriptor)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_workflow_runs_through_independent_owner_and_resident_recorder(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_normal_one_step_is_executed_and_recorded(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,normal32=True,execute_steps=1)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_normal_default_steps_are_executed_and_recorded(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,normal32=True,execute_steps=15)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_ros_feedback_stays_fresh_when_control_time_projection_lags(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,normal32=True,stale_control_clock=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_normal_full_chunk_is_executed_and_recorded(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,normal32=True,execute_steps=32)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_observations_continue_while_page_polls_a_long_control_session(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(
            wam32=True,normal32=True,execute_steps=32,poll_page=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_operator_stop_pending_model_keeps_position_and_recording(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,normal32=True,operator_stop=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated Linux ROS only')
    def test_wam32_direct_run_stop_restores_prepared_hold_and_all_pedals(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,normal32=True,direct_run=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_operator_home_uses_shared_owner_and_records(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(
            wam32=True,normal32=True,operator_teach=True,prepared_session=True,native_home=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated Linux ROS only')
    def test_wam32_independent_stop_during_native_home_seals_session_without_release(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(
            wam32=True,normal32=True,operator_teach=True,prepared_session=True,native_home=True,stop_home=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated Linux ROS only')
    def test_wam32_pedal2_teach_hold_records_manual_start_and_next_infer_uses_new_pose(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,normal32=True,operator_teach=True,prepared_session=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated Linux ROS only')
    def test_wam32_pedal_gestures_start_stop_shared_owner_and_seal_one_recording(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(
            wam32=True,normal32=True,operator_teach=True,prepared_session=True,pedal_gestures=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated Linux ROS only')
    def test_wam32_pedal2_global_stop_revokes_positioning_without_late_hold_or_model(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,normal32=True,operator_teach=True,teach_stop=True,prepared_session=True)

    def test_wam32_manual_boundary_feedback_returns_legal_hold_and_complete_capture(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(
            wam32=True,normal32=True,operator_teach=True,prepared_session=True,manual_boundary=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_stop_during_prediction_records_late_output_without_sdk_submission(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,stop_during_model=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_wam32_supply_expiry_stops_without_owner_fault_and_preserves_late_result(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(wam32=True,expire_supply=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_ros_state_projection_has_no_raw_feedback_on_control_rpc(self):
        self.acquire()
        wire=self.channel.call('snapshot')
        self.assertNotIn('calibration_feedback',wire)
        self.assertEqual(wire['control_owner']['data_plane']['domain_id'],126)
        direct=self.channel.call('inputs')
        self.assertIsNone(direct['calibration_feedback'])
        self.assertIsInstance(direct['feedback_reference'],dict)
        client=ControlRuntimeClient(self.descriptor);self.addCleanup(client.detach)
        deadline=time.monotonic()+5;state={}
        while time.monotonic()<deadline:
            state=client.refresh()
            if state.get('calibration_feedback'):break
            time.sleep(.02)
        feedback=state.get('calibration_feedback');self.assertIsNotNone(feedback,state.get('feedback_source_error'))
        self.assertEqual(feedback['ros_reference'],state['feedback_reference'])
        self.assertEqual(len(feedback['qpos']),14)
        projected=client.control_inputs()
        self.assertEqual(projected['calibration_feedback']['ros_reference'],projected['feedback_reference'])
        wrong=dict(state['feedback_reference'],source_instance_id='another_owner')
        with self.assertRaisesRegex(ValueError,'another Owner'):client.ros_state.resolve(wrong)
        self.assertIsNone(client.control_inputs(include_feedback=False)['calibration_feedback'])
        from unittest.mock import patch
        entered=threading.Event();release=threading.Event();returned=threading.Event()
        original=client.ros_state.resolve
        def blocked(reference):entered.set();release.wait(2);return original(reference)
        with patch.object(client.ros_state,'resolve',side_effect=blocked):
            reader=threading.Thread(target=client.refresh);reader.start()
            self.assertTrue(entered.wait(1))
            stopper=threading.Thread(target=lambda:(client.request_stop(),returned.set()));stopper.start()
            try:self.assertTrue(returned.wait(.5),'blocked state read delayed the independent Stop request')
            finally:release.set();reader.join(3);stopper.join(3)
        client.detach()
        self.assertTrue(self.channel.call('snapshot')['connected'],'detaching the display must not release the Owner')

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_ros_workflow_uses_simulation_clock_for_the_complete_run(self):
        self.test_ros_workflow_runs_through_independent_owner_and_resident_recorder(simulated=True)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
    def test_ros_workflow_runs_through_independent_owner_and_resident_recorder(self,simulated=False,wam32=False,stop_during_model=False,expire_supply=False,normal32=False,execute_steps=15,operator_stop=False,operator_home=False,stop_home=False,operator_teach=False,teach_stop=False,prepared_session=False,native_home=False,manual_boundary=False,poll_page=False,pedal_gestures=False,stale_control_clock=False,direct_run=False):
        import numpy as np
        from types import SimpleNamespace
        from sensor_msgs.msg import Image
        from test_inference_workflow import InferenceWorkflowTests
        from test_inference_transport import StubModel
        from robot_workbench.inference_transport import ModelService,serve_model
        from robot_workbench.inference_workflow import InferenceWorkflow,CAMERAS
        from robot_workbench.ros_data import RosNode,descriptor_message
        from robot_workbench.service import Device
        from robot_workbench import wam
        fixture=InferenceWorkflowTests('runTest');fixture.setUp();self.addCleanup(fixture.doCleanups)
        fixture.workflow.close()
        for patcher in fixture.patches:patcher.stop()
        if wam32:
            from test_mixed366_control import native_mapping_fixture,jaw
            from test_wam import WamInputWorkflowTests
            _,mapping=native_mapping_fixture(Path(self.temp.name)/'native');jaw.configure_mapping(mapping['path'])
            self.addCleanup(lambda:jaw.configure_mapping(None));bundle=WamInputWorkflowTests().descriptor()
        self.acquire();client=ControlRuntimeClient(self.descriptor).acquire();self.addCleanup(client.detach)
        if stale_control_clock:
            actual_inputs=client.control_inputs
            def delayed_projection(**kwargs):
                value=actual_inputs(**kwargs)
                return dict(value,control_time_s=value['control_time_s']-.25)
            client.control_inputs=delayed_projection
        parent=Device('arx6'if wam32 else'isolated-cpu','fake',None);parent.attach_existing_owner(client);self.addCleanup(parent.closed.set)
        client.handoff(parent.credentials['calibration'],'inference');fixture.binding['robot_id']=parent.id
        if wam32:
            import hashlib
            sdk=Path(self.temp.name)/'native/sdk.bin';sdk.write_bytes(b'CPU binding fixture, not an SDK')
            fixture.profile,fixture.binding=wam.native_binding(dict(robot_id=parent.id,disabled_grippers=[],
                sdk_quiesce=dict(module=dict(path=str(sdk),sha256=hashlib.sha256(sdk.read_bytes()).hexdigest())),
                native_gripper_calibration=json.loads(Path(mapping['path']).read_text())['native_gripper_calibration'],
                cameras={s:dict(serial=v)for s,v in fixture.binding['camera_serials'].items()}))
        model_started=threading.Event();model_continue=threading.Event()
        class Model(StubModel):
            def prepare(self):return wam.worker_identity(bundle)if wam32 else super().prepare()
            def worker_status(self):return dict(worker_pid=os.getpid(),worker_exit_code=None,worker_alive=True)
            def infer(self,request):
                self.requests.append(request)
                if (stop_during_model or operator_stop)and len(self.requests)>1:
                    model_started.set()
                    if not model_continue.wait(5):raise TimeoutError('CPU late-output barrier expired')
                if wam32 and not stop_during_model and len(self.requests)>=3:
                    time.sleep(2.6 if expire_supply else .4)
                held=np.zeros(14);held[[6,13]]=.081
                if operator_home:held[0]=.04
                return dict(request_id=request['request_id'],action14=np.tile(held if wam32 else request['observation']['model_state14'],(32 if wam32 else 60,1)),audit={'cpu_stub':True})
        model=Model();token=Path(self.temp.name)/'model-token';token.write_text('test-only-'+40*'x')
        if wam32:model.bundle=bundle;model.model_id=bundle['model_id']
        server=serve_model(ModelService(model),token_file=token)
        serving=threading.Thread(target=server.serve_forever,daemon=True);serving.start()
        camera=RosNode('workflow_camera',namespace='/wb_process_state',domain_id=126)
        closed=threading.Event();pause_frames=threading.Event();thread=None;flow=None
        try:
            publishers={};declarations=[]
            clock=client.control_inputs()['calibration_feedback']['source_clock_id']
            for role,name in CAMERAS.items():
                descriptor=dict(device_id=parent.id,stream_id=name,source_instance_id='camera',source_clock_id=clock,
                    topic='/wb_process_state/'+name,message_type='sensor_msgs/msg/Image',kind='image',units='rgb8',
                    coordinate_frame=role,nominal_hz=30.,available=True,
                    metadata=dict(width=224,height=224,serial=fixture.binding['camera_serials'][role],environment='fake'))
                declared=camera.publisher('sources/'+name,camera.types.SourceDescriptor,retained=True,depth=1)
                declared.publish(descriptor_message(camera,descriptor));declarations.append(declared)
                publishers[name]=camera.publisher(descriptor['topic'],Image)
            def frames():
                while not closed.wait(1/30):
                    if pause_frames.is_set():continue
                    stamp=round(client.owner.control_time()*1e9)if simulated else time.monotonic_ns()
                    for publisher in publishers.values():
                        image=Image(width=224,height=224,encoding='rgb8',step=672,data=bytes(224*224*3))
                        image.header.stamp.sec=stamp//10**9;image.header.stamp.nanosec=stamp%10**9;publisher.publish(image)
            thread=threading.Thread(target=frames);thread.start()
            model_options=dict(model_id=bundle['model_id'],model_bundle=bundle)if wam32 else{}
            if wam32 and not stop_during_model:
                from test_variable_inference_steps import config as planned_config
                model_options.update(recording_enabled=False,reference_planning=planned_config()['reference_planning'],finite_chunk_count=3,task='coffee')
                if normal32:
                    from robot_workbench.execution_settings import NORMAL_BRAKE_DURATION_S,NORMAL_GAIN_TRANSITION_S
                    model_options.pop('finite_chunk_count')
                    policy=model_options['reference_planning']
                    model_options.update(normal_initial_mode='HOLD',normal_modes=dict(brake_duration_s=NORMAL_BRAKE_DURATION_S,
                        velocity_limit=policy['velocity_limit_rad_s'],acceleration_limit=policy['acceleration_limit_rad_s2'],
                        gain_transition_s=NORMAL_GAIN_TRANSITION_S,teach_kd_scale=.1,recording_required=False,
                        recorder_ids=[],recording_clock_id='host_wall_time_ns'))
            flow=InferenceWorkflow(parent,Path(self.temp.name)/'ros-infer',camera_service=SimpleNamespace(descriptor={}),
                model_endpoint='http://127.0.0.1:'+str(server.server_port),model_token_file=str(token),
                profile_provider=lambda:(fixture.profile,fixture.binding),action_chunks=True,
                recording_backend='capture_session_v1',**model_options,ros_plane=dict(schema='workbench.ros-data-plane.v1',
                    namespace='/wb_process_state',domain_id=126))
            parent.maintenance=True
            try:flow.operations_prepare_model(still_current=lambda:True)
            finally:parent.maintenance=False
            if normal32:flow.set_execution_steps(execute_steps,flow.execution_settings()['revision'])
            flow.bind_device()
            if wam32 and not stop_during_model and not expire_supply:
                pause_frames.set();time.sleep(.15)
                resume=threading.Timer(.1,pause_frames.clear);resume.start();self.addCleanup(resume.cancel)
            duration=5. if normal32 else .5 if wam32 and not stop_during_model else 2.
            if direct_run:
                from robot_workbench.service import Application
                import uuid
                parent.workflows['inference']=flow;app=Application({'fake':parent})
                flow.operator_controls=dict(home_velocity_rad_s=.1,timeout_s=30.,focus_timeout_s=8.,double_press_window_s=.5)
                self.assertIsNone(getattr(flow,'operator_session',None))
                view=app.state('fake','inference');info=view['workflow']['inference']
                app.action('inference_run',dict(environment='fake',business='inference',request_id=uuid.uuid4().hex,
                    seed=1,duration_s=duration,model_id=info['selected_model_id'],expected_bundle_digest=info['bundle_digest'],
                    expected_context=info['run_context'],state_wall_ns=view['server_wall_ns'],physical_clearance=True,scene_reset=True))
                def wait_mode(target):
                    deadline=time.monotonic()+15
                    while True:
                        state=flow.current['runner'].snapshot()if flow.current and flow.current.get('runner')else{}
                        if state.get('mode')==target and not state.get('transition_stage'):return state
                        self.assertLess(time.monotonic(),deadline,state);time.sleep(.02)
                wait_mode('MODEL');facts=client.refresh()
                app.action('inference_hold',dict(environment='fake',business='inference',request_id='direct-run-stop',
                    runtime_nonce=client.nonce,epoch=facts['command_epoch']))
                wait_mode('HOLD');deadline=time.monotonic()+5
                while app.jobs[('fake','inference')]['status']=='running':
                    self.assertLess(time.monotonic(),deadline);time.sleep(.02)
                self.assertEqual(app.jobs[('fake','inference')]['status'],'completed')
                run=flow.current;session=flow.operator_session
                self.assertEqual(session['run_id'],run['id']);self.assertFalse(run['recorder'].done.is_set())
                gates=app.state('fake','inference')['runtime']['allowed_operations']
                for name in ('inference_run','inference_teach','inference_home'):self.assertTrue(gates[name]['allowed'],gates[name])
                sequence=0;focus=dict(client_id='cpu-direct-run',started_wall=time.time())
                def press(command):
                    nonlocal sequence
                    sequence+=1
                    return app._operator_pedal_action(parent,flow,command,
                        SimpleNamespace(source_instance_id='cpu-direct-pedal',sequence=sequence),time.time(),focus)
                press(22);wait_mode('TEACH');press(22);wait_mode('HOLD')
                deadline=time.monotonic()+5
                while app.operator_tasks['fake']['status']=='running':
                    self.assertLess(time.monotonic(),deadline);time.sleep(.02)
                press(33);deadline=time.monotonic()+10
                while app.operator_tasks['fake']['status']=='running':
                    self.assertLess(time.monotonic(),deadline);time.sleep(.02)
                self.assertEqual(app.operator_tasks['fake']['status'],'completed')
                wait_mode('HOLD');press(11);time.sleep(.3);self.assertIn('job_id',press(11));wait_mode('MODEL')
                press(11);time.sleep(.3);self.assertFalse(press(11)['accepted']);wait_mode('HOLD')
                self.assertIs(flow.current,run);self.assertEqual(flow.operator_session,session)
                app.action('cancel',dict(environment='fake',business='inference',request_id='direct-run-end'))
                self.assertTrue(run['recorder'].done.wait(15));manifest=run['recorder'].manifest
                self.assertEqual(manifest['recording_outcome'],'complete',manifest['issues'])
                self.assertTrue(client.refresh()['connected'])
                return
            review=flow.preview_episode(seed=1,duration_s=duration)
            flow.execute_review(review['preview']['token'])
            if normal32:
                from robot_workbench.service import Application
                app=Application.__new__(Application);app.lock=threading.RLock()
                if poll_page:
                    from robot_workbench.service import serve
                    from urllib.request import urlopen
                    # Build real transition receipts before exercising the
                    # normal page endpoint throughout multiple model chunks.
                    for cycle in range(8):
                        for target in ('TEACH','HOLD'):
                            deadline=time.monotonic()+5
                            while True:
                                run=flow.current
                                mode=run['runner'].snapshot()if run and run.get('runner')else{}
                                if mode.get('mode')in('TEACH','HOLD')and not mode.get('transition_stage'):break
                                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                            key=f'page-cycle-{cycle}-{target}'
                            flow.request_normal_mode(dict(session_id=mode['mode_session_id'],runtime_nonce=client.nonce,
                                request_id=key,epoch=mode['command_epoch'],mode=target,target=target))
                            while True:
                                mode=flow.current['runner'].snapshot()
                                receipt=mode.get('mode_receipts',{}).get(key,{})
                                if receipt.get('completed'):break
                                self.assertNotEqual(receipt.get('status'),'failed',receipt)
                                self.assertLess(time.monotonic(),deadline,receipt);time.sleep(.01)
                    parent.workflows['inference']=flow;app=Application({'fake':parent})
                    page=serve(app,port=0);serving_page=threading.Thread(target=page.serve_forever,daemon=True)
                    serving_page.start();poll_stop=threading.Event();page_reads=[];page_errors=[]
                    def poll():
                        while not poll_stop.is_set():
                            try:
                                with urlopen(f'http://127.0.0.1:{page.server_port}/api/state?environment=fake&business=inference',timeout=4)as reply:
                                    page_reads.append(len(reply.read()))
                            except Exception as error:page_errors.append(str(error));break
                            poll_stop.wait(.1)
                    polling=threading.Thread(target=poll);polling.start()
                    def close_page():
                        poll_stop.set();polling.join(5);page.shutdown();page.server_close();serving_page.join(2)
                    self.addCleanup(close_page)
                if operator_stop:
                    result=[];errors=[]
                    def run_episode():
                        try:result.append(app.run_normal_episode(parent,flow,{},lambda:True,client,client.nonce,duration))
                        except Exception as error:errors.append(error)
                    running=threading.Thread(target=run_episode);running.start()
                    self.assertTrue(model_started.wait(8))
                    flow.request_operator_stop('cpu-operator-stop')
                    deadline=time.monotonic()+3
                    while True:
                        state=flow.current['runner'].snapshot()
                        if state['mode']=='HOLD'and not state['transition_stage']:break
                        self.assertLess(time.monotonic(),deadline,state);time.sleep(.01)
                    self.assertIsNone(state['normal_model_segment'])
                    model_continue.set();running.join(12);self.assertFalse(running.is_alive());self.assertFalse(errors,errors)
                    self.assertTrue(result[0]['operator_stopped'])
                else:app.run_normal_episode(parent,flow,{},lambda:True,client,client.nonce,duration)
                if poll_page:
                    poll_stop.set();polling.join(5)
                    self.assertFalse(page_errors,page_errors);self.assertGreater(len(page_reads),3)
                    self.assertGreaterEqual(len(model.requests),3)
            if stop_during_model:
                self.assertTrue(model_started.wait(8),'WAM request never reached the actual HTTP model worker')
                client.request_stop()
                stopped_deadline=time.monotonic()+3
                while True:
                    state=client.refresh();stopped=flow.current['runner'].snapshot()
                    if stopped.get('runner_status')=='STOPPED'and(state.get('stop_receipt')or{}).get('stop_hold_return_confirmed'):break
                    self.assertLess(time.monotonic(),stopped_deadline,'Stop did not confirm while model remained blocked')
                    time.sleep(.01)
                stopped_submissions=stopped['actual_submission_count']
                model_continue.set()
            deadline=time.monotonic()+25
            while flow.current is None or flow.current.get('receipt')is None:
                self.assertLess(time.monotonic(),deadline,dict(stage=flow.stage,message=flow.message,
                    recording_error=client.recording_error,current=flow.current.get('recording_error')if flow.current else None))
                time.sleep(.02)
            run=flow.current;manifest=run['recorder'].manifest
            self.assertEqual(run['receipt']['controller']['status'],'stopped'if stop_during_model or wam32 and not normal32 else'completed',run['receipt'])
            if normal32 and not operator_stop:
                metrics=flow.snapshot()['inference']['metrics']
                self.assertEqual(metrics['returned_predictions'],len(manifest['model_results']))
                self.assertGreater(metrics['accepted_predictions'],0,'normal MODEL admissions must be visible in UI metrics')
                self.assertEqual(run['config']['execute_steps'],execute_steps)
                self.assertAlmostEqual(run['config']['reference_planning']['max_execution_duration_s'],execute_steps/30*4)
                segment=run['runner'].snapshot()['normal_model_segment']
                self.assertEqual(segment['complete_source_steps'],execute_steps)
                self.assertEqual(run['runner'].snapshot()['mode'],'HOLD')
            if wam32 and not normal32 and not stop_during_model:
                self.assertEqual(run['runner'].snapshot()['finite_chunks']['completed_count'],1 if expire_supply else 3)
                if expire_supply:
                    self.assertEqual(client.refresh()['state'],'STOPPED_DAMPING_HOLD')
                    self.assertIn('freshness deadline',run['receipt']['controller']['reason'])
            if stop_during_model:
                self.assertEqual(run['runner'].snapshot()['actual_submission_count'],stopped_submissions,
                    'late WAM output produced an SDK submission after confirmed Stop')
            self.assertEqual(manifest['recording_outcome'],'complete',manifest['issues'])
            self.assertEqual(manifest['binding']['execution_owner']['owner_pid'],self.process.pid)
            self.assertNotEqual(run['recorder'].session.process.pid,self.process.pid)
            self.assertTrue(manifest['model_inputs']);self.assertEqual(set(manifest['model_inputs']),set(manifest['model_results']))
            self.assertEqual(len(model.requests),len(manifest['model_results'])+1,'one prepare warmup plus one call per recorded request')
            self.assertEqual(run['recorder'].session.publish_calls,0)
            self.assertIsNone(flow.input_evidence);self.assertFalse(list(flow.output.rglob('input.npz')))
            self.assertEqual(len(manifest['playback']['streams']),3)
            if operator_teach:
                from robot_workbench.service import Application
                parent.workflows['inference']=flow
                app=Application({'fake':parent})
                flow.operator_controls=dict(home_velocity_rad_s=.1,timeout_s=30.,focus_timeout_s=8.)
                def operator(action,identity):
                    facts=client.refresh()
                    return app.action(action,dict(environment='fake',business='inference',request_id=identity,
                        runtime_nonce=client.nonce,epoch=facts['command_epoch']))
                calls_before=len(model.requests)
                def session_run(prepare_only):
                    view=app.state('fake','inference');info=view['workflow']['inference']
                    response=app.action('inference_run',dict(environment='fake',business='inference',request_id=uuid.uuid4().hex,
                        seed=1,duration_s=5.,model_id=info['selected_model_id'],expected_bundle_digest=info['bundle_digest'],
                        expected_context=info['run_context'],state_wall_ns=view['server_wall_ns'],
                        physical_clearance=True,scene_reset=True,prepare_only=prepare_only))
                    deadline=time.monotonic()+25
                    while app.jobs[('fake','inference')]['status']=='running':
                        self.assertLess(time.monotonic(),deadline,app.jobs[('fake','inference')]);time.sleep(.02)
                    self.assertEqual(app.jobs[('fake','inference')]['status'],'completed',app.jobs[('fake','inference')])
                    self.assertEqual(app.jobs[('fake','inference')]['id'],response['job_id'])
                if prepared_session:
                    import uuid
                    session_run(True)
                    self.assertEqual(len(model.requests),calls_before)
                    prepared_run=flow.current
                    if pedal_gestures:
                        flow.operator_controls['double_press_window_s']=.5
                        sequence=[0];focus=dict(client_id='cpu-page',started_wall=time.time())
                        def press(command):
                            sequence[0]+=1
                            return app._operator_pedal_action(parent,flow,command,
                                SimpleNamespace(source_instance_id='cpu-pedal',sequence=sequence[0]),time.time(),focus)
                        def wait_mode(mode):
                            deadline=time.monotonic()+8
                            while True:
                                value=prepared_run['runner'].snapshot()
                                if value.get('mode')==mode and not value.get('transition_stage'):return value
                                receipt=value.get('mode_receipts',{}).get('pedal-cpu-pedal-'+str(sequence[0]),{})
                                if mode=='MODEL'and receipt.get('status')=='failed':
                                    cache=flow.cache or{};latest_ref,latest_state=flow.ros_inputs.latest('state')
                                    self.fail(dict(receipt=receipt,cache_error=flow.cache_error,cached_owner=cache.get('owner'),
                                        latest_state_reference=latest_ref.__dict__,
                                        latest_state_owner={k:latest_state.get(k)for k in('runtime_nonce','command_epoch','owner_pid')},
                                        client_epoch=client.state.get('command_epoch'),model_binding=flow.ros_worker.current,
                                        ros_errors=list(flow.ros_node.errors),observation_thread_alive=flow.cache_thread.is_alive()))
                                self.assertNotEqual(value.get('runner_status'),'FAULT',value)
                                self.assertLess(time.monotonic(),deadline,value);time.sleep(.01)
                        def wait_job():
                            deadline=time.monotonic()+8
                            while app.jobs[('fake','inference')]['status']=='running':
                                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                            self.assertEqual(app.jobs[('fake','inference')]['status'],'completed',app.jobs[('fake','inference')])
                        entered=threading.Event();resume=threading.Event();reason=app.inference_run_reason
                        def delay_admission(*args,**kwargs):
                            result=reason(*args,**kwargs)
                            if kwargs.get('ignore_job'):
                                entered.set()
                                if not resume.wait(3):raise TimeoutError('CPU Run admission barrier')
                            return result
                        before_requests=len(model.requests)
                        with patch.object(app,'inference_run_reason',side_effect=delay_admission):
                            press(11);time.sleep(.3);press(11)
                            try:
                                self.assertTrue(entered.wait(2));time.sleep(.3);press(11)
                            finally:resume.set()
                            wait_job();wait_mode('HOLD')
                        self.assertEqual(len(model.requests),before_requests,'early Stop must survive the prepared-session cleanup')
                        self.assertFalse(prepared_run['recorder'].done.is_set())
                        time.sleep(.55)
                        for stop_command in (22,11):
                            press(11);time.sleep(.3);started=press(11)
                            self.assertIn('job_id',started)
                            value=wait_mode('MODEL')
                            self.assertIn('pedal-cpu-pedal-'+str(sequence[0]),value['mode_receipts'])
                            self.assertFalse(press(33)['accepted'])
                            press(stop_command)
                            if stop_command==11:
                                time.sleep(.3);self.assertFalse(press(11)['accepted'])
                            wait_mode('HOLD');wait_job()
                            self.assertEqual(app.jobs[('fake','inference')]['id'],started['job_id'])
                            self.assertIs(flow.current,prepared_run)
                            time.sleep(.55)
                        receipts=prepared_run['runner'].snapshot()['mode_receipts'].values()
                        self.assertFalse(any(row.get('target')in('TEACH','HOME')for row in receipts))
                        app.stop_device(parent)
                        self.assertTrue(prepared_run['recorder'].done.wait(15))
                        recorded=prepared_run['recorder'].manifest
                        self.assertEqual(recorded['recording_outcome'],'complete',recorded['issues'])
                        self.assertGreaterEqual(len(recorded['model_inputs']),2)
                        self.assertEqual(set(recorded['model_inputs']),set(recorded['model_results']))
                        self.assertEqual(len(recorded['playback']['streams']),3)
                        self.assertTrue(client.refresh()['connected'])
                        return
                facts=client.refresh()
                start_body=dict(environment='fake',business='inference',request_id='cpu-drag-start',
                    runtime_nonce=client.nonce,epoch=facts['command_epoch'])
                accepted=app.action('inference_teach',start_body)
                deadline=time.monotonic()+35
                while app.operator_tasks['fake']['phase']!='teaching':
                    task=app.operator_tasks['fake']
                    self.assertEqual(task['status'],'running',task)
                    self.assertLess(time.monotonic(),deadline,task)
                    time.sleep(.02)
                self.assertFalse(app.state('fake','inference')['runtime']['allowed_operations']['inference_run']['allowed'])
                manual_run=flow.current
                if prepared_session:self.assertIs(manual_run,prepared_run)
                self.assertEqual(manual_run['runner'].snapshot()['mode'],'TEACH')
                self.assertEqual(app.action('inference_teach',start_body),accepted,'duplicate press identity must not toggle twice')
                lower=client.refresh()['joint_limits'][3][0]
                dragged=[.04,None,None,lower-.004]if manual_boundary else[.04]
                (Path(self.temp.name)/'manual-drag-q.json').write_text(json.dumps(dragged))
                deadline=time.monotonic()+2
                while abs(client.refresh()['calibration_feedback']['qpos'][0]-.04)>1e-8:
                    self.assertLess(time.monotonic(),deadline)
                    time.sleep(.02)
                time.sleep(.35)
                if teach_stop:
                    app.stop_device(parent)
                    deadline=time.monotonic()+5
                    while app.operator_tasks['fake']['status']=='running':
                        self.assertLess(time.monotonic(),deadline)
                        time.sleep(.02)
                    self.assertEqual(app.operator_tasks['fake']['status'],'stopped')
                    self.assertTrue(manual_run['recorder'].done.wait(15))
                    self.assertEqual(len(model.requests),calls_before)
                    self.assertEqual(manual_run['runner'].snapshot()['runner_status'],'STOPPED')
                    self.assertTrue(client.refresh()['connected'])
                    return
                operator('inference_teach','cpu-drag-finish')
                deadline=time.monotonic()+35
                while app.operator_tasks['fake']['status']=='running':
                    self.assertLess(time.monotonic(),deadline,app.operator_tasks['fake'])
                    time.sleep(.02)
                task=app.operator_tasks['fake']
                self.assertEqual(task['status'],'completed',task)
                self.assertEqual(task['phase'],'start_ready')
                self.assertEqual(len(model.requests),calls_before,'manual positioning must never call the model')
                self.assertAlmostEqual(client.refresh()['calibration_feedback']['qpos'][0],.04,places=6)
                if manual_boundary:
                    mode=manual_run['runner'].snapshot()
                    held=[v for v in mode['mode_receipts'].values()if v.get('requested_mode')=='HOLD'and v.get('completed')][-1]
                    self.assertGreaterEqual(held['native_output']['q_sdk14'][3],lower)
                    self.assertTrue(all(held['native_output']['gains'][side]['gripper_kp']==5. for side in ('left','right')))
                    flow.request_normal_mode(dict(session_id=mode['mode_session_id'],runtime_nonce=client.nonce,
                        request_id='cpu-boundary-end',epoch=mode['command_epoch'],mode='HOLD',target='END'))
                    self.assertTrue(manual_run['recorder'].done.wait(15))
                    self.assertEqual(manual_run['recorder'].manifest['recording_outcome'],'complete')
                    self.assertEqual(len(model.requests),calls_before)
                    self.assertTrue(client.refresh()['connected'])
                    return
                if prepared_session:
                    self.assertIsNone(manual_run['receipt'])
                    self.assertFalse(manual_run['recorder'].done.is_set())
                    self.assertTrue(app.state('fake','inference')['runtime']['allowed_operations']['inference_run']['allowed'])
                    session_run(False)
                    self.assertIs(flow.current,manual_run)
                    stats=wam.normalization(json.loads(bundle['normalization_json']))
                    expected=wam.prepare_state(np.array([.04]+[0.]*13),stats)[0]
                    self.assertEqual(model.requests[calls_before]['observation']['model_state14'][0],float(expected))
                    self.assertEqual(manual_run['runner'].snapshot()['mode'],'HOLD')
                    self.assertFalse(manual_run['recorder'].done.is_set())
                    if native_home:
                        facts=client.refresh()
                        app.action('inference_home',dict(environment='fake',business='inference',request_id='cpu-native-home',
                            runtime_nonce=client.nonce,epoch=facts['command_epoch'],physical_clearance=True))
                        deadline=time.monotonic()+10
                        if stop_home:
                            while manual_run['runner'].snapshot().get('transition_stage')!='NATIVE_HOME':
                                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                            waiting=operator('inference_hold','cpu-normal-home-wait')
                            self.assertTrue(waiting['waiting_native_home'])
                            app.stop_device(parent)
                        while app.operator_tasks['fake']['status']=='running':
                            self.assertLess(time.monotonic(),deadline,app.operator_tasks['fake']);time.sleep(.02)
                        self.assertEqual(app.operator_tasks['fake']['status'],'stopped'if stop_home else'completed',app.operator_tasks['fake'])
                        if not stop_home:
                            self.assertIs(flow.current,manual_run)
                            self.assertAlmostEqual(client.refresh()['calibration_feedback']['qpos'][0],0.,places=6)
                    app.stop_device(parent)
                    self.assertTrue(manual_run['recorder'].done.wait(15))
                    self.assertTrue(client.refresh()['connected'])
                    return
                manual_manifest=manual_run['recorder'].manifest
                self.assertEqual(manual_manifest['recording_outcome'],'complete',manual_manifest.get('issues'))
                self.assertEqual(manual_manifest['model_inputs'],{})
                self.assertEqual(len(manual_manifest['playback']['streams']),3)
                self.assertTrue(client.refresh()['connected'])
                flow.bind_device()
                next_review=flow.preview_episode(seed=1,duration_s=5.)
                flow.execute_review(next_review['preview']['token'])
                app.run_normal_episode(parent,flow,{},lambda:True,client,client.nonce,5.)
                stats=wam.normalization(json.loads(bundle['normalization_json']))
                expected=wam.prepare_state(np.array([.04]+[0.]*13),stats)[0]
                self.assertEqual(model.requests[calls_before]['observation']['model_state14'][0],float(expected))
                self.assertNotEqual(flow.current['id'],manual_run['id'])
            if operator_home:
                from robot_workbench.service import Application
                parent.workflows['inference']=flow;app=Application({'fake':parent})
                flow.operator_controls=dict(home_velocity_rad_s=.1,timeout_s=20.,focus_timeout_s=8.)
                facts=client.refresh()
                app.action('inference_home',dict(environment='fake',business='inference',request_id='cpu-home',
                    runtime_nonce=client.nonce,epoch=facts['command_epoch'],physical_clearance=True))
                original_home=app.operator_tasks['fake']
                deadline=time.monotonic()+25
                if stop_home:
                    while not(original_home.get('run_id')and client.refresh().get('automatic_enabled')):
                        self.assertLess(time.monotonic(),deadline,original_home);time.sleep(.01)
                    facts=client.refresh()
                    app.action('inference_hold',dict(environment='fake',business='inference',request_id='cpu-stop-home',
                        runtime_nonce=client.nonce,epoch=facts['command_epoch']))
                while app.operator_tasks['fake']['status']=='running':
                    self.assertLess(time.monotonic(),deadline,app.operator_tasks['fake']);time.sleep(.02)
                home=app.operator_tasks['fake'];facts=client.refresh()
                self.assertEqual(home['status'],'completed',dict(home=home,facts={k:facts.get(k)for k in(
                    'state','motion','recording','execution_instance_id','stationary','reason')},recording_error=client.recording_error))
                self.assertTrue(client.refresh()['connected'])
                home_manifests=[]
                for path in Path(self.temp.name).rglob('manifest.json'):
                    value=json.loads(path.read_text())
                    if value.get('binding',{}).get('run_id')==home['run_id']:home_manifests.append(value)
                self.assertEqual(len(home_manifests),1,dict(home=home,state=client.refresh().get('recording')))
                self.assertEqual(home_manifests[0]['recording_outcome'],'complete',home_manifests[0].get('issues'))
                self.assertEqual(len(home_manifests[0]['playback']['streams']),3)
                if stop_home:
                    self.assertEqual(original_home['status'],'stopped',original_home)
                    self.assertNotEqual(original_home['run_id'],home['run_id'])
                    self.assertEqual(home['kind'],'hold')
                    self.assertEqual(home_manifests[0]['binding']['execution_context']['operator_intent'],'preview_operator_hold')
                    q=client.refresh()['calibration_feedback']['qpos'][0]
                    self.assertGreater(q,0.,'Stop must hold before the Home endpoint')
                    time.sleep(.3);self.assertAlmostEqual(client.refresh()['calibration_feedback']['qpos'][0],q,places=6)
        finally:
            model_continue.set()
            if flow is not None:
                client.request_stop()
                if flow.current and flow.current.get('recorder'):flow.current['recorder'].done.wait(30)
                flow.close()
            closed.set()
            if thread is not None:thread.join(3)
            camera.close();server.shutdown();server.server_close();serving.join(3);server.model_service.close()

    def cleanup(self):
        if hasattr(self,'channel')and self.process.is_alive():
            try:
                state=self.channel.call('inspect')
                if state.get('connected'):
                    self.channel.call('stop');self.wait(lambda s:s.get('stationary')and not s.get('automatic_enabled'))
                    lease=self.channel.call('maintenance_begin')
                    self.channel.call('release',token=lease['token'],arms_supported=True)
                    self.wait(lambda s:s.get('connected')is False)
            except Exception:pass
        if self.process.is_alive():self.closing.set()
        self.process.join(3)
        if self.process.is_alive():self.process.terminate();self.process.join(3)  # Only this CPU fake; no physical process.

    def wait(self,predicate,seconds=6):
        deadline=time.monotonic()+seconds;state=None
        while time.monotonic()<deadline:
            state=self.channel.call('snapshot',business='calibration')
            if predicate(state):return state
            time.sleep(.02)
        self.fail('CPU Owner did not reach expected state: '+str({k:state.get(k)for k in ('state','reason','automatic_enabled','motion')}))

    def acquire(self):
        self.channel.call('acquire',arms_supported=True)
        return self.wait(lambda s:s.get('feedback_healthy')and s.get('stationary'))

    def test_inert_start_explicit_acquire_and_current_service_authentication(self):
        self.assertFalse(self.channel.call('inspect')['connected'])
        with self.assertRaisesRegex(ValueError,'supported'):self.channel.call('acquire',arms_supported=False)
        state=self.acquire();self.assertEqual(state['pid'],self.process.pid)
        self.assertTrue(state['target_interpreter']['enabled'])
        token=self.channel.token;self.channel.token='bad'
        with self.assertRaises(PermissionError):self.channel.call('inspect')
        self.channel.token=token

    def test_normal_release_consumes_business_lease_before_second_acquire(self):
        from robot_workbench.service import Device
        parent=Device('isolated-cpu','fake',lambda:ControlRuntimeClient(self.descriptor).acquire())
        self.addCleanup(parent.closed.set);parent.connect({})
        client=parent.runtime;before=client.nonce;client.begin_maintenance()
        client.channel.call('release',expected_nonce=before,token=client.lease['token'],arms_supported=True)
        self.wait(lambda s:s.get('connected')is False)
        deadline=time.monotonic()+4
        while time.monotonic()<deadline and(parent.runtime is not None or parent.thread.is_alive()):time.sleep(.01)
        self.assertIsNone(parent.runtime,parent.error);self.assertIsNone(client.lease)
        self.assertTrue(parent.release_receipt['maintenance_cleanup_confirmed'])
        parent.connect({});self.assertNotEqual(parent.runtime.nonce,before)
        self.assertTrue(parent.snapshot('calibration')['connected'])

    def test_research_is_an_explicit_holder_on_the_same_owner_and_sdk(self):
        initial=self.acquire();client=ControlRuntimeClient(self.descriptor);self.addCleanup(client.detach)
        calibration=client.register_business('calibration');research=client.register_business('research')
        client.handoff(calibration,'research');state=client.snapshot(research)
        self.assertEqual(state['holder'],'research');self.assertEqual(state['runtime_nonce'],initial['runtime_nonce'])
        self.assertEqual(state['pid'],initial['pid']);self.assertTrue(state['connected'])
        self.assertTrue(state['execution_settings']['settings']['speed_protection']['enabled'])
        with self.assertRaises(PermissionError):client.submit(calibration,dict(action='preview_workbench',waypoints=[]))
        row=dict(q12=[.01]+[0.]*11,speeds12=[.1]*12)
        review=client.submit(research,dict(action='preview_workbench',waypoints=[row]))
        client.submit(research,dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
        self.wait(lambda s:s.get('automatic_enabled'));self.channel.call('stop')
        done=self.wait(lambda s:s.get('stationary')and not s.get('automatic_enabled'))
        self.assertEqual(done['runtime_nonce'],initial['runtime_nonce'])

    def test_prepare_after_completed_return_clears_binding_via_actual_owner_admission(self):
        from types import SimpleNamespace as NS
        from robot_workbench.cup_return import clear
        self.acquire();client=ControlRuntimeClient(self.descriptor)
        self.addCleanup(client.detach);client.begin_maintenance()
        actor=NS(action='ops_prepare_collection',require=lambda runtime:None)
        root=Path(self.temp.name);binding=root/'binding.json';previous=root/'previous-binding.json'
        binding.write_text(json.dumps(dict(source_digest='completed-return')))
        previous.write_text(json.dumps(dict(source_digest='ordinary-collection')))
        state=client.refresh('collection')
        task=dict(submitted=True,submitted_monotonic_s=0.,timeline_digest='return',directory=str(root),
            parent_speed_scale=1.,parent_plan_id='parent',duration_s=2.,offset_xy_mm=[0.,0.],binding_path=str(binding),source_digest='completed-return',
            runtime_nonce=state['runtime_nonce'],command_epoch=state['command_epoch'])
        state.update(frozen=dict(timeline_digest='return',status='completed'),execution_calibration=dict(accepted_monotonic_s=1.))
        device=NS(runtime=client,cup_return_task=task,snapshot=lambda _:state)
        # The remote maintenance lease is authoritative; a local actor alone is insufficient.
        lease=client.lease;client.lease=None
        with self.assertRaisesRegex(PermissionError,'lease required'):clear(device,maintenance_actor=actor)
        self.assertEqual(json.loads(binding.read_text())['source_digest'],'completed-return')
        self.assertIs(device.cup_return_task,task)
        client.lease=lease
        clear(device,maintenance_actor=actor)
        self.assertEqual(json.loads(binding.read_text())['source_digest'],'ordinary-collection')
        self.assertIsNone(device.cup_return_task);self.assertEqual(client.refresh()['holder'],'collection')
        self.assertTrue(client.state['feedback_healthy']);self.assertFalse(client.state['automatic_enabled'])
        client.end_maintenance()

    def check_business_failure(self,mode):
        initial=self.acquire();business_ready=self.context.Queue()
        process=self.context.Process(target=_business_process,args=(self.descriptor,business_ready,mode));process.start()
        self.addCleanup(lambda:process.is_alive()and(process.terminate(),process.join(2)))
        start=business_ready.get(timeout=5);self.assertNotEqual(start['pid'],self.process.pid)
        active=self.wait(lambda s:s.get('automatic_enabled'))
        time.sleep(.7)
        current=self.channel.call('snapshot',business='calibration')
        self.assertEqual(current['runtime_nonce'],initial['runtime_nonce'])
        self.assertTrue(current['feedback_healthy']);self.assertNotIn('FAULT',current['state'])
        self.assertGreater(current['calibration_feedback']['qpos'][0],active['calibration_feedback']['qpos'][0])
        final=self.wait(lambda s:(s.get('motion')or{}).get('status')=='READY',seconds=8)
        self.assertLess(abs(final['calibration_feedback']['qpos'][0]-.04),1e-4)
        process.join(3);self.assertFalse(process.is_alive());self.assertEqual(process.exitcode,0)

    def test_business_long_gc_does_not_pause_actual_owner(self):self.check_business_failure('gc')
    def test_business_blocked_http_does_not_pause_actual_owner(self):self.check_business_failure('http')
    def test_business_exit_does_not_kill_owner_or_replay_motion(self):self.check_business_failure('exit')

    def test_existing_business_device_uses_remote_facts_and_rebinds_same_owner(self):
        from robot_workbench.service import Device,Application
        parent=Device('isolated-cpu','fake',lambda:ControlRuntimeClient(self.descriptor).acquire())
        app=Application({'fake':parent});parent.connect({})
        self.addCleanup(parent.closed.set)
        first=parent.snapshot('calibration');self.assertEqual(first['pid'],self.process.pid)
        self.assertNotEqual(first['pid'],os.getpid());self.assertTrue(first['target_interpreter']['enabled'])
        from unittest.mock import patch
        with patch.object(parent.runtime.channel,'call',side_effect=ConnectionError('CPU temporary IPC outage')):
            unknown=parent.snapshot('calibration')
            self.assertIsNone(unknown['connected']);self.assertEqual(unknown['state'],'UNKNOWN')
            self.assertFalse(unknown['allowed_operations']['confirm']['allowed'])
            self.assertTrue(unknown['allowed_operations']['stop']['allowed'])
        self.assertEqual(parent.snapshot('calibration')['runtime_nonce'],first['runtime_nonce'])
        parent.runtime.detach();parent.closed.set();parent.thread.join(2)
        restored=Device('isolated-cpu','fake',None)
        restored.attach_existing_owner(ControlRuntimeClient(self.descriptor))
        self.addCleanup(restored.closed.set)
        second=restored.snapshot('calibration')
        self.assertEqual(second['runtime_nonce'],first['runtime_nonce'])
        self.assertEqual(second['pid'],first['pid'])
        self.assertFalse(second['automatic_enabled'])

    def inference_client(self,*,finite,chunk=False,capture=False,input_age=None):
        import numpy as np
        from test_physical_motiongen import fake_inputs
        from robot_workbench.inference_workflow import calibrated_adapter
        from robot_workbench.inference_owner import device_state
        from robot_workbench.motion_reference import prepare_reference
        from robot_workbench.owner_inference import reference_wire
        self.acquire();client=ControlRuntimeClient(self.descriptor)
        self.addCleanup(client.detach)
        calibration=client.register_business('calibration');inference=client.register_business('inference')
        client.handoff(calibration,'inference');inputs=client.control_inputs()
        source,_,_=fake_inputs();profile=source['calibration_profile']
        profile['gripper']['max_opening_mm']=83.
        model='base379-epoch010-step29450'
        adapter,mapping=calibrated_adapter(profile,inputs['joint_limits'],model_id=model)
        config=dict(seed=1,model_id=model,duration_s=2 if finite else 40,recording_enabled=False,
            disabled_grippers=[],conversion_identity=dict(mapping=mapping),reference_supply_timeout_s=.5)
        if capture:config['recording_backend']='capture_session_v1'
        if input_age is not None:config['state_max_age_s']=input_age
        if chunk:
            contract=dict(schema='workbench.model-action14.v1',shape=[60,14],
                axes=[f'{s}_{a}'for s in('left','right')for a in('j1','j2','j3','j4','j5','j6','jaw')],
                units=(['rad']*6+['m'])*2,action_type='absolute_position',coordinate_domain='training_model_state14',
                jaw_semantics='legacy_model_jaw_coordinate',normalized=False,device_mapping_applied=False,
                sampling=dict(nominal_period_s=1/30,time_origin='unassigned',first_sample_offset_s=None,
                    basis='existing_workbench_nominal_grid',execution_timestamps=False),history_steps=60)
            # Frozen by this CPU service-preparation fixture, never from a result.
            config['action_chunk_binding']=dict(session_id='cpu-model-session',model_binding={'instance_id':'cpu-model'},
                conversion_id='cpu-conversion',action_contract=contract,task='cpu-task')
            config.pop('reference_supply_timeout_s')
            client.cpu_chunk_binding=config['action_chunk_binding'];client.cpu_adapter=adapter
        reference=None
        if finite:
            device=device_state(inputs['calibration_feedback']['qpos']);rows=np.tile(device,(61,1));rows[:,0]+=.02*np.linspace(0,1,61)
            reference=prepare_reference(dict(device_targets14=rows.tolist(),duration_s=2.),
                inputs['calibration_feedback']['qpos'],inputs['joint_limits'],source_sha256='a'*64)
            config.update(prepared_reference=reference.metadata,trajectory_kind='prepared_native_reference',execution_duration_s=reference.duration_s+.14)
        events=[];captures=[]
        if capture:
            from robot_workbench.inference_evidence import InferenceEvidence
            inputs=InferenceEvidence(Path(self.temp.name)/'capture-inputs')
            self.addCleanup(inputs.close)
            self.addCleanup(lambda:[value.close() for value in captures])
        def created(owner,runner,configuration,policy=None):
            events.append(dict(kind='created',pid=owner.cache['pid'],policy=policy))
            if capture:
                from types import SimpleNamespace
                from robot_workbench.inference_trace import CaptureRun
                value=CaptureRun(runner.identity,Path(self.temp.name)/('capture-'+runner.identity),configuration,policy,
                    SimpleNamespace(capture_sources=lambda:[]),'fake',inputs,lambda:True,lambda _:None)
                captures.append(value);client.test_capture=value;client.test_capture_policy=policy;client.attach_recording(value)
        def recorded(name,value):
            if capture:client.test_capture.record(name,value)
        def predict(_):raise AssertionError('finite reference never invokes model; starvation uses no observation')
        client.configure_remote_inference(config,dict(profile=profile,reference=reference_wire(reference),
            preloaded_prediction=None,feedforward=None),created=created,metric=lambda *args:events.append(args),record=recorded,predict=predict,
            predict_chunk=(lambda request,context:predict(request))if chunk else None)
        review=client.submit(inference,dict(action='preview_inference',duration_s=config['duration_s'],seed=1))
        client.submit(inference,dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
        return client,events

    def test_owner_input_guard_uses_the_run_limit_instead_of_the_old_point1_default(self):
        client,_=self.inference_client(finite=True,input_age=.03)
        inputs=client.control_inputs();now=inputs['control_time_s']
        observation=dict(source_time=now,device_state14=[0.]*14,frame_ids={},feedback_sample_time_s=now-.06)
        with self.assertRaisesRegex(ValueError,'G03'):
            client._call('inference_observation',session_id=client.inference_session,command_epoch=inputs['command_epoch'],
                observation=observation,support=None)
        inputs=client.control_inputs();now=inputs['control_time_s']
        observation.update(source_time=now,feedback_sample_time_s=now)
        self.assertTrue(client._call('inference_observation',session_id=client.inference_session,
            command_epoch=inputs['command_epoch'],observation=observation,support=None)['accepted'])

    def test_common_capture_keeps_owner_policy_and_rejects_previous_run_facts(self):
        client,events=self.inference_client(finite=True,capture=True)
        deadline=time.monotonic()+5
        while not hasattr(client,'test_capture') and time.monotonic()<deadline:time.sleep(.01)
        self.assertTrue(hasattr(client,'test_capture'))
        capture=client.test_capture
        with self.assertRaisesRegex(ValueError,'another Run'):
            client._call('inference_recording',session_id=client.inference_session,execution_instance_id='previous-run',
                         prepared=True,done=True,alive=False,error=None,ingress_closed=True)
        self.wait(lambda state:(state.get('motion')or{}).get('status')=='READY',seconds=8)
        self.assertTrue(capture.done.wait(10),capture.snapshot())
        self.assertEqual(capture.manifest['binding']['policy_hash'],client.test_capture_policy['policy_hash'])
        self.assertEqual(capture.manifest['recording_outcome'],'complete')
        self.assertGreater(capture.manifest['streams']['control']['committed'],1)

    def test_common_capture_stop_waits_for_the_ordered_final_sequence(self):
        client,_=self.inference_client(finite=True,capture=True)
        self.wait(lambda state:state.get('automatic_enabled'))
        self.channel.call('stop')
        deadline=time.monotonic()+10
        while not hasattr(client,'test_capture') and time.monotonic()<deadline:time.sleep(.01)
        capture=client.test_capture;self.assertTrue(capture.done.wait(10),capture.snapshot())
        self.assertEqual(capture.manifest['control_terminal']['status'],'stopped')
        self.assertEqual(capture.manifest['streams']['control']['missing'],0)
        self.assertEqual(capture.manifest['recording_outcome'],'complete')

    def test_action_chunk_crosses_real_client_pending_ledger_and_rejects_tampering(self):
        from copy import deepcopy
        import numpy as np
        from robot_workbench.inference_owner import device_state
        client,events=self.inference_client(finite=False,chunk=True)
        observed=threading.Event();sent=[]
        def predict(request,context):
            self.assertEqual(set(context),{'epoch','clock_domain','valid_until_source_time'})
            self.assertTrue(context['clock_domain'].startswith('owner:'+client.nonce+':'))
            self.assertEqual(context['valid_until_source_time'],request['observation']['source_time']+.5)
            rows=np.tile(client.cpu_adapter.device_to_model(request['observation']['device_state14']),(60,1))
            response=dict(deepcopy(client.cpu_chunk_binding),schema='workbench.action-chunk.v1',request_id=request['request_id'],
                epoch=context['epoch'],action14=rows.tolist(),audit={},execution_authorized=False,
                observation=dict(source_time=request['observation']['source_time'],clock_domain=context['clock_domain'],
                    frame_ids=deepcopy(request['observation']['frame_ids'])),
                validity=dict(clock_domain=context['clock_domain'],valid_until_source_time=context['valid_until_source_time']))
            with self.assertRaises(ValueError):client._call('inference_result',session_id=client.inference_session,
                request_id=request['request_id'],response=dict(response,session_id='wrong-model-session'),error=None)
            sent.append(response);observed.set();return response
        client.inference_callbacks['predict_chunk']=predict
        deadline=time.monotonic()+3
        while time.monotonic()<deadline and not observed.is_set():
            inputs=client.control_inputs();fb=inputs['calibration_feedback']
            client.push_observation(dict(source_time=fb['sample_time_s'],device_state14=device_state(fb['qpos']),
                frame_ids={'cpu':{'frame':1}},feedback_sample_time_s=fb['sample_time_s'],images={'cpu':'original'}),None,inputs['command_epoch'])
            time.sleep(.01)
        self.assertTrue(observed.is_set(),client.recording_error)
        self.wait(lambda state:(state.get('inference')or{}).get('actual_submission_count',0)>2,seconds=5)
        with self.assertRaises(ValueError):client._call('inference_result',session_id=client.inference_session,
            request_id=sent[0]['request_id'],response=sent[0],error=None)
        self.channel.call('stop')

    def test_inference_finite_reference_outlives_business_detach(self):
        client,events=self.inference_client(finite=True)
        self.wait(lambda s:s.get('automatic_enabled'))
        client.detach()
        final=self.wait(lambda s:(s.get('motion')or{}).get('status')=='READY',seconds=8)
        self.assertLess(abs(final['calibration_feedback']['qpos'][0]-.02),1e-4)
        self.assertTrue(final['feedback_healthy'])

    def test_inference_reference_supply_loss_stops_without_fake_heartbeat(self):
        client,_=self.inference_client(finite=False)
        final=self.wait(lambda s:'reference supply stopped'in str(s.get('reason')),seconds=5)
        self.assertFalse(final['automatic_enabled'])
        self.assertTrue(final['feedback_healthy'])

    def test_online_prediction_bridge_keeps_pixels_in_business_and_stop_revokes_late_result(self):
        import numpy as np
        from robot_workbench.inference_owner import device_state
        client,events=self.inference_client(finite=False)
        entered=threading.Event();release=threading.Event();finished=threading.Event()
        self.addCleanup(release.set)
        def predict(request):
            self.assertEqual(request['observation']['images'],{'cpu':'business-only-pixels'})
            entered.set();release.wait(3);finished.set()
            return dict(request_id=request['request_id'],action14=np.zeros((60,14)))
        client.inference_callbacks['predict']=predict
        deadline=time.monotonic()+2
        while time.monotonic()<deadline and not entered.is_set():
            inputs=client.control_inputs();fb=inputs['calibration_feedback']
            client.push_observation(dict(source_time=fb['sample_time_s']-.01,device_state14=device_state(fb['qpos']),
                frame_ids={'cpu':{'frame':1}},feedback_sample_time_s=fb['sample_time_s'],images={'cpu':'business-only-pixels'}),None,inputs['command_epoch'])
            time.sleep(.01)
        self.assertTrue(entered.is_set(),client.recording_error)
        self.channel.call('stop');stopped=self.wait(lambda s:s.get('state')=='STOPPED_DAMPING_HOLD')
        before=client._call('inference_state')['snapshot']['actual_submission_count']
        release.set();self.assertTrue(finished.wait(2));time.sleep(.25)
        after=client._call('inference_state')['snapshot']
        self.assertEqual(after['actual_submission_count'],before);self.assertEqual(after['runner_status'],'STOPPED')
        self.assertFalse(stopped['automatic_enabled'])

    def test_independent_manager_stop_reaches_owner_with_no_business_http(self):
        from robot_workbench.management import WorkbenchManager
        self.acquire()
        manager=WorkbenchManager.__new__(WorkbenchManager)
        manager.config=dict(device_id='isolated-cpu',control_owner_descriptor=self.descriptor)
        manager.stop_generation=0
        result=manager.stop(dict(expected_device_id='isolated-cpu',request_id='cpu-direct-stop'))
        self.assertEqual(result['status'],'accepted')
        state=self.wait(lambda s:s.get('state')=='STOPPED_DAMPING_HOLD')
        self.assertFalse(state['automatic_enabled'])

    def test_successful_stop_encoding_failure_is_unknown(self):
        from robot_workbench.control_owner import ControlCallUnknown
        self.acquire()
        with self.assertRaises(ControlCallUnknown)as result:self.channel.call('stop')
        state=self.wait(lambda s:s.get('state')=='STOPPED_DAMPING_HOLD')
        receipt=self.channel.call('receipt',request_id=result.exception.request_id)
        self.assertTrue(receipt['done']);self.assertNotIn('error',receipt)
        self.assertEqual(receipt['operation'],'stop');self.assertFalse(state['automatic_enabled'])

    def test_event_large_reads_do_not_starve_owner_or_guard(self):
        from concurrent.futures import ThreadPoolExecutor
        self.acquire();ready=self.context.Queue();closed=self.context.Event()
        guard=self.context.Process(target=_guard_process,args=(self.heartbeat,self.tripped,ready,closed));guard.start()
        try:
            self.assertTrue(ready.get(timeout=3)['armed'])
            state=self.channel.call('snapshot',business='calibration');q=state['calibration_feedback']['qpos'][:];q[0]=.04
            review=self.channel.call('submit',business='calibration',command=dict(action='preview',active_arm='left',q_active=q[:6],max_velocity=.03))
            self.channel.call('submit',business='calibration',command=dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
            self.wait(lambda s:s.get('automatic_enabled'))
            def read():
                for _ in range(8):
                    batch=ControlChannel(self.descriptor).call('events',after=0)
                    self.assertEqual(batch['events'][0]['sequence'],1)
                    self.assertEqual(batch['events'][0]['event'],'fault_diagnostic')
                    self.assertFalse(batch['incomplete'])
            with ThreadPoolExecutor(max_workers=2)as pool:
                tasks=[pool.submit(read)for _ in range(2)]
                for task in tasks:task.result(timeout=15)
            final=self.wait(lambda s:(s.get('motion')or{}).get('status')=='READY',seconds=8)
            self.assertLess(abs(final['calibration_feedback']['qpos'][0]-.04),1e-4)
            self.assertFalse(self.tripped.value);self.assertTrue(final['feedback_healthy'])
        finally:
            closed.set();guard.join(3)
            if guard.is_alive():guard.terminate();guard.join(2)

    def test_event_encoder_exit_keeps_stop_and_maintenance_available(self):
        import signal
        self.acquire();encoder=self.channel.call('inspect')['control_owner']['event_encoding']
        self.assertEqual(encoder['mode'],'spawned_cpu');self.assertNotEqual(encoder['pid'],self.process.pid)
        os.kill(encoder['pid'],signal.SIGTERM)
        with self.assertRaises(ValueError):self.channel.call('events',after=0)
        self.assertTrue(self.channel.call('inspect')['feedback_healthy'])
        self.channel.call('stop');self.wait(lambda s:s.get('state')=='STOPPED_DAMPING_HOLD')
        lease=self.channel.call('maintenance_begin')
        self.channel.call('release',token=lease['token'],arms_supported=True)
        self.wait(lambda s:not s['connected'])
        self.closing.set();self.process.join(4);self.assertEqual(self.process.exitcode,0)

    def test_read_encoder_exit_keeps_stop_and_event_evidence_independent(self):
        import signal
        self.acquire();facts=self.channel.call('inspect')['control_owner']
        self.assertEqual(facts['read_encoding']['mode'],'spawned_cpu')
        self.assertNotEqual(facts['read_encoding']['pid'],facts['event_encoding']['pid'])
        os.kill(facts['read_encoding']['pid'],signal.SIGTERM)
        with self.assertRaises(ValueError):self.channel.call('inspect')
        self.channel.call('stop')
        end=time.monotonic()+2
        while time.monotonic()<end:
            rows=self.channel.call('events',after=0)['events']
            if any(r['event']=='stop'for r in rows):break
            time.sleep(.01)
        else:self.fail('Stop did not reach Owner after facts encoder exit')
        lease=self.channel.call('maintenance_begin')
        result=self.channel.call('release',token=lease['token'],arms_supported=True)
        self.assertEqual(result['queued'],'release')

    def test_read_encoder_delay_does_not_stall_tick_or_stop_and_preserves_sample_age(self):
        self.acquire();results=[]
        (Path(self.temp.name)/'delay-read').touch()
        def read():
            results.append(self.channel.call('snapshot',business='calibration'))
        reader=threading.Thread(target=read);reader.start()
        end=time.monotonic()+1
        while not(Path(self.temp.name)/'read-delayed').exists()and time.monotonic()<end:time.sleep(.002)
        self.assertTrue((Path(self.temp.name)/'read-delayed').exists())
        initial=self.heartbeat.value;time.sleep(.06)
        self.assertGreater(self.heartbeat.value,initial)
        self.assertLess(time.monotonic()-self.heartbeat.value,.1)
        started=time.monotonic();self.channel.call('stop')
        self.assertLess(time.monotonic()-started,.1)
        reader.join(2);self.assertFalse(reader.is_alive());self.assertEqual(len(results),1)
        self.assertGreaterEqual(results[0]['control_owner']['reply_age_s'],.2)
        self.assertGreaterEqual(time.monotonic()-results[0]['calibration_feedback']['sample_time_s'],.2)
        state=self.wait(lambda s:s.get('state')=='STOPPED_DAMPING_HOLD')
        self.assertTrue(state['feedback_healthy']);self.assertFalse(state['automatic_enabled'])

    def test_concurrent_state_readers_wait_outside_control_instead_of_failing(self):
        from concurrent.futures import ThreadPoolExecutor
        self.acquire();(Path(self.temp.name)/'delay-read').write_text('.06')
        gate=threading.Barrier(3)
        def read():
            gate.wait();return self.channel.call('snapshot',business='calibration')
        before=self.heartbeat.value
        with ThreadPoolExecutor(max_workers=3)as pool:
            futures=[pool.submit(read)for _ in range(3)]
            values=[future.result(2)for future in futures]
        self.assertEqual(len(values),3);self.assertTrue(all(v['feedback_healthy']for v in values))
        self.assertGreater(self.heartbeat.value,before)

    @unittest.skipUnless(sys.platform=='linux','Linux parent-death lifecycle')
    def test_event_encoder_follows_owner_lifetime(self):
        import signal
        facts=self.channel.call('inspect')['control_owner']
        encoders=[facts[k]['pid']for k in('event_encoding','read_encoding')]
        os.kill(self.process.pid,signal.SIGKILL);self.process.join(3)
        end=time.monotonic()+3
        while time.monotonic()<end:
            live=[]
            for encoder in encoders:
                try:state=Path('/proc/'+str(encoder)+'/stat').read_text().rsplit(')',1)[1].split()[0]
                except (FileNotFoundError,ProcessLookupError):continue
                if state!='Z':live.append(encoder)
            if not live:return
            time.sleep(.02)
        self.fail('CPU event encoder survived Owner process exit')

    def test_event_encoder_normal_shutdown(self):
        from unittest.mock import patch
        import robot_workbench.control_owner as transport
        from robot_workbench.service import Device
        with tempfile.TemporaryDirectory(prefix='wb-codec-',dir='/tmp')as directory:
            with patch.object(transport,'_identity',return_value=dict(pid=os.getpid())):
                service=transport.ControlOwnerService(Device('cpu-codec','fake',None),directory)
            serving=threading.Thread(target=service.serve_forever);serving.start()
            service.close();serving.join(2)
            self.assertFalse(serving.is_alive())
            with self.assertRaises(RuntimeError):service.event_encoder.submit(os.getpid)
            with self.assertRaises(RuntimeError):service.read_encoder.submit(os.getpid)
            self.assertFalse((Path(directory)/'control-token').exists())
            with patch.object(transport,'_identity',return_value=dict(pid=os.getpid())):
                restarted=transport.ControlOwnerService(Device('cpu-codec','fake',None),directory)
            self.assertNotEqual(restarted.instance,service.instance)
            self.assertNotEqual(restarted.token,service.token)
            serving=threading.Thread(target=restarted.serve_forever);serving.start()
            restarted.close();serving.join(2)
            self.assertFalse(serving.is_alive())

    def test_lost_mutating_response_is_unknown_and_never_replayed(self):
        import socket
        from robot_workbench.control_owner import ControlCallUnknown,SCHEMA
        path=Path(self.temp.name)/'dropped.sock';server=socket.socket(socket.AF_UNIX);server.bind(str(path));os.chmod(path,0o600);server.listen(2)
        descriptor=Path(self.temp.name)/'dropped.json';token=Path(self.temp.name)/'dropped.token'
        token.write_text('x'*40);token.chmod(0o600)
        descriptor.write_text(json.dumps(dict(schema=SCHEMA,environment='fake',instance_id='dropped',socket_path=str(path),token_file=str(token))))
        descriptor.chmod(0o600);calls=[]
        def drop():
            connection,_=server.accept()
            with connection:
                with connection.makefile('rb')as stream:calls.append(json.loads(stream.readline()))
        worker=threading.Thread(target=drop);worker.start()
        try:
            with self.assertRaises(ControlCallUnknown)as result:ControlChannel(descriptor).call('stop')
            worker.join(2);self.assertEqual(len(calls),1)
            self.assertEqual(calls[0]['request_id'],result.exception.request_id)
        finally:server.close()

    def test_actual_workflow_records_remote_execution_identity_and_completed_feedback(self,cycles=2,report=None):
        from unittest.mock import patch
        import numpy as np
        from test_inference_workflow import InferenceWorkflowTests,Recorder
        from robot_workbench.service import Device
        from robot_workbench.inference_owner import device_state
        from robot_workbench.inference_workflow import CAMERAS
        fixture=InferenceWorkflowTests('runTest');fixture.setUp();self.addCleanup(fixture.doCleanups)
        self.acquire();client=ControlRuntimeClient(self.descriptor).acquire()
        parent=Device('isolated-cpu','fake',None);parent.attach_existing_owner(client)
        self.addCleanup(parent.closed.set);self.addCleanup(client.detach)
        client.handoff(parent.credentials['calibration'],'inference')
        fixture.device=parent;fixture.workflow.device=parent;fixture.binding['robot_id']=parent.id
        def latest(camera):
            value=client.control_inputs()['calibration_feedback']
            role=next(role for role,name in CAMERAS.items()if name==camera)
            return dict(width=224,height=224,rgb=bytes(224*224*3),depth=bytes(224*224*2),metadata=dict(
                serial=fixture.binding['camera_serials'][role],environment='fake',capture_instance_id='remote-fixture',
                source_clock_id=value['source_clock_id'],clock_domain='simulation',source_time_ns=round((value['sample_time_s']-.01)*1e9),
                host_receive_ns=time.time_ns(),frame_number=1,depth_frame_number=1))
        fixture.cameras.latest_rgbd.side_effect=latest
        fixture.prepare();workflow=fixture.workflow
        workflow.reference_pin=dict(path='/cpu-only/not-loaded',sha256='a'*64);workflow.prepared_reference_execution=True
        class Recording(Recorder):
            def submit(self,name,**value):self.kwargs.setdefault('windows',[]).append((name,value))
        with patch('robot_workbench.recording_process.ProcessRecordingSession',Recording),patch(
                'robot_workbench.recording_process.export_playback_process',return_value=dict(files={})):
            for cycle in range(cycles):
                feedback=client.control_inputs()['calibration_feedback']
                device=device_state(feedback['qpos']);rows=np.tile(device,(61,1))
                rows[:,0]+=(.02 if cycle%2==0 else -.02)*np.linspace(0,1,61)
                workflow.reference=dict(model_id=workflow.selected_model_id,calibration_profile_digest=fixture.binding['calibration_profile_digest'],
                    duration_s=2.,device_targets14=rows.tolist())
                review=workflow.preview_episode(seed=3,duration_s=2)
                workflow.execute_review(review['preview']['token'])
                deadline=time.monotonic()+8
                while time.monotonic()<deadline:
                    if workflow.current and workflow.current.get('receipt')and not workflow.snapshot()['busy']:break
                    time.sleep(.02)
                self.assertIsNotNone(workflow.current)
                run=workflow.current
                self.assertIsNotNone(run['receipt'],client.recording_error)
                self.assertIsNone(run['recording_error'],run['receipt'])
                self.assertEqual(run['receipt']['controller']['status'],'completed')
                self.assertEqual(run['execution_owner']['owner_pid'],self.process.pid)
                self.assertTrue(run['recorder'].commands);self.assertGreater(run['feedback_count'],1)
                self.assertEqual(run['id'],client.refresh()['execution_instance_id'])
                if report:report(cycle+1,self.process.pid,run,client)

    @unittest.skipUnless(sys.platform=='linux','Linux process suspension and production guard')
    def test_actual_owner_pause_trips_original_100ms_guard_despite_healthy_motor_traffic(self):
        import signal
        self.acquire();ready=self.context.Queue();closed=self.context.Event()
        guard=self.context.Process(target=_guard_process,args=(self.heartbeat,self.tripped,ready,closed));guard.start()
        try:
            self.assertTrue(ready.get(timeout=3)['armed'])
            os.kill(self.process.pid,signal.SIGSTOP)
            receipt=ready.get(timeout=3)
            self.assertEqual(receipt['reason'],'SDK owner heartbeat lost')
            self.assertGreater(receipt['age_s'],.1);self.assertLess(receipt['age_s'],.2)
        finally:
            os.kill(self.process.pid,signal.SIGCONT);closed.set();guard.join(3)
            if guard.is_alive():guard.terminate();guard.join(2)
        final=self.wait(lambda s:s.get('automatic_enabled')is False and 'FAULT'in s['state'])
        self.assertEqual(final['reason'],'SDK owner heartbeat lost')

if __name__=='__main__':unittest.main()
