"""Actual Runtime/target interpreter in an independent CPU process; no SDK/CAN."""
from pathlib import Path
import gc,json,multiprocessing,os,sys,tempfile,threading,time,unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]
from robot_workbench.control_owner import ControlChannel,ControlRuntimeClient


class EventReadLockTest(unittest.TestCase):
    def test_event_reads_snapshot_without_parent_encoding_and_preserve_cursors(self):
        from collections import deque
        from types import SimpleNamespace
        from unittest.mock import patch
        from robot_workbench.control_owner import ControlOwnerService,_encode_event_response,LIMIT
        service=object.__new__(ControlOwnerService);service.device=SimpleNamespace(runtime=None)
        service.event_lock=threading.Lock();service.events=deque(maxlen=8192);service.event_sequence=0
        for i in range(35):
            service.event('owner_inference'if i%2 else 'control_study_feedback',sample=i)
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
    jaw.configure_mapping(None);device=Device('isolated-cpu','fake',None)
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
    def factory():
        gc.disable()  # Explicit CPU analogue of SDKProcess's connected GC policy.
        raw=WindowBackend(time.monotonic);raw.target_interpreter_profile='x5_shared_precision_v1'
        from collections import deque
        raw.packets=deque(maxlen=64)  # Test diagnostics must not retain every trajectory for a memory soak.
        if heartbeat is not None:
            read=raw.read_calibration_state
            def measured():
                heartbeat.value=time.monotonic()
                if tripped.value:raw.terminal_failure='SDK owner heartbeat lost';raw.window=None
                return read()
            raw.read_calibration_state=measured
        return Runtime(calibration,Safety,lambda:raw,device_id='isolated-cpu',environment='fake',
            lock_root=Path(root)/'locks',initialize_backend=lambda b:b.set_damping_hold(manual=True),event=service.event)
    def drive(runtime,closed):
        runtime.owner.backend._backend.control_thread=threading.get_ident()
        while not closed.is_set():
            started=time.monotonic()
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
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='wb-owner-',dir='/tmp');self.addCleanup(self.temp.cleanup)
        self.context=multiprocessing.get_context('spawn');self.ready=self.context.Queue();self.closing=self.context.Event()
        self.heartbeat=self.context.Value('d',0.);self.tripped=self.context.Value('b',False)
        fixture_mode=('large_events'if self._testMethodName.startswith('test_event_')else
                      'write_encoding_failure'if self._testMethodName=='test_successful_stop_encoding_failure_is_unknown'else
                      'slow_read_encoding'if self._testMethodName in ('test_read_encoder_delay_does_not_stall_tick_or_stop_and_preserves_sample_age','test_concurrent_state_readers_wait_outside_control_instead_of_failing')else None)
        self.process=self.context.Process(target=_owner_process,args=(self.temp.name,self.ready,self.closing,self.heartbeat,self.tripped,fixture_mode));self.process.start()
        self.addCleanup(self.cleanup)
        self.descriptor=self.ready.get(timeout=8);self.channel=ControlChannel(self.descriptor)

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

    def inference_client(self,*,finite):
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
        reference=None
        if finite:
            device=device_state(inputs['calibration_feedback']['qpos']);rows=np.tile(device,(61,1));rows[:,0]+=.02*np.linspace(0,1,61)
            reference=prepare_reference(dict(device_targets14=rows.tolist(),duration_s=2.),
                inputs['calibration_feedback']['qpos'],inputs['joint_limits'],source_sha256='a'*64)
            config.update(prepared_reference=reference.metadata,trajectory_kind='prepared_native_reference',execution_duration_s=reference.duration_s+.14)
        events=[]
        def created(owner,runner,configuration):events.append(dict(kind='created',pid=owner.cache['pid']))
        def predict(_):raise AssertionError('finite reference never invokes model; starvation uses no observation')
        client.configure_remote_inference(config,dict(profile=profile,reference=reference_wire(reference),
            preloaded_prediction=None,feedforward=None),created=created,metric=lambda *args:events.append(args),record=lambda *args:None,predict=predict)
        review=client.submit(inference,dict(action='preview_inference',duration_s=config['duration_s'],seed=1))
        client.submit(inference,dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
        return client,events

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
                except FileNotFoundError:continue
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
