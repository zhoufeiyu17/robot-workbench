"""Real CPU subprocess, immutable artifact and shared-camera lease regression."""
from copy import deepcopy
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import shutil
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'tests')]
import test_rgbd_service as fixtures
from test_recording_wall_clock import plan_fixture
from robot_workbench.recording_process import ProcessRecordingSession,_CaptureWriter as CaptureSession
from real_recording import verify_raw,iter_rgbd
from integration import _digest
from robot_workbench.sample_clock import clock_spec


def start_recording_fixture(extra_env=None):
    """Explicit resident CPU service; shared by actual Run integration tests."""
    from types import SimpleNamespace
    root=Path(tempfile.mkdtemp(prefix='wb-rec-'))
    folder=SimpleNamespace(name=str(root),cleanup=lambda:shutil.rmtree(root))
    with (root/'service.log').open('wb') as log:
        process=subprocess.Popen([sys.executable,'-B',str(ROOT/'scripts/serve_recording.py'),
            '--root',str(root/'service'),'--data-root',str(Path(tempfile.gettempdir()).resolve())],
            stdout=subprocess.DEVNULL,stderr=log,env=dict(os.environ,CUDA_VISIBLE_DEVICES='',**(extra_env or{})))
    descriptor=root/'service/service.json';deadline=time.monotonic()+10
    while not descriptor.exists():
        if process.poll() is not None or time.monotonic()>deadline:
            raise RuntimeError('CPU recorder startup failed: '+(root/'service.log').read_text())
        time.sleep(.02)
    previous=os.environ.get('WORKBENCH_RECORDING_SERVICE');os.environ['WORKBENCH_RECORDING_SERVICE']=str(descriptor)
    return process,folder,previous


def stop_recording_fixture(fixture):
    from robot_workbench.recording_process import recording_call
    process,folder,previous=fixture
    descriptor=Path(folder.name)/'service/service.json'
    try:
        recording_call(descriptor,'shutdown');process.wait(10)
        if process.returncode:raise AssertionError((Path(folder.name)/'service.log').read_text())
    finally:
        if previous is None:os.environ.pop('WORKBENCH_RECORDING_SERVICE',None)
        else:os.environ['WORKBENCH_RECORDING_SERVICE']=previous
        if process.poll() is not None:folder.cleanup()


def setUpModule():
    global recording_fixture
    recording_fixture=start_recording_fixture()


def tearDownModule():stop_recording_fixture(recording_fixture)


class ResidentRecordingTest(unittest.TestCase):
    def test_load_observer_timeout_preserves_writer_and_accepts_its_later_seal(self):
        self.test_closed_ingress_allows_another_run_but_service_stop_waits_for_old_writer(load_observer=True)

    def test_business_seal_wait_does_not_report_completion_while_writer_still_exports(self):
        self.test_closed_ingress_allows_another_run_but_service_stop_waits_for_old_writer(run_adapter=True)

    def test_closed_ingress_allows_another_run_but_service_stop_waits_for_old_writer(self,run_adapter=False,load_observer=False):
        import threading
        from robot_workbench.recording_process import CaptureSession,recording_call
        ready=self.root/'seal-entered';release=self.root/'seal-continue';wrapper=self.root/'slow-writer.py'
        wrapper.write_text(f'''#!{sys.executable}
import json,time
from pathlib import Path
from robot_workbench import recording_process as recorder
original=recorder._seal_capture
def delayed(root,*args):
    if json.loads((root/'capture.json').read_text())['request_id']=='old':
        Path({str(ready)!r}).touch()
        deadline=time.monotonic()+20
        while not Path({str(release)!r}).exists():
            if time.monotonic()>deadline:raise TimeoutError('test seal gate')
            time.sleep(.01)
    return original(root,*args)
recorder._seal_capture=delayed
recorder._capture_worker()
''');wrapper.chmod(0o700)
        fixture=start_recording_fixture({'WORKBENCH_CPU_PYTHON':str(wrapper)})
        sessions=[];capture=None;original_done=None
        try:
            for identity in ('old','new'):
                session=CaptureSession.create(self.root/identity,request_id=identity,
                    source_descriptors=[self.source],purpose_profile='data_required');sessions.append(session)
                session.prepare();session.bind(run_id=identity,business_kind='inference',policy_hash='a'*64);session.start()
                self.assertTrue(session.try_publish('feedback',1,1.,{'run':identity})['accepted'])
                if run_adapter and identity=='old':
                    from robot_workbench.inference_trace import CaptureRun
                    from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
                    observed=threading.Event();original_done=session.done
                    class ObservedDone:
                        # Expire a bounded observer wait without waiting90s;
                        # the real resident writer remains blocked at export.
                        def wait(self,timeout=None):observed.set();return original_done.wait(min(timeout,.01)if timeout is not None else None)
                        def is_set(self):return original_done.is_set()
                        def set(self):return original_done.set()
                    session.done=ObservedDone()
                    parameters=control_parameters(defaults('inference'),business='inference')
                    policy=dict(revision=0,parameters=parameters,policy_hash=policy_hash(parameters))
                    capture=CaptureRun('old',self.root/'old',{},policy,None,'fake',None,lambda:True,lambda record:None,
                        ros_sources=lambda:[],ros_watermarks=lambda:{'feedback':1})
                    capture.session=session;capture.root=session.directory;capture.prepared.set()
                    capture.finish('completed',value=dict(ros_final_seq={},source_time=1.,finished_wall_ns=time.time_ns()))
                    self.assertTrue(observed.wait(2))
                else:session.finish({'feedback':1},{'status':'completed'})
                if identity=='old':
                    deadline=time.monotonic()+5
                    while not ready.exists()or not session.snapshot().get('ingress_closed'):
                        self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                    self.assertFalse(session.done.is_set());self.assertFalse(session.snapshot()['writer_exited'])
                    if load_observer:
                        from test_ros_load import wait_for_seal
                        closed=[]
                        with self.assertRaisesRegex(TimeoutError,'pending'):
                            wait_for_seal(session,.03,close_sources=lambda:closed.append(identity))
                        self.assertEqual(closed,['old'])
                        self.assertFalse(session.snapshot()['writer_exited'])
                    if capture is not None:
                        self.assertFalse(capture.done.wait(.1),'an observation timeout must not end the live background capture')
                        self.assertIsNone(capture.error)
                    with self.assertRaisesRegex(ValueError,'writer|sealed|session'):
                        recording_call(Path(fixture[1].name)/'service/service.json','shutdown')
                else:
                    self.assertTrue(session.done.wait(5),session.snapshot())
                    manifest=json.loads((session.directory/'manifest.json').read_text())
                    self.assertEqual(manifest['binding']['run_id'],'new');self.assertEqual(manifest['recording_outcome'],'complete')
                    self.assertFalse(sessions[0].done.is_set(),'new Run must finish while old verification is blocked')
            release.touch();self.assertTrue((original_done or sessions[0].done).wait(5))
            manifest=json.loads((sessions[0].directory/'manifest.json').read_text())
            self.assertEqual(manifest['binding']['run_id'],'old');self.assertEqual(manifest['recording_outcome'],'complete')
            if load_observer:
                final=wait_for_seal(sessions[0],5)
                self.assertTrue(final['sealed']);self.assertTrue(final['writer_exited'])
            if capture is not None:
                self.assertTrue(capture.done.wait(5));self.assertIsNone(capture.error)
                self.assertEqual(capture.manifest['recording_outcome'],'complete')
        finally:
            release.touch()
            if original_done is not None:sessions[0].done=original_done
            for session in sessions:session.close()
            stop_recording_fixture(fixture)

    def test_finish_during_preparation_does_not_publish_a_late_binding_or_reopen_readiness(self):
        import threading
        from types import SimpleNamespace
        from robot_workbench.recording_process import CaptureControlSession
        from robot_workbench.inference_trace import CaptureRun
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        parameters=control_parameters(defaults('calibration'),business='calibration')
        policy=dict(revision=1,parameters=parameters,policy_hash=policy_hash(parameters))
        prepared=threading.Event();release=threading.Event();faults=[]
        original=CaptureRun.prepare
        def delayed(capture,value):
            original(capture,value)
            prepared.set();release.wait(5)
        with patch.object(CaptureRun,'prepare',delayed):
            session=CaptureControlSession(self.root.resolve()/'cancelled-prepare',execution_instance_id='cancelled-prepare',
                business_kind='calibration',control_policy=policy,
                cameras=SimpleNamespace(channels={},capture_sources=lambda:[]),
                execution_owner=dict(owner_pid=os.getpid(),runtime_nonce='cpu-owner',command_epoch=1),
                environment='fake',clock={'source_clock_id':'cpu-clock'},source_time=time.monotonic,
                on_fault=faults.append,preparation_deadline_monotonic=time.monotonic()+5,
                artifact_refs={'review':artifact(self.root,'review.json',{'cpu':'review'})})
            try:
                self.assertTrue(prepared.wait(5));self.assertIsNone(session.capture.error)
                session.submit('owner_event',data={'stage':'before-finish'})
                session.finish('stopped','cancel while preparation callback returns')
                with self.assertRaisesRegex(ValueError,'current prepared capture binding'):
                    session.bind_execution({'timeline_digest':'a'*64},execution_instance_id=session.execution_instance_id,
                        execution_owner=session.execution_owner)
                self.assertFalse(session.execution_bound.is_set())
                self.assertTrue(session.done.wait(10),session.snapshot())
                release.set();session._prepare_thread.join(3)
                self.assertFalse(session._prepare_thread.is_alive())
                self.assertIsNone(session.capture.error)
                self.assertEqual(faults,[])
                self.assertFalse(session.ready.is_set())
                self.assertEqual(session.manifest['recording_outcome'],'complete')
                events=[json.loads(line)['payload']['event']for line in (session.root/'control.jsonl').read_text().splitlines()]
                self.assertEqual(events,['owner_event'])
            finally:
                release.set();session._prepare_thread.join(3);session.close()

    def test_old_service_protocol_is_rejected_before_contact_or_writer_creation(self):
        from robot_workbench.recording_process import _recording_descriptor,RECORDING_SERVICE_SCHEMA
        with tempfile.TemporaryDirectory()as folder:
            path=Path(folder)/'old-recording.json'
            path.write_text(json.dumps(dict(schema='workbench.recording-service.v3')));path.chmod(0o600)
            with self.assertRaisesRegex(ValueError,'protocol version mismatch'):_recording_descriptor(path)
            path.write_text(json.dumps(dict(schema=RECORDING_SERVICE_SCHEMA)))
            self.assertEqual(_recording_descriptor(path)['schema'],RECORDING_SERVICE_SCHEMA)

    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.root=Path(self.folder.name)
        self.source=dict(stream_id='feedback',kind='feedback',source_instance_id='cpu',source_clock_id='clock',
            units='rad',coordinate_frame='joint',nominal_hz=100.,required=True)

    def create(self,request='resident',purpose='data_required'):
        from robot_workbench.recording_process import CaptureSession
        return CaptureSession.create(self.root,request_id=request,source_descriptors=[self.source],purpose_profile=purpose)

    def test_rejected_registration_has_no_phantom_writer_but_transport_loss_stays_unknown(self):
        import threading
        from unittest.mock import patch
        from robot_workbench.recording_process import RecordingService,CaptureSession
        allowed=self.root/'allowed';allowed.mkdir()
        service=RecordingService(self.root/'bounded-service',data_roots=[allowed])
        serving=threading.Thread(target=service.serve_forever,daemon=True);serving.start()
        try:
            with patch.dict(os.environ,WORKBENCH_RECORDING_SERVICE=str(service.descriptor_path)):
                session=CaptureSession.create(self.root/'outside',request_id='rejected-create',
                    source_descriptors=[self.source],purpose_profile='data_required')
                with self.assertRaisesRegex(ValueError,'outside registered'):session.prepare()
                self.assertTrue(session.done.wait(5));session.thread.join(2)
                self.assertIsNone(session.process,'same-service absence proof must remove the placeholder')
                self.assertFalse(service.snapshot()['active_sessions'])
                with patch('robot_workbench.recording_process._recording_request',side_effect=OSError('reply lost'))as call:
                    unknown=CaptureSession.create(allowed,request_id='unknown-create',
                        source_descriptors=[self.source],purpose_profile='data_required')
                    with self.assertRaisesRegex(ValueError,'reply lost'):unknown.prepare()
                    self.assertTrue(unknown.done.wait(5));unknown.thread.join(2)
                    self.assertIsNotNone(unknown.process,'a lost reply cannot prove no writer exists')
                    self.assertIsNone(unknown.process.poll())
                    self.assertEqual(call.call_count,1,'do not exonerate transport loss with a racing inspect')
        finally:
            service.dispatch(dict(action='shutdown'));serving.join(3);service.close()

    def test_business_crash_preserves_service_writer_and_reconnects_same_run(self):
        from robot_workbench.recording_process import recording_call
        code='''import json,os,time
from pathlib import Path
from robot_workbench.recording_process import CaptureSession
root=Path(os.environ['CAPTURE_TEST_ROOT']);source=json.loads(os.environ['CAPTURE_TEST_SOURCE'])
s=CaptureSession.create(root,request_id='resident',source_descriptors=[source],purpose_profile='infer_debug')
s.prepare();s.bind(run_id='resident',business_kind='calibration',policy_hash='a'*64);s.start()
s.try_publish('feedback',1,1.,{'q':1.})
end=time.monotonic()+5
while s.snapshot().get('streams',{}).get('feedback',{}).get('committed',0)<1:
    if time.monotonic()>end:raise TimeoutError('commit not confirmed')
    time.sleep(.02)
os._exit(0)
'''
        result=subprocess.run([sys.executable,'-B','-c',code],env=dict(os.environ,
            CAPTURE_TEST_ROOT=str(self.root),CAPTURE_TEST_SOURCE=json.dumps(self.source)),capture_output=True,text=True,timeout=15)
        self.assertEqual(result.returncode,0,result.stderr)
        descriptor=os.environ['WORKBENCH_RECORDING_SERVICE'];before=recording_call(descriptor,'inspect')
        self.assertTrue(before['active_sessions']);pid=before['active_sessions'][0]['worker_pid']
        if sys.platform.startswith('linux'):
            from robot_workbench.managed_services import process_identity
            self.assertEqual(process_identity(pid)['ppid'],before['pid'])
        with self.assertRaisesRegex(ValueError,'must seal'):recording_call(descriptor,'shutdown')
        session=self.create(purpose='infer_debug');self.addCleanup(session.close)
        self.assertEqual(session.prepare()['state'],'recording');self.assertEqual(session.process.pid,pid)
        self.assertEqual(session.last_sequence,{'feedback':1})
        self.assertTrue(session.try_publish('feedback',2,1.01,{'q':2.})['accepted'])
        session.finish({'feedback':2},{'status':'stopped'})
        self.assertTrue(session.done.wait(10),session.snapshot())
        manifest=json.loads((session.directory/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'incomplete');self.assertEqual(manifest['streams']['feedback']['committed'],2)
        self.assertTrue(any('producer exited' in issue for issue in manifest['issues']))
        self.assertEqual(recording_call(descriptor,'inspect')['pid'],before['pid'])

    def test_missing_registration_and_outside_root_fail_without_writer(self):
        from robot_workbench.recording_process import CaptureSession,recording_call
        with patch.dict(os.environ,{},clear=True):
            with self.assertRaisesRegex(ValueError,'resident recording service'):self.create()
        with self.assertRaisesRegex(ValueError,'outside registered'):
            recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'status',directory='/unregistered/capture')

    def test_run_context_cannot_write_a_receipt_outside_its_bound_run_directory(self):
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        session=self.create(purpose='infer_debug');self.addCleanup(session.close)
        session.prepare();parameters=control_parameters(defaults('inference'),business='inference')
        policy=dict(revision=1,parameters=parameters,policy_hash=policy_hash(parameters))
        with self.assertRaisesRegex(ValueError,'Run artifact directory'):
            session.bind(run_id='resident',business_kind='inference',policy_hash=policy['policy_hash'],control_policy=policy,
                execution_context=dict(device_id='cpu',environment='fake'))
        self.assertEqual(session.snapshot()['state'],'prepared')

    def test_service_seals_run_receipt_after_business_exit_and_rebuilds_both_indexes(self):
        from robot_workbench.collection_history import CaptureResults
        from robot_workbench.inference_history import InferenceHistory
        for business in ('calibration','inference'):
            run='receipt-'+business;root=self.root/business;root.mkdir()
            code='''import json,os,time
from pathlib import Path
from robot_workbench.recording_process import CaptureSession,recording_call
from robot_workbench.execution_settings import defaults,control_parameters,policy_hash
b=os.environ['CAPTURE_BUSINESS'];run='receipt-'+b;root=Path(os.environ['CAPTURE_TEST_ROOT'])
p=control_parameters(defaults(b),business=b);policy=dict(revision=1,parameters=p,policy_hash=policy_hash(p))
directory=root/(('inference-'if b=='inference'else'')+run)
s=CaptureSession.create(directory/'capture',request_id=run,source_descriptors=[json.loads(os.environ['CAPTURE_TEST_SOURCE'])],purpose_profile=p['recording']['purpose_profile'])
s.prepare();s.bind(run_id=run,business_kind=b,policy_hash=policy['policy_hash'],control_policy=policy,execution_context=dict(device_id='cpu',environment='fake'));s.start()
s.try_publish('feedback',1,1.,{'q':1.});end=time.monotonic()+10
while s.snapshot().get('streams',{}).get('feedback',{}).get('committed',0)<1:
    if time.monotonic()>end:raise TimeoutError('commit missing')
    time.sleep(.02)
recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'finish',directory=str(s.directory),final_seq_by_stream={'feedback':1},control_terminal={'status':'failed','reason':'CPU test'})
os._exit(0)
'''
            result=subprocess.run([sys.executable,'-B','-c',code],env=dict(os.environ,CAPTURE_BUSINESS=business,
                CAPTURE_TEST_ROOT=str(root),CAPTURE_TEST_SOURCE=json.dumps(self.source)),capture_output=True,text=True,timeout=20)
            self.assertEqual(result.returncode,0,result.stderr)
            run_root=root/(('inference-'if business=='inference'else'')+run);path=run_root/'receipt-r000001.json'
            deadline=time.monotonic()+15
            while not path.exists():self.assertLess(time.monotonic(),deadline);time.sleep(.02)
            original=path.read_bytes();receipt=json.loads(original)
            self.assertEqual(receipt['schema'],'workbench.capture-run.v1');self.assertEqual(receipt['controller']['status'],'failed')
            self.assertEqual(receipt['recording']['recording_outcome'],'complete')
            history=InferenceHistory(root)if business=='inference'else None
            if history:
                debug=run_root/'debug-receipt.json';debug.write_text('{}')
                history.append([(dict(id='debug-'+run,run_id=run,kind='inference_debug'),debug)])
            for restart in range(2):
                index=CaptureResults(root,environment='fake',device_id='cpu',inference_history=history)
                try:
                    deadline=time.monotonic()+10
                    while index.histories[business].result('capture-'+run+'-receipt')is None:
                        self.assertLess(time.monotonic(),deadline,index.error);time.sleep(.02)
                    if history:self.assertIsNotNone(history.result('debug-'+run))
                finally:index.close()
            self.assertEqual(path.read_bytes(),original)

    def test_stalled_service_io_does_not_serialize_or_wait_on_the_producer(self):
        import threading
        session=self.create('stalled');self.addCleanup(session.close)
        session.prepare();session.bind(run_id='stalled',business_kind='calibration',policy_hash='a'*64);session.start()
        entered=threading.Event();resume=threading.Event();original=session._write
        def stalled(message):entered.set();resume.wait(5);original(message)
        try:
            with patch.object(session,'_write',side_effect=stalled):
                session.try_publish('feedback',1,1.,{'q':1.});self.assertTrue(entered.wait(2))
                encode=json.dumps
                def background_only(*args,**kwargs):
                    self.assertNotEqual(threading.current_thread(),threading.main_thread(),'producer serialized')
                    return encode(*args,**kwargs)
                with patch('robot_workbench.recording_process.json.dumps',side_effect=background_only):
                    before=time.monotonic()
                    for seq in range(2,102):self.assertTrue(session.try_publish('feedback',seq,seq/100+1.,{'q':seq})['accepted'])
                    self.assertLess(time.monotonic()-before,.1)
        finally:resume.set()
        session.finish({'feedback':101},{'status':'completed'});self.assertTrue(session.done.wait(15),session.snapshot())
        self.assertLessEqual(session.snapshot()['parent_queue']['peak_bytes'],session.spec['resource_budget']['event_queue_bytes'])

    def test_dead_writer_requires_prefix_seal_before_normal_service_stop(self):
        # Separate service so the test never disrupts the shared fixture.
        from robot_workbench.recording_process import RecordingService,_capture_spec
        import threading
        root=self.root/'private';root.mkdir(mode=0o700)
        service=RecordingService(root,data_roots=[self.root]);spec=_capture_spec('restart',[self.source],'data_required',None)
        directory=self.root/('capture-'+hashlib.sha256(b'restart').hexdigest())
        state=service.dispatch(dict(action='create',directory=str(directory),spec=spec))
        writer=service.sessions[str(directory.resolve())];writer.prepare()
        writer.bind(run_id='restart',business_kind='calibration',policy_hash='a'*64);writer.start()
        writer.try_publish('feedback',1,1.,{'q':1.})
        deadline=time.monotonic()+5
        while writer.snapshot().get('streams',{}).get('feedback',{}).get('committed',0)<1 and time.monotonic()<deadline:time.sleep(.02)
        # A second supervisor cannot acquire the resident lock.
        with self.assertRaises(BlockingIOError):RecordingService(root,data_roots=[self.root])
        os.killpg(writer.process.pid,__import__('signal').SIGKILL)
        self.assertTrue(writer.done.wait(10));writer.thread.join(2)
        with self.assertRaisesRegex(ValueError,'cannot close'):service.close()
        service.dispatch(dict(action='recover',directory=str(directory)))
        self.assertTrue(writer.done.wait(10),writer.snapshot());writer.thread.join(2)
        service.close()
        restarted=RecordingService(root,data_roots=[self.root])
        self.assertTrue(restarted.dispatch(dict(action='status',directory=str(directory)))['writer_exited'])
        manifest=json.loads((directory/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'incomplete');self.assertIsNone(manifest['control_terminal'])
        self.assertEqual(manifest['streams']['feedback']['committed'],1)
        restarted.close()

    @unittest.skipUnless(sys.platform.startswith('linux'),'requires Linux verified process exit')
    def test_real_service_crash_keeps_verified_prefix_and_restart_never_invents_a_terminal(self):
        from robot_workbench.recording_process import CaptureSession,recording_call
        from robot_workbench.managed_services import process_identity,_live
        fixture=start_recording_fixture();process,folder,previous=fixture
        descriptor=Path(folder.name)/'service/service.json'
        session=CaptureSession.create(self.root,request_id='service-crash',source_descriptors=[self.source],purpose_profile='infer_debug')
        try:
            session.prepare();session.bind(run_id='service-crash',business_kind='inference',policy_hash='a'*64);session.start()
            session.try_publish('feedback',1,1.,{'q':1.});deadline=time.monotonic()+10
            while session.snapshot().get('streams',{}).get('feedback',{}).get('committed',0)<1:
                self.assertLess(time.monotonic(),deadline);time.sleep(.02)
            writer=session.process.pid
            os.kill(process.pid,__import__('signal').SIGTERM)
            self.assertEqual(recording_call(descriptor,'inspect')['pid'],process.pid,'active service must refuse normal shutdown')
            process.kill();process.wait(10)
            self.assertTrue(session.done.wait(15));session.thread.join(2)
            deadline=time.monotonic()+15
            while _live(process_identity(writer)):
                self.assertLess(time.monotonic(),deadline);time.sleep(.02)
            prefix=(session.directory/'feedback.jsonl').read_bytes()
            with (Path(folder.name)/'service.log').open('ab') as log:
                process=subprocess.Popen(process.args,stdout=subprocess.DEVNULL,stderr=log,env=dict(os.environ,CUDA_VISIBLE_DEVICES=''))
            fixture=(process,folder,previous);deadline=time.monotonic()+10
            while True:
                try:
                    state=recording_call(descriptor,'inspect')
                    if state['pid']==process.pid:break
                except (OSError,ValueError):pass
                self.assertLess(time.monotonic(),deadline);time.sleep(.02)
            state=recording_call(descriptor,'status',directory=str(session.directory))
            self.assertTrue(state['writer_exited'])
            manifest=json.loads((session.directory/'manifest.json').read_text())
            self.assertEqual(manifest['recording_outcome'],'incomplete');self.assertIsNone(manifest['control_terminal'])
            self.assertEqual((session.directory/'feedback.jsonl').read_bytes(),prefix)
        finally:stop_recording_fixture(fixture)


def artifact(root,name,value):
    path=root/name;raw=json.dumps(value,allow_nan=False).encode();path.write_bytes(raw);path.chmod(0o400)
    return dict(path=str(path),size_bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())


class CaptureSessionTest(unittest.TestCase):
    def test_capture_done_receipt_does_not_hide_a_live_writer_or_bridge(self):
        from types import SimpleNamespace
        from robot_workbench.inference_trace import CaptureRun
        capture=CaptureRun('cpu',self.root,{},None,None,'fake',None,lambda:True,lambda c:None)
        live=[True,False]
        capture.session=SimpleNamespace(process=SimpleNamespace(poll=lambda:None if live[0]else 0),
            thread=SimpleNamespace(is_alive=lambda:live[1]))
        capture.done.set();self.assertTrue(capture.thread.is_alive())
        live[:]=[False,True];self.assertTrue(capture.thread.is_alive())
        live[:]=[False,False];self.assertFalse(capture.thread.is_alive())

    def test_run_seal_recovers_a_writer_that_exits_after_finish_was_submitted(self):
        from types import SimpleNamespace
        from robot_workbench.inference_trace import CaptureRun
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        steps=[];parameters=control_parameters(defaults('calibration'),business='calibration')
        policy=dict(revision=1,parameters=parameters,policy_hash=policy_hash(parameters))
        class Done:
            def is_set(self):return False  # The bridge has not yet delivered child exit.
            def wait(self,timeout):steps.append('wait');return True
        def recover():
            steps.append('recover')
            (self.root/'manifest.json').write_text(json.dumps(dict(seal_revision=1,issues=['writer exit'],
                streams={'control':{'missing':0,'final_sequence':1}},control_terminal={'status':'failed'})))
        session=SimpleNamespace(last_sequence={'control':1},camera_sources={},directory=self.root,done=Done(),
            process=SimpleNamespace(poll=lambda:-9),thread=SimpleNamespace(join=lambda timeout:steps.append('join')),
            finish=lambda *a,**k:steps.append('finish'),recover=recover,snapshot=lambda:{})
        capture=CaptureRun('cpu',self.root,{},policy,None,'fake',None,lambda:True,lambda c:None,business_kind='calibration')
        capture.session=session;capture.prepared.set();capture.controller_result={'status':'failed'}
        capture._seal(dict(final_capture_sequence=1,source_time=1.))
        self.assertEqual(steps,['finish','wait','join','recover','wait']);self.assertIsNotNone(capture.manifest)

    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.sessions=[]
        self.addCleanup(lambda:[session.close(timeout=3) for session in self.sessions])

    def create(self,*,purpose='infer_debug',missing=False,budget=None,request_id='capture-test'):
        source=dict(stream_id='feedback',kind='feedback',source_instance_id=None if missing else 'cpu-source',
            source_clock_id='cpu-clock',units='rad',coordinate_frame='joint',nominal_hz=100.,required=True)
        session=CaptureSession.create(self.root/'captures',request_id=request_id,source_descriptors=[source],
            purpose_profile=purpose,resource_budget=budget)
        self.sessions.append(session)
        return session

    def start(self,session):
        ready=session.prepare();self.assertTrue(ready['ready'],ready)
        session.bind(run_id='run:cpu',business_kind='inference',policy_hash='a'*64)
        session.start()

    def finish(self,session,sequence,**terminal):
        receipt=session.finish({'feedback':sequence},terminal or dict(status='completed'))
        self.assertEqual(receipt['state'],'sealing')
        self.assertTrue(session.done.wait(10),session.snapshot())
        self.assertIsNotNone(session.process.poll())
        self.assertFalse(session.snapshot().get('error'),session.snapshot())
        return json.loads((session.directory/'manifest.json').read_text())

    def test_binding_idempotency_snapshot_and_failed_run_complete_data(self):
        session=self.create(purpose='data_required');self.start(session)
        self.assertNotEqual(session.process.pid,os.getpid())
        self.assertIs(self.create(purpose='data_required'),session)
        session.bind(run_id='run:cpu',business_kind='inference',policy_hash='a'*64)
        with self.assertRaisesRegex(ValueError,'binding changed'):
            session.bind(run_id='different-run',business_kind='inference',policy_hash='a'*64)
        payload={'q':[.1]*14};original=json.dumps
        def dumps(*args,**kwargs):
            self.assertNotEqual(__import__('threading').current_thread(),__import__('threading').main_thread(),
                                'try_publish must not serialize on the producer')
            return original(*args,**kwargs)
        with patch('robot_workbench.recording_process.json.dumps',side_effect=dumps):
            self.assertTrue(session.try_publish('feedback',1,10.,payload)['accepted'])
        payload['q'][0]=99.
        self.assertTrue(session.try_publish('feedback',2,10.01,{'q':(.2,)*14})['accepted'])
        manifest=self.finish(session,2,status='failed',reason='model unavailable')
        self.assertEqual(manifest['recording_outcome'],'complete')
        self.assertEqual(manifest['control_terminal']['status'],'failed')
        self.assertEqual(manifest['binding']['policy_hash'],'a'*64)
        rows=[json.loads(line) for line in (session.directory/'feedback.jsonl').read_text().splitlines()]
        self.assertEqual(rows[0]['payload']['q'][0],.1)
        self.assertEqual(manifest['streams']['feedback']['committed'],2)
        with self.assertRaisesRegex(ValueError,'different request'):self.create(purpose='infer_debug')

    def test_nonblocking_queue_gap_and_final_sequence_barrier(self):
        session=self.create(budget={'event_queue_bytes':1024});self.start(session)
        self.assertFalse(session.try_publish('feedback',1,1.,'x'*2000)['accepted'])
        self.assertTrue(session.try_publish('feedback',2,1.01,{'q':1.})['accepted'])
        with self.assertRaises(ValueError):session.finish({'feedback':1},{'status':'completed'})
        manifest=self.finish(session,3)
        self.assertEqual(manifest['recording_outcome'],'incomplete')
        self.assertEqual(manifest['streams']['feedback']['missing'],2)
        gaps=[json.loads(line) for line in (session.directory/manifest['gaps_ref']).read_text().splitlines()]
        self.assertEqual([(row['first_sequence'],row['last_sequence']) for row in gaps],[(1,1),(3,3)])
        self.assertFalse(session.try_publish('feedback',4,1.04,{})['accepted'])
        state=session.snapshot();cost=state['producer_publish_cost']
        self.assertEqual(cost['calls'],3);self.assertGreater(cost['maximum_s'],0.)
        self.assertGreaterEqual(cost['maximum_s'],cost['mean_s'])
        self.assertGreaterEqual(state['parent_queue']['oldest_age_s'],0.)

    def test_missing_source_is_rejected_or_degraded_by_purpose(self):
        required=self.create(purpose='data_required',missing=True,request_id='required')
        self.assertFalse(required.prepare()['ready'])
        with self.assertRaisesRegex(ValueError,'not ready'):
            required.bind(run_id='run',business_kind='calibration',policy_hash='b'*64)
        debug=self.create(missing=True,request_id='debug');self.start(debug)
        self.assertTrue(debug.snapshot()['degraded'])
        result=debug.try_publish('feedback',1,1.,{})
        self.assertEqual(result['reason'],'source_unavailable')
        self.assertEqual(self.finish(debug,1)['recording_outcome'],'incomplete')

    def test_payload_reference_is_archived_in_child(self):
        session=self.create();self.start(session)
        source=self.root/'model-output.json';blob=b'{"action":[1,2,3]}'
        source.write_bytes(blob)
        ref=dict(path=str(source),size_bytes=len(blob),sha256=hashlib.sha256(blob).hexdigest())
        self.assertTrue(session.try_publish('feedback',1,1.,{'payload_ref':ref})['accepted'])
        manifest=self.finish(session,1);self.assertEqual(manifest['recording_outcome'],'complete')
        row=json.loads((session.directory/'feedback.jsonl').read_text())
        archived=session.directory/row['payload']['payload_ref']['path']
        source.write_bytes(b'replaced')
        self.assertEqual(archived.read_bytes(),blob)

    def test_dead_writer_recovers_committed_prefix_without_inventing_finish(self):
        import signal
        session=self.create();self.start(session)
        self.assertTrue(session.try_publish('feedback',1,1.,{'q':1.})['accepted'])
        deadline=time.monotonic()+5
        while session.snapshot()['streams']['feedback'].get('committed',0)<1 and time.monotonic()<deadline:time.sleep(.02)
        self.assertEqual(session.snapshot()['streams']['feedback']['committed'],1)
        os.killpg(session.process.pid,signal.SIGKILL)
        self.assertTrue(session.done.wait(10));session.thread.join(1)
        session.recover();self.assertTrue(session.done.wait(10),session.snapshot())
        manifest=json.loads((session.directory/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'incomplete')
        self.assertIsNone(manifest['control_terminal'])
        self.assertEqual(manifest['streams']['feedback']['committed'],1)
        self.assertIn('no final sequence barrier received',manifest['issues'])

    def test_bad_reference_keeps_an_incomplete_prefix_receipt(self):
        session=self.create();self.start(session)
        source=self.root/'changed-output';source.write_bytes(b'current')
        bad=dict(path=str(source),size_bytes=7,sha256='0'*64)
        session.try_publish('feedback',1,1.,{'payload_ref':bad})
        self.assertTrue(session.done.wait(10),session.snapshot())
        manifest=json.loads((session.directory/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'incomplete')
        self.assertTrue(any('checksum changed' in issue for issue in manifest['issues']))

    def test_recovery_preserves_prior_manifest_and_reports_invalid_tail(self):
        session=self.create();self.start(session)
        session.try_publish('feedback',1,1.,{'q':1.})
        first=self.finish(session,1)
        old=session.directory/f"manifest-r{first['seal_revision']:06d}.json"
        original=old.read_bytes();events=session.directory/'feedback.jsonl';prefix=events.read_bytes()
        with events.open('ab') as stream:stream.write(b'{incomplete tail')
        changed=events.read_bytes()
        session.recover();self.assertTrue(session.done.wait(10),session.snapshot())
        recovered=json.loads((session.directory/'manifest.json').read_text())
        self.assertEqual(recovered['recording_outcome'],'incomplete')
        self.assertGreater(recovered['seal_revision'],first['seal_revision'])
        self.assertEqual(recovered['streams']['feedback']['verified_prefix_bytes'],len(prefix))
        self.assertEqual(events.read_bytes(),changed)
        self.assertEqual(old.read_bytes(),original)

    def test_lost_client_reopens_same_session_without_another_writer(self):
        session=self.create();self.start(session)
        CaptureSession._sessions.pop(str(session.directory))
        reopened=self.create()
        self.assertTrue(reopened.prepare()['read_only'])
        self.assertIsNone(reopened.process)
        self.assertEqual(reopened.capture_session_id,session.capture_session_id)
        with self.assertRaisesRegex(ValueError,'not writable'):
            reopened.bind(run_id='run:cpu',business_kind='inference',policy_hash='a'*64)
        session.try_publish('feedback',1,1.,{})
        self.finish(session,1)
        self.assertEqual(reopened.snapshot()['state'],'complete')

    def test_writer_enospc_reports_failure_and_preserves_incomplete_receipt(self):
        from robot_workbench.recording_process import _capture_spec,_environment
        from robot_workbench.inference_trace import write_json
        root=self.root/'faulted';root.mkdir()
        source=dict(stream_id='feedback',kind='feedback',source_instance_id='cpu',source_clock_id='cpu',
            units='rad',coordinate_frame='joint',nominal_hz=100.,required=True)
        write_json(root/'capture.json',_capture_spec('enospc',[source],'data_required',None))
        code="""import errno
import robot_workbench.recording_process as r
def fail(*args):raise OSError(errno.ENOSPC,'injected full storage')
r._archive_capture_payload=fail
r._capture_worker()
"""
        commands=[dict(directory=str(root)),dict(action='bind',binding=dict(run_id='run',business_kind='calibration',policy_hash='a'*64)),
            dict(action='start'),dict(action='event',stream_id='feedback',sequence=1,source_time=1.,payload={})]
        child=subprocess.run([sys.executable,'-B','-c',code],input=''.join(json.dumps(row)+'\n' for row in commands),
            capture_output=True,text=True,env=_environment(),timeout=10)
        state=json.loads(child.stdout.splitlines()[-1])
        self.assertTrue(state['done']);self.assertEqual(state['state'],'failed')
        self.assertIn('full storage',state['error'])
        manifest=json.loads((root/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'incomplete')

    def test_initial_storage_failure_does_not_leave_a_writer_lock(self):
        with patch('robot_workbench.inference_trace.write_json',side_effect=OSError('full disk')):
            with self.assertRaises(OSError):self.create()
        retry=self.create()
        self.assertTrue(retry.prepare()['ready'])
        self.assertIsNotNone(retry.process)

    def test_repeated_bind_and_start_frames_do_not_reset_the_recording(self):
        session=self.create();self.start(session)
        session.commands.put(dict(action='bind',binding=dict(session.binding)))
        session.commands.put(dict(action='start'))
        session.try_publish('feedback',1,1.,{'q':1.})
        self.assertEqual(self.finish(session,1)['recording_outcome'],'complete')

    def test_another_worker_cannot_bypass_the_recording_lease(self):
        from robot_workbench.recording_process import _environment
        session=self.create();self.start(session)
        original=(session.directory/'binding.json').read_bytes()
        second=subprocess.run([sys.executable,'-B','-m','robot_workbench.recording_process','capture'],
            input=json.dumps(dict(directory=str(session.directory)))+'\n',text=True,
            capture_output=True,timeout=5,env=_environment())
        self.assertNotEqual(second.returncode,0)
        self.assertEqual((session.directory/'binding.json').read_bytes(),original)
        session.try_publish('feedback',1,1.,{})
        self.assertEqual(self.finish(session,1)['recording_outcome'],'complete')


class CaptureCameraTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.fixture=fixtures.SharedCameraTest()
        self.service=self.fixture.service();self.addCleanup(self.fixture.doCleanups)
        for camera in self.service.channels:self.fixture.publish(self.service,camera,0,None)
        self.sources=self.service.capture_sources();self.sessions=[]
        self.addCleanup(lambda:[session.close(timeout=10) for session in self.sessions])

    def create(self,*,purpose='data_required',sources=None):
        session=CaptureSession.create(self.root/'captures',request_id='camera-run',
            source_descriptors=sources or self.sources,purpose_profile=purpose)
        self.sessions.append(session);return session

    def bind_start(self,session):
        self.assertTrue(session.prepare()['ready'],session.snapshot())
        session.bind(run_id='camera-run',business_kind='inference',policy_hash='d'*64,execution_owner=self.fixture.owner)
        session.start()

    def endpoints(self,sources=None,time_s=100.12):
        return {row['stream_id']:dict(source_clock_id=row['source_clock_id'],source_time=time_s) for row in sources or self.sources}

    def publish(self,index):
        for camera,channel in self.service.channels.items():
            lease=channel.current_lease()
            self.fixture.publish(self.service,camera,index,lease['run_id'] if lease else None)

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_resident_three_camera_streams_continue_after_business_client_disappears(self):
        from robot_workbench.recording_process import CaptureSession as ResidentSession
        session=ResidentSession.create(self.root/'captures',request_id='resident-camera',
            source_descriptors=self.sources,purpose_profile='data_required')
        self.sessions.append(session);self.bind_start(session)
        self.publish(1)
        session.disconnected.set();session.thread.join(8)
        ResidentSession._sessions.pop(str(session.directory))
        self.publish(2);self.publish(3)
        reopened=ResidentSession.create(self.root/'captures',request_id='resident-camera',
            source_descriptors=self.sources,purpose_profile='data_required')
        self.sessions.append(reopened);self.assertEqual(reopened.prepare()['state'],'recording')
        reopened.finish({},dict(status='stopped'),source_endpoints=self.endpoints());self.publish(4)
        self.assertTrue(reopened.done.wait(30),reopened.snapshot())
        manifest=json.loads((reopened.directory/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'complete',manifest['issues'])
        for source in self.sources:
            rows=[json.loads(line) for line in (reopened.directory/(source['stream_id']+'.jsonl')).read_text().splitlines()]
            self.assertEqual([r['payload']['metadata']['frame_number'] for r in rows],[1,2,3,4])

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_three_cameras_preserve_pixels_clock_and_bracketing_tail(self):
        from rgbd_segments import iter_segment
        session=self.create();self.bind_start(session)
        for index in (1,2,3):self.publish(index)
        started=time.monotonic();ack=session.finish({},dict(status='completed'),source_endpoints=self.endpoints())
        self.assertEqual(ack['state'],'sealing');self.assertLess(time.monotonic()-started,1.)
        self.publish(4)
        self.assertTrue(session.done.wait(30),session.snapshot())
        manifest=json.loads((session.directory/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'complete',manifest['issues'])
        self.assertNotIn('playback_error',manifest)
        for source in self.sources:
            name=source['stream_id'];rows=[json.loads(line) for line in (session.directory/(name+'.jsonl')).read_text().splitlines()]
            self.assertEqual([r['payload']['metadata']['frame_number'] for r in rows],[1,2,3,4])
            self.assertEqual(manifest['streams'][name]['final_sequence'],4)
            self.assertEqual(session.snapshot()['streams'][name]['final_sequence'],4)
            self.assertGreaterEqual(rows[-1]['source_time'],100.12)
            segment=rows[0]['payload']['frame_ref']['segment'].removeprefix('camera/')
            decoded=list(iter_segment(session.directory/'camera',segment,png=False,ffmpeg=shutil.which('ffmpeg'),ffprobe=shutil.which('ffprobe')))
            self.assertEqual(len(decoded),4)
            for index,(meta,rgb,depth) in enumerate(decoded,1):self.assertEqual((rgb,depth),self.fixture.pixels(index))
            replay=manifest['playback']['streams'][name]
            self.assertEqual(replay['frame_count'],4)
            for actual,expected in zip(replay['frame_offsets_s'],[0,1/30,2/30,3/30]):
                self.assertAlmostEqual(actual,expected,delta=1e-6)
            self.assertIsNone(self.service.channels[source['source_ref']['camera']].current_lease())
        # Replay may export a verified prefix after an interrupted recording,
        # but it must not relabel pixels with altered frame metadata or times.
        from segmented_playback import export_capture
        from playback import sha256
        import segmented_playback
        import threading
        original_run=segmented_playback.subprocess.run
        barrier=threading.Barrier(2,timeout=5)
        first_pair={source['stream_id']for source in self.sources[:2]}
        active=peak=0;export_lock=threading.Lock()
        def run_codec(argv,**kwargs):
            nonlocal active,peak
            if Path(argv[0]).name!='ffmpeg':return original_run(argv,**kwargs)
            with export_lock:active+=1;peak=max(peak,active)
            try:
                if Path(argv[-1]).stem in first_pair:barrier.wait()
                return original_run(argv,**kwargs)
            finally:
                with export_lock:active-=1
        with patch.object(segmented_playback.subprocess,'run',side_effect=run_codec):
            parallel=export_capture(session.directory,manifest,session.directory/'parallel-replay')
        self.assertEqual((peak,active),(2,0))
        self.assertEqual(set(parallel['streams']),set(manifest['playback']['streams']))
        for name,value in parallel['streams'].items():
            self.assertEqual(value['frame_offsets_s'],manifest['playback']['streams'][name]['frame_offsets_s'])
            self.assertEqual(value['duration_s'],manifest['playback']['streams'][name]['duration_s'])
        prefix=deepcopy(manifest)
        name=self.sources[0]['stream_id'];prefix['streams']={name:dict(manifest['streams'][name],committed=3)}
        replay=export_capture(session.directory,prefix,session.directory/'prefix-replay')
        self.assertEqual(replay['streams'][name]['frame_count'],3)
        index=session.directory/(name+'.jsonl');original=index.read_bytes()
        for field in ('source_time','frame'):
            changed=[json.loads(line) for line in original.splitlines()]
            if field=='source_time':changed[0][field]+=.001
            else:changed[0]['payload']['frame_ref'][field]=1
            index.write_text(''.join(json.dumps(row)+'\n' for row in changed))
            altered=deepcopy(prefix);altered['files'][index.name]=dict(size_bytes=index.stat().st_size,sha256=sha256(index))
            with self.assertRaisesRegex(ValueError,'timestamp|frame order'):
                export_capture(session.directory,altered,session.directory/('invalid-'+field))
        index.write_bytes(original)
        self.assertIsNone(self.service.process.poll(),'recorder must preserve the shared camera service')

    def test_foreign_recording_lease_is_preserved(self):
        token=self.fixture.lease(self.service)
        session=self.create()
        self.assertFalse(session.prepare()['ready'])
        self.assertEqual(set(session.snapshot()['missing_streams']),{row['stream_id'] for row in self.sources})
        session.close()
        for channel in self.service.channels.values():self.assertEqual(channel.current_lease()['run_id'],token)

    def test_source_without_recording_run_capability_is_rejected_before_lease(self):
        from robot_workbench.recording_process import _CaptureCamera
        original=self.service.call
        def old_source(action,**fields):
            result=original(action,**fields)
            if action=='identity':result.pop('capabilities',None)
            return result
        with patch.object(fixtures.CommonCameraService,'connect',return_value=self.service),\
                patch.object(self.service,'call',side_effect=old_source):
            with self.assertRaisesRegex(ValueError,'recording_run_lease_v1'):
                _CaptureCamera(self.root,self.sources[0])
        self.assertTrue(all(channel.current_lease() is None for channel in self.service.channels.values()))

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_infer_run_publishes_three_replays_and_model_artifacts(self):
        import threading
        import numpy as np
        from types import SimpleNamespace
        from robot_workbench.inference_evidence import InferenceEvidence
        from robot_workbench.inference_trace import CaptureRun
        from robot_workbench.inference_workflow import InferenceWorkflow
        from robot_workbench.inference_history import InferenceHistory
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        inputs=InferenceEvidence(self.root/'inputs');self.addCleanup(inputs.close)
        parameters=control_parameters(defaults('inference'),business='inference')
        policy=dict(revision=4,parameters=parameters,policy_hash=policy_hash(parameters))
        flow=object.__new__(InferenceWorkflow);flow.device=SimpleNamespace(environment='fake')
        flow.lock=threading.RLock();flow.history=InferenceHistory(self.root)
        config=dict(model_id='cpu-model',clock=dict(source_clock_id=self.sources[0]['source_clock_id']))
        run=dict(id='cpu-infer',directory=self.root/'inference-cpu',config=config,control_policy=policy)
        capture=CaptureRun(run['id'],run['directory'],config,policy,self.service,'fake',inputs,lambda:True,
                           lambda value:flow._finish_capture(run,value))
        self.addCleanup(capture.close)
        capture.record('prepare',dict(capture_sequence=1,capture_source_time=100.,execution_owner=self.fixture.owner))
        self.assertIsNone(capture.error)
        for index in (1,2,3):self.publish(index)
        capture.model_begin('prediction',100.05)
        inputs.input('prediction',np.zeros(14),{'base_0_rgb':np.zeros((2,2,3),dtype=np.uint8)},{'request_id':'prediction'})
        capture.model_end('prediction',inputs.output('prediction',{'action14':np.zeros((60,14)),'audit':{}},chunk=False))
        capture.record('command',dict(capture_sequence=2,capture_source_time=100.06,q_sdk14=[0.]*14))
        capture.record('feedback',dict(capture_sequence=3,capture_source_time=100.1,raw={'qpos':[0.]*14}))
        capture.record('finish',dict(status='stopped',reason='CPU stop',final_capture_sequence=3,source_time=100.12))
        self.publish(4)
        self.assertTrue(capture.done.wait(30),capture.snapshot())
        self.assertIsNone(capture.error)
        self.assertEqual(capture.manifest['recording_outcome'],'complete')
        self.assertEqual(capture.manifest['control_terminal']['status'],'stopped')
        rows=flow.history.page()['results']
        self.assertEqual(len(rows),4)
        videos=[row for row in rows if row['kind']=='replay']
        self.assertEqual(len(videos),3)
        for row in rows:self.assertTrue(flow.history.media(row['id']).is_file())
        self.assertEqual(run['receipt']['model_requests'],['prediction'])
        self.assertEqual(capture.manifest['streams']['control']['committed'],3)
        self.assertEqual(capture.manifest['streams']['model']['committed'],1)
        self.assertEqual({row['rule_id']for row in capture.guard_decisions},{'G33','G36','G37'})
        self.assertTrue(all(row['verdict']=='pass'and row['policy_hash']==policy['policy_hash']for row in capture.guard_decisions))
        for channel in self.service.channels.values():self.assertIsNone(channel.current_lease())

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_common_run_adapter_records_three_businesses_without_a_model_stream(self):
        from robot_workbench.inference_trace import CaptureRun
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        for index,business in enumerate(('calibration','collection','research')):
            parameters=control_parameters(defaults(business),business=business)
            if business=='research':parameters['recording']['purpose_profile']='data_required'
            policy=dict(revision=6,parameters=parameters,policy_hash=policy_hash(parameters))
            config=dict(clock=dict(source_clock_id=self.sources[0]['source_clock_id']))
            capture=CaptureRun('cpu-'+business,self.root/business,config,policy,self.service,'fake',None,lambda:True,
                lambda value:None,business_kind=business)
            self.addCleanup(capture.close);base=index*5
            capture.record('prepare',dict(capture_sequence=1,capture_source_time=100.+base/30.,execution_owner=self.fixture.owner))
            self.assertIsNone(capture.error)
            for frame in (base+1,base+2,base+3):self.publish(frame)
            capture.record('command',dict(capture_sequence=2,capture_source_time=100.+(base+2)/30.,q_sdk14=[0.]*14))
            capture.record('finish',dict(status='completed',final_capture_sequence=2,source_time=100.+(base+3)/30.))
            self.publish(base+4)
            self.assertTrue(capture.done.wait(30),capture.snapshot());self.assertIsNone(capture.error)
            self.assertEqual(capture.manifest['binding']['business_kind'],business)
            self.assertEqual(capture.manifest['recording_outcome'],'complete',capture.manifest['issues'])
            self.assertNotIn('model',capture.manifest['streams'])
            self.assertTrue(all(row['verdict']=='pass'for row in capture.guard_decisions),capture.guard_decisions)

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_control_adapter_binds_once_preserves_sample_times_and_drains_after_stop(self):
        from robot_workbench.recording_process import CaptureControlSession
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        from robot_workbench.sample_clock import clock_spec
        parameters=control_parameters(defaults('calibration'),business='calibration')
        policy=dict(revision=8,parameters=parameters,policy_hash=policy_hash(parameters))
        active=[True];now=[100.];faults=[]
        recording=CaptureControlSession(self.root/'ordinary',execution_instance_id='cpu-ordinary',business_kind='calibration',
            control_policy=policy,cameras=self.service,execution_owner=self.fixture.owner,environment='fake',
            clock=clock_spec('fake',source_clock_id='cpu-source'),source_time=lambda:now[0],
            on_fault=faults.append,preparation_active=lambda:active[0],preparation_deadline_monotonic=time.monotonic()+10)
        self.addCleanup(recording.close)
        self.assertTrue(recording.prepared.wait(10));self.assertIsNone(recording.error);self.assertTrue(recording.ready.is_set())
        lease=next(iter(self.service.channels.values())).current_lease()
        self.assertTrue(recording.lease_active(lease))
        recording.bind_execution(dict(timeline_digest='b'*64),execution_instance_id='cpu-ordinary',execution_owner=self.fixture.owner)
        now[0]=100.06
        recording.submit('feedback',sample_monotonic_s=100.01,raw=dict(qpos=[.1]*14))
        for index in (1,2,3):self.publish(index)
        now[0]=100.12;active[0]=False
        recording.finish('stopped','CPU stop returned')
        self.assertTrue(recording.lease_active(lease),'bound recording must drain its tail after the control epoch ends')
        self.publish(4)
        self.assertTrue(recording.done.wait(30),recording.snapshot());self.assertEqual(faults,[])
        receipt=json.loads((self.root/'ordinary/receipt.json').read_text())
        self.assertEqual(receipt['recording']['recording_outcome'],'complete')
        self.assertEqual(receipt['controller']['status'],'stopped')
        events=[json.loads(row)for row in (recording.root/'control.jsonl').read_text().splitlines()]
        self.assertEqual(events[1]['source_time'],100.06)
        self.assertEqual(events[1]['payload']['data']['sample_monotonic_s'],100.01)
        self.assertFalse(recording.lease_active(lease))

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_actual_calibration_entry_records_sdk_windows_feedback_and_three_videos(self):
        self.actual_control_entry('calibration')

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_actual_rule_collection_entry_archives_its_plan_and_continuous_streams(self):
        self.actual_control_entry('collection')

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_actual_research_entry_uses_explicit_purpose_and_the_same_recorder(self):
        self.actual_control_entry('research')

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_actual_stop_records_epoch_transition_and_the_camera_tail(self):
        self.actual_control_entry('calibration',stop=True)

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_rgb_only_actual_three_business_entries_and_stop_have_no_depth_artifacts(self):
        for business in ('calibration','collection','research'):
            with self.subTest(business=business):self.actual_control_entry(business,pixel_format='rgb8')
        self.actual_control_entry('calibration',stop=True,pixel_format='rgb8')

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_actual_required_calibration_writer_exit_preserves_prefix_and_stops(self):
        self.actual_control_entry('calibration',fault='writer_exit')

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_actual_required_collection_queue_fault_stops_and_marks_gap(self):
        self.actual_control_entry('collection',fault='queue')

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_actual_research_missing_tail_keeps_completed_control_and_incomplete_data(self):
        self.actual_control_entry('research',fault='missing_tail')

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_actual_debug_research_queue_fault_continues_with_explicit_gap(self):
        self.actual_control_entry('research',fault='queue',purpose='infer_debug')

    def actual_control_entry(self,business,stop=False,fault=None,purpose='data_required',pixel_format='rgb8-depth16le'):
        from robot_workbench.recording_process import capture_control
        from robot_workbench.sample_clock import clock_spec
        if business=='collection':
            from test_unified_runtime import UnifiedRuntimeTest
            harness=UnifiedRuntimeTest();harness.setUp();self.addCleanup(harness.doCleanups)
            runtime,credentials,command=harness.collection_case('recorded-rule-collection')
            now=lambda:harness.sim;advance=lambda:harness.advance(runtime)
            confirm=lambda review:harness.confirm(runtime,credentials[business],review)
        else:
            from test_shared_target_routes import SharedTargetRoutesTest
            harness=SharedTargetRoutesTest();harness.setUp();self.addCleanup(harness.doCleanups);harness.create()
            runtime=harness.runtime;credentials=harness.creds;now=lambda:harness.now;advance=harness.advance
            if business=='research':
                credentials[business]=runtime.register_business(business)
                runtime.handoff(credentials['calibration'],business);harness.advance(42)
                policy=runtime.control_policy_snapshot(business)
                runtime.update_control_policy(credentials[business],dict(expected_revision=policy['revision'],
                    changes={'recording.purpose_profile':purpose}))
            command=dict(action='preview_workbench',waypoints=[dict(q12=[.02]*12,speeds12=[.1]*12)])
            confirm=lambda review:harness.confirm(business,review)
        owner=runtime.owner;backend=owner.backend._backend;last=[-1];cameras=self.fixture.service(pixel_format=pixel_format)
        from robot_workbench.collection_history import CaptureResults
        record_root=self.root/'actual-runs'/runtime.device_id
        results=CaptureResults(record_root,environment='fake',device_id=runtime.device_id);self.addCleanup(results.close)
        runtime.capture_result_handler=results.notify
        def publish_current():
            index=int(now()*30+1e-7)
            if index<=last[0]:return
            last[0]=index;stamp=index*1_000_000_000//30
            for camera,channel in cameras.channels.items():
                meta=self.fixture.metadata(camera,index)
                meta.update(source_clock_id=backend.source_clock_id,physics_step=stamp,source_time_ns=stamp,
                    device_timestamp_ms=stamp/1e6,depth_device_timestamp_ms=stamp/1e6,step_kind='protocol_clock_tick',
                    owner_pid=os.getpid(),runtime_nonce=runtime.nonce,command_epoch=owner.epoch)
                lease=channel.current_lease()
                rgb,depth=self.fixture.pixels(index)
                if pixel_format=='rgb8':meta=self.fixture.rgb_metadata(meta);depth=None
                cameras.publish(camera,meta,rgb,depth,width=2,height=2,
                    sampled_token=lease['run_id']if lease else None)
        publish_current()
        def factory(payload,on_fault):
            return capture_control(record_root,payload,cameras=cameras,environment='fake',
                clock=clock_spec('fake',source_clock_id=backend.source_clock_id),source_time=now,on_fault=on_fault)
        factory.recording_backend='capture_session_v1';factory.supports_deferred_preparation=True;owner.recording_factory=factory
        review=runtime.submit(credentials[business],command);confirm(review)
        deadline=time.monotonic()+20;recording=None;requested=False;injected=False;freeze_video=False
        while time.monotonic()<deadline:
            advance()
            if not freeze_video:publish_current()
            recording=owner.recording
            if stop and not requested and getattr(backend,'packets',None):
                runtime.request_stop();requested=True
            sent=bool(getattr(backend,'packets',None))or any(row[0]=='send'for row in backend.events)
            if fault and not injected and recording is not None and sent:
                injected=True
                if fault=='writer_exit':
                    import signal
                    child=recording.capture.session
                    wait=time.monotonic()+5
                    while child.snapshot()['streams']['control'].get('committed',0)<1 and time.monotonic()<wait:time.sleep(.01)
                    self.assertGreater(child.snapshot()['streams']['control'].get('committed',0),0)
                    os.killpg(child.process.pid,signal.SIGKILL)
                elif fault=='queue':
                    recording.submit('owner_event',data='x'*recording.capture.session.spec['resource_budget']['message_bytes'])
                else:freeze_video=True
            if recording is not None and recording.done.is_set():break
            time.sleep(.002)
        self.assertIsNotNone(recording);self.addCleanup(recording.close)
        self.assertTrue(recording.done.is_set(),runtime.fact_snapshot())
        wait=time.monotonic()+10
        while results.page(business)['total_runs']<1 and time.monotonic()<wait:time.sleep(.01)
        indexed=results.page(business)
        self.assertEqual(indexed['total_runs'],1,results.error)
        receipt_row=next(row for row in indexed['results']if row['kind']=='control_recording')
        self.assertTrue(results.media(business,receipt_row['id']).is_file())
        with self.assertRaisesRegex(ValueError,'another device'):
            CaptureResults(record_root,environment='fake',device_id='foreign-device')
        detail=dict(reason=owner.runner.reason,error=recording.diagnostics_error,controller=recording.controller_result,
            clock=now(),streams={name:{k:row.get(k)for k in('source_time_range','missing','committed')}for name,row in (recording.manifest or{}).get('streams',{}).items()})
        rejected=bool(fault in ('writer_exit','queue')and purpose=='data_required')
        self.assertEqual(owner.runner.status,'STOPPED'if stop or rejected else'READY',str(detail))
        self.assertEqual(recording.snapshot()['recording_outcome'],'incomplete'if fault else'complete',recording.snapshot())
        if fault:
            self.assertTrue(injected)
            self.assertIsNotNone(recording.manifest,recording.snapshot())
            receipt=json.loads((recording.capture.directory/'receipt.json').read_text())
            self.assertEqual(receipt['business_kind'],business)
            self.assertEqual(receipt['control_policy']['policy_hash'],recording.capture.policy['policy_hash'])
            before=len(backend.events);advance();self.assertFalse(any(row[0]=='send'for row in backend.events[before:]))
            if rejected:self.assertEqual(recording.controller_result['status'],'failed')
            else:self.assertEqual(recording.controller_result['status'],'completed')
            self.assertTrue(all(channel.current_lease()is None for channel in cameras.channels.values()))
            if fault=='writer_exit':
                self.assertGreater(recording.manifest['streams']['control']['committed'],0)
                self.assertGreaterEqual(recording.manifest['seal_revision'],1)
            elif fault=='queue':self.assertGreater(recording.manifest['streams']['control']['missing'],0)
            else:self.assertTrue(any('endpoint'in issue for issue in recording.manifest['issues']))
            return
        self.assertEqual(len(recording.manifest['playback']['streams']),3)
        events=[json.loads(row)['payload']for row in (recording.root/'control.jsonl').read_text().splitlines()]
        self.assertTrue(any(row['event']=='feedback'and row['data'].get('phase')=='sdk_read'for row in events))
        self.assertTrue(any(row['event']=='command'and row['data'].get('operation')in('queue_reference','send')and row['data'].get('phase')=='returned'for row in events))
        self.assertTrue(all(row['verdict']=='pass'for row in recording.capture.guard_decisions),recording.capture.guard_decisions)
        self.assertTrue((recording.capture.directory/'receipt-r000001.json').is_file())
        if pixel_format=='rgb8':
            from rgbd_segments import iter_segment
            commits=list((recording.root/'camera').glob('*/*.commit.json'));self.assertTrue(commits)
            for path in commits:
                segment=json.loads(path.read_text());self.assertEqual(segment['schema'],'rgb-segment-v1')
                self.assertEqual(set(segment['files']),{'rgb','frames'})
                rows=list(iter_segment(recording.root/'camera',str(path.relative_to(recording.root/'camera')),png=False))
                self.assertTrue(rows);self.assertTrue(all(depth is None for meta,rgb,depth in rows))
        if stop:
            bound=recording.execution_owner['command_epoch']
            for source in recording.manifest['playback']['streams']:
                frames=[json.loads(row)for row in (recording.root/(source+'.jsonl')).read_text().splitlines()]
                self.assertTrue(any(row['payload']['metadata']['command_epoch']>bound for row in frames))
                self.assertTrue(all(row['payload']['metadata']['recording_execution_owner']==recording.execution_owner for row in frames))
                self.assertIsNone(cameras.channels[source.removeprefix('video-')].current_lease())
        if business=='collection':self.assertTrue((recording.root/'artifacts.jsonl').read_text().strip())

    def test_wrong_service_and_endpoint_clock_are_rejected(self):
        changed=deepcopy(self.sources);changed[0]['source_ref']['service_instance_id']='other-service'
        session=self.create(sources=changed)
        self.assertFalse(session.prepare()['ready']);session.close()
        self.assertTrue(all(channel.current_lease() is None for channel in self.service.channels.values()))

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_missing_tail_is_incomplete_and_only_own_lease_is_released(self):
        sources=self.sources[:1];session=self.create(sources=sources);self.bind_start(session)
        self.publish(1)
        with self.assertRaises(ValueError):session.finish({},dict(status='completed'),source_endpoints={sources[0]['stream_id']:dict(source_clock_id='wrong',source_time=100.5)})
        session.finish({},dict(status='completed'),source_endpoints=self.endpoints(sources,100.5))
        self.assertTrue(session.done.wait(30),session.snapshot())
        manifest=json.loads((session.directory/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'incomplete')
        self.assertTrue(any('endpoint' in issue for issue in manifest['issues']))
        self.assertIsNone(self.service.channels[sources[0]['source_ref']['camera']].current_lease())

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_crashed_writer_recovery_releases_only_its_camera_lease(self):
        import signal
        session=self.create(sources=self.sources[:1]);self.bind_start(session)
        camera=self.sources[0]['source_ref']['camera']
        self.assertIsNotNone(self.service.channels[camera].current_lease())
        os.killpg(session.process.pid,signal.SIGKILL);self.assertTrue(session.done.wait(10));session.thread.join(1)
        session.recover();self.assertTrue(session.done.wait(30),session.snapshot())
        self.assertIsNone(self.service.channels[camera].current_lease())
        self.assertEqual(json.loads((session.directory/'manifest.json').read_text())['recording_outcome'],'incomplete')

    def test_missing_codec_is_not_reported_as_ready(self):
        from robot_workbench.recording_process import _environment
        environment=_environment();environment['PATH']='/not-an-installed-codec-directory'
        with patch('robot_workbench.recording_process._environment',return_value=environment):
            session=self.create()
            state=session.prepare()
        self.assertFalse(state['ready'])
        self.assertTrue(any('codecs' in reason for reason in state['source_errors'].values()))

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'requires installed CPU codecs')
    def test_immediate_finish_drains_frames_already_accepted(self):
        sources=self.sources[:1];session=self.create(sources=sources);self.bind_start(session)
        camera=sources[0]['source_ref']['camera'];channel=self.service.channels[camera]
        token=channel.current_lease()['run_id']
        for index in range(1,6):self.assertTrue(self.fixture.publish(self.service,camera,index,token)['recorded'])
        session.finish({},dict(status='failed',reason='CPU cancelled run'),source_endpoints=self.endpoints(sources,0.))
        self.assertTrue(session.done.wait(30),session.snapshot())
        manifest=json.loads((session.directory/'manifest.json').read_text())
        self.assertEqual(manifest['recording_outcome'],'complete',manifest['issues'])
        self.assertEqual(manifest['streams'][sources[0]['stream_id']]['committed'],5)


class RecordingProcessTest(unittest.TestCase):
    def test_clean_child_keeps_selected_codecs_without_inheriting_python_or_cuda_paths(self):
        from robot_workbench.recording_process import _environment
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name in ('ffmpeg','ffprobe'):
                tool=root/name;tool.write_text('#!/bin/sh\necho selected-'+name+'\n');tool.chmod(0o700)
            with patch.dict(os.environ,dict(PATH=str(root)+os.pathsep+'/unrelated/runtime/bin',
                    PYTHONHOME='/bad/python',LD_LIBRARY_PATH='/bad/cuda')):
                env=_environment()
            self.assertNotIn('/unrelated/runtime/bin',env['PATH'])
            self.assertNotIn('PYTHONHOME',env);self.assertNotIn('LD_LIBRARY_PATH',env)
            self.assertEqual(env['CUDA_VISIBLE_DEVICES'],'')
            for name in ('ffmpeg','ffprobe'):
                self.assertEqual(subprocess.check_output([name],env=env,text=True).strip(),'selected-'+name)

    def session(self,root,service,owner,environment,kind='frozen',bad=None):
        source,plan,binding,timeline=plan_fixture(4,environment)
        if kind=='streaming':
            source=dict(schema_version='policy_execution_source.v1',environment=environment,
                robot_id=source['robot_id'],camera_serials=source['camera_serials'],calibration_profile=source['calibration_profile'],
                clock=clock_spec(environment,source_clock_id='cpu-source' if environment=='fake' else None),
                execution_instance_id='process-test',runtime_nonce=owner['runtime_nonce'],command_epoch=owner['command_epoch'],duration_s=.1)
            binding=dict(calibration_accepted=True,robot_id=source['robot_id'],camera_serials=source['camera_serials'],
                calibration_profile_digest=_digest(source['calibration_profile']))
            options=dict(session_kind='streaming',source=source,binding=binding)
        else:
            ref=dict(schema='collection_plan_ref.v1',environment=environment,
                origin=dict(owner_pid=owner['owner_pid'],runtime_nonce=owner['runtime_nonce'],epoch=owner['command_epoch']),
                **{key:artifact(root,key+'.json',value) for key,value in [('source',source),('plan',plan),('binding',binding)]})
            if bad=='owner':ref['origin']['epoch']+=1
            if bad=='hash':ref['plan']['sha256']='0'*64
            options=dict(plan_ref=ref)
        timeline_ref=dict(execution_timeline_ref=artifact(root,'timeline.json',timeline))
        session=ProcessRecordingSession(root/'raw',camera_channels=service.channels,execution_owner=owner,
            execution_instance_id='process-test',environment=environment,storage_format='zip',persistent_service=True,
            max_wall_time_seconds=30,preparation_deadline_monotonic=time.monotonic()+10,**options)
        return session,source,plan,timeline_ref

    def close(self,session):
        if not session.done.is_set():session.finish('stopped','test cleanup')
        self.assertTrue(session.done.wait(15),session.snapshot());session.thread.join(2)
        self.assertFalse(session.thread.is_alive());self.assertIsNotNone(session.process.poll())

    def test_frozen_and_policy_both_clocks_actual_process_and_exact_pixels(self):
        for kind in ('frozen','streaming'):
            for environment in ('fake','physical'):
                with self.subTest(kind=kind,environment=environment),tempfile.TemporaryDirectory() as tmp:
                    root=Path(tmp);fixture=fixtures.SharedCameraTest();self.addCleanup(fixture.doCleanups)
                    service=fixture.service(environment,camera_serials={'head':'h','left':'l','right':'r'})
                    with patch('robot_workbench.control_plan.load_artifact',side_effect=AssertionError('parent must not load large artifacts')):
                        session,source,plan,timeline=self.session(root,service,fixture.owner,environment,kind)
                        try:
                            self.assertTrue(session.prepared.wait(10),session.snapshot())
                            self.assertNotEqual(session.worker_pid,os.getpid())
                            self.assertTrue(all(ch.cached_token for ch in service.channels.values()))
                            epoch=time.time_ns();mono=time.monotonic();clock=source.get('clock') or clock_spec(environment)
                            def publish(index):
                                for camera,channel in service.channels.items():
                                    meta=fixture.metadata(camera,index);meta.update(clock_domain=clock['clock_domain'],source_clock_id=clock['source_clock_id'],host_receive_ns=epoch+index*1_000_000_000//30)
                                    service.publish(camera,meta,*fixture.pixels(index),width=2,height=2,sampled_token=channel.cached_token)
                            publish(0)
                            if kind=='frozen':
                                session.bind_execution(timeline,execution_instance_id='process-test',execution_owner=fixture.owner)
                                if environment=='physical':self.assertTrue(session.ready.wait(10),session.snapshot())
                            for index in range(5):
                                wall=epoch+index*1_000_000_000//30;source_ns=100_000_000_000+index*1_000_000_000//30
                                observed=dict(fixture.owner,sample_wall_ns=wall,clock_domain=clock['clock_domain'],source_clock_id=clock['source_clock_id'])
                                if environment=='fake':observed.update(source_time_ns=source_ns,physics_step=index*8)
                                now=source_ns/1e9 if environment=='fake' else mono+index/30
                                target=plan['ticks'][min(index,3)]['target']
                                session.append_feedback(tick=min(index,3),state=target,sample_monotonic_s=now,sample_wall_ns=wall,raw=observed)
                                if index<4:
                                    fields=dict(fixture.owner,clock_domain=clock['clock_domain'],source_clock_id=clock['source_clock_id'],feedback_at_send=observed)
                                    if kind=='frozen' and environment=='physical':fields={}
                                    if environment=='fake':fields.update(send_source_time_ns=source_ns,physics_step=None)
                                    session.append_command(tick=index,target=target,send_monotonic_s=now,send_wall_ns=wall,**fields)
                                    if index==0:
                                        native=dict(schema='workbench.target_window.v1',reference_id='a'*64,active_arm='both',
                                            controller_profile='normal_X5',host_times_s=[now+.03,now+.23],
                                            q_sdk14=[[0.]*14]*2,dq_sdk14=[[0.]*14]*2,torque_bias_sdk14=[[0.]*14]*2)
                                        session.submit('native_window',packet=native,admission=dict(firmware_ack=False),
                                            send_monotonic_s=now,send_wall_ns=wall)
                                if index:publish(index)
                            session.finish('completed');self.assertTrue(session.done.wait(15),session.snapshot())
                            self.assertIsNone(session.error,session.snapshot());self.assertIsNone(session.seal_error,session.snapshot())
                            manifest,_,_=verify_raw(session.root,environment=environment)
                            self.assertEqual(manifest['capture_status'],'complete',manifest)
                            self.assertEqual(manifest['native_window_count'],1)
                            native_rows=[json.loads(row)for row in(session.root/'native-windows.jsonl').read_text().splitlines()]
                            self.assertEqual(native_rows[0]['packet'],native)
                            self.assertEqual(manifest['native_windows_sha256'],manifest['files']['native-windows.jsonl']['sha256'])
                            for camera in service.channels:
                                rows=list(iter_rgbd(session.root,camera,environment=environment));self.assertEqual(len(rows),5)
                                from PIL import Image
                                for index,(_,_,color,depth) in enumerate(rows):
                                    with Image.open(io.BytesIO(color)) as picture:self.assertEqual(picture.tobytes(),fixture.pixels(index)[0])
                                    with Image.open(io.BytesIO(depth)) as picture:self.assertEqual(list(picture.getdata()),[0,1,32768,65535])
                            self.assertEqual(service.snapshot()['recording_budget']['queued_bytes'],0)
                            self.assertTrue(all(ch.cached_token is None for ch in service.channels.values()))
                        finally:self.close(session)

    def test_owner_epoch_and_hash_fail_closed_before_camera_lease(self):
        for bad in ('owner','hash'):
            with self.subTest(bad=bad),tempfile.TemporaryDirectory() as tmp:
                fixture=fixtures.SharedCameraTest();self.addCleanup(fixture.doCleanups);service=fixture.service()
                session,*_=self.session(Path(tmp),service,fixture.owner,'fake',bad=bad)
                try:
                    self.assertTrue(session.done.wait(10),session.snapshot());self.assertTrue(session.error)
                    self.assertFalse(session.prepared.is_set())
                    self.assertTrue(all(ch.current_lease() is None for ch in service.channels.values()))
                finally:self.close(session)

    def test_stop_before_binding_releases_only_own_leases_and_exits(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture=fixtures.SharedCameraTest();self.addCleanup(fixture.doCleanups);service=fixture.service('physical',camera_serials={'head':'h','left':'l','right':'r'})
            session,*_=self.session(Path(tmp),service,fixture.owner,'physical')
            self.assertTrue(session.prepared.wait(10),session.snapshot());session.finish('stopped','revoked')
            self.close(session)
            self.assertTrue(all(ch.current_lease() is None for ch in service.channels.values()))
            self.assertIsNone(service.process.poll(),'shared camera service is not a recording child')

    def test_child_crash_closes_leases_retains_prefix_and_never_claims_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            fixture=fixtures.SharedCameraTest();self.addCleanup(fixture.doCleanups)
            service=fixture.service('physical',camera_serials={'head':'h','left':'l','right':'r'})
            session,*_=self.session(Path(tmp),service,fixture.owner,'physical')
            self.assertTrue(session.prepared.wait(10),session.snapshot())
            deadline=time.monotonic()+10
            while not (session.root/'source.json').is_file() and not session.done.is_set() and time.monotonic()<deadline:time.sleep(.01)
            self.assertTrue((session.root/'source.json').is_file(),'crash fixture requires an actual persisted prefix')
            session.process.kill()
            self.close(session)
            self.assertTrue(session.error);self.assertFalse((session.manifest or {}).get('capture_status')=='complete')
            self.assertTrue(session.root.is_dir(),'partial recording survives worker loss')
            self.assertTrue(all(ch.current_lease() is None for ch in service.channels.values()))
            self.assertIsNone(service.process.poll())


if __name__=='__main__':unittest.main()
