"""Actual CPU subprocess failures; no CAN, SDK, model, camera or GPU imports."""
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import gc
import mmap
from pathlib import Path
import socket
import subprocess
import sys
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench.sdk_process import SDKProcess,SDKCommandRevoked,SDKCommandRejected,send,receive
from robot_workbench.control_alerts import ControlAlerts

WORKER='''import os,socket,sys,time,threading,signal
sys.path.insert(0,sys.argv[1])
from robot_workbench.sdk_process import serve_worker,ReferenceWindowRejected
class Backend:
 is_connected=False
 def connect(self):self.is_connected=True
 def read_calibration_state(self):
  threading.Timer(.15,lambda:os.kill(os.getpid(),signal.SIGKILL)).start()
  return dict(qpos=[0.]*14,future_queued=getattr(self,'future',False),can_output_guard=dict(heartbeat_monotonic_s=time.monotonic()))
 def close(self):self.is_connected=False
 def read_gains(self):return dict(future_queued=getattr(self,'future',False))
 def read_execution_output(self):raise ReferenceWindowRejected('coherent output unsupported')
 def begin(self,side,q,**kw):
  if kw.get('preserve_jaws'):raise ReferenceWindowRejected('strict begin output unavailable')
  return q
 def set_execution_gains(self,rows):
  self.command_guard()
  if rows.get('reject'):raise ReferenceWindowRejected('invalid mode gain')
  return dict(returned=True,firmware_ack=False)
 def get_bimanual_joint_position_limits(self):return [[-3.,3.]]*14
 def configure_motion_gains(self,value,**kw):
  if kw.get('preserve_hold'):self.queue_reference(value);return dict(preserved_hold=True)
  return dict(configured=value is not None,hardware_write=False)
 def send(self,*args):self.command_guard();return None
 def set_damping_hold(self,**kwargs):self.future=False;return dict(stopped=True)
 def queue_reference(self,value):
  if value.get('reject'):raise ReferenceWindowRejected('bounded, ordered future reference window required')
  if value.get('native_failure'):raise ValueError('unclassified native failure')
  self.future=True;time.sleep(.03)
  try:self.command_guard()
  except BaseException:
   self.future=False
   if not value.get('lie'):self.reference_cancel_sequence=getattr(self,'reference_cancel_sequence',0)+1
   raise
import mmap
epoch=mmap.mmap(int(sys.argv[3]),32,access=mmap.ACCESS_READ)
serve_worker(socket.socket(fileno=int(sys.argv[2])),Backend(),epoch)
'''

SUPERVISOR='''import subprocess,sys,os,json,time
fd,epoch_fd=int(sys.argv[3]),int(sys.argv[4])
child=subprocess.Popen([sys.executable,'-B','-c',sys.argv[1],sys.argv[2],str(fd),str(epoch_fd)],pass_fds=(fd,epoch_fd))
code=child.wait()
from pathlib import Path
Path(sys.argv[5]).write_text(json.dumps(dict(state='latched'if code else'disconnected',supervisor_pid=os.getpid(),child_pid=child.pid,
 reason='SDK owner heartbeat lost'if code else None,process_evidence=dict(wait=dict(status='returned'),returncode=code,reaped_monotonic_ns=time.monotonic_ns()))))
raise SystemExit(int(sys.argv[6])if len(sys.argv)>6 else 1 if code else 0)
'''

PARENT_BOUND_WORKER='''import sys,socket,mmap,time
sys.path.insert(0,sys.argv[1])
from robot_workbench.arx_output_guard import parent_death
from robot_workbench.sdk_process import serve_worker
parent_death(int(sys.argv[4]))
class Backend:
 is_connected=False
 def connect(self):self.is_connected=True;return dict(heartbeat_monotonic_s=time.monotonic())
 def read_calibration_state(self):return dict(qpos=[0.]*14,can_output_guard=dict(heartbeat_monotonic_s=time.monotonic()))
 def close(self):self.is_connected=False
serve_worker(socket.socket(fileno=int(sys.argv[2])),Backend(),mmap.mmap(int(sys.argv[3]),32,access=mmap.ACCESS_READ))
'''

CONTEXT_WORKER='''import sys,socket,mmap,time
sys.path.insert(0,sys.argv[1])
from robot_workbench.sdk_process import serve_worker,EXECUTION_CONTEXT_SCHEMA
class Backend:
 is_connected=False
 execution_context_capabilities={'schema':EXECUTION_CONTEXT_SCHEMA}
 bindings=0
 future=False
 last_window=None
 def connect(self):self.is_connected=True;return {}
 def close(self):self.is_connected=False
 def bind_execution_context(self,value):
  if value['controller_profile']=='slow':time.sleep(.03)
  self.context=value;self.bindings+=1;return dict(bound=True)
 def queue_reference(self,value):
  self.command_guard();self.future=True;self.last_window=value
  return dict(accepted=True,reference_id=value['reference_id'])
 def set_damping_hold(self,**kwargs):self.future=False
 def read_gains(self):return dict(bindings=self.bindings,window=self.last_window,future=self.future)
epoch=mmap.mmap(int(sys.argv[3]),32,access=mmap.ACCESS_READ)
serve_worker(socket.socket(fileno=int(sys.argv[2])),Backend(),epoch)
'''


class SDKProcessTests(unittest.TestCase):
    def test_mismatched_worker_protocol_refuses_before_native_connect(self):
        with TemporaryDirectory()as folder:
            marker=Path(folder)/'native-connect-was-called'
            worker=CONTEXT_WORKER.replace("{'schema':EXECUTION_CONTEXT_SCHEMA}","{'schema':'obsolete-context'}")
            worker=worker.replace('def connect(self):self.is_connected=True;return {}',
                'def connect(self):__import__("pathlib").Path('+repr(str(marker))+').touch();self.is_connected=True;return {}')
            backend=self.fixture(worker=worker)
            with self.assertRaisesRegex(RuntimeError,'protocol version mismatch'):backend.connect()
            self.assertFalse(marker.exists());self.assertFalse(backend.is_connected)

    def test_pinned_sdk_import_precedes_installed_modules_and_rejects_wrong_preload(self):
        from scripts.serve_sdk_control import _bind_sdk_import
        from types import SimpleNamespace
        with TemporaryDirectory()as tmp:
            path=Path(tmp)/'pinned'/'arx5_interface.so';path.parent.mkdir();path.touch()
            profile={'sdk_quiesce':{'module':{'path':str(path)},'library_build_id':'expected'}}
            with patch.object(sys,'path',list(sys.path)),patch.dict(sys.modules,{},clear=False):
                sys.modules.pop('arx5_interface',None);_bind_sdk_import(profile)
                self.assertEqual(Path(sys.path[0]),path.parent.resolve())
                sys.modules['arx5_interface']=SimpleNamespace(__file__=str(Path(tmp)/'old.so'),workbench_build_id='expected')
                with self.assertRaisesRegex(ValueError,'pinned candidate'):_bind_sdk_import(profile)

    def context_packet(self, profile='normal_X5'):
        from robot_workbench.target_interpreter import SCHEMA
        now=time.monotonic()
        return dict(schema=SCHEMA,controller_profile=profile,reference_id='a'*64,active_arm='both',
            host_times_s=[now+.03,now+.05],q_sdk14=[[0.]*14]*2,dq_sdk14=[[0.]*14]*2,torque_bias_sdk14=[[0.]*14]*2)

    def test_bound_context_windows_are_minimal_and_repeat_binding_is_memory_only(self):
        from robot_workbench.sdk_process import ReferenceWindowRejected
        backend=self.fixture(worker=CONTEXT_WORKER);backend.connect()
        context=dict(controller_profile='normal_X5',jaw_calibration_id=None)
        with self.assertRaises(SDKCommandRevoked):backend.queue_reference(self.context_packet())
        backend.bind_execution_context(context);sequence=backend.sequence
        for _ in range(4):self.assertTrue(backend.bind_execution_context(context)['bound'])
        self.assertEqual(backend.sequence,sequence)
        packet=self.context_packet();backend.queue_reference(packet)
        facts=backend.read_gains();self.assertEqual(facts['bindings'],1)
        self.assertEqual(set(facts['window']),{'reference_id','active_arm','host_times_s','q_sdk14','dq_sdk14'})
        packet['torque_bias_sdk14']=[[.01]*14]*2;backend.queue_reference(packet)
        self.assertIn('torque_bias_sdk14',backend.read_gains()['window'])
        packet['torque_bias_sdk14']=[[0.]*14]
        with self.assertRaises(ReferenceWindowRejected):backend.queue_reference(packet)
        self.assertTrue(backend.is_connected);backend.close()

    def test_context_change_renews_generation_and_stop_requires_rebinding(self):
        backend=self.fixture(worker=CONTEXT_WORKER);backend.connect()
        context=dict(controller_profile='normal_X5',jaw_calibration_id=None)
        backend.bind_execution_context(context);prior=backend.generation
        changed=dict(context,limits=dict(velocity_limit_rad_s=1.,acceleration_limit_rad_s2=40.,jaw_velocity_limit_mm_s=40.))
        backend.bind_execution_context(changed)
        self.assertNotEqual(backend.generation,prior)
        backend.queue_reference(self.context_packet());prior=backend.generation;backend.revoke_output()
        compact={k:v for k,v in self.context_packet().items() if k not in ('schema','controller_profile','torque_bias_sdk14')}
        with self.assertRaises(SDKCommandRejected):backend.call('queue_reference',compact)
        with self.assertRaises(SDKCommandRevoked):backend.call('queue_reference',compact,_generation=prior)
        self.assertFalse(backend.read_gains()['future']);self.assertTrue(backend.is_connected)
        backend.bind_execution_context(changed);backend.queue_reference(self.context_packet());backend.close()
        replacement=self.fixture(worker=CONTEXT_WORKER);replacement.connect()
        with self.assertRaises(SDKCommandRevoked):replacement.queue_reference(self.context_packet())
        replacement.close()

    def test_stop_during_context_binding_does_not_authorize_a_window(self):
        backend=self.fixture(worker=CONTEXT_WORKER);backend.connect()
        timer=threading.Timer(.01,backend.revoke_output);timer.start()
        with self.assertRaises(SDKCommandRevoked):
            backend.bind_execution_context(dict(controller_profile='slow',jaw_calibration_id=None))
        timer.join()
        self.assertIsNone(backend._bound_execution_context)
        self.assertTrue(backend.is_connected)
        with self.assertRaises(SDKCommandRevoked):backend.queue_reference(self.context_packet('slow'))
        backend.bind_execution_context(dict(controller_profile='normal_X5',jaw_calibration_id=None))
        backend.queue_reference(self.context_packet());backend.close()

    def test_worker_rejects_context_replacement_without_generation_and_extra_metadata(self):
        backend=self.fixture(worker=CONTEXT_WORKER);backend.connect()
        context=dict(controller_profile='normal_X5',jaw_calibration_id=None)
        backend.bind_execution_context(context)
        with self.assertRaises(SDKCommandRejected):
            backend.call('bind_execution_context',dict(context,controller_profile='other'))
        with self.assertRaises(SDKCommandRejected):backend.call('queue_reference',self.context_packet())
        with self.assertRaises(ValueError):backend.bind_execution_context(dict(context,runtime_nonce='unnecessary'))
        self.assertTrue(backend.is_connected);backend.close()

    def test_strict_begin_and_output_capability_refusal_keep_healthy_sdk_connected(self):
        backend=self.fixture();backend.connect();pid=backend.process.pid
        for call in (backend.read_execution_output,lambda:backend.begin('left',[0.]*14,preserve_jaws=True)):
            with self.assertRaises(SDKCommandRejected):call()
            self.assertIsNone(backend.failure);self.assertTrue(backend.is_connected)
            self.assertEqual(backend.process.pid,pid)
        self.assertEqual(backend.set_damping_hold(manual=False),dict(stopped=True))
        backend.close()

    def test_mode_gain_rpc_uses_same_epoch_channel_and_typed_refusal(self):
        backend=self.fixture();backend.connect()
        self.assertTrue(backend.set_execution_gains({})['returned'])
        with self.assertRaises(SDKCommandRejected):backend.set_execution_gains({'reject':True})
        self.assertTrue(backend.is_connected);self.assertIsNone(backend.failure)
        backend.close()

    def test_disabled_point_send_is_rejected_before_ipc_without_tripping_sdk(self):
        from robot_workbench.sdk_process import ReferenceWindowRejected
        backend=object.__new__(SDKProcess);backend.target_interpreter_profile='x5_control50_damping_v1'
        with patch.object(backend,'call')as call:
            with self.assertRaisesRegex(ReferenceWindowRejected,'point-send is disabled'):backend.send('left',[0.]*14)
            call.assert_not_called()
            backend.target_interpreter_profile=None;backend.send('left',[0.]*14)
            call.assert_called_once_with('send','left',[0.]*14)

    def test_static_limits_are_connection_scoped_and_never_hide_worker_loss(self):
        backend=self.fixture();backend.connect();before=backend.sequence
        limits=backend.get_bimanual_joint_position_limits();self.assertEqual(backend.sequence,before+1)
        limits[0][0]=-999.
        for _ in range(20):self.assertEqual(backend.get_bimanual_joint_position_limits(),[[-3.,3.]]*14)
        self.assertEqual(backend.sequence,before+1,'steady ticks must not send another static RPC')
        backend.connect();before=backend.sequence;backend.get_bimanual_joint_position_limits()
        self.assertEqual(backend.sequence,before+1,'an explicit connect must invalidate prior limits')
        backend.read_calibration_state();backend.process.wait(timeout=3)
        with self.assertRaisesRegex(RuntimeError,'not connected'):backend.get_bimanual_joint_position_limits()
        fresh=self.fixture();fresh.connect();before=fresh.sequence;fresh.get_bimanual_joint_position_limits()
        self.assertEqual(fresh.sequence,before+1);fresh.close()
        self.assertIsNone(fresh._joint_limits)
        with self.assertRaises(RuntimeError):fresh.get_bimanual_joint_position_limits()

    def test_feedback_rpc_reports_worker_and_client_timing_without_extra_calls(self):
        backend=self.fixture();backend.connect();before=backend.sequence
        result=backend.read_calibration_state()
        self.assertEqual(backend.sequence,before+1)
        worker=result['sdk_worker_read_timing'];client=result['sdk_client_read_timing']
        self.assertEqual(backend.worker_pid,worker['pid'])
        self.assertGreaterEqual(worker['execute_s'],0.)
        self.assertGreaterEqual(client['total_s'],worker['execute_s'])
        self.assertAlmostEqual(client['lock_wait_s']+client['request_encode_send_s']+client['response_wait_decode_s'],client['total_s'])
        backend.process.wait(timeout=3)  # This existing fixture intentionally SIGKILLs itself after a read.
    def test_application_window_rejection_keeps_channel_available_for_stop(self):
        backend=self.fixture();backend.connect();pid=backend.process.pid
        backend.queue_reference({})
        timing=backend.last_reference_timing
        self.assertGreaterEqual(timing['worker']['wall_s'],.03)
        client=timing['client']
        self.assertAlmostEqual(client['total_s'],client['lock_wait_s']+client['request_encode_send_s']+client['response_wait_decode_s'])
        with self.assertRaisesRegex(SDKCommandRejected,'bounded, ordered future reference window')as caught:
            backend.queue_reference({'reject':True})
        self.assertIs(caught.exception.timing,backend.last_reference_timing)
        self.assertIsNot(caught.exception.timing,timing,'failed call must carry its own timing')
        self.assertGreaterEqual(caught.exception.timing['worker']['cpu_s'],0.)
        self.assertIsNone(backend.failure);self.assertTrue(backend.is_connected)
        self.assertEqual(backend.process.pid,pid);self.assertIsNone(backend.process.poll())
        self.assertEqual(backend.set_damping_hold(manual=False),dict(stopped=True))
        self.assertFalse(backend.read_gains()['future_queued'])
        backend.close()

    def test_unclassified_native_error_is_not_misreported_as_application_rejection(self):
        backend=self.fixture();backend.connect()
        with self.assertRaisesRegex(RuntimeError,'unclassified native failure'):
            backend.queue_reference({'native_failure':True})
        self.assertIsNotNone(backend.failure);self.assertFalse(backend.is_connected)

    def test_gain_configuration_is_an_explicit_ram_only_worker_call(self):
        backend=self.fixture();backend.connect()
        self.assertEqual(backend.configure_motion_gains({'cpu':'fixture'}),dict(configured=True,hardware_write=False))
        backend.close()

    def test_hold_gain_rpc_revocation_preserves_connection_only_after_cancel(self):
        backend=self.fixture();backend.connect()
        timer=threading.Timer(.01,backend.revoke_output);timer.start()
        with self.assertRaises(SDKCommandRevoked):backend.configure_motion_gains({},preserve_hold=True)
        timer.join();self.assertIsNone(backend.failure);self.assertTrue(backend.is_connected)
        self.assertFalse(backend.read_gains()['future_queued']);backend.close()

    def test_revoked_reference_with_confirmed_cancel_keeps_sdk_connected(self):
        backend=self.fixture();backend.connect()
        timer=threading.Timer(.01,backend.revoke_output);timer.start()
        with self.assertRaises(SDKCommandRevoked):backend.queue_reference({})
        timer.join();self.assertIsNone(backend.failure);self.assertTrue(backend.is_connected)
        self.assertFalse(backend.read_gains()['future_queued'])
        backend.close()

    def test_unconfirmed_reference_cancel_still_fails_closed(self):
        backend=self.fixture();backend.connect()
        timer=threading.Timer(.01,backend.revoke_output);timer.start()
        with self.assertRaises(RuntimeError):backend.queue_reference({'lie':True})
        timer.join();self.assertIsNotNone(backend.failure)
    @unittest.skipUnless(sys.platform=='linux','real Linux PR_SET_PDEATHSIG required')
    def test_supervisor_creator_outlives_the_temporary_http_request_thread(self):
        with TemporaryDirectory() as temporary:
            backend=SDKProcess(dict(storage_root=temporary,disabled_grippers=[]))
            actual_popen=subprocess.Popen;errors=[]
            def spawn(command,**kwargs):
                values=[command[command.index(key)+1]for key in ('--fd','--epoch-fd','--owner')]
                return actual_popen([sys.executable,'-B','-c',PARENT_BOUND_WORKER,str(ROOT),*values],**kwargs)
            def connect():
                try:backend.connect()
                except Exception as error:errors.append(str(error))
            try:
                with patch('robot_workbench.sdk_process.subprocess.Popen',side_effect=spawn):
                    request=threading.Thread(target=connect,name='temporary-http-request')
                    request.start();request.join(5)
                self.assertFalse(request.is_alive());self.assertEqual(errors,[])
                with self.assertRaises(subprocess.TimeoutExpired):backend.process.wait(timeout=.1)
                self.assertTrue(backend.supervisor_owner.is_alive())
                self.assertFalse(gc.isenabled())
                self.assertEqual(len(backend.read_calibration_state()['qpos']),14)
                backend.close();backend.supervisor_owner.join(1)
                self.assertFalse(backend.supervisor_owner.is_alive())
                self.assertTrue(gc.isenabled())
            finally:
                if backend.process is not None:
                    if backend.process.poll() is None:backend.process.terminate()
                    backend.process.wait(timeout=2);backend.dispose()

    def fixture(self, supervisor_exit=None, worker=None):
        worker = WORKER if worker is None else worker
        backend=SDKProcess({'storage_root':'/unused','disabled_grippers':['right']})
        backend.connection,peer=socket.socketpair()
        backend.attempt='cpu-owned-attempt'
        import tempfile
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        backend.guard_record=Path(directory.name)/'guard.json'
        backend.epoch_file=tempfile.TemporaryFile();backend.epoch_file.truncate(32)
        backend.epoch=mmap.mmap(backend.epoch_file.fileno(),32);backend.epoch[:]=backend.generation.encode()
        command=[sys.executable,'-B','-c',SUPERVISOR,worker,str(ROOT),str(peer.fileno()),str(backend.epoch_file.fileno()),str(backend.guard_record)]
        if supervisor_exit is not None:command.append(str(supervisor_exit))
        backend.process=subprocess.Popen(command,
            pass_fds=(peer.fileno(),backend.epoch_file.fileno()),stderr=subprocess.DEVNULL)
        peer.close()
        self.addCleanup(backend.connection.close)
        def cleanup():
            if backend.process.poll() is None:backend.process.kill()
            backend.process.wait(timeout=3)
            backend.dispose()
        self.addCleanup(cleanup)
        return backend

    def test_sdk_loss_does_not_kill_business_http_and_never_restarts(self):
        backend=self.fixture();backend.connect();backend.read_calibration_state()
        self.assertEqual(backend.protection_snapshot()['heartbeat']['status'],'healthy')
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                raw=json.dumps(backend.protection_snapshot()).encode()
                self.send_response(200);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        self.addCleanup(server.server_close);self.addCleanup(server.shutdown)
        backend.process.wait(timeout=3)
        with self.assertRaises(RuntimeError):backend.send('left',[0.]*14)
        connection=HTTPConnection('127.0.0.1',server.server_port,timeout=2)
        connection.request('GET','/');response=connection.getresponse();value=json.loads(response.read());connection.close()
        self.assertEqual(response.status,200);self.assertFalse(value['active_anomaly'])
        self.assertEqual(value['heartbeat'],dict(status='not_monitored',age_s=None,monitored=False))
        self.assertTrue(value['recovery_required'])
        self.assertFalse(value['event']['sdk_close_confirmed']);self.assertTrue(value['worker_exit_confirmed'])
        record=json.loads(backend.guard_record.read_text())
        record['process_evidence']['wait']['status']='timeout'
        backend.guard_record.write_text(json.dumps(record))
        unknown=backend.protection_snapshot()
        self.assertTrue(unknown['active_anomaly']);self.assertEqual(unknown['heartbeat']['status'],'unavailable')
        self.assertFalse(unknown['worker_exit_confirmed']);self.assertTrue(unknown['recovery_required'])
        pid=backend.process.pid
        with self.assertRaises(RuntimeError):backend.connect()
        self.assertEqual(backend.process.pid,pid)

    def test_live_protection_snapshot_never_reads_disk(self):
        from types import SimpleNamespace
        backend=SDKProcess({})
        backend.process=SimpleNamespace(pid=123,poll=lambda:None,returncode=None)
        backend.connected=True;backend.heartbeat=time.monotonic()
        backend.guard_record=Path('/not-read/guard.json')
        with patch('robot_workbench.sdk_process.os.open',side_effect=AssertionError('disk in live health read')):
            self.assertEqual(backend.protection_snapshot()['heartbeat']['status'],'healthy')
            backend.heartbeat=time.monotonic()-.101
            self.assertTrue(backend.protection_snapshot()['active_anomaly'])

    def test_live_heartbeat_timeout_is_still_an_active_anomaly(self):
        from types import SimpleNamespace
        backend=SDKProcess({})
        backend.process=SimpleNamespace(pid=123,poll=lambda:None,returncode=None)
        backend.connected=True;backend.heartbeat=time.monotonic()-.101
        value=backend.protection_snapshot()
        self.assertEqual(value['heartbeat']['status'],'lost')
        self.assertTrue(value['heartbeat']['monitored']);self.assertTrue(value['active_anomaly'])
        self.assertEqual((backend.watchdog_decision['rule_id'],backend.watchdog_decision['action']),('G30','FAULT_STOP'))

    def test_recorded_guard_ipc_failure_is_distinct_from_a_current_heartbeat_fault(self):
        from types import SimpleNamespace
        with TemporaryDirectory()as temporary:
            backend=SDKProcess({});backend.connected=True
            backend.process=SimpleNamespace(pid=123,poll=lambda:1,returncode=1)
            backend.guard_record=Path(temporary)/'guard.json'
            receipt=dict(state='latched',supervisor_pid=123,child_pid=124,reason='SDK process exited before protected release completed',
                process_evidence=dict(wait=dict(status='returned'),returncode=-9,reaped_monotonic_ns=1,terminations=[]),
                guard_client_failure=dict(schema='guard_ipc_failure.v1',source_pid=124,self_sigkill_requested=True,
                    started_monotonic_ns=1,failed_monotonic_ns=100_000_001,action='heartbeat',stage='receive',exception='TimeoutError'))
            backend.guard_record.write_text(json.dumps(receipt));value=backend.protection_snapshot()
            self.assertIn('Guard IPC heartbeat receive',value['reason']);self.assertIn('100.0ms',value['reason'])
            self.assertFalse(value['active_anomaly']);self.assertTrue(value['recovery_required'])
            self.assertEqual(value['heartbeat']['status'],'not_monitored');self.assertFalse(value['event']['sdk_close_confirmed'])
            receipt['guard_client_failure'].pop('started_monotonic_ns');backend.guard_record.write_text(json.dumps(receipt))
            self.assertTrue(backend.protection_snapshot()['recovery_required'])

    def test_explicit_normal_close_is_not_a_heartbeat_fault(self):
        backend=self.fixture();backend.connect();backend.close()
        self.assertFalse(backend.is_connected)
        self.assertFalse(backend.protection_snapshot()['active_anomaly'])
        self.assertEqual(backend.protection_snapshot()['heartbeat']['status'],'not_monitored')

    def test_supervisor_exit_alone_never_confirms_sdk_worker_exit(self):
        backend=self.fixture();backend.connect();backend.close()
        record=json.loads(backend.guard_record.read_text())
        self.assertTrue(backend.protection_snapshot()['worker_exit_confirmed'])
        for invalid in ({**record,'supervisor_pid':0},
                        {**record,'process_evidence':{'wait':{'status':'timeout'}}},{}):
            backend.guard_record.write_text(json.dumps(invalid))
            self.assertFalse(backend.protection_snapshot()['worker_exit_confirmed'])

    def test_abnormal_supervisor_exit_does_not_qualify_normal_close(self):
        backend=self.fixture(supervisor_exit=3);backend.connect()
        with self.assertRaisesRegex(RuntimeError,'supervisor failed'):backend.close()
        value=backend.protection_snapshot()
        self.assertFalse(value['active_anomaly']);self.assertTrue(value['recovery_required'])
        self.assertFalse(value['event']['sdk_close_confirmed'])

    def test_expired_queued_request_cannot_call_native_backend(self):
        backend=self.fixture();backend.connection.settimeout(2)
        send(backend.connection,dict(id=1,method='send',args=['left',[0.]*14],kwargs={},deadline=time.monotonic()-.01,generation=backend.generation))
        with self.assertRaises((ConnectionError,ConnectionResetError)):receive(backend.connection,b'')
        backend.process.wait(timeout=2)
        self.assertNotEqual(backend.process.returncode,0)

    def test_command_authorization_precedes_worker_rpc(self):
        backend=self.fixture()
        def reject():raise PermissionError('epoch revoked')
        backend.command_guard=reject
        with self.assertRaises(PermissionError):backend.send('left',[0.]*14)
        self.assertEqual(backend.sequence,0)

    def test_stop_revokes_an_already_authorized_in_flight_command(self):
        backend=self.fixture();backend.connect()
        backend.command_guard=backend.revoke_output
        with self.assertRaisesRegex(RuntimeError,'epoch revoked'):backend.send('left',[0.]*14)

    def test_persistent_warning_live_recovery_and_ack_are_independent(self):
        saved=[];alerts=ControlAlerts({},lambda value:saved.append(json.loads(json.dumps(value))))
        event=dict(id='first',reason='SDK owner heartbeat lost',wall_ns=1)
        bad=dict(event=event,active_anomaly=True,heartbeat=dict(status='lost'))
        alerts.observe(bad)
        restored=ControlAlerts(saved[-1],lambda value:saved.append(value))
        healthy=dict(active_anomaly=False,heartbeat=dict(status='healthy'))
        self.assertEqual(len(restored.snapshot(healthy,reachable=True)['warnings']),1)
        restored.observe(dict(event={**event,'id':'second'},active_anomaly=True))
        result=restored.acknowledge('first')
        self.assertFalse(result['motion_resumed']);self.assertFalse(result['control_recovered'])
        value=restored.snapshot(bad,reachable=True)
        self.assertEqual([row['id'] for row in value['warnings']],['second']);self.assertTrue(value['current']['active_anomaly'])
        restored.acknowledge('second')
        value=restored.snapshot(healthy,reachable=True)
        self.assertEqual(value['warnings'],[]);self.assertFalse(value['current']['active_anomaly'])
        self.assertIsNone(restored.snapshot(healthy,reachable=False)['current']['active_anomaly'])


if __name__=='__main__':unittest.main()
