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
from robot_workbench.sdk_process import SDKProcess,SDKCommandRevoked,send,receive
from robot_workbench.control_alerts import ControlAlerts

WORKER='''import os,socket,sys,time,threading,signal
sys.path.insert(0,sys.argv[1])
from robot_workbench.sdk_process import serve_worker
class Backend:
 is_connected=False
 def connect(self):self.is_connected=True
 def read_calibration_state(self):
  threading.Timer(.15,lambda:os.kill(os.getpid(),signal.SIGKILL)).start()
  return dict(qpos=[0.]*14,future_queued=getattr(self,'future',False),can_output_guard=dict(heartbeat_monotonic_s=time.monotonic()))
 def close(self):self.is_connected=False
 def read_gains(self):return dict(future_queued=getattr(self,'future',False))
 def send(self,*args):self.command_guard();return None
 def queue_reference(self,value):
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


class SDKProcessTests(unittest.TestCase):
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

    def fixture(self, supervisor_exit=None):
        backend=SDKProcess({'storage_root':'/unused','disabled_grippers':['right']})
        backend.connection,peer=socket.socketpair()
        backend.attempt='cpu-owned-attempt'
        import tempfile
        directory=tempfile.TemporaryDirectory();self.addCleanup(directory.cleanup)
        backend.guard_record=Path(directory.name)/'guard.json'
        backend.epoch_file=tempfile.TemporaryFile();backend.epoch_file.truncate(32)
        backend.epoch=mmap.mmap(backend.epoch_file.fileno(),32);backend.epoch[:]=backend.generation.encode()
        command=[sys.executable,'-B','-c',SUPERVISOR,WORKER,str(ROOT),str(peer.fileno()),str(backend.epoch_file.fileno()),str(backend.guard_record)]
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

    def test_live_heartbeat_timeout_is_still_an_active_anomaly(self):
        from types import SimpleNamespace
        backend=SDKProcess({})
        backend.process=SimpleNamespace(pid=123,poll=lambda:None,returncode=None)
        backend.connected=True;backend.heartbeat=time.monotonic()-.101
        value=backend.protection_snapshot()
        self.assertEqual(value['heartbeat']['status'],'lost')
        self.assertTrue(value['heartbeat']['monitored']);self.assertTrue(value['active_anomaly'])

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
