"""Real CPU groups and UI failure propagation; no GPU, SSH, Kit or robot."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch

from owned_process import read_failure_result,wait_owned
from cancellable_process import run
from playground import Playground
from playground_pipeline import run_stages
from test_playground_interrupt import Bridge
from test_owned_process import live

WB=Path(__file__).resolve().parent
WORKER="import os,signal,sys,time;from pathlib import Path;signal.signal(signal.SIGTERM,signal.SIG_IGN);Path(sys.argv[1]).write_text(str(os.getpid()));time.sleep(60)"
PARENT="""import json,subprocess,sys,time
from pathlib import Path
subprocess.Popen([sys.executable,'-c',sys.argv[1],sys.argv[2]])
while not Path(sys.argv[2]).exists():time.sleep(.01)
Path(sys.argv[3]).write_text(json.dumps({'status':'failed','error':'original native setup failure'}))
time.sleep(60)
"""


class NativeFailureTest(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);self.result=self.root/'result.json';self.pid=self.root/'worker.pid'

    def child(self,script,*args):
        value=subprocess.Popen([sys.executable,'-B','-c',script,*map(str,args)],start_new_session=True)
        def cleanup():
            try:os.killpg(value.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            value.wait(timeout=5)
        self.addCleanup(cleanup)
        return value

    def wait_ready(self,path):
        deadline=time.monotonic()+5
        while not path.exists() and time.monotonic()<deadline:time.sleep(.01)
        self.assertTrue(path.exists())

    def test_failed_receipt_ends_hung_child_and_term_resistant_descendant_only(self):
        unrelated=self.child('import time;time.sleep(60)')
        child=self.child(PARENT,WORKER,self.pid,self.result)
        started=time.monotonic()
        self.assertEqual(wait_owned(child,10,grace=.2,failure_result=self.result),1)
        self.assertLess(time.monotonic()-started,3)
        self.assertFalse(live(int(self.pid.read_text())))
        self.assertIsNone(unrelated.poll())

    def test_success_receipt_waits_for_exit_and_missing_partial_do_not_end_early(self):
        for payload in ('{"status":"completed"}','{"status":',None):
            self.result.unlink(missing_ok=True)
            script="from pathlib import Path;import sys,time;\nif sys.argv[2]!='missing':Path(sys.argv[1]).write_text(sys.argv[2])\ntime.sleep(.35)"
            child=self.child(script,self.result,payload or 'missing');started=time.monotonic()
            self.assertEqual(wait_owned(child,3,grace=.1,failure_result=self.result),0)
            self.assertGreater(time.monotonic()-started,.3)
        self.result.unlink(missing_ok=True)
        script="from pathlib import Path;import sys,time;p=Path(sys.argv[1]);p.write_text('{\"status\":');time.sleep(.25);p.write_text('{\"status\":\"failed\"}');time.sleep(60)"
        child=self.child(script,self.result);started=time.monotonic()
        self.assertEqual(wait_owned(child,3,grace=.1,failure_result=self.result),1)
        self.assertGreater(time.monotonic()-started,.2)

    def test_bounded_regular_receipt_and_stale_cli_rejection(self):
        self.assertIsNone(read_failure_result(self.result))
        for payload in ('null','[]','{"status":"running"}','{"status":"failed"' ,' '* (4*1024*1024+1)):
            self.result.write_text(payload);self.assertIsNone(read_failure_result(self.result))
        self.result.write_text('{"status":"failed"}')
        link=self.root/'link';link.symlink_to(self.result)
        self.assertIsNone(read_failure_result(link))
        fifo=self.root/'fifo';os.mkfifo(fifo);self.assertIsNone(read_failure_result(fifo))
        marker=self.root/'should-not-start'
        result=subprocess.run([sys.executable,str(WB/'owned_process.py'),'--timeout','2',
            '--failure-result',str(self.result),'--',sys.executable,'-c',f'open({str(marker)!r},"w").close()'],capture_output=True)
        self.assertEqual(result.returncode,2);self.assertFalse(marker.exists())

    def test_success_receipt_cannot_hide_nonzero_process_exit(self):
        stage=dict(name='playground-geometry-inputs',result_path='/current/result.json')
        def fetch(source,destination,**kwargs):Path(destination).write_text('{"status":"geometry-inputs-ready"}')
        with patch('playground_pipeline.bounded_command',return_value=(['fixture'],1)), \
             patch('cancellable_process.run',return_value=subprocess.CompletedProcess(['fixture'],7)) as command, \
             patch('playground_pipeline.fetch',fetch):
            with self.assertRaisesRegex(RuntimeError,r'failed \(7\)'):
                run_stages([stage,dict(name='must-not-run')],self.root,{'resources':{'max_wall_time_minutes':1}})
        self.assertEqual(command.call_count,1)
        receipt=json.loads((self.root/'timings.json').read_text())[0]
        self.assertEqual(receipt['status'],'failed');self.assertEqual(receipt['native_status'],'geometry-inputs-ready')
        self.assertEqual(receipt['exit_code'],7)

    def test_cancel_allows_real_supervisor_twenty_second_group_cleanup(self):
        unrelated=self.child('import time;time.sleep(60)')
        command=[sys.executable,str(WB/'owned_process.py'),'--timeout','60','--',sys.executable,'-c',PARENT,
                 WORKER,str(self.pid),str(self.result)]
        started=time.monotonic()
        try:
            with self.assertRaises(InterruptedError):
                run(command,cancel=self.result.exists,timeout=40,terminate_grace_s=25)
            self.assertGreater(time.monotonic()-started,19)
            self.assertLess(time.monotonic()-started,28)
            self.assertFalse(live(int(self.pid.read_text())))
            self.assertIsNone(unrelated.poll())
        finally:
            if self.pid.exists() and live(int(self.pid.read_text())):os.kill(int(self.pid.read_text()),signal.SIGKILL)

    def test_original_error_visible_while_busy_then_failure_timings_and_no_next_stage(self):
        supervisor="""import sys,subprocess,signal
sys.path.insert(0,sys.argv[1])
from owned_process import wait_owned
def stop(signum,frame):raise SystemExit(128+signum)
signal.signal(signal.SIGTERM,stop)
child=subprocess.Popen(sys.argv[3:],start_new_session=True)
sys.exit(wait_owned(child,10,grace=1,failure_result=sys.argv[2]))
"""
        argv=[sys.executable,'-B','-c',supervisor,str(WB),str(self.result),sys.executable,'-c',PARENT,WORKER,str(self.pid),str(self.result)]
        stage=dict(name='playground-geometry-inputs',result_path=str(self.result),failure_result=str(self.result),terminate_grace_s=25)
        service=Playground(self.root/'ui',Bridge(),lambda:{'status':'ready'})
        service.state['environment']={'ready':True,'checks':[dict(key='dsw',status='ready')]}
        service.state['warmup']={'status':'ready','message':'CPU fixture: model initialization already completed'}
        def plan(config,output,bridge,speed,progress=None,cancel=None):
            return run_stages([stage,dict(name='must-not-run')],output,{'resources':{'max_wall_time_minutes':1}},progress,cancel)
        def fetch(source,destination,**kwargs):Path(destination).write_bytes(Path(source).read_bytes())
        with patch('playground_pipeline.plan',plan),patch('playground_pipeline.bounded_command',return_value=(argv,10)) as bounded, \
             patch('playground_pipeline.fetch',fetch):
            service.start({'action':'plan'})
            deadline=time.monotonic()+5
            while '正在清理本轮计算进程' not in service.state.get('message','') and time.monotonic()<deadline:time.sleep(.01)
            self.assertIn('original native setup failure',service.state['message'])
            self.assertTrue(service.busy);self.assertFalse(service.snapshot()['can_execute'])
            service.thread.join(5)
            self.assertFalse(service.thread.is_alive());self.assertFalse(service.busy)
            self.assertEqual(service.state['status'],'failed')
            self.assertIn('original native setup failure',service.state['error'])
            self.assertEqual(bounded.call_count,1)
        timings=json.loads(next((self.root/'ui/jobs').glob('*/timings.json')).read_text())
        self.assertEqual(len(timings),1);self.assertEqual(timings[0]['status'],'failed')
        self.assertEqual(timings[0]['native_error'],'original native setup failure')
        self.assertEqual(timings[0]['native_status'],'failed');self.assertEqual(timings[0]['exit_code'],1)
        self.assertFalse(live(int(self.pid.read_text())))


if __name__=='__main__':unittest.main()
