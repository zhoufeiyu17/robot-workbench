"""CPU launcher ownership/exit checks; no deployment, SDK, GPU or model imports."""
from contextlib import ExitStack
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
SCRIPT=ROOT/'scripts/launch_workbench_backend.py'
spec=importlib.util.spec_from_file_location('backend_launcher_cpu_test',SCRIPT)
launcher=importlib.util.module_from_spec(spec);spec.loader.exec_module(launcher)


class Child:
    pid=12345
    def __init__(self,code=0,stubborn=False):
        self.returncode=None;self.code=code;self.stubborn=stubborn
        self.signals=[];self.waits=[];self.terminations=0
    def poll(self):return self.returncode
    def send_signal(self,value):self.signals.append(value);self.returncode=-value
    def terminate(self):
        self.terminations+=1
        if not self.stubborn:self.returncode=-signal.SIGTERM
    def wait(self,timeout=None):
        self.waits.append(timeout)
        if timeout is not None and self.stubborn:raise subprocess.TimeoutExpired('CPU fixture',timeout)
        if self.returncode is None:self.returncode=self.code
        return self.returncode
    def kill(self):raise AssertionError('launcher must not KILL')


class BackendLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.path=self.root/'spec.json';self.output=self.root/'attempts'
        self.value=dict(schema='robot_workbench.backend_launch.v1',output_root=str(self.output),
            argv=[sys.executable,'/cpu-fixture.py','{output}'],cwd=str(self.root),env={'PRIVATE_FIXTURE':'not-in-receipt'})
        self.path.write_text(json.dumps(self.value));self.child=Child();self.handlers={}
    def run_mock(self,*,spawn=None,read_stat=None):
        original_read=Path.read_text;original_file=Path.is_file
        def read(path,*args,**kwargs):
            if str(path).startswith('/proc/'):
                if read_stat is not None:return read_stat()
                return '12345 (CPU fixture) '+' '.join(['S']+['0']*18+['42'])
            return original_read(path,*args,**kwargs)
        with ExitStack() as stack:
            stack.enter_context(patch.object(sys,'argv',['launcher','--spec',str(self.path)]))
            stack.enter_context(patch.object(sys,'platform','linux'))
            stack.enter_context(patch.object(Path,'read_text',new=read))
            stack.enter_context(patch.object(Path,'is_file',new=lambda path:True if str(path)=='/proc/self/stat' else original_file(path)))
            stack.enter_context(patch.object(launcher.signal,'signal',side_effect=lambda sig,handler:self.handlers.__setitem__(sig,handler)))
            spawn_mock=stack.enter_context(patch.object(launcher.subprocess,'Popen',side_effect=spawn,return_value=self.child))
            result=launcher.main()
        return result,spawn_mock
    def receipt(self):
        return json.loads(next(self.output.glob('attempt-*-exit.json')).read_text())

    def test_normal_exit_preserves_output_identity_and_trusted_environment(self):
        self.child.code=7;result,spawn=self.run_mock()
        self.assertEqual(result,7);self.assertEqual(self.receipt()['exit_code'],7)
        pointer=json.loads((self.output/'current.json').read_text())
        self.assertEqual(pointer['start_ticks'],'42');self.assertNotIn('{output}',pointer['argv'])
        self.assertTrue(Path(pointer['log']).exists());self.assertEqual(self.child.terminations,0)
        self.assertEqual(spawn.call_args.kwargs['env']['PRIVATE_FIXTURE'],'not-in-receipt')
        self.assertNotIn('not-in-receipt',json.dumps(pointer));self.assertFalse(list(self.output.glob('*-current.tmp')))

    def test_platform_and_invalid_specs_fail_before_spawning(self):
        with patch.object(sys,'argv',['launcher','--spec',str(self.path)]),patch.object(sys,'platform','darwin'),\
                patch.object(launcher.subprocess,'Popen') as spawn:
            with self.assertRaisesRegex(RuntimeError,'Linux'):launcher.main()
            spawn.assert_not_called();self.assertFalse(self.output.exists())
        for changes in ({'argv':[]},{'env':{'NAME':3}},{'env':{'BAD=NAME':'x'}},{'cwd':'relative'}):
            self.path.write_text(json.dumps(dict(self.value,**changes)))
            with self.subTest(changes=changes),self.assertRaises(ValueError):self.run_mock()
        self.assertFalse(self.output.exists())

    def test_stop_during_spawn_forwards_signal_and_normalizes_exit_code(self):
        def spawn(*args,**kwargs):self.handlers[signal.SIGTERM](signal.SIGTERM,None);return self.child
        result,_=self.run_mock(spawn=spawn)
        self.assertEqual(result,143);self.assertEqual(self.child.signals,[signal.SIGTERM])
        self.assertEqual(self.receipt()['exit_code'],-signal.SIGTERM)

    def test_early_child_exit_before_proc_identity_still_has_exit_receipt(self):
        self.child.returncode=7
        def vanished():raise FileNotFoundError('child already reaped')
        result,_=self.run_mock(read_stat=vanished)
        self.assertEqual(result,7);self.assertEqual(self.receipt()['identity_status'],'exited_before_identity')
        self.assertEqual(self.child.terminations,0)

    def test_ignored_stop_signal_has_bounded_wait_and_unknown_receipt(self):
        self.child.stubborn=True;self.child.send_signal=lambda value:self.child.signals.append(value)
        def spawn(*args,**kwargs):self.handlers[signal.SIGTERM](signal.SIGTERM,None);return self.child
        with self.assertRaises(subprocess.TimeoutExpired):self.run_mock(spawn=spawn)
        self.assertEqual(self.child.waits,[5,5]);self.assertEqual(self.child.terminations,1)
        self.assertEqual(self.receipt()['status'],'unknown');self.assertIsNone(self.receipt()['exit_code'])

    def test_proc_failure_terminates_only_owned_child_and_preserves_failure(self):
        def inaccessible():raise PermissionError('CPU fixture proc denial')
        with self.assertRaisesRegex(PermissionError,'proc denial'):self.run_mock(read_stat=inaccessible)
        self.assertEqual(self.child.terminations,1);self.assertEqual(self.child.waits,[5])
        self.assertEqual(self.receipt()['status'],'exited');self.assertIn('proc denial',self.receipt()['error'])

    def test_pointer_failure_has_same_cleanup_and_stubborn_child_remains_unknown(self):
        self.child.stubborn=True
        with patch.object(Path,'replace',side_effect=OSError('CPU fixture pointer write failed')):
            with self.assertRaisesRegex(OSError,'pointer write failed'):self.run_mock()
        self.assertEqual(self.child.terminations,1);self.assertEqual(self.child.waits,[5])
        receipt=self.receipt();self.assertEqual(receipt['status'],'unknown');self.assertIsNone(receipt['exit_code'])
        self.assertIn('cleanup_error',receipt);self.assertFalse(list(self.output.glob('*-current.tmp')))

    def test_exit_receipt_failure_never_restarts_or_signals_an_exited_child(self):
        original=Path.write_text
        def write(path,*args,**kwargs):
            if path.name.endswith('-exit.json'):raise OSError('CPU fixture receipt unavailable')
            return original(path,*args,**kwargs)
        with patch.object(Path,'write_text',new=write):
            with self.assertRaisesRegex(OSError,'receipt unavailable'):self.run_mock()
        self.assertEqual(self.child.terminations,0);self.assertEqual(self.child.returncode,0)

    @unittest.skipUnless(sys.platform=='linux' and Path('/proc/self/stat').is_file(),'real lifecycle requires Linux /proc')
    def test_real_cpu_child_signal_and_exit_propagation(self):
        worker=self.root/'worker.py'
        worker.write_text('import sys,time\nfrom pathlib import Path\np=Path(sys.argv[1]);p.mkdir()\n(p/"ready").write_text("ready")\ntime.sleep(30)\n')
        self.path.write_text(json.dumps(dict(self.value,argv=[sys.executable,str(worker),'{output}'])))
        child=subprocess.Popen([sys.executable,'-B',str(SCRIPT),'--spec',str(self.path)],
            stdout=subprocess.PIPE,stderr=subprocess.PIPE,start_new_session=True)
        try:
            deadline=time.monotonic()+3
            while not list(self.output.glob('attempt-*/ready')) and time.monotonic()<deadline:
                if child.poll() is not None:self.fail(child.communicate()[1].decode())
                time.sleep(.01)
            self.assertTrue(list(self.output.glob('attempt-*/ready')))
            child.send_signal(signal.SIGTERM);stdout,stderr=child.communicate(timeout=7)
            self.assertEqual(child.returncode,143,stderr.decode());self.assertEqual(self.receipt()['exit_code'],-15)
        finally:
            if child.poll() is None:
                # CPU fixture's own session only; never any external deployment.
                os.killpg(child.pid,signal.SIGTERM);child.communicate(timeout=7)


if __name__=='__main__':unittest.main()
