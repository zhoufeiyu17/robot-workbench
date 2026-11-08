"""CPU-only service identity checks; Linux lifecycle tests skip on non-/proc hosts."""
from contextlib import ExitStack
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.managed_services import ManagedService, ServiceSpec, process_identity


LINUX = Path('/proc/self/stat').exists()
MODULE = 'robot_workbench.managed_services.'


class ManagedServiceTests(unittest.TestCase):
    def setUp(self):
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.script = self.root/'worker.py'
        self.script.write_text('import time\ntime.sleep(30)\n')
        self.spec = ServiceSpec('cpu-worker', 'process', str(self.script), str(self.root/'identity.json'),
            command=[sys.executable, '-u', str(self.script)], cwd=str(self.root),
            start_timeout_s=.3, stop_timeout_s=.08, poll_s=.005)
        self.service = ManagedService(self.spec)

    def identity(self, **changes):
        # Synthetic identities are only mocked test inputs, never a host fallback.
        return dict(dict(pid=12345, start_ticks='100', pgid=12345, sid=12345, uid=os.getuid(),
                         boot_id='test-boot-fixture', process_state='S', script=str(self.script)), **changes)

    def mock_identity(self, current, members=None):
        self.stack.enter_context(patch(MODULE+'_boot_id', return_value='test-boot-fixture'))
        self.stack.enter_context(patch(MODULE+'process_identity', return_value=current))
        self.stack.enter_context(patch.object(self.service, '_members', return_value=members or []))
        self.stack.enter_context(patch.object(self.service, '_descendants', return_value=[]))

    def unit(self):
        self.spec = replace(self.spec, kind='user_unit', command=None, cwd=None, unit='workbench-cpu.service')
        self.service = ManagedService(self.spec)
        self.stack.enter_context(patch.object(self.service,'_unit_members',return_value=[]))
        return dict(MainPID=12345, ControlPID=0, ActiveState='active', SubState='running', LoadState='loaded',
                    SendSIGKILL='no', KillSignal='15')

    def test_validation_rejects_shell_inline_code_and_mixed_unit_schema(self):
        for changes in ({'command':'python worker.py'}, {'command':['/bin/sh',str(self.script)]},
                        {'command':[sys.executable,'-c','pass',str(self.script)]},
                        {'command':[sys.executable,'-m','example',str(self.script)]},
                        {'command':['python3',str(self.script)]}, {'cwd':'relative'},
                        {'unit':'foreign.service'}, {'kind':'shell'}, {'stop_timeout_s':float('inf')}):
            with self.assertRaises(ValueError):
                replace(self.spec, **changes)
        with self.assertRaises(TypeError):
            ServiceSpec(**dict(self.spec.__dict__, shell=True))
        for unit in ('--all', '../foreign.service', 'name;echo.service'):
            with self.assertRaises(ValueError):
                replace(self.spec, kind='user_unit', command=None, cwd=None, unit=unit)

    def test_live_matching_process_is_reused_without_spawning(self):
        identity = self.identity()
        self.service._save('running', identity)
        self.mock_identity(identity)
        with patch(MODULE+'subprocess.Popen') as spawn:
            result = self.service.start()
        self.assertEqual(result['state'], 'running')
        self.assertTrue(result['owned'])
        self.assertTrue(result['reused'])
        spawn.assert_not_called()

    def test_trusted_environment_is_passed_separately_and_never_saved_publicly(self):
        from unittest.mock import Mock
        environment={'PYTHONPATH':'/frozen/isaac/path','CUDA_VISIBLE_DEVICES':'','PRIVATE_FIXTURE_VALUE':'not-for-receipts'}
        service=ManagedService(replace(self.spec,env=environment))
        environment['PYTHONPATH']='changed-after-validation'
        child=Mock(pid=12345,returncode=None)
        child.poll.return_value=None
        with patch.dict(os.environ,{'INHERITED':'yes','PYTHONPATH':'old'},clear=True), patch(MODULE+'_boot_id',return_value='test-boot-fixture'), patch(MODULE+'process_identity',return_value=self.identity()), patch.object(service,'_members',return_value=[]), patch(MODULE+'subprocess.Popen',return_value=child) as spawn:
            result=service.start()
        self.assertEqual(result['state'],'running')
        kwargs=spawn.call_args.kwargs
        self.assertEqual(kwargs['env']['INHERITED'],'yes')
        self.assertEqual(kwargs['env']['PYTHONPATH'],'/frozen/isaac/path')
        self.assertEqual(kwargs['env']['CUDA_VISIBLE_DEVICES'],'')
        self.assertIs(kwargs['shell'],False)
        self.assertEqual(spawn.call_args.args[0],tuple(self.spec.command))
        self.assertNotIn('not-for-receipts',json.dumps(result))
        self.assertNotIn('not-for-receipts',Path(self.spec.identity_path).read_text())
        for env in ({'INVALID=NAME':'x'},{'NAME':1},{'NAME':'bad\0value'},['NAME=value']):
            with self.assertRaises(ValueError):replace(self.spec,env=env)
        with self.assertRaises(ValueError):replace(self.spec,kind='user_unit',unit='x.service',command=None,cwd=None,env={})

    def test_proc_parser_reads_ticks_and_actual_script_without_inline_code_aliases(self):
        proc = self.root/'proc-fixture'
        (proc/'self').mkdir(parents=True)
        (proc/'self/stat').write_text('fixture only')
        (proc/'sys/kernel/random').mkdir(parents=True)
        (proc/'sys/kernel/random/boot_id').write_text('fixture-kernel-boot')
        process = proc/'12345'
        process.mkdir()
        fields = ['S','1','12345','12345']+['0']*15+['987654']
        (process/'stat').write_text('12345 (worker (fixture)) '+' '.join(fields))
        (process/'cmdline').write_bytes(b'/usr/bin/python3\0-u\0'+os.fsencode(self.script)+b'\0')
        with patch(MODULE+'PROC_ROOT', proc):
            identity = process_identity(12345)
            self.assertEqual(identity['start_ticks'], '987654')
            self.assertEqual(identity['script'], str(self.script))
            self.assertEqual(identity['boot_id'], 'fixture-kernel-boot')
            self.assertIsNone(process_identity(12346))
            (process/'cmdline').write_bytes(b'/usr/bin/python3\0-c\0pass\0'+os.fsencode(self.script)+b'\0')
            self.assertIsNone(process_identity(12345)['script'])

    def test_unit_pending_start_without_pid_stays_unknown_after_manager_restart(self):
        unit = self.unit()
        self.service._save('starting', None)
        self.mock_identity(None)
        unit.update(MainPID=0, ActiveState='inactive')
        with patch.object(self.service, '_unit_state', return_value=unit), patch.object(self.service, '_systemctl') as command:
            self.assertEqual(self.service.start()['state'], 'unknown')
            command.assert_not_called()

    def test_relative_python_script_uses_verified_process_cwd_and_missing_file_is_unknown(self):
        proc = self.root/'relative-proc'
        (proc/'self').mkdir(parents=True)
        (proc/'self/stat').write_text('fixture only')
        (proc/'sys/kernel/random').mkdir(parents=True)
        (proc/'sys/kernel/random/boot_id').write_text('fixture-boot')
        process = proc/'12345'
        process.mkdir()
        fields = ['S','1','12345','12345']+['0']*15+['987654']
        (process/'stat').write_text('12345 (fixture) '+' '.join(fields))
        (process/'cwd').symlink_to(self.root, target_is_directory=True)
        (process/'cmdline').write_bytes(b'/usr/bin/python3\0-B\0worker.py\0')
        with patch(MODULE+'PROC_ROOT', proc):
            identity = process_identity(12345)
            self.assertEqual(identity['script'], str(self.script.resolve(strict=True)))
            self.assertEqual(identity['start_ticks'], '987654')
            (process/'cmdline').write_bytes(b'/usr/bin/python3\0-c\0worker.py\0')
            self.assertIsNone(process_identity(12345)['script'])
            (process/'cmdline').write_bytes(b'/usr/bin/python3\0-B\0missing.py\0')
            with self.assertRaises(FileNotFoundError):process_identity(12345)
            (process/'cmdline').write_bytes(b'/usr/bin/python3\0-B\0.\0')
            with self.assertRaisesRegex(OSError, 'regular file'):process_identity(12345)

    def test_pid_reuse_and_wrong_script_are_unknown_and_never_signalled(self):
        recorded = self.identity()
        self.service._save('running', recorded)
        self.stack.enter_context(patch(MODULE+'_boot_id', return_value='test-boot-fixture'))
        for foreign in (self.identity(start_ticks='200'), self.identity(script='/different/script.py')):
            with patch(MODULE+'process_identity', return_value=foreign), patch(MODULE+'os.killpg') as kill, patch(MODULE+'subprocess.Popen') as spawn:
                self.assertEqual(self.service.stop()['state'], 'unknown')
                self.assertEqual(self.service.start()['state'], 'unknown')
                kill.assert_not_called()
                spawn.assert_not_called()
        self.assertEqual(json.loads(Path(self.spec.identity_path).read_text())['identity'], recorded)

    def test_identity_is_rechecked_immediately_before_signal(self):
        recorded = self.identity()
        self.service._save('running', recorded)
        self.stack.enter_context(patch(MODULE+'_boot_id', return_value='test-boot-fixture'))
        with patch.object(self.service, '_members', return_value=[]), patch.object(self.service,'_descendants',return_value=[]), patch(MODULE+'process_identity', side_effect=[recorded,self.identity(start_ticks='200')]), patch(MODULE+'os.killpg') as kill:
            result = self.service.stop()
        self.assertEqual(result['state'], 'unknown')
        kill.assert_not_called()

    def test_unknown_identity_and_surviving_group_block_start_and_stop(self):
        self.service._save('running', self.identity())
        self.mock_identity(None, [self.identity(pid=12346)])
        with patch(MODULE+'os.killpg') as kill, patch(MODULE+'subprocess.Popen') as spawn:
            self.assertEqual(self.service.start()['state'], 'unknown')
            self.assertEqual(self.service.stop()['state'], 'unknown')
            kill.assert_not_called()
            spawn.assert_not_called()
        with patch(MODULE+'process_identity', side_effect=PermissionError('denied')):
            self.assertEqual(self.service.snapshot()['state'], 'unknown')

    def test_crash_between_start_and_identity_capture_cannot_look_stopped(self):
        self.service._save('starting', None, 'pending start')
        self.mock_identity(None)
        with patch(MODULE+'subprocess.Popen') as spawn:
            self.assertEqual(self.service.snapshot()['state'], 'unknown')
            self.assertEqual(self.service.start()['state'], 'unknown')
            spawn.assert_not_called()

    def test_term_timeout_never_escalates_or_restarts(self):
        identity = self.identity()
        self.service._save('running', identity)
        self.mock_identity(identity)
        with patch(MODULE+'os.killpg') as kill, patch(MODULE+'subprocess.Popen') as spawn:
            result = self.service.restart()
        self.assertEqual(result['state'], 'unknown')
        self.assertTrue(result['running'])
        kill.assert_called_once_with(identity['pgid'], signal.SIGTERM)
        spawn.assert_not_called()
        receipt = json.loads(Path(self.spec.identity_path).read_text())
        self.assertEqual(receipt['phase'], 'unknown')
        self.assertEqual(receipt['identity'], identity)

    def test_changed_deployment_cannot_overwrite_live_receipt(self):
        self.service._save('running', self.identity())
        original = Path(self.spec.identity_path).read_bytes()
        different = ManagedService(replace(self.spec, command=[sys.executable,'-u',str(self.script),'other']))
        self.stack.enter_context(patch(MODULE+'_boot_id', return_value='test-boot-fixture'))
        self.assertEqual(different.start()['state'], 'unknown')
        self.assertEqual(Path(self.spec.identity_path).read_bytes(), original)

    def test_unit_dead_state_does_not_hide_a_live_historical_pid(self):
        unit = self.unit()
        self.service._save('running', self.identity())
        self.mock_identity(self.identity())
        unit.update(MainPID=0, ActiveState='inactive')
        with patch.object(self.service, '_unit_state', return_value=unit), patch.object(self.service, '_systemctl') as command:
            self.assertEqual(self.service.snapshot()['state'], 'unknown')
            self.assertEqual(self.service.start()['state'], 'unknown')
            self.assertEqual(self.service.stop()['state'], 'unknown')
            command.assert_not_called()

    def test_named_unit_mainpid_must_match_script_and_safe_term_policy(self):
        unit = self.unit()
        self.mock_identity(self.identity(script='/foreign/script.py'))
        with patch.object(self.service, '_unit_state', return_value=unit), patch.object(self.service, '_systemctl') as command:
            self.assertEqual(self.service.stop()['state'], 'unknown')
            command.assert_not_called()
        unit['SendSIGKILL'] = 'yes'
        with patch(MODULE+'process_identity', return_value=self.identity()), patch.object(self.service, '_unit_state', return_value=unit), patch.object(self.service, '_systemctl') as command:
            result = self.service.stop()
            self.assertEqual(result['state'], 'unknown')
            self.assertIn('SendSIGKILL=no', result['reason'])
            command.assert_not_called()

    def test_unit_stop_verifies_inactive_and_historical_exit(self):
        unit = self.unit()
        identity = self.identity()
        inactive = dict(unit, MainPID=0, ActiveState='inactive')
        self.stack.enter_context(patch(MODULE+'_boot_id', return_value='test-boot-fixture'))
        with patch.object(self.service, '_unit_state', side_effect=[unit,unit,inactive]), patch(MODULE+'process_identity', side_effect=[identity,identity,None]), patch.object(self.service, '_systemctl') as command:
            result = self.service.stop()
        self.assertEqual(result['state'], 'stopped')
        self.assertFalse(result['running'])
        command.assert_called_once_with('stop','--no-block','--','workbench-cpu.service')

    def test_systemctl_errors_and_timeouts_remain_unknown(self):
        self.unit()
        self.stack.enter_context(patch(MODULE+'_boot_id', return_value='test-boot-fixture'))
        with patch.object(self.service, '_systemctl', side_effect=subprocess.TimeoutExpired('systemctl',.1)):
            self.assertEqual(self.service.snapshot()['state'], 'unknown')
            self.assertEqual(self.service.stop()['state'], 'unknown')

    def test_unit_leader_exit_does_not_hide_a_tracked_child_or_allow_restart(self):
        unit=self.unit();leader=self.identity();child=self.identity(pid=12346,pgid=12346,sid=12346,script='/owned/child.py')
        self.service._save('running',leader,children=[child])
        self.stack.enter_context(patch(MODULE+'_boot_id',return_value='test-boot-fixture'))
        inactive=dict(unit,MainPID=0,ActiveState='inactive')
        with patch.object(self.service,'_unit_state',return_value=inactive), patch(MODULE+'process_identity',side_effect=lambda pid:child if pid==child['pid'] else None), patch.object(self.service,'_systemctl') as command:
            result=self.service.restart()
            self.assertEqual(result['state'],'unknown');self.assertTrue(result['running'])
            command.assert_not_called()

    def test_detached_process_child_must_exit_even_after_parent_and_group_vanish(self):
        leader=self.identity();child=self.identity(pid=12346,pgid=12346,sid=12346,script='/owned/worker.py')
        self.service._save('stopping',leader,children=[child])
        self.stack.enter_context(patch(MODULE+'_boot_id',return_value='test-boot-fixture'))
        with patch.object(self.service,'_members',return_value=[]), patch(MODULE+'process_identity',side_effect=lambda pid:child if pid==child['pid'] else None), patch(MODULE+'os.killpg') as kill, patch(MODULE+'subprocess.Popen') as spawn:
            self.assertEqual(self.service.snapshot()['state'],'unknown')
            self.assertEqual(self.service.restart()['state'],'unknown')
            kill.assert_not_called();spawn.assert_not_called()

    def test_reported_child_requires_proven_ownership_and_retains_start_ticks(self):
        leader=self.identity();child=self.identity(pid=12346,pgid=12346,sid=12346,script='/owned/worker.py')
        self.service._save('running',leader);self.mock_identity(leader)
        with patch.object(self.service,'_descendants',return_value=[child]):
            self.assertEqual(self.service.track_child(child['pid'])['tracked_child']['start_ticks'],'100')
        saved=json.loads(Path(self.spec.identity_path).read_text());self.assertEqual(saved['children'],[child])
        with patch.object(self.service,'_descendants',return_value=[]), patch(MODULE+'process_identity',side_effect=lambda pid:leader if pid==leader['pid'] else child):
            self.assertEqual(self.service.track_child(98765)['state'],'unknown')
        self.assertEqual(json.loads(Path(self.spec.identity_path).read_text())['children'],[child])

    def test_mutation_lock_does_not_wait_or_signal_a_competing_manager(self):
        self.mock_identity(self.identity())
        self.service._save('running', self.identity())
        with self.service._locked(), patch(MODULE+'os.killpg') as kill:
            other = ManagedService(self.spec)
            self.assertEqual(other.stop()['state'], 'unknown')
            kill.assert_not_called()

    @unittest.skipIf(LINUX, 'non-Linux failure-closed behavior is exercised on macOS')
    def test_non_linux_never_fabricates_ticks_or_stops_an_actual_cpu_process(self):
        child = subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'])
        try:
            with patch(MODULE+'os.killpg') as kill:
                result = self.service.start()
                self.assertEqual(result['state'], 'unknown')
                self.assertIsNone(result['identity'])
                self.assertEqual(self.service.stop()['state'], 'unknown')
                kill.assert_not_called()
            self.assertIsNone(child.poll())
            with self.assertRaises(OSError):
                process_identity(child.pid)
        finally:
            child.terminate()
            child.wait(timeout=5)

    @unittest.skipUnless(LINUX, 'real PID/start_ticks lifecycle requires Linux /proc; no synthetic macOS identity')
    def test_real_cpu_process_start_reuse_term_and_unrelated_process_survives(self):
        unrelated = subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'])
        service = ManagedService(replace(self.spec, start_timeout_s=2., stop_timeout_s=2.))
        try:
            started = service.start()
            self.assertEqual(started['state'], 'running', started)
            self.assertTrue(started['identity']['start_ticks'].isdigit())
            reused = service.start()['identity']
            self.assertEqual((reused['pid'],reused['start_ticks']),
                             (started['identity']['pid'],started['identity']['start_ticks']))
            self.assertTrue(service.start()['reused'])
            stopped = service.stop()
            self.assertEqual(stopped['state'], 'stopped', stopped)
            self.assertFalse(stopped['running'])
            self.assertIsNone(unrelated.poll())
        finally:
            if service.child is not None and service.child.poll() is None:
                service.child.terminate()
                service.child.wait(timeout=5)
            unrelated.terminate()
            unrelated.wait(timeout=5)

    @unittest.skipUnless(LINUX, 'real TERM-resistant lifecycle requires Linux /proc')
    def test_real_term_resistant_process_remains_alive_after_timeout(self):
        ready = self.root/'ready'
        self.script.write_text('import signal,time\nfrom pathlib import Path\nsignal.signal(signal.SIGTERM,signal.SIG_IGN)\nPath('+repr(str(ready))+').touch()\ntime.sleep(30)\n')
        service = ManagedService(replace(self.spec, start_timeout_s=2.))
        try:
            self.assertEqual(service.start()['state'], 'running')
            deadline = time.monotonic()+2
            while not ready.exists() and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertTrue(ready.exists())
            self.assertEqual(service.restart()['state'], 'unknown')
            self.assertIsNone(service.child.poll())
        finally:
            # Test-owned fixture cleanup only; ManagedService never escalates.
            if service.child is not None and service.child.poll() is None:
                service.child.kill()
                service.child.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
