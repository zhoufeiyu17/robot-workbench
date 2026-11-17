"""PID-bound supported retirement using inert Linux subprocesses only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from tempfile import TemporaryDirectory
import time
import unittest

from robot_workbench.sdk_retirement import retire_supported, retained_processes


SUPERVISOR = '''import json,os,signal,subprocess,sys,time
from pathlib import Path
if '--child' in sys.argv:
 signal.signal(signal.SIGTERM,signal.SIG_IGN)
 while True:time.sleep(1)
path=Path(sys.argv[1])
child=subprocess.Popen([sys.executable,__file__,'--child'])
stamp=time.monotonic_ns()
value=dict(schema='arx_can_output_guard.v1',state='latched',supervisor_pid=os.getpid(),
 child_pid=child.pid,reason='synthetic lost command channel',native_fault_hold=dict(
 schema='workbench.native-fault-hold.v1',armed=True,fault_code=1,
 committed_monotonic_ns=dict(left=stamp,right=stamp)))
path.write_text(json.dumps(value))
result=child.wait()
value['process_evidence']=dict(wait=dict(status='returned'),returncode=result,
 reaped_monotonic_ns=time.monotonic_ns())
temporary=path.with_suffix('.next');temporary.write_text(json.dumps(value));temporary.replace(path)
'''


@unittest.skipUnless(sys.platform == 'linux', 'Linux pidfd required')
class SDKRetirementTests(unittest.TestCase):
    def test_support_identity_hold_and_final_reap_are_required(self):
        with TemporaryDirectory() as temporary:
            root = Path(temporary)
            script, record = root / 'worker.py', root / 'guard.json'
            script.write_text(SUPERVISOR)
            process = subprocess.Popen([sys.executable, str(script), str(record)])
            try:
                deadline = time.monotonic() + 3
                while not record.exists():
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(.01)
                options = dict(supervisor_pid=process.pid, owner_pid=os.getpid(), script=script)
                with self.assertRaisesRegex(ValueError, 'supported-arms'):
                    retire_supported(record, arms_supported=False, **options)
                with self.assertRaisesRegex(ValueError, 'identity'):
                    retained_processes(record, **dict(options, owner_pid=os.getpid() + 1))
                original = json.loads(record.read_text())
                invalid = dict(original, native_fault_hold=dict(original['native_fault_hold'], armed=False))
                record.write_text(json.dumps(invalid))
                with self.assertRaisesRegex(ValueError, 'confirmed native hold'):
                    retire_supported(record, arms_supported=True, **options)
                record.write_text(json.dumps(original))
                self.assertIsNone(process.poll())
                result = retire_supported(record, arms_supported=True, **options)
                process.wait(timeout=2)
                self.assertTrue(result['worker_exit_confirmed'])
                self.assertFalse(result['sdk_close_confirmed'])
                self.assertEqual(result['worker']['ppid'], process.pid)
                self.assertEqual(result['process_evidence']['returncode'], -signal.SIGKILL)
            finally:
                if process.poll() is None:
                    if record.exists():
                        child = json.loads(record.read_text())['child_pid']
                        try:
                            os.kill(child, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    else:
                        process.kill()
                    process.wait(timeout=3)


if __name__ == '__main__':
    unittest.main()
