"""Real subprocess regression; no Isaac, GPU, network, or hardware."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from unittest.mock import Mock, patch

from owned_process import wait_owned


def live(pid):
    result = subprocess.run(['ps', '-o', 'stat=', '-p', str(pid)], capture_output=True, text=True)
    state = result.stdout.strip()
    return bool(state) and not state.startswith('Z')


def test():
    for membership, denied in [('', False), ('123 S\n', True)]:
        child=Mock(pid=123);child.wait.return_value=7
        with patch('owned_process.os.killpg',side_effect=PermissionError), \
                patch('owned_process.subprocess.check_output',return_value=membership):
            try:assert wait_owned(child,.2,.2)==7
            except PermissionError:assert denied, 'vanished group must not mask the original result'
            else:assert not denied, 'live group permission failure must remain an error'
    worker = "import os,signal,sys,time; from pathlib import Path; signal.signal(signal.SIGTERM,signal.SIG_IGN); Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(60)"
    parent = "import subprocess,sys,time; from pathlib import Path; subprocess.Popen([sys.executable,'-c',sys.argv[1],sys.argv[2]]);\nwhile not Path(sys.argv[2]).exists(): time.sleep(.01)\nif sys.argv[3]=='exit': sys.exit(7)\ntime.sleep(60)"
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
    try:
        with tempfile.TemporaryDirectory() as directory:
            for mode, expected in [('timeout', 124), ('exit', 7)]:
                path = Path(directory)/mode
                child = subprocess.Popen([sys.executable, '-c', parent, worker, str(path), mode], start_new_session=True)
                deadline = time.monotonic()+5
                while not path.exists() and time.monotonic()<deadline:
                    time.sleep(.01)
                if not path.exists():
                    wait_owned(child, .1, .1)
                    raise AssertionError('worker failed to become ready')
                pid = int(path.read_text())
                try:
                    # Worker readiness does not mean its parent has been scheduled
                    # to exit; exercise cleanup of an actually exited parent.
                    if mode=='exit':assert child.wait(timeout=5)==expected
                finally:
                    actual=wait_owned(child,.2,.2)
                assert actual==expected
                deadline = time.monotonic()+2
                while live(pid) and time.monotonic()<deadline:
                    time.sleep(.02)
                assert not live(pid), (mode, 'orphan still running', pid)
                assert unrelated.poll() is None, 'unrelated process was touched'
    finally:
        unrelated.terminate(); unrelated.wait(timeout=5)
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'cli-child'
        command = "import os,time,sys; from pathlib import Path; Path(sys.argv[1]).write_text(str(os.getpid())); time.sleep(60)"
        from launch_environment import owned_command
        supervisor = subprocess.Popen(owned_command(Path(__file__).parent,
            [sys.executable,'-c',command,str(path)],None))
        try:
            deadline = time.monotonic()+5
            while not path.exists() and time.monotonic()<deadline: time.sleep(.01)
            assert path.exists()
            pid = int(path.read_text())
            supervisor.terminate()
            assert supervisor.wait(timeout=25) == 143
            assert not live(pid), 'outer timeout left owned child alive'
        finally:
            if supervisor.poll() is None:
                supervisor.terminate(); supervisor.wait(timeout=25)
    print('PASS: timeout, exited parent, TERM-resistant worker cleanup, exit status, unrelated process preserved')


if __name__ == '__main__':
    test()
