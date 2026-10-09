"""Real subprocess regression; no Isaac, GPU, network, or hardware."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from owned_process import wait_owned


def live(pid):
    result = subprocess.run(['ps', '-o', 'stat=', '-p', str(pid)], capture_output=True, text=True)
    state = result.stdout.strip()
    return bool(state) and not state.startswith('Z')


def test():
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
                assert wait_owned(child, .2, .2)==expected
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
        supervisor = subprocess.Popen([sys.executable, str(Path(__file__).with_name('owned_process.py')),
                                       '--timeout', '60', '--', sys.executable, '-c', command, str(path)])
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
