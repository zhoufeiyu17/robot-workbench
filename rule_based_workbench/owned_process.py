"""Bound a diagnostic process group, including workers whose parent has exited."""
import os
import json
from pathlib import Path
import signal
import stat
import subprocess
import time


def group_alive(pgid):
    try:
        os.killpg(pgid,0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        # Some macOS runtimes report EPERM for an already vanished group. Only
        # accept that case after a read-only membership check of this exact PGID.
        groups=subprocess.check_output(['ps','-axo','pgid=,stat='],text=True)
        if any(parts[0]==str(pgid) and not parts[1].startswith('Z')
               for line in groups.splitlines() if len(parts:=line.split())>=2):raise
        return False


def read_failure_result(path):
    """Partial writes are pending; only a bounded regular failed receipt is terminal."""
    if path is None:return None
    try:
        fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK|os.O_NOFOLLOW)
        with os.fdopen(fd,'rb') as stream:
            info=os.fstat(stream.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size>4*1024*1024:return None
            raw=stream.read(4*1024*1024+1)
        if len(raw)>4*1024*1024:return None
        value=json.loads(raw)
        return value if isinstance(value,dict) and value.get('status')=='failed' else None
    except (OSError,ValueError,UnicodeError):
        return None


def wait_owned(child, timeout, grace=30, *, failure_result=None):
    """Caller must create child with start_new_session=True; never discover PIDs."""
    if timeout <= 0 or grace <= 0:
        raise ValueError('positive timeout and shutdown grace required')
    timed_out = False
    try:
        try:
            if failure_result is None:
                return_code = child.wait(timeout=timeout)
            else:
                deadline=time.monotonic()+timeout
                while True:
                    if read_failure_result(failure_result):
                        return_code=1
                        break
                    remaining=deadline-time.monotonic()
                    if remaining<=0:raise subprocess.TimeoutExpired(child.args,timeout)
                    try:
                        return_code=child.wait(timeout=min(.1,remaining))
                    except subprocess.TimeoutExpired:
                        continue
                    # A worker may write failure and exit zero before the poll.
                    if read_failure_result(failure_result):return_code=1
                    break
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = 124
    finally:
        # Do this even after the leader exits: Isaac workers may ignore TERM.
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        else:
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                child.poll()  # Reap the leader, so it cannot keep an empty group alive.
                if not group_alive(child.pid):
                    break
                time.sleep(min(.05, max(0, deadline-time.monotonic())))
            if group_alive(child.pid):
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        child.wait(timeout=5)
    return 124 if timed_out else return_code


def main():
    import argparse
    import sys
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('--failure-result',type=Path,
                        help='optional absent current-run result; only status=failed ends the child early')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not 0 < args.timeout < float('inf'):
        parser.error('a command and finite positive timeout are required')
    if args.failure_result is not None and (not args.failure_result.is_absolute()
            or args.failure_result.exists() or args.failure_result.is_symlink()):
        parser.error('failure result must be an absent absolute current-run path')
    def stop(signum, frame):
        raise SystemExit(128 + signum)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    child = subprocess.Popen(command, start_new_session=True)
    raise SystemExit(wait_owned(child, args.timeout, grace=20,failure_result=args.failure_result))


if __name__ == '__main__':
    main()
