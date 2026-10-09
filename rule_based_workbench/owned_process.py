"""Bound a diagnostic process group, including workers whose parent has exited."""
import os
import signal
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


def wait_owned(child, timeout, grace=30):
    """Caller must create child with start_new_session=True; never discover PIDs."""
    if timeout <= 0 or grace <= 0:
        raise ValueError('positive timeout and shutdown grace required')
    timed_out = False
    try:
        try:
            return_code = child.wait(timeout=timeout)
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
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not 0 < args.timeout < float('inf'):
        parser.error('a command and finite positive timeout are required')
    def stop(signum, frame):
        raise SystemExit(128 + signum)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    child = subprocess.Popen(command, start_new_session=True)
    raise SystemExit(wait_owned(child, args.timeout, grace=20))


if __name__ == '__main__':
    main()
