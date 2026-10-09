"""Bounded renderer recovery under an existing supervisord; no robot commands."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def process_memory(rows, group):
    members = {int(pid): int(rss)*1024 for pid, pgid, rss in
               (line.split() for line in rows.splitlines()) if int(pgid) == group}
    return members, sum(members.values())


def gpu_memory(rows, members):
    return sum({int(pid): int(used)*1024**2 for pid, used in
                (line.split(',') for line in rows.splitlines()) if int(pid) in members}.values())


def main(root):
    root = Path(root)
    launch = json.loads((root/'launch.json').read_text())
    os.sched_setaffinity(0, launch['cpu_ids'])
    env = dict(os.environ, **launch['env'])
    def state(**fields):
        temporary = root/'status.tmp'
        temporary.write_text(json.dumps(dict(timestamp_s=time.time(), **fields), indent=2))
        temporary.replace(root/'status.json')
    child = None
    def stop(*_):
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            try: child.wait(timeout=15)
            except subprocess.TimeoutExpired: os.killpg(child.pid, signal.SIGKILL); child.wait()
    def shutdown(*_):
        stop(); state(status='stopped'); raise SystemExit(0)
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    for attempt in range(4):
        while True:
            used = int(subprocess.check_output(['nvidia-smi', '--id='+env['CUDA_VISIBLE_DEVICES'],
                '--query-gpu=memory.used', '--format=csv,noheader,nounits'], text=True).strip())
            if used < 100: break
            state(status='waiting_gpu', memory_used_mib=used, attempt=attempt); time.sleep(10)
        output = root/('output-'+str(time.time_ns()))
        argv = list(launch['argv']); argv[argv.index('--output')+1] = str(output)
        with (root/(output.name+'.log')).open('x') as log:
            child = subprocess.Popen(argv, cwd=launch['cwd'], env=env, stdin=subprocess.DEVNULL,
                                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            state(status='running', process_group=child.pid, output=str(output), attempt=attempt)
            reason = 'process_exit'
            try:
                while child.poll() is None:
                    time.sleep(5)
                    members, rss = process_memory(subprocess.check_output(['ps', '-eo', 'pid=,pgid=,rss='], text=True), child.pid)
                    vram = gpu_memory(subprocess.check_output(['nvidia-smi', '--query-compute-apps=pid,used_gpu_memory',
                                          '--format=csv,noheader,nounits'], text=True), members)
                    if rss > 32*1024**3 or vram > 32*1024**3:
                        reason = 'memory_budget'; stop(); break
            except Exception as error:
                stop(); state(status="recovery_required", reason="monitor_failure", error=str(error)); return
        code = child.wait()
        with (root/'exits.jsonl').open('a') as log:
            log.write(json.dumps(dict(timestamp_s=time.time(), attempt=attempt, exit_code=code, reason=reason))+'\n')
        if code == 0 or reason == 'memory_budget':
            state(status='stopped' if code == 0 else 'recovery_required', reason=reason, exit_code=code); return
        if attempt < 3:
            state(status='backoff', attempt=attempt, exit_code=code); time.sleep((5,15,45)[attempt])
    state(status='recovery_required', reason='retry_budget_exhausted', exit_code=code)


if __name__ == '__main__':
    main(sys.argv[1])
