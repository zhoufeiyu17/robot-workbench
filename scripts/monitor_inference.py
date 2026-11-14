#!/usr/bin/env python3
"""Read-only numeric inference monitor. No model or hardware API is imported."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.runtime_monitor import IncidentCollector, replay_incident


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--replay', type=Path, help='Analyze a saved numeric incident with no network access')
    parser.add_argument('--interval', type=float, default=.5)
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--parent-pid', type=int)
    args = parser.parse_args()
    if args.replay:
        print(json.dumps(replay_incident(json.loads(args.replay.read_text())), ensure_ascii=False, indent=2))
        return
    if not args.endpoint or args.output is None:
        parser.error('--endpoint and --output are required for live monitoring')
    if args.interval < .25 or args.duration < 0:
        parser.error('interval must be at least .25 s; duration must be nonnegative')
    if args.parent_pid is not None:
        from robot_workbench.arx_output_guard import parent_death
        parent_death(args.parent_pid)
    stopped = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stopped.set())
    monitor = IncidentCollector(args.endpoint, args.output)
    started = time.monotonic()
    while not stopped.is_set() and (not args.duration or time.monotonic()-started < args.duration):
        began = time.monotonic()
        monitor.poll()
        stopped.wait(max(0., args.interval-(time.monotonic()-began)))
    monitor.flush(ready_only=False)


if __name__ == '__main__':
    main()
