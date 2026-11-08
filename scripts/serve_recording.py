#!/usr/bin/env python3
"""Resident CPU recording service. No SDK, model or camera acquisition."""
import argparse
from pathlib import Path
import signal
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]


def main():
    from robot_workbench.recording_process import RecordingService
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--data-root',type=Path,action='append',required=True)
    args=parser.parse_args()
    service=RecordingService(args.root,data_roots=args.data_root)
    def stop(*_):
        try:service.dispatch(dict(action='shutdown'))
        except ValueError as error:print(str(error),file=sys.stderr,flush=True)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    try:service.serve_forever()
    finally:service.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
