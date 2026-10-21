"""Register exact authorized recording paths, offline; no SDK, sensors or actions."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from robot_workbench.collection_history import inspect_recording,register_recording


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',type=Path,required=True)
    parser.add_argument('--raw',type=Path,required=True)
    parser.add_argument('--playback',type=Path,required=True)
    parser.add_argument('--environment',choices=('fake','physical'),required=True)
    parser.add_argument('--robot-id',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    options=dict(environment=args.environment,robot_id=args.robot_id)
    if args.dry_run:
        result=inspect_recording(args.source_run,args.raw,args.playback,**options)
        print(json.dumps(dict(status='dry-run-passed',**result),ensure_ascii=False,indent=2))
    else:
        history=register_recording(args.source_run,args.raw,args.playback,args.output,**options)
        print(json.dumps(dict(status='registered',catalog=str(args.output),results=history.results()),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
