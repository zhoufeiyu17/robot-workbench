#!/usr/bin/env python3
"""Preflight an explicit Owner profile without creating hardware controllers."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

from robot_workbench.gravity_calibration import check_sdk_solver


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    result=check_sdk_solver(json.loads(args.profile.read_text()))
    result['profile']=str(args.profile.resolve())
    raw=json.dumps(result,indent=2)+'\n'
    if args.output is not None:args.output.write_text(raw)
    print(raw,end='')


if __name__=='__main__':main()
