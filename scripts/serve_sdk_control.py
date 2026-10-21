#!/usr/bin/env python3
"""Internal SDK process entry; only inherited private IPC, no HTTP listener."""
import argparse
import mmap
import os
from pathlib import Path
import socket
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--record',type=Path,required=True)
    parser.add_argument('--fd',type=int,required=True)
    parser.add_argument('--owner',type=int,required=True)
    parser.add_argument('--epoch-fd',type=int,required=True)
    parser.add_argument('--child',action='store_true')
    args=parser.parse_args()
    from robot_workbench.device_bootstrap import read_profile
    from robot_workbench.arx_output_guard import bind_child,supervise,parent_death
    profile=read_profile(args.profile)
    if not args.child:
        parent_death(args.owner)
        result=supervise(profile,[sys.executable,'-B',str(Path(__file__).resolve()),*sys.argv[1:],'--child'],
                         record_path=args.record,inherited_fds=(args.fd,args.epoch_fd))
        return 0 if result==0 else 1
    bind_child()
    connection=socket.socket(fileno=args.fd);os.set_inheritable(args.fd,False)
    from examples.scripts.debug import joint_head_real_owner as ref
    from robot_workbench.sdk_process import serve_worker
    ref.legacy_modules(profile['legacy_source'])
    backend=ref.hardware_backend(disabled_grippers=profile.get('disabled_grippers',[]),
        right_j4_diagnostic_kp=profile.get('right_j4_diagnostic_kp'),
        right_j4_diagnostic_kd=profile.get('right_j4_diagnostic_kd'),
        left_j6_diagnostic=profile.get('left_j6_diagnostic',False),
        prepared_reference_enabled=profile.get('prepared_reference_enabled',False),
        target_interpreter_profile=profile.get('target_interpreter_profile'))
    epoch=mmap.mmap(args.epoch_fd,32,access=mmap.ACCESS_READ)
    os.set_inheritable(args.epoch_fd,False)
    try:serve_worker(connection,backend,epoch)
    except BaseException:
        # No native destructor or implicit release on IPC failure. Guard is the
        # sole owner of exceptional termination and retained failure evidence.
        from robot_workbench.arx_output_guard import require_guard
        require_guard().call('trip',detail='SDK worker IPC or request failed')
        raise
    return 0


if __name__=='__main__':raise SystemExit(main())
