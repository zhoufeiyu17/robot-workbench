#!/usr/bin/env python3
"""Internal SDK process entry; only inherited private IPC, no HTTP listener."""
import argparse
import mmap
import os
from pathlib import Path
import socket
import sys
import signal
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]


def _bind_sdk_import(profile):
    contract=profile.get('sdk_quiesce')
    if contract:
        # The validated deployment pin must take precedence over an installed SDK.
        sys.path.insert(0,str(Path(contract['module']['path']).resolve().parent))
        loaded=sys.modules.get('arx5_interface')
        if loaded is not None:
            from robot_workbench.sdk_quiesce import verify_module
            verify_module(loaded,contract)


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
    _bind_sdk_import(profile)
    connection=socket.socket(fileno=args.fd);os.set_inheritable(args.fd,False)
    from examples.scripts.debug import joint_head_real_owner as ref
    from robot_workbench.sdk_quiesce import dm_joint_velocity_wire_scale,dm_torque_units,ec_torque_units,load_gripper_calibration
    from robot_workbench.gravity_calibration import load_gravity_calibration
    from robot_workbench.sdk_process import serve_worker
    if profile.get('native_gripper_calibration'):
        from arx_env.joint_head_jaw_mapping import configure_mapping
        configure_mapping(profile['jaw_mapping_file'])
    ref.legacy_modules(profile['legacy_source'])
    backend=ref.hardware_backend(disabled_grippers=profile.get('disabled_grippers',[]),
        raw_state_enabled=profile.get('raw_state_enabled',False),
        can_interfaces=profile['can_interfaces'],
        right_j4_diagnostic_kp=profile.get('right_j4_diagnostic_kp'),
        right_j4_diagnostic_kd=profile.get('right_j4_diagnostic_kd'),
        left_j6_diagnostic=profile.get('left_j6_diagnostic',False),
        prepared_reference_enabled=profile.get('prepared_reference_enabled',False),
        target_interpreter_profile=profile.get('target_interpreter_profile'),
        motion_gain_profile=profile.get('motion_gain_profile'),
        native_reference_api=profile.get('native_reference_api','legacy_recompute'),
        dm_joint_velocity_wire_scale=dm_joint_velocity_wire_scale(profile.get('dm_joint_velocity_contract'),can_interfaces=profile['can_interfaces'],robot_id=profile['robot_id']),
        dm_torque_unit_contract=dm_torque_units(profile.get('dm_torque_contract'),can_interfaces=profile['can_interfaces'],robot_id=profile['robot_id']),
        gripper_calibration=load_gripper_calibration(profile.get('native_gripper_calibration'),robot_id=profile['robot_id']),
        ec_torque_unit_contract=ec_torque_units(profile.get('ec_torque_contract')),
        gravity_calibration=load_gravity_calibration(profile.get('sdk_gravity_calibration')))
    epoch=mmap.mmap(args.epoch_fd,32,access=mmap.ACCESS_READ)
    os.set_inheritable(args.epoch_fd,False)
    from robot_workbench.arx_output_guard import require_guard
    hold = getattr(require_guard(), 'fault_hold', None)
    if hold is not None:
        def retain_on_signal(signum, frame):
            if hold.read(1):
                hold.trip(3)
            else:
                raise SystemExit(128 + signum)
        signal.signal(signal.SIGTERM, retain_on_signal)
        signal.signal(signal.SIGINT, retain_on_signal)
    try:serve_worker(connection,backend,epoch)
    except BaseException:
        # No native destructor or implicit release on IPC failure. Guard is the
        # sole owner of exceptional termination and retained failure evidence.
        if hold is not None and hold.read(1):
            hold.trip(3)
            # Python/Owner IPC may be gone. Native threads still own the only
            # controller and hold the latched pose until supported retirement.
            while True:time.sleep(1)
        require_guard().call('trip',detail='SDK worker IPC or request failed')
        raise
    return 0


if __name__=='__main__':raise SystemExit(main())
