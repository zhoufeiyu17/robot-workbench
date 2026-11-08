#!/usr/bin/env python3
"""Independent inert control service; acquires hardware only by explicit IPC action."""
from pathlib import Path
import argparse,hashlib,signal,sys,threading

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args()
    from robot_workbench.device_bootstrap import read_profile,physical_runtime_factory
    from robot_workbench.service import Device
    from robot_workbench.control_owner import ControlOwnerService,collection_recording_root
    profile=read_profile(args.profile,require_ros=True)
    from robot_workbench.ros_data import require_runtime
    require_runtime()
    device=Device(profile['robot_id'],'physical',None,label=profile.get('label'))
    service=ControlOwnerService(device,args.root,profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest())
    from robot_workbench.recording_process import capture_control
    from robot_workbench.sample_clock import clock_spec
    def recording(payload,on_fault):
        return capture_control(collection_recording_root(args.root),payload,cameras=None,environment='physical',
            clock=clock_spec(),source_time=device.runtime.owner.control_time,on_fault=on_fault,ros_runtime=device.runtime)
    recording.supports_deferred_preparation=True
    recording.recording_backend='capture_session_v1'
    device.factory=physical_runtime_factory(profile,event=service.event,recording_factory=recording,
        execution_root=args.root/'execution',local_owner=True)
    def stop(*_):
        if device.runtime is not None:
            print('Control Owner still acquired; normal supported release required before stopping service.',file=sys.stderr,flush=True)
            return
        threading.Thread(target=service.server.shutdown,daemon=True).start()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    try:service.serve_forever()
    finally:service.close()
    return 0

if __name__=='__main__':raise SystemExit(main())
