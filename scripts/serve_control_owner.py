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
    profile=read_profile(args.profile)
    device=Device(profile['robot_id'],'physical',None,label=profile.get('label'))
    service=ControlOwnerService(device,args.root,profile_sha256=hashlib.sha256(args.profile.read_bytes()).hexdigest())
    recording=None
    if profile.get('camera_service_descriptor'):
        from robot_workbench.rgbd_service import CommonCameraService
        from robot_workbench.recording_process import ProcessRecordingSession
        import secrets
        def recording(payload,on_fault):
            cameras=CommonCameraService.connect(profile['camera_service_descriptor'])
            return ProcessRecordingSession(collection_recording_root(args.root)/secrets.token_hex(16),plan_ref=payload['plan_ref'],
                camera_channels=cameras.channels,storage_format='segments',execution_timeline=payload.get('execution_timeline'),
                environment='physical',execution_owner=payload.get('execution_owner'),on_fault=on_fault,
                persistent_service=True,max_wall_time_seconds=7200,**payload.get('recording_preparation',{}))
        recording.supports_deferred_preparation=True
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
