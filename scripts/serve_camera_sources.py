#!/usr/bin/env python3
"""Camera-only lifetime for the existing shared RGB-D ingress; no robot control."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from robot_workbench.rgbd_service import CommonCameraService, CAMERAS, start_capture_sources


def atomic(path, value):
    temporary=path.with_suffix('.tmp')
    with os.fdopen(os.open(temporary,os.O_WRONLY|os.O_CREAT|os.O_TRUNC,0o600),'w') as stream:
        json.dump(value,stream,allow_nan=False)
    temporary.replace(path)


def reset_camera(profile_path,root,role,*,sdk=None,timeout_s=15.,poll_s=.1):
    """One explicit RealSense reset, after the camera owner and all captures exit."""
    if role not in CAMERAS:raise ValueError('one fixed camera role required')
    profile=json.loads(Path(profile_path).read_text())
    if profile.get('environment')!='physical' or set(profile.get('cameras',{}))!=set(CAMERAS):
        raise ValueError('explicit physical three-camera deployment profile required')
    row=profile['cameras'][role];serial=row.get('serial')
    if not isinstance(serial,str) or not serial.isascii() or not serial.isdecimal() or len(serial)>64:
        raise ValueError('configured RealSense serial required')
    if any(other!=role and value.get('serial')==serial for other,value in profile['cameras'].items()):
        raise ValueError('camera serial must identify exactly one configured role')
    if json.loads(Path(row['intrinsics']).read_text()).get('serial')!=serial:
        raise ValueError('camera serial differs from configured intrinsics')
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    with (root/'owner.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if sdk is None:
            import pyrealsense2 as sdk
        def matching():
            return [device for device in sdk.context().query_devices()
                    if device.get_info(sdk.camera_info.serial_number)==serial]
        devices=matching()
        if len(devices)!=1:raise ValueError('configured camera is absent or ambiguous; no reset was sent')
        requested=time.monotonic();devices[0].hardware_reset()  # Never retry a reset whose outcome may be unknown.
        deadline=requested+timeout_s;disappeared=False
        while time.monotonic()<deadline:
            time.sleep(poll_s)
            try:devices=matching()
            except RuntimeError:continue  # USB enumeration can be transient; never resend the reset.
            if not devices:disappeared=True
            if len(devices)>1:raise RuntimeError('camera identity ambiguous after reset; outcome unknown')
            if devices and (disappeared or time.monotonic()-requested>=2.):
                return dict(status='enumerated',role=role,serial=serial,reset_calls=1,
                    disappearance_observed=disappeared,raw_frames_verified=False)
        raise TimeoutError('camera did not enumerate after the single reset; outcome unknown')


def run(profile_path, root):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    lock=(root/'owner.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    profile=json.loads(Path(profile_path).read_text())
    intrinsics={name:json.loads(Path(profile['cameras'][role]['intrinsics']).read_text()) for role,name in CAMERAS.items()}
    for role,name in CAMERAS.items():
        if intrinsics[name]['serial']!=profile['cameras'][role]['serial']:
            raise ValueError('camera serial differs from configured intrinsics: '+role)
    output=root/('session-'+str(time.time_ns()));output.mkdir()
    state=root/'state.json';stopped=threading.Event();service=None;processes=[]
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopped.set())
    failures={}
    def status(phase,error=None):
        atomic(state,dict(phase=phase,error=error,camera_failures=failures,output=str(output),pid=os.getpid(),wall_ns=time.time_ns()))
    status('starting')
    try:
        service=CommonCameraService.start(intrinsics,output/'camera-service','physical')
        service.start_servers()
        atomic(root/'current.json',service.descriptor)
        processes=start_capture_sources(profile,service,output)
        while not stopped.wait(.5):
            if service.process.poll() is not None:
                raise RuntimeError('共享 RGB-D 进程退出，请查看相机日志')
            new_failures={}
            for role,process in zip(profile['cameras'],processes):
                if role not in failures and process.poll() is not None:
                    log=output/('capture-'+role+'.log')
                    lines=log.read_text(errors='replace').splitlines()if log.exists()else[]
                    detail=next((line for line in reversed(lines) if 'Error:' in line), '采集进程退出')
                    new_failures[role]=detail[:400]
            failures.update(new_failures)
            if failures and len(failures)==len(processes):raise RuntimeError('; '.join(k+': '+v for k,v in failures.items()))
            for role,error in new_failures.items():
                name=CAMERAS[role]
                service.frames[name].publish(status=dict(ready=False,phase='capture_failed',error=error,serial=profile['cameras'][role]['serial']))
                token=service.channels[name].token()
                if token:service.channels[name].fail(error,sampled_token=token)
            status('degraded'if failures else 'running' if all(service.frames[name].snapshot(formats=[])['status'].get('ready') for name in intrinsics) else 'starting',
                '; '.join(k+': '+v for k,v in failures.items())or None)
        status('stopped')
    except BaseException as error:
        status('error',str(error));raise
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        if service is not None:service.close()
        lock.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--reset-role',choices=tuple(CAMERAS),help='one device-only USB reset; camera source must be stopped')
    args=parser.parse_args()
    if args.reset_role:print(json.dumps(reset_camera(args.profile,args.root,args.reset_role),allow_nan=False))
    else:run(args.profile,args.root)
