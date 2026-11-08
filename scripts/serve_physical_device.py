#!/usr/bin/env python3
"""Shared workbench on a robot host; startup never acquires arm control.

Use --preflight for a file/interface-only check. Camera SDK acquisition requires
--start-cameras; arm acquisition always requires the UI's current support check.
Existing reviewed LAN gateways may proxy this loopback service.
"""
import argparse
import json
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]
from robot_workbench.device_bootstrap import read_profile,preflight,build_physical_application,close_deployment


def retire_exited_control(device):
    """Explicit service stop may retire an already reaped guard, never live SDK."""
    with device.lock:
        runtime=device.runtime
        if runtime is None:return True
        if not getattr(runtime.owner.backend,'terminal_failure',None):return False
        try:retirement=runtime.retire_protected()
        except (ValueError,RuntimeError):return False
        device.closed.set();device.runtime=None;device.credentials={}
        device.protected_shutdown_retirement=retirement
        return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--preflight',action='store_true')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--port',type=int,default=18868)
    parser.add_argument('--listen',default='127.0.0.1',help='explicit private robot-host IPv4; default remains loopback')
    parser.add_argument('--browser-port',type=int,action='append',default=[])
    parser.add_argument('--start-cameras',action='store_true')
    parser.add_argument('--supervised-child',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if not 1024<=args.port<=65535:parser.error('an explicit unprivileged loopback port is required')
    from ipaddress import IPv4Address
    from robot_workbench.physical_endpoint import NETWORKS
    try:listen=IPv4Address(args.listen)
    except ValueError:parser.error('--listen must be one literal private or loopback IPv4')
    if not listen.is_loopback and not any(listen in network for network in NETWORKS):
        parser.error('--listen must be one explicit private robot-host IPv4')
    profile=read_profile(args.profile,require_ros=True)
    from robot_workbench.ros_data import require_runtime
    require_runtime()
    report=preflight(profile)
    if args.preflight:
        if args.start_cameras:parser.error('--preflight cannot start cameras')
        print(json.dumps(report,ensure_ascii=False,allow_nan=False,indent=2))
        return 0 if report['configuration_ready'] else 2
    output=args.output or Path(profile['storage_root'])/('service-'+time.strftime('%Y%m%d-%H%M%S'))
    if not output.is_absolute():parser.error('--output must be absolute')
    # The business service survives guard latches; only SDKProcess is supervised.
    from robot_workbench.gc_timing import install
    install()
    events=queue.Queue(maxsize=8192);done=threading.Event();log_error=[]
    def event(name,**fields):
        try:events.put_nowait(dict(event=name,wall_ns=time.time_ns(),**fields))
        except queue.Full:
            if not log_error:log_error.append('event queue overflow; evidence incomplete')
    deployment=build_physical_application(profile,output=output,start_cameras=args.start_cameras,event=event)
    def write_events():
        try:
            with (output/'events.jsonl').open('x') as stream:
                while not done.is_set() or not events.empty():
                    try:value=events.get(timeout=.1)
                    except queue.Empty:continue
                    stream.write(json.dumps(value,allow_nan=False)+'\n');stream.flush()
        except Exception as exc:log_error.append(str(exc))
    writer=threading.Thread(target=write_events,daemon=True,name='physical-workbench-events');writer.start()
    from robot_workbench.service import serve,handler
    try:
        if str(listen)=='127.0.0.1':server=serve(deployment.application,host=str(listen),port=args.port,browser_ports=args.browser_port)
        else:
            from http.server import ThreadingHTTPServer
            server=ThreadingHTTPServer((str(listen),args.port),handler(deployment.application));server.daemon_threads=True
            server.allowed_hosts={f'{listen}:{server.server_port}'}
    except BaseException:
        close_deployment(deployment);done.set();writer.join(timeout=2)
        raise
    def stop_service(*_):
        runtime=deployment.device.runtime
        if runtime is not None and not getattr(runtime,'remote_control_owner',False) and not retire_exited_control(deployment.device):
            deployment.device.submit(runtime.holder,{'action':'stop'})
            print('停止已受理；请先在页面确认双臂支撑并释放控制连接，再结束服务。',file=sys.stderr,flush=True)
            return
        threading.Thread(target=server.shutdown,daemon=True).start()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop_service)
    receipt=dict(schema='physical_workbench_service.v1',robot_id=profile['robot_id'],
        endpoint=f'http://{listen}:{server.server_port}',camera_ports=deployment.camera_ports,
        camera_sources_requested=args.start_cameras,hardware_connected=False,
        model_loaded=False,controller_acquisition='explicit_current_arms_supported_only',preflight=report,
        inference=(dict(model_id=deployment.inference.selected_model_id,
            model_endpoint=profile['inference']['model_endpoint'],
            catalog_root=deployment.inference.catalog_root,
            disabled_grippers=list(deployment.inference.disabled_grippers),
            model_prepare_requires_device_connection=False,model_loaded=False,control_ready=False,
            model_service_started_by_workbench=False) if deployment.inference is not None else None))
    (output/'service.json').write_text(json.dumps(receipt,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
    print(json.dumps(receipt,ensure_ascii=False,allow_nan=False),flush=True)
    try:server.serve_forever()
    finally:
        server.server_close();close_deployment(deployment);done.set();writer.join(timeout=2)
    return 0


if __name__=='__main__':raise SystemExit(main())
