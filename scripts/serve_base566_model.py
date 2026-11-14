#!/usr/bin/env python3
"""Registered ARX model endpoint with host-exclusive admission, no robot API.

The native client keeps its existing GPU UUID/free-memory check and shared flock
for the complete worker lifetime. A permanent operator service is not an old
experiment lease; Agent callers must still reserve and account for their GPU use.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import stat
import sys
import threading


def validate_registry(value, machine_id):
    required={'workspace','owner_session','gpu_uuid','models','native_source','resource_policy'}
    if not isinstance(value,dict) or required-set(value) or set(value)-required-{'min_free_memory_mib'}:
        raise ValueError('explicit registered model deployment required')
    policy=value['resource_policy']
    if policy!={'mode':'host_exclusive','machine_id_sha256':hashlib.sha256(machine_id.encode()).hexdigest()}:
        raise ValueError('model admission is bound to this host and its exclusive GPU lock')
    workspace=Path(value['workspace']).resolve(strict=True)
    source=Path(value['native_source']).resolve(strict=True)
    if not source.is_relative_to(workspace) or not(source/'scripts/serve_base566_model.py').is_file():
        raise ValueError('registered read-only native model source required')
    return workspace,source


def create_service(registry, machine_id):
    workspace,source=validate_registry(registry,machine_id)
    sys.path.insert(0,str(source))
    from manipulation.runtime.workbench_client import ResidentBase566Client
    # Reuse the qualified native registry/catalog/transport construction.
    spec=importlib.util.spec_from_file_location('native_base566_endpoint',source/'scripts/serve_base566_model.py')
    native=importlib.util.module_from_spec(spec);spec.loader.exec_module(native)
    class HostResident(ResidentBase566Client):
        def _acquire_gpu(self):
            from robot_workbench.gpu_admission import acquire
            self._gpu_lock=acquire(dict(registry,model_id=self.model_id,
                min_free_memory_mib=self.min_free_memory_mib))

        def _lease_check(self,remaining=0):
            if Path('/etc/machine-id').read_text().strip()!=machine_id:
                raise ValueError('registered model host changed')
            if remaining==0 and (self._gpu_lock is None or self._gpu_lock.closed):
                raise ValueError('resident GPU lock is not held')
            return {'resource_admission':'host_exclusive','gpu_uuid':self.gpu_uuid}
    native.ResidentBase566Client=HostResident
    return native.create_service({k:v for k,v in registry.items()if k not in ('native_source','resource_policy')})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('registry','token-file','ready-file'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--port',type=int,required=True);args=parser.parse_args()
    info=args.registry.lstat()
    if not stat.S_ISREG(info.st_mode) or info.st_uid!=os.getuid() or info.st_mode&0o077:
        raise ValueError('private owned registry required')
    if args.ready_file.exists():raise ValueError('ready receipt must be fresh')
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from robot_workbench.inference_transport import serve_model
    service=create_service(json.loads(args.registry.read_text()),Path('/etc/machine-id').read_text().strip())
    server=serve_model(service,token_file=args.token_file,port=args.port)
    def stop(*_):threading.Thread(target=server.shutdown,daemon=True).start()
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    receipt=dict(pid=os.getpid(),uid=os.getuid(),boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        start_ticks=int(Path('/proc/self/stat').read_text().rpartition(')')[2].split()[19]),
        script=str(Path(__file__).resolve()),loaded=False,resource_admission='host_exclusive')
    args.ready_file.parent.mkdir(parents=True,exist_ok=True)
    with args.ready_file.open('x')as stream:json.dump(receipt,stream)
    try:server.serve_forever()
    finally:server.server_close();server.model_service.close()


if __name__=='__main__':main()
