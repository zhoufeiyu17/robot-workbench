#!/usr/bin/env python3
"""Start an inert loopback model service on the explicitly allocated model host."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.inference_model import FrozenBaseClient
from robot_workbench.inference_transport import ModelDomainAdapter, ModelService, serve_model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle-manifest', type=Path, required=True)
    parser.add_argument('--catalog-root', type=Path)
    parser.add_argument('--token-file', type=Path, required=True)
    parser.add_argument('--worker-log', type=Path, required=True)
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--forwarded-port',type=int,action='append',default=[],
                        help='explicit trusted loopback Host port on an existing forward; repeatable')
    parser.add_argument('--ready-file', type=Path)
    parser.add_argument('--gpu-uuid')
    parser.add_argument('--min-free-memory-mib', type=int, default=16384)
    parser.add_argument('--startup-timeout-s', type=float, default=600.)
    parser.add_argument('--inference-timeout-s', type=float, default=120.)
    args = parser.parse_args()
    def factory(bundle):
        log = str(args.worker_log)+(('.'+bundle['model_id']) if args.catalog_root else '')
        client = FrozenBaseClient(bundle, log_path=log, adapter=ModelDomainAdapter(),
            startup_timeout_s=args.startup_timeout_s, inference_timeout_s=args.inference_timeout_s)
        if args.gpu_uuid:
            import subprocess
            original_prepare=client.prepare
            def checked_prepare():
                gpu=os.environ.get('CUDA_VISIBLE_DEVICES')
                if not gpu or not gpu.isdecimal():raise ValueError('one explicitly assigned model GPU required')
                row=subprocess.check_output(['nvidia-smi','-i',gpu,'--query-gpu=uuid,memory.free',
                    '--format=csv,noheader,nounits'],text=True).strip().split(',')
                if row[0].strip()!=args.gpu_uuid or int(row[1])<args.min_free_memory_mib:
                    raise ValueError('model GPU identity or available memory differs from deployment contract')
                return original_prepare()
            client.prepare=checked_prepare
        return client
    client = factory(json.loads(args.bundle_manifest.read_text()))
    catalog = catalog_rows = None
    if args.catalog_root:
        from robot_workbench.inference_catalog import catalog_manifests, scan_catalog
        catalog = lambda:catalog_manifests(args.catalog_root, args.bundle_manifest)
        catalog_rows = lambda:scan_catalog(args.catalog_root, args.bundle_manifest)
    service = ModelService(client, catalog=catalog, client_factory=factory if catalog else None, catalog_rows=catalog_rows)
    server = serve_model(service, token_file=args.token_file, port=args.port,forwarded_ports=args.forwarded_port)
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    for sig in (signal.SIGINT, signal.SIGTERM):signal.signal(sig, stop)
    print(json.dumps(dict(status='not_loaded', endpoint=f'http://127.0.0.1:{server.server_port}',
        model_id=client.model_id,forwarded_ports=list(server.forwarded_ports),
        message='explicit authenticated prepare required; model has not been loaded')), flush=True)
    if args.ready_file:
        temporary=args.ready_file.with_suffix('.tmp')
        with temporary.open('x') as stream:
            json.dump(dict(pid=os.getpid(),endpoint=f'http://127.0.0.1:{server.server_port}',
                model_id=client.model_id,forwarded_ports=list(server.forwarded_ports),
                loaded=False,bundle_manifest=str(args.bundle_manifest),worker_log=str(args.worker_log)),stream)
        if args.ready_file.exists():raise FileExistsError(args.ready_file)
        temporary.rename(args.ready_file)
    try:server.serve_forever()
    finally:server.server_close();service.close()


if __name__ == '__main__':main()
