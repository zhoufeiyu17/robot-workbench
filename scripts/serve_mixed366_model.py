#!/usr/bin/env python3
"""Inert local HTTP model service; explicit asynchronous prepare owns GPU loading."""
from pathlib import Path
import argparse,json,os,signal,sys,threading
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--registry',type=Path,required=True);p.add_argument('--token-file',type=Path,required=True);p.add_argument('--port',type=int,required=True)
    a=p.parse_args();info=a.registry.stat()
    if info.st_uid!=os.getuid()or info.st_mode&0o077:raise ValueError('private registry required')
    from robot_workbench.mixed366 import Service
    from robot_workbench.inference_transport import ModelService,serve_model
    registry=json.loads(a.registry.read_text())
    from robot_workbench import wam,wam_mem
    service=(wam_mem.model_service(registry)if wam_mem.is_model_id(registry.get('model_id'))else
             wam.model_service(registry)if wam.is_model_id(registry.get('model_id'))else Service(registry))
    server=serve_model(service,token_file=a.token_file,port=a.port)
    for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,lambda *_:threading.Thread(target=server.shutdown,daemon=True).start())
    try:server.serve_forever()
    finally:server.server_close();server.model_service.close()

if __name__=='__main__':main()
