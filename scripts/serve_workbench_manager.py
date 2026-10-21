#!/usr/bin/env python3
"""Serve an independent management plane; startup never connects the SDK."""
import argparse
import json
from pathlib import Path
import signal
import sys
import threading

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from robot_workbench.management import WorkbenchManager,serve


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--host',default='127.0.0.1');parser.add_argument('--port',type=int,required=True)
    parser.add_argument('--forwarded-port',type=int,action='append',default=[])
    args=parser.parse_args()
    from ipaddress import IPv4Address
    if (not IPv4Address(args.host).is_private or not 1024<=args.port<=65535
            or any(not 1024<=value<=65535 for value in args.forwarded_port)):
        parser.error('private IPv4 and unprivileged explicit ports required')
    manager=WorkbenchManager(json.loads(args.profile.read_text()))
    aliases=[f'{name}:{port}' for port in args.forwarded_port for name in ('127.0.0.1','localhost')]
    server=None
    try:
        server=serve(manager,host=args.host,port=args.port,allowed_hosts=aliases)
        for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:threading.Thread(target=server.shutdown,daemon=True).start())
        server.serve_forever()
    finally:
        if server is not None:server.server_close()
        manager.close()


if __name__=='__main__':main()
