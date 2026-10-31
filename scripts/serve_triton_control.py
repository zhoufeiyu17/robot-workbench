#!/usr/bin/env python3
"""Start the explicitly configured, loopback-only CPU Triton control plane."""
import argparse
import json
from pathlib import Path
import signal
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench.triton_control import ControlPlane,serve

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--config',type=Path,required=True);parser.add_argument('--port',type=int,default=18924);args=parser.parse_args()
    plane=ControlPlane(json.loads(args.config.read_text()));server=serve(plane,args.port)
    def stop(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:plane.close();server.server_close()

if __name__=='__main__':main()
