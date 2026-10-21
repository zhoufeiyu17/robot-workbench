#!/usr/bin/env python3
"""Read native model files and export display assets; no SDK/cameras/GPU."""
from pathlib import Path
import argparse,json,sys
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--device-id',required=True);args=p.parse_args()
    from arx_env.joint_head_workbench_geometry import Geometry
    from robot_workbench.target_model import export_model
    print(json.dumps(export_model(Geometry(json.loads(args.config.read_text())),args.output,args.device_id)))
if __name__=='__main__':main()
