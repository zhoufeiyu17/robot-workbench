#!/usr/bin/env python3
"""Read-only environment/process diagnostic. Never initializes a robot SDK."""
import argparse,importlib.util,json,platform,shutil,subprocess,math
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--feedback',type=Path,help='optional per-motor feedback snapshot JSON');a=p.parse_args()
def check(argv):
    if not shutil.which(argv[0]):return {'available':False}
    r=subprocess.run(argv,capture_output=True,text=True,timeout=10)
    return {'returncode':r.returncode,'output':r.stdout[-12000:]}
report={'platform':platform.system(),'dependencies':{m:importlib.util.find_spec(m) is not None for m in ['numpy','scipy','cv2']},'processes':check(['pgrep','-af','joint_head|arx|controller']),'robot_sdk_initialized':False,'motor_feedback':'not checked: no per-motor snapshot supplied'}
if a.feedback:
    snap=json.loads(a.feedback.read_text());motors=snap.get('motors',[])
    report['motor_feedback']={'valid':bool(motors) and all(m.get('valid') is True and isinstance(m.get('age_s'),(int,float)) and math.isfinite(m['age_s']) and 0<=m['age_s']<=.1 for m in motors),'motors':motors,'scope':'supplied snapshot only; does not prove live CAN freshness'}
if platform.system()=='Linux':report['can']=check(['ip','-details','-json','link','show','type','can'])
print(json.dumps(report,indent=2))
