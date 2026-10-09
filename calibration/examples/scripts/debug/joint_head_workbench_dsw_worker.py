#!/usr/bin/env python3
"""Trusted local SSH launcher for task-owned X5 RTX capture; no robot access."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import tempfile
import uuid
import signal
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--request',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    c=json.loads(a.config.read_text())
    root=c['root'].rstrip('/')+'/job-'+uuid.uuid4().hex[:16]
    if not root.startswith('/mnt/data/feiyu.zhou/code/joint-head-workbench-'):raise ValueError('untrusted task root')
    renderer=Path(__file__).with_name('joint_head_workbench_dsw.py')
    sha=hashlib.sha256(renderer.read_bytes()).hexdigest()
    ssh=['ssh','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','IdentitiesOnly=yes','-i',c['ssh_key'],'-p',str(c['port']),c['host']]
    scp=['scp','-q','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','IdentitiesOnly=yes','-i',c['ssh_key'],'-P',str(c['port'])]
    def remote(command,timeout=30):return subprocess.run(ssh+[command],check=True,capture_output=True,text=True,timeout=timeout).stdout
    occupied=remote('nvidia-smi --query-compute-apps=pid --format=csv,noheader').strip()
    if occupied:raise RuntimeError('GPU process present; refusing shared capture')
    contract={'schema_version':'anyverse.dsw-codex-experiment.v1','identity':{'experiment_id':Path(root).name,'owner':'feiyu.zhou','dsw_instance_id':'dsw-3lduujsvlem1a91wdl'},'code':{'root':root,'revision':sha,'repository_expected':False},'filesystem':{'readonly_roots':[c['isaac_python'],c['deps']],'writable_roots':[root],'allow_overwrite':False,'allow_delete':False},'resources':{'cuda_visible_devices':[0],'max_gpu_count':1,'max_wall_time_minutes':10,'max_workers':1,'max_cpu_threads':4,'max_ram_gb':32,'require_gpu_idle_check':True},'network':{'allowed_endpoints':[],'expose_service':False},'execution':{'preflight_commands':['nvidia-smi compute-apps'],'dry_run_commands':['render.py --dry-run'],'smoke_commands':['render.py bounded 600 seconds'],'formal_commands':['render.py bounded 600 seconds after parent smoke accepted'],'stop_conditions':['GPU occupied','timeout','renderer failure']},'provenance':{'code_sha256':{'render.py':sha},'input_sha256':{'request.json':hashlib.sha256(a.request.read_bytes()).hexdigest()}},'permissions':{'install_project_dependencies':False,'install_system_packages':False,'modify_network_or_proxy':False,'kill_untracked_processes':False},'acceptance':{'expected_artifacts':['dataset.json','completion.json','PNG','per-frame JSON'],'success_conditions':['RTX render','actual pixel marker detection','kinematic only explicitly labelled']},'cleanup':{'stop_only_recorded_processes':True,'automatic_shutdown':False}}
    with tempfile.TemporaryDirectory(prefix='x5-rtx-') as td:
        td=Path(td)
        (td/'contract.json').write_text(json.dumps(contract,indent=2))
        with a.request.open('rb') as src,gzip.open(td/'request.json.gz','wb') as dst:
            import shutil
            shutil.copyfileobj(src,dst)
        remote('mkdir -p '+shlex.quote(root))
        subprocess.run(scp+[str(renderer),str(td/'contract.json'),str(td/'request.json.gz'),c['host']+':'+root+'/'],check=True,timeout=120)
    def cancel(signum, frame):
        stop = 'if test -f '+shlex.quote(root+'/controller.pid')+'; then p=$(cat '+shlex.quote(root+'/controller.pid')+'); kill -TERM -- -"$p" 2>/dev/null || true; sleep 1; kill -KILL -- -"$p" 2>/dev/null || true; fi'
        try:
            remote(stop,timeout=10)
            check=remote('if test -f '+shlex.quote(root+'/controller.pid')+'; then p=$(cat '+shlex.quote(root+'/controller.pid')+'); if kill -0 -- -"$p" 2>/dev/null; then echo STILL_PRESENT; else echo STOPPED; fi; else echo NOT_STARTED; fi',timeout=10)
            print('Task-owned remote stop status: '+check.strip(),flush=True)
        finally:
            raise SystemExit(128+signum)
    signal.signal(signal.SIGTERM,cancel)
    signal.signal(signal.SIGINT,cancel)
    command='cd '+shlex.quote(root)+' && gunzip request.json.gz && mkdir cache && '
    env='CUDA_VISIBLE_DEVICES=0 OMP_NUM_THREADS=4 XDG_CACHE_HOME='+shlex.quote(root+'/cache')+' PYTHONPATH='+shlex.quote(c['deps'])+' '
    base=shlex.quote(c['isaac_python'])+' joint_head_workbench_dsw.py --request request.json --output capture'
    run='echo $$ > controller.pid; exec '+env+'timeout --kill-after=15s 600 '+base+' > capture.log 2>&1'
    run=run.replace('exec CUDA_VISIBLE_DEVICES=', 'exec env CUDA_VISIBLE_DEVICES=')
    command+=env+base+' --dry-run && setsid --wait sh -c '+shlex.quote(run)
    try:
        remote(command,timeout=630)
    except Exception:
        print(remote('tail -40 '+shlex.quote(root+'/capture.log')),flush=True)
        raise
    a.output.parent.mkdir(parents=True,exist_ok=True)
    subprocess.run(scp+['-r',c['host']+':'+root+'/capture',str(a.output)],check=True,timeout=120)
    (a.output/'contract.json').write_text(json.dumps(contract,indent=2))
    (a.output/'remote.json').write_text(json.dumps({'root':root,'renderer_sha256':sha}))
    completion=json.loads((a.output/'completion.json').read_text())
    if not completion.get('passed'):raise RuntimeError('renderer did not complete')
    print(json.dumps({'output':str(a.output),**completion}),flush=True)

if __name__=='__main__':main()
