"""CPU management entry for a fixed, pre-reserved full-fake resource window."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import shlex
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT/'rule_based_workbench')]
from robot_workbench.fake_session import FakeSession,serve_session,PublicSessionError
from calibration_resume_source import source_spec as calibration_source
from robot_workbench.collection_history import source_spec as history_source


def save(path,value):
    path=Path(path);temporary=path.with_name(path.name+'.'+secrets.token_hex(4)+'.tmp')
    try:
        with temporary.open('x') as stream:json.dump(value,stream,ensure_ascii=False,allow_nan=False,indent=2)
        os.replace(temporary,path)
    finally:temporary.unlink(missing_ok=True)


class FullFakeProfile:
    """Derive child paths from one administrator-frozen contract, never HTTP input."""
    def __init__(self,*,contract,base,nominal,state_dir,lease,deadline_wall_ns,
                 resume=None,history=None,browser_ports=()):
        self.contract_path=Path(contract).resolve();self.contract=json.loads(self.contract_path.read_bytes())
        self.base=Path(base).resolve();self.nominal=Path(nominal).resolve();self.root=Path(state_dir).resolve()
        self.lease=Path(lease).resolve();self.deadline=deadline_wall_ns;self.browser_ports=list(browser_ports)
        self.root.mkdir(parents=True,exist_ok=True)
        self.source_file=self.root/'sources.json'
        initial=dict(resume=str(Path(resume).resolve()) if resume else None,
                     history=str(Path(history).resolve()) if history else None,catalogs=[],last_output=None)
        self.sources=json.loads(self.source_file.read_text()) if self.source_file.exists() else initial
        self.code=Path(self.contract['code']['root']).resolve()
        self.template=shlex.split(self.contract['execution']['smoke_commands'][0])
        if (self.contract['identity']['session_id']=='' or self.contract['resources']['cuda_visible_devices']!=[1]
                or '--output' not in self.template or '--gpu' not in self.template
                or self.template[self.template.index('--gpu')+1]!='1'
                or str(self.code/'scripts/serve_physx_device.py') not in self.template):
            raise ValueError('explicit frozen full-fake contract required')

    def guard(self,generation,remaining_s):
        lease=json.loads(self.lease.read_text());c=self.contract
        if (lease.get('schema')!='fake_session_lease.v1' or lease.get('session_id')!=c['identity']['session_id']
                or lease.get('status')!='active' or not lease.get('gpu_record')
                or type(lease.get('deadline_wall_ns')) is not int or self.deadline>lease['deadline_wall_ns']
                or time.time_ns()>=self.deadline or remaining_s<180
                or set(lease.get('gpu_uuids',[]))!=set(c['resources']['exclusive_host_gpu_uuids'])):
            raise PublicSessionError('当前资源时段无效或不足三分钟；需要新的明确资源授权')
        cards=ET.fromstring(subprocess.check_output(['nvidia-smi','-q','-x'],text=True,timeout=10)).findall('gpu')
        if ({g.findtext('uuid') for g in cards}!=set(lease['gpu_uuids'])
                or any(g.findall('processes/process_info') for g in cards)):
            raise PublicSessionError('GPU仍有进程或设备身份不匹配；不会停止其他任务')
        return True

    def prepare(self,generation,remaining_s):
        # Keep each stopped run; immutable catalogs are read directly on restart.
        tag='session-'+str(generation)+'-'+secrets.token_hex(6)
        run=self.root/'runs'/tag;run.mkdir(parents=True,exist_ok=False);output=run/'data'
        c=deepcopy(self.contract);sources=deepcopy(self.sources)
        last=Path(sources['last_output']) if sources.get('last_output') else None
        if last is not None:
            if (last/'calibrations/hardware/active/manifest.json').exists():
                sources['resume']=str(run/'calibration-resume-source.json')
                save(sources['resume'],calibration_source(last,self.base))
            jobs=[];skipped=[]
            for job in sorted(last.glob('collection-*/playground/jobs/*')):
                if not (job/'native-task-verification.json').is_file():continue
                try:history_source(last,[job]);jobs.append(job)
                except (OSError,ValueError,KeyError) as error:skipped.append(dict(job=job.name,reason=str(error)))
            if jobs:
                sources['history']=str(run/'collection-history-source.json')
                save(sources['history'],history_source(last,jobs))
            else:sources['history']=None
            save(run/'history-discovery.json',dict(imported_jobs=[str(p) for p in jobs],not_imported=skipped))
        duration=min(math.floor(remaining_s),math.floor((self.deadline-time.time_ns())/1e9),
                     self.contract['resources'].get('service_lifetime_seconds',86400))
        if duration<180:raise PublicSessionError('本次资源时段剩余不足三分钟')
        spool_root=Path(self.contract['camera_spool_root'])
        spool_root.mkdir(parents=True,exist_ok=True);spool=spool_root/tag
        c['identity']['experiment_id']=tag
        c['filesystem']['writable_roots']=[str(self.code),str(output),str(spool)]
        c['camera_spool_root']=str(spool)
        c['browser_ports']=sorted(set(c.get('browser_ports',[])+self.browser_ports))
        c['resources'].update(service_lifetime_seconds=duration,max_wall_time_minutes=math.ceil(duration/60)+5)
        if sources.get('resume'):
            c['filesystem']['readonly_roots']+=list(json.loads(Path(sources['resume']).read_text())['input_sha256'])
        c['filesystem']['readonly_roots']+=sources['catalogs']
        if sources.get('history'):
            c['filesystem']['readonly_roots'].append(json.loads(Path(sources['history']).read_text())['source_run'])
        argv=list(self.template)
        def replace(flag,value):
            if flag in argv:argv[argv.index(flag)+1]=str(value)
            else:argv.extend([flag,str(value)])
        replace('--base',self.base);replace('--nominal-config',self.nominal)
        replace('--output',output);replace('--port',0);replace('--duration-wall-s',duration)
        replace('--deadline-wall-ns',self.deadline);replace('--collection-contract',run/'contract.json')
        if 'timeout' in argv:argv[argv.index('timeout')+3]=str(duration+45)
        for flag,name in [('--calibration-resume-source','resume'),('--collection-history-source','history')]:
            if flag in argv:
                index=argv.index(flag);del argv[index:index+2]
            if sources.get(name):argv.extend([flag,sources[name]])
        for catalog in sources['catalogs']:argv.extend(['--collection-history-catalog',catalog])
        c['execution']['smoke_commands']=[shlex.join(argv)]
        c['execution']['stop_conditions']=['absolute manager/child deadline '+str(self.deadline),'explicit user stop; no automatic restart']
        save(run/'contract.json',c);save(run/'sources.json',sources)
        for name,digest in c['provenance']['code_sha256'].items():
            path=Path(name) if Path(name).is_absolute() else self.code/name
            if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:raise ValueError('frozen code changed: '+name)
        with (run/'preflight.log').open('xb') as log:
            subprocess.run(argv+['--dry-run'],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=min(120,duration))
        return dict(argv=argv,cwd=str(self.code),run_path=str(run),output_path=str(output),
                    service_path=str(output/'service.json'),log_path=str(run/'resident.log'))

    def stopped(self,spec,receipt):
        if spec is None:return
        run=Path(spec['run_path']);save(run/'cleanup.json',receipt)
        if not receipt['all_owned_exited']:return
        output=Path(spec['output_path'])
        self.sources=json.loads((run/'sources.json').read_text())
        if (output/'run.json').exists():
            self.sources['last_output']=str(output)
            catalog=output/'collection-history'
            if (catalog/'history.json').is_file() and str(catalog) not in self.sources['catalogs']:
                self.sources['catalogs'].append(str(catalog))
        save(self.source_file,self.sources)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('contract','base','nominal-config','state-dir','lease'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--calibration-resume-source',type=Path);p.add_argument('--collection-history-source',type=Path)
    p.add_argument('--deadline-wall-ns',type=int,required=True)
    p.add_argument('--manager-deadline-wall-ns',type=int,required=True,help='independent absolute CPU entry deadline')
    p.add_argument('--port',type=int,default=43759);p.add_argument('--browser-port',type=int,action='append',default=[])
    a=p.parse_args()
    now=time.time_ns()
    if not now<a.deadline_wall_ns<=a.manager_deadline_wall_ns<=now+86400*10**9:p.error('finite authorized lifetimes required')
    profile=FullFakeProfile(contract=a.contract,base=a.base,nominal=a.nominal_config,state_dir=a.state_dir,
        lease=a.lease,deadline_wall_ns=a.deadline_wall_ns,resume=a.calibration_resume_source,
        history=a.collection_history_source,browser_ports=a.browser_port)
    manager=FakeSession(profile.prepare,state_dir=a.state_dir,deadline_wall_ns=a.deadline_wall_ns,
        profile='full_fake',start_guard=profile.guard,on_stopped=profile.stopped,grace_s=30)
    server=serve_session(manager,port=a.port,browser_ports=a.browser_port)
    profile.browser_ports.append(server.server_port)
    save(a.state_dir/'manager-service.json',dict(pid=os.getpid(),port=server.server_port,
        deadline_wall_ns=a.deadline_wall_ns,manager_deadline_wall_ns=a.manager_deadline_wall_ns,
        session_id=profile.contract['identity']['session_id'],autostart=False))
    done=threading.Event()
    def stop(*_):done.set()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:done.wait(max(0,(a.manager_deadline_wall_ns-time.time_ns())/1e9))
    finally:manager.close();server.shutdown();server.server_close();thread.join(2)


if __name__=='__main__':main()
