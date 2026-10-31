"""Manage one declared model container using real host PID and immutable CID proofs.

This is not a generic Docker runner. Robot/SDK services retain their existing
process/user-unit adapters. A container starts an inert model HTTP service;
the authenticated prepare operation owns GPU admission separately.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import threading
import time
import uuid

from .managed_services import ManagedService, process_identity, _live, _matches


def option(argv,name):
    values=[]
    for i,value in enumerate(argv):
        if value==name and i+1<len(argv):values.append(argv[i+1])
        elif value.startswith(name+'='):values.append(value.split('=',1)[1])
    return values


@dataclass(frozen=True)
class DockerModelSpec:
    name: str
    kind: str
    expected_script: str
    identity_path: str
    ready_path: str
    cid_path: str
    container_name: str
    image_id: str
    command: list[str]
    gpu_uuid: str | None = None
    start_timeout_s: float = 30.
    stop_timeout_s: float = 30.
    poll_s: float = .1

    def __post_init__(self):
        if self.name!='model' or self.kind!='docker_model':raise ValueError('container adapter is restricted to the model service')
        paths=(self.expected_script,self.identity_path,self.ready_path,self.cid_path)
        if len(set(paths))!=4 or any(not isinstance(p,str)or not Path(p).is_absolute()or'..'in Path(p).parts for p in paths):
            raise ValueError('distinct absolute model script/identity/ready/CID paths required')
        if not self.expected_script.endswith('/serve_base566_model.py'):raise ValueError('declared Base566 model service entry required')
        if not re.fullmatch(r'pi05-arx13-resident-[a-z0-9-]+',self.container_name):raise ValueError('fixed owned model container name required')
        if not re.fullmatch(r'sha256:[a-f0-9]{64}',self.image_id):raise ValueError('immutable model container image ID required')
        a=self.command
        if (not isinstance(a,list)or len(a)<3 or any(not isinstance(v,str)or not v or'\0'in v for v in a)
                or not Path(a[0]).is_absolute()or Path(a[0]).name!='docker'or a[1]!='run'
                or '--detach'not in a or '--privileged'in a or '--read-only'not in a
                or option(a,'--name')!=[self.container_name]or option(a,'--cidfile')!=[self.cid_path]
                or option(a,'--pid')!=['host']or option(a,'--network')!=['host']
                or option(a,'--user')!=[f'{os.getuid()}:{os.getgid()}']
                or option(a,'--cap-drop')!=['ALL']or 'no-new-privileges'not in option(a,'--security-opt')
                or option(a,'--ready-file')!=[self.ready_path]or self.image_id not in a or self.expected_script not in a):
            raise ValueError('bounded declared Docker model launch argv required')
        if option(a,'--gpus') != ([] if self.gpu_uuid is None else ['device='+self.gpu_uuid]):
            raise ValueError('model GPU request must match the explicit UUID or CPU-only configuration')
        if any(type(x)not in(int,float)or not 0<x<=120 for x in(self.start_timeout_s,self.stop_timeout_s,self.poll_s)):
            raise ValueError('bounded lifecycle deadlines required')


class DockerModelService(ManagedService):
    def __init__(self,spec):
        if not isinstance(spec,DockerModelSpec):raise TypeError('validated DockerModelSpec required')
        self.spec=spec;self.path=Path(spec.identity_path);self.child=None;self._mutation=threading.Lock()
        self.fingerprint=hashlib.sha256(json.dumps(asdict(spec),sort_keys=True).encode()).hexdigest()

    def _docker(self,*args):
        return subprocess.run([self.spec.command[0],*args],stdin=subprocess.DEVNULL,capture_output=True,text=True,timeout=10)

    def _inspect(self):
        p=self._docker('inspect',self.spec.container_name)
        if p.returncode:
            if 'no such object' in p.stderr.lower() or 'no such container' in p.stderr.lower():return None
            raise OSError('Docker could not verify the declared model container')
        if len(p.stdout)>1024*1024:raise ValueError('container inspection exceeds bound')
        rows=json.loads(p.stdout)
        if not isinstance(rows,list)or len(rows)!=1:raise ValueError('one exact model container required')
        value=rows[0];cid=value.get('Id');host=value.get('HostConfig')or{};config=value.get('Config')or{}
        devices=host.get('DeviceRequests')or[]
        if (not isinstance(cid,str)or not re.fullmatch('[a-f0-9]{64}',cid)
                or value.get('Name')!='/'+self.spec.container_name or value.get('Image')!=self.spec.image_id
                or host.get('PidMode')!='host' or host.get('NetworkMode')!='host'
                or config.get('User')!=f'{os.getuid()}:{os.getgid()}' or host.get('Privileged')is not False
                or host.get('ReadonlyRootfs')is not True or 'ALL'not in [str(x).upper()for x in host.get('CapDrop')or[]]
                or 'no-new-privileges'not in(host.get('SecurityOpt')or[])):
            raise ValueError('model container image, namespace, user or isolation differs')
        if (self.spec.gpu_uuid is None and devices or self.spec.gpu_uuid is not None
                and (len(devices)!=1 or devices[0].get('DeviceIDs')!=[self.spec.gpu_uuid])):
            raise ValueError('observed container GPU assignment differs')
        return value

    def _receipt_file(self,path):
        fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
        with os.fdopen(fd)as f:
            info=os.fstat(f.fileno())
            if not stat.S_ISREG(info.st_mode)or info.st_uid!=os.getuid()or info.st_mode&0o022 or info.st_size>65536:
                raise ValueError('owned bounded model receipt required')
            return f.read(65537)

    def _in_container(self,pid,cid):
        return cid in (Path('/proc')/str(pid)/'cgroup').read_text()

    def _descendant(self,pid,parent):
        seen=set()
        while pid>1 and pid not in seen:
            if pid==parent:return True
            seen.add(pid);row=process_identity(pid)
            if row is None:return False
            pid=row['ppid']
        return False

    def snapshot(self):
        try:
            saved=self._load();value=self._inspect();historical=(saved or{}).get('identity')
            if value is None or not value.get('State',{}).get('Running'):
                old=process_identity(historical['pid'])if historical else None
                children=self._tracked_live(saved)
                if _live(old)or children:return self._unknown('model container is absent/stopped but recorded processes still live',running=True,identity=old)
                if saved and saved.get('phase')=='starting':return self._unknown('model start outcome is not verified; no retry',running=None)
                return self._view('stopped','Declared model container and tracked processes are stopped.',running=False,owned=bool(saved),identity=historical,container_id=value['Id']if value else None)
            cid=value['Id']
            if self._receipt_file(self.spec.cid_path).strip()!=cid:raise ValueError('model CID receipt differs from inspected container')
            ready=json.loads(self._receipt_file(self.spec.ready_path));identity=process_identity(ready['pid'])
            init=process_identity(value['State']['Pid'])
            if (not _live(identity)or not _live(init)or identity['script']!=self.spec.expected_script
                    or identity['uid']!=os.getuid()or ready.get('uid')!=identity['uid']
                    or ready.get('boot_id')!=identity['boot_id']or str(ready.get('start_ticks'))!=identity['start_ticks']
                    or ready.get('script')!=self.spec.expected_script or not self._descendant(identity['pid'],init['pid'])
                    or not self._in_container(identity['pid'],cid)):
                raise ValueError('model ready PID is not the declared real host/container process')
            identity=dict(identity,container_id=cid)
            if historical and (not _matches(identity,historical)or historical.get('container_id')!=cid):
                raise ValueError('model process/CID changed; no new instance was adopted')
            if saved and saved.get('phase')in('stopping','unknown'):
                return self._unknown(saved.get('reason')or'model stop outcome unconfirmed',running=True,owned=True,
                                     identity=identity,container_id=cid,container_init=init)
            return self._view('running','Exact image/CID and real host service PID verified.',running=True,owned=True,
                              identity=identity,container_id=cid,container_init=init)
        except (OSError,ValueError,KeyError,subprocess.SubprocessError)as e:return self._unknown(str(e))

    def _start(self):
        before=self.snapshot()
        if before['state']!='stopped':return before
        saved=self._load();old_cid=before.get('container_id')
        if old_cid:
            if not saved or(saved.get('identity')or{}).get('container_id')!=old_cid:return self._unknown('unregistered stopped container; not removed')
            p=self._docker('rm',old_cid)
            if p.returncode and self._inspect() is not None:raise OSError('prior owned model container removal unconfirmed')
        for filename in(self.spec.ready_path,self.spec.cid_path):
            p=Path(filename)
            if p.exists():
                if not saved or not saved.get('identity'):return self._unknown('unregistered model receipt; not overwritten')
                self._receipt_file(filename)
                p.rename(p.with_name(p.name+'.previous-'+uuid.uuid4().hex))
            p.parent.mkdir(parents=True,exist_ok=True)
        if not Path(self.spec.expected_script).is_file():raise ValueError('declared model entry script missing')
        self._save('starting',None,'Waiting for a real host PID receipt.',children=[])
        result=subprocess.run(self.spec.command,stdin=subprocess.DEVNULL,capture_output=True,text=True,
                              timeout=self.spec.start_timeout_s,umask=0o077)
        if result.returncode or not re.fullmatch('[a-f0-9]{64}',result.stdout.strip()):
            return self._unknown('model launch outcome not confirmed; no retry or automatic removal')
        cid=result.stdout.strip();deadline=time.monotonic()+self.spec.start_timeout_s
        while time.monotonic()<deadline:
            observed=self.snapshot()
            if observed['state']=='running'and observed.get('container_id')==cid:
                self._save('running',observed['identity']);return observed
            time.sleep(self.spec.poll_s)
        return self._unknown('model ready deadline exceeded; inspect the owned container, no retry')

    def track_child(self,pid):
        def action():
            state=self.snapshot()
            if state['state']!='running'or not state['owned']:return state
            child=process_identity(pid);root=state['identity']
            if (not _live(child)or child['uid']!=os.getuid()or not self._descendant(pid,root['pid'])
                    or not self._in_container(pid,state['container_id'])):
                return self._unknown('reported worker is not a live descendant in this model container')
            children=(self._load()or{}).get('children',[])
            if any(x['pid']==pid and not _matches(x,child)for x in children):raise ValueError('tracked model PID reused')
            if not any(_matches(x,child)for x in children):children.append(child)
            self._save('running',root,children=children);return dict(state,tracked_child=child)
        return self._operate(action)

    def _stop(self):
        before=self.snapshot()
        if before['state']=='stopped':return dict(before,exit_verified=True)
        if before['state']!='running'or not before['owned']:return before
        identity=before['identity'];cid=before['container_id'];children=(self._load()or{}).get('children',[])
        for child in [before['container_init'],*self._descendants(before['container_init'])]:
            if self._in_container(child['pid'],cid)and not any(_matches(x,child)for x in children):children.append(child)
        self._save('stopping',identity,children=children)
        observed=self._inspect()
        if observed is None or observed['Id']!=cid:raise ValueError('model CID changed before stop; not signalled')
        p=self._docker('kill','--signal=TERM',cid)
        if p.returncode and(self._inspect()or{}).get('State',{}).get('Running'):raise OSError('model TERM request unconfirmed')
        deadline=time.monotonic()+self.spec.stop_timeout_s
        while time.monotonic()<deadline:
            state=self.snapshot()
            if state['state']=='stopped' and ('--rm'not in self.spec.command or state.get('container_id')is None):
                self._save('stopped',identity,'Model service and tracked workers exited.',children=children)
                return dict(state,exit_verified=True,children_exited=children)
            time.sleep(self.spec.poll_s)
        self._save('unknown',identity,'Model TERM exit unconfirmed; no SIGKILL sent.',children=children)
        return self._unknown('model exit unconfirmed; no force kill or retry',identity=identity,running=True,owned=True)
