#!/usr/bin/env python3
"""Opt-in real Linux systemd cold-start test using inert services, no sensors/SDK.

Run with --root <fresh task-owned directory> --source <installed workbench source>
--tool <workbench_deploy.py>. Never uses production unit names or network ports.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import shlex
import socket
import subprocess
import sys
import time


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True);ap.add_argument('--source',required=True);ap.add_argument('--tool',required=True);a=ap.parse_args()
    root=Path(a.root);root.mkdir(mode=0o700,parents=True,exist_ok=False);sys.path.insert(0,a.source)
    spec=importlib.util.spec_from_file_location('startup',a.tool);d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
    def script(name,body):
        p=root/(name+'.py');p.write_text(body);return str(p)
    owner=script('owner',f'''import sys,signal,threading
sys.path.insert(0,{a.source!r})
from robot_workbench.service import Device
from robot_workbench.control_owner import ControlOwnerService
if __name__=='__main__':
 service=ControlOwnerService(Device('startup-fixture','physical',None),{str(root/'owner')!r})
 signal.signal(signal.SIGTERM,lambda *_:threading.Thread(target=service.server.shutdown).start())
 try:service.serve_forever()
 finally:service.close()
''')
    cameras=script('cameras',f'''import json,os,signal,threading
from pathlib import Path
stop=threading.Event();signal.signal(signal.SIGTERM,lambda *_:stop.set())
Path({str(root/'camera.json')!r}).write_text(json.dumps(dict(parent_pid=os.getpid())))
stop.wait()
''')
    business=script('business','import signal,threading\ne=threading.Event();signal.signal(signal.SIGTERM,lambda *_:e.set());e.wait()\n')
    sock=socket.socket();sock.bind(('127.0.0.1',0));port=sock.getsockname()[1];sock.close()
    manager=script('manager',f'''import json,time,signal,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*args):pass
 def do_GET(self):
  if self.path.startswith('/api/environment'):
   v=dict(deployment=dict(files_unchanged=True),activity=dict(busy=False))
  else:
   stamp=time.time_ns();v=dict(device=dict(id='startup-fixture'),runtime=dict(connected=False,state='DISCONNECTED',runtime_nonce=None,holder=None,command_epoch=0,automatic_enabled=False,feedback_healthy=False,stationary=False),server_instance_id='CPU-fixture',server_wall_ns=stamp,media=[dict(id=s,ready=True,source_wall_ns=stamp)for s in ('head','left','right')],preview=dict(webgl_model={{'fixture':True}}),manual=dict(limits=[[-1,1]]*12))
  raw=json.dumps(v).encode();self.send_response(200);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
server=ThreadingHTTPServer(('127.0.0.1',{port}),Handler)
signal.signal(signal.SIGTERM,lambda *_:threading.Thread(target=server.shutdown).start())
server.serve_forever();server.server_close()
''')
    scripts=dict(control_owner=owner,cameras=cameras,workbench=business,manager=manager)
    unit_dir=Path.home()/'.config/systemd/user';units={};manifest=dict(schema=d.SCHEMA,uid=os.getuid(),machine_id=Path('/etc/machine-id').read_text().strip(),startup_acquires_sdk=False,root=str(root),source=a.source,device_id='startup-fixture',url='http://127.0.0.1:'+str(port),owner_descriptor=str(root/'owner/control-owner.json'),camera_descriptor=str(root/'camera.json'),conflicts=[],files={p:d.sha(p)for p in scripts.values()},can_interfaces={},services={},ports={'manager':['127.0.0.1',port]})
    result={};managed={}
    try:
        for role,p in scripts.items():
            name='codex-wb-startup-test-'+str(os.getpid())+'-'+role+'.service';path=unit_dir/name;assert not path.exists()
            text='\n'.join(['[Service]','Type=simple','ExecStart='+shlex.join([sys.executable,'-B',p]),'WorkingDirectory='+str(root),'Environment=PYTHONDONTWRITEBYTECODE=1','Environment=OPENBLAS_NUM_THREADS=1','KillSignal=SIGTERM','SendSIGKILL=no','Restart=no','TimeoutStopSec=10',''])
            path.write_text(text);units[name]=text
        subprocess.run(['systemctl','--user','daemon-reload'],check=True,timeout=10)
        for (role,p),name in zip(scripts.items(),units):manifest['services'][role]=dict(kind='user_unit',expected_script=p,identity_path=str(root/(role+'-identity.json')),**d.unit_pin(name))
        d.private_write(root/'release.json',manifest);managed=d.preflight(manifest)
        first=d.start(manifest,managed);assert first['ok'] and first['actions']==['started '+r for r in d.ORDER]
        ids={k:v.snapshot()['identity']for k,v in managed.items()}
        second=d.start(manifest,managed);assert second['ok'] and not second['actions']
        assert ids=={k:v.snapshot()['identity']for k,v in managed.items()}
        result=dict(passed=True,scope='real user systemd + real inert Owner IPC; synthetic HTTP/camera metadata; no robot hardware',first=first,repeat=second,identities_preserved=True)
    finally:
        cleanup={}
        for role in reversed(d.ORDER):
            if role in managed:
                view=managed[role].snapshot()
                if view['state']=='running':
                    stopped=managed[role].stop();cleanup[role]=stopped['state'];assert stopped['state']=='stopped',stopped
        for name,text in units.items():
            p=unit_dir/name
            st=d.unit_state(name);assert st['MainPID']=='0' and st['ControlPID']=='0' and p.read_text()==text
            p.unlink()
        subprocess.run(['systemctl','--user','daemon-reload'],check=True,timeout=10)
        result['test_unit_cleanup']=cleanup
        d.private_write(root/'result.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
