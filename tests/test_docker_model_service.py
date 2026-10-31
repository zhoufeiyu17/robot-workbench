"""Docker adapter identity/exit contracts. CPU fixtures, no Docker or robot calls."""
from copy import deepcopy
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from robot_workbench.docker_model_service import DockerModelService,DockerModelSpec


class DockerModelTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        script=self.root/'scripts/serve_base566_model.py';script.parent.mkdir();script.write_text('# CPU fixture')
        self.cid='2'*64;self.image='sha256:'+'1'*64;self.name='pi05-arx13-resident-cpu-test'
        ready=str(self.root/'ready.json');cid=str(self.root/'container.cid')
        command=['/usr/bin/docker','run','--detach','--rm','--name',self.name,'--cidfile',cid,
            '--pid','host','--network','host','--user',f'{os.getuid()}:{os.getgid()}',
            '--read-only','--cap-drop','ALL','--security-opt','no-new-privileges',self.image,str(script),'--ready-file',ready]
        self.spec=DockerModelSpec(name='model',kind='docker_model',expected_script=str(script),identity_path=str(self.root/'identity.json'),
            ready_path=ready,cid_path=cid,container_name=self.name,image_id=self.image,command=command,start_timeout_s=.1,stop_timeout_s=.04,poll_s=.001)
        self.service=DockerModelService(self.spec);self.running=False;self.rows={};self.calls=[];self.orphan=False
        self.data=dict(Id=self.cid,Name='/'+self.name,Image=self.image,Config=dict(User=f'{os.getuid()}:{os.getgid()}'),
            HostConfig=dict(PidMode='host',NetworkMode='host',Privileged=False,ReadonlyRootfs=True,CapDrop=['ALL'],SecurityOpt=['no-new-privileges'],DeviceRequests=[]),
            State=dict(Running=True,Pid=10))
        self.addCleanup(patch.stopall)
        patch('robot_workbench.docker_model_service.process_identity',side_effect=lambda p:deepcopy(self.rows.get(p))).start()
        patch('robot_workbench.managed_services.process_identity',side_effect=lambda p:deepcopy(self.rows.get(p))).start()
        patch('robot_workbench.managed_services._boot_id',return_value='boot').start()
        self.service._in_container=lambda p,c:p in self.rows and c==self.cid
        self.service._descendants=lambda root:[deepcopy(v)for p,v in self.rows.items()if p!=root['pid']]
        def docker(*args):
            if args[0]=='inspect':return SimpleNamespace(returncode=0 if self.running else 1,stdout=json.dumps([self.data])if self.running else'',stderr=''if self.running else'No such object')
            self.calls.append(args)
            if args[0]=='kill':
                self.running=False;self.rows={p:v for p,v in self.rows.items()if self.orphan and p==12}
            return SimpleNamespace(returncode=0,stdout='',stderr='')
        self.service._docker=docker

    def launch(self,*args,**kwargs):
        self.running=True
        for pid,ppid,script in ((10,1,'/isaac-sim/python.sh'),(11,10,self.spec.expected_script),(12,11,'/python/model-worker.py')):
            self.rows[pid]=dict(pid=pid,ppid=ppid,start_ticks=str(pid*100),pgid=10,sid=10,uid=os.getuid(),boot_id='boot',process_state='S',script=script)
        Path(self.spec.cid_path).write_text(self.cid)
        Path(self.spec.ready_path).write_text(json.dumps(dict(pid=11,start_ticks=1100,boot_id='boot',uid=os.getuid(),script=self.spec.expected_script,loaded=False)))
        for name in (self.spec.cid_path,self.spec.ready_path):Path(name).chmod(0o600)  # Emulate the real child umask=0077, including group-writable test shells.
        return SimpleNamespace(returncode=0,stdout=self.cid+'\n',stderr='')

    def start(self):
        with patch('robot_workbench.docker_model_service.subprocess.run',side_effect=self.launch)as spawn:
            result=self.service.start()
            self.assertEqual(spawn.call_args.kwargs['umask'],0o077)
            return result

    def test_missing_container_cli_case_does_not_hide_real_inspect_errors(self):
        service=DockerModelService(self.spec)
        for message in ('No such object: test','error: no such object: test','No such container: test'):
            with self.subTest(message=message),patch.object(service,'_docker',return_value=SimpleNamespace(returncode=1,stderr=message)):
                self.assertIsNone(service._inspect())
        with patch.object(service,'_docker',return_value=SimpleNamespace(returncode=1,stderr='permission denied')):
            with self.assertRaises(OSError):service._inspect()

    def test_real_service_pid_not_driver_or_container_init_and_exact_cid_term(self):
        view=self.start();self.assertEqual(view['state'],'running',view);self.assertEqual(view['identity']['pid'],11)
        self.assertEqual(view['container_init']['pid'],10)
        self.assertEqual(self.service.track_child(12)['tracked_child']['pid'],12)
        stopped=self.service.stop();self.assertTrue(stopped['exit_verified'],stopped)
        self.assertEqual(self.calls,[('kill','--signal=TERM',self.cid)])
        self.assertEqual(self.service.snapshot()['state'],'stopped')

    def test_wrong_image_namespace_and_ready_identity_are_never_signalled(self):
        for changed in ('image','namespace','ready'):
            with self.subTest(changed=changed):
                self.launch();self.calls=[];data=deepcopy(self.data)
                if changed=='image':self.data['Image']='sha256:'+'f'*64
                elif changed=='namespace':self.data['HostConfig']['PidMode']='private'
                else:
                    value=json.loads(Path(self.spec.ready_path).read_text());value['start_ticks']=999;Path(self.spec.ready_path).write_text(json.dumps(value))
                self.assertEqual(self.service.stop()['state'],'unknown');self.assertEqual(self.calls,[]);self.data=data

    def test_worker_exit_unconfirmed_blocks_restart_without_force_kill(self):
        self.start();self.service.track_child(12);self.orphan=True
        stopped=self.service.stop();self.assertEqual(stopped['state'],'unknown')
        with patch('robot_workbench.docker_model_service.subprocess.run')as spawn:
            self.assertEqual(self.service.start()['state'],'unknown');spawn.assert_not_called()
        self.assertEqual(self.calls,[('kill','--signal=TERM',self.cid)])

    def test_unrelated_reported_worker_and_unregistered_receipt_fail_closed(self):
        self.start();self.rows[99]=dict(self.rows[12],pid=99,ppid=1,start_ticks='9900')
        self.assertEqual(self.service.track_child(99)['state'],'unknown')
        other=DockerModelService(self.spec)
        with patch.object(other,'_inspect',return_value=None),patch.object(other,'_tracked_live',return_value=[]):
            self.rows={};self.assertEqual(other.snapshot()['state'],'stopped')
        args=vars(self.spec).copy();args['name']='control_owner'
        with self.assertRaises(ValueError):DockerModelSpec(**args)


if __name__=='__main__':unittest.main()
