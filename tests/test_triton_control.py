"""Control-plane lifecycle and denial tests: no GPU, Docker or robot calls."""
from datetime import datetime, timezone, timedelta
import fcntl
import json
from pathlib import Path
import threading
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from robot_workbench.triton_control import ControlPlane, Rejected, save, sha
from robot_workbench.gpu_admission import lock_path


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.root=Path(self.temp.name).resolve()
        self.config=dict(workspace=str(self.root),owner_session='owner',gpu_uuid='GPU-fixture',image='sha256:fixture',workbench_url='http://127.0.0.1:1')
        self.plane=ControlPlane(self.config);self.allocation()

    def tearDown(self):self.plane.close();self.temp.cleanup()

    def allocation(self,**changes):
        now=datetime.now(timezone.utc)
        row=dict(session_id='owner',run_id='lease1',gpu_uuid='GPU-fixture',updated_at=now.isoformat(),expires_at=(now+timedelta(hours=1)).isoformat(),jobs_enabled=False)
        row.update(changes);save(self.root/'gpu-allocation.json',row)

    def test_closed_stale_short_wrong_owner_and_missing_leases_never_launch(self):
        now=datetime.now(timezone.utc)
        with patch('robot_workbench.triton_control.subprocess.Popen')as spawn:
            for changes in ({},{'jobs_enabled':True,'updated_at':(now-timedelta(hours=2)).isoformat()},
                            {'jobs_enabled':True,'expires_at':(now+timedelta(minutes=19)).isoformat()},
                            {'jobs_enabled':True,'session_id':'other'}):
                self.allocation(**changes)
                with self.assertRaises(Rejected):self.plane.start('conversions',dict(source_id='base566-e008',request_id='request1'))
            self.assertFalse(self.plane.jobs);spawn.assert_not_called()

    def test_idempotency_single_job_and_restart_unknown(self):
        self.allocation(jobs_enabled=True);started=threading.Event();finish=threading.Event()
        def fake(job,deployment):started.set();finish.wait(2)
        self.plane._run=fake
        body=dict(source_id='base566-e008',request_id='r1');first=self.plane.start('conversions',body);self.assertTrue(started.wait(1))
        self.assertEqual(self.plane.start('conversions',body)['job_id'],first['job_id'])
        with self.assertRaises(Rejected):self.plane.start('conversions',dict(source_id='other',request_id='r1'))
        with self.assertRaises(Rejected):self.plane.start('conversions',dict(source_id='base566-e008',request_id='r2'))
        finish.set();self.plane.worker.join(2)
        recovered=ControlPlane(self.config);self.assertTrue(recovered.blocked)
        self.assertEqual(recovered.catalog()['recent_jobs'][0]['status'],'UNKNOWN')
        with self.assertRaises(Rejected):recovered.start('conversions',dict(source_id='base566-e008',request_id='r3'))

    def test_failed_artifacts_never_selectable_and_unknown_model_rejected(self):
        folder=self.root/'conversion-source-rounding-r1';folder.mkdir();save(folder/'run.json',dict(schema='manipulation.triton_conversion_run.v1',status='FAILED',error='parity failed'))
        save(folder/'deployment.json',dict(status='READY'))  # Contradictory stray file must not override failure.
        models=self.plane.catalog()['models'];self.assertEqual(models[0]['status'],'FAILED');self.assertFalse(models[0]['selectable'])
        with patch('robot_workbench.triton_control.subprocess.Popen')as spawn:
            with self.assertRaises(Rejected)as error:self.plane.start('infer',dict(model_id=models[0]['id'],request_id='r1'))
            self.assertEqual(error.exception.code,'MODEL_NOT_READY');spawn.assert_not_called()

    def test_no_live_input_does_not_connect_or_allocate(self):
        with patch('robot_workbench.triton_control.urlopen',side_effect=OSError('no input')),patch('robot_workbench.triton_control.subprocess.Popen')as spawn:
            with self.assertRaises(Rejected)as error:self.plane.live_input()
            self.assertEqual(error.exception.code,'INPUT_UNAVAILABLE');spawn.assert_not_called()

    def test_thin_infer_wrapper_receives_job_name_and_cid_without_text_probe(self):
        self.allocation(jobs_enabled=True);scripts=self.root/'source/scripts';scripts.mkdir(parents=True)
        (scripts/'infer_pi05_triton.sh').write_text('exec bash deploy_pi05_triton.sh --infer "$@"')
        (scripts/'deploy_pi05_triton.sh').write_text('# shared wrapper')
        folder=self.plane.jobs_root/('a'*32);folder.mkdir();output=folder/'output';output.mkdir()
        deployment=self.root/'deployment.json';save(deployment,dict(status='READY'))
        result=dict(hardware_control=False,command_publication=False,deployment_sha256=sha(deployment),
                    prediction=dict(shape=[60,14],joint_units='rad',gripper_units='m',values=[[0.]*14 for _ in range(60)]))
        save(output/'result.json',result)
        job=dict(job_id='a'*32,type='infer',source_id='model1')
        child=SimpleNamespace(pid=12345,poll=lambda:0,returncode=0)
        def command(args,**kwargs):
            return SimpleNamespace(returncode=1 if args[:2]==['docker','inspect'] else 0,stdout='',stderr='')
        with patch('robot_workbench.gpu_admission.acquire',return_value=SimpleNamespace(close=lambda:None)),patch.object(self.plane,'live_input',return_value={}),patch('robot_workbench.triton_control.subprocess.run',side_effect=command),patch('robot_workbench.triton_control.subprocess.Popen',return_value=child)as spawn:
            self.plane._run(job,deployment)
        self.assertEqual(job['status'],'SUCCEEDED',job);self.assertTrue(job['cleanup_confirmed'])
        args=spawn.call_args.args[0];env=spawn.call_args.kwargs['env']
        self.assertTrue(args[1].endswith('/infer_pi05_triton.sh'))
        self.assertEqual(env['MANIPULATION_JOB_CONTAINER_NAME'],'pi05-arx13-'+'a'*32)
        self.assertEqual(env['MANIPULATION_JOB_CIDFILE'],str(folder/'container.cid'))

    def test_resident_flock_blocks_conversion_without_inspecting_or_stopping_gpu(self):
        self.allocation(jobs_enabled=True);folder=self.plane.jobs_root/'blocked';folder.mkdir()
        job=dict(job_id='blocked',type='conversions',source_id='base566-e008')
        with patch.dict('os.environ',WORKBENCH_GPU_LOCK_DIR=str(self.root/'gpu-locks')), lock_path('GPU-fixture').open('a+b')as lock:
            lock_path('GPU-fixture').chmod(0o600)
            fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
            with patch('robot_workbench.triton_control.subprocess.run')as run,patch('robot_workbench.triton_control.subprocess.Popen')as spawn:
                self.plane._run(job,None)
                self.assertEqual(job['error']['code'],'GPU_BUSY');run.assert_not_called();spawn.assert_not_called()
        self.assertTrue(job['cleanup_confirmed'])

    def test_private_candidate_source_stays_in_workspace(self):
        configured=ControlPlane(dict(self.config,source_root=str(self.root/'source-resident-v1')))
        self.assertEqual(configured.source,self.root/'source-resident-v1');configured.close()
        with self.assertRaises(ValueError):ControlPlane(dict(self.config,source_root='/outside/source'))


if __name__=='__main__':unittest.main()
