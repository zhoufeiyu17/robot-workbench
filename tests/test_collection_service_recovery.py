"""P2 CPU fault fixtures. No SDK, PF_CAN, model, or GPU process is started."""
from copy import deepcopy
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from detector_service import DetectorService,RETIRE_ROOT,probe_service
from planner_service import PlannerService
from robot_workbench.resident_collection import CollectionPreparation


class Role:
    """Protocol fixture only; process numbers are deliberately synthetic."""
    def __init__(self,pid):
        self.config={};self.state=dict(status='ready',pid=pid,process_start_ticks=pid,model_initialized=True)
        self.recoveries=0;self.closes=0;self.probes=0
    def status(self):return deepcopy(self.state)
    def probe(self):
        self.probes+=1
        return dict(service=self.status(),needs_warmup=not self.state['model_initialized'],evidence_type='cpu_protocol_fixture')
    def recover(self,*,restart,active,launch,**kwargs):
        assert active()
        def create():
            self.recoveries+=1
            self.state.update(status='ready',pid=self.state['pid']+10,process_start_ticks=self.state['process_start_ticks']+10,model_initialized=False)
        launch(create)
    def close(self):self.closes+=1;self.state['status']='stopped'


def preparation(root,environment):
    p=CollectionPreparation.__new__(CollectionPreparation)
    installed=dict(verified=True,head_id='head',wrist_id='wrist')
    p.device=NS(environment=environment,maintenance=True,runtime=NS(holder='collection'),workflows={},
        snapshot=lambda _:dict(connected=True,feedback_healthy=True,stationary=True,automatic_enabled=False,sample_age_s=.01))
    p.application=NS(lock=threading.RLock(),generations={environment:0},invalidate_collection_preparation=Mock())
    p.lock=threading.RLock();p.closed=threading.Event();p.busy=False;p.running=False
    p._operation_current=lambda:True;p._lifecycle_admission=lambda _:True
    p.request_generation=0;p.request_app_generation=None;p.identity={'manifest_sha256':'fixture'}
    p.installation=lambda _:deepcopy(p.identity);p.installed_snapshot=lambda:installed
    p.value=dict(status='ready',installed=installed,attempt=0);p.deadline_wall_ns=None;p.persistent_service=True
    p.started=None;p.diagnostic_invalidated=False;p._status=lambda status,**kw:p.value.update(status=status,**kw)
    pg=NS(lock=threading.RLock(),state=dict(status='ready',busy=False,warmup={'status':'ready'},progress={}),persist=Mock())
    def warmup(*args,**kw):pg.state.update(status='ready',busy=False,warmup={'status':'ready'});return {}
    pg.start=Mock(side_effect=warmup)
    p.parts=NS(detector=Role(100),planner=Role(200),scene=Role(300),playground=pg,workflow=NS(),
        configured_path=root/'configured.json',attachment_identity={'config':'original'},pipeline_config=root/'pipeline.json',worker_state_directory=root)
    p._verify_parts=Mock()
    return p


class PreparationRecoveryTest(unittest.TestCase):
    def test_business_preflight_only_checks_existing_preview_units(self):
        import playground_pipeline as pipeline
        cfg=dict(workbench_code='/fixture/wb',native_code='/fixture/native',assets='/fixture/assets')
        with patch.object(pipeline,'load_config',return_value=(cfg,{},{})),patch.object(pipeline,'transport',return_value=['cpu-readonly']),patch('cancellable_process.run'),patch.object(pipeline.subprocess,'run')as unit:
            pipeline.prepare_environment(dict(pipeline_config='/fixture/config',preview_units=['collection-preview-fixture.service']))
        unit.assert_called_once_with(['systemctl','--user','is-active','collection-preview-fixture.service'],check=True,timeout=30,capture_output=True)

    def test_business_warmup_runs_only_jobs_in_both_modes_and_preserves_valid_heat(self):
        from playground import Playground
        from test_playground import Bridge
        from robot_workbench.workflows import CollectionWorkflow
        for environment in ('fake','physical'):
            for failed in (False,True):
                with self.subTest(environment=environment,failed=failed),tempfile.TemporaryDirectory()as folder:
                    p=preparation(Path(folder),environment);p.device.collection_preparation=p
                    p.device.maintenance=False;p._operation_current=p._lifecycle_admission=None
                    if environment=='physical':p.parts.scene=None
                    p.application.reviews={};p.application.add_review=Mock()
                    snapshot=p.device.snapshot
                    p.device.snapshot=lambda business:dict(snapshot(business),command_epoch=1)
                    bridge=Bridge();bridge.connected=True;bridge.config['environment']=environment
                    original=bridge.status
                    def health():
                        value=original()
                        if environment=='fake':value['owner'].update(mode='simulation',environment='fake',evidence_source='fake')
                        return value
                    bridge.status=health
                    def initialization():
                        value=p.parts.detector.status()
                        for role in ('planner','scene'):
                            manager=getattr(p.parts,role,None)
                            if manager:value[role]=manager.status()
                        return value
                    pg=Playground(Path(folder)/'business',bridge,initialization,recover_planner=Mock())
                    p.parts.playground=pg
                    flow=CollectionWorkflow(pg,p.device,p.application);p.parts.workflow=flow;p.device.workflows['collection']=flow
                    failure=dict(stage='scene_setup',native_reason='cup_table_recontact_or_invalid_support')
                    def warmup(*args):
                        if failed:
                            error=RuntimeError('scene rejected');error.preparation_failure=failure;raise error
                        return dict(model_initialized=True)
                    pipeline=NS(prepare_environment=Mock(return_value=[dict(key='cpu-preflight',status='ready')]),
                        warmup=Mock(side_effect=warmup),restore_plan=Mock(return_value=None))
                    try:
                        p.parts.detector.state['status']='stopped'
                        with self.assertRaisesRegex(ValueError,'服务'):flow.action('collection_prepare',{})
                        p.parts.detector.state['status']='ready';runtime=p.device.runtime;p.device.runtime=None
                        with self.assertRaisesRegex(ValueError,'接入'):flow.action('collection_prepare',{})
                        p.device.runtime=runtime
                        with patch.dict('sys.modules',playground_pipeline=pipeline):
                            result=flow.action('collection_prepare',{});pg.thread.join(3)
                            self.assertFalse(pg.thread.is_alive());self.assertEqual(result['action'],'prepare')
                            self.assertEqual(pg.state['warmup']['status'],'failed' if failed else 'ready')
                            self.assertEqual(p.environment_status()['ready'],not failed)
                            if failed:self.assertEqual(pg.state['warmup']['failure'],failure)
                            else:
                                pg.state['plan']={'validated':True,'timeline_digest':'retained'}
                                self.assertTrue(flow.action('collection_prepare',{})['reused'])
                            p.device.maintenance=True;p._operation_current=lambda:True;p._lifecycle_admission=lambda _:True
                            self.assertTrue(p._reuse_parts(p.identity,p.request_fence(p.installed_snapshot(),p.identity)))
                        pipeline.warmup.assert_called_once();pipeline.prepare_environment.assert_called_once()
                        if not failed:self.assertEqual(pg.state['plan']['timeline_digest'],'retained')
                        self.assertTrue(p.environment_status()['services_ready']);pg.recover_planner.assert_not_called()
                        self.assertEqual(bridge.commands,[])
                        for role in ('detector','planner','scene'):
                            manager=getattr(p.parts,role,None)
                            if manager:self.assertEqual((manager.recoveries,manager.closes),(0,0))
                    finally:
                        flow.closed.set();flow.thread.join(2)
                        if pg.thread:pg.thread.join(3)

    def test_service_readiness_and_failed_warmup_remain_separate_in_both_modes(self):
        for environment in ('fake','physical'):
            with self.subTest(environment=environment),tempfile.TemporaryDirectory()as folder:
                p=preparation(Path(folder),environment)
                if environment=='physical':p.parts.scene=None
                p.parts.planner.state['model_initialized']=False;p.value['status']='failed'
                value=p.environment_status()
                self.assertTrue(value['services_ready']);self.assertFalse(value['ready'])
                self.assertEqual(value['declared_roles'],['detector','planner']+(['scene']if environment=='fake'else[]))
                failure=dict(stage='scene_setup',native_status='failed',native_phase='setup',
                    native_reason='reset characterization rejected: cup_table_recontact_or_invalid_support')
                p.parts.playground.state.update(status='failed',busy=False,error='scene rejected',
                    warmup=dict(status='failed',failure=failure))
                current=p.request_fence(p.installed_snapshot(),p.identity)
                self.assertTrue(p._reuse_parts(p.identity,current))
                self.assertEqual(p.environment_status()['warmup']['failure'],failure)
                p.parts.playground.start.assert_not_called()
                self.assertFalse(p.environment_status()['ready'])
                p.parts.detector.state['status']='unknown'
                self.assertFalse(p.environment_status()['services_ready'])

    def test_healthy_reuse_and_failed_planner_keep_other_workers_in_both_modes(self):
        for environment in ('fake','physical'):
            with self.subTest(environment=environment),tempfile.TemporaryDirectory()as folder:
                p=preparation(Path(folder),environment);current=p.request_fence(p.installed_snapshot(),p.identity)
                self.assertTrue(p._reuse_parts(p.identity,current))
                self.assertEqual([x.recoveries for x in (p.parts.detector,p.parts.planner,p.parts.scene)],[0,0,0])
                self.assertTrue(all(x['outcome']=='reused'for x in p.value['service_steps']))
                p.parts.planner.state['status']='failed';scene=deepcopy(p.parts.scene.state)
                self.assertTrue(p._reuse_parts(p.identity,current))
                self.assertEqual(p.parts.planner.recoveries,1)
                self.assertEqual(p.parts.scene.state,scene)
                self.assertEqual(p.parts.detector.closes+p.parts.scene.closes,0)
                self.assertEqual(p.value['service_steps'][1]['outcome'],'recovered')

    def test_explicit_role_restart_is_not_collection_ready_and_keeps_scene_hot(self):
        with tempfile.TemporaryDirectory()as folder:
            p=preparation(Path(folder),'physical');scene=deepcopy(p.parts.scene.state)
            # A distinct maintenance request has a fresh application generation;
            # it must not inherit the queued preparation's old generation.
            p.request_app_generation=0;p.application.generations['physical']=1
            p._operation_current=None
            result=p.operations_prepare(still_current=lambda:True,lifecycle_admission=lambda _:True,service_role='planner',restart=True)
            self.assertTrue(result['service_ready']);self.assertTrue(result['needs_warmup']);self.assertFalse(result['ready'])
            self.assertFalse(result['environment']['ready']);self.assertEqual(p.parts.scene.state,scene)
            self.assertFalse(p.parts.playground.state['plan']['validated'])

    def test_unknown_or_changed_assets_do_not_rebuild_or_stop_siblings(self):
        with tempfile.TemporaryDirectory()as folder:
            p=preparation(Path(folder),'fake');p.parts.detector.probe=Mock(side_effect=RuntimeError('SSH unknown'))
            with self.assertRaisesRegex(RuntimeError,'SSH unknown'):p._reuse_parts(p.identity,p.request_fence(p.installed_snapshot(),p.identity))
            self.assertEqual(p.parts.detector.closes+p.parts.planner.closes+p.parts.scene.closes,0)
            del p._verify_parts
            with patch('robot_workbench.collection_setup.prepare_collection_attachment_process',return_value={'attachment_sha256':{'config':'changed'}}):
                with self.assertRaisesRegex(ValueError,'身份已变化'):p._verify_parts(lambda:True)

    def test_managed_recovery_uses_initial_startup_budget_for_a_slow_cold_loader(self):
        with tempfile.TemporaryDirectory()as folder:
            p=preparation(Path(folder),'fake');manager=p.parts.detector;manager.state['status']='failed'
            def slow_loader(*,timeout=180,launch,**kwargs):
                if timeout<200:raise TimeoutError('CPU fixture: cold loader needs200s; initial start allows240s')
                launch(lambda:manager.state.update(status='ready',pid=110,process_start_ticks=110))
            manager.recover=slow_loader
            result=p._ensure_role('detector',p.request_fence(p.installed_snapshot(),p.identity))
            self.assertEqual(result['outcome'],'recovered');self.assertEqual(manager.closes,0)
            self.assertEqual(p.parts.planner.closes+p.parts.scene.closes,0)

    def test_busy_timeout_preserves_the_original_worker(self):
        with tempfile.TemporaryDirectory()as folder:
            p=preparation(Path(folder),'fake');worker=p.parts.detector;worker.state['status']='busy'
            worker.recover=Mock(side_effect=TimeoutError('busy fixture'))
            with self.assertRaises(TimeoutError):p._ensure_role('detector',p.request_fence(p.installed_snapshot(),p.identity))
            self.assertEqual(worker.closes,0);self.assertEqual(worker.state['pid'],100)
            self.assertEqual(p.value['service_steps'][-1]['outcome'],'unknown')

    def test_spawn_gate_is_short_and_stop_before_it_creates_nothing(self):
        with tempfile.TemporaryDirectory()as folder:
            p=preparation(Path(folder),'fake');current=p.request_fence(p.installed_snapshot(),p.identity);gate=p._spawn_gate('planner',current)
            self.assertEqual(gate(lambda:p.application.lock._is_owned()),True)
            p.request_generation+=1;spawn=Mock()
            with self.assertRaises(InterruptedError):gate(spawn)
            spawn.assert_not_called()

    def test_failed_spawn_registration_closes_only_this_started_role(self):
        with tempfile.TemporaryDirectory()as folder:
            p=preparation(Path(folder),'fake');worker=p.parts.planner
            def recover(*,launch,**_):
                def create():raise RuntimeError('registration fixture')
                launch(create)
            worker.recover=recover
            with self.assertRaisesRegex(RuntimeError,'registration fixture'):
                p._ensure_role('planner',p.request_fence(p.installed_snapshot(),p.identity),restart=True)
            self.assertEqual(worker.closes,1);self.assertEqual(p.parts.detector.closes+p.parts.scene.closes,0)

    def test_managed_warmup_cannot_restart_a_worker_outside_operations(self):
        from playground import Playground
        pg=Playground.__new__(Playground);pg.cancel_event=threading.Event();pg.managed_lifecycle=True
        pg.lifecycle_admission=lambda _:True;pg.recover_planner=Mock()
        pg.initialization=lambda:dict(status='ready',planner={'status':'failed'})
        with self.assertRaisesRegex(ValueError,'不在业务预热中自动重启'):pg.prepare_services(Mock(),{})
        pg.recover_planner.assert_not_called()

    def test_stop_during_planner_retirement_prevents_gpu_wait_and_start(self):
        manager=PlannerService();manager.config_path=Path('/unused');manager.directory=Path('/unused')
        manager.state={'status':'ready'};active=[True]
        with patch.object(manager,'close',side_effect=lambda:active.__setitem__(0,False)),patch.object(manager,'start')as start,patch('detector_service.wait_for_gpu_idle')as wait:
            with self.assertRaises(InterruptedError):manager.recover(restart=True,active=lambda:active[0])
            start.assert_not_called();wait.assert_not_called()


class DetectorReaderTest(unittest.TestCase):
    def test_new_probe_uses_stdin_and_foreign_gpu_failure_never_launches(self):
        token='fixture-private-probe-nonce'
        config=dict(workbench_code='/remote/workbench',detector_service={'owner_token':token})
        with patch('run_pipeline.transport',return_value=['cpu-probe'])as transport,patch('subprocess.run',return_value=NS(returncode=1,stderr='selected GPU has an unowned process '+token))as run,patch('subprocess.Popen')as spawn:
            with self.assertRaisesRegex(RuntimeError,'unowned process')as caught:probe_service(config,'detector')
            self.assertNotIn(token,str(caught.exception));self.assertNotIn(token,transport.call_args.args[2])
            self.assertEqual(json.loads(run.call_args.kwargs['input'])['detector_service']['owner_token'],token)
            spawn.assert_not_called()

    def test_late_reader_is_fenced_and_join_timeout_never_reuses_root(self):
        manager=DetectorService();released=threading.Event();reading=threading.Event()
        spec=dict(root='/unused/service',socket='/unused/service/d',owner_token='private-fixture-capability-12345',identity={})
        value=dict(owner_token=spec['owner_token'],identity={},service_root=spec['root'],socket=spec['socket'],status='ready',initialization={})
        class Late:
            def __iter__(self):
                reading.set();released.wait(3);yield json.dumps(value)+'\n'
        child=NS(stdout=Late(),stdin=io.StringIO(),wait=lambda timeout=None:0)
        with tempfile.TemporaryDirectory()as folder,patch.object(manager,'_wait_for_gpu',return_value=True),patch('detector_service.launch_command',return_value='fixture'),patch('subprocess.Popen',return_value=child),patch('subprocess.run',return_value=NS(returncode=0)):
            t=threading.Thread(target=manager._run,args=({'detector_service':spec},{'provenance':{}},Path(folder)));manager._thread=t;t.start()
            self.assertTrue(reading.wait(2));join=t.join
            with patch.object(t,'join',return_value=None):
                with self.assertRaises(TimeoutError):manager.close()
            self.assertEqual(manager.status()['status'],'unknown');released.set();join(2)
            self.assertFalse(t.is_alive());self.assertEqual(manager.status()['status'],'unknown')
            with patch.object(manager,'start')as start:
                with self.assertRaises(RuntimeError):manager.recover(restart=True)
                start.assert_not_called()
            self.assertNotIn(spec['owner_token'],(Path(folder)/'detector-service.stdout.log').read_text())


@unittest.skipUnless(sys.platform=='linux','retirement fixture requires Linux /proc; no GPU or CAN')
class DetectorRetirementTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='rbw-detector-',dir='/tmp');self.addCleanup(self.temp.cleanup)
        self.parent=Path(self.temp.name);self.root=self.parent/'service';self.root.mkdir(mode=0o700)
        self.spec=dict(root=str(self.root),socket=str(self.root/'d'),owner_token='fixture-capability-not-a-real-secret',
            identity={'adapter':'fixture'},allowed_read_roots=[str(self.parent)],allowed_output_root=str(self.parent))
        self.state=dict(**{k:v for k,v in self.spec.items()if k!='root'},service_root=str(self.root),gpu=0,physical_gpu=0,logical_cuda_index=0,
            pid=1000000000,process_start_ticks=1,boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        self.payload=dict(spec=self.spec,physical_gpu=0,writable_roots=[str(self.parent),str(self.root)])
        self.save()
    def save(self):
        (self.root/'status.json').write_text(json.dumps(self.state));(self.root/'diagnostic.log').write_bytes(b'preserve all old evidence\n')
    def run_helper(self):
        args=[sys.executable,'-I','-c',RETIRE_ROOT]
        self.assertNotIn(self.spec['owner_token'],' '.join(args))
        result=subprocess.run(args,input=json.dumps(self.payload)+'\n',text=True,capture_output=True,timeout=5)
        self.assertNotIn(self.spec['owner_token'],result.stdout+result.stderr)
        return result
    def test_dead_scope_is_atomically_archived_and_evidence_is_unchanged(self):
        result=self.run_helper();self.assertEqual(result.returncode,0,result.stderr)
        retired=Path(json.loads(result.stdout)['retired_root']);self.assertFalse(self.root.exists())
        self.assertEqual((retired/'diagnostic.log').read_bytes(),b'preserve all old evidence\n')
        self.assertEqual(json.loads((retired/'status.json').read_text()),self.state)
    def test_same_ticks_live_is_refused_pid_reuse_does_not_signal_new_process(self):
        self.state.update(pid=os.getpid(),process_start_ticks=int(Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19]));self.save()
        self.assertNotEqual(self.run_helper().returncode,0);self.assertTrue(self.root.is_dir())
        self.state['process_start_ticks']+=1;self.save();result=self.run_helper();self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual(json.loads(result.stdout)['observation'],'pid_reused_old_process_exited')
    def test_bad_identity_missing_boot_corrupt_and_symlink_status_never_move(self):
        original=deepcopy(self.state)
        for change in ({'identity':{}},{'boot_id':None},{'physical_gpu':False}):
            self.state={**original,**change};self.save();self.assertNotEqual(self.run_helper().returncode,0);self.assertTrue(self.root.is_dir())
        path=self.root/'status.json';path.write_text('{broken');self.assertNotEqual(self.run_helper().returncode,0)
        path.unlink();path.symlink_to(self.root/'diagnostic.log');self.assertNotEqual(self.run_helper().returncode,0);self.assertTrue(self.root.is_dir())
    def test_rename_failure_and_unreadable_proc_leave_the_original_scope(self):
        def execute():
            with patch('sys.stdin',io.TextIOWrapper(io.BytesIO((json.dumps(self.payload)+'\n').encode()))),patch('sys.stdout',new_callable=io.StringIO):exec(compile(RETIRE_ROOT,'retirement-fixture','exec'),{})
        with patch('os.rename',side_effect=PermissionError('rename fixture')):
            with self.assertRaises(PermissionError):execute()
        read=Path.read_text
        def denied(path,*args,**kwargs):
            if str(path)==f"/proc/{self.state['pid']}/stat":raise PermissionError('proc fixture')
            return read(path,*args,**kwargs)
        with patch.object(Path,'read_text',denied):
            with self.assertRaises(PermissionError):execute()
        self.assertTrue(self.root.is_dir());self.assertFalse(list(self.parent.glob('service-retired-*')))

    def test_scope_symlinks_permissions_and_oversized_status_are_refused(self):
        self.parent.chmod(0o755);self.assertNotEqual(self.run_helper().returncode,0);self.parent.chmod(0o700)
        self.root.chmod(0o755);self.assertNotEqual(self.run_helper().returncode,0);self.root.chmod(0o700)
        alias=self.parent.with_name(self.parent.name+'-alias');alias.symlink_to(self.parent,target_is_directory=True);self.addCleanup(alias.unlink)
        original=deepcopy(self.payload);self.payload['spec']=dict(self.spec,root=str(alias/'service'),socket=str(alias/'service/d'))
        self.payload['writable_roots']=[str(alias),str(alias/'service')]
        self.assertNotEqual(self.run_helper().returncode,0);self.payload=original
        path=self.root/'status.json';path.write_bytes(b'{}'+b' '*65536)
        self.assertNotEqual(self.run_helper().returncode,0);self.assertTrue(self.root.is_dir())


if __name__=='__main__':unittest.main()
