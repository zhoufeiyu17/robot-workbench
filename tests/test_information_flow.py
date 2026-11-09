"""Construct past information-flow failures without hardware, weights or motion."""
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch,Mock

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench')]
from robot_workbench.management import WorkbenchManager,private_save
from robot_workbench.operations import OperationService,Operation,OperationError
from robot_workbench.maintenance import operate,target
from robot_workbench.machine_profile import Deployment


class InformationFlowTests(unittest.TestCase):
    def test_common_publication_closure_restores_warm_binding_without_warmup_or_control(self):
        import test_management
        harness=test_management.ManagementTests();harness.setUp();self.addCleanup(harness.doCleanups)
        h=harness.h;manager=h['manager'];model=h['services']['model'];backend=h['services']['workbench']
        test_management.json_request(model.port,'/warm',{})
        manager.refresh();binding=manager.warm_model_binding(manager.state())
        before=deepcopy(binding['evidence']);backend.restart();manager.refresh()
        result=manager.complete_publication(binding,lambda *a,**k:None)
        self.assertEqual(result['status'],'restored');self.assertEqual(result['evidence']['worker_pid'],before['worker_pid'])
        self.assertEqual(test_management.json_request(model.port,'/fixture')['events'].count('warm'),1)
        self.assertNotIn('connect_control',test_management.json_request(backend.port,'/fixture')['events'])
        self.assertFalse(manager.state()['runtime']['connected'])
        self.assertTrue(manager.state()['model']['model_ready'])

    def test_lost_response_queries_same_result_without_replaying_and_fences_old_instance(self):
        import test_maintenance
        harness=test_maintenance.MaintenanceTests();harness.setUp();self.addCleanup(harness.doCleanups)
        app,device=harness.app,harness.device
        with tempfile.TemporaryDirectory() as folder:
            manager=WorkbenchManager.__new__(WorkbenchManager)
            manager.environment='fake';manager.port=12345;manager.timeouts={'state':.2}
            manager.window_path=Path(folder)/'window.json'
            manager.state=lambda:dict(server_instance_id=app.server_instance_id,device={'id':device.id},
                runtime=device.runtime.fact_snapshot(),csrf='cpu-fixture')
            ticket=operate(app,device,'begin',dict(environment='fake',request_id='claim',purpose='prepare_model',expected_target=target(app,device)))
            manager.save_window(dict(environment='fake',device_id=device.id,backend_instance=app.server_instance_id,
                expected_target=target(app,device),token=ticket['token']))
            calls=[]
            def request(_port,path,body,**kwargs):
                action=path.rsplit('/',1)[-1];calls.append(action)
                result=operate(app,device,action,body)
                if action=='prepare_model':raise ConnectionResetError('injected lost completed reply')
                return result
            with patch('robot_workbench.management.json_request',side_effect=request):
                result=manager.post('ops_prepare_model',token=ticket['token'])
            self.assertTrue(result['model_ready']);self.assertEqual(calls,['prepare_model','receipt'])
            self.assertEqual(len([c for c in harness.flow.calls if isinstance(c,tuple)]),1)
            self.assertIsNone(manager.window['pending_call'])
            manager.save_window(dict(manager.window,pending_call=dict(action='prepare_model',body=dict(request_id='missing',
                maintenance_token=ticket['token'],expected_target=target(app,device)))))
            with patch('robot_workbench.management.json_request',side_effect=request),self.assertRaises(OperationError) as error:
                manager.read_pending_call()
            self.assertTrue(error.exception.unknown);self.assertEqual(calls[-1],'receipt')
            manager.state=lambda:dict(server_instance_id='new-instance')
            with patch('robot_workbench.management.json_request') as remote,self.assertRaisesRegex(OperationError,'实例变化'):
                manager.read_pending_call()
            remote.assert_not_called()

    def test_historical_unknown_has_scoped_write_set_but_active_window_remains_exclusive(self):
        import test_management
        harness=test_management.ManagementTests();harness.setUp();self.addCleanup(harness.doCleanups)
        manager=harness.h['manager'];manager.refresh()
        job=dict(job_id='a'*32,status='interrupted_unknown',request=dict(action='prepare_model',parameters={}))
        with manager.operations.lock:manager.operations.jobs['historical']=job
        self.assertIsNone(manager.check('start_cameras'))
        self.assertIn('未知操作',manager.check('prepare_model'))
        self.assertEqual(manager.unresolved(),['a'*32],'independent work must not rewrite unknown history')
        manager.window={'purpose':'prepare_model'}
        self.assertIn('遗留维护',manager.check('start_cameras'))
        manager.window=None

    def test_repeated_get_cannot_make_expired_health_current_but_stop_stays_available(self):
        import test_management
        harness=test_management.ManagementTests();harness.setUp();self.addCleanup(harness.doCleanups)
        manager=harness.h['manager'];manager.refresh()
        with manager.health_lock:manager.health_cache['checked_wall_ns']=time.time_ns()-11_000_000_000
        for _ in range(3):
            view=manager.environment_snapshot(summary=True)
            self.assertFalse(view['health']['freshness']['valid'])
            self.assertTrue(all(row['ready'] is None for row in view['capabilities'].values()))
        self.assertTrue(manager.stop(dict(expected_device_id='cpu-device',request_id='stale-facts-stop'))['receipt']['stopped'])

    def test_job_summary_growth_is_bounded_and_full_evidence_survives(self):
        with tempfile.TemporaryDirectory() as folder:
            service=OperationService([Operation('check','Check','CPU only',lambda _:None,lambda *a:{},lambda *a:{'ok':True})],
                target=lambda:dict(environment='fake'),receipt_dir=folder)
            self.addCleanup(service.close)
            cold='x'*1000000
            def job(i,size):return dict(job_id=f'{i:032x}',request=dict(action='check',request_id=str(i)),status='succeeded',
                created_at_ns=i,verification=dict(ok=True,health={'cold':cold[:size]}),result={'cold':cold[:size]},events=[{'cold':cold[:size]}])
            service.jobs={str(i):job(i,100000) for i in range(50)}
            first=json.dumps(service.snapshot(summary=True))
            service.jobs={str(i):job(i,1000000) for i in range(500)}
            second=json.dumps(service.snapshot(summary=True))
            self.assertLess(len(second),16000);self.assertLess(abs(len(first)-len(second)),500)
            self.assertEqual(len(service.snapshot(summary=True)['recent_jobs']),20)
            self.assertEqual(len(service.snapshot()['recent_jobs'][0]['result']['cold']),1000000)

    def test_deployment_identity_is_captured_once_without_runtime_file_scans(self):
        from types import SimpleNamespace as NS
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);manager=root/'manager';camera=root/'camera'
            for source in (manager,camera):
                (source/'scripts').mkdir(parents=True);(source/'robot_workbench').mkdir()
                (source/'scripts/service.py').write_text('# CPU fixture\n')
                (source/'robot_workbench/source.py').write_text('# dependency one\n')
            profile=root/'profile.json';profile.write_text('{"disabled_grippers":["right"]}')
            deployed=Deployment(manager,{'cameras':NS(spec=NS(expected_script=str(camera/'scripts/service.py')))})
            deployed.bind_configuration(dict(publication_files={'profile':str(profile)}))
            baseline=deployed.snapshot()
            self.assertEqual(baseline['source_verification'],'captured_at_startup');self.assertIn('cameras',baseline['components'])
            with patch.object(Path,'read_bytes',side_effect=AssertionError('polling must not hash files')),patch.object(Path,'stat',side_effect=AssertionError('polling must not scan files')):
                self.assertEqual(deployed.snapshot(),baseline)
            changed=camera/'robot_workbench/source.py';stamp=changed.stat()
            changed.write_text('# dependency two\n')
            os.utime(changed,ns=(stamp.st_atime_ns,stamp.st_mtime_ns+1_000_000_000))
            self.assertEqual(deployed.snapshot(),baseline)
            profile.write_text('{"disabled_grippers":[]}')
            self.assertEqual(deployed.snapshot(),baseline)
            replacement=Deployment(manager,{'cameras':NS(spec=NS(expected_script=str(camera/'scripts/service.py')))})
            replacement.bind_configuration(dict(publication_files={'profile':str(profile)}))
            self.assertNotEqual(replacement.sha256,baseline['sha256'])

    def test_execution_packet_does_not_grow_with_historical_evidence(self):
        from physical_cup_maintenance import execution_package,derive_return,validate_evidence
        from test_physical_cup_maintenance import fixture
        parent,old,receipt,source,binding,offset,geometry,_=fixture()
        plan=derive_return(parent,old,receipt,source,offset,geometry)
        with tempfile.TemporaryDirectory() as folder:
            raw,files=execution_package(plan,folder)
            for name,data in files.items():(Path(folder)/name).write_bytes(data)
            validate_evidence(json.loads(raw),source)
            larger=deepcopy(plan);larger['maintenance_derivation']['parent_execution']['cold_history']='x'*1000000
            larger_raw,_=execution_package(larger,folder)
            self.assertLess(abs(len(raw)-len(larger_raw)),10)
            bad=json.loads(raw);bad['ticks'][0]['target']['left_arm_joints'][0]+=.01
            with self.assertRaises(ValueError):validate_evidence(bad,source)
            pinned=next(iter(files));(Path(folder)/pinned).write_text('{}')
            with self.assertRaisesRegex(ValueError,'SHA256'):validate_evidence(json.loads(raw),source)


if __name__=='__main__':unittest.main()
