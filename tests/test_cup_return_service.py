"""Maintenance publication and UI contracts; CPU fixtures, never GPU/motion."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench')]
from integration import _digest
from robot_workbench.cup_return import prepare,view,configure,remember_success
from test_cup_return_replay import fixture
import test_http_service as http_fixture


class ReturnPublisherTest(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
        source,plan,receipt,replay_source,binding,_=fixture()
        active=self.root/'active';active.mkdir();(active/'calibration.json').write_bytes(b'installed calibration')
        binding.update(initial_state=source['initial_state'],clock=source['clock'],source_digest=_digest(source),
            head_id='head',wrist_id='wrist',model_binding=source['calibration_profile']['model_binding'],
            active_sha256={'calibration.json':hashlib.sha256((active/'calibration.json').read_bytes()).hexdigest()},
            calibration_origins={},calibration_snapshot_id='installed-parent')
        base={'initial_suite':{'episodes':[{'environment_from_machine_root':source['scene']['environment_from_machine_root']}]}}
        manifest={}
        for key,value in dict(source=source,plan=plan,native_receipt=receipt,binding=binding,base=base).items():
            path=self.root/(key+'.json');path.write_text(json.dumps(value))
            manifest[key]=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        current=self.root/'current-binding.json';current.write_text(json.dumps(binding))
        installed=dict(verified=True,head_id='head',wrist_id='wrist',path=str(active))
        initial=replay_source['initial_state']
        self.samples=[(initial['measured_reported'],initial['owner_samples'][0]),
                      (initial['after_observation_reported'],initial['owner_samples'][1])]
        self.device=NS(environment='fake',cup_return_inputs=manifest,cup_return_root=str(self.root/'returns'),
            runtime=NS(owner=NS(hardware_binding=current),lock=threading.RLock()),workflows={},
            collection_preparation=NS(snapshot=lambda:dict(status='awaiting_prepare',installed=installed)),
            fake_scene_snapshot=lambda:dict(physical_configuration_sha256=receipt['physical_configuration_sha256'],
                cup=receipt['last_observation']['sample']['world_from_cup'],
                environment_from_machine_root=base['initial_suite']['episodes'][0]['environment_from_machine_root']),
            snapshot=lambda _:dict(holder='collection',command_epoch=1,runtime_nonce='current-owner',calibration_revision=0,
                allowed_operations=dict(preview=dict(allowed=True))),
            submit=Mock(return_value=dict(token='preview-token',runtime_nonce='current-owner',command_epoch=1,owner_pid=42)))
        self.current,self.installed,self.active=current,installed,active

    def test_without_d_ready_and_binding_publish_current_replay_without_new_rgbd_claim(self):
        self.current.unlink()
        with patch('real_observation.read_initial_sample',side_effect=self.samples):result=prepare(self.device)
        self.assertEqual(result['preview']['purpose'],'cup_return')
        self.assertEqual(self.device.submit.call_args.args[1]['action'],'preview_frozen')
        self.assertFalse(self.device.cup_return_task['submitted'])
        binding=json.loads(self.current.read_text())
        self.assertEqual(binding['schema_version'],'fake_current_replay_binding.v1')
        self.assertEqual(binding['evidence_source'],'fake_current_replay_context')
        self.assertEqual(binding['observation_scope'],'historical_parent_only')
        self.assertNotIn('rgbd_manifest_sha256',binding)

    def test_existing_old_binding_does_not_authorize_changed_installed_files_or_group(self):
        before=self.current.read_bytes()
        for mutation in ('bytes','group'):
            with self.subTest(mutation=mutation):
                (self.active/'calibration.json').write_bytes(b'changed' if mutation=='bytes' else b'installed calibration')
                self.installed['head_id']='other-head' if mutation=='group' else 'head'
                with patch('real_observation.read_initial_sample',side_effect=self.samples),self.assertRaises(ValueError):prepare(self.device)
                self.device.submit.assert_not_called();self.assertEqual(self.current.read_bytes(),before)

    def test_rejected_preview_restores_ordinary_binding(self):
        before=json.loads(self.current.read_text())
        self.device.submit.side_effect=ValueError('preview rejected')
        with patch('real_observation.read_initial_sample',side_effect=self.samples),self.assertRaisesRegex(ValueError,'preview rejected'):
            prepare(self.device,[0,0])
        self.assertEqual(json.loads(self.current.read_text()),before)
        self.assertIsNone(getattr(self.device,'cup_return_task',None))

    def test_explicit_table_initialization_needs_new_collection_before_return(self):
        self.device.scene_recovery_state={'last_completed':{'task_kind':'scene_initialization','motion_executed':False}}
        with self.assertRaisesRegex(ValueError,'场景已恢复'):prepare(self.device)
        self.device.submit.assert_not_called()


class ReturnViewTest(unittest.TestCase):
    def task(self):
        return dict(directory='return-job',timeline_digest='return-timeline',duration_s=37.57,
            parent_plan_id='parent',parent_speed_scale=.97,submitted=True,submitted_monotonic_s=100.,
            command_epoch=3,runtime_nonce='owner',plan_sha256=['return-proxy'])

    def test_confirm_expiration_exits_pending_instead_of_claiming_running_forever(self):
        runtime=dict(state='READY',command_epoch=3,runtime_nonce='owner',frozen={'timeline_digest':'old-plan','status':'completed'},
            last_confirm_rejection=dict(plan_sha256=['return-proxy'],command_epoch=3,motion_started=False,
                monotonic_s=101.,reason='review_expired'))
        result=view(NS(cup_return_task=self.task()),runtime,{})
        self.assertFalse(result['busy']);self.assertNotEqual(result['status'],'running')
        self.assertRegex(result['message'],'未执行|未启动')

    def test_handoff_or_owner_replacement_retires_old_return_without_claiming_success(self):
        device=NS(cup_return_task=self.task())
        for nonce,epoch in [('owner',4),('replacement',0)]:
            with self.subTest(nonce=nonce,epoch=epoch):
                result=view(device,dict(runtime_nonce=nonce,command_epoch=epoch,state='READY',frozen={}),{})
                self.assertFalse(result['busy']);self.assertEqual(result['status'],'superseded')
                self.assertNotEqual(result['outcomes']['motion'],'completed')
        # Unknown context and a still-current pending execution remain busy.
        for runtime in ({},dict(runtime_nonce='owner',command_epoch=3,frozen={})):
            self.assertTrue(view(device,runtime,{})['busy'])

    def test_old_qualified_native_receipt_cannot_mark_current_return_successful(self):
        runtime=dict(state='READY',command_epoch=3,runtime_nonce='owner',
            frozen=dict(timeline_digest='return-timeline',status='completed',execution_timeline_digest='return-execution'),
            native_task=dict(parent_timeline_digest='old-parent',execution_timeline_digest='old-execution',task_qualified=True,task_outcome='success'))
        result=view(NS(cup_return_task=self.task()),runtime,{})
        self.assertNotIn('杯子已放回',result['message'])
        self.assertNotEqual(result['outcomes']['task'],'success')

    def test_process_failure_explains_stage_and_speed_without_claiming_motion_stopped(self):
        runtime=dict(state='READY',execution_instance_id='current',
            execution_calibration={'accepted_monotonic_s':101.},
            frozen=dict(timeline_digest='return-timeline',status='completed',execution_timeline_digest='return-execution'),
            native_task=dict(execution_instance_id='current',execution_timeline_digest='return-execution',
                task_qualified=False,task_outcome='failed',first_failure=dict(reason='finger_cup_forbidden_contact',
                    applied_context={'stage':'lower'},contact_speed_m_s=.08025035,contact_speed_limit_m_s=.08)))
        result=view(NS(cup_return_task=self.task()),runtime,{})
        self.assertEqual(result['outcomes']['motion'],'completed')
        self.assertEqual(result['status'],'failed')
        for text in ('下降放杯','80.25','80.00','finger_cup_forbidden_contact'):
            self.assertIn(text,result['message'])
        self.assertIn('不要直接重复取杯',result['failure']['recovery'])


class RecoveryPersistenceTest(unittest.TestCase):
    def test_failed_return_preserves_endpoint_without_promoting_successful_parent(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            state=dict(schema='scene_recovery_state.v1',robot_id='fake-test',origin={'position':[.4,.2,.78]},
                latest_parent={'old':'parent'},last_completed={'task_kind':'collection'})
            device=NS(scene_recovery_state=deepcopy(state),cup_return_root=root)
            receipt=dict(execution_instance_id='a'*32,motion_outcome='completed',task_outcome='failed',task_qualified=False,
                actual_environment_from_machine_root={'position':[.6,0,.78]},
                endpoint_observation=dict(status='observed',physics_step=100,environment_from_cup={'position':[.43,.18,.78]}))
            remember_success(device,NS(execution_instance_id='a'*32),receipt,{'receipt_path':'failed-receipt'})
            result=json.loads((root/'scene-recovery-state.json').read_text())
            self.assertEqual(result['latest_parent'],state['latest_parent'])
            self.assertEqual(result['last_completed'],state['last_completed'])
            self.assertEqual(result['last_observed_scene']['cup'],receipt['endpoint_observation']['environment_from_cup'])
            self.assertEqual(result['last_observed_scene']['task_outcome'],'failed')

    def test_two_successful_collections_update_parent_without_accumulating_origin_and_survive_restart(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);output=root/'collection';model=output/'calibration/model/robot.urdf'
            model.parent.mkdir(parents=True);model.write_bytes(b'CPU model fixture')
            device=NS(id='fake-test',collection_preparation=NS(snapshot=lambda:dict(output=str(output))))
            configure(device,root/'data',{})
            origin=None;first_manifest=None
            for index in range(2):
                source,plan,native,_,binding,_=fixture()
                source['scene']['environment_from_cup']['position'][0]+=.03*index
                source['calibration_profile']['model_binding']['urdf_sha256']=hashlib.sha256(model.read_bytes()).hexdigest()
                binding['source_digest']=_digest(source)
                ident=str(index+1)*32
                native.update(execution_instance_id=ident,source_digest=_digest(source),parent_timeline_digest=plan['timeline_digest'],
                    actual_environment_from_machine_root=deepcopy(source['scene']['environment_from_machine_root']))
                receipt=root/(ident+'.json');receipt.write_text(json.dumps(native))
                reference=dict(receipt_path=str(receipt),receipt_sha256=hashlib.sha256(receipt.read_bytes()).hexdigest())
                runner=NS(execution_instance_id=ident,payload=dict(plan=plan,source=source),recovery_parent_binding=binding)
                if index==0:origin=deepcopy(source['scene']['environment_from_cup'])
                remember_success(device,runner,native,reference)
                self.assertEqual(device.scene_recovery_state['origin'],origin)
                self.assertEqual(device.scene_recovery_state['last_completed']['execution_instance_id'],ident)
                if index==0:first_manifest=deepcopy(device.cup_return_inputs)
                else:self.assertNotEqual(device.cup_return_inputs,first_manifest)
            saved=root/'data/scene-recovery-state.json';restored=NS(id=device.id)
            configure(restored,root/'restarted',dict(recovery_state_source=dict(path=str(saved),sha256=hashlib.sha256(saved.read_bytes()).hexdigest())))
            self.assertEqual(restored.scene_recovery_state,device.scene_recovery_state)
            self.assertEqual(restored.cup_return_inputs,device.cup_return_inputs)
            saved.write_text('{}')
            with self.assertRaisesRegex(ValueError,'来源文件已变化'):
                configure(NS(id=device.id),root/'other',dict(recovery_state_source=dict(path=str(saved),sha256='0'*64)))

    def test_successful_plan_refs_are_promoted_in_cpu_child(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);output=root/'collection';model=output/'calibration/model/robot.urdf'
            model.parent.mkdir(parents=True);model.write_bytes(b'CPU model fixture')
            device=NS(id='fake-test',collection_preparation=NS(snapshot=lambda:dict(output=str(output))))
            configure(device,root/'data',{})
            source,plan,native,_,binding,_=fixture()
            source['calibration_profile']['model_binding']['urdf_sha256']=hashlib.sha256(model.read_bytes()).hexdigest()
            binding['source_digest']=_digest(source);ident='b'*32
            native.update(execution_instance_id=ident,source_digest=_digest(source),parent_timeline_digest=plan['timeline_digest'],
                actual_environment_from_machine_root=deepcopy(source['scene']['environment_from_machine_root']))
            refs=dict(schema='collection_plan_ref.v1',environment='fake',origin=dict(runtime_nonce=native.get('runtime_nonce')))
            for name,value in (('plan',plan),('source',source),('binding',binding)):
                path=root/(name+'.json');raw=json.dumps(value).encode();path.write_bytes(raw)
                refs[name]=dict(path=str(path),sha256=hashlib.sha256(raw).hexdigest(),size_bytes=len(raw))
            receipt=root/'native.json';receipt.write_text(json.dumps(native))
            reference=dict(receipt_path=str(receipt),receipt_sha256=hashlib.sha256(receipt.read_bytes()).hexdigest())
            runner=NS(execution_instance_id=ident,payload=dict(plan_ref=refs))
            with patch('robot_workbench.cup_return._remember_success_local',side_effect=AssertionError('parent must not decode plan')):
                remember_success(device,runner,native,reference)
            self.assertEqual(device.scene_recovery_state['origin'],source['scene']['environment_from_cup'])
            self.assertEqual(device.scene_recovery_state['last_completed']['execution_instance_id'],ident)
            self.assertEqual(json.loads(Path(device.cup_return_inputs['plan']['path']).read_text()),plan)
            before=deepcopy(device.scene_recovery_state)
            Path(refs['source']['path']).write_text('{}')
            with self.assertRaisesRegex(ValueError,'场景恢复记录未完成'):remember_success(device,runner,native,reference)
            self.assertEqual(device.scene_recovery_state,before)

    def test_failed_or_wrong_execution_does_not_replace_recovery_parent(self):
        device=NS(scene_recovery_state={'existing':'unchanged'})
        remember_success(device,None,dict(motion_outcome='failed'),{})
        self.assertEqual(device.scene_recovery_state,{'existing':'unchanged'})
        with self.assertRaisesRegex(ValueError,'执行身份'):
            remember_success(device,NS(execution_instance_id='other'),dict(motion_outcome='completed',task_qualified=True,
                task_outcome='success',execution_instance_id='actual'),{})


class ReturnWithoutWarmupHTTPTest(unittest.TestCase):
    setUp=http_fixture.HTTPTest.setUp;cleanup=http_fixture.HTTPTest.cleanup;state=http_fixture.HTTPTest.state
    action=http_fixture.HTTPTest.action;wait=http_fixture.HTTPTest.wait;connect=http_fixture.HTTPTest.connect

    def test_service_prepare_revokes_restore_review_before_old_confirmation_can_submit(self):
        self.connect();self.action('handoff',target_business='collection')
        self.app.operations_managed=True
        owner=self.device.snapshot('collection');sent=[]
        self.device.cup_return_inputs={'fixture':'bound-parent'}
        retry=Mock(side_effect=AssertionError('legacy retry must not prepare services'))
        def warm_prepare(*,still_current,lifecycle_admission):
            self.assertTrue(still_current());self.assertTrue(lifecycle_admission('start:collection'))
            with self.assertRaises(ValueError):lifecycle_admission('warmup:collection')
            self.assertTrue(self.device.maintenance);self.assertTrue(self.device.runtime.maintenance_active)
            self.assertEqual(self.device.runtime.holder,'collection')
            self.assertTrue(self.device.snapshot('collection')['feedback_healthy'])
            self.assertNotIn(('fake','collection'),self.app.reviews,'review must be revoked before preparation')
            return dict(reused=True,ready=True)
        prepare_hook=Mock(side_effect=warm_prepare)
        self.device.collection_preparation=NS(snapshot=lambda:dict(status='ready',busy=False,can_retry=True,
            retry_reason=None,installed=dict(verified=True)),retry=retry,operations_prepare=prepare_hook)
        self.device.workflows['collection']=NS(snapshot=lambda:dict(status='ready',busy=False,
            stage_id='collection.idle',outcomes={},collection={}),results=lambda:[],
            allowed=lambda action:dict(allowed=False,reason='ordinary fixture'),invalidate=Mock())
        directory=Path(self.temp.name)/'restore-review';directory.mkdir()
        binding=directory/'current-binding.json';ordinary=dict(source_digest='ordinary')
        binding.write_text(json.dumps(dict(source_digest='restore')))
        (directory/'previous-binding.json').write_text(json.dumps(ordinary))
        def prepare_fixture(device,offset_xy_mm=None):
            device.cup_return_task=dict(directory=str(directory),binding_path=str(binding),source_digest='restore',
                timeline_digest='restore-timeline',duration_s=1.,parent_plan_id='parent',parent_speed_scale=.97,
                submitted=False,runtime_nonce=owner['runtime_nonce'],command_epoch=owner['command_epoch'])
            return dict(preview=dict(token='restore-fixture-preview',owner_pid=owner['pid'],
                runtime_nonce=owner['runtime_nonce'],command_epoch=owner['command_epoch'],
                purpose='cup_return',speed_scale=.97,offset_xy_mm=offset_xy_mm),summary='恢复场景测试')
        def completed_job():
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                job=self.app.jobs[('fake','collection')]
                if job['status']!='running':break
                time.sleep(.01)
            self.assertEqual(job['status'],'completed',job)
        def maintenance(action,**fields):
            from urllib.request import Request
            from robot_workbench.maintenance import target
            self.counter+=1
            data=dict(environment='fake',request_id='restore-maintenance-'+str(self.counter),
                expected_target=target(self.app,self.device),**fields)
            request=Request(self.url+'/api/maintenance/'+action,json.dumps(data).encode(),headers={
                'Content-Type':'application/json','Origin':self.url,'X-CSRF-Token':self.app.csrf})
            with self.http.open(request,timeout=2) as response:return json.loads(response.read())
        with patch('robot_workbench.cup_return.prepare',side_effect=prepare_fixture),patch.object(self.device,'submit',
                side_effect=lambda business,command:sent.append(command) or {'queued':'confirm'}),\
                patch.object(self.device,'connect',side_effect=AssertionError('warm prepare must not acquire control')) as connect:
            self.action('return_cup',business='collection',offset_xy_mm=[0,0]);completed_job()
            review=self.state('collection')['workflow']['review']
            window=maintenance('begin',purpose='prepare_collection')
            self.assertEqual(window['status'],'ready',window)
            result=maintenance('prepare_collection',maintenance_token=window['token'])
            self.assertTrue(result['reused']);prepare_hook.assert_called_once();retry.assert_not_called();connect.assert_not_called()
            maintenance('end',maintenance_token=window['token'])
            self.assertIsNone(self.device.cup_return_task)
            self.assertEqual(json.loads(binding.read_text()),ordinary)
            self.assertNotIn(('fake','collection'),self.app.reviews)
            self.assertIsNone(self.state('collection')['workflow'].get('review'))
            from urllib.error import HTTPError
            with self.assertRaises(HTTPError):
                self.action('execute',business='collection',review_id=review['id'],physical_clearance=True,
                    speed_scale=.97,offset_xy_mm=[0,0])
            self.assertEqual(sent,[],'old restoration review must be rejected before submitting any confirm')

    def test_review_and_confirm_do_not_require_detector_or_curobo_warmup(self):
        self.connect();self.action('handoff',target_business='collection')
        owner=self.device.snapshot('collection');sent=[]
        self.device.cup_return_inputs={'fixture':'bound-parent'}
        self.device.collection_preparation=NS(snapshot=lambda:dict(status='awaiting_prepare',busy=False,can_retry=True,
            retry_reason=None,persistent_service=True,installed=dict(verified=True,head_id='head',wrist_id='wrist')))
        self.assertTrue(self.state('collection')['runtime']['allowed_operations']['return_cup']['allowed'])
        def prepare_fixture(device,offset_xy_mm=None):
            device.cup_return_task=dict(directory='return-job',timeline_digest='return-timeline',duration_s=37.57,
                parent_plan_id='parent',parent_speed_scale=.97,submitted=False,
                runtime_nonce=owner['runtime_nonce'],command_epoch=owner['command_epoch'])
            return dict(preview=dict(token='return-preview',runtime_nonce=owner['runtime_nonce'],command_epoch=owner['command_epoch'],
                owner_pid=owner['pid'],purpose='cup_return',speed_scale=.97,offset_xy_mm=offset_xy_mm),summary='取回维护动作')
        with patch('robot_workbench.cup_return.prepare',side_effect=prepare_fixture),patch.object(self.device,'submit',
                side_effect=lambda business,command:sent.append(command) or {'queued':'confirm'}):
            _,request=self.action('return_cup',business='collection',offset_xy_mm=[20,-10])
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                state=self.state('collection')['workflow']
                if state.get('review'):break
                time.sleep(.01)
            self.assertTrue(state.get('review'),state)
            self.assertEqual(state['review']['request_job_id'],request['job_id'])
            self.assertEqual(state['review']['offset_xy_mm'],[20,-10])
            self.assertEqual(sent,[])
            from urllib.error import HTTPError
            with self.assertRaises(HTTPError):
                self.action('execute',business='collection',review_id=state['review']['id'],physical_clearance=True,speed_scale=.97,offset_xy_mm=[0,0])
            self.action('execute',business='collection',review_id=state['review']['id'],physical_clearance=True,speed_scale=.97,offset_xy_mm=[20,-10])
            self.assertEqual([row['action'] for row in sent],['confirm'])


if __name__=='__main__':unittest.main()
