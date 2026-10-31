"""CPU workflow assembly; model/SDK/codec calls are owned stubs, never hardware."""
from copy import deepcopy
import json
import shutil
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench'),str(ROOT/'calibration/infra')]
from arx_env import joint_head_jaw_mapping as jaw
from integration import _digest
from test_physical_motiongen import fake_inputs
from robot_workbench.inference_transport import conversion_digest
from robot_workbench.inference_workflow import CAMERAS,InferenceWorkflow,calibrated_adapter,_Run
from robot_workbench.runtime import Runtime


class Model:
    loaded=set()
    calls=[]
    def __init__(self,endpoint,*,token_file,adapter,conversion_identity,model_id='base379-epoch010-step29450',model_bundle=None):
        self.adapter=adapter;self.conversion_id=conversion_digest(conversion_identity,model_id);self.identity=None;self.model_id=model_id
    def prepare(self,*,ready_only=False):
        self.calls.append((self.model_id,ready_only))
        if ready_only and self.model_id not in self.loaded:raise ValueError('model not loaded')
        self.loaded.add(self.model_id);self.identity={'model':'CPU fixture only'};return self.identity
    def list_models(self):
        return [dict(model_id=self.model_id,label='CPU fixture',epoch=10,step=29450,status='ready',selectable=True,reason=None)]
    def status(self):
        loaded=self.model_id in self.loaded
        return dict(model_id=self.model_id,loaded=loaded,status='ready' if loaded else 'error',warmup={'completed':loaded},
            service_pid=123,worker_pid=456)
    def infer(self,request):raise AssertionError('model inference must require actual Owner execution')
    def close(self):pass


class Recorder:
    def __init__(self,root,source,binding,**kwargs):
        self.root=Path(root);self.root.mkdir();self.source=source;self.binding=binding;self.kwargs=kwargs
        self.execution_instance_id=source['execution_instance_id'];self.thread=SimpleNamespace(is_alive=lambda:False,join=lambda timeout:None)
        self.prepared=threading.Event();self.prepared.set();self.done=threading.Event();self.commands=[];self.feedback=[]
        self.ready=self.prepared;self.manifest={};self.error=None;self.terminal=None;self.controller_result=None
        self.diagnostics_error=self.seal_error=None
    def append_command(self,**fields):self.commands.append(fields)
    def append_feedback(self,**fields):self.feedback.append(fields)
    def finish(self,status,reason):self.terminal=(status,reason);self.controller_result={'status':status};self.manifest={'streaming_policy':True,'capture_status':'complete','command_count':len(self.commands)};self.done.set()
    def snapshot(self):return dict(done=self.done.is_set(),error=self.error,seal_error=None,manifest=self.manifest)


class Runner:
    def __init__(self,*args,**kwargs):
        self.kwargs=kwargs;self.status='VERIFYING';self.value=dict(status='idle',runner_status=self.status,
            elapsed_source_s=0.,remaining_source_s=40.,submitted_commands=0,completed_commands=0,recording_pending=True)
    def snapshot(self):return {**self.value,'runner_status':self.status}


class InferenceWorkflowTests(unittest.TestCase):
    def test_same_checkpoint_native_modes_are_selected_by_bound_runtime_digest(self):
        from test_native_model_runtime import descriptor
        from robot_workbench.inference_model import model_bundle_digest
        values=[descriptor('torch'),descriptor('triton')]
        rows=[dict(model_id=v['model_id'],backend=v['backend'],selectable=True,status='ready',model_descriptor=v,
                   bundle_digest=model_bundle_digest(v['model_id'],v))for v in values]
        self.workflow.model_catalog_provider=lambda:rows
        view=self.workflow.refresh_models();self.assertEqual({x['backend']for x in view['models']},{'torch','triton'})
        self.assertEqual(len({x['id']for x in view['models']}),1)
        self.assertEqual(len(self.workflow.model_descriptors),2)
        with self.assertRaisesRegex(ValueError,'多个推理模式'):self.workflow._model_selection(values[0]['model_id'])
        for row in rows:
            identity,bundle,digest=self.workflow._model_selection(row['model_id'],row['bundle_digest'])
            self.assertEqual(bundle['backend'],row['backend']);self.assertEqual(digest,row['bundle_digest'])
        self.assertEqual(self.submitted,[])

    def test_preview_observation_uses_authoritative_envelope_even_if_status_cache_lags_handoff(self):
        self.prepare();now,feedback,identity,_=self.workflow._feedback()
        self.owner.epoch-=1  # Explicitly stale asynchronous business status projection.
        self.assertIsNotNone(self.workflow._observe(now,feedback,self.workflow.prepared['binding_digest'],owner_identity=identity))
        self.assertIsNone(self.workflow._observe(now,feedback,self.workflow.prepared['binding_digest']))

    def test_device_camera_and_calibration_changes_invalidate_binding_without_unloading_model(self):
        for changed in ('runtime','calibration','camera_service','capture'):
            with self.subTest(changed=changed):
                self.prepare();before=list(Model.calls);identity=deepcopy(self.workflow.model_identity)
                if changed=='runtime':self.owner.runtime_nonce='replacement-runtime'
                elif changed=='calibration':self.device.runtime.calibration_revision=4
                elif changed=='camera_service':self.cameras.descriptor={'instance_id':'replacement-service'}
                else:
                    original=self.cameras.latest_rgbd.side_effect
                    self.cameras.latest_rgbd.side_effect=lambda camera:dict(original(camera),metadata={**original(camera)['metadata'],'capture_instance_id':'replacement-capture'})
                with self.assertRaisesRegex(ValueError,'变化'):self.workflow._refresh_observation()
                self.assertEqual(self.workflow.model_identity,identity);self.assertEqual(Model.calls,before)
                self.workflow.invalidate();self.workflow.cache=None

    def test_late_model_status_from_previous_generation_does_not_restore_ready(self):
        self.prepare();client=self.workflow.model_client;old=client.status();entered=threading.Event();release=threading.Event()
        def slow():entered.set();release.wait(2);return old
        with patch.object(client,'status',side_effect=slow):
            worker=threading.Thread(target=self.workflow._refresh_model_fact);worker.start();self.assertTrue(entered.wait(1))
            self.workflow.invalidate();self.workflow.model_service_fact=dict(status='error',loaded=False)
            release.set();worker.join(1)
        self.assertFalse(self.workflow.snapshot()['inference']['model_ready'])

    def test_exited_worker_and_stale_status_revoke_cached_model_readiness(self):
        self.prepare();self.assertTrue(self.workflow.snapshot()['inference']['model_ready'])
        self.assertIsNotNone(self.workflow.model_identity)
        Model.loaded.clear();self.workflow._refresh_model_fact()
        state=self.workflow.snapshot()
        self.assertFalse(state['inference']['model_ready'])
        self.assertIsNotNone(self.workflow.model_identity,'binding evidence must not masquerade as worker health')
        self.assertFalse(self.workflow.allowed('inference_preview')['allowed'])
        Model.loaded.add(self.workflow.selected_model_id);self.workflow._refresh_model_fact()
        self.assertTrue(self.workflow.snapshot()['inference']['model_ready'])
        self.workflow.model_fact_received=time.monotonic()-4
        self.assertFalse(self.workflow.snapshot()['inference']['model_ready'])

    def test_same_checkpoint_in_a_new_worker_needs_explicit_rebinding(self):
        self.prepare();old=self.workflow.model_client.status()
        with patch.object(self.workflow.model_client,'status',return_value={**old,'worker_pid':789}):
            self.workflow._refresh_model_fact()
        self.assertFalse(self.workflow.snapshot()['inference']['model_ready'])
        self.assertEqual(self.workflow.model_binding_identity['bound_worker_pid'],456)

    def setUp(self):
        Model.loaded=set();Model.calls=[];self.model_operations=[]
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);jaw.configure_mapping(None)
        source,_,binding=fake_inputs();self.profile=deepcopy(source['calibration_profile'])
        self.profile['source']['simulation_only']=True
        self.profile['joints']['signs']=[1]*12;self.profile['joints']['zero_offsets_rad']=[.01]+[0.]*11
        self.profile['gripper'].update(mapping='operator-pad-basis-1p5mm-per-side-v2',max_opening_mm=83.)
        self.profile['model_binding']=dict(a1_sha256='113b975458ae273ea583e6d1b3376dc9f0df7e71bf1e3bca825985a0da650d76',
            geometry_id='44c6160a0a33a78710c228c5af2b1f9e364d8dc808f3330500830b79a2dc3b16',jaw_mapping_sha256='a'*64)
        self.binding=dict(calibration_accepted=True,robot_id='workflow-fixture',camera_serials={'head':'head-1','left':'left-1','right':'right-1'},
            calibration_profile_digest=_digest(self.profile))
        self.now=20.;self.wall=5_000_000_000_000
        self.limits=[[-3.,3.] for _ in range(14)]
        self.limits[6]=[0.,jaw.command_sdk('left',83.)];self.limits[13]=[0.,jaw.command_sdk('right',83.)]
        q=[0.]*14;q[6]=jaw.command_sdk('left',70.);q[13]=jaw.command_sdk('right',70.)
        self.feedback=dict(qpos=q,qvel=[0.]*14,sample_time_s=self.now,sample_wall_ns=self.wall,
            clock_domain='simulation',source_clock_id='test-clock',source_time_ns=round(self.now*1e9),physics_step=4800,
            owner_pid=123,command_epoch=2,runtime_nonce='fixture-owner')
        self.owner=SimpleNamespace(lock=threading.RLock(),cache={'calibration_feedback':self.feedback,'joint_limits':self.limits},
            control_time=lambda:self.now,feedback_healthy=True,epoch=2,runtime_nonce='fixture-owner',execution_instance_id='episode-fixture',runner=None,recording=None,
            backend=SimpleNamespace(passive_jaws=()),pending={},inference_payloads={},shared_target_profile=None)
        self.configured=[];self.submitted=[]
        def configure(config,factory):self.configured.append((deepcopy(config),factory))
        runtime=SimpleNamespace(owner=self.owner,holder='inference',configure_inference=configure,
            lock=threading.RLock(),preview_revision=0,reviewed_plans={},inference_factory=None,inference_configuration=None,
            nonce='fixture-owner',calibration_revision=0,calibration_snapshot_id=None)
        runtime.control_inputs=lambda:Runtime.control_inputs(runtime)
        runtime.revoke_inference=lambda:Runtime.revoke_inference(runtime)
        def submit(business,body):
            self.submitted.append((business,deepcopy(body)))
            return dict(token='review-token',owner_pid=123,command_epoch=2,runtime_nonce='fixture-owner',duration_s=body.get('duration_s',40))
        self.device=SimpleNamespace(id='workflow-fixture',environment='fake',runtime=runtime,submit=submit)
        self.frame_times={name:self.now-.05 for name in CAMERAS.values()}
        def latest(camera):
            role=next(role for role,name in CAMERAS.items() if name==camera);stamp=self.frame_times[camera]
            return dict(width=224,height=224,rgb=bytes(224*224*3),depth=bytes(224*224*2),metadata=dict(
                serial=self.binding['camera_serials'][role],environment=self.device.environment,capture_instance_id='capture-1',
                frame_number=12,depth_frame_number=12,source_time_ns=round(stamp*1e9),physics_step=4788,
                source_clock_id='test-clock' if self.device.environment=='fake' else None,
                clock_domain='simulation' if self.device.environment=='fake' else 'host_wall',
                host_receive_ns=self.wall+round((stamp-self.now)*1e9)))
        self.cameras=SimpleNamespace(latest_rgbd=Mock(side_effect=latest),channels=dict.fromkeys(CAMERAS.values(),object()))
        self.patches=[patch('robot_workbench.inference_workflow.RemoteBaseClient',Model),
            patch.object(InferenceWorkflow,'_start_cache',lambda self:None)]
        for item in self.patches:item.start();self.addCleanup(item.stop)
        self.workflow=self.make_workflow();self.addCleanup(self.workflow.close)

    def make_workflow(self,**kwargs):
        flow=InferenceWorkflow(self.device,Path(self.temp.name)/'infer',camera_service=self.cameras,
            model_endpoint='http://127.0.0.1:1234',model_token_file='/not-opened-in-cpu-test',
            profile_provider=lambda:(self.profile,self.binding),**kwargs)
        def operation(parameters):
            self.model_operations.append(deepcopy(parameters));self.device.maintenance=True
            try:return flow.operations_prepare_model(**parameters,still_current=lambda:True)
            finally:self.device.maintenance=False
        flow.model_prepare_operation=operation
        return flow

    def prepare(self):
        self.workflow.prepare_model(seed=7)
        self.workflow.action('inference_prepare',{'seed':7,'duration_s':40})
        self.workflow._refresh_observation()

    def test_execution_length_persists_validates_and_revokes_old_review(self):
        self.assertEqual(self.workflow.execution_settings()['execute_steps'],30)
        self.prepare();self.workflow.preview_episode()
        old_config,old_factory=self.configured[-1]
        self.assertEqual(old_config['execute_steps'],30)
        for invalid in (0,61,True,1.5,'30'):
            with self.assertRaises(ValueError):self.workflow.set_execution_steps(invalid,0)
        self.workflow.set_execution_steps(45,0)
        with self.assertRaisesRegex(ValueError,'撤销'):old_factory(self.owner,None,self.feedback,old_config)
        with self.assertRaisesRegex(ValueError,'变化'):self.workflow.set_execution_steps(30,0)
        restored=self.make_workflow();self.addCleanup(restored.close)
        self.assertEqual(restored.execution_settings()['execute_steps'],45)
        result=self.workflow.preview_episode()
        self.assertEqual(self.configured[-1][0]['execute_steps'],45)
        self.assertIn('连续执行45步',result['summary'])
        self.workflow.busy=True
        with self.assertRaisesRegex(ValueError,'结束'):self.workflow.set_execution_steps(30,1)
        self.assertEqual(self.workflow.execute_steps,45)

    def test_camera_connection_failure_identifies_observation_stage_without_motion(self):
        self.prepare();self.cameras.latest_rgbd.side_effect=ConnectionRefusedError(111,'Connection refused')
        with self.assertRaisesRegex(ValueError,'头部相机 RGB-D.*共享相机服务连接中断'):
            self.workflow.preview_episode(seed=0)
        self.assertEqual(self.submitted,[])

    def test_exact_millimetre_offset_and_raw_prediction_projection(self):
        adapter,identity=calibrated_adapter(self.profile,self.limits)
        for gap,model_m in ((83.,.082992),(65.,.064992)):
            device=np.zeros(14);device[0]=.2;device[[6,13]]=gap
            model=adapter.device_to_model(device)
            self.assertAlmostEqual(model[0],.21);self.assertAlmostEqual(model[6],model_m,places=12)
            np.testing.assert_allclose(adapter.model_to_device(model),device,atol=1e-12)
        self.assertEqual(identity['model_gap_offset_mm'],.008)
        raw=np.ones(14)*100;raw[6]=.02
        decoded=adapter.decode(raw,np.zeros(14))
        self.assertGreater(decoded['decoded_device14'][0],3.)
        self.assertEqual(decoded['final_device14'][0],3.);self.assertTrue(decoded['projections'])

    def checkpoint_rows(self):
        from robot_workbench.inference_model import model_spec,model_bundle_digest
        rows=[]
        for identity in ('base566-epoch005-step42130','base566-epoch007-step58982'):
            spec=model_spec(identity)
            rows.append(dict(model_id=identity,label=spec['label'],epoch=spec['epoch'],step=spec['step'],
                status='ready',selectable=True,reason=None,bundle_digest=model_bundle_digest(identity)))
        return rows

    def test_offline_catalog_select_prepares_model_without_device_or_camera_access(self):
        rows=self.checkpoint_rows();rows[1].update(status='downloading',selectable=False,reason='下载中')
        rows.append(dict(model_id=None,label='epoch008',epoch=8,step=None,status='missing_assets',selectable=False,reason='等待metadata'))
        self.device.runtime=None
        workflow=self.make_workflow(model_id=rows[0]['model_id'],model_catalog_provider=lambda:rows,
                                    catalog_root='/model-host/checkpoints',disabled_grippers=('right',))
        self.addCleanup(workflow.close)
        with patch.object(workflow,'_binding',side_effect=AssertionError('offline preparation must not bind device')),\
             patch.object(self.cameras,'latest_rgbd',side_effect=AssertionError('offline preparation must not acquire cameras')):
            result=workflow.select_model(rows[0]['model_id'],rows[0]['bundle_digest'])
        snapshot=workflow.snapshot()['inference']
        self.assertTrue(result['model_ready']);self.assertTrue(snapshot['model_ready'])
        self.assertFalse(snapshot['control_ready']);self.assertIsNone(workflow.prepared)
        self.assertEqual(snapshot['selected_model_id'],rows[0]['model_id'])
        self.assertEqual(snapshot['bundle_digest'],rows[0]['bundle_digest'])
        self.assertEqual(snapshot['catalog_root'],'/model-host/checkpoints')
        self.assertNotIn('model_descriptor',snapshot['models'][0]);self.assertEqual(self.submitted,[])
        self.assertFalse(snapshot['models'][2]['selectable']);self.assertTrue(snapshot['models'][2]['id'].startswith('unavailable-'))
        with self.assertRaisesRegex(ValueError,'下载中'):workflow.select_model(rows[1]['model_id'])
        with self.assertRaisesRegex(ValueError,'身份已变化'):workflow.select_model(rows[0]['model_id'],'f'*64)
        with self.assertRaisesRegex(ValueError,'连接设备'):workflow.bind_device()

    def test_continuous_mapping_and_disabled_right_assumption_are_exactly_once(self):
        adapter,mapping=calibrated_adapter(self.profile,self.limits,model_id='base566-epoch005-step42130',disabled_grippers=('right',))
        held=np.zeros(14);held[0]=.2;held[7:13]=.1;held[6]=52.25;held[13]=2.
        model=adapter.device_to_model(held)
        self.assertAlmostEqual(model[0],.21);self.assertAlmostEqual(model[6],.052242,places=12)
        self.assertAlmostEqual(model[13],.082992,places=12)
        decoded=adapter.decode(model,held)
        self.assertAlmostEqual(decoded['final_device14'][6],52.25,places=10)
        self.assertEqual(decoded['final_device14'][7:13],[.1]*6)
        self.assertAlmostEqual(decoded['final_device14'][13],83.,places=10)
        self.assertNotIn('legacy_binary_threshold_m',mapping)
        self.assertEqual(mapping['disabled_right_gripper_assumption'],'profile_max_opening_not_measured')

    def test_fresh_disabled_backend_policy_binds_review_and_preserves_raw_feedback(self):
        self.owner.backend.passive_jaws=('right',)
        self.feedback['gripper_feedback_valid']=[True,False];self.feedback['qpos'][13]=0.
        workflow=self.make_workflow(model_id='base566-epoch005-step42130',disabled_grippers=('right',))
        self.addCleanup(workflow.close)
        workflow.prepare_model();generation=workflow.generation;workflow.bind_device();workflow._refresh_observation()
        self.assertEqual(workflow.generation,generation+1)
        generation=workflow.generation
        self.assertEqual(workflow.cache['observation']['device_state14'][13],83.)
        self.assertEqual(self.feedback['qpos'][13],0.)
        workflow.preview_episode(seed=7)
        self.assertEqual(workflow.generation,generation)
        config=self.configured[-1][0]
        self.assertEqual(config['disabled_grippers'],['right'])
        self.assertAlmostEqual(config['disabled_right_gripper_model_m'],.082992,places=12)
        self.assertEqual(config['model_id'],'base566-epoch005-step42130')
        self.owner.backend.passive_jaws=()
        with self.assertRaisesRegex(ValueError,'disabled-gripper'):workflow.preview_episode(seed=7)

    def test_model_switch_revokes_inference_review_history_and_rejects_busy(self):
        rows=self.checkpoint_rows()
        workflow=self.make_workflow(model_id=rows[0]['model_id'],model_catalog_provider=lambda:rows)
        self.addCleanup(workflow.close)
        workflow.select_model(rows[0]['model_id'])
        self.owner.pending={'old':((SimpleNamespace(kind='inference'),),2,0,123,[])}
        self.owner.inference_payloads={'old':{}}
        self.device.runtime.reviewed_plans={'old':{'business':'inference'}}
        loop=SimpleNamespace(generation=1,history=[1],applied=[1],_commands=[1],_outstanding=1)
        self.owner.runner=SimpleNamespace(status='READY',loop=loop,outstanding=1)
        workflow.select_model(rows[1]['model_id'])
        self.assertEqual(self.owner.pending,{})
        self.assertEqual(loop.history,[]);self.assertEqual(loop._commands,[]);self.assertIsNone(loop._outstanding)
        self.assertEqual(workflow.selected_model_id,rows[1]['model_id'])
        workflow.busy=True
        with self.assertRaisesRegex(ValueError,'未结束'):workflow.select_model(rows[0]['model_id'])
        workflow.busy=False
        self.device.runtime.holder='collection';self.owner.runner.status='RUNNING'
        self.owner.pending={'collection':object()};self.device.runtime.reviewed_plans={'collection':{'business':'collection'}}
        workflow.prepare_model(force=True)
        self.assertEqual(set(self.owner.pending),{'collection'})
        self.assertEqual(set(self.device.runtime.reviewed_plans),{'collection'})

    def test_sealed_manifest_failure_is_never_displayed_as_valid_data(self):
        runner=Runner();runner.status='READY';runner.value.update(runner_status='READY',recording_pending=False)
        self.workflow.current=dict(id='failed-data',runner=runner,config={'seed':0,'duration_s':40},metrics={},started_wall=None,ended_wall=None,
            recording_error=None,receipt={'recording':{'manifest':{'capture_status':'incomplete',
                'validation':{'status':'failed','problems':['feedback does not cover complete command duration']}}}},audit={})
        self.assertEqual(self.workflow.snapshot()['outcomes']['data'],'invalid')

    def test_prepare_and_preview_do_not_execute_and_snapshot_tracks_seed(self):
        self.assertEqual(self.submitted,[]);self.prepare();self.assertEqual(self.submitted,[])
        self.assertEqual(self.configured,[],'loading a model does not configure or confirm an Owner run')
        result=self.workflow.action('inference_preview',{'seed':13,'duration_s':40})
        self.assertEqual(self.submitted,[('inference',{'action':'preview_inference','seed':13,'duration_s':40.})])
        self.assertEqual(result['preview']['seed'],13);self.assertIn('不检查碰撞',result['summary'])
        config=self.configured[-1][0]
        self.assertEqual(config['duration_s'],40);self.assertEqual(config['velocity_limits'][6],None)
        self.assertEqual(config['conversion_identity']['mapping']['model_gap_offset_mm'],.008)
        self.assertEqual(self.workflow.snapshot()['inference']['seed'],13)
        self.workflow.action('execute',{'token':'review-token','physical_clearance':True})
        self.assertEqual(self.submitted[-1][1]['action'],'confirm')
        self.workflow.invalidate();self.assertNotEqual(self.workflow.snapshot()['primary_action']['id'],'execute')

    def test_one_second_preview_freezes_duration_and_default_is_120(self):
        self.prepare()
        result=self.workflow.preview_episode(seed=13,duration_s=1)
        self.assertEqual(result['preview']['duration_s'],1)
        self.assertEqual(self.configured[-1][0]['duration_s'],1)
        self.assertEqual(self.submitted[-1][1]['duration_s'],1)
        self.assertIn('本轮 1 个设备源时间秒',result['summary'])
        self.assertEqual(self.workflow.snapshot()['inference']['duration_s'],1)
        self.assertIn('本轮 1 秒',self.workflow.snapshot()['primary_action']['label'])
        self.workflow.execute_review(result['preview']['token'])
        self.assertNotIn('duration_s',self.submitted[-1][1],'execution consumes the frozen review only')
        short=self.workflow.preview_episode(seed=14,duration_s=2)
        self.assertEqual(short['preview']['duration_s'],2)
        self.assertEqual(self.configured[-1][0]['duration_s'],2)
        self.assertEqual(self.submitted[-1][1]['duration_s'],2)
        default=self.workflow.preview_episode(seed=14)
        self.assertEqual(default['preview']['duration_s'],120)
        self.assertEqual(self.configured[-1][0]['duration_s'],120)
        self.assertEqual(self.workflow.snapshot()['inference']['duration_s'],120)
        with patch.object(self.workflow,'_refresh_observation',side_effect=AssertionError('invalid duration rejected before capture')):
            for invalid in (3,0,True,'1',float('nan')):
                with self.assertRaises(ValueError):self.workflow.preview_episode(seed=0,duration_s=invalid)

    def test_native_preparation_exposes_actual_duration_separately_from_two_second_source(self):
        from robot_workbench.inference_owner import device_state
        self.prepare();initial=device_state(self.feedback['qpos'])
        rows=[list(initial)for _ in range(61)]
        for i,row in enumerate(rows):row[0]+=.05*i/60
        self.workflow.reference=dict(model_id=self.workflow.selected_model_id,
            calibration_profile_digest=self.binding['calibration_profile_digest'],device_targets14=rows)
        self.workflow.reference_pin=dict(path='/cpu/reference.json',sha256='a'*64)
        self.workflow.prepared_reference_execution=True
        self.workflow.reference_integral_feedback=True
        result=self.workflow.preview_episode(seed=0,duration_s=2)
        config=self.configured[-1][0]
        self.assertEqual(config['duration_s'],2)
        self.assertEqual(config['trajectory_kind'],'prepared_native_reference')
        self.assertGreater(config['execution_duration_s'],2)
        self.assertIn('实际执行',result['summary'])
        self.assertTrue(config['reference_integral_feedback']);self.assertIn('低频积分',result['summary'])
        self.assertAlmostEqual(config['execution_duration_s'],config['prepared_reference']['duration_s']+4.04)
        import hashlib
        profile=dict(schema='reference_feedforward.v1',source_sha256='a'*64,controller_profile='normal_X5',phase=[0.,1.],
            torque_bias_sdk6=[[0.]*6]*2,provenance=dict(run='cpu-only'))
        raw=json.dumps(profile).encode();sha=hashlib.sha256(raw).hexdigest()
        folder=self.workflow.output/'reference-feedforward';folder.mkdir()
        (folder/(sha+'.json')).write_bytes(raw)
        result=self.workflow.preview_episode(seed=0,duration_s=2,reference_feedforward_sha256=sha)
        config=self.configured[-1][0]
        self.assertEqual(config['reference_feedforward']['sha256'],sha)
        self.assertAlmostEqual(config['execution_duration_s'],config['prepared_reference']['duration_s']+8.)
        self.assertIn('同源实测前馈',result['summary'])

    def test_fake_and_physical_observation_clocks_and_memory_only_owner_reader(self):
        for environment in ('fake','physical'):
            self.device.environment=environment;self.profile['source']['simulation_only']=environment=='fake'
            self.binding['calibration_profile_digest']=_digest(self.profile)
            self.feedback.update(source_clock_id='test-clock' if environment=='fake' else None,
                clock_domain='simulation' if environment=='fake' else 'host_wall')
            self.prepare();prepared=self.workflow.prepared
            with patch.object(self.cameras,'latest_rgbd',side_effect=AssertionError('Owner must not do camera IPC')):
                before=time.monotonic();value=self.workflow._observe(self.now,self.feedback,prepared['binding_digest'])
                self.assertLess(time.monotonic()-before,.05)
            self.assertAlmostEqual(value['source_time'],19.95)
            self.assertEqual(value['images']['base_0_rgb'].shape,(224,224,3))
            self.assertEqual(value['frame_ids']['base_0_rgb']['feedback_sample_wall_ns'],self.wall)
            self.assertNotIn('support',value);self.assertNotIn('task',value)
            if environment=='fake':
                wrong=dict(self.feedback,source_clock_id='different-clock')
                self.assertIsNone(self.workflow._observe(self.now,wrong,prepared['binding_digest']))
            self.assertIsNone(self.workflow._observe(self.now,self.feedback,prepared['binding_digest'],dict.fromkeys(CAMERAS,'replaced-capture')))
            self.assertIsNone(self.workflow._observe(self.now+.2,self.feedback,prepared['binding_digest']))
        self.frame_times['right_wrist_0']=self.now-.25
        with self.assertRaisesRegex(ValueError,'0.1 秒'):self.workflow._refresh_observation()
        self.frame_times['right_wrist_0']=self.now-.05;self.now+=.2
        with self.assertRaisesRegex(ValueError,'实测反馈'):self.workflow._refresh_observation()

    def test_changed_installation_revokes_factory_and_stale_model_preparation(self):
        self.prepare();self.workflow.action('inference_preview',{'seed':9})
        factory=self.configured[-1][1];self.workflow.invalidate()
        with self.assertRaisesRegex(ValueError,'撤销'):factory(self.owner,None,self.feedback,self.configured[-1][0])
        self.profile['joints']['zero_offsets_rad'][0]=.02;self.binding['calibration_profile_digest']=_digest(self.profile)
        with self.assertRaisesRegex(ValueError,'已变化'):self.workflow.action('inference_preview',{'seed':9})
        def interrupted(_,**kwargs):self.workflow.invalidate();return {'model':'discarded'}
        with patch.object(Model,'prepare',interrupted):
            with self.assertRaisesRegex(ValueError,'已撤销'):self.workflow.prepare_model(seed=2)
        self.assertIsNone(self.workflow.prepared)

    def test_business_missing_model_never_prepares_and_selection_has_one_operation(self):
        for environment in ('fake','physical'):
            with self.subTest(environment=environment):
                self.device.environment=environment
                flow=self.make_workflow();self.addCleanup(flow.close)
                before=list(Model.calls)
                with self.assertRaisesRegex(ValueError,'先选择并准备模型'):
                    flow.action('inference_prepare',{'seed':3})
                with self.assertRaises(ValueError):flow.preview_episode()
                self.assertEqual(Model.calls,before)
                flow.model_prepare_operation=None
                with self.assertRaisesRegex(ValueError,'运维 Operation'):flow.select_model(flow.selected_model_id)
                self.assertEqual(Model.calls,before)
        self.device.environment='fake';flow=self.workflow
        before=len(self.model_operations);flow.select_model(flow.selected_model_id)
        self.assertEqual(len(self.model_operations),before+1)
        loads=sum(not ready for _,ready in Model.calls)
        flow.select_model(flow.selected_model_id)
        self.assertEqual(len(self.model_operations),before+2)
        self.assertEqual(sum(not ready for _,ready in Model.calls),loads)
        with patch.object(flow,'snapshot',side_effect=AssertionError('environment status read business state')):
            self.assertTrue(flow.environment_status()['ready'])
        flow.bind_device()
        self.assertEqual(sum(not ready for _,ready in Model.calls),loads)
        self.assertEqual(self.submitted,[])

    def test_model_hook_requires_live_maintenance_and_drops_late_identity(self):
        with self.assertRaisesRegex(ValueError,'运维维护窗口'):
            self.workflow.operations_prepare_model(still_current=lambda:True)
        self.assertEqual(Model.calls,[])
        self.device.maintenance=True;current=[True]
        def late(client,**kwargs):current[0]=False;return {'late':'identity'}
        with patch.object(Model,'prepare',late):
            with self.assertRaisesRegex(ValueError,'撤销'):
                self.workflow.operations_prepare_model(still_current=lambda:current[0])
        self.assertFalse(self.workflow.environment_status()['ready'])
        self.assertIsNone(self.workflow.model_identity)

    def test_model_invalidation_revokes_local_bindings_without_loading_or_stopping_worker(self):
        self.prepare();self.workflow.preview_episode()
        generation=self.workflow.generation;calls=list(Model.calls)
        model_client,bound_client=self.workflow.model_client,self.workflow.client
        model_client.close=Mock();bound_client.close=Mock()
        with self.assertRaisesRegex(ValueError,'维护窗口'):
            self.workflow.operations_invalidate_model(still_current=lambda:True)
        self.assertEqual(self.workflow.generation,generation)
        self.device.maintenance=True
        result=self.workflow.operations_invalidate_model(still_current=lambda:True)
        self.assertTrue(result['invalidated']);self.assertFalse(result['worker_lifecycle_changed'])
        self.assertGreater(self.workflow.generation,generation)
        self.assertEqual(Model.calls,calls);model_client.close.assert_called_once();bound_client.close.assert_called_once()
        for field in ('client','model_client','prepared','cache','support_cache','model_identity'):
            self.assertIsNone(getattr(self.workflow,field),field)
        state=self.workflow.environment_status()
        self.assertFalse(state['loaded']);self.assertFalse(state['ready']);self.assertEqual(state['status'],'not_loaded')
        self.assertIsNone(state['error']);self.assertIsNone(self.device.runtime.inference_factory)
        generation=self.workflow.generation
        with self.assertRaisesRegex(ValueError,'维护窗口'):
            self.workflow.operations_invalidate_model(still_current=lambda:False)
        self.assertEqual(self.workflow.generation,generation)

    def test_recording_disabled_uses_existing_runner_without_recorder(self):
        self.workflow.recording_enabled=False
        self.prepare()
        result=self.workflow.action('inference_preview',{'seed':13,'duration_s':40})
        config,factory=self.configured[-1]
        with patch('robot_workbench.inference_workflow.make_inference_runner',side_effect=Runner):
            runner=factory(self.owner,None,self.feedback,config)
        self.assertFalse(config['recording_enabled'])
        self.assertIsNone(runner.kwargs['record'])
        self.assertIsNone(runner.kwargs['record_prepared'])
        self.assertIsNone(self.owner.recording)
        self.assertIn('录制已关闭',result['summary'])
        self.assertEqual(self.workflow.snapshot()['outcomes']['data'],'not_applicable')
        self.assertFalse(self.workflow.snapshot()['inference']['recording_enabled'])

    def test_discarded_predictions_report_no_motion_reason_and_recover_on_acceptance(self):
        self.prepare();self.workflow.preview_episode()
        config,factory=self.configured[-1]
        with patch('robot_workbench.inference_workflow.make_inference_runner',side_effect=Runner):
            runner=factory(self.owner,None,self.feedback,config)
        runner.status='RUNNING';event=runner.kwargs['event']
        timing=dict(worker=dict(infer_wall_s=1.1,gpu_compute_s=None),host_total_wall_s=1.2)
        for _ in range(2):
            event('prediction_requested',{})
            event('prediction_discarded',dict(reason='stale_observation',observation_age_source_s=1.4,
                model_wall_s=1.3,worker_audit=dict(timing=timing)))
        snapshot=self.workflow.snapshot();metrics=snapshot['inference']['metrics']
        self.assertEqual([metrics[key] for key in ('queries','returned_predictions','accepted_predictions',
            'predictions','discarded_predictions','consecutive_discarded_predictions')],[2,2,0,0,2,2])
        self.assertEqual(metrics['discard_reasons'],{'stale_observation':2})
        metrics['discard_reasons']['stale_observation']=99
        self.assertEqual(self.workflow.snapshot()['inference']['metrics']['discard_reasons'],{'stale_observation':2})
        self.assertEqual(metrics['last_observation_age_source_s'],1.4)
        self.assertEqual(metrics['last_model_wall_s'],1.3)
        self.assertEqual(snapshot['inference']['audit']['last_worker_timing'],timing)
        self.assertEqual(snapshot['status'],'running');self.assertEqual(metrics['submitted'],0)
        self.assertIn('预测已丢弃',snapshot['message']);self.assertIn('1.400',snapshot['message'])
        self.assertEqual((config['maximum_observation_age_s'],config['execute_steps'],config['duration_s']),(.5,30,120.))
        event('prediction_requested',{})
        event('prediction_received',dict(model_wall_s=.1,observation_age_source_s=.2,
            worker_audit=dict(timing={'worker':{'infer_wall_s':.09}})))
        snapshot=self.workflow.snapshot();metrics=snapshot['inference']['metrics']
        self.assertEqual([metrics[key] for key in ('returned_predictions','accepted_predictions',
            'discarded_predictions','consecutive_discarded_predictions')],[3,1,2,0])
        self.assertEqual(metrics['predictions'],1);self.assertEqual(metrics['last_prediction_status'],'accepted')
        self.assertNotIn('预测已丢弃',snapshot['message'])
        event('prediction_discarded',dict(reason='revoked_generation',model_wall_s=.3))
        self.assertIn('执行已撤销',self.workflow.snapshot()['message'])
        runner.status='STOPPED';runner.value.update(recording_pending=False,reason='operator_stop')
        snapshot=self.workflow.snapshot()
        self.assertEqual(snapshot['status'],'stopped');self.assertEqual(snapshot['message'],'operator_stop')
        self.assertEqual(snapshot['inference']['metrics']['discard_reasons'],
            {'stale_observation':2,'revoked_generation':1})
        self.assertIsNone(snapshot['inference']['metrics']['last_observation_age_source_s'])

    def test_shared_recording_actual_events_persist_results_without_a_plan(self):
        self.prepare();self.workflow.action('inference_preview',{'seed':13,'duration_s':1})
        config,factory=self.configured[-1]
        with patch('robot_workbench.inference_workflow.make_inference_runner',side_effect=Runner):
            runner=factory(self.owner,None,self.feedback,config)
        self.owner.runner=runner
        self.assertEqual(self.submitted[-1][1]['action'],'preview_inference')
        prepared=dict(configuration=config,execution_instance_id='episode-fixture',
            execution_owner=dict(owner_pid=123,command_epoch=2,runtime_nonce='fixture-owner'),
            clock=dict(environment='fake',clock_domain='simulation',source_clock_id='test-clock'),preparation_deadline_monotonic=time.monotonic()+1)
        errors=[]
        def on_data_thread(name,value):
            def call():
                try:runner.kwargs['record'](name,value)
                except Exception as error:errors.append(error)
            thread=threading.Thread(target=call);thread.start();thread.join(2)
            self.assertFalse(thread.is_alive());self.assertEqual(errors,[])
        def export(raw,output,**kwargs):
            output=Path(output);output.mkdir()
            files={f'source-{camera}.mp4':{} for camera in CAMERAS.values()}
            for name in files:(output/name).write_bytes(b'CPU video fixture')
            return dict(files=files,recording_kind='streaming_policy')
        with patch('robot_workbench.recording_process.ProcessRecordingSession',Recorder),patch('robot_workbench.recording_process.export_playback_process',side_effect=export):
            on_data_thread('prepare',prepared)
            self.assertTrue(runner.kwargs['record_prepared']())
            run=self.workflow.current;recorder=run['recorder']
            self.assertEqual(recorder.source['duration_s'],1)
            self.assertEqual(recorder.source['duration_tolerance_s'],.03)
            self.assertEqual(json.loads((run['directory']/'configuration.json').read_text())['duration_s'],1)
            self.assertNotIn('execution_start_source_time_ns',recorder.source)
            command=dict(tick=0,send_monotonic_s=20.,send_wall_ns=self.wall,send_source_time_ns=20_000_000_000,
                owner_pid=123,command_epoch=2,runtime_nonce='fixture-owner',feedback_at_send=deepcopy(self.feedback),
                final_device14=[.2]*6+[65.]+[.1]*6+[83.])
            on_data_thread('command',command)
            on_data_thread('feedback',dict(raw=self.feedback,device_state14=command['final_device14']))
            self.assertEqual(recorder.commands[0]['target']['left_gripper'],65.)
            self.assertEqual(recorder.commands[0]['send_wall_ns'],self.wall)
            self.assertEqual(recorder.commands[0]['feedback_at_send'],self.feedback)
            self.assertEqual(recorder.feedback[0]['sample_monotonic_s'],20.)
            runner.kwargs['event']('prediction_requested',{})
            runner.kwargs['event']('prediction_requested',{})
            runner.kwargs['event']('prediction_received',dict(model_wall_s=.125,audit={'problems':[{'kind':'velocity'}]},conversions=[{'projections':[{'axis':0}]}]))
            self.assertEqual(self.workflow.snapshot()['inference']['metrics']['queries'],2)
            self.assertEqual(self.workflow.snapshot()['inference']['metrics']['predictions'],1)
            runner.status='READY';runner.value.update(recording_pending=False,outcome={'success':True},submitted_commands=1,completed_commands=1)
            on_data_thread('finish',dict(status='completed',reason='1 source seconds completed',inference=runner.snapshot()))
        self.assertEqual(len(self.workflow.results()),3)
        receipt=json.loads((run['directory']/'receipt.json').read_text());self.assertEqual(receipt['inference']['outcome'],{'success':True})
        self.assertFalse((run['directory']/'plan.json').exists())
        self.assertNotIn('images',(run['directory']/'policy-events.jsonl').read_text())
        self.device.runtime=None;restored=self.make_workflow();self.addCleanup(restored.close)
        self.assertEqual(len(restored.results()),3)
        self.assertTrue(restored.media(restored.results()[0]['id']).is_file())
        self.assertIsNone(restored.media('../../outside'))

    def test_actual_shared_owner_starts_after_camera_prepare_and_stops_without_model_wait(self):
        from test_inference_owner import InferenceOwnerTests
        harness=InferenceOwnerTests();harness.setUp();self.addCleanup(harness.doCleanups)
        self.device.runtime=harness.runtime;self.device.id=harness.runtime.device_id
        self.device.submit=lambda business,body:harness.runtime.submit(harness.creds[business],body)
        self.binding['robot_id']=self.device.id
        def latest(camera):
            feedback=deepcopy(harness.runtime.owner.cache['calibration_feedback'])
            role=next(role for role,name in CAMERAS.items() if name==camera)
            return dict(width=224,height=224,rgb=bytes(224*224*3),depth=bytes(224*224*2),metadata=dict(
                serial=self.binding['camera_serials'][role],environment='fake',capture_instance_id='actual-owner-camera-fixture',
                source_clock_id=feedback['source_clock_id'],clock_domain='simulation',
                source_time_ns=round((feedback['sample_time_s']-.02)*1e9),host_receive_ns=time.time_ns(),frame_number=1,depth_frame_number=1))
        self.cameras.latest_rgbd.side_effect=latest
        self.prepare();result=self.workflow.action('inference_preview',{'seed':4})
        self.assertIsNone(harness.runtime.owner.runner)
        def export(raw,output,**kwargs):Path(output).mkdir();return {'files':{}}
        with patch('robot_workbench.recording_process.ProcessRecordingSession',Recorder),patch('robot_workbench.recording_process.export_playback_process',side_effect=export):
            self.workflow.action('execute',{'token':result['preview']['token'],'physical_clearance':True})
            harness.advance();run=self.workflow.current
            deadline=time.monotonic()+1
            while run['recorder'] is None and time.monotonic()<deadline:time.sleep(.001)
            self.assertIsNotNone(run['recorder']);self.assertTrue(run['recorder'].prepared.is_set())
            harness.advance(40)
            runner=harness.runtime.owner.runner;self.assertEqual(runner.status,'RUNNING',runner.reason)
            before=time.monotonic();harness.runtime.submit(harness.creds['calibration'],{'action':'stop'})
            self.assertLess(time.monotonic()-before,.1)
            self.assertTrue(runner.data_done.wait(2))
            self.assertEqual(runner.status,'STOPPED');self.assertIsNone(runner.recording_error,runner.recording_error)
            self.assertTrue(run['recorder'].commands,'initial hold must be an actual shared Owner send')
            self.assertEqual(run['recorder'].commands[0]['tick'],0)
            self.assertEqual(run['recorder'].source['execution_instance_id'],run['id'])

    def test_actual_camera_producer_owner_guard_delivers_first_raw_frame(self):
        from test_physx_camera import PhysxCameraTest
        from robot_workbench.inference_owner import device_state
        import real_recording as recording
        h=PhysxCameraTest();h.setUp();self.addCleanup(h.doCleanups)
        h.backend.owner=self.owner
        def guard(lease):
            session=getattr(h.backend.owner,'recording',None)
            return session is not None and session.lease_active(lease)
        producer=h.producer(memory_recording=True,recording_guard=guard)
        self.workflow.cameras=producer.common
        camera_serials={role:h.k[name]['serial'] for role,name in CAMERAS.items()}
        binding={**self.binding,'camera_serials':camera_serials}
        clock=dict(environment='fake',clock_domain='simulation',source_clock_id=h.backend.source_clock_id)
        runner=SimpleNamespace(status='VERIFYING');self.owner.runner=runner
        run=_Run(id=self.owner.execution_instance_id,directory=Path(self.temp.name)/'actual-camera-run',
            config={'clock':clock,'duration_s':40},prepared=dict(profile=self.profile,binding=binding),owner=self.owner,
            runner=runner,recorder=None,recording_error=None,event_bytes=0,feedback_count=0)
        owner=dict(owner_pid=123,command_epoch=self.owner.epoch,runtime_nonce=self.owner.runtime_nonce)
        payload=dict(clock=clock,execution_owner=owner,preparation_deadline_monotonic=time.monotonic()+1)
        from robot_workbench.recording_process import ProcessRecordingSession
        class CpuRecorder(ProcessRecordingSession):
            def __init__(self,*args,**kwargs):
                kwargs['storage_format']='segments' if shutil.which('ffmpeg') and shutil.which('ffprobe') else 'zip'
                super().__init__(*args,**kwargs)
        errors=[]
        def prepare():
            try:self.workflow._record_event(run,'prepare',payload)
            except Exception as error:errors.append(error)
        with patch('robot_workbench.recording_process.ProcessRecordingSession',CpuRecorder):
            thread=threading.Thread(target=prepare);thread.start();thread.join(3)
            self.assertFalse(thread.is_alive());self.assertEqual(errors,[])
            session=run['recorder']
            try:
                self.assertTrue(session.prepared.wait(3),session.snapshot())
                self.assertIs(self.owner.recording,session)
                self.assertIs(session.camera_channels,producer.common.channels)
                for camera in producer.channels:
                    self.assertIs(session.camera_channels[camera],producer.channels[camera])
                    self.assertIsNotNone(producer.channels[camera].cached_token)
                    self.assertTrue(guard(producer.channels[camera].current_lease()))
                h.collect(producer,dict(owner_available=True,**owner));h.wait(producer)
                feedback=h.backend.read_calibration_state();feedback.update(owner)
                self.workflow._record_event(run,'feedback',dict(raw=feedback,device_state14=device_state(feedback['qpos'])))
                self.workflow._record_event(run,'command',dict(owner,tick=0,
                    final_device14=device_state(feedback['qpos']),feedback_at_send=deepcopy(feedback),
                    send_monotonic_s=feedback['sample_time_s'],send_wall_ns=feedback['sample_wall_ns'],
                    send_source_time_ns=feedback['source_time_ns']))
                self.assertTrue(session.ready.wait(5),session.snapshot())
                for camera in producer.channels:
                    self.assertTrue(producer.snapshot()['last_publish']['cameras'][camera]['recorded'])
                # A revoked execution cannot admit another sampled token into this recorder.
                self.owner.epoch+=1
                h.collect(producer,dict(owner_available=True,**{**owner,'command_epoch':self.owner.epoch}));h.wait(producer)
                for camera in producer.channels:self.assertFalse(producer.snapshot()['last_publish']['cameras'][camera]['recorded'])
                session.finish('stopped','CPU first-frame qualification')
                self.assertTrue(session.done.wait(10),session.snapshot());session.thread.join(2)
                self.assertIsNone(session.error,session.snapshot());self.assertIsNone(session.seal_error,session.snapshot())
                for camera in producer.channels:
                    frames=list(recording.iter_rgbd(session.root,camera,environment='fake'))
                    self.assertEqual(len(frames),1)
                    self.assertEqual(frames[0][1]['frame_number'],0)
                self.assertFalse(session.thread.is_alive());self.assertIs(self.owner.recording,session)
            finally:
                if not session.done.is_set():session.finish('stopped','CPU cleanup')
                self.assertTrue(session.done.wait(10));session.thread.join(2)

    def test_old_recorder_completion_does_not_detach_replacement(self):
        self.prepare();self.workflow.action('inference_preview',{'seed':3})
        config,factory=self.configured[-1]
        with patch('robot_workbench.inference_workflow.make_inference_runner',side_effect=Runner):runner=factory(self.owner,None,self.feedback,config)
        self.owner.runner=runner;run=self.workflow.current
        payload=dict(clock=config['clock'],execution_owner=dict(owner_pid=123,command_epoch=2,runtime_nonce='fixture-owner'),
            preparation_deadline_monotonic=time.monotonic()+1)
        with patch('robot_workbench.recording_process.ProcessRecordingSession',Recorder):self.workflow._record_event(run,'prepare',payload)
        self.assertIs(self.owner.recording,run['recorder'])
        replacement=object();self.owner.recording=replacement
        with patch('robot_workbench.recording_process.export_playback_process',return_value={'files':{}}):
            self.workflow._finish_recording(run,dict(status='stopped',reason='old generation',inference={}))
        self.assertIs(self.owner.recording,replacement)

    def test_physical_one_second_recording_preserves_sdk_wall_clocks_without_simulation_fields(self):
        self._physical_one_second_clock_roundtrip()

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'actual policy video export requires CPU codecs')
    def test_physical_one_second_export_and_restart_history_use_the_actual_recorded_clock(self):
        self._physical_one_second_clock_roundtrip(export=True)

    def _physical_one_second_clock_roundtrip(self,*,export=False):
        import io
        from PIL import Image
        import real_recording as recording
        from test_rgbd_service import SharedCameraTest
        from robot_workbench.sample_clock import clock_spec
        self.device.environment='physical';self.profile['source']['simulation_only']=False
        binding={**self.binding,'calibration_profile_digest':_digest(self.profile)}
        h=SharedCameraTest();self.addCleanup(h.doCleanups)
        service=h.service('physical',camera_serials=binding['camera_serials']);self.workflow.cameras=service
        runner=SimpleNamespace(status='VERIFYING');self.owner.runner=runner
        run=_Run(id=self.owner.execution_instance_id,directory=self.workflow.output/'physical-one-second',
            config={'clock':clock_spec('physical'),'duration_s':1.,'calibration_profile_digest':binding['calibration_profile_digest'],
                    'conversion_id':'c'*64},prepared=dict(profile=self.profile,binding=binding),
            owner=self.owner,runner=runner,recorder=None,recording_error=None,event_bytes=0,feedback_count=0)
        owner=dict(owner_pid=123,command_epoch=self.owner.epoch,runtime_nonce=self.owner.runtime_nonce)
        from robot_workbench.recording_process import ProcessRecordingSession
        class CpuRecorder(ProcessRecordingSession):
            def __init__(self,*args,**kwargs):
                kwargs['storage_format']='segments' if shutil.which('ffmpeg') and shutil.which('ffprobe') else 'zip'
                super().__init__(*args,**kwargs)
        with patch('robot_workbench.recording_process.ProcessRecordingSession',CpuRecorder):
            self.workflow._record_event(run,'prepare',dict(clock=None,execution_owner=owner,
                preparation_deadline_monotonic=time.monotonic()+1))
            session=run['recorder'];original_feedback=[];original_commands=[]
            try:
                self.assertTrue(session.prepared.wait(3),session.snapshot())
                epoch=time.time_ns();mono=100.
                state=[0.]*6+[70.]+[0.]*6+[70.]
                scheduled=[(i*10_000_000,0,i) for i in range(101)]
                scheduled += [(i*1_000_000_000//30,1,i) for i in range(30)]
                scheduled += [(i*1_000_000_000//30,2,i) for i in range(31)]
                latest=None
                for ns,kind,index in sorted(scheduled):
                    if kind==0:
                        raw={key:deepcopy(value) for key,value in self.feedback.items() if key not in (
                            'clock_domain','source_clock_id','source_time_ns','physics_step','owner_pid','command_epoch','runtime_nonce')}
                        raw.update(sample_time_s=mono+ns/1e9,sample_wall_ns=epoch+ns,sdk_timestamps=[500+ns/1e9,541+ns/1e9])
                        original_feedback.append(deepcopy(raw));latest=raw
                        self.workflow._record_event(run,'feedback',dict(raw=raw,device_state14=state))
                        self.assertEqual(raw,original_feedback[-1])
                    elif kind==1:
                        command=dict(owner,tick=index,send_monotonic_s=mono+ns/1e9,send_wall_ns=epoch+ns,
                            send_source_time_ns=round((mono+ns/1e9)*1e9),feedback_at_send=deepcopy(latest),final_device14=state)
                        original_commands.append(deepcopy(command));self.workflow._record_event(run,'command',command)
                    else:
                        for camera in service.channels:
                            metadata=h.metadata(camera,index);metadata['host_receive_ns']=epoch+ns
                            self.assertTrue(service.publish(camera,metadata,*h.pixels(index),width=2,height=2,
                                sampled_token=service.channels[camera].token())['recorded'])
                session.finish('completed');self.assertTrue(session.done.wait(15),session.snapshot())
                self.assertIsNone(session.error,session.snapshot());self.assertIsNone(session.seal_error)
                manifest,source,plan=recording.verify_raw(session.root,environment='physical')
                self.assertEqual(manifest['capture_status'],'complete',manifest['validation'])
                self.assertEqual(source['clock'],clock_spec('physical'));self.assertIsNone(plan)
                self.assertEqual(manifest['command_count'],30);self.assertEqual(manifest['feedback_count'],101)
                self.assertEqual(manifest['camera_frames'],dict.fromkeys(service.channels,31))
                self.assertTrue(manifest['duration_coverage']['passed'])
                self.assertAlmostEqual(manifest['actual_duration_s'],1.)
                feedback=[json.loads(line) for line in (session.root/'feedback.jsonl').read_text().splitlines()]
                for row,original in zip(feedback,original_feedback):
                    self.assertEqual(row['raw'],original);self.assertEqual(row['sample_wall_ns'],original['sample_wall_ns'])
                    self.assertEqual(row['sample_monotonic_s'],original['sample_time_s'])
                    self.assertEqual(row['clock_domain'],'host_wall');self.assertIsNone(row['source_clock_id'])
                commands=[json.loads(line) for line in (session.root/'commands.jsonl').read_text().splitlines()]
                for row,original in zip(commands,original_commands):
                    self.assertEqual(row['send_wall_ns'],original['send_wall_ns'])
                    self.assertEqual(row['send_monotonic_s'],original['send_monotonic_s'])
                    self.assertEqual(row['feedback_at_send'],original['feedback_at_send'])
                    self.assertEqual(row['clock_domain'],'host_wall');self.assertNotIn('send_source_time_ns',row)
                for camera in service.channels:
                    rows=list(recording.iter_rgbd(session.root,camera,environment='physical'))
                    self.assertEqual(len(rows),31)
                    for index,(_row,metadata,color,depth) in enumerate(rows):
                        self.assertEqual(int(metadata['host_receive_ns']),epoch+index*1_000_000_000//30)
                        with Image.open(io.BytesIO(color)) as image:self.assertEqual(image.tobytes(),h.pixels(index)[0])
                        with Image.open(io.BytesIO(depth)) as image:self.assertEqual(list(image.getdata()),[0,1,32768,65535])
                if export:
                    self.assertEqual(session.snapshot()['storage_format'],'segments')
                    self.workflow._finish_recording(run,dict(status='completed',reason='CPU 1 second complete',
                        inference=dict(submitted_commands=30,completed_commands=30,outcome={'scope':'synthetic CPU fixture'})))
                    self.assertIsNone(run['recording_error'],run['receipt'])
                    playback=run['receipt']['playback']
                    self.assertEqual(playback['recording_kind'],'streaming_policy')
                    self.assertEqual(set(playback['files']),{f'source-{camera}.mp4' for camera in service.channels})
                    for timing in playback['source_timing'].values():
                        self.assertEqual(timing['frame_count'],30)
                        self.assertAlmostEqual(timing['encoded_duration_s'],1.,places=6)
                    self.assertEqual(len(self.workflow.results()),3)
                    self.assertTrue(all(row['status']=='valid' for row in self.workflow.results()))
                    self.device.runtime=None
                    restored=self.make_workflow();self.addCleanup(restored.close)
                    self.assertEqual(len(restored.results()),3)
                    self.assertTrue(all(restored.media(row['id']).is_file() for row in restored.results()))
            finally:
                if not session.done.is_set():session.finish('stopped','CPU physical clock cleanup')
                self.assertTrue(session.done.wait(15));session.thread.join(2)

    def test_physical_recording_rejects_explicit_nonphysical_raw_clock_instead_of_relabeling(self):
        from robot_workbench.sample_clock import clock_spec
        self.device.environment='physical'
        recorder=Mock();run=dict(recorder=recorder,feedback_count=0,source={'clock':clock_spec('physical')},
            execution_owner=dict(owner_pid=123,command_epoch=2,runtime_nonce='fixture-owner'))
        for change in ({'clock_domain':'simulation'},{'clock_domain':None},{'source_clock_id':'not-physical'},{'environment':'fake'},
                       {'simulation_only':True},{'sample_wall_ns':None}):
            raw=dict(sample_time_s=20.,sample_wall_ns=self.wall,**change) if 'sample_wall_ns' not in change else dict(sample_time_s=20.,**change)
            before=deepcopy(raw)
            with self.assertRaises(ValueError):self.workflow._record_event(run,'feedback',dict(raw=raw,device_state14=[0.]*14))
            self.assertEqual(raw,before)
        recorder.append_feedback.assert_not_called();self.assertEqual(run['feedback_count'],0)

    def test_revoked_preparation_cannot_replace_new_owner_recording(self):
        self.prepare();self.workflow.action('inference_preview',{'seed':3})
        config,factory=self.configured[-1]
        with patch('robot_workbench.inference_workflow.make_inference_runner',side_effect=Runner):runner=factory(self.owner,None,self.feedback,config)
        self.owner.runner=runner;run=self.workflow.current;replacement=object()
        payload=dict(clock=config['clock'],execution_owner=dict(owner_pid=123,command_epoch=2,runtime_nonce='fixture-owner'),
            preparation_deadline_monotonic=time.monotonic()+1)
        def late_candidate(*args,**kwargs):
            recorder=Recorder(*args,**kwargs)
            self.owner.execution_instance_id='later-execution';self.owner.recording=replacement
            return recorder
        with patch('robot_workbench.recording_process.ProcessRecordingSession',side_effect=late_candidate):
            with self.assertRaisesRegex(ValueError,'未替换当前 Owner'):self.workflow._record_event(run,'prepare',payload)
        self.assertIs(self.owner.recording,replacement)
        self.assertTrue(run['recorder'].done.is_set());self.assertEqual(run['recorder'].terminal[0],'stopped')


if __name__=='__main__':unittest.main()
