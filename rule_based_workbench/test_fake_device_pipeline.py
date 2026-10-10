"""Explicit fake-device namespace through real CPU callers; no GPU/hardware claim."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import time
import sys

from integration import _digest
from physical_motiongen import freeze_physical
from real_contract import validate_real_plan, validate_real_source
from real_recording import RealRecorder, verify_raw, bundle, export_playback, CAMERAS
from real_observation import decode_snapshot
from test_physical_motiongen import fake_inputs as physical_inputs
import test_playground_motiongen_entry as entry
from test_real_observation import fixture as snapshot
from robot_workbench.sample_clock import clock_spec, owner_identity

FAKE_CLOCK_ID='cpu-fake-device-clock-v1'
FAKE_RUNTIME_NONCE='cpu-fake-runtime-v1'


def fake_metadata(meta, source_ns=1_000_000_000, *, step=None):
    meta.update(environment='fake',simulation_only=True,evidence_source='fake_device_rgbd',timestamp_kind='simulation_step',
        clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,source_time_ns=source_ns,
        physics_step=source_ns*240//1_000_000_000 if step is None else step,host_receive_ns=time.time_ns(),
        device_timestamp_ms=source_ns/1e6,depth_device_timestamp_ms=source_ns/1e6,timestamp_domain='simulation',
        owner_available=True,owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
    return meta


def fake_inputs(source_clock_id=FAKE_CLOCK_ID, runtime_nonce=FAKE_RUNTIME_NONCE):
    """Synthetic GPU/model fixture; use actual acquisition for any L2 qualification."""
    source, model, binding = physical_inputs()
    profile=source['calibration_profile']; profile['source']['simulation_only']=True
    source.update(schema_version='fake_device_collection_source.v1',lane='fake-device',
                  evidence_source='fake_device_rgbd',environment='fake',simulation_only=True)
    source['initial_state']['calibration_profile_digest']=_digest(profile)
    clock=clock_spec('fake',source_clock_id=source_clock_id)
    source['clock']=source['initial_state']['clock']=clock
    for row in source['initial_state']['owner_samples']:
        row.update(clock_domain='simulation',source_clock_id=source_clock_id,
            source_time_ns=round(row['sample_time_s']*1e9),physics_step=round(row['sample_time_s']*240),runtime_nonce=runtime_nonce)
    binding.update(evidence_source='fake_device_rgbd',environment='fake',simulation_only=True,
                   calibration_profile_digest=_digest(profile),source_digest=_digest(source),clock=clock,
                   capture_owner=owner_identity(source['initial_state']['owner_samples'][0],pid_key='pid'))
    model['calibration_profile_digest']=_digest(profile)
    model['planning_validation']['source_digest']=_digest(source)
    from test_physical_motiongen import bind_contact_fixture
    bind_contact_fixture(model,source)
    return source,model,binding


class FakePlaygroundEntryTest(entry.PlaygroundMotionGenEntryTest):
    environment='fake'


class FakeDeviceContractTest(unittest.TestCase):
    def test_cached_retime_retains_source_clock_and_scales_only_the_checked_base(self):
        from playground_plan import retime
        from physical_motiongen import target_vector,vector_target
        from control_motion import validate_control
        source,model,binding=fake_inputs();start=model['ticks'][0]['target']
        targets=[deepcopy(start) for _ in range(6)]
        for index in range(1,5):targets[index]['left_arm_joints'][0]+=.001*min(index,3)
        targets[4]['left_gripper']=64.992
        model.update(ticks=[dict(tick=i,time_s=i/30,target=value) for i,value in enumerate(targets)],
            native_actions=[target_vector(value) for value in targets],native_substeps=[8]*6,duration_s=.2,
            native_identity={'control':deepcopy(model['control'])},gripper_opening_mm=82.992,
            stages=[dict(name=name,start_tick=i,motiongen_receipt_index=0) for name,i in
                [('initial_hold',0),('approach_pre',1),('close_and_hold',4),('home_hold',5)]])
        model['gripper_distance_control']['geometry_derivation']=dict(open_travel_mm=82.992,
            closed_travel_mm=64.992,native_travel_offset_mm=.008,max_metal_gap_mm=83.)
        from cup_presets import bind_plan_preset
        model['native_identity']['physical_configuration']={'fixture_physics':'unchanged'}
        preset=dict(preset_id='soft',stiffness_n_m=329.24414834950727,damping_n_s_m=10.,diagnostic_claw_compression=True)
        bind_plan_preset(model,preset)
        model['timeline_digest']=_digest(model['ticks'])
        model['planning_validation']['timeline_digest']=model['timeline_digest']
        model['control_validation']=validate_control(model)
        from test_physical_motiongen import bind_contact_fixture
        bind_contact_fixture(model,source)
        base=freeze_physical(model,source,binding,environment='fake');before=deepcopy((base,source))
        # Real cached retiming and validators; only native action decoding is a
        # CPU fixture, not a new GPU/geometry qualification.
        action=SimpleNamespace(ac_one_dual_arm_zero_home_action_spec=lambda:None)
        with patch.dict(sys.modules,{'robot_sim.specs.action':action}), \
                patch('isaac_lane.native_target',side_effect=lambda values,*a,**kw:vector_target(values)):
            first=retime(base,source,.3,environment='fake')
            second=retime(base,source,.3,environment='fake')
            self.assertEqual(first,second);self.assertGreater(first['duration_s'],base['duration_s'])
            self.assertEqual(first['cup_contact_preset'],preset)
            self.assertEqual(first['native_identity'],base['native_identity'])
            self.assertEqual((base,source),before)
            validate_real_plan(first,source,binding,environment='fake')
            self.assertEqual(first['physical_gpu_conversion']['model_validation']['derivation']['speed_scale'],.3)
            with self.assertRaisesRegex(ValueError,'original checked base plan'):
                retime(first,source,.3,environment='fake')
            changed=deepcopy(source);changed['initial_state']['clock']=clock_spec('fake',source_clock_id='another-plant')
            with self.assertRaisesRegex(ValueError,'clocks differ'):retime(base,changed,.3,environment='fake')
            changed=deepcopy(source);changed.pop('clock')
            with self.assertRaisesRegex(ValueError,'sealed source clock'):retime(base,changed,.3,environment='fake')

    def test_same_bridge_pipeline_and_capture_readiness_do_not_gate_motion(self):
        from real_bridge import RealBridge, CAMERAS
        from playground import Playground
        source,model,binding=fake_inputs();plan=freeze_physical(model,source,binding,environment='fake')
        with TemporaryDirectory() as folder:
            root=Path(folder)
            for name,value in [('source',source),('plan',plan),('binding',binding)]:
                (root/(name+'.json')).write_text(json.dumps(value))
            (root/'token').write_text('t'*32)
            config=dict(environment='fake',robot_id=source['robot_id'],control_url='http://127.0.0.1:20001',
                token_file=str(root/'token'),source_path=str(root/'source.json'),plan_path=str(root/'plan.json'),
                hardware_binding=str(root/'binding.json'),playground={'enabled':True},
                cameras={camera:dict(url='http://127.0.0.1:'+str(20002+i),serial=serial)
                         for i,(camera,serial) in enumerate(zip(CAMERAS,source['camera_serials'].values()))})
            bridge=RealBridge(config)
            owner=dict(connected=True,mode='simulation',environment='fake',evidence_source='physx',
                feedback_healthy=True,stationary=True,automatic_enabled=False,state='READY',holder='collection',
                recording_configured=False,sample_age_s=.01,pedal=dict(input_healthy=True,physically_down=False))
            cameras_ready=[True]
            def camera(name,path):
                return json.dumps(dict(serial=config['cameras'][name]['serial'],ready=cameras_ready[0])).encode(),'application/json'
            with patch.object(bridge,'owner',side_effect=lambda:deepcopy(owner)),patch.object(bridge,'camera',side_effect=camera):
                state=bridge.status()
                self.assertTrue(state['collection_ready'],state['blockers'])
                self.assertFalse(state['capture_ready'])
                self.assertEqual(state['environment'],'fake')
                service=Playground(root/'workbench',bridge,lambda:{'status':'ready'})
                service.state.update(environment=dict(ready=True,checks=[dict(key='dsw',status='ready')]),plan={'validated':True})
                self.assertTrue(service.snapshot()['can_execute'])
                cameras_ready[0]=False
                state=service.snapshot()
                self.assertTrue(state['can_execute'],'camera/recording readiness became a motion gate')
                self.assertFalse(state['capture_ready'])
                self.assertIn('数据',state['capture_warning'])
                owner['state']='STOPPED_DAMPING_HOLD'
                self.assertTrue(service.snapshot()['can_execute'],'explicit fresh action after stop was incorrectly hidden')
                owner['state']='FAULT_STOP_UNCONFIRMED'
                self.assertFalse(service.snapshot()['can_execute'])
                owner['state']='READY'
                owner['allowed_operations']={'preview':{'allowed':False,'reason':'control command still pending'}}
                self.assertFalse(service.snapshot()['can_execute'])
                self.assertFalse(bridge.status()['collection_ready'])
                owner.pop('allowed_operations')
                owner['holder']='calibration'
                self.assertFalse(service.snapshot()['can_execute'])
                self.assertTrue(any('交接' in reason for reason in bridge.status()['blockers']))
                owner.update(holder='collection',mode='hardware')
                self.assertFalse(service.snapshot()['can_execute'])
                self.assertFalse(bridge.status()['collection_ready'])

    def test_same_model_conversion_and_strict_cross_environment_rejection(self):
        source,model,binding=fake_inputs()
        plan=freeze_physical(model,source,binding,environment='fake')
        validate_real_plan(plan,source,binding,environment='fake')
        self.assertEqual(plan['ticks'][0]['target'],source['initial_state']['planned_reported'])
        self.assertEqual(plan['environment'],'fake')
        self.assertTrue(plan['simulation_only'])
        for call in (lambda:validate_real_source(source,binding),
                     lambda:validate_real_plan(plan,source,binding),
                     lambda:freeze_physical(model,source,binding),
                     lambda:validate_real_plan({**plan,'environment':'physical'},source,binding,environment='fake'),
                     lambda:validate_real_plan(plan,source,{**binding,'environment':'physical'},environment='fake')):
            with self.assertRaises(ValueError):call()
        physical_source,physical_model,physical_binding=physical_inputs()
        physical_plan=freeze_physical(physical_model,physical_source,physical_binding)
        with self.assertRaises(ValueError):validate_real_plan(physical_plan,physical_source,physical_binding,environment='fake')
        with self.assertRaises(ValueError):validate_real_plan(plan,physical_source,physical_binding)

    def test_missing_measured_start_and_changed_model_are_rejected(self):
        source,model,binding=fake_inputs()
        missing=deepcopy(source); missing.pop('initial_state')
        receipt=deepcopy(binding);receipt.pop('initial_state');receipt['source_digest']=_digest(missing)
        with self.assertRaises(ValueError):validate_real_source(missing,receipt,environment='fake')
        model['planning_backend']['native_robot_sha256']='c'*64
        with self.assertRaises(ValueError):freeze_physical(model,source,binding,environment='fake')

    def test_camera_bundles_cannot_cross_environment(self):
        source,_,_=fake_inputs();profile=source['calibration_profile']
        fake=snapshot(1,fake_metadata)
        decode_snapshot(fake,'h',profile,environment='fake')
        bundle(fake,'h',environment='fake')
        with self.assertRaises(ValueError):decode_snapshot(fake,'h',profile)
        with self.assertRaises(ValueError):bundle(fake,'h')
        with self.assertRaises(ValueError):decode_snapshot(snapshot(1),'h',profile,environment='fake')
        with self.assertRaises(ValueError):bundle(snapshot(1),'h',environment='fake')

    def test_raw_schema_and_validation_stay_isolated_even_for_failed_capture(self):
        source,model,binding=fake_inputs();plan=freeze_physical(model,source,binding,environment='fake')
        with TemporaryDirectory() as folder:
            root=Path(folder)
            recorder=RealRecorder(root/'fake',source,plan,binding,environment='fake')
            manifest=recorder.seal('recording_failed','camera unavailable')
            self.assertEqual(manifest['schema_version'],'fake_device_raw_dataset.v1')
            self.assertEqual(manifest['evidence_source'],'fake_device_rgbd')
            self.assertEqual(manifest['capture_status'],'incomplete')
            self.assertEqual(manifest['controller_outcome'],'unverified')
            self.assertEqual(verify_raw(root/'fake',environment='fake')[0],manifest)
            with self.assertRaises(ValueError):verify_raw(root/'fake')
            with self.assertRaises(ValueError):RealRecorder(root/'physical',source,plan,binding)

    def test_complete_fake_raw_and_three_camera_replay_keep_fake_provenance(self):
        from real_rgbd import pair_metadata, snapshot_bundle
        from test_real_recording import png
        from real_workflow import playback
        source,model,binding=fake_inputs();plan=freeze_physical(model,source,binding,environment='fake')
        with TemporaryDirectory() as folder:
            root=Path(folder);raw=root/'raw'
            recorder=RealRecorder(raw,source,plan,binding,environment='fake')
            identity=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
            for row in plan['ticks']:
                stamp=1_000_000_000+row['tick']*10_000_000_000  # Slow wall transport, unchanged simulated cadence.
                source_ns=100_000_000_000+round(row['time_s']*1e9)
                fields=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,physics_step=24000+row['tick']*8,**identity)
                recorder.record_command(row['tick'],row['target'],100+row['time_s'],stamp,send_source_time_ns=source_ns,**(fields|dict(physics_step=None,feedback_at_send={**fields,'source_time_ns':source_ns,'sample_wall_ns':stamp})))
                recorder.record_feedback(row['tick'],row['target'],100+row['time_s'],stamp,source_time_ns=source_ns,**fields)
            recorder.record_feedback(2,plan['ticks'][-1]['target'],100.1,31_000_000_000,
                clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,source_time_ns=100_100_000_000,physics_step=24024,**identity)
            for camera,side in zip(CAMERAS,('head','left','right')):
                for index in range(5):
                    meta=pair_metadata(serial=source['camera_serials'][side],camera_name=side,color_number=index+10,
                        depth_number=index+12,color_ms=index*33,depth_ms=index*33+1,color_domain='simulation',
                        depth_domain='simulation',host_receive_ns=990_000_000+index*33_000_000,
                        depth_scale_m=.001,intrinsics={'fx':600})
                    step=23996+index*8
                    fake_metadata(meta,step*1_000_000_000//240,step=step)
                    meta['host_receive_ns']=900_000_000+index*10_000_000_000
                    recorder.record_rgbd(camera,snapshot_bundle(meta,png(),png(True)))
            manifest=recorder.seal('completed')
            self.assertEqual(manifest['capture_status'],'complete',manifest['validation'])
            scratch_inputs=[];expected_scratch=None
            def encoder(argv,**kwargs):
                if argv[0]=='ffmpeg':
                    concat=Path(argv[argv.index('-i')+1]);scratch_inputs.append(concat)
                    self.assertEqual(concat.parent.parent,expected_scratch)
                    self.assertEqual(len(list(concat.parent.glob(concat.stem+'-*.png'))),5)
                    self.assertTrue(all(path.read_bytes()==png() for path in concat.parent.glob('*.png')))
                    Path(argv[-1]).write_bytes(b'0000ftyp-fake');return SimpleNamespace(stdout='')
                return SimpleNamespace(stdout=json.dumps({'streams':[dict(codec_name='h264',pix_fmt='yuv420p',nb_read_frames='5',duration='.167')]}))
            artifact=root/'artifact'
            expected_scratch=(artifact/'playback').resolve()
            replay=export_playback(raw,None,artifact/'playback',environment='fake',runner=encoder)
            self.assertTrue(all(not path.exists() for path in scratch_inputs))
            scratch=root/'declared-camera-spool';scratch.mkdir();expected_scratch=scratch.resolve()
            retained=scratch/'base_0'/'unconsumed.zip';retained.parent.mkdir();retained.write_bytes(b'retain original spool evidence')
            fast=export_playback(raw,None,artifact/'fast-playback',environment='fake',runner=encoder,scratch_root=scratch)
            self.assertEqual(fast,replay,'temporary location must not change PTS, codec, pixels or manifest')
            self.assertTrue(all(not path.exists() for path in scratch_inputs))
            for error_type in (OSError,InterruptedError):
                def failed_encoder(argv,**kwargs):
                    scratch_inputs.append(Path(argv[argv.index('-i')+1]))
                    raise error_type('export fixture failure')
                with self.assertRaises(error_type):
                    export_playback(raw,None,artifact/error_type.__name__,environment='fake',runner=failed_encoder,scratch_root=scratch)
                self.assertTrue(all(not path.exists() for path in scratch_inputs))
                self.assertEqual(retained.read_bytes(),b'retain original spool evidence')
                self.assertEqual(list(scratch.iterdir()),[retained.parent])
            for invalid in (Path('relative-scratch'),root/'missing-scratch',raw):
                with self.assertRaisesRegex(ValueError,'fake playback scratch'):
                    export_playback(raw,None,artifact/'invalid-scratch',environment='fake',runner=encoder,scratch_root=invalid)
            with self.assertRaisesRegex(ValueError,'fake playback scratch'):
                export_playback(raw,None,artifact/'physical-scratch',runner=encoder,scratch_root=scratch)
            self.assertEqual(replay['schema_version'],'fake_device_workbench_playback.v1')
            self.assertEqual(replay['provenance'],{'source':'fake_device_rgbd'})
            self.assertEqual(replay['source_timing']['base_0']['clock'],'simulation')
            self.assertLess(replay['source_timing']['base_0']['duration_s'],.2)
            pairs=json.loads((raw/'frame-feedback-pairs.json').read_text())['pairs']
            self.assertTrue(pairs)
            self.assertTrue(all(row['association']=='feedback_bracket' and row['interpolated'] for row in pairs))
            config=dict(environment='fake',robot_id=source['robot_id'],render_replay=False,artifact_dir=str(artifact),
                source_path=str(raw/'source.json'),plan_path=str(raw/'plan.json'),hardware_binding=str(raw/'binding.json'),
                cameras={camera:{'serial':source['camera_serials'][side]} for camera,side in zip(CAMERAS,('head','left','right'))})
            self.assertEqual(playback(config)[0]['status'],'ready')
            self.assertEqual(playback({**config,'environment':'physical'})[0]['status'],'unavailable')

    def test_target_replay_requires_its_explicit_environment(self):
        from playback import validate_target, sha256
        source,model,binding=fake_inputs();plan=freeze_physical(model,source,binding,environment='fake')
        with TemporaryDirectory() as folder:
            root=Path(folder);files={}
            for camera in CAMERAS:
                path=root/('target-'+camera+'.mp4');path.write_bytes(b'CPU fixture video bytes')
                files[path.name]=dict(size_bytes=path.stat().st_size,sha256=sha256(path),probe=dict(
                    codec_name='h264',pix_fmt='yuv420p',width=640,height=480,r_frame_rate='30/1',nb_read_frames='3'))
            manifest=dict(schema_version='fake_device_target_playback.v1',evidence_source='simulation',
                source_run_id=source['run_id'],plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'],camera_frames=3,files=files)
            (root/'manifest.json').write_text(json.dumps(manifest))
            self.assertEqual(validate_target(root,source,plan,environment='fake'),manifest)
            with self.assertRaises(ValueError):validate_target(root,source,plan)
            with self.assertRaises(ValueError):validate_target(root,source,{**plan,'environment':'physical'},environment='fake')


if __name__=='__main__':unittest.main()
