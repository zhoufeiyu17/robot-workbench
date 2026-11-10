"""Actual Owner/runner and resident ROS recording with an ideal CPU plant."""
from copy import deepcopy
import os
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),
             str(ROOT/'calibration/tests'),str(ROOT/'rule_based_workbench')]


class EventRouteTests(unittest.TestCase):
    def test_missing_ros_feedback_is_visible_without_relabelling_hardware_health(self):
        from robot_workbench.service import Device
        device=Device('cpu','fake',None)
        device.runtime=SimpleNamespace(fact_snapshot=lambda:dict(connected=True,feedback_healthy=True,
            automatic_enabled=False,feedback_source_error='referenced sample missing'))
        state=device.snapshot(None)
        self.assertIn('referenced sample missing',state['service_error'])
        self.assertTrue(state['feedback_healthy'])

    def test_ros_fact_uses_one_producer_route(self):
        from robot_workbench.inference_owner import InferenceRunner
        runner=object.__new__(InferenceRunner);runner.config={'data_plane':{}}
        runner.execution_instance_id='run';runner.owner=SimpleNamespace(event=Mock())
        runner.event=Mock();runner.record=Mock();runner._record=Mock()
        runner._emit('prediction_received',{'request_id':'request'})
        runner.owner.event.assert_not_called();runner.event.assert_not_called()
        runner._record.assert_called_once_with('prediction_received',dict(request_id='request',execution_instance_id='run'))


@unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
class RosRunTests(unittest.TestCase):
    def test_default_fake_control_devices_use_one_owner_and_simulation_clock(self):
        from scripts.serve_physx_device import control_devices
        from robot_workbench.runtime import Runtime
        from test_owner_clock import calibration,Safety
        from test_physx_camera import PhysxCameraTest,CAMERAS
        from robot_workbench.physx import SimulationClock
        from robot_workbench.collection_setup import _ros_connection,connection_config
        from real_observation import decode_snapshot
        with tempfile.TemporaryDirectory()as folder:
            root=Path(folder);sim=SimulationClock();created=[]
            plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_fake_startup',domain_id=126)
            camera=PhysxCameraTest();camera.setUp();self.addCleanup(camera.doCleanups)
            backend=camera.backend;producer=camera.producer(ros_plane=plane,device_id='cpu_fake')
            def acquire():
                runtime=Runtime(calibration,Safety,lambda:backend,device_id='cpu_fake',environment='fake',
                    lock_root=root/'locks',control_clock=sim,event=service.event,data_plane=plane,
                    initialize_backend=lambda value:value.set_damping_hold(manual=True))
                from robot_workbench.recording_process import capture_control
                def recording(payload,on_fault):
                    return capture_control(root/'runs',payload,cameras=None,environment='fake',
                        clock=lambda:None,source_time=runtime.owner.control_time,on_fault=on_fault,ros_runtime=runtime)
                recording.recording_backend='capture_session_v1';runtime.owner.recording_factory=recording
                backend.owner=runtime.owner;created.append(runtime);return runtime
            owner,service,business=control_devices('cpu_fake',root,acquire)
            def advance():
                runtime=owner.runtime
                actor=dict(owner_available=bool(runtime and runtime.owner.feedback_healthy),owner_pid=os.getpid(),
                    runtime_nonce=runtime.nonce if runtime else None,command_epoch=runtime.owner.epoch if runtime else None)
                for _ in range(12):
                    sim.advance(runtime or SimpleNamespace(tick=lambda:None),backend,
                        after_physics=lambda:producer.collect(camera.plant,backend,actor))
            errors=[]
            def connect():
                try:business.connect({})
                except Exception as error:errors.append(error)
            thread=threading.Thread(target=connect)
            try:
                self.assertEqual(created,[],'startup must not acquire the plant')
                thread.start();deadline=time.monotonic()+8
                while thread.is_alive()and time.monotonic()<deadline:
                    advance()
                    time.sleep(.005)
                thread.join(.1);self.assertFalse(thread.is_alive());self.assertEqual(errors,[])
                self.assertEqual(len(created),1)
                self.assertTrue(business.runtime.remote_control_owner)
                self.assertIs(owner.runtime,created[0])
                self.assertEqual(business.runtime.nonce,owner.runtime.nonce)
                value=business.runtime.control_inputs()
                self.assertEqual(value['feedback_reference'],value['calibration_feedback']['ros_reference'])
                self.assertEqual(value['feedback_reference']['source_clock_id'],backend.source_clock_id)
                from real_bridge import RealBridge
                token=root/'reader-token';token.write_text('test-only-'+40*'x')
                setup=dict(environment='fake',robot_id='cpu_fake',resident_root=str(root),
                    token_file=str(token),control_url='http://127.0.0.1:9',trusted_binding_path=str(root/'binding'),
                    local_recording_root=str(root/'recording'),producer_ports=dict(head=9,left=9,right=9),
                    camera_serials={role:camera.k[name]['serial']for role,name in CAMERAS.items()})
                setup.update(_ros_connection(plane,dict(descriptor=str(service.descriptor_path)),'cpu_fake','fake'))
                config=connection_config(setup,root/'pipeline',root/'calibration',root/'capture',root/'jobs')
                self.assertTrue(all('recording_directory'not in row for row in config['cameras'].values()))
                bridge=RealBridge(config)
                try:
                    bridge.request=lambda *a,**k:self.fail('ROS state must not cross the legacy HTTP snapshot')
                    deadline=time.monotonic()+3
                    while True:
                        advance();received=bridge.owner()
                        if received.get('calibration_feedback'):break
                        self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                    self.assertEqual(received['feedback_reference'],received['calibration_feedback']['ros_reference'])
                    self.assertEqual(received['runtime_nonce'],owner.runtime.nonce)
                    deadline=time.monotonic()+5
                    while True:
                        advance()
                        try:body,_=bridge.camera('base_0','snapshot.zip');break
                        except ValueError:
                            self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                    decoded=decode_snapshot(body,camera.k['base_0']['serial'],{'frames':{'environment_from_camera':
                        {'position':[0,0,0],'orientation_wxyz':[1,0,0,0]}}},environment='fake')
                    self.assertEqual(decoded[0]['source_clock_id'],backend.source_clock_id)
                    self.assertTrue(np.all(decoded[-1]['depth_m']==.5))
                    for name in CAMERAS.values():self.assertEqual(producer.ros.node.count_publishers(plane['namespace']+'/'+name),1)
                finally:bridge.close()
                self.assertLess(business.runtime.owner.control_time(),100.)
                self.assertEqual(owner.runtime.ros_node.node.count_publishers('/wb_fake_startup/state'),1)
                business.request_stop();owner.runtime.tick()
                self.assertFalse(owner.runtime.fact_snapshot()['automatic_enabled'])
            finally:
                if business.runtime is not None:business.runtime.detach();business.closed.set()
                if owner.runtime is not None:
                    owner.runtime.owner.submit(dict(action='stop'));owner.runtime.tick()
                    owner.runtime.owner.submit(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))
                    owner.runtime.tick();owner.runtime.close();owner.closed.set()
                    if owner.thread is not None:owner.thread.join(2)
                    owner.runtime=None;owner.credentials={}
                service.close()

    def test_other_businesses_share_ros_capture_and_resident_receipts(self):
        for business in ('calibration','collection','research'):
            with self.subTest(business=business):self.run_control_business(business)

    def test_previous_run_seal_event_cannot_break_current_control_source_barrier(self):
        self.run_control_business('collection',previous_seal_event=True)

    def test_source_ready_transition_does_not_wait_for_periodic_health_heartbeat(self):
        self.run_control_business('calibration',deferred_health_heartbeat=True)

    def test_rejected_recorder_stops_without_output_and_releases_maintenance_gate(self):
        self.run_control_business('calibration',reject_registration=True)

    def test_codec_failure_seals_verified_prefix_and_releases_maintenance_gate(self):
        self.run_control_business('calibration',missing_codec=True)

    def run_control_business(self,business,*,reject_registration=False,missing_codec=False,previous_seal_event=False,deferred_health_heartbeat=False):
        from sensor_msgs.msg import Image
        from robot_workbench.ros_data import RosNode,descriptor_message
        from robot_workbench.recording_process import capture_control
        from robot_workbench.sample_clock import clock_spec
        from test_recording_process import start_recording_fixture,stop_recording_fixture
        plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_businesses',domain_id=126)
        fixture=start_recording_fixture();harness=None;camera=None;recording=None;service=None;consumer=None;index=None;serving=None
        limited=None;limited_thread=None;descriptor_patch=None
        try:
            if business=='collection':
                from test_unified_runtime import UnifiedRuntimeTest
                harness=UnifiedRuntimeTest();harness.setUp();harness.ros_plane=plane
                runtime,credentials,command=harness.collection_case('ros-rule-collection')
                now=lambda:harness.sim;advance=lambda:harness.advance(runtime)
                confirm=lambda review:harness.confirm(runtime,credentials[business],review)
            else:
                from test_shared_target_routes import SharedTargetRoutesTest
                harness=SharedTargetRoutesTest();harness.setUp();harness.ros_plane=plane;harness.create()
                runtime=harness.runtime;credentials=harness.creds;now=lambda:harness.now;advance=harness.advance
                if business=='research':
                    credentials[business]=runtime.register_business(business)
                    runtime.handoff(credentials['calibration'],business);harness.advance(42)
                    policy=runtime.control_policy_snapshot(business)
                    runtime.update_control_policy(credentials[business],dict(expected_revision=policy['revision'],
                        changes={'recording.purpose_profile':'data_required'}))
                command=dict(action='preview_workbench',waypoints=[dict(q12=[.02]*12,speeds12=[.1]*12)])
                if missing_codec:command['waypoints'][0]['speeds12']=[.005]*12
                confirm=lambda review:harness.confirm(business,review)
            camera=RosNode('business_camera',namespace=plane['namespace'],domain_id=126)
            publishers={};declarations=[];clock=runtime.owner.execution_clock_id
            for name in ('base_0','left_wrist_0','right_wrist_0'):
                source=dict(device_id=runtime.device_id,stream_id=name,source_instance_id='camera',source_clock_id=clock,
                    topic=plane['namespace']+'/'+name,message_type='sensor_msgs/msg/Image',kind='image',units='rgb8',
                    coordinate_frame=name,nominal_hz=30.,available=True,metadata=dict(width=2,height=2,environment='fake'))
                declared=camera.publisher('sources/'+name,camera.types.SourceDescriptor,retained=True,depth=1)
                declared.publish(descriptor_message(camera,source));declarations.append(declared)
                publishers[name]=camera.publisher(source['topic'],Image)
            record_root=Path(harness.temp.name if hasattr(harness,'temp')else harness.directory.name)/'ros-runs'
            if reject_registration or missing_codec or deferred_health_heartbeat:
                from unittest.mock import patch
                from robot_workbench.recording_process import RecordingService
                allowed=record_root/'allowed'if reject_registration else record_root
                allowed.mkdir(parents=True)
                python=None
                if missing_codec:
                    python=record_root.parent/'missing-codec-python'
                    python.write_text('#!'+sys.executable+'\n'+
                        'import sys\nimport rgbd_segments as segments\n'+
                        'Original=segments.SegmentWriter\n'+
                        'class MissingCodec(Original):\n'+
                        ' def __init__(self,*args,**kwargs):\n'+
                        '  kwargs["ffmpeg"]="/deliberately-missing-recorder-codec"\n'+
                        '  super().__init__(*args,**kwargs)\n'+
                        'segments.SegmentWriter=MissingCodec\n'+
                        'from robot_workbench.recording_process import _capture_worker\n'+
                        '_capture_worker(recover=sys.argv[-1]=="capture-recover")\n')
                    python.chmod(0o700)
                elif deferred_health_heartbeat:
                    python=record_root.parent/'deferred-health-python'
                    python.write_text('#!'+sys.executable+'\n'+
                        'import sys,threading\nwait=threading.Event.wait\n'+
                        'def defer(self,timeout=None):\n'+
                        ' if threading.current_thread().name=="recording-health" and timeout==.2:timeout=2.\n'+
                        ' return wait(self,timeout)\n'+
                        'threading.Event.wait=defer\n'+
                        'from robot_workbench.recording_process import _capture_worker\n'+
                        '_capture_worker(recover=sys.argv[-1]=="capture-recover")\n')
                    python.chmod(0o700)
                limited=RecordingService(record_root.parent/'limited-recorder',data_roots=[allowed],python=python)
                limited_thread=threading.Thread(target=limited.serve_forever,daemon=True);limited_thread.start()
                descriptor_patch=patch.dict(os.environ,WORKBENCH_RECORDING_SERVICE=str(limited.descriptor_path))
                descriptor_patch.start()
            def factory(payload,on_fault):
                capture=capture_control(record_root,payload,cameras=None,environment='fake',
                    clock=clock_spec('fake',source_clock_id=clock),source_time=now,on_fault=on_fault,ros_runtime=runtime)
                if previous_seal_event:
                    flush=capture.capture.ros_watermarks
                    def watermark():
                        runtime.ros_publisher.event('control_capture_sealed',{'execution_instance_id':'previous_run'},
                            run_id='previous_run',source_time_ns=round(now()*1e9))
                        return flush()
                    capture.capture.ros_watermarks=watermark
                return capture
            factory.recording_backend='capture_session_v1';factory.supports_deferred_preparation=True
            runtime.owner.recording_factory=factory
            from robot_workbench.service import Device
            from robot_workbench.control_owner import ControlOwnerService,ControlRuntimeClient
            from robot_workbench.collection_history import CaptureResults
            device=Device(runtime.device_id,'fake',None);device.runtime=runtime;device.credentials=credentials
            service=ControlOwnerService(device,record_root.parent/'result-owner')
            serving=threading.Thread(target=service.serve_forever,daemon=True);serving.start()
            consumer=ControlRuntimeClient(service.descriptor_path)
            index=CaptureResults(record_root,environment='fake',device_id=runtime.device_id);notifications=[]
            def sealed(value):
                notifications.append(value)
                # This synthetic old notification has no archived old Run;
                # index only real receipts while still checking its delivery.
                if previous_seal_event and value and value.get('execution_instance_id')=='previous_run':return
                index.notify(value)
            consumer.set_capture_result_handler(sealed)
            last=[None]
            def publish():
                if deferred_health_heartbeat:
                    current=getattr(runtime.owner,'recording',None)
                    paths=list((current.capture.directory/'capture').glob('capture-*/status.json'))if current else[]
                    # Start real DDS frames only after the writer has emitted
                    # its initial not-ready state. Keep the Owner's 1s budget.
                    if not paths or 'health_source'not in json.loads(paths[0].read_text()):return
                stamp=round(now()*30)*1_000_000_000//30
                if stamp==last[0]:return
                last[0]=stamp
                for writer in publishers.values():
                    frame=Image(width=2,height=2,step=6,encoding='rgb8',data=bytes(range(12)))
                    frame.header.stamp.sec=stamp//10**9;frame.header.stamp.nanosec=stamp%10**9;writer.publish(frame)
            publish();review=runtime.submit(credentials[business],command);confirm(review);direct_health=False
            deadline=time.monotonic()+25
            while time.monotonic()<deadline:
                advance();publish();recording=runtime.owner.recording
                if recording is not None and recording.done.is_set():break
                if recording is not None and recording.ready.is_set()and not direct_health:
                    wire=consumer.channel.call('snapshot')['recording']
                    self.assertNotIn('streams',wire);self.assertNotIn('ready',wire)
                    info=consumer.refresh().get('recording')or{}
                    direct_health=bool(info.get('streams')and consumer.recording_health is not None)
                time.sleep(.005)
            self.assertIsNotNone(recording)
            self.assertTrue(recording.done.is_set(),runtime.fact_snapshot())
            if reject_registration or missing_codec:
                for _ in range(42):advance();publish();time.sleep(.002)
                self.assertEqual(runtime.owner.runner.status,'STOPPED')
                if reject_registration:
                    self.assertFalse(runtime.owner.backend._backend.packets,'rejected recording must precede motion')
                    self.assertIn('outside registered data roots',recording.error)
                    self.assertIsNone(recording.capture.session.process)
                else:
                    self.assertEqual(recording.manifest['recording_outcome'],'incomplete')
                    self.assertGreater(recording.manifest['streams']['state']['committed'],0)
                    self.assertTrue(any('deliberately-missing-recorder-codec'in reason
                                        for reason in recording.manifest['issues']),recording.manifest['issues'])
                self.assertFalse(recording.thread.is_alive())
                self.assertTrue(runtime.fact_snapshot()['idle_readiness']['allowed'],runtime.fact_snapshot())
                runtime.begin_maintenance();runtime.end_maintenance()
                receipt=json.loads((recording.capture.directory/'receipt.json').read_text())
                self.assertEqual(receipt['controller']['status'],'failed')
                self.assertEqual(receipt['recording']['recording_outcome'],'incomplete')
                indexed_deadline=time.monotonic()+5
                while not notifications or not index.page(business)['results']:
                    self.assertLess(time.monotonic(),indexed_deadline,dict(notifications=notifications,index_error=index.error))
                    time.sleep(.01)
                self.assertIsNone(index.error)
                self.assertEqual(index.page(business)['results'][0]['details']['recording_outcome'],'incomplete')
                return
            if runtime.owner.runner.status!='READY':
                folder=recording.capture.session.directory if recording.capture.session else recording.root
                boundaries={name:json.loads((folder/name).read_text())for name in ('finish-request.json','finish.json')if(folder/name).is_file()}
                self.fail(dict(reason=runtime.owner.runner.reason,error=recording.diagnostics_error,
                    missing=recording.snapshot().get('missing_streams'),source_errors=recording.snapshot().get('source_errors'),
                    boundaries=boundaries,owner_execution_id=runtime.owner.execution_instance_id,
                    capture_execution_id=recording.execution_instance_id,publisher_sequences=runtime.ros_publisher.sequences))
            self.assertEqual(recording.snapshot()['recording_outcome'],'complete',recording.snapshot())
            self.assertEqual(recording.manifest['binding']['business_kind'],business)
            self.assertTrue((recording.capture.directory/'receipt.json').is_file())
            self.assertEqual(len(recording.manifest['playback']['streams']),3)
            self.assertGreater(recording.manifest['streams']['state']['committed'],0)
            events=[json.loads(row)['payload']for row in (recording.root/'control.jsonl').read_text().splitlines()]
            self.assertTrue(any(row['event']=='command'and row['data'].get('phase')=='returned'for row in events))
            if previous_seal_event:
                old=[row for row in events if row['event']=='control_capture_sealed']
                self.assertEqual(len(old),1)
                self.assertEqual(old[0]['run_id'],'previous_run','preserve original attribution inside the captured source window')
            self.assertEqual(recording.capture.session.publish_calls,0)
            if business=='collection':self.assertTrue((recording.root/'artifacts.jsonl').read_text().strip())
            deadline=time.monotonic()+5
            while not notifications or not index.page(business)['results']:
                self.assertLess(time.monotonic(),deadline,dict(notifications=notifications,index_error=index.error,
                    ros_errors=list(consumer.ros_state.ros.errors)))
                time.sleep(.01)
            self.assertEqual(len(notifications),2 if previous_seal_event else 1)
            self.assertTrue(direct_health,'business/UI must read original ROS health during the Run')
            self.assertEqual(notifications[-1]['execution_instance_id'],recording.execution_instance_id)
            self.assertIsNone(consumer.event_thread,'sealed data events must not use RPC polling')
            self.assertIsNone(index.error)
        finally:
            if recording is not None and not recording.done.is_set():recording.close()
            if consumer is not None:consumer.detach()
            if index is not None:index.close()
            if descriptor_patch is not None:descriptor_patch.stop()
            if limited is not None:
                limited.dispatch(dict(action='shutdown'));limited_thread.join(3);limited.close()
            if harness is not None:harness.doCleanups()
            if service is not None:
                service.device.runtime=None;service.close();serving.join(3)
            if camera is not None:camera.close()
            stop_recording_fixture(fixture)

    def test_complete_run_has_one_event_stream_and_resident_artifacts(self):
        self.run_episode(stop_during_model=False)

    def test_completed_owner_binding_and_recording_facts_retire_without_gc(self):
        self.run_episode(stop_during_model=False,check_retirement=True)

    def test_stop_during_model_does_not_apply_late_output_and_still_seals(self):
        self.run_episode(stop_during_model=True)

    def run_episode(self,*,stop_during_model,check_retirement=False):
        from sensor_msgs.msg import Image
        from robot_workbench.ros_data import RosNode,SourceDirectory,descriptor_message
        from robot_workbench.ros_inputs import RosInputs
        from robot_workbench.ros_inference import RosModelWorker
        from robot_workbench.ros_recording import capture_source
        from robot_workbench.runtime import Runtime
        from robot_workbench.owner_inference import OwnerInference
        from robot_workbench.inference_workflow import calibrated_adapter,CAMERAS
        from robot_workbench.inference_owner import device_state
        from robot_workbench.inference_trace import CaptureRun
        from robot_workbench.observation_pairing import TimestampPairer
        from robot_workbench.execution_settings import Guard,TRAINING_RGB_PAIR_ERROR_S,TRAINING_STATE_PAIR_ERROR_S
        from test_shared_target_routes import WindowBackend
        from test_owner_clock import calibration,Safety,jaw
        from test_physical_motiongen import fake_inputs
        from test_recording_process import start_recording_fixture,stop_recording_fixture

        fixture=start_recording_fixture();nodes=[];runtime=None;bridge=None;model=None;capture=None;gc_enabled=None
        closed=threading.Event();threads=[];failures=[];model_started=threading.Event();model_continue=threading.Event()
        try:
            with tempfile.TemporaryDirectory()as directory:
                root=Path(directory);plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_actual_run',domain_id=126)
                jaw.configure_mapping(None)
                backend=WindowBackend(time.monotonic);backend.target_interpreter_profile='x5_shared_precision_v1'
                backend.q[6]=jaw.feedback_sdk('left',82.);backend.q[13]=jaw.feedback_sdk('right',82.)
                runtime=Runtime(calibration,Safety,lambda:backend,device_id='cpu_robot',environment='fake',
                    lock_root=root/'locks',initial_holder='inference',control_clock=time.monotonic,
                    initialize_backend=lambda b:b.set_damping_hold(manual=True),data_plane=plane)
                published=[];enqueue=runtime.ros_publisher._enqueue
                def traced_enqueue(stream,stamp,value):
                    with runtime.ros_publisher.lock:
                        if stream=='control':published.append((runtime.ros_publisher.sequences[stream]+1,
                            value['event'],value['run_id']))
                        return enqueue(stream,stamp,value)
                runtime.ros_publisher._enqueue=traced_enqueue
                credential=runtime.register_business('inference')
                def drive():
                    backend.control_thread=threading.get_ident()
                    while not closed.wait(.01):
                        try:runtime.tick()
                        except Exception as error:failures.append(error);return
                threads.append(threading.Thread(target=drive));threads[-1].start()
                camera=RosNode('camera',namespace=plane['namespace'],domain_id=126);nodes.append(camera)
                sources=[];publishers={};declarations=[]
                for role,name in CAMERAS.items():
                    source=dict(device_id='cpu_robot',stream_id=name,source_instance_id='camera',
                        source_clock_id=backend.source_clock_id,topic=plane['namespace']+'/'+name,
                        message_type='sensor_msgs/msg/Image',kind='image',units='rgb8',coordinate_frame=role,
                        nominal_hz=30.,available=True,metadata=dict(width=224,height=224,serial=role,environment='fake'))
                    sources.append(source);pub=camera.publisher('sources/'+name,camera.types.SourceDescriptor,retained=True,depth=1)
                    pub.publish(descriptor_message(camera,source));declarations.append(pub)
                    publishers[name]=camera.publisher(source['topic'],Image)
                def frames():
                    while not closed.wait(1/30):
                        stamp=time.monotonic_ns()
                        for pub in publishers.values():
                            value=Image(width=224,height=224,encoding='rgb8',step=672,data=bytes([10,20,30])*(224*224))
                            value.header.stamp.sec=stamp//10**9;value.header.stamp.nanosec=stamp%10**9;pub.publish(value)
                threads.append(threading.Thread(target=frames));threads[-1].start()
                node=RosNode('input_model',namespace=plane['namespace'],domain_id=126);nodes.append(node)
                delivered=[]
                node.subscribe('control/events',node.types.DataEvent,lambda message:delivered.append(message.reference.sequence))
                registry=SourceDirectory(node,'cpu_robot')
                sources+=registry.resolve({'state':{},'control':{}})
                inputs=RosInputs(node,[r for r in sources if r['kind']in('state','image')])
                source,_,_=fake_inputs();profile=source['calibration_profile'];profile['gripper']['max_opening_mm']=83.
                adapter,mapping=calibrated_adapter(profile,runtime.control_inputs()['joint_limits'])
                contract=dict(shape=[60,14],axes=[f'{s}_{a}'for s in('left','right')for a in('j1','j2','j3','j4','j5','j6','jaw')],
                    units=(['rad']*6+['m'])*2,action_type='absolute_position',normalized=False,device_mapping_applied=False)
                binding=dict(session_id='cpu_model',model_binding={'id':'cpu'},conversion_id='conversion',action_contract=contract,task='cpu')
                config=dict(model_id='base379-epoch010-step29450',seed=1,duration_s=1.,execute_steps=15,
                    maximum_observation_age_s=.5,recording_enabled=False,recording_backend='capture_session_v1',
                    disabled_grippers=[],conversion_identity={'mapping':mapping},action_chunk_binding=binding,data_plane=plane,
                    camera_capture_instances={r:'camera'for r in CAMERAS},camera_serials={r:r for r in CAMERAS})
                def predict(request,prepared):
                    model_started.set()
                    if stop_during_model:model_continue.wait(2)
                    actions=np.tile(prepared[0],(60,1));actions[:,0]+=.002*np.linspace(0,1,60)
                    return dict(binding,schema='workbench.action-chunk.v1',request_id=request['request_id'],epoch=request['epoch'],
                        action14=actions,execution_authorized=False,audit={},
                        observation=dict(source_time=request['observation']['source_time'],clock_domain=request['clock_domain'],
                            frame_ids=request['observation']['frame_ids']),
                        validity=dict(valid_until_source_time=request['valid_until_source_time'],clock_domain=request['clock_domain']))
                model=RosModelWorker(node,inputs,adapter.model_input,predict,binding=binding,
                    clock_id=backend.source_clock_id,state_to_device=lambda raw:device_state(raw['qpos']))
                sources+=model.source_descriptors('cpu_robot')
                all_sources=[capture_source(r,namespace=plane['namespace'],domain_id=126)for r in sources]
                def event(event_name,**value):
                    nonlocal capture
                    if event_name!='owner_inference':return
                    if value['kind']=='created':
                        identity=value['execution_instance_id'];model.bind(identity,runtime.owner.epoch)
                        def sealed(record):
                            bridge.update('inference_recording',dict(prepared=True,done=True,alive=False,error=record.error,
                                execution_instance_id=record.execution_instance_id,ingress_closed=record.ingress_closed))
                        capture=CaptureRun(identity,root/('inference-'+identity),
                            dict(value['configuration'],execution_context=dict(device_id='cpu_robot',environment='fake')),
                            value['control_policy'],None,'fake',None,lambda:True,sealed,
                            ros_sources=lambda:all_sources,ros_watermarks=model.watermarks)
                    elif value['kind']=='record':
                        capture.record(value['name'],value['value'])
                        if value['name']=='prepare':
                            bridge.update('inference_recording',dict(prepared=True,done=False,alive=True,error=capture.error,
                                execution_instance_id=capture.execution_instance_id,ingress_closed=False))
                device=SimpleNamespace(id='cpu_robot',environment='fake',runtime=runtime)
                service=SimpleNamespace(device=device,event=event,reference_executor=None)
                bridge=OwnerInference(service,config,dict(profile=profile,reference=None,preloaded_prediction=None,feedforward=None))
                policy=runtime._guard_policy('inference')
                pairer=TimestampPairer(Guard.bind(policy,dict(training_rgb_pair_error_s=TRAINING_RGB_PAIR_ERROR_S,
                                                            training_state_pair_error_s=TRAINING_STATE_PAIR_ERROR_S)))
                observations=node.publisher('observations',node.types.Observation)
                def observe():
                    while not closed.wait(.01):
                        try:
                            refs,state,stamp,pairing=inputs.paired({c+'_rgb':c for c in CAMERAS.values()},pairer)
                            observations.publish(inputs.observation(refs,state,run_id='',epoch=runtime.owner.epoch,
                                model_io_digest='conversion',source_time_ns=stamp,pairing=pairing))
                        except (ValueError,LookupError):pass
                threads.append(threading.Thread(target=observe));threads[-1].start()
                deadline=time.monotonic()+5
                while (not runtime.owner.stationary or bridge._ros_observe(time.monotonic())is None)and time.monotonic()<deadline:time.sleep(.01)
                runtime.configure_inference(config,bridge.factory)
                review=runtime.submit(credential,dict(action='preview_inference',seed=1,duration_s=1.))
                runtime.submit(credential,dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
                self.assertTrue(model_started.wait(8),dict(failures=list(map(str,failures)),
                    runner=runtime.owner.runner.snapshot()if runtime.owner.runner else None,
                    capture=capture.snapshot()if capture else None,model_binding=model.current,
                    observation_error=bridge.ros_error,ros_errors=list(node.errors)))
                if stop_during_model:
                    runtime.submit(credential,dict(action='stop'));time.sleep(.05)
                    packets=len(backend.packets);model_continue.set();time.sleep(.1)
                    self.assertEqual(len(backend.packets),packets,'late prediction produced an SDK window after Stop')
                deadline=time.monotonic()+12
                while (capture is None or not capture.done.is_set())and time.monotonic()<deadline:time.sleep(.02)
                self.assertEqual(failures,[])
                self.assertIsNotNone(capture);self.assertTrue(capture.done.is_set(),capture.snapshot())
                self.assertIsNone(capture.error,capture.snapshot())
                details=dict(issues=capture.manifest.get('issues'),source_errors=capture.session.snapshot().get('source_errors'),
                    publisher_error=runtime.ros_publisher.error,publisher_dropped=runtime.ros_publisher.dropped,
                    final=json.loads((capture.root/'finish-request.json').read_text()),
                    seen=json.loads((capture.root/'finish.json').read_text()).get('source_watermarks'))
                if capture.manifest['recording_outcome']!='complete':
                    details['control_events']=[(row['payload']['reference']['sequence'],row['payload']['event'])
                        for row in map(json.loads,(capture.root/'control.jsonl').read_text().splitlines())]
                    received={row[0]for row in details['control_events']}
                    details['missing_control_events']=[row for row in published if min(received)<row[0]<max(received)and row[0]not in received]
                    details['seen_by_other_subscriber']=[row[0]for row in details['missing_control_events']if row[0]in delivered]
                self.assertEqual(capture.manifest['recording_outcome'],'complete',details)
                self.assertTrue(list(capture.root.rglob('output.npz')))
                self.assertFalse(list(root.rglob('input.npz')))
                if not stop_during_model:
                    self.assertGreater(len(backend.packets),0)
                    self.assertEqual(runtime.owner.runner.config['execute_steps'],15)
                    self.assertEqual(capture.manifest['control_terminal']['status'],'completed')
                if check_retirement:
                    import gc,weakref
                    gc_enabled=gc.isenabled();gc.collect();gc.disable()
                    def restore_gc():
                        if gc_enabled:gc.enable()
                        gc.collect()
                    self.addCleanup(restore_gc)
                    runtime.revoke_inference()
                    retired=weakref.ref(bridge);facts=weakref.ref(runtime.owner.recording)
                    bridge.close();runtime.owner.recording=None;bridge=None
                    self.assertTrue(backend.is_connected,'retirement must be safe while the Owner stays connected')
                    self.assertEqual({'binding':retired()is not None,'recording':facts()is not None},
                        {'binding':False,'recording':False})
                    for _ in range(16):
                        bridge=OwnerInference(service,config,dict(profile=profile,reference=None,preloaded_prediction=None,feedforward=None))
                        runtime.configure_inference(config,bridge.factory);runtime.revoke_inference()
                        retired=weakref.ref(bridge);bridge.close();bridge=None
                        self.assertIsNone(retired(),'repeated model binding must release the old ROS callback graph')
        finally:
            model_continue.set();closed.set()
            for thread in threads:thread.join(5)
            if bridge is not None:bridge.close()
            if model is not None:model.close()
            if runtime is not None:
                runtime.owner.backend.is_connected=False;runtime.owner.recording=None;runtime.close()
            for node in reversed(nodes):node.close()
            stop_recording_fixture(fixture)


if __name__=='__main__':unittest.main()
