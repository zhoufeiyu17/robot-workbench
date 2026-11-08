"""Resident recorder subscribes to real DDS; the business sends lifecycle only."""
from pathlib import Path
from contextlib import redirect_stdout
from copy import deepcopy
from io import StringIO
import json
import os
import sys
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch,Mock
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench')]
from robot_workbench.ros_data import RosNode, StreamRef
from robot_workbench.ros_inputs import RosInputs
from robot_workbench.ros_inference import RosModelClient,RosModelWorker
from robot_workbench.ros_recording import capture_source
from robot_workbench.recording_process import CaptureSession
from test_ros_inputs import sources
from test_recording_process import start_recording_fixture,stop_recording_fixture


class StartupSnapshotTests(unittest.TestCase):
    def test_registration_version_refuses_old_protocol_before_socket_or_output(self):
        from robot_workbench.recording_process import require_recording_protocol,RECORDING_SERVICE_SCHEMA
        with tempfile.TemporaryDirectory()as folder:
            path=Path(folder)/'service.json';path.write_text(json.dumps({'schema':'workbench.recording-service.v1'}));path.chmod(0o600)
            with patch.dict(os.environ,WORKBENCH_RECORDING_SERVICE=str(path)),patch('socket.socket',side_effect=AssertionError('no socket operation')):
                with self.assertRaisesRegex(ValueError,'protocol version mismatch'):require_recording_protocol()
                path.write_text(json.dumps({'schema':RECORDING_SERVICE_SCHEMA}))
                require_recording_protocol()

    def test_expired_prestart_depth_does_not_crash_writer_or_substitute_a_frame(self):
        self.prestart_snapshot()

    def test_ingress_closure_is_durable_before_verification_and_export(self):
        self.prestart_snapshot(check_ingress=True)

    def test_changed_source_identity_is_still_rejected_at_start(self):
        with self.assertRaisesRegex(ValueError,'source identity changed'):
            self.prestart_snapshot(invalid_identity=True)

    def prestart_snapshot(self,invalid_identity=False,check_ingress=False):
        from robot_workbench.ros_data import SampleCache
        from robot_workbench.ros_recording import capture_worker
        from robot_workbench.recording_process import _capture_spec
        rgb=sources()[0];rgb['metadata'].update(depth_stream_id='head_depth')
        depth=dict(rgb,stream_id='head_depth',topic=rgb['topic']+'_depth',units='depth16le',
            metadata=dict(width=2,height=2,pixel_format='depth16le',depth_scale_m=.001,depth_aligned_to='color'))
        inputs=object.__new__(RosInputs);inputs.sources={'head':rgb,'head_depth':depth}
        inputs.caches={key:SampleCache(row,max_samples=1)for key,row in inputs.sources.items()}
        ref=StreamRef('head','cpu_source','simulation',1,1)
        inputs.caches['head'].put(ref,dict(rgb=bytes(12),depth=None,metadata={}),12)
        # Pair1 was ready before bind; a newer depth then evicted only its
        # depth half while the RGB remains in the prepared bounded history.
        inputs.caches['head_depth'].put(StreamRef('head_depth','cpu_source','simulation',2,2),dict(depth=bytes(8)),8)
        if invalid_identity:inputs.caches['head_depth'].error='source identity changed'
        inputs.depth_for={'head':'head_depth'};inputs.last_pairs={'head':ref}
        inputs.reliability={key:'reliable'for key in inputs.sources}
        inputs.gaps={};inputs.pending_rgbd=set();inputs.pending_metadata=set()
        node=SimpleNamespace(errors=[],closed=False)
        node.close=lambda:setattr(node,'closed',True)
        with tempfile.TemporaryDirectory()as folder:
            root=Path(folder);spec=_capture_spec('startup',[capture_source(rgb,namespace='/wb_cpu_inputs',domain_id=126)],'data_required',None)
            (root/'capture.json').write_text(json.dumps(spec))
            health_source=dict(kind='ros',namespace='/wb_cpu_inputs',domain_id=126,source=dict(rgb,
                stream_id='health',source_instance_id='unit',kind='event',message_type='workbench_interfaces/msg/DataEvent',
                units='capture_health',nominal_hz=5.,metadata={}))
            health_node=SimpleNamespace(closed=False,types=SimpleNamespace(DataEvent=lambda **kw:SimpleNamespace(**kw),
                SourceDescriptor=lambda **kw:SimpleNamespace(**kw)))
            health_node.close=lambda:setattr(health_node,'closed',True)
            binding=dict(run_id='startup',business_kind='collection',policy_hash='a'*64,
                execution_owner=dict(owner_pid=os.getpid(),runtime_nonce='cpu',command_epoch=0),health_source=health_source)
            commands=[dict(action='bind',binding=binding),dict(action='start'),dict(action='finish',final_seq_by_stream={},
                source_endpoints={'head':dict(source_clock_id='simulation',source_time=0.)},control_terminal={'status':'completed'})]
            read_fd,write_fd=os.pipe();os.write(write_fd,b''.join((json.dumps(c)+'\n').encode()for c in commands))
            output=StringIO()
            from robot_workbench.recording_process import _seal_capture
            def seal(*args):
                if check_ingress:
                    self.assertTrue(node.closed,'source subscriptions must close before admission proof')
                    self.assertTrue((root/'finish.json').is_file())
                    state=json.loads((root/'status.json').read_text())
                    self.assertIs(state.get('ingress_closed'),True)
                    self.assertFalse(state['done'],'closure must not claim seal completion')
                    self.assertTrue(any(json.loads(line).get('ingress_closed')for line in output.getvalue().splitlines()))
                return _seal_capture(*args)
            try:
                with os.fdopen(read_fd)as pipe,patch('robot_workbench.ros_recording.RosNode',return_value=node)as node_factory,\
                        patch('robot_workbench.ros_recording.RosInputs',return_value=inputs),\
                        patch('robot_workbench.ros_recording._recording_health_source',return_value=(health_node,health_source,Mock(),Mock())),\
                        patch('robot_workbench.recording_process._seal_capture',side_effect=seal),\
                        patch.object(StreamRef,'message',lambda ref:SimpleNamespace(**ref.__dict__)),patch('sys.stdin',pipe),redirect_stdout(output):
                    capture_worker(root,spec)
                    self.assertFalse(node_factory.call_args.kwargs['localhost_only'],
                        'registered physical sources must honor deployment discovery scope')
            finally:os.close(write_fd)
            states=[json.loads(line)for line in output.getvalue().splitlines()]
            self.assertTrue(any(s['state']=='recording'for s in states))
            self.assertTrue(states[-1]['done'])
            manifest=json.loads((root/'manifest.json').read_text())
            self.assertEqual(manifest['recording_outcome'],'incomplete','no usable frame must not be relabelled complete')
            self.assertEqual(manifest['streams']['head']['committed'],0,'the newer depth must not replace the original pair')


@unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
class RosRecordingTests(unittest.TestCase):
    def test_direct_sources_and_model_refs_have_one_persistent_owner(self):
        self.record_run(False,health_consumers=True)

    def test_wam32_bf16_inputs_reconstruct_from_originals_and_one_output_is_saved(self):
        self.record_run(False,wam32=True)

    def test_wam_stall_cover_and_actual_seed_are_reconstructed_and_sealed(self):
        self.record_run(False,wam32=True,stall=True)

    def test_wam_stall_decision_is_sealed_even_when_model_fails(self):
        self.record_run(False,wam32=True,stall=True,model_failure=True)

    def test_existing_capture_run_only_orchestrates_ros_and_resident_receipt(self):
        self.record_run(True)

    def test_failed_final_boundary_seals_incomplete_prefix_and_exits_writer(self):
        self.record_run(True,failed_boundary=True)

    def test_control_gap_after_declared_final_sequence_does_not_poison_camera_tail(self):
        self.record_run(True,post_end_gap=True)

    def test_writer_crash_recovers_model_index_and_verified_prefix_without_replay(self):
        self.record_run(False,crash=True)

    def test_write_limit_failure_recovers_verified_model_prefix(self):
        self.record_run(False,crash=True,file_limit=True)

    def test_original_rgbd_uses_one_existing_lossless_segment_per_camera(self):
        self.record_run(False,depth=True)

    def test_original_frame_metadata_and_depth_share_the_saved_frame(self):
        self.record_run(False,depth=True,include_metadata=True)

    def test_model_input_cannot_seal_complete_without_its_exact_original_frame(self):
        self.record_run(False,missing_model_ref=True)

    def test_failed_model_keeps_actual_input_error_result_and_original_sources(self):
        self.record_run(False,model_failure=True)

    def test_ros_sources_continue_while_business_client_is_disconnected(self):
        self.record_run(False,disconnect_client=True)

    def test_retry_of_earlier_model_request_does_not_duplicate_or_poison_recording(self):
        self.record_run(False,replay_result=True)

    def test_missing_pedal_tail_does_not_report_covered_cameras_as_missing(self):
        self.record_run(False,missing_pedal=True)

    def record_run(self,use_run_adapter,*,failed_boundary=False,crash=False,depth=False,file_limit=False,include_metadata=False,missing_model_ref=False,model_failure=False,disconnect_client=False,replay_result=False,health_consumers=False,wam32=False,post_end_gap=False,stall=False,missing_pedal=False):
        from sensor_msgs.msg import Image
        fixture=start_recording_fixture()
        nodes=[];closing=threading.Event();session=None;publisher_thread=None;model=None;capture=None;health_readers=[]
        folder=tempfile.TemporaryDirectory()
        try:
            nodes=[RosNode(n,namespace='/wb_cpu_inputs',domain_id=126)for n in('record_source','record_infer','record_client')]
            source,infer,control=nodes
            raw=sources()
            for row in raw:row['source_clock_id']='host_monotonic'
            # Small lossless image frames keep this a transport/recording test.
            publishers={r['stream_id']:source.publisher(r['topic'],Image if r['kind']=='image'else source.types.RobotState)for r in raw}
            depth_publishers={};declarations=[]
            if depth:
                from robot_workbench.ros_data import descriptor_message
                for row in raw[:3]:
                    name=row['stream_id']+'_depth';row['metadata']['depth_stream_id']=name
                    desc=dict(row,stream_id=name,topic=row['topic']+'_depth',units='depth16le',
                        metadata=dict(row['metadata'],pixel_format='depth16le',depth_scale_m=.001,depth_aligned_to='color'))
                    desc['metadata'].pop('depth_stream_id')
                    depth_publishers[row['stream_id']]=source.publisher(desc['topic'],Image)
                    pub=source.publisher('sources/'+name,source.types.SourceDescriptor,retained=True,depth=1)
                    pub.publish(descriptor_message(source,desc));declarations.append(pub)
            metadata_publishers={};sample_metadata={}
            if include_metadata:
                for row in raw[:3]:
                    row['metadata']['metadata_topic']=row['topic']+'/metadata'
                    metadata_publishers[row['stream_id']]=source.publisher(row['metadata']['metadata_topic'],source.types.DataEvent)
            counter=[0]
            def publish():
                while not closing.wait(.02):
                    counter[0]+=1;now=time.monotonic_ns()
                    if include_metadata:
                        sample_metadata[now]=dict(host_receive_ns=time.time_ns(),physics_step=counter[0],frame_number=counter[0],
                            owner_available=True,owner_pid=os.getpid(),command_epoch=1,runtime_nonce='cpu_owner')
                    for role in('head','left','right'):
                        image=Image(height=2,width=2,encoding='rgb8',step=6,data=bytes(range(12)))
                        image.header.stamp.sec=now//10**9;image.header.stamp.nanosec=now%10**9
                        publishers[role].publish(image)
                        if depth:
                            depth_image=Image(height=2,width=2,encoding='16UC1',step=4,data=bytes([1,0,2,0,3,0,4,0]))
                            depth_image.header.stamp=image.header.stamp;depth_publishers[role].publish(depth_image)
                        if include_metadata:
                            metadata_publishers[role].publish(source.types.DataEvent(reference=StreamRef(role,'cpu_source','host_monotonic',now,now).message(),
                                run_id='',epoch=1,event='frame_metadata',payload_json=json.dumps(sample_metadata[now])))
                    ref=StreamRef('state','cpu_source','host_monotonic',now,counter[0])
                    publishers['state'].publish(source.types.RobotState(reference=ref.message(),positions=[.1]*14,
                        velocities=[0.]*14,efforts=[],metadata_json='{}'))
            publisher_thread=threading.Thread(target=publish);publisher_thread.start()
            inputs=RosInputs(infer,raw)
            from robot_workbench.inference_model import action_contract
            binding={'session_id':'model_session','model_binding':{},'conversion_id':'conversion','action_contract':action_contract(),'task':'coffee'}
            if wam32:
                from robot_workbench import wam
                # Actual InferenceWorkflow binds/warm-loads this CPU dependency
                # before selecting Run frames; this lower-level fixture must too.
                wam._torch()
                binding['model_binding']={'model_id':'wam-fixture-step3190'};binding['action_contract'].update(shape=[32,14],history_steps=0)
            expected=np.arange((32 if wam32 else 60)*14,dtype=np.float64).reshape(-1,14)
            def predict(request,prepared):
                if stall:
                    self.assertNotEqual(request['seed'],1)
                    self.assertTrue(0<=request['seed']<2**63)
                    self.assertTrue(np.array_equal(prepared[1]['base_0_rgb'],wam.image_bits(np.zeros((2,2,3),dtype=np.uint8))))
                    self.assertFalse(np.array_equal(prepared[1]['left_wrist_0_rgb'],prepared[1]['base_0_rgb']))
                if model_failure:raise ValueError('synthetic model failure')
                return dict(request_id=request['request_id'],action14=expected,audit={})
            prepare=lambda obs:(wam.bf16_values(obs['device_state14']),wam.prepare_images(obs['images']))if wam32 else(np.asarray(obs['device_state14']),obs['images'])
            model=RosModelWorker(infer,inputs,prepare,
                predict,binding=binding)
            if stall:
                recovery=wam.StallRecovery({'left':80.,'right':89.})
                recovery.event('normal_model_segment_started',dict(request_id='executed',start_source_s=0.,
                    execution_duration_s=3.,source_steps=90),1)
                for step in range(1,91):recovery.event('feedback',dict(source_time=step/30,device_state14=[0.]*14),step+1)
                model.stall_recovery=lambda request:recovery.choose(request.seed)
            model.bind('run',1);client=RosModelClient(control,binding=binding);client.bind('run',1)
            descriptions=raw+[
                dict(device_id='cpu_robot',stream_id=name,source_instance_id='model_session',source_clock_id='host_monotonic',
                     topic='/wb_cpu_inputs/model/'+topic,message_type='workbench_interfaces/msg/'+kind,kind=name,
                     units='model',coordinate_frame='model',nominal_hz=0.,available=True,metadata={})
                for name,topic,kind in [('model_input','inputs','ModelInput'),('model_result','results','ModelResult')]]
            directory=folder.name
            if missing_pedal:
                descriptions.append(dict(device_id='cpu_robot',stream_id='pedal',source_instance_id='cpu_pedal',
                    source_clock_id='host_monotonic',topic='/wb_cpu_inputs/pedal',message_type='workbench_interfaces/msg/DataEvent',
                    kind='event',units='pedal_command',coordinate_frame='operator',nominal_hz=10.,available=True,
                    metadata={'continuous_window':True}))
            root=Path(directory)/'inference-run'/'capture'
            execution_owner=dict(owner_pid=os.getpid(),runtime_nonce='cpu_owner',command_epoch=1)
            if use_run_adapter:
                from robot_workbench.inference_trace import CaptureRun
                from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
                parameters=control_parameters(defaults('inference'),business='inference')
                policy=dict(revision=0,policy_hash=policy_hash(parameters),parameters=parameters)
                descriptions.append(dict(device_id='cpu_robot',stream_id='control',source_instance_id='cpu_owner',
                    source_clock_id='host_monotonic',topic='/wb_cpu_inputs/control',message_type='workbench_interfaces/msg/DataEvent',
                    kind='event',units='control',coordinate_frame='device',nominal_hz=0.,available=True,metadata={}))
                events=source.publisher('/wb_cpu_inputs/control',source.types.DataEvent)
                capture=CaptureRun('run',root.parent,dict(execution_context=dict(device_id='cpu_robot',environment='fake')),
                    policy,None,'fake',None,lambda:True,lambda value:None,
                    ros_sources=lambda:[capture_source(r,namespace='/wb_cpu_inputs',domain_id=126)for r in descriptions],
                    ros_watermarks=model.watermarks)
                capture.record('prepare',dict(execution_owner=execution_owner))
                self.assertIsNone(capture.error)
                session=capture.session
            else:
                session=CaptureSession.create(root,request_id='request-run',source_descriptors=[capture_source(r,namespace='/wb_cpu_inputs',domain_id=126)for r in descriptions],purpose_profile='infer_debug')
                session.prepare();session.bind(run_id='run',business_kind='inference',policy_hash='a'*64,
                    execution_owner=execution_owner);session.start()
            root=session.directory
            if health_consumers:
                from robot_workbench.ros_data import RecordingHealth
                received=[{},{}]
                for i,node in enumerate((infer,control)):
                    reader=RecordingHealth(session.health_source,capture_session_id=session.capture_session_id,run_id='run',device_id='cpu_robot',
                        writer_identity=session.snapshot()['writer_identity'],plane=session.health_source,ros=node,
                        on_value=lambda ref,value,i=i:received[i].update({ref:deepcopy(value)}))
                    health_readers.append(reader)
                deadline=time.monotonic()+3
                while not(set(received[0])&set(received[1])):
                    self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                shared=next(iter(set(received[0])&set(received[1])))
                self.assertEqual(received[0][shared],received[1][shared])
                from robot_workbench.recording_process import recording_call
                proof=recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'status',directory=str(root))
                self.assertNotIn('streams',proof);self.assertNotIn('data_queue',proof)
            deadline=time.monotonic()+10
            while (not all(c.history()for c in inputs.caches.values()) or model.inputs.get_subscription_count()<1
                   or model.results.get_subscription_count()<2 or client.requests.get_subscription_count()<1)and time.monotonic()<deadline:time.sleep(.02)
            state=inputs.latest('state')[0]
            frames={k:inputs.latest(k)[0]for k in('head','left','right')}
            if wam32:frames={role:frames[key]for role,key in zip(wam.CAMS,('head','left','right'))}
            obs=inputs.observation(frames,state,
                run_id='run',epoch=1,model_io_digest='io',source_time_ns=min(ref.source_time_ns for ref in frames.values()),pairing={})
            valid_until=time.monotonic_ns()+2_000_000_000
            if model_failure:
                with self.assertRaisesRegex(ValueError,'synthetic model failure'):
                    client.predict('request',obs,seed=1,task='coffee',history=[],valid_until_ns=valid_until)
            else:
                result=client.predict('request',obs,seed=1,task='coffee',history=[],valid_until_ns=valid_until)
                self.assertTrue(np.array_equal(result['action14'],expected))
            if replay_result:
                client.predict('request2',obs,seed=1,task='coffee',history=[],valid_until_ns=valid_until)
                replay=client.predict('request',obs,seed=1,task='coffee',history=[],valid_until_ns=valid_until)
                self.assertTrue(np.array_equal(replay['action14'],expected))
            if disconnect_client:
                from robot_workbench.recording_process import recording_call
                from robot_workbench.ros_data import RecordingHealth
                old=session;writer_pid=old.process.pid;before=old.snapshot()['streams']['head']['received']
                description=old.health_source
                observer=RecordingHealth(description,capture_session_id=old.capture_session_id,run_id='run',device_id='cpu_robot',
                    writer_identity=old.snapshot()['writer_identity'],plane=description,ros=control)
                self.addCleanup(observer.close)
                old.disconnected.set();old.thread.join(8);self.assertFalse(old.thread.is_alive())
                CaptureSession._sessions.pop(str(old.directory),None)
                deadline=time.monotonic()+3
                while True:
                    state=recording_call(os.environ['WORKBENCH_RECORDING_SERVICE'],'status',directory=str(old.directory))
                    self.assertNotIn('streams',state,'live coverage must come directly from ROS')
                    if observer.snapshot().get('streams',{}).get('head',{}).get('received',0)>before+2:break
                    self.assertLess(time.monotonic(),deadline);time.sleep(.02)
                self.assertFalse(state['writer_exited']);self.assertEqual(state['state'],'recording')
                session=CaptureSession.create(old.directory.parent,request_id=old.spec['request_id'],
                    source_descriptors=old.spec['source_descriptors'],purpose_profile='infer_debug')
                self.assertEqual(session.prepare()['state'],'recording');self.assertEqual(session.process.pid,writer_pid)
            end=time.monotonic_ns()/1e9
            if crash:
                from robot_workbench.managed_services import process_identity,_live
                import signal
                deadline=time.monotonic()+10
                while not all(session.snapshot().get('streams',{}).get(k,{}).get('committed',0)==1
                              for k in('model_input','model_result')):
                    self.assertLess(time.monotonic(),deadline,session.snapshot());time.sleep(.02)
                self.assertLessEqual(session.snapshot()['streams']['head']['committed'],
                                     session.snapshot()['streams']['head']['received'])
                pid=session.process.pid
                deadline=time.monotonic()+5
                while True:
                    # Linux lists children under the thread which spawned
                    # them; camera codecs now start on bounded writer threads.
                    children=list({int(child)for task in Path(f'/proc/{pid}/task').glob('*/children')
                        for child in task.read_text().split()})
                    if children or time.monotonic()>=deadline:break
                    time.sleep(.01)
                self.assertTrue(children,'exercise a writer with real active encoders')
                identities=[process_identity(p)for p in children]
                prefix=(root/'model_result.jsonl').read_bytes()
                if file_limit:
                    import resource
                    resource.prlimit(pid,resource.RLIMIT_FSIZE,(1024,1024))
                    deadline=time.monotonic()+10
                    while not session.snapshot().get('source_errors')and not session.done.is_set():
                        self.assertLess(time.monotonic(),deadline,'writer did not report the storage failure');time.sleep(.02)
                    if not session.done.is_set():
                        session.finish(model.watermarks(),dict(status='failed',reason='storage failure'),source_endpoints={
                            row['stream_id']:dict(source_clock_id=row['source_clock_id'],source_time=0.)for row in raw})
                else:os.kill(pid,signal.SIGKILL)
                self.assertTrue(session.done.wait(15),session.snapshot())
                deadline=time.monotonic()+10
                while any(_live(process_identity(identity['pid']))for identity in identities):
                    self.assertLess(time.monotonic(),deadline,'orphan encoder after writer crash');time.sleep(.02)
                closing.set();publisher_thread.join(5)
                session.recover()
            elif capture is not None:
                deadline=time.monotonic()+5
                while events.get_subscription_count()!=1 and time.monotonic()<deadline:time.sleep(.01)
                events.publish(source.types.DataEvent(reference=StreamRef('control','cpu_owner','host_monotonic',round(end*1e9),1).message(),
                    run_id='run',epoch=1,event='finish',payload_json='{}'))
                tail=.3 if post_end_gap else 0.
                terminal=dict(status='completed',source_time=end+tail,finished_wall_ns=time.time_ns()+round(tail*1e9))
                if not failed_boundary:terminal['ros_final_seq']={'control':1}
                capture.record('finish',terminal)
                if post_end_gap:
                    deadline=time.monotonic()+5
                    while session.snapshot().get('state')!='sealing':
                        self.assertLess(time.monotonic(),deadline,session.snapshot());time.sleep(.005)
                    events.publish(source.types.DataEvent(reference=StreamRef('control','cpu_owner','host_monotonic',round((end+.01)*1e9),3).message(),
                        run_id='run',epoch=1,event='outside_run',payload_json='{}'))
                self.assertTrue(capture.done.wait(30),capture.snapshot())
                if failed_boundary:self.assertIsNotNone(capture.error)
                else:self.assertIsNone(capture.error)
                self.assertTrue((capture.directory/'receipt.json').is_file(),'the resident service must produce the Run receipt')
                self.assertEqual(session.publish_calls,0,'the business adapter must never inject source data over IPC')
            else:
                session.finish(model.watermarks(),{'status':'failed'if model_failure else'completed'},source_endpoints={r['stream_id']:
                    dict(source_clock_id=r['source_clock_id'],source_time=end)for r in descriptions if r['kind']in('image','state')or r['stream_id']=='pedal'})
            self.assertTrue(session.done.wait(30),session.snapshot())
            final=session.snapshot();self.assertIsNone(final.get('error'),final)
            manifest=json.loads((session.directory/'manifest.json').read_text())
            self.assertEqual(manifest['recording_outcome'],'incomplete'if failed_boundary or crash or missing_pedal else'complete',manifest.get('issues'))
            if missing_pedal:
                self.assertTrue(any('pedal'in issue for issue in manifest['issues']),manifest['issues'])
                self.assertFalse(any('camera endpoint not covered'in issue for issue in manifest['issues']),manifest['issues'])
                for camera in ('head','left','right'):
                    self.assertTrue(json.loads((root/(camera+'-source.json')).read_text())['endpoint_reached'])
                self.assertTrue(session.snapshot()['writer_exited'])
            if failed_boundary:self.assertTrue(session.snapshot()['writer_exited'])
            requests={'request','request2'}if replay_result else{'request'}
            self.assertEqual(set(manifest['model_inputs']),requests)
            self.assertEqual(set(manifest['model_results']),requests)
            if stall:
                decisions=[json.loads(line)for line in (root/'stall_recovery.jsonl').read_text().splitlines()]
                self.assertEqual(len(decisions),1)
                self.assertEqual(decisions[0]['request_id'],'request')
                self.assertTrue(decisions[0]['head_black']);self.assertNotEqual(decisions[0]['actual_seed'],1)
                self.assertIn('stall_recovery.jsonl',manifest['files'])
                metadata=json.loads(manifest['model_inputs']['request']['metadata_json'])
                self.assertEqual(metadata['seed'],decisions[0]['actual_seed'])
                self.assertEqual(metadata['stall_recovery']['configured_seed'],1)
            if model_failure:self.assertEqual(manifest['model_results']['request']['error'],'synthetic model failure')
            self.assertFalse(list(session.directory.rglob('input.npz')),'raw images must stay in one source stream')
            outputs=list(session.directory.rglob('output.npz'));self.assertEqual(len(outputs),len(requests))
            with np.load(outputs[0])as saved:self.assertTrue(np.array_equal(saved['action14'],np.empty((0,14))if model_failure else expected))
            if depth:
                from rule_based_workbench.rgbd_segments import iter_segment
                for name in ('head','left','right'):
                    commit=next((root/'camera'/name).glob('*.commit.json'))
                    frames=iter_segment(root/'camera',str(commit.relative_to(root/'camera')),png=False)
                    try:
                        meta,rgb,depth_pixels=next(frames)
                        self.assertEqual(rgb,bytes(range(12)));self.assertEqual(depth_pixels,bytes([1,0,2,0,3,0,4,0]))
                        self.assertEqual(meta['depth_reference']['source_time_ns'],meta['ros_reference']['source_time_ns'])
                        if include_metadata:
                            for key,value in sample_metadata[meta['ros_reference']['source_time_ns']].items():self.assertEqual(meta[key],value)
                    finally:frames.close()
                self.assertEqual(len(manifest['streams']),6,'depth stays inside the original RGB-D stream, not a duplicate video')
            if crash:
                self.assertIsNone(manifest['control_terminal'])
                self.assertEqual((root/'model_result.jsonl').read_bytes(),prefix)
                self.assertEqual(model.watermarks(),{'model_input':1,'model_result':1},'recovery must not re-infer')
                output_ref=manifest['model_results']['request']['output_ref']
                self.assertEqual(manifest['files'][output_ref['path']]['sha256'],output_ref['sha256'])
                # A later integrity failure cannot keep a stale result index.
                outputs[0].write_bytes(b'corrupted')
                session.recover();self.assertTrue(session.done.wait(30),session.snapshot())
                damaged=json.loads((root/'manifest.json').read_text())
                self.assertEqual(damaged['model_results'],{})
                self.assertIn('model output bytes differ',damaged['streams']['model_result']['tail_error'])
                self.assertTrue(session.snapshot()['writer_exited'])
            else:self.assertEqual(len(manifest['playback']['streams']),3)
            if missing_model_ref:
                from robot_workbench.recording_process import _seal_capture
                path=root/'model_input.jsonl';rows=[json.loads(line)for line in path.read_text().splitlines()]
                ref=rows[0]['payload']['observation']['frames'][0]
                ref['source_time_ns']+=1;ref['sequence']+=1
                path.write_text(''.join(json.dumps(row)+'\n'for row in rows))
                resealed=_seal_capture(root,json.loads((root/'capture.json').read_text()))
                self.assertEqual(resealed['recording_outcome'],'incomplete')
                self.assertTrue(any('missing original head sample'in issue for issue in resealed['issues']),resealed['issues'])
        finally:
            for reader in health_readers:reader.close()
            # A disconnected producer's done flag is not writer exit. Reattach
            # before ending the test, keeping the original sources alive.
            if session is not None and session.done.is_set()and session.process.poll()is None:
                old=session;CaptureSession._sessions.pop(str(old.directory),None)
                session=CaptureSession.create(old.directory.parent,request_id=old.spec['request_id'],
                    source_descriptors=old.spec['source_descriptors'],purpose_profile=old.spec['purpose_profile'],
                    resource_budget=old.spec['resource_budget'])
                session.prepare()
            if session is not None and session.done.is_set() and not (session.directory/'manifest.json').exists():
                session.recover();session.done.wait(30)
            if session is not None and not session.done.is_set():session.close(timeout=30)
            closing.set()
            if publisher_thread is not None:publisher_thread.join(5)
            if model is not None:model.close()
            for node in reversed(nodes):node.close()
            stop_recording_fixture(fixture)
            folder.cleanup()


if __name__=='__main__':unittest.main()
