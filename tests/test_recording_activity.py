"""Mode-selected capture and a shared compact timeline; CPU sources only."""
from copy import deepcopy
from types import SimpleNamespace
import json,os,tempfile,threading,time,unittest
from pathlib import Path
from robot_workbench.recording_activity import ActivityTimeline,ActiveTimeAxis,POLICY


class ActivityTests(unittest.TestCase):
    def event(self,timeline,mode,stamp,request):
        ref=SimpleNamespace(source_clock_id='clock',source_time_ns=round(stamp*1e9))
        return timeline.observe('inference_mode_transition',dict(op='normal_mode',requested_mode=mode,
            request_id=request,accepted=True,status='accepted',capture_wall_ns=1_790_000_000_000_000_000+ref.source_time_ns),ref,'run')

    def test_replay_configuration_uses_recorded_model_changes_and_compact_time(self):
        from robot_workbench.recording_activity import inference_replay_segments
        timeline=ActivityTimeline('run','clock')
        for mode,stamp,name in [('MODEL',10,'m1'),('HOLD',20,'h1'),('TEACH',30,'t1'),
                                ('HOLD',35,'h2'),('MODEL',50,'m2'),('HOLD',55,'h3')]:
            self.event(timeline,mode,stamp,name)
        def recorded(stamp,steps):
            return dict(observation=dict(source_time_ns=round(stamp*1e9)),metadata_json=json.dumps(
                dict(binding=dict(model_binding=dict(model_id='wam-fixture-step1',backend='torch',
                    worker_identity=dict(inference_steps=steps))))))
        manifest=dict(recording_activity=timeline.manifest(),
            binding=dict(control_policy=dict(parameters=dict(execution=dict(execute_steps=8)))),
            model_inputs={'a':recorded(10.2,2),'b':recorded(15,4),'c':recorded(50.1,10)})
        rows=inference_replay_segments(manifest)['segments']
        self.assertEqual([(r['start_s'],r['end_s'],r['mode'],r['denoise_steps'])for r in rows],
                         [(0.,5.,'MODEL',2),(5.,10.,'MODEL',4),(10.,15.,'TEACH',4),(15.,20.,'MODEL',10)])
        self.assertTrue(all(r['execute_steps']==8 for r in rows))
        self.assertTrue(all(r['model_id']=='wam-fixture-step1'for r in rows))
        self.assertEqual(rows[2]['start_wall_ns'],1_790_000_030_000_000_000)
        manifest['model_inputs']={}
        missing=inference_replay_segments(manifest)['segments']
        self.assertTrue(all(r['model_id']is None and r['denoise_steps']is None for r in missing))

    def test_model_and_teach_join_without_waiting_gaps(self):
        timeline=ActivityTimeline('run','clock')
        self.event(timeline,'HOLD',1,'initial');self.event(timeline,'MODEL',10,'model')
        self.event(timeline,'HOLD',12.8,'stop');self.event(timeline,'TEACH',50,'teach')
        self.event(timeline,'HOLD',53,'hold');timeline.finish(100_000_000_000)
        axis=ActiveTimeAxis(timeline.manifest())
        self.assertEqual(axis.duration_ns,5_800_000_000)
        self.assertIsNone(axis.offset_ns('clock',30_000_000_000))
        self.assertEqual(axis.offset_ns('clock',50_000_000_000),2_800_000_000)
        self.assertEqual(axis.offset_ns('host_wall',1_790_000_051_000_000_000),3_800_000_000)
        self.assertIsNone(axis.offset_ns('clock',53_000_000_000))
        self.assertEqual(timeline.sample_group('clock',12_700_000_000),'model')
        self.assertIsNone(timeline.sample_group('clock',30_000_000_000))
        broken=timeline.manifest();broken['intervals'][1]['effective_start_ns']+=1
        with self.assertRaises(ValueError):ActiveTimeAxis(broken)
        with self.assertRaises(ValueError):axis.offset_ns('unbound',1)

    def test_hold_only_and_failed_model_attempt_keep_truthful_boundaries(self):
        timeline=ActivityTimeline('run','clock');self.event(timeline,'HOLD',1,'initial')
        self.assertEqual(ActiveTimeAxis(timeline.manifest()).duration_ns,0)
        self.event(timeline,'MODEL',2,'attempt')
        ref=SimpleNamespace(source_clock_id='clock',source_time_ns=3_000_000_000)
        timeline.observe('inference_mode_transition',dict(status='failed',request_id='attempt',
            capture_wall_ns=1_790_000_003_000_000_000),ref,'run')
        self.assertEqual(ActiveTimeAxis(timeline.manifest()).duration_ns,1_000_000_000)
        self.assertIsNone(timeline.summary()['active_mode'])


@unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated Linux ROS and CPU codecs required')
class ActiveCaptureTests(unittest.TestCase):
    def test_real_recorder_skips_hold_flushes_segments_and_joins_all_cameras(self,active_gap=False):
        from robot_workbench.ros_data import RosNode,StreamRef,json_text
        from sensor_msgs.msg import Image
        from robot_workbench.ros_recording import capture_source
        from robot_workbench.recording_process import CaptureSession
        from robot_workbench.execution_settings import control_parameters,defaults,policy_hash
        from test_recording_process import start_recording_fixture,stop_recording_fixture
        from test_ros_inputs import sources
        fixture=start_recording_fixture();self.addCleanup(stop_recording_fixture,fixture)
        node=RosNode('active_capture_source',namespace='/wb_active_capture',domain_id=168)
        self.addCleanup(node.close)
        descriptions=sources()
        for row in descriptions:
            row['source_clock_id']='clock';row['topic']='/wb_active_capture/'+row['stream_id']
        descriptions.append(dict(device_id='cpu_robot',stream_id='control',source_instance_id='cpu-owner',
            source_clock_id='clock',topic='/wb_active_capture/control',message_type='workbench_interfaces/msg/DataEvent',
            kind='event',units='control',coordinate_frame='device',nominal_hz=30.,available=True,metadata={}))
        publishers={row['stream_id']:node.publisher(row['topic'],Image if row['kind']=='image'else getattr(node.types,'RobotState'if row['kind']=='state'else'DataEvent'))for row in descriptions}
        lock=threading.RLock();stopped=threading.Event();sequence=0;state_sequence=0
        run='activity-run'
        def control(event,data):
            nonlocal sequence
            with lock:
                sequence+=1;stamp=time.monotonic_ns();ref=StreamRef('control','cpu-owner','clock',stamp,sequence)
                publishers['control'].publish(node.types.DataEvent(reference=ref.message(),event=event,run_id=run,epoch=1,
                    payload_json=json_text(dict(data,capture_wall_ns=time.time_ns(),runtime_nonce='cpu-owner'))))
                return stamp
        def state(stamp):
            nonlocal state_sequence
            state_sequence+=1
            ref=StreamRef('state','cpu_source','clock',stamp,state_sequence)
            publishers['state'].publish(node.types.RobotState(reference=ref.message(),positions=[0.]*14,velocities=[0.]*14,
                metadata_json=json_text(dict(sample_time_s=stamp/1e9,sample_wall_ns=time.time_ns(),runtime_nonce='cpu-owner',command_epoch=1,owner_pid=os.getpid()))))
        def pump():
            while not stopped.is_set():
                with lock:
                    stamp=time.monotonic_ns()
                    for role in ('head','left','right'):
                        image=Image()
                        image.header.stamp.sec,image.header.stamp.nanosec=divmod(stamp,1_000_000_000)
                        image.height=image.width=2;image.encoding='rgb8';image.step=6;image.data=bytes([state_sequence%255]*12)
                        publishers[role].publish(image)
                    state(stamp);control('feedback',{})
                stopped.wait(1/30)
        worker=threading.Thread(target=pump);worker.start()
        def close_source():stopped.set();worker.join(3)
        self.addCleanup(close_source)
        temporary=tempfile.TemporaryDirectory(prefix='active-capture-')
        self.addCleanup(temporary.cleanup);directory=temporary.name
        root=Path(directory)/('inference-'+run)/'capture'
        session=CaptureSession.create(root,request_id=run,source_descriptors=[capture_source(row,namespace='/wb_active_capture',domain_id=168)for row in descriptions],purpose_profile='infer_debug')
        self.addCleanup(session.close)
        prepared=session.prepare();parameters=control_parameters(defaults('inference'),business='inference')
        policy=dict(revision=0,parameters=parameters,policy_hash=policy_hash(parameters))
        session.bind(run_id=run,business_kind='inference',policy_hash=policy['policy_hash'],control_policy=policy,
            execution_context=dict(device_id='cpu_robot',environment='fake',recording_activity=POLICY),
            execution_owner=dict(owner_pid=os.getpid(),runtime_nonce='cpu-owner',command_epoch=1),health_source=prepared['health_source'])
        session.start();time.sleep(.5)
        self.assertFalse((session.directory/'camera').exists(),'HOLD must not start video encoding')
        def mode(value,request):return control('inference_mode_transition',dict(op='normal_mode',requested_mode=value,request_id=request,accepted=True,status='accepted'))
        start=mode('MODEL','m1');time.sleep(2.8);end=mode('HOLD','h1')
        time.sleep(.7)
        initial={k:session.snapshot()['streams'][k]['received']for k in ('head','left','right')}
        time.sleep(.4);self.assertEqual(initial,{k:session.snapshot()['streams'][k]['received']for k in initial})
        self.assertTrue(list((session.directory/'camera').rglob('*.commit.json')),'paused tail must be committed before ending the session')
        self.assertTrue(list((session.directory/'camera').rglob('*.browser.json')),'browser encoding must finish during capture')
        with lock:
            teach=mode('TEACH','t1')
            # Deliver a HOLD sample after the recorder has prefetched its prefix.
            # Its deliberate exclusion must not become a gap in active TEACH.
            time.sleep(.05);state(teach-1)
        time.sleep(.3)
        if active_gap:
            with lock:state_sequence+=1
        time.sleep(2.7);held=mode('HOLD','h2');time.sleep(.2)
        endpoint=time.monotonic_ns()
        session.finish({'control':sequence},dict(status='completed'),source_endpoints={row['stream_id']:
            dict(source_clock_id='clock',source_time=endpoint/1e9)for row in descriptions if row['kind']in('image','state')})
        self.assertTrue(session.done.wait(30),session.snapshot())
        manifest=json.loads((session.directory/'manifest.json').read_text())
        self.assertEqual(manifest['raw_validation'],'not_requested')
        self.assertNotIn('recording.seal.verify_streams',manifest['pipeline_timing']['stages'])
        if active_gap:
            self.assertEqual(manifest['recording_outcome'],'incomplete')
            self.assertTrue(any('source sequence gap'in issue for issue in manifest['issues']),manifest['issues'])
            return
        self.assertEqual(manifest['recording_outcome'],'complete',manifest['issues'])
        expected=(end-start+held-teach)/1e9
        self.assertAlmostEqual(manifest['recording_activity']['effective_duration_ns']/1e9,expected,places=6)
        self.assertEqual(len(manifest['playback']['streams']),3)
        for video in manifest['playback']['streams'].values():
            self.assertEqual(video['encoding']['mode'],'streaming_segments')
            self.assertGreater(video['encoding']['reused_segments'],0)
            self.assertLess(abs(video['duration_s']-expected),.08)
            self.assertLess(max(b-a for a,b in zip(video['frame_offsets_s'],video['frame_offsets_s'][1:])),.15)
        self.assertGreater(manifest['streams']['state']['effective_count'],0)
        session.close()
        for outside in (True,False):
            idle=CaptureSession.create(Path(directory)/('after-end'if outside else'within-end')/('inference-'+run)/'capture',request_id=run+('-after'if outside else'-within'),
                source_descriptors=[capture_source(row,namespace='/wb_active_capture',domain_id=168)for row in descriptions],purpose_profile='infer_debug')
            self.addCleanup(idle.close)
            prepared=idle.prepare()
            idle.bind(run_id=run,business_kind='inference',policy_hash=policy['policy_hash'],control_policy=policy,
                execution_context=dict(device_id='cpu_robot',environment='fake',recording_activity=POLICY),
                execution_owner=dict(owner_pid=os.getpid(),runtime_nonce='cpu-owner',command_epoch=1),health_source=prepared['health_source'])
            idle.start();time.sleep(.3)
            with lock:
                final_control=sequence;sequence+=2
            control('outside_run',{})
            # The source gap can be seen before the finish RPC, while raw camera
            # tails are still draining. It lies wholly after the declared control end.
            time.sleep(.1)
            if not outside:final_control=sequence
            endpoint=time.monotonic_ns()+300_000_000
            idle.finish({'control':final_control},dict(status='completed'),source_endpoints={row['stream_id']:
                dict(source_clock_id='clock',source_time=endpoint/1e9)for row in descriptions if row['kind']in('image','state')})
            self.assertTrue(idle.done.wait(10))
            empty=json.loads((idle.directory/'manifest.json').read_text())
            self.assertEqual(empty['recording_outcome'],'complete'if outside else'incomplete',empty['issues'])
            if not outside:self.assertTrue(any('control source sequence gap'in issue for issue in empty['issues']))
            self.assertEqual(empty['recording_activity']['effective_duration_ns'],0)
            self.assertEqual(empty['playback']['streams'],{})
            self.assertFalse((idle.directory/'camera').exists())
            idle.close()

    def test_real_recorder_still_rejects_state_gap_inside_active_interval(self):
        self.test_real_recorder_skips_hold_flushes_segments_and_joins_all_cameras(active_gap=True)



if __name__=='__main__':unittest.main()
