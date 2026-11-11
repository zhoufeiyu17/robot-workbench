"""Operator intents against shared Owner logic; native ROS tests use CPU sources only."""
from copy import deepcopy
import json,os,struct,subprocess,sys,tempfile,threading,time,unittest
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests')]


class PedalReconnectTests(unittest.TestCase):
    def test_same_configured_reader_reopens_after_unplug_without_restarting_source(self):
        import errno
        from scripts.serve_pedal_source import read_reconnecting
        stopped=threading.Event();attempts=[];disconnected=[]
        class Reader:
            def _run_evdev_backend(self):
                attempts.append(len(attempts)+1)
                if len(attempts)==1:raise OSError(errno.ENODEV,'unplugged')
                if len(attempts)==2:
                    try:raise FileNotFoundError(errno.ENOENT,'waiting for same by-id path')
                    except OSError as error:raise RuntimeError('open failed')from error
                stopped.set()
        read_reconnecting(Reader,stopped,lambda:disconnected.append(True))
        self.assertEqual(attempts,[1,2,3]);self.assertEqual(len(disconnected),2)
        stopped.clear()
        class Denied:
            def _run_evdev_backend(self):raise PermissionError(errno.EACCES,'configuration error')
        with self.assertRaises(PermissionError):read_reconnecting(Denied,stopped,lambda:None)


class OperatorControlTests(unittest.TestCase):
    def fixture(self):
        from test_normal_complete_segment import CompleteNormalTests
        f=CompleteNormalTests();f.setUp();self.addCleanup(f.doCleanups)
        f.setup_modes(recording=False,initial_mode='HOLD',reviewed=True,reviewed_duration_s=40.)
        return f

    def test_stop_preempts_pending_model_without_waiting_or_admitting_late_motion(self):
        f=self.fixture();runner=f.runtime.owner.runner;entered=threading.Event();release=threading.Event();original=runner.predict
        def predict(request):entered.set();release.wait(3);return original(request)
        runner.predict=predict;self.addCleanup(release.set)
        model,_=f.request('MODEL');f.until(entered.is_set)
        self.assertEqual(runner.normal_job['stage'],'RESUMING')
        held=runner.target_interpreter.reference.sample([0])[0][0].copy()
        request,_=f.request('HOLD');f.completed(request)
        self.assertEqual(runner.mode,'HOLD');self.assertIsNone(runner.normal_segment)
        self.assertEqual(runner.mode_receipts[model['request_id']]['error_class'],'operator')
        release.set();runner.bounded_worker.join(2);f.advance(30)
        import numpy as np
        np.testing.assert_array_equal(runner.target_interpreter.reference.sample([0])[0][0],held)
        self.assertTrue(f.raw.is_connected)

    def test_home_preview_retains_native_jaws_and_bounds_joint_motion(self):
        f=self.fixture();request,_=f.request('END');f.completed(request)
        before=f.raw.read_execution_output()['q_sdk14']
        review=f.runtime.submit(f.creds['inference'],dict(action='preview_operator_home',max_velocity=.1))
        self.assertTrue(review['plans'])
        plans=next(iter(f.runtime.owner.pending.values()))[0]
        plan=plans[0]
        self.assertEqual([plan.q_target[6],plan.q_target[13]],[before[6],before[13]])
        self.assertEqual([plan.q_target[i]for i in (*range(6),*range(7,13))],[0.]*12)
        self.assertEqual(plan.max_velocity,.1)
        self.assertTrue(plan.q_start_is_command)


class PedalGestureTests(unittest.TestCase):
    def setUp(self):
        from robot_workbench.service import Application
        self.mode='HOLD';self.phase=None;self.sequence=0;self.stamp=1000.;self.calls=[]
        self.device=SimpleNamespace(id='cpu',environment='physical',runtime=SimpleNamespace(nonce='owner'))
        self.app=Application({'physical':self.device})
        self.flow=SimpleNamespace(generation=1,current={'id':'run','ended_wall':None},
            operator_controls={'double_press_window_s':.6},
            operator_session=dict(run_id='run',runtime_nonce='owner',seed=7,duration_s=12.,
                model_id='wam-cpu',expected_bundle_digest='a'*64))
        self.device.snapshot=lambda business:dict(runtime_nonce='owner',command_epoch=4,holder='inference',
            connected=True,feedback_healthy=True,stationary=self.mode=='HOLD',
            inference=dict(mode=self.mode,transition_stage=self.phase,runner_status='RUNNING'))
        self.flow.snapshot=lambda:dict(id='run',inference=dict(model_ready=True))
        self.focus=dict(client_id='page',started_wall=999.)
        def action(name,body):
            self.calls.append((name,body))
            if name=='inference_run':self.mode='MODEL'
            elif name=='inference_hold':self.mode='HOLD';self.phase=None
            elif name=='inference_teach':self.mode='HOLD'if self.mode=='TEACH'else'TEACH'
            return {'accepted':True}
        self.app.action=action

    def press(self,command,delta=0.):
        self.sequence+=1;self.stamp+=delta
        ref=SimpleNamespace(source_instance_id='pedal-source',sequence=self.sequence)
        return self.app._operator_pedal_action(self.device,self.flow,command,ref,self.stamp,self.focus)

    def test_hold_double_press_reuses_prepared_parameters_without_web_confirmation(self):
        self.flow.operator_session['wam_stall_recovery']=True
        self.press(11);self.assertEqual([x[0]for x in self.calls],['inference_hold'])
        self.press(11,.3)
        action,body=self.calls[-1];self.assertEqual(action,'inference_run')
        self.assertEqual((body['seed'],body['duration_s'],body['model_id']),(7,12.,'wam-cpu'))
        self.assertTrue(body['physical_clearance']);self.assertTrue(body['arms_supported'])
        self.assertEqual(body['request_id'],'pedal-pedal-source-2')
        self.assertTrue(body['wam_stall_recovery'])
        self.assertEqual(body['memory_start'],'continue')

    def test_six_hundred_ms_window_is_inclusive_and_memory_start_is_retained(self):
        self.flow.operator_session['model_id']='wam-mem-fixture-step1'
        for gap,accepted in ((.599999,True),(.6,True),(.600001,False)):
            with self.subTest(gap=gap):
                self.mode='HOLD';self.calls=[];self.flow.operator_pedal_gesture=None;self.stamp+=1.
                self.press(11);self.press(11,gap)
                starts=[body for action,body in self.calls if action=='inference_run']
                self.assertEqual(bool(starts),accepted)
                if starts:self.assertEqual(starts[0]['memory_start'],'continue')

    def test_first_press_in_model_stops_and_second_never_restarts(self):
        for mode,phase in [('MODEL',None),('HOLD','RESUMING'),('HOLD','MODEL_ADMITTED')]:
            with self.subTest(mode=mode,phase=phase):
                self.mode,self.phase=mode,phase;self.calls=[];self.stamp+=1
                self.press(11);self.assertEqual(self.mode,'HOLD')
                self.press(11,.3);self.assertEqual([x[0]for x in self.calls],['inference_hold'])
                self.press(11,.7);self.press(11,.3)
                self.assertEqual(self.calls[-1][0],'inference_run')

    def test_model_pedal_two_stops_and_pedal_three_is_ignored(self):
        self.mode='MODEL';self.press(33);self.assertEqual(self.calls,[])
        self.press(22,.3);self.assertEqual(self.calls[-1][0],'inference_hold')
        self.assertEqual(self.mode,'HOLD')

    def test_teach_must_finish_hold_before_a_separate_start_gesture(self):
        self.mode='TEACH';self.press(11);self.press(11,.3)
        self.assertEqual([x[0]for x in self.calls],['inference_hold'])
        self.press(11,.7);self.press(11,.3);self.assertEqual(self.calls[-1][0],'inference_run')

    def test_expiry_other_pedal_focus_and_replayed_sequence_cannot_start(self):
        self.press(11);self.press(11,.601)
        self.assertNotIn('inference_run',[x[0]for x in self.calls])
        self.press(22,.1);self.press(11,.1);self.press(11,.3)
        self.assertNotIn('inference_run',[x[0]for x in self.calls])
        self.press(11,.7);self.focus['started_wall']+=1;self.press(11,.3)
        self.assertNotIn('inference_run',[x[0]for x in self.calls])
        before=len(self.calls);self.sequence-=1;self.press(11,.1)
        self.assertEqual(len(self.calls),before)

    def test_unprepared_or_new_session_never_starts_from_old_first_press(self):
        self.press(11);self.flow.operator_session=None;self.press(11,.3)
        self.assertNotIn('inference_run',[x[0]for x in self.calls])


@unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated Linux ROS only')
class PedalSourceTests(unittest.TestCase):
    def test_fifty_ms_debounce_rejects_bounce_but_accepts_the_boundary(self):
        sys.path.insert(0,os.environ['PROMETHEUS_PEDAL_TEST_ROOT'])
        from prometheus.input.pedal_publisher import PedalKeyFilter,PedalMapping
        from robot_workbench.execution_settings import PEDAL_DEBOUNCE_S
        self.assertEqual(PEDAL_DEBOUNCE_S,.05)
        for gap,expected in ((.049999,None),(.05,11),(.050001,11)):
            pedal=PedalKeyFilter(PedalMapping(),confirm_presses=1,debounce_s=PEDAL_DEBOUNCE_S)
            self.assertEqual(pedal.process('1','down',now=0.),11)
            self.assertIsNone(pedal.process('1','up',now=.001))
            self.assertEqual(pedal.process('1','down',now=gap),expected)
        self.assertIsNone(pedal.process('1','down',now=1.),'held key must remain locked')
        self.assertIsNone(pedal.process('1','repeat',now=2.))

    def test_single_evdev_source_focus_and_shared_action_routing(self):
        from unittest.mock import patch
        environment=patch.dict(os.environ,ROS_LOCALHOST_ONLY='0');environment.start();self.addCleanup(environment.stop)
        from robot_workbench.ros_data import RosNode,SourceDirectory
        from robot_workbench.service import Application,Device
        reader_root=Path(os.environ['PROMETHEUS_PEDAL_TEST_ROOT'])
        import hashlib
        with tempfile.TemporaryDirectory(prefix='pedal-cpu-')as folder:
            root=Path(folder);read_fd,write_fd=os.pipe();self.addCleanup(os.close,write_fd)
            next_read,next_write=os.pipe();self.addCleanup(os.close,next_write)
            device_path=root/'same-pedal'
            profile=dict(device_id='pedal_cpu',data_plane=dict(schema='workbench.ros-data-plane.v1',namespace='/operator_pedal_cpu',domain_id=127),
                device=str(device_path),reader_root=str(reader_root),
                reader_sha256=hashlib.sha256((reader_root/'prometheus/input/pedal_publisher.py').read_bytes()).hexdigest(),rate_hz=100.,heartbeat_hz=10.)
            config=root/'config.json';config.write_text(json.dumps(profile));source_root=root/'source'
            entry=root/'reader-entry.py'
            entry.write_text('import signal,errno,runpy\ndef unplug(*args):raise OSError(errno.ENODEV,"synthetic evdev unplug")\nsignal.signal(signal.SIGUSR1,unplug)\nrunpy.run_path('+repr(str(ROOT/'scripts/serve_pedal_source.py'))+',run_name="__main__")\n')
            with(root/'source.log').open('w')as log:
                process=subprocess.Popen([sys.executable,'-u','-B',str(entry),'--profile',str(config),'--root',str(source_root)],pass_fds=(read_fd,next_read),stdout=log,stderr=log)
            os.close(read_fd);os.close(next_read)
            def stop_reader():
                if process.poll()is None:process.terminate()
                process.wait(5)
            self.addCleanup(stop_reader)
            deadline=time.monotonic()+8
            while not(source_root/'source.json').exists()or not json.loads((source_root/'source.json').read_text())['available']:
                self.assertIsNone(process.poll(),(root/'source.log').read_text());self.assertLess(time.monotonic(),deadline);time.sleep(.03)
            device=Device('pedal_cpu','fake',None);device.runtime=SimpleNamespace(nonce='cpu-owner')
            mode=['HOLD']
            device.snapshot=lambda business:dict(runtime_nonce='cpu-owner',command_epoch=1,holder='inference',
                connected=True,feedback_healthy=True,stationary=mode[0]=='HOLD',
                inference=dict(mode=mode[0],transition_stage=None,runner_status='RUNNING'))
            app=Application({'fake':device});calls=[]
            flow=SimpleNamespace(ros_plane=profile['data_plane'],generation=1,
                operator_controls=dict(focus_timeout_s=8.,event_max_age_s=1.,double_press_window_s=.4))
            device.workflows['inference']=flow
            def operator(dev,workflow,action,body):
                calls.append((action,body));mode[0]='TEACH'if action=='inference_teach'and mode[0]!='TEACH'else'HOLD'
                return {'accepted':True}
            app._operator_action=operator
            # Physical deployments can expose ROS beyond loopback. The consumer
            # must follow its declared plane, just like camera/state consumers.
            app.install_operator_pedal(device,flow,str(source_root/'source.json'))
            def close_consumer():
                flow.operator_input_closed.set();flow.operator_input_thread.join(3)
            self.addCleanup(close_consumer)
            deadline=time.monotonic()+5
            while getattr(flow,'operator_input_status',{}).get('state')!='unavailable':
                self.assertIsNone(process.poll());self.assertLess(time.monotonic(),deadline);time.sleep(.02)
            self.assertTrue(flow.operator_input_thread.is_alive(),'a missing USB device must not retire its consumer')
            device_path.symlink_to('/proc/self/fd/'+str(read_fd))
            deadline=time.monotonic()+5
            while getattr(flow,'operator_input_status',{}).get('state')!='ready':
                self.assertLess(time.monotonic(),deadline,getattr(flow,'operator_input_status',{}));time.sleep(.02)
            from test_recording_process import start_recording_fixture,stop_recording_fixture
            from robot_workbench.recording_process import CaptureSession
            from robot_workbench.ros_recording import capture_source
            recorder=start_recording_fixture();self.addCleanup(stop_recording_fixture,recorder)
            session=CaptureSession.create(root/'capture',request_id='pedal-record',purpose_profile='infer_debug',
                source_descriptors=[capture_source(flow.operator_pedal_source,**{k:profile['data_plane'][k]for k in ('namespace','domain_id')})])
            self.addCleanup(session.close)
            prepared=session.prepare()
            session.bind(run_id='pedal-record',business_kind='inference',policy_hash='a'*64,
                execution_owner=dict(owner_pid=os.getpid(),runtime_nonce='cpu-owner',command_epoch=1),health_source=prepared['health_source'])
            session.start()
            def key(code,age=0,values=(1,0)):
                stamp=time.time()-age;sec=int(stamp);usec=int((stamp-sec)*1e6)
                os.write(write_fd,b''.join(struct.pack('llHHI',sec,usec,1,code,value)for value in values))
            key(2);time.sleep(.3);self.assertEqual(calls,[],'unfocused pedal must not control the device')
            focus=dict(environment='fake',business='inference',request_id='focus-1',runtime_nonce='cpu-owner',epoch=1,client_id='page',active=True)
            app.action('inference_focus',focus)
            key(2);time.sleep(.3);key(3);time.sleep(.3);key(4)
            deadline=time.monotonic()+3
            while len(calls)<3:
                self.assertLess(time.monotonic(),deadline,getattr(flow,'operator_input_status',{}));time.sleep(.02)
            self.assertEqual([a for a,_ in calls],['inference_hold','inference_teach','inference_home'])
            self.assertTrue(calls[2][1]['physical_clearance']);self.assertTrue(calls[0][1]['request_id'].startswith('pedal-'))
            flow.current=dict(id='control-run',ended_wall=None)
            flow.operator_session=dict(run_id='control-run',runtime_nonce='cpu-owner',seed=7,duration_s=12.,
                model_id='wam-cpu',expected_bundle_digest='a'*64)
            flow.snapshot=lambda:dict(id='control-run',inference={'model_ready':True})
            ordinary_action=app.action
            def action(name,body):
                if name=='inference_run':calls.append((name,body));mode[0]='MODEL';return {'accepted':True}
                return ordinary_action(name,body)
            app.action=action
            self.assertEqual(flow.operator_pedal_source['metadata']['debounce_s'],.05)
            time.sleep(.3);key(2);time.sleep(.02);key(2);time.sleep(.2)
            self.assertEqual(len(calls),4,'50ms evdev debounce must suppress a20ms bounce')
            time.sleep(.4);key(2);time.sleep(.1);key(2)
            deadline=time.monotonic()+2
            while mode[0]!='MODEL':
                self.assertLess(time.monotonic(),deadline,getattr(flow,'operator_input_status',{}));time.sleep(.01)
            self.assertEqual(calls[-1][0],'inference_run');self.assertEqual(calls[-1][1]['duration_s'],12.)
            self.assertEqual(calls[-1][1]['memory_start'],'continue')
            count=len(calls);key(4);time.sleep(.3);self.assertEqual(len(calls),count,'pedal3 is ignored in MODEL')
            key(3);time.sleep(.3);self.assertEqual(calls[-1][0],'inference_hold');self.assertEqual(mode[0],'HOLD')
            time.sleep(.3);key(2);time.sleep(.3);key(2)
            deadline=time.monotonic()+2
            while mode[0]!='MODEL':
                self.assertLess(time.monotonic(),deadline,getattr(flow,'operator_input_status',{}));time.sleep(.01)
            time.sleep(.3);count=len(calls);key(2);time.sleep(.3);key(2);time.sleep(.3)
            self.assertEqual(len(calls),count+1);self.assertEqual(mode[0],'HOLD','second press of a MODEL stop gesture cannot restart')
            count=len(calls)
            time.sleep(.3);key(2,age=3);time.sleep(.3);self.assertEqual(len(calls),count,'old physical events must not trigger control')
            app.action('inference_focus',dict(focus,request_id='blur',active=False));time.sleep(.3);key(4);time.sleep(.2)
            self.assertEqual(len(calls),count)
            import signal
            original_source=deepcopy(flow.operator_pedal_source)
            device_path.unlink();process.send_signal(signal.SIGUSR1)
            deadline=time.monotonic()+3
            while flow.operator_input_status.get('state')!='unavailable':
                self.assertIsNone(process.poll());self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            device_path.symlink_to('/proc/self/fd/'+str(next_read));write_fd=next_write
            deadline=time.monotonic()+3
            while flow.operator_input_status.get('state')!='ready':
                self.assertIsNone(process.poll());self.assertLess(time.monotonic(),deadline,flow.operator_input_status);time.sleep(.01)
            self.assertEqual(flow.operator_pedal_source,original_source,'USB reconnect must not replace the logical source or require reprepare')
            app.action('inference_focus',dict(focus,request_id='focus-after-usb',active=True))
            key(3,values=(2,));time.sleep(.3);self.assertEqual(len(calls),count,'held-key repeats after reconnect cannot synthesize a new command')
            key(3);time.sleep(.3);self.assertEqual(mode[0],'TEACH')
            key(3);time.sleep(.3);self.assertEqual(mode[0],'HOLD')
            self.assertEqual(len(calls),count+2);count=len(calls)
            session.finish({},dict(status='completed'),source_endpoints={'pedal':dict(source_clock_id='host_wall',source_time=time.time())})
            self.assertTrue(session.done.wait(10),session.snapshot())
            manifest=json.loads((session.directory/'manifest.json').read_text())
            self.assertEqual(manifest['recording_outcome'],'complete',manifest.get('issues'))
            events=[json.loads(line)for line in(session.directory/'pedal.jsonl').read_text().splitlines()]
            self.assertTrue(any(row['payload']['event']=='pedal_command'and 'occurred_at'in row['payload']['data']for row in events))
            import signal
            process.send_signal(signal.SIGSTOP)
            try:
                deadline=time.monotonic()+3
                while flow.operator_input_status['state']!='unavailable':
                    self.assertLess(time.monotonic(),deadline,flow.operator_input_status);time.sleep(.02)
                self.assertIsNone(flow.operator_pedal_gesture)
            finally:process.send_signal(signal.SIGCONT)
            deadline=time.monotonic()+3
            while flow.operator_input_status['state']!='ready':
                self.assertLess(time.monotonic(),deadline,flow.operator_input_status);time.sleep(.02)
            process.terminate();process.wait(3);self.assertEqual(process.returncode,0)
            deadline=time.monotonic()+3
            while flow.operator_input_status['state']!='unavailable':
                self.assertLess(time.monotonic(),deadline,flow.operator_input_status);time.sleep(.02)
            self.assertEqual(len(calls),count,'heartbeat loss and recovery must not synthesize a pedal action')
            flow.operator_input_closed.set();flow.operator_input_thread.join(2)


if __name__=='__main__':unittest.main()
