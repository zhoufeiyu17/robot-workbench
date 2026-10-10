"""Real recorder/spool CPU fixtures with injected wall time; no plant or SDK."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import threading
import unittest
from unittest.mock import patch

from control_motion import validate_control
from execution_timeline import build
from integration import _digest
from physical_motiongen import freeze_physical
from real_recording import RecordingSession,RealRecorder,CAMERAS,SIDES,verify_raw,_renew_recording_lease
from real_rgbd import RawSpool,pair_metadata
from test_fake_device_pipeline import fake_inputs,fake_metadata,FAKE_CLOCK_ID,FAKE_RUNTIME_NONCE
from test_physical_motiongen import fake_inputs as physical_inputs
from test_real_recording import png


def plan_fixture(count,environment):
    source,model,binding=fake_inputs() if environment=='fake' else physical_inputs()
    ticks,actions=model['ticks'],model['native_actions']
    model.update(ticks=[dict(tick=i,time_s=i/30,target=deepcopy(ticks[min(i,2)]['target'])) for i in range(count)],
        native_actions=[deepcopy(actions[min(i,2)]) for i in range(count)],native_substeps=[8]*count,duration_s=count/30)
    model['timeline_digest']=_digest(model['ticks'])
    model['planning_validation']['timeline_digest']=model['timeline_digest']
    model['control_validation']=validate_control(model)
    plan=freeze_physical(model,source,binding,environment=environment)
    return source,plan,binding,build(plan)


class RecordingWallClockTest(unittest.TestCase):
    def wait_for(self,predicate,session=None,seconds=5):
        deadline=time.monotonic()+seconds
        while not predicate():
            if session and session.done.is_set():self.fail('recorder stopped early: '+str(session.snapshot()))
            if time.monotonic()>=deadline:self.fail('CPU recorder fixture did not progress')
            time.sleep(.001)

    def setup_session(self,root,count=300,environment='fake',budget=900,recorder_factory=RealRecorder):
        self.wall=[1_000_000_000_000];self.start=self.wall[0]
        self.source,self.plan,self.binding,self.timeline=plan_fixture(count,environment)
        for camera in CAMERAS:(root/camera).mkdir()
        self.spools={camera:RawSpool(root/camera,clock_ns=lambda:self.wall[0],min_free_bytes=0) for camera in CAMERAS}
        self.directories={camera:str(root/camera) for camera in CAMERAS}
        self.faults=[]
        owner=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
        self.session=RecordingSession(root/'raw',self.source,self.plan,self.binding,
            environment=environment,execution_timeline=self.timeline,execution_owner=owner if environment=='fake' else None,
            recorder_factory=recorder_factory,
            camera_directories=self.directories,clock_ns=lambda:self.wall[0],on_fault=self.faults.append,
            **({'resident_deadline_wall_ns':self.start+round((budget+45)*1e9)} if environment=='fake' else {}))
        self.addCleanup(self.close_session)
        self.wait_for(lambda:all((Path(path)/'recording-request.json').exists() for path in self.directories.values()),self.session)
        self.tokens={camera:json.loads((Path(path)/'recording-request.json').read_text())['run_id']
            for camera,path in self.directories.items()}

    def close_session(self):
        if hasattr(self,'session') and not self.session.done.is_set():
            self.session.finish('stopped');self.session.done.wait(5)

    def publish(self,index,*,sample_wall_ns=None,changes=None):
        source_ns=100_000_000_000+index*1_000_000_000//30
        for camera,spool in self.spools.items():
            side=SIDES[camera]
            meta=pair_metadata(serial=self.source['camera_serials'][side],camera_name=side,
                color_number=index,depth_number=index,color_ms=source_ns/1e6,depth_ms=source_ns/1e6,
                color_domain='simulation',depth_domain='simulation',host_receive_ns=self.wall[0],
                depth_scale_m=.001,intrinsics={'fx':600})
            if self.session.environment=='fake':fake_metadata(meta,source_ns,step=24_000+index*8)
            meta['host_receive_ns']=self.wall[0] if sample_wall_ns is None else sample_wall_ns
            meta.update(changes or {})
            spool.write(meta,png(),png(True))
        return source_ns

    def submit_feedback(self,index,**changes):
        source_ns=100_000_000_000+index*1_000_000_000//30
        raw=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
            source_time_ns=source_ns,physics_step=24_000+index*8,sample_wall_ns=self.wall[0],
            owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
        raw.update(changes)
        self.session.submit('feedback',tick=min(index,len(self.plan['ticks'])-1),
            state=self.plan['ticks'][min(index,len(self.plan['ticks'])-1)]['target'],
            sample_monotonic_s=raw['source_time_ns']/1e9,sample_wall_ns=self.wall[0],raw=raw)

    def submit_command(self,index):
        source_ns=100_000_000_000+index*1_000_000_000//30
        sample=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
            source_time_ns=source_ns,physics_step=24_000+index*8,sample_wall_ns=self.wall[0])
        self.session.submit('command',tick=index,target=self.plan['ticks'][index]['target'],
            send_monotonic_s=source_ns/1e9,send_wall_ns=self.wall[0],send_source_time_ns=source_ns,
            physics_step=None,feedback_at_send=sample,clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
            owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)

    def test_slow_source_and_cross_six_hundred_wall_seconds_keep_complete_raw_and_one_spool(self):
        for count,slowdown in ((300,4),(300,24),(4530,4)):
            with self.subTest(source_seconds=count/30,slowdown=slowdown),TemporaryDirectory() as temp:
                root=Path(temp);self.setup_session(root,count)
                self.publish(0);self.submit_feedback(0)
                self.assertTrue(self.session.ready.wait(5),self.session.error)
                for index in range(count+1):
                    if index:
                        self.wall[0]=self.start+round(index*slowdown/30*1e9)
                        if index==1 and slowdown==24:
                            time.sleep(.05)  # No new sample yet; old wall-only freshness must fail here.
                            self.assertFalse(self.session.done.is_set(),self.session.error)
                        source_ns=self.publish(index)
                    else:source_ns=100_000_000_000
                    identity=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                        owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
                    sample=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                        source_time_ns=source_ns,physics_step=24_000+index*8,sample_wall_ns=self.wall[0])
                    target=self.plan['ticks'][min(index,count-1)]['target']
                    if index<count:
                        self.session.submit('command',tick=index,target=target,send_monotonic_s=source_ns/1e9,
                            send_wall_ns=self.wall[0],send_source_time_ns=source_ns,physics_step=None,
                            feedback_at_send=sample,**identity)
                    if index:self.submit_feedback(index)
                    self.wait_for(lambda:len(self.session.recorder.feedback)==index+1
                        and len(self.session.recorder.commands)==min(index+1,count)
                        and all(len(rows)==index+1 for rows in self.session.recorder.frames.values()),self.session)
                for camera,spool in self.spools.items():
                    self.assertEqual(spool.active,self.tokens[camera]);self.assertEqual(spool.count,count+1)
                    request=json.loads((Path(self.directories[camera])/'recording-request.json').read_text())
                    self.assertEqual(request['run_id'],self.tokens[camera])
                    self.assertLessEqual(request['deadline_wall_ns']-request['start_wall_ns'],600_000_000_000)
                    self.assertLessEqual(request['deadline_wall_ns'],self.start+900_000_000_000)
                    if count>300:self.assertGreater(request['deadline_wall_ns'],self.start+600_000_000_000)
                self.session.finish('completed');self.assertTrue(self.session.done.wait(5),self.session.error)
                self.assertFalse(self.faults);self.assertEqual(self.session.manifest['capture_status'],'complete')
                self.assertEqual(self.session.manifest['execution_duration_s'],count/30)
                self.assertEqual(self.session.manifest['camera_frames'],dict.fromkeys(CAMERAS,count+1))
                self.assertEqual(verify_raw(root/'raw',environment='fake')[0],self.session.manifest)
                self.assertTrue(all(not (Path(path)/'recording-request.json').exists() for path in self.directories.values()))
                self.wall[0]+=1_000_000_000_000;self.publish(count+1)
                self.assertTrue(all(spool.active is None and spool.count==count+1 for spool in self.spools.values()))

    def test_source_stall_and_fixed_parent_deadline_fail_data_and_remove_leases(self):
        for cause in ('source_stall','resident_deadline'):
            with self.subTest(cause=cause),TemporaryDirectory() as temp:
                root=Path(temp);self.setup_session(root,budget=60 if cause=='source_stall' else 10)
                if cause=='source_stall':
                    self.publish(0);self.submit_feedback(0);self.assertTrue(self.session.ready.wait(5),self.session.error)
                    self.wait_for(lambda:all(self.session.recorder.frames.values()),self.session)
                self.wall[0]+=30_001_000_000 if cause=='source_stall' else 10_000_000_000
                self.assertTrue(self.session.done.wait(5));self.assertTrue(self.faults)
                self.assertIn('stalled' if cause=='source_stall' else 'fixed resident wall deadline',self.session.error)
                self.assertEqual(self.session.manifest['controller_outcome'],'unverified')
                self.assertEqual(self.session.manifest['capture_status'],'incomplete')
                self.assertTrue(all(not (Path(path)/'recording-request.json').exists() for path in self.directories.values()))

    def test_fake_first_frame_uses_actual_feedback_not_wall_age_or_future_reference(self):
        with TemporaryDirectory() as temp:
            self.setup_session(Path(temp))
            # A genuine frame may spend >500ms awaiting encoding/IO. Keep its old
            # actual wall stamp while supplying same-source Owner evidence later.
            self.wall[0]+=800_000_000
            self.publish(1,sample_wall_ns=self.start)
            time.sleep(.05);self.assertFalse(self.session.ready.is_set())
            self.submit_feedback(0)
            time.sleep(.05);self.assertFalse(self.session.ready.is_set(),'frame ahead of Owner must wait for actual feedback')
            self.wall[0]+=1_000_000
            self.submit_feedback(1)
            self.assertTrue(self.session.ready.wait(5),self.session.error)
            self.wait_for(lambda:all(self.session.recorder.frames.values()),self.session)
            for rows in self.session.recorder.frames.values():
                self.assertEqual(int(rows[0]['host_receive_ns']),self.start,'never relabel original sample time')
                self.assertEqual(rows[0]['physics_step'],24008)
            self.wall[0]+=800_000_000;self.publish(2)
            self.wait_for(lambda:all(len(rows)==2 for rows in self.session.recorder.frames.values()),self.session)
            time.sleep(.05);self.assertFalse(self.session.done.is_set(),self.session.error)
            self.submit_feedback(2)
            self.wait_for(lambda:len(self.session.recorder.feedback)==3,self.session)

    def test_fake_rejects_source_lag_instance_change_and_feedback_regression(self):
        for cause in ('source_lag','wrong_frame_clock','wrong_feedback_clock','feedback_regression'):
            with self.subTest(cause=cause),TemporaryDirectory() as temp:
                self.setup_session(Path(temp))
                self.publish(0);self.submit_feedback(0)
                self.assertTrue(self.session.ready.wait(5),self.session.error)
                self.wait_for(lambda:all(self.session.recorder.frames.values()),self.session)
                self.wall[0]+=10_000_000
                if cause=='source_lag':self.submit_feedback(18)
                elif cause=='wrong_frame_clock':self.publish(1,changes={'source_clock_id':'other-plant'})
                else:
                    with self.assertRaises(ValueError):
                        if cause=='wrong_feedback_clock':self.submit_feedback(1,source_clock_id='other-plant')
                        else:self.submit_feedback(0,source_time_ns=99_995_833_333,physics_step=23999)
                self.assertTrue(self.session.done.wait(5));self.assertTrue(self.faults)
                self.assertIn({'source_lag':'source lag exceeds 0.5','wrong_frame_clock':'source clock differs',
                    'wrong_feedback_clock':'source clock differs','feedback_regression':'repeated or regressed'}[cause],self.session.error)
                self.assertEqual(self.session.manifest['controller_outcome'],'unverified')
                self.assertEqual(self.session.manifest['capture_status'],'incomplete')
                diagnostic=json.loads((Path(temp)/'raw/recording-diagnostics.json').read_text())
                self.assertEqual(diagnostic['error'],self.session.error)
                self.assertEqual(diagnostic['queue']['capacity'],512)
                self.assertIn('recording-diagnostics.json',self.session.manifest['files'])
                if cause=='source_lag':
                    camera=diagnostic['cameras'][CAMERAS[0]]
                    self.assertEqual(camera['checked_spool_index'],1)
                    self.assertEqual(camera['checked_spool_head']['source_time_ns'],100_000_000_000)
                    self.assertEqual(diagnostic['checked_reference']['source_time_ns'],100_600_000_000)

    def test_fake_writer_backlog_is_not_source_lag_and_both_inputs_get_bounded_turns(self):
        entered=threading.Event();release=threading.Event();camera_turns=[];test=self
        class DelayedRecorder(RealRecorder):
            def record_rgbd(self,camera,blob):
                if not entered.is_set():
                    entered.set()
                    if not release.wait(5):raise TimeoutError('CPU fixture did not release archive writer')
                camera_turns.append((len(self.feedback),test.session.events.qsize()))
                return super().record_rgbd(camera,blob)
        with TemporaryDirectory() as temp:
            count=120;root=Path(temp)
            self.setup_session(root,count=count,recorder_factory=DelayedRecorder)
            self.publish(0);self.submit_feedback(0)
            self.assertTrue(entered.wait(5),self.session.error)
            try:
                for index in range(count+1):
                    self.wall[0]=self.start+index*200_000_000
                    if index:self.publish(index)
                    if index<count:self.submit_command(index)
                    if index:self.submit_feedback(index)
                self.assertGreater(self.session.events.qsize(),64)
            finally:release.set()
            self.wait_for(lambda:len(self.session.recorder.feedback)==count+1
                and len(self.session.recorder.commands)==count
                and all(len(rows)==count+1 for rows in self.session.recorder.frames.values()),self.session)
            self.assertTrue(any(feedback>2 and pending>0 for feedback,pending in camera_turns),camera_turns)
            for camera,rows in self.session.recorder.frames.items():
                self.assertEqual([row['frame_number'] for row in rows],list(range(count+1)))
                self.assertEqual(self.session.spool_heads[camera]['index'],count+1)
            self.session.finish('completed');self.assertTrue(self.session.done.wait(5),self.session.error)
            self.assertFalse(self.faults);self.assertEqual(self.session.manifest['capture_status'],'complete')
            verify_raw(root/'raw',environment='fake')

    def test_fresh_spool_does_not_hide_missing_execution_start_coverage(self):
        with TemporaryDirectory() as temp:
            count=30;first_frame=6;self.setup_session(Path(temp),count=count)
            for index in range(count+1):
                self.wall[0]=self.start+index*50_000_000
                if index>=first_frame:self.publish(index)
                if index<count:self.submit_command(index)
                self.submit_feedback(index)
            self.wait_for(lambda:len(self.session.recorder.feedback)==count+1
                and all(len(rows)==count-first_frame+1 for rows in self.session.recorder.frames.values()),self.session)
            self.session.finish('completed');self.assertTrue(self.session.done.wait(5),self.session.error)
            self.assertFalse(self.faults)
            self.assertEqual(self.session.manifest['capture_status'],'incomplete')
            self.assertEqual(set(self.session.manifest['validation']['problems']),
                {camera+': RGB-D does not bracket execution' for camera in CAMERAS})
            self.assertEqual(self.session.manifest['controller_outcome'],'completed')

    def test_fake_camera_source_stall_is_bounded_even_when_owner_keeps_advancing(self):
        with TemporaryDirectory() as temp:
            self.setup_session(Path(temp));self.publish(0);self.submit_feedback(0)
            self.assertTrue(self.session.ready.wait(5),self.session.error)
            self.wait_for(lambda:all(self.session.recorder.frames.values()),self.session)
            time.sleep(.02)
            for index in range(1,31):
                self.wall[0]+=1_000_000_000
                self.submit_feedback(index,source_time_ns=100_000_000_000+index*1_000_000_000//240,
                    physics_step=24000+index)
                time.sleep(.02)
            self.assertTrue(self.session.done.wait(5));self.assertTrue(self.faults)
            self.assertIn('native RGB-D source stalled for 30 wall seconds',self.session.error)

    def test_camera_stays_ahead_but_owner_reaches_each_previously_pending_frame(self):
        # Reproduce the ordering of each IO round: freeze actual Owner feedback,
        # then observe a newer camera frame while the other cameras were written.
        session=object.__new__(RecordingSession)
        session.clock=fake_inputs()[0]['clock'];session.pending_reference={}
        wall=[1_000_000_000_000];session.clock_ns=lambda:wall[0]
        def stamp(index):return (100_000_000_000+index*1_000_000_000//30,24_000+index*8)
        for index in range(1,41):
            wall[0]+=2_000_000_000
            current=stamp(index)
            meta=dict(host_receive_ns=wall[0],clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                source_time_ns=current[0],physics_step=current[1])
            reference=(stamp(index-1),wall[0],time.monotonic())
            self.assertEqual(session.check_fake_head(CAMERAS[-1],meta,reference),current)
            self.assertEqual(session.pending_reference[CAMERAS[-1]][0],current)
        session.check_fake_head(CAMERAS[-1],meta,(stamp(40),wall[0],time.monotonic()))
        self.assertFalse(session.pending_reference)

    def test_owner_advancement_short_of_fixed_pending_frame_cannot_restart_its_timeout(self):
        session=object.__new__(RecordingSession)
        session.clock=fake_inputs()[0]['clock'];session.pending_reference={}
        wall=[1_000_000_000_000];session.clock_ns=lambda:wall[0]
        for index in range(16):
            wall[0]+=2_000_000_000
            current=(101_000_000_000+index*1_000_000_000//30,24_240+index*8)
            meta=dict(host_receive_ns=wall[0],clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                source_time_ns=current[0],physics_step=current[1])
            reference=((100_000_000_000+index*1_000_000_000//240,24_000+index),wall[0],time.monotonic())
            if index<15:session.check_fake_head(CAMERAS[-1],meta,reference)
            else:
                with self.assertRaisesRegex(ValueError,'did not reach frame within 30 wall seconds'):
                    session.check_fake_head(CAMERAS[-1],meta,reference)
            self.assertEqual(session.pending_reference[CAMERAS[-1]][0],(101_000_000_000,24_240))

    def test_diagnostic_write_failure_keeps_primary_error_and_still_attempts_seal(self):
        for fail_seal in (False,True):
            with self.subTest(fail_seal=fail_seal),TemporaryDirectory() as temp:
                class FailedDiagnosticRecorder(RealRecorder):
                    def write_json(self,name,value):
                        if name=='recording-diagnostics.json':raise OSError('diagnostic storage failure')
                        if name=='manifest.json' and fail_seal:raise OSError('manifest storage failure')
                        return super().write_json(name,value)
                    def record_rgbd(self,camera,blob):raise ValueError('original frame failure')
                self.setup_session(Path(temp),recorder_factory=FailedDiagnosticRecorder)
                self.publish(0);self.submit_feedback(0)
                self.assertTrue(self.session.done.wait(5),self.session.snapshot())
                self.assertEqual(self.session.error,'original frame failure')
                self.assertEqual(self.faults,['original frame failure'])
                self.assertEqual(self.session.snapshot()['diagnostics_error'],'OSError: diagnostic storage failure')
                if fail_seal:
                    self.assertIsNone(self.session.manifest)
                    self.assertEqual(self.session.snapshot()['seal_error'],'OSError: manifest storage failure')
                else:
                    self.assertIsNone(self.session.seal_error)
                    self.assertEqual(self.session.manifest['recording_error'],'original frame failure')
                    self.assertEqual(self.session.manifest['capture_status'],'incomplete')
                    self.assertEqual(self.session.manifest['controller_outcome'],'unverified')
                    verify_raw(Path(temp)/'raw',environment='fake')
                self.assertTrue(all(not (Path(path)/'recording-request.json').exists() for path in self.directories.values()))

    def test_physical_first_frame_and_running_freshness_remain_five_hundred_wall_ms(self):
        for stale_at_start in (False,True):
            with self.subTest(stale_at_start=stale_at_start),TemporaryDirectory() as temp:
                self.setup_session(Path(temp),environment='physical')
                if stale_at_start:self.wall[0]+=501_000_000
                self.publish(0,sample_wall_ns=self.start)
                if not stale_at_start:
                    self.assertTrue(self.session.ready.wait(5),self.session.error)
                    self.wait_for(lambda:all(self.session.recorder.frames.values()),self.session)
                    self.wall[0]+=501_000_000
                self.assertTrue(self.session.done.wait(5));self.assertTrue(self.faults)
                self.assertIn('absent or stale' if stale_at_start else 'recording stalled',self.session.error)

    def test_physical_lease_retains_source_duration_plus_fifteen_and_completion_stops_renewal(self):
        with TemporaryDirectory() as temp:
            self.setup_session(Path(temp),environment='physical')
            for path in self.directories.values():
                request=json.loads((Path(path)/'recording-request.json').read_text())
                self.assertEqual(request['deadline_wall_ns']-request['start_wall_ns'],25_000_000_000)
            self.session.finish('stopped','explicit completion during preparation')
            self.assertTrue(self.session.done.wait(5));self.assertFalse(self.faults)
            self.assertTrue(all(not (Path(path)/'recording-request.json').exists() for path in self.directories.values()))

    def test_renewal_rejects_changed_expired_and_no_longer_active_lease(self):
        for cause in ('other_owner','expired','changed_during_write','finished'):
            with self.subTest(cause=cause),TemporaryDirectory() as temp:
                path=Path(temp)/'recording-request.json';token='a'*32
                lease=dict(run_id=token,start_wall_ns=1_000_000_000,deadline_wall_ns=601_000_000_000)
                if cause=='other_owner':lease['run_id']='b'*32
                path.write_text(json.dumps(lease));before=path.read_bytes();now=601_000_000_000 if cause=='expired' else 301_000_000_000
                def active():
                    if cause=='changed_during_write':path.write_text(json.dumps(dict(lease,run_id='b'*32)))
                    return cause!='finished'
                if cause=='finished':
                    self.assertFalse(_renew_recording_lease(path,token,601_000_000_000,901_000_000_000,clock_ns=lambda:now,active=active))
                else:
                    with self.assertRaises(ValueError):_renew_recording_lease(path,token,601_000_000_000,901_000_000_000,clock_ns=lambda:now,active=active)
                if cause=='changed_during_write':self.assertEqual(json.loads(path.read_text())['run_id'],'b'*32)
                else:self.assertEqual(path.read_bytes(),before)
                self.assertEqual(list(Path(temp).iterdir()),[path])


if __name__=='__main__':unittest.main()
