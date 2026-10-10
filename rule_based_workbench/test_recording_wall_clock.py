"""Real recorder/spool CPU fixtures with injected wall time; no plant or SDK."""
from copy import deepcopy
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch

from control_motion import validate_control
from execution_timeline import build
from integration import _digest
from physical_motiongen import freeze_physical
from real_recording import RecordingSession,CAMERAS,SIDES,verify_raw,_renew_recording_lease
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

    def setup_session(self,root,count=300,environment='fake',budget=900):
        self.wall=[1_000_000_000_000];self.start=self.wall[0]
        self.source,self.plan,self.binding,self.timeline=plan_fixture(count,environment)
        for camera in CAMERAS:(root/camera).mkdir()
        self.spools={camera:RawSpool(root/camera,clock_ns=lambda:self.wall[0],min_free_bytes=0) for camera in CAMERAS}
        self.directories={camera:str(root/camera) for camera in CAMERAS}
        self.faults=[]
        owner=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
        self.session=RecordingSession(root/'raw',self.source,self.plan,self.binding,
            environment=environment,execution_timeline=self.timeline,execution_owner=owner if environment=='fake' else None,
            camera_directories=self.directories,clock_ns=lambda:self.wall[0],on_fault=self.faults.append,
            **({'resident_deadline_wall_ns':self.start+round((budget+45)*1e9)} if environment=='fake' else {}))
        self.addCleanup(self.close_session)
        self.wait_for(lambda:all((Path(path)/'recording-request.json').exists() for path in self.directories.values()),self.session)
        self.tokens={camera:json.loads((Path(path)/'recording-request.json').read_text())['run_id']
            for camera,path in self.directories.items()}

    def close_session(self):
        if hasattr(self,'session') and not self.session.done.is_set():
            self.session.finish('stopped');self.session.done.wait(5)

    def publish(self,index):
        source_ns=100_000_000_000+index*1_000_000_000//30
        for camera,spool in self.spools.items():
            side=SIDES[camera]
            meta=pair_metadata(serial=self.source['camera_serials'][side],camera_name=side,
                color_number=index,depth_number=index,color_ms=source_ns/1e6,depth_ms=source_ns/1e6,
                color_domain='simulation',depth_domain='simulation',host_receive_ns=self.wall[0],
                depth_scale_m=.001,intrinsics={'fx':600})
            fake_metadata(meta,source_ns,step=24_000+index*8);meta['host_receive_ns']=self.wall[0]
            spool.write(meta,png(),png(True))
        return source_ns

    def test_slow_source_and_cross_six_hundred_wall_seconds_keep_complete_raw_and_one_spool(self):
        for count in (300,4530):  # 10/151 source seconds, 40/604 injected wall seconds.
            with self.subTest(source_seconds=count/30),TemporaryDirectory() as temp:
                root=Path(temp);self.setup_session(root,count)
                self.publish(0);self.assertTrue(self.session.ready.wait(5),self.session.error)
                for index in range(count+1):
                    if index:
                        self.wall[0]=self.start+round(index*4/30*1e9)
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
                    self.session.submit('feedback',tick=min(index,count-1),state=target,sample_monotonic_s=source_ns/1e9,
                        sample_wall_ns=self.wall[0],source_time_ns=source_ns,physics_step=sample['physics_step'],**identity)
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
                root=Path(temp);self.setup_session(root,budget=10)
                if cause=='source_stall':
                    self.publish(0);self.assertTrue(self.session.ready.wait(5),self.session.error)
                    self.wait_for(lambda:all(self.session.recorder.frames.values()),self.session)
                self.wall[0]+=600_000_000 if cause=='source_stall' else 10_000_000_000
                self.assertTrue(self.session.done.wait(5));self.assertTrue(self.faults)
                self.assertIn('stalled' if cause=='source_stall' else 'fixed resident wall deadline',self.session.error)
                self.assertEqual(self.session.manifest['controller_outcome'],'unverified')
                self.assertEqual(self.session.manifest['capture_status'],'incomplete')
                self.assertTrue(all(not (Path(path)/'recording-request.json').exists() for path in self.directories.values()))

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
