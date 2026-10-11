"""Controlled slow-IO fixtures using the actual recorder; no CPFS/GPU qualification."""
from copy import deepcopy
from pathlib import Path
import queue
import sys
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from real_recording import RecordingSession,RealRecorder,CAMERAS,SIDES,verify_raw
from real_rgbd import pair_metadata,snapshot_bundle
import test_recording_wall_clock as clock_fixture
from test_fake_device_pipeline import FAKE_RUNTIME_NONCE


class ObservedSession(RecordingSession):
    def _drain_events(self):
        super()._drain_events()
        if not hasattr(self,'batches'):self.batches=[]
        self.batches.append(deepcopy(self.last_event_batch))


class OldWallCappedSession(RecordingSession):
    """The previous 20 ms policy, using the same production event writer."""
    def _drain_events(self):
        started=time.monotonic()
        for index in range(64):
            try:event=self.events.get(timeout=.01 if index==0 else 0)
            except queue.Empty:break
            self._write_event(event)
            if time.monotonic()-started>=.02:break


class RecordingIOTest(unittest.TestCase):
    def helper(self):
        helper=clock_fixture.RecordingWallClockTest();self.addCleanup(helper.doCleanups)
        return helper

    def test_twenty_ms_policy_overflows_but_count_policy_preserves_every_event(self):
        # Compare scheduling policy, not this host's ability to meet a wall-clock
        # producer rate. Real sleep + catch-up bursts made both policies overflow
        # under load. Drive identical camera turns and virtual IO costs instead;
        # submit, event validation/writes, RGB-D archives and sealing remain real.
        for session_factory,expected_error in ((OldWallCappedSession,True),(ObservedSession,False)):
            with self.subTest(policy=session_factory.__name__),TemporaryDirectory() as temp:
                h=clock_fixture.RecordingWallClockTest();count=30;mono=[0.]
                h.wall=[1_000_000_000_000];h.start=h.wall[0]
                h.source,h.plan,h.binding,h.timeline=clock_fixture.plan_fixture(count,'fake')
                owner=dict(owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
                class VirtualIO(RealRecorder):
                    def append(self,name,row):
                        result=super().append(name,row)
                        mono[0]+=.006 if name in ('commands.jsonl','feedback.jsonl') else .020
                        return result
                kwargs=dict(environment='fake',execution_timeline=h.timeline,execution_owner=owner,
                            clock_ns=lambda:h.wall[0])
                # Use the real session setup, but this test owns each worker turn.
                with patch('threading.Thread.start'):
                    h.session=session_factory(Path(temp)/'raw',h.source,h.plan,h.binding,capacity=16,**kwargs)
                h.session.recorder=VirtualIO(h.session.root,h.source,h.plan,h.binding,**kwargs)
                attempted_commands=0;max_queue=0;drained=[]
                with patch('real_recording.time.monotonic',side_effect=lambda:mono[0]):
                    for first in range(0,count+1,4):
                        frames=[]
                        for index in range(first,min(first+4,count+1)):
                            h.wall[0]=h.start+index*100_000_000
                            try:h.submit_feedback(index)
                            except ValueError:pass  # Data failure never halts the declared source sequence.
                            if index<count:
                                attempted_commands+=1
                                try:h.submit_command(index)
                                except ValueError:pass
                            max_queue=max(max_queue,h.session.events.qsize())
                            source_ns=100_000_000_000+index*1_000_000_000//30
                            for camera in CAMERAS:
                                side=SIDES[camera]
                                meta=pair_metadata(serial=h.source['camera_serials'][side],camera_name=side,
                                    color_number=index,depth_number=index,color_ms=source_ns/1e6,depth_ms=source_ns/1e6,
                                    color_domain='simulation',depth_domain='simulation',host_receive_ns=h.wall[0],
                                    depth_scale_m=.001,intrinsics={'fx':600})
                                clock_fixture.fake_metadata(meta,source_ns,step=24000+index*8)
                                meta['host_receive_ns']=h.wall[0]
                                frames.append((camera,snapshot_bundle(meta,clock_fixture.png(),clock_fixture.png(True))))
                        before=h.session.events.qsize();h.session._drain_events()
                        drained.append(before-h.session.events.qsize())
                        # Same four frames per camera between event batches as _run.
                        for camera,blob in frames:h.session.recorder.record_rgbd(camera,blob)
                    h.session.finish('completed')
                    while not h.session.events.empty():h.session._drain_events()
                    h.session.manifest=h.session.recorder.seal('recording_failed' if h.session.error else 'completed')
                self.assertEqual(attempted_commands,count)
                self.assertEqual(h.session.controller_result['status'],'completed')
                if expected_error:
                    self.assertEqual(drained[:2],[4,4])
                    self.assertEqual(max_queue,16)
                    self.assertEqual(h.session.error,'bounded recording queue overflow')
                    self.assertEqual(h.session.manifest['capture_status'],'incomplete')
                else:
                    self.assertEqual(drained,[8]*7+[5])
                    self.assertIsNone(h.session.error,h.session.snapshot())
                    self.assertEqual(h.session.manifest['capture_status'],'complete',h.session.manifest)
                    self.assertEqual(len(h.session.recorder.commands),count)
                    self.assertEqual(len(h.session.recorder.feedback),count+1)
                    self.assertTrue(all(len(rows)==count+1 for rows in h.session.recorder.frames.values()))
                    self.assertLessEqual(max_queue,16)
                    self.assertTrue(all(row['count']<=64 for row in h.session.batches))
                    self.assertGreater(max(row['duration_wall_s'] for row in h.session.batches),.02)
                    verify_raw(h.session.root,environment='fake')

    def tail_case(self,*,missing=False,expire=False,late=False):
        h=self.helper();temp=TemporaryDirectory();self.addCleanup(temp.cleanup)
        entered=threading.Event();release=threading.Event();count=30
        class SlowEvents(RealRecorder):
            def append(self,name,row):
                if name in ('commands.jsonl','feedback.jsonl'):time.sleep(.022)
                result=super().append(name,row)
                if expire and name=='commands.jsonl':h.wall[0]=h.start+2_000_000_000
                return result
            def record_rgbd(self,camera,blob):
                if not entered.is_set():
                    entered.set()
                    if not release.wait(5):raise TimeoutError('tail fixture not released')
                return super().record_rgbd(camera,blob)
        h.setup_session(Path(temp.name),count=count,budget=2 if expire else 900,
            recorder_factory=SlowEvents,session_factory=ObservedSession)
        h.publish(0);h.submit_feedback(0);self.assertTrue(entered.wait(2),h.session.error)
        for index in range(count+1):
            h.wall[0]=h.start+index*1_000_000
            if index:h.publish(index)
            if index<count:h.submit_command(index)
            if index:h.submit_feedback(index)
        if missing:
            spool=h.spools[CAMERAS[-1]]
            (spool.root/spool.active/f'{count+1:09d}.zip').unlink()  # Deliberate missing-tail fault, never source relabelling.
        delayed=None
        if late:
            path=spool.root/spool.active/f'{count:09d}.zip';payload=path.read_bytes();path.unlink()
            def publish_late():
                deadline=time.monotonic()+5
                while len(h.session.recorder.frames[CAMERAS[-1]])<count-1 and time.monotonic()<deadline:time.sleep(.002)
                time.sleep(.6)
                temporary=path.with_suffix('.partial');temporary.write_bytes(payload);temporary.replace(path)
            delayed=threading.Thread(target=publish_late,daemon=True);delayed.start()
        requested=time.monotonic();h.session.finish('completed');release.set()
        self.assertTrue(h.session.done.wait(8),h.session.snapshot())
        if delayed:delayed.join(1);self.assertFalse(delayed.is_alive())
        return h,time.monotonic()-requested

    def test_existing_complete_tail_survives_event_drain_longer_than_one_wall_second(self):
        h,elapsed=self.tail_case()
        self.assertGreater(elapsed,1.)
        self.assertGreater(max(row['duration_wall_s'] for row in h.session.batches),1.)
        self.assertIsNone(h.session.error,h.session.snapshot())
        self.assertEqual(h.session.manifest['capture_status'],'complete',h.session.manifest)
        self.assertTrue(all(len(rows)==31 for rows in h.session.recorder.frames.values()))

    def test_missing_tail_waits_only_one_second_after_archiving_available_data(self):
        h,elapsed=self.tail_case(missing=True)
        event_s=sum(row['duration_wall_s'] for row in h.session.batches if row['count'])
        self.assertGreaterEqual(elapsed-event_s,1.)
        self.assertLess(elapsed-event_s,1.75)
        self.assertIsNone(h.session.error,h.session.snapshot())
        self.assertEqual(h.session.manifest['capture_status'],'incomplete')
        self.assertIn(CAMERAS[-1]+': RGB-D does not bracket execution',h.session.manifest['validation']['problems'])

    def test_resident_deadline_still_rejects_terminal_drain(self):
        h,_=self.tail_case(expire=True)
        self.assertIn('fixed resident wall deadline',h.session.error)
        self.assertEqual(h.session.manifest['capture_status'],'incomplete')

    def test_partial_late_tail_does_not_restart_the_one_second_wait(self):
        h,elapsed=self.tail_case(missing=True,late=True)
        event_s=sum(row['duration_wall_s'] for row in h.session.batches if row['count'])
        self.assertGreaterEqual(elapsed-event_s,1.)
        self.assertLess(elapsed-event_s,1.45,'a partial frame must not grant a fresh second of waiting')
        self.assertEqual(len(h.session.recorder.frames[CAMERAS[-1]]),30)
        self.assertEqual(h.session.manifest['capture_status'],'incomplete')


if __name__=='__main__':unittest.main()
