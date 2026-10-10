"""Controlled slow-IO fixtures using the actual recorder; no CPFS/GPU qualification."""
from copy import deepcopy
from pathlib import Path
import queue
import sys
from tempfile import TemporaryDirectory
import threading
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench')]
from real_recording import RecordingSession,RealRecorder,CAMERAS,verify_raw
import test_recording_wall_clock as clock_fixture
from test_fake_device_pipeline import FAKE_CLOCK_ID,FAKE_RUNTIME_NONCE


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
        for session_factory,expected_error in ((OldWallCappedSession,True),(ObservedSession,False)):
            with self.subTest(policy=session_factory.__name__),TemporaryDirectory() as temp:
                h=self.helper();entered=threading.Event();release=threading.Event();count=30;prelude=12
                class SlowIO(RealRecorder):
                    def append(self,name,row):
                        time.sleep(.006 if name in ('commands.jsonl','feedback.jsonl') else .020)
                        return super().append(name,row)
                    def record_rgbd(self,camera,blob):
                        if not entered.is_set():
                            entered.set()
                            if not release.wait(5):raise TimeoutError('slow IO fixture not released')
                        return super().record_rgbd(camera,blob)
                h.setup_session(Path(temp),count=count,recorder_factory=SlowIO,session_factory=session_factory,capacity=16)
                identity=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                    owner_pid=1234,command_epoch=3,runtime_nonce=FAKE_RUNTIME_NONCE)
                # Same-lease startup frames legitimately precede the commands;
                # use a monotonic source/wall history, not a future frame backlog.
                for frame in range(prelude+1):
                    h.wall[0]=h.start+round(frame*2.6/30*1e9);h.publish(frame)
                base_ns=100_000_000_000+prelude*1_000_000_000//30;base_step=24000+prelude*8
                previous=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                    source_time_ns=base_ns,physics_step=base_step,sample_wall_ns=h.wall[0])
                h.session.submit('feedback',tick=0,state=h.plan['ticks'][0]['target'],sample_monotonic_s=base_ns/1e9,
                    sample_wall_ns=h.wall[0],raw=dict(previous,**{k:identity[k] for k in ('owner_pid','command_epoch','runtime_nonce')}))
                self.assertTrue(entered.wait(2),h.session.error)
                anchor=h.wall[0];started=time.monotonic();attempted_commands=0;max_queue=0;release.set()
                for grid in range(count*10+1):
                    time.sleep(max(0.,started+grid*2.6/300-time.monotonic()))
                    h.wall[0]=anchor+round((time.monotonic()-started)*1e9)
                    tick=min(grid//10,count-1)
                    if grid and grid%3==0:
                        step=base_step+grid*240//300;ns=step*1_000_000_000//240
                        previous=dict(clock_domain='simulation',source_clock_id=FAKE_CLOCK_ID,
                            source_time_ns=ns,physics_step=step,sample_wall_ns=h.wall[0])
                        try:h.session.submit('feedback',tick=tick,state=h.plan['ticks'][tick]['target'],sample_monotonic_s=ns/1e9,
                            sample_wall_ns=h.wall[0],raw=dict(previous,**{k:identity[k] for k in ('owner_pid','command_epoch','runtime_nonce')}))
                        except ValueError:pass  # The declared source/command sequence still advances after a data fault.
                    if grid%10==0:
                        frame=grid//10
                        if grid:h.publish(prelude+frame)
                        if frame<count:
                            attempted_commands+=1;ns=base_ns+grid*1_000_000_000//300
                            try:h.session.submit('command',tick=frame,target=h.plan['ticks'][frame]['target'],
                                send_monotonic_s=ns/1e9,send_wall_ns=h.wall[0],send_source_time_ns=ns,
                                physics_step=None,feedback_at_send=previous,**identity)
                            except ValueError:pass
                    max_queue=max(max_queue,h.session.events.qsize())
                h.session.finish('completed');self.assertTrue(h.session.done.wait(10),h.session.snapshot())
                self.assertEqual(attempted_commands,count)
                if expected_error:self.assertIn('bounded recording queue overflow',h.session.error)
                else:
                    self.assertIsNone(h.session.error,h.session.snapshot())
                    self.assertEqual(h.session.manifest['capture_status'],'complete',h.session.manifest)
                    self.assertEqual(len(h.session.recorder.commands),count)
                    self.assertEqual(len(h.session.recorder.feedback),101)
                    self.assertTrue(all(len(rows)==count+prelude+1 for rows in h.session.recorder.frames.values()))
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
