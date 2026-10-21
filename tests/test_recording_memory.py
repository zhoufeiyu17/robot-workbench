"""Actual Owner/recorder preparation with memory cameras; CPU only, no ffmpeg."""
from pathlib import Path
import sys
import threading
import time
from copy import deepcopy
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench')]
import test_recording_preparation as prior
from robot_workbench.rgbd_channel import MemoryRgbdBudget,MemoryRgbdChannel


class SharedMemoryBudgetTest(unittest.TestCase):
    def channels(self,budget):
        owner=dict(owner_pid=1234,command_epoch=1,runtime_nonce='shared-budget-test')
        lease=dict(run_id='a'*32,start_wall_ns=10,deadline_wall_ns=100,
                   execution_instance_id='byte-budget-test',execution_owner=owner)
        channels=[MemoryRgbdChannel(lease_guard=lambda _:True,budget=budget,clock_ns=lambda:20) for _ in range(3)]
        for channel in channels:channel.begin(lease)
        return channels,dict(host_receive_ns=20,**owner),lease['run_id']

    def test_three_cameras_share_one_gib_without_frame_cap(self):
        budget=MemoryRgbdBudget();channels,meta,token=self.channels(budget)
        self.assertEqual(budget.max_bytes,1024**3)
        for channel in channels:
            for index in range(33):
                self.assertTrue(channel.write(dict(meta,frame_number=index),b'color',b'depth',sampled_token=token))
        self.assertEqual([ch.snapshot()['queued_frames'] for ch in channels],[33]*3)
        self.assertEqual(budget.size,sum(ch.size for ch in channels))
        self.assertTrue(all(ch.snapshot()['capacity'] is None for ch in channels))
        for channel in channels:
            for index in range(33):
                frame=channel.pop(token)
                self.assertEqual(frame['metadata']['frame_number'],index)
                self.assertEqual((frame['color_png'],frame['depth_png']),(b'color',b'depth'))
            channel.end(token);channel.close();channel.close()
        self.assertEqual(budget.size,0)

    def test_concurrent_last_byte_capacity_and_cleanup_do_not_overbook(self):
        budget=MemoryRgbdBudget(10000);channels,meta,token=self.channels(budget)
        self.assertTrue(channels[0].write(deepcopy(meta),b'color',b'depth',sampled_token=token))
        one=budget.size;budget.max_bytes=one*2
        barrier=threading.Barrier(3);accepted=[]
        def write(channel):
            barrier.wait()
            accepted.append(channel.write(deepcopy(meta),b'color',b'depth',sampled_token=token))
        threads=[threading.Thread(target=write,args=(ch,)) for ch in channels[1:]]
        for thread in threads:thread.start()
        barrier.wait()
        for thread in threads:thread.join(2)
        self.assertEqual(sorted(accepted),[False,True])
        self.assertEqual(budget.size,one*2)
        self.assertEqual(budget.peak_size,one*2)
        failed=next(ch for ch in channels if ch.error)
        self.assertIn('shared byte budget',failed.error)
        self.assertIn(str(budget.max_bytes)+' bytes',failed.error)
        channels[0].pop(token)
        self.assertEqual(budget.size,one)
        for ch in channels:
            ch.end(token);ch.end(token);ch.close()
        self.assertEqual(budget.size,0)


class MemoryRecordingTest(unittest.TestCase):
    cleanup=prior.RecordingPreparationTest.cleanup
    step=prior.RecordingPreparationTest.step
    until=prior.RecordingPreparationTest.until
    start=prior.RecordingPreparationTest.start

    def setUp(self):
        producer_class=prior.CameraProducer
        session_class=prior.RecordingSession
        def producer(*args,**kwargs):
            return producer_class(*args,**dict(kwargs,memory_recording=True))
        def session(*args,**kwargs):
            kwargs.pop('camera_directories')
            return session_class(*args,**kwargs,camera_channels=self.producer.channels,storage_format='zip')
        for name,value in (('CameraProducer',producer),('RecordingSession',session)):
            active=patch.object(prior,name,value);active.start();self.addCleanup(active.stop)
        prior.RecordingPreparationTest.setUp(self)

    test_sampled_frames_bracket_execution_and_timeline_binds_once=(
        prior.RecordingPreparationTest.test_actual_sampled_first_frames_precede_command_and_final_timeline_is_bound_once)
    test_old_sample_is_not_relabelled=(
        prior.RecordingPreparationTest.test_preparation_does_not_relabel_an_old_queued_camera_packet)
    test_preparation_failure_and_missing_images_do_not_gate_motion=(
        prior.RecordingPreparationTest.test_factory_failure_and_absent_images_do_not_gate_motion)
    test_handoff_rejects_late_factory=(
        prior.RecordingPreparationTest.test_handoff_revokes_a_timed_out_factory_before_its_late_return)

    def test_prebind_frames_are_consumed_and_preserve_initial_bracket(self):
        runner=self.start()
        entered,release=threading.Event(),threading.Event();self.releases.append(release)
        compile_runtime=runner.compile_runtime
        def delayed(q):
            entered.set()
            if not release.wait(5):raise TimeoutError('CPU initialization fixture not released')
            return compile_runtime(q)
        runner.compile_runtime=delayed
        self.until(entered.is_set)
        session=self.owner.recording
        self.until(lambda:session.recorder is not None and all(len(rows)>20 for rows in session.recorder.frames.values()))
        self.assertFalse(session.execution_bound.is_set())
        self.assertFalse(session.ready.is_set())
        self.assertEqual(runner.last_sent_index,-1)
        self.assertFalse((session.root/'execution-timeline.json').exists())
        first={camera:deepcopy(rows[0]) for camera,rows in session.recorder.frames.items()}
        release.set()
        self.until(lambda:runner.status=='READY')
        self.until(session.done.is_set)
        self.assertIsNone(session.error,session.snapshot())
        self.assertEqual(session.manifest['capture_status'],'complete',session.manifest)
        self.assertEqual(len(self.timeline_writes),1)
        for camera,rows in session.recorder.frames.items():
            self.assertEqual(rows[0],first[camera])
            self.assertLess(rows[0]['source_time_ns'],session.recorder.commands[0]['send_source_time_ns'])
        self.assertEqual(self.producer.common.snapshot()['recording_budget']['queued_bytes'],0)
        prior.verify_raw(session.root,environment='fake')

    def assert_prepared_revoked(self,command):
        runner=self.start()
        self.until(lambda:self.owner.recording is not None and self.owner.recording.prepared.is_set())
        session=self.owner.recording
        leases=[channel.current_lease() for channel in self.producer.channels.values()]
        started=time.monotonic();self.owner.submit(command)
        self.assertLess(time.monotonic()-started,.1)
        self.assertTrue(all(not session.lease_active(lease) for lease in leases))
        self.assertTrue(session.done.wait(3),session.snapshot())
        self.assertTrue(all(channel.token() is None for channel in self.producer.channels.values()))
        self.assertEqual(runner.last_sent_index,-1)
        self.assertFalse(list((self.root/'spool').rglob('*.png'))+list((self.root/'spool').rglob('*.zip')))
        self.assertEqual(len(list((self.root/'spool').rglob('recording-service.json'))),3)

    def test_stop_revokes_prepared_memory(self):
        self.assert_prepared_revoked(dict(action='stop'))

    def test_release_revokes_prepared_memory(self):
        self.assert_prepared_revoked(dict(action='release',confirmation='ARMS_SUPPORTED_RELEASE'))

    def test_pedal_revokes_prepared_memory(self):
        self.assert_prepared_revoked(dict(action='simulate_pedal'))

    def test_late_registration_after_stop_never_admits_a_frame(self):
        channel=self.producer.channels['base_0'];begin=channel.begin
        entered,release,visible,return_begin=(threading.Event() for _ in range(4))
        self.releases.extend((release,return_begin))
        def delayed(lease):
            entered.set()
            if not release.wait(5):raise TimeoutError('CPU fixture begin not released')
            begin(lease);visible.set()
            if not return_begin.wait(5):raise TimeoutError('CPU fixture post-begin not released')
        channel.begin=delayed
        runner=self.start();self.until(entered.is_set)
        started=time.monotonic();self.owner.submit(dict(action='stop'))
        self.assertLess(time.monotonic()-started,.1)
        self.until(lambda:runner.status=='STOPPED')
        release.set();self.assertTrue(visible.wait(2))
        for _ in range(100):self.step()
        self.producer.queue.join()
        self.assertEqual(channel.snapshot()['published_frames'],0)
        return_begin.set()
        self.until(lambda:self.sessions and all(session.done.is_set() for session in self.sessions))
        self.assertTrue(all(source.token() is None for source in self.producer.channels.values()))
        self.assertEqual(runner.last_sent_index,-1)

    def test_timed_out_factory_cannot_replace_same_epoch_new_execution(self):
        entered,release=threading.Event(),threading.Event();self.releases.append(release)
        calls=[]
        def delayed(payload):
            calls.append(payload)
            if len(calls)==1:
                entered.set()
                if not release.wait(5):raise TimeoutError('CPU fixture factory not released')
        self.factory_hook=delayed
        old=self.start();self.until(entered.is_set)
        epoch=self.owner.epoch
        self.until(lambda:old.status=='READY',seconds=4)
        self.assertTrue(old.preparation_closed)
        self.assertIn('exceeded 1 wall second',self.owner.recording_error)
        self.assertIsNone(self.owner.recording)
        new=self.start()
        self.until(lambda:self.owner.recording is not None and self.owner.recording.prepared.is_set())
        current=self.owner.recording
        self.assertEqual(self.owner.epoch,epoch)
        self.assertNotEqual(new.execution_instance_id,old.execution_instance_id)
        release.set();old.preparation_thread.join(3)
        self.until(lambda:len(self.sessions)==2 and all(s is current or s.done.is_set() for s in self.sessions))
        self.assertIs(self.owner.recording,current)
        self.assertIsNone(self.owner.recording_error)
        for channel in self.producer.channels.values():
            self.assertEqual(channel.current_lease()['execution_instance_id'],new.execution_instance_id)

    def test_overflow_preserves_partial_data_and_does_not_stop_or_restart_motion(self):
        runner=self.start()
        self.until(lambda:self.owner.recording is not None and self.owner.recording.ready.is_set())
        session=self.owner.recording
        self.until(lambda:all(session.recorder.frames.values()))
        for channel in self.producer.channels.values():
            channel.fail('injected camera recording memory queue overflow',sampled_token=channel.token())
        self.until(lambda:session.done.is_set())
        self.assertIn('memory queue overflow',session.error)
        self.assertTrue(any(session.recorder.frames.values()))
        self.assertEqual(session.manifest['capture_status'],'incomplete')
        self.assertEqual(session.manifest['validation']['status'],'failed')
        self.assertEqual(session.manifest['controller_outcome'],'unverified')
        self.assertTrue((session.root/'manifest.json').is_file())
        self.assertTrue((session.root/'recording-diagnostics.json').is_file())
        self.until(lambda:runner.status=='READY')
        self.assertGreater(runner.last_sent_index,0)
        commands=len([row for row in self.events if row['kind']=='command'])
        for _ in range(240):self.step()
        self.assertIs(self.owner.runner,runner)
        self.assertEqual(len([row for row in self.events if row['kind']=='command']),commands)
        self.assertTrue(all(channel.token() is None for channel in self.producer.channels.values()))


if __name__=='__main__':unittest.main()
