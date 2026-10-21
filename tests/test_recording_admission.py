"""Stop recording admission without clearing queued source pairs or stopping cameras."""
from pathlib import Path
import sys
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'tests'),str(ROOT/'rule_based_workbench')]
from test_rgbd_service import SharedCameraTest, CAMERAS


class RecordingAdmissionTests(unittest.TestCase):
    def test_both_clocks_keep_bracketing_frame_and_queue_but_stop_later_admission(self):
        for environment in ('fake','physical'):
            with self.subTest(environment=environment):
                fixture=SharedCameraTest();self.addCleanup(fixture.doCleanups)
                service=fixture.service(environment);token=fixture.lease(service)
                epoch=time.time_ns();original=fixture.metadata
                def metadata(camera,index):
                    value=original(camera,index);stamp=epoch+index*33_333_333
                    value['host_receive_ns']=str(stamp) if environment=='physical' else stamp
                    return value
                fixture.metadata=metadata
                endpoint=100_000_000_000+2*1_000_000_000//30 if environment=='fake' else epoch+2*33_333_333
                for camera,channel in service.channels.items():
                    self.assertTrue(fixture.publish(service,camera,0,token)['recorded'])
                    self.assertFalse(channel.stop_after(token,endpoint)['admission_closed'])
                    self.assertTrue(fixture.publish(service,camera,1,token)['recorded'])
                    self.assertTrue(fixture.publish(service,camera,2,token)['recorded'])
                    before=channel.snapshot()
                    for index in range(3,8):self.assertFalse(fixture.publish(service,camera,index,token)['recorded'])
                    after=channel.snapshot()
                    self.assertTrue(after['admission_closed']);self.assertIsNone(after['error'])
                    self.assertEqual(after['queued_frames'],3);self.assertEqual(after['queued_bytes'],before['queued_bytes'])
                    self.assertEqual(after['outside_endpoint_frames'],5)
                    self.assertEqual([channel.pop(token)['metadata']['frame_number'] for _ in range(3)],[0,1,2])
                    self.assertIsNone(channel.peek(token))
                    latest=service.frames[camera].snapshot(formats=[])['status']
                    self.assertTrue(latest['data_valid']);self.assertFalse(latest['recording_enabled'])
                    self.assertEqual(latest['frame_number'],7,'camera keeps publishing preview/inference after recording cutoff')
                    with self.assertRaisesRegex(ValueError,'cannot change'):channel.stop_after(token,endpoint+1)
                    channel.end(token)
                self.assertEqual(service.snapshot()['recording_budget']['queued_bytes'],0)

    def test_existing_backlog_is_preserved_and_cutoff_cannot_extend_to_next_lease(self):
        fixture=SharedCameraTest();self.addCleanup(fixture.doCleanups)
        service=fixture.service();token=fixture.lease(service);channel=service.channels['base_0']
        for index in range(5):fixture.publish(service,'base_0',index,token)
        before=channel.snapshot();endpoint=100_000_000_000+2*1_000_000_000//30
        self.assertTrue(channel.stop_after(token,endpoint)['admission_closed'])
        self.assertEqual(channel.snapshot()['queued_bytes'],before['queued_bytes'])
        self.assertEqual([channel.pop(token)['metadata']['frame_number'] for _ in range(5)],list(range(5)))
        for item in service.channels.values():item.end(token)
        next_token=fixture.lease(service,'b'*32)
        self.assertFalse(channel.snapshot()['admission_closed'])
        with self.assertRaisesRegex(ValueError,'another execution'):channel.stop_after(token,endpoint)
        self.assertTrue(fixture.publish(service,'base_0',5,next_token)['recorded'])
        for item in service.channels.values():item.end(next_token)


if __name__=='__main__':unittest.main()
