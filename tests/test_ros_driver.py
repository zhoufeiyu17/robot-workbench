"""Unique state publication, source lifecycle and bounded control-side enqueue."""
from pathlib import Path
import os
import sys
import time
import threading
import unittest
from tempfile import TemporaryDirectory

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from robot_workbench.ros_data import RosNode,SourceDirectory
from robot_workbench.ros_driver import RosDriverPublisher
from robot_workbench.ros_inputs import RosInputs


@unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','native ROS DDS needs isolated Linux')
class DriverTests(unittest.TestCase):
    def test_preparation_burst_survives_busy_subscriber_with_bounded_history(self):
        producer=RosNode('burst_driver',namespace='/wb_burst',domain_id=126)
        consumer=RosNode('burst_recorder',namespace='/wb_burst',domain_id=126)
        publisher=RosDriverPublisher(producer,device_id='cpu_robot',instance_id='owner',clock_id='host_monotonic',
            state_topic='/wb_burst/state',event_topic='/wb_burst/events')
        entered=threading.Event();release=threading.Event();received=[]
        def record(message):
            received.append(message.reference.sequence)
            if len(received)==1:entered.set();release.wait(3)
        depth=publisher.queue.max_bytes//(512*1024)
        consumer.subscribe('/wb_burst/events',consumer.types.DataEvent,record,depth=depth)
        try:
            deadline=time.monotonic()+5
            while publisher.events.get_subscription_count()!=1:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            publisher.event('feedback',{},run_id='run');publisher.flush()
            self.assertTrue(entered.wait(2))
            for i in range(95):publisher.event('feedback',{'point':i},run_id='run')
            publisher.flush();release.set()
            deadline=time.monotonic()+5
            while len(received)<96 and time.monotonic()<deadline:time.sleep(.01)
            self.assertEqual(received,list(range(1,97)))
            self.assertEqual(publisher.events.qos_profile.depth,128)
            self.assertEqual(publisher.dropped,0)
        finally:
            release.set();publisher.close();consumer.close();producer.close()

    def test_actual_runtime_publishes_original_clock_sample_and_admission_epoch(self):
        from robot_workbench.runtime import Runtime
        from test_owner_clock import calibration,Safety,ClockBackend
        sim=[10.]
        with TemporaryDirectory()as directory:
            runtime=Runtime(calibration,Safety,lambda:ClockBackend(lambda:sim[0],source_clock_id='cpu_clock'),
                device_id='cpu_robot',environment='fake',lock_root=directory,control_clock=lambda:sim[0],
                data_plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_runtime',domain_id=126))
            consumer=RosNode('runtime_subscriber',namespace='/wb_runtime',domain_id=126)
            try:
                runtime.owner.backend.read_calibration_state();runtime.ros_publisher.flush()
                rows=SourceDirectory(consumer,'cpu_robot').resolve({'state':{}},timeout=5)
                inputs=RosInputs(consumer,rows)
                deadline=time.monotonic()+5
                while runtime.ros_publisher.state.get_subscription_count()!=1 and time.monotonic()<deadline:time.sleep(.01)
                old_epoch=runtime.owner.epoch
                backend=runtime.owner.backend._backend
                # A Stop concurrent with a read cannot relabel the old sample.
                def revoke_during_read():runtime.owner.epoch=old_epoch+1
                backend.read_delay=revoke_during_read;sim[0]+=.01
                raw=runtime.owner.backend.read_calibration_state();runtime.ros_publisher.flush()
                deadline=time.monotonic()+5
                while not inputs.caches['state'].samples and time.monotonic()<deadline:time.sleep(.01)
                ref,saved=inputs.latest('state')
                self.assertEqual(ref.source_clock_id,'cpu_clock')
                self.assertEqual(ref.source_time_ns,round(raw['sample_time_s']*1e9))
                self.assertEqual(saved['qpos'],raw['qpos']);self.assertEqual(saved['physics_step'],raw['physics_step'])
                self.assertEqual(saved['command_epoch'],old_epoch)
                self.assertEqual(saved['runtime_nonce'],runtime.nonce)
                self.assertIsNone(runtime.control_inputs(include_feedback=False)['calibration_feedback'])
            finally:
                runtime.owner.backend.is_connected=False;runtime.close();consumer.close()

    def test_state_source_is_unique_and_release_invalidates_consumers(self):
        producer=RosNode('driver',namespace='/wb_driver',domain_id=126)
        consumer=RosNode('consumer',namespace='/wb_driver',domain_id=126)
        publisher=RosDriverPublisher(producer,device_id='cpu_robot',instance_id='owner',clock_id='host_monotonic',
            state_topic='/wb_driver/state',event_topic='/wb_driver/events')
        try:
            directory=SourceDirectory(consumer,'cpu_robot')
            with self.assertRaises(TimeoutError):directory.resolve({'state':{}},timeout=.1)
            sample=dict(qpos=[.1]*14,qvel=[0.]*14,sample_time_s=time.monotonic(),sample_wall_ns=time.time_ns())
            self.assertTrue(publisher.publish_state(sample));publisher.flush()
            self.assertTrue(publisher.publish_state(sample),'duplicate read is not a second sample')
            self.assertEqual(publisher.sequences['state'],1)
            # Integer simulation stamps are authoritative even when converting
            # the matching float seconds back to ns would round upward.
            sample.update(sample_time_s=1/240,source_time_ns=4_166_666,source_clock_id='host_monotonic')
            publisher.publish_state(sample);publisher.flush()
            self.assertEqual(publisher.reference('state')['source_time_ns'],4_166_666)
            sources=directory.resolve({'state':{'topic':'/wb_driver/state'}},timeout=5)
            inputs=RosInputs(consumer,sources)
            deadline=time.monotonic()+5
            while publisher.state.get_subscription_count()!=1 and time.monotonic()<deadline:time.sleep(.01)
            sample['sample_time_s']=time.monotonic();sample.pop('source_time_ns');sample.pop('source_clock_id')
            publisher.publish_state(sample);publisher.flush()
            deadline=time.monotonic()+5
            while not inputs.caches['state'].samples and time.monotonic()<deadline:time.sleep(.01)
            ref,value=inputs.latest('state');self.assertEqual(ref.sequence,3);self.assertEqual(value['qpos'],[.1]*14)
            self.assertEqual(producer.node.count_publishers('/wb_driver/state'),1)
            publisher.close()
            deadline=time.monotonic()+5
            while not inputs.caches['state'].error and time.monotonic()<deadline:time.sleep(.01)
            with self.assertRaisesRegex(ValueError,'rebind'):inputs.latest('state')
        finally:
            publisher.close();consumer.close();producer.close()


if __name__=='__main__':unittest.main()
