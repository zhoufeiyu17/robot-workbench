"""Source-reference contracts; optional native DDS check uses no robot or GPU."""
import multiprocessing
import os
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_workbench.ros_data import SampleCache, StreamRef, json_text, json_value, source_descriptor


def descriptor():
    return dict(device_id='cpu_robot', stream_id='head', source_instance_id='source_a',
                source_clock_id='host_wall', topic='/wb_cpu/head', message_type='sensor_msgs/msg/Image',
                kind='image', units='rgb8', coordinate_frame='head', nominal_hz=30.,
                available=True, metadata={})


def native_subscriber(pipe):
    from robot_workbench.ros_data import RosNode
    node = RosNode('subscriber', namespace='/wb_cpu', domain_id=126)
    try:
        def receive(msg):
            pipe.send((StreamRef.from_message(msg.reference).__dict__, list(msg.positions)))
        node.subscribe('state', node.types.RobotState, receive)
        pipe.send('ready')
        pipe.recv()
    finally:
        node.close(); pipe.close()


class SourceContracts(unittest.TestCase):
    def test_image_publisher_transport_default_and_explicit_override(self):
        import xml.etree.ElementTree as ET
        from unittest.mock import patch
        from robot_workbench.ros_data import image_transport_environment
        with patch.dict(os.environ,{'ROS_DOMAIN_ID':'126'},clear=True):
            environment=image_transport_environment()
            self.assertNotIn('FASTRTPS_DEFAULT_PROFILES_FILE',os.environ)
            self.assertEqual(environment['ROS_DOMAIN_ID'],'126')
            profile=Path(environment['FASTRTPS_DEFAULT_PROFILES_FILE'])
            tree=ET.parse(profile);ns={'dds':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
            self.assertEqual(tree.find('.//dds:segment_size',ns).text,'16777216')
            self.assertEqual(tree.find('.//dds:useBuiltinTransports',ns).text,'false')
            self.assertEqual([node.text for node in tree.findall('.//dds:type',ns)],['SHM','UDPv4'])
            with patch.object(Path,'is_file',return_value=False):
                with self.assertRaisesRegex(ValueError,'profile is missing'):image_transport_environment()
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE'):
            with patch.dict(os.environ,{key:'/deployment/custom.xml'},clear=True):
                self.assertEqual(image_transport_environment(),{key:'/deployment/custom.xml'})

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST') == '1', 'native ROS DDS needs isolated Linux')
    def test_timing_keeps_rmw_metadata_and_direct_callbacks(self):
        from robot_workbench.ros_data import RosNode
        from robot_workbench.pipeline_timing import timing
        producer=RosNode('timing_source',namespace='/wb_timing',domain_id=126)
        consumer=RosNode('timing_reader',namespace='/wb_timing',domain_id=126)
        received=[];direct=[]
        try:
            publisher=producer.publisher('state',producer.types.RobotState)
            consumer.subscribe('state',consumer.types.RobotState,lambda message:received.append(message.reference.sequence))
            raw=consumer.node.create_subscription(consumer.types.RobotState,'/wb_timing/state',
                lambda message:direct.append(message.reference.sequence),consumer.qos())
            consumer.subscriptions.append(raw)
            deadline=time.monotonic()+5
            while publisher.get_subscription_count()!=2:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            ref=StreamRef('state','timing_source','host_wall',time.time_ns(),1)
            publisher.publish(producer.types.RobotState(reference=ref.message(),positions=[0.]*14,velocities=[0.]*14))
            while not received or not direct:
                self.assertLess(time.monotonic(),deadline);time.sleep(.01)
            self.assertEqual(received,[1]);self.assertEqual(direct,[1])
            rows=[row for row in timing.snapshot(limit=512)['events']
                  if row.get('consumer')=='timing_reader' and row['stage']=='ros.callback.RobotState']
            self.assertEqual(len(rows),1)
            self.assertGreater(rows[0]['rmw_received_wall_ns'],0)
            self.assertEqual(rows[0]['source_time_ns'],ref.source_time_ns)
            self.assertEqual(rows[0]['outcome'],'ok')
            self.assertFalse(consumer.errors)
        finally:
            consumer.close();producer.close()

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST') == '1', 'native ROS DDS needs the isolated Linux ROS environment')
    def test_closed_nodes_retire_without_cyclic_gc(self):
        import gc,weakref
        from robot_workbench.ros_data import RosNode
        enabled=gc.isenabled();gc.collect();gc.disable();retired=[];retired_handles=[]
        try:
            for index in range(8):
                node=RosNode('retiring_'+str(index),namespace='/wb_retirement',domain_id=127)
                retired_handles.append(node.subscribe('unused',node.types.DataEvent,lambda message:None))
                node.publisher('output',node.types.DataEvent)
                retired.append(weakref.ref(node));node.close();del node
            self.assertEqual(sum(ref()is not None for ref in retired),0,'closed ROS nodes must not need cyclic GC to release their callbacks')
        finally:
            if enabled:gc.enable()
            gc.collect()

    def test_old_message_layout_is_rejected_before_creating_a_ros_context(self):
        from types import SimpleNamespace
        from unittest.mock import Mock,patch
        from robot_workbench.ros_data import RosNode
        runtime=SimpleNamespace(create_node=Mock())
        with patch.dict(sys.modules,{'rclpy':runtime,'workbench_interfaces':SimpleNamespace(msg=SimpleNamespace())}):
            with self.assertRaisesRegex(RuntimeError,'exact workbench_interfaces'):
                RosNode('old_schema',namespace='/wb_cpu',domain_id=126,localhost_only=False)
        runtime.create_node.assert_not_called()

    def test_exact_source_and_sample_no_fallback(self):
        cache = SampleCache(descriptor(), max_samples=2, max_bytes=10)
        ref = lambda n: StreamRef('head', 'source_a', 'host_wall', n, n)
        cache.put(ref(1), b'a', 1); cache.put(ref(2), b'b', 1)
        self.assertFalse(cache.put(ref(2), b'b', 1))
        cache.put(ref(3), b'c', 1)
        self.assertEqual(cache.get(ref(2)), b'b')
        with self.assertRaisesRegex(ValueError, 'evicted'): cache.get(ref(1))
        with self.assertRaises(LookupError): cache.get(ref(4))
        self.assertEqual(cache.bytes, 2)

    def test_same_identity_cannot_change_payload(self):
        cache = SampleCache(descriptor())
        ref = StreamRef('head', 'source_a', 'host_wall', 1, 1)
        cache.put(ref, b'a', 1)
        with self.assertRaisesRegex(ValueError, 'different data'): cache.put(ref, b'b', 1)

    def test_source_replacement_invalidates_instead_of_retargeting(self):
        cache = SampleCache(descriptor())
        with self.assertRaisesRegex(ValueError, 'rebind'):
            cache.put(StreamRef('head', 'source_b', 'host_wall', 1, 1), b'a', 1)
        with self.assertRaises(ValueError): cache.history()

    def test_metadata_and_descriptor_boundaries(self):
        self.assertEqual(json_value(json_text({'a': 1})), {'a': 1})
        for raw in ('{"x":NaN}', '{"x":Infinity}'):
            with self.assertRaises(ValueError): json_value(raw)
        with self.assertRaises(ValueError): json_text({'x': float('nan')})
        with self.assertRaises(ValueError): source_descriptor(dict(descriptor(), topic='http://host'))
        for n in (-1, True, 2**64):
            with self.assertRaises(ValueError): StreamRef('head', 'source_a', 'host_wall', 1, n)

    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST') == '1', 'native ROS DDS needs the isolated Linux ROS environment')
    def test_two_process_native_dds_roundtrip(self):
        from robot_workbench.ros_data import RosNode
        context = multiprocessing.get_context('spawn'); parent, child = context.Pipe()
        worker = context.Process(target=native_subscriber, args=(child,)); worker.start(); child.close()
        node = RosNode('publisher', namespace='/wb_cpu', domain_id=126)
        try:
            self.assertTrue(parent.poll(10)); self.assertEqual(parent.recv(), 'ready')
            publisher = node.publisher('state', node.types.RobotState)
            deadline = time.monotonic()+10
            while publisher.get_subscription_count() != 1 and time.monotonic() < deadline: time.sleep(.02)
            self.assertEqual(publisher.get_subscription_count(), 1)
            ref = StreamRef('state', 'source_a', 'host_monotonic', 123, 7)
            publisher.publish(node.types.RobotState(reference=ref.message(), positions=[.1]*14,
                velocities=[0.]*14, efforts=[], metadata_json='{}'))
            self.assertTrue(parent.poll(5)); actual, positions = parent.recv()
            self.assertEqual(actual, ref.__dict__); self.assertEqual(positions, [.1]*14)
        finally:
            parent.send('close'); worker.join(10); node.close(); parent.close()
            if worker.is_alive(): worker.terminate(); worker.join(5)
        self.assertEqual(worker.exitcode, 0)


if __name__ == '__main__':
    unittest.main()
