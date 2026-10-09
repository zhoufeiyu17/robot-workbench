"""Offline raw-owner normalization; never contact hardware or import its SDK."""
import copy
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'infra'))
from arx_env.joint_head_workbench_hardware import HardwareReadAdapter, HardwareCameraAdapter, camera_process_inventory, normalize_owner_state, _NoRedirect


class HardwareReadTest(unittest.TestCase):
    def test_process_inventory_exact_producers_and_unknown_serial(self):
        with tempfile.TemporaryDirectory() as directory:
            def process(pid,args,state='S'):
                root=Path(directory)/str(pid);root.mkdir()
                (root/'cmdline').write_bytes(b'\0'.join(arg.encode() for arg in args)+b'\0')
                (root/'comm').write_text('python\n')
                (root/'stat').write_text(f'{pid} (python worker) '+state+' '+' '.join(['0']*18+['456']+['0']*5))
            process(10,['python','-m','prometheus.web.preview','--token','never-expose-this'])
            process(11,['python','camera-stream-v2.py','--serial=123'])
            process(12,['python','unrelated.py','123'])
            process(13,['python','camera-stream-v2.py','123'],'Z')
            state=camera_process_inventory(['123','456'],proc_root=directory)
            self.assertEqual([row['pid'] for row in state['processes']],[10,11])
            self.assertTrue(state['cameras']['123']['process_alive'])
            self.assertIsNone(state['cameras']['456']['process_alive'])
            self.assertEqual(state['processes'][0]['start_ticks'],456)
            self.assertNotIn('never-expose',json.dumps(state))
            self.assertEqual(set(state['processes'][0]),{'pid','comm','start_ticks','serial_match'})
        self.assertFalse(camera_process_inventory(['123'],proc_root=directory)['scan_complete'])
        with self.assertRaises(ValueError):camera_process_inventory(['../bad'])

    def setUp(self):
        self.now = 10_000_000_000
        self.owner = dict(pid=123, mode='hardware', state='READY', command_epoch=2,
                          motion_protocol='reviewed-single-arm-v1', connected=True, feedback_healthy=True,
                          calibration_feedback=dict(qpos=list(range(14)), sdk_timestamps=[42., 900.],
                                                    sample_wall_ns=self.now),
                          pedal=dict(input_healthy=True, physically_down=False), reason='', recoverable_fault=None)

    def normalize(self, owner=None, **kwargs):
        return normalize_owner_state(self.owner if owner is None else owner, now_ns=self.now, **kwargs)

    def test_mapping_and_capabilities(self):
        state = self.normalize()
        self.assertEqual(state['q_actual_rad'], list(range(6))+list(range(7, 13)))
        self.assertEqual(state['jaw_positions_m'], dict(left=6, right=13))
        self.assertIsNone(state['feedback']['per_motor_freshness'])
        self.assertEqual(state['feedback']['sdk_advanced'], dict(left=None, right=None))
        self.assertEqual(state['capabilities'], dict(read_state=True, execute_queue=False, stop=False, capture=False))
        self.assertEqual(state['pedal'], self.owner['pedal'])

    def test_reject_invalid_stale_simulated_disconnected_and_unhealthy(self):
        for key, value in [('qpos', [0.]*12), ('qpos', [float('nan')]*14),
                           ('qpos', [True]*14), ('sdk_timestamps', [-1., 2.]),
                           ('sample_wall_ns', self.now-600_000_000), ('sample_wall_ns', self.now+1)]:
            owner = copy.deepcopy(self.owner)
            owner['calibration_feedback'][key] = value
            self.assertIsNone(self.normalize(owner)['q_actual_rad'])
        for key, value in [('mode', 'simulation'), ('connected', False), ('feedback_healthy', False),
                           ('pid', True), ('command_epoch', -1), ('motion_protocol', 'unknown')]:
            owner = dict(self.owner, **{key: value})
            self.assertIsNone(self.normalize(owner)['q_actual_rad'])

    def test_clock_regression_and_changed_identity(self):
        previous = self.normalize()
        self.owner['calibration_feedback']['sdk_timestamps'] = [41., 901.]
        state = self.normalize(previous=previous)
        self.assertIn('left_sdk_clock_regressed', state['feedback']['reasons'])
        self.assertEqual(state['feedback']['sdk_advanced'], dict(left=False, right=True))
        self.owner['calibration_feedback']['sdk_timestamps'] = [43., 901.]
        self.owner['calibration_feedback']['sample_wall_ns'] -= 1
        self.assertIn('host_sample_time_regressed', self.normalize(previous=previous)['feedback']['reasons'])
        for key in ('pid', 'command_epoch'):
            owner = dict(self.owner, **{key: self.owner[key]+1})
            self.assertIn('owner_identity_or_epoch_changed', self.normalize(owner, previous=previous)['feedback']['reasons'])

    def test_remote_age_ignores_workstation_clock_skew(self):
        adapter=HardwareReadAdapter(8120);adapter.http=Mock()
        payload=dict(source='read-only-status-gateway',owner=self.owner,source_age_s=.02)
        adapter.http.open.return_value=io.BytesIO(json.dumps(payload).encode())
        with patch('time.time_ns',return_value=self.now+99_000_000_000), patch('time.monotonic',side_effect=[10.,10.03]):
            result=adapter.read_state()
        self.assertTrue(result['feedback']['valid'])
        self.assertAlmostEqual(result['feedback']['host_sample_age_s'],.05,places=6)
        payload['source_age_s']=1
        adapter.http.open.return_value=io.BytesIO(json.dumps(payload).encode())
        self.assertFalse(adapter.read_state()['feedback']['valid'])

    def test_only_fixed_get_no_redirect_no_stale_fallback(self):
        adapter = HardwareReadAdapter(8098)
        adapter.http = Mock()
        adapter.http.open.return_value = io.BytesIO(json.dumps(self.owner).encode())
        with patch('time.time_ns', return_value=self.now):
            state = adapter.read_state()
            self.assertTrue(state['feedback']['valid'])
            adapter.http.open.assert_called_once_with('http://127.0.0.1:8098/state', timeout=.8)
            adapter.http.open.side_effect = OSError('offline')
            state = adapter.read_state()
            self.assertFalse(state['reachable'])
            self.assertIsNone(state['q_actual_rad'])
            self.assertIsNone(adapter.previous)
        with self.assertRaises(ValueError):
            _NoRedirect().redirect_request(None, None, 302, '', {}, 'http://remote/state')
        for port in (True, 0, 65536, '8098'):
            with self.assertRaises(ValueError): HardwareReadAdapter(port)

    def test_large_bounded_owner_state_preserves_feedback(self):
        adapter = HardwareReadAdapter(8098)
        adapter.http = Mock()
        owner = {**self.owner, 'arrival_events': ['event' * 20000]}
        adapter.http.open.return_value = io.BytesIO(json.dumps(owner).encode())
        with patch('time.time_ns', return_value=self.now):
            self.assertTrue(adapter.read_state()['feedback']['valid'])
        adapter.http.open.return_value = io.BytesIO(b'x' * (4*1024*1024+1))
        self.assertIn('too large', adapter.read_state()['error'])


class CameraReadTest(unittest.TestCase):
    def test_rgbd_atomic_serial_binds_intrinsics_without_overwriting_mismatch(self):
        for intrinsics, expected in [({'fx':400},'camera-1'),({'serial':'other','fx':400},'other')]:
            meta=dict(self.status,raw=True,annotated=False,image_domain='raw',serial='camera-1',intrinsics=intrinsics)
            response=io.BytesIO(self.jpeg);response.headers={'X-Camera-Metadata':json.dumps(meta)}
            self.adapter.http=Mock();self.adapter.http.open.return_value=response
            _,status=self.adapter.read_calibration_frame()
            self.assertEqual(status['intrinsics']['serial'],expected)
            self.assertEqual(status['intrinsics']['fx'],400)

    def test_calibration_uses_atomic_raw_route_without_preview_fallback(self):
        meta=dict(self.status,raw=True,annotated=False,image_domain='raw',serial='test',intrinsics={'serial':'test'})
        response=io.BytesIO(self.jpeg);response.headers={'X-Camera-Metadata':json.dumps(meta)}
        self.adapter.http=Mock();self.adapter.http.open.return_value=response
        raw,status=self.adapter.read_calibration_frame()
        self.assertEqual(raw,self.jpeg);self.assertTrue(status['metadata_matched'])
        self.assertEqual(status['intrinsics'],{'serial':'test'})
        self.adapter.http.open.assert_called_once_with('http://127.0.0.1:8114/raw-frame.jpg',timeout=.8)
        response=io.BytesIO(self.jpeg);response.headers={'X-Camera-Metadata':json.dumps(self.status)}
        self.adapter.http.open.return_value=response
        with self.assertRaisesRegex(ValueError,'unannotated'):self.adapter.read_calibration_frame()

    def setUp(self):
        self.adapter = HardwareCameraAdapter(8114)
        self.status = dict(ready=True, host_receive_ns='10000000000', frame_number=24,
                           device_timestamp_ms=12.5, timestamp_domain='hardware_clock')
        self.jpeg = b'\xff\xd8offline-test\xff\xd9'
        self.clock = patch('time.time_ns', return_value=10_100_000_000)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def responses(self, *values):
        self.adapter.http = Mock()
        responses = [io.BytesIO(self.jpeg)] + [io.BytesIO(json.dumps(v).encode() if isinstance(v, dict) else v) for v in values]
        for response in responses: response.headers = {}
        self.adapter.http.open.side_effect = responses

    def test_independent_poll_retains_last_failure_after_recovery(self):
        self.adapter.http=Mock()
        self.adapter.http.open.side_effect=ConnectionRefusedError('camera endpoint refused')
        failed=self.adapter.poll_state()
        self.assertFalse(failed['frame_readable'])
        self.assertIsNone(failed['process_alive'])
        self.assertIn('ConnectionRefusedError',failed['last_failure']['reason'])
        self.assertEqual(failed['last_failure']['at_ns'],10_100_000_000)
        self.responses(self.status,self.jpeg,self.status)
        recovered=self.adapter.poll_state()
        self.assertTrue(recovered['frame_readable'])
        self.assertEqual(recovered['last_failure'],failed['last_failure'])
        self.assertEqual(recovered['last_success_ns'],10_100_000_000)
        with patch('time.time_ns',return_value=13_000_000_000):
            stale=self.adapter.snapshot()
        self.assertFalse(stale['frame_readable'])
        self.assertIn('stale',stale['last_failure']['reason'])
        self.assertEqual(stale['last_failure']['at_ns'],13_000_000_000)

    def test_monitor_only_reads_existing_endpoint_and_honors_stop(self):
        stop=Mock();stop.is_set.return_value=False;stop.wait.return_value=True
        self.responses(self.status,self.jpeg,self.status)
        self.adapter.monitor(stop,.1)
        stop.wait.assert_called_once_with(.1)
        self.assertTrue(self.adapter.snapshot()['frame_readable'])
        self.assertTrue(all(call.args[0].startswith('http://127.0.0.1:8114/') for call in self.adapter.http.open.call_args_list))
        stop.is_set.return_value=True
        with patch.object(self.adapter,'poll_state') as poll:
            self.adapter.monitor(stop)
            poll.assert_not_called()
        with self.assertRaises(ValueError):self.adapter.monitor(stop,0)

    def test_coherent_frame_fixed_routes_no_joint_claim(self):
        self.responses(self.status, self.jpeg, self.status)
        jpeg, state = self.adapter.read_frame()
        self.assertEqual(jpeg, self.jpeg)
        self.assertEqual(state['frame_seq'], 24)
        self.assertEqual(state['frame_timestamp_ns'], 10_000_000_000)
        self.assertTrue(state['metadata_matched'])
        self.assertIsNone(state['q_actual_rad'])
        self.assertFalse(state['synchronized'])
        self.assertEqual([c.args[0] for c in self.adapter.http.open.call_args_list],
                         ['http://127.0.0.1:8114/frame.jpg', 'http://127.0.0.1:8114/status', 'http://127.0.0.1:8114/frame.jpg', 'http://127.0.0.1:8114/status'])
        with patch('time.time_ns', return_value=13_000_000_000):
            self.assertFalse(self.adapter.snapshot()['ready'])

    def test_race_retry_then_reject_no_cached_image(self):
        newer = dict(self.status, frame_number=25)
        self.responses(self.status, self.jpeg, newer, newer, self.jpeg, newer)
        self.assertEqual(self.adapter.read_frame()[1]['frame_seq'], 25)
        self.responses(self.status, self.jpeg, newer, self.status, self.jpeg, newer)
        with self.assertRaisesRegex(ValueError, 'changed'):
            self.adapter.read_frame()
        self.assertFalse(self.adapter.snapshot()['ready'])
        self.assertIsNone(self.adapter.snapshot()['frame_seq'])

    def test_stale_missing_timestamp_invalid_image_and_timeout(self):
        for value in ('7000000000', '11000000000'):
            status = dict(self.status, host_receive_ns=value)
            self.responses(status, self.jpeg, status)
            with self.assertRaisesRegex(ValueError, 'stale'):
                self.adapter.read_frame()
        status = dict(self.status)
        del status['host_receive_ns']
        self.responses(status, self.jpeg, status)
        state = self.adapter.read_frame()[1]
        self.assertIsNone(state['age_s'])
        self.assertIsNone(state['frame_timestamp_ns'])
        # received_monotonic marks a camera that computed its own age. A camera that only
        # reports host_receive_ns keeps it None and is aged against the wall clock by
        # snapshot(); consumers must read whichever basis exists rather than subtract None.
        self.responses(self.status, self.jpeg, self.status)
        state = self.adapter.read_frame()[1]
        self.assertIsNotNone(state['source_age_s'])
        self.assertIsNone(state['received_monotonic'])
        self.responses(self.status, b'<html>error</html>', self.status)
        with self.assertRaisesRegex(ValueError, 'JPEG'):
            self.adapter.read_frame()
        self.adapter.http.open.side_effect = TimeoutError('timeout')
        with self.assertRaises(TimeoutError): self.adapter.read_frame()
        self.assertFalse(self.adapter.snapshot()['reachable'])

    def test_atomic_frame_and_stream_use_source_age_not_skewed_wall_clock(self):
        status = dict(self.status, source_age_s=.01, host_receive_ns='99999999999999', delivered_fps=30)
        response = io.BytesIO(self.jpeg)
        response.headers = {'X-Camera-Metadata': json.dumps(status)}
        self.adapter.http = Mock()
        self.adapter.http.open.return_value = response
        with patch('time.monotonic', return_value=5.):
            self.assertTrue(self.adapter.read_frame()[1]['ready'])
        self.assertEqual(self.adapter.http.open.call_count, 1)
        with patch('time.monotonic', return_value=8.):
            self.assertFalse(self.adapter.snapshot()['ready'])
        parts = []
        for seq in (24, 25):
            meta = dict(status, frame_number=seq)
            parts.append(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(self.jpeg)).encode()
                         +b'\r\nX-Camera-Metadata: '+json.dumps(meta).encode()+b'\r\n\r\n'+self.jpeg+b'\r\n')
        response = io.BytesIO(b''.join(parts))
        response.headers = {'Content-Type': 'multipart/x-mixed-replace; boundary=frame'}
        self.adapter.http.open.return_value = response
        frames = self.adapter.iter_frames()
        self.assertEqual(next(frames)[1]['frame_seq'], 24)
        self.assertEqual(next(frames)[1]['frame_seq'], 25)
        with self.assertRaisesRegex(ValueError, 'ended'):
            next(frames)
        self.assertFalse(self.adapter.snapshot()['ready'])

    def test_constructor_and_bounded_transport(self):
        for port in (True, 0, 65536, '8114'):
            with self.assertRaises(ValueError): HardwareCameraAdapter(port)
        self.responses(b'x'*65537)
        with self.assertRaisesRegex(ValueError, 'large'):
            self.adapter.read_frame()


if __name__ == '__main__':
    unittest.main()
