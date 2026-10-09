import json
import http.client
import socket
import tempfile
import time
import unittest
from pathlib import Path
import numpy as np

from examples.scripts.debug.joint_head_workbench_live import Session, serve, overview_camera, validate_calibration, validate_overview_view


class FakeGeometry:
    velocity_limits = np.ones(12)
    limits = np.column_stack((-np.ones(12), np.ones(12)))
    k = {'width': 32, 'height': 24, 'fx': 20., 'fy': 20., 'cx': 16., 'cy': 12.,
         'rectified': False, 'coefficients': [.1, .2, 0, 0, 0]}
    camera = np.eye(4)
    marker_size = .015
    mounts = {'left': np.eye(4), 'right': np.eye(4)}

    def path_check(self, start, target):
        return {'ok': True}


class FakeRenderer:
    def render(self, q, gaps=None):
        return b'jpeg', b'head'

    def capture(self, image, q, waypoint, output, index):
        name = f'frame-{index:03d}'
        (output / (name + '.png')).write_bytes(b'png')
        row = {'id': name, 'image': name + '.png', 'q_reported_rad': q.tolist(),
               'qpos': q.tolist(), 'split': waypoint['split'], 'valid_for_fit': True,
               'detections': {}}
        (output / (name + '.json')).write_text(json.dumps(row))
        return row


class LiveProtocol(unittest.TestCase):
    def test_execute_accepts_transition_without_arm_and_normalizes_defaults(self):
        s = self.session()
        s.status = 'READY'
        s.command('/execute', {'expected_start':[0.]*12, 'expected_jaw_gap_mm':[83.,83.],
                               'waypoints':[{'q':[0.]*12, 'speeds':[.2]*12, 'capture':False}]})
        self.assertEqual(s.status, 'VALIDATING')
        self.assertIsNone(s.pending[0]['arm'])
        self.assertEqual(s.pending[0]['split'], 'fit')

    def test_overlay_key_switches_release_old_gpu_owners(self):
        import base64, cv2, gc, sys, weakref
        from types import SimpleNamespace
        from unittest.mock import patch
        from arx_env.joint_head_workbench_geometry import Geometry
        from examples.scripts.debug.joint_head_workbench_live import real_overlay_request
        geometry=Geometry.__new__(Geometry)
        geometry.mounts={side:np.eye(4) for side in ('left','right')}
        geometry.mount_chain={};geometry.side_meshes={'left':[],'right':[]}
        geometry.bounds={};geometry.hulls={};geometry.config={}
        refs=[]
        class Overlay:
            def __init__(self,view):
                self.geometry=view;refs.append(weakref.ref(self))
            def render(self,image,*args):return image
        image=np.zeros((24,32,3),np.uint8)
        _,jpeg=cv2.imencode('.jpg',image)
        body=dict(jpeg_base64=base64.b64encode(jpeg).decode(),qpos=[0.]*14,
                  camera=FakeGeometry.k,candidate=dict(id='test',camera_T_world_optical=np.eye(4).tolist(),
                  joint_offsets_rad={'left':[0.]*6,'right':[0.]*6}))
        with patch.dict(sys.modules,{'arx_env.joint_head_gpu_overlay':SimpleNamespace(GPUOverlay=Overlay)}):
            real_overlay_request(geometry,body)
            for i in range(50):
                marker=np.eye(4);marker[0,3]=.01 if i%2 else .02
                body['candidate']['T_link6_marker']={'24':marker.tolist(),'30':np.eye(4).tolist()}
                real_overlay_request(geometry,body)
        gc.collect()
        self.assertEqual(len(refs),51)
        self.assertEqual(sum(ref() is not None for ref in refs),2,'Only head and latest wrist overlay may remain')
        self.assertNotIn('_wrist_gpu_overlay_cache',geometry._wrist_gpu_overlay_cache[1].geometry.__dict__)
        self.assertNotIn('_real_gpu_overlay',geometry._wrist_gpu_overlay_cache[1].geometry.__dict__)
        del geometry
        gc.collect()
        self.assertTrue(all(ref() is None for ref in refs))

    def test_real_overlay_uses_uploaded_pixels_and_feedback(self):
        import base64, cv2
        from unittest.mock import patch
        from examples.scripts.debug.joint_head_workbench_live import real_overlay_request
        image = np.full((24,32,3),87,np.uint8)
        _, jpeg = cv2.imencode('.jpg', image)
        body = dict(jpeg_base64=base64.b64encode(jpeg).decode(), qpos=list(np.arange(14)/100),
                    camera=FakeGeometry.k, candidate=dict(id='real', camera_T_world_optical=np.eye(4).tolist(),
                    joint_offsets_rad=dict(left=[0]*6,right=[0]*6)))
        from types import SimpleNamespace
        import sys
        from unittest.mock import Mock
        render=Mock(return_value=image)
        module=SimpleNamespace(GPUOverlay=Mock(return_value=SimpleNamespace(render=render)))
        with patch.dict(sys.modules, {'arx_env.joint_head_gpu_overlay':module}):
            result = real_overlay_request(FakeGeometry(),body)
            np.testing.assert_array_equal(render.call_args.args[0], image)
            self.assertEqual(render.call_args.args[1], body['qpos'])
            self.assertTrue(result.startswith(b'\xff\xd8'))
        body['jpeg_base64']='invalid!'
        with self.assertRaises(ValueError):real_overlay_request(FakeGeometry(),body)

    def test_unlimited_service_and_execution_heartbeat(self):
        from unittest.mock import Mock, patch
        from examples.scripts.debug import joint_head_workbench_live as live
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'config.json').write_text('{}')
            (root/'token').write_text('x'*32)
            app = Mock()
            app.is_running.side_effect = [True, False]
            session = Mock(client_seen=0)
            session.tick.return_value = False
            argv = ['live', '--config', str(root/'config.json'), '--output', str(root/'out'),
                    '--token-file', str(root/'token'), '--render-gpu', '1']
            simulation_app = Mock(return_value=app)
            with patch('sys.argv', argv), patch.dict('sys.modules', {'isaacsim': Mock(SimulationApp=simulation_app)}), \
                 patch('arx_env.joint_head_workbench_geometry.Geometry'), \
                 patch.object(live, 'Session', return_value=session), patch.object(live, 'RTXRenderer'), \
                 patch.object(live, 'serve'), patch.object(live.time, 'monotonic', side_effect=[0, 10000, 10000, 10000, 10000]), \
                 patch.object(live.time, 'sleep'):
                live.main()
            self.assertEqual(simulation_app.call_args.args[0]['active_gpu'], 1)
            app.update.assert_called_once()  # Still runs after 10,000s without a client.
            with patch('sys.argv', argv[:-1]+['-1']), self.assertRaisesRegex(ValueError, 'render GPU'):
                live.main()
        s = self.session()
        s.status = 'EXECUTING'
        s.client_seen = time.monotonic()-11
        s.tick(FakeRenderer(), .1)
        self.assertEqual(s.status, 'STOPPED')
        self.assertIn('heartbeat', s.reason)

    def session(self):
        return Session(FakeGeometry(), tempfile.mkdtemp(), {'initial_q': [0.] * 12,'mesh_rendering_enabled':True})

    def test_mirror_cancels_queue_rejects_stale_and_expires_without_resume(self):
        from unittest.mock import patch
        s = self.session()
        s.status = 'READY'
        s.capture_id = 'mirror-cancel'
        s.pending = [{'q': np.ones(12)*.1, 'speeds': np.ones(12), 'capture': False,
                      'split': 'fit', 'arm': None}]
        s.tick(FakeRenderer(), 0)
        feedback = {'source_id': 'owner-1', 'sequence': 1, 'source_age_s': .02, 'q': [.3]*12}
        s.command('/mirror', feedback)
        self.assertIsNone(s.active)
        self.assertIsNone(s.pending)
        self.assertTrue(s.dataset['stopped'])
        np.testing.assert_array_equal(s.q, [.3]*12)
        with self.assertRaises(ValueError):
            s.command('/execute', {'expected_start': [.3]*12, 'waypoints': [{'q': [0.]*12}]})
        with self.assertRaises(ValueError):
            s.command('/mirror', feedback)
        with self.assertRaises(ValueError):
            s.command('/mirror', {**feedback, 'sequence': 2, 'source_age_s': 1.1})
        s.command('/preview', {'q': [.8]*12, 'revision': 8})
        s.tick(FakeRenderer(), 0)
        np.testing.assert_array_equal(s.q, [.3]*12)
        with patch('examples.scripts.debug.joint_head_workbench_live.time.monotonic', return_value=s.mirror['received_at']+2.1):
            self.assertEqual(s.state()['state'], 'READY')
        self.assertIsNone(s.mirror)
        s.tick(FakeRenderer(), .1)
        self.assertIsNone(s.active)
        np.testing.assert_array_equal(s.q, [.3]*12)

    def test_mirror_arriving_during_render_keeps_new_feedback(self):
        s = self.session()
        class Renderer(FakeRenderer):
            def render(self, q, gaps=None):
                s.command('/mirror', {'source_id': 'owner', 'sequence': 1, 'source_age_s': 0., 'q': [.5]*12})
                return super().render(q, gaps)
        s.tick(Renderer(), 0)
        np.testing.assert_array_equal(s.q, [.5]*12)
        np.testing.assert_array_equal(s.actual_q, [0.]*12)
        self.assertEqual(s.status, 'MIRRORING')

    def test_actual_and_target_preview_use_one_batch_without_state_cross_talk(self):
        s = self.session()
        class Renderer(FakeRenderer):
            def render_pair(self, q, gaps, target, target_gaps):
                np.testing.assert_array_equal(q, [0.]*12)
                np.testing.assert_array_equal(target, [.6]*12)
                return (b'actual', b'head'), (b'preview', b'target-head')
            def render(self, *args, **kwargs):
                raise AssertionError('batch should render both views')
        s.command('/preview', {'q': [.6]*12, 'revision': 10})
        s.tick(Renderer(), 0)
        self.assertEqual(s.jpeg['actual'], b'actual')
        self.assertEqual(s.jpeg['preview'], b'preview')
        self.assertEqual(s.actual_head, b'head')
        np.testing.assert_array_equal(s.q, [0.]*12)
        self.assertEqual(s.preview_revision, 10)

    def test_jaw_preview_and_execution(self):
        s=self.session();s.status='READY'
        with self.assertRaises(ValueError):s.command('/preview',{'q':[0.]*12,'jaw_gap_mm':[84,0]})
        s.command('/preview',{'q':[0.]*12,'jaw_gap_mm':[0,40],'revision':2})
        class Renderer(FakeRenderer):
            def render(self,q,gaps=None):
                self.gaps=np.array(gaps)
                return super().render(q,gaps)
        r=Renderer();s.tick(r,0)
        np.testing.assert_array_equal(r.gaps,[0,40])
        np.testing.assert_array_equal(s.jaw_mm,[83,83])
        s.command('/execute',{'expected_start':[0.]*12,'expected_jaw_gap_mm':[83,83],
            'waypoints':[{'q':[0.]*12,'jaw_gap_mm':[81,82],'capture':False}]})
        deadline=time.monotonic()+2
        while s.validation and time.monotonic()<deadline:time.sleep(.001)
        s.tick(r,.1)
        self.assertTrue(np.all(s.jaw_mm>[81,82]), 'must not teleport jaws')
        for _ in range(10):s.tick(r,.1)
        np.testing.assert_array_equal(s.jaw_mm,[81,82])
        self.assertEqual(s.status,'READY')

    def test_latest_preview_and_stop_epoch(self):
        s = self.session()
        s.command('/preview', {'q': [.1] * 12, 'revision': 1})
        s.command('/preview', {'q': [.2] * 12, 'revision': 0})
        self.assertEqual(s.preview[1], 1)
        state = s.command('/stop', {})
        self.assertEqual(state['state'], 'STOPPED')
        s.active = {'sentinel': True}
        with self.assertRaises(ValueError):
            s.command('/ack', {})
        s.active = None
        self.assertEqual(s.command('/ack', {})['state'], 'READY')

    def test_execute_validates_and_captures(self):
        s = self.session()
        s.status = 'READY'
        state = s.command('/execute', {'expected_start': [0.] * 12,
            'waypoints': [{'q': [.05] * 12, 'speeds': [.5] * 12,
                           'capture': True, 'split': 'holdout'}]})
        self.assertEqual(state['state'], 'VALIDATING')
        deadline = time.monotonic()+2
        while s.validation and time.monotonic() < deadline:
            time.sleep(.001)
        self.assertIsNotNone(s.pending)
        r = FakeRenderer()
        s.tick(r, 0)
        for _ in range(8):
            s.tick(r, .1)
        self.assertEqual(s.status, 'READY')
        self.assertTrue(s.dataset['capture_id'].startswith('live-'))
        self.assertEqual(s.dataset['frames'][0]['split'], 'holdout')
        self.assertEqual(s.dataset['planned_frames'], [{'id': 'frame-000', 'split': 'holdout'}])
        self.assertEqual(s.dataset['camera'], json.loads((s.batch_dir/'camera.json').read_text()))
        self.assertTrue(s.dataset['camera']['rectified'])
        self.assertEqual(s.dataset['camera']['coefficients'], [0.]*5)

    def test_stop_during_render_never_captures_or_resumes(self):
        s = self.session()
        s.status = 'READY'
        row = {'q': np.zeros(12), 'speeds': np.ones(12), 'capture': True,
               'split': 'fit', 'arm': None}
        s.capture_id = 'live-test'
        s.pending = [row]
        class StopRenderer(FakeRenderer):
            def render(self, q, gaps=None):
                s.command('/stop', {})
                return super().render(q,gaps)
        s.tick(StopRenderer(), .1)
        s.tick(FakeRenderer(), .1)
        self.assertEqual(s.status, 'STOPPED')
        self.assertEqual(s.rows, [])
        self.assertTrue(s.dataset['stopped'])
        self.assertIsNone(s.active)
        s.command('/ack', {})
        s.tick(FakeRenderer(), .1)
        self.assertEqual(s.status, 'READY')
        self.assertEqual(s.rows, [])

    def test_overview_fits_all_initial_mesh_vertices(self):
        class Geometry(FakeGeometry):
            config = {}
            side_meshes = {'left': [('link', np.array([[x,y,z] for x in (-.3,.3)
                for y in (-.6,.6) for z in (0,.8)]), None)]}
            def frames(self, q):
                return {'left': {'link': np.eye(4)}}
        g = Geometry()
        camera = overview_camera(g)
        T = camera['T_world_optical']
        points = (g.side_meshes['left'][0][1]-T[:3,3])@T[:3,:3]
        self.assertTrue(np.all(points[:,2] > 0))
        uv = points[:,:2]/points[:,2,None]*640
        self.assertTrue(np.all(abs(uv) <= np.array([480,360])*.800001))
        self.assertGreater(np.max(abs(uv)/[480,360]), .79)

    def test_calibration_pairing_and_clear(self):
        s = self.session()
        candidate = {'id': 'fit-a', 'camera_T_world_optical': np.eye(4).tolist(),
                     'joint_offsets_rad': {'left': [.01]*6, 'right': [0.]*6}}
        class Renderer(FakeRenderer):
            calls = []
            def encode_head(self, head):
                return b'original'
            def calibration_overlay(self, head, q, value):
                self.calls.append((head, q.copy(), value['id']))
                return b'overlay'
        r = Renderer()
        s.tick(r, 0)
        q_before = s.q.copy()
        self.assertFalse(s.command('/calibration', candidate)['calibration']['ready'])
        s.tick(r, 0)
        while s.calibration_busy:
            time.sleep(.001)
        self.assertEqual(r.calls[0][0], b'head')
        np.testing.assert_array_equal(r.calls[0][1], q_before)
        np.testing.assert_array_equal(s.q, q_before)
        self.assertEqual(s.calibration_state['frame_sequence'], s.frame_sequence)
        self.assertEqual(s.calibration_state['timestamp_s'], s.frame_timestamp['actual'])
        self.assertEqual(s.jpeg['calibration-original'], b'original')
        candidate['id'] = 'fit-b'
        s.command('/calibration', candidate)
        self.assertNotIn('calibration', s.jpeg)
        s.tick(r, 0)
        while s.calibration_busy:
            time.sleep(.001)
        self.assertEqual(s.calibration_state['id'], 'fit-b')
        s.command('/calibration', {'clear': True})
        self.assertNotIn('calibration', s.jpeg)
        s.tick(r, 0)
        self.assertFalse(s.calibration_state['ready'])
        candidate['camera_T_world_optical'][0][0] = -1
        with self.assertRaises(ValueError):
            validate_calibration(candidate)

    def test_overlay_camera_and_joint_offsets_preserve_nominal(self):
        from arx_env.joint_head_workbench_overlay import mesh_overlay
        class Geometry(FakeGeometry):
            side_meshes = {side: [('link', np.array([[-.25,-.2,1],[.25,-.2,1],[0,.25,1]]), None)]
                           for side in ('left','right')}
            def frames(self, q):
                left, right = np.eye(4), np.eye(4)
                left[0,3], right[0,3] = q[0], q[6]
                return {'left': {'link': left}, 'right': {'link': right}}
        g = Geometry()
        image = np.zeros((24,32,3), np.uint8)
        q = np.zeros(12)
        candidate = {'id': 'a', 'camera_T_world_optical': np.eye(4), 'offsets': np.zeros(12)}
        first = mesh_overlay(g, image, q, candidate)
        candidate['offsets'][0] = .3
        joints = mesh_overlay(g, image, q, candidate)
        self.assertFalse(np.array_equal(first, joints))
        candidate['camera_T_world_optical'][0,3] = .3
        camera = mesh_overlay(g, image, q, candidate)
        self.assertFalse(np.array_equal(joints, camera))
        np.testing.assert_array_equal(g.camera, np.eye(4))
        np.testing.assert_array_equal(q, np.zeros(12))
        self.assertEqual(int(image.sum()), 0)

    def test_http_auth_host_and_artifact_boundary(self):
        s = self.session()
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        token = 'x'*40
        server = serve(s, token, port)
        try:
            def request(path, headers):
                connection = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
                connection.request('GET', path, headers=headers)
                response = connection.getresponse()
                status = response.status
                response.read()
                connection.close()
                return status
            self.assertEqual(request('/state', {}), 403)
            self.assertEqual(request('/state', {'Authorization': 'Bearer '+token, 'Host': 'evil.test'}), 403)
            self.assertEqual(request('/state', {'Authorization': 'Bearer '+token}), 200)
            self.assertEqual(request('/artifacts/../auth-token', {'Authorization': 'Bearer '+token}), 404)
        finally:
            server.shutdown()
            server.server_close()

    def test_camera_jpeg_metadata_uses_rendered_pose_not_latest_command(self):
        s = self.session()
        s.jpeg['head'] = b'jpeg'
        s.frame_timestamp['head'] = 123.5
        s.frame_sequence = 17
        s.actual_q[:] = .1
        s.q[:] = .2  # Mirror command can lead the last rendered image.
        server = serve(s, 'x'*40, 0)
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2)
        try:
            connection.request('GET', '/frame.jpg?view=head', headers={'Authorization': 'Bearer '+'x'*40})
            response = connection.getresponse()
            metadata = json.loads(response.getheader('X-Rendered-Frame'))
            self.assertEqual(response.read(), b'jpeg')
            self.assertEqual(metadata['frame_sequence'], 17)
            self.assertEqual(metadata['timestamp_s'], 123.5)
            self.assertEqual(metadata['q_reported_rad'], [.1]*12)
        finally:
            connection.close()
            server.shutdown()
            server.server_close()

    def test_mjpeg_stream_requires_auth_and_yields_actual_frame(self):
        s = self.session()
        s.tick(FakeRenderer(), 0)
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        server = serve(s, 'x'*40, port)
        connection = http.client.HTTPConnection('127.0.0.1', port, timeout=2)
        try:
            connection.request('GET', '/stream.mjpg?view=actual', headers={'Authorization': 'Bearer '+'x'*40})
            response = connection.getresponse()
            self.assertEqual(response.status, 200)
            self.assertIn('multipart/x-mixed-replace', response.getheader('Content-Type'))
            self.assertEqual(response.readline(), b'--frame\r\n')
            self.assertEqual(response.readline(), b'Content-Type: image/jpeg\r\n')
            self.assertEqual(response.readline(), b'Content-Length: 4\r\n')
            self.assertTrue(response.readline().startswith(b'X-Frame-Timestamp:'))
            metadata=json.loads(response.readline().decode().removeprefix('X-Rendered-Frame: '))
            self.assertEqual(metadata['view'],'actual')
            self.assertEqual(metadata['renderer_instance_id'],s.renderer_instance_id)
            self.assertEqual(response.readline(), b'\r\n')
            self.assertEqual(response.read(4), b'jpeg')
        finally:
            connection.close()
            server.shutdown()
            server.server_close()

    def test_start_mismatch_rejected(self):
        s = self.session()
        s.status = 'READY'
        with self.assertRaises(ValueError):
            s.command('/execute', {'expected_start12': [.1] * 12, 'waypoints': [{'q': [0.] * 12}]})

    def test_preview_starts_at_current_pose_and_idle_stream_repeats(self):
        from examples.scripts.debug.joint_head_workbench import Workbench
        s = self.session()
        s.tick(FakeRenderer(), 0)
        self.assertEqual(s.jpeg['preview'], b'jpeg')
        np.testing.assert_array_equal(s.q, np.zeros(12))
        server = serve(s, 'x'*40, 0)
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2)
        try:
            connection.request('GET', '/stream.mjpg?view=preview', headers={'Authorization': 'Bearer '+'x'*40})
            response = connection.getresponse()
            first = Workbench.read_stream_frame(response)
            second = Workbench.read_stream_frame(response)
            self.assertEqual(first, second)
            self.assertEqual(first[0], b'jpeg')
            response.close()
        finally:
            connection.close()
            server.shutdown()
            server.server_close()

    def test_missing_stream_frame_returns_instead_of_leaking_handler(self):
        s = self.session()
        server = serve(s, 'x'*40, 0)
        connection = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2)
        try:
            connection.request('GET', '/stream.mjpg?view=preview', headers={'Authorization': 'Bearer '+'x'*40})
            response = connection.getresponse()
            self.assertEqual(response.status, 503)
            self.assertIn(b'frame not ready', response.read())
        finally:
            connection.close()
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    unittest.main()

class OverviewViewTests(unittest.TestCase):
    def test_camera_validation(self):
        self.assertEqual(validate_overview_view({'yaw':0,'pitch':.4,'zoom':1})['pan_y'],0)
        for field,value in [('yaw',4),('pitch',2),('zoom',0),('pan_y',float('nan'))]:
            with self.assertRaises(ValueError):validate_overview_view({**{'yaw':0,'pitch':.4,'zoom':1},field:value})


class ArticulatedRendererPoseTest(unittest.TestCase):
    def test_gap_only_change_updates_finger_transform(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from examples.scripts.debug.joint_head_workbench_live import RTXRenderer
        frames=[]
        class Geometry:
            hand=None
            def with_gaps(self,left,right):
                matrix=np.eye(4);matrix[0,3]=left/2000
                frames.append((left,right))
                return SimpleNamespace(frames=lambda q:{'left':{'finger':matrix}})
        r=RTXRenderer.__new__(RTXRenderer);r.g=r.wrist_geometry=Geometry()
        r.simulation_joint_offsets=np.zeros(12);r.pose_cache={};r.wrist_contract=None
        op=Mock();r.parts=r.preview_parts=[('left','finger',op)];r.tags=r.preview_tags=[]
        r.Gf=SimpleNamespace(Matrix4d=lambda *args:args)
        r.pose(np.zeros(12),[0.,0.]);r.pose(np.zeros(12),[80.,80.])
        self.assertEqual(op.Set.call_count,2)
        self.assertEqual(op.Set.call_args.args[0][12],.04)
        r.pose(np.zeros(12),[80.,80.]);self.assertEqual(op.Set.call_count,2)
        r.pose(np.zeros(12),[40.,40.],preview=True)
        self.assertEqual(frames[-1],(40.,40.))
