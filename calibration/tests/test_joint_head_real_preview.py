import importlib.util
from pathlib import Path
import unittest
import tempfile
from types import SimpleNamespace
from unittest.mock import Mock, patch

import cv2
import numpy as np


class PreviewCheck(unittest.TestCase):
    def test_capture_publishes_every_frame_after_warmup(self):
        path = Path(__file__).resolve().parents[1] / 'examples/scripts/debug/joint_head_real_preview.py'
        spec = importlib.util.spec_from_file_location('real_preview', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        frame = Mock()
        frame.get_data.return_value = np.full((480, 640, 2), 128, np.uint8)
        frame.get_frame_number.return_value = 23
        frame.get_timestamp.return_value = 100.
        frame.get_frame_timestamp_domain.return_value = 'hardware_clock'
        frames = Mock()
        frames.get_color_frame.return_value = frame
        pipeline = Mock()
        pipeline.wait_for_frames.side_effect = [frames]*23+[RuntimeError('test ended')]
        pipeline.start.return_value.get_stream.return_value.as_video_stream_profile.return_value.get_intrinsics.return_value = SimpleNamespace(
            width=640, height=480, fx=600, fy=600, ppx=320, ppy=240, model='none', coeffs=[0]*5)
        rs = Mock()
        rs.pipeline.return_value = pipeline
        with tempfile.TemporaryDirectory() as root, patch.dict('sys.modules', {'pyrealsense2': rs}), \
             patch('sys.argv', ['preview', '--serial', 'fake', '--duration-s', '0', '--output', root+'/camera']), \
             patch.object(module, 'ThreadingHTTPServer') as server, patch.object(module.threading, 'Thread'), \
             patch.object(module.time, 'monotonic', return_value=1.), \
             patch.object(module, 'marker_check', wraps=module.marker_check) as detect:
            with self.assertRaisesRegex(RuntimeError, 'test ended'):
                module.main()
            self.assertEqual(detect.call_count, 4)  # Frames 20..23, not one out of 15.
            self.assertTrue((Path(root)/'camera/annotated.jpg').exists())
            pipeline.stop.assert_called_once()
            handler_type = server.call_args.args[1]
            handler = object.__new__(handler_type)
            handler.connection = Mock()
            with patch.object(module.BaseHTTPRequestHandler, 'setup'):
                handler.setup()
            handler.connection.setsockopt.assert_called_once_with(module.socket.IPPROTO_TCP, module.socket.TCP_NODELAY, 1)

    def test_placement_ids_and_blank(self):
        path = Path(__file__).resolve().parents[1] / 'examples/scripts/debug/joint_head_real_preview.py'
        spec = importlib.util.spec_from_file_location('real_preview', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        image = np.full((300, 500, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for marker_id, x in [(24, 30), (30, 270)]:
            marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 150)
            image[75:225, x:x+150] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        _, rows = module.marker_check(image, [24, 30])
        self.assertEqual({row['id'] for row in rows}, {24, 30})
        self.assertTrue(all(row['placement_size_pass'] for row in rows))
        self.assertEqual(module.marker_check(np.full_like(image, 255), [24, 30])[1], [])


if __name__ == '__main__':
    unittest.main()
