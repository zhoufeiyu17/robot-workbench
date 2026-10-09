import io
import tempfile
import time
import unittest
from pathlib import Path

from live_preview import PREVIEW_CAMERAS, LatestFrameStream, PreviewPublisher, camera_frame_path, receive_stream, read_frame


class LivePreviewTest(unittest.TestCase):
    def test_camera_slots_preserve_head_and_are_independent(self):
        with tempfile.TemporaryDirectory() as root:
            publishers = {camera: PreviewPublisher(camera_frame_path(root, camera)) for camera in PREVIEW_CAMERAS}
            self.assertEqual(publishers['base_0'].path.name, 'live-preview.frame')
            self.assertEqual(publishers['left_wrist_0'].path.name, 'live-preview-left_wrist_0.frame')
            for camera, publisher in publishers.items():
                publisher.publish(camera.encode(), {'camera': camera, 'tick': 1})
            publishers['left_wrist_0'].publish(b'new-left', {'camera': 'left_wrist_0', 'tick': 2})
            for camera, publisher in publishers.items():
                frame = read_frame(publisher.path)
                self.assertEqual(frame.metadata['camera'], camera)
                self.assertEqual(frame.metadata['tick'], 2 if camera == 'left_wrist_0' else 1)
            with self.assertRaises(ValueError):
                camera_frame_path(root, '../unexpected')

    def test_atomic_latest_frame_and_framed_transport(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root) / "latest.frame"
            cache = Path(root) / "cache.frame"
            publisher = PreviewPublisher(source)
            publisher.publish(b"old", {"tick": 1})
            publisher.publish(b"new", {"tick": 2})
            latest = read_frame(source)
            self.assertEqual(latest.payload, b"new")
            output = io.BytesIO()
            result = LatestFrameStream(source, output=output, hz=1000).run(0.003)
            self.assertEqual(result["sent_frames"], 1)
            receive = receive_stream(io.BytesIO(b"PAI DSW banner\n" + output.getvalue()), cache, 0.1)
            self.assertEqual(receive["received_frames"], 1)
            self.assertEqual(read_frame(cache).metadata["tick"], 2)

    def test_missing_frame_is_not_an_error(self):
        with tempfile.TemporaryDirectory() as root:
            self.assertIsNone(read_frame(Path(root) / "missing"))

    def test_failed_capture_exits_without_ever_publishing_a_frame(self):
        with tempfile.TemporaryDirectory() as root:
            (Path(root)/'failure.json').write_text('{}')
            result=LatestFrameStream(Path(root)/'missing',output=io.BytesIO()).run()
            self.assertEqual(result['sent_frames'],0)


if __name__ == "__main__":
    unittest.main()
