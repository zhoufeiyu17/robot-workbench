"""Exercise owner/client separation without ROS or hardware."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.camera_owner import atomic, profile, read_owner, reuse_camera_owner


class CameraOwnerTest(unittest.TestCase):
    def args(self, ready):
        return argparse.Namespace(cameras='{"base_0":"serial"}', topic_prefix='/camera', width=640,
            height=480, ir_width=640, ir_height=480, fps=30, enable_ir=True, qos_depth=10, ready_file=str(ready))

    def test_reuse_signals_readiness_and_client_exit_leaves_owner_alive(self):
        with tempfile.TemporaryDirectory() as folder:
            state, ready = Path(folder)/'state.json', Path(folder)/'ready.json'
            args = self.args(ready)
            atomic(state, dict(instance='one', state='running', profile=profile(args), heartbeat=time.monotonic()))
            before = state.read_bytes()
            def stop_client(_):
                self.assertEqual(json.loads(ready.read_text())['devices'], ['base_0'])
                raise KeyboardInterrupt
            with patch('robot_workbench.camera_owner.time.sleep', stop_client):
                self.assertEqual(reuse_camera_owner(args, state), 0)
            self.assertFalse(ready.exists())
            self.assertEqual(state.read_bytes(), before)

    def test_mismatch_stale_and_owner_restart_fail_without_opening_hardware(self):
        with tempfile.TemporaryDirectory() as folder:
            state, ready = Path(folder)/'state.json', Path(folder)/'ready.json'
            args = self.args(ready)
            value = dict(instance='one', state='running', profile=profile(args), heartbeat=time.monotonic())
            atomic(state, dict(value, heartbeat=time.monotonic()-10))
            with self.assertRaisesRegex(ValueError, '心跳'): read_owner(state)
            atomic(state, value)
            args.width = 320
            self.assertEqual(reuse_camera_owner(args, state), 1)
            args.width = 640
            def restart(_): atomic(state, dict(value, instance='two'))
            with patch('robot_workbench.camera_owner.time.sleep', restart):
                self.assertEqual(reuse_camera_owner(args, state), 1)
            self.assertFalse(ready.exists())


if __name__ == '__main__': unittest.main()
