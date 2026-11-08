"""Exercise owner/client separation without ROS or hardware."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_workbench.camera_owner import atomic, profile, read_owner, reuse_camera_owner


class CameraOwnerTest(unittest.TestCase):
    def test_existing_source_registration_never_opens_or_stops_shared_cameras(self):
        import os, runpy, signal
        entry=Path(__file__).resolve().parents[1]/'robot_workbench/camera_owner.py'
        with tempfile.TemporaryDirectory() as folder:
            state=Path(folder)/'state.json';args=self.args(Path(folder)/'ready.json')
            atomic(state,dict(instance='shared',state='running',profile=profile(args),heartbeat=time.monotonic()))
            original=state.read_bytes();signals={};messages=[]
            ros=Mock();ros.types=SimpleNamespace(SourceDescriptor=SimpleNamespace)
            ros.publisher.return_value.publish.side_effect=messages.append
            argv=[str(entry),'--state',str(state),'--cameras',args.cameras,'--topic_prefix','/camera',
                  '--device-id','cpu','--ros-namespace','/cpu','--observe-existing']
            with patch.object(sys,'argv',argv),patch('signal.signal',side_effect=lambda sig,cb:signals.__setitem__(sig,cb)),\
                    patch('subprocess.Popen') as construct,patch('os.killpg') as terminate,\
                    patch('time.sleep',side_effect=lambda _:signals[signal.SIGTERM](signal.SIGTERM,None)),\
                    patch('robot_workbench.ros_data.RosNode',return_value=ros):
                with self.assertRaises(SystemExit) as result:runpy.run_path(str(entry),run_name='__main__')
            self.assertEqual(result.exception.code,0)
            self.assertEqual(state.read_bytes(),original)
            self.assertEqual([message.available for message in messages],[True,False])
            self.assertEqual({message.source_instance_id for message in messages},{'shared'})
            construct.assert_not_called();terminate.assert_not_called();ros.close.assert_called_once()

    def test_registered_file_entry_publishes_ros_identity_and_stops_its_child(self):
        import os,runpy,signal
        from robot_workbench.managed_services import _script_argument
        entry=Path(__file__).resolve().parents[1]/'robot_workbench/camera_owner.py'
        self.assertEqual(_script_argument([sys.executable,str(entry)]),str(entry))
        with tempfile.TemporaryDirectory()as folder:
            state=Path(folder)/'state.json';signals={};messages=[]
            child=Mock(pid=999999,returncode=0);child.poll.return_value=None
            ros=Mock();ros.types=SimpleNamespace(SourceDescriptor=SimpleNamespace)
            ros.publisher.return_value.publish.side_effect=messages.append
            def publish(command,**kwargs):
                Path(command[command.index('--ready_file')+1]).write_text('{}')
                return child
            def stop(_):signals[signal.SIGTERM](signal.SIGTERM,None)
            argv=[str(entry),'--state',str(state),'--cameras','{"base_0":"123"}',
                  '--device-id','cpu','--ros-namespace','/cpu']
            with patch.object(sys,'argv',argv),patch.dict(os.environ,ROS_DOMAIN_ID='126'),\
                    patch('signal.signal',side_effect=lambda sig,callback:signals.__setitem__(sig,callback)),\
                    patch('subprocess.Popen',side_effect=publish),patch('os.killpg')as terminate,\
                    patch('time.sleep',side_effect=stop),patch('robot_workbench.ros_data.RosNode',return_value=ros):
                with self.assertRaises(SystemExit)as result:runpy.run_path(str(entry),run_name='__main__')
            self.assertEqual(result.exception.code,0)
            self.assertEqual(json.loads(state.read_text())['state'],'stopped')
            self.assertEqual([message.available for message in messages],[True,False])
            self.assertEqual({message.device_id for message in messages},{'cpu'})
            terminate.assert_called_once_with(child.pid,signal.SIGINT)
            child.wait.assert_called_once_with(timeout=8)
            ros.close.assert_called_once()

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
