"""The recording factory must resolve the current camera instance per session."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import json,sys,unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


class OwnerRecordingCameraTest(unittest.TestCase):
    def test_camera_restart_between_sessions_resolves_new_instance_without_reconnecting_sdk(self):
        from scripts.serve_control_owner import main
        with TemporaryDirectory()as directory:
            root=Path(directory);profile=root/'profile.json';descriptor=root/'camera.json'
            profile.write_text('{}');descriptor.write_text(json.dumps(dict(instance_id='first')))
            channels={'first':{'base_0':object()},'second':{'base_0':object()}}
            def connect(path):return NS(channels=channels[json.loads(Path(path).read_text())['instance_id']])
            service=Mock();recorder=Mock();runtime_factory=Mock(return_value=lambda:None)
            def serve():
                factory=runtime_factory.call_args.kwargs['recording_factory']
                factory(dict(plan_ref={'fixture':True}),lambda reason:None)
                descriptor.write_text(json.dumps(dict(instance_id='second')))
                factory(dict(plan_ref={'fixture':True}),lambda reason:None)
            service.serve_forever.side_effect=serve
            with patch.object(sys,'argv',['owner','--profile',str(profile),'--root',str(root/'owner')]),patch(
                    'robot_workbench.device_bootstrap.read_profile',return_value=dict(robot_id='cpu',camera_service_descriptor=str(descriptor))),patch(
                    'robot_workbench.device_bootstrap.physical_runtime_factory',runtime_factory),patch(
                    'robot_workbench.control_owner.ControlOwnerService',return_value=service),patch(
                    'robot_workbench.rgbd_service.CommonCameraService.connect',side_effect=connect),patch(
                    'robot_workbench.recording_process.ProcessRecordingSession',recorder),patch('scripts.serve_control_owner.signal.signal'):
                self.assertEqual(main(),0)
            self.assertIs(recorder.call_args_list[0].kwargs['camera_channels'],channels['first'])
            self.assertIs(recorder.call_args_list[1].kwargs['camera_channels'],channels['second'])
            self.assertEqual(runtime_factory.call_count,1)


if __name__=='__main__':unittest.main()
