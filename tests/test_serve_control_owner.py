"""The recording factory must resolve the current camera instance per session."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import json,sys,unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))


class OwnerRecordingCameraTest(unittest.TestCase):
    def test_recording_uses_current_ros_runtime_without_opening_a_legacy_camera(self):
        from scripts.serve_control_owner import main
        with TemporaryDirectory()as directory:
            root=Path(directory);profile=root/'profile.json';profile.write_text('{}')
            service=Mock();recorder=Mock();runtime_factory=Mock(return_value=lambda:None)
            device=NS(runtime=None)
            runtimes=[NS(owner=NS(control_time=lambda:1.)),NS(owner=NS(control_time=lambda:2.))]
            def serve():
                factory=runtime_factory.call_args.kwargs['recording_factory']
                for runtime in runtimes:
                    device.runtime=runtime;factory(dict(plan_ref={'fixture':True}),lambda reason:None)
            service.serve_forever.side_effect=serve
            with patch.object(sys,'argv',['owner','--profile',str(profile),'--root',str(root/'owner')]),patch(
                    'robot_workbench.device_bootstrap.read_profile',return_value=dict(robot_id='cpu',data_plane={})),patch(
                    'robot_workbench.device_bootstrap.physical_runtime_factory',runtime_factory),patch(
                    'robot_workbench.service.Device',return_value=device),patch(
                    'robot_workbench.control_owner.ControlOwnerService',return_value=service),patch(
                    'robot_workbench.rgbd_service.CommonCameraService.connect',side_effect=AssertionError('no legacy source')),patch(
                    'robot_workbench.recording_process.capture_control',recorder),patch(
                    'robot_workbench.ros_data.require_runtime'),patch('scripts.serve_control_owner.signal.signal'):
                self.assertEqual(main(),0)
            for call,runtime in zip(recorder.call_args_list,runtimes):
                self.assertIs(call.kwargs['ros_runtime'],runtime);self.assertIsNone(call.kwargs['cameras'])
            self.assertEqual(runtime_factory.call_count,1)


if __name__=='__main__':unittest.main()
