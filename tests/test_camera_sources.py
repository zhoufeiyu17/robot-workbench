"""Camera-only lifetime: real state files and cleanup, no hardware SDK or arms."""
import json
import fcntl
from pathlib import Path
import signal
import sys
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT/'rule_based_workbench')]
from serve_camera_sources import run,reset_camera
from robot_workbench.rgbd_service import start_capture_sources


class CameraSourcesTest(unittest.TestCase):
    def reset_profile(self,root,environment='physical'):
        profile=dict(environment=environment,cameras={})
        for index,role in enumerate(('head','left','right')):
            path=root/(role+'.json');path.write_text(json.dumps(dict(serial=str(index+100))))
            profile['cameras'][role]=dict(serial=str(index+100),intrinsics=str(path))
        path=root/'profile.json';path.write_text(json.dumps(profile));return path

    def test_reset_matches_one_profile_serial_and_never_resets_again_while_waiting(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);path=self.reset_profile(root)
            devices=[Mock() for _ in range(3)]
            for index,device in enumerate(devices):device.get_info.return_value=str(index+100)
            context=Mock();context.query_devices.side_effect=[devices,[devices[0],devices[2]],devices]
            sdk=SimpleNamespace(context=lambda:context,camera_info=SimpleNamespace(serial_number='serial'))
            result=reset_camera(path,root/'owner','left',sdk=sdk,poll_s=.001,timeout_s=.1)
            self.assertEqual(result['serial'],'101');self.assertEqual(result['reset_calls'],1)
            self.assertTrue(result['disappearance_observed']);self.assertFalse(result['raw_frames_verified'])
            devices[1].hardware_reset.assert_called_once();devices[0].hardware_reset.assert_not_called();devices[2].hardware_reset.assert_not_called()
            context.query_devices.side_effect=[devices]+[[]]*100
            with self.assertRaisesRegex(TimeoutError,'outcome unknown'):reset_camera(path,root/'owner','right',sdk=sdk,poll_s=.001,timeout_s=.01)
            devices[2].hardware_reset.assert_called_once()

    def test_reset_rejects_fake_unknown_roles_and_live_camera_owner_before_sdk(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);sdk=Mock();path=self.reset_profile(root,'fake')
            with self.assertRaisesRegex(ValueError,'physical'):reset_camera(path,root/'owner','head',sdk=sdk)
            path=self.reset_profile(root)
            with self.assertRaisesRegex(ValueError,'fixed camera role'):reset_camera(path,root/'owner','arbitrary-serial',sdk=sdk)
            owner=root/'owner';owner.mkdir()
            with (owner/'owner.lock').open('a') as lock:
                fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                with self.assertRaises(BlockingIOError):reset_camera(path,owner,'head',sdk=sdk)
            sdk.context.assert_not_called()

    def test_failed_source_preserves_error_and_only_cleans_its_camera_processes(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);profile={'cameras':{}}
            for n,role in enumerate(('head','left','right')):
                path=root/(role+'.json');path.write_text(json.dumps(dict(serial=str(n),width=640,height=480)))
                profile['cameras'][role]=dict(serial=str(n),intrinsics=str(path))
            path=root/'profile.json';path.write_text(json.dumps(profile))
            source=Mock();source.poll.return_value=1
            service=Mock();service.descriptor={'instance_id':'cpu-only','secret':'private-local'};service.process.poll.return_value=None
            def sources(profile,service,output):
                (output/'capture-head.log').write_text('RuntimeError: camera busy\n')
                return [source]
            with patch('serve_camera_sources.signal.signal'),patch('serve_camera_sources.CommonCameraService.start',return_value=service),patch('serve_camera_sources.start_capture_sources',side_effect=sources):
                with self.assertRaisesRegex(RuntimeError,'camera busy'):run(path,root/'session')
            state=json.loads((root/'session/state.json').read_text())
            self.assertEqual(state['phase'],'error');self.assertIn('camera busy',state['error'])
            self.assertNotIn('private-local',json.dumps(state))
            self.assertEqual((root/'session/current.json').stat().st_mode&0o777,0o600)
            service.close.assert_called_once();source.terminate.assert_not_called()

    def test_raw_capture_launcher_has_no_control_command_and_uses_common_descriptor(self):
        with TemporaryDirectory() as temp:
            root=Path(temp);service=SimpleNamespace(directories={'base_0':str(root/'camera-service/base_0')})
            profile={'cameras':{role:{'serial':str(n)} for n,role in enumerate(('head','left','right'))}}
            with patch('robot_workbench.rgbd_service.subprocess.Popen') as launch:
                self.assertEqual(len(start_capture_sources(profile,service,root)),3)
            for call in launch.call_args_list:
                args=call.args[0]
                self.assertTrue(args[2].endswith('/real_rgbd.py'))
                self.assertEqual(args[args.index('--service-descriptor')+1],str(root/'camera-service/camera-service.json'))
                self.assertNotIn('--connect-hardware',args)

    def test_one_failed_capture_invalidates_its_lease_but_keeps_healthy_sources_live(self):
        with TemporaryDirectory()as temp:
            root=Path(temp);profile=self.reset_profile(root);owner=root/'owner'
            head,left,right=(Mock()for _ in range(3))
            head.poll.return_value=right.poll.return_value=None;left.poll.return_value=1
            service=Mock();service.descriptor={'instance_id':'cpu-only'};service.process.poll.return_value=None
            service.frames={name:Mock()for name in ('base_0','left_wrist_0','right_wrist_0')}
            service.channels={name:Mock()for name in service.frames}
            for frame in service.frames.values():frame.snapshot.return_value={'status':{'ready':True}}
            service.channels['left_wrist_0'].token.return_value='left-recording-lease'
            observed=[];waits=[]
            def wait(_):
                waits.append(True)
                if len(waits)==1:return False
                observed.append(json.loads((owner/'state.json').read_text()))
                head.terminate.assert_not_called();right.terminate.assert_not_called();service.close.assert_not_called()
                return True
            stopped=Mock();stopped.wait.side_effect=wait
            def sources(profile,service,output):
                (output/'capture-left.log').write_text('ValueError: injected capture failure\n')
                return [head,left,right]
            with patch('serve_camera_sources.signal.signal'),patch('serve_camera_sources.threading.Event',return_value=stopped),\
                 patch('serve_camera_sources.CommonCameraService.start',return_value=service),\
                 patch('serve_camera_sources.start_capture_sources',side_effect=sources):run(profile,owner)
            self.assertEqual(observed[0]['phase'],'degraded')
            self.assertEqual(set(observed[0]['camera_failures']),{'left'})
            service.frames['left_wrist_0'].publish.assert_called_once()
            service.frames['base_0'].publish.assert_not_called();service.frames['right_wrist_0'].publish.assert_not_called()
            service.channels['left_wrist_0'].fail.assert_called_once_with('ValueError: injected capture failure',sampled_token='left-recording-lease')
            head.terminate.assert_called_once();right.terminate.assert_called_once();left.terminate.assert_not_called()


if __name__=='__main__':unittest.main()
