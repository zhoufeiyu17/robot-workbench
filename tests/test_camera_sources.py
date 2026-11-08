"""Camera-only lifetime: real state files and cleanup, no hardware SDK or arms."""
import json
import fcntl
import multiprocessing
import os
from pathlib import Path
import signal
import sys
import time
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts'),str(ROOT/'rule_based_workbench')]
from serve_camera_sources import run,reset_camera
from robot_workbench.rgbd_service import start_capture_sources


def physical_source_fixture(profile_path,output):
    """Run the actual source CLI with a synthetic SDK, in its own process."""
    import numpy as np
    from real_rgbd import main
    profile=json.loads(Path(profile_path).read_text());intr=json.loads(Path(profile['cameras']['head']['intrinsics']).read_text())
    counter=[0]
    native=SimpleNamespace(width=2,height=2,fx=10.,fy=10.,ppx=1.,ppy=1.,model='none',coeffs=[])
    def frame(depth):
        return SimpleNamespace(get_frame_number=lambda:counter[0]+(10 if depth else 0),
            get_timestamp=lambda:counter[0]*1000/30+(1 if depth else 0),get_frame_timestamp_domain=lambda:'hardware_clock',
            profile=SimpleNamespace(as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:native)),
            get_data=lambda:np.full((2,2),500,np.uint16)if depth else np.zeros((2,2,2),np.uint8))
    def wait(_):
        time.sleep(1/30);counter[0]+=1
        return SimpleNamespace(as_frameset=lambda:SimpleNamespace(get_color_frame=lambda:frame(False),get_depth_frame=lambda:frame(True)))
    pipeline=SimpleNamespace(start=lambda *a:SimpleNamespace(get_device=lambda:SimpleNamespace(
        first_depth_sensor=lambda:SimpleNamespace(get_depth_scale=lambda:.001))),stop=lambda:None)
    sdk=SimpleNamespace(pipeline=lambda:pipeline,config=lambda:SimpleNamespace(enable_device=lambda *a:None,enable_stream=lambda *a:None),
        stream=SimpleNamespace(color=1,depth=2),format=SimpleNamespace(yuyv=1,z16=2),
        frame_queue=lambda *a:SimpleNamespace(wait_for_frame=wait),align=lambda *a:SimpleNamespace(process=lambda p:p))
    cv=SimpleNamespace(COLOR_YUV2RGB_YUY2=1,cvtColor=lambda *a:np.arange(12,dtype=np.uint8).reshape(2,2,3))
    with patch.dict(sys.modules,{'pyrealsense2':sdk,'cv2':cv}),patch.object(sys,'argv',['source','--profile',str(profile_path),
            '--serial',intr['serial'],'--camera-name','head','--output',str(output),'--duration-s','3']):main()


class CameraSourcesTest(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('WORKBENCH_NATIVE_ROS_TEST')=='1','isolated native ROS required')
    def test_actual_source_cli_with_synthetic_sdk_publishes_and_exits_without_spool(self):
        from robot_workbench.ros_data import RosNode,SourceDirectory
        from robot_workbench.ros_inputs import RosInputs
        with TemporaryDirectory()as temp:
            root=Path(temp);intr=root/'intrinsics.json'
            intr.write_text(json.dumps(dict(serial='123',width=2,height=2,fx=10.,fy=10.,cx=1.,cy=1.,coefficients=[],distortion_model='none',rectified=False)))
            profile=root/'profile.json';profile.write_text(json.dumps(dict(environment='physical',robot_id='cpu',
                data_plane=dict(schema='workbench.ros-data-plane.v1',namespace='/wb_native_camera',domain_id=128),
                cameras=dict(head=dict(serial='123',intrinsics=str(intr))))))
            process=multiprocessing.get_context('spawn').Process(target=physical_source_fixture,args=(profile,root/'source'))
            receiver=RosNode('native_camera_reader',namespace='/wb_native_camera',domain_id=128);process.start()
            try:
                rows=SourceDirectory(receiver,'cpu').resolve({'base_0':dict(kind='image')},timeout=8)
                saved=[];inputs=RosInputs(receiver,rows,include_depth=True,on_sample=lambda ref,value:saved.append((ref,value)))
                deadline=time.monotonic()+5
                while not saved:
                    self.assertLess(time.monotonic(),deadline);time.sleep(.02)
                ref,value=saved[0]
                self.assertEqual(value['rgb'],bytes(range(12)));self.assertEqual(value['depth'],bytes([244,1])*4)
                self.assertEqual(value['metadata']['depth_frame_number']-value['metadata']['frame_number'],10)
                self.assertEqual(value['metadata']['host_receive_ns'],ref.source_time_ns)
                process.join(8);self.assertEqual(process.exitcode,0)
                deadline=time.monotonic()+3
                while not inputs.caches['base_0'].error:
                    self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                self.assertFalse(list(root.rglob('*.zip')));self.assertFalse(list(root.rglob('*.rgb')))
                self.assertEqual(json.loads((root/'source/status.json').read_text())['phase'],'stopped')
            finally:
                if process.is_alive():process.terminate();process.join(5)
                receiver.close()

    def test_ros_parent_uses_original_capture_processes_and_no_common_service(self):
        with TemporaryDirectory()as temp:
            root=Path(temp);path=self.reset_profile(root);profile=json.loads(path.read_text())
            profile.update(robot_id='cpu',data_plane=dict(schema='workbench.ros-data-plane.v1',namespace='/cpu',domain_id=126))
            path.write_text(json.dumps(profile));process=Mock();process.poll.return_value=1
            view=Mock();view.descriptor=dict(environment='physical',data_plane=profile['data_plane'])
            def sources(profile,service,output):
                (output/'capture-head.log').write_text('RuntimeError: original publisher failed\n');return [process]
            with patch('serve_camera_sources.signal.signal'),patch('robot_workbench.ros_data.require_runtime'),\
                    patch('robot_workbench.ros_inputs.RosCameraView',return_value=view),\
                    patch('serve_camera_sources.CommonCameraService.start',side_effect=AssertionError('no legacy IPC')),\
                    patch('serve_camera_sources.start_capture_sources',side_effect=sources):
                with self.assertRaisesRegex(RuntimeError,'original publisher failed'):run(path,root/'owner')
            view.close.assert_called_once();process.terminate.assert_not_called()

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

    def test_ros_capture_launcher_uses_original_source_without_common_ingress(self):
        with TemporaryDirectory()as temp:
            root=Path(temp);plane=dict(schema='workbench.ros-data-plane.v1',namespace='/cpu_source',domain_id=126)
            profile=dict(environment='physical',robot_id='cpu',data_plane=plane,
                cameras={role:dict(serial=str(n),intrinsics=str(root/(role+'.json')))for n,role in enumerate(('head','left','right'))})
            service=SimpleNamespace(descriptor=dict(data_plane=plane))
            with patch('robot_workbench.rgbd_service.subprocess.Popen')as launch:
                self.assertEqual(len(start_capture_sources(profile,service,root)),3)
            for call in launch.call_args_list:
                args=call.args[0];self.assertNotIn('--service-descriptor',args)
                frozen=json.loads(Path(args[args.index('--profile')+1]).read_text())
                self.assertEqual(frozen,profile)

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
