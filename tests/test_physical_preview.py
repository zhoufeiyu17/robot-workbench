"""CPU camera/feedback pairing and display race checks; no SDK, GPU or motion."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import threading
import time
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'calibration'),str(ROOT/'calibration/infra'),str(ROOT/'rule_based_workbench')]
from examples.scripts.debug.joint_head_workbench import Workbench
from arx_env.joint_head_jaw_mapping import configure_mapping
from robot_workbench.physical_preview import PhysicalPreview
from robot_workbench.target_preview import PendingFrame
from robot_workbench.service import Application,Device


class PreviewTests(unittest.TestCase):
    def test_one_group_identity_survives_all_camera_views(self):
        from arx_env.joint_head_calibration_store import calibration_group_id
        identity=calibration_group_id('head-a','wrist-a')
        self.preview.bind_context('group-context');self.preview.start()
        for camera in ('head','left','right','all'):
            selected=self.preview.mesh(True,camera=camera,candidate_id=identity)
            self.assertEqual(selected['candidate_id'],identity)
            self.wait(lambda:self.preview.state().get('frames',{}).get('mesh',{}).get('camera')==camera)
            raw,meta=self.preview.media('mesh')
            self.assertEqual(meta['candidate_id'],identity)
            self.assertEqual(meta['head_candidate_id'],'head-a')
            self.assertEqual(meta['wrist_candidate_id'],'wrist-a')
            self.assertTrue(raw)

    def test_only_explicit_historical_wrist_with_same_sealed_asset_is_displayable(self):
        assets=dict(asset_geometry_id='asset-a',asset_seal_sha256='seal-a')
        self.app.config.update(assets)
        parameters=Workbench.calibration_snapshot(dict(self.head['parameters'],**assets))
        self.head['parameters']=parameters
        self.contract.update(assets,geometry_id='old-workbench',head_snapshot=parameters,robot_source='physical',simulation_only=False)
        self.wrist.update(assets,pair_status='complete',version='v000006')
        self.wrist['result']=dict(contract_sha256=hashlib.sha256(json.dumps(self.contract,sort_keys=True,allow_nan=False).encode()).hexdigest(),
            head_snapshot_sha256=parameters['snapshot_sha256'])
        self.app.wrist_results=lambda:[]  # Current installation/solver eligibility stays strict.
        self.app.calibration_store_record=lambda identity:dict(id='wrist-a',kind='wrist',version='v000006',environment='hardware',
            payload=dict(row=deepcopy(self.wrist),contract=deepcopy(self.contract)))
        self.app.wrist_display_results=lambda:Workbench.wrist_display_results(self.app)
        with self.assertRaises(ValueError):self.preview._candidates('left','wrist-a')
        self.app.config['readonly_wrist_candidate_id']='wrist-a'
        for camera in ('left','right'):
            selected=self.preview._candidates(camera,'wrist-a')
            self.assertEqual(selected['candidate_id'],'wrist-a');self.assertEqual(selected['head']['id'],'head-a')
        self.assertEqual(self.app.wrist_results(),[])
        self.assertIsNotNone(self.app.wrist_overlay_error(dict(self.wrist,id='another-wrist')))
        self.app.config['asset_seal_sha256']='different-seal'
        self.assertIsNotNone(self.app.wrist_overlay_error(self.wrist))

    def setUp(self):
        configure_mapping(None)
        self.tmp=TemporaryDirectory();self.addCleanup(self.tmp.cleanup);root=Path(self.tmp.name)
        T=np.eye(4).tolist()
        parameters=Workbench.calibration_snapshot(dict(id='head-a',camera_T_world_optical=T,
            joint_offsets_rad=dict(left=[0.]*6,right=[0.]*6),T_link6_marker={'24':T,'30':T}))
        self.head=dict(id='head-a',parameters=parameters,environment='hardware',robot_source='physical',simulation_only=False)
        cameras={role:dict(serial=str(900+i),width=32,height=24,fx=25.,fy=25.,cx=16.,cy=12.,
            coefficients=[0.]*5,distortion_model='distortion.brown_conrady',rectified=False)
            for i,role in enumerate(('head','left','right'))}
        self.contract=dict(head_snapshot=parameters,cameras={r:cameras[r] for r in ('left','right')},
            environment='hardware',geometry_id='geometry')
        digest=hashlib.sha256(json.dumps(self.contract,sort_keys=True,allow_nan=False).encode()).hexdigest()
        self.wrist=dict(id='wrist-a',kind='wrist',environment='hardware',robot_source='physical',simulation_only=False,
            left=dict(observer='left',T_link6_camera=T),right=dict(observer='right',T_link6_camera=T),
            result=dict(contract_sha256=digest,head_snapshot_sha256=parameters['snapshot_sha256']))
        for role,k in cameras.items():(root/(role+'.json')).write_text(json.dumps(k))
        triangle=np.array([[-.2,-.15,1],[.2,-.15,1],[0,.2,1]])
        geometry=SimpleNamespace(hand=None,gaps=dict(left=.04,right=.04),
            frames=lambda q:{side:dict(link6=np.eye(4)) for side in ('left','right')},
            side_meshes={side:[('link6',triangle,None)] for side in ('left','right')})
        self.owner=dict(pid=123,command_epoch=2,runtime_nonce='owner-a')
        self.history=[];self.serial_override=None;self.delta_ms=0.;self.frame_age_s=0.;self.seq=0
        test=self
        class Camera:
            def __init__(self,role):self.role=role
            def read_calibration_frame(self):
                test.seq+=1;stamp=time.time_ns()-int(test.frame_age_s*1e9)
                q=[0.]*14;q[6]=q[13]=.04
                test.history.append(dict(owner=dict(test.owner,calibration_feedback=dict(qpos=q,
                    sample_wall_ns=stamp+int(test.delta_ms*1e6)))))
                raw=cv2.imencode('.jpg',np.zeros((24,32,3),np.uint8))[1].tobytes()
                return raw,dict(ready=True,environment='physical',serial=test.serial_override or cameras[self.role]['serial'],
                    intrinsics=deepcopy(cameras[self.role]),host_receive_ns=str(stamp),frame_number=test.seq,
                    raw=True,annotated=False,image_domain='raw')
        self.app=SimpleNamespace(config=dict(environment='physical',camera=str(root/'head.json'),
            hardware_wrist_camera_files={r:str(root/(r+'.json')) for r in ('left','right')},
            hardware_camera_serials={r:k['serial'] for r,k in cameras.items()}),geometry=geometry,geometry_id='geometry',
            hardware_camera=Camera('head'),wrist_cameras={r:Camera(r) for r in ('left','right')},
            hardware_connected=lambda:True,hardware_feedback=dict(owner=self.owner),
            feedback_history_snapshot=lambda:list(self.history),calibration_results=lambda:[self.head],
            wrist_results=lambda:[self.wrist],installed_pair=lambda:dict(head_id='head-a',wrist_id='wrist-a'),
            calibration_asset_matches=lambda row:True,calibration_snapshot=Workbench.calibration_snapshot,
            wrist_contract=lambda row:self.contract)
        self.app.wrist_overlay_error=lambda row:Workbench.wrist_overlay_error(self.app,row)
        self.source_cameras=cameras
        self.preview=PhysicalPreview(self.app,interval_s=.02);self.addCleanup(self.preview.close)

    def wait(self,predicate):
        end=time.monotonic()+3
        while time.monotonic()<end:
            if predicate():return
            time.sleep(.01)
        self.fail('preview did not reach expected state: '+str(self.preview.state()))

    def test_cpu_render_three_real_frames_and_service_contract(self):
        with patch('subprocess.Popen',side_effect=AssertionError('no process starts')):
            self.preview.start();self.preview.start()
            device=Device('ARX6','physical',None);device.preview=self.preview
            application=Application({'physical':device});application.preview_state(device)
            selected=self.preview.mesh(True,camera='all')
            self.assertEqual(selected['candidate_id'],'wrist-a')
            self.wait(lambda:bool(application.preview_state(device)['frames']['mesh']['ready']))
            raw,meta=application.preview_media('physical','mesh')
            self.assertEqual(cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR).shape,(48,96,3))
            self.assertEqual([m['camera'] for m in meta['cameras']],['head','left','right'])
            self.assertEqual([m['candidate_id'] for m in meta['cameras']],['head-a','wrist-a','wrist-a'])
            self.assertEqual(meta['head_candidate_id'],'head-a')
            self.assertFalse(meta['synchronized_capture']);self.assertFalse(meta['measurement_qualified'])
            self.assertTrue(all(m['image_joint_delta_ms']==0 for m in meta['cameras']))
            self.assertGreater(int(cv2.imdecode(np.frombuffer(raw,np.uint8),1).sum()),0)
            self.assertFalse(self.preview.state()['capabilities']['target'])
            with self.assertRaisesRegex(ValueError,'目标姿态'):self.preview.target_pose([],[])
            with self.assertRaisesRegex(ValueError,'整机仿真'):self.preview.media('overview')
        self.assertIsNone(device.runtime)

    def test_camera_switch_keeps_enabled_and_uses_exact_head_dependency(self):
        self.preview.mesh(True,camera='left',candidate_id='wrist-a')
        self.assertEqual(self.preview.mesh(True,camera='head')['candidate_id'],'head-a')
        self.assertEqual(self.preview.mesh(True,camera='right')['candidate_id'],'wrist-a')
        self.assertTrue(self.preview.state()['mesh']['enabled'])
        self.contract['head_snapshot']=dict(self.contract['head_snapshot'],id='missing-head')
        with self.assertRaisesRegex(ValueError,'头部标定缺失'):
            self.preview.mesh(True,camera='all',candidate_id='wrist-a')

    def test_rejects_fake_candidates_changed_intrinsics_and_bad_pairing(self):
        self.head['simulation_only']=True
        with self.assertRaises(ValueError):self.preview.mesh(True,camera='head',candidate_id='head-a')
        self.head['simulation_only']=False
        chosen=self.preview._candidates('head','head-a');owner=self.preview._owner()
        self.serial_override='other'
        with self.assertRaisesRegex(ValueError,'序列号'):self.preview._render_camera('head',chosen,owner)
        self.serial_override=None;self.history.clear();self.delta_ms=201
        with self.assertRaisesRegex(ValueError,'时间差'):self.preview._render_camera('head',chosen,owner)
        self.history.clear();self.delta_ms=0;self.frame_age_s=3
        with self.assertRaisesRegex(ValueError,'已过期'):self.preview._render_camera('head',chosen,owner)
        self.frame_age_s=0
        path=Path(self.app.config['camera']);intrinsics=json.loads(path.read_text());intrinsics['fx']=999.
        path.write_text(json.dumps(intrinsics))
        with self.assertRaisesRegex(ValueError,'冻结标定内参'):self.preview._render_camera('head',chosen,owner)
        self.contract['cameras']['left']=dict(self.contract['cameras']['left'],fx=999.)
        with self.assertRaisesRegex(ValueError,'标定依赖摘要'):
            self.preview.mesh(True,camera='left',candidate_id='wrist-a')

    def test_disabled_or_changed_context_discards_in_flight_render(self):
        entered=threading.Event();release=threading.Event();completed=threading.Event()
        render=self.preview._render
        def slow(*args):
            entered.set();release.wait(2)
            try:return render(*args)
            finally:completed.set()
        self.preview._render=slow
        self.preview.start();self.preview.mesh(True,camera='head',candidate_id='head-a')
        self.assertTrue(entered.wait(1));self.preview.bind_context('new-context');self.preview.mesh(False)
        release.set();self.assertTrue(completed.wait(1))
        self.assertEqual(self.preview.state()['frames'],{})
        with self.assertRaises(PendingFrame):self.preview.media('mesh')

    def test_owner_change_and_feedback_loss_expire_cached_image(self):
        self.preview.start();self.preview.mesh(True,camera='head',candidate_id='head-a')
        self.wait(lambda:bool(self.preview.state()['frames']))
        self.app.hardware_connected=lambda:False
        self.assertEqual(self.preview.state()['frames'],{})
        with self.assertRaises(PendingFrame):self.preview.media('mesh')
        self.app.hardware_connected=lambda:True
        self.owner['command_epoch']+=1
        self.assertEqual(self.preview.state()['frames'],{})
        self.wait(lambda:bool(self.preview.state()['frames']))
        self.assertEqual(self.preview.media('mesh')[1]['command_epoch'],3)

    def test_zero_distortion_rectified_flag_equivalence_retains_original_metadata(self):
        path=Path(self.app.config['camera']);frozen=json.loads(path.read_text());frozen['rectified']=True
        path.write_text(json.dumps(frozen));frozen_bytes=path.read_bytes()
        chosen=self.preview._candidates('head','head-a');owner=self.preview._owner()
        _,meta=self.preview._render_camera('head',chosen,owner)
        self.assertFalse(meta['source_camera_metadata']['intrinsics']['rectified'])
        self.assertTrue(meta['calibration_intrinsics']['rectified'])
        self.assertEqual(meta['intrinsics_equivalence'],'identical_zero_distortion_with_different_rectified_flag')
        self.assertEqual(path.read_bytes(),frozen_bytes)
        self.assertFalse(self.source_cameras['head']['rectified'])
        frozen['coefficients'][0]=self.source_cameras['head']['coefficients'][0]=.01
        path.write_text(json.dumps(frozen))
        with self.assertRaisesRegex(ValueError,'冻结标定内参'):
            self.preview._render_camera('head',chosen,owner)

    def test_rectified_wrist_calibration_projects_with_actual_raw_distortion(self):
        from arx_env.joint_head_workbench_overlay import real_mesh_overlay
        role='left';path=Path(self.app.config['hardware_wrist_camera_files'][role])
        frozen=json.loads(path.read_text());frozen['rectified']=True
        frozen['distortion_model']='distortion.inverse_brown_conrady'
        path.write_text(json.dumps(frozen));frozen_bytes=path.read_bytes()
        self.contract['cameras'][role]=deepcopy(frozen)
        self.wrist['result']['contract_sha256']=hashlib.sha256(json.dumps(self.contract,sort_keys=True,allow_nan=False).encode()).hexdigest()
        actual=self.source_cameras[role];actual['distortion_model']=frozen['distortion_model']
        actual['coefficients']=[-.05,.053,.001,.0008,-.017]
        chosen=self.preview._candidates(role,'wrist-a');owner=self.preview._owner()
        with patch('arx_env.joint_head_workbench_overlay.real_mesh_overlay',wraps=real_mesh_overlay) as render:
            _,meta=self.preview._render_camera(role,chosen,owner)
            self.assertEqual(render.call_args.args[-1],actual)
            self.assertEqual(render.call_args.args[-2]['id'],'wrist-a')
        self.assertEqual(meta['source_camera_metadata']['intrinsics'],actual)
        self.assertEqual(meta['calibration_intrinsics'],frozen)
        self.assertEqual(meta['projection_intrinsics'],actual)
        self.assertEqual(meta['projection_domain'],'actual_raw_sdk_intrinsics_from_rectified_calibration')
        self.assertEqual(meta['intrinsics_equivalence'],'different_distortion_domains')
        self.assertEqual(path.read_bytes(),frozen_bytes)
        for invalid in ([0.]*4,[float('nan'),0.,0.,0.,0.],[True,0.,0.,0.,0.]):
            actual['coefficients']=invalid
            with self.assertRaisesRegex(ValueError,'冻结标定内参'):
                self.preview._render_camera(role,chosen,owner)
        actual['coefficients']=[-.05,.053,.001,.0008,-.017];actual['fx']+=1
        with self.assertRaisesRegex(ValueError,'冻结标定内参'):
            self.preview._render_camera(role,chosen,owner)
        # A calibration originally fitted in the raw domain cannot substitute a
        # different set of distortion coefficients, even with identical K.
        self.source_cameras['head']['coefficients']=[.01,0.,0.,0.,0.]
        with self.assertRaisesRegex(ValueError,'冻结标定内参'):
            self.preview._render_camera('head',self.preview._candidates('head','head-a'),owner)


if __name__=='__main__':unittest.main()
