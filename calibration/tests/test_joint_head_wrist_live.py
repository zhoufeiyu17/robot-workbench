"""CPU contracts for three independent RTX camera products and frozen geometry."""
import copy
import hashlib
import http.client
import json
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from examples.scripts.debug.joint_head_workbench_live import Session, RTXRenderer, validate_wrist_contract, contract_sha256, simulation_wrist_mounts, serve


def contract():
    head = {'id':'frozen', 'camera_T_world_optical':np.eye(4).tolist(),
            'joint_offsets_rad':{'left':[.02]*6,'right':[-.01]*6},
            'marker_ids':{'left':24,'right':30},
            'T_link6_marker':{'24':np.eye(4).tolist(),'30':np.eye(4).tolist()}}
    head['snapshot_sha256'] = hashlib.sha256(json.dumps(head,sort_keys=True,allow_nan=False).encode()).hexdigest()
    head['snapshot_complete'] = True
    k = dict(width=32,height=24,fx=20.,fy=20.,cx=16.,cy=12.,rectified=True,coefficients=[0.]*5)
    return dict(head_snapshot=head,cameras={'left':k,'right':copy.deepcopy(k)},
                nominal_mounts={s:np.eye(4).tolist() for s in ('left','right')},
                planned_frames=[dict(id='planned-left',observer='left',target='right',split='fit')],
                table_z_m=0,table_clearance_m=.005)


class Geometry:
    velocity_limits = np.ones(12)
    limits = np.column_stack((-np.ones(12),np.ones(12)))
    k = dict(width=32,height=24,fx=20.,fy=20.,cx=16.,cy=12.)
    camera = np.eye(4)
    marker_size = .015
    config = {'untouched':True}
    checked = []
    def with_marker_mounts(self,mounts):
        view=copy.copy(self);view.mounts=mounts;return view
    def frames(self,q):return q
    def path_check(self,start,target):
        self.checked.append((self.frames(start).copy(),self.frames(target).copy(),dict(self.config)))
        return {'ok':True}


class WristLive(unittest.TestCase):
    def test_gpu_wrist_overlay_preserves_color_joints_jaw_mapping_and_cache(self):
        import cv2
        from unittest.mock import patch
        from arx_env.joint_head_jaw_mapping import feedback_gaps, FEEDBACK
        c=contract();c['cameras']['right']={**c['cameras']['right'],'fx':22.}
        calls=[];builds=[];seen=[];encoded=[]
        class FrozenGeometry(Geometry):
            def frames(self,q):
                seen.append(q.copy())
                left=np.eye(4);left[0,3]=.1
                right=np.eye(4);right[1,3]=.2
                return {'left':{'link6':left},'right':{'link6':right}}
        def build(view):
            builds.append(view)
            def render(image,raw,candidate,camera):
                calls.append((image.copy(),raw.copy(),candidate,camera))
                return image+1
            return SimpleNamespace(render=render)
        def encode(extension,image,quality):
            encoded.append(image.copy());return True,np.array([1,2,3],np.uint8)
        r=RTXRenderer.__new__(RTXRenderer);r.g=FrozenGeometry();r.wrist_geometry=r.g.with_marker_mounts({s:np.eye(4) for s in ('left','right')});r.wrist_contract=c
        r.cv2=SimpleNamespace(cvtColor=cv2.cvtColor,COLOR_RGB2BGR=cv2.COLOR_RGB2BGR,IMWRITE_JPEG_QUALITY=cv2.IMWRITE_JPEG_QUALITY,imencode=encode)
        pixels=np.zeros((24,32,3),np.uint8);pixels[:]=[10,30,90]
        q=np.arange(12)/100.;mount=np.eye(4);mount[2,3]=.04;candidate={'T_link6_camera':mount}
        with patch.dict('sys.modules',{'arx_env.joint_head_gpu_overlay':SimpleNamespace(GPUOverlay=build)}):
            self.assertEqual(r.wrist_calibration_overlay(pixels,q,[43.,63.],'left',candidate,c),b'\x01\x02\x03')
            image,raw,selected,camera=calls[-1]
            np.testing.assert_array_equal(image[0,0],[90,30,10])
            np.testing.assert_array_equal(encoded[-1][0,0],[91,31,11])
            np.testing.assert_allclose(raw[np.r_[0:6,7:13]],q)
            np.testing.assert_allclose(feedback_gaps(raw),[43.,63.])
            self.assertEqual(raw[6],FEEDBACK['left'][2]);self.assertEqual(raw[13],FEEDBACK['right'][3])
            np.testing.assert_allclose(seen[-1],q+np.r_[[.02]*6,[-.01]*6])
            self.assertEqual(selected['joint_offsets_rad'],c['head_snapshot']['joint_offsets_rad'])
            np.testing.assert_allclose(np.array(selected['camera_T_world_optical'])[:3,3],[.1,0.,.04])
            self.assertEqual(camera,c['cameras']['left'])
            r.wrist_calibration_overlay(pixels,q,[83.,0.],'right',candidate,c)
            self.assertEqual(len(builds),1,'Both wrists reuse the same frozen mesh GPU cache')
            self.assertEqual(calls[-1][3],c['cameras']['right'])
            np.testing.assert_allclose(feedback_gaps(calls[-1][1]),[83.,0.])
            altered=copy.deepcopy(c);altered['head_snapshot']['T_link6_marker']['24'][0][3]=.1
            r.wrist_calibration_overlay(pixels,q,[0.,0.],'left',candidate,altered)
            self.assertEqual(len(builds),2)
            self.assertEqual(builds[-1].mounts['left'][0,3],.1)
            r.g=FrozenGeometry()
            r.wrist_calibration_overlay(pixels,q,[0.,0.],'left',candidate,altered)
            self.assertEqual(len(builds),3,'Replacing base geometry invalidates GPU mesh data')
            r.wrist_contract=altered;r.g=FrozenGeometry();r.g.current_base_marker='new'
            r.wrist_calibration_overlay(pixels,q,[0.,0.],'left',candidate,altered)
            self.assertEqual(builds[-1].current_base_marker,'new','Matching contract must not reuse an old renderer geometry view')
        np.testing.assert_array_equal(pixels[0,0],[10,30,90])

    def test_pending_wrist_selection_is_identified_and_idempotent_during_render(self):
        with tempfile.TemporaryDirectory() as directory:
            s=Session(Geometry(),directory,{})
            s.status='READY';s.wrist_contract=contract()
            body=dict(id='slow-overlay',observer='left',T_link6_camera=np.eye(4).tolist(),
                      head_snapshot_sha256=s.wrist_contract['head_snapshot']['snapshot_sha256'],
                      contract_sha256=contract_sha256(s.wrist_contract))
            s.command('/wrist-calibration',body)
            pending=s.state()['wrist_calibration']['left']
            self.assertEqual(pending['id'],'slow-overlay');self.assertFalse(pending['ready'])
            revision=s.wrist_overlay_revision
            def render(*args):
                for _ in range(3):s.command('/wrist-calibration',body)
                self.assertEqual(s.wrist_overlay_revision,revision)
                return b'overlay'
            renderer=SimpleNamespace(wrist_calibration_overlay=render,encode_head=lambda image:b'original')
            s.update_wrist_calibration(renderer,{'left':b'pixels'},np.zeros(12),np.array([83.,83.]),
                                       copy.deepcopy(s.wrist_calibrations),s.wrist_contract,revision,12,123.)
            self.assertTrue(s.wrist_overlay_state['left']['ready'])
            s.command('/wrist-calibration',body)
            self.assertEqual(s.wrist_overlay_revision,revision)
            self.assertEqual(s.jpeg['left-calibration'],b'overlay')
            changed=copy.deepcopy(body);changed['T_link6_camera'][0][3]=.01
            s.command('/wrist-calibration',changed)
            self.assertEqual(s.wrist_overlay_revision,revision+1)
            self.assertFalse(s.wrist_overlay_state['left']['ready'])
            s.command('/wrist-calibration',{'observer':'left','clear':True})
            self.assertIsNone(s.wrist_overlay_state['left']['id'])
            cleared=s.wrist_overlay_revision
            s.command('/wrist-calibration',{'observer':'left','clear':True})
            self.assertEqual(s.wrist_overlay_revision,cleared)

    def test_hash_rigid_intrinsics_and_plan_are_validated(self):
        value = contract()
        frozen = validate_wrist_contract(value)
        value['head_snapshot']['joint_offsets_rad']['left'][0] = .8
        self.assertEqual(frozen['head_snapshot']['joint_offsets_rad']['left'][0],.02)
        with self.assertRaisesRegex(ValueError,'digest'):validate_wrist_contract(value)
        for mutate in (lambda c:c['nominal_mounts']['left'][0].__setitem__(0,2),
                       lambda c:c['cameras']['left'].__setitem__('rectified',False),
                       lambda c:c['cameras']['left'].__setitem__('fx',0),
                       lambda c:c['planned_frames'][0].__setitem__('target','left')):
            value = contract();mutate(value)
            with self.assertRaises(ValueError):validate_wrist_contract(value)

    def test_execute_checks_physical_offsets_and_emits_frozen_dataset(self):
        with tempfile.TemporaryDirectory() as directory:
            g=Geometry();g.checked=[]
            session=Session(g,Path(directory)/'measurements',{})
            session.status='READY'
            value=contract()
            class Renderer:
                def configure_wrist(self,value):self.contract=value
                def render(self,q,gaps):return b'jpeg',b'head'
                def simulation_truth(self,capture_id,value):
                    fixture={'schema':'x5-renderer-truth-v1','head':{'private_fixture_sentinel':.123}}
                    return {**fixture,'capture_id':capture_id,'fixture_sha256':contract_sha256(fixture)}
                def capture(self,image,q,row,output,index):
                    return dict(valid_for_fit=True,planned_id=row['planned_id'],observer=row['observer'],target=row['target'],
                                q_reported_rad=q.tolist(),frame_sequence=row['frame_sequence'],timestamp_s=row['timestamp_s'])
            session.command('/execute',dict(expected_start=[0.]*12,wrist_contract=value,
                waypoints=[dict(q=[0.]*12,planned_id='planned-left',observer='left',target='right',split='fit')]))
            value['head_snapshot']['joint_offsets_rad']['left'][0]=.9
            deadline=time.monotonic()+2
            while session.validation and time.monotonic()<deadline:time.sleep(.001)
            self.assertIsNotNone(session.pending,session.reason)
            np.testing.assert_allclose(g.checked[0][0],[.02]*6+[-.01]*6)
            self.assertEqual(g.checked[0][2]['table_z_m'],0)
            self.assertEqual(g.config,{'untouched':True})
            r=Renderer()
            for _ in range(6):session.tick(r,.1)
            self.assertEqual(session.dataset['wrist_contract']['head_snapshot']['joint_offsets_rad']['left'][0],.02)
            self.assertEqual(session.rows[0]['planned_id'],'planned-left')
            np.testing.assert_array_equal(session.rows[0]['q_reported_rad'],np.zeros(12))
            self.assertGreater(session.rows[0]['frame_sequence'],0)
            self.assertEqual(set(session.dataset['cameras']),{'head','left','right'})
            private=Path(directory)/'simulation-truth'/(session.capture_id+'.json')
            truth=json.loads(private.read_text())
            reference=session.dataset['simulation_truth']
            self.assertEqual(reference,dict(capture_id=session.capture_id,
                fixture_sha256=truth['fixture_sha256'],sha256=contract_sha256(truth)))
            self.assertEqual(json.loads((session.batch_dir/'dataset.json').read_text())['simulation_truth'],reference)
            self.assertFalse(any('truth' in p.name for p in session.output.rglob('*')))
            self.assertNotIn('private_fixture_sentinel',json.dumps(session.dataset))
            server=serve(session,'x'*40,0)
            try:
                for url in ('/artifacts/simulation-truth.json',
                            '/artifacts/../../simulation-truth/'+session.capture_id+'.json'):
                    connection=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=2)
                    connection.request('GET',url,headers={'Authorization':'Bearer '+'x'*40})
                    response=connection.getresponse();self.assertEqual(response.status,404)
                    response.read();connection.close()
            finally:server.shutdown();server.server_close()
            with self.assertRaisesRegex(ValueError,'snapshot mismatch'):
                session.command('/wrist-calibration',dict(observer='left',id='wrong',head_snapshot_sha256='wrong',T_link6_camera=np.eye(4).tolist()))

    def test_independent_images_are_saved_and_detected_in_same_capture(self):
        import cv2
        r=RTXRenderer.__new__(RTXRenderer)
        r.cv2=cv2;r.wrist_contract=contract()
        patterns=[np.full((24,32,3),v,np.uint8) for v in (10,30,50)]
        r.last_camera_images=dict(zip(('head','left','right'),patterns))
        r.detector=Mock()
        r.detector.detectMarkers.side_effect=[([],None,[]),([np.zeros((1,4,2))],np.array([[30]]),[]),([],None,[])]
        row=dict(arm=None,split='fit',planned_id='planned-left',observer='left',target='right',frame_sequence=17,timestamp_s=123.5)
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory)
            result=r.capture(patterns[0],np.zeros(12),row,out,0)
            self.assertTrue(result['valid_for_fit'])
            self.assertEqual(result['frame_sequence'],17)
            self.assertEqual(result['camera_detections']['left'],{'30':[[0.,0.]]*4})
            for side,pixels in r.last_camera_images.items():
                np.testing.assert_array_equal(cv2.imread(str(out/result['camera_images'][side])),pixels)
            self.assertEqual(json.loads((out/'frame-000.json').read_text()),result)
            s=Session(Geometry(),out/'session',{});s.dataset={};s.batch_dir=out;s.jpeg['left']=b'left-jpeg'
            s.jpeg['left-calibration']=b'overlay';s.wrist_overlay_state['left']={'id':'chosen'}
            server=serve(s,'x'*40,0)
            try:
                for url,status in [('/artifacts/frame-000-left.png',200),('/artifacts/../frame-000-left.png',404),('/frame.jpg?view=left',200),('/frame.jpg?view=left-calibration&candidate_id=old',409),('/frame.jpg?view=left-calibration&candidate_id=chosen',200)]:
                    c=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=2)
                    c.request('GET',url,headers={'Authorization':'Bearer '+'x'*40})
                    response=c.getresponse();self.assertEqual(response.status,status);response.read();c.close()
            finally:server.shutdown();server.server_close()

    def test_display_restore_queues_only_and_binds_full_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            s=Session(Geometry(),directory,{})
            s.status='READY';s.jpeg['left-calibration']=b'stale';s.jpeg['left']=b'stale-frame'
            c=contract();sha=contract_sha256(c)
            before=s.q.copy()
            self.assertEqual(s.command('/wrist-config',{'wrist_contract':c})['contract_sha256'],sha)
            self.assertIsNone(s.wrist_contract)
            self.assertIsNone(s.active)
            self.assertIsNone(s.capture_id)
            self.assertIsNone(s.dataset)
            self.assertNotIn('left-calibration',s.jpeg)
            self.assertNotIn('left',s.jpeg)
            result=dict(id='selected',observer='left',T_link6_camera=np.eye(4).tolist(),
                        head_snapshot_sha256=c['head_snapshot']['snapshot_sha256'],contract_sha256=sha)
            altered=copy.deepcopy(c);altered['nominal_mounts']['left'][0][3]=.1
            with self.assertRaisesRegex(ValueError,'full contract'):
                s.command('/wrist-calibration',{**result,'contract_sha256':contract_sha256(altered)})
            s.command('/wrist-calibration',result)
            self.assertEqual(s.wrist_overlay_state['left']['contract_sha256'],sha)
            r=SimpleNamespace(configure_wrist=Mock(),render=Mock(return_value=(b'jpeg',b'head')))
            s.tick(r,0)
            self.assertIsNone(s.pending_wrist_config)
            self.assertEqual(s.wrist_contract,c)
            self.assertEqual(s.state()['wrist_contract_sha256'],sha)
            np.testing.assert_array_equal(s.q,before)
            self.assertIsNone(s.active)
            self.assertIsNone(s.capture_id)
            self.assertFalse(s.wrist_overlay_state['left']['ready'])
            s.command('/wrist-calibration',dict(observer='left',clear=True))
            self.assertNotIn('left',s.wrist_calibrations)
            for state in ('STARTING','VALIDATING','EXECUTING','STOPPED'):
                s.status=state
                with self.assertRaisesRegex(ValueError,'idle READY'):s.command('/wrist-config',{'wrist_contract':c})
            s.status='READY'
            s.command('/wrist-config',{'wrist_contract':c})
            s.command('/mirror',dict(source_id='real',sequence=1,source_age_s=0,q=[0.]*12))
            self.assertIsNone(s.pending_wrist_config)
            s.command('/wrist-config',{'wrist_contract':c})
            s.command('/wrist-calibration',result)
            s.tick(r,0)
            self.assertEqual(s.status,'MIRRORING')
            self.assertEqual(s.state()['wrist_contract_sha256'],sha)
            self.assertEqual(s.wrist_overlay_state['left']['id'],'selected')
            self.assertIsNone(s.active)

    def test_real_overlay_accepts_wrist_resolution_and_rejects_mismatched_jpeg(self):
        import base64
        import cv2
        from unittest.mock import patch
        from examples.scripts.debug.joint_head_workbench_live import real_overlay_request
        pixels = np.full((12,16,3),80,np.uint8)
        _, jpeg = cv2.imencode('.jpg',pixels)
        camera = {**Geometry.k,'width':16,'height':12}
        candidate = contract()['head_snapshot']
        candidate.pop('T_link6_marker')
        body = dict(camera=camera,candidate=candidate,qpos=[0.]*14,jpeg_base64=base64.b64encode(jpeg).decode())
        gpu = Mock();gpu.render.return_value=pixels
        g=Geometry();g._real_gpu_overlay=gpu
        with patch.dict('sys.modules',{'arx_env.joint_head_gpu_overlay':SimpleNamespace(GPUOverlay=Mock())}):
            self.assertTrue(real_overlay_request(g,body).startswith(b'\xff\xd8'))
            self.assertEqual(gpu.render.call_args.args[3],camera)
            body['camera']={**camera,'width':32}
            with self.assertRaisesRegex(ValueError,'dimensions'):real_overlay_request(g,body)
            body['camera']={**camera,'fx':float('nan')}
            with self.assertRaisesRegex(ValueError,'intrinsics'):real_overlay_request(g,body)

    def test_candidate_does_not_change_actual_mesh_or_head_camera(self):
        class ProxyGeometry(Geometry):
            side_meshes={'left':[('camera_shell_proxy',np.zeros((3,3)),None)]}
            def with_marker_mounts(self,mounts):
                view=copy.copy(self)
                view.side_meshes={'left':[('camera_shell_proxy',np.ones((3,3))*.2,None)]}
                return view
        r=RTXRenderer.__new__(RTXRenderer);r.g=ProxyGeometry();r.wrist_contract=None
        r.wrist_geometry=r.g;r.simulation_wrist_mounts={s:np.eye(4) for s in ('left','right')}
        r.pose_cache={};r.last_camera_images={};r.stage=Mock()
        writes={}
        def mesh(stage,path):
            attribute=Mock();attribute.Set.side_effect=lambda points:writes.__setitem__(path,np.array(points))
            return SimpleNamespace(GetPointsAttr=lambda:attribute)
        r.UsdGeom=SimpleNamespace(Mesh=SimpleNamespace(Get=mesh),Xformable=lambda prim:SimpleNamespace(GetOrderedXformOps=lambda:[Mock()]))
        r.Vt=SimpleNamespace(Vec3fArray=SimpleNamespace(FromNumpy=lambda a:a))
        r.Gf=SimpleNamespace(Matrix4d=lambda *v:v)
        c=contract();r.wrist_camera_specs=c['cameras'];r.wrist_products={s:Mock() for s in ('left','right')}
        r.configure_wrist(c)
        self.assertIs(r.wrist_geometry,r.g)
        self.assertEqual(writes,{})
        r.stage.GetPrimAtPath.assert_not_called()
        np.testing.assert_array_equal(r.g.side_meshes['left'][0][1],np.zeros((3,3)))
        r.configure_wrist(None)
        self.assertIs(r.wrist_geometry,r.g)
        self.assertEqual(writes,{})
        r.simulation_wrist_mounts={}
        with self.assertRaisesRegex(ValueError,'fixed renderer wrist fixture'):
            r.configure_wrist(c)

    def test_real_overlay_proxy_cache_changes_only_with_frozen_mounts(self):
        import base64
        import cv2
        from unittest.mock import patch
        from examples.scripts.debug.joint_head_workbench_live import real_overlay_request
        pixels=np.zeros((24,32,3),np.uint8);_,jpeg=cv2.imencode('.jpg',pixels)
        g=Geometry();g._real_gpu_overlay=Mock();g._real_gpu_overlay.render.return_value=pixels
        builds=[]
        def gpu(view):
            builds.append(view)
            return SimpleNamespace(render=Mock(return_value=pixels))
        body=dict(camera=Geometry.k,candidate=contract()['head_snapshot'],qpos=[0.]*14,jpeg_base64=base64.b64encode(jpeg).decode())
        with patch.dict('sys.modules',{'arx_env.joint_head_gpu_overlay':SimpleNamespace(GPUOverlay=gpu)}):
            real_overlay_request(g,body);real_overlay_request(g,body)
            self.assertEqual(len(builds),1)
            self.assertIsNot(builds[0],g)
            body['candidate']['T_link6_marker']['24'][0][3]=.2
            real_overlay_request(g,body)
            self.assertEqual(len(builds),2)
            self.assertEqual(builds[1].mounts['left'][0,3],.2)
            body['candidate'].pop('T_link6_marker')
            real_overlay_request(g,body)
            g._real_gpu_overlay.render.assert_called_once()
            self.assertEqual(len(builds),2)

    def test_pose_offsets_and_stickers_do_not_mutate_nominal_geometry(self):
        r=RTXRenderer.__new__(RTXRenderer);r.wrist_contract=contract()
        r.simulation_joint_offsets=np.arange(12)*.001
        actual=np.eye(4);actual[0,3]=.123
        r.simulation_wrist_mounts=simulation_wrist_mounts({'wrist_simulation_mounts':{'left':actual.tolist()}})
        with self.assertRaises(ValueError):simulation_wrist_mounts({'wrist_simulation_mounts':{'left':np.zeros((4,4)).tolist()}})
        r.wrist_contract['T_unused']=None
        r.wrist_contract['nominal_mounts']['left'][0][3]=.1
        r.wrist_contract['head_snapshot']['T_link6_marker']['24'][1][3]=.2
        seen=[]
        def frames(q):
            seen.append(q.copy())
            return {s:{'link6':np.eye(4)} for s in ('left','right')}
        r.g=SimpleNamespace(frames=frames,mounts={s:np.eye(4) for s in ('left','right')})
        r.wrist_geometry=r.g
        r.Gf=SimpleNamespace(Matrix4d=lambda *a:np.asarray(a).reshape(4,4).T)
        r.pose_cache={'/World':(None,np.array([83.,83.]))};r.parts=[]
        tag=Mock();wrist=Mock();r.tags=[('left',tag)];r.wrist_camera_ops={'left':wrist}
        r.pose(np.zeros(12),[83.,83.])
        np.testing.assert_allclose(seen[0],r.simulation_joint_offsets)
        self.assertAlmostEqual(tag.Set.call_args.args[0][1,3],0.)
        self.assertAlmostEqual(wrist.Set.call_args.args[0][0,3],.123)
        self.assertEqual(r.wrist_contract['nominal_mounts']['left'][0][3],.1)
        self.assertNotIn('wrist_simulation_mounts',r.wrist_contract)
        np.testing.assert_array_equal(r.g.mounts['left'],np.eye(4))
        r.wrist_contract['head_snapshot']['joint_offsets_rad']['left'][0]=1.
        r.wrist_contract['head_snapshot']['T_link6_marker']['24'][1][3]=.4
        r.pose_cache.clear();r.pose_cache['/World']=(None,np.array([83.,83.]))
        r.pose(np.zeros(12),[83.,83.])
        np.testing.assert_array_equal(seen[-1],r.simulation_joint_offsets)
        self.assertEqual(tag.Set.call_args.args[0][1,3],0.)

    def test_truth_export_is_candidate_independent_and_detached(self):
        with tempfile.TemporaryDirectory() as directory:
            urdf=Path(directory)/'robot.urdf';urdf.write_text('<robot name="fixture"/>')
            r=RTXRenderer.__new__(RTXRenderer)
            r.g=SimpleNamespace(config={'urdf':str(urdf)},bases={s:np.eye(4) for s in ('left','right')},
                side_meshes={s:[('link6',np.zeros((3,3)),np.array([[0,1,2]]))] for s in ('left','right')},
                static_meshes=[],camera=np.eye(4),mounts={s:np.eye(4) for s in ('left','right')},
                k=copy.deepcopy(Geometry.k),marker_size=.015,hand={'nominal_gap_m':.06})
            r.simulation_joint_offsets=np.arange(12)*.001
            r.simulation_wrist_mounts={s:np.eye(4) for s in ('left','right')}
            c=contract();first=r.simulation_truth('capture-a',c)
            c['head_snapshot']['camera_T_world_optical'][0][3]=1.
            c['head_snapshot']['joint_offsets_rad']['left'][0]=1.
            c['head_snapshot']['T_link6_marker']['24'][0][3]=1.
            c['nominal_mounts']['left'][0][3]=1.
            self.assertEqual(first,r.simulation_truth('capture-a',c))
            second=r.simulation_truth('capture-b',c)
            self.assertEqual(first['fixture_sha256'],second['fixture_sha256'])
            self.assertNotEqual(contract_sha256(first),contract_sha256(second))
            self.assertEqual(first['head']['joint_offsets_rad']['left'],r.simulation_joint_offsets[:6].tolist())
            c['cameras']['left']['fx']=100.
            r.g.k['fx']=100.
            self.assertEqual(first['intrinsics']['left']['fx'],20.)
            self.assertEqual(first['intrinsics']['head']['fx'],20.)
            self.assertNotEqual(first['fixture_sha256'],r.simulation_truth('capture-a',c)['fixture_sha256'])


if __name__=='__main__':unittest.main()
