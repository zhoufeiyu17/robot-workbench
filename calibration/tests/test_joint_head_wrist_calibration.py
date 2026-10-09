import copy
import hashlib
import json
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from arx_env.joint_head_wrist_calibration import project_marker, solve_wrist_calibration, solve_wrist_dataset
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples/scripts/debug'))
from joint_head_wrist_plan import plan_wrist, wrist_visibility


class WristCalibrationTest(unittest.TestCase):
    def test_wrist_mesh_masks_and_contours_include_observer_and_static_occlusion(self):
        import tempfile
        from arx_env import joint_head_markerless as mesh
        from arx_env.joint_head_x5_hybrid import indexed_mesh
        from arx_env.joint_head_wrist_calibration import _wrist_mesh_evidence
        def plane(name, left, right, height, depth):
            vertices=np.array([[left,-height,depth],[right,-height,depth],
                               [right,height,depth],[left,height,depth]])
            return indexed_mesh(name,vertices,np.array([[0,1,2],[0,2,3]]))
        parts=[plane('right/link6/0',-.4,.4,.4,1.),
               plane('left/link5',-.25,-.1,.3,.5),
               plane('static/chassis',.1,.25,.3,.5)]
        frames={part['name']:np.eye(4) for part in parts};frames['left/link6']=np.eye(4)
        adapter=SimpleNamespace(meshes=parts,frames_with_feedback=lambda *args:frames)
        image=np.zeros((64,64,3),np.uint8);masks=[]
        def foreground(pixels,mask):
            masks.append(mask.copy())
            return mesh.edges_from_mask(pixels,mask)
        camera={'fx':50.,'fy':50.,'cx':32.,'cy':32.,'width':64,'height':64}
        with tempfile.NamedTemporaryFile() as rgb, \
                patch('arx_env.joint_head_x5_hybrid.X5MeshModel',return_value=adapter), \
                patch('arx_env.joint_head_wrist_calibration.cv2.imread',return_value=image), \
                patch.object(mesh,'foreground_edges',side_effect=foreground):
            evidence,_=_wrist_mesh_evidence(None,{},'left','right',camera,np.eye(4),np.zeros(12),rgb.name)
        self.assertEqual(masks[0][32,8],1,'Observer arm must seed the full RGB foreground')
        self.assertEqual(masks[0][32,56],1,'Static chassis must seed the full RGB foreground')
        points=np.asarray(evidence['points_link6'])
        np.testing.assert_allclose(points[:,2],1.,err_msg='Only target mesh may provide residual samples')
        projected_x=50*points[:,0]/points[:,2]+32
        self.assertTrue(np.all((projected_x>18)&(projected_x<46)),
                        'Target silhouettes hidden by either occluder must be excluded')

    def test_arm_mesh_scope_excludes_gripper_samples_but_keeps_occlusion(self):
        import tempfile
        from arx_env import joint_head_markerless as mesh
        from arx_env.joint_head_wrist_calibration import _wrist_mesh_evidence
        adapter = SimpleNamespace(meshes=[dict(name='right/link6/0', v=np.zeros((3,3)), f=np.array([[0,1,2]])),
                                          dict(name='right/link5', v=np.ones((3,3)), f=np.array([[0,1,2]]))],
                                  frames_with_feedback=lambda *args: {'left/link6':np.eye(4),
                                      'right/link6/0':np.eye(4), 'right/link5':np.eye(4)})
        contour = dict(points=np.tile([0.,0.,1.],(8,1)), normals=np.tile([1.,0.],(8,1)))
        edge = dict(points=np.zeros((20,2)), normals=np.tile([1.,0.],(20,1)),
                    tree=SimpleNamespace(query=lambda uv,k: (np.zeros(8),np.zeros(8,dtype=int))))
        with tempfile.NamedTemporaryFile() as rgb, \
                patch('arx_env.joint_head_x5_hybrid.X5MeshModel',return_value=adapter), \
                patch('arx_env.joint_head_wrist_calibration.cv2.imread',return_value=np.zeros((64,64,3),np.uint8)), \
                patch.object(mesh,'camera_mesh_parts'), patch.object(mesh,'raster_depth',return_value=np.zeros((64,64))), \
                patch.object(mesh,'foreground_edges',return_value=edge), \
                patch.object(mesh,'contours',return_value=contour) as contours, \
                patch.object(mesh,'match',return_value=(np.zeros((8,2)),np.ones(8,dtype=bool))):
            evidence,_ = _wrist_mesh_evidence(None,{},'left','right',
                dict(fx=50.,fy=50.,cx=32.,cy=32.,width=64,height=64),np.eye(4),np.zeros(12),rgb.name,mesh_scope='arm')
        import re
        args=contours.call_args.args
        self.assertEqual({m['name'] for m in args[3]}, {'target/link6/0','target/link5'})
        for name in ('link6/0', 'base_link', 'camera_body', 'camera_shell_proxy', 'bracket_proxy'):
            self.assertIsNone(re.match(args[6], 'target/'+name))
        self.assertIsNotNone(re.match(args[6], 'target/link5'))
        self.assertEqual(evidence['mesh_scope'],'arm')

    def test_paper_sightlines_use_full_static_and_arm_meshes(self):
        from joint_head_wrist_plan import marker_occluder
        from arx_env.joint_head_workbench_geometry import box_triangles
        optical=np.eye(4);marker=np.eye(4);marker[2,3]=.2
        obstacle=np.eye(4);obstacle[2,3]=.1
        vertices=box_triangles([.04,.04,.01],obstacle)
        geometry=SimpleNamespace(marker_size=.015,static_meshes=[('mast',vertices,np.empty((0,3),int))])
        self.assertEqual(marker_occluder(geometry,{},optical,marker),'environment/mast')
        geometry.static_meshes=[];geometry.side_meshes={'left':[('link5',vertices,np.empty((0,3),int))]}
        frames={'left':{'link5':np.eye(4)}}
        self.assertEqual(marker_occluder(geometry,frames,optical,marker),'left/link5')
        frames['left']['link5'][0,3]=.1
        self.assertIsNone(marker_occluder(geometry,frames,optical,marker))
        frames['left']['link5'][0,3]=0;frames['left']['link5'][2,3]=.2
        self.assertIsNone(marker_occluder(geometry,frames,optical,marker),'Object behind paper must not occlude it')

    def setUp(self):
        self.camera = dict(fx=630., fy=628., cx=320., cy=240.)
        self.truth = np.eye(4)
        self.truth[:3, :3] = Rotation.from_euler('xyz', [.08, -.12, .04]).as_matrix()
        self.truth[:3, 3] = [.025, -.018, .043]
        self.rows = []
        rng = np.random.default_rng(128)
        for i in range(14):
            observer = np.eye(4)
            observer[:3, :3] = Rotation.from_rotvec(rng.normal(0, .3, 3)).as_matrix()
            observer[:3, 3] = rng.normal(0, .2, 3)
            relative = np.eye(4)
            relative[:3, :3] = Rotation.from_rotvec(rng.normal(0, .3, 3)).as_matrix()
            relative[:3, 3] = [rng.uniform(-.14, .14), rng.uniform(-.1, .1), rng.uniform(.3, .65)]
            marker = observer @ self.truth @ relative
            uv, _ = project_marker(self.truth, observer, marker, self.camera, .045)
            self.rows.append(dict(id=str(i), split='holdout' if i >= 10 else 'fit',
                                  T_world_observer_link6=observer.tolist(),
                                  T_world_marker=marker.tolist(), corners_uv=uv.tolist()))
        self.initial = self.truth.copy()
        self.initial[:3, 3] += [.012, -.007, .009]
        self.initial[:3, :3] = self.truth[:3, :3] @ Rotation.from_rotvec([.06, -.04, .03]).as_matrix()

    def solve(self, rows=None):
        return solve_wrist_calibration(self.rows if rows is None else rows, self.camera, .045,
                                      self.initial, source_calibration_id='frozen-head-fit-123')

    def test_recovers_transform_and_does_not_fit_holdout(self):
        original = copy.deepcopy(self.rows)
        result = self.solve()
        np.testing.assert_allclose(result['T_link6_camera'], self.truth, atol=1e-8)
        self.assertTrue(result['numerically_valid'])
        self.assertEqual(result['jacobian_rank'], 6)
        self.assertLess(result['holdout']['rms_px'], 1e-6)
        self.assertFalse(result['physical_validated'])
        self.assertEqual(self.rows, original)
        damaged = copy.deepcopy(self.rows)
        for row in damaged[10:]:
            row['corners_uv'] = (np.array(row['corners_uv']) + [12., 0.]).tolist()
        second = self.solve(damaged)
        np.testing.assert_allclose(second['T_link6_camera'], result['T_link6_camera'], atol=1e-12)
        self.assertAlmostEqual(second['holdout']['rms_px'], 12., places=5)
        self.assertNotEqual(second['input_sha256'], result['input_sha256'])

    def test_noisy_pixels_recover_mount_and_projection_rejects_behind(self):
        noisy = copy.deepcopy(self.rows)
        rng = np.random.default_rng(772)
        for row in noisy:
            row['corners_uv'] = (np.array(row['corners_uv']) + rng.normal(0, .12, (4, 2))).tolist()
        result = self.solve(noisy)
        recovered = np.array(result['T_link6_camera'])
        self.assertLess(np.linalg.norm(recovered[:3, 3] - self.truth[:3, 3]), .001)
        self.assertLess(Rotation.from_matrix(recovered[:3, :3].T @ self.truth[:3, :3]).magnitude(), .003)
        self.assertLess(result['holdout']['rms_px'], .4)
        behind = np.eye(4)
        behind[2, 3] = -1
        with self.assertRaises(ValueError):
            project_marker(np.eye(4), np.eye(4), behind, self.camera, .045)

    def test_observability_and_invalid_inputs(self):
        rows = copy.deepcopy(self.rows)
        # Collapse all four corners to one 3D point: translation along viewing
        # ray and rotations become ambiguous; rank must not report a valid fit.
        for row in rows:
            row['T_world_observer_link6'] = np.eye(4).tolist()
            row['T_world_marker'] = np.eye(4).tolist()
            row['T_world_marker'][2][3] = 1.
            row['corners_uv'] = [[320., 240.]] * 4
        tiny = solve_wrist_calibration(rows, self.camera, 1e-12, np.eye(4),
                                      source_calibration_id='degenerate')
        self.assertLess(tiny['jacobian_rank'], 6)
        self.assertFalse(tiny['numerically_valid'])
        for change in ('split', 'id', 'corners_uv', 'T_world_marker'):
            bad = copy.deepcopy(self.rows)
            bad[0][change] = {'split': 'test', 'id': '1', 'corners_uv': [[float('nan'), 0]] * 4,
                              'T_world_marker': np.zeros((4, 4)).tolist()}[change]
            with self.assertRaises(ValueError):
                self.solve(bad)
        with self.assertRaises(ValueError):
            self.solve(self.rows[:10])

    def dataset_case(self):
        offset=np.arange(12)*.001+.02
        mount=np.eye(4);mount[:3,3]=[.01,.02,.03]
        snapshot={'id':'frozen-head','camera_T_world_optical':np.eye(4).tolist(),
                  'joint_offsets_rad':{'left':offset[:6].tolist(),'right':offset[6:].tolist()},
                  'marker_ids':{'left':24,'right':30},'T_link6_marker':{'24':mount.tolist(),'30':mount.tolist()}}
        snapshot['snapshot_sha256']=hashlib.sha256(json.dumps(snapshot,sort_keys=True,allow_nan=False).encode()).hexdigest()
        camera={**self.camera,'width':640,'height':480,'rectified':True,'coefficients':[0.]*5}
        initial={side:self.initial.tolist() for side in ('left','right')}
        planned=[];frames=[];expected={}
        for side,target in [('left','right'),('right','left')]:
            for index,row in enumerate(self.rows):
                identity=side+'-'+row['id'];q=np.zeros(12);q[0]=len(frames)
                planned.append(dict(id=identity,observer=side,target=target,split=row['split']))
                frames.append(dict(id='frame-'+str(len(frames)),planned_id=identity,observer=side,target=target,
                                   split=row['split'],q_reported_rad=q.tolist(),camera_detections={side:{str(snapshot['marker_ids'][target]):row['corners_uv']}}))
                expected[q[0]]=(q+offset,{side:{'link6':np.array(row['T_world_observer_link6'])},
                                         target:{'link6':np.array(row['T_world_marker'])@np.linalg.inv(mount)}})
        def fk(q):
            wanted,result=expected[round(q[0]-offset[0])]
            np.testing.assert_allclose(q,wanted,atol=1e-12)
            return result
        geometry=SimpleNamespace(marker_size=.045,frames=fk)
        dataset={'wrist_contract':{'head_snapshot':snapshot,'cameras':{side:camera for side in initial},
                                  'nominal_mounts':initial,'planned_frames':planned},'frames':frames}
        return dataset,geometry,initial

    def test_progress_uses_real_frame_side_and_iteration_completion(self):
        dataset,geometry,initial=self.dataset_case()
        events=[]
        actual=solve_wrist_dataset(dataset,geometry,initial,progress=events.append)
        expected=solve_wrist_dataset(dataset,geometry,initial)
        self.assertEqual(actual,expected)
        fractions=[event['fraction'] for event in events]
        self.assertEqual(fractions,sorted(fractions))
        self.assertEqual((fractions[0],fractions[-1]),(0.,1.))
        frames=[event for event in events if event['label']=='wrist_rgb_evidence']
        self.assertEqual(frames[-1]['completed'],len(dataset['frames']))
        self.assertTrue(any(event['label']=='left_wrist_optimizer' for event in events))
        self.assertTrue(any(event['label']=='right_wrist_optimizer' for event in events))

    def test_hardware_environment_uses_each_observer_feedback(self):
        dataset,geometry,initial=self.dataset_case()
        dataset['wrist_contract']['environment']='hardware'
        for row in dataset['frames']:
            row['camera_feedback']={row['observer']:{'q_reported12':row['q_reported_rad']}}
            row['q_reported_rad']=[999.]*12
        reports=solve_wrist_dataset(dataset,geometry,initial)
        for report in reports.values():
            self.assertEqual(report['environment'],'hardware')
            self.assertFalse(report['simulation_only'])
            self.assertFalse(report['physical_validated'])
            self.assertTrue(report['accepted'])

    def test_refreshed_mesh_has_fixed_size_and_penalizes_missing_edges(self):
        from scipy.spatial import cKDTree
        from arx_env.joint_head_wrist_calibration import _refresh_wrist_mesh_evidence, mesh_observation_residual
        points=np.c_[np.linspace(0,.01,8),np.zeros(8),np.ones(8)]
        context={'points_link6':points,'normals':np.tile([1.,0.],(8,1)),
                 'edge_points':np.zeros((1,2)),'tree':cKDTree(np.zeros((1,2))),
                 'camera':{'fx':100.,'fy':100.,'cx':0.,'cy':0.}}
        T=np.eye(4)
        first=_refresh_wrist_mesh_evidence(T,context)
        T[0,3]=1.
        last=_refresh_wrist_mesh_evidence(T,context)
        self.assertEqual(len(first['points_link6']),len(last['points_link6']))
        self.assertEqual(last['coverage'],0.)
        residual=mesh_observation_residual(T,{'mesh_observation':last},context['camera'])
        np.testing.assert_allclose(residual,20.)
        np.testing.assert_allclose(mesh_observation_residual(np.eye(4),{'mesh_observation':last},context['camera']),20.)
        legacy={key:value for key,value in first.items() if key!='valid'}
        np.testing.assert_allclose(mesh_observation_residual(np.eye(4),{'mesh_observation':legacy},context['camera']),np.linspace(0,1,8))
        # A radius crossing can jump by 20 px under a 1 nm parameter perturbation.
        context.update(points_link6=np.tile([0.,0.,1.],(8,1)),normals=np.tile([0.,1.],(8,1)),
                       edge_points=np.array([[-20.,0.]]),tree=cKDTree([[-20.,0.]]))
        frozen=_refresh_wrist_mesh_evidence(np.eye(4),context)
        T=np.eye(4);T[0,3]=-1e-9
        refreshed=_refresh_wrist_mesh_evidence(T,context)
        np.testing.assert_allclose(mesh_observation_residual(T,{'mesh_observation':refreshed},context['camera']),20.)
        np.testing.assert_allclose(mesh_observation_residual(T,{'mesh_observation':frozen},context['camera']),0.)
        with self.assertRaisesRegex(ValueError,'one boolean per point'):
            mesh_observation_residual(T,{'mesh_observation':{**frozen,'valid':[1]*8}},context['camera'])

    def test_mesh_icp_freezes_fit_correspondences_and_scores_holdout_only_after_solving(self):
        from scipy.spatial import cKDTree
        from arx_env import joint_head_wrist_calibration as module
        rows=copy.deepcopy(self.rows);contexts={}
        corners=module.marker_corners(.045)
        for row in rows:
            relative=np.linalg.inv(row['T_world_observer_link6'])@row['T_world_marker']
            points=corners@relative[:3,:3].T+relative[:3,3]
            edges=np.asarray(row['corners_uv'])
            contexts[row['id']]={'points_link6':np.tile(points,(2,1)),
                'normals':np.tile([1.,0.],(8,1)), 'edge_points':edges,
                'tree':cKDTree(edges),'camera':self.camera}
            row['mesh_observation']=module._refresh_wrist_mesh_evidence(self.initial,contexts[row['id']])
        original=copy.deepcopy(rows);inside=False;solves=[];calls=[];heldout_shift=0.
        solve=module.least_squares
        def inner(*args,**kwargs):
            nonlocal inside
            inside=True
            try:
                result=solve(*args,**kwargs);solves.append(result)
                return result
            finally:inside=False
        def builder(T,row):
            self.assertFalse(inside,'Correspondence refresh must not run inside LS/Jacobian evaluation')
            if row['split']=='holdout':self.assertTrue(solves,'Holdout must wait for final scoring')
            calls.append(row['split'])
            evidence=module._refresh_wrist_mesh_evidence(T,contexts[row['id']])
            if row['split']=='holdout':
                evidence['observed_uv']=(np.asarray(evidence['observed_uv'])+[heldout_shift,0.]).tolist()
            return evidence
        def run():
            with patch.object(module,'least_squares',side_effect=inner):
                return solve_wrist_calibration(rows,self.camera,.045,self.initial,
                    source_calibration_id='frozen-head-fit-123',mesh_residual=lambda T,row:module.mesh_observation_residual(T,row,self.camera),
                    mesh_observation_builder=builder)
        report=run()
        self.assertEqual(rows,original)
        self.assertLessEqual(report['correspondence_iterations'],8)
        self.assertEqual(calls.count('fit'),10*(report['correspondence_iterations']+2))
        self.assertEqual(calls.count('holdout'),8)  # Final hybrid and corner-only comparison scores.
        np.testing.assert_allclose(report['jacobian_singular_values'],np.linalg.svd(solves[-2].jac,compute_uv=False))
        np.testing.assert_allclose(report['T_link6_camera'],self.truth,atol=1e-8)
        heldout_shift=100.
        second=run()
        np.testing.assert_allclose(second['T_link6_camera'],report['T_link6_camera'],atol=1e-12)
        self.assertGreater(second['holdout']['mesh_rms_px'],report['holdout']['mesh_rms_px']+50.)

    def test_partial_capture_requires_explicit_authorization_and_preserves_contract(self):
        dataset,geometry,initial=self.dataset_case()
        dataset['frames'].pop(0)
        before=copy.deepcopy(dataset)
        with self.assertRaisesRegex(ValueError, 'incomplete planned'):
            solve_wrist_dataset(dataset,geometry,initial)
        reports=solve_wrist_dataset(dataset,geometry,initial,allow_partial=True,
                                    source_completion={'passed':False})
        self.assertEqual(dataset,before)
        for report in reports.values():
            self.assertTrue(report['accepted'])
            self.assertFalse(report['coverage_complete'])
            self.assertTrue(report['partial_capture_authorized'])
            self.assertTrue(report['capture_coverage']['partial'])
            self.assertFalse(report['capture_coverage']['source_completion_passed'])
            self.assertEqual(len(report['capture_coverage']['missing_ids']),1)
        for mutate in (lambda d:d['frames'][0].update(valid_for_fit=False),
                       lambda d:d['frames'].append(copy.deepcopy(d['frames'][0])),
                       lambda d:d['frames'][0]['camera_detections'].clear()):
            bad=copy.deepcopy(dataset);mutate(bad)
            with self.assertRaises(ValueError):
                solve_wrist_dataset(bad,geometry,initial,allow_partial=True)

    def test_dataset_full_coverage_snapshot_and_offsets_once(self):
        dataset,geometry,initial=self.dataset_case();before=copy.deepcopy(dataset)
        reports=solve_wrist_dataset(dataset,geometry,initial)
        for side,result in reports.items():
            self.assertTrue(result['accepted']);self.assertTrue(result['coverage_complete'])
            self.assertTrue(result['simulation_only']);self.assertFalse(result['physical_validated'])
            self.assertEqual(result['head_snapshot_sha256'],dataset['wrist_contract']['head_snapshot']['snapshot_sha256'])
            np.testing.assert_allclose(result['T_link6_camera'],self.truth,atol=1e-8)
        self.assertEqual(dataset,before)
        for row in dataset['frames']:
            if row['observer']=='left' and row['split']=='holdout':
                row['camera_detections']['left']['30']=(np.array(row['camera_detections']['left']['30'])+[3.,0.]).tolist()
        changed=solve_wrist_dataset(dataset,geometry,initial)
        self.assertFalse(changed['left']['accepted']);self.assertTrue(changed['right']['accepted'])
        np.testing.assert_allclose(changed['left']['T_link6_camera'],reports['left']['T_link6_camera'],atol=1e-12)

    def test_corners_only_declared_three_pixel_gate(self):
        dataset,geometry,initial=self.dataset_case()
        dataset['wrist_contract'].update(solver='corners',estimate_marker_mount=False,
                                        acceptance={'max_holdout_rms_px':3.})
        for shift,accepted in ((2.99,True),(3.01,False)):
            sample=copy.deepcopy(dataset)
            for row in sample['frames']:
                if row['observer']=='left' and row['split']=='holdout':
                    row['camera_detections']['left']['30']=(np.array(row['camera_detections']['left']['30'])+[shift,0.]).tolist()
            reports=solve_wrist_dataset(sample,geometry,initial)
            self.assertEqual(reports['left']['accepted'],accepted)
            self.assertTrue(reports['right']['accepted'])
            for report in reports.values():
                self.assertEqual(report['objective'],'sticker_corners')
                self.assertFalse(report['marker_mount_estimated'])
                self.assertEqual(report['acceptance'],{'max_holdout_rms_px':3.})

    def test_dataset_rejects_missing_holdout_detection_split_and_bad_camera(self):
        dataset,geometry,initial=self.dataset_case()
        for mutate in (
            lambda d:d['frames'].pop(),
            lambda d:d['frames'][-1]['camera_detections'].clear(),
            lambda d:d['frames'][-1].update(split='fit'),
            lambda d:d['frames'].append(copy.deepcopy(d['frames'][0])),
            lambda d:d['wrist_contract']['cameras']['left'].update(rectified=False),
            lambda d:d['wrist_contract']['cameras']['left'].update(coefficients=[.01,0,0,0,0]),
            lambda d:d['wrist_contract']['head_snapshot']['joint_offsets_rad']['left'].__setitem__(0,.9),
        ):
            bad=copy.deepcopy(dataset);mutate(bad)
            with self.assertRaises(ValueError):solve_wrist_dataset(bad,geometry,initial)

    def test_planner_progress_reports_each_side_without_changing_sampling(self):
        geometry=SimpleNamespace(limits=np.tile([-3.,3.],(12,1)),config={'margin_px':18},marker_size=.015,
            k=dict(fx=630.,fy=630.,cx=320.,cy=240.,width=640,height=480),
            mounts={side:np.eye(4) for side in ('left','right')},
            frames=lambda q:{side:{'link6':np.eye(4)} for side in ('left','right')},
            path_check=lambda start,end:{'ok':True})
        optical=np.eye(4);optical[2,3]=-.15;mounts={s:optical.tolist() for s in ('left','right')}
        events=[]
        with patch('joint_head_wrist_plan.least_squares',return_value=SimpleNamespace(x=np.zeros(12))):
            result=plan_wrist(geometry,mounts,poses_per_side=6,progress=events.append)
            baseline=plan_wrist(geometry,mounts,poses_per_side=6)
        self.assertEqual(result,baseline)
        self.assertTrue(all(row['jaw_gap_mm']==[80.,80.] for row in result['waypoints']))
        self.assertEqual(len(result['planned_frames']),12)
        captures=[row for row in result['waypoints'] if row.get('capture') is not False]
        self.assertEqual([row['id'] for row in captures],[row['id'] for row in result['planned_frames']])
        returns=[row for row in result['waypoints'] if '-return' in row['id']]
        self.assertEqual(len(returns),2)
        self.assertTrue(all(row['capture'] is False for row in returns))
        for side in ('left','right'):
            planned=[row for row in result['planned_frames'] if row['observer']==side]
            self.assertEqual(len(planned),6)
            self.assertEqual(sum(row['split']=='fit' for row in planned),4)
            self.assertEqual([row['id'] for row in planned if row['split']=='holdout'],[side+'-2',side+'-5'])
            side_events=[row for row in events if row['observer']==side]
            self.assertEqual(side_events[0]['accepted'],0)
            self.assertEqual(side_events[-1]['accepted'],6)
            self.assertEqual(side_events[-1]['target'],6)
            self.assertEqual([row['attempt'] for row in side_events],sorted(row['attempt'] for row in side_events))
            self.assertEqual({row['attempt'] for row in side_events},set(range(1,side_events[-1]['attempt']+1)))
            self.assertTrue(all(isinstance(row['rejection_counts'],dict) for row in side_events))

    def test_visibility_records_actual_marker_pose_in_optical_frame(self):
        row=self.rows[0];observer=np.array(row['T_world_observer_link6'])
        marker=np.array(row['T_world_marker']);mount=np.eye(4);mount[:3,3]=[.02,-.01,.03]
        geometry=SimpleNamespace(config={},marker_size=.045,mounts={'right':mount},
            frames=lambda q:{'left':{'link6':observer},'right':{'link6':marker@np.linalg.inv(mount)}})
        camera={**self.camera,'width':640,'height':480}
        visibility=wrist_visibility(geometry,np.zeros(12),'left','right',self.truth,camera)
        relative=np.linalg.inv(observer@self.truth)@marker
        np.testing.assert_allclose(visibility['marker_position_camera_m'],relative[:3,3],atol=1e-12)
        np.testing.assert_allclose(visibility['marker_normal_camera'],relative[:3,2],atol=1e-12)

    def test_planner_random_restarts_and_checked_via_start_preserve_32_samples(self):
        class Geometry:
            limits=np.tile([-3.,3.],(12,1));config={'margin_px':18};marker_size=.015
            k=dict(fx=630.,fy=630.,cx=320.,cy=240.,width=640,height=480)
            mounts={side:np.eye(4) for side in ('left','right')}
            def __init__(self):self.paths=[];self.block=False
            def frames(self,q):return {side:{'link6':np.eye(4)} for side in ('left','right')}
            def path_check(self,start,end):
                a,b=float(start[0]),float(end[0]);self.paths.append((a,b))
                ok=(a==b or a==0 or b==0) and not (self.block and b>0)
                return {'ok':ok,'collisions':[] if ok else [['left/link6','environment/chassis']]}
        class RNG:
            def uniform(self,low,high,size=None):
                if isinstance(low,np.ndarray):return np.r_[np.ones(11)*.8,4.]
                if size:return np.zeros(size)
                return (low+high)/2
            def normal(self,mean,sigma,size):return np.ones(size)*.15
        geometry=Geometry();optical=np.eye(4);optical[2,3]=-.16
        starts=[];references=[]
        def fitted(fun,start,**kwargs):
            starts.append(start.copy());references.append(-fun(np.zeros(12))[-12:]/.002)
            np.testing.assert_allclose(fun(references[-1])[-12:],0.,atol=1e-15)
            return SimpleNamespace(x=np.ones(12)*(len(starts)%16+1)*.01)
        with patch('joint_head_wrist_plan.np.random.default_rng',return_value=RNG()),patch('joint_head_wrist_plan.least_squares',side_effect=fitted):
            plan=plan_wrist(geometry,{side:optical.tolist() for side in ('left','right')})
        self.assertEqual(len(plan['planned_frames']),32)
        self.assertEqual(sum(row['split']=='holdout' for row in plan['planned_frames']),8)
        via=[row for row in plan['waypoints'] if '-via-start-' in row['id']]
        self.assertEqual(len(via),30)
        self.assertTrue(all(row['capture'] is False and row['path_check']['ok'] for row in via))
        self.assertEqual(len({row['id'] for row in plan['waypoints']}),len(plan['waypoints']))
        for index in range(32):
            if index%8==0:
                np.testing.assert_allclose(starts[index],np.r_[np.ones(11)*.8,3.-1e-6])
                np.testing.assert_allclose(references[index],starts[index])
            else:
                np.testing.assert_allclose(references[index],(index%16+1)*.01)
                np.testing.assert_allclose(starts[index],references[index]+.15)
        for row in plan['waypoints']:
            if row.get('capture') is False:continue
            np.testing.assert_allclose(row['visibility']['marker_position_camera_m'],[0.,0.,.16])
            np.testing.assert_allclose(row['visibility']['marker_normal_camera'],[0.,0.,1.])
        current=np.zeros(12)
        for row in plan['waypoints']:
            self.assertTrue(geometry.path_check(current,row['q'])['ok'])
            current=np.array(row['q'])
        self.assertIn('collision: left/link6 / environment/chassis',plan['rejection_counts']['left'])
        geometry.block=True
        with patch('joint_head_wrist_plan.np.random.default_rng',return_value=RNG()),patch('joint_head_wrist_plan.least_squares',side_effect=fitted):
            with self.assertRaisesRegex(ValueError,'only 0/16.*rejection_counts=.*environment/chassis'):
                plan_wrist(geometry,{side:optical.tolist() for side in ('left','right')})

    def test_planner_rejects_endpoint_before_paths_and_caches_return_until_accept(self):
        class Geometry:
            limits=np.tile([-3.,3.],(12,1));config={'margin_px':18};marker_size=.015
            k=dict(fx=630.,fy=630.,cx=320.,cy=240.,width=640,height=480)
            mounts={side:np.eye(4) for side in ('left','right')}
            def __init__(self):self.paths=[]
            def frames(self,q):return {side:{'link6':np.eye(4)} for side in ('left','right')}
            def path_check(self,start,end):
                a,b=round(float(start[0]),2),round(float(end[0]),2);self.paths.append((a,b))
                ok=not (b==.02 or (a!=b and b==.03) or (a!=b and a>0 and b>.03))
                return {'ok':ok,'collisions':[] if ok else [['left/link6','environment/chassis']]}
        class RNG:
            def uniform(self,low,high,size=None):
                if isinstance(low,np.ndarray):return np.zeros(12)
                if size:return np.zeros(size)
                return (low+high)/2
            def normal(self,mean,sigma,size):return np.zeros(size)
        geometry=Geometry();optical=np.eye(4);optical[2,3]=-.16
        candidates=iter([.01,.02,.03,.03,.04,.05,.06]*2)
        with patch('joint_head_wrist_plan.np.random.default_rng',return_value=RNG()),patch('joint_head_wrist_plan.least_squares',side_effect=lambda *a,**kw:SimpleNamespace(x=np.ones(12)*next(candidates))):
            plan=plan_wrist(geometry,{side:optical.tolist() for side in ('left','right')},poses_per_side=4)
        self.assertEqual(len(plan['planned_frames']),8)
        self.assertEqual([pair for pair in geometry.paths if .02 in pair],[(.02,.02)]*2)
        self.assertEqual(geometry.paths.count((.01,0.)),2,'Repeated failed candidates reuse one return check per current pose and side')
        self.assertEqual(geometry.paths.count((.04,0.)),2,'Accepted pose invalidates the return cache')
        self.assertTrue(any(row.get('stage')=='endpoint' for row in plan['rejected']))

    def test_planner_reverse_return_uses_checked_route_without_recapture(self):
        class Geometry:
            limits=np.tile([-3.,3.],(12,1));config={'margin_px':18};marker_size=.015
            k=dict(fx=630.,fy=630.,cx=320.,cy=240.,width=640,height=480)
            mounts={side:np.eye(4) for side in ('left','right')}
            def __init__(self):self.paths=[];self.reverse_block=False;self.captures=[0]
            def frames(self,q):return {side:{'link6':np.eye(4)} for side in ('left','right')}
            def path_check(self,start,end):
                a,b=round(float(start[0]),2),round(float(end[0]),2)
                self.paths.append((a,b))
                if b>a:self.captures[0]+=1
                return {'ok':(a,b)!=(.04,0.) and not (self.reverse_block and (a,b)==(.03,.02))}
        geometry=Geometry();optical=np.eye(4);optical[2,3]=-.15
        def fitted(*args,**kwargs):
            return SimpleNamespace(x=np.ones(12)*(geometry.captures[0]%4+1)*.01)
        with patch('joint_head_wrist_plan.least_squares',side_effect=fitted):
            plan=plan_wrist(geometry,{side:optical.tolist() for side in ('left','right')},poses_per_side=4)
        for side in ('left','right'):
            returns=[row for row in plan['waypoints'] if row['id'].startswith(side+'-return')]
            self.assertEqual([round(row['q'][0],2) for row in returns],[.03,.02,.01,0.])
            self.assertTrue(all(row['capture'] is False and row['path_check']['ok'] for row in returns))
        self.assertEqual(len(plan['planned_frames']),8)
        self.assertEqual(sum(row.get('capture') is not False for row in plan['waypoints']),8)
        self.assertEqual(geometry.paths[9:14],[(.04,0.),(.04,.03),(.03,.02),(.02,.01),(.01,0.)])
        geometry.reverse_block=True;geometry.captures[0]=0
        with patch('joint_head_wrist_plan.least_squares',side_effect=fitted):
            with self.assertRaisesRegex(ValueError,'reverse return path rejected'):
                plan_wrist(geometry,{side:optical.tolist() for side in ('left','right')},poses_per_side=4)

    def test_planner_actual_start_complete_return_and_explicit_offset(self):
        class Geometry:
            limits=np.tile([-3.,3.],(12,1));config={'margin_px':18};marker_size=.015
            k=dict(fx=630.,fy=630.,cx=320.,cy=240.,width=640,height=480)
            mounts={side:np.eye(4) for side in ('left','right')}
            def __init__(self):self.calls=[];self.paths=[]
            def frames(self,q):
                self.calls.append(np.array(q));return {side:{'link6':np.eye(4)} for side in ('left','right')}
            def path_check(self,start,end):
                self.paths.append((np.array(start),np.array(end)))
                self.frames(start);self.frames(end)
                return {'ok':True}
        geometry=Geometry();start=np.ones(12)*.1;offsets=np.ones(12)*.02
        optical=np.eye(4);optical[2,3]=-.15
        with patch('joint_head_wrist_plan.least_squares',return_value=SimpleNamespace(x=np.zeros(12))):
            plan=plan_wrist(geometry,{side:optical.tolist() for side in ('left','right')},poses_per_side=4,start_q=start,joint_offsets=offsets)
        np.testing.assert_allclose(geometry.calls[0],start+offsets)
        np.testing.assert_allclose(plan['start_q'],start)
        returns=[row for row in plan['waypoints'] if row.get('capture') is False]
        self.assertEqual(len(returns),2)
        for row in returns:np.testing.assert_allclose(row['q'],start)
        self.assertEqual(len(plan['planned_frames']),8)
        self.assertEqual(sum(row['split']=='holdout' for row in plan['planned_frames']),2)
        self.assertEqual(len(geometry.paths),19)
        geometry.frames(start);np.testing.assert_allclose(geometry.calls[-1],start)
        camera={**geometry.k,'rectified':True}
        self.assertTrue(wrist_visibility(geometry,start,'left','right',optical,camera)['visible'])
        optical[0,3]=1.
        self.assertFalse(wrist_visibility(geometry,start,'left','right',optical,camera)['visible'])
        optical=np.eye(4);optical[2,3]=.15
        self.assertFalse(wrist_visibility(geometry,start,'left','right',optical,camera)['visible'])


class WristJointMarkerMountTest(unittest.TestCase):
    """The wrist half can also estimate the *target* sticker's installation.

    The head half solves that mount next to a camera, twelve joint offsets and a mesh
    (twenty-six parameters), which is why it is the weakest of them; the wrist half
    watches the same sticker from six degrees of freedom and can therefore re-measure
    it. These tests pin down that the six extra parameters are identifiable from
    opposite-wrist motion, that the estimate removes a mount the head got wrong, and
    that the solver refuses to claim observability when the arms never move relative
    to each other (which is exactly when the two mounts trade off against each other).
    """

    def setUp(self):
        self.camera = dict(fx=620., fy=618., cx=319., cy=241.)
        self.size = .045
        self.camera_mount = np.eye(4)
        self.camera_mount[:3, :3] = Rotation.from_euler('xyz', [.06, -.09, .03]).as_matrix()
        self.camera_mount[:3, 3] = [.021, -.014, .038]
        self.marker_mount = np.eye(4)
        self.marker_mount[:3, :3] = Rotation.from_euler('xyz', [-.04, .07, -.02]).as_matrix()
        self.marker_mount[:3, 3] = [.004, .017, -.006]
        # What the head half handed over: the same mount, ~1.8 mm and ~2 deg off.
        error = np.eye(4)
        error[:3, :3] = Rotation.from_rotvec([.02, -.03, .01]).as_matrix()
        error[:3, 3] = [.0015, -.0008, .0006]
        self.head_mount = self.marker_mount @ error
        self.initial = self.camera_mount.copy()
        self.initial[:3, 3] += [.008, -.005, .007]
        rng = np.random.default_rng(4231)
        self.rows = []
        for index in range(16):
            observer = np.eye(4)
            observer[:3, :3] = Rotation.from_rotvec(rng.normal(0, .35, 3)).as_matrix()
            observer[:3, 3] = rng.normal(0, .22, 3)
            # Sample where the sticker sits in the observer's camera, then ask which
            # target-arm pose would put it there: that keeps the sticker in front of
            # the lens while leaving the two arms moving independently.
            relative = np.eye(4)
            relative[:3, :3] = Rotation.from_rotvec(rng.normal(0, .3, 3)).as_matrix()
            relative[:3, 3] = [rng.uniform(-.12, .12), rng.uniform(-.09, .09), rng.uniform(.32, .6)]
            observed = observer @ self.camera_mount @ relative
            target = observed @ np.linalg.inv(self.marker_mount)
            uv, _ = project_marker(self.camera_mount, observer, observed, self.camera, self.size)
            self.rows.append(dict(id=str(index), split='holdout' if index >= 12 else 'fit',
                                  T_world_observer_link6=observer.tolist(),
                                  T_world_marker=(target @ self.head_mount).tolist(),
                                  corners_uv=uv.tolist()))
        self.original = copy.deepcopy(self.rows)

    def solve(self, rows=None, marker=True):
        return solve_wrist_calibration(self.rows if rows is None else rows, self.camera, self.size,
                                       self.initial, source_calibration_id='frozen-head-fit-123',
                                       initial_T_link6_marker=self.head_mount if marker else None)

    def test_free_mount_is_identifiable_and_recovers_the_truth(self):
        result = self.solve()
        self.assertEqual(result['jacobian_rank'], 12)
        self.assertEqual(result['parameter_count'], 12)
        self.assertTrue(result['marker_mount_estimated'])
        self.assertTrue(result['numerically_valid'])
        np.testing.assert_allclose(result['T_link6_camera'], self.camera_mount, atol=1e-7)
        np.testing.assert_allclose(result['T_link6_marker'], self.marker_mount, atol=1e-7)
        np.testing.assert_allclose(np.asarray(result['initial_T_link6_marker']), self.head_mount, atol=1e-12)
        self.assertLess(result['holdout']['rms_px'], 1e-6)
        # The correction is reported where the operator reads it, in mm and degrees.
        self.assertAlmostEqual(result['marker_mount_delta']['translation_mm'], 1.8028, places=3)
        self.assertAlmostEqual(result['marker_mount_delta']['rotation_deg'], 2.1436, places=3)
        self.assertEqual(self.rows, self.original)

    def test_a_frozen_mount_leaves_the_head_error_in_the_residual(self):
        # This is the situation the iteration exists for: the head's mount is wrong, so
        # every wrist view of that sticker is explained through a wrong model.
        frozen = self.solve(marker=False)
        self.assertEqual(frozen['jacobian_rank'], 6)
        self.assertNotIn('T_link6_marker', frozen)
        self.assertFalse(frozen['marker_mount_estimated'])
        self.assertGreater(frozen['holdout']['rms_px'], 1.)
        freed = self.solve()
        self.assertLess(freed['holdout']['rms_px'], frozen['holdout']['rms_px'] / 100.)
        for split in ('fit', 'holdout'):
            self.assertLess(freed[split]['rms_px'], 1e-6)

    def test_noisy_views_still_land_closer_than_the_head_mount(self):
        noisy = copy.deepcopy(self.rows)
        rng = np.random.default_rng(91)
        for row in noisy:
            row['corners_uv'] = (np.asarray(row['corners_uv']) + rng.normal(0, .15, (4, 2))).tolist()
        result = self.solve(noisy)
        self.assertLess(result['holdout']['rms_px'], 1.)
        estimated, truth = np.asarray(result['T_link6_marker']), self.marker_mount
        self.assertLess(np.linalg.norm(estimated[:3, 3] - truth[:3, 3]), .0005)
        self.assertLess(Rotation.from_matrix(estimated[:3, :3].T @ truth[:3, :3]).magnitude(), .01)
        head_error = np.linalg.norm(self.head_mount[:3, 3] - truth[:3, 3])
        self.assertLess(np.linalg.norm(estimated[:3, 3] - truth[:3, 3]), head_error / 2)

    def test_a_static_relative_pose_leaves_the_two_mounts_trading_off(self):
        # With no relative motion between the arms, only the product of the two mounts
        # is observed, so the joint solve must report itself unobservable rather than
        # publish a split that the data never chose.
        relative = np.eye(4)
        relative[:3, :3] = Rotation.from_rotvec([.3, -.2, .1]).as_matrix()
        relative[:3, 3] = [.05, .1, .45]
        frozen_pose = copy.deepcopy(self.rows)
        for row in frozen_pose:
            target = np.asarray(row['T_world_observer_link6']) @ relative
            row['T_world_marker'] = (target @ self.head_mount).tolist()
        joint = self.solve(frozen_pose)
        self.assertLess(joint['jacobian_rank'], 12)
        self.assertFalse(joint['numerically_valid'])
        self.assertTrue(joint['marker_mount_estimated'])
        # The mount alone is still estimated, so the frozen path is unaffected.
        self.assertTrue(self.solve(frozen_pose, marker=False)['numerically_valid'])

    def test_the_mount_initial_is_part_of_the_input_digest(self):
        first = self.solve()
        other = self.solve()
        other_mount = self.head_mount.copy()
        other_mount[:3, 3] += [.01, 0., 0.]
        moved = solve_wrist_calibration(self.rows, self.camera, self.size, self.initial,
                                        source_calibration_id='frozen-head-fit-123',
                                        initial_T_link6_marker=other_mount)
        self.assertEqual(first['input_sha256'], other['input_sha256'])
        self.assertNotEqual(first['input_sha256'], moved['input_sha256'],
                            'a different starting mount is different evidence')

    def test_the_dataset_reports_the_mount_for_the_target_arm(self):
        fixture = WristCalibrationTest('test_recovers_transform_and_does_not_fit_holdout')
        fixture.setUp()
        dataset, geometry, initial = fixture.dataset_case()
        dataset['wrist_contract']['estimate_marker_mount'] = True
        mount = np.asarray(dataset['wrist_contract']['head_snapshot']['T_link6_marker']['24'])
        reports = solve_wrist_dataset(dataset, geometry, initial)
        self.assertEqual(reports['left']['marker_mount']['marker_id'], 30)
        self.assertEqual(reports['right']['marker_mount']['marker_id'], 24)
        self.assertEqual(reports['left']['marker_mount']['side'], 'right')
        self.assertIn('delta_from_head_snapshot', reports['right']['marker_mount'])
        for side, report in reports.items():
            self.assertTrue(report['marker_mount_estimated'])
            self.assertEqual(report['jacobian_rank'], 12)
            self.assertTrue(report['accepted'])
            np.testing.assert_allclose(report['T_link6_camera'], fixture.truth, atol=1e-8)
            # These views were made with the snapshot's mount, so the correction the
            # wrist reports is zero - it confirms the model instead of moving it.
            np.testing.assert_allclose(report['marker_mount']['T_link6_marker'], mount, atol=1e-7)
            self.assertLess(report['marker_mount']['delta_from_head_snapshot']['translation_mm'], 1e-4)
        plain = json.loads(json.dumps(dataset))
        plain['wrist_contract'].pop('estimate_marker_mount')
        for report in solve_wrist_dataset(plain, geometry, initial).values():
            self.assertIsNone(report['marker_mount'])
            self.assertEqual(report['jacobian_rank'], 6)

    def test_the_estimate_is_opt_in(self):
        fixture = WristCalibrationTest('test_recovers_transform_and_does_not_fit_holdout')
        fixture.setUp()
        dataset, geometry, initial = fixture.dataset_case()
        for report in solve_wrist_dataset(dataset, geometry, initial).values():
            self.assertIsNone(report['marker_mount'], 'the estimate is opt-in, not implied')
            self.assertFalse(report['marker_mount_estimated'])
            self.assertEqual(report['jacobian_rank'], 6)
            self.assertEqual(report['parameter_count'], 6)


if __name__ == '__main__':
    unittest.main()
