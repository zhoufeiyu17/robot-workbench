"""Shared mesh adapter preserves full triangles, FK corrections and jaw feedback."""
import unittest
from types import SimpleNamespace
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

from arx_env.joint_head_x5_hybrid import X5MeshModel, independent_pose_coverage
from arx_env.joint_head_markerless import frames_with_feedback, camera_mesh_parts


class X5MeshCheck(unittest.TestCase):
    def test_fit_sticker_seed_freezes_masks_and_starts_joint_optimization(self):
        from unittest.mock import patch
        from contextlib import ExitStack
        from arx_env import joint_head_x5_hybrid as hybrid
        class SeedChecked(Exception): pass
        for frozen in (None, {'24': np.eye(4).tolist(), '30': np.eye(4).tolist()}):
            with self.subTest(frozen=bool(frozen)), tempfile.TemporaryDirectory() as tmp, ExitStack() as stack:
                root = Path(tmp)
                rows = []
                for i, split in enumerate(('fit', 'holdout')):
                    path = root/f'{i}.png'; path.write_bytes(bytes([i]))
                    rows.append(dict(id=str(i), image=path.name, split=split,
                                     q=np.zeros(12), jaw_gap_mm=[80.,80.], uv={24:np.ones((4,2))}))
                geometry = self.geometry(); geometry.marker_size = .015
                seen = []
                def marker_residual(model, manifest, selected, parameters):
                    self.assertEqual([r['id'] for r in selected], ['0'])
                    return np.ones(8)
                def solve(objective, initial, **kwargs):
                    seen.append(np.array(initial))
                    if len(seen) == 1:
                        objective(initial)
                        return SimpleNamespace(x=np.full(len(initial), .2))
                    np.testing.assert_allclose(initial, .2)
                    # Sticker residuals must be active immediately, not zeroed by warmup.
                    np.testing.assert_allclose(objective(initial)[-8:], 2.)
                    raise SeedChecked
                stack.enter_context(patch.object(hybrid, 'marker_residual', side_effect=marker_residual))
                stack.enter_context(patch.object(hybrid, 'least_squares', side_effect=solve))
                projected = stack.enter_context(patch.object(hybrid.mesh, 'camera_mesh_parts', return_value=[None]*6))
                stack.enter_context(patch.object(hybrid.mesh, 'raster_depth', return_value=np.ones((8,8))))
                segmented = stack.enter_context(patch.object(hybrid.mesh, 'foreground_edges', return_value={'points':[]}))
                stack.enter_context(patch.object(hybrid.mesh, 'evaluate', return_value=({}, [], [])))
                stack.enter_context(patch.object(hybrid.mesh, 'objective', return_value=np.zeros(40)))
                segmented.return_value['mask'] = np.ones((8,8), np.uint8)
                stack.enter_context(patch.object(hybrid.cv2, 'imread', return_value=np.zeros((8,8,3),np.uint8)))
                stack.enter_context(patch.object(hybrid.cv2, 'imwrite', return_value=True))
                stack.enter_context(patch('builtins.print'))
                with self.assertRaises(SeedChecked):
                    hybrid.fit_head(geometry, rows, root, dict(width=8,height=8,fx=10.,fy=10.,cx=4.,cy=4.),
                                    np.eye(4), [np.eye(4),np.eye(4)], root/'fit',
                                    samples=40, iterations=1, frozen_mounts=frozen)
                expected = .2*np.r_[[.01]*3,[.02]*15]
                self.assertEqual(segmented.call_count, 2)
                for call in projected.call_args_list:
                    np.testing.assert_allclose(call.args[-1], expected)

    def test_solver_progress_snapshot_is_replaced_without_touching_result(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples/scripts/debug'))
        from joint_head_real_fit import write_progress
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'fit.json'
            for fraction in (0., .5, 1.):
                event=dict(fraction=fraction,label='head_optimizer',completed=1,total=2)
                write_progress(output,event)
                self.assertEqual(json.loads(output.with_suffix('.progress.json').read_text()),event)
            self.assertFalse(output.exists())
            self.assertEqual(len(list(Path(tmp).iterdir())),1)

    def test_manifest_truth_is_not_read_and_rejected_coverage_is_retained(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples/scripts/debug'))
        from joint_head_real_fit import load_rows
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            good={'id':'frame-000','image':'image.png','qpos':[0.]*14,'split':'fit',
                  'bracket_gap_s':0.,'pose_span_rad':[0.]*12,'valid_for_fit':True}
            bad={**good,'id':'frame-001','valid_for_fit':False}
            (root/'dataset.json').write_text(json.dumps({'frames':[good,bad],'planned_frames':[{'sample_id':'frame-000'}, {'id':'frame-001'}, {'capture':False}]}))
            (root/'frame-000.json').write_text(json.dumps(good));(root/'frame-001.json').write_text(json.dumps(bad))
            (root/'truth.json').write_text('must never parse this')
            (root/'fit-report.json').write_text('must never parse this either')
            (root/'image.png').write_bytes(b'image hash only')
            rows,coverage=load_rows(root,{},return_coverage=True)
            self.assertEqual(len(rows),1);self.assertEqual(coverage['captured_frames'],2)
            self.assertFalse(coverage['all_captured_valid'])
            self.assertEqual(coverage['planned_frames'],2)
            self.assertEqual(coverage['missing_planned_ids'],[])
            self.assertEqual(coverage['rejected_frames'][0]['id'],'frame-001')
            (root/'dataset.json').unlink()
            self.assertEqual(len(load_rows(root,{})),1)

    def test_repeated_opposite_arm_motion_is_not_independent_marker_pose(self):
        rows=[]
        for i in range(6):
            q=np.zeros(12);q[0]=i*.1;q[6]=i*.2
            rows.append({'split':'fit' if i<4 else 'holdout','q_reported_rad':q,'marker_ids':[24,30]})
        self.assertTrue(independent_pose_coverage(rows)['passed'])
        for row in rows[4:]:row['q_reported_rad'][0]=rows[0]['q_reported_rad'][0]
        result=independent_pose_coverage(rows)
        self.assertFalse(result['passed'])
        self.assertEqual(result['markers']['24']['holdout_not_in_fit'],0)

    def geometry(self):
        class Arm:
            def frames(self, q):
                T = np.eye(4); T[0, 3] = q.sum()
                return {'base_link': np.eye(4), 'link6': T}
        tri = np.array([[0., .03, 0], [.02,.03,0], [0.,.04,0]])
        vertices = np.concatenate([tri, tri-[0,.08,0], tri+[0,0,.02]])
        return SimpleNamespace(model=Arm(), bases={'left':np.eye(4),'right':np.eye(4)},
                               hand={'signs':np.repeat([1,-1,0],3),'nominal_gap_m':.06},
                               meshes=[('link6',vertices,np.arange(9).reshape(3,3))], static_meshes=[])

    def test_offsets_and_fingers_full_mesh(self):
        model = X5MeshModel(self.geometry())
        row = {'q_reported_rad':np.zeros(12), 'jaw_gap_mm':[80.,40.]}
        offsets = np.arange(12)*.001
        fk = frames_with_feedback(model,row,{},offsets)
        self.assertAlmostEqual(fk['left/link6'][0,3],offsets[:6].sum())
        self.assertAlmostEqual(fk['right/link6'][0,3],offsets[6:].sum())
        self.assertAlmostEqual(fk['left/link6/1'][1,3],.01)
        self.assertAlmostEqual(fk['right/link6/1'][1,3],-.01)
        self.assertEqual(sum(len(m['f']) for m in model.meshes),6)
        manifest={'camera_nominal_T_base_optical':np.eye(4), 'intrinsics':{}}
        parts=camera_mesh_parts(model,manifest,row,model.meshes,np.r_[np.zeros(6),offsets])
        self.assertEqual(len(parts),6)
        with self.assertRaises(KeyError):
            model.frames_with_feedback({'q_reported_rad':np.zeros(12)}, {}, np.zeros(12))
        with self.assertRaises(ValueError):
            model.frames_with_feedback({**row,'jaw_gap_mm':[89.001,40]}, {}, offsets)

    def test_measured_endpoint_error_is_retained_but_not_commandable(self):
        from arx_env.joint_head_jaw_mapping import measured_jaw_gaps, command_sdk, hold_command_sdk
        from arx_env.joint_head_workbench_geometry import jaw_gaps
        model = X5MeshModel(self.geometry())
        row = {'q_reported_rad': np.zeros(12), 'jaw_gap_mm': [-2., 87.]}
        fk = model.frames_with_feedback(row, {}, np.zeros(12))
        self.assertAlmostEqual(fk['left/link6/1'][1,3], (-.002-.06)/2)
        self.assertAlmostEqual(fk['right/link6/1'][1,3], (.087-.06)/2)
        np.testing.assert_array_equal(measured_jaw_gaps([-2,87]), [-2,87])
        for value in ([-2.001,0], [0,89.001], [0,float('nan')], [0,float('inf')], [0]):
            with self.assertRaises(ValueError): measured_jaw_gaps(value)
        with self.assertRaises(ValueError): jaw_gaps([-2,87])
        for side, gap in [('left',-2),('right',87)]:
            with self.assertRaises(ValueError): command_sdk(side,gap)
        for side in ('left','right'):
            self.assertEqual(hold_command_sdk(side,87),command_sdk(side,83))
            with self.assertRaises(ValueError): command_sdk(side,83.001)

    def test_real_fit_passes_dataset_provenance_to_solver(self):
        from unittest.mock import patch
        import joint_head_real_fit as real_fit
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            def data(name,value):
                path=root/name;path.write_text(json.dumps(value));return path
            source={'environment':'hardware','robot_source':'physical','simulation_only':False}
            data('dataset.json',source);data('camera.json',{})
            data('completion.json',{'passed':True,'captured':0,'accepted':0})
            urdf=data('urdf.json',{});scene=data('scene.json',{});camera=data('seed.json',{})
            candidate=data('candidate.json',{});manifest=data('mesh.json',{'model':{'meshes':{}}})
            config=data('config.json',dict(urdf=str(urdf),scene_seed=str(scene),mesh_manifest=str(manifest),candidate=str(candidate)))
            args=SimpleNamespace(solver='hybrid',dataset=root,workbench_config=config,output=root/'new-report.json',
                urdf=urdf,scene=scene,camera_seed=camera,size=.015,marker_mounts=None,marker_mounts_provenance=None,
                iterations=1,samples=40,max_nfev=1)
            coverage=dict(captured_frames=0,used_frames=0,rejected_frames=[],source_files=[])
            seed=SimpleNamespace(camera=np.eye(4),mounts=[],initialize_mounts=lambda *a,**k:None)
            report={'source':{},'solver':'test','acceptance':{},'comparison':{}}
            with patch.object(real_fit,'load_rows',return_value=([],coverage)), \
                 patch.object(real_fit,'Fit',return_value=seed), \
                 patch('arx_env.joint_head_workbench_geometry.Geometry',return_value=SimpleNamespace(marker_size=.015)), \
                 patch('arx_env.joint_head_x5_hybrid.fit_head',return_value=report) as solver, patch('builtins.print'):
                real_fit.run(args)
            self.assertEqual(solver.call_args.kwargs['source_metadata'],source)
            self.assertTrue(args.output.exists())


if __name__ == '__main__':
    unittest.main()
