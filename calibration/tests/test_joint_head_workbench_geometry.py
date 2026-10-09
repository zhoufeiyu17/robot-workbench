"""Small deterministic geometry check; optional real mesh contract smoke."""
import json
import os
from pathlib import Path
import struct
import tempfile
import unittest
import numpy as np
from scipy.spatial import ConvexHull
from arx_env.joint_head_workbench_geometry import boxes_overlap, box_triangles, closed_component_representatives, meshes_intersect, Geometry, sticker_support


class GeometryCheck(unittest.TestCase):
    def test_frozen_seed_splits_pose_groups_without_changing_trajectory(self):
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'examples/scripts/debug'))
        from joint_head_workbench import Workbench
        from arx_env.joint_head_x5_hybrid import independent_pose_coverage
        seed = json.loads((Path(__file__).resolve().parents[1]/'standard_head_motion_seed.json').read_text())
        original = json.dumps(seed, sort_keys=True)
        g = Geometry.__new__(Geometry)
        g.limits=np.tile([-3.,3.],(12,1))
        g.preview = lambda *args: self.fail('seed must not run geometry preview/collision checks')
        g.collision_method = 'test'
        plan = g._plan_from_seed(np.zeros(12), seed)
        self.assertEqual(json.dumps(seed, sort_keys=True), original)
        self.assertEqual(len(plan['waypoints']), 35)
        self.assertFalse(plan['waypoints'][0]['capture'])
        self.assertFalse(plan['collision_checked'])
        measured = []
        for side, offset, marker in (('right', 6, 30), ('left', 0, 24)):
            rows = [w for w in plan['waypoints'] if w['arm'] == side]
            self.assertEqual(len(rows), 17)
            self.assertEqual([i for i,w in enumerate(rows) if w['split'] == 'holdout'], [0, 7, 8, 15])
            for source, row in zip(seed['right_waypoints'], rows):
                expected = np.zeros(12)
                expected[offset:offset+6] = np.asarray(source['q'])[6:12] * (1 if side == 'right' else np.asarray(seed['mirror_mapping']))
                np.testing.assert_array_equal(row['q'], expected)
                for key in ('kind', 'capture', 'transition', 'joint', 'cell', 'jaw_gap_mm', 'speeds'):
                    self.assertEqual(row.get(key), source.get(key))
                measured.append({'q_reported_rad': row['q'], 'marker_ids': [marker], 'split': row['split']})
        self.assertEqual(sum(w.get('capture', True) for w in plan['waypoints']), 34)
        fake_seed = json.loads((Path(__file__).resolve().parents[1]/'standard_head_motion_seed_a1_fake_v2.json').read_text())
        fake_plan = g._plan_from_seed(np.zeros(12), fake_seed)
        self.assertEqual(len(fake_plan['waypoints']), 35)
        self.assertEqual(sum(w.get('capture', True) for w in fake_plan['waypoints']), 34)
        self.assertEqual([i for i, w in enumerate(fake_plan['waypoints'][18:])
                          if w['split'] == 'holdout'], [2, 16])
        with self.assertRaisesRegex(ValueError, 'left holdout indices'):
            g._plan_from_seed(np.zeros(12), {**fake_seed, 'left_holdout_indices': [{}, 2]})
        coverage = independent_pose_coverage(measured)
        self.assertTrue(coverage['passed'])
        for counts in coverage['markers'].values():
            self.assertEqual(counts, {'distinct_fit': 7, 'distinct_holdout': 2, 'holdout_not_in_fit': 2})
        g.side_meshes = {s: [] for s in ('left', 'right')}
        g.frames = lambda q: {s: {'link6': np.eye(4)} for s in ('left', 'right')}
        g.mounts = {s: np.eye(4) for s in ('left', 'right')}
        g.marker_size = .015; g.k = {}; g.camera = np.eye(4)
        g.mount_chain = {}; g.mount_status = 'test'
        app = Workbench.__new__(Workbench); app.geometry = g
        stations = app.capture_request(plan['waypoints'])['stations']
        self.assertEqual([r['split'] for r in stations], [r['split'] for r in measured])
        app.queue = plan['waypoints']; app.frames = []
        self.assertEqual(app.counts(), {'waypoints': 35, 'capture_points': 34, 'transition_points': 1,
                                      'captured': 0, 'valid': 0, 'failed': 0, 'unclassified': 0})
        app.queue = [{'capture': False}, {'transition': True, 'capture': True}, {}]
        self.assertEqual(app.counts()['transition_points'], 1)
        self.assertEqual(app.counts()['capture_points'], 2)

    def test_partial_grid_contract_and_short_return(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from arx_env import joint_head_workbench_geometry as module
        seed_dir = tempfile.TemporaryDirectory()
        self.addCleanup(seed_dir.cleanup)
        class Planner(Geometry):
            def __init__(self, blocked=False, diversity=True):
                self.config={'head_motion_seed': str(Path(seed_dir.name)/'absent.json')}
                self.k={'width':640,'height':480};self.margin=5
                self.limits=np.tile([-3.,3.],(12,1));self.collision_method='test'
                self.blocked=blocked;self.diversity=diversity
            def frames(self,q):return q
            def _marker(self,q,side):
                j=0 if side=='left' else 6
                uv=np.array([320,240])+100*q[j:j+2]
                return dict(arm=side,visible=np.linalg.norm(q[j:j+2])>.1,center_uv=uv,
                            corners_uv=[uv]*4,hand_uv=[uv],normal_facing=-1.)
            def preview(self,q,include_parts=True):
                markers=[self._marker(q,s) for s in ('left','right')]
                return dict(in_limits=True,collision_free=True,collisions=[],markers=markers,
                            visible_ids=[i for i,m in zip((24,30),markers) if m['visible']])
            def path_check(self,start,target,side=None):
                if not self.diversity and np.any(abs(target[[2,3,4,5,8,9,10,11]])>1e-8):return {'ok':False}
                return {'ok':not(self.blocked and side is None and np.max(abs(target))<1e-8 and max(start[0],start[6])>.1)}
        for blocked in (False,True):
            plan=Planner(blocked).auto_plan(np.zeros(12))
            self.assertIn(plan['plan_complete'],(True,False))
            self.assertIn('coverage',plan)
            for side in ('left','right'):
                rows=[w for w in plan['waypoints'] if w['arm']==side]
                if rows and any(w['kind']=='joint_diversity' for w in rows): self.assertEqual(next(w for w in rows if w['kind']=='joint_diversity')['cell'],[1,1])
                self.assertGreaterEqual(sum(w['kind']=='park_return' for w in rows),0)
                self.assertTrue(all(w.get('capture') is True for w in rows if w['kind'].endswith('_return')))
                if rows and np.max(np.abs(rows[-1]['q'])) < 1e-8:
                    np.testing.assert_allclose(rows[-1]['q'],np.zeros(12),atol=1e-8)
        insufficient=Planner(diversity=False).auto_plan(np.zeros(12))
        self.assertIn('plan_complete',insufficient)
        self.assertIn('coverage',insufficient)
        # A failed first optimization seed must not prevent trying the bounded alternatives.
        solve=module.least_squares;calls=[]
        def first_seed_fails(fun,seed,**kwargs):
            calls.append(seed.copy())
            return SimpleNamespace(x=np.zeros(6)) if len(calls)%2 else solve(fun,seed,**kwargs)
        with patch.object(module,'least_squares',side_effect=first_seed_fails):
            self.assertTrue(Planner().auto_plan(np.zeros(12))['coverage']['right']['center_visible'])
        self.assertGreater(len(calls),1)

    def test_uncertain_hull_solver_uses_closed_mesh_narrow_phase(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from arx_env import joint_head_workbench_geometry as module
        g=Geometry.__new__(Geometry)
        g.config={'static_boxes':[dict(name='top_camera_body',size_m=[.2,.2,.2],T_world_box=np.eye(4).tolist())]}
        g._load_chassis('.')
        outer=box_triangles([2,2,2],np.eye(4))
        inner=box_triangles([1,1,1],np.eye(4)).reshape(-1,3,3)[:,::-1].reshape(-1,3)
        hollow=np.concatenate((outer,inner))
        g.bounds={('left','link1'):(np.zeros(3),np.ones(3))}
        g.hulls={('left','link1'):ConvexHull(hollow).equations}
        g.side_meshes={'left':[('link1',hollow,np.arange(len(hollow)).reshape(-1,3))]}
        g.containment_representatives={('left','link1'):closed_component_representatives(hollow)}
        g.adjacent=set();frames={'left':{'link1':np.eye(4)}}
        with patch.object(module,'linprog',return_value=SimpleNamespace(success=False,status=4)):
            self.assertEqual(g.collisions(frames),[])  # Fixed box is inside the moving cavity.
            frames['left']['link1'][0,3]=.5
            self.assertEqual(g.collisions(frames),[['left/link1','environment/top_camera_body']])
            frames['left']['link1'][0,3]=.8  # Complete containment in the solid wall.
            self.assertEqual(g.collisions(frames),[['left/link1','environment/top_camera_body']])
            frames['left']['link1'][0,3]=0
            g.containment_representatives['left','link1']=None
            self.assertEqual(g.collisions(frames),[['left/link1','environment/top_camera_body','intersection solver uncertain']])

    def test_closed_component_containment_matches_full_vertex_checks(self):
        outer=box_triangles([2,2,2],np.eye(4)).reshape(-1,3,3)
        inner=box_triangles([1,1,1],np.eye(4)).reshape(-1,3,3)
        hollow=np.concatenate((outer,inner[:,::-1]))
        small=box_triangles([.2,.2,.2],np.eye(4)).reshape(-1,3,3)
        separate=np.concatenate((small+[3,0,0],small))
        representatives=closed_component_representatives(separate)
        self.assertEqual(len(representatives),2)
        self.assertTrue(meshes_intersect(separate,outer,representatives))
        for moving in (small,small+[.5,0,0],small+[3,0,0],separate,outer,hollow):
            representatives=closed_component_representatives(moving)
            self.assertIsNotNone(representatives)
            for fixed in (outer,hollow,small):
                self.assertEqual(meshes_intersect(moving,fixed,representatives),meshes_intersect(moving,fixed))
        opened=small[:-1]
        self.assertIsNone(closed_component_representatives(opened))
        self.assertEqual(meshes_intersect(opened,outer,None),meshes_intersect(opened,outer))
        self.assertIsNone(closed_component_representatives(np.zeros((1,3,3))))

    def test_frozen_marker_mount_rebuilds_only_changed_support_view(self):
        g=Geometry.__new__(Geometry)
        g.config={'wrist_support_proxy':{'shell_size_m':[.025,.025,.012],'bracket_thickness_m':.006}}
        old=np.eye(4);old[:3,3]=[.07,0,.09]
        g.mounts={side:old.copy() for side in ('left','right')}
        g.side_meshes={};g.mount_chain={};g.bounds={};g.hulls={}
        for side in g.mounts:
            body,top,sticker,shell,rod=sticker_support(old,[.025,.025,.012],.006)
            g.mount_chain[side]={'T_link6_camera_body_proxy':body.tolist(),'physical_geometry_verified':False}
            g.side_meshes[side]=[('link6',np.zeros((3,3)),np.array([[0,1,2]]))]
            for name,vertices in [('camera_shell_proxy',shell),('camera_bracket_proxy',rod)]:
                g.side_meshes[side].append((name,vertices,np.arange(len(vertices)).reshape(-1,3)))
                lo,hi=vertices.min(0),vertices.max(0)
                g.bounds[side,name]=((lo+hi)/2,(hi-lo)/2)
                g.hulls[side,name]=ConvexHull(vertices).equations
        self.assertIs(g.with_marker_mounts(g.mounts),g)
        new=old.copy();new[:3,3]+=[.1,.02,0]
        view=g.with_marker_mounts({'left':new,'right':old})
        np.testing.assert_allclose(g.mounts['left'],old)
        np.testing.assert_allclose(view.mounts['left'],new)
        chain=view.mount_chain['left']
        np.testing.assert_allclose(np.asarray(chain['T_link6_camera_body_proxy'])@chain['T_body_upper_shell']@chain['T_upper_shell_marker'],new)
        self.assertIs(view.side_meshes['left'][0][1],g.side_meshes['left'][0][1])
        self.assertIs(view.side_meshes['right'][1][1],g.side_meshes['right'][1][1])
        self.assertIsNot(view.side_meshes['left'][1][1],g.side_meshes['left'][1][1])
        for name,vertices,_ in view.side_meshes['left'][1:]:
            center,half=view.bounds['left',name]
            np.testing.assert_allclose(center-half,vertices.min(0))
            self.assertLess(np.max(vertices@view.hulls['left',name][:,:3].T+view.hulls['left',name][:,3]),1e-10)
        self.assertIs(view.with_marker_mounts(view.mounts),view)
        new[0,3]=9
        self.assertAlmostEqual(view.mounts['left'][0,3],.17)
        with self.assertRaises(ValueError):g.with_marker_mounts({'left':old})

    def test_chassis_keeps_component_gap_and_checks_moving_shell(self):
        with tempfile.TemporaryDirectory() as directory:
            triangles=[]
            for x in (-2,2):
                T=np.eye(4);T[0,3]=x
                triangles.extend(box_triangles([1,1,1],T).reshape(-1,3,3))
            raw=b'fixture'.ljust(80,b'\0')+struct.pack('<I',len(triangles))
            for triangle in triangles:
                raw+=struct.pack('<12fH',0,0,0,*triangle.reshape(-1),0)
            (Path(directory)/'chassis.stl').write_bytes(raw)
            g=Geometry.__new__(Geometry)
            T=np.eye(4);T[2,3]=1
            g.config={'chassis_mesh':'chassis.stl','chassis_scale':[.5,.5,.5], 'chassis_T_world_mesh':T.tolist()}
            g._load_chassis(directory)
            self.assertEqual(len(g.static_hulls),2)
            self.assertEqual(g.static_meshes[0][0],'chassis')
            np.testing.assert_allclose(g.static_meshes[0][1].min(0),[-1.25,-.25,.75])
            np.testing.assert_allclose(g.static_meshes[0][1].max(0),[1.25,.25,1.25])
            points=box_triangles([.1,.1,.1],np.eye(4))
            g.bounds={('left','camera_shell_proxy'):(np.zeros(3),np.full(3,.05))}
            g.hulls={('left','camera_shell_proxy'):ConvexHull(points).equations}
            g.side_meshes={'left':[('camera_shell_proxy',points,np.arange(len(points)).reshape(-1,3))]}
            g.adjacent=set()
            frames={'left':{'camera_shell_proxy':T.copy()}}
            self.assertEqual(g.collisions(frames),[])  # Between components, inside the whole-chassis hull.
            frames['left']['camera_shell_proxy'][0,3]=1
            self.assertEqual(g.collisions(frames),[['left/camera_shell_proxy','environment/chassis']])
            g.bounds={('left','base_link'):g.bounds['left','camera_shell_proxy']}
            frames['left']['base_link']=frames['left']['camera_shell_proxy']
            self.assertEqual(g.collisions(frames),[])
            for scale in ([1,0,1],[1,2],[1,float('nan'),1]):
                g.config['chassis_scale']=scale
                with self.assertRaises(ValueError):g._load_chassis(directory)

    def test_static_top_box_without_chassis_has_named_collision(self):
        g=Geometry.__new__(Geometry)
        top=np.eye(4);top[:3,3]=[.085,0,.505]
        g.config={'static_boxes':[dict(name='top_camera_body',size_m=[.025,.045,.045],T_world_box=top.tolist())]}
        g._load_chassis('.')
        self.assertEqual(g.static_labels,['top_camera_body'])
        points=box_triangles([.01,.01,.01],np.eye(4))
        g.bounds={('left','camera_body'):(np.zeros(3),np.full(3,.005))}
        g.hulls={('left','camera_body'):ConvexHull(points).equations}
        g.side_meshes={'left':[('camera_body',points,np.arange(len(points)).reshape(-1,3))]};g.adjacent=set()
        frames={'left':{'camera_body':top.copy()}}
        self.assertEqual(g.collisions(frames),[['left/camera_body','environment/top_camera_body']])
        frames['left']['camera_body'][0,3]+=.1
        self.assertEqual(g.collisions(frames),[])
        g.config['static_boxes']*=2
        with self.assertRaisesRegex(ValueError,'unique'):g._load_chassis('.')

    def test_source_wrist_body_is_fixed_collision_geometry_and_survives_views(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from arx_env import joint_head_workbench_geometry as module
        cube=box_triangles([.01,.01,.01],np.eye(4))
        T=np.eye(4);T[0,3]=.1
        payloads={'seed':{'T_world_arm_base':{s:np.eye(4).tolist() for s in ('left','right')}},
                  'candidate':{'T_world_optical':np.eye(4).tolist(),'T_link6_marker':{'24':np.eye(4).tolist(),'30':np.eye(4).tolist()}},
                  'camera':{},'manifest':{'model':{'meshes':{'link5':'unused','link6':'unused'}}}}
        model=SimpleNamespace(names=['joint'+str(i) for i in range(1,7)],joints=[],
                              frames=lambda q:{'link5':np.eye(4),'link6':np.eye(4)})
        with tempfile.TemporaryDirectory() as directory:
            urdf=Path(directory)/'robot.urdf'
            links=''.join('<link name="'+name+'"><visual><geometry><mesh filename="unused"/></geometry></visual></link>' for name in ('link5','link6'))
            joints=''.join('<joint name="joint'+str(i)+'"><limit lower="-3" upper="3" velocity="1"/></joint>' for i in range(1,7))
            urdf.write_text('<robot>'+links+joints+'</robot>')
            config=dict(urdf=str(urdf),scene_seed='seed',candidate='candidate',camera='camera',mesh_manifest='manifest',
                        wrist_camera_housings={'left':{'size_m':[.025,.045,.045],'T_link6_body':T.tolist()}})
            with patch.object(module,'Model',return_value=model),patch.object(module,'_read',side_effect=lambda key:payloads[key]),                  patch.object(module,'stl_vertices',return_value=cube),patch.object(module,'hand_template',return_value={'vertices':cube,'faces':np.arange(36).reshape(-1,3),'signs':np.zeros(36),'nominal_gap_m':.083}):
                g=Geometry(config)
            vertices=next(v for name,v,_ in g.side_meshes['left'] if name=='camera_body')
            np.testing.assert_allclose((vertices.min(0)+vertices.max(0))/2,T[:3,3])
            frames=g.frames(np.zeros(12))
            np.testing.assert_array_equal(frames['left']['camera_body'],frames['left']['link6'])
            frames['right']['link5'][0,3]=1;frames['right']['link6'][0,3]=1
            frames['left']['link5'][0,3]=.1
            self.assertIn(['left/link5','left/camera_body'],g.collisions(frames))
            self.assertNotIn(frozenset(('link5','camera_body')),g.adjacent)
            self.assertIn(frozenset(('link6','camera_body')),g.adjacent)
            new=np.eye(4);new[0,3]=.2
            view=g.with_marker_mounts({'left':new,'right':np.eye(4)}).with_gaps(0,43)
            np.testing.assert_array_equal(next(v for name,v,_ in view.side_meshes['left'] if name=='camera_body'),vertices)
            np.testing.assert_array_equal(g.mounts['left'],np.eye(4))

    def test_triangle_collision_preserves_hollow_space_and_containment(self):
        outer=box_triangles([2,2,2],np.eye(4)).reshape(-1,3,3)
        inner=box_triangles([1,1,1],np.eye(4)).reshape(-1,3,3)
        hollow=np.concatenate((outer,inner[:,::-1]))
        small=box_triangles([.2,.2,.2],np.eye(4)).reshape(-1,3,3)
        self.assertFalse(meshes_intersect(small,hollow))
        self.assertTrue(meshes_intersect(small,outer))
        self.assertTrue(meshes_intersect(outer,small))
        self.assertTrue(meshes_intersect(small+[.5,0,0],hollow))
        self.assertFalse(meshes_intersect(small+[3,0,0],hollow))
        self.assertTrue(meshes_intersect(small,small))  # Coplanar faces.

    def test_table_rejects_moving_mesh_but_allows_fixed_base_contact(self):
        g=Geometry.__new__(Geometry);g.config={'table_z_m':0.,'table_clearance_m':.005};g.bounds={}
        vertices=np.array([[0,0,-.01],[0,0,.02],[.02,0,.02]])
        g.side_meshes={'left':[('base_link',vertices,[]),('link6',vertices,[])]}
        frames={'left':{'base_link':np.eye(4),'link6':np.eye(4)}}
        self.assertEqual(g.collisions(frames),[['left/link6','environment/table']])
        frames['left']['link6'][2,3]=.02
        self.assertEqual(g.collisions(frames),[])

    def test_estimated_shell_chain_supports_sticker(self):
        marker=np.eye(4);marker[:3,3]=[.07,0,.09]
        body,top,sticker,shell,rod=sticker_support(marker,[.025,.025,.012],.006)
        np.testing.assert_allclose(body@top@sticker,marker,atol=1e-12)
        local=(shell-body[:3,3])@body[:3,:3]
        self.assertAlmostEqual(local[:,2].min(),-.006)
        self.assertGreater((shell-marker[:3,3])[:,2].min(),0)
        self.assertEqual(shell.shape,(36,3));self.assertEqual(rod.shape,(36,3))
        with self.assertRaises(ValueError):sticker_support(marker,[.025,.025,.012],-.1)

    def test_separating_axes_and_invalid_feedback(self):
        box=(np.zeros(3),np.eye(3),np.ones(3))
        self.assertTrue(boxes_overlap(box,(np.array([1.9,0,0]),np.eye(3),np.ones(3))))
        self.assertFalse(boxes_overlap(box,(np.array([2.1,0,0]),np.eye(3),np.ones(3))))
        g=Geometry.__new__(Geometry)
        for bad in ([0]*11,[0]*11+[float('nan')]):
            with self.assertRaises(ValueError):g._q(bad)

    @unittest.skipUnless(os.environ.get('WORKBENCH_GEOMETRY_CONFIG'),'explicit local mesh fixture only')
    def test_real_mesh_plan(self):
        with open(os.environ['WORKBENCH_GEOMETRY_CONFIG']) as f:g=Geometry(json.load(f))
        q=np.array([-.47,.108,.046,.059,-.038,-.026,.558,.084,.058,.022,.012,-.03])
        plan=g.auto_plan(q)
        self.assertGreater(len(plan['waypoints']),10)
        seen_right=False
        for row in plan['waypoints']:
            if row['arm']=='right':seen_right=True
            elif seen_right:self.fail('left must finish before right')
            p=g.preview(row['q'],False)
            self.assertTrue(p['collision_free'] and p['in_limits'])
            if row.get('capture',True):self.assertIn(24 if row['arm']=='left' else 30,p['visible_ids'])
            self.assertTrue(g.path_check(q,row['q'])['ok'])
            q=np.array(row['q'])
        self.assertFalse(plan['hardware_execution_approved'])

if __name__=='__main__':unittest.main()
