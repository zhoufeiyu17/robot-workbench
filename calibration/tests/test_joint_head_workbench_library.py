"""Robot-local versions survive raw deletion; replay never selects a fit source."""
import copy
import hashlib
import json
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

import test_joint_head_workbench_live_bridge as bridge_fixture
from arx_env.joint_head_calibration_store import CalibrationStore
from joint_head_workbench import Workbench


class LibraryTests(unittest.TestCase):
    def test_bad_precision_is_viewable_but_not_installable(self):
        a=self.app;self.populated()
        a.store=CalibrationStore(a.output/'empty-library','arx6')
        path=a.output/'batch-live-a/capture/wrist-fit-test.json'
        report=json.loads(path.read_text())
        report['left'].update(accepted=False,optimizer_success=False,jacobian_rank=3)
        report['left']['holdout']['rms_px']=100.
        path.write_text(json.dumps(report))
        row=a.wrist_results()[0]
        self.assertIsNone(a.wrist_overlay_error(row))
        self.assertFalse(row['accepted'])
        self.assertTrue(a.wrist_overlay_options()[0]['selectable'])
        with self.assertRaises(ValueError):a.register_result(row)
        before=a.store.list()
        a.action('/api/wrist-select',{'result_id':row['id']})
        self.assertEqual(a.wrist_selection_id(),row['id'])
        self.assertEqual(a.wrist_selected['pair_status'],'incomplete_left')
        self.assertEqual(a.store.list(),before)
        from joint_head_workbench import load_wrist_selection
        self.assertEqual(load_wrist_selection(a.output/'wrist-selected.json')['id'],row['id'])
        for field,value in [('T_link6_camera',[[0]*4]*4),('contract_sha256','wrong')]:
            bad=copy.deepcopy(report);bad['left'][field]=value
            path.write_text(json.dumps(bad))
            self.assertIsNotNone(a.wrist_overlay_error(a.wrist_results()[0]))

    def test_source_guards_apply_offline_and_after_mode_switch(self):
        a=self.app;self.populated()
        head=a.calibration_results()[0];wrist=a.wrist_results()[0]
        a.hardware_connected=lambda:False
        a.set_control_mode('hardware_sync')
        self.assertIsNone(a.calibration);self.assertEqual(a.wrist_selected,{})
        self.assertIsNone(a.workflow['head_result_id']);self.assertEqual(a.queue,[])
        before=list(self.calls)
        for path,body in [('/api/workflow',{'kind':'wrist','head_result_id':head['id']}),
                          ('/api/calibration',{'result_id':head['id']}),
                          ('/api/wrist-select',{'result_id':wrist['id']}),
                          ('/api/calibration-register',{'result_id':head['id']})]:
            with self.assertRaises(ValueError):a.action(path,body)
        self.assertEqual(self.calls,before)
        self.assertEqual(a.calibration_state()['results'],[])
        for environment in ('simulation',None,'unknown'):
            with self.assertRaises(ValueError):a.require_calibration_environment({'environment':environment})
        a.require_calibration_environment({'environment':'hardware'})
        a.action('/api/workflow',{'kind':'wrist'})
        self.assertEqual(a.workflow,{'kind':'wrist','head_result_id':None})

    def setUp(self):
        bridge_fixture.BridgeTests.setUp(self)
        a=self.app;a.output=a.output.resolve()
        a.store=CalibrationStore(a.output/'versions','arx6');a.delete_tokens={}
        a.geometry_id='geometry';a.wrist_selected={};a.queue=[{'q':[.1]*12}]
        a.plan={'waypoints':copy.deepcopy(a.queue)};a.plan_geometry_id=a.geometry_id
        a.frames=[];a.evaluation=None;a.batch=None;a.workflow={'kind':'head','head_result_id':None}
        a.control_request=lambda *args,**kwargs:self.fail('library actions must not control hardware')

    def batch(self,name):
        path=self.app.output/name;path.mkdir(parents=True)
        (path/'completion.json').write_text(json.dumps({'passed':True,'captured':1}))
        row={'id':'frame-000','image':'frame-000.png','q_reported_rad':[.2]*12,
             'camera_images':{'left':'frame-000-left.png'},'camera_detections':{'left':{'30':[[1,2]]*4}}}
        (path/'dataset.json').write_text(json.dumps({'frames':[row]}))
        (path/'frame-000.png').write_bytes(b'head pixels')
        (path/'frame-000-left.png').write_bytes(b'left pixels')
        return path

    def populated(self):
        a=self.app;head=self.batch('head-capture');a.selected_batch='head-capture'
        wrist,contract,_=bridge_fixture.BridgeTests.wrist_case(self)
        fit={**a.fit_result,'optimizer':{'success':True},'metrics_rectified_px':{'fit':{'corners':16,'rms_px':.5}}}
        (head/'fit.json').write_text(json.dumps(fit))
        (wrist.parent.parent/'request.json').write_text('{}')
        a.register_existing_calibrations()
        a.calibration=a.calibration_results()[0]
        wrist_row=a.wrist_results()[0]
        a.wrist_selected={'id':wrist_row['id'],'contract_sha256':wrist_row['result']['contract_sha256'],
                          'pair_status':wrist_row.get('pair_status'),'file':wrist_row.get('file'),
                          'environment':wrist_row.get('environment'),'left':wrist_row.get('left'),'right':wrist_row.get('right')}
        (a.output/'realtime-calibration.json').write_text(json.dumps(a.calibration))
        (a.output/'wrist-selected.json').write_text(json.dumps(a.wrist_selected))
        a.selected_batch='unrelated-fit-source'
        return head,wrist.parent,contract

    def delete(self,batch_id):
        preview=self.app.action('/api/history-delete-preview',{'id':batch_id})
        return self.app.action('/api/history-delete',{'id':batch_id,'token':preview['token']})

    def pair(self):
        """A solved head + its paired wrist result, with an empty library.

        `populated` registers what it finds; this measures the "solved but not yet
        saved" state. The fixture's minimal fit report does not satisfy the current
        head-eligibility policy (that policy has its own tests), so pairing tests take
        the head as eligible and check what the pairing itself decides.
        """
        a=self.app;self.populated()
        head=a.calibration_results()[0];wrist=a.wrist_results()[0]
        a.head_dependency_eligible=lambda row: True
        a.store=CalibrationStore(a.output/'library-unregistered','arx6')
        return head,wrist

    def groups(self):
        self.app._mesh_view_mode='simulation'
        return self.app.calibration_groups()

    def versions(self):
        return {record['kind']:record['version'] for record in self.app.store.list()}

    def test_a_head_on_its_own_is_never_an_installable_version(self):
        # The unit is one group: a head plus the wrist result solved against it.
        a=self.app;head,wrist=self.pair()
        rows=self.groups()
        self.assertEqual(len(rows),1,'every head appears as one group')
        group=rows[0]
        self.assertTrue(group['installable'],'head + its matching paired wrist is installable')
        self.assertEqual(group['wrist']['id'],wrist['id'])
        self.assertEqual(group['wrist']['pair_status'],'complete')
        self.assertIsNone(group['reason_short'],'an installable group has nothing to warn about')
        before=len(a.store.list())
        for body in ({'head_result_id':head['id']},{'result_id':head['id']},
                     {'head_result_id':head['id'],'wrist_result_id':'someone-else'}):
            with self.assertRaisesRegex(ValueError,'一组'):
                a.action('/api/calibration-register',body)
        self.assertEqual(len(a.store.list()),before,'a refused save publishes nothing')

    def test_the_marker_prior_is_a_wrist_result_not_a_typed_value(self):
        # The head half solves both sticker installations next to a camera, twelve
        # offsets and the mesh, so the prior that stops it doing that has to name the
        # evidence that measured them - a result id - and read the mounts back from it.
        a=self.app;self.pair()
        rows=a.wrist_results()
        self.assertTrue(rows,'the fixture solved a wrist result')
        source=rows[0]['id']
        with self.assertRaisesRegex(ValueError,'没有估计标识安装'):
            a.action('/api/marker-prior',{'source':source})
        self.assertIsNone(getattr(a,'marker_prior_source',None),'a refused prior must not stick')

        path=a.output/rows[0]['file']
        report=json.loads(path.read_text())
        for side,marker,target in (('left','30','right'),('right','24','left')):
            report[side]['marker_mount']={
                'marker_id':int(marker),'side':target,'T_link6_marker':[[1.,0.,0.,.01],[0.,1.,0.,.02],[0.,0.,1.,.03],[0.,0.,0.,1.]],
                'delta_from_head_snapshot':{'translation_mm':1.437,'rotation_deg':2.994}}
        path.write_text(json.dumps(report))
        # The record id is a digest of the record, so writing the estimate changes it.
        source=a.wrist_results()[0]['id']

        result=a.action('/api/marker-prior',{'source':source})
        self.assertEqual(result['marker_prior']['source_result_id'],source)
        self.assertEqual(sorted(result['marker_prior']['mounts']),['24','30'])
        self.assertEqual(result['marker_prior']['deltas']['30']['translation_mm'],1.437)
        self.assertIn('标识 24',result['message'])
        self.assertEqual(getattr(a,'marker_prior_source',None),source)
        self.assertEqual(json.loads((a.output/'workbench-state.json').read_text())['marker_prior_source'],source)

        for bad in ({'source':'missing'},{'source':7},{'source':[]}):
            with self.assertRaises(ValueError):
                a.action('/api/marker-prior',bad)
        self.assertEqual(getattr(a,'marker_prior_source',None),source,'a rejected change must keep the working prior')
        # A source that disappears later must say so instead of quietly fitting free.
        a.marker_prior_source='deleted-result'
        self.assertIn('已不可用',a.marker_prior()['error'])
        cleared=a.action('/api/marker-prior',{'clear':True})
        self.assertIsNone(cleared['marker_prior'])
        self.assertIsNone(a.marker_prior_source)
        self.assertIsNone(json.loads((a.output/'workbench-state.json').read_text())['marker_prior_source'])

    def test_a_group_publishes_the_head_and_its_wrist_as_one_unit(self):
        a=self.app;head,wrist=self.pair()
        result=a.action('/api/calibration-register',{'head_result_id':head['id'],'wrist_result_id':wrist['id']})
        versions=self.versions()
        self.assertEqual(sorted(versions),['head','wrist'],'one group publishes both versions')
        self.assertEqual(versions['head'],result['head_version'])
        self.assertEqual(versions['wrist'],result['version'])
        self.assertEqual(a.store.get(wrist['id'])['dependency_id'],head['id'],'the wrist keeps its head dependency')
        group=self.groups()[0]
        self.assertTrue(group['installable'],'publishing does not install; the group is still offered')
        self.assertTrue(group['registered'])
        self.assertFalse(group['installed'])
        self.assertIsNone(group['reason_short'])

    def test_installing_makes_one_group_the_active_calibration(self):
        a=self.app;head,wrist=self.pair()
        result=a.action('/api/calibration-install',{'head_result_id':head['id'],'wrist_result_id':wrist['id']})
        versions=self.versions()
        self.assertEqual(sorted(versions),['head','wrist'],'installing publishes what it installs')
        active=Path(result['path'])
        self.assertEqual(sorted(path.name for path in active.iterdir()),
                         ['head.json','manifest.json','wrist.json'])
        self.assertEqual((active/'head.json').read_bytes(),
                         (a.store.root/(versions['head']+'.json')).read_bytes())
        manifest=json.loads((active/'manifest.json').read_text())
        self.assertEqual(manifest['environment'],'simulation')
        self.assertEqual(manifest['head']['id'],head['id'])
        self.assertEqual(manifest['wrist']['dependency_id'],head['id'])
        acceptance=manifest['acceptance']
        self.assertEqual(acceptance['pair_status'],'complete')
        self.assertEqual(acceptance['left_holdout_rms_px'],wrist['holdout']['left']['rms_px'])
        self.assertEqual(acceptance['right_holdout_rms_px'],wrist['holdout']['right']['rms_px'])
        self.assertTrue(result['installed']['verified'])
        group=self.groups()[0]
        self.assertTrue(group['installed'])
        self.assertEqual(group['reason_short'],'当前有效')
        self.assertEqual(a.installed_pair()['head_version'],versions['head'])
        self.assertEqual(a.installed_pair()['wrist_version'],versions['wrist'])

    def test_the_wrist_holdout_gate_is_declared_by_the_run(self):
        a=self.app
        a.config={}
        self.assertEqual(a.wrist_acceptance_px(),3.0,'new wrist plans declare 3 px')
        self.assertIsNone(a.wrist_acceptance_note())
        a.config['wrist_acceptance']={'max_holdout_rms_px':2.5,'reason':'几何修正后左腕最优 2.20 px'}
        self.assertEqual(a.wrist_acceptance_px(),2.5)
        self.assertIn('2.50 px',a.install_notes({'version':'v1','source_batch':'b'},{'version':'v2','source_batch':'b'}))
        self.assertIn('左腕最优 2.20 px',a.install_notes({'version':'v1','source_batch':'b'},{'version':'v2','source_batch':'b'}))
        for bad in (0.2,6.0,'wide'):
            a.config['wrist_acceptance']={'max_holdout_rms_px':bad}
            with self.assertRaises((ValueError,TypeError)):
                a.wrist_acceptance_px()

    def test_the_active_group_is_not_offered_again_but_can_be_refreshed(self):
        a=self.app;head,wrist=self.pair()
        a.action('/api/calibration-install',{'head_result_id':head['id'],'wrist_result_id':wrist['id']})
        published=len(a.store.list())
        group=self.groups()[0]
        self.assertFalse(group['installable'],'the active group is reported as active, not as pending')
        self.assertTrue(group['installed'])
        self.assertEqual(group['reason_short'],'当前有效')
        # Installing the active group again is a refresh, not a second group.
        again=a.action('/api/calibration-install',{'head_result_id':head['id'],'wrist_result_id':wrist['id']})
        self.assertEqual(len(a.store.list()),published,'a refresh publishes nothing new')
        self.assertTrue(again['installed']['verified'])
        self.assertEqual(sorted(path.name for path in Path(again['path']).iterdir()),
                         ['head.json','manifest.json','wrist.json'])

    def test_a_head_without_a_matching_wrist_is_shown_with_the_reason(self):
        a=self.app;head,_=self.pair()
        for path in a.output.glob('batch-live-*/capture/wrist-fit-*.json'):
            path.unlink()
        group=self.groups()[0]
        self.assertIsNone(group['wrist'])
        self.assertFalse(group['installable'])
        self.assertIn('尚无与该头部匹配的双腕结果',group['reason'])
        self.assertEqual(group['reason_short'],'缺匹配双腕','the row label needs the short form')
        for endpoint in ('/api/calibration-register','/api/calibration-install'):
            with self.assertRaisesRegex(ValueError,'一组'):
                a.action(endpoint,{'head_result_id':head['id']})

    def test_a_wrist_left_behind_by_a_geometry_change_is_named_in_the_reason(self):
        # The mesh fix moved the geometry fingerprint, which filters the old wrist
        # results out of the catalog. The head must still say *which* bound wrist became
        # unusable and why, instead of silently having no wrist at all.
        a=self.app;self.populated()
        head=a.calibration_results()[0]
        registered=self.versions()['wrist']
        a.head_dependency_eligible=lambda row: True
        a.geometry_id='geometry-after-mesh-fix'
        self.assertEqual(a.wrist_results(),[])
        group=self.groups()[0]
        self.assertEqual(group['id'],head['id'])
        self.assertIsNone(group['wrist'])
        self.assertFalse(group['installable'])
        self.assertIn('几何不符',group['reason'])
        self.assertIn(registered,group['reason'])
        self.assertIn(registered,group['reason_short'])

    def test_overlay_wrist_options_keep_what_cannot_be_drawn(self):
        # The overlay selector must be able to answer "why can I not select my wrist
        # calibration": the version stays listed with the reason, and the row names the
        # head it was solved against so the pairing is visible without the library.
        a=self.app;self.populated()
        a._mesh_view_mode='simulation'
        options=a.wrist_overlay_options()
        self.assertEqual(len(options),1)
        option=options[0]
        self.assertTrue(option['selectable'])
        self.assertEqual(option['head_id'],a.calibration_results()[0]['id'])
        self.assertEqual(option['head_version'],'v000001')
        self.assertIsNone(option['reason'])
        self.assertIn('wrist_options',a.calibration_state(),'the panel reads it from the state')
        a.geometry_id='geometry-after-mesh-fix'
        self.assertEqual(a.wrist_results(),[])
        stale=a.wrist_overlay_options()
        self.assertEqual(len(stale),1,'a filtered version stays visible as an explanation')
        self.assertEqual(stale[0]['version'],'v000002')
        self.assertFalse(stale[0]['selectable'])
        self.assertIn('与当前几何不符',stale[0]['reason'])
        # A historical single-side record is never an overlay source either.
        a.store.register('left','legacy-left',{'row':{},'contract':{}},'old-batch',
                         environment='simulation',dependency_id=a.calibration_results()[0]['id'])
        by_version={row.get('version'):row for row in a.wrist_overlay_options()}
        self.assertEqual(len(by_version),2)
        legacy=[row for row in a.wrist_overlay_options() if row['id']=='legacy-left'][0]
        self.assertFalse(legacy['selectable'])
        self.assertIn('历史单侧记录',legacy['reason'])

    def test_a_rejected_wrist_draft_never_blocks_the_backfill(self):
        # `register_result` refuses an incomplete pair by design, and the backfill runs from
        # __init__ - so without a skip, one rejected solve leaves the workbench unable to
        # start at all (this happened: a left-side holdout of 2.11 px vs the 2.0 px gate).
        a=self.app;self.populated()
        a.store=CalibrationStore(a.output/'library-empty','arx6')
        path=a.output/'batch-live-a'/'capture'/'wrist-fit-test.json'
        report=json.loads(path.read_text())
        report['left']['holdout']['rms_px']=3.   # over the acceptance threshold
        path.write_text(json.dumps(report))
        self.assertEqual(a.wrist_results()[0]['pair_status'],'incomplete_left')
        a.register_existing_calibrations()        # must not raise
        self.assertEqual([record['kind'] for record in a.store.list()],['head'],
                         'the head backfill is unchanged; the draft is not a version')
        self.assertEqual(len(a.wrist_results()),1,'and it stays visible as a draft')
        self.assertEqual(a.registration_skips,[],'an expected refusal is not a failure')

    def test_a_rejected_draft_never_hides_a_complete_pair(self):
        # The wrist solve used to publish every result on disk, so an earlier rejected
        # draft made a later *successful* solve report itself as REJECTED.
        a=self.app;self.populated()
        wrist_case=bridge_fixture.BridgeTests.wrist_case
        wrist_case(self,'draft')
        path=a.output/'batch-live-draft'/'capture'/'wrist-fit-test.json'
        report=json.loads(path.read_text())
        report['left']['holdout']['rms_px']=3.
        path.write_text(json.dumps(report))
        a.store=CalibrationStore(a.output/'library-mixed','arx6')
        a.register_existing_calibrations()        # must not raise
        self.assertEqual(sorted(record['kind'] for record in a.store.list()),['head','wrist'],
                         'the complete pair is published and the draft is not')
        self.assertEqual({row['pair_status'] for row in a.wrist_results()},{'incomplete_left','complete'},
                         'both stay visible in the catalog')
        self.assertEqual(a.registration_skips,[])

    def test_a_row_that_cannot_be_explained_is_reported_not_swallowed(self):
        # A complete pair whose head is nowhere on disk is a real inconsistency: it must be
        # visible in the library state instead of disappearing (or taking startup down).
        a=self.app;wrist_case=bridge_fixture.BridgeTests.wrist_case
        wrist_case(self,'orphan')
        a.store=CalibrationStore(a.output/'library-orphan','arx6')
        a.register_existing_calibrations()
        self.assertEqual(a.store.list(),[])
        self.assertEqual(len(a.registration_skips),1)
        self.assertIn('unknown calibration identifier',a.registration_skips[0]['reason'])
        self.assertEqual(a.library_state()['skipped'],a.registration_skips)

    def test_replay_during_fit_isolated_from_solver_and_live_choices(self):
        a=self.app;self.populated();self.batch('review-only')
        a.status='FITTING';a.tokens={'preview':{'revision':4}}
        fields=('selected_batch','q','jaw_mm','queue','tokens','calibration','wrist_selected','fit_result','frames','workflow','control_mode')
        before={k:copy.deepcopy(getattr(a,k)) for k in fields}
        review=a.action('/api/history-select',{'id':'review-only'})
        self.assertEqual(review['id'],'review-only')
        self.assertEqual(set(review['frames'][0]['image_urls']),{'head','left'})
        self.assertEqual(review['frames'][0]['detections_by_camera']['left']['30'],[[1,2]]*4)
        for key,value in before.items():
            if isinstance(value,np.ndarray):np.testing.assert_array_equal(getattr(a,key),value)
            else:self.assertEqual(getattr(a,key),value,key)
        self.assertEqual(a.status,'FITTING');self.assertEqual(self.calls,[])

    def test_workflow_dependency_selection_is_independent_of_live_display(self):
        a=self.app;self.populated()
        selected=copy.deepcopy(a.calibration);wrists=copy.deepcopy(a.wrist_selected);q=a.q.copy()
        alternate=self.batch('alternate-head')
        report=copy.deepcopy(a.fit_result);report['camera_T_world_optical'][0][3]=.02
        report.update(objective='sticker_corners_plus_mesh',acceptance={'image_passed':True,'image_checks':{'positive_depth':True}},optimizer={'success':True},metrics_rectified_px={'fit':{'corners':16,'rms_px':.5},'holdout':{'rms_px':.5}},mesh_metrics={'holdout':{'macro_coverage':.8,'macro_mean_penalized_px':5.}})
        (alternate/'fit.json').write_text(json.dumps(report))
        choice=next(r for r in a.calibration_results() if r['batch']=='alternate-head')
        a.tokens={'old-preview':{}}
        result=a.action('/api/workflow',{'kind':'wrist','head_result_id':choice['id']})
        self.assertEqual(result['workflow'],{'kind':'wrist','head_result_id':choice['id']})
        self.assertNotEqual(choice['id'],selected['id'])
        self.assertEqual(a.calibration,selected);self.assertEqual(a.wrist_selected,wrists)
        np.testing.assert_array_equal(a.q,q);self.assertEqual(self.calls,[])
        self.assertEqual(a.queue,[]);self.assertIsNone(a.plan);self.assertEqual(a.tokens,{})
        a.queue=[{'q':[.3]*12}];a.tokens={'fresh-preview':{}}
        a.action('/api/workflow',{'kind':'wrist','head_result_id':choice['id']})
        self.assertEqual(a.queue,[{'q':[.3]*12}]);self.assertEqual(a.tokens,{'fresh-preview':{}})
        a.action('/api/workflow',{'kind':'head'})
        self.assertEqual(a.workflow,{'kind':'head','head_result_id':None})
        self.assertEqual(a.calibration,selected);self.assertEqual(a.wrist_selected,wrists)
        self.assertEqual(self.calls,[])

    def test_wrist_dependency_rejects_corner_only_and_failed_joint_result(self):
        for parameters in ({'objective':'sticker_corners'},
                           {'objective':'sticker_corners_plus_mesh','acceptance':{'image_passed':False}}):
            with self.assertRaisesRegex(ValueError, '独立留出'):
                Workbench.require_joint_head_dependency({'parameters':parameters})
        with self.assertRaises(ValueError):
            Workbench.require_joint_head_dependency({'parameters':{
                'objective':'sticker_corners_plus_mesh','snapshot_complete':True,'acceptance':{'image_passed':True}}})

    def test_partial_simulation_head_uses_pixel_gate_without_changing_acceptance(self):
        parameters={'objective':'sticker_corners_plus_mesh','snapshot_complete':True,
            'acceptance':{'image_passed':False,'image_checks':{'positive_depth':True}},
            'optimizer':{'success':True},'mesh_metrics':{'holdout':{'macro_coverage':.7,'macro_mean_penalized_px':7.}},
            'metrics_rectified_px':{'holdout':{'rms_px':2.}}}
        row={'environment':'simulation','parameters':parameters}
        Workbench.require_joint_head_dependency(row)
        self.assertFalse(parameters['acceptance']['image_passed'])
        for field,value in [('macro_mean_penalized_px',7.001),('macro_mean_penalized_px',float('-inf')),
                            ('macro_mean_penalized_px',-1.),('macro_coverage',.699),('macro_coverage',float('inf'))]:
            bad=copy.deepcopy(row);bad['parameters']['mesh_metrics']['holdout'][field]=value
            self.assertFalse(Workbench.head_dependency_eligible(bad))
        for value in (2.001,float('nan'),-1.):
            bad=copy.deepcopy(row);bad['parameters']['metrics_rectified_px']['holdout']['rms_px']=value
            self.assertFalse(Workbench.head_dependency_eligible(bad))
        parameters['snapshot_complete']=False
        self.assertFalse(Workbench.head_dependency_eligible(row))

    def test_physical_head_mesh_thresholds_match_solver_and_install_group(self):
        from arx_env.joint_head_x5_hybrid import head_mesh_thresholds
        parameters={'objective':'sticker_corners_plus_mesh','snapshot_complete':True,
            'acceptance':{'image_passed':False,'image_checks':{'positive_depth':True}},
            'optimizer':{'success':True},'mesh_metrics':{'holdout':{'macro_coverage':.6,'macro_mean_penalized_px':10.}},
            'metrics_rectified_px':{'holdout':{'rms_px':2.}}}
        physical={'environment':'hardware','robot_source':'physical','simulation_only':False}
        row={**physical,'id':'head-gate','parameters':parameters}
        before=copy.deepcopy(row)
        self.assertEqual(head_mesh_thresholds(physical),{'holdout_mesh_max_px':10.,'holdout_mesh_min_coverage':.6})
        Workbench.require_joint_head_dependency(row)
        # Frozen wrist dependency snapshots carry source flags but no environment field.
        Workbench.require_joint_head_dependency({'parameters':{**parameters,'robot_source':'physical','simulation_only':False}})
        a=self.app
        wrist={'id':'wrist-gate','environment':'hardware','pair_status':'complete','head_result_id':'head-gate'}
        with patch.object(a,'calibration_results',return_value=[row]), patch.object(a,'wrist_results',return_value=[wrist]), \
             patch.object(a,'wrist_head_id',return_value='head-gate'), patch.object(a,'registered_records',return_value=[]), \
             patch.object(a,'installed_pair',return_value=None):
            self.assertTrue(a.calibration_groups()[0]['installable'])
        self.assertEqual(row,before)
        parameters['acceptance']['image_passed']=True
        for changed in ({'environment':'simulation'}, {'robot_source':'simulated','simulation_only':True},
                        {'robot_source':'unknown'}, {'simulation_only':None}):
            source={**physical,**changed}
            self.assertEqual(head_mesh_thresholds(source),{'holdout_mesh_max_px':7.,'holdout_mesh_min_coverage':.7})
            self.assertFalse(Workbench.head_dependency_eligible({**row,**changed}))
        for explicit in ({'holdout_mesh_max_px':7.}, {'holdout_mesh_min_coverage':.7},
                         {'holdout_mesh_max_px':float('nan')}, {'holdout_mesh_min_coverage':2.}):
            bad=copy.deepcopy(row);bad['parameters']['acceptance']['thresholds']=explicit
            self.assertFalse(Workbench.head_dependency_eligible(bad))
        loose=copy.deepcopy(row)
        loose['parameters']['acceptance']['thresholds']={'holdout_mesh_max_px':100.,'holdout_mesh_min_coverage':.1}
        loose['parameters']['mesh_metrics']['holdout']['macro_mean_penalized_px']=10.01
        self.assertFalse(Workbench.head_dependency_eligible(loose))
        for field,value in [('macro_coverage',.5999),('macro_mean_penalized_px',10.0001)]:
            bad=copy.deepcopy(row);bad['parameters']['mesh_metrics']['holdout'][field]=value
            self.assertFalse(Workbench.head_dependency_eligible(bad))

    def test_current_eligibility_does_not_mutate_immutable_library_payload(self):
        a=self.app;self.populated()
        before={p.name:p.read_bytes() for p in a.store.root.glob('v*.json')}
        for row in a.calibration_results():
            self.assertNotIn('head_dependency_eligible',row)
        with patch.object(Workbench,'head_dependency_eligible',return_value=True):
            self.assertTrue(all(r['head_dependency_eligible'] for r in a.calibration_state()['results']))
            a.register_existing_calibrations()
        a.store=CalibrationStore(a.store.root,'arx6');a.register_existing_calibrations()
        self.assertEqual({p.name:p.read_bytes() for p in a.store.root.glob('v*.json')},before)

    def test_repeated_migration_and_reopened_store_do_not_rewrite_versions(self):
        a=self.app;self.populated()
        before={p.name:(p.read_bytes(),p.stat().st_mtime_ns) for p in a.store.root.glob('v*.json')}
        records=a.store.list()
        for _ in range(2):a.register_existing_calibrations()
        a.store=CalibrationStore(a.store.root,'arx6');a.register_existing_calibrations()
        self.assertEqual(a.store.list(),records)
        self.assertEqual({p.name:(p.read_bytes(),p.stat().st_mtime_ns) for p in a.store.root.glob('v*.json')},before)
        self.assertEqual(len(records),2)

    def test_versions_and_frozen_dependency_survive_raw_deletion_and_restart(self):
        a=self.app;head,wrist,contract=self.populated()
        review=a.review_batch(str(wrist.relative_to(a.output)))
        # The wrist record is paired now: one entry covers both sides.
        self.assertEqual({r['kind'] for r in review['results']},{'head','wrist'})
        self.assertIn('left',review['fit_result'])
        unrelated=a.output/'unrelated.bin';unrelated.write_bytes(b'keep')
        # head plus one paired wrist version (was head + left + right).
        before={r['id']:r for r in a.store.list()};self.assertEqual(len(before),2)
        for path in (head,wrist):self.delete(str(path.relative_to(a.output)))
        self.assertFalse(head.exists());self.assertFalse(wrist.exists())
        self.assertEqual(a.history(),[]);self.assertEqual(unrelated.read_bytes(),b'keep')
        restarted=Workbench.__new__(Workbench)
        restarted.__dict__.update(a.__dict__)
        restarted.store=CalibrationStore(a.store.root,'arx6')
        restarted.fit_result=None;restarted.frames=[];restarted.calibration=None
        # Go through the real startup loader: the previous hand-assignment hid a crash
        # that only happens when __init__ reads the persisted paired selection.
        from joint_head_workbench import load_wrist_selection
        restarted.wrist_selected=load_wrist_selection(a.output/'wrist-selected.json')
        self.assertEqual(restarted.wrist_selected['pair_status'],'complete')
        self.assertEqual(restarted.wrist_selection_id(),restarted.wrist_selected['id'])
        self.assertEqual({r['id']:r for r in restarted.store.list()},before)
        restarted.restore_calibration()
        self.assertTrue(restarted.calibration['parameters']['snapshot_complete'])
        self.assertEqual(restarted.calibration['parameters'],contract['head_snapshot'])
        self.assertEqual(len(restarted.calibration_results()),1)
        # One paired record per solved dataset, not one per side.
        self.assertEqual(len(restarted.wrist_results()),1)
        for row in restarted.wrist_results():
            self.assertEqual(row['pair_status'],'complete')
            self.assertEqual(restarted.wrist_contract(row),contract)
            self.assertEqual(restarted.store.get(row['id'])['dependency_id'],contract['head_snapshot']['id'])
        self.calls.clear();restarted.restore_wrist_display({'status':'READY','mirror':None})
        self.assertEqual([p for p,_ in self.calls],['/wrist-config','/wrist-calibration','/wrist-calibration'])
        self.assertEqual(self.calls[0][1]['wrist_contract'],contract)
        with self.assertRaises(ValueError):restarted.review_batch('head-capture')

    def test_deletion_preview_rejects_changed_files_and_reused_token(self):
        a=self.app;path=self.batch('delete-me')
        preview=a.action('/api/history-delete-preview',{'id':'delete-me'})
        (path/'frame-000.png').write_bytes(b'changed image bytes')
        with self.assertRaisesRegex(ValueError,'文件变化'):
            a.action('/api/history-delete',{'id':'delete-me','token':preview['token']})
        self.assertTrue(path.exists())
        self.delete('delete-me');self.assertFalse(path.exists())
        with self.assertRaises(ValueError):
            a.action('/api/history-delete',{'id':'delete-me','token':preview['token']})

    def test_deletion_refuses_selected_source_busy_state_and_symlink(self):
        a=self.app;path=self.batch('protected');a.selected_batch='protected'
        with self.assertRaisesRegex(ValueError,'栏目中使用'):
            a.action('/api/history-delete-preview',{'id':'protected'})
        a.selected_batch='other';a.status='FITTING'
        with self.assertRaisesRegex(ValueError,'进行中'):
            a.action('/api/history-delete-preview',{'id':'protected'})
        a.status='READY';sentinel=a.output/'sentinel.bin';sentinel.write_bytes(b'untouched')
        (path/'linked.bin').symlink_to(sentinel)
        with self.assertRaisesRegex(ValueError,'symlink'):
            a.action('/api/history-delete-preview',{'id':'protected'})
        self.assertTrue(path.exists());self.assertEqual(sentinel.read_bytes(),b'untouched')
        for identity in ('../outside',str(path),'versions'):
            with self.assertRaises(ValueError):a.action('/api/history-delete-preview',{'id':identity})

    def test_store_failure_aborts_before_deleting_any_raw_files(self):
        a=self.app;head,_,_=self.populated()
        preview=a.action('/api/history-delete-preview',{'id':'head-capture'})
        before={str(p.relative_to(head)):hashlib.sha256(p.read_bytes()).hexdigest() for p in head.rglob('*') if p.is_file()}
        with patch.object(a.store,'register',side_effect=OSError('disk full')):
            with self.assertRaisesRegex(OSError,'disk full'):
                a.action('/api/history-delete',{'id':'head-capture','token':preview['token']})
        self.assertEqual({str(p.relative_to(head)):hashlib.sha256(p.read_bytes()).hexdigest() for p in head.rglob('*') if p.is_file()},before)


    def test_full_wipe_removes_every_batch_and_keeps_versions_marked_stale(self):
        a=self.app;self.populated();self.batch('extra-capture')
        # A batch whose dataset is all there is gets its now-empty directory dropped;
        # one that still holds the renamed request record keeps it (evidence wins).
        self.batch('batch-dropped/capture');self.batch('batch-kept/capture')
        (a.output/'batch-kept'/'request.json').write_text('{}')
        expected=sorted(row['id'] for row in a.history());self.assertGreaterEqual(len(expected),5)
        versions=sorted(r['version'] for r in a.library_state()['versions']);self.assertTrue(versions)
        self.assertFalse(any(r['stale'] for r in a.library_state()['versions']))
        single=a.action('/api/history-delete-preview',{'id':'head-capture'})
        # A token is bound to its scope: a one-batch preview cannot be replayed as a wipe.
        with self.assertRaisesRegex(ValueError,'文件变化'):
            a.action('/api/history-delete',{'scope':'all','token':single['token']})
        preview=a.action('/api/history-delete-preview',{'scope':'all'})
        self.assertEqual(preview['scope'],'all');self.assertEqual(sorted(preview['batches']),expected)
        self.assertGreater(preview['bytes'],0)
        self.assertEqual(set(preview['clears']),{'所选求解批次','实时叠加选择'})
        self.assertEqual(sorted(preview['preserved_versions']),versions)
        result=a.action('/api/history-delete',{'scope':'all','token':preview['token']})
        self.assertEqual(sorted(result['removed']),expected)
        self.assertFalse((a.output/'batch-dropped').exists(),'Empty batch directory is pruned')
        self.assertEqual(sorted(p.name for p in (a.output/'batch-kept').iterdir()),['request-deleted.json'])
        # Frames are gone; the versions survive and now say so.
        self.assertEqual(a.history(),[])
        self.assertEqual(sorted(r['version'] for r in a.library_state()['versions']),versions)
        self.assertTrue(all(r['stale'] for r in a.library_state()['versions']))
        self.assertIsNone(a.selected_batch);self.assertIsNone(a.calibration);self.assertEqual(a.wrist_selected,{})
        self.assertFalse(json.loads((a.output/'realtime-calibration.json').read_text()))
        self.assertFalse(json.loads((a.output/'wrist-selected.json').read_text()))
        with self.assertRaises(ValueError):
            a.action('/api/history-delete',{'scope':'all','token':preview['token']})

    def test_full_wipe_refuses_active_task_and_empty_history(self):
        a=self.app;self.populated()
        for status in ('RUNNING','PLANNING','CAPTURING','FITTING'):
            a.status=status
            with self.assertRaisesRegex(ValueError,'进行中'):
                a.action('/api/history-delete-preview',{'scope':'all'})
            with self.assertRaisesRegex(ValueError,'进行中'):
                a.action('/api/history-delete',{'scope':'all','token':'unused'})
        a.status='READY'
        preview=a.action('/api/history-delete-preview',{'scope':'all'})
        a.action('/api/history-delete',{'scope':'all','token':preview['token']})
        with self.assertRaisesRegex(ValueError,'没有可删除'):
            a.action('/api/history-delete-preview',{'scope':'all'})


if __name__=='__main__':unittest.main()
