"""Editable fake calibration restart; CPU only, using the existing real-store fixture."""
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import shutil
import sys
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'calibration'),str(ROOT/'scripts')]
import test_warm_diagnostic as warm
from a1_cal import canonical
from arx_env.joint_head_arrival_stats import ArrivalStats
from arx_env.joint_head_calibration_store import CalibrationStore
from examples.scripts.debug.joint_head_workbench import Workbench
from robot_workbench.calibration_profile import validate_resume_source,restore_calibration,build_fake_profile
from robot_workbench.resident_collection import CollectionPreparation
from robot_workbench.workflows import CalibrationWorkflow
from serve_physx_device import bootstrap_calibration
import serve_physx_device


class DevResumeTest(unittest.TestCase):
    repin=warm.WarmDiagnosticTest.repin

    def test_source_spec_is_readonly_and_admitted_without_runtime_state(self):
        from calibration_resume_source import source_spec
        before={path:warm.sha(path) for path in self.spec['input_sha256']}
        spec=source_spec(self.origin,self.source_base)
        self.assertEqual(self.validate(spec=spec)['expected_install'],self.expected)
        self.assertFalse(any('workbench-state' in path or Path(path).name=='.lock'
                             for path in spec['input_sha256']))
        self.assertEqual(before,{path:warm.sha(path) for path in before})

    def setUp(self):
        warm.WarmDiagnosticTest.setUp(self)
        self.spec['mode']='dev_resume';self.new['robot_id']=self.config['robot_id']
        warm.write(self.current/'registration.json',self.registration)
        for kind in ('head','wrist'):
            capture=self.capture/('batch-'+kind)/'capture'
            data=json.loads((capture/'dataset.json').read_text())
            data['frames']=[dict(id='frame-0',image='frame-0.png',q_reported_rad=[0.]*12,
                camera_images={'left':'frame-left.png','right':'frame-right.png'})]
            warm.write(capture/'dataset.json',data)
            for filename in ('frame-0.png','frame-left.png','frame-right.png'):(capture/filename).write_bytes(b'CPU image fixture')
            record=json.loads((self.active/(kind+'.json')).read_text())
            record['payload']['row'].update(id=record['id'],batch='batch-'+kind+'/capture',environment='hardware',
                file='fit.json' if kind=='head' else 'batch-wrist/capture/wrist-fit.json')
            record.pop('sha256');record['sha256']=hashlib.sha256(canonical(record)).hexdigest()
            warm.write(self.active/(kind+'.json'),record);warm.write(self.active.parent/(record['version']+'.json'),record)
            manifest=json.loads((self.active/'manifest.json').read_text());manifest[kind]['sha256']=record['sha256']
            warm.write(self.active/'manifest.json',manifest)
        self.expected['manifest_sha256']=warm.sha(self.active/'manifest.json');self.spec['expected_install']=deepcopy(self.expected)
        stats=ArrivalStats(self.origin/'calibrations/diagnostics/arrival-errors/simulated',self.config['robot_id'],'simulated')
        self.event=dict(event_id='old-owner:1',batch_id='batch-head/capture',workflow='head',plan_sha256='plan',
            status='success',timestamp_ns=1,owner_pid=1,command_epoch=0,q_target_rad=[0.]*12,q_actual_rad=[.001]*12,
            clock=deepcopy(self.config['clock']))
        stats.record(self.event)
        warm.write(self.origin/'calibration-workbench/workbench-state.json',{'tokens':'must not restore','queue':['old plan']})
        self.pin_all()

    def pin_all(self):
        self.repin()
        self.spec['input_sha256'].update({str(path):warm.sha(path) for path in self.origin.rglob('*')
            if path.is_file() and path.name!='.lock' and path.suffix!='.tmp'})

    def validate(self,**kwargs):
        return validate_resume_source(kwargs.get('spec',self.spec),kwargs.get('base',self.base),
            kwargs.get('nominal',self.config),kwargs.get('markers',self.markers),self.current)

    def restore(self):
        source=self.validate()
        self.new['calibration_resume']=restore_calibration(source,self.new,self.current)
        warm.write(self.current/'calibration-config.json',self.new)
        return source

    def workbench(self):
        app=Workbench.__new__(Workbench);app.output=self.current/'calibration-workbench';app.config=self.new
        app.calibration_root=self.current/'calibrations';app.control_mode='hardware_sync';app.geometry_id='current-workbench-geometry'
        app.hardware_store=CalibrationStore(app.calibration_root/'hardware',self.new['robot_id'],'hardware')
        app.store=CalibrationStore(app.calibration_root/'simulation',self.new['robot_id'],'simulation')
        return app

    def build(self,name,expected=None):
        result=build_fake_profile(self.current/'calibration-config.json',self.current/'calibration-workbench',warm.PROFILE,
            self.registration,self.current/name,expected_install=expected or self.expected)
        return json.loads((Path(result['output'])/'bridge.json').read_text())

    def test_copied_library_history_and_cumulative_stats_remain_independent(self):
        before={path:warm.sha(path) for path in self.spec['input_sha256']};source=self.restore();app=self.workbench()
        self.assertTrue(app.installed_pair()['verified'])
        self.assertEqual({row['id'] for row in app.calibration_results()+app.wrist_results()},{'current-head','current-wrist'})
        review=app.review_batch('batch-head/capture')
        self.assertEqual(set(review['frames'][0]['image_urls']),{'head','left','right'})
        self.assertEqual(app.calibration_store_record('current-head')['sha256'],source['records']['head']['sha256'])
        for relative in source['copy_files']:
            old=self.origin/relative;new=self.current/relative
            self.assertNotEqual(old.stat().st_ino,new.stat().st_ino)
            if new.name!='summary.json':self.assertEqual(old.read_bytes(),new.read_bytes())
        self.assertFalse((app.output/'workbench-state.json').exists())
        stats=ArrivalStats(self.current/'calibrations/diagnostics/arrival-errors/simulated',self.new['robot_id'],'simulated')
        self.assertEqual(stats.summary()['total_count'],1)
        stats.record({**self.event,'event_id':'new-owner:1','owner_pid':2,'clock':self.new['clock']})
        shutil.rmtree(app.output/'batch-head')
        self.assertEqual(stats.summary()['total_count'],2)
        self.assertTrue(app.installed_pair()['verified'])
        self.assertEqual(before,{path:warm.sha(path) for path in before})
        self.assertNotIn('isaaclab.app',sys.modules)

    def new_install(self,*,head=False):
        app=self.workbench();records={kind:app.hardware_store.get('current-'+kind) for kind in ('head','wrist')}
        selected={}
        for kind in (('head','wrist') if head else ('wrist',)):
            record=records[kind];identity=('fresh-' if head else 'new-')+kind
            batch='batch-'+identity+'/capture';payload=deepcopy(record['payload'])
            data=json.loads((self.capture/('batch-'+kind)/'capture/dataset.json').read_text());data['clock']=self.new['clock']
            if kind=='wrist':
                payload['contract']['head_snapshot']['id']='fresh-head' if head else 'current-head'
                data['wrist_contract']=payload['contract']
            warm.write(app.output/batch/'dataset.json',data);warm.write(app.output/batch/'completion.json',{'passed':True})
            payload['row'].update(id=identity,batch=batch)
            selected[kind]=app.hardware_store.register(kind,identity,payload,batch,environment='hardware',
                dependency_id=('fresh-head' if head else 'current-head') if kind=='wrist' else None)
        installed={**records,**selected}
        app.hardware_store.install(installed['head'],installed['wrist'],geometry_id=app.geometry_id,robot_source='simulated')
        return dict(head_id=installed['head']['id'],wrist_id=installed['wrist']['id'],
            manifest_sha256=warm.sha(self.current/'calibrations/hardware/active/manifest.json'))

    def test_profiles_keep_old_old_mixed_and_new_new_capture_clocks(self):
        self.restore();old=self.build('old-pair')
        self.assertEqual(old['calibration_capture_clock'],self.config['clock'])
        self.assertEqual(old['observation_clock'],self.new['clock']);self.assertFalse(old['fresh_C_completed'])
        mixed=self.build('mixed-pair',self.new_install())
        self.assertIsNone(mixed['calibration_capture_clock']);self.assertFalse(mixed['fresh_C_completed'])
        self.assertEqual(mixed['calibration_capture_clocks'],dict(head=self.config['clock'],wrist=self.new['clock']))
        self.assertEqual(mixed['calibration_origins']['head']['source_run'],str(self.origin))
        self.assertIsNone(mixed['calibration_origins']['wrist'])
        fresh=self.build('fresh-pair',self.new_install(head=True))
        self.assertTrue(fresh['fresh_C_completed']);self.assertFalse(fresh['empty_store_qualification'])
        self.assertEqual(fresh['calibration_capture_clock'],self.new['clock'])
        graph='\n'.join(path.read_text() for path in (self.current/'mixed-pair').rglob('*.json'))
        for forbidden in ('truth-fixture.json','plant-fixture.json','plant_joint_offsets_rad',str(self.source_base)):
            self.assertNotIn(forbidden,graph)

    def test_only_exact_imported_batches_can_use_old_clocks(self):
        self.restore();path=self.current/'calibration-workbench/batch-head/capture/dataset.json';raw=path.read_bytes()
        data=json.loads(raw);data['clock']=self.new['clock'];warm.write(path,data)
        with self.assertRaisesRegex(ValueError,'pinned original batch'):self.build('relabelled')
        path.write_bytes(raw);expected=self.new_install()
        path=self.current/'calibration-workbench/batch-new-wrist/capture/dataset.json'
        data=json.loads(path.read_text());data['clock']=self.config['clock'];warm.write(path,data)
        with self.assertRaisesRegex(ValueError,'completed current-run fake capture'):self.build('unadmitted-old-clock',expected)

    def test_bad_source_pins_fixture_symlinks_and_identity_are_rejected(self):
        image=self.capture/'batch-head/capture/frame-0.png';spec=deepcopy(self.spec);spec['input_sha256'].pop(str(image))
        with self.assertRaisesRegex(ValueError,'omits'):self.validate(spec=spec)
        with self.assertRaisesRegex(ValueError,'fixture'):self.validate(base={**self.base,'plant_joint_offsets_rad':[.003]+[0.]*11})
        link=self.capture/'batch-head/capture/link.png';link.symlink_to(image)
        with self.assertRaisesRegex(ValueError,'symlink'):self.validate()
        link.unlink();source=self.validate()
        for field,value in [('robot_id','fake-other'),('clock',self.config['clock']),('environment','physical')]:
            with self.subTest(field=field),self.assertRaisesRegex(ValueError,'same fake robot'):
                restore_calibration(source,{**self.new,field:value},self.current)
        camera=Path(self.new['hardware_wrist_camera_files']['left']);raw=camera.read_bytes();value=json.loads(raw);value['fx']+=1
        warm.write(camera,value)
        with self.assertRaisesRegex(ValueError,'intrinsics'):restore_calibration(source,self.new,self.current)
        camera.write_bytes(raw);self.registration['T_environment_calibration_world'][0][3]+=.01
        warm.write(self.current/'registration.json',self.registration)
        with self.assertRaisesRegex(ValueError,'registration'):restore_calibration(source,self.new,self.current)

    def test_bootstrap_uses_real_install_for_runtime_and_editable_preparation(self):
        self.restore();app=self.workbench();runtime=Mock();notify=Mock()
        snapshot=bootstrap_calibration(app,runtime=runtime,notify=notify)
        runtime.notify_calibration_installed.assert_called_once_with(CalibrationWorkflow.install_identity(snapshot))
        notify.assert_called_once_with(snapshot)
        device=NS(runtime=None);application=Mock()
        with patch.object(CollectionPreparation,'_run',lambda self:None):
            prep=CollectionPreparation(contract={},output=self.current,base=self.base,device=device,application=application,
                ports={},directories={},control_url='http://127.0.0.1:1',token_file='unused',
                deadline_wall_ns=time.time_ns()+120_000_000_000,installed_snapshot=app.installed_pair)
        bootstrap_calibration(app,notify=prep.notify);self.assertEqual(prep.notifications.get_nowait(),snapshot)
        self.new_install();changed=bootstrap_calibration(app,notify=prep.notify)
        self.assertEqual(prep.notifications.get_nowait(),changed)
        self.assertFalse(prep.closed.is_set());self.assertFalse(prep.diagnostic_invalidated)
        app.geometry_id='changed'
        with self.assertRaisesRegex(ValueError,'geometry'):bootstrap_calibration(app,runtime=runtime)
        self.assertEqual(runtime.notify_calibration_installed.call_count,1)

    def test_cli_dry_run_is_read_only_and_modes_are_exclusive(self):
        spec_path=self.root/'resume.json';warm.write(spec_path,self.spec)
        base_path=self.root/'base.json';base={**self.base,'lanes':{'source':{'configuration':{'cameras':{
            'calibration_binding':{'a1_sha256':self.config['asset_seal_sha256'],'geometry_id':self.config['asset_geometry_id']}}}}}}
        warm.write(base_path,base)
        output=self.root/'cli-new';argv=['serve','--base',str(base_path),'--nominal-config',str(self.root/'nominal-config.json'),
            '--output',str(output),'--gpu','1','--calibration-resume-source',str(spec_path),'--dry-run']
        nominal={**self.config,'candidate':str(self.root/'candidate.json')};warm.write(Path(nominal['candidate']),{})
        with patch.object(serve_physx_device,'nominal_inputs',return_value=nominal),patch.object(serve_physx_device,'preflight',return_value={}), \
                patch.object(serve_physx_device,'printed_marker_fixture',return_value=self.markers), \
                patch('robot_workbench.calibration_profile.validate_resume_source',return_value=self.validate()) as validate, \
                patch.object(serve_physx_device,'run') as run,patch.object(sys,'argv',argv),patch('sys.stdout',new_callable=io.StringIO) as stdout:
            serve_physx_device.main();run.assert_not_called();validate.assert_called_once()
            self.assertEqual(json.loads(stdout.getvalue())['calibration_resume']['mode'],'dev_resume')
        self.assertFalse(output.exists())
        with patch('robot_workbench.resident_collection.load_collection_contract',return_value={'diagnostic_calibration_source':self.spec}), \
                patch.object(sys,'argv',argv+['--collection-contract','unused.json']):
            with self.assertRaisesRegex(ValueError,'mutually exclusive'):serve_physx_device.main()


if __name__=='__main__':unittest.main()
