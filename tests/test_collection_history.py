"""Small synthetic bytes exercise real archived identity checks; no GPU/video qualification."""
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import subprocess
import sys
import shutil
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from robot_workbench.collection_history import CAMERAS,source_spec,validate_source,import_history,CollectionHistory,CollectionHistoryGroup,_pin
import serve_physx_device


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,sort_keys=True,allow_nan=False))


class CollectionHistoryTest(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name).resolve()
        self.origin=self.root/'source';self.origin.mkdir();self.collection=self.origin/'collection-fixture'
        self.job=self.collection/'playground/jobs'/('a'*32);self.job.mkdir(parents=True)
        self.raw=self.origin/'collection-raw'/('b'*32)/'raw';self.raw.mkdir(parents=True)
        self.output=self.root/'new-run/collection-history';self.robot='fake-physx-same-device'
        self.clock=dict(environment='fake',clock_domain='simulation',source_clock_id='old-physical-clock')
        serials=dict(head='head-serial',left='left-serial',right='right-serial')
        binding=dict(environment='fake',simulation_only=True,robot_id=self.robot,camera_serials=serials,
            head_id='1'*64,wrist_id='2'*64,calibration_snapshot_id='3'*64,calibration_profile_digest='4'*64)
        bridge=dict(head_id=binding['head_id'],wrist_id=binding['wrist_id'],calibration_snapshot_id='3'*64,
            active_sha256={'manifest.json':'3'*64})
        source=dict(robot_id=self.robot,camera_serials=serials,clock=self.clock,run_id='original-observation')
        source_digest=hashlib.sha256(json.dumps(source,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
        write(self.collection/'calibration/acquisition-binding.json',binding);write(self.collection/'calibration/bridge.json',bridge)
        write(self.raw/'source.json',source)
        write(self.raw/'binding.json',dict(binding,source_digest=source_digest,clock=self.clock,initial_state={'old':True}))
        instance=dict(execution_instance_id='c'*32,command_epoch=4,runtime_nonce='original-runtime')
        write(self.job/'execution-instance.json',instance)
        identities=dict(source_run_id='original-observation',plan_run_id='original-plan',timeline_digest='5'*64)
        raw=dict(identities,environment='fake',simulation_only=True,clock=self.clock,source_clock_id=self.clock['source_clock_id'],
            robot_id=self.robot,execution_owner=instance,execution_timeline_digest='6'*64,
            controller_outcome='completed',capture_status='complete',validation={'status':'passed'},
            files={name:_pin(self.raw/name) for name in ('source.json','binding.json')})
        write(self.raw/'manifest.json',raw);write(self.job/'recording-directory.json',{'directory':str(self.raw)})
        native=dict(instance,schema_version='owner_physx_native_task.v1',environment='fake',simulation_only=True,hardware_qualified=False,
            source_clock_id=self.clock['source_clock_id'],execution_timeline_digest='6'*64,parent_timeline_digest='5'*64,
            source_digest=source_digest,task_outcome='success',task_qualified=True,motion_outcome='completed')
        native_path=self.origin/'native-tasks'/(instance['execution_instance_id']+'.json');write(native_path,native)
        proof=dict(native,scope='L2_same_plant',receipt_verified=True,receipt_path=str(native_path),receipt_sha256=_pin(native_path)['sha256'])
        write(self.job/'native-task-verification.json',{'task_outcome':'success','native_task':proof})
        exported=dict(identities,environment='fake',simulation_only=True,source_clock=self.clock,
            controller_outcome='completed',capture_status='complete',raw_manifest_sha256=_pin(self.raw/'manifest.json')['sha256'],files={})
        simulation=dict(identities,execution_timeline_digest='6'*64,status='completed',scope='L3_auxiliary_independent_target',
            same_plant=False,task_success_evidence=False,files={})
        for camera in CAMERAS:
            for directory,filename,manifest,key in ((self.job/'physical-video','source-'+camera+'.mp4',exported,'source-'+camera+'.mp4'),
                    (self.job,'sim-'+camera+'.mp4',simulation,camera+'.mp4')):
                directory.mkdir(exist_ok=True);path=directory/filename
                path.write_bytes(b'CPU-only synthetic MP4 transport fixture '+filename.encode());manifest['files'][key]=_pin(path)
        write(self.job/'physical-video/manifest.json',exported);write(self.job/'simulation-result.json',simulation)
        for path in (self.raw/'camera/forbidden.zip',self.collection/'model/weights.bin',
                self.job.parent.parent/'prepared-plan.json',self.origin/'collection-gateway-token'):
            path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(b'not history input')

    def spec(self):return source_spec(self.origin,[self.job])

    def test_retained_catalog_survives_original_removal_and_deduplicates_restarts(self):
        history=import_history(validate_source(self.spec()),self.output);rows=history.results()
        shutil.rmtree(self.origin)  # Only this test's synthetic disposable source.
        group=CollectionHistoryGroup([CollectionHistory(self.output),CollectionHistory(self.output)])
        self.assertEqual(group.results(),rows)
        for row in rows:self.assertTrue(group.media(row['id']).is_file())
        self.assertIsNone(group.media('not-a-result'))

    def test_readonly_history_preserves_identity_and_six_scoped_pairs_without_execution_state(self):
        spec=self.spec();before={str(path):_pin(path) for path in self.origin.rglob('*') if path.is_file()}
        checked=validate_source(spec,robot_id=self.robot);history=import_history(checked,self.output)
        rows=history.results();self.assertEqual(len(rows),6)
        self.assertEqual({row['run_id'] for row in rows},{str(self.job)})
        self.assertEqual({(row['media']['camera'],row['media']['role']) for row in rows},{(c,r) for c in CAMERAS for r in ('source','target')})
        for row in rows:
            self.assertTrue(row['readonly']);self.assertEqual(row['details']['source_clock'],self.clock)
            self.assertEqual(row['details']['installation']['manifest_sha256'],'3'*64)
            self.assertEqual(row['details']['execution_instance_id'],'c'*32)
            self.assertEqual(row['details']['outcomes']['simulation_scope'],'L3_auxiliary_independent_target')
            self.assertEqual(row['media']['run_id'],str(self.job));self.assertNotIn('execute',row)
            path=history.media(row['id']);self.assertEqual(path.read_bytes(),Path(row['details']['source_file']).read_bytes())
            self.assertNotEqual(path.stat().st_ino,Path(row['details']['source_file']).stat().st_ino)
        self.assertIsNone(history.media('unknown'));self.assertEqual(CollectionHistory(self.output).results(),rows)
        self.assertFalse(any(path.name in ('forbidden.zip','weights.bin','prepared-plan.json','collection-gateway-token') for path in self.output.rglob('*')))
        self.assertEqual(before,{str(path):_pin(path) for path in self.origin.rglob('*') if path.is_file()})
        rows.clear();self.assertEqual(len(history.results()),6)
        path=history.media(history.results()[0]['id']);path.write_bytes(b'tampered')
        with self.assertRaisesRegex(ValueError,'SHA or size'):history.media(history.results()[0]['id'])

    def test_pin_set_size_sha_and_cross_robot_rejection(self):
        original=self.spec()
        for edit in ('missing','extra','size','sha'):
            spec=deepcopy(original);key=next(iter(spec['files']))
            if edit=='missing':spec['files'].pop(key)
            elif edit=='extra':spec['files']['collection-gateway-token']=_pin(self.origin/'collection-gateway-token')
            else:spec['files'][key]['size_bytes' if edit=='size' else 'sha256']=0 if edit=='size' else '0'*64
            with self.subTest(edit=edit),self.assertRaisesRegex(ValueError,'file set, SHA or size'):validate_source(spec)
        with self.assertRaisesRegex(ValueError,'another virtual robot'):validate_source(original,robot_id='fake-other')
        checked=validate_source(original);checked['entries'][0]['details']['outcomes']['task']='forged'
        with self.assertRaisesRegex(ValueError,'identity changed'):import_history(checked,self.output)
        self.assertFalse(self.output.exists())

    def test_symlinks_paths_and_changed_original_evidence_are_refused(self):
        video=self.job/'sim-base_0.mp4';raw=video.read_bytes();video.unlink();outside=self.root/'outside.mp4';outside.write_bytes(raw);video.symlink_to(outside)
        with self.assertRaisesRegex(ValueError,'regular file'):self.spec()
        video.unlink();video.write_bytes(raw)
        for job in (str(self.root/'outside-job'),'../escape','collection-fixture/playground/jobs/../../escape'):
            with self.subTest(job=job),self.assertRaises(ValueError):source_spec(self.origin,[job])
        spec=self.spec();path=self.job/'execution-instance.json';value=json.loads(path.read_text());value['runtime_nonce']='another-runtime';write(path,value)
        with self.assertRaisesRegex(ValueError,'original execution'):validate_source(spec)

    def test_clock_install_and_l3_cannot_relabel_original_task(self):
        paths=[(self.collection/'calibration/bridge.json','head_id','9'*64),
            (self.job/'physical-video/manifest.json','source_clock',{**self.clock,'source_clock_id':'new-clock'}),
            (self.job/'simulation-result.json','task_success_evidence',True)]
        for path,key,value in paths:
            original=path.read_bytes();changed=json.loads(original);changed[key]=value;write(path,changed)
            with self.subTest(field=key),self.assertRaises(ValueError):self.spec()
            path.write_bytes(original)

    def test_source_cli_and_serve_dry_run_do_not_copy_or_start_gpu(self):
        printed=subprocess.run([sys.executable,'-B',str(ROOT/'scripts/collection_history_source.py'),
            '--source-run',str(self.origin),'--job',str(self.job)],check=True,capture_output=True,text=True)
        spec=json.loads(printed.stdout);self.assertEqual(spec,self.spec())
        spec_path=self.root/'history-spec.json';write(spec_path,spec)
        base=self.root/'base.json';write(base,{'pipeline_config':{},'lanes':{'source':{'configuration':{'cameras':{
            'calibration_binding':{'a1_sha256':'seal','geometry_id':'geometry'}}}}}})
        nominal_path=self.root/'nominal.json';resume_path=self.root/'resume.json';write(nominal_path,{});write(resume_path,{})
        output=self.root/'entry';argv=['serve','--base',str(base),'--nominal-config',str(nominal_path),'--output',str(output),
            '--gpu','1','--calibration-resume-source',str(resume_path),'--collection-history-source',str(spec_path),
            '--collection-contract',str(self.root/'cpu-contract.json')]
        candidate=self.root/'candidate.json';write(candidate,{})
        nominal=dict(asset_seal_sha256='seal',asset_geometry_id='geometry',candidate=str(candidate))
        resume=dict(source_run=str(self.origin),origin_robot_id=self.robot,expected_install={})
        with patch.object(serve_physx_device,'nominal_inputs',return_value=nominal),patch.object(serve_physx_device,'preflight',return_value={}), \
                patch('robot_workbench.resident_collection.load_collection_contract',return_value=dict(
                    data_plane=dict(schema='workbench.ros-data-plane.v1',namespace='/cpu_history',domain_id=126),
                    collection_setup=dict(detection_runtime_root=str(self.root/'runtime')))), \
                patch('detector_adapter.preflight_release.preflight_release',return_value={}), \
                patch.object(serve_physx_device,'printed_marker_fixture',return_value=[]), \
                patch('robot_workbench.calibration_profile.validate_resume_source',return_value=resume), \
                patch.object(serve_physx_device,'run') as run:
            with patch.object(sys,'argv',argv+['--dry-run']),patch('sys.stdout',new_callable=io.StringIO) as stdout:
                serve_physx_device.main()
            self.assertEqual(json.loads(stdout.getvalue())['collection_history']['videos'],6)
            self.assertFalse(output.exists());run.assert_not_called()
            with patch.object(sys,'argv',argv):serve_physx_device.main()
            self.assertEqual(len(run.call_args.args[0].collection_history.results()),6)
        self.assertNotIn('isaaclab.app',sys.modules)

if __name__=='__main__':unittest.main()
