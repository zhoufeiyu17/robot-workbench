"""Startup input/configuration checks only; importing the CLI must not load Isaac."""
import hashlib
import errno
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from copy import deepcopy
from contextlib import redirect_stdout
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
import serve_physx_device
from serve_physx_device import configure_calibration,nominal_inputs,printed_marker_fixture,plant_joint_offsets
from robot_workbench.physx_camera import camera_intrinsics,CAMERAS
from robot_workbench.sample_clock import clock_spec


class RetainedInferenceTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name).resolve();self.source=self.root/'previous-inference';self.source.mkdir()
        self.video=self.source/'previous-run/videos/source-base_0.mp4';self.video.parent.mkdir(parents=True)
        self.video.write_bytes(b'\x00\x00\x00\x18ftypmp42\x00\x00\x00\x00mp42isom')
        self.settings=dict(execute_steps=45,revision=7)
        original_receipt=self.source/'failed-run/receipt.json';original_receipt.parent.mkdir()
        original_receipt.write_text(json.dumps(dict(status='original failure',execution_instance_id='failed-run')))
        self.history=dict(schema='inference_history.v1',entries=[dict(path='previous-run/videos/source-base_0.mp4',
            result=dict(id='old-video',run_id='previous-run',kind='recording',status='valid')),
            dict(path=None,result=dict(id='old-failure',run_id='failed-run',status='invalid',
                 details=dict(receipt=str(self.source/'failed-run/receipt.json'))))])
        self.write_metadata()

    def write_metadata(self):
        (self.source/'execution-settings.json').write_text(json.dumps(self.settings))
        (self.source/'history.json').write_text(json.dumps(self.history))

    def test_restore_preserves_settings_results_media_and_old_receipt_paths_without_model_work(self):
        from types import SimpleNamespace
        from robot_workbench.inference_workflow import InferenceWorkflow
        before={name:(self.source/name).read_bytes() for name in ('execution-settings.json','history.json')}
        source=serve_physx_device.inference_source(self.source);output=self.root/'new-run/inference'
        with patch('subprocess.Popen',side_effect=AssertionError('restore must not launch a process')):
            receipt=serve_physx_device.restore_inference(source,output)
            flow=InferenceWorkflow(SimpleNamespace(id='fake-old',environment='fake'),output,camera_service=None,
                model_endpoint='http://127.0.0.1:1',model_token_file=self.root/'unused',profile_provider=lambda:None)
        self.assertEqual(flow.execute_steps,45);self.assertEqual(flow.execution_revision,7)
        self.addCleanup(flow.close)
        self.assertEqual([row['id'] for row in flow.results()],['old-video','old-failure'])
        self.assertEqual(flow.results()[1]['details']['receipt'],self.history['entries'][1]['result']['details']['receipt'])
        self.assertEqual(json.loads(Path(flow.results()[1]['details']['receipt']).read_text())['status'],'original failure')
        restored=flow.media('old-video');self.assertEqual(restored.read_bytes(),self.video.read_bytes())
        self.assertEqual(restored.stat().st_ino,self.video.stat().st_ino)
        self.assertTrue(receipt['original_receipt_paths_preserved']);self.assertIn('no new execution',receipt['scope'])
        for name,raw in before.items():self.assertEqual((output/name).read_bytes(),raw)
        (output/'execution-settings.json').write_text(json.dumps(dict(execute_steps=10,revision=8)))
        self.assertEqual((self.source/'execution-settings.json').read_bytes(),before['execution-settings.json'])
        with self.assertRaisesRegex(ValueError,'new independent'):serve_physx_device.restore_inference(source,output)

    def test_cross_filesystem_playback_copy_is_verified_without_reexport(self):
        source=serve_physx_device.inference_source(self.source);output=self.root/'copied'
        with patch('serve_physx_device.os.link',side_effect=OSError(errno.EXDEV,'different filesystems')):
            receipt=serve_physx_device.restore_inference(source,output)
        self.assertEqual((output/'previous-run/videos/source-base_0.mp4').read_bytes(),self.video.read_bytes())
        self.assertEqual(receipt['files']['previous-run/videos/source-base_0.mp4'],'copy')

    def test_sqlite_restore_preserves_all_pages_and_copies_an_independent_writable_index(self):
        from robot_workbench.inference_history import InferenceHistory, DATABASE
        history=InferenceHistory(self.source)
        history.append((dict(id=f'new-{i}',run_id=f'new-run-{i}'),None) for i in range(50))
        source=serve_physx_device.inference_source(self.source)
        self.assertIn(DATABASE,source['files']);self.assertIn('previous-run/videos/source-base_0.mp4',source['files'])
        output=self.root/'sqlite-restored';serve_physx_device.restore_inference(source,output)
        restored=InferenceHistory(output)
        self.assertEqual(restored.page()['total_results'],52)
        self.assertEqual(restored.media('old-video').read_bytes(),self.video.read_bytes())
        self.assertNotEqual((output/DATABASE).stat().st_ino,(self.source/DATABASE).stat().st_ino)
        restored.append([(dict(id='only-in-new-service'),None)])
        self.assertEqual(InferenceHistory(self.source).page()['total_results'],52)
        self.assertEqual(restored.page()['total_results'],53)
        self.assertEqual((output/'history.json').read_bytes(),(self.source/'history.json').read_bytes())

    def test_tampered_metadata_media_missing_files_and_duplicate_ids_fail_before_copy(self):
        source=serve_physx_device.inference_source(self.source)
        for path in (self.video,self.source/'history.json',self.source/'execution-settings.json'):
            original=path.read_bytes();path.write_bytes(original+b' ')
            with self.subTest(file=path.name),self.assertRaisesRegex(ValueError,'SHA or size changed'):
                serve_physx_device.restore_inference(source,self.root/'bad')
            self.assertFalse((self.root/'bad').exists());path.write_bytes(original)
        self.history['entries'][1]['result']['id']='old-video';self.write_metadata()
        with self.assertRaisesRegex(ValueError,'duplicate'):serve_physx_device.inference_source(self.source)
        self.history['entries'][1]['result']['id']='old-failure';self.write_metadata();self.video.unlink()
        with self.assertRaisesRegex(ValueError,'nonempty regular'):serve_physx_device.inference_source(self.source)

    def test_absolute_parent_and_symlink_media_paths_are_rejected(self):
        outside=self.root/'outside.mp4';outside.write_bytes(b'outside')
        for value in (str(outside),'../outside.mp4','previous-run/../../outside.mp4','./previous-run/videos/source-base_0.mp4'):
            self.history['entries'][0]['path']=value;self.write_metadata()
            with self.subTest(path=value),self.assertRaisesRegex(ValueError,'path escapes'):serve_physx_device.inference_source(self.source)
        self.history['entries'][0]['path']='previous-run/videos/source-base_0.mp4';self.write_metadata()
        self.video.unlink();self.video.symlink_to(outside)
        with self.assertRaisesRegex(ValueError,'regular file'):serve_physx_device.inference_source(self.source)

    def test_only_existing_settings_are_retained_and_empty_source_creates_no_manifest(self):
        (self.source/'history.json').unlink();source=serve_physx_device.inference_source(self.source)
        self.assertEqual(set(source['files']),{'execution-settings.json'})
        output=self.root/'settings-only';serve_physx_device.restore_inference(source,output)
        self.assertFalse((output/'history.json').exists())
        (self.source/'execution-settings.json').unlink();self.assertIsNone(serve_physx_device.inference_source(self.source))


class ResidentConfigurationTest(unittest.TestCase):
    plane=dict(schema='workbench.ros-data-plane.v1',namespace='/fake_test',domain_id=126)
    def test_both_approved_compute_targets_pass_fake_dry_run_without_isaac(self):
        from local_execution import TARGET,PHYSICAL_TARGET
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);base=root/'base.json';nominal=root/'nominal.json';template=root/'template.json'
            runtime=root/'runtime';runtime.mkdir();nominal.write_text('{}');template.write_text('{}')
            base.write_text(json.dumps(dict(pipeline_config={},lanes={'source':{'configuration':{'cameras':{
                'calibration_binding':{'a1_sha256':'seal','geometry_id':'geometry'}}}}})))
            for target in (TARGET,PHYSICAL_TARGET):
                contract=dict(schema_version='anyverse.dsw-codex-experiment.v1',
                    data_plane=self.plane,
                    identity=dict(session_id='cpu-dry-run',dsw_instance_id=target['instance_id']),
                    resources=dict(gpu_index=target['gpu_index'],gpu_uuid=target['gpu_uuid'],
                        cuda_visible_devices=[target['gpu_index']],max_workers=4,max_cpu_threads=4,
                        service_lifetime_seconds=600,max_wall_time_minutes=11),
                    filesystem=dict(writable_roots=[str(root)]),collection_setup=dict(profile_template=str(template),
                        detection_runtime_root=str(runtime),plant_max_memory_mib=12288,min_free_memory_mib=1024))
                path=root/'contract.json';path.write_text(json.dumps(contract));output=root/('gpu'+str(target['gpu_index']))
                argv=['serve','--base',str(base),'--nominal-config',str(nominal),'--output',str(output),
                      '--gpu',str(target['gpu_index']),'--duration-wall-s','300','--collection-contract',str(path),'--dry-run']
                with patch.object(serve_physx_device,'nominal_inputs',return_value={'asset_seal_sha256':'seal','asset_geometry_id':'geometry'}),\
                        patch.object(serve_physx_device,'preflight',return_value={}),\
                        patch('detector_adapter.preflight_release.preflight_release',return_value={}),\
                        patch.object(serve_physx_device,'run') as run,patch.object(sys,'argv',argv),redirect_stdout(io.StringIO()) as stdout:
                    serve_physx_device.main()
                self.assertEqual(json.loads(stdout.getvalue())['status'],'dry-run-passed')
                run.assert_not_called();self.assertFalse(output.exists())
        self.assertNotIn('isaaclab.app',sys.modules)

    def test_trusted_fake_inference_options_keep_selection_catalog_and_recording(self):
        from robot_workbench.inference_model import BASE566_ID,BASE379_ID,model_descriptor
        bundle=model_descriptor(BASE566_ID)
        setup=dict(model_id=BASE566_ID,model_bundle=bundle,catalog_root='/approved/catalog',recording_enabled=False)
        with patch('subprocess.Popen',side_effect=AssertionError('CPU configuration must not start models')):
            options=serve_physx_device.inference_options(setup)
        self.assertEqual(options,dict(setup,disabled_grippers=[],control_mapping_pending=None,recording_backend='capture_session_v1'))
        self.assertIsNone(serve_physx_device.inference_options(dict(setup,recording_backend=None))['recording_backend'])
        self.assertEqual(serve_physx_device.inference_options({})['model_id'],BASE379_ID)
        self.assertIsNot(options['model_bundle'],bundle)
        for change in ({'recording_enabled':'false'},{'catalog_root':'relative'},
                       {'model_bundle':'path'},{'model_id':'unknown'},{'recording_backend':'unknown'}):
            with self.assertRaises(ValueError):serve_physx_device.inference_options(dict(setup,**change))

    def test_frozen_browser_ports_reach_dry_run_and_runtime_args_without_gpu(self):
        from local_execution import TARGET
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);base=root/'base.json';nominal_path=root/'nominal.json';template=root/'template.json';runtime=root/'runtime';runtime.mkdir()
            nominal_path.write_text('{}');template.write_text('{}')
            base.write_text(json.dumps(dict(pipeline_config={},lanes={'source':{'configuration':{'cameras':{
                'calibration_binding':{'a1_sha256':'seal','geometry_id':'geometry'}}}}})))
            nominal={'asset_seal_sha256':'seal','asset_geometry_id':'geometry'}
            contract=dict(schema_version='anyverse.dsw-codex-experiment.v1',identity={'session_id':'browser-fixture','instance':TARGET['instance_id']},
                data_plane=self.plane,
                filesystem={'writable_roots':[str(root)]},resources=dict(gpu_uuid=TARGET['gpu_uuid'],gpu_index=1,cuda_visible_devices=[1],
                    max_workers=4,max_cpu_threads=4,service_lifetime_seconds=600,max_wall_time_minutes=11),
                collection_setup=dict(profile_template=str(template),detection_runtime_root=str(runtime),plant_max_memory_mib=12288,min_free_memory_mib=16384),
                browser_ports=[18867],camera_spool_root=str(root/'private-spool'))
            contract['filesystem']['writable_roots'].append(contract['camera_spool_root'])
            contract_path=root/'contract.json';contract_path.write_text(json.dumps(contract))
            from robot_workbench.resident_collection import load_collection_contract
            for spool in ('relative-spool',str(root/'undeclared'),str(runtime),str(root)):
                invalid=deepcopy(contract);invalid['camera_spool_root']=spool
                contract_path.write_text(json.dumps(invalid))
                with self.assertRaisesRegex(ValueError,'camera spool'):
                    load_collection_contract(contract_path,root/'output',1,300)
            contract_path.write_text(json.dumps(contract))
            for configured in (False,True):
                output=root/('forwarded' if configured else 'default')
                argv=['serve','--base',str(base),'--nominal-config',str(nominal_path),'--output',str(output),'--gpu','1','--duration-wall-s','300']
                if configured:argv+=['--collection-contract',str(contract_path)]
                with patch.object(serve_physx_device,'nominal_inputs',return_value=nominal), \
                        patch.object(serve_physx_device,'preflight',return_value={}), \
                        patch('detector_adapter.preflight_release.preflight_release',return_value={'fixture':True}), \
                        patch.object(serve_physx_device,'run') as run:
                    stdout=io.StringIO()
                    if not configured:
                        with patch.object(sys,'argv',argv+['--dry-run']),self.assertRaisesRegex(ValueError,'ROS data-plane'):
                            serve_physx_device.main()
                        run.assert_not_called();self.assertFalse(output.exists());continue
                    with patch.object(sys,'argv',argv+['--dry-run']),redirect_stdout(stdout):serve_physx_device.main()
                    report=json.loads(stdout.getvalue())
                    self.assertEqual(report['browser_ports'],[18867] if configured else [])
                    self.assertEqual(report['browser_hosts'],['127.0.0.1:18867','localhost:18867'] if configured else [])
                    run.assert_not_called();self.assertFalse(output.exists())
                    self.assertFalse(Path(report['camera_spool_root']).exists())
                    with patch.object(sys,'argv',argv):serve_physx_device.main()
                    self.assertEqual(run.call_args.args[0].browser_ports,report['browser_ports'])
                    spool=Path(report['camera_spool_root'])
                    self.assertFalse(spool.exists())  # CommonCameraService owns exclusive creation inside run().
                    self.assertEqual(spool,(root/'private-spool' if configured else output/'camera-spool').resolve())
                    self.assertEqual(json.loads((output/'run.json').read_text())['browser_ports'],report['browser_ports'])
            self.assertNotIn('isaaclab.app',sys.modules)

    def test_persistent_cli_keeps_null_deadline_and_requires_matching_resource_contract(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);base=root/'base.json';output=root/'output';nominal=root/'nominal.json'
            base.write_text(json.dumps(dict(pipeline_config={},lanes={'source':{'configuration':{'cameras':{
                'calibration_binding':{'a1_sha256':'seal','geometry_id':'geometry'}}}}})))
            nominal.write_text('{}')
            contract={'data_plane':self.plane,'resources':{'persistent_service':True,'service_lifetime_seconds':None,'max_wall_time_minutes':120},
                'collection_setup':{'detection_runtime_root':'/fixture'}}
            retained=(root/'previous-inference').resolve();retained.mkdir()
            (retained/'execution-settings.json').write_text(json.dumps(dict(execute_steps=45,revision=9)))
            contract['maintenance_restore']=dict(inference_source=serve_physx_device.inference_source(retained))
            argv=['serve','--base',str(base),'--nominal-config',str(nominal),'--output',str(output),
                '--gpu','1','--persistent']
            with patch.object(sys,'argv',argv),self.assertRaisesRegex(ValueError,'requires a resource contract'):
                serve_physx_device.main()
            argv+=['--collection-contract',str(root/'contract.json')]
            with patch.object(serve_physx_device,'nominal_inputs',return_value={'asset_seal_sha256':'seal','asset_geometry_id':'geometry'}), \
                    patch.object(serve_physx_device,'preflight',return_value={}), \
                    patch('robot_workbench.resident_collection.load_collection_contract',return_value=contract) as load, \
                    patch('detector_adapter.preflight_release.preflight_release',return_value={}), \
                    patch.object(serve_physx_device,'run') as run:
                with patch.object(sys,'argv',argv):serve_physx_device.main()
                self.assertIsNone(load.call_args.args[3])
                args=run.call_args.args[0]
                self.assertTrue(args.persistent);self.assertIsNone(args.duration_wall_s);self.assertIsNone(args.resident_deadline_wall_ns)
                receipt=json.loads((output/'run.json').read_text())
                self.assertTrue(receipt['persistent_service']);self.assertIsNone(receipt['resident_deadline_wall_ns'])
                self.assertEqual(json.loads((output/'inference/execution-settings.json').read_text())['execute_steps'],45)
                self.assertEqual(receipt['retained_inference']['root'],str(retained))
            self.assertNotIn('isaaclab.app',sys.modules)

    def test_invalid_plant_offsets_fail_before_inputs_output_or_gpu(self):
        invalid=(None,{},'0',[],[0.]*11,[0.]*13,[True]+[0.]*11,['0']+[0.]*11,
                 [float('nan')]+[0.]*11,[float('inf')]+[0.]*11,[-float('inf')]+[0.]*11,[10**400]+[0.]*11)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);base=root/'base.json';output=root/'output'
            argv=['serve','--base',str(base),'--nominal-config',str(root/'nominal.json'),
                  '--output',str(output),'--gpu','1']
            for value in invalid:
                base.write_text(json.dumps(dict(plant_joint_offsets_rad=value)))
                for dry in (False,True):
                    with self.subTest(value=value,dry_run=dry),patch.object(sys,'argv',argv+(['--dry-run'] if dry else [])), \
                            patch.object(serve_physx_device,'nominal_inputs') as nominal,patch.object(serve_physx_device,'run') as run:
                        with self.assertRaisesRegex(ValueError,'twelve finite'):serve_physx_device.main()
                        nominal.assert_not_called();run.assert_not_called();self.assertFalse(output.exists())
            self.assertNotIn('isaaclab.app',sys.modules)

    def test_default_and_explicit_zero_preserve_existing_backend_sdk_mapping(self):
        from test_physx_boundary import LaggingPlant
        from robot_workbench.physx import PhysxBackend
        from arx_env.joint_head_jaw_mapping import command_sdk
        absent={};explicit=dict(plant_joint_offsets_rad=[0.]*12)
        before=deepcopy((absent,explicit))
        values=([0.]*12,plant_joint_offsets(absent),plant_joint_offsets(explicit))
        backends=[PhysxBackend(LaggingPlant(),joint_offsets=value,metal_gap_offset_mm=.008) for value in values]
        target=[.05]*14;target[6]=command_sdk('left',80.);target[13]=command_sdk('right',80.)
        for backend in backends:
            for side in ('left','right'):backend.send(side,target)
            backend.advance(1/240)
        for backend in backends[1:]:
            for key in ('qpos','qvel'):self.assertEqual(backend.read_calibration_state()[key],backends[0].read_calibration_state()[key])
            self.assertEqual(backend.plant.read(),backends[0].plant.read())
            self.assertEqual(backend.get_bimanual_joint_position_limits(),backends[0].get_bimanual_joint_position_limits())
            self.assertEqual(backend.gap_offset,.008)
        self.assertEqual((absent,explicit),before)

    def test_nonzero_truth_applies_once_in_original_backend_command_and_feedback(self):
        from test_physx_boundary import LaggingPlant
        from robot_workbench.physx import PhysxBackend
        from arx_env.joint_head_jaw_mapping import command_sdk
        values=[.003,0.,0.,0.,0.,0.,-.003,0.,0.,0.,0.,0.]
        base=dict(plant_joint_offsets_rad=values);before=deepcopy(base)
        plant=LaggingPlant();backend=PhysxBackend(plant,joint_offsets=plant_joint_offsets(base),metal_gap_offset_mm=.008)
        target=[.02]*14;target[6]=command_sdk('left',80.);target[13]=command_sdk('right',80.)
        for n,side in enumerate(('left','right')):
            initial=backend.read_calibration_state()['qpos']
            for j in range(6):self.assertAlmostEqual(initial[n*7+j],-values[n*6+j])
            backend.send(side,target)
            native_target=backend.pending[side][0]
            for j in range(6):self.assertAlmostEqual(native_target[j],target[n*7+j]+values[n*6+j])
            self.assertAlmostEqual(native_target[6],80.-.008)
        backend.advance(1/240)
        feedback=backend.read_calibration_state()
        for n,side in enumerate(('left','right')):
            for j in range(6):
                self.assertAlmostEqual(feedback['qpos'][n*7+j],plant.q[side][j]-values[n*6+j])
                self.assertEqual(feedback['qvel'][n*7+j],plant.v[side][j])
        self.assertEqual(base,before)

    def test_printed_face_is_outside_housing_without_changing_estimator_mount(self):
        candidate={'T_link6_marker':{'24':[[0,0,1,.1],[1,0,0,.2],[0,1,0,.3],[0,0,0,1]],
                                     '30':[[1,0,0,.4],[0,1,0,.5],[0,0,1,.6],[0,0,0,1]]}}
        before=deepcopy(candidate);markers=printed_marker_fixture(candidate)
        self.assertEqual(candidate,before)
        self.assertAlmostEqual(markers[0]['T_link_marker'][0][3],.0999)
        self.assertAlmostEqual(markers[1]['T_link_marker'][2][3],.5999)
        for marker in markers:
            original=candidate['T_link6_marker'][str(marker['id'])]
            for axis in range(3):self.assertEqual(marker['T_link_marker'][axis][:3],original[axis][:3])
            self.assertEqual(marker['print_surface_offset_m'],.0001)

    def test_nominal_files_are_sealed_and_generated_calibration_root_is_empty(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);assets=root/'nominal';assets.mkdir();output=root/'run';output.mkdir()
            files={'robot.urdf':'<robot name="fixture"/>','manifest.json':'{}','scene_seed.json':'{}',
                   'candidate.json':'{}','camera_seed.json':'{}','jaw-mapping.json':'{}'}
            for name,value in files.items():(assets/name).write_text(value)
            seal={name:hashlib.sha256((assets/name).read_bytes()).hexdigest() for name in files}
            (assets/'SHA256SUMS.json').write_text(json.dumps(seal))
            nominal=dict(model_family='ac_one',simulation_only=True,asset_geometry_id='g',
                asset_seal_sha256=hashlib.sha256((assets/'SHA256SUMS.json').read_bytes()).hexdigest(),
                **{key:str(assets/name) for key,name in dict(urdf='robot.urdf',mesh_manifest='manifest.json',
                    scene_seed='scene_seed.json',candidate='candidate.json',camera_seed='camera_seed.json',
                    jaw_mapping_file='jaw-mapping.json').items()})
            config=root/'nominal.json';config.write_text(json.dumps(nominal))
            self.assertEqual(nominal_inputs(config),nominal)
            rig={'device_calibrations':{name:dict(serial=str(i),width=4,height=3,
                 intrinsics=dict(fx=100.,fy=100.,cx=2.,cy=1.),distortion=dict(coefficients=[0.]*5))
                 for i,name in enumerate(CAMERAS.values())}}
            clock=clock_spec('fake',source_clock_id='resident-fixture')
            resident_deadline=1_900_000_000_000_000_000
            result=configure_calibration(nominal,output,camera_intrinsics(rig),dict(head=18001,left=18002,right=18003),
                18004,root/'credential-path',source_clock=clock,resident_deadline_wall_ns=resident_deadline)
            self.assertEqual(result['environment'],'fake')
            self.assertEqual(result['robot_source'],'simulated')
            self.assertEqual(result['clock'],clock)
            self.assertEqual(result['resident_deadline_wall_ns'],resident_deadline)
            self.assertEqual(list(Path(result['calibration_root']).iterdir()),[])
            self.assertNotIn('live_backend',result)
            self.assertNotIn('truth_fixture',result)
            self.assertNotIn('plant_joint_offsets_rad',result)
            self.assertTrue(Path(result['head_motion_seed']).is_file())
            (assets/'robot.urdf').write_text('changed')
            with self.assertRaisesRegex(ValueError,'differs'):nominal_inputs(config)


if __name__=='__main__':unittest.main()
