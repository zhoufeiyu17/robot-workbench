"""Fake physical receipts only: no cameras, network, model inference or robot motion."""
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path
import shlex
import subprocess
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch

from integration import _digest
from real_pipeline import bind_detection, execute_target, file_sha, import_capture, preparation_stages, run, validate_capture
from live_preview import PREVIEW_CAMERAS, PreviewPublisher, camera_frame_path, read_frame
from run_pipeline import PATH_KEYS
from test_frozen_executor import fixture


class PhysicalPreparationTest(unittest.TestCase):
    def test_independent_target_transports_all_three_cameras_and_checks_receipt(self):
        with TemporaryDirectory() as folder:
            root=Path(folder); processes=[]
            def start(config,side,label,path,camera):
                self.assertEqual((side,label),('dsw-scene','target'))
                source=camera_frame_path(root/'remote',camera)
                PreviewPublisher(source).publish(camera.encode(),{'camera':camera,'tick':5})
                process=Mock(); process.poll.return_value=0
                processes.append(process)
                path.parent.mkdir(parents=True,exist_ok=True)
                return process,io.BytesIO(source.read_bytes())
            with patch('real_pipeline.start_preview_stream',side_effect=start), \
                    patch('real_pipeline.pull_telemetry'),patch('real_pipeline.replay_receipt') as receipt, \
                    patch('real_pipeline.subprocess.run',return_value=subprocess.CompletedProcess(['fake'],0)):
                result=execute_target({},['fake'],10,{'remote_run_root':'/fake'},root/'local',io.BytesIO())
            self.assertEqual(result.returncode,0)
            receipt.assert_called_once()
            for camera in PREVIEW_CAMERAS:
                frame=read_frame(camera_frame_path(root/'local/replay-target',camera))
                self.assertEqual(frame.payload,camera.encode())
                self.assertEqual(frame.metadata['camera'],camera)
            self.assertEqual(len(processes),3)
            for process in processes: process.wait.assert_called_once()

    def test_capture_binding_and_detector_input_chain(self):
        source,_=fixture(1/30)
        with TemporaryDirectory() as folder:
            root=Path(folder); capture=root/'input'; capture.mkdir()
            (capture/'profile.json').write_text(json.dumps(source['calibration_profile']))
            serials=dict(head='fixture-head',left='fixture-left',right='fixture-right')
            rows=[]
            for n in range(6):
                path=capture/f'{n}.npz'; path.write_bytes(f'fake-npz-{n}'.encode())
                rows.append(dict(path=path.name,sha256=file_sha(path),source_stamp_ns=1+n*1_000_000_000,capture_sequence_id=n))
            manifest=dict(evidence_source='physical_rgbd',robot_id='fixture-only',camera_serials=serials,source_clock='host',frames=rows)
            (capture/'rgbd-manifest.json').write_text(json.dumps(manifest))
            binding=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture-only',camera_serials=serials,
                         calibration_profile_digest=_digest(source['calibration_profile']),rgbd_manifest_sha256=file_sha(capture/'rgbd-manifest.json'))
            self.assertEqual(validate_capture(capture,binding),manifest)
            binding_path=root/'binding.json'; binding_path.write_text(json.dumps(binding))
            import_capture(capture,binding_path,root/'capture')
            (root/'detection').mkdir()
            summary=dict(manifest_sha256='sha256:'+binding['rgbd_manifest_sha256'],
                         inputs=[{**row,'sha256':'sha256:'+row['sha256']} for row in rows])
            (root/'detection/summary.json').write_text(json.dumps(summary))
            (root/'detection/observations.jsonl').write_text('{}\n')
            bind_detection(root)
            sealed=json.loads((root/'physical-detection-binding.json').read_text())
            self.assertEqual(sealed['detection_files']['summary_sha256'],file_sha(root/'detection/summary.json'))
            (root/'capture/0.npz').write_bytes(b'substituted')
            with self.assertRaises(ValueError): bind_detection(root)
            manifest['source_clock']='simulation'
            (capture/'rgbd-manifest.json').write_text(json.dumps(manifest))
            binding['rgbd_manifest_sha256']=file_sha(capture/'rgbd-manifest.json')
            with self.assertRaises(ValueError): validate_capture(capture,binding)

    def test_stage_isolation_and_progress(self):
        config={key:'/approved/'+key for key in PATH_KEYS}
        config.update(remote_run_root='/mnt/data/feiyu.zhou/runs/robot_sim/physical-test',opening_mm=83,
                      finger_envelope_sha256='a'*64,record_raw=True,prompt_file='/approved/prompt.json')
        capture='/approved/physical-capture'; binding='/approved/accepted-binding.json'
        pins={p:'a'*64 for p in (capture+'/profile.json',capture+'/rgbd-manifest.json',binding)}
        contract=dict(filesystem={'readonly_roots':['/approved']},resources={'max_wall_time_minutes':20},provenance={'input_sha256':pins})
        stages=preparation_stages(config,contract,capture,binding)
        names=[s['name'] for s in stages]
        self.assertNotIn('replay-source',names)
        self.assertFalse(any(n.startswith('observe') for n in names))
        self.assertIn('physical-target-replay',names)
        self.assertIn('physical-capture-import',names)
        commands={s['name']:shlex.split(s['argv'][-1])[-1] for s in stages}
        self.assertIn('--physical-binding',commands['bridge'])
        self.assertIn(' && ',commands['export'])
        self.assertTrue(any('hardware_binding=' in x for x in shlex.split(commands['export'])))
        self.assertIn('--independent-replay',commands['physical-target-replay'])
        self.assertIn('--hardware-binding',commands['physical-target-replay'])
        self.assertNotIn('--barrier',commands['physical-target-replay'])
        self.assertIn('--record-raw',commands['physical-target-replay'])
        large_contract = deepcopy(contract)
        large_contract['provenance'].setdefault('input_sha256', {}).update({
            '/fixture/assets/' + str(i) + '/mesh.obj': hashlib.sha256(str(i).encode()).hexdigest()
            for i in range(2000)})
        large_stages = preparation_stages(config, large_contract, capture, binding)
        model_stage = next(s for s in large_stages if s['name'] == 'physical-model-bind')
        self.assertLess(len(model_stage['argv'][-1].encode()), 120000)
        with self.assertRaises(ValueError): preparation_stages(config,contract,capture,'/outside/binding.json')

        source,plan=fixture(1/30)
        source={k:v for k,v in source.items() if k not in ('lane','adapter','instance_id','gpu')}
        source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',robot_id='fixture-only',
                      camera_serials=dict(head='h',left='l',right='r'))
        receipt=dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='fixture-only',camera_serials=source['camera_serials'],
                     calibration_profile_digest=_digest(source['calibration_profile']),source_digest=_digest(source))
        from test_joint_space import physical_fixture
        physical_fixture(plan, source)
        payloads={'fetch-source.json':source,'fetch-plan.json':plan,'fetch-hardware-binding.json':receipt}
        video=b'fake-h264'; probe=dict(codec_name='h264',pix_fmt='yuv420p',width=640,height=480,r_frame_rate='30/1',nb_read_frames='3')
        files={f'target-{camera}.mp4':dict(size_bytes=len(video),sha256=hashlib.sha256(video).hexdigest(),probe=probe)
               for camera in ('base_0','left_wrist_0','right_wrist_0')}
        payloads['fetch-target-playback-manifest.json']=dict(schema_version='physical_target_playback.v1',evidence_source='simulation',
            source_run_id=source['run_id'],plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'],camera_frames=3,files=files)
        payloads['fetch-replay-target-execution.json']=dict(status='completed',side='dsw-scene',source_run_id=source['run_id'],
            plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'],tick_count=len(plan['ticks']))
        payloads['fetch-replay-target-performance.json']={}
        calls=[]
        def fake_run(argv,**kwargs):
            stage=stages[len(calls)]; calls.append(stage['name'])
            if stage['name'] in payloads: kwargs['stdout'].write(json.dumps(payloads[stage['name']]).encode())
            elif stage['output'] and stage['output'].endswith('.mp4'): kwargs['stdout'].write(video)
            return subprocess.CompletedProcess(argv,0)
        with TemporaryDirectory() as folder, patch('real_pipeline.load_config',return_value=(config,contract,{})), \
                patch('real_pipeline.subprocess.run',side_effect=fake_run), \
                patch('real_pipeline.execute_target',side_effect=lambda stage,argv,seconds,config,output,log:fake_run(argv,stdout=log)):
            result=run(Path(folder)/'config.json',Path(folder)/'result',capture_dir=capture,physical_binding=binding)
            self.assertEqual(result['status'],'prepared',result.get('error'))
            self.assertTrue(result['execution_requires_operator_confirmation'])
            self.assertEqual(calls,names)
            self.assertEqual(len(result['receipts']),len(stages)+1)
            self.assertTrue(all(row['actual_s']>=0 for row in result['receipts']))
            self.assertTrue((Path(folder)/'result/hardware-binding.json').is_file())
            calls.clear()
            config['render_replay']=False
            stages=preparation_stages(config,contract,capture,binding)
            self.assertIn('place',[s['name'] for s in stages])
            self.assertFalse(any('replay' in s['name'] or 'playback' in s['name'] for s in stages))
            simple=run(Path(folder)/'config.json',Path(folder)/'no-render',capture_dir=capture,physical_binding=binding)
            self.assertEqual(simple['status'],'prepared',simple.get('error'))
            self.assertIsNone(simple['target_playback'])
            self.assertFalse((Path(folder)/'no-render/target-playback').exists())
            calls.clear()
            payloads['fetch-source.json']={**source,'evidence_source':'simulation'}
            failed=run(Path(folder)/'config.json',Path(folder)/'bad-result',capture_dir=capture,physical_binding=binding)
            self.assertEqual(failed['status'],'failed')
            self.assertFalse((Path(folder)/'bad-result/source.json').exists())
            self.assertTrue((Path(folder)/'bad-result/.artifacts/source.json').exists())
            self.assertEqual(failed['receipts'][-1]['stage'],'verify-results')
            self.assertIn('error',failed['receipts'][-1])


if __name__=='__main__': unittest.main()
