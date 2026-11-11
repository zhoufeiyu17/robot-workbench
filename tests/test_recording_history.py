"""A failed/incomplete recording stays readable without passing qualified-fake gates."""
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest


class CommonCaptureIndexTest(unittest.TestCase):
    def test_http_results_and_json_download_work_without_device_or_model(self):
        import threading
        from urllib.request import build_opener,ProxyHandler
        from robot_workbench.service import Device,Application,serve
        device=Device('cpu','fake',None);device.capture_results=self.index
        self.index.register(self.sealed(1,'stopped','incomplete'))
        app=Application({'fake':device});server=serve(app,host='127.0.0.1',port=0)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        opener=build_opener(ProxyHandler({}));base='http://127.0.0.1:'+str(server.server_port)
        try:
            state=json.load(opener.open(base+'/api/state?environment=fake&business=calibration',timeout=3))
            self.assertFalse(state['runtime']['connected']);self.assertEqual(state['results_page']['total_runs'],1)
            row=next(row for row in state['results']if row['kind']=='control_recording')
            receipt=json.load(opener.open(base+row['download_url'],timeout=3))
            self.assertEqual(receipt['controller']['status'],'stopped')
            page=json.load(opener.open(base+'/api/state?environment=fake&business=calibration&results_before=1',timeout=3))
            self.assertEqual(page['results'],[])
        finally:server.shutdown();server.server_close();thread.join(2)

    def setUp(self):
        from robot_workbench.collection_history import CaptureResults
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name).resolve()
        self.index=CaptureResults(self.root,environment='fake',device_id='cpu');self.addCleanup(self.index.close)

    def sealed(self,revision,status='completed',outcome='complete',business='calibration'):
        from robot_workbench.collection_history import _pin
        run='cpu-run';folder=self.root/(('inference-'if business=='inference'else'')+run);data=folder/'capture'/'data';data.mkdir(parents=True,exist_ok=True)
        manifest=dict(schema='workbench.capture-manifest.v1',capture_session_id=run,seal_revision=revision,
            binding=dict(run_id=run,business_kind=business,policy_hash='a'*64),files={},playback={},recording_outcome=outcome)
        mp=data/f'manifest-r{revision:06d}.json';mp.write_text(json.dumps(manifest))
        receipt=dict(schema='workbench.capture-run.v1',execution_instance_id=run,business_kind=business,seal_revision=revision,
            execution_context=dict(device_id='cpu',environment='fake'),control_policy={'policy_hash':'a'*64},
            recording=dict(directory=str(data),recording_outcome=outcome),controller={'status':status})
        rp=folder/f'receipt-r{revision:06d}.json';rp.write_text(json.dumps(receipt))
        return dict(execution_instance_id=run,business_kind=business,seal_revision=revision,
            receipt_ref=dict(path=str(rp),**_pin(rp)),manifest_ref=dict(path=str(mp),**_pin(mp)))

    def test_inference_replay_metadata_http_reads_only_pinned_recorded_configuration(self):
        import threading
        from types import SimpleNamespace
        from urllib.request import build_opener,ProxyHandler
        from urllib.error import HTTPError
        from robot_workbench.service import Device,Application,serve
        from robot_workbench.collection_history import CaptureResults,_pin
        from robot_workbench.inference_history import InferenceHistory
        from robot_workbench.recording_activity import ActivityTimeline
        self.index.close()
        self.index=CaptureResults(self.root,environment='fake',device_id='cpu',inference_history=InferenceHistory(self.root))
        self.addCleanup(self.index.close)
        event=self.sealed(1,business='inference');mp=Path(event['manifest_ref']['path'])
        manifest=json.loads(mp.read_text())
        config=dict(model_id='wam-fixture-step7',model_label='Recorded fixture',backend='torch',denoise_steps=6,execute_steps=20)
        manifest['binding']['execution_context']=dict(inference_configuration=config)
        timeline=ActivityTimeline('cpu-run','clock')
        timeline.observe('inference_mode_transition',dict(op='normal_mode',requested_mode='MODEL',request_id='start',
            accepted=True,status='accepted',capture_wall_ns=100_000_000_000),
            SimpleNamespace(source_clock_id='clock',source_time_ns=10_000_000_000),'cpu-run')
        timeline.finish(12_000_000_000);manifest['recording_activity']=timeline.manifest()
        mp.write_text(json.dumps(manifest));event['manifest_ref'].update(_pin(mp));self.index.register(event)
        device=Device('cpu','fake',None)
        device.workflows['inference']=SimpleNamespace(replay_metadata=lambda run:self.index.replay_metadata('inference',run))
        server=serve(Application({'fake':device}),host='127.0.0.1',port=0)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        opener=build_opener(ProxyHandler({}));url='http://127.0.0.1:'+str(server.server_port)+'/api/inference-replay-metadata?environment=fake&run_id=cpu-run'
        try:
            result=json.load(opener.open(url,timeout=3));self.assertEqual(result['run_id'],'cpu-run')
            row=result['segments'][0]
            self.assertEqual((row['model_id'],row['denoise_steps'],row['execute_steps']),('wam-fixture-step7',6,20))
            self.assertEqual((row['start_s'],row['end_s']),(0.,2.))
            mp.write_text('{}')
            with self.assertRaises(HTTPError):opener.open(url,timeout=3)
            with self.assertRaises(ValueError):self.index.replay_metadata('inference','../outside')
        finally:server.shutdown();server.server_close();thread.join(2)

    def test_recording_date_uses_original_activity_or_existing_frame_not_file_mtime(self):
        from robot_workbench.collection_history import _capture_started_wall_ns,_pin
        root=self.root/'date';root.mkdir();manifest=root/'manifest.json'
        def read(value):
            manifest.write_text(json.dumps(value))
            return _capture_started_wall_ns(str(manifest),_pin(manifest)['sha256'])
        stamp=1_794_201_234_000_000_000
        self.assertEqual(read(dict(recording_activity={'intervals':[{'start_wall_ns':stamp}]})),stamp)
        (root/'camera.jsonl').write_text(json.dumps(dict(recorded_wall_ns=stamp+100,
            payload={'metadata':{'ros_reference':{'source_clock_id':'host_wall','source_time_ns':stamp}}}))+'\n')
        self.assertEqual(read(dict(streams={'missing':{},'camera':{}})),stamp)
        self.assertIsNone(read(dict(streams={'absent':{}})))

    def test_revision_idempotency_late_notice_restart_and_tampered_media(self):
        from robot_workbench.collection_history import CaptureResults
        first=self.sealed(1);self.index.register(first);before=Path(first['receipt_ref']['path']).read_bytes()
        self.assertFalse(self.index.register(first));second=self.sealed(2,'failed','incomplete');self.index.register(second)
        self.assertFalse(self.index.register(first));rows=self.index.page('calibration')
        self.assertEqual((rows['total_runs'],rows['total_results']),(1,1));row=rows['results'][0]
        self.assertEqual((row['status'],row['details']['seal_revision']),('failed',2))
        self.assertEqual(Path(first['receipt_ref']['path']).read_bytes(),before)
        self.index.close();reopened=CaptureResults(self.root,environment='fake',device_id='cpu');self.addCleanup(reopened.close)
        self.assertEqual(reopened.page('calibration')['results'][0]['details']['seal_revision'],2)
        media=reopened.media('calibration',row['id']);self.assertEqual(media,Path(second['receipt_ref']['path']))
        media.write_text('{}')
        with self.assertRaisesRegex(ValueError,'changed'):reopened.media('calibration',row['id'])

    def test_foreign_root_device_business_and_changed_manifest_are_refused(self):
        from copy import deepcopy
        from robot_workbench.collection_history import CaptureResults
        event=self.sealed(1)
        for key,value in [('execution_instance_id','other-run'),('business_kind','research')]:
            bad=deepcopy(event);bad[key]=value
            with self.assertRaises(ValueError):self.index.register(bad)
        with self.assertRaisesRegex(ValueError,'another device'):CaptureResults(self.root,environment='fake',device_id='foreign')
        self.index.register(event);Path(event['manifest_ref']['path']).write_text('{}')
        with self.assertRaisesRegex(ValueError,'changed'):self.index.register(event)
        bad=deepcopy(event);bad['receipt_ref']['path']='/tmp/foreign-receipt.json'
        with self.assertRaises(ValueError):self.index.register(bad)

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'rule_based_workbench'),str(ROOT/'tests')]
import test_recording_quality as quality_fixture
from test_recording_quality import hashes
from robot_workbench.collection_history import register_recording,inspect_recording,load_history,_pin,CollectionHistory
from real_contract import namespace


class RecordingHistoryTest(unittest.TestCase):
    def source(self,root,environment):
        origin=root/'origin';origin.mkdir();raw=origin/'raw'/'episode'
        manifest=quality_fixture.RecordingQualityTest.recording(self,raw,environment)
        playback=origin/'playback';playback.mkdir()
        spec=namespace(environment);files={}
        for camera in ('base_0','left_wrist_0','right_wrist_0'):
            p=playback/('source-'+camera+'.mp4')
            p.write_bytes(b'synthetic video bytes: pin/path tests, not codec qualification '+camera.encode())
            files[p.name]=_pin(p)
        exported=dict(schema_version=spec['playback_schema'],mode=spec['mode'],provenance={'source':spec['evidence']},
            raw_manifest_sha256=_pin(raw/'manifest.json')['sha256'],source_run_id=manifest['source_run_id'],
            controller_outcome=manifest['controller_outcome'],capture_status=manifest['capture_status'],files=files,
            **{k:manifest[k] for k in ('plan_run_id','timeline_digest') if k in manifest})
        if environment=='fake':exported.update(environment='fake',simulation_only=True)
        (playback/'manifest.json').write_text(json.dumps(exported))
        return origin,manifest

    def test_incomplete_recording_is_readonly_discoverable_and_environment_bound(self):
        for environment in ('physical','fake'):
            with self.subTest(environment=environment),tempfile.TemporaryDirectory() as folder:
                root=Path(folder).resolve();origin,manifest=self.source(root,environment);before=hashes(origin)
                checked=inspect_recording(origin,'raw/episode','playback',environment=environment,robot_id=manifest['robot_id'])
                self.assertEqual(checked['capture_status'],'incomplete')
                self.assertEqual(hashes(origin),before)
                output=root/'catalog'
                history=register_recording(origin,'raw/episode','playback',output,environment=environment,robot_id=manifest['robot_id'])
                self.assertEqual(len(history.results()),3)
                first=history.results()[0]
                self.assertEqual(history.quality_report(first['id'])['report']['raw_manifest_sha256'],_pin(origin/'raw'/'episode'/'manifest.json')['sha256'])
                self.assertIsNone(history.quality_report('unknown-recording'))
                self.assertEqual(first['status'],'failed')
                self.assertTrue(first['readonly'])
                self.assertEqual(first['recording_quality']['dimensions']['video']['status'],'ready')
                self.assertEqual(first['recording_quality']['dimensions']['data']['status'],'gaps')
                self.assertEqual(history.media(first['id']),origin/'playback'/'source-base_0.mp4')
                self.assertEqual(load_history(output,environment=environment,robot_id=manifest['robot_id']).results(),history.results())
                self.assertEqual(hashes(origin),before)
                with self.assertRaisesRegex(ValueError,'another device'):
                    load_history(output,environment=environment,robot_id='different-robot')
                with self.assertRaisesRegex(ValueError,'another device'):
                    load_history(output,environment='fake' if environment=='physical' else 'physical',robot_id=manifest['robot_id'])
                with self.assertRaises((ValueError,KeyError)):
                    CollectionHistory(output)  # Never silently weaken the qualified-fake importer.

    def test_changed_video_does_not_hide_other_verified_videos(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder).resolve();origin,manifest=self.source(root,'physical');output=root/'catalog'
            register_recording(origin,'raw/episode','playback',output,environment='physical',robot_id=manifest['robot_id'])
            broken=origin/'playback'/'source-left_wrist_0.mp4';broken.write_bytes(b'changed')
            history=load_history(output,environment='physical',robot_id=manifest['robot_id'])
            rows=history.results();self.assertEqual(sum(row['media']['ready'] for row in rows),2)
            missing=next(row for row in rows if not row['media']['ready'])
            self.assertIsNone(history.media(missing['id']))
            self.assertEqual(missing['recording_quality']['dimensions']['training']['status'],'unassessed')

    def test_wrong_pins_and_escaping_paths_do_not_create_a_catalog(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder).resolve();origin,manifest=self.source(root,'physical')
            with self.assertRaises(ValueError):
                register_recording(origin,'../../outside','playback',root/'catalog',environment='physical',robot_id=manifest['robot_id'])
            p=origin/'playback'/'manifest.json';value=json.loads(p.read_text());value['raw_manifest_sha256']='f'*64;p.write_text(json.dumps(value))
            with self.assertRaisesRegex(ValueError,'identity differs'):
                register_recording(origin,'raw/episode','playback',root/'catalog',environment='physical',robot_id=manifest['robot_id'])
            self.assertFalse((root/'catalog').exists())

    def test_bad_quality_sidecar_does_not_remove_independently_pinned_videos(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder).resolve();origin,manifest=self.source(root,'physical');output=root/'catalog'
            register_recording(origin,'raw/episode','playback',output,environment='physical',robot_id=manifest['robot_id'])
            saved=json.loads((output/'history.json').read_text());(output/saved['quality_file']).write_text('{}')
            history=load_history(output,environment='physical',robot_id=manifest['robot_id'])
            self.assertEqual(len(history.results()),3)
            for row in history.results():
                self.assertTrue(row['media']['ready'])
                self.assertEqual(row['recording_quality']['dimensions']['data']['status'],'unknown')
                self.assertIn('quality_error',row['recording_quality'])
                self.assertIsNotNone(history.media(row['id']))

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'),'actual CPU video tools required')
    def test_actual_export_creates_quality_and_is_accepted_by_the_history_reader(self):
        from real_recording import export_playback
        from robot_workbench.recording_quality import read_for_playback
        for environment in ('fake','physical'):
            with self.subTest(environment=environment),tempfile.TemporaryDirectory() as folder:
                root=Path(folder).resolve();origin=root/'origin';origin.mkdir();raw=origin/'raw'
                manifest=quality_fixture.RecordingQualityTest.recording(self,raw,environment)
                playback=origin/'playback';before=hashes(raw)
                export_playback(raw,None,playback,environment=environment)
                self.assertIsNotNone(read_for_playback(playback))
                history=register_recording(origin,'raw','playback',root/'catalog',environment=environment,robot_id=manifest['robot_id'])
                self.assertEqual(len(history.results()),3)
                self.assertTrue(all(row['media']['ready'] for row in history.results()))
                self.assertEqual(hashes(raw),before)


if __name__=='__main__':unittest.main()
