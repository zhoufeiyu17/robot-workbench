"""A failed/incomplete recording stays readable without passing qualified-fake gates."""
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

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
