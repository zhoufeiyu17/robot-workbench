"""Real sealed CPU recordings: gaps are evidence, not erased episodes."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'rule_based_workbench')]
from robot_workbench import recording_quality as quality
from real_recording import RealRecorder, CAMERAS, SIDES
from real_rgbd import pair_metadata
from test_real_recording import png
from test_recording_wall_clock import plan_fixture
from test_fake_device_pipeline import fake_metadata, FAKE_CLOCK_ID, FAKE_RUNTIME_NONCE


def hashes(root):
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in root.rglob('*') if p.is_file()}


class RecordingQualityTest(unittest.TestCase):
    def recording(self, root, environment):
        source, plan, binding, timeline = plan_fixture(4, environment)
        owner = dict(owner_pid=1234, command_epoch=3, runtime_nonce=FAKE_RUNTIME_NONCE)
        recorder = RealRecorder(root, source, plan, binding, environment=environment,
            execution_timeline=timeline, execution_owner=owner if environment=='fake' else None)
        self.addCleanup(recorder.abort)
        for i in range(5):
            stamp = 100_000_000_000 + i*1_000_000_000//30
            wall = stamp + 900_000_000_000
            target = recorder.frozen.ticks[min(i,3)]['target']
            clock = dict(clock_domain='simulation', source_clock_id=FAKE_CLOCK_ID,
                source_time_ns=stamp, physics_step=24_000+i*8, sample_wall_ns=wall, **owner)
            if i < 4:
                options = dict(clock_domain='simulation', source_clock_id=FAKE_CLOCK_ID,
                    send_source_time_ns=stamp, feedback_at_send=clock, **owner) if environment=='fake' else {}
                recorder.record_command(i, target, stamp/1e9, wall, **options)
            recorder.record_feedback(min(i,3), target, stamp/1e9, wall,
                raw=clock if environment=='fake' else None)
            for camera in CAMERAS:
                if camera=='left_wrist_0' and i==2:
                    continue
                side = SIDES[camera]
                meta = pair_metadata(serial=source['camera_serials'][side], camera_name=side,
                    color_number=i, depth_number=i+14, color_ms=stamp/1e6, depth_ms=stamp/1e6,
                    color_domain='simulation' if environment=='fake' else 'hardware_clock',
                    depth_domain='simulation' if environment=='fake' else 'hardware_clock',
                    host_receive_ns=wall, depth_scale_m=.001, intrinsics={'fx':600})
                if environment=='fake':
                    fake_metadata(meta, stamp, step=24_000+i*8)
                meta['capture_instance_id']='test-camera-instance'
                recorder.record_rgbd_frame(camera, meta, png(), png(True))
        manifest = recorder.seal('completed')
        self.assertEqual(manifest['capture_status'], 'incomplete')
        return manifest

    def test_gap_report_preserves_raw_and_v1_verdict_in_both_environments(self):
        for environment in ('fake', 'physical'):
            with self.subTest(environment=environment), tempfile.TemporaryDirectory() as folder:
                root=Path(folder); raw=root/'raw'; output=root/'playback'
                manifest=self.recording(raw, environment); before=hashes(raw)
                report=quality.generate(raw, output, environment=environment)
                self.assertEqual(hashes(raw), before)
                self.assertEqual(report['legacy']['controller_outcome'], 'completed')
                self.assertEqual(report['legacy']['capture_status'], 'incomplete')
                left=report['cameras']['left_wrist_0']
                self.assertEqual(left['accepted_pairs'], 4)
                self.assertEqual(left['missing_counter_values'], {'rgb':1, 'depth':1})
                self.assertEqual(len(left['discontinuities']), 1)
                self.assertEqual(len(left['consecutive_sequences']), 2)
                gap=left['discontinuities'][0]
                self.assertEqual(gap['before_counters'], [1,15])
                self.assertEqual(gap['after_counters'], [3,17])
                self.assertEqual(gap['attribution'], 'unknown')
                self.assertEqual(report['cameras']['base_0']['missing_counter_values'], {'rgb':0,'depth':0})
                self.assertEqual(quality.generate(raw,output,environment=environment), report)
                (output/'manifest.json').write_text(json.dumps({'raw_manifest_sha256':report['raw_manifest_sha256']}))
                cache={};self.assertEqual(quality.read_for_playback(output,cache=cache),report)
                view=quality.view_for_playback(output, video_ready=True, cache=cache)
                self.assertEqual([view['dimensions'][name]['status'] for name in ('motion','data','video','training')],
                                 ['completed','gaps','ready','unassessed'])
                self.assertEqual(json.loads((raw/'manifest.json').read_bytes()), manifest)

    def test_corrupt_sealed_index_cannot_be_certified(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); raw=root/'raw';self.recording(raw,'fake')
            index=raw/'base_0-pairs.jsonl';index.write_bytes(index.read_bytes()+b'{}\n')
            with self.assertRaisesRegex(ValueError, 'sealed raw file changed'):
                quality.generate(raw,root/'reports',environment='fake')
            self.assertFalse((root/'reports').exists())

    def test_missing_or_mismatched_quality_does_not_hide_verified_video(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder);digest='a'*64
            (output/'manifest.json').write_text(json.dumps({'raw_manifest_sha256':digest}))
            view=quality.view_for_playback(output,video_ready=True,controller_outcome='stopped')
            self.assertEqual(view['dimensions']['video']['status'],'ready')
            self.assertEqual(view['dimensions']['data']['status'],'unknown')
            path=output/'quality'/digest/quality.REPORT;path.parent.mkdir(parents=True)
            path.write_text(json.dumps({'schema':quality.SCHEMA,'raw_manifest_sha256':'b'*64}))
            view=quality.view_for_playback(output,video_ready=True)
            self.assertIn('different recording',view['quality_error'])
            self.assertEqual(view['dimensions']['video']['status'],'ready')
            path.write_text(json.dumps({'schema':quality.SCHEMA,'raw_manifest_sha256':digest,'legacy':{},'cameras':[]}))
            view=quality.view_for_playback(output,video_ready=True)
            self.assertIn('structure',view['quality_error'])
            self.assertEqual(view['dimensions']['video']['status'],'ready')

    def test_instance_reset_is_not_counted_as_a_huge_drop_or_cross_clock_window(self):
        row=dict(frame_number=99,depth_frame_number=113,host_receive_ns='100',capture_instance_id='a')
        rows=[row,dict(row,frame_number=0,depth_frame_number=14,host_receive_ns='200',capture_instance_id='b')]
        streams={camera:deepcopy(rows) for camera in CAMERAS}
        raw=dict(source_run_id='episode',camera_frames=dict.fromkeys(CAMERAS,2),capture_status='incomplete')
        report=quality.from_verified(raw,{'robot_id':'fixture'},streams,environment='physical',manifest_sha256='a'*64)
        gap=report['cameras']['base_0']['discontinuities'][0]
        self.assertIsNone(gap['missing_rgb'])
        self.assertEqual(gap['reason'],'instance_or_clock_changed')

    def test_journal_references_and_usable_sequences_do_not_grant_training_validity(self):
        row=dict(frame_number=1,depth_frame_number=10,host_receive_ns='100',capture_instance_id='a')
        journal=dict(schema='capture_ingress_journal.v1',capture_instance_id='a',path='/capture/events.jsonl',
            committed_bytes=30,committed_records=1,committed_prefix_sha256='b'*64,queue_dropped=3,full=True)
        rows=[row,dict(row,frame_number=2,depth_frame_number=11,host_receive_ns='200',
            capture_diagnostics=dict(schema='capture_ingress_diagnostics.v1',journal=journal)),
            dict(row,frame_number=4,depth_frame_number=13,host_receive_ns='300')]
        streams={camera:deepcopy(rows) for camera in CAMERAS}
        raw=dict(source_run_id='episode',camera_frames=dict.fromkeys(CAMERAS,3),capture_status='incomplete')
        report=quality.from_verified(raw,{'robot_id':'fixture'},streams,environment='physical',manifest_sha256='a'*64)
        view=quality.presentation(report,video_ready=True)
        evidence=view['cameras']['base_0']
        self.assertEqual([(s['first_index'],s['last_index']) for s in evidence['consecutive_sequences']],[(0,1),(2,2)])
        self.assertEqual(evidence['ingress_diagnostics']['references'][0]['queue_dropped'],3)
        self.assertEqual(evidence['ingress_diagnostics']['status'],'snapshot_references')
        self.assertEqual(view['dimensions']['training']['status'],'unassessed')
        self.assertEqual(view['dimensions']['data']['status'],'gaps')
        self.assertFalse(report['raw_modified'])
        self.assertEqual(quality._ingress(rows,'fake')['status'],'unsupported')


if __name__=='__main__':
    unittest.main()
