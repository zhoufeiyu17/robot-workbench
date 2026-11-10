"""Replay-only acceptance using existing-style RGB-D frames, never acquisition."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import threading
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from playback import sha256
from rgbd_segments import SegmentWriter, _png, verify_segment
import segmented_playback as playback

FFMPEG = os.environ.get('RGBD_TEST_FFMPEG', shutil.which('ffmpeg') or '')
FFPROBE = os.environ.get('RGBD_TEST_FFPROBE', shutil.which('ffprobe') or '')


class ExpressionTest(unittest.TestCase):
    def test_camera_exports_overlap_and_failed_worker_publishes_no_manifest(self):
        # Scheduling fixture only; actual codec/PTS/pixel tests remain below.
        for fail in (False, True):
            with self.subTest(fail=fail), TemporaryDirectory() as folder:
                root = Path(folder) / 'raw'; root.mkdir()
                output = Path(folder) / 'video'
                clock = {'environment': 'fake', 'clock_domain': 'simulation', 'source_clock_id': 'scheduling-test'}
                source = {'run_id': 'source-one', 'clock': clock}
                plan = {'run_id': 'plan-one', 'timeline_digest': 'timeline-one'}
                command = dict(clock, simulation_only=True, send_wall_ns=1_000_000_000, send_source_time_ns=0)
                for name, value in [('source.json', source), ('plan.json', plan), ('commands.jsonl', command)]:
                    (root / name).write_text(json.dumps(value) + '\n')
                raw = dict(schema_version='fake_device_raw_dataset.v1', evidence_source='fake_device_rgbd',
                           environment='fake', simulation_only=True, rgbd_storage='lossless_segments_v1',
                           source_run_id=source['run_id'], plan_run_id=plan['run_id'],
                           timeline_digest=plan['timeline_digest'], capture_status='incomplete',
                           controller_outcome='completed', files={
                               path.name: {'size_bytes': path.stat().st_size, 'sha256': sha256(path)}
                               for path in root.iterdir()})
                (root / 'manifest.json').write_text(json.dumps(raw))
                rows = [dict(clock, simulation_only=True, host_receive_ns=1_000_000_000 + index * 33_333_333,
                             source_time_ns=index * 33_333_333, physics_step=index, capture_fps=30)
                        for index in range(2)]
                barrier = threading.Barrier(3, timeout=5)
                lock = threading.Lock()
                active = maximum = 0
                checked = []
                def inputs(_root, _raw, camera):
                    nonlocal active, maximum
                    with lock:
                        active += 1
                        maximum = max(active, maximum)
                    barrier.wait()  # All three cameras must overlap.
                    return rows, [root / (camera + '.rgb.mp4')]
                def encode(args, **kwargs):
                    nonlocal active
                    self.assertFalse((output / 'manifest.json').exists())
                    if fail and args[-1].endswith('left_wrist_0.mp4'):
                        with lock:
                            active -= 1
                        raise ValueError('fixture encoder failed')
                    Path(args[-1]).write_bytes(b'derived fixture')
                def probe(path, *_args):
                    nonlocal active
                    checked.append(path.name)
                    with lock:
                        active -= 1
                    return {'nb_read_frames': '2'}
                with patch.object(playback, '_camera_inputs', side_effect=inputs), \
                        patch.object(playback, '_probe_video', side_effect=probe):
                    if fail:
                        with self.assertRaisesRegex(ValueError, 'fixture encoder failed'):
                            playback.export_segments(root, raw, source, plan, output, runner=encode)
                        self.assertFalse((output / 'manifest.json').exists())
                        self.assertEqual(len(checked), 2)  # Other owned workers were joined.
                    else:
                        result = playback.export_segments(root, raw, source, plan, output, runner=encode)
                        self.assertEqual(len(checked), 3)
                        self.assertEqual(result['capture_status'], 'incomplete')
                        self.assertEqual(result['task_outcome'], 'unverified')
                        self.assertEqual(list(result['source_timing']), list(playback.CAMERAS))
                self.assertEqual(maximum, 3)
                self.assertEqual(active, 0)

    def test_balanced_tree_exact_offsets_and_bounded_depth(self):
        offsets = [(index * 100_000_000 + 1500) // 3000 for index in range(1181)]
        expression = playback._setpts(offsets)
        depth = maximum = 0
        for char in expression:
            depth += (char == '(') - (char == ')')
            maximum = max(maximum, depth)
        self.assertLessEqual(maximum, 12)
        for index in (0, 1, 10, 119, 120, 999, 1180):
            actual = eval(expression.replace('if(', 'choose('), {'__builtins__': {}},
                          {'N': index, 'lt': lambda a, b: a < b, 'choose': lambda condition, a, b: a if condition else b})
            self.assertEqual(actual, offsets[index])
        for values in ([], [1], [0, 0], [0, -1]):
            with self.assertRaises(ValueError):
                playback._setpts(values)

    def test_same_size_input_change_and_symlink_rejected(self):
        with TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / 'pairs.jsonl'
            path.write_bytes(b'original')
            raw = {'files': {'pairs.jsonl': {'size_bytes': 8, 'sha256': sha256(path)}}}
            self.assertEqual(playback._pinned(root, raw, 'pairs.jsonl'), path)
            path.write_bytes(b'changed!')
            with self.assertRaisesRegex(ValueError, 'changed'):
                playback._pinned(root, raw, 'pairs.jsonl')
            target = root / 'target'
            path.rename(target)
            path.symlink_to(target)
            with self.assertRaisesRegex(ValueError, 'symlink'):
                playback._pinned(root, raw, 'pairs.jsonl')

    def test_probe_rejects_wrong_pts_with_correct_frame_count(self):
        from types import SimpleNamespace
        payload = {'streams': [{'codec_name': 'h264', 'pix_fmt': 'yuv420p', 'nb_read_frames': '2', 'duration': '.066667'}],
                   'frames': [{'best_effort_timestamp_time': '0.0'}, {'best_effort_timestamp_time': '.033400'}]}
        runner = lambda *args, **kwargs: SimpleNamespace(stdout=json.dumps(payload))
        with self.assertRaisesRegex(ValueError, 'PTS'):
            playback._probe_video(Path('unused'), runner, [0, 33333], 66667)


@unittest.skipUnless(FFMPEG and FFPROBE, 'real CPU ffmpeg/ffprobe binaries required')
class ExportTest(unittest.TestCase):
    def run_tool(self, args, **kwargs):
        return subprocess.run([{'ffmpeg': FFMPEG, 'ffprobe': FFPROBE}.get(args[0], args[0]), *args[1:]], **kwargs)

    def fixture(self, root):
        root.mkdir()
        clock = {'environment': 'fake', 'clock_domain': 'simulation', 'source_clock_id': 'existing-cpu-frames'}
        source = {'run_id': 'source-one', 'clock': clock}
        plan = {'run_id': 'plan-one', 'timeline_digest': 'timeline-one'}
        offsets = [0, 33_333_333, 66_666_666, 99_999_999, 140_123_456, 170_456_789, 203_790_123]
        wall, start = 1_000_000_000_000_001, 2_000_000_000
        for camera in playback.CAMERAS:
            writer = SegmentWriter(root / 'camera', camera, segment_frames=3, ffmpeg=FFMPEG, ffprobe=FFPROBE)
            rows = []
            for index, offset in enumerate(offsets):
                meta = dict(clock, simulation_only=True, host_receive_ns=wall + offset, source_time_ns=start + offset,
                            physics_step=index + 100, frame_number=index, capture_fps=30)
                color = _png(bytes((value + index * 11) % 256 for value in range(16 * 12 * 3)), 16, 12, False)
                depth = _png(bytes([index, 128]) * (16 * 12), 16, 12, True)
                ref = writer.add(meta, color, depth)
                rows.append(dict(meta, segment='camera/' + ref['segment'], segment_frame=ref['frame']))
            writer.finish()
            (root / (camera + '-pairs.jsonl')).write_text(''.join(json.dumps(row) + '\n' for row in rows))
        command = dict(clock, simulation_only=True, send_wall_ns=wall, send_source_time_ns=start + 10_000_000)
        (root / 'commands.jsonl').write_text(json.dumps(command) + '\n')
        for name, value in [('source.json', source), ('plan.json', plan)]:
            (root / name).write_text(json.dumps(value))
        raw = dict(schema_version='fake_device_raw_dataset.v1', evidence_source='fake_device_rgbd', environment='fake',
                   simulation_only=True, rgbd_storage='lossless_segments_v1', source_run_id=source['run_id'],
                   plan_run_id=plan['run_id'], timeline_digest=plan['timeline_digest'], capture_status='complete',
                   controller_outcome='completed', camera_frames=dict.fromkeys(playback.CAMERAS, len(offsets)),
                   files={str(path.relative_to(root)): {'size_bytes': path.stat().st_size, 'sha256': sha256(path)}
                          for path in root.rglob('*') if path.is_file()})
        (root / 'manifest.json').write_text(json.dumps(raw))
        return raw, source, plan, offsets

    def test_direct_segment_export_preserves_pts_and_creates_no_pngs(self):
        with TemporaryDirectory() as folder:
            root = Path(folder)
            raw, source, plan, offsets = self.fixture(root / 'raw')
            pins_before = {name: sha256(root / 'raw' / name) for name in raw['files']}
            with patch.object(playback, 'verify_segment', side_effect=lambda *args, **kwargs: verify_segment(*args, ffprobe=FFPROBE, **kwargs)):
                result = playback.export_segments(root / 'raw', raw, source, plan, root / 'playback', runner=self.run_tool)
            self.assertEqual(set(result['files']), {f'source-{camera}.mp4' for camera in playback.CAMERAS})
            for camera in playback.CAMERAS:
                timing = result['source_timing'][camera]
                self.assertEqual(timing['frame_count'], len(offsets))
                self.assertEqual(timing['frame_offsets_s'], [(offset - 10_000_000) / 1e9 for offset in offsets])
                self.assertEqual(int(result['files'][f'source-{camera}.mp4']['probe']['nb_read_frames']), len(offsets))
            self.assertFalse(list(root.rglob('*.png')))
            self.assertEqual(pins_before, {name: sha256(root / 'raw' / name) for name in raw['files']})
            # A reordered pair index with a refreshed file pin cannot relabel video frames.
            pairs = root / 'raw/base_0-pairs.jsonl'
            rows = pairs.read_text().splitlines()
            rows[0], rows[1] = rows[1], rows[0]
            pairs.write_text('\n'.join(rows) + '\n')
            raw['files']['base_0-pairs.jsonl'] = {'size_bytes': pairs.stat().st_size, 'sha256': sha256(pairs)}
            (root / 'raw/manifest.json').write_text(json.dumps(raw))
            with patch.object(playback, 'verify_segment', side_effect=lambda *args, **kwargs: verify_segment(*args, ffprobe=FFPROBE, **kwargs)):
                with self.assertRaisesRegex(ValueError, 'metadata or frame order'):
                    playback.export_segments(root / 'raw', raw, source, plan, root / 'reordered', runner=self.run_tool)

    def test_policy_source_only_uses_actual_command_identity_and_observed_window(self):
        from integration import _digest
        with TemporaryDirectory() as folder:
            root=Path(folder);raw,source,_plan,_offsets=self.fixture(root/'raw')
            source=dict(clock=source['clock'],schema_version='policy_execution_source.v1',
                        execution_instance_id='actual-policy-run',duration_s=.14)
            (root/'raw/source.json').write_text(json.dumps(source))
            (root/'raw/plan.json').unlink();raw['files'].pop('plan.json')
            raw['files']['source.json']={'size_bytes':(root/'raw/source.json').stat().st_size,'sha256':sha256(root/'raw/source.json')}
            raw.update(recording_kind='streaming_policy',source_run_id=source['execution_instance_id'],
                       source_digest=_digest(source),actual_commands_sha256=raw['files']['commands.jsonl']['sha256'],actual_duration_s=.16)
            raw.pop('plan_run_id');raw.pop('timeline_digest');(root/'raw/manifest.json').write_text(json.dumps(raw))
            with patch.object(playback,'verify_segment',side_effect=lambda *a,**kw:verify_segment(*a,ffprobe=FFPROBE,**kw)):
                result=playback.export_segments(root/'raw',raw,source,None,root/'video',runner=self.run_tool)
            self.assertNotIn('plan_run_id',result);self.assertNotIn('timeline_digest',result)
            self.assertEqual(result['actual_commands_sha256'],raw['actual_commands_sha256'])
            for timing in result['source_timing'].values():
                self.assertAlmostEqual(timing['encoded_duration_s'],.14,places=6)
                self.assertEqual(timing['frame_count'],5)


if __name__ == '__main__':
    unittest.main()
