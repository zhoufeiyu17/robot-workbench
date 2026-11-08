"""CPU-only RGB-D codec/recovery acceptance; no acquisition or robot calls."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import select
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import rgbd_segments as segments

FFMPEG = os.environ.get('RGBD_TEST_FFMPEG', shutil.which('ffmpeg') or '')
FFPROBE = os.environ.get('RGBD_TEST_FFPROBE', shutil.which('ffprobe') or '')


def frame(index, width=17, height=12):
    # Includes 16-bit extremes and odd RGB widths; no RGB/depth quantization.
    color = bytes((value * 37 + index * 19) % 256 for value in range(width * height * 3))
    values = [0, 1, 255, 256, 32767, 32768, 65535]
    depth = b''.join(values[(value + index) % len(values)].to_bytes(2, 'little') for value in range(width * height))
    meta = {'frame_number': index, 'source_time_ns': 1_000_000_000_000_000_001 + index * 33333333,
            'serial': 'exact-test-source', 'device_timestamp_ms': 17.123456789 + index, 'intrinsics': {'fx': 610.123}}
    return meta, color, depth, segments._png(color, width, height, False), segments._png(depth, width, height, True)


def program(root, name, body):
    path = root / name
    path.write_text(f'#!{sys.executable}\n' + body)
    path.chmod(0o700)
    return str(path)


class BoundsTest(unittest.TestCase):
    def test_pixels_reject_malformed_payloads_before_starting_codec(self):
        meta, color, depth, _, _ = frame(0)
        with TemporaryDirectory() as directory:
            writer = segments.SegmentWriter(directory, 'base_0')
            try:
                invalid = [
                    (color[:-1], depth, 17, 12), (color, depth + b'\0', 17, 12),
                    (bytearray(color), depth, 17, 12), (color, memoryview(depth), 17, 12),
                    (color, depth, True, 12), (color, depth, 17, 12.),
                    (color, depth, 0, 12), (color, depth, 17, -1),
                    (color, depth, segments._LIMIT // 3 + 1, 1),
                ]
                with patch.object(writer, '_start') as start:
                    for args in invalid:
                        with self.subTest(args=args[2:]), self.assertRaises(ValueError):
                            writer.add_pixels(meta, *args)
                    writer.shape = (12, 17)
                    with self.assertRaisesRegex(ValueError, 'dimensions changed'):
                        writer.add_pixels(meta, color, depth, 17, 12)
                    start.assert_not_called()
                self.assertFalse(writer.aborted)
                self.assertEqual(list(writer.directory.iterdir()), [])
            finally:
                writer.abort()
            with self.assertRaisesRegex(ValueError, 'closed'):
                writer.add_pixels(meta, color, depth, 17, 12)

    def test_cpu_codec_does_not_inherit_isaac_runtime_environment(self):
        with TemporaryDirectory() as directory:
            executable=program(Path(directory),'show-env','import json,os\nprint(json.dumps({k:os.environ.get(k) for k in ("LD_PRELOAD","LD_LIBRARY_PATH","PYTHONHOME","PATH")}))\n')
            with patch.dict(os.environ,{'LD_PRELOAD':'/isaac-sim/kit/libcarb.so',
                                        'LD_LIBRARY_PATH':'/isaac-sim/kit','PYTHONHOME':'/isaac-sim/kit/python',
                                        'PATH':directory+os.pathsep+os.environ['PATH']}):
                child=segments._Child([Path(executable).name],2)
                try:
                    child.process.stdin.close();output=bytearray()
                    while block:=child.read(1,eof=True):output.extend(block)
                    child.finish()
                finally:child.abort()
            value=json.loads(output)
            self.assertIsNone(value['LD_PRELOAD']);self.assertIsNone(value['LD_LIBRARY_PATH'])
            self.assertIsNone(value['PYTHONHOME'])
            self.assertNotIn('isaac',value['PATH'])
            self.assertNotIn(directory,value['PATH'])

    def test_invalid_paths_and_headers(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            for camera in ('../escape', '/absolute', '.', 'a/b'):
                with self.assertRaises(ValueError):
                    segments.SegmentWriter(root, camera)
            writer = segments.SegmentWriter(root, 'base_0')
            meta, _, _, color, depth = frame(0)
            rgba = bytearray(color)
            rgba[25] = 6
            with self.assertRaises(ValueError):
                writer.add(meta, bytes(rgba), depth)
            with self.assertRaises(ValueError):
                segments.verify_segment(root, '../escape')
            writer.abort()

    def test_encoder_error_retains_partial_and_reaps(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            encoder = program(root, 'fail-codec', 'import sys\nsys.exit(9)\n')
            writer = segments.SegmentWriter(root, 'base_0', ffmpeg=encoder, timeout_s=1)
            meta, _, _, color, depth = frame(0)
            with self.assertRaises((RuntimeError, BrokenPipeError)):
                writer.add(meta, color, depth)
                writer.finish()
            self.assertTrue(all(child.process.poll() is not None for child in writer.children.values()))
            self.assertFalse(list((root / 'base_0').glob('*.commit.json')))
            self.assertTrue(list((root / 'base_0').glob('*.partial.*')))

    def test_encoder_stall_times_out_and_reaps(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            encoder = program(root, 'stall-codec', 'import time\ntime.sleep(60)\n')
            writer = segments.SegmentWriter(root, 'base_0', ffmpeg=encoder, timeout_s=.1)
            # Larger than a pipe's capacity; no reader means a bounded failure.
            color = segments._png(bytes(1024 * 1024 * 3), 1024, 1024, False)
            depth = segments._png(bytes(1024 * 1024 * 2), 1024, 1024, True)
            with self.assertRaises(TimeoutError):
                writer.add({'frame_number': 0}, color, depth)
            self.assertEqual(writer.timings['first_frame']['status'],'failed')
            self.assertEqual(writer.timings['first_frame']['stage'],'rgb_pipe_write')
            self.assertIn('segment encoder write timeout',writer.timings['first_frame']['error'])
            self.assertGreater(writer.timings['first_frame']['total_wall_s'],.1)
            self.assertTrue(all(child.process.poll() is not None for child in writer.children.values()))
            self.assertFalse(list((root / 'base_0').glob('*.commit.json')))


class AsyncCommitTest(unittest.TestCase):
    def fake_codec(self, root):
        # A lifecycle fixture, deliberately not evidence of codec correctness.
        return program(root, 'pipe-codec', 'import sys\nfrom pathlib import Path\n'
                       'with Path(sys.argv[-1]).open("wb") as stream:\n'
                       '    while chunk := sys.stdin.buffer.read(4096): stream.write(chunk)\n')

    def add(self, writer, index):
        meta, color, depth, color_png, depth_png = frame(index)
        # Exercise both front doors through the same async commit lifecycle.
        return writer.add_pixels(meta, color, depth, 17, 12) if index % 2 == 0 else writer.add(meta, color_png, depth_png)

    def test_pixels_use_no_png_decode_and_preserve_input_and_metadata(self):
        from PIL import Image
        with TemporaryDirectory() as directory:
            root = Path(directory)
            writer = segments.SegmentWriter(root, 'base_0', ffmpeg=self.fake_codec(root))
            meta, color, depth, _, _ = frame(0)
            expected_meta = json.loads(json.dumps(meta))
            try:
                with patch.object(Image, 'open', side_effect=AssertionError('raw pixels must not decode PNG')), \
                        patch.object(segments, '_check_probe'):
                    writer.add_pixels(meta, color, depth, 17, 12)
                    meta['intrinsics']['fx'] = -1
                    writer.finish()
                # This fixture copies raw stdin; it proves routing, not codec correctness.
                self.assertEqual((writer.directory / '000000.rgb.mp4').read_bytes(), color)
                self.assertEqual((writer.directory / '000000.depth.mkv').read_bytes(), depth)
                self.assertEqual(json.loads((writer.directory / '000000.frames.jsonl').read_bytes()), expected_meta)
                self.assertEqual(writer.timings['first_frame']['input_format'], 'rgb8_depth16le')
                self.assertEqual(writer.timings['first_frame']['status'], 'written')
                self.assertNotIn('png_decode_wall_s', writer.timings['first_frame'])
            finally:
                writer.abort()

    def test_slow_commit_allows_next_segment_and_other_camera(self):
        with TemporaryDirectory() as directory, ThreadPoolExecutor(max_workers=1) as executor:
            root = Path(directory)
            codec = self.fake_codec(root)
            entered, release = threading.Event(), threading.Event()
            def slow_probe(*args, **kwargs):
                if 'base_0' in str(args[0]) and '000000.' in str(args[0]):
                    entered.set()
                    if not release.wait(3):
                        raise TimeoutError('test did not release probe')
            writers = [segments.SegmentWriter(root, camera, segment_frames=3, ffmpeg=codec,
                                              commit_executor=executor, timeout_s=3)
                       for camera in ('base_0', 'left_wrist_0')]
            try:
                with patch.object(segments, '_check_probe', side_effect=slow_probe):
                    for index in range(3):
                        self.add(writers[0], index)
                    self.assertTrue(entered.wait(2))
                    # The blocked commit stays pending, but add() need not wait.
                    ref = self.add(writers[0], 3)
                    self.assertEqual(ref, {'segment': 'base_0/000001.commit.json', 'frame': 0})
                    for index in range(3):
                        self.add(writers[1], index)
                    self.assertFalse(writers[0].pending.done())
                    self.assertFalse(writers[1].pending.done())
                    release.set()
                    self.assertEqual([x['frame_count'] for x in writers[0].wait_committed()], [3])
                    self.assertEqual(writers[0].count, 1)  # Public wait did not close the active segment.
                    self.assertEqual([x['frame_count'] for x in writers[0].finish()], [3, 1])
                    self.assertEqual([x['frame_count'] for x in writers[1].finish()], [3])
                    self.assertEqual(writers[0].timings['commit_segments'], 2)
                    self.assertGreater(writers[0].timings['probe_wall_s'], 0)
            finally:
                release.set()
                for writer in writers:
                    writer.abort()

    def test_abort_cancels_only_its_queued_commit(self):
        with TemporaryDirectory() as directory, ThreadPoolExecutor(max_workers=1) as executor:
            root = Path(directory)
            codec = self.fake_codec(root)
            entered, release = threading.Event(), threading.Event()
            def slow_probe(*args, **kwargs):
                if 'base_0' in str(args[0]):
                    entered.set()
                    if not release.wait(3):
                        raise TimeoutError('test did not release probe')
            first = segments.SegmentWriter(root, 'base_0', segment_frames=3, ffmpeg=codec, commit_executor=executor)
            second = segments.SegmentWriter(root, 'left_wrist_0', segment_frames=3, ffmpeg=codec, commit_executor=executor)
            try:
                with patch.object(segments, '_check_probe', side_effect=slow_probe):
                    for index in range(3):
                        self.add(first, index)
                    self.assertTrue(entered.wait(2))
                    for index in range(3):
                        self.add(second, index)
                    second.abort()
                    self.assertTrue(second.pending.cancelled())
                    self.assertFalse(list((root / 'left_wrist_0').glob('*.commit.json')))
                    release.set()
                    self.assertEqual(len(first.finish()), 1)
                    self.assertEqual(executor.submit(lambda: 42).result(), 42)
            finally:
                release.set()
                first.abort()
                second.abort()

    def test_running_probe_is_owned_and_aborted_without_publishing(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            codec = self.fake_codec(root)
            probe = program(root, 'stall-probe', 'import time\ntime.sleep(60)\n')
            writer = segments.SegmentWriter(root, 'base_0', segment_frames=3, ffmpeg=codec,
                                             ffprobe=probe, timeout_s=2)
            for index in range(3):
                self.add(writer, index)
            deadline = time.monotonic() + 2
            while not any(child.process.args[0] == probe for child in tuple(writer._owned_children)):
                if time.monotonic() > deadline:
                    self.fail('background probe did not start')
                time.sleep(.01)
            children = tuple(writer._owned_children)
            writer.abort()
            self.assertTrue(all(child.process.poll() is not None for child in children))
            self.assertFalse(list((root / 'base_0').glob('*.commit.json')))
            self.assertTrue(writer.pending.done())

    def test_background_failure_surfaces_before_accepting_next_frame(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            writer = segments.SegmentWriter(root, 'base_0', segment_frames=3, ffmpeg=self.fake_codec(root))
            with patch.object(segments, '_check_probe', side_effect=ValueError('bad counted frames')):
                for index in range(3):
                    self.add(writer, index)
                while not writer.pending.done():
                    time.sleep(.01)
                with self.assertRaisesRegex(ValueError, 'bad counted frames'):
                    self.add(writer, 3)
            self.assertTrue(writer.aborted)
            self.assertFalse(list((root / 'base_0').glob('*.commit.json')))

    def test_third_full_segment_has_bounded_backpressure(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            probe = program(root, 'stall-probe', 'import time\ntime.sleep(60)\n')
            writer = segments.SegmentWriter(root, 'base_0', segment_frames=3, ffmpeg=self.fake_codec(root),
                                             ffprobe=probe, timeout_s=.3)
            with self.assertRaises(TimeoutError):
                for index in range(9):
                    self.add(writer, index)
            self.assertEqual(writer.issued_index, 3)
            self.assertTrue(writer.pending.done())
            self.assertFalse(writer._owned_children)
            self.assertFalse(list((root / 'base_0').glob('*.commit.json')))


@unittest.skipUnless(FFMPEG and FFPROBE, 'real CPU ffmpeg/ffprobe binaries required')
class CodecTest(unittest.TestCase):
    def writer(self, root, **kwargs):
        return segments.SegmentWriter(root, 'base_0', segment_frames=3, ffmpeg=FFMPEG, ffprobe=FFPROBE, **kwargs)

    def test_repeated_probe_reuses_content_proof_but_detects_changed_bytes(self):
        with segments._PROBE_LOCK:segments._PROBE_CACHE.clear()
        with TemporaryDirectory()as directory:
            root=Path(directory);writer=self.writer(root)
            original=segments.subprocess.Popen;probes=[]
            def launch(argv,*args,**kwargs):
                if argv[0]==FFPROBE:probes.append(tuple(argv))
                return original(argv,*args,**kwargs)
            with patch.object(segments.subprocess,'Popen',side_effect=launch):
                for i in range(3):
                    meta,_,_,color,depth=frame(i);writer.add(meta,color,depth)
                manifest=writer.finish()[0];count=len(probes)
                self.assertGreater(count,0)
                for _ in range(2):segments.verify_segment(root,'base_0/000000.commit.json',ffprobe=FFPROBE)
                self.assertEqual(len(probes),count,'identical bytes must not be decoded again just to count frames')
                path=root/manifest['files']['rgb']['path'];before=path.stat();data=bytearray(path.read_bytes())
                data[-1]^=1;path.write_bytes(data);os.utime(path,ns=(before.st_atime_ns,before.st_mtime_ns))
                with self.assertRaisesRegex(ValueError,'hash/size'):
                    segments.verify_segment(root,'base_0/000000.commit.json',ffprobe=FFPROBE)
                self.assertEqual(len(probes),count)

    def test_exact_pixels_at_tiny_odd_and_camera_dimensions(self):
        # Small images exposed FFV1's automatic slice-grid row loss. Exercise
        # both ingress formats and a segment boundary at every supported size.
        for width, height in ((2, 2), (4, 3), (17, 12), (352, 288),
                              (353, 288), (352, 289), (640, 480)):
            with self.subTest(width=width, height=height), TemporaryDirectory() as directory:
                root = Path(directory)
                writer = self.writer(root)
                expected = [frame(index, width, height) for index in range(4)]
                refs = []
                try:
                    for index, (meta, rgb, depth, rgb_png, depth_png) in enumerate(expected):
                        refs.append(writer.add(meta, rgb_png, depth_png) if index % 2 else
                                    writer.add_pixels(meta, rgb, depth, width, height))
                    manifests = writer.finish()
                    self.assertEqual([row['frame_count'] for row in manifests], [3, 1])
                    for manifest, frames in zip(manifests, (expected[:3], expected[3:])):
                        self.assertEqual(manifest['decoded_pixel_sha256'], {
                            kind: hashlib.sha256(b''.join(row[offset] for row in frames)).hexdigest()
                            for kind, offset in (('rgb', 1), ('depth', 2))})
                    actual = []
                    for name in dict.fromkeys(ref['segment'] for ref in refs):
                        actual.extend(segments.iter_segment(root, name, ffmpeg=FFMPEG,
                                                            ffprobe=FFPROBE, png=False))
                    self.assertEqual(actual, [row[:3] for row in expected])
                finally:
                    writer.abort()

    @unittest.skipUnless(os.name == 'posix', 'owned process-group SIGKILL requires POSIX')
    def test_sigkill_recovers_only_committed_prefix_and_reaps_owned_group(self):
        # Linux containers may have a PID 1 which never reaps orphans. Temporarily
        # adopt this test's grandchildren, and restore the prior setting below.
        libc = previous_subreaper = None
        if sys.platform.startswith('linux'):
            import ctypes
            libc = ctypes.CDLL(None, use_errno=True)
            previous_subreaper = ctypes.c_int()
            if libc.prctl(37, ctypes.byref(previous_subreaper), 0, 0, 0) != 0 or libc.prctl(36, 1, 0, 0, 0) != 0:
                self.skipTest('Linux child-subreaper unavailable for exact orphan cleanup')
        child = None
        owned = []
        try:
            with TemporaryDirectory() as directory:
                root = Path(directory)
                script = '''import json, os, sys, time
from rgbd_segments import SegmentWriter
from test_rgbd_segments import frame
writer = SegmentWriter(sys.argv[1], 'base_0', segment_frames=3, ffmpeg=sys.argv[2], ffprobe=sys.argv[3])
for index in range(3):
    meta, color, depth, _, _ = frame(index)
    writer.add_pixels(meta, color, depth, 17, 12)
writer.wait_committed()
meta, color, depth, _, _ = frame(3)
writer.add_pixels(meta, color, depth, 17, 12)
print(json.dumps({'ready': True, 'pid': os.getpid(), 'pgid': os.getpgrp(),
                  'children': [item.process.pid for item in writer._owned_children]}), flush=True)
while True: time.sleep(60)
'''
                env = dict(os.environ)
                env['PYTHONPATH'] = str(Path(__file__).resolve().parent) + os.pathsep + env.get('PYTHONPATH', '')
                child = subprocess.Popen([sys.executable, '-c', script, directory, FFMPEG, FFPROBE],
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                         start_new_session=True, env=env)
                if not select.select([child.stdout], [], [], 30)[0]:
                    self.fail('owned writer did not publish its committed checkpoint within 30 seconds')
                receipt = json.loads(child.stdout.readline())
                self.assertIsNone(child.poll())
                self.assertTrue(receipt['ready'])
                self.assertEqual((receipt['pid'], receipt['pgid']), (child.pid, child.pid))
                self.assertEqual(os.getpgid(child.pid), child.pid)
                self.assertNotEqual(child.pid, os.getpgrp())
                owned = receipt['children']
                self.assertEqual(len(owned), 2)
                self.assertTrue(all(os.getpgid(pid) == child.pid for pid in owned))
                os.killpg(child.pid, signal.SIGKILL)
                self.assertEqual(child.wait(timeout=5), -signal.SIGKILL)
                deadline = time.monotonic() + 5
                for pid in owned:
                    while True:
                        if libc is not None:
                            try:
                                if os.waitpid(pid, os.WNOHANG)[0] == pid:
                                    break
                            except ChildProcessError:
                                pass
                        try:
                            os.kill(pid, 0)
                        except ProcessLookupError:
                            break
                        if time.monotonic() > deadline:
                            self.fail(f'owned codec PID {pid} remained after group SIGKILL')
                        time.sleep(.01)
                with self.assertRaises(ProcessLookupError):
                    os.killpg(child.pid, 0)
                recovered = segments.recover(root, 'base_0', ffprobe=FFPROBE)
                self.assertEqual(recovered['frame_count'], 3)
                self.assertFalse(recovered['episode_complete'])
                self.assertTrue(recovered['uncommitted_files'])
                self.assertTrue(any(name.startswith('000001.') for name in recovered['uncommitted_files']))
                self.assertEqual(sorted(path.name for path in (root / 'base_0').glob('*.commit.json')), ['000000.commit.json'])
                actual = list(segments.iter_segment(root, 'base_0/000000.commit.json',
                                                    ffmpeg=FFMPEG, ffprobe=FFPROBE, png=False))
                self.assertEqual(actual, [frame(index)[:3] for index in range(3)])
        finally:
            if child is not None:
                if child.poll() is None:
                    # Only the still-live Popen-created group may be cleaned up.
                    if os.getpgid(child.pid) == child.pid and child.pid != os.getpgrp():
                        os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=5)
                if libc is not None:
                    deadline = time.monotonic() + 5
                    while time.monotonic() < deadline:
                        try:
                            reaped, _ = os.waitpid(-child.pid, os.WNOHANG)
                        except ChildProcessError:
                            break
                        if not reaped:
                            time.sleep(.01)
                child.stdout.close()
                child.stderr.close()
            if libc is not None:
                libc.prctl(36, previous_subreaper.value, 0, 0, 0)

    def test_roundtrip_exact_pixels_metadata_boundaries(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            writer = self.writer(root)
            refs = []
            for index in range(7):
                meta, _, _, color, depth = frame(index)
                refs.append(writer.add(meta, color, depth))
            manifests = writer.finish()
            self.assertEqual([item['frame_count'] for item in manifests], [3, 3, 1])
            self.assertEqual([item['frame'] for item in refs], [0, 1, 2, 0, 1, 2, 0])
            actual = []
            for name in dict.fromkeys(row['segment'] for row in refs):
                actual.extend(segments.iter_segment(root, name, ffmpeg=FFMPEG, ffprobe=FFPROBE, png=False))
            self.assertEqual(actual, [frame(index)[:3] for index in range(7)])
            self.assertEqual(len(list((root / 'base_0').iterdir())), 12)
            recovered = segments.recover(root, 'base_0', ffprobe=FFPROBE)
            self.assertEqual(recovered['frame_count'], 7)
            self.assertFalse(recovered['episode_complete'])
            self.assertEqual(recovered['uncommitted_files'], [])
            # PNG readers preserve the same pixels even though PNG compression changes.
            from PIL import Image
            import io
            import numpy as np
            restored = list(segments.iter_segment(root, refs[0]['segment'], ffmpeg=FFMPEG, ffprobe=FFPROBE))
            for index, (meta, color, depth) in enumerate(restored):
                self.assertEqual(Image.open(io.BytesIO(color)).tobytes(), frame(index)[1])
                self.assertEqual(np.asarray(Image.open(io.BytesIO(depth)), dtype='<u2').tobytes(), frame(index)[2])

    def test_pixels_match_png_wrapper_exactly_across_commits(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            writers = {kind: self.writer(root / kind) for kind in ('pixels', 'png')}
            try:
                references = {kind: [] for kind in writers}
                for index in range(4):
                    meta, color, depth, color_png, depth_png = frame(index)
                    references['pixels'].append(writers['pixels'].add_pixels(meta, color, depth, 17, 12))
                    references['png'].append(writers['png'].add(meta, color_png, depth_png))
                manifests = {kind: writer.finish() for kind, writer in writers.items()}
                self.assertEqual(references['pixels'], references['png'])
                self.assertEqual([row['decoded_pixel_sha256'] for row in manifests['pixels']],
                                 [row['decoded_pixel_sha256'] for row in manifests['png']])
                for kind, writer in writers.items():
                    actual = []
                    for segment in dict.fromkeys(ref['segment'] for ref in references[kind]):
                        actual.extend(segments.iter_segment(root / kind, segment,
                                                           ffmpeg=FFMPEG, ffprobe=FFPROBE, png=False))
                    # The frame fixture includes 0, 1, 255, 256, 32767, 32768, 65535 in LE order.
                    self.assertEqual(actual, [frame(index)[:3] for index in range(4)])
                    recovered = segments.recover(root / kind, 'base_0', ffprobe=FFPROBE)
                    self.assertEqual(recovered['frame_count'], 4)
                    self.assertEqual(recovered['uncommitted_files'], [])
                    self.assertFalse(recovered['episode_complete'])
                    first = writer.timings['first_frame']
                    self.assertEqual(first['input_format'], 'png' if kind == 'png' else 'rgb8_depth16le')
                    self.assertEqual('png_decode_wall_s' in first, kind == 'png')
            finally:
                for writer in writers.values():
                    writer.abort()

    def test_partial_recovery_and_integrity_rejections(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            writer = self.writer(root)
            for index in range(4):
                meta, _, _, color, depth = frame(index)
                writer.add(meta, color, depth)
                if index == 2:
                    writer.wait_committed()
            writer.abort()
            recovered = segments.recover(root, 'base_0', ffprobe=FFPROBE)
            self.assertEqual(recovered['frame_count'], 3)
            self.assertTrue(recovered['uncommitted_files'])
            self.assertFalse(recovered['episode_complete'])
            self.assertTrue(all(child.process.poll() is not None for child in writer.children.values()))
            path = root / 'base_0/000000.rgb.mp4'
            original = path.read_bytes()
            path.write_bytes(original[:-4])
            with self.assertRaisesRegex(ValueError, 'hash/size'):
                list(segments.iter_segment(root, 'base_0/000000.commit.json', ffmpeg=FFMPEG, ffprobe=FFPROBE))
            path.write_bytes(original)
            target = root / 'original-video.mp4'
            path.rename(target)
            path.symlink_to(target)
            with self.assertRaisesRegex(ValueError, 'symlink'):
                segments.verify_segment(root, 'base_0/000000.commit.json', ffprobe=FFPROBE)

    def test_decoded_hash_and_early_reader_close(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            writer = self.writer(root)
            for index in range(3):
                meta, _, _, color, depth = frame(index)
                writer.add(meta, color, depth)
            writer.finish()
            spawned = []
            original = segments._Child
            def track(*args, **kwargs):
                child = original(*args, **kwargs)
                spawned.append(child)
                return child
            with patch.object(segments, '_Child', side_effect=track):
                reader = segments.iter_segment(root, 'base_0/000000.commit.json', ffmpeg=FFMPEG, ffprobe=FFPROBE)
                next(reader)
                reader.close()
            self.assertTrue(all(child.process.poll() is not None for child in spawned))
            manifest_path = root / 'base_0/000000.commit.json'
            manifest = json.loads(manifest_path.read_text())
            manifest['decoded_pixel_sha256']['depth'] = hashlib.sha256(b'wrong-original').hexdigest()
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'decoded pixels'):
                list(segments.iter_segment(root, 'base_0/000000.commit.json', ffmpeg=FFMPEG, ffprobe=FFPROBE))


if __name__ == '__main__':
    unittest.main()
