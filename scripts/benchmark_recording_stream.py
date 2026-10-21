#!/usr/bin/env python3
"""CPU-only replay of a sealed fake recording into the current storage writer.

This never constructs an Owner, simulator or camera. All source observations,
commands and clocks are historical; successful storage does not qualify a new
motion or task. Use a new output directory for every invocation.
"""
import argparse
from copy import deepcopy
import hashlib
import io
from itertools import zip_longest
import json
from pathlib import Path
import queue
import sys
import threading
import time
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'rule_based_workbench'), str(ROOT / 'calibration/infra')]

from real_recording import CAMERAS, RealRecorder, RecordingSession, iter_rgbd, verify_raw
from robot_workbench.rgbd_channel import MemoryRgbdBudget, MemoryRgbdChannel

TRANSPORT_FIELDS = frozenset(('recording_run_id', 'recording_deadline_wall_ns',
                             'recording_duration_s', 'recorded_frames',
                             'recording_enabled', 'recording_error'))


def read(path):
    return json.loads(Path(path).read_text())


def rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]


def save(path, value):
    temporary = Path(str(path) + '.partial')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n')
    temporary.replace(path)


def load_input(root):
    started = time.monotonic()
    manifest, source, plan = verify_raw(root, environment='fake')
    if (manifest.get('capture_status') != 'complete' or manifest['validation']['status'] != 'passed'
            or manifest.get('controller_outcome') != 'completed'):
        raise ValueError('benchmark requires one complete prior recording')
    commands, feedback = rows(root / 'commands.jsonl'), rows(root / 'feedback.jsonl')
    frames = {camera: rows(root / (camera + '-pairs.jsonl')) for camera in CAMERAS}
    if (len(commands) != manifest['command_count'] or len(feedback) != manifest['feedback_count']
            or {camera: len(items) for camera, items in frames.items()} != manifest['camera_frames']):
        raise ValueError('input index counts differ from the sealed manifest')
    return dict(manifest=manifest, source=source, plan=plan, binding=read(root / 'binding.json'),
                timeline=read(root / 'execution-timeline.json'), owner=manifest['execution_owner'],
                commands=commands, feedback=feedback, frames=frames,
                validation_wall_s=time.monotonic()-started)


def pixels(blob, *, depth=False):
    from PIL import Image
    import numpy as np
    with Image.open(io.BytesIO(blob)) as image:
        if not depth and image.mode != 'RGB':
            raise ValueError('RGB pixels must retain their original color channels')
        values = np.asarray(image, dtype='<u2' if depth else 'uint8')
        return image.size, values.tobytes()


def compare(source_root, output_root, *, transport_changes=False):
    """Compare every original payload; never substitute video PTS for clocks."""
    began = time.monotonic()
    manifest, _, _ = verify_raw(output_root, environment='fake')
    if manifest.get('capture_status') != 'complete' or manifest['validation']['status'] != 'passed':
        raise ValueError('benchmark output did not seal as complete raw')
    for name in ('source.json', 'plan.json', 'binding.json', 'execution-timeline.json'):
        if read(source_root / name) != read(output_root / name):
            raise ValueError('frozen input changed: ' + name)
    for name in ('commands.jsonl', 'feedback.jsonl'):
        if rows(source_root / name) != rows(output_root / name):
            raise ValueError('actual command or feedback changed: ' + name)
    result = {}; changes = {}
    for camera in CAMERAS:
        digests = {kind: hashlib.sha256() for kind in ('rgb', 'depth')}
        count = 0
        for old, new in zip_longest(iter_rgbd(source_root, camera, environment='fake'),
                                   iter_rgbd(output_root, camera, environment='fake')):
            if old is None or new is None:
                raise ValueError(camera + ': frame count changed')
            _, old_meta, old_color, old_depth = old
            _, new_meta, new_color, new_depth = new
            differing = {key for key in old_meta.keys() | new_meta.keys()
                         if (key in old_meta) != (key in new_meta) or old_meta.get(key) != new_meta.get(key)}
            if differing and (not transport_changes or not differing <= TRANSPORT_FIELDS):
                raise ValueError(camera + ': original frame metadata changed: ' + str(sorted(differing)))
            for key in differing:
                changes.setdefault(camera, {}).setdefault(key, 0)
                changes[camera][key] += 1
            for kind, a, b in (('rgb', old_color, new_color), ('depth', old_depth, new_depth)):
                old_pixels = pixels(a, depth=kind == 'depth')
                new_pixels = pixels(b, depth=kind == 'depth')
                if old_pixels != new_pixels:
                    raise ValueError(f'{camera} frame {count}: decoded {kind} pixels changed')
                digests[kind].update(old_pixels[1])
            count += 1
        result[camera] = dict(frames=count, decoded_pixel_sha256={k: v.hexdigest() for k, v in digests.items()})
    return dict(passed=True, comparison_wall_s=time.monotonic()-began, cameras=result,
                command_feedback_exact=True, source_timestamps_exact=True,
                metadata_exact=not any(changes.values()),
                metadata_exact_except_new_transport_fields=transport_changes,
                changed_transport_fields=changes,
                capture_status=manifest['capture_status'], raw_validation=manifest['validation'])


def unpaced(source_root, output, inputs):
    """A: drain the prior complete input as quickly as the real writer accepts."""
    began = time.monotonic(); raw = output / 'raw'
    recorder = RealRecorder(raw, inputs['source'], inputs['plan'], inputs['binding'],
        environment='fake', execution_owner=inputs['owner'], execution_timeline=inputs['timeline'],
        storage_format='segments')
    try:
        for row in inputs['commands']:
            recorder.record_command(**row)
        for row in inputs['feedback']:
            recorder.record_feedback(**row)
        for camera in CAMERAS:
            for index, (_, meta, color, depth) in enumerate(iter_rgbd(source_root, camera, environment='fake'), 1):
                if not recorder.record_rgbd_frame(camera, meta, color, depth):
                    raise ValueError('original frame was not accepted')
                if index % 120 == 0:
                    save(output / 'progress.json', dict(mode='unpaced', camera=camera, frames=index,
                                                      elapsed_wall_s=time.monotonic()-began))
        supplied = time.monotonic()
        recorder.seal('completed')
        sealed = time.monotonic()
    except BaseException:
        for writer in recorder.segment_writers.values():
            writer.abort()
        for stream in recorder.index_streams.values():
            stream.close()
        raise
    timing = dict(input_and_write_wall_s=supplied-began, supply_complete_to_sealed_wall_s=sealed-supplied,
                  total_write_and_seal_wall_s=sealed-began, seal=recorder.seal_timings)
    return dict(mode='unpaced', timing=timing, equivalence=compare(source_root, raw))


class ReplayClock:
    """Original wall epoch plus actual elapsed time; source timestamps unchanged."""
    def __init__(self, first_wall_ns):
        self.first_wall_ns = first_wall_ns
        self.anchor = None

    def start(self):
        self.anchor = time.monotonic() + .3

    def __call__(self):
        return self.first_wall_ns - 300_000_000 if self.anchor is None else (
            self.first_wall_ns + round((time.monotonic()-self.anchor)*1e9))


def paced(source_root, output, inputs, tail_timeout):
    """B: historical wall-cadence producer -> bounded memory -> current session."""
    events = [(r['send_wall_ns'], 1, 'command', r) for r in inputs['commands']]
    events += [(r['sample_wall_ns'], 0, 'feedback', r) for r in inputs['feedback']]
    events += [(int(r['host_receive_ns']), 2, camera, index) for camera in CAMERAS
               for index, r in enumerate(inputs['frames'][camera])]
    events.sort(key=lambda event: event[:3])
    first_wall, final_wall = events[0][0], events[-1][0]
    historical_duration = (final_wall-first_wall)/1e9
    if not 0 < historical_duration < 6600:
        raise ValueError('historical wall replay must fit the bounded 7200 second recording budget')
    clock = ReplayClock(first_wall); current = []; cancelled = threading.Event()
    budget = MemoryRgbdBudget()
    channels = {camera: MemoryRgbdChannel(clock_ns=clock, budget=budget,
        lease_guard=lambda lease: bool(current) and current[0].lease_active(lease)) for camera in CAMERAS}
    instance = 'offline-replay-' + uuid.uuid4().hex
    session = RecordingSession(output/'raw', inputs['source'], inputs['plan'], inputs['binding'],
        environment='fake', execution_owner=inputs['owner'], camera_channels=channels,
        storage_format='segments', clock_ns=clock, defer_execution=True, execution_instance_id=instance,
        preparation_active=lambda: not cancelled.is_set(), preparation_deadline_monotonic=time.monotonic()+10,
        persistent_service=True, max_wall_time_seconds=min(7200, historical_duration+tail_timeout+60))
    current.append(session)
    prefetch = queue.Queue(maxsize=2); stop_reader = threading.Event()
    reader_stats = dict(frames=0, peak_queued_frames=0, peak_payload_bytes=0, error=None)
    def load_frames():
        streams = {camera: iter_rgbd(source_root, camera, environment='fake') for camera in CAMERAS}
        try:
            for _, _, camera, index in events:
                if camera not in channels:
                    continue
                frame = next(streams[camera])
                reader_stats['peak_payload_bytes'] = max(reader_stats['peak_payload_bytes'], len(frame[2])+len(frame[3]))
                while not stop_reader.is_set():
                    try:
                        prefetch.put((camera, index, frame), timeout=.1)
                        reader_stats['frames'] += 1
                        reader_stats['peak_queued_frames'] = max(reader_stats['peak_queued_frames'], prefetch.qsize())
                        break
                    except queue.Full:
                        pass
                if stop_reader.is_set():
                    return
        except BaseException as error:
            reader_stats['error'] = f'{type(error).__name__}: {error}'
        finally:
            for stream in streams.values():
                stream.close()
    loader = threading.Thread(target=load_frames, name='historical-rgbd-reader', daemon=True)
    lateness=[]; event_peak=0; published={camera: 0 for camera in CAMERAS}; finished=None
    try:
        if not session.prepared.wait(10):
            raise RuntimeError('recording preparation failed: ' + str(session.error))
        loader.start(); clock.start(); last_progress=time.monotonic()
        for wall, _, kind, value in events:
            target = clock.anchor+(wall-first_wall)/1e9
            delay = target-time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if session.error or session.done.is_set():
                raise RuntimeError('recording stopped before all historical input: ' + str(session.snapshot()))
            if kind in channels:
                while True:
                    try:
                        camera, index, frame = prefetch.get(timeout=.1)
                        break
                    except queue.Empty:
                        if reader_stats['error'] or not loader.is_alive():
                            raise RuntimeError('historical input reader failed: ' + str(reader_stats['error']))
                if (camera, index) != (kind, value):
                    raise ValueError('bounded readahead changed frame ordering')
                _, meta, color, depth = frame
                # Only the new transport lease fields change. Captured host and
                # source clocks, original Owner and image provenance stay exact.
                meta=deepcopy(meta)
                if not channels[camera].write(meta, color, depth, sampled_token=channels[camera].token()):
                    raise RuntimeError('memory producer rejected historical frame: ' + str(channels[camera].snapshot()))
                published[camera] += 1
            else:
                if not session.execution_bound.is_set():
                    # Preserve historical camera pre-roll before the first Owner
                    # event; supplying only after bind hid live startup backlog.
                    session.bind_execution(inputs['timeline'], execution_instance_id=instance, execution_owner=inputs['owner'])
                session.submit(kind, **value)
                event_peak=max(event_peak,session.events.qsize())
                if kind == 'feedback' and value is inputs['feedback'][-1]:
                    session.finish('completed'); finished=time.monotonic()
            lateness.append(max(0.,time.monotonic()-target))
            if time.monotonic()-last_progress >= 5:
                last_progress=time.monotonic()
                save(output/'progress.json',dict(mode='paced',elapsed_wall_s=last_progress-clock.anchor,
                    historical_wall_duration_s=historical_duration,published_frames=published,
                    event_queue_observed_peak=event_peak,writer_stage=session.writer_stage,
                    camera_queue={c: ch.snapshot() for c,ch in channels.items()}))
        supplied=time.monotonic()
        if finished is None:
            raise ValueError('historical final feedback was not submitted')
        if not session.done.wait(tail_timeout):
            raise TimeoutError('bounded historical writer tail exceeded timeout')
        done=time.monotonic(); snapshot=session.snapshot()
        if session.error or snapshot['seal_error'] or not session.manifest:
            raise RuntimeError('historical recording failed: ' + str(snapshot))
        if session.manifest['validation']['status'] != 'passed':
            raise ValueError('historical recording incomplete: ' + str(session.manifest['validation']))
    except BaseException as error:
        session.fail('offline benchmark input failed: ' + str(error))
        session.done.wait(min(tail_timeout,30))
        save(output/'failed-session.json',session.snapshot())
        raise
    finally:
        stop_reader.set()
        if loader.is_alive():
            loader.join(timeout=5)
    sorted_lateness=sorted(lateness)
    timing=dict(historical_wall_duration_s=historical_duration,supply_wall_s=supplied-clock.anchor,
        last_feedback_submission_to_done_wall_s=done-finished,supply_complete_to_done_wall_s=done-supplied,
        total_replay_wall_s=done-clock.anchor,seal=snapshot['seal_timings'],
        supply_lateness_p95_s=sorted_lateness[int((len(sorted_lateness)-1)*.95)],
        supply_lateness_max_s=max(lateness),queue_observed_peak=event_peak,
        memory_channels=snapshot['memory_channels'],input_readahead=reader_stats)
    return dict(mode='paced',timing=timing,
                equivalence=compare(source_root,output/'raw',transport_changes=True))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-raw',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mode',choices=('unpaced','paced','both'),default='both')
    parser.add_argument('--tail-timeout',type=float,default=180)
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    source=args.source_raw.resolve(strict=True); output=args.output.resolve()
    if output.exists() or output.is_relative_to(source) or source.is_relative_to(output):
        raise ValueError('new benchmark output must be separate from the immutable prior recording')
    if not 1 <= args.tail_timeout <= 600:
        raise ValueError('tail timeout must be 1..600 seconds')
    inputs=load_input(source)
    receipt=dict(schema='offline_recording_stream_benchmark.v1',source_raw=str(source),
        source_manifest_sha256=hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest(),
        original_run_id=inputs['source']['run_id'],source_clock_id=inputs['source']['clock']['source_clock_id'],
        original_counts=dict(commands=len(inputs['commands']),feedback=len(inputs['feedback']),
                             rgbd={c:len(r) for c,r in inputs['frames'].items()}),
        replay_of_prior_input=True,new_collection=False,motion_requested=False,gpu_used=False,
        task_success_evidence=False,source_validation_wall_s=inputs['validation_wall_s'],results={})
    if args.dry_run:
        print(json.dumps(dict(receipt,status='dry-run-passed'),ensure_ascii=False));return
    output.mkdir(parents=True,exist_ok=False);save(output/'receipt.json',dict(receipt,status='running'))
    try:
        for mode in ('unpaced','paced') if args.mode=='both' else (args.mode,):
            directory=output/mode;directory.mkdir()
            receipt['results'][mode]=(unpaced(source,directory,inputs) if mode=='unpaced'
                                      else paced(source,directory,inputs,args.tail_timeout))
            save(output/'receipt.json',dict(receipt,status='running'))
        save(output/'receipt.json',dict(receipt,status='passed'))
    except BaseException as error:
        save(output/'receipt.json',dict(receipt,status='failed',error=f'{type(error).__name__}: {error}',
                                       traceback=traceback.format_exc()))
        raise
    print(json.dumps(dict(status='passed',receipt=str(output/'receipt.json')),ensure_ascii=False))


if __name__=='__main__':
    main()
