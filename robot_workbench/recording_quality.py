"""Derived recording evidence; never changes a sealed recording or its v1 verdict.

Generation runs in the existing CPU export/offline worker. UI readers only load
the bounded sidecar; they never hash recordings or decode images on the Owner.
"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re


SCHEMA = 'recording_quality.v1'
REPORT = 'recording-quality-v1.json'
CAMERAS = ('base_0', 'left_wrist_0', 'right_wrist_0')
LABELS = dict(base_0='头部', left_wrist_0='左腕', right_wrist_0='右腕')
MAX_JSON = 4 * 1024**2


def _nonfinite(value):
    raise ValueError('nonfinite quality metadata: '+value)


def _json(path):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_JSON:
        raise ValueError('bounded regular quality metadata required')
    with path.open('rb') as stream:
        raw = stream.read(MAX_JSON + 1)
    if len(raw) > MAX_JSON:
        raise ValueError('quality metadata exceeds limit')
    value = json.loads(raw, parse_constant=_nonfinite)
    if not isinstance(value, dict):
        raise ValueError('quality metadata object required')
    return value


def validate_report(value, *, manifest_sha256):
    if (value.get('schema') != SCHEMA or value.get('raw_manifest_sha256') != manifest_sha256
            or not isinstance(value.get('legacy'), dict) or not isinstance(value.get('cameras'), dict)
            or set(value['cameras']) != set(CAMERAS)):
        raise ValueError('quality report identity or structure differs')
    for row in value['cameras'].values():
        if (not isinstance(row, dict) or type(row.get('accepted_pairs')) is not int or row['accepted_pairs'] < 0
                or not isinstance(row.get('discontinuities'), list)
                or not isinstance(row.get('missing_counter_values'), dict)):
            raise ValueError('invalid per-camera quality evidence')
    return value


def _stamp(row, environment):
    key = 'source_time_ns' if environment == 'fake' else 'host_receive_ns'
    value = row.get(key)
    if isinstance(value, str) and value.isascii() and value.isdigit():
        value = int(value)
    if type(value) is not int or value < 0:
        raise ValueError('original integer recording timestamp required')
    return value


def _stream(rows, environment):
    gaps, spans = [], []
    missing = dict(rgb=0, depth=0)
    previous = None
    maximum = 0
    for index, row in enumerate(rows):
        stamp = _stamp(row, environment)
        counters = [row.get('frame_number'), row.get('depth_frame_number')]
        if any(type(value) is not int or value < 0 for value in counters):
            raise ValueError('RGB and depth frame counters required')
        identity = tuple(row.get(key) for key in ('capture_instance_id', 'clock_domain', 'source_clock_id', 'timestamp_domain'))
        split = previous is None
        if previous is not None:
            before, old_counters, old_stamp, old_identity = previous
            delta = stamp - old_stamp
            maximum = max(maximum, delta)
            advances = [b - a for a, b in zip(old_counters, counters)]
            split = identity != old_identity or delta <= 0 or advances != [1, 1]
            if split:
                absent = [max(0, step - 1) if identity == old_identity else None for step in advances]
                for name, count in zip(('rgb', 'depth'), absent):
                    missing[name] += count or 0
                gaps.append(dict(before_index=before, after_index=index,
                    before_time_ns=old_stamp, after_time_ns=stamp,
                    before_counters=old_counters, after_counters=counters,
                    missing_rgb=absent[0], missing_depth=absent[1],
                    reason='instance_or_clock_changed' if identity != old_identity else
                        'timestamp_not_increasing' if delta <= 0 else 'accepted_counter_discontinuity',
                    attribution='unknown', missing_sample_times='not_observed'))
        if split:
            spans.append(dict(first_index=index, last_index=index,
                first_time_ns=stamp, last_time_ns=stamp,
                capture_instance_id=row.get('capture_instance_id'),
                clock_domain=row.get('clock_domain'), source_clock_id=row.get('source_clock_id')))
        else:
            spans[-1].update(last_index=index, last_time_ns=stamp)
        previous = (index, counters, stamp, identity)
    first, last = (rows[0], rows[-1]) if rows else ({}, {})
    return dict(accepted_pairs=len(rows), missing_counter_values=missing,
        first_time_ns=_stamp(first, environment) if rows else None,
        last_time_ns=_stamp(last, environment) if rows else None,
        maximum_interval_ns=maximum if len(rows) > 1 else None,
        timing_basis='source_time_ns' if environment == 'fake' else 'host_receive_ns',
        first_device_timestamps={key:first.get(key) for key in
            ('device_timestamp_ms', 'depth_device_timestamp_ms', 'timestamp_domain', 'synchronized')},
        consecutive_sequences=spans, discontinuities=gaps,
        sequence_scope='consecutive accepted counters; no consumer gap tolerance or cross-camera synchronization asserted',
        pixel_quality='not_assessed')


def _ingress(rows,environment):
    if environment=='fake':return dict(status='unsupported',evidence_type='simulation_sample',
        reason='simulation has no physical SDK frameset/align stages')
    references={}
    for index,row in enumerate(rows):
        diagnostic=row.get('capture_diagnostics') or {};journal=diagnostic.get('journal') or {}
        instance=row.get('capture_instance_id')
        if (diagnostic.get('schema')!='capture_ingress_diagnostics.v1' or not instance
                or journal.get('schema')!='capture_ingress_journal.v1'
                or journal.get('capture_instance_id')!=instance):continue
        references[instance]={key:deepcopy(journal.get(key)) for key in ('capture_instance_id','path',
            'committed_bytes','committed_records','committed_prefix_sha256','submitted_records',
            'queued_records','queue_dropped','file_omitted','full','limit_bytes')}
        references[instance]['write_error']=diagnostic.get('diagnostic_write_error')
        references[instance].update(as_of_accepted_index=index,
            as_of_record_sequence=journal.get('submitted_records'),verification='reference_only')
    return dict(status='snapshot_references' if references else 'not_available',references=list(references.values()),
        scope='latest journal prefix seen in accepted episode metadata per capture instance; not a complete episode journal or independent file verification')


def from_verified(raw, source, rows, *, environment, manifest_sha256):
    """Pure summary of bytes already checked by verify_raw and pinned indexes."""
    if environment not in ('fake', 'physical') or not re.fullmatch('[a-f0-9]{64}', manifest_sha256):
        raise ValueError('explicit recording environment and manifest SHA256 required')
    cameras = {camera:_stream(rows[camera], environment) for camera in CAMERAS}
    for camera in CAMERAS:cameras[camera]['ingress_diagnostics']=_ingress(rows[camera],environment)
    if any(cameras[camera]['accepted_pairs'] != raw.get('camera_frames', {}).get(camera) for camera in CAMERAS):
        raise ValueError('quality indexes differ from sealed frame counts')
    return dict(schema=SCHEMA, raw_manifest_sha256=manifest_sha256,
        analyzer=dict(version=SCHEMA, code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),
        environment=environment, robot_id=source['robot_id'], source_run_id=raw['source_run_id'],
        recording_kind=raw.get('recording_kind', 'frozen'),
        legacy=dict(capture_status=raw['capture_status'], validation=deepcopy(raw.get('validation')),
                    controller_outcome=raw.get('controller_outcome'), controller_reason=raw.get('controller_reason')),
        file_integrity=dict(status='passed', scope='sealed file sizes/hashes and recording identity; pixel quality is separate'),
        cameras=cameras, training=dict(status='unassessed', reason='consumer modality and timing contract not verified'),
        raw_modified=False, resampled=False, interpolated_depth=False)


def _write_verified(raw_root, output, raw, source, *, environment):
    from real_recording import _raw_file_index
    from segmented_playback import _rows
    root, output = Path(raw_root).resolve(), Path(output).resolve()
    if output.is_relative_to(root):
        raise ValueError('quality report must stay outside sealed raw')
    manifest_bytes = (root/'manifest.json').read_bytes()
    if json.loads(manifest_bytes) != raw:
        raise ValueError('raw manifest changed before quality analysis')
    names = [camera+'-pairs.jsonl' for camera in CAMERAS]
    _raw_file_index(root, names, expected=raw['files'])
    rows = {camera:list(_rows(root/(camera+'-pairs.jsonl'))) for camera in CAMERAS}
    # Bind parsed metadata to the same sealed bytes after the read as well.
    _raw_file_index(root, names, expected=raw['files'])
    if (root/'manifest.json').read_bytes() != manifest_bytes:
        raise ValueError('raw manifest changed during quality analysis')
    digest = hashlib.sha256(manifest_bytes).hexdigest()
    report = from_verified(raw, source, rows, environment=environment, manifest_sha256=digest)
    destination = output/'quality'/digest/REPORT
    destination.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2)+'\n').encode()
    if len(encoded) > MAX_JSON:
        raise ValueError('quality report exceeds metadata budget')
    if destination.exists():
        if destination.is_symlink() or destination.read_bytes() != encoded:
            raise ValueError('existing quality report differs; preserved without overwrite')
    else:
        with destination.open('xb') as stream:
            stream.write(encoded)
    return report


def generate(raw_root, output, *, environment):
    """Offline entry; full raw verification is mandatory, including incomplete raw."""
    from real_recording import verify_raw
    raw, source, _ = verify_raw(raw_root, environment=environment)
    return _write_verified(raw_root, output, raw, source, environment=environment)


def for_export(raw_root, output, raw, source, *, environment):
    """Called only after the existing exporter verified/decoded the recording."""
    try:
        return _write_verified(raw_root, output, raw, source, environment=environment)
    except (OSError, ValueError, TypeError, KeyError) as error:
        # A report failure must not relabel independently verified videos.
        try:
            (Path(output)/'quality-error.json').write_text(json.dumps(dict(status='unknown', error=str(error)[:700]))+'\n')
        except OSError:
            pass
        return None


def read_for_playback(output, *, cache=None):
    """Read small derived evidence only; caller owns the trusted playback root."""
    output = Path(output).resolve()
    manifest = _json(output/'manifest.json')
    digest = manifest.get('raw_manifest_sha256')
    if not isinstance(digest, str) or not re.fullmatch('[a-f0-9]{64}', digest):
        raise ValueError('playback must identify its sealed raw manifest')
    path = output/'quality'/digest/REPORT
    if not path.exists():
        return None
    if not path.resolve().is_relative_to(output):
        raise ValueError('quality report escapes playback root')
    info = path.stat()
    key = (str(path), info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)
    value = cache.get(key) if cache is not None else None
    if value is None:
        value = _json(path)
    if value.get('raw_manifest_sha256') != digest:
        raise ValueError('quality report belongs to a different recording')
    validate_report(value, manifest_sha256=digest)
    if cache is not None:
        if len(cache) >= 32:
            cache.clear()
        cache[key] = value
    return value


def download_for_playback(output):
    report=read_for_playback(output)
    if report is None:return None
    raw=(Path(output)/'quality'/report['raw_manifest_sha256']/REPORT).read_bytes()
    if len(raw)>MAX_JSON or json.loads(raw)!=report:raise ValueError('quality report changed during download')
    return dict(report=report,sidecar_sha256=hashlib.sha256(raw).hexdigest())


def presentation(report, *, video_ready, controller_outcome=None):
    """Additive UI view; does not rewrite v1 outcomes or grant training validity."""
    motion = controller_outcome or (report or {}).get('legacy', {}).get('controller_outcome') or 'unknown'
    cameras = (report or {}).get('cameras', {})
    gaps = any(row.get('discontinuities') for row in cameras.values())
    absent = any(not row.get('accepted_pairs') for row in cameras.values())
    complete = (report or {}).get('legacy', {}).get('capture_status') == 'complete'
    data = 'unknown' if not report else 'missing' if absent else 'gaps' if gaps else 'complete' if complete else 'requires_review'
    dimensions = dict(
        motion=dict(status=motion, label={'completed':'控制执行完成', 'stopped':'已停止', 'failed':'控制执行失败'}.get(motion, '控制结果待核验')),
        data=dict(status=data, label={'unknown':'数据质量待核验', 'missing':'存在缺失的数据流', 'gaps':'存在采样缺口', 'complete':'原始完整性通过', 'requires_review':'数据需复核'}[data]),
        video=dict(status='ready' if video_ready else 'unknown', label='视频可回放' if video_ready else '视频状态待核验'),
        training=dict(status='unassessed', label='训练用途待筛选'))
    return dict(schema='recording_quality_view.v1', dimensions=dimensions,
        raw_manifest_sha256=(report or {}).get('raw_manifest_sha256'),
        cameras={name:dict(label=LABELS[name], accepted_pairs=row.get('accepted_pairs'),
            missing_counter_values=row.get('missing_counter_values'), maximum_interval_ns=row.get('maximum_interval_ns'),
            sequence_count=len(row.get('consecutive_sequences',[])),discontinuity_count=len(row.get('discontinuities',[])),
            consecutive_sequences=deepcopy(row.get('consecutive_sequences',[])[:100]),
            discontinuities=deepcopy(row.get('discontinuities',[])[:20]),
            ingress_diagnostics=deepcopy(row.get('ingress_diagnostics',dict(status='not_available'))))
            for name,row in cameras.items() if name in LABELS})


def view_for_playback(output, *, video_ready, controller_outcome=None, cache=None):
    try:
        report = read_for_playback(output, cache=cache)
        error = None
    except (OSError, ValueError, KeyError, TypeError) as exc:
        report, error = None, str(exc)[:700]
    result = presentation(report, video_ready=video_ready, controller_outcome=controller_outcome)
    if error:
        result['quality_error'] = error
    return result
