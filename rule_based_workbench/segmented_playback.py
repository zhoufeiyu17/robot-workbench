"""Derive browser replay directly from lossless segments, without PNG staging."""
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import json
import math
import re
from pathlib import Path
import shutil
import statistics
import subprocess
import tempfile
import time

from playback import sha256
from real_contract import namespace, validate_namespace
from rgbd_segments import _safe, verify_segment
from robot_workbench.sample_clock import clock_for_source, sample_stamp, assert_progress

CAMERAS = ('base_0', 'left_wrist_0', 'right_wrist_0')
_EXPORT_WORKERS = 3  # One bounded, single-thread codec path per declared camera.


@lru_cache(maxsize=8)
def require_packet_remux_codec(ffmpeg):
    version=subprocess.check_output([ffmpeg,'-version'],text=True,timeout=5).splitlines()[0]
    match=re.search(r'ffmpeg version (?:n)?(\d+)\.',version)
    if match is None or int(match[1])<7:
        raise ValueError('streaming browser replay requires FFmpeg7+ packet timing-file support')
    return version


def _setpts(offsets):
    """N is the decoded frame index. Balanced branches avoid parser-depth limits."""
    if not offsets or offsets[0] != 0 or any(b <= a for a, b in zip(offsets, offsets[1:])):
        raise ValueError('strictly increasing microsecond frame offsets required')
    return _frame_expression(offsets)


def _frame_expression(values):
    def branch(start, end):
        if end - start == 1:
            return str(values[start])
        middle = (start + end) // 2
        return f'if(lt(N,{middle}),{branch(start, middle)},{branch(middle, end)})'
    return branch(0, len(values))


def encode_browser_segment(root, manifest, indices, destination, *, ffmpeg='ffmpeg', runner=subprocess.run):
    """Encode one committed raw segment; timestamps are assigned at final remux."""
    root, destination = Path(root), Path(destination)
    # The owned child sanitizes PATH before nice launches the codec.
    ffmpeg = shutil.which(ffmpeg) or ffmpeg
    if indices != sorted(set(indices)) or any(type(i)is not int or not 0<=i<manifest['frame_count']for i in indices):
        raise ValueError('ordered source frame selection required')
    source = _safe(root, manifest['files']['rgb']['path'])
    result = dict(schema='workbench.browser-segment.v1', source_rgb=manifest['files']['rgb'],
                  source_frames=manifest['files']['frames'], indices=indices, file=None)
    if not indices:return result
    selection = '+'.join(f'eq(n,{i})'for i in indices)
    command = [ffmpeg,'-nostdin','-v','error','-n','-threads','1','-filter_threads','1','-i',str(source),
        '-vf',f"select='{selection}',setpts=N/(30*TB)",'-frames:v',str(len(indices)),'-an',
        '-c:v','libx264','-threads','1','-preset','fast','-crf','24','-pix_fmt','yuv420p','-bf','0',
        '-vsync','vfr','-video_track_timescale','1000000',str(destination)]
    nice = shutil.which('nice')
    if nice:command = [nice,'-n','10',*command]
    runner(command, check=True, timeout=60, capture_output=True)
    result['file'] = dict(path=str(destination.relative_to(root)),size_bytes=destination.stat().st_size,
                          sha256=sha256(destination))
    return result


def _browser_parts(root, manifests, selected, temporary, run, ffmpeg):
    """Reuse live encodes, rebuilding only missing or differently selected segments."""
    parts=[];reused=rebuilt=0
    for index,(manifest,indices)in enumerate(zip(manifests,selected)):
        if not indices:continue
        source=_safe(root,manifest['files']['rgb']['path'])
        sidecar=source.with_suffix('.browser.json');cached=None
        try:
            if sidecar.stat().st_size>64*1024:raise ValueError('oversized replay sidecar')
            candidate=json.loads(sidecar.read_bytes())
            if (candidate.get('schema')!='workbench.browser-segment.v1' or candidate.get('indices')!=indices
                    or candidate.get('source_rgb')!=manifest['files']['rgb']
                    or candidate.get('source_frames')!=manifest['files']['frames']):
                raise ValueError('live replay selection differs from final activity')
            pin=candidate['file'];path=_safe(root,pin['path'])
            if path.stat().st_size!=pin['size_bytes'] or sha256(path)!=pin['sha256']:
                raise ValueError('live replay bytes changed')
            cached=path
        except (OSError,ValueError,KeyError,TypeError):pass
        if cached is None:
            # The caller's temporary directory is inside the capture camera root.
            cached=temporary/f'rebuilt-{index}.mp4'
            encode_browser_segment(root,manifest,indices,cached,ffmpeg=ffmpeg,runner=run)
            rebuilt+=1
        else:reused+=1
        parts.append(cached)
    return parts,dict(reused_segments=reused,rebuilt_segments=rebuilt)


def _remux_browser(parts,destination,offsets,tail,temporary,run,ffmpeg):
    concat=temporary/'browser.concat';filters=temporary/'packet-timing.txt'
    concat.write_text('ffconcat version 1.0\n'+''.join("file '"+str(p).replace("'","'\\''")+"'\n"for p in parts))
    durations=[b-a for a,b in zip(offsets,offsets[1:])]+[tail]
    filters.write_text("setts=ts='"+_setpts(offsets)+"':duration='"+_frame_expression(durations)+"':time_base=1/1000000")
    run([ffmpeg,'-nostdin','-v','error','-n','-f','concat','-safe','0','-i',str(concat),'-map','0:v:0',
         '-c:v','copy','-/bsf:v',str(filters),'-video_track_timescale','1000000','-movflags','+faststart',
         str(destination)],check=True,timeout=120,capture_output=True)


def _pinned(root, raw, name):
    path = _safe(root, name)
    pin = raw.get('files', {}).get(name)
    if not isinstance(pin, dict) or path.stat().st_size != pin.get('size_bytes') or sha256(path) != pin.get('sha256'):
        raise ValueError('sealed replay input changed: ' + name)
    return path


def _rows(path):
    with path.open('rb') as stream:
        while line := stream.readline(64 * 1024 + 1):
            if len(line) > 64 * 1024 or not line.endswith(b'\n'):
                raise ValueError('bounded complete frame metadata required')
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError('frame metadata object required')
            yield row


def _camera_inputs(root, raw, camera):
    """Bind ordered pair-index rows to current committed segment bytes."""
    rows = list(_rows(_pinned(root, raw, camera + '-pairs.jsonl')))
    if not rows or len(rows) != raw.get('camera_frames', {}).get(camera):
        raise ValueError('all camera frames must match sealed counts')
    paths, index = [], 0
    while index < len(rows):
        segment = rows[index].get('segment')
        expected = f'camera/{camera}/{len(paths):06d}.commit.json'
        if segment != expected:
            raise ValueError('replay requires the complete ordered segment prefix')
        commit = _pinned(root, raw, segment)
        manifest = verify_segment(root / 'camera', str(commit.relative_to(root / 'camera')))
        for pin in manifest['files'].values():
            # verify_segment just hashed these files; bind that check to the
            # episode's independent sealed file pins without reading them twice.
            sealed = raw.get('files', {}).get('camera/' + pin['path'])
            if sealed != {key: pin[key] for key in ('size_bytes', 'sha256')}:
                raise ValueError('segment bytes differ from sealed raw file identity')
        metadata_path = _safe(root / 'camera', manifest['files']['frames']['path'])
        for frame, metadata in enumerate(_rows(metadata_path)):
            if index >= len(rows):
                raise ValueError('segment has unindexed frames')
            row = rows[index]
            if (row.get('segment') != segment or row.get('segment_frame') != frame
                    or metadata != {key: value for key, value in row.items() if key not in ('segment', 'segment_frame')}):
                raise ValueError('segment metadata or frame order differs from sealed pairs')
            index += 1
        paths.append(_safe(root / 'camera', manifest['files']['rgb']['path']))
    return rows, paths


def _probe_video(path, runner, offsets_us, duration_us, *, decode=True):
    unit = 'frame' if decode else 'packet'
    count_key = 'nb_read_' + unit + 's'
    time_key = 'best_effort_timestamp_time' if decode else 'pts_time'
    payload = json.loads(runner(['ffprobe', '-v', 'error', '-threads', '1', '-select_streams', 'v:0', '-count_' + unit + 's',
        '-show_entries', f'stream=codec_name,pix_fmt,width,height,{count_key},duration:{unit}={time_key}',
        '-show_' + unit + 's', '-of', 'json', str(path)], check=True, timeout=120, capture_output=True, text=True).stdout)
    streams, frames = payload.get('streams', []), payload.get(unit + 's', [])
    if (len(streams) != 1 or streams[0].get('codec_name') != 'h264' or streams[0].get('pix_fmt') != 'yuv420p'
            or int(streams[0].get(count_key, -1)) != len(offsets_us) or len(frames) != len(offsets_us)):
        raise ValueError('browser replay codec/frame count differs')
    for frame, expected in zip(frames, offsets_us):
        actual = float(frame.get(time_key, 'nan')) * 1e6
        if not math.isfinite(actual) or abs(actual - expected) > 1.000001:
            raise ValueError('source replay frame PTS differs by more than 1 microsecond')
    actual_duration = float(streams[0].get('duration', 'nan')) * 1e6
    if not math.isfinite(actual_duration) or abs(actual_duration - duration_us) > 1.000001:
        raise ValueError('source replay duration/final-frame hold differs')
    return streams[0]


def export_segments(raw_root, raw_manifest, source, plan, output, *, runner=subprocess.run, environment='fake',
                    target=None, target_paths=None, target_manifest_path=None):
    """The caller validates the complete raw once; recheck only consumed inputs here.

    target and target_paths, when supplied, have already passed the existing
    source/plan/timeline identity checks in export_playback.
    """
    started = time.monotonic()
    root, output = Path(raw_root), Path(output)
    if root.is_symlink() or output.is_symlink():
        raise ValueError('symlink raw/output directory rejected')
    root, output = root.resolve(), output.resolve()
    if output.is_relative_to(root):
        raise ValueError('derived playback must stay outside sealed raw directory')
    expected_targets = {f'target-{camera}.mp4' for camera in CAMERAS} if target is not None else set()
    if set(target_paths or {}) != expected_targets:
        raise ValueError('target replay requires its complete three-camera file set')
    if json.loads(_safe(root, 'manifest.json').read_bytes()) != raw_manifest:
        raise ValueError('raw manifest changed before segmented export')
    spec = namespace(environment)
    validate_namespace(raw_manifest, environment)
    policy=raw_manifest.get('recording_kind')=='streaming_policy'
    if (raw_manifest.get('schema_version') != spec['raw_schema']
            or raw_manifest.get('rgbd_storage') != 'lossless_segments_v1'):
        raise ValueError('segmented raw source/plan identity differs')
    if policy:
        from integration import _digest
        if (plan is not None or target is not None or source.get('schema_version')!='policy_execution_source.v1'
                or raw_manifest.get('source_run_id')!=source.get('execution_instance_id')
                or raw_manifest.get('source_digest')!=_digest(source)
                or raw_manifest.get('actual_commands_sha256')!=raw_manifest['files'].get('commands.jsonl',{}).get('sha256')):
            raise ValueError('policy playback requires actual command identity and no invented plan')
    elif (plan is None or raw_manifest.get('source_run_id')!=source.get('run_id')
            or raw_manifest.get('plan_run_id')!=plan.get('run_id') or raw_manifest.get('timeline_digest')!=plan.get('timeline_digest')):
        raise ValueError('segmented raw source/plan identity differs')
    for name, value in [('source.json', source), *([] if policy else [('plan.json',plan)])]:
        if json.loads(_pinned(root, raw_manifest, name).read_bytes()) != value:
            raise ValueError('sealed replay identity changed')
    command_path = _pinned(root, raw_manifest, 'commands.jsonl')
    with command_path.open('rb') as stream:
        command = json.loads(stream.readline(64 * 1024 + 1))
    clock = clock_for_source(source, environment)
    command_start = sample_stamp(command, clock, wall_key='send_wall_ns', source_key='send_source_time_ns', require_step=False)[0]
    output.mkdir(parents=True, exist_ok=False)
    files, timing = {}, {}
    diagnostics = {'schema': 'source_playback_timings.v1', 'storage': 'lossless_segments_v1',
                   'camera_workers': 2, 'cameras': {}}
    with tempfile.TemporaryDirectory(dir=output) as temporary:
        temporary = Path(temporary)
        def export_camera(camera):
            before = time.monotonic()
            rows, paths = _camera_inputs(root, raw_manifest, camera)
            detail = {'frames': len(rows), 'segments': len(paths),
                      'verify_inputs_wall_s': time.monotonic() - before}
            stamps, previous = [], None
            for row in rows:
                current = sample_stamp(row, clock)
                previous = assert_progress(previous, current)
                stamps.append(current[0])
            periods = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
            last = statistics.median(periods) if periods else 1 / rows[0]['capture_fps']
            if not math.isfinite(last) or last <= 0:
                raise ValueError('positive final source-frame duration required')
            offsets_us = [(stamp - stamps[0] + 500) // 1000 for stamp in stamps]
            tail_us = max(1, round(last * 1e6))
            trim=''
            if policy:
                from bisect import bisect_right,bisect_left
                duration=min(source['duration_s'],raw_manifest['actual_duration_s'])
                end=command_start+round(duration*1e9)
                first_index=bisect_right(stamps,command_start)-1
                end_index=bisect_left(stamps,end)
                if duration<=0 or first_index<0 or end_index<=first_index:
                    raise ValueError('policy replay lacks the initial command bracket or a positive observed window')
                trim=f'trim=start_frame={first_index}:end_frame={end_index},'
                rows,stamps=rows[first_index:end_index],stamps[first_index:end_index]
                offsets_us=[max(0,(stamp-command_start+500)//1000) for stamp in stamps]
                tail_us=round(duration*1e6)-offsets_us[-1]
                if tail_us<=0:raise ValueError('policy replay final source-frame hold must be positive')
                last=tail_us/1e6
                detail.update(raw_start_frame=first_index,raw_end_frame=end_index,
                    display_window='actual command start to bounded observed end; initial bracket held at t=0')
            concat, filters = temporary / (camera + '.concat'), temporary / (camera + '.filter')
            def escaped(path):
                return str(path).replace("'", "'\\''")
            concat.write_text('ffconcat version 1.0\n' + ''.join(f"file '{escaped(path)}'\n" for path in paths))
            filters.write_text(trim+"settb=1/1000000,setpts='" + _setpts(offsets_us) + "'\n")
            destination = output / f'source-{camera}.mp4'
            before = time.monotonic()
            # The input rate supplies the final packet duration; setpts restores
            # every source timestamp. Output -r conflicts with VFR in FFmpeg 8.
            runner(['ffmpeg', '-nostdin', '-v', 'error', '-n', '-threads', '1', '-filter_threads', '1',
                    '-f', 'concat', '-safe', '0', '-r', f'1000000/{tail_us}', '-i', str(concat), '-filter_script:v', str(filters), '-an',
                    '-c:v', 'libx264', '-threads', '1', '-preset', 'fast', '-crf', '24', '-pix_fmt', 'yuv420p',
                    '-bf', '0',
                    '-vsync', 'vfr', '-enc_time_base', '1:1000000', '-video_track_timescale', '1000000',
                    '-movflags', '+faststart', str(destination)], check=True, timeout=300)
            detail['encode_wall_s'] = time.monotonic() - before
            before = time.monotonic()
            probe = _probe_video(destination, runner, offsets_us, offsets_us[-1] + tail_us)
            pin = {'size_bytes': destination.stat().st_size, 'sha256': sha256(destination), 'probe': probe}
            detail['verify_video_wall_s'] = time.monotonic() - before
            camera_timing = {'clock': 'simulation' if environment == 'fake' else 'host_receive',
                             'frame_offsets_s': [(stamp - command_start) / 1e9 for stamp in stamps],
                             'frame_count': len(rows), 'depth_in_raw': True, 'last_frame_duration_s': last,
                             'duration_s': (offsets_us[-1]+tail_us)/1e6 if policy else (stamps[-1] - stamps[0]) / 1e9 + last,
                             'encoded_duration_s': (offsets_us[-1] + tail_us) / 1e6,
                             'pts_quantization_s': .000001}
            if environment == 'fake':
                camera_timing.update(source_clock_id=clock['source_clock_id'], host_receive_ns=[row['host_receive_ns'] for row in rows])
            return camera, destination.name, pin, camera_timing, detail
        # Each camera owns separate files and bounded codec processes. Join all
        # workers before cleanup; publish no manifest unless every check passes.
        with ThreadPoolExecutor(max_workers=_EXPORT_WORKERS, thread_name_prefix='rgbd-export') as workers:
            for camera, name, pin, camera_timing, detail in workers.map(export_camera, CAMERAS):
                files[name], timing[camera], diagnostics['cameras'][camera] = pin, camera_timing, detail
        for name, path in (target_paths or {}).items():
            if name not in {f'target-{camera}.mp4' for camera in CAMERAS} or target is None:
                raise ValueError('invalid target replay file')
            pin, path = target['files'][name], Path(path)
            if path.is_symlink() or path.stat().st_size != pin['size_bytes'] or sha256(path) != pin['sha256']:
                raise ValueError('target replay changed before copy')
            shutil.copyfile(path, output / name)
            if sha256(output / name) != pin['sha256']:
                raise ValueError('target replay copy differs')
            files[name] = pin
    manifest = dict(schema_version=spec['playback_schema'], mode=spec['mode'], clock='elapsed',
                    source_run_id=source['execution_instance_id'] if policy else source['run_id'],
                    provenance={'source': spec['evidence'], **({'target': 'simulation'} if target is not None else {})},
                    files=files, source_timing=timing, raw_manifest_sha256=sha256(root / 'manifest.json'),
                    target_manifest_sha256=sha256(target_manifest_path) if target_manifest_path is not None else None,
                    capture_status=raw_manifest['capture_status'], controller_outcome=raw_manifest['controller_outcome'],
                    task_outcome='unverified')
    if policy:manifest.update(recording_kind='streaming_policy',actual_commands_sha256=raw_manifest['actual_commands_sha256'])
    else:manifest.update(plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'])
    if environment == 'fake':
        manifest.update(environment='fake', simulation_only=True, source_clock=clock)
    (output / 'manifest.json').write_text(json.dumps(manifest, allow_nan=False, indent=2) + '\n')
    diagnostics['total_wall_s'] = time.monotonic() - started
    (output / 'export-timings.json').write_text(json.dumps(diagnostics, allow_nan=False, indent=2) + '\n')
    return manifest


def export_capture(root, manifest, output, *, probe_cache=None,verify_raw=True):
    """Reuse source-time replay checks for the generic CaptureSession format."""
    root,output=Path(root),Path(output)
    output.mkdir(exist_ok=False)
    ffmpeg,ffprobe=shutil.which('ffmpeg'),shutil.which('ffprobe')
    if not ffmpeg or not ffprobe:raise ValueError('CPU replay codecs unavailable')
    files={};streams={};activity=None
    if 'recording_activity'in manifest:
        from robot_workbench.recording_activity import ActiveTimeAxis
        activity=ActiveTimeAxis(manifest['recording_activity'])
    def source_file(name):return _pinned(root,manifest,name)if verify_raw else _safe(root,name)
    spec=json.loads(source_file('capture.json').read_text())
    descriptors={row['stream_id']:row for row in spec['source_descriptors']}
    segment_pins={row['segment']:{key:row[key] for key in ('size_bytes','sha256')}
                  for row in _rows(source_file(manifest['segments_ref']))}
    def run(arguments,**kwargs):
        arguments=[ffprobe if arguments[0]=='ffprobe' else arguments[0],*arguments[1:]]
        return subprocess.run(arguments,**kwargs)
    with tempfile.TemporaryDirectory(dir=output) as tmp:
        temporary=Path(tmp)
        def export_camera(item):
            identity,state=item
            descriptor=descriptors[identity]
            if not state.get('committed') or descriptor['kind']!='video' or 'source_ref' not in descriptor:return None
            rows=[]
            with source_file(identity+'.jsonl').open('rb') as source:
                for _ in range(state['committed']):
                    rows.append(json.loads(source.readline()))
            if not rows:return None
            paths=[];manifests=[];segment_names=[];previous=None;metadata_rows=None;active_segment=None;frame_index=0
            for row in rows:
                ref=row['payload']['frame_ref'];meta=row['payload']['metadata'];segment=ref['segment']
                if segment!=active_segment:
                    if metadata_rows is not None and frame_index!=len(metadata_rows):
                        raise ValueError('capture replay skipped segment frames')
                    if segment!=f'camera/{identity}/{len(paths):06d}.commit.json':
                        raise ValueError('capture replay requires ordered segment prefix')
                    commit=_pinned(root,{'files':segment_pins},segment)if verify_raw else _safe(root,segment)
                    verified=(verify_segment(root/'camera',str(commit.relative_to(root/'camera')),ffprobe=ffprobe,probe_cache=probe_cache)
                              if verify_raw else json.loads(commit.read_text()))
                    metadata_rows=list(_rows(_safe(root/'camera',verified['files']['frames']['path'])))if verify_raw else None
                    paths.append(_safe(root/'camera',verified['files']['rgb']['path']))
                    manifests.append(verified);segment_names.append(segment)
                    active_segment=segment;frame_index=0
                if verify_raw and (ref['frame']!=frame_index or frame_index>=len(metadata_rows) or meta!=metadata_rows[frame_index]):
                    raise ValueError('capture replay metadata or frame order differs')
                clock=dict(environment=meta['environment'],clock_domain=meta.get('clock_domain'),source_clock_id=meta.get('source_clock_id'))
                stamp=(meta['source_time_ns'],None)if meta.get('schema')=='workbench.ros-image.v1'else sample_stamp(meta,clock)
                if verify_raw and (meta.get('capture_instance_id')!=descriptor['source_instance_id']
                        or (clock['source_clock_id'] or clock['clock_domain'])!=descriptor['source_clock_id']
                        or row['source_time']!=stamp[0]/1e9):
                    raise ValueError('capture replay source identity or timestamp differs')
                if verify_raw:previous=assert_progress(previous,stamp)
                frame_index+=1
            selected=list(range(len(rows)));effective=None;origin_ns=0
            if activity is not None:
                clock_id=descriptor['source_clock_id']
                effective=[activity.offset_ns(clock_id,row['payload']['metadata']['ros_reference']['source_time_ns'])for row in rows]
                selected=[index for index,stamp in enumerate(effective)if stamp is not None]
                if not selected:return None
                origin_ns=effective[selected[0]]
                offsets=[round((effective[index]-origin_ns)/1000)for index in selected]
                kept=[rows[index]for index in selected]
            else:
                kept=rows;offsets=[round((row['source_time']-rows[0]['source_time'])*1e6)for row in rows]
            stamps=[row['source_time']for row in kept]
            periods=([b-a for a,b in zip(offsets,offsets[1:])]if activity is not None
                     else[(b-a)*1e6 for a,b in zip(stamps,stamps[1:])])
            tail=max(1,round(statistics.median(periods)if periods else 1e6/(descriptor.get('nominal_hz')or 30)))
            if activity is not None:tail=min(tail,max(1,round((activity.duration_ns-origin_ns)/1000)-offsets[-1]))
            concat=temporary/(identity+'.concat');filters=temporary/(identity+'.filter')
            concat.write_text('ffconcat version 1.0\n'+''.join("file '"+str(path).replace("'","'\\''")+"'\n" for path in paths))
            select=''
            if len(selected)!=len(rows):
                spans=[];start=end=selected[0]
                for index in selected[1:]:
                    if index==end+1:end=index
                    else:spans.append((start,end));start=end=index
                spans.append((start,end))
                select="select='"+'+'.join(f'between(n,{first},{last})'for first,last in spans)+"',"
            filters.write_text(select+"settb=1/1000000,setpts='"+_setpts(offsets)+"'\n")
            destination=output/(identity+'.mp4')
            encoding=dict(mode='post_stop_encode',reused_segments=0,rebuilt_segments=len(paths))
            if any(path.with_suffix('.browser.json').exists()for path in paths):
                groups={name:[]for name in segment_names}
                for row in kept:
                    ref=row['payload']['frame_ref'];groups[ref['segment']].append(ref['frame'])
                with tempfile.TemporaryDirectory(dir=root/'camera',prefix='replay-')as stage:
                    stage=Path(stage)
                    parts,counts=_browser_parts(root/'camera',manifests,list(groups.values()),stage,run,ffmpeg)
                    _remux_browser(parts,destination,offsets,tail,stage,run,ffmpeg)
                encoding=dict(mode='streaming_segments',**counts)
            else:
                # Existing captures have no live browser segments.
                run([ffmpeg,'-nostdin','-v','error','-n','-threads','1','-filter_threads','1','-f','concat','-safe','0',
                     '-r',f'1000000/{tail}','-i',str(concat),'-filter_script:v',str(filters),'-frames:v',str(len(kept)),'-an','-c:v','libx264','-threads','1','-preset','fast',
                     '-crf','24','-pix_fmt','yuv420p','-bf','0',
                     '-vsync','vfr','-enc_time_base','1:1000000','-video_track_timescale','1000000','-movflags','+faststart',
                     str(destination)],check=True,timeout=300)
            # These H.264 streams have no B frames, so packet PTS cover the
            # display timeline without decoding the completed video again.
            probe=_probe_video(destination,run,offsets,offsets[-1]+tail,decode=verify_raw)
            relative=str(destination.relative_to(root))
            pin=dict(size_bytes=destination.stat().st_size,sha256=sha256(destination),probe=probe)
            timing=dict(path=relative,frame_count=len(kept),first_source_time=stamps[0],
                frame_offsets_s=[value/1e6 for value in offsets],duration_s=(offsets[-1]+tail)/1e6,encoding=encoding)
            if activity is not None:timing.update(effective_origin_ns=origin_ns,source_clock_id=descriptor['source_clock_id'])
            return identity,relative,pin,timing
        with ThreadPoolExecutor(max_workers=_EXPORT_WORKERS,thread_name_prefix='capture-export')as workers:
            for result in workers.map(export_camera,manifest['streams'].items()):
                if result is not None:
                    identity,relative,pin,timing=result
                    files[relative]=pin;streams[identity]=timing
    return dict(schema='workbench.capture-playback.v1',files=files,streams=streams,
                timing=('compact MODEL/TEACH intervals; original source timestamps retained in capture indexes'
                        if activity is not None else'original source frame intervals; microsecond container time base'))
