"""Physical raw evidence and derived replay. No SDK, acquisition, or motion calls.

The caller feeds every camera pair from its capture loop (polling latest snapshots
cannot establish complete raw capture). Commands and measured feedback remain
separate. Sealing a command timeline never declares task or grasp success.
"""
from copy import deepcopy
import io
import json
import math
import os
import uuid
from pathlib import Path
import statistics
import struct
import subprocess
import tempfile
import time
import zipfile
import zlib

from contract import validate_physical14, validate_physical_feedback
from frozen_executor import FrozenExecutor
from integration import _digest
from playback import sha256
from real_contract import validate_real_plan, namespace, validate_namespace
from real_rgbd import pair_metadata
from robot_workbench.sample_clock import clock_spec, clock_for_source, sample_stamp, assert_progress, bracket_samples, owner_identity

CAMERAS = ('base_0','left_wrist_0','right_wrist_0')
SIDES = dict(zip(CAMERAS,('head','left','right')))


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def feedback_clock_fields(raw, supplied):
    result={}
    for key in ('clock_domain','source_clock_id','source_time_ns','physics_step','owner_pid','command_epoch','runtime_nonce'):
        value=supplied.get(key)
        if value is None:value=(raw or {}).get(key)
        elif raw is not None and key in raw and raw[key]!=value:
            raise ValueError('feedback source clock differs from raw sample')
        result[key]=value
    return result


def png_shape(image, *, depth=False):
    if len(image)<33 or image[:8] != b'\x89PNG\r\n\x1a\n' or image[12:16] != b'IHDR':
        raise ValueError('PNG payload required')
    width,height,bits,kind,compression,filtering,interlace = struct.unpack('!IIBBBBB',image[16:29])
    channels = {0:1,2:3,6:4}.get(kind)
    if (not channels or bits != (16 if depth else 8) or depth and kind != 0
            or min(width,height)<=0 or compression or filtering or interlace):
        raise ValueError('noninterlaced RGB8 / grayscale depth16 required')
    stride = width*channels*(bits//8)+1
    expected = stride*height
    if expected>16*1024*1024: raise ValueError('PNG dimensions exceed recording limit')
    offset,compressed,ended = 8,bytearray(),False
    while offset<len(image):
        if offset+12>len(image): raise ValueError('truncated PNG chunk')
        length = struct.unpack('!I',image[offset:offset+4])[0]
        end = offset+12+length
        if end>len(image): raise ValueError('truncated PNG payload')
        kind_bytes = image[offset+4:offset+8]
        data = image[offset+8:offset+8+length]
        if zlib.crc32(kind_bytes+data) != struct.unpack('!I',image[end-4:end])[0]:
            raise ValueError('PNG checksum differs')
        if kind_bytes==b'IDAT': compressed.extend(data)
        if kind_bytes==b'IEND':
            ended=True
            if end != len(image): raise ValueError('PNG trailing data')
        offset=end
    decoder=zlib.decompressobj()
    decoded=decoder.decompress(bytes(compressed),expected+1)
    if not ended or not decoder.eof or len(decoded)!=expected or any(decoded[i]>4 for i in range(0,len(decoded),stride)):
        raise ValueError('PNG image data incomplete')
    return width,height


def bundle(blob, serial, *, environment='physical', clock=None):
    namespace(environment)
    if not isinstance(blob,bytes) or len(blob) > 16*1024*1024:
        raise ValueError('bounded snapshot ZIP bytes required')
    with zipfile.ZipFile(io.BytesIO(blob)) as archive:
        if set(archive.namelist()) != {'metadata.json','color.png','depth.png'} or len(archive.infolist()) != 3:
            raise ValueError('atomic RGB-D bundle requires exactly metadata/color/depth')
        if sum(row.file_size for row in archive.infolist()) > 16*1024*1024:
            raise ValueError('RGB-D bundle decompressed size exceeds limit')
        meta = json.loads(archive.read('metadata.json'))
        color,depth = archive.read('color.png'),archive.read('depth.png')
    validate_rgbd_metadata(meta,serial,environment=environment,clock=clock)
    if png_shape(color) != png_shape(depth,depth=True):
        raise ValueError('RGB and aligned depth dimensions differ')
    return meta,color,depth


def validate_rgbd_metadata(meta,serial,*,environment,clock):
    """Shared provenance checks; a spool watermark never qualifies PNG payloads."""
    if meta.get('data_valid') is False:
        raise ValueError('camera data invalid: '+str(meta.get('recording_error') or 'producer rejected this frame'))
    if environment=='fake':
        validate_namespace(meta,environment)
        owner_identity(meta,require_available=True)
        if meta.get('timestamp_kind')!='simulation_step':
            raise ValueError('fake RGB-D requires simulation_step timestamps')
        clock=clock or clock_spec('fake',clock_domain=meta.get('clock_domain'),source_clock_id=meta.get('source_clock_id'))
        sample_stamp(meta,clock)
        attached=meta.get('frame_readback')
        if attached is not None and (not isinstance(attached,dict) or any(attached.get(key)!=meta.get(key)
                for key in ('clock_domain','source_clock_id','source_time_ns','physics_step'))):
            raise ValueError('RGB-D frame readback belongs to another source step')
    elif (meta.get('environment','physical')!='physical' or meta.get('simulation_only') is True
          or meta.get('evidence_source','physical_rgbd')!='physical_rgbd'):
        raise ValueError('fake camera frames cannot enter a physical recording')
    if (meta.get('serial') != serial or meta.get('rgbd_paired') is not True
            or meta.get('depth_aligned_to') != 'color' or meta.get('depth_encoding') != 'uint16_png'):
        raise ValueError('camera identity or aligned RGB-D provenance differs')
    pair_metadata(serial=serial,camera_name=meta['camera_name'],color_number=meta['frame_number'],
                  depth_number=meta['depth_frame_number'],color_ms=meta['device_timestamp_ms'],
                  depth_ms=meta['depth_device_timestamp_ms'],color_domain=meta['timestamp_domain'],
                  depth_domain=meta['timestamp_domain'],host_receive_ns=int(meta['host_receive_ns']),
                  depth_scale_m=meta['depth_scale_m'],intrinsics=meta['intrinsics'],fps=meta['capture_fps'])


class RealRecorder:
    def __init__(self, root, source, plan, binding, *, clock_ns=time.time_ns, execution_timeline=None, environment='physical', clock_domain=None, source_clock_id=None, execution_owner=None):
        self.namespace=namespace(environment)
        self.environment=environment
        self.execution_owner=owner_identity(execution_owner) if execution_owner is not None else None
        self.clock=clock_for_source(source,environment)
        if (clock_domain is not None or source_clock_id is not None) and clock_spec(environment,
                clock_domain=clock_domain,source_clock_id=source_clock_id)!=self.clock:
            raise ValueError('recorder clock differs from sealed source')
        validate_real_plan(plan,source,binding,environment=environment)
        self.frozen = FrozenExecutor(plan,source,sink=lambda _:None,gate=lambda:True,stop=lambda _:None,
                                    source_validator=lambda p,s:validate_real_plan(p,s,binding,environment=environment),
                                    execution_policy='observation' if execution_timeline else 'strict')
        if execution_timeline is not None:
            from execution_timeline import validate
            validate(execution_timeline,plan)
            self.frozen.configure_runtime(execution_timeline)
        self.root = Path(root).resolve()
        self.root.mkdir(parents=True,exist_ok=False)
        self.source,self.plan = deepcopy(source),deepcopy(plan)
        self.clock_ns = clock_ns
        self.created_ns = clock_ns()
        self.commands,self.feedback = [],[]
        self.frames = {camera:[] for camera in CAMERAS}
        self.closed = False
        self.write_json('source.json',source)
        self.write_json('plan.json',plan)
        self.write_json('binding.json',binding)
        if execution_timeline is not None:self.write_json('execution-timeline.json',execution_timeline)

    def write_json(self,name,value):
        (self.root/name).write_text(json.dumps(value,allow_nan=False,indent=2)+'\n')

    def append(self,name,row):
        if self.closed: raise ValueError('recording sealed')
        with (self.root/name).open('a') as stream:
            stream.write(json.dumps(row,allow_nan=False)+'\n')

    def _stamp(self,row,kind):
        wall_key={'command':'send_wall_ns','feedback':'sample_wall_ns','rgbd':'host_receive_ns'}[kind]
        value=row.get(wall_key)
        if kind=='rgbd' and isinstance(value,str) and value.isdigit():row={**row,wall_key:int(value)}
        return sample_stamp(row,self.clock,wall_key=wall_key,
            source_key='send_source_time_ns' if kind=='command' else 'source_time_ns',require_step=kind!='command')

    def _check_owner(self,row,*,frame=False):
        identity=owner_identity(row,require_available=frame)
        if self.execution_owner is None and not frame:
            self.execution_owner=identity
            if any(owner_identity(value,require_available=True)!=identity for rows in self.frames.values() for value in rows):
                raise ValueError('pre-roll frame Owner differs from the execution Owner')
        if self.execution_owner is not None and identity!=self.execution_owner:
            raise ValueError('sample Owner instance/epoch differs from this execution')

    def record_command(self,tick,target,send_monotonic_s,send_wall_ns,*,clock_domain=None,source_clock_id=None,send_source_time_ns=None,physics_step=None,feedback_at_send=None,owner_pid=None,command_epoch=None,runtime_nonce=None):
        if self.environment=='physical' and any(value is not None for value in
                (clock_domain,source_clock_id,send_source_time_ns,physics_step,feedback_at_send,owner_pid,command_epoch,runtime_nonce)):
            raise ValueError('simulation command-clock fields cannot enter physical recording')
        if (type(tick) is not int or tick != len(self.commands) or tick >= len(self.frozen.ticks)
                or target != self.frozen.ticks[tick]['target'] or not finite(send_monotonic_s)
                or type(send_wall_ns) is not int or send_wall_ns <= 0
                or self.commands and (send_monotonic_s <= self.commands[-1]['send_monotonic_s']
                                       or send_wall_ns <= self.commands[-1]['send_wall_ns'])):
            raise ValueError('ordered unchanged frozen command and advancing timestamps required')
        row = dict(tick=tick,target=deepcopy(target),send_monotonic_s=send_monotonic_s,send_wall_ns=send_wall_ns)
        if self.environment=='fake':
            row.update(clock_domain=clock_domain,source_clock_id=source_clock_id,send_source_time_ns=send_source_time_ns,physics_step=physics_step)
            row.update(owner_pid=owner_pid,command_epoch=command_epoch,runtime_nonce=runtime_nonce)
            self._check_owner(row)
            stamp=self._stamp(row,'command')
            if abs(round(send_monotonic_s*1e9)-stamp[0])>1:
                raise ValueError('command control time differs from source time')
            if physics_step is not None:
                raise ValueError('command submission is not a completed physics sample; use feedback_at_send')
            observed=sample_stamp(feedback_at_send,self.clock,wall_key='sample_wall_ns')
            if not 0<=stamp[0]-observed[0]<=100_000_000 or feedback_at_send['sample_wall_ns']>send_wall_ns:
                raise ValueError('command requires its preceding fresh feedback sample')
            row['feedback_at_send']=deepcopy(feedback_at_send)
            assert_progress(self._stamp(self.commands[-1],'command') if self.commands else None,stamp)
        self.append('commands.jsonl',row)
        self.commands.append(row)

    def record_feedback(self,tick,state,sample_monotonic_s,sample_wall_ns,raw=None,*,clock_domain=None,source_clock_id=None,source_time_ns=None,physics_step=None,owner_pid=None,command_epoch=None,runtime_nonce=None):
        if self.environment=='physical' and any(value is not None for value in
                (clock_domain,source_clock_id,source_time_ns,physics_step,owner_pid,command_epoch,runtime_nonce)):
            raise ValueError('simulation feedback-clock fields cannot enter physical recording')
        validate_physical_feedback(state,self.source['calibration_profile'],enforce_range=False)
        if (type(tick) is not int or not 0 <= tick < len(self.frozen.ticks) or not finite(sample_monotonic_s)
                or type(sample_wall_ns) is not int or sample_wall_ns <= 0
                or self.feedback and (sample_monotonic_s <= self.feedback[-1]['sample_monotonic_s']
                                       or sample_wall_ns <= self.feedback[-1]['sample_wall_ns'])):
            raise ValueError('measured feedback and advancing sample clocks required')
        row = dict(tick=tick,state=deepcopy(state),sample_monotonic_s=sample_monotonic_s,sample_wall_ns=sample_wall_ns,raw=raw)
        if self.environment=='fake':
            supplied=dict(clock_domain=clock_domain,source_clock_id=source_clock_id,source_time_ns=source_time_ns,physics_step=physics_step,
                owner_pid=owner_pid,command_epoch=command_epoch,runtime_nonce=runtime_nonce)
            row.update(feedback_clock_fields(raw,supplied))
            self._check_owner(row)
            stamp=self._stamp(row,'feedback')
            if abs(round(sample_monotonic_s*1e9)-stamp[0])>1:
                raise ValueError('feedback control time differs from source time')
            assert_progress(self._stamp(self.feedback[-1],'feedback') if self.feedback else None,stamp)
        self.append('feedback.jsonl',row)
        self.feedback.append(row)

    def record_rgbd(self,camera,blob):
        if self.closed or camera not in CAMERAS: raise ValueError('open recording and known camera required')
        meta,_,_ = bundle(blob,self.source['camera_serials'][SIDES[camera]],environment=self.environment,clock=self.clock)
        if self.environment=='fake':self._check_owner(meta,frame=True)
        rows = self.frames[camera]
        if rows and meta['pair_id'] == rows[-1]['pair_id']: return False
        assert_progress(self._stamp(rows[-1],'rgbd') if rows else None,self._stamp(meta,'rgbd'))
        if rows and (int(meta['host_receive_ns']) <= int(rows[-1]['host_receive_ns'])
                     or meta['frame_number'] <= rows[-1]['frame_number']
                     or meta['depth_frame_number'] <= rows[-1]['depth_frame_number']):
            raise ValueError('camera pair timestamps/frame counters regressed')
        directory = self.root/'camera'/camera
        directory.mkdir(parents=True,exist_ok=True)
        path = directory/f'{len(rows):08d}.zip'
        path.write_bytes(blob)
        row = dict(meta,path=str(path.relative_to(self.root)))
        self.append(f'{camera}-pairs.jsonl',row)
        rows.append(row)
        return True

    def seal(self,status,reason=None):
        if self.closed or status not in ('completed','failed','stopped','recording_failed'):
            raise ValueError('one explicit terminal outcome required')
        problems = []
        pairings=[]
        if len(self.commands) != len(self.frozen.ticks): problems.append('not every execution command recorded')
        if not self.feedback or not self.commands:
            problems.append('command or measured feedback stream missing')
        else:
            commands=[self._stamp(row,'command') for row in self.commands]
            feedback=[self._stamp(row,'feedback') for row in self.feedback]
            endpoint = (commands[0][0]/1e9 if self.environment=='fake' else self.commands[0]['send_monotonic_s'])+self.frozen.duration_s
            last_feedback = feedback[-1][0]/1e9 if self.environment=='fake' else self.feedback[-1]['sample_monotonic_s']
            if last_feedback+1e-9 < endpoint:
                problems.append('feedback does not cover complete command duration')
            feedback_gap = (any(b[0]-a[0]>100_000_000 for a,b in zip(feedback,feedback[1:])) if self.environment=='fake'
                else any(b['sample_monotonic_s']-a['sample_monotonic_s']>.1 for a,b in zip(self.feedback,self.feedback[1:])))
            if feedback_gap:
                problems.append('measured feedback gaps exceed 0.1 s')
            for camera,rows in self.frames.items():
                if not rows:
                    problems.append(camera+': RGB-D stream missing'); continue
                frames=[self._stamp(row,'rgbd') for row in rows]
                if frames[0][0] > commands[0][0] or frames[-1][0] < feedback[-1][0]:
                    problems.append(camera+': RGB-D does not bracket execution')
                if any(b['frame_number'] != a['frame_number']+1 or b['depth_frame_number'] != a['depth_frame_number']+1
                       for a,b in zip(rows,rows[1:])):
                    problems.append(camera+': dropped RGB-D frames')
                if any(b[0]-a[0]>100_000_000 for a,b in zip(frames,frames[1:])):
                    problems.append(camera+': RGB-D capture gap exceeds 0.1 s')
                if self.environment=='fake':
                    for frame,row in zip(frames,rows):
                        if not commands[0][0]<=frame[0]<=feedback[-1][0]:continue
                        try:left,right=bracket_samples(feedback,frame)
                        except ValueError as error:
                            problems.append(camera+': '+str(error));continue
                        pairings.append(dict(camera=camera,frame_number=row['frame_number'],source_time_ns=frame[0],physics_step=frame[1],
                            feedback_left_index=left,feedback_right_index=right,
                            feedback_left_source_time_ns=feedback[left][0],feedback_right_source_time_ns=feedback[right][0],
                            feedback_left_physics_step=feedback[left][1],feedback_right_physics_step=feedback[right][1],
                            association='exact_feedback_sample' if left==right else 'feedback_bracket',interpolated=left!=right))
        if status != 'completed': problems.append('controller outcome: '+status)
        if self.environment=='fake':self.write_json('frame-feedback-pairs.json',dict(clock=self.clock,pairs=pairings))
        files = {str(path.relative_to(self.root)):dict(size_bytes=path.stat().st_size,sha256=sha256(path))
                 for path in self.root.rglob('*') if path.is_file()}
        manifest = dict(schema_version=self.namespace['raw_schema'],evidence_source=self.namespace['evidence'],
                        source_run_id=self.source['run_id'],plan_run_id=self.plan['run_id'],timeline_digest=self.plan['timeline_digest'],
                        robot_id=self.source['robot_id'],camera_serials=self.source['camera_serials'],
                        created_wall_ns=self.created_ns,sealed_wall_ns=self.clock_ns(),
                        command_count=len(self.commands),feedback_count=len(self.feedback),
                        camera_frames={camera:len(rows) for camera,rows in self.frames.items()},
                        capture_status='complete' if not problems else 'incomplete',
                        validation=dict(status='passed' if not problems else 'failed',problems=problems,scope='raw completeness and file integrity; not tracking or task success'),
                        controller_outcome='unverified' if status=='recording_failed' else status,
                        controller_reason=None if status=='recording_failed' else reason,
                        recording_error=reason if status=='recording_failed' else None,
                        task_outcome='unverified',files=files)
        if self.environment=='fake':manifest.update(environment='fake',simulation_only=True,clock=self.clock,source_clock='simulation',
            source_clock_id=self.clock['source_clock_id'],execution_owner=self.execution_owner,
            frame_feedback_association='exact samples or explicit same-clock brackets')
        if self.frozen.runtime_timeline:
            manifest.update(execution_timeline_digest=self.frozen.runtime_timeline['timeline_digest'],
                execution_hz=self.frozen.runtime_timeline['control_hz'],
                execution_duration_s=self.frozen.duration_s)
        self.write_json('manifest.json',manifest)
        self.closed = True
        return manifest


def verify_raw(root, *, environment='physical'):
    root = Path(root).resolve()
    if (root/'manifest.json').is_symlink(): raise ValueError('raw manifest symlink forbidden')
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest.get('schema_version') != namespace(environment)['raw_schema']:
        raise ValueError('physical raw manifest required')
    validate_namespace(manifest, environment)
    if not {'source.json','plan.json','binding.json'} <= set(manifest.get('files',{})):
        raise ValueError('raw identity files must be sealed')
    for name,pin in manifest['files'].items():
        path = root/name
        if path.is_symlink() or not path.resolve().is_relative_to(root) or path.stat().st_size != pin['size_bytes'] or sha256(path) != pin['sha256']:
            raise ValueError('sealed raw file changed: '+name)
    source,plan,binding = [json.loads((root/name).read_text()) for name in ('source.json','plan.json','binding.json')]
    validate_real_plan(plan,source,binding,environment=environment)
    if environment=='fake' and (manifest.get('clock')!=clock_for_source(source,environment)
            or manifest.get('source_clock')!='simulation' or manifest.get('source_clock_id')!=source['clock']['source_clock_id']):
        raise ValueError('raw clock differs from sealed source')
    if manifest.get('source_run_id') != source['run_id'] or manifest.get('plan_run_id') != plan['run_id'] or manifest.get('timeline_digest') != plan['timeline_digest']:
        raise ValueError('raw identity differs')
    if manifest.get('execution_timeline_digest'):
        from execution_timeline import validate
        if 'execution-timeline.json' not in manifest['files']:raise ValueError('execution timeline not sealed')
        timeline=validate(json.loads((root/'execution-timeline.json').read_text()),plan)
        if (timeline['timeline_digest']!=manifest['execution_timeline_digest']
                or len(timeline['commands'])!=manifest['command_count'] and manifest['capture_status']=='complete'):
            raise ValueError('raw execution timeline differs')
    return manifest,source,plan


def export_playback(raw_root,target_manifest_path,output,*,runner=subprocess.run,environment='physical',scratch_root=None):
    """Derived videos may exist for a failed run; raw/task outcome stays explicit."""
    raw_root = Path(raw_root).resolve()
    if scratch_root is not None:
        scratch_root=Path(scratch_root)
        if (environment!='fake' or not scratch_root.is_absolute() or '..' in scratch_root.parts
                or scratch_root.is_symlink() or not scratch_root.is_dir() or scratch_root.resolve().is_relative_to(raw_root)):
            raise ValueError('fake playback scratch must be an existing explicit directory outside sealed raw')
        scratch_root=scratch_root.resolve()
    target_manifest_path = Path(target_manifest_path).resolve() if target_manifest_path is not None else None
    spec=namespace(environment)
    raw,source,plan = verify_raw(raw_root,environment=environment)
    target = json.loads(target_manifest_path.read_text()) if target_manifest_path is not None else None
    if target is not None and target.get('execution_timeline_digest') != raw.get('execution_timeline_digest'):
        raise ValueError('physical and simulation execution commands differ')
    if target is not None and (target.get('schema_version') != spec['target_playback_schema'] or target.get('evidence_source') != 'simulation'
            or target.get('source_run_id') != source['run_id'] or target.get('plan_run_id') != plan['run_id']
            or target.get('timeline_digest') != plan['timeline_digest']
            or set(target.get('files',{})) != {f'target-{c}.mp4' for c in CAMERAS}):
        raise ValueError('target replay must be bound to this physical source and plan')
    target_paths = {}
    for name,pin in (target['files'] if target is not None else {}).items():
        path = target_manifest_path.parent/name
        if path.is_symlink() or path.stat().st_size != pin['size_bytes'] or sha256(path) != pin['sha256']:
            raise ValueError('target replay integrity differs')
        target_paths[name] = path
    frames = {c:[json.loads(line) for line in (raw_root/f'{c}-pairs.jsonl').read_text().splitlines()] for c in CAMERAS}
    if any(not rows for rows in frames.values()): raise ValueError('all three physical streams needed for replay')
    output = Path(output).resolve()
    if output.is_relative_to(raw_root): raise ValueError('derived playback must stay outside sealed raw directory')
    output.mkdir(parents=True,exist_ok=False)
    files = {}
    timing = {}
    clock=clock_for_source(source,environment)
    first_command=json.loads((raw_root/'commands.jsonl').read_text().splitlines()[0])
    command_start = sample_stamp(first_command,clock,wall_key='send_wall_ns',source_key='send_source_time_ns',require_step=False)[0]
    with tempfile.TemporaryDirectory(dir=scratch_root if scratch_root is not None else output) as temporary:
        temporary = Path(temporary)
        for camera,rows in frames.items():
            stamps = [sample_stamp({**row,'host_receive_ns':int(row['host_receive_ns'])},clock)[0] for row in rows]
            periods = [(b-a)/1e9 for a,b in zip(stamps,stamps[1:])]
            last = statistics.median(periods) if periods else 1/rows[0]['capture_fps']
            entries = []
            for index,(row,duration) in enumerate(zip(rows,periods+[last])):
                path = temporary/f'{camera}-{index:08d}.png'
                _,color,_ = bundle((raw_root/row['path']).read_bytes(),source['camera_serials'][SIDES[camera]],environment=environment,clock=clock)
                path.write_bytes(color)
                escaped = str(path).replace("'", "'\\''")
                entries.extend([f"file '{escaped}'",'option framerate 1000',f'duration {duration:.9f}'])
            concat = temporary/f'{camera}.txt'; concat.write_text('\n'.join(entries)+'\n')
            destination = output/f'source-{camera}.mp4'
            # Concat does not carry the final duration through encoding. Disable
            # frame reordering so the final packet is the final displayed frame,
            # then set its duration without adding a frame or changing captured PTS.
            tail_ns = max(1, round(last * 1e9))
            runner(['ffmpeg','-nostdin','-v','error','-n','-threads','2','-f','concat','-safe','0','-i',str(concat),
                    '-an','-c:v','libx264','-threads','2','-preset','fast','-crf','24','-pix_fmt','yuv420p',
                    '-bf','0','-bsf:v',f'setts=duration=if(eq(N\\,{len(rows)-1})\\,{tail_ns}/1000000000/TB\\,DURATION)',
                    '-fps_mode','vfr','-enc_time_base','1:1000','-video_track_timescale','1000000','-movflags','+faststart',str(destination)],check=True,timeout=300)
            timing[camera] = dict(clock='simulation' if environment=='fake' else 'host_receive',frame_offsets_s=[(stamp-command_start)/1e9 for stamp in stamps],
                                 frame_count=len(rows),depth_in_raw=True,last_frame_duration_s=last,
                                 duration_s=(stamps[-1]-stamps[0])/1e9+last)
            if environment=='fake':timing[camera].update(source_clock_id=clock['source_clock_id'],host_receive_ns=[int(row['host_receive_ns']) for row in rows])
        import shutil
        for name,path in target_paths.items(): shutil.copyfile(path,output/name)
    for path in sorted(output.glob('*.mp4')):
        probe = json.loads(runner(['ffprobe','-v','error','-count_frames','-select_streams','v:0','-show_streams','-of','json',str(path)],
                                 check=True,timeout=120,capture_output=True,text=True).stdout)['streams']
        if len(probe)!=1 or probe[0].get('codec_name')!='h264' or probe[0].get('pix_fmt')!='yuv420p':
            raise ValueError('browser H264 yuv420p replay required')
        if path.name.startswith('source-') and int(probe[0]['nb_read_frames']) != timing[path.stem.removeprefix('source-')]['frame_count']:
            raise ValueError('source replay lost or duplicated recorded frames')
        if path.name.startswith('source-'):
            duration = float(probe[0].get('duration', 'nan'))
            expected = timing[path.stem.removeprefix('source-')]['duration_s']
            # Both terminal PTS and packet duration are quantized at the 1 ms encoder time base.
            if not math.isfinite(duration) or abs(duration-expected) > .002:
                raise ValueError('source replay duration differs from recorded timing and final-frame hold')
        files[path.name] = dict(size_bytes=path.stat().st_size,sha256=sha256(path),probe=probe[0])
    manifest = dict(schema_version=spec['playback_schema'],mode=spec['mode'],clock='elapsed',
                    source_run_id=source['run_id'],plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'],
                    provenance={'source':spec['evidence'], **({'target':'simulation'} if target is not None else {})},files=files,source_timing=timing,
                    raw_manifest_sha256=sha256(raw_root/'manifest.json'),target_manifest_sha256=sha256(target_manifest_path) if target_manifest_path is not None else None,capture_status=raw['capture_status'],
                    controller_outcome=raw['controller_outcome'],task_outcome='unverified')
    if environment=='fake':manifest.update(environment='fake',simulation_only=True,source_clock=clock)
    (output/'manifest.json').write_text(json.dumps(manifest,allow_nan=False,indent=2)+'\n')
    return manifest


def _renew_recording_lease(request,token,previous_deadline,new_deadline,*,clock_ns,active):
    """Replace only this live token; retain RawSpool's bounded 600 s window."""
    before=request.read_bytes();lease=json.loads(before);now=clock_ns()
    if (lease.get('run_id')!=token or lease.get('deadline_wall_ns')!=previous_deadline
            or not lease['start_wall_ns']<=now<previous_deadline
            or not previous_deadline<new_deadline<=now+600_000_000_000):
        raise ValueError('recording lease ownership/window changed; expired leases cannot resume')
    temporary=request.with_name(token+'.renew.tmp')
    try:
        with temporary.open('x') as stream:
            json.dump(dict(lease,start_wall_ns=now,deadline_wall_ns=new_deadline,
                admission_start_wall_ns=lease.get('admission_start_wall_ns',lease['start_wall_ns'])),stream)
        if not active():return False
        if request.read_bytes()!=before or clock_ns()>=previous_deadline:
            raise ValueError('recording lease changed or expired before renewal')
        os.replace(temporary,request)
        return True
    finally:
        temporary.unlink(missing_ok=True)


class RecordingSession:
    """Bounded nonblocking owner input; this worker alone writes recording files.

    Native camera directories come from preview-only real_rgbd services.
    An exclusive bounded lease enables full-frame spooling only for this session.
    Tests may omit directories and submit paired bundles through the same queue.
    """
    def __init__(self,root,source,plan,binding,*,camera_directories=None,clock_ns=time.time_ns,
                 capacity=512,on_fault=lambda reason:None,recorder_factory=RealRecorder,execution_timeline=None,environment='physical',clock_domain=None,source_clock_id=None,execution_owner=None,
                 resident_deadline_wall_ns=None,defer_execution=False,execution_instance_id=None,
                 preparation_active=None,preparation_deadline_monotonic=None):
        import queue
        import threading
        self.root=Path(root)
        self.environment=namespace(environment)['environment']
        self.clock=clock_for_source(source,environment)
        if (clock_domain is not None or source_clock_id is not None) and clock_spec(environment,clock_domain=clock_domain,source_clock_id=source_clock_id)!=self.clock:
            raise ValueError('recording session clock differs from source')
        self.execution_owner=execution_owner
        if type(defer_execution) is not bool:raise ValueError('deferred execution must be explicit boolean')
        if defer_execution and (environment!='fake' or execution_timeline is not None
                or not isinstance(execution_instance_id,str) or not execution_instance_id
                or not callable(preparation_active) or not finite(preparation_deadline_monotonic)
                or camera_directories is None):
            raise ValueError('deferred recording requires a fake execution instance and bounded active preparation')
        if defer_execution:owner_identity(execution_owner)
        self.defer_execution=defer_execution
        self.execution_instance_id=execution_instance_id
        self.preparation_active=preparation_active
        self.preparation_deadline_monotonic=preparation_deadline_monotonic
        self.prepared,self.execution_bound,self.cancelled=threading.Event(),threading.Event(),threading.Event()
        self.preparation_started_wall_ns=clock_ns()
        self.prepared_wall_ns=self.bound_wall_ns=None
        self.lease_tokens=set()
        self.events=queue.Queue(maxsize=capacity)
        self.ready,self.done=threading.Event(),threading.Event()
        self.lock=threading.Lock()
        self.error,self.manifest,self.terminal=None,None,None
        self.controller_result=None
        self.on_fault=on_fault
        self.clock_ns=clock_ns
        self.camera_directories=camera_directories
        self.resident_deadline_wall_ns=resident_deadline_wall_ns
        self.recorder=None
        self.execution_timeline=execution_timeline
        self.feedback_progress=None
        self.spool_heads={}
        self.pending_reference={}
        self.checked_reference=None
        self.writer_stage='starting'
        self.last_event_write=None
        self.last_event_batch=None
        self.last_nonempty_event_batch=None
        self.diagnostics=None
        self.diagnostics_error=self.seal_error=None
        self.thread=threading.Thread(target=self._run,args=(source,plan,binding,recorder_factory),daemon=True)
        self.thread.start()

    def fail(self,reason):
        self.cancelled.set()
        with self.lock:
            first=self.error is None
            self.error=self.error or str(reason)
            self.terminal=('failed',self.error,time.monotonic())
        if first:self.on_fault(self.error)

    def preparation_current(self):
        return (self.defer_execution and not self.done.is_set() and not self.cancelled.is_set() and self.preparation_active()
            and (self.prepared.is_set() or time.monotonic()<self.preparation_deadline_monotonic))

    def check_preparation(self):
        if not self.preparation_current():raise ValueError('fake recording preparation expired or execution was revoked')

    def lease_active(self,lease):
        """Fake producer admission also fences a cancelled in-flight os.link."""
        return (self.preparation_current() and lease.get('run_id') in self.lease_tokens
            and lease.get('execution_instance_id')==self.execution_instance_id
            and lease.get('execution_owner')==self.execution_owner)

    def bind_execution(self,timeline,*,execution_instance_id,execution_owner):
        # The compiled timeline is private and immutable; validation/file writes
        # remain on the recorder thread, never on the Owner tick.
        with self.lock:
            self.check_preparation()
            if (not self.prepared.is_set() or self.execution_bound.is_set() or self.execution_timeline is not None
                    or execution_instance_id!=self.execution_instance_id or execution_owner!=self.execution_owner):
                raise ValueError('one final timeline for the prepared execution instance required')
            self.execution_timeline=timeline
            self.bound_wall_ns=self.clock_ns()
            self.execution_bound.set()

    def preparation_receipt(self):
        return dict(schema='fake_recording_preparation.v1',execution_instance_id=self.execution_instance_id,
            execution_owner=self.execution_owner,clock=self.clock,started_wall_ns=self.preparation_started_wall_ns,
            prepared_wall_ns=self.prepared_wall_ns,bound_wall_ns=self.bound_wall_ns,
            preparation_deadline_monotonic_s=self.preparation_deadline_monotonic,
            prepared=self.prepared.is_set(),execution_bound=self.execution_bound.is_set(),cancelled=self.cancelled.is_set(),
            error=self.error,lease_tokens=sorted(self.lease_tokens),image_readiness_gate=False)

    def submit(self,kind,**fields):
        import queue
        error=None
        with self.lock:
            if self.terminal or self.error or self.done.is_set():raise ValueError('recording is closing or failed')
            if kind not in ('command','feedback','rgbd'):raise ValueError('unknown recording input')
            try:
                if self.environment=='fake' and kind=='feedback':
                    # Latest actual Owner submission, not writer progress or an
                    # extrapolated simulation clock. This never waits for IO.
                    meta=dict(sample_wall_ns=fields.get('sample_wall_ns'),
                        **feedback_clock_fields(fields.get('raw'),fields))
                    identity=owner_identity(meta)
                    if self.execution_owner is not None and identity!=self.execution_owner:
                        raise ValueError('feedback Owner differs from this execution')
                    stamp=sample_stamp(meta,self.clock,wall_key='sample_wall_ns')
                    if not finite(fields.get('sample_monotonic_s')) or abs(round(fields['sample_monotonic_s']*1e9)-stamp[0])>1:
                        raise ValueError('feedback control time differs from source time')
                    assert_progress(self.feedback_progress[0] if self.feedback_progress else None,stamp)
                    self.feedback_progress=(stamp,self.clock_ns(),time.monotonic())
                self.events.put_nowait((kind,fields,self.clock_ns(),time.monotonic()))
            except queue.Full:error='bounded recording queue overflow'
            except ValueError as exc:error=str(exc)
        if error:
            self.fail(error)
            raise ValueError(self.error)

    def fake_frame_stamp(self,meta,reference,*,check_lag=True):
        if reference is None:return None
        stamp=sample_stamp({**meta,'host_receive_ns':int(meta['host_receive_ns'])},self.clock)
        # A camera may sample between Owner ticks; compare actual steps without
        # pretending a command timestamp or wall elapsed time is new feedback.
        first,last=sorted((stamp,reference[0]))
        assert_progress(first,last,allow_same=True)
        if stamp[0]>reference[0][0]:return None
        if check_lag and reference[0][0]-stamp[0]>500_000_000:
            raise ValueError('native RGB-D source lag exceeds 0.5 seconds')
        return stamp

    def check_fake_head(self,camera,meta,reference):
        stamp=sample_stamp({**meta,'host_receive_ns':int(meta['host_receive_ns'])},self.clock)
        pending=self.pending_reference.get(camera)
        # Complete the previous wait against its fixed frame, even when a newer
        # camera head stays ahead of the reference captured before this IO round.
        if pending and reference and reference[0][0]>=pending[0][0]:
            assert_progress(pending[0],reference[0],allow_same=True)
            self.pending_reference.pop(camera)
        if self.fake_frame_stamp(meta,reference) is None:
            pending=self.pending_reference.setdefault(camera,(stamp,self.clock_ns(),time.monotonic()))
            if self.clock_ns()-pending[1]>=30_000_000_000 or time.monotonic()-pending[2]>=30:
                raise ValueError(camera+': actual Owner reference did not reach frame within 30 wall seconds')
        else:self.pending_reference.pop(camera,None)
        return stamp

    def spool_head(self,camera,directory):
        # Only this writer reclaims ZIPs; the producer atomically adds them.
        # Read metadata, not PNGs, and keep the last committed watermark after
        # consuming it. Historical frames still pass full validation in order.
        latest=max(directory.glob('[0-9]'*9+'.zip'),default=None)
        previous=self.spool_heads.get(camera)
        if latest is not None and (previous is None or int(latest.stem)>previous['index']):
            if latest.stat().st_size>16*1024*1024:raise ValueError('bounded spool ZIP required')
            with zipfile.ZipFile(latest) as archive:
                if archive.getinfo('metadata.json').file_size>256*1024:raise ValueError('bounded RGB-D metadata required')
                meta=json.loads(archive.read('metadata.json'))
            validate_rgbd_metadata(meta,self.recorder.source['camera_serials'][SIDES[camera]],
                environment=self.environment,clock=self.clock)
            self.recorder._check_owner(meta,frame=True)
            if meta.get('recording_run_id')!=directory.name or meta.get('recording_enabled') is not True:
                raise ValueError(camera+': raw recording lease differs')
            if previous:assert_progress(self.recorder._stamp(previous['metadata'],'rgbd'),self.recorder._stamp(meta,'rgbd'))
            previous=dict(index=int(latest.stem),metadata=meta,observed_wall_ns=self.clock_ns(),observed_monotonic_s=time.monotonic())
            self.spool_heads[camera]=previous
        return previous

    def failure_diagnostics(self,cursors):
        def reference(value):
            return (dict(source_time_ns=value[0][0],physics_step=value[0][1],submitted_wall_ns=value[1],
                submitted_monotonic_s=value[2]) if value else None)
        def frame(row):
            return {key:row.get(key) for key in ('source_clock_id','source_time_ns','physics_step','host_receive_ns',
                'frame_number','depth_frame_number','recording_run_id','path')}
        with self.lock:latest_reference=self.feedback_progress
        now=self.clock_ns();mono=time.monotonic()
        with self.events.mutex:
            pending=list(self.events.queue)
        oldest=None
        if pending:
            kind,fields,wall,queued=pending[0]
            raw=fields.get('raw') or {}
            oldest=dict(kind=kind,tick=fields.get('tick'),queued_wall_ns=wall,queued_monotonic_s=queued,
                queue_wait_wall_s=(now-wall)/1e9,queue_wait_monotonic_s=mono-queued,
                source_time_ns=fields.get('send_source_time_ns',fields.get('source_time_ns',raw.get('source_time_ns'))))
        cameras={}
        for camera,(directory,index) in cursors.items():
            rows=self.recorder.frames[camera] if self.recorder else []
            head=self.spool_heads.get(camera)
            waiting=self.pending_reference.get(camera)
            pending_indices=sorted(int(p.stem) for p in directory.glob('[0-9]'*9+'.zip'))
            cameras[camera]=dict(next_archive_index=index,archived_count=len(rows),
                archived_first=frame(rows[0]) if rows else None,archived_tail=frame(rows[-1]) if rows else None,
                checked_spool_index=head['index'] if head else None,checked_spool_head=frame(head['metadata']) if head else None,
                checked_spool_observed_wall_ns=head['observed_wall_ns'] if head else None,
                pending_reference=(dict(source_time_ns=waiting[0][0],physics_step=waiting[0][1],
                    wait_started_wall_ns=waiting[1],wait_started_monotonic_s=waiting[2]) if waiting else None),
                spool_pending_count=len(pending_indices),spool_first_index=pending_indices[0] if pending_indices else None,
                spool_last_index=pending_indices[-1] if pending_indices else None)
        return dict(schema='recording_failure_diagnostics.v1',error=self.error,writer_stage=self.writer_stage,
            captured_wall_ns=now,captured_monotonic_s=mono,checked_reference=reference(self.checked_reference),
            latest_submitted_reference=reference(latest_reference),queue=dict(depth=len(pending),capacity=self.events.maxsize,oldest=oldest),
            last_event_write=self.last_event_write,last_event_batch=self.last_event_batch,
            last_nonempty_event_batch=self.last_nonempty_event_batch,
            command_count=len(self.recorder.commands) if self.recorder else 0,
            feedback_count=len(self.recorder.feedback) if self.recorder else 0,cameras=cameras)

    def save_failure_diagnostics(self,cursors):
        if self.error:
            try:
                self.diagnostics=self.failure_diagnostics(cursors)
                self.recorder.write_json('recording-diagnostics.json',self.diagnostics)
            except Exception as error:
                self.diagnostics_error=f'{type(error).__name__}: {error}'

    def _write_event(self,event):
        kind,fields,queued_wall,queued_mono=event
        started=time.monotonic();self.writer_stage='event:'+kind
        self.last_event_write=dict(kind=kind,tick=fields.get('tick'),queued_wall_ns=queued_wall,
            write_started_wall_ns=self.clock_ns(),queue_wait_monotonic_s=started-queued_mono,status='failed')
        try:
            getattr(self.recorder,'record_'+kind)(**fields)
            self.events.task_done()
            self.last_event_write['status']='written'
        finally:
            finished=time.monotonic()
            wall=self.clock_ns()
            self.last_event_write.update(finished_wall_ns=wall,
                written_wall_ns=wall if self.last_event_write['status']=='written' else None,processing_wall_s=finished-started,
                enqueue_to_write_monotonic_s=finished-queued_mono)

    def _drain_events(self):
        import queue
        started=time.monotonic();self.writer_stage='event_batch'
        self.last_event_batch=dict(started_wall_ns=self.clock_ns(),count=0)
        try:
            # A count bound gives cameras a turn without turning slow metadata
            # IO into a one-event quota. Every event is still validated/written.
            for index in range(64):
                try:event=self.events.get(timeout=.01 if index==0 else 0)
                except queue.Empty:break
                self._write_event(event)
                self.last_event_batch['count']+=1
        finally:
            self.last_event_batch.update(finished_wall_ns=self.clock_ns(),duration_wall_s=time.monotonic()-started)
            if self.last_event_batch['count']:self.last_nonempty_event_batch=dict(self.last_event_batch)

    def finish(self,status,reason=None):
        if status not in ('completed','failed','stopped'):raise ValueError('terminal status required')
        if status!='completed':self.cancelled.set()
        with self.lock:
            self.controller_result=dict(status=status,reason=reason)
            if self.terminal is None:self.terminal=(status,reason,time.monotonic())

    def snapshot(self):
        return dict(ready=self.ready.is_set(),done=self.done.is_set(),error=self.error,
                    controller_result=self.controller_result,
                    directory=str(self.root),manifest_path=str(self.root/'manifest.json') if self.manifest else None,
                    diagnostics=deepcopy(self.diagnostics),
                    diagnostics_error=self.diagnostics_error,seal_error=self.seal_error,
                    preparation=self.preparation_receipt() if self.defer_execution else None,
                    manifest={k:v for k,v in self.manifest.items() if k!='files'} if self.manifest else None)

    def _run(self,source,plan,binding,factory):
        cursors={}
        camera_progress={}
        leases=[]
        lease_deadline_wall_ns=None
        recording_deadline_wall_ns=None;recording_deadline_monotonic=None;renew_at_wall_ns=None
        tail_wait_started=None;tail_wait_elapsed=0.
        def create_recorder():
            return factory(self.root,source,plan,binding,clock_ns=self.clock_ns,
                **({'execution_timeline':self.execution_timeline} if self.execution_timeline is not None else {}),
                **({'environment':self.environment,'clock_domain':self.clock['clock_domain'],'source_clock_id':self.clock['source_clock_id'],
                    'execution_owner':self.execution_owner} if self.environment=='fake' else {}))
        try:
            if not self.defer_execution:self.recorder=create_recorder()
            if self.camera_directories is not None:
                if set(self.camera_directories)!=set(CAMERAS):raise ValueError('three native RGB-D spool directories required')
                if self.environment=='fake':
                    if type(self.resident_deadline_wall_ns) is not int:
                        raise ValueError('fake recording requires the fixed resident wall deadline')
                    now=self.clock_ns();recording_deadline_wall_ns=self.resident_deadline_wall_ns-45_000_000_000
                    if now>=recording_deadline_wall_ns:raise ValueError('resident recording budget exhausted; 45 s cleanup reserve required')
                    recording_deadline_monotonic=time.monotonic()+(recording_deadline_wall_ns-now)/1e9
                    lease_deadline_wall_ns=min(recording_deadline_wall_ns,now+600_000_000_000)
                    renew_at_wall_ns=now+(lease_deadline_wall_ns-now)//2
                else:
                    duration=self.recorder.frozen.duration_s+15
                    if not 0 < duration <= 600:raise ValueError('raw recording supports at most 585 seconds of execution')
                for camera,value in self.camera_directories.items():
                    if self.defer_execution:self.check_preparation()
                    directory=Path(value).resolve(strict=True)
                    if json.loads((directory/'recording-service.json').read_text()).get('schema') != 'rgbd_lease.v1':
                        raise ValueError(camera+': on-demand recording service required')
                    start=self.clock_ns()
                    deadline_wall_ns=lease_deadline_wall_ns if self.environment=='fake' else start+round(duration*1e9)
                    lease_deadline_wall_ns=min(lease_deadline_wall_ns or deadline_wall_ns,deadline_wall_ns)
                    token=uuid.uuid4().hex
                    request=directory/'recording-request.json'
                    temporary=directory/(token+'.request.tmp')
                    lease=dict(run_id=token,start_wall_ns=start,deadline_wall_ns=deadline_wall_ns,
                        admission_start_wall_ns=start)
                    if self.defer_execution:
                        lease.update(execution_instance_id=self.execution_instance_id,execution_owner=self.execution_owner)
                        self.lease_tokens.add(token)
                    temporary.write_text(json.dumps(lease))
                    leases.append((request,token))
                    try:
                        if self.defer_execution:self.check_preparation()
                        os.link(temporary,request)  # Atomic exclusive publish; never replace another run.
                        if self.defer_execution:self.check_preparation()
                    finally:temporary.unlink()
                    cursors[camera]=(directory/token,1)
                if self.defer_execution:
                    self.check_preparation()
                    self.prepared_wall_ns=self.clock_ns()
                    self.prepared.set()
                    while not self.execution_bound.wait(.01):
                        self.check_preparation()
                        if self.clock_ns()>=recording_deadline_wall_ns or time.monotonic()>=recording_deadline_monotonic:
                            raise ValueError('recording reached fixed resident wall deadline minus cleanup reserve')
                        if self.clock_ns()>=lease_deadline_wall_ns:
                            raise ValueError('recording lease actual wall deadline exceeded')
                    self.check_preparation()
                    self.recorder=create_recorder()
                    self.recorder.write_json('recording-preparation.json',self.preparation_receipt())
                ready_deadline=time.monotonic()+5
                while True:
                    self.writer_stage='awaiting_first_frames'
                    if recording_deadline_wall_ns is not None and (self.clock_ns()>=recording_deadline_wall_ns
                            or time.monotonic()>=recording_deadline_monotonic):
                        raise ValueError('recording reached fixed resident wall deadline minus cleanup reserve')
                    with self.lock:self.checked_reference=self.feedback_progress
                    first=[];first_referenced=True
                    for camera,(directory,index) in cursors.items():
                        path=directory/f'{index:09d}.zip'
                        if path.is_file():
                            meta,_,_=bundle(path.read_bytes(),source['camera_serials'][SIDES[camera]],environment=self.environment,clock=self.clock)
                            if meta.get('recording_run_id') != directory.name or not meta.get('recording_enabled'):
                                raise ValueError(camera+': raw recording lease differs')
                            first.append(meta)
                            if self.environment=='fake':
                                head=self.spool_head(camera,directory)
                                self.fake_frame_stamp(head['metadata'],self.checked_reference)
                                # The first historical frame need only be covered
                                # by actual feedback; freshness belongs to the head.
                                first_referenced &= self.fake_frame_stamp(meta,self.checked_reference,check_lag=False) is not None
                    if len(first)==3 and first_referenced:break
                    with self.lock:terminal=self.terminal
                    if terminal:
                        status,reason,_=terminal
                        self.save_failure_diagnostics(cursors)
                        self.manifest=self.recorder.seal('recording_failed' if self.error and self.execution_timeline else status,reason)
                        return
                    if time.monotonic()>=ready_deadline:
                        raise ValueError('three native raw streams did not start within 5 seconds'
                            + (' with actual Owner source feedback' if self.environment=='fake' else ''))
                    time.sleep(.01)
                if self.environment!='fake' and any(not 0 <= self.clock_ns()-int(meta['host_receive_ns']) <= 500_000_000 for meta in first):
                    raise ValueError('native raw stream absent or stale')
            self.ready.set()
            while True:
                now=self.clock_ns()
                if recording_deadline_wall_ns is not None and (now>=recording_deadline_wall_ns
                        or time.monotonic()>=recording_deadline_monotonic):
                    raise ValueError('recording reached fixed resident wall deadline minus cleanup reserve')
                if lease_deadline_wall_ns is not None and now>=lease_deadline_wall_ns:
                    raise ValueError('recording lease actual wall deadline exceeded')
                if self.environment=='fake' and cursors and self.terminal is None:
                    with self.lock:feedback_progress=self.feedback_progress
                    if feedback_progress and (now-feedback_progress[1]>=30_000_000_000 or time.monotonic()-feedback_progress[2]>=30):
                        raise ValueError('Owner source feedback stalled for 30 wall seconds')
                if (recording_deadline_wall_ns is not None and lease_deadline_wall_ns<recording_deadline_wall_ns
                        and now>=renew_at_wall_ns and self.terminal is None):
                    renewed=min(recording_deadline_wall_ns,now+600_000_000_000)
                    for request,token in leases:
                        if not _renew_recording_lease(request,token,lease_deadline_wall_ns,renewed,
                                clock_ns=self.clock_ns,active=lambda:self.terminal is None
                                    and (not self.defer_execution or self.preparation_current())):break
                    else:
                        lease_deadline_wall_ns=renewed
                        renew_at_wall_ns=now+(renewed-now)//2
                self._drain_events()
                # Snapshot Owner first: a later publication may be ahead of it,
                # but a writer delay must not make this reference race forward.
                with self.lock:
                    self.checked_reference=self.feedback_progress
                    terminal_round=self.terminal
                endpoint=(self.recorder._stamp(self.recorder.feedback[-1],'feedback')[0] if self.recorder.feedback else 0)
                tail_endpoint=endpoint if self.environment=='fake' and terminal_round and self.events.empty() else None
                for camera,(directory,index) in list(cursors.items()):
                    self.writer_stage='camera:'+camera
                    rows=self.recorder.frames[camera]
                    head=self.spool_head(camera,directory) if self.environment=='fake' and terminal_round is None else None
                    # Bounded batch prevents a fast camera from starving command evidence.
                    for _ in range(4):
                        if tail_endpoint is not None and rows and self.recorder._stamp(rows[-1],'rgbd')[0]>=tail_endpoint:break
                        path=directory/f'{index:09d}.zip'
                        if not path.is_file():break
                        if tail_wait_started is not None:
                            tail_wait_elapsed+=time.monotonic()-tail_wait_started;tail_wait_started=None
                        self.recorder.record_rgbd(camera,path.read_bytes())
                        # The recorder has persisted both frame and index before reclaiming our spool copy.
                        path.unlink()
                        index+=1
                        cursors[camera]=(directory,index)
                    cursors[camera]=(directory,index)
                    rows=self.recorder.frames[camera]
                    if rows and (self.environment!='fake' or terminal_round is None):
                        if self.environment=='fake':
                            # A frame ahead of the last Owner tick remains pending
                            # that feedback; its actual source advance is still
                            # liveness evidence, never a fabricated fresh reference.
                            stamp=self.check_fake_head(camera,head['metadata'],self.checked_reference)
                            previous=camera_progress.get(camera)
                            if previous is None or stamp!=previous[0]:
                                assert_progress(previous[0] if previous else None,stamp)
                                camera_progress[camera]=(stamp,self.clock_ns(),time.monotonic())
                            progress=camera_progress[camera]
                            if self.clock_ns()-progress[1]>=30_000_000_000 or time.monotonic()-progress[2]>=30:
                                raise ValueError(camera+': native RGB-D source stalled for 30 wall seconds')
                        elif self.clock_ns()-int(rows[-1]['host_receive_ns'])>500_000_000:
                            raise ValueError(camera+': native RGB-D recording stalled')
                with self.lock:terminal=self.terminal
                if terminal and self.events.empty():
                    status,reason,requested=terminal
                    endpoint=self.recorder._stamp(self.recorder.feedback[-1],'feedback')[0] if self.recorder.feedback else 0
                    covered=all(rows and self.recorder._stamp(rows[-1],'rgbd')[0]>=endpoint for rows in self.recorder.frames.values())
                    expired=time.monotonic()-requested>=1
                    if self.environment=='fake':
                        if recording_deadline_wall_ns is not None and (self.clock_ns()>=recording_deadline_wall_ns
                                or time.monotonic()>=recording_deadline_monotonic):
                            raise ValueError('recording reached fixed resident wall deadline minus cleanup reserve')
                        available=any((not self.recorder.frames[camera] or self.recorder._stamp(self.recorder.frames[camera][-1],'rgbd')[0]<endpoint)
                            and (directory/f'{index:09d}.zip').is_file() for camera,(directory,index) in cursors.items())
                        if available:
                            if tail_wait_started is not None:tail_wait_elapsed+=time.monotonic()-tail_wait_started
                            tail_wait_started=None
                        elif tail_wait_started is None:tail_wait_started=time.monotonic()
                        waited=tail_wait_elapsed+(time.monotonic()-tail_wait_started if tail_wait_started is not None else 0.)
                        expired=not available and waited>=1
                    if not cursors or covered or expired:
                        self.save_failure_diagnostics(cursors)
                        self.manifest=self.recorder.seal('recording_failed' if self.error and self.execution_timeline else status,reason)
                        break
        except Exception as error:
            self.fail(error)
            if self.recorder and not self.recorder.closed:
                try:
                    self.save_failure_diagnostics(cursors)
                    self.manifest=self.recorder.seal('recording_failed' if self.execution_timeline else 'failed',self.error)
                except Exception as seal_error:self.seal_error=f'{type(seal_error).__name__}: {seal_error}'
        finally:
            for request,token in leases:
                try:
                    if json.loads(request.read_text()).get('run_id')==token:request.unlink()
                    try:(request.parent/token).rmdir()  # Only empty, fully consumed session spools.
                    except (FileNotFoundError,OSError):pass
                except FileNotFoundError:pass
                except Exception as error:self.fail('recording lease cleanup failed: '+str(error))
            if self.defer_execution and self.recorder is None:
                try:
                    self.root.parent.mkdir(parents=True,exist_ok=True)
                    (self.root.parent/'recording-preparation.json').write_text(json.dumps(self.preparation_receipt(),allow_nan=False))
                except Exception as error:self.diagnostics_error=f'preparation receipt: {type(error).__name__}: {error}'
            self.done.set()


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    export=commands.add_parser('export',help='derive replay from already-local sealed physical raw and target videos')
    export.add_argument('--raw-root',type=Path,required=True)
    export.add_argument('--target-manifest',type=Path,required=True)
    export.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(export_playback(args.raw_root,args.target_manifest,args.output),allow_nan=False))


if __name__=='__main__':main()
