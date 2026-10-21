"""Physical raw evidence and derived replay. No SDK, acquisition, or motion calls.

The caller feeds every camera pair from its capture loop (polling latest snapshots
cannot establish complete raw capture). Commands and measured feedback remain
separate. Sealing a command timeline never declares task or grasp success.
"""
from copy import deepcopy
from contextlib import ExitStack
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
from itertools import islice
import json
import math
import os
import pickle
import queue
import sys
import uuid
from pathlib import Path
import statistics
import stat
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


def _raw_file_index(root,names,*,expected=None,consume_rgbd=None):
    """Two bounded reads; consume verified bytes in order on the caller thread."""
    result={};directories={}
    def read_file(item):
        name,path,parent=item;blob=None
        fd=os.open(path.name,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK,dir_fd=parent)
        try:
            info=os.fstat(fd)
            if expected is None and stat.S_ISDIR(info.st_mode):return name,None,None
            if not stat.S_ISREG(info.st_mode):raise ValueError('regular raw file required: '+name)
            if expected is not None and info.st_size!=expected[name]['size_bytes']:
                raise ValueError('sealed raw file changed: '+name)
            with os.fdopen(fd,'rb',closefd=False) as stream:
                rgbd=consume_rgbd is not None and path.parts[0]=='camera' and path.suffix=='.zip'
                digest=hashlib.sha256()
                if rgbd:
                    if info.st_size>16*1024*1024:raise ValueError('bounded snapshot ZIP bytes required')
                    blob=stream.read();digest.update(blob)
                else:
                    for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
            pin=dict(size_bytes=info.st_size,sha256=digest.hexdigest())
            if expected is not None and pin['sha256']!=expected[name]['sha256']:raise ValueError('sealed raw file changed: '+name)
            return name,pin,blob
        finally:os.close(fd)
    with ExitStack() as stack:
        # Join readers before closing shared directory FDs, including failures.
        with ThreadPoolExecutor(max_workers=2) as readers:
            names=iter(names)
            while batch:=list(islice(names,2)):
                jobs=[]
                for name in batch:
                    path=Path(name)
                    if path.is_absolute() or '..' in path.parts or str(path)!=name or not path.parts:
                        raise ValueError('invalid sealed raw path: '+str(name))
                    for count in range(len(path.parts)):
                        parts=path.parts[:count]
                        if parts not in directories:
                            fd=os.open(root if not parts else parts[-1],os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW,
                                **({'dir_fd':directories[parts[:-1]]} if parts else {}))
                            directories[parts]=fd;stack.callback(os.close,fd)
                    jobs.append((name,path,directories[path.parts[:-1]]))
                for name,pin,blob in readers.map(read_file,jobs):
                    if pin is not None:result[name]=pin
                    if blob is not None:consume_rgbd(name,blob)
                    blob=None  # Do not retain the last image into the next batch.
    return result


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
    def __init__(self, root, source, plan, binding, *, clock_ns=time.time_ns, execution_timeline=None, environment='physical', clock_domain=None, source_clock_id=None, execution_owner=None, storage_format='zip', defer_execution=False):
        started=time.monotonic()
        if type(defer_execution) is not bool or defer_execution and execution_timeline is not None:
            raise ValueError('pre-roll binds its final execution timeline once')
        self.defer_execution=defer_execution
        self.startup_timings={}
        if storage_format not in ('zip','segments'):raise ValueError('known RGB-D storage format required')
        self.storage_format=storage_format
        self.segment_writers={}
        self.index_streams={}
        self.segment_executor=ThreadPoolExecutor(max_workers=len(CAMERAS),thread_name_prefix='rgbd-commit') if storage_format=='segments' else None
        self.namespace=namespace(environment)
        self.environment=environment
        self.execution_owner=owner_identity(execution_owner) if execution_owner is not None else None
        self.clock=clock_for_source(source,environment)
        if (clock_domain is not None or source_clock_id is not None) and clock_spec(environment,
                clock_domain=clock_domain,source_clock_id=source_clock_id)!=self.clock:
            raise ValueError('recorder clock differs from sealed source')
        # FrozenExecutor invokes this same complete source/plan validator below.
        # Do not rescan a multi-thousand-tick plan once more before constructing it.
        self.frozen = FrozenExecutor(plan,source,sink=lambda _:None,gate=lambda:True,stop=lambda _:None,
                                    source_validator=lambda p,s:validate_real_plan(p,s,binding,environment=environment),
                                    disabled_grippers=(source.get('initial_state') or {}).get('disabled_grippers',()),
                                    execution_policy='observation' if execution_timeline or defer_execution else 'strict')
        if execution_timeline is not None:
            from execution_timeline import validate
            validate(execution_timeline,plan)
            self.frozen.configure_runtime(execution_timeline)
        validated=time.monotonic()
        self.startup_timings['validate_wall_s']=validated-started
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
        self.startup_timings.update(storage_setup_wall_s=time.monotonic()-validated,
                                   total_wall_s=time.monotonic()-started)
        self.first_camera_timings={}

    def bind_execution(self,timeline):
        """Add the actual timeline after pre-roll; never replace captured images."""
        if not self.defer_execution or self.closed or self.commands or self.feedback:
            raise ValueError('one final timeline before execution events required')
        from execution_timeline import validate
        validate(timeline,self.plan)
        self.frozen.configure_runtime(timeline)
        self.write_json('execution-timeline.json',timeline)
        self.defer_execution=False

    def write_json(self,name,value):
        (self.root/name).write_text(json.dumps(value,allow_nan=False,indent=2)+'\n')

    def append(self,name,row):
        if self.closed: raise ValueError('recording sealed')
        if self.storage_format=='segments':
            # One append stream per index; line buffering persists each record
            # without the CPFS open/close roundtrip for every frame or feedback.
            if name not in self.index_streams:self.index_streams[name]=(self.root/name).open('x',buffering=1)
            self.index_streams[name].write(json.dumps(row,allow_nan=False)+'\n')
            return
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

    def record_command(self,tick,target,send_monotonic_s,send_wall_ns,*,clock_domain=None,source_clock_id=None,send_source_time_ns=None,physics_step=None,feedback_at_send=None,owner_pid=None,command_epoch=None,runtime_nonce=None,execution_semantics='SDK_point_send'):
        if self.defer_execution:raise ValueError('execution timeline not bound')
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
        if execution_semantics not in ('SDK_point_send','source_reference_knot'):
            raise ValueError('explicit command evidence semantics required')
        if execution_semantics=='source_reference_knot':row['execution_semantics']=execution_semantics
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

    def record_native_window(self,packet,admission,send_monotonic_s,send_wall_ns):
        from robot_workbench.target_interpreter import SCHEMA
        if (not isinstance(packet,dict)or packet.get('schema')!=SCHEMA
                or not 2<=len(packet.get('host_times_s',[]))<=32
                or not finite(send_monotonic_s)or type(send_wall_ns)is not int
                or not isinstance(admission,dict)):
            raise ValueError('bounded actual native-window evidence required')
        self.append('native-windows.jsonl',dict(packet=deepcopy(packet),admission=deepcopy(admission),
            send_monotonic_s=send_monotonic_s,send_wall_ns=send_wall_ns,
            evidence='native_queue_return_not_firmware_ACK'))
        self.native_window_count=getattr(self,'native_window_count',0)+1

    def record_feedback(self,tick,state,sample_monotonic_s,sample_wall_ns,raw=None,*,clock_domain=None,source_clock_id=None,source_time_ns=None,physics_step=None,owner_pid=None,command_epoch=None,runtime_nonce=None):
        if self.defer_execution:raise ValueError('execution timeline not bound')
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
        meta,color,depth = bundle(blob,self.source['camera_serials'][SIDES[camera]],environment=self.environment,clock=self.clock)
        return self._record_rgbd(camera,meta,color,depth,blob=blob)

    def record_rgbd_frame(self,camera,meta,color_png,depth_png):
        if self.closed or camera not in CAMERAS:raise ValueError('open recording and known camera required')
        validate_rgbd_metadata(meta,self.source['camera_serials'][SIDES[camera]],environment=self.environment,clock=self.clock)
        if png_shape(color_png)!=png_shape(depth_png,depth=True):raise ValueError('RGB and aligned depth dimensions differ')
        return self._record_rgbd(camera,meta,color_png,depth_png)

    def record_rgbd_pixels(self,camera,meta,rgb,depth,width,height):
        if self.closed or camera not in CAMERAS:
            raise ValueError('open recording and known camera required')
        validate_rgbd_metadata(meta,self.source['camera_serials'][SIDES[camera]],environment=self.environment,clock=self.clock)
        if self.storage_format=='zip':
            from rgbd_segments import _png
            return self._record_rgbd(camera,meta,_png(rgb,width,height,False),_png(depth,width,height,True))
        return self._record_rgbd(camera,meta,rgb,depth,pixel_shape=(width,height))

    def _record_rgbd(self,camera,meta,color,depth,*,blob=None,pixel_shape=None):
        started=time.monotonic()
        if self.environment=='fake':self._check_owner(meta,frame=True)
        rows = self.frames[camera]
        if rows and meta['pair_id'] == rows[-1]['pair_id']: return False
        assert_progress(self._stamp(rows[-1],'rgbd') if rows else None,self._stamp(meta,'rgbd'))
        if rows and (int(meta['host_receive_ns']) <= int(rows[-1]['host_receive_ns'])
                     or meta['frame_number'] <= rows[-1]['frame_number']
                     or meta['depth_frame_number'] <= rows[-1]['depth_frame_number']):
            raise ValueError('camera pair timestamps/frame counters regressed')
        if self.storage_format=='segments':
            from rgbd_segments import SegmentWriter
            detail=None
            if not rows:
                detail=self.first_camera_timings.setdefault(camera,
                    dict(stage='writer_setup',status='started',started_wall_ns=time.time_ns()))
            if camera not in self.segment_writers:
                self.segment_writers[camera]=SegmentWriter(self.root/'camera',camera,commit_executor=self.segment_executor)
            writer_ready=time.monotonic()
            if detail is not None:detail.update(writer_setup_wall_s=writer_ready-started,stage='encode')
            writer=self.segment_writers[camera]
            ref=(writer.add_pixels(meta,color,depth,*pixel_shape) if pixel_shape else writer.add(meta,color,depth))
            encoded=time.monotonic()
            if detail is not None:detail.update(encode_wall_s=encoded-writer_ready,stage='index_append')
            row=dict(meta,segment='camera/'+ref['segment'],segment_frame=ref['frame'])
        else:
            from real_rgbd import snapshot_bundle
            directory = self.root/'camera'/camera
            directory.mkdir(parents=True,exist_ok=True)
            path = directory/f'{len(rows):08d}.zip'
            path.write_bytes(blob if blob is not None else snapshot_bundle(meta,color,depth))
            row = dict(meta,path=str(path.relative_to(self.root)))
        self.append(f'{camera}-pairs.jsonl',row)
        rows.append(row)
        if self.storage_format=='segments' and len(rows)==1:
            detail.update(index_append_wall_s=time.monotonic()-encoded,
                total_wall_s=time.monotonic()-started,status='written',stage='complete',finished_wall_ns=time.time_ns())
        return True

    def seal(self,status,reason=None):
        started=time.monotonic()
        if self.closed or status not in ('completed','failed','stopped','recording_failed'):
            raise ValueError('one explicit terminal outcome required')
        problems = []
        if self.storage_format=='segments':
            try:
                for writer in self.segment_writers.values():writer.finish()
            finally:
                self.segment_executor.shutdown(wait=True)
                for stream in self.index_streams.values():stream.close()
                self.index_streams.clear()
        self.write_json('recording-timings.json',dict(startup=self.startup_timings,
            first_camera=self.first_camera_timings,
            segments={camera:writer.timings for camera,writer in self.segment_writers.items()}))
        pairings=[];duration_coverage=None
        streaming=getattr(self,'streaming',False)
        shared=(self.source.get('target_execution')=='shared_reference_windows'
                or any(row.get('execution_semantics')=='source_reference_knot'for row in self.commands))
        if shared and not getattr(self,'native_window_count',0):
            problems.append('shared target execution has no actual native-window evidence')
        if not streaming and len(self.commands) != len(self.frozen.ticks): problems.append('not every execution command recorded')
        if not self.feedback or not self.commands:
            problems.append('command or measured feedback stream missing')
        else:
            commands=[self._stamp(row,'command') for row in self.commands]
            feedback=[self._stamp(row,'feedback') for row in self.feedback]
            duration=self.source['duration_s'] if streaming else self.frozen.duration_s
            endpoint = (commands[0][0]/1e9 if self.environment=='fake' else self.commands[0]['send_monotonic_s'])+duration
            last_feedback = feedback[-1][0]/1e9 if self.environment=='fake' else self.feedback[-1]['sample_monotonic_s']
            if streaming:
                from robot_workbench.sample_clock import policy_duration_coverage
                duration_coverage=policy_duration_coverage(duration,last_feedback-(endpoint-duration))
            if (not duration_coverage['passed'] if streaming else last_feedback+1e-9 < endpoint):
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
        semantics_finished=time.monotonic()
        files = _raw_file_index(self.root,(str(path.relative_to(self.root)) for path in self.root.rglob('*')))
        self.seal_timings=dict(semantic_wall_s=semantics_finished-started,
            file_integrity_wall_s=time.monotonic()-semantics_finished,files=len(files),bytes=sum(v['size_bytes'] for v in files.values()))
        self.seal_timings.update(startup=self.startup_timings,first_camera=self.first_camera_timings)
        if self.storage_format=='segments':
            self.seal_timings['segments']={camera:getattr(writer,'timings',{}) for camera,writer in self.segment_writers.items()}
        manifest = dict(schema_version=self.namespace['raw_schema'],evidence_source=self.namespace['evidence'],
                        source_run_id=self.source['execution_instance_id'] if streaming else self.source['run_id'],
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
        if getattr(self,'native_window_count',0):
            semantics=('source_reference_knots'if any(row.get('execution_semantics')=='source_reference_knot'for row in self.commands)
                else'native_window_admissions'if self.commands and all(row.get('command_kind')=='native_future_window'for row in self.commands)
                else'SDK_point_sends')
            manifest.update(native_window_count=self.native_window_count,
                native_windows_sha256=files.get('native-windows.jsonl',{}).get('sha256'),
                command_semantics=semantics,
                native_evidence_scope='bounded native queue returns; firmware ACK unsupported')
        if streaming:
            manifest.update(recording_kind='streaming_policy',source_digest=_digest(self.source),duration_coverage=duration_coverage,
                execution_instance_id=self.source['execution_instance_id'],scheduled_duration_s=self.source['duration_s'],
                actual_commands_sha256=files.get('commands.jsonl',{}).get('sha256'),
                command_scope='actual submitted commands; no planned future timeline',
                actual_duration_s=((self._stamp(self.feedback[-1],'feedback')[0]-self._stamp(self.commands[0],'command')[0])/1e9
                    if self.feedback and self.commands else 0.))
        else:manifest.update(plan_run_id=self.plan['run_id'],timeline_digest=self.plan['timeline_digest'])
        if self.storage_format=='segments':manifest['rgbd_storage']='lossless_segments_v1'
        if self.environment=='fake':manifest.update(environment='fake',simulation_only=True,clock=self.clock,source_clock='simulation',
            source_clock_id=self.clock['source_clock_id'],execution_owner=self.execution_owner,
            frame_feedback_association='exact samples or explicit same-clock brackets')
        if not streaming and self.frozen.runtime_timeline:
            manifest.update(execution_timeline_digest=self.frozen.runtime_timeline['timeline_digest'],
                execution_hz=self.frozen.runtime_timeline['control_hz'],
                execution_duration_s=self.frozen.duration_s)
        self.write_json('manifest.json',manifest)
        self.closed = True
        self.seal_timings['total_wall_s']=time.monotonic()-started
        # Timings are diagnostics outside the frozen raw file set, not data.
        (self.root.parent/'recording-seal-timings.json').write_text(json.dumps(self.seal_timings,indent=2)+'\n')
        return manifest

    def abort(self):
        for writer in self.segment_writers.values():writer.abort()
        if self.segment_executor is not None:self.segment_executor.shutdown(wait=True,cancel_futures=True)
        for stream in self.index_streams.values():stream.close()
        self.index_streams.clear()


def validate_policy_source(source,binding,environment):
    from integration import validate_profile
    from robot_workbench.sample_clock import POLICY_DURATION_TOLERANCE_S
    if (source.get('schema_version')!='policy_execution_source.v1' or source.get('environment')!=environment
            or not isinstance(source.get('execution_instance_id'),str) or not source['execution_instance_id']
            or type(source.get('duration_s')) not in (int,float) or not 0<source['duration_s']<=40
            or source.get('duration_tolerance_s',POLICY_DURATION_TOLERANCE_S)!=POLICY_DURATION_TOLERANCE_S
            or binding.get('calibration_accepted') is not True or binding.get('robot_id')!=source.get('robot_id')
            or binding.get('camera_serials')!=source.get('camera_serials')
            or set(source.get('camera_serials',{}))!={'head','left','right'}
            or len(set(source['camera_serials'].values()))!=3):
        raise ValueError('current device/calibration identity and bounded policy execution source required')
    clock_for_source(source,environment);validate_profile(source['calibration_profile'])
    if binding.get('calibration_profile_digest')!=_digest(source['calibration_profile']):
        raise ValueError('policy recording calibration differs from accepted installed snapshot')
    if bool(source['calibration_profile']['source'].get('simulation_only'))!=(environment=='fake'):
        raise ValueError('policy recording calibration environment differs')
    return source


def _recording_identity(root,environment):
    from rgbd_segments import _safe
    for name in ('source.json','binding.json'):_safe(root,name)
    source,binding=[json.loads((root/name).read_text()) for name in ('source.json','binding.json')]
    if source.get('schema_version')=='policy_execution_source.v1':
        validate_policy_source(source,binding,environment);return source,None,binding
    plan=json.loads(_safe(root,'plan.json').read_text());validate_real_plan(plan,source,binding,environment=environment)
    return source,plan,binding


def verify_raw(root, *, environment='physical', _consume_rgbd=None):
    root = Path(root).resolve()
    if (root/'manifest.json').is_symlink(): raise ValueError('raw manifest symlink forbidden')
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest.get('schema_version') != namespace(environment)['raw_schema']:
        raise ValueError('physical raw manifest required')
    validate_namespace(manifest, environment)
    streaming=manifest.get('recording_kind')=='streaming_policy'
    if not ({'source.json','binding.json'} | (set() if streaming else {'plan.json'})) <= set(manifest.get('files',{})):
        raise ValueError('raw identity files must be sealed')
    storage=manifest.get('rgbd_storage','zip')
    if storage not in ('zip','lossless_segments_v1'):raise ValueError('unknown raw RGB-D storage')
    _raw_file_index(root,manifest['files'],expected=manifest['files'],consume_rgbd=_consume_rgbd if storage=='zip' else None)
    source,plan,binding=_recording_identity(root,environment)
    if streaming!=(plan is None):raise ValueError('recording kind differs from source identity')
    if environment=='fake' and (manifest.get('clock')!=clock_for_source(source,environment)
            or manifest.get('source_clock')!='simulation' or manifest.get('source_clock_id')!=source['clock']['source_clock_id']):
        raise ValueError('raw clock differs from sealed source')
    if streaming:
        if (manifest.get('source_digest')!=_digest(source) or manifest.get('source_run_id')!=source['execution_instance_id']
                or manifest.get('actual_commands_sha256')!=manifest['files'].get('commands.jsonl',{}).get('sha256')):
            raise ValueError('actual policy execution identity or command hash differs')
    elif manifest.get('source_run_id') != source['run_id'] or manifest.get('plan_run_id') != plan['run_id'] or manifest.get('timeline_digest') != plan['timeline_digest']:
        raise ValueError('raw identity differs')
    if manifest.get('execution_timeline_digest'):
        from execution_timeline import validate
        if 'execution-timeline.json' not in manifest['files']:raise ValueError('execution timeline not sealed')
        timeline=validate(json.loads((root/'execution-timeline.json').read_text()),plan)
        if (timeline['timeline_digest']!=manifest['execution_timeline_digest']
                or len(timeline['commands'])!=manifest['command_count'] and manifest['capture_status']=='complete'):
            raise ValueError('raw execution timeline differs')
    if storage=='lossless_segments_v1' and _consume_rgbd is not None:
        from real_rgbd import snapshot_bundle
        for camera in CAMERAS:
            for row,meta,color,depth in iter_rgbd(root,camera,environment=environment):
                _consume_rgbd(_frame_key(row),snapshot_bundle(meta,color,depth))
    return manifest,source,plan


def _frame_key(row):
    return row['path'] if 'path' in row else row['segment']+'#'+str(row['segment_frame'])


def iter_rgbd(root,camera,*,environment='physical'):
    """Read old ZIP or lossless segments in order, with only one frame buffered.

    Call verify_raw before treating an entire recording as valid. This reader
    also checks the actual image bytes and their metadata/index association.
    """
    if camera not in CAMERAS:raise ValueError('known camera required')
    root=Path(root).resolve()
    source=json.loads((root/'source.json').read_text())
    clock=clock_for_source(source,environment)
    rows=[json.loads(line) for line in (root/f'{camera}-pairs.jsonl').read_text().splitlines()]
    if not rows:return
    if 'path' in rows[0]:
        from rgbd_segments import _safe
        for row in rows:
            path=_safe(root,row['path'])
            meta,color,depth=bundle(path.read_bytes(),source['camera_serials'][SIDES[camera]],environment=environment,clock=clock)
            if meta!={k:v for k,v in row.items() if k!='path'}:raise ValueError('RGB-D index differs from stored frame')
            yield row,meta,color,depth
        return
    from rgbd_segments import iter_segment
    from itertools import groupby
    seen=set()
    for segment,group in groupby(rows,key=lambda row:row['segment']):
        if segment in seen:raise ValueError('repeated or noncontiguous segment')
        seen.add(segment)
        part=Path(segment)
        if len(part.parts)!=3 or part.parts[:2]!=('camera',camera):raise ValueError('RGB-D segment belongs to another camera')
        expected=list(group)
        count=0
        for count,(meta,color,depth) in enumerate(iter_segment(root/'camera',str(part.relative_to('camera'))),1):
            if count>len(expected):raise ValueError('segment has unindexed frames')
            row=expected[count-1]
            if row.get('segment_frame')!=count-1 or meta!={k:v for k,v in row.items() if k not in ('segment','segment_frame')}:
                raise ValueError('RGB-D index differs from stored segment frame')
            validate_rgbd_metadata(meta,source['camera_serials'][SIDES[camera]],environment=environment,clock=clock)
            yield row,meta,color,depth
        if count!=len(expected):raise ValueError('indexed segment frames missing')


def recover_raw(root,output,*,environment='physical'):
    """Publish a separate read-only prefix report; never resume motion or edit raw."""
    from rgbd_segments import recover,iter_segment,_safe
    root=Path(root)
    if root.is_symlink():raise ValueError('symlink raw root forbidden')
    root=root.resolve()
    for name in ('source.json','binding.json'):_safe(root,name)
    source,plan,binding=_recording_identity(root,environment)
    identity_names=('source.json','binding.json') if plan is None else ('source.json','plan.json','binding.json')
    report=dict(schema='rgbd_recovered_prefix_v1',raw_root=str(root),environment=environment,
        status='incomplete',episode_complete=False,motion_resumed=False,cameras={})
    report['identity_files']={name:dict(size_bytes=(root/name).stat().st_size,sha256=sha256(root/name))
        for name in identity_names}
    report['event_streams']={}
    for name in ('commands.jsonl','feedback.jsonl'):
        path=root/name;count=0;last_complete_offset=0;truncated=False;digest=hashlib.sha256()
        if path.is_symlink():raise ValueError('symlink event stream forbidden')
        if path.exists():
            with path.open('rb') as stream:
                while line:=stream.readline(64*1024+1):
                    if len(line)>64*1024:raise ValueError('bounded event record required')
                    if not line.endswith(b'\n'):truncated=True;break
                    if not isinstance(json.loads(line),dict):raise ValueError('event object required')
                    count+=1;last_complete_offset+=len(line);digest.update(line)
        report['event_streams'][name]=dict(complete_records=count,complete_bytes=last_complete_offset,
            complete_prefix_sha256=digest.hexdigest(),truncated_tail=truncated)
    for camera in CAMERAS:
        recovered=recover(root/'camera',camera)
        # Fully consume each committed segment to check lossless pixel evidence,
        # source/Owner identity and strictly advancing original source stamps.
        count=0;previous=None;verified=[]
        for segment in recovered['segments']:
            ref=segment.get('commit_path')
            if ref is None:ref=f"{camera}/{segment['segment']:06d}.commit.json"
            before=count
            try:
                for meta,_,_ in iter_segment(root/'camera',ref):
                    validate_rgbd_metadata(meta,source['camera_serials'][SIDES[camera]],environment=environment,clock=clock_for_source(source,environment))
                    stamp=sample_stamp({**meta,'host_receive_ns':int(meta['host_receive_ns'])},clock_for_source(source,environment))
                    assert_progress(previous,stamp);previous=stamp;count+=1
            except (ValueError,RuntimeError,OSError,TimeoutError) as error:
                recovered['errors'].append(str(error));count=before;break
            verified.append(segment)
        recovered['segments']=verified
        recovered['frame_count']=count
        report['cameras'][camera]=dict(recovered,verified_frame_count=count)
    output=Path(output)
    if output.resolve().is_relative_to(root):raise ValueError('recovery report must stay outside original raw')
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return report


def iter_recovered_rgbd(report_path,camera):
    """Read only the explicitly recovered prefix; no complete-dataset claim."""
    if camera not in CAMERAS:raise ValueError('known camera required')
    from rgbd_segments import _safe,iter_segment,verify_segment
    report=json.loads(Path(report_path).read_text())
    if report.get('schema')!='rgbd_recovered_prefix_v1' or report.get('episode_complete') is not False:
        raise ValueError('explicit incomplete-prefix recovery report required')
    root=Path(report['raw_root'])
    for name,pin in report['identity_files'].items():
        path=_safe(root,name)
        if path.stat().st_size!=pin['size_bytes'] or sha256(path)!=pin['sha256']:
            raise ValueError('recovered identity changed')
    for expected in report['cameras'][camera]['segments']:
        ref=f"{camera}/{expected['segment']:06d}.commit.json"
        if verify_segment(root/'camera',ref)!=expected:raise ValueError('recovered segment changed')
        for index,(meta,color,depth) in enumerate(iter_segment(root/'camera',ref)):
            yield dict(meta,segment='camera/'+ref,segment_frame=index),meta,color,depth


def export_playback(raw_root,target_manifest_path,output,*,runner=subprocess.run,environment='physical',scratch_root=None):
    """Derived videos may exist for a failed run; raw/task outcome stays explicit."""
    started=time.monotonic();diagnostics=dict(schema='source_playback_timings.v1',cameras={})
    raw_root = Path(raw_root).resolve()
    if scratch_root is not None:
        scratch_root=Path(scratch_root)
        if (environment!='fake' or not scratch_root.is_absolute() or '..' in scratch_root.parts
                or scratch_root.is_symlink() or not scratch_root.is_dir() or scratch_root.resolve().is_relative_to(raw_root)):
            raise ValueError('fake playback scratch must be an existing explicit directory outside sealed raw')
        scratch_root=scratch_root.resolve()
    target_manifest_path = Path(target_manifest_path).resolve() if target_manifest_path is not None else None
    spec=namespace(environment)
    # These provisional identities only prepare the export. The complete frozen
    # verifier below checks every input while extracting the same verified bytes.
    raw=json.loads((raw_root/'manifest.json').read_text())
    source=json.loads((raw_root/'source.json').read_text())
    plan=None if raw.get('recording_kind')=='streaming_policy' else json.loads((raw_root/'plan.json').read_text())
    if plan is None and target_manifest_path is not None:raise ValueError('policy execution has no planned target replay')
    if plan is None and raw.get('rgbd_storage')!='lossless_segments_v1':raise ValueError('policy video export requires common segment storage')
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
    if raw.get('rgbd_storage')=='lossless_segments_v1':
        from segmented_playback import export_segments
        verifying=time.monotonic()
        verified=verify_raw(raw_root,environment=environment)
        if verified!=(raw,source,plan):raise ValueError('raw identity changed during export')
        verified_elapsed=time.monotonic()-verifying
        result=export_segments(raw_root,raw,source,plan,output,runner=runner,environment=environment,
            target=target,target_paths=target_paths,target_manifest_path=target_manifest_path)
        path=Path(output)/'export-timings.json';timings=json.loads(path.read_text())
        timings.update(verify_raw_wall_s=verified_elapsed,export_total_wall_s=time.monotonic()-started)
        path.write_text(json.dumps(timings,indent=2)+'\n')
        from robot_workbench.recording_quality import for_export
        for_export(raw_root,output,raw,source,environment=environment)
        return result
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
        indexed={row['path']:(camera,index,row) for camera,rows in frames.items() for index,row in enumerate(rows)}
        if len(indexed)!=sum(map(len,frames.values())):raise ValueError('duplicate RGB-D index path')
        consumed=set()
        def consume(name,blob):
            camera,index,row=indexed[name];before=time.monotonic()
            meta,color,_=bundle(blob,source['camera_serials'][SIDES[camera]],environment=environment,clock=clock)
            if meta!={k:v for k,v in row.items() if k!='path'}:raise ValueError('RGB-D index metadata differs from bundle')
            (temporary/f'{camera}-{index:08d}.png').write_bytes(color)
            consumed.add(name)
            detail=diagnostics['cameras'].setdefault(camera,dict(frames=0,decode_and_stage_wall_s=0.))
            detail['frames']+=1;detail['decode_and_stage_wall_s']+=time.monotonic()-before
        before=time.monotonic()
        verified=verify_raw(raw_root,environment=environment,_consume_rgbd=consume)
        if verified!=(raw,source,plan) or consumed!=set(indexed):raise ValueError('raw identities or indexed RGB-D files changed during export')
        diagnostics['verify_and_extract_wall_s']=time.monotonic()-before
        for camera,rows in frames.items():
            stamps = [sample_stamp({**row,'host_receive_ns':int(row['host_receive_ns'])},clock)[0] for row in rows]
            periods = [(b-a)/1e9 for a,b in zip(stamps,stamps[1:])]
            last = statistics.median(periods) if periods else 1/rows[0]['capture_fps']
            # Concat parses each duration in microseconds. Quantize absolute
            # offsets first so sub-microsecond rounding cannot accumulate.
            offsets_us = [(stamp-stamps[0]+500)//1000 for stamp in stamps]
            concat_periods = [(b-a)/1e6 for a,b in zip(offsets_us,offsets_us[1:])]
            entries = []
            for index,(row,duration) in enumerate(zip(rows,concat_periods+[last])):
                path = temporary/f'{camera}-{index:08d}.png'
                escaped = str(path).replace("'", "'\\''")
                entries.extend([f"file '{escaped}'",'option framerate 1000',f'duration {duration:.9f}'])
            concat = temporary/f'{camera}.txt'; concat.write_text('\n'.join(entries)+'\n')
            destination = output/f'source-{camera}.mp4'
            # Concat does not carry the final duration through encoding. Disable
            # frame reordering so the final packet is the final displayed frame,
            # then set its duration without adding a frame or changing captured PTS.
            tail_ns = max(1, round(last * 1e9))
            before=time.monotonic()
            runner(['ffmpeg','-nostdin','-v','error','-n','-threads','2','-f','concat','-safe','0','-i',str(concat),
                    '-an','-c:v','libx264','-threads','2','-preset','fast','-crf','24','-pix_fmt','yuv420p',
                    '-bf','0','-bsf:v',f'setts=duration=if(eq(N\\,{len(rows)-1})\\,{tail_ns}/1000000000/TB\\,DURATION)',
                    '-fps_mode','vfr','-enc_time_base','1:1000','-video_track_timescale','1000000','-movflags','+faststart',str(destination)],check=True,timeout=300)
            diagnostics['cameras'][camera]['encode_wall_s']=time.monotonic()-before
            timing[camera] = dict(clock='simulation' if environment=='fake' else 'host_receive',frame_offsets_s=[(stamp-command_start)/1e9 for stamp in stamps],
                                 frame_count=len(rows),depth_in_raw=True,last_frame_duration_s=last,
                                 duration_s=(stamps[-1]-stamps[0])/1e9+last)
            if environment=='fake':timing[camera].update(source_clock_id=clock['source_clock_id'],host_receive_ns=[int(row['host_receive_ns']) for row in rows])
        import shutil
        for name,path in target_paths.items(): shutil.copyfile(path,output/name)
    before=time.monotonic()
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
    diagnostics.update(video_verify_wall_s=time.monotonic()-before,total_wall_s=time.monotonic()-started)
    (output/'export-timings.json').write_text(json.dumps(diagnostics,allow_nan=False,indent=2)+'\n')
    from robot_workbench.recording_quality import for_export
    for_export(raw_root,output,raw,source,environment=environment)
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


class RecordingEventQueue(queue.Queue):
    """Immutable event snapshots bounded by stored bytes, not control tick count.

    Pickle is only an internal snapshot of locally submitted events; no external
    pickle input is accepted. Accounting includes each bytes object's allocation
    and 64 bytes for its deque slot/block overhead. The writer decodes one event
    at a time; camera pixels normally use their separate shared byte budget.
    """
    def __init__(self, *, max_bytes=64*1024**2, capacity=None):
        if type(max_bytes) is not int or max_bytes<=0 or (capacity is not None and (type(capacity) is not int or capacity<=0)):
            raise ValueError('positive event byte budget and optional explicit event capacity required')
        super().__init__(maxsize=capacity or 0)
        self.max_bytes=max_bytes
        self.queued_bytes=self.peak_bytes=self.peak_events=self.accepted_events=self.largest_event_bytes=0

    def put(self,event,block=True,timeout=None):
        # Serialize once before acquiring the queue mutex. This also prevents a
        # caller mutating nested SDK feedback while the writer is initializing.
        payload=pickle.dumps(event,protocol=5)
        return super().put(payload,block=block,timeout=timeout)

    def _put(self,payload):
        size=sys.getsizeof(payload)+64
        if self.queued_bytes+size>self.max_bytes:raise queue.Full
        super()._put(payload)
        self.queued_bytes+=size;self.accepted_events+=1
        self.peak_bytes=max(self.peak_bytes,self.queued_bytes)
        self.peak_events=max(self.peak_events,len(self.queue))
        self.largest_event_bytes=max(self.largest_event_bytes,size)

    def _get(self):
        payload=super()._get()
        self.queued_bytes-=sys.getsizeof(payload)+64
        return pickle.loads(payload)

    def snapshot(self,*,include_oldest=False):
        with self.mutex:
            result=dict(depth=len(self.queue),capacity=self.maxsize or None,queued_bytes=self.queued_bytes,
                max_bytes=self.max_bytes,peak_bytes=self.peak_bytes,peak_events=self.peak_events,
                accepted_events=self.accepted_events,largest_event_bytes=self.largest_event_bytes,
                accounting='serialized_bytes_object_plus_64_bytes_per_queue_entry',scope='pending_control_events')
            if include_oldest:result['oldest_event']=pickle.loads(self.queue[0]) if self.queue else None
            return result


class RecordingSession:
    """Bounded nonblocking owner input; this worker alone writes recording files.

    Native camera directories come from preview-only real_rgbd services.
    An exclusive bounded lease enables full-frame spooling only for this session.
    Tests may omit directories and submit paired bundles through the same queue.
    """
    def __init__(self,root,source,plan,binding,*,camera_directories=None,camera_channels=None,storage_format='zip',clock_ns=time.time_ns,
                 capacity=None,event_budget_bytes=64*1024**2,on_fault=lambda reason:None,recorder_factory=RealRecorder,execution_timeline=None,environment='physical',clock_domain=None,source_clock_id=None,execution_owner=None,
                 resident_deadline_wall_ns=None,persistent_service=False,max_wall_time_seconds=None,
                 defer_execution=False,execution_instance_id=None,
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
        if defer_execution and (execution_timeline is not None
                or not isinstance(execution_instance_id,str) or not execution_instance_id
                or not callable(preparation_active) or not finite(preparation_deadline_monotonic)
                or camera_directories is None and camera_channels is None):
            raise ValueError('deferred recording requires an execution instance and bounded active preparation')
        if camera_channels is not None and (camera_directories is not None
                or set(camera_channels)!=set(CAMERAS) or not defer_execution):
            raise ValueError('shared cameras require one deferred execution and three channels')
        if storage_format not in ('zip','segments'):raise ValueError('known RGB-D storage format required')
        self.storage_format=storage_format
        self.camera_channels=camera_channels
        self.channel_tokens={}
        self.channel_receipts={}
        self.channel_cutoffs={}
        if defer_execution:owner_identity(execution_owner)
        self.defer_execution=defer_execution
        self.execution_instance_id=execution_instance_id
        self.preparation_active=preparation_active
        self.preparation_deadline_monotonic=preparation_deadline_monotonic
        self.prepared,self.execution_bound,self.cancelled=threading.Event(),threading.Event(),threading.Event()
        self.preparation_started_wall_ns=clock_ns()
        self.prepared_wall_ns=self.bound_wall_ns=None
        self.lease_tokens=set()
        self.events=RecordingEventQueue(max_bytes=event_budget_bytes,capacity=capacity)
        self.ready,self.done=threading.Event(),threading.Event()
        self.lock=threading.Lock()
        self.error,self.manifest,self.terminal=None,None,None
        self.controller_result=None
        self.on_fault=on_fault
        self.clock_ns=clock_ns
        self.camera_directories=camera_directories
        self.resident_deadline_wall_ns=resident_deadline_wall_ns
        if type(persistent_service) is not bool or (persistent_service and (resident_deadline_wall_ns is not None or not finite(max_wall_time_seconds)
                or not 0<max_wall_time_seconds<=7200)):
            raise ValueError('persistent recording requires a separate finite budget up to 120 minutes')
        self.persistent_service=persistent_service
        self.max_wall_time_seconds=max_wall_time_seconds
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
        if not self.preparation_current():raise ValueError('recording preparation expired or execution was revoked')

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
        return dict(schema='recording_preparation.v2',environment=self.environment,execution_instance_id=self.execution_instance_id,
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
            if kind not in ('command','feedback','rgbd','native_window'):raise ValueError('unknown recording input')
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
            except queue.Full:error='bounded recording event queue overflow: '+json.dumps(self.events.snapshot(),sort_keys=True)
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
        if self.camera_channels is not None:
            meta=directory.latest(self.channel_tokens[camera])
            if meta is None:return None
            validate_rgbd_metadata(meta,self.recorder.source['camera_serials'][SIDES[camera]],environment=self.environment,clock=self.clock)
            if self.environment=='fake':self.recorder._check_owner(meta,frame=True)
            previous=dict(index=meta['recorded_frames'],metadata=meta,observed_wall_ns=self.clock_ns(),observed_monotonic_s=time.monotonic())
            self.spool_heads[camera]=previous
            return previous
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
            if self.environment=='fake':self.recorder._check_owner(meta,frame=True)
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
        queue_status=self.events.snapshot(include_oldest=True)
        pending=queue_status.pop('oldest_event')
        oldest=None
        if pending:
            kind,fields,wall,queued=pending
            raw=fields.get('raw') or {}
            oldest=dict(kind=kind,tick=fields.get('tick'),queued_wall_ns=wall,queued_monotonic_s=queued,
                queue_wait_wall_s=(now-wall)/1e9,queue_wait_monotonic_s=mono-queued,
                source_time_ns=fields.get('send_source_time_ns',fields.get('source_time_ns',raw.get('source_time_ns'))))
        cameras={}
        for camera,(directory,index) in cursors.items():
            rows=self.recorder.frames[camera] if self.recorder else []
            head=self.spool_heads.get(camera)
            waiting=self.pending_reference.get(camera)
            memory=directory.snapshot() if self.camera_channels is not None else None
            pending_indices=[] if memory is not None else sorted(int(p.stem) for p in directory.glob('[0-9]'*9+'.zip'))
            cameras[camera]=dict(next_archive_index=index,archived_count=len(rows),
                memory=memory,
                archived_first=frame(rows[0]) if rows else None,archived_tail=frame(rows[-1]) if rows else None,
                checked_spool_index=head['index'] if head else None,checked_spool_head=frame(head['metadata']) if head else None,
                checked_spool_observed_wall_ns=head['observed_wall_ns'] if head else None,
                pending_reference=(dict(source_time_ns=waiting[0][0],physics_step=waiting[0][1],
                    wait_started_wall_ns=waiting[1],wait_started_monotonic_s=waiting[2]) if waiting else None),
                spool_pending_count=len(pending_indices),spool_first_index=pending_indices[0] if pending_indices else None,
                spool_last_index=pending_indices[-1] if pending_indices else None)
        return dict(schema='recording_failure_diagnostics.v1',error=self.error,writer_stage=self.writer_stage,
            captured_wall_ns=now,captured_monotonic_s=mono,checked_reference=reference(self.checked_reference),
            latest_submitted_reference=reference(latest_reference),queue=dict(queue_status,oldest=oldest),
            last_event_write=self.last_event_write,last_event_batch=self.last_event_batch,
            last_nonempty_event_batch=self.last_nonempty_event_batch,
            command_count=len(self.recorder.commands) if self.recorder else 0,
            feedback_count=len(self.recorder.feedback) if self.recorder else 0,cameras=cameras,
            startup_timings=getattr(self.recorder,'startup_timings',None),
            first_camera_timings=getattr(self.recorder,'first_camera_timings',None),
            segments={camera:writer.timings for camera,writer in self.recorder.segment_writers.items()} if self.recorder else {})

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
                    storage_format=self.storage_format,
                    event_queue=self.events.snapshot(),
                    memory_channels=self.channel_receipts or ({c:deepcopy(ch.cached_snapshot) if hasattr(ch,'cached_snapshot') else ch.snapshot()
                        for c,ch in self.camera_channels.items()} if self.camera_channels else None),
                    camera_admission_cutoffs=deepcopy(self.channel_cutoffs),
                    seal_timings=getattr(self.recorder,'seal_timings',None),
                    manifest={k:v for k,v in self.manifest.items() if k!='files'} if self.manifest else None)

    def _camera_available(self,camera,directory,index):
        return (directory.peek(self.channel_tokens[camera]) is not None if self.camera_channels is not None
                else (directory/f'{index:09d}.zip').is_file())

    def _consume_memory_frame(self,camera,channel):
        self.writer_stage='camera:'+camera
        token=self.channel_tokens[camera]
        frame=channel.peek(token)
        if frame is None:return False
        try:
            if frame.get('pixel_format')=='rgb8-depth16le':
                self.recorder.record_rgbd_pixels(camera,frame['metadata'],frame['rgb'],frame['depth'],frame['width'],frame['height'])
            else:self.recorder.record_rgbd_frame(camera,frame['metadata'],frame['color_png'],frame['depth_png'])
        except Exception as error:
            detail=self.recorder.first_camera_timings.get(camera)
            if detail is not None and detail['status']=='started':
                detail.update(status='failed',error=str(error),finished_wall_ns=time.time_ns())
            raise
        if hasattr(channel,'ack'):channel.ack(token,frame['metadata']['pair_id'])
        else:channel.pop(token)
        return True

    def _end_channels(self):
        if self.camera_channels is not None:
            for camera,channel in self.camera_channels.items():
                token=self.channel_tokens.get(camera)
                if token is not None:
                    self.channel_receipts.setdefault(camera,channel.snapshot())
                    channel.end(token)

    def _stop_camera_admission(self,terminal):
        if not terminal or not self.events.empty() or self.camera_channels is None:return
        endpoint=(self.recorder._stamp(self.recorder.feedback[-1],'feedback')[0]
                  if terminal[0]=='completed' and self.recorder.feedback else 0)
        for camera,channel in self.camera_channels.items():
            if camera not in self.channel_cutoffs:
                self.channel_cutoffs[camera]=channel.stop_after(self.channel_tokens[camera],endpoint)

    def _run(self,source,plan,binding,factory):
        cursors={}
        camera_progress={}
        leases=[]
        lease_deadline_wall_ns=None
        recording_deadline_wall_ns=None;recording_deadline_monotonic=None;renew_at_wall_ns=None
        deadline_reason=('recording reached its fixed per-job wall-time budget' if self.persistent_service else
                         'recording reached fixed resident wall deadline minus cleanup reserve')
        tail_wait_started=None;tail_wait_elapsed=0.
        def create_recorder(*,defer_timeline=False):
            return factory(self.root,source,plan,binding,clock_ns=self.clock_ns,
                **({'storage_format':self.storage_format} if self.storage_format!='zip' else {}),
                **({'defer_execution':True} if defer_timeline else
                   {'execution_timeline':self.execution_timeline} if self.execution_timeline is not None else {}),
                **({'environment':self.environment,'clock_domain':self.clock['clock_domain'],'source_clock_id':self.clock['source_clock_id'],
                    'execution_owner':self.execution_owner} if self.environment=='fake' else {}))
        try:
            if not self.defer_execution:self.recorder=create_recorder()
            sources=self.camera_channels if self.camera_channels is not None else self.camera_directories
            if sources is not None:
                if set(sources)!=set(CAMERAS):raise ValueError('three native RGB-D sources required')
                if self.environment=='fake':
                    if not self.persistent_service and type(self.resident_deadline_wall_ns) is not int:
                        raise ValueError('fake recording requires the fixed resident wall deadline')
                    now=self.clock_ns()
                    recording_deadline_wall_ns=(now+int(self.max_wall_time_seconds*1e9) if self.persistent_service
                        else self.resident_deadline_wall_ns-45_000_000_000)
                    if now>=recording_deadline_wall_ns:raise ValueError('resident recording budget exhausted; 45 s cleanup reserve required')
                    recording_deadline_monotonic=time.monotonic()+(recording_deadline_wall_ns-now)/1e9
                    lease_deadline_wall_ns=min(recording_deadline_wall_ns,now+600_000_000_000)
                    renew_at_wall_ns=now+(lease_deadline_wall_ns-now)//2
                else:
                    duration=(self.recorder.frozen.duration_s if self.recorder else plan['duration_s'])+15
                    if not 0 < duration <= 600:raise ValueError('raw recording supports at most 585 seconds of execution')
                    now=self.clock_ns();recording_deadline_wall_ns=now+round(duration*1e9)
                    recording_deadline_monotonic=time.monotonic()+duration
                    lease_deadline_wall_ns=recording_deadline_wall_ns
                for camera,value in sources.items():
                    if self.defer_execution:self.check_preparation()
                    directory=value if self.camera_channels is not None else Path(value).resolve(strict=True)
                    if self.camera_channels is None:
                        service=json.loads((directory/'recording-service.json').read_text())
                        if (service.get('schema') != 'rgbd_lease.v1'
                                or service.get('transport','filesystem') != 'filesystem'):
                            raise ValueError(camera+': on-demand recording service required')
                    start=self.clock_ns()
                    deadline_wall_ns=lease_deadline_wall_ns if self.environment=='fake' else start+round(duration*1e9)
                    lease_deadline_wall_ns=min(lease_deadline_wall_ns or deadline_wall_ns,deadline_wall_ns)
                    token=uuid.uuid4().hex
                    lease=dict(run_id=token,start_wall_ns=start,deadline_wall_ns=deadline_wall_ns,
                        admission_start_wall_ns=start)
                    if self.defer_execution:
                        lease.update(execution_instance_id=self.execution_instance_id,execution_owner=self.execution_owner,environment=self.environment)
                        self.lease_tokens.add(token)
                    if self.camera_channels is not None:
                        directory.begin(lease)
                        self.channel_tokens[camera]=token
                        leases.append((directory,token))
                        cursors[camera]=(directory,1)
                        self.check_preparation()
                        continue
                    request=directory/'recording-request.json'
                    temporary=directory/(token+'.request.tmp')
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
                    if self.camera_channels is not None:
                        # Lease readiness still releases the Owner immediately.
                        # Persist its pre-roll now instead of leaving every camera
                        # queued throughout the stationary/initialization window.
                        self.recorder=create_recorder(defer_timeline=True)
                    while not self.execution_bound.wait(.01):
                        self.check_preparation()
                        if self.clock_ns()>=recording_deadline_wall_ns or time.monotonic()>=recording_deadline_monotonic:
                            raise ValueError(deadline_reason)
                        if self.clock_ns()>=lease_deadline_wall_ns:
                            raise ValueError('recording lease actual wall deadline exceeded')
                        if self.camera_channels is not None:
                            for camera,(channel,index) in list(cursors.items()):
                                for _ in range(4):
                                    if self.execution_bound.is_set() or not self._consume_memory_frame(camera,channel):break
                                    index+=1
                                cursors[camera]=(channel,index)
                    self.check_preparation()
                    if self.camera_channels is not None:self.recorder.bind_execution(self.execution_timeline)
                    else:self.recorder=create_recorder()
                    self.recorder.write_json('recording-preparation.json',self.preparation_receipt())
                ready_deadline=time.monotonic()+5
                while True:
                    self.writer_stage='awaiting_first_frames'
                    if recording_deadline_wall_ns is not None and (self.clock_ns()>=recording_deadline_wall_ns
                            or time.monotonic()>=recording_deadline_monotonic):
                        raise ValueError(deadline_reason)
                    with self.lock:self.checked_reference=self.feedback_progress
                    first=[];first_referenced=True
                    for camera,(directory,index) in cursors.items():
                        stored=self.recorder.frames[camera]
                        if stored or self._camera_available(camera,directory,index):
                            if stored:meta=stored[0]
                            elif self.camera_channels is not None:meta=directory.peek(self.channel_tokens[camera])['metadata']
                            else:meta,_,_=bundle((directory/f'{index:09d}.zip').read_bytes(),source['camera_serials'][SIDES[camera]],environment=self.environment,clock=self.clock)
                            if meta.get('recording_run_id') != (self.channel_tokens[camera] if self.camera_channels is not None else directory.name) or not meta.get('recording_enabled'):
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
                fresh=( [directory.latest(self.channel_tokens[camera]) for camera,(directory,_) in cursors.items()]
                    if self.camera_channels is not None else first )
                if self.environment!='fake' and any(not 0 <= self.clock_ns()-int(meta['host_receive_ns']) <= 500_000_000 for meta in fresh):
                    raise ValueError('native raw stream absent or stale')
            self.ready.set()
            while True:
                now=self.clock_ns()
                if recording_deadline_wall_ns is not None and (now>=recording_deadline_wall_ns
                        or time.monotonic()>=recording_deadline_monotonic):
                    raise ValueError(deadline_reason)
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
                        if self.camera_channels is not None:
                            self.check_preparation()
                            if not request.renew(token,lease_deadline_wall_ns,renewed):break
                        elif not _renew_recording_lease(request,token,lease_deadline_wall_ns,renewed,
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
                self._stop_camera_admission(terminal_round)
                endpoint=(self.recorder._stamp(self.recorder.feedback[-1],'feedback')[0] if self.recorder.feedback else 0)
                tail_endpoint=endpoint if terminal_round and self.events.empty() else None
                for camera,(directory,index) in list(cursors.items()):
                    self.writer_stage='camera:'+camera
                    rows=self.recorder.frames[camera]
                    head=self.spool_head(camera,directory) if terminal_round is None else None
                    # Bounded batch prevents a fast camera from starving command evidence.
                    for _ in range(4):
                        if tail_endpoint is not None and rows and self.recorder._stamp(rows[-1],'rgbd')[0]>=tail_endpoint:break
                        if not self._camera_available(camera,directory,index):break
                        if tail_wait_started is not None:
                            tail_wait_elapsed+=time.monotonic()-tail_wait_started;tail_wait_started=None
                        if self.camera_channels is not None:
                            self._consume_memory_frame(camera,directory)
                        else:
                            path=directory/f'{index:09d}.zip'
                            self.recorder.record_rgbd(camera,path.read_bytes())
                            # Only reclaim the spool after the recorder accepted it.
                            path.unlink()
                        index+=1
                        cursors[camera]=(directory,index)
                    cursors[camera]=(directory,index)
                    rows=self.recorder.frames[camera]
                    # After execution ends, these are historical queued frames.
                    # Freshness is a live-producer check; terminal draining is
                    # instead qualified by complete indices and end coverage.
                    if rows and terminal_round is None:
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
                        else:
                            # Disk/codec work above may outlast the original head
                            # sample while ingress continues. Judge the current
                            # producer, not the queued frame captured before IO.
                            current_head=self.spool_head(camera,directory)
                            age=self.clock_ns()-int((current_head['metadata'] if current_head else rows[-1])['host_receive_ns'])
                            if age>500_000_000:
                                with self.lock:still_live=self.terminal is None
                                if still_live:raise ValueError(camera+': native RGB-D recording stalled')
                with self.lock:terminal=self.terminal
                if terminal and self.events.empty():
                    status,reason,requested=terminal
                    endpoint=self.recorder._stamp(self.recorder.feedback[-1],'feedback')[0] if self.recorder.feedback else 0
                    covered=all(rows and self.recorder._stamp(rows[-1],'rgbd')[0]>=endpoint for rows in self.recorder.frames.values())
                    if self.environment=='fake':
                        if recording_deadline_wall_ns is not None and (self.clock_ns()>=recording_deadline_wall_ns
                                or time.monotonic()>=recording_deadline_monotonic):
                            raise ValueError(deadline_reason)
                    # Both devices may finish while valid camera frames remain
                    # queued behind startup/encoding. Only an empty producer
                    # tail consumes the bounded wait, never time spent draining.
                    available=any((not self.recorder.frames[camera] or self.recorder._stamp(self.recorder.frames[camera][-1],'rgbd')[0]<endpoint)
                        and self._camera_available(camera,directory,index) for camera,(directory,index) in cursors.items())
                    if available:
                        if tail_wait_started is not None:tail_wait_elapsed+=time.monotonic()-tail_wait_started
                        tail_wait_started=None
                    elif tail_wait_started is None:tail_wait_started=time.monotonic()
                    waited=tail_wait_elapsed+(time.monotonic()-tail_wait_started if tail_wait_started is not None else 0.)
                    expired=not available and waited>=1
                    if not cursors or covered or expired:
                        self.save_failure_diagnostics(cursors)
                        self._end_channels()
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
            self._end_channels()
            for request,token in leases:
                if self.camera_channels is not None:continue
                try:
                    if json.loads(request.read_text()).get('run_id')==token:request.unlink()
                    try:(request.parent/token).rmdir()  # Only empty, fully consumed session spools.
                    except (FileNotFoundError,OSError):pass
                except FileNotFoundError:pass
                except Exception as error:self.fail('recording lease cleanup failed: '+str(error))
            if self.recorder is not None and not self.recorder.closed:self.recorder.abort()
            if self.defer_execution and self.recorder is None:
                try:
                    self.root.parent.mkdir(parents=True,exist_ok=True)
                    (self.root.parent/'recording-preparation.json').write_text(json.dumps(self.preparation_receipt(),allow_nan=False))
                except Exception as error:self.diagnostics_error=f'preparation receipt: {type(error).__name__}: {error}'
            self.done.set()


class StreamingRecorder(RealRecorder):
    """Same RGB-D writer, with actual policy events instead of a future plan."""
    def __init__(self,root,source,binding,*,execution_owner,clock_ns=time.time_ns,storage_format='segments'):
        started=time.monotonic();environment=source['environment']
        validate_policy_source(source,binding,environment)
        self.execution_owner=owner_identity(execution_owner)
        if any(source.get(k)!=execution_owner[k] for k in ('runtime_nonce','command_epoch')):
            raise ValueError('policy recording source differs from execution Owner')
        self.streaming=True;self.defer_execution=False;self.frozen=self.plan=None
        self.environment=environment;self.namespace=namespace(environment);self.clock=clock_for_source(source,environment)
        self.source=deepcopy(source);self.root=Path(root).resolve();self.root.mkdir(parents=True,exist_ok=False)
        self.clock_ns=clock_ns;self.created_ns=clock_ns();self.closed=False
        if storage_format not in ('zip','segments'):raise ValueError('known RGB-D storage format required')
        self.storage_format=storage_format;self.commands=[];self.feedback=[];self.frames={camera:[] for camera in CAMERAS}
        self.segment_writers={};self.index_streams={};self.first_camera_timings={}
        self.segment_executor=ThreadPoolExecutor(max_workers=len(CAMERAS),thread_name_prefix='rgbd-commit')
        self.write_json('source.json',source);self.write_json('binding.json',binding)
        self.startup_timings=dict(total_wall_s=time.monotonic()-started)

    def record_command(self,tick,target,send_monotonic_s,send_wall_ns,**fields):
        validate_physical14(target)
        if type(tick) is not int or tick!=len(self.commands) or not finite(send_monotonic_s):
            raise ValueError('contiguous actual command index and finite send time required')
        row=dict(fields,tick=tick,target=deepcopy(target),send_monotonic_s=send_monotonic_s,send_wall_ns=send_wall_ns)
        self._check_owner(row);stamp=self._stamp(row,'command')
        if self.environment=='fake' and abs(round(send_monotonic_s*1e9)-stamp[0])>1:
            raise ValueError('command control time differs from source time')
        if self.commands:
            if send_monotonic_s<=self.commands[-1]['send_monotonic_s'] or send_wall_ns<=self.commands[-1]['send_wall_ns']:
                raise ValueError('actual command timestamps regressed')
            assert_progress(self._stamp(self.commands[-1],'command'),stamp)
            if stamp[0]-self._stamp(self.commands[0],'command')[0]>round(self.source['duration_s']*1e9):
                raise ValueError('actual policy command exceeds bounded source duration')
        preceding=fields.get('feedback_at_send')
        if not isinstance(preceding,dict):raise ValueError('actual preceding Owner feedback required for command evidence')
        seen=sample_stamp(preceding,self.clock,wall_key='sample_wall_ns')
        if not 0<=stamp[0]-seen[0]<=100_000_000 or preceding['sample_wall_ns']>send_wall_ns:
            raise ValueError('command does not have fresh preceding measured feedback')
        self.append('commands.jsonl',row);self.commands.append(row)

    def record_feedback(self,tick,state,sample_monotonic_s,sample_wall_ns,raw=None,**fields):
        validate_physical_feedback(state,self.source['calibration_profile'],enforce_range=False)
        if type(tick) is not int or tick<0 or not finite(sample_monotonic_s):
            raise ValueError('actual feedback index and finite sample time required')
        identity={key:(fields[key] if key in fields else (raw or {}).get(key))
            for key in ('clock_domain','source_clock_id','source_time_ns','physics_step','owner_pid','command_epoch','runtime_nonce')}
        row=dict(identity,tick=tick,state=deepcopy(state),sample_monotonic_s=sample_monotonic_s,
                 sample_wall_ns=sample_wall_ns,raw=deepcopy(raw))
        self._check_owner(row);stamp=self._stamp(row,'feedback')
        if self.environment=='fake' and abs(round(sample_monotonic_s*1e9)-stamp[0])>1:
            raise ValueError('feedback control time differs from source time')
        if self.feedback:
            if sample_monotonic_s<=self.feedback[-1]['sample_monotonic_s'] or sample_wall_ns<=self.feedback[-1]['sample_wall_ns']:
                raise ValueError('actual feedback timestamps regressed')
            assert_progress(self._stamp(self.feedback[-1],'feedback'),stamp)
        self.append('feedback.jsonl',row);self.feedback.append(row)


class StreamingRecordingSession(RecordingSession):
    """Bounded camera subscription and nonblocking actual-event append for policy runs."""
    def __init__(self,root,source,binding,*,camera_channels,execution_owner,on_fault=lambda reason:None,
                 preparation_active=lambda:True,clock_ns=time.time_ns,max_wall_time_seconds=600,storage_format='segments'):
        if not finite(max_wall_time_seconds) or not 0<max_wall_time_seconds<=7200:
            raise ValueError('bounded policy recording wall budget required')
        self.stream_wall_budget=max_wall_time_seconds
        super().__init__(root,source,None,binding,camera_channels=camera_channels,storage_format=storage_format,
            execution_owner=execution_owner,environment=source['environment'],on_fault=on_fault,clock_ns=clock_ns,
            defer_execution=True,execution_instance_id=source['execution_instance_id'],
            preparation_active=preparation_active,preparation_deadline_monotonic=time.monotonic()+1)

    def start(self):return self
    def append_command(self,**fields):
        if not self.execution_bound.is_set():self.bound_wall_ns=self.clock_ns();self.execution_bound.set()
        return self.submit('command',**deepcopy(fields))
    def append_feedback(self,**fields):return self.submit('feedback',**deepcopy(fields))

    def append_native_window(self,**fields):return self.submit('native_window',**deepcopy(fields))
    def status(self):return self.snapshot()

    def _run(self,source,_plan,binding,_factory):
        cursors={};deadline=time.monotonic()+self.stream_wall_budget;empty_tail_started=None
        try:
            self.recorder=StreamingRecorder(self.root,source,binding,execution_owner=self.execution_owner,clock_ns=self.clock_ns,storage_format=self.storage_format)
            for camera,channel in self.camera_channels.items():
                self.check_preparation();token=uuid.uuid4().hex;now=self.clock_ns()
                lease=dict(run_id=token,start_wall_ns=now,admission_start_wall_ns=now,
                    deadline_wall_ns=now+round(min(self.stream_wall_budget,600)*1e9),environment=self.environment,
                    execution_instance_id=self.execution_instance_id,execution_owner=self.execution_owner)
                self.lease_tokens.add(token);channel.begin(lease);self.channel_tokens[camera]=token;cursors[camera]=(channel,1)
            self.prepared_wall_ns=self.clock_ns();self.prepared.set()
            while True:
                if time.monotonic()>=deadline:raise TimeoutError('policy recording wall budget expired')
                self._drain_events()
                with self.lock:terminal=self.terminal
                self._stop_camera_admission(terminal)
                endpoint=self.recorder._stamp(self.recorder.feedback[-1],'feedback')[0] if self.recorder.feedback else 0
                for camera,(channel,index) in list(cursors.items()):
                    for _ in range(4):
                        rows=self.recorder.frames[camera]
                        if terminal and rows and self.recorder._stamp(rows[-1],'rgbd')[0]>=endpoint:break
                        if not self._consume_memory_frame(camera,channel):break
                        index+=1
                    cursors[camera]=(channel,index)
                if all(self.recorder.frames.values()):self.ready.set()
                with self.lock:terminal=self.terminal
                if terminal and self.events.empty():
                    status,reason,finished=terminal
                    endpoint=self.recorder._stamp(self.recorder.feedback[-1],'feedback')[0] if self.recorder.feedback else 0
                    covered=all(rows and self.recorder._stamp(rows[-1],'rgbd')[0]>=endpoint for rows in self.recorder.frames.values())
                    available=any((not self.recorder.frames[camera] or self.recorder._stamp(self.recorder.frames[camera][-1],'rgbd')[0]<endpoint)
                        and self._camera_available(camera,channel,index) for camera,(channel,index) in cursors.items())
                    if available:empty_tail_started=None
                    elif empty_tail_started is None:empty_tail_started=time.monotonic()
                    if covered or status!='completed' or empty_tail_started is not None and time.monotonic()-empty_tail_started>=5:
                        self._end_channels();self.save_failure_diagnostics(cursors)
                        self.manifest=self.recorder.seal('recording_failed' if self.error else status,self.error or reason);break
        except Exception as error:
            self.fail(error)
            if self.recorder and not self.recorder.closed:
                try:
                    self.save_failure_diagnostics(cursors);self._end_channels()
                    self.manifest=self.recorder.seal('recording_failed',self.error)
                except Exception as seal_error:self.seal_error=str(seal_error)
        finally:
            self._end_channels()
            if self.recorder and not self.recorder.closed:self.recorder.abort()
            self.done.set()


def main():
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    export=commands.add_parser('export',help='derive replay from already-local sealed physical raw and target videos')
    export.add_argument('--raw-root',type=Path,required=True)
    export.add_argument('--target-manifest',type=Path,required=True)
    export.add_argument('--output',type=Path,required=True)
    recovery=commands.add_parser('recover',help='verify committed RGB-D prefix after interruption; never resume motion')
    recovery.add_argument('--raw-root',type=Path,required=True)
    recovery.add_argument('--output',type=Path,required=True)
    recovery.add_argument('--environment',choices=('fake','physical'),required=True)
    args=parser.parse_args()
    result=(recover_raw(args.raw_root,args.output,environment=args.environment) if args.command=='recover'
        else export_playback(args.raw_root,args.target_manifest,args.output))
    print(json.dumps(result,allow_nan=False))


if __name__=='__main__':main()
