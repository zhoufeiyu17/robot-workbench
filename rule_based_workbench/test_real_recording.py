"""CPU fake-frame acceptance: no SDK/network; encoder runner is injectable."""
from copy import deepcopy
import json
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch
import zlib

from integration import _digest
from playback import sha256
from real_recording import RealRecorder, export_playback, verify_raw, png_shape, RecordingSession, CAMERAS, _raw_file_index
from real_rgbd import pair_metadata,snapshot_bundle,RawSpool
from test_frozen_executor import fixture


def png(depth=False):
    def chunk(kind,data):
        return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data))
    data = b'\0'+(b'\0\x01'*2 if depth else b'\x80\x80\x80'*2)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!IIBBBBB',2,2,16 if depth else 8,0 if depth else 2,0,0,0))+chunk(b'IDAT',zlib.compress(data*2))+chunk(b'IEND',b'')


def main():
    # Two reads may overlap, but completed image payloads cannot queue beyond
    # the current pair; callbacks stay ordered on the calling thread.
    import os,threading,time
    from contextlib import contextmanager
    with TemporaryDirectory() as folder:
        root=Path(folder);camera=root/'camera/base_0';camera.mkdir(parents=True)
        pins={}
        for index in range(8):
            path=camera/f'{index:08d}.zip';path.write_bytes(bytes([index])*32)
            pins[str(path.relative_to(root))]=dict(size_bytes=32,sha256=sha256(path))
        counts=dict(active=0,peak_active=0,queued=0,peak_queued=0);lock=threading.Lock();consumed=[]
        caller=threading.get_ident();fdopen=os.fdopen
        @contextmanager
        def observed(fd,*args,**kwargs):
            with fdopen(fd,*args,**kwargs) as stream:
                class Reader:
                    def read(self,*args):
                        with lock:
                            counts['active']+=1;counts['peak_active']=max(counts['peak_active'],counts['active'])
                        time.sleep(.005);value=stream.read(*args)
                        with lock:
                            counts['active']-=1;counts['queued']+=1;counts['peak_queued']=max(counts['peak_queued'],counts['queued'])
                        return value
                yield Reader()
        def consume(name,blob):
            assert threading.get_ident()==caller and blob==bytes([len(consumed)])*32
            time.sleep(.01)
            with lock:counts['queued']-=1
            consumed.append(name)
        with patch('real_recording.os.fdopen',side_effect=observed):
            assert _raw_file_index(root,pins,expected=pins,consume_rgbd=consume)==pins
        assert consumed==list(pins) and counts['peak_active']==2 and counts['peak_queued']<=2 and counts['queued']==0,counts
    corrupt = bytearray(png(True)); corrupt[-1] ^= 1
    try: png_shape(bytes(corrupt),depth=True)
    except ValueError: pass
    else: raise AssertionError('corrupt depth PNG accepted')
    fake,plan = fixture(.001)
    source = {k:deepcopy(v) for k,v in fake.items() if k not in ('lane','adapter','instance_id','gpu')}
    serials = dict(head='h',left='l',right='r')
    source.update(schema_version='physical_collection_source.v1',evidence_source='physical_rgbd',robot_id='arx6',camera_serials=serials)
    binding = dict(calibration_accepted=True,evidence_source='physical_rgbd',robot_id='arx6',camera_serials=serials,
                   source_digest=_digest(source),calibration_profile_digest=_digest(source['calibration_profile']))
    from test_joint_space import physical_fixture
    physical_fixture(plan, source)
    def fill(recorder):
        for tick in plan['ticks']:
            i,t = tick['tick'],tick['time_s']
            recorder.record_command(i,tick['target'],100+t,1_000_000_000+round(t*1e9))
            recorder.record_feedback(i,tick['target'],100+t,1_000_000_000+round(t*1e9),raw={'fixture':True})
        recorder.record_feedback(2,plan['ticks'][-1]['target'],100.084,1_084_000_000)
        for camera,side in zip(CAMERAS,serials):
            for index,stamp in enumerate((990_000_000,1_023_000_000,1_056_000_000,1_090_000_000)):
                metadata = pair_metadata(serial=serials[side],camera_name=side,color_number=index+10,depth_number=index+12,
                    color_ms=index*33,depth_ms=index*33+1,color_domain='hardware_clock',depth_domain='hardware_clock',
                    host_receive_ns=stamp,depth_scale_m=.001,intrinsics={'fx':600})
                blob = snapshot_bundle(metadata,png(),png(True))
                assert recorder.record_rgbd(camera,blob)
                assert recorder.record_rgbd(camera,blob) is False
    with TemporaryDirectory() as directory:
        root = Path(directory)
        recorder = RealRecorder(root/'raw',source,plan,binding,clock_ns=lambda:2_000_000_000)
        fill(recorder)
        manifest = recorder.seal('completed')
        assert manifest['capture_status']=='complete' and manifest['task_outcome']=='unverified'
        assert manifest['command_count']==3 and manifest['feedback_count']==4
        assert manifest['camera_frames']==dict.fromkeys(CAMERAS,4)
        assert verify_raw(root/'raw')[0] == manifest
        from execution_timeline import build
        execution_parent=deepcopy(plan)
        execution_parent['stages']=[dict(name='transport',start_tick=0)]
        timeline=build(execution_parent)
        runtime_recorder=RealRecorder(root/'runtime-raw',source,execution_parent,binding,
            clock_ns=lambda:2_000_000_000,execution_timeline=timeline)
        fill(runtime_recorder)
        measured=deepcopy(plan['ticks'][-1]['target']);measured['left_gripper']=87.
        runtime_recorder.record_feedback(2,measured,100.085,1_085_000_000)
        runtime_manifest=runtime_recorder.seal('completed')
        assert runtime_manifest['capture_status']=='complete' and runtime_manifest['execution_hz']==30
        assert runtime_manifest['execution_timeline_digest']==timeline['timeline_digest']
        assert verify_raw(root/'runtime-raw')[0]==runtime_manifest
        assert json.loads((root/'runtime-raw/feedback.jsonl').read_text().splitlines()[-1])['state']['left_gripper']==87.
        try: recorder.record_feedback(2,plan['ticks'][-1]['target'],101,2_000_000_000)
        except ValueError: pass
        else: raise AssertionError('sealed recording accepts writes')
        session=RecordingSession(root/'queued',source,plan,binding,clock_ns=lambda:2_000_000_000)
        assert session.ready.wait(2), session.error
        for row in recorder.commands:session.submit('command',**row)
        for row in recorder.feedback:session.submit('feedback',**row)
        for camera,rows in recorder.frames.items():
            for row in rows:session.submit('rgbd',camera=camera,blob=(root/'raw'/row['path']).read_bytes())
        session.finish('completed')
        assert session.done.wait(2) and session.manifest['capture_status']=='complete',session.snapshot()
        # Native spool consumer reads every file, including pairs arriving after terminal feedback.
        directories={}
        blobs={}
        spools={}
        import threading,time
        for camera,rows in recorder.frames.items():
            directory=root/('spool-'+camera);directory.mkdir();directories[camera]=str(directory)
            blobs[camera]=[]
            spools[camera]=RawSpool(directory,clock_ns=lambda:1_000_000_000,min_free_bytes=0)
            import zipfile,io
            for row in rows:
                with zipfile.ZipFile(root/'raw'/row['path']) as archive:
                    meta=json.loads(archive.read('metadata.json'))
                    meta.update(recording_enabled=True,recording_duration_s=60,recording_deadline_wall_ns=61_000_000_000)
                    blobs[camera].append(snapshot_bundle(meta,archive.read('color.png'),archive.read('depth.png')))
        def publish(camera,value):
            with zipfile.ZipFile(io.BytesIO(value)) as archive:
                spools[camera].write(json.loads(archive.read('metadata.json')),archive.read('color.png'),archive.read('depth.png'))
        def startup():
            for camera in CAMERAS:
                deadline=time.monotonic()+2
                while not (Path(directories[camera])/'recording-request.json').exists():
                    assert time.monotonic()<deadline
                    time.sleep(.005)
                publish(camera,blobs[camera][0])
        producer=threading.Thread(target=startup);producer.start()
        native=RecordingSession(root/'native',source,plan,binding,camera_directories=directories,clock_ns=lambda:1_000_000_000)
        assert native.ready.wait(2),native.error
        for row in recorder.commands:native.submit('command',**row)
        for row in recorder.feedback:native.submit('feedback',**row)
        native.finish('completed')
        for camera,values in blobs.items():
            for value in values[1:]:publish(camera,value)
        assert native.done.wait(2) and native.manifest['capture_status']=='complete',native.snapshot()
        producer.join()
        assert all(not (Path(d)/'recording-request.json').exists() for d in directories.values())
        counts=[len(list(Path(d).rglob('*.zip'))) for d in directories.values()]
        assert counts==[0,0,0], 'sealed frames must not leave duplicate native spool copies'
        assert all(not [p for p in Path(d).iterdir() if p.is_dir()] for d in directories.values())
        for camera in CAMERAS:publish(camera,blobs[camera][-1])
        assert counts==[len(list(Path(d).rglob('*.zip'))) for d in directories.values()]
        class FailedFrameRecorder(RealRecorder):
            def record_rgbd(self,camera,blob):
                raise OSError('injected destination write failure')
        producer=threading.Thread(target=startup);producer.start()
        write_failure=RecordingSession(root/'write-failure',source,plan,binding,camera_directories=directories,
                                       clock_ns=lambda:1_000_000_000,recorder_factory=FailedFrameRecorder)
        assert write_failure.done.wait(2) and 'destination write failure' in write_failure.error
        producer.join()
        assert all(len(list(Path(d).rglob('*.zip')))==1 for d in directories.values()), 'unconsumed evidence lost'
        assert all(not (Path(d)/'recording-request.json').exists() for d in directories.values())

        bad=RecordingSession(root/'missing-native',source,plan,binding,camera_directories=dict.fromkeys(CAMERAS,str(root/'absent')))
        assert bad.done.wait(2) and bad.error and not bad.ready.is_set()
        partial=dict(directories);partial[CAMERAS[1]]=str(root/'absent')
        bad=RecordingSession(root/'partial-start',source,plan,binding,camera_directories=partial)
        assert bad.done.wait(2) and bad.error and not bad.ready.is_set()
        assert not (Path(directories[CAMERAS[0]])/'recording-request.json').exists()
        # A second consumer cannot take over another session's lease.
        request=Path(directories[CAMERAS[0]])/'recording-request.json'
        request.write_text(json.dumps({'run_id':'other-owner'}))
        conflict=RecordingSession(root/'conflict',source,plan,binding,camera_directories=directories)
        assert conflict.done.wait(2) and conflict.error and not conflict.ready.is_set()
        assert json.loads(request.read_text())['run_id']=='other-owner'
        request.unlink()

        # Cancellation before the first camera frame preserves the controller's
        # real reason instead of inventing a five-second camera timeout.
        cancelled=RecordingSession(root/'cancelled-start',source,plan,binding,camera_directories=directories)
        cancelled.finish('failed','host feedback is older than 0.1 s')
        assert cancelled.done.wait(2) and cancelled.error is None,cancelled.snapshot()
        assert cancelled.manifest['controller_reason']=='host feedback is older than 0.1 s'
        assert cancelled.manifest['camera_frames']==dict.fromkeys(CAMERAS,0)
        assert all(not (Path(d)/'recording-request.json').exists() for d in directories.values())

        # A blocked writer cannot grow memory; overflow faults without waiting for its IO.
        import threading
        release=threading.Event()
        def blocked_factory(*args,**kwargs):
            release.wait(2)
            return RealRecorder(*args,**kwargs)
        overflow=RecordingSession(root/'overflow',source,plan,binding,capacity=1,recorder_factory=blocked_factory)
        overflow.submit('command',**recorder.commands[0])
        try:overflow.submit('command',**recorder.commands[1])
        except ValueError:pass
        else:raise AssertionError('queue overflow accepted')
        release.set()
        assert overflow.done.wait(2) and overflow.error and overflow.manifest['capture_status']=='incomplete'
        target_root=root/'target';target_root.mkdir()
        files={}
        for camera in CAMERAS:
            name=f'target-{camera}.mp4';path=target_root/name;path.write_bytes(b'0000ftyp-target')
            files[name]=dict(size_bytes=path.stat().st_size,sha256=sha256(path))
        target=dict(schema_version='physical_target_playback.v1',evidence_source='simulation',source_run_id=source['run_id'],
                    plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'],files=files)
        target_path=target_root/'manifest.json';target_path.write_text(json.dumps(target))
        calls=[]
        def runner(args,**kwargs):
            calls.append(args)
            path=Path(args[-1])
            if args[0]=='ffmpeg':
                path.write_bytes(b'0000ftyp-source')
                return SimpleNamespace(stdout='')
            return SimpleNamespace(stdout=json.dumps({'streams':[{'codec_name':'h264','pix_fmt':'yuv420p',
                'nb_read_frames':'4' if path.name.startswith('source-') else '7','duration':'0.133'}]}))
        replay=export_playback(root/'raw',target_path,root/'playback',runner=runner)
        import os
        opens=[];original_open=os.open
        def opened(path,*args,**kwargs):
            if str(path).endswith('.zip'):opens.append(str(path))
            return original_open(path,*args,**kwargs)
        with patch('real_recording.os.open',side_effect=opened):
            same=export_playback(root/'raw',target_path,root/'single-pass',runner=runner)
        assert same==replay and len(opens)==12, 'export must hash and unpack each sealed frame in one read'
        diagnostics=json.loads((root/'single-pass/export-timings.json').read_text())
        assert diagnostics['verify_and_extract_wall_s']>0 and set(diagnostics['cameras'])==set(CAMERAS)
        assert recorder.seal_timings['files']==len(manifest['files'])
        assert len(replay['files'])==6 and replay['task_outcome']=='unverified'
        assert replay['source_timing']['base_0']['frame_offsets_s']==[-.01,.023,.056,.09]
        assert all('-fps_mode' in cmd for cmd in calls if cmd[0]=='ffmpeg')
        assert replay['source_timing']['base_0']['duration_s']==.133
        # A correct frame count cannot hide a muxer that drops the final display interval.
        def truncated_tail(args,**kwargs):
            result=runner(args,**kwargs)
            if args[0]=='ffprobe':
                probe=json.loads(result.stdout);probe['streams'][0]['duration']='0.100'
                result.stdout=json.dumps(probe)
            return result
        try:export_playback(root/'raw',None,root/'truncated-tail',runner=truncated_tail)
        except ValueError as error:assert 'duration differs' in str(error)
        else:raise AssertionError('truncated final-frame duration accepted')
        failed=RealRecorder(root/'failed',source,plan,binding)
        fill(failed)
        fail=failed.seal('failed','tracking exceeded')
        assert fail['capture_status']=='incomplete' and fail['controller_reason']=='tracking exceeded'
        from real_workflow import finalize_collection, resolve_raw_source
        import shutil
        artifact=root/'artifact';artifact.mkdir()
        shutil.copytree(target_root,artifact/'target-playback')
        config=dict(artifact_dir=str(artifact),raw_source_dir=str(root/'raw'),robot_id=source['robot_id'],
                    cameras={c:dict(serial=source['camera_serials'][side]) for c,side in zip(CAMERAS,('head','left','right'))},
                    source_path=str(root/'raw/source.json'),plan_path=str(root/'raw/plan.json'),hardware_binding=str(root/'raw/binding.json'))
        reported=dict(done=True,manifest=manifest,error=None,directory='/remote/path/must-not-be-read')
        result=finalize_collection(config,reported,runner=runner)
        assert result['status']=='ready' and result['raw_complete'] and result['task_outcome']=='unverified',result
        count=len(calls)
        assert finalize_collection(config,reported,runner=runner)['status']=='ready'
        assert len(calls)==count,'verified finalization must not transcode twice'
        # Same-host recordings need no raw transport or fixed per-run config path.
        local_config={k:v for k,v in config.items() if k!='raw_source_dir'}
        local_config['local_recording_root']=str(root.resolve())
        local_reported=dict(reported,directory=str((root/'raw').resolve()))
        assert resolve_raw_source(local_config,local_reported)==(root/'raw').resolve()
        assert finalize_collection(local_config,local_reported,runner=runner)['status']=='ready'
        assert len(calls)==count
        # Explicit no-render mode still seals physical raw and all three real videos.
        from real_workflow import playback, summary
        physical_only=dict(config,artifact_dir=str(root/'physical-only'),render_replay=False)
        result=finalize_collection(physical_only,reported,runner=runner)
        assert result['status']=='ready' and result['raw_complete'] and result['target_manifest_sha256'] is None,result
        preview,paths=playback(physical_only)
        assert preview['status']=='ready' and set(paths)=={f'source-{c}.mp4' for c in CAMERAS}
        assert all(preview['cameras'][c]['source']['available'] and not preview['cameras'][c]['target']['available'] for c in CAMERAS)
        assert not (root/'physical-only/target-playback').exists()
        physical_manifest=json.loads((root/'physical-only/playback/manifest.json').read_text())
        assert physical_manifest['provenance']=={'source':'physical_rgbd'}
        actual_owner=dict(mode='hardware',recording=reported,frozen=dict(status='completed',timeline_digest=plan['timeline_digest']))
        assert summary(physical_only,actual_owner)['pipeline']['collection_complete']
        count_after_physical=len(calls)
        assert finalize_collection(physical_only,reported,runner=runner)['status']=='ready'
        assert len(calls)==count_after_physical
        assert playback(dict(physical_only,render_replay=True))[0]['status']=='unavailable'
        def rejected_directory(directory):
            result=finalize_collection(local_config,dict(local_reported,directory=directory),runner=runner)
            assert result['status']=='failed',result
        for directory in (str(root.resolve()),str(root.resolve().parent),'ssh://robot/raw',
                          str(root.resolve()/'..'/'raw')):
            rejected_directory(directory)
        (root/'raw-link').symlink_to(root/'raw',target_is_directory=True)
        rejected_directory(str(root.resolve()/'raw-link'))
        (root/'unsealed').mkdir()
        rejected_directory(str(root.resolve()/'unsealed'))
        old_plan=deepcopy(plan); old_plan['run_id']='previous-plan'
        old=RealRecorder(root/'previous-raw',source,old_plan,binding)
        fill(old)
        old_manifest=old.seal('completed')
        assert verify_raw(root/'previous-raw')[0]==old_manifest
        old_result=finalize_collection(local_config,dict(local_reported,
            directory=str((root/'previous-raw').resolve()),manifest=old_manifest),runner=runner)
        assert old_result['status']=='failed' and 'source/plan 不一致' in old_result['error'],old_result
        assert finalize_collection(config,dict(done=False),runner=runner)['status']=='waiting-recording'
        artifact_failed=root/'artifact-failed';artifact_failed.mkdir()
        shutil.copytree(target_root,artifact_failed/'target-playback')
        failed_config=dict(config,artifact_dir=str(artifact_failed),raw_source_dir=str(root/'failed'))
        result=finalize_collection(failed_config,dict(done=True,manifest=fail,error=None),runner=runner)
        assert result['status']=='failed' and result['replay_ready'] and not result['raw_complete'],result
        assert json.loads((artifact_failed/'collection-finalization.json').read_text())['controller_reason']=='tracking exceeded'
        mismatch=finalize_collection(config,dict(reported,manifest=dict(manifest,timeline_digest='wrong')),runner=runner)
        assert mismatch['status']=='failed' and '终态回执' in mismatch['error']
        incomplete=RealRecorder(root/'incomplete',source,plan,binding).seal('completed')
        assert incomplete['capture_status']=='incomplete' and incomplete['validation']['problems']
        # The directory-FD fast path rejects leaf/parent links and traversal,
        # even if the redirected file would have exactly the expected bytes.
        frame=root/'raw'/recorder.frames[CAMERAS[0]][0]['path'];original=frame.read_bytes()
        outside=root/'outside.zip';outside.write_bytes(original)
        frame.unlink();frame.symlink_to(outside)
        try:
            try:verify_raw(root/'raw')
            except (ValueError,OSError):pass
            else:raise AssertionError('symlink frame accepted')
        finally:frame.unlink();frame.write_bytes(original)
        (root/'raw/camera').rename(root/'moved-camera');(root/'raw/camera').symlink_to(root/'moved-camera',target_is_directory=True)
        try:
            try:verify_raw(root/'raw')
            except (ValueError,OSError):pass
            else:raise AssertionError('symlink frame parent accepted')
        finally:(root/'raw/camera').unlink();(root/'moved-camera').rename(root/'raw/camera')
        saved_manifest=(root/'raw/manifest.json').read_bytes()
        modified=deepcopy(manifest);modified['files']['../outside.zip']=dict(size_bytes=len(original),sha256=sha256(outside))
        (root/'raw/manifest.json').write_text(json.dumps(modified))
        try:
            try:verify_raw(root/'raw')
            except ValueError:pass
            else:raise AssertionError('traversal frame accepted')
        finally:(root/'raw/manifest.json').write_bytes(saved_manifest)
        # A matching file hash is insufficient when its depth PNG is corrupt.
        with zipfile.ZipFile(io.BytesIO(original)) as archive:meta=json.loads(archive.read('metadata.json'))
        broken=bytearray(png(True));broken[-1]^=1
        frame.write_bytes(snapshot_bundle(meta,png(),bytes(broken)))
        modified=deepcopy(manifest);modified['files'][str(frame.relative_to(root/'raw'))]=dict(size_bytes=frame.stat().st_size,sha256=sha256(frame))
        (root/'raw/manifest.json').write_text(json.dumps(modified))
        try:
            try:export_playback(root/'raw',None,root/'bad-depth-export',runner=runner)
            except ValueError as error:assert 'PNG checksum' in str(error)
            else:raise AssertionError('hash-valid corrupt depth accepted by export')
            assert not (root/'bad-depth-export/manifest.json').exists()
        finally:frame.write_bytes(original);(root/'raw/manifest.json').write_bytes(saved_manifest)
        (root/'raw/commands.jsonl').write_text('{}\n')
        try: verify_raw(root/'raw')
        except ValueError: pass
        else: raise AssertionError('raw tamper accepted')
    print('real recording: paired RGBD, exact commands/measured feedback, honest incomplete/failure, VFR replay and tamper checks PASS')


if __name__=='__main__':main()
