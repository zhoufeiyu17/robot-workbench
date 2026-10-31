"""Read only physical-collection artifacts. Never falls back to dual-DSW state."""
from functools import lru_cache
import hashlib
import json
from pathlib import Path

from integration import _digest, _validate_scene, validate_profile
from real_contract import validate_real_plan, validate_real_source, namespace, validate_namespace, owner_matches_environment

CAMERAS = ('base_0', 'left_wrist_0', 'right_wrist_0')
FILES = {f'{side}-{camera}.mp4' for side in ('source','target') for camera in CAMERAS}


def read_object(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value,dict):
        raise ValueError('JSON artifact must contain an object')
    return value


def load(config):
    """Independent stages remain visible; invalid evidence is never displayed as ready."""
    result = dict(binding=None,source=None,plan=None,blockers=[])
    environment=namespace(config.get('environment','physical'))['environment']
    names = dict(hardware_binding='本台真机已验收的标定与观测绑定',source_path='本轮真实 RGB-D 观测与检测场景',
                 plan_path='基于本轮真实场景的仿真验证轨迹')
    values = {}
    for key,label in names.items():
        path = config.get(key)
        if not path:
            result['blockers'].append(f'缺少{label}（{key} 未配置）')
            continue
        try: values[key] = read_object(path)
        except (OSError,ValueError) as error:
            result['blockers'].append(f'{label}不可用：{error}')
    binding = values.get('hardware_binding')
    try:
        cameras = dict(zip(('head','left','right'), (config['cameras'][c]['serial'] for c in CAMERAS)))
        if binding is not None:
            validate_namespace(binding,environment)
            if (binding.get('calibration_accepted') is not True
                    or binding.get('robot_id') != config['robot_id'] or binding.get('camera_serials') != cameras
                    or not binding.get('calibration_profile_digest')):
                raise ValueError('真实标定尚未验收，或机器人/相机身份与当前配置不同')
            result['binding'] = binding
        source = values.get('source_path')
        if source is not None and result['binding'] is not None:
            validate_real_source(source,binding,environment=environment)
            result['source'] = source
        plan = values.get('plan_path')
        if plan is not None and result['source'] is not None:
            validate_real_plan(plan,source,binding,environment=environment)
            from frozen_executor import FrozenExecutor
            frozen = FrozenExecutor(plan,source,sink=lambda _:None,gate=lambda:True,stop=lambda _:None,
                                    disabled_grippers=(source.get('initial_state') or {}).get('disabled_grippers',()),
                                    source_validator=lambda p,s:validate_real_plan(p,s,binding,environment=environment))
            if plan['control']['physics_hz']/plan['control']['substeps'] != 30:
                raise ValueError('真实采集轨迹必须使用 30 Hz 冻结指令')
            result['plan'] = plan
    except (ValueError,KeyError,TypeError) as error:
        result['blockers'].append(str(error))
    return result


def summary(config, owner):
    spec=namespace(config.get('environment','physical'))
    data = load(config)
    source, plan = data['source'],data['plan']
    profile = source['calibration_profile'] if source else None
    frozen = owner.get('frozen') or {}
    matching = bool(plan and owner_matches_environment(owner,spec['environment']) and frozen.get('timeline_digest') == plan['timeline_digest'])
    execution = frozen.get('status','waiting') if matching else 'waiting'
    actual = owner.get('physical_feedback') if owner_matches_environment(owner,spec['environment']) else None
    recording, lanes = {}, []
    raw = (owner.get('recording') or {}).get('manifest') or {}
    if matching and raw.get('source_run_id') == source['run_id'] and raw.get('plan_run_id') == plan['run_id'] and raw.get('timeline_digest') == plan['timeline_digest']:
        recording[spec['lane']] = {**raw, 'validation': (raw.get('validation') or {}).get('status')}
    if matching and execution == 'completed':
        lanes.append(spec['lane'])
    target_rows = []
    if source and plan and config.get('artifact_dir') and config.get('render_replay', True) is not False:
        root = Path(config['artifact_dir'])
        try:
            target = read_object(root/'replay-target/execution.json')
            if (target.get('source_run_id') == source['run_id'] and target.get('plan_run_id') == plan['run_id']
                    and target.get('timeline_digest') == plan['timeline_digest'] and target.get('side') == 'dsw-scene'
                    and target.get('status') == 'completed' and target.get('applied_ticks') == plan['ticks']):
                lanes.append('dsw-scene')
                recording['dsw-scene'] = target.get('recording', {})
                for line in (root/'replay-target/telemetry.jsonl').read_text().splitlines():
                    row = json.loads(line)
                    if isinstance(row, dict) and type(row.get('tick')) is int:
                        target_rows.append(row)
        except (OSError, ValueError, KeyError, TypeError):
            pass
    complete = len(lanes) == (1 if config.get('render_replay', True) is False else 2) and all(recording.get(side, {}).get('capture_status') == 'complete'
                 and recording.get(side, {}).get('validation') == 'passed' for side in lanes)
    dataset=None
    if config.get('dataset_format'):
        dataset=dict(status='waiting',format=config['dataset_format'])
        try:
            path=Path(config['artifact_dir'])/'collection-finalization.json'
            if path.is_symlink() or path.stat().st_size>64*1024:raise ValueError('bounded dataset finalization receipt required')
            report=read_object(path)
            if (not source or not plan or report.get('source_run_id')!=source['run_id']
                    or report.get('plan_run_id')!=plan['run_id'] or report.get('timeline_digest')!=plan['timeline_digest']):
                raise ValueError('dataset finalization belongs to another observation/plan')
            dataset.update(status='ready' if report.get('status')=='ready' and report.get('dataset_ready') is True else 'failed',
                root=report.get('dataset_root'),receipt=report.get('dataset_receipt'),error=report.get('error'))
        except FileNotFoundError:pass
        except (OSError,ValueError,KeyError,TypeError) as error:dataset.update(status='failed',error=str(error))
        complete=complete and dataset['status']=='ready'
    stages = [dict(id=name,label=label,status='ready' if ready else 'blocked') for name,label,ready in (
        ('calibration','真机标定验收',data['binding'] is not None), ('observe','真实 RGB-D 观测与检测',source is not None),
        ('plan','仿真规划与验证',plan is not None))]
    stages.append(dict(id='execute',label='真机控制与录制',status=execution))
    blockers = list(data['blockers'])
    if config.get('hardware_access_enabled', True) is False:
        blockers.append('相机与控制权当前由标定工作台使用；数采工作台处于开发模式')
    if not source:
        blockers.append('真实 5 s 观测/检测及可信 source 产物尚未就绪；当前仅完成 RGB-D 接入，不会自动伪造场景')
    scene = source['scene'] if source else None
    rows = []
    if matching and actual and type(frozen.get('last_tick')) is int and frozen['last_tick'] >= 0:
        rows = [dict(tick=frozen['last_tick'],stage='physical-execution',
                     jaw_opening_mm=dict(left=actual['left_gripper'],right=actual['right_gripper']),
                     measured_state=actual,sample_age_s=owner.get('sample_age_s'),
                     event=frozen.get('reason'),grasp_state=None)]
    result = dict(state=actual,adapters={
        'calibration':dict(status='ready' if data['binding'] else 'awaiting-profile',detail='已验收的真实标定' if data['binding'] else '等待本台真机标定验收'),
        'online_detection':dict(status='ready' if source else 'awaiting-5s-window',detail='真实 RGB-D 观测' if source else '等待真实观测与检测产物'),
        'simulation':dict(status='planned' if plan else 'awaiting-scene',detail='同一冻结指令' if plan else '等待真实场景仿真规划'),
        'control':dict(status=owner.get('state','DISCONNECTED'),detail='独立真机控制服务，30 Hz')},
        calibration_profile=dict(profile_id=profile['profile_id'],schema_version=profile['schema_version'],digest=_digest(profile)) if profile else None,
        scene=scene,trajectory=dict(state='validated',receipt_digest=plan['timeline_digest'],tick_count=len(plan['ticks'])) if plan else None,
        pipeline=dict(schema_version='physical_collection_pipeline.v1',
            source_lane=dict(status='ready' if source else 'awaiting-observation',adapter='physical-rgbd',robot_id=config['robot_id'],scene_digest=scene['scene_digest'] if scene else None),
            target_lane=dict(status='planned' if plan else 'awaiting-scene',adapter='isaac',scene_digest=scene['scene_digest'] if scene else None),
            alignment='aligned' if plan else 'waiting',execution=execution,
            alignment_scope='同一份真实观测场景与冻结指令；视频帧率可不同',
            source_run_id=source['run_id'] if source else None,run_id=plan['run_id'] if plan else None,
            plan_run_id=plan['run_id'] if plan else None,timeline_digest=plan['timeline_digest'] if plan else None,
            tick_count=len(plan['ticks']) if plan else 0,stages=plan.get('stages',[]) if plan else [],
            recording=recording,dataset=dataset,execution_lanes=lanes,collection_complete=complete,
            timing=dict(status='independent-rendering'),scene_comparison=dict(status='unverified'),measured_scene_comparison=dict(status='unverified')),
        job=None,telemetry={'status':('running' if execution in ('running','awaiting_final_feedback') else 'complete') if rows else 'unavailable',
                           'source':dict(rows=rows,row_count=len(rows),latest=rows[-1] if rows else None,failure=rows[-1] if rows and rows[-1]['event'] else None,physical_feedback=actual,sample_age_s=owner.get('sample_age_s'),
                                         frozen=frozen if matching else None),
                           'target':dict(rows=target_rows[-180:],row_count=len(target_rows),latest=target_rows[-1] if target_rows else None,failure=None)},
        live_preview={'source':None,'target':None},pipeline_configured=bool(plan),evidence={},
        workflow=dict(mode=spec['mode'],environment=spec['environment'],environment_label=spec['label'],ready=not blockers and bool(plan),stages=stages,blockers=blockers,
                      execution_requires_operator_confirmation=True))
    if spec['environment']=='fake':
        result['pipeline']['schema_version']='fake_device_collection_pipeline.v1'
        result['pipeline']['source_lane']['adapter']='fake-device-rgbd'
        result['pipeline']['alignment_scope']='同一份fake设备观测与冻结指令；不代表真机验收'
        for value in result['adapters'].values():
            value['detail']=value['detail'].replace('真实','fake设备').replace('真机','fake设备')
        for stage in result['workflow']['stages']:
            stage['label']=stage['label'].replace('真实','fake设备').replace('真机','fake设备')
    return result


@lru_cache(maxsize=24)
def file_digest(path, modified_ns, changed_ns, size):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        if stream.read(8)[4:] != b'ftyp': raise ValueError('MP4 required')
        stream.seek(0)
        for chunk in iter(lambda:stream.read(1024*1024),b''): digest.update(chunk)
    return digest.hexdigest()


def playback(config):
    spec=namespace(config.get('environment','physical'))
    data = load(config)
    source,plan = data['source'],data['plan']
    empty = dict(clock='elapsed',status='unavailable',run_id=None,timeline_digest=None,
                 cameras={c:{'source':dict(available=False,url=None,revision=None),
                             'target':dict(available=False,url=None,revision=None),'ready':False} for c in CAMERAS})
    if not source or not plan or not config.get('artifact_dir'):
        return empty,{}
    root = (Path(config['artifact_dir'])/'playback').resolve()
    base = Path(config['artifact_dir']).resolve()
    if not root.is_relative_to(base): return empty,{}
    try:
        manifest_path = root/'manifest.json'
        if manifest_path.is_symlink(): raise ValueError('symlink manifest forbidden')
        manifest = read_object(manifest_path)
        source_only = config.get('render_replay', True) is False
        expected_files = {f'source-{c}.mp4' for c in CAMERAS} if source_only else FILES
        provenance = {'source':spec['evidence']} if source_only else {'source':spec['evidence'],'target':'simulation'}
        if (manifest.get('schema_version') != spec['playback_schema'] or manifest.get('mode') != spec['mode']
                or manifest.get('clock') != 'elapsed' or manifest.get('source_run_id') != source['run_id']
                or manifest.get('plan_run_id') != plan['run_id'] or manifest.get('timeline_digest') != plan['timeline_digest']
                or manifest.get('provenance') != provenance
                or set(manifest.get('files',{})) != expected_files):
            raise ValueError('physical replay identity/provenance differs from this run')
        if spec['environment']=='fake' and manifest.get('source_clock')!=source.get('clock'):
            raise ValueError('fake playback source clock differs from this run')
        paths = {}
        for name,record in manifest['files'].items():
            path = root/name
            if path.is_symlink() or not path.is_file() or path.stat().st_size != record.get('size_bytes'):
                raise ValueError('physical replay file missing or size differs')
            stat = path.stat()
            if file_digest(str(path),stat.st_mtime_ns,stat.st_ctime_ns,stat.st_size) != record.get('sha256'):
                raise ValueError('physical replay digest differs')
            paths[name] = path
        for camera in CAMERAS:
            for side in (('source',) if source_only else ('source','target')):
                path = paths[f'{side}-{camera}.mp4']
                empty['cameras'][camera][side] = dict(available=True,url=f'/api/playback/{path.name}',revision=f'{path.stat().st_mtime_ns}-{path.stat().st_size}')
            empty['cameras'][camera]['ready'] = True
        empty.update(status='ready',run_id=plan['run_id'],timeline_digest=plan['timeline_digest'])
        return empty,paths
    except (OSError,ValueError,TypeError,KeyError) as error:
        empty['error'] = str(error)
        return empty,{}


def live_preview(config, side, camera='base_0'):
    """Only the bound target renderer is local; source stays the live RGB-D service."""
    import time
    from live_preview import read_frame, camera_frame_path
    if side != 'target' or camera not in CAMERAS or not config.get('artifact_dir'):
        return None
    data = load(config)
    source,plan = data['source'],data['plan']
    if not source or not plan: return None
    root = Path(config['artifact_dir']).resolve()
    path = camera_frame_path(root/'replay-target',camera)
    if path.is_symlink() or not path.resolve().is_relative_to(root): return None
    try:
        frame = read_frame(path)
        if not frame: return None
        meta = dict(frame.metadata)
        if (meta.get('source_run_id') != source['run_id'] or meta.get('plan_run_id') != plan['run_id']
                or meta.get('timeline_digest') != plan['timeline_digest'] or meta.get('camera') != camera
                or meta.get('evidence_source') != 'simulation' or not frame.payload.startswith(b'\xff\xd8')):
            return None
        meta['age_s'] = max(0.,(time.time_ns()-int(meta['published_at_ns']))/1e9)
        return frame,meta
    except (OSError,ValueError,KeyError,TypeError):
        return None


def image_evidence(config, name):
    spec=namespace(config.get('environment','physical'))
    if name not in ('source.png','target.png','detection.png') or not config.get('artifact_dir'):
        return None
    data = load(config)
    source,plan = data['source'],data['plan']
    if not source or name == 'target.png' and not plan: return None
    root = Path(config['artifact_dir']).resolve()
    try:
        if (root/'evidence.json').is_symlink(): return None
        evidence = read_object(root/'evidence.json')
        if evidence.get('schema_version') != ('fake_device_collection_evidence.v1' if spec['environment']=='fake' else 'physical_collection_evidence.v1'): return None
        row = evidence['images'][name]
        if (row.get('source_run_id') != source['run_id'] or row.get('source_sequence') != source['sequence']
                or row.get('scene_digest') != source['scene']['scene_digest']
                or row.get('evidence_source') != ('simulation' if name == 'target.png' else spec['evidence'])
                or name == 'target.png' and row.get('plan_run_id') != plan['run_id']): return None
        path = root/row['path']
        if path.is_symlink() or not path.resolve().is_relative_to(root) or not path.is_file(): return None
        content = path.read_bytes()
        if not content.startswith(b'\x89PNG\r\n\x1a\n') or hashlib.sha256(content).hexdigest() != row.get('sha256'): return None
        return path
    except (OSError,ValueError,KeyError,TypeError):
        return None


def resolve_raw_source(config, owner_recording):
    """Resolve local sealed raw; an owner response cannot choose arbitrary files.

    The recorder and workbench may share one host. Legacy synchronized raw paths
    remain supported; local_recording_root explicitly trusts a task-owned root.
    Source/plan identity and every file digest are still checked by finalization.
    """
    if config.get('raw_source_dir'):
        return Path(config['raw_source_dir']).resolve(strict=True)
    root_value = config.get('local_recording_root')
    if not isinstance(root_value, str) or not root_value:
        raise ValueError('须配置 raw_source_dir 或真机本地 local_recording_root')
    directory = owner_recording.get('directory') if isinstance(owner_recording, dict) else None
    if not isinstance(directory, str) or not directory or owner_recording.get('done') is not True:
        raise ValueError('等待本轮本地 raw 目录封存')
    root, path = Path(root_value), Path(directory)
    for candidate in (root, path):
        if not candidate.is_absolute() or '..' in candidate.parts:
            raise ValueError('raw 根与目录必须是本地绝对路径，禁止路径逃逸')
        if any(item.is_symlink() for item in (candidate, *candidate.parents)):
            raise ValueError('raw 根与目录禁止符号链接')
    root, path = root.resolve(strict=True), path.resolve(strict=True)
    if not root.is_dir() or not path.is_dir() or path == root or not path.is_relative_to(root):
        raise ValueError('owner raw 目录必须位于 local_recording_root 内且不是根本身')
    sealed = path/'manifest.json'
    if sealed.is_symlink() or not sealed.is_file():
        raise ValueError('本地 raw 尚未封存，或 manifest 是符号链接')
    manifest = read_object(sealed)
    if (manifest.get('schema_version') != namespace(config.get('environment','physical'))['raw_schema']
            or type(manifest.get('sealed_wall_ns')) is not int or manifest['sealed_wall_ns'] <= 0
            or manifest.get('capture_status') not in ('complete', 'incomplete')):
        raise ValueError('本地 raw 缺少有效封存回执')
    return path


def finalize_collection(config, owner_recording, *, runner=None):
    """Finalize local evidence, either synchronized or recorded on this host.

    This runs in a job worker, not the HTTP/UI or SDK loop. A failed raw run can
    still have useful replay, but retains a failed finalization receipt.
    """
    import fcntl
    import subprocess
    import time
    from real_recording import export_playback,verify_raw
    from playback import sha256
    if not config.get('artifact_dir'):
        return dict(status='failed',error='artifact_dir 未配置',task_outcome='unverified')
    root=Path(config['artifact_dir']).resolve()
    root.mkdir(parents=True,exist_ok=True)
    if not isinstance(owner_recording,dict) or owner_recording.get('done') is not True:
        return dict(status='waiting-recording',error='等待控制服务完成 raw 封存',task_outcome='unverified')
    # A process lock prevents duplicate transcodes without a stale crash lock.
    with (root/'collection-finalization.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            return dict(status='finalizing',task_outcome='unverified')
        environment=config.get('environment','physical')
        receipt=dict(schema_version='fake_device_collection_finalization.v1' if environment=='fake' else 'physical_collection_finalization.v1',status='failed',
                     replay_ready=False,raw_complete=False,task_outcome='unverified',updated_wall_ns=time.time_ns())
        try:
            data=load(config)
            if data['blockers'] or not data['source'] or not data['plan']:
                raise ValueError('；'.join(data['blockers']) or '本轮真实 source/plan 尚未就绪')
            source,plan=data['source'],data['plan']
            receipt.update(source_run_id=source['run_id'],plan_run_id=plan['run_id'],timeline_digest=plan['timeline_digest'])
            raw_root=resolve_raw_source(config, owner_recording)
            environment=config.get('environment','physical')
            raw,recorded_source,recorded_plan=verify_raw(raw_root,environment=environment)
            if _digest(recorded_source)!=_digest(source) or _digest(recorded_plan)!=_digest(plan):
                raise ValueError('本地 raw 与当前真实 source/plan 不一致')
            reported=owner_recording.get('manifest') or {}
            keys=('source_run_id','plan_run_id','timeline_digest','capture_status','controller_outcome',
                  'command_count','feedback_count','camera_frames','validation')
            if any(reported.get(key)!=raw.get(key) for key in keys):
                raise ValueError('本地 raw 与控制服务终态回执不一致')
            raw_hash=sha256(raw_root/'manifest.json')
            receipt.update(raw_manifest_sha256=raw_hash,capture_status=raw['capture_status'],
                           validation=raw['validation'],controller_outcome=raw['controller_outcome'],
                           controller_reason=raw.get('controller_reason'),recording_error=owner_recording.get('error'))
            raw_complete=(raw.get('capture_status')=='complete' and raw.get('validation',{}).get('status')=='passed'
                          and raw.get('controller_outcome')=='completed' and not owner_recording.get('error'))
            receipt['raw_complete']=raw_complete
            source_only = config.get('render_replay', True) is False
            target = None if source_only else root/'target-playback/manifest.json'
            if target is not None and (target.is_symlink() or not target.resolve().is_relative_to(root)):
                raise ValueError('target replay 必须位于本轮 artifact_dir 内')
            target_hash=sha256(target) if target is not None else None
            output=root/'playback'
            if output.exists():
                existing,paths=playback(config)
                if existing['status']!='ready' or len(paths)!=(3 if source_only else 6):
                    raise ValueError('已有未完成或无效回放目录；保留失败产物，未覆盖重试')
                manifest=read_object(output/'manifest.json')
                if manifest.get('raw_manifest_sha256')!=raw_hash or manifest.get('target_manifest_sha256')!=target_hash:
                    raise ValueError('已有回放与本轮 raw/target manifest 不一致')
            else:
                manifest=export_playback(raw_root,target,output,runner=runner or subprocess.run,environment=environment)
            if config.get('dataset_format'):
                if environment!='physical' or config['dataset_format']!='lerobot-v2.1':
                    raise ValueError('当前保存链仅支持 physical 的 lerobot-v2.1')
                receipt['dataset_ready']=False
                if raw_complete:
                    from robot_workbench.recording_process import export_lerobot_process
                    dataset=export_lerobot_process(raw_root,root/'lerobot',
                        task=config.get('task_description') or '将杯子从桌面放到咖啡机托盘')
                    if (dataset.get('status')!='NATIVE_READBACK_PASS' or len(dataset.get('episodes',[]))!=1
                            or dataset['episodes'][0].get('raw_manifest_sha256')!=raw_hash):
                        raise ValueError('LeRobot 保存验收与本轮原始记录不一致')
                    receipt.update(dataset_ready=True,dataset_format=dataset['format_version'],
                        dataset_root=dataset['root'],dataset_receipt=str(Path(dataset['root'])/'audit/export-receipt.json'))
            receipt.update(status='ready' if raw_complete else 'failed',replay_ready=True,raw_complete=raw_complete,
                           playback_manifest=str(output/'manifest.json'),target_manifest_sha256=target_hash)
            if not raw_complete:
                receipt['error']=owner_recording.get('error') or raw.get('controller_reason') or 'raw 未通过完整性验收'
        except (OSError,ValueError,TypeError,KeyError,subprocess.SubprocessError) as error:
            receipt['error']=str(error)
        temporary=root/'collection-finalization.json.partial'
        temporary.write_text(json.dumps(receipt,allow_nan=False,indent=2)+'\n')
        temporary.replace(root/'collection-finalization.json')
        return receipt
