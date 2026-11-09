"""Pinned read-only history; qualified fake imports and recording archives stay distinct."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from functools import lru_cache
import re
import shutil
import os
from urllib.parse import urlencode

CAMERAS=('base_0','left_wrist_0','right_wrist_0')
SCHEMA='collection_history_source.v1'


def _file(root,value,*,allow_empty=False):
    path=Path(value)
    if not path.is_absolute():path=root/path
    if ('..' in path.parts or any(part.is_symlink() for part in (path,*path.parents))
            or not path.resolve().is_relative_to(root) or not path.is_file() or not allow_empty and path.stat().st_size<=0):
        raise ValueError('history file must be a nonempty regular file within its declared run: '+str(path))
    return path


def _pin(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return dict(sha256=digest.hexdigest(),size_bytes=path.stat().st_size)


def _inspect(root,jobs):
    """Read exact export/receipt paths; no raw-frame, model or credential discovery."""
    files={};entries=[]
    def read(path):
        path=_file(root,path)
        if path.stat().st_size>32*1024*1024:raise ValueError('history metadata exceeds 32 MiB')
        files[str(path.relative_to(root))]=_pin(path)
        return json.loads(path.read_bytes())
    for relative in jobs:
        job=root/relative
        if (Path(relative).is_absolute() or '..' in Path(relative).parts or len(Path(relative).parts)!=4
                or job.parent.name!='jobs' or job.parent.parent.name!='playground'
                or not re.fullmatch('[0-9a-f]{32}',job.name) or not Path(relative).parts[0].startswith('collection-')):
            raise ValueError('history requires an exact resident collection/playground/jobs task')
        collection=job.parents[2]
        exported=read(job/'physical-video/manifest.json');simulation=read(job/'simulation-result.json')
        verification=read(job/'native-task-verification.json');instance=read(job/'execution-instance.json')
        bridge=read(collection/'calibration/bridge.json');binding=read(collection/'calibration/acquisition-binding.json')
        raw_root=Path(read(job/'recording-directory.json')['directory'])
        if (not raw_root.is_absolute() or not raw_root.is_relative_to(root/'collection-raw')
                or raw_root.name!='raw'):
            raise ValueError('historical raw metadata must belong to this resident recording root')
        raw=read(raw_root/'manifest.json');raw_binding=read(raw_root/'binding.json');source=read(raw_root/'source.json')
        execution=instance.get('execution_instance_id')
        if not isinstance(execution,str) or not re.fullmatch('[0-9a-f]{32}',execution):
            raise ValueError('historical execution instance missing')
        native_path=root/'native-tasks'/(execution+'.json');native=read(native_path)
        clock=raw.get('clock');robot=raw.get('robot_id');proof=verification.get('native_task') or {}
        if (not isinstance(clock,dict) or clock.get('environment')!='fake' or clock.get('clock_domain')!='simulation'
                or not isinstance(clock.get('source_clock_id'),str) or not clock['source_clock_id']
                or not isinstance(robot,str) or not robot.startswith('fake-')
                or raw.get('environment')!='fake' or raw.get('simulation_only') is not True
                or raw.get('source_clock_id')!=clock['source_clock_id'] or source.get('clock')!=clock
                or source.get('robot_id')!=robot or exported.get('source_clock')!=clock
                or exported.get('environment')!='fake' or exported.get('simulation_only') is not True
                or binding.get('robot_id')!=robot or binding.get('environment')!='fake'
                or binding.get('simulation_only') is not True
                or any(raw_binding.get(key)!=value for key,value in binding.items())
                or source.get('camera_serials')!=binding.get('camera_serials')):
            raise ValueError('historical robot, camera binding or source clock differs')
        installation=dict(head_id=bridge.get('head_id'),wrist_id=bridge.get('wrist_id'),
            manifest_sha256=(bridge.get('active_sha256') or {}).get('manifest.json'))
        if (any(not isinstance(value,str) or not value for value in installation.values())
                or binding.get('head_id')!=installation['head_id'] or binding.get('wrist_id')!=installation['wrist_id']
                or binding.get('calibration_snapshot_id')!=installation['manifest_sha256']
                or bridge.get('calibration_snapshot_id')!=installation['manifest_sha256']):
            raise ValueError('historical installed calibration identity differs')
        timeline=raw.get('timeline_digest');executed=raw.get('execution_timeline_digest')
        if any(not isinstance(value,str) or not re.fullmatch('[0-9a-f]{64}',value) for value in (timeline,executed)):
            raise ValueError('historical timeline identities missing')
        expected=dict(execution_instance_id=execution,source_clock_id=clock['source_clock_id'],
            execution_timeline_digest=executed,runtime_nonce=instance.get('runtime_nonce'))
        if (any(native.get(key)!=value or proof.get(key)!=value for key,value in expected.items())
                or native.get('command_epoch')!=instance.get('command_epoch')
                or any((raw.get('execution_owner') or {}).get(key)!=instance.get(key) for key in ('runtime_nonce','command_epoch'))
                or native.get('parent_timeline_digest')!=timeline
                or native.get('source_digest')!=hashlib.sha256(json.dumps(source,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
                or proof.get('receipt_path')!=str(native_path) or proof.get('receipt_sha256')!=_pin(native_path)['sha256']
                or proof.get('receipt_verified') is not True or proof.get('scope')!='L2_same_plant'
                or native.get('schema_version')!='owner_physx_native_task.v1'
                or native.get('environment')!='fake' or native.get('simulation_only') is not True
                or native.get('hardware_qualified') is not False):
            raise ValueError('historical native task receipt differs from the original execution')
        if (raw.get('controller_outcome')!='completed' or raw.get('capture_status')!='complete'
                or (raw.get('validation') or {}).get('status')!='passed'
                or exported.get('controller_outcome')!='completed' or exported.get('capture_status')!='complete'
                or exported.get('raw_manifest_sha256')!=_pin(raw_root/'manifest.json')['sha256']
                or verification.get('task_outcome')!='success' or native.get('task_outcome')!='success'
                or native.get('motion_outcome')!='completed' or native.get('task_qualified') is not True
                or proof.get('task_outcome')!='success' or proof.get('task_qualified') is not True
                or simulation.get('status')!='completed' or simulation.get('scope')!='L3_auxiliary_independent_target'
                or simulation.get('same_plant') is not False or simulation.get('task_success_evidence') is not False
                or simulation.get('execution_timeline_digest')!=executed):
            raise ValueError('history import requires the completed, separately verified fake task and L3 replay')
        for artifact in (exported,simulation):
            if any(artifact.get(key)!=raw.get(key) for key in ('source_run_id','plan_run_id','timeline_digest')):
                raise ValueError('historical export belongs to another observation or plan')
        for name in ('source.json','binding.json'):
            if (raw.get('files') or {}).get(name)!=_pin(raw_root/name):
                raise ValueError('historical raw metadata pin differs')
        detail=dict(readonly=True,source_run=str(root),job_id=job.name,robot_id=robot,source_clock=clock,
            installation=installation,**expected,timeline_digest=timeline,
            outcomes=dict(motion='completed',data='valid',task='success',simulation_scope='L3_auxiliary_independent_target'),
            qualification='original completed fake evidence; file/identity verification only, no new execution or raw audit')
        for role,manifest in (('source',exported),('target',simulation)):
            for camera in CAMERAS:
                name=('source-'+camera if role=='source' else camera)+'.mp4'
                path=_file(root,job/'physical-video'/name if role=='source' else job/('sim-'+camera+'.mp4'))
                pin=_pin(path);declared=(manifest.get('files') or {}).get(name) or {}
                if any(pin[key]!=declared.get(key) for key in pin):raise ValueError('historical video SHA or size differs')
                relative_file=str(path.relative_to(root));files[relative_file]=pin
                entries.append(dict(run_id=str(job),file=relative_file,camera=camera,role=role,
                    label='历史 '+job.name[:8]+' · '+('同 plant 源录像 ' if role=='source' else 'L3 辅助回放 ')+camera,
                    details=dict(detail,source_file=str(path),camera=camera,role=role,**pin)))
    return files,entries


def source_spec(root,jobs):
    root=Path(root)
    if not root.is_absolute() or root.is_symlink() or root.resolve()!=root or not root.is_dir():
        raise ValueError('history source must be a canonical absolute resident run')
    relative=[]
    for job in jobs:
        path=Path(job)
        relative.append(str(path.relative_to(root)) if path.is_absolute() else str(path))
    if not relative or len(relative)!=len(set(relative)):raise ValueError('explicit distinct completed history jobs required')
    files,_=_inspect(root,relative)
    return dict(schema=SCHEMA,source_run=str(root),jobs=relative,files=files)


def validate_source(spec,*,robot_id=None):
    if not isinstance(spec,dict) or set(spec)!={'schema','source_run','jobs','files'} or spec['schema']!=SCHEMA:
        raise ValueError('explicit collection history source specification required')
    root=Path(spec['source_run'])
    if not isinstance(spec['jobs'],list) or not all(isinstance(job,str) for job in spec['jobs']):
        raise ValueError('history jobs must be an explicit list')
    expected=source_spec(root,spec['jobs'])
    if expected!=spec:raise ValueError('collection history file set, SHA or size changed')
    _,entries=_inspect(root,spec['jobs'])
    if robot_id is not None and any(row['details']['robot_id']!=robot_id for row in entries):
        raise ValueError('history belongs to another virtual robot')
    return dict(spec=deepcopy(spec),entries=entries)


def import_history(validated,output):
    spec=validated['spec'];checked=validate_source(spec)
    if checked!=validated:raise ValueError('validated history identity changed')
    root=Path(output)
    if (not root.is_absolute() or root.resolve()!=root or root.exists()
            or root.is_relative_to(Path(spec['source_run']))):
        raise ValueError('history destination must be a new independent directory')
    root.mkdir(parents=True)
    for relative,pin in spec['files'].items():
        source=_file(Path(spec['source_run']),relative);target=root/relative
        target.parent.mkdir(parents=True,exist_ok=True)
        with source.open('rb') as src,target.open('xb') as dst:shutil.copyfileobj(src,dst)
        if _pin(target)!=pin or _pin(source)!=pin:raise ValueError('history file changed during copy')
    with (root/'history.json').open('x') as stream:json.dump(checked,stream,ensure_ascii=False,allow_nan=False,indent=2)
    return CollectionHistory(root)


class CollectionHistory:
    def __init__(self,root):
        self.root=Path(root).resolve();saved=json.loads(_file(self.root,'history.json').read_bytes())
        self.catalog=[];self.files={}
        for row in saved['entries']:
            pin=saved['spec']['files'][row['file']];path=_file(self.root,row['file'])
            if _pin(path)!=pin:raise ValueError('copied historical video changed')
            identity='history-'+hashlib.sha256((row['run_id']+'\0'+row['file']+'\0'+pin['sha256']).encode()).hexdigest()
            self.files[identity]=(row['file'],pin)
            self.catalog.append(dict(id=identity,run_id=row['run_id'],label=row['label'],kind='recording',status='completed',
                readonly=True,details=row['details'],media=dict(id=identity,label=row['label'],run_id=row['run_id'],
                    kind='replay',ready=True,mime_type='video/mp4',camera=row['camera'],role=row['role'],
                    url='/api/result-media?'+urlencode(dict(environment='fake',business='collection',result_id=identity)))))

    def results(self):return deepcopy(self.catalog)

    def media(self,result_id):
        if result_id not in self.files:return None
        relative,pin=self.files[result_id];path=_file(self.root,relative)
        if _pin(path)!=pin:raise ValueError('historical media SHA or size changed')
        return path


class CollectionHistoryGroup:
    """Read retained immutable catalogs without copying videos at every restart."""
    def __init__(self,histories):
        self.histories=tuple(histories)
        rows={}
        for history in self.histories:
            for row in history.results():
                if row['id'] in rows and row!=rows[row['id']]:
                    raise ValueError('conflicting historical result identity')
                rows[row['id']]=row
        self.catalog=list(rows.values())

    def results(self):return deepcopy(self.catalog)

    def media(self,result_id):
        for history in self.histories:
            path=history.media(result_id)
            if path is not None:return path
        return None

    def quality_report(self,result_id):
        for history in self.histories:
            reader=getattr(history,'quality_report',None)
            report=reader(result_id) if reader else None
            if report is not None:return report
        return None


def inspect_recording(root, raw_directory, playback_directory, *, environment, robot_id):
    """Offline, exact paths only. Incomplete recordings retain their original verdict."""
    from real_recording import verify_raw
    from real_contract import namespace
    root=Path(root)
    if not root.is_absolute() or root.resolve()!=root or root.is_symlink() or environment not in ('fake','physical'):
        raise ValueError('explicit canonical recording run and environment required')
    raw_manifest=_file(root,Path(raw_directory)/'manifest.json')
    playback_manifest=_file(root,Path(playback_directory)/'manifest.json')
    raw,source,_=verify_raw(raw_manifest.parent,environment=environment)
    exported=json.loads(playback_manifest.read_bytes())
    spec=namespace(environment)
    if (exported.get('schema_version')!=spec['playback_schema'] or exported.get('mode')!=spec['mode']
            or (exported.get('provenance') or {}).get('source')!=spec['evidence']
            or environment=='fake' and (exported.get('environment')!='fake' or exported.get('simulation_only') is not True)
            or environment=='physical' and (exported.get('environment','physical')!='physical' or exported.get('simulation_only') is True)
            or source['robot_id']!=robot_id or raw.get('robot_id')!=robot_id
            or exported.get('raw_manifest_sha256')!=_pin(raw_manifest)['sha256']
            or exported.get('source_run_id')!=raw['source_run_id']
            or exported.get('controller_outcome')!=raw.get('controller_outcome')):
        raise ValueError('recording archive device, raw or playback identity differs')
    if raw.get('recording_kind')!='streaming_policy' and any(exported.get(key)!=raw.get(key)
            for key in ('plan_run_id','timeline_digest') if key in raw):
        raise ValueError('recording archive playback belongs to another frozen execution')
    files={str(path.relative_to(root)):_pin(path) for path in
        (raw_manifest,_file(root,raw_manifest.parent/'source.json'),playback_manifest)}
    videos=[]
    for camera in CAMERAS:
        name='source-'+camera+'.mp4'
        path=_file(root,playback_manifest.parent/name);pin=_pin(path)
        if pin!={key:exported.get('files',{}).get(name,{}).get(key) for key in pin}:
            raise ValueError('recording archive video SHA or size differs')
        relative=str(path.relative_to(root));files[relative]=pin
        videos.append(dict(camera=camera,file=relative))
    return dict(source_root=str(root),raw_directory=str(raw_manifest.parent.relative_to(root)),
        playback_directory=str(playback_manifest.parent.relative_to(root)),
        raw_manifest_sha256=_pin(raw_manifest)['sha256'],environment=environment,robot_id=robot_id,
        source_run_id=raw['source_run_id'],controller_outcome=raw.get('controller_outcome'),
        capture_status=raw['capture_status'],files=files,videos=videos)


def register_recording(root, raw_directory, playback_directory, output, *, environment, robot_id):
    """Create a new small catalog/report; never copy, move or rewrite source bytes."""
    from .recording_quality import generate, REPORT
    checked=inspect_recording(root,raw_directory,playback_directory,environment=environment,robot_id=robot_id)
    output=Path(output)
    if (not output.is_absolute() or output.resolve()!=output or output.exists()
            or output.is_relative_to(Path(root))):
        raise ValueError('recording archive requires a new independent output directory')
    # generate performs the independent raw verification in this offline worker.
    report=generate(Path(root)/checked['raw_directory'],output,environment=environment)
    if report['raw_manifest_sha256']!=checked['raw_manifest_sha256']:
        raise ValueError('recording changed during archive registration')
    for relative,pin in checked['files'].items():
        if _pin(_file(Path(root),relative))!=pin:
            raise ValueError('recording archive input changed during registration')
    relative='quality/'+checked['raw_manifest_sha256']+'/'+REPORT
    value=dict(schema='recording_history.v1',**checked,quality_file=relative,
        quality_pin=_pin(_file(output,relative)),readonly=True,
        qualification='archived recording; does not promote raw, motion or task qualification')
    with (output/'history.json').open('x') as stream:
        json.dump(value,stream,ensure_ascii=False,allow_nan=False,indent=2)
    return RecordingHistory(output,environment=environment,robot_id=robot_id)


class RecordingHistory:
    """A trusted deployment-selected catalog, including failed/incomplete recordings."""
    def __init__(self,root,*,environment,robot_id):
        from .recording_quality import _json,presentation,validate_report,SCHEMA as QUALITY_SCHEMA
        self.root=Path(root).resolve();saved=_json(_file(self.root,'history.json'))
        if (saved.get('schema')!='recording_history.v1' or saved.get('environment')!=environment
                or saved.get('robot_id')!=robot_id or saved.get('readonly') is not True):
            raise ValueError('recording history belongs to another device or environment')
        source=Path(saved['source_root'])
        if not source.is_absolute() or source.resolve()!=source or source.is_symlink():
            raise ValueError('recording history requires its declared canonical run')
        quality_error=None
        try:
            report_path=_file(self.root,saved['quality_file'])
            if _pin(report_path)!=saved['quality_pin']:
                raise ValueError('archived quality report changed')
            report=_json(report_path)
            if (report.get('schema')!=QUALITY_SCHEMA or report.get('raw_manifest_sha256')!=saved['raw_manifest_sha256']
                    or report.get('environment')!=environment or report.get('robot_id')!=robot_id):
                raise ValueError('archived quality belongs to a different recording')
            validate_report(report,manifest_sha256=saved['raw_manifest_sha256'])
        except (OSError,ValueError,TypeError,KeyError) as error:
            report=None;quality_error=str(error)[:700]
        changed=[]
        video_names={row['file'] for row in saved['videos']}
        for relative,pin in saved['files'].items():
            if relative in video_names:continue
            try:
                if _pin(_file(source,relative))!=pin:changed.append(relative)
            except (OSError,ValueError):changed.append(relative)
        self.catalog=[];self.files={};self.source=source
        self.quality_path=report_path if report is not None else None
        self.quality_pin=saved['quality_pin']
        for video in saved['videos']:
            relative=video['file'];pin=saved['files'][relative];error=None
            try:
                if _pin(_file(source,relative))!=pin:raise ValueError('archived video SHA or size changed')
            except (OSError,ValueError) as exc:error=str(exc)
            identity='recording-history-'+hashlib.sha256((saved['raw_manifest_sha256']+'\0'+relative+'\0'+pin['sha256']).encode()).hexdigest()
            view=presentation(report,video_ready=error is None,controller_outcome=saved['controller_outcome'])
            if quality_error:view['quality_error']=quality_error
            if changed:
                view['dimensions']['data']=dict(status='requires_review',label='源证据已变化，需复核')
                view['changed_evidence']=changed
            if error:view['video_error']=error
            if error is None:self.files[identity]=(relative,pin)
            label='历史录制 · '+{'base_0':'头部','left_wrist_0':'左腕','right_wrist_0':'右腕'}[video['camera']]
            self.catalog.append(dict(id=identity,run_id=saved['source_run_id'],kind='recording',label=label,
                status='completed' if saved['capture_status']=='complete' and saved['controller_outcome']=='completed' else 'failed',
                readonly=True,recording_quality=view,details=dict(robot_id=robot_id,environment=environment,
                    raw_manifest_sha256=saved['raw_manifest_sha256'],qualification=saved['qualification'],
                    original_controller_outcome=saved['controller_outcome'],original_capture_status=saved['capture_status']),
                media=dict(id=identity,run_id=saved['source_run_id'],kind='replay',ready=error is None,
                    role='source',camera=video['camera'],mime_type='video/mp4',label=label,
                    url='/api/result-media?'+urlencode(dict(environment=environment,business='collection',result_id=identity)))))

    def results(self):return deepcopy(self.catalog)

    def quality_report(self,result_id):
        from .recording_quality import _json
        if self.quality_path is None or not any(row['id']==result_id for row in self.catalog):return None
        if _pin(self.quality_path)!=self.quality_pin:raise ValueError('archived quality report changed')
        report=_json(self.quality_path)
        if _pin(self.quality_path)!=self.quality_pin:raise ValueError('archived quality report changed during read')
        return dict(report=report,sidecar_sha256=self.quality_pin['sha256'])

    def media(self,result_id):
        if result_id not in self.files:return None
        relative,pin=self.files[result_id];path=_file(self.source,relative)
        if _pin(path)!=pin:raise ValueError('archived video SHA or size changed')
        return path


def load_history(root,*,environment,robot_id):
    """Keep the original qualified-fake validator unchanged for its own schema."""
    path=Path(root).resolve();saved=json.loads(_file(path,'history.json').read_bytes())
    if saved.get('schema')=='recording_history.v1':
        return RecordingHistory(path,environment=environment,robot_id=robot_id)
    if environment!='fake':raise ValueError('qualified fake history cannot enter physical results')
    history=CollectionHistory(path)
    if any(row['details']['robot_id']!=robot_id for row in history.results()):
        raise ValueError('retained history belongs to another virtual robot')
    return history


@lru_cache(maxsize=128)
def _capture_started_wall_ns(manifest_path,expected_digest):
    """Bounded metadata projection for both new and existing immutable Runs."""
    try:
        path=Path(manifest_path)
        if path.stat().st_size>8*1024**2 or _pin(path)['sha256']!=expected_digest:return None
        manifest=json.loads(path.read_text());activity=manifest.get('recording_activity')
        if activity is not None:return activity['intervals'][0]['start_wall_ns']if activity.get('intervals')else None
        streams=list((manifest.get('playback')or{}).get('streams',{}))or list(manifest.get('streams',{}))
        starts=[]
        for stream in streams:
            try:
                source=_file(path.parent,stream+'.jsonl',allow_empty=True)
                with source.open('rb')as data:line=data.readline(512*1024+1)
                if not line.endswith(b'\n')or len(line)>512*1024:continue
                row=json.loads(line);reference=row.get('payload',{}).get('metadata',{}).get('ros_reference')or{}
                stamp=reference.get('source_time_ns')if reference.get('source_clock_id')=='host_wall'else row.get('recorded_wall_ns')
                if type(stamp)is int and stamp>0:starts.append(stamp)
            except (OSError,ValueError,KeyError,TypeError):continue
        return min(starts)if starts else None
    except (OSError,ValueError,KeyError,TypeError):return None


class CaptureResults:
    """Business-side, bounded-page projection of sealed common-control Runs."""
    BUSINESSES=('calibration','collection','research')

    def __init__(self,root,*,environment,device_id,inference_history=None):
        import queue
        import threading
        from .inference_history import InferenceHistory
        raw=Path(root)
        if not raw.is_absolute()or raw.is_symlink()or environment not in('fake','physical')or not device_id:
            raise ValueError('declared device/environment capture root required')
        self.root=raw.resolve();self.root.mkdir(parents=True,exist_ok=True)
        self.environment,self.device_id=environment,device_id
        import fcntl
        from .inference_trace import write_json
        identity=dict(environment=environment,device_id=device_id)
        with (self.root/'capture-index.lock').open('a')as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            source=self.root/'capture-index-identity.json'
            if source.exists():
                if self._read(_file(self.root,source))!=identity:raise ValueError('capture index belongs to another device/environment')
            else:write_json(source,identity)
        self.histories=({'inference':inference_history} if inference_history is not None else
            {business:InferenceHistory(self.root,database='capture-'+business+'.sqlite3')for business in self.BUSINESSES})
        self.queue=queue.Queue(maxsize=64);self.closed=threading.Event();self.rescan=threading.Event();self.rescan.set()
        self.error=None;self.thread=threading.Thread(target=self._run,daemon=True,name='capture-result-index');self.thread.start()

    def notify(self,value=None):
        import queue
        if self.closed.is_set():return
        if value is None:self.rescan.set();return
        try:self.queue.put_nowait(deepcopy(value))
        except queue.Full:self.rescan.set();self.error='capture result index queue full; sealed files retained for reconciliation'

    def _read(self,path):
        if path.stat().st_size>8*1024**2:raise ValueError('bounded capture receipt/manifest required')
        return json.loads(path.read_text())

    def _checked_ref(self,ref):
        if not isinstance(ref,dict)or set(ref)!={'path','size_bytes','sha256'}:raise ValueError('sealed artifact pin required')
        path=_file(self.root,ref['path'])
        if _pin(path)!={key:ref[key]for key in('size_bytes','sha256')}:raise ValueError('sealed artifact changed')
        return path

    def register(self,event):
        required={'execution_instance_id','business_kind','seal_revision','receipt_ref','manifest_ref'}
        if not isinstance(event,dict)or set(event)!=required:raise ValueError('declared seal notification required')
        run,business,revision=(event[k]for k in('execution_instance_id','business_kind','seal_revision'))
        if (not isinstance(run,str)or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',run)or business not in self.histories
                or type(revision)is not int or revision<0):raise ValueError('known capture Run/revision required')
        path=self._checked_ref(event['receipt_ref'])
        run_root=self.root/(('inference-'if business=='inference'else'')+run)
        if path!=run_root/f'receipt-r{revision:06d}.json':raise ValueError('receipt belongs to another Run/revision')
        history=self.histories[business];identity='capture-'+run+'-receipt';known=history.result(identity)
        if known and known['details']['seal_revision']>revision:return False
        checked_manifest=self._checked_ref(event['manifest_ref'])if event['manifest_ref']is not None else None
        if known and known['details']['seal_revision']==revision and known['details']['receipt_pin']==_pin(path):
            if known['details']['manifest_pin']!=(_pin(checked_manifest)if checked_manifest else None):raise ValueError('same sealed manifest changed')
            return False
        receipt=self._read(path);context=receipt.get('execution_context')or{}
        if (receipt.get('schema')!='workbench.capture-run.v1'or receipt.get('execution_instance_id')!=run
                or receipt.get('business_kind')!=business or receipt.get('seal_revision')!=revision
                or context.get('device_id')!=self.device_id or context.get('environment')!=self.environment):
            raise ValueError('capture result belongs to another device, environment, business or revision')
        policy=receipt['control_policy']['policy_hash']
        if known and known['details']['policy_hash']!=policy:raise ValueError('capture Run policy changed')
        manifest={};manifest_path=None
        if event['manifest_ref']is not None:
            manifest_path=checked_manifest;manifest=self._read(manifest_path)
            if (manifest.get('schema')!='workbench.capture-manifest.v1'or manifest.get('capture_session_id')!=run
                    or manifest_path.name!=f'manifest-r{revision:06d}.json'or not manifest_path.is_relative_to(run_root)
                    or manifest.get('seal_revision')!=revision or (manifest.get('binding')or{}).get('run_id')!=run
                    or manifest['binding'].get('business_kind')!=business or manifest['binding'].get('policy_hash')!=policy):
                raise ValueError('capture manifest identity changed')
            for relative,pin in manifest.get('files',{}).items():
                if _pin(_file(manifest_path.parent,relative,allow_empty=True))!=pin:raise ValueError('capture member changed')
        elif revision!=0:raise ValueError('sealed capture manifest missing')
        pin=_pin(path);outcome=(receipt.get('recording')or{}).get('recording_outcome','incomplete')
        controller=receipt.get('controller')or{};base=dict(seal_revision=revision,policy_hash=policy,
            device_id=self.device_id,environment=self.environment,
            recording_outcome=outcome,controller_outcome=controller.get('status'),receipt_pin=pin,
            manifest_pin=_pin(manifest_path)if manifest_path else None)
        if manifest_path:base['started_wall_ns']=_capture_started_wall_ns(str(manifest_path),base['manifest_pin']['sha256'])
        url=lambda ident:'/api/result-media?'+urlencode(dict(environment=self.environment,business=business,result_id=ident))
        row=dict(id=identity,run_id=run,kind='control_recording',label=f'控制录制 · {business} · {outcome}',
            status=controller.get('status','unknown'),download_url=url(identity),details=dict(base,file_pin=pin))
        rows=[(row,path)]
        playback=manifest.get('playback')or{}
        for stream,video in playback.get('streams',{}).items():
            video_path=_file(manifest_path.parent,video['path']);video_pin=manifest['files'].get(video['path'])
            if not video_pin or _pin(video_path)!=video_pin:raise ValueError('capture playback changed')
            ident='capture-'+run+'-'+stream
            rows.append((dict(id=ident,run_id=run,kind='replay',label=stream,status=outcome,
                details=dict(base,file_pin=video_pin),media=dict(id=ident,run_id=run,kind='replay',role='source',
                    camera=stream.removeprefix('video-'),ready=True,mime_type='video/mp4',url=url(ident))),video_path))
        if _pin(path)!={k:event['receipt_ref'][k]for k in('size_bytes','sha256')}:raise ValueError('receipt changed during registration')
        if manifest_path and _pin(manifest_path)!={k:event['manifest_ref'][k]for k in('size_bytes','sha256')}:raise ValueError('manifest changed during registration')
        return False if self.closed.is_set()else history.replace_capture_run(rows,preserve_other_results=business=='inference')

    def _scan(self):
        def receipts():
            with os.scandir(self.root)as directories:
                for directory in directories:
                    if not directory.is_dir(follow_symlinks=False):continue
                    with os.scandir(directory.path)as entries:
                        for entry in entries:
                            if re.fullmatch(r'receipt-r\d{6}\.json',entry.name)and entry.is_file(follow_symlinks=False):yield Path(entry.path)
        for path in receipts():
            if self.closed.is_set():break
            try:
                receipt=self._read(_file(self.root,path));manifest=(receipt.get('recording')or{}).get('directory')
                revision=receipt.get('seal_revision',0);mp=Path(manifest)/f'manifest-r{revision:06d}.json'if manifest else None
                self.register(dict(execution_instance_id=receipt['execution_instance_id'],business_kind=receipt['business_kind'],
                    seal_revision=revision,receipt_ref=dict(path=str(path),**_pin(path)),
                    manifest_ref=dict(path=str(mp),**_pin(_file(self.root,mp)))if mp and mp.is_file()else None))
            except (OSError,ValueError,KeyError,TypeError)as error:self.error=str(error)

    def _run(self):
        import queue
        while not self.closed.is_set():
            try:
                if self.rescan.is_set():self.rescan.clear();self._scan()
                try:value=self.queue.get(timeout=.1)
                except queue.Empty:continue
                try:self.register(value)
                finally:self.queue.task_done()
            except Exception as error:self.error=str(error)

    def page(self,business,*,before=None,after=None):
        page=self.histories[business].page(before=before,after=after)
        for row in page['results']:
            details=row.get('details')or{};revision=details.get('seal_revision');run=row.get('run_id')
            if 'started_wall_ns'in details or revision is None or not run:continue
            try:
                receipt=self._read(_file(self.root,('inference-'if business=='inference'else'')+run+f'/receipt-r{revision:06d}.json'))
                directory=(receipt.get('recording')or{}).get('directory')
                if directory:
                    manifest=_file(self.root,str(Path(directory)/f'manifest-r{revision:06d}.json'))
                    details['started_wall_ns']=_capture_started_wall_ns(str(manifest),details['manifest_pin']['sha256'])
            except (OSError,ValueError,KeyError,TypeError):pass
        return page

    def media(self,business,identity):
        row=self.histories[business].result(identity)
        if row is None:return None
        path=self.histories[business].media(identity)
        if path is None or _pin(_file(self.root,path))!=row['details']['file_pin']:raise ValueError('indexed capture media changed')
        return path

    def close(self):
        # ponytail: bounded shutdown; an interrupted derived index is rebuilt
        # from immutable sealed receipts on the next business-service start.
        self.closed.set();self.thread.join(1)
