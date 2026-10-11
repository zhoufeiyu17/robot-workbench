"""Pinned, read-only completed fake recordings; never a Playground resume input."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re
import shutil
from urllib.parse import urlencode

CAMERAS=('base_0','left_wrist_0','right_wrist_0')
SCHEMA='collection_history_source.v1'


def _file(root,value):
    path=Path(value)
    if not path.is_absolute():path=root/path
    if ('..' in path.parts or any(part.is_symlink() for part in (path,*path.parents))
            or not path.resolve().is_relative_to(root) or not path.is_file() or path.stat().st_size<=0):
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
