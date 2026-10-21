"""Immutable robot-local calibration snapshots, independent of raw captures."""
from contextlib import contextmanager
from copy import deepcopy
from datetime import datetime, timezone
import fcntl
import hashlib
import hmac
import json
import os
from pathlib import Path
import re
import secrets
import shutil
import stat
import tempfile
import threading


_LOCK = threading.RLock()
_SCHEMA = 'joint-head-calibration-store-v1'
# The one installed calibration lives here: `<library>/active/`, self-contained.
_ACTIVE = 'active'
_ACTIVE_SCHEMA = 'joint-head-calibration-active-v1'
_ACTIVE_TMP = '.active.staging-'
_ACTIVE_SUPERSEDED = '.active.superseded-'
# One list, used by both the writer and the reader, so a new kind can never be
# accepted on write and rejected on read.
_KINDS = ('head', 'left', 'right', 'wrist')


def _identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,199}', value):
        raise ValueError('invalid calibration or robot identifier')
    return value


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode('utf-8')


class CalibrationStore:
    def __init__(self, root, robot_id, environment=None):
        self.root = Path(root)
        self.robot_id = _identifier(robot_id)
        if environment not in (None,'simulation','hardware'):raise ValueError('invalid environment')
        self.environment=environment
        if not self.root.is_absolute() or '..' in self.root.parts:
            raise ValueError('calibration root must be an absolute path without traversal')
        self._check_root()
        self.root.mkdir(parents=True, exist_ok=True)
        self._check_root()
        self._record_cache={};self._ids={};self._names=()
        self._active_cache=None
        self.generation=0

    @staticmethod
    def _stamp(path):
        try:value=path.lstat()
        except FileNotFoundError:return None
        return (value.st_dev,value.st_ino,value.st_mode,value.st_size,value.st_mtime_ns,value.st_ctime_ns)

    def _content_stamp(self,path):
        if path.is_symlink():raise ValueError('calibration dependency must not be a symlink')
        if not path.is_file():return self._stamp(path)
        fd=os.open(path,os.O_RDONLY | os.O_NOFOLLOW)
        with os.fdopen(fd,'rb') as stream:return hashlib.sha256(stream.read()).digest()

    def invalidate(self,*,catalog=True,active=True):
        """Store writes invalidate immediately; every read still hashes actual file bytes."""
        with _LOCK:
            if catalog:self._record_cache.clear();self._ids={};self._names=()
            if active:self._active_cache=None
            self.generation+=1

    def _check_root(self):
        if any(path.is_symlink() for path in (self.root, *self.root.parents)):
            raise ValueError('calibration root and parents must not be symlinks')
        if self.root.exists() and not self.root.is_dir():
            raise ValueError('calibration root must be a directory')

    @contextmanager
    def _locked(self):
        # One process lock plus flock keeps separate store instances/processes coherent.
        with _LOCK:
            self._check_root()
            try:
                fd = os.open(self.root/'.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            except OSError as exc:
                raise ValueError('cannot safely open calibration lock') from exc
            try:
                fcntl.flock(fd, fcntl.LOCK_EX)
                yield
            finally:
                os.close(fd)

    def _record(self,path):
        if not re.fullmatch(r'v[0-9]{6,}\.json',path.name) or path.is_symlink():raise ValueError('invalid calibration version file')
        try:
            fd=os.open(path,os.O_RDONLY | os.O_NOFOLLOW)
            with os.fdopen(fd,'rb') as stream:
                if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):raise ValueError('regular calibration record required')
                raw=stream.read()
            # ARX filesystems can report identical nanosecond stats for rapid,
            # same-size writes. Content hashes, never stat alone, fence this cache.
            content_hash=hashlib.sha256(raw).digest();cached=self._record_cache.get(path.name)
            if cached is not None and cached[0]==content_hash:return cached[1]
            record=json.loads(raw);digest=record.pop('sha256')
            if not isinstance(digest,str) or not hmac.compare_digest(digest,hashlib.sha256(_canonical(record)).hexdigest()):
                raise ValueError('calibration hash mismatch')
            if record['schema']!=_SCHEMA or record['robot_id']!=self.robot_id or record['version']!=path.stem:
                raise ValueError('calibration identity mismatch')
            _identifier(record['id'])
            if record['kind'] not in _KINDS or not isinstance(record['payload'],dict):raise ValueError('invalid calibration payload or kind')
            if record.get('environment') not in ('simulation','hardware') or self.environment is not None and record['environment']!=self.environment:
                raise ValueError('calibration source does not match library')
            record={**record,'sha256':digest,'path':str(path)}
            self._record_cache[path.name]=(content_hash,record);self.generation+=1
            return record
        except (OSError,KeyError,TypeError,AttributeError,UnicodeError,ValueError) as exc:
            self._record_cache.pop(path.name,None)
            raise ValueError('invalid calibration record: '+path.name) from exc

    def _records(self):
        records=[];ids={}
        for path in sorted(self.root.glob('v*.json')):
            record=self._record(path)
            if record['id'] in ids:raise ValueError('invalid or duplicate calibration identity')
            ids[record['id']]=path.name;records.append(record)
        names=tuple(record['version']+'.json' for record in records)
        if names!=self._names:self.generation+=1
        self._names=names;self._ids=ids
        self._record_cache={name:self._record_cache[name] for name in names}
        return sorted(records,key=lambda row:int(row['version'][1:]))

    def list(self):
        with self._locked():
            return deepcopy(self._records())

    def get(self, id):
        _identifier(id)
        with self._locked():
            names=tuple(path.name for path in sorted(self.root.glob('v*.json')))
            if names!=self._names or id not in self._ids:self._records()
            if id in self._ids:
                record=self._record(self.root/self._ids[id])
                if record['id']==id:return deepcopy(record)
                # A legitimate external replacement may also change its ID.
                for record in self._records():
                    if record['id']==id:return deepcopy(record)
        raise ValueError('unknown calibration identifier')

    def split_legacy(self):
        """Preserve version bytes and IDs while partitioning the old flat library."""
        with self._locked():
            records=self._records()  # Validate every legacy record before moving any.
            targets=[]
            for record in records:
                store=CalibrationStore(self.root/record['environment'],self.robot_id,record['environment'])
                source=Path(record['path']);target=store.root/source.name
                if target.is_symlink() or target.exists() and target.read_bytes()!=source.read_bytes():
                    raise ValueError('legacy calibration version conflicts with destination')
                targets.append((source,target))
            for source,target in targets:
                if not target.exists():os.link(source,target)  # Exclusive, atomic publication.
                source.unlink()
            self.invalidate()

    def register(self, kind, id, payload, source_batch, environment='simulation', dependency_id=None, validation=None):
        _identifier(id)
        if kind not in _KINDS:
            raise ValueError('calibration kind must be head, left or right')
        if not isinstance(payload,dict) or not isinstance(source_batch,str) or not source_batch:
            raise ValueError('calibration requires object payload and source batch')
        if environment not in ('simulation','hardware') or self.environment is not None and environment!=self.environment or (validation is not None and not isinstance(validation,dict)):
            raise ValueError('invalid environment or validation')
        if dependency_id is not None:
            _identifier(dependency_id)
        # Freeze caller-owned objects and reject NaN/non-JSON input before acquiring locks.
        snapshot = json.loads(_canonical(dict(payload=payload,validation=validation)))
        with self._locked():
            records = self._records()
            for record in records:
                if record['id'] == id:
                    if record['kind'] != kind or record['environment']!=environment or record.get('dependency_id')!=dependency_id or _canonical(record['payload']) != _canonical(snapshot['payload']):
                        raise ValueError('calibration identifier conflicts with an existing snapshot')
                    return deepcopy(record)
            number = max((int(row['version'][1:]) for row in records),default=0)+1
            version = f'v{number:06d}'
            record = dict(schema=_SCHEMA,robot_id=self.robot_id,id=id,kind=kind,version=version,
                          created_at=datetime.now(timezone.utc).isoformat(),source_batch=source_batch,
                          environment=environment,dependency_id=dependency_id,**snapshot)
            record['sha256'] = hashlib.sha256(_canonical(record)).hexdigest()
            target = self.root/(version+'.json')
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=self.root,prefix='.pending-',delete=False) as stream:
                    temporary = Path(stream.name)
                    stream.write(_canonical(record)+b'\n')
                    stream.flush()
                    os.fsync(stream.fileno())
                os.link(temporary,target)  # Atomic publication; never replaces an existing version.
                self.invalidate(catalog=True,active=False)
                directory = os.open(self.root,os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
            finally:
                if temporary is not None:
                    temporary.unlink()
            return {**record,'path':str(target)}

    # -- installed (active) calibration ------------------------------------------------
    #
    # The versioned files above are history: immutable snapshots, one per publish. What
    # the robot and downstream consumers actually need is the *one* calibration that is
    # in force, at one known path, as one group - a head plus the wrist solved against
    # it. `active/` is that path: a self-contained directory holding exactly one group,
    # replaced atomically on install, never merged with the versions next to it.

    def active(self,*,asset_geometry_id=None,asset_seal_sha256=None,robot_source=None):
        """The installed group for this library, or None when nothing is installed."""
        qualification=(asset_geometry_id,asset_seal_sha256,robot_source)
        with self._locked():
            # Reading recovers a crash-orphaned install too: a consumer must never be
            # told "nothing is installed" because the previous install died mid-swap.
            self._heal_active()
            if self._active_cache is not None:
                paths,key,result=self._active_cache
                if tuple(self._content_stamp(path) for path in paths)+(qualification,)==key:return deepcopy(result)
            manifest=self._read_active()
            if manifest is None:
                return None
            paths=[self.root/_ACTIVE,self.root/_ACTIVE/'manifest.json',self.root/_ACTIVE/'head.json',self.root/_ACTIVE/'wrist.json']
            for kind in ('head','wrist'):
                version=str((manifest.get(kind) or {}).get('version') or '')
                if re.fullmatch(r'v[0-9]{6,}',version):paths.append(self.root/(version+'.json'))
            key=tuple(self._content_stamp(path) for path in paths)+(qualification,)
            if key[1]!=self._active_manifest_hash:raise ValueError('installed manifest changed while reading')
            problems=self._active_problems(manifest,asset_geometry_id=asset_geometry_id,
                asset_seal_sha256=asset_seal_sha256,robot_source=robot_source)
            if tuple(self._content_stamp(path) for path in paths)+(qualification,)!=key:raise ValueError('installed calibration changed while verifying')
            result={**manifest,'path':str(self.root/_ACTIVE),'verified':not problems,'problems':problems}
            self._active_cache=(paths,key,result);self.generation+=1
            return deepcopy(result)

    def install(self,head,wrist,*,geometry_id,acceptance=None,notes=None,robot_source=None):
        """Make one head+wrist group the installed calibration, atomically.

        The two payloads are copied in byte-identical form so the directory stands on
        its own; the manifest records the source versions and digests so a mismatch
        against the library is detectable rather than silent.
        """
        for record,kind in ((head,'head'),(wrist,'wrist')):
            if not isinstance(record,dict) or record.get('kind')!=kind:
                raise ValueError('an installed group is one head plus one paired wrist')
            if self.environment is not None and record.get('environment')!=self.environment:
                raise ValueError('the installed group must come from this library')
            if not re.fullmatch(r'v[0-9]{6,}',str(record.get('version') or '')):
                raise ValueError('installed versions must be library versions')
        if wrist.get('dependency_id')!=head['id']:
            raise ValueError('the wrist must be the one solved against this head')
        if geometry_id is not None and not isinstance(geometry_id,str):
            raise ValueError('invalid geometry identifier')
        if robot_source not in (None,'physical','simulated'):
            raise ValueError('installation requires a known robot source')
        with self._locked():
            published={record['id']:record for record in self._records()}
            for record in (head,wrist):
                if published.get(record['id'])!=record:
                    raise ValueError('installation must use the unchanged published record')
            rows=[record['payload'].get('row',{}) for record in (head,wrist)]
            sources=[row.get('robot_source') for row in rows]
            # Legacy fake records may lack provenance; none may become physical by default.
            if robot_source=='physical' or 'physical' in sources:
                if (robot_source=='simulated' or any(record['environment']!='hardware' for record in (head,wrist))
                        or any(row.get('robot_source')!='physical' or row.get('simulation_only') is not False for row in rows)):
                    raise ValueError('physical installation requires explicit physical head and wrist sources')
                robot_source='physical'
            elif any(source not in (None,'unknown','simulated') for source in sources):
                raise ValueError('invalid calibration robot source')
            elif all(source=='simulated' for source in sources):
                robot_source='simulated'
            sources={kind:self.root/(record['version']+'.json') for kind,record in (('head',head),('wrist',wrist))}
            for kind,path in sources.items():
                if not path.is_file() or path.is_symlink():
                    raise ValueError('cannot install '+kind+' '+path.name+': not a published version')
            previous=self._read_active()
            environment=wrist.get('environment') or self.environment
            if environment not in ('simulation','hardware'):
                raise ValueError('the installed group needs an environment')
            manifest={'schema':_ACTIVE_SCHEMA,'environment':environment,'robot_id':self.robot_id,
                      'robot_source':robot_source or 'unknown','simulation_only':robot_source!='physical',
                      'installed_at':datetime.now(timezone.utc).isoformat(),'geometry_id':geometry_id,
                      'head':{'id':head['id'],'version':head['version'],'sha256':head['sha256'],
                              'source_batch':head.get('source_batch')},
                      'wrist':{'id':wrist['id'],'version':wrist['version'],'sha256':wrist['sha256'],
                               'source_batch':wrist.get('source_batch'),
                               'dependency_id':wrist.get('dependency_id'),
                               'contract_sha256':(wrist.get('payload') or {}).get('row',{}).get('result',{}).get('contract_sha256')},
                      'acceptance':acceptance or {},'notes':notes,
                      # Only a *different* group counts as replaced; re-installing the
                      # active group is a refresh and must not read as a change.
                      'replaces':({'head_version':previous['head']['version'],'wrist_version':previous['wrist']['version']}
                                  if previous and (previous['head']['id'],previous['wrist']['id'])!=(head['id'],wrist['id'])
                                  else None)}
            self._heal_active()
            token=secrets.token_hex(8)
            temporary=self.root/(_ACTIVE_TMP+token)
            superseded=self.root/(_ACTIVE_SUPERSEDED+token)
            target=self.root/_ACTIVE
            try:
                temporary.mkdir(mode=0o755)
                for kind,path in sources.items():
                    self._write_file(temporary/(kind+'.json'),path.read_bytes())
                self._write_file(temporary/'manifest.json',_canonical(manifest)+b'\n')
                self._sync_directory(temporary)
                if target.exists():
                    os.rename(target,superseded)
                os.rename(temporary,target)
                self.invalidate(catalog=False,active=True)
                self._sync_directory(self.root)
            except BaseException:
                shutil.rmtree(temporary,ignore_errors=True)
                if not target.exists() and superseded.exists():
                    os.rename(superseded,target)
                raise
            shutil.rmtree(superseded,ignore_errors=True)
            # Leftover staging directories from an interrupted install can never be valid.
            for stale in self.root.glob(_ACTIVE_TMP+'*'):
                shutil.rmtree(stale,ignore_errors=True)
            return {**manifest,'path':str(target),'verified':True,'problems':[]}

    def _read_active(self):
        path=self.root/_ACTIVE
        manifest_path=path/'manifest.json'
        if path.is_symlink() or manifest_path.is_symlink() or not manifest_path.is_file():
            return None
        try:
            raw=manifest_path.read_bytes();manifest=json.loads(raw)
            self._active_manifest_hash=hashlib.sha256(raw).digest()
        except (OSError,ValueError) as exc:
            raise ValueError('installed calibration manifest is unreadable') from exc
        if not isinstance(manifest,dict) or manifest.get('schema')!=_ACTIVE_SCHEMA:
            raise ValueError('installed calibration manifest has an unknown schema')
        return manifest

    def _active_problems(self,manifest,*,asset_geometry_id=None,asset_seal_sha256=None,robot_source=None):
        problems=[];records={}
        if self.environment is not None and manifest.get('environment')!=self.environment:
            problems.append('installed calibration belongs to another environment')
        if manifest.get('robot_id')!=self.robot_id:
            problems.append('installed calibration belongs to another robot')
        for kind in ('head','wrist'):
            entry=manifest.get(kind) or {}
            version=str(entry.get('version') or '')
            installed=self.root/_ACTIVE/(kind+'.json')
            published=self.root/(version+'.json')
            if not re.fullmatch(r'v[0-9]{6,}',version) or not installed.is_file():
                problems.append(kind+' file is missing')
            elif not published.is_file():
                problems.append(kind+' '+version+' is no longer in the library')
            elif installed.read_bytes()!=published.read_bytes():
                problems.append(kind+' file differs from library '+version)
            else:
                try:record=self._record(published)
                except ValueError:problems.append(kind+' published record is invalid');continue
                if record['id']!=entry.get('id') or record['sha256']!=entry.get('sha256') or record['kind']!=kind:
                    problems.append(kind+' installed identity differs from the validated published record')
                records[kind]=record
        if asset_geometry_id and not problems:
            head,wrist=records['head'],records['wrist']
            rows=[record['payload'].get('row',{}) for record in (head,wrist)]
            contract=wrist['payload'].get('contract',{})
            if (any(not isinstance(row,dict) for row in rows) or not isinstance(contract,dict)
                    or not isinstance(rows[0].get('parameters'),dict)
                    or not isinstance(contract.get('head_snapshot'),dict)):
                return ['installed calibration asset/group identity is incomplete']
            # The Workbench fingerprint also contains software/support policy. The
            # immutable installed pair pins the asset separately, as A1-cal does.
            identities=(rows[0].get('parameters',{}),rows[1],contract)
            if (not asset_seal_sha256 or any(row.get('asset_geometry_id')!=asset_geometry_id
                    or row.get('asset_seal_sha256')!=asset_seal_sha256 for row in identities)):
                problems.append('installed calibration must match the current sealed native A1 geometry')
            if (not manifest.get('geometry_id') or contract.get('geometry_id')!=manifest['geometry_id']
                    or wrist.get('dependency_id')!=head['id']
                    or manifest['wrist'].get('dependency_id')!=head['id']
                    or contract.get('head_snapshot',{}).get('id')!=head['id']):
                problems.append('installed calibration head/wrist group dependency mismatch')
            if (robot_source not in ('physical','simulated')
                    or manifest.get('robot_source')!=robot_source
                    or manifest.get('simulation_only') is not (robot_source!='physical')
                    or any(record.get('environment')!=manifest.get('environment') for record in (head,wrist))
                    or any(row.get('robot_source')!=robot_source
                        or row.get('simulation_only') is not (robot_source!='physical') for row in rows)):
                problems.append('installed calibration source differs from this device environment')
        return problems

    def _heal_active(self):
        """Finish an install that a crash interrupted between its two renames."""
        target=self.root/_ACTIVE
        if target.exists():
            return
        stale=sorted(self.root.glob(_ACTIVE_SUPERSEDED+'*'),key=lambda path:path.stat().st_mtime)
        if stale:
            os.rename(stale[-1],target)
            self.invalidate(catalog=False,active=True)
            self._sync_directory(self.root)

    @staticmethod
    def _write_file(path,payload):
        fd=os.open(path,os.O_CREAT | os.O_WRONLY | os.O_EXCL | os.O_NOFOLLOW,0o644)
        try:
            os.write(fd,payload)
            os.fsync(fd)
        finally:
            os.close(fd)

    @staticmethod
    def _sync_directory(path):
        fd=os.open(path,os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
