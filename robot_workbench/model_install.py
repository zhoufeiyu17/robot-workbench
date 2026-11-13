"""Portable completed model releases and atomic target-side catalog installation."""
from copy import deepcopy
from contextlib import contextmanager
import configparser
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
from urllib.parse import urlsplit

RELEASE_SCHEMA = 'workbench.model-release.v1'
TARGET_SCHEMA = 'workbench.model-install-target.v1'
SMALL_BYTES = 16 * 1024**2
WEIGHT_BYTES = 64 * 1024**2
WEIGHT_SUFFIXES = {'.pt', '.pth', '.ckpt', '.safetensors', '.bin'}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def relative_path(value):
    if (not isinstance(value, str) or not value or '\\' in value
            or PurePosixPath(value).is_absolute() or any(p in ('', '.', '..') for p in value.split('/'))):
        raise ValueError('normalized relative artifact path required')
    return Path(value)


def read_json(path):
    path = Path(path)
    if not path.is_file() or path.stat().st_size > SMALL_BYTES:
        raise ValueError('bounded JSON manifest required')
    return json.loads(path.read_bytes())


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = path.with_name(path.name + '.incoming')
    with temporary.open('wb') as stream:
        os.chmod(temporary, 0o600)
        stream.write(canonical(value) + b'\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def validate_release(value):
    from . import wam, wam_mem
    value = deepcopy(value)
    required = {'schema', 'complete', 'model_id', 'step', 'producer', 'files', 'backends'}
    if not isinstance(value, dict) or set(value) != required or value['schema'] != RELEASE_SCHEMA or value['complete'] is not True:
        raise ValueError('completed portable model release required')
    identity = value['model_id']
    family = wam_mem if wam_mem.is_model_id(identity) else wam if wam.is_model_id(identity) else None
    if family is None or type(value['step']) is not int or value['step'] != int(identity.rsplit('step', 1)[1]):
        raise ValueError('supported family and matching checkpoint step required')
    if not isinstance(value['producer'], dict) or not value['producer']:
        raise ValueError('training producer identity required')
    files = value['files']
    if not isinstance(files, list) or not files or len(files) > 10000:
        raise ValueError('bounded artifact list required')
    seen = set()
    for row in files:
        if not isinstance(row, dict) or set(row) != {'path', 'bytes', 'sha256', 'object'}:
            raise ValueError('artifact path, size, digest and object required')
        path = relative_path(row['path'])
        if str(path) in seen or type(row['bytes']) is not int or row['bytes'] < 0:
            raise ValueError('duplicate artifact or invalid size')
        if path.parts[0] in ('env', 'logs', '.install') or row['path'] in ('source-receipt.json', 'install-receipt.json'):
            raise ValueError('artifact collides with target-owned files')
        seen.add(str(path))
        relative_path(row['object'])
        if not re.fullmatch('[0-9a-f]{64}', str(row['sha256'])) and not (
                row['sha256'] is None and row['bytes'] > WEIGHT_BYTES and path.suffix.lower() in WEIGHT_SUFFIXES):
            raise ValueError('small assets need a digest; large weights may use producer completion and transfer CRC')
    backends = value['backends']
    if not isinstance(backends, dict) or not backends or set(backends) - ({'torch'} if family is wam_mem else {'torch', 'anyinfer'}):
        raise ValueError('supported backend declarations required')
    for backend, spec in backends.items():
        roles = set(wam_mem.ROLES) if family is wam_mem else wam.ARTIFACT_KINDS | {'engine'}
        fields = {'runtime_id', 'artifacts', 'inference_capabilities'}
        if family is wam_mem:fields |= {'model_source', 'instruction', 'cached_text_identity'}
        if not isinstance(spec, dict) or set(spec) != fields or set(spec.get('artifacts', {})) != roles:
            raise ValueError('complete family artifact roles required')
        if not isinstance(spec['runtime_id'], str) or not re.fullmatch('[A-Za-z0-9_.-]{1,100}', spec['runtime_id']):
            raise ValueError('explicit runtime profile identity required')
        for path in spec['artifacts'].values():
            if str(relative_path(path)) not in seen:raise ValueError('artifact role is absent from file list')
        capabilities = spec['inference_capabilities']
        wam.validate_inference_options({'denoise_steps': capabilities['denoise_steps']['default']}, capabilities)
        if family is wam_mem:
            source = relative_path(spec['model_source'])
            if str(source / 'world_memory/inference.py') not in seen:
                raise ValueError('memory model source is absent from release')
            if not isinstance(spec['instruction'], str) or not 0 < len(spec['instruction']) <= 4096:
                raise ValueError('frozen training instruction required')
    return value


def export_release(spec, source_root, destination):
    """Training completion hook. Read small files and stat weights once."""
    root = Path(source_root).resolve(strict=True)
    value = deepcopy(spec)
    rows = []
    for row in value['files']:
        path = root / relative_path(row.get('source', row['path']))
        if path.is_symlink() or not path.resolve(strict=True).is_relative_to(root) or not path.is_file():
            raise ValueError('producer artifact must be a regular source file')
        size = path.stat().st_size
        large_weight = size > WEIGHT_BYTES and path.suffix.lower() in WEIGHT_SUFFIXES
        sha = row.get('sha256') if large_weight else hashlib.sha256(path.read_bytes()).hexdigest()
        rows.append(dict(path=row['path'], object=row.get('object', row['path']), bytes=size, sha256=sha))
    value.update(schema=RELEASE_SCHEMA, complete=True, files=rows)
    value = validate_release(value)
    write_json(destination, value)
    return value


def resolve_training_release(family, experiment, checkpoint, source_root, output, *, runtime_id, name=None, instruction=None):
    """Resolve native checkpoint markers and their own frozen dependencies."""
    experiment, source_root, output = (Path(p).resolve() for p in (experiment, source_root, output))
    output.mkdir(parents=True, exist_ok=True, mode=0o700)
    if family not in ('wam', 'wam-mem'):raise ValueError('explicit supported training family required')
    if checkpoint == 'latest':
        if family == 'wam':checkpoint=read_json(experiment/'outputs/formal/latest.json')['path']
        else:
            candidates={p.parent for name in ('complete.json','export.json')for p in experiment.glob('*/step_*/'+name)}
            if not candidates:raise ValueError('no committed memory checkpoint found')
            if len({p.parent for p in candidates})!=1:raise ValueError('multiple training outputs require an explicit checkpoint')
            checkpoint=max(candidates,key=lambda p:read_json(p/('complete.json'if(p/'complete.json').exists()else'export.json'))['step'])
    checkpoint=Path(checkpoint)
    if not checkpoint.is_absolute():checkpoint=experiment/checkpoint
    checkpoint=checkpoint.resolve(strict=True)
    if not checkpoint.is_relative_to(experiment):raise ValueError('checkpoint belongs to another experiment')
    marker=checkpoint/('COMPLETE.json'if family=='wam'else'complete.json')
    if family=='wam-mem'and not marker.exists():marker=checkpoint/'export.json'
    complete=read_json(marker)
    if family=='wam-mem'and marker.name=='export.json':
        if complete.get('format')!='world_memory_policy'or complete.get('weights_only')is not True:
            raise ValueError('native weights-only policy export marker required')
        complete=dict(schema=1,step=complete['step'],contract=read_json(checkpoint.parent/'config.json'),
            dependencies=read_json(checkpoint.parent/'run.json')['dependencies'],
            files={'weights.pt':dict(size=complete['size'],sha256=complete.get('sha256'))})
    if family=='wam'and complete.get('schema')not in ('phoenix.training_state.v1','phoenix.model_checkpoint.v1'):
        raise ValueError('native WAM completion marker required')
    if family=='wam-mem'and complete.get('schema')!=1:raise ValueError('native memory completion marker required')
    step=complete['step']
    if type(step)is not int or step<=0:raise ValueError('positive committed optimizer step required')
    weight=complete['files']['weights.pt'];expected=weight.get('bytes',weight.get('size'))
    if type(expected)is not int or expected<=0:raise ValueError('nonempty committed weight required')
    if (checkpoint/'weights.pt').stat().st_size!=expected:raise ValueError('checkpoint weight is incomplete')
    if weight.get('mtime_ns') is not None and (checkpoint/'weights.pt').stat().st_mtime_ns!=weight['mtime_ns']:
        raise ValueError('checkpoint changed after native completion')
    label=re.sub('[^a-z0-9]+','-',(name or experiment.name).lower()).strip('-')
    identity=f'{family}-{label}-step{step:06d}'
    producer=dict(family=family,experiment=str(experiment),checkpoint=str(checkpoint),checkpoint_committed=True,
                  completion_sha256=hashlib.sha256(marker.read_bytes()).hexdigest(),completion_kind=marker.name)
    files=[];artifacts={};sources={}
    def add(path,destination,expected_sha=None,expected_size=None):
        path=Path(path).resolve(strict=True);relative_path(destination)
        if not path.is_relative_to(source_root)or not path.is_file():raise ValueError('producer dependency escaped its source root')
        if expected_size is not None and path.stat().st_size!=expected_size:raise ValueError('producer dependency size differs')
        if expected_sha and path.stat().st_size<=WEIGHT_BYTES and hashlib.sha256(path.read_bytes()).hexdigest()!=expected_sha:
            raise ValueError('producer dependency content differs')
        files.append(dict(path=destination,source=str(path.relative_to(source_root)),sha256=expected_sha))
        sources[destination]=path
        return destination
    artifacts['checkpoint']=add(checkpoint/'weights.pt','weights/weights.pt',weight.get('sha256'),expected)
    add(marker,'metadata/native-complete.json')
    capabilities={'denoise_steps':{'minimum':1,'maximum':10,'default':2}}
    backend=dict(runtime_id=runtime_id,artifacts=artifacts,inference_capabilities=capabilities)
    if family=='wam':
        contract=read_json(experiment/'contract.json');training=read_json(checkpoint/'trainer_state.json')
        if training['step']!=step or training['contract']!=complete['contract']:
            raise ValueError('trainer state and checkpoint completion differ')
        producer['epoch']=training['epoch']
        frozen=read_json(experiment/'frozen-inputs.json')
        source=Path(contract['runtime']['source']).resolve(strict=True)
        source_files={Path(path).relative_to(source).as_posix():sha for path,sha in frozen.items()
                      if Path(path).is_relative_to(source)}
        if not source_files:raise ValueError('frozen WAM source inventory required')
        pins=complete['contract']['assets']
        artifacts['stats']=add(contract['normalization'],'normalization.json',pins['normalization'])
        artifacts['model_config']=add(contract['model_definition'],'model-config.yaml',pins['model_definition'])
        index_path=Path(contract['text_index']);add(index_path,'metadata/text-index.json',pins['text_index'])
        index=read_json(index_path)
        contexts=[r for r in index['prompts'].values()if Path(r['path']).resolve()==Path(contract['text_pack']).resolve()]
        if len(contexts)!=1:raise ValueError('one exact training prompt context required')
        artifacts['context']=add(contract['text_pack'],'context.pt',contexts[0]['sha256'])
        for relative,sha in sorted(source_files.items()):add(source/relative,'source/'+relative,sha)
        manifest=output/'source-manifest.json';write_json(manifest,source_files)
        artifacts['engine']=add(manifest,'source-manifest.json')
        add(experiment/'contract.json','metadata/training-contract.json')
        add(checkpoint/'trainer_state.json','metadata/trainer-state.json')
    else:
        config=read_json(checkpoint.parent/'config.json');dependencies=complete['dependencies']
        run=read_json(checkpoint.parent/'run.json')
        if run['dependencies']!=dependencies:raise ValueError('memory run and checkpoint dependencies differ')
        data=complete['contract']['data']
        if any(data.get(k)!=v for k,v in dict(state_dim=14,action_dim=14,action_horizon=32,control_frequency=30,relative_action_dims=0).items()):
            raise ValueError('unsupported memory action/state contract')
        for role,field in (('vae','checkpoint'),('phoenix_encoder','phoenix_encoder'),('csvae_checkpoint','csvae_checkpoint')):
            rows=dependencies[role]
            if len(rows)!=1 or Path(rows[0]['path']).resolve()!=Path(config['codec'][field]).resolve():
                raise ValueError('memory codec path differs from committed dependency')
            row=rows[0];artifacts[role]=add(row['path'],'weights/'+Path(row['path']).name,row['sha256'],row['size'])
        index_path=Path(config['data']['cache_dir'])/'complete.json';pin=dependencies['text_cache']
        if index_path.resolve()!=Path(pin['path']).resolve():raise ValueError('text cache path differs from training')
        artifacts['text_index']=add(index_path,'text/complete.json',pin['sha256'],pin['size'])
        index=read_json(index_path);records=index['records']
        if instruction is None:
            if len(records)!=1:raise ValueError('multiple training prompts require an explicit instruction')
            instruction=next(iter(records))
        if instruction not in records:raise ValueError('instruction is absent from committed text cache')
        if len(records)!=1:raise ValueError('current memory adapter requires a single-instruction cache')
        row=records[instruction]
        artifacts['text_features']=add(index_path.parent/relative_path(row['file']),'text/'+row['file'],row['sha256'])
        source=experiment/'source/src/world_memory'
        for relative,sha in sorted(dependencies['source'].items()):
            add(source/relative_path(relative),'source/world_memory/'+relative,sha)
        manifest=output/'source-manifest.json';write_json(manifest,dependencies['source'])
        artifacts['source_manifest']=add(manifest,'source-manifest.json')
        backend.update(model_source='source',instruction=instruction,cached_text_identity=index['identity'])
        add(checkpoint.parent/'config.json','metadata/training-config.json')
    spec=dict(model_id=identity,step=step,producer=producer,files=files,backends={'torch':backend})
    release=export_release(spec,source_root,output/'release.json')
    return release,sources


class OssSource:
    """The target uses its existing private OSS configuration and intranet route."""
    def __init__(self, manifest_uri, settings, transfer_root, *, config_text=None):
        import oss2
        uri = urlsplit(manifest_uri)
        if uri.scheme != 'oss' or not uri.netloc or uri.query or uri.fragment or not uri.path.endswith('.json'):
            raise ValueError('OSS manifest object URI required')
        if not re.fullmatch(r'https://oss-[a-z0-9-]+-internal\.aliyuncs\.com', settings['endpoint']):
            raise ValueError('configured OSS intranet endpoint required')
        self.key = str(relative_path(uri.path.lstrip('/')))
        self.prefix = str(PurePosixPath(self.key).parent)
        config = configparser.ConfigParser(interpolation=None)
        if config_text:config.read_string(config_text)
        else:config.read(settings['config_file'])
        values = config['default']
        auth = oss2.AuthV4(values['accesskeyid'], values['accesskeysecret'])
        self.bucket = oss2.Bucket(auth, settings['endpoint'], uri.netloc, region=settings['region'], enable_crc=True)
        self.oss2 = oss2
        self.transfer_root=Path(transfer_root)
        self.store = oss2.ResumableDownloadStore(root=str(transfer_root))

    def manifest(self):
        head = self.bucket.head_object(self.key)
        if head.content_length > SMALL_BYTES:raise ValueError('release manifest exceeds bound')
        result = self.bucket.get_object(self.key)
        try:
            raw = result.read(SMALL_BYTES + 1)
        finally:result.close()
        if len(raw) > SMALL_BYTES:raise ValueError('release manifest grew beyond bound')
        return json.loads(raw)

    def download(self, row, destination):
        key = str(PurePosixPath(self.prefix) / relative_path(row['object']))
        before = self.bucket.head_object(key)
        if before.content_length != row['bytes']:raise ValueError('OSS artifact size differs from completed release')
        self.oss2.resumable_download(self.bucket, key, str(destination), store=self.store,
            multiget_threshold=32*1024**2, part_size=32*1024**2, num_threads=4)
        after = self.bucket.head_object(key)
        if (before.etag, before.content_length, before.last_modified) != (after.etag, after.content_length, after.last_modified):
            raise ValueError('producer object changed during download')


def publish_release(release,sources,settings,prefix,output,*,config_text=None):
    """Reuse the registered OSS CLI; commit the small release after all objects."""
    uri=urlsplit(prefix)
    if uri.scheme!='oss'or not uri.netloc or uri.query or uri.fragment:
        raise ValueError('private OSS release prefix required')
    relative_path(uri.path.lstrip('/').rstrip('/'))
    if not re.fullmatch(r'https://oss-[a-z0-9-]+-internal\.aliyuncs\.com',settings['endpoint']):
        raise ValueError('OSS intranet endpoint required')
    def upload(path,destination):
        transfer=Path(settings['transfer_root']);transfer.mkdir(parents=True,exist_ok=True,mode=0o700)
        args=[settings['binary'],'cp',str(path),destination,'--ignore-existing','--quiet',
              '--region',settings['region'],'--endpoint',settings['endpoint'],
              '--checkpoint-dir',str(transfer/'checkpoints'),'--output-dir',str(transfer/'reports'),
              '--parallel','4','--part-size','33554432','--bigfile-threshold','33554432']
        read_fd=None
        try:
            if config_text:
                read_fd=os.memfd_create('oss-config',os.MFD_CLOEXEC)
                os.write(read_fd,config_text.encode());os.lseek(read_fd,0,os.SEEK_SET)
                config_file='/proc/self/fd/'+str(read_fd)
            else:config_file=settings['config_file']
            subprocess.run([*args,'--config-file',config_file],check=True,timeout=1800,
                pass_fds=()if read_fd is None else(read_fd,),capture_output=True)
        finally:
            if read_fd is not None:os.close(read_fd)
    release=deepcopy(release)
    for index,row in enumerate(release['files']):
        asset=row['sha256']or digest(dict(producer=release['producer'],path=row['path'],bytes=row['bytes']))
        row['object']='objects/'+asset
        path=Path(sources[row['path']]);before=path.stat()
        write_json(Path(output)/'publish-progress.json',dict(status='publishing',model_id=release['model_id'],
            completed_files=index,total_files=len(release['files']),current_file=row['path'],current_bytes=row['bytes']))
        if not verify_file(path,row):raise ValueError('producer artifact changed before upload')
        upload(path,prefix.rstrip('/')+'/'+row['object'])
        after=path.stat()
        if (before.st_ino,before.st_size,before.st_mtime_ns)!=(after.st_ino,after.st_size,after.st_mtime_ns):
            raise ValueError('producer artifact changed during upload')
    release=validate_release(release)
    manifest=Path(output)/(release['model_id']+'-'+digest(release)+'.json');write_json(manifest,release)
    destination=prefix.rstrip('/')+'/'+manifest.name;upload(manifest,destination)
    write_json(Path(output)/'publish-progress.json',dict(status='published',model_id=release['model_id'],
        completed_files=len(release['files']),total_files=len(release['files']),source=destination))
    return destination,release



def verify_file(path, row):
    if path.is_symlink() or not path.is_file() or path.stat().st_size != row['bytes']:
        return False
    if row['bytes'] <= WEIGHT_BYTES or Path(row['path']).suffix.lower() not in WEIGHT_SUFFIXES:
        return hashlib.sha256(path.read_bytes()).hexdigest() == row['sha256']
    return True


@contextmanager
def install_lock(path):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    with path.open('a') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        yield


def install_release(release, target, download):
    """Publish all backend manifests together, after resumable asset preparation."""
    if sys.platform == 'darwin':raise ValueError('model assets must be installed on the target inference host')
    from . import wam, wam_mem
    from .inference_catalog import RegisteredCatalog
    release = validate_release(release)
    if target.get('schema') != TARGET_SCHEMA:raise ValueError('explicit target deployment profile required')
    asset_root, catalog_root = (Path(target[k]) for k in ('asset_root', 'catalog_root'))
    for path in (asset_root, catalog_root):
        if not path.is_absolute() or path.is_symlink():raise ValueError('absolute owned target roots required')
        path.mkdir(parents=True, exist_ok=True, mode=0o700)
    host_hash = hashlib.sha256(Path('/etc/machine-id').read_bytes()).hexdigest()
    if target['machine_id_sha256'] != host_hash or not re.fullmatch('GPU-[a-fA-F0-9-]+', target['gpu_uuid']):
        raise ValueError('target host and assigned GPU identity required')
    identity, release_id = release['model_id'], digest(release)
    family = wam_mem if wam_mem.is_model_id(identity) else wam
    for backend, spec in release['backends'].items():
        runtime = target['runtimes'][spec['runtime_id']]
        if runtime['family'] != ('wam-mem' if family is wam_mem else 'wam') or runtime['backend'] != backend:
            raise ValueError('runtime family/backend differs from release')
        python = Path(runtime['python'])
        if not python.is_absolute() or not python.is_file():raise ValueError('registered target interpreter missing')
    from . import model_runtime
    shared={}
    for backend, spec in release['backends'].items():
        runtime=target['runtimes'][spec['runtime_id']]
        pin=runtime.get('environment_lock')
        expected=model_runtime.read_pin(pin)
        model_runtime.verify(runtime['python'],expected)
        name=model_runtime.lock_path(backend)
        shared[name]=dict(path=name,source=pin['path'],bytes=Path(pin['path']).stat().st_size,sha256=pin['sha256'])
    for spec in release['backends'].values():
        for row in target['runtimes'][spec['runtime_id']].get('shared_files',[]):
            relative_path(row['path'])
            if row['path']in shared and shared[row['path']]!=row:raise ValueError('runtime shared assets conflict')
            if not verify_file(Path(row['source']),row):raise ValueError('qualified runtime asset changed')
            shared[row['path']]=row
    destination = asset_root / identity / release_id
    published = catalog_root / identity
    with install_lock(asset_root / '.install' / (identity + '.lock')):
        receipt_path = destination / 'install-receipt.json'
        if published.exists():
            receipt = read_json(receipt_path) if receipt_path.exists() else {}
            if receipt.get('release_id') != release_id:raise ValueError('model ID is already registered with different assets')
            if any(not verify_file(destination / r['path'], r) for r in release['files']):
                raise ValueError('installed model assets changed')
            rows = [r for r in RegisteredCatalog(catalog_root).rows() if r.get('model_id') == identity]
            if not rows or not all(r['selectable'] for r in rows):raise ValueError('installed catalog entry is inconsistent')
            for backend,spec in release['backends'].items():
                registered=next((r['registry']for r in rows if r['backend']==backend),None)
                if registered is None:raise ValueError('installed model backend is missing')
                runtime=target['runtimes'][spec['runtime_id']]
                lock_name=model_runtime.lock_path(backend)
                lock=next((r for r in registered['files']if r['path']==lock_name),{})
                if (str(model_runtime.interpreter(registered))!=runtime['python']
                        or lock.get('sha256')!=runtime['environment_lock']['sha256']):
                    raise ValueError('installed model runtime mapping differs from the target runtime')
            return dict(receipt, reused=True)
        staging = asset_root / '.install' / (identity + '-' + release_id)
        staging.mkdir(parents=True, exist_ok=True, mode=0o700)
        completed_path = staging / 'completed.json'
        completed = read_json(completed_path) if completed_path.exists() else {}
        data = destination if destination.exists() else staging / 'assets'
        data.mkdir(parents=True, exist_ok=True, mode=0o700)
        for row in release['files']:
            path = data / row['path']
            if (completed.get(row['path']) == digest(row) or destination.exists()) and verify_file(path, row):continue
            path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            if not path.resolve().is_relative_to(data.resolve()):raise ValueError('staging artifact escaped its target root')
            part = path.with_name(path.name + '.incoming')
            if row['sha256']:
                cached = asset_root / '.assets' / row['sha256']
                with install_lock(asset_root / '.install' / (row['sha256'] + '.lock')):
                    if not verify_file(cached, row):
                        cached.parent.mkdir(exist_ok=True, mode=0o700)
                        incoming = cached.with_suffix('.incoming')
                        download(row, incoming)
                        if not verify_file(incoming, row):raise ValueError('downloaded shared artifact differs')
                        incoming.replace(cached)
                    if part.exists():part.unlink()
                    os.link(cached, part)
            else:download(row, part)
            if not verify_file(part, row):raise ValueError('downloaded artifact failed size or small-content check')
            part.replace(path)
            completed[row['path']] = digest(row)
            write_json(completed_path, completed)
        release_paths={row['path']for row in release['files']}
        if release_paths&shared.keys():raise ValueError('producer files overlap target runtime dependencies')
        for name,row in shared.items():
            path=data/name;path.parent.mkdir(parents=True,exist_ok=True,mode=0o700)
            if not path.resolve().is_relative_to(data.resolve()):raise ValueError('runtime asset escaped staging')
            if path.exists():
                if not verify_file(path,row):raise ValueError('staged runtime asset changed')
                continue
            source=Path(row['source'])
            if source.stat().st_dev==data.stat().st_dev:os.link(source,path)
            else:shutil.copyfile(source,path)
        proof = dict(schema='workbench.model-source.v1', model_id=identity, step=release['step'],
            producer=release['producer'], release_id=release_id,
            files={r['path']: {'bytes': r['bytes']} for r in [*release['files'],*shared.values()]})
        if family is wam:
            checkpoints = {v['artifacts']['checkpoint'] for v in release['backends'].values()}
            if len(checkpoints) != 1:raise ValueError('backends must share one checkpoint artifact')
            proof['checkpoint_path'] = next(iter(checkpoints))
        write_json(data / 'source-receipt.json', proof)
        proof_bytes = (data / 'source-receipt.json').read_bytes()
        proof_sha = hashlib.sha256(proof_bytes).hexdigest()
        files = [{k: r[k] for k in ('path', 'bytes', 'sha256')} for r in [*release['files'],*shared.values()]]
        files.append(dict(path='source-receipt.json', bytes=len(proof_bytes), sha256=proof_sha))
        file_map = {r['path']: r for r in files}
        registries = {}
        for backend, spec in release['backends'].items():
            runtime = target['runtimes'][spec['runtime_id']]
            python = Path(runtime['python'])
            registry = dict(root=str(data), python=str(python), model_id=identity, step=release['step'], backend=backend,
                files=files, bundle_sha256=digest(sorted(files, key=lambda r: r['path'])),
                machine_id_sha256=host_hash, gpu_uuid=target['gpu_uuid'],
                inference_capabilities=spec['inference_capabilities'])
            for key in ('desktop_gpu_processes', 'min_free_memory_mib'):
                if key in target:registry[key] = target[key]
            if family is wam_mem:
                registry.update({k: spec[k] for k in ('model_source', 'instruction', 'cached_text_identity')})
                registry['artifacts'] = {role: ({'sha256': file_map[path]['sha256']} if file_map[path]['sha256'] else
                    dict(source_receipt_sha256=proof_sha, path=path, bytes=file_map[path]['bytes']))
                    for role, path in spec['artifacts'].items()}
            else:
                registry['artifacts'] = {role: file_map[spec['artifacts'][role]]['sha256'] for role in wam.ARTIFACT_KINDS}
                registry['engine_sha256'] = file_map[spec['artifacts']['engine']]['sha256']
                checkpoint = file_map[spec['artifacts']['checkpoint']]
                registry.update(checkpoint_path=checkpoint['path'], checkpoint_source=dict(receipt_sha256=proof_sha, bytes=checkpoint['bytes']))
            family.verify_registry(registry)
            registry['root'] = str(destination)
            registries[backend] = registry
        destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        if data != destination:data.rename(destination)
        receipt = dict(schema='workbench.model-install.v1', model_id=identity, release_id=release_id,
            backends=list(registries), status='installed', model_loaded=False, reused=False)
        write_json(receipt_path, receipt)
        staged_catalog = staging / 'catalog'
        for backend, registry in registries.items():write_json(staged_catalog / backend / 'manifest.json', registry)
        staged_catalog.rename(published)
        rows = [r for r in RegisteredCatalog(catalog_root).rows() if r.get('model_id') == identity]
        if not rows or not all(r['selectable'] for r in rows):raise ValueError('published model catalog verification failed')
        return receipt


def lock_runtimes(profile_path):
    """Freeze declared environments and bind existing WAM catalog entries atomically."""
    from . import model_runtime, wam, wam_mem
    from .inference_catalog import RegisteredCatalog
    profile_path=Path(profile_path).resolve(strict=True)
    with install_lock(profile_path.with_suffix(profile_path.suffix+'.runtime.lock')):
        target=read_json(profile_path)
        if (target.get('schema')!=TARGET_SCHEMA
                or target['machine_id_sha256']!=hashlib.sha256(Path('/etc/machine-id').read_bytes()).hexdigest()):
            raise ValueError('runtime lock target host identity differs')
        pins={}

        def freeze(python, existing=None):
            python=str(python)
            if python in pins:
                if existing is not None and model_runtime.read_pin(existing)!=model_runtime.read_pin(pins[python]):
                    raise ValueError('one interpreter has conflicting runtime locks')
                return pins[python]
            path=profile_path.parent/'runtime-locks'/(hashlib.sha256(python.encode()).hexdigest()+'.json')
            if existing is not None:
                pin=existing
                model_runtime.verify(python,model_runtime.read_pin(pin))
            elif path.exists():
                pin=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                model_runtime.verify(python,model_runtime.read_pin(pin))
            else:
                write_json(path,model_runtime.capture(python))
                pin=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
            pins[python]=pin
            return pin

        for runtime in target['runtimes'].values():
            runtime['environment_lock']=freeze(runtime['python'],runtime.get('environment_lock'))
        write_json(profile_path,target)
        migrated=[]
        manifests={row['manifest_path']for row in RegisteredCatalog(target['catalog_root']).rows()if row.get('selectable')}
        for manifest_name in sorted(manifests):
            manifest=Path(manifest_name)
            registry=read_json(manifest)
            if not (wam.is_model_id(registry.get('model_id')) or wam_mem.is_model_id(registry.get('model_id'))):continue
            if registry.get('backend','torch') not in ('torch','anyinfer'):continue
            identity=registry['model_id']
            with install_lock(Path(target['asset_root'])/'.install'/(identity+'.lock')):
                registry=read_json(manifest)
                python=model_runtime.interpreter(registry)
                pin=freeze(python)
                registry['python']=str(python)
                root=Path(registry['root']).resolve(strict=True)
                name=model_runtime.lock_path(registry.get('backend','torch'))
                path=root/name
                if path.exists() and path.read_bytes()!=Path(pin['path']).read_bytes():
                    raise ValueError('existing model environment lock differs')
                write_json(path,model_runtime.read_pin(pin))
                row=dict(path=name,bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                registry['files']=[r for r in registry['files']if r['path']!=name]+[row]
                registry['bundle_sha256']=digest(sorted(registry['files'],key=lambda r:r['path']))
                write_json(manifest,registry)
                migrated.append(identity)
        return dict(status='locked',environments=len(pins),models=migrated)
