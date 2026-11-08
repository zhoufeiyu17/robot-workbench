"""Authenticated loopback model transport; no controller, calibration or GPU work on import.

The device converts observations to the frozen model domain before transport.
Conversion identity is echoed for the caller/Owner to verify, not attested by
the model server. An explicitly configured tunnel may bridge two hosts.
"""
from copy import deepcopy
import hashlib
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
import os
from pathlib import Path
import re
import secrets
import threading
import time
from urllib.parse import urlsplit
import zipfile

import numpy as np

from .inference_model import (BASE379_ID, CAMS, model_family, model_spec, model_descriptor, model_bundle_digest,
                              validate_bundle, validate_worker_identity, native_runtime, action_contract)

MAX_PACKET = 8 * 1024 * 1024
MAX_METADATA = 256 * 1024


class ModelSelectionChanged(ValueError):
    """The service is reachable, but a different explicitly selected backend is loaded."""
PROTOCOL = 'workbench.model-http.v2'
MODEL_ID = BASE379_ID


def start_model_service(config, output, token_file):
    """Launch the owned CPU endpoint; explicit prepare is the only GPU load gate."""
    import subprocess,sys
    from rule_based_workbench.launch_environment import clean_environment
    root=Path(__file__).resolve().parents[1];output=Path(output);output.mkdir(exist_ok=False)
    required={'model_manifest','gpu','gpu_uuid','min_free_memory_mib'}
    if (required-set(config) or set(config)-required-{'catalog_root'} or type(config['gpu']) is not int or config['gpu']<0
            or not str(config['gpu_uuid']).startswith('GPU-') or type(config['min_free_memory_mib']) is not int
            or config['min_free_memory_mib']<1 or not Path(config['model_manifest']).is_absolute()):
        raise ValueError('explicit local model manifest, GPU identity and free-memory budget required')
    if 'catalog_root' in config and (not isinstance(config['catalog_root'], str) or not Path(config['catalog_root']).is_absolute()):
        raise ValueError('absolute trusted checkpoint catalog root required')
    from .inference_model import validate_bundle
    validate_bundle(json.loads(Path(config['model_manifest']).read_text()))
    ready=output/'service.json'
    env=clean_environment(dict(PYTHONPATH=os.pathsep.join(dict.fromkeys([str(root),*sys.path])),
        PYTHONDONTWRITEBYTECODE='1',CUDA_VISIBLE_DEVICES=str(config['gpu']),OMP_NUM_THREADS='2',OPENBLAS_NUM_THREADS='1'))
    argv=[sys.executable,'-u','-B',str(root/'scripts/serve_inference_model.py'),
        '--bundle-manifest',config['model_manifest'],'--token-file',str(token_file),'--worker-log',str(output/'worker.log'),
        '--port','0','--ready-file',str(ready),'--gpu-uuid',config['gpu_uuid'],
        '--min-free-memory-mib',str(config['min_free_memory_mib'])]
    if 'catalog_root' in config:argv += ['--catalog-root',config['catalog_root']]
    with (output/'service.log').open('xb') as log:
        process=subprocess.Popen(argv,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
    deadline=time.monotonic()+10
    while not ready.exists():
        if process.poll() is not None or time.monotonic()>=deadline:
            if process.poll() is None:process.terminate();process.wait(10)
            raise RuntimeError('model endpoint failed to start; inspect '+str(output/'service.log'))
        time.sleep(.02)
    metadata=json.loads(ready.read_text())
    if metadata['pid']!=process.pid:raise ValueError('model endpoint process identity differs')
    return process,metadata['endpoint']


def conversion_digest(identity, model_id=MODEL_ID):
    model_family(model_id)
    if not isinstance(identity, dict) or not identity:
        raise ValueError('explicit device conversion identity required')
    return hashlib.sha256(json.dumps(dict(model_id=model_id, conversion=identity),
        sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def read_token(path):
    token = Path(path).read_text().strip()
    if not 32 <= len(token) <= 4096 or not token.isascii() or any(c.isspace() for c in token):
        raise ValueError('token file must contain 32..4096 non-whitespace ASCII characters')
    return token


def _identity(value, model_id=MODEL_ID):
    model_family(model_id)
    if (not isinstance(value, dict) or value.get('protocol') != PROTOCOL or value.get('model_id') != model_id
            or not isinstance(value.get('conversion_id'), str)
            or not re.fullmatch('[0-9a-f]{64}', value['conversion_id'])):
        raise ValueError('model protocol and conversion SHA256 identity required')
    if 'expected_bundle_digest' in value and (not isinstance(value['expected_bundle_digest'], str)
            or not re.fullmatch('[0-9a-f]{64}',value['expected_bundle_digest'])):
        raise ValueError('expected bundle digest must be a SHA256')


def _finite(value, shape, name):
    array = np.asarray(value)
    if array.shape != shape or array.dtype.kind != 'f' or not np.isfinite(array).all():
        raise ValueError(name + ' has invalid shape, dtype or nonfinite values')
    return array


def encode_packet(metadata, arrays):
    raw = json.dumps(metadata, separators=(',', ':'), allow_nan=False).encode()
    if len(raw) > MAX_METADATA:
        raise ValueError('model metadata exceeds bound')
    if 'metadata' in arrays or sum(np.asarray(value).nbytes for value in arrays.values()) + len(raw) > MAX_PACKET:
        raise ValueError('model arrays exceed bound')
    if any(np.asarray(value).dtype.hasobject for value in arrays.values()):
        raise ValueError('object arrays are forbidden')
    buffer = io.BytesIO()
    np.savez(buffer, metadata=np.frombuffer(raw, dtype=np.uint8), **arrays)
    payload = buffer.getvalue()
    if len(payload) > MAX_PACKET:
        raise ValueError('model packet exceeds bound')
    return payload


def decode_packet(payload, expected_arrays, model_id=MODEL_ID):
    """Validate ZIP expansion and NPY allocation sizes before NumPy loads anything."""
    if not isinstance(payload, bytes) or not 0 < len(payload) <= MAX_PACKET:
        raise ValueError('invalid model packet size')
    expected = {'metadata', *expected_arrays}
    try:
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            entries = archive.infolist()
            if (len(entries) != len(expected) or {row.filename for row in entries} != {name+'.npy' for name in expected}
                    or sum(row.file_size for row in entries) > MAX_PACKET):
                raise ValueError('unexpected arrays or expanded model packet exceeds bound')
            for row in entries:
                if row.flag_bits & 1 or row.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED):
                    raise ValueError('unsupported NPZ compression or encryption')
                with archive.open(row) as stream:
                    version = np.lib.format.read_magic(stream)
                    if version not in ((1, 0), (2, 0)):
                        raise ValueError('unsupported NPY version')
                    reader = np.lib.format.read_array_header_1_0 if version == (1, 0) else np.lib.format.read_array_header_2_0
                    shape, _, dtype = reader(stream, max_header_size=4096)
                    size = math.prod(shape) * dtype.itemsize
                    if (dtype.hasobject or dtype.fields or len(shape) > 3 or any(n < 0 for n in shape)
                            or size > MAX_PACKET or size != row.file_size-stream.tell()):
                        raise ValueError('invalid NPY allocation shape or payload size')
        with np.load(io.BytesIO(payload), allow_pickle=False, max_header_size=4096) as archive:
            raw = archive['metadata']
            if raw.dtype != np.uint8 or raw.ndim != 1 or raw.nbytes > MAX_METADATA:
                raise ValueError('bounded UTF8 metadata required')
            metadata = json.loads(raw.tobytes())
            arrays = {name: archive[name].copy() for name in expected_arrays}
        _identity(metadata, model_id)
        return metadata, arrays
    except (OSError, EOFError, zipfile.BadZipFile, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError('invalid model packet') from error


def request_packet(request, adapter, conversion_id, model_id=MODEL_ID, model_bundle=None, *, prepared_input=None):
    state, images = adapter.model_input(request['observation']) if prepared_input is None else prepared_input
    metadata = dict(protocol=PROTOCOL, model_id=model_id, conversion_id=conversion_id,
        request_id=request['request_id'], seed=request['seed'], source_time=request['observation']['source_time'],
        frame_ids=request['observation'].get('frame_ids', {}))
    if model_family(model_id) in ('base566','mixed366','wam'):metadata['expected_bundle_digest'] = model_bundle_digest(model_id,model_bundle)
    if native_runtime(model_bundle):metadata['backend']=model_bundle['backend']
    if model_family(model_id)=='mixed366':metadata['task']=request['task']
    arrays = dict(model_state14=np.asarray(state), model_history=np.asarray(request['applied_history']), **images)
    _validate_request(metadata, arrays, model_id, model_bundle)
    return encode_packet(metadata, arrays)


def _validate_request(metadata, arrays, model_id=MODEL_ID, model_bundle=None):
    _identity(metadata, model_id)
    keys = {'protocol', 'model_id', 'conversion_id', 'request_id', 'seed', 'source_time', 'frame_ids'}
    if model_family(model_id)=='mixed366':
        keys.add('task')
        if metadata.get('task') not in model_spec(model_id,model_bundle)['tasks']:raise ValueError('task is not registered for this checkpoint')
    if model_family(model_id) in ('base566','mixed366','wam'):keys.add('expected_bundle_digest')
    if native_runtime(model_bundle):
        keys.add('backend')
        if metadata.get('backend')!=model_bundle['backend']:raise ValueError('model backend differs from prepared runtime')
    if set(metadata) != keys:
        raise ValueError('unknown model request metadata')
    if not isinstance(metadata['request_id'], str) or not re.fullmatch('[A-Za-z0-9_-]{1,128}', metadata['request_id']):
        raise ValueError('bounded request identity required')
    if type(metadata['seed']) is not int or not 0 <= metadata['seed'] < 2**63:
        raise ValueError('model seed must be uint63')
    if type(metadata['source_time']) not in (int, float) or not math.isfinite(metadata['source_time']):
        raise ValueError('finite observation source time required')
    if not isinstance(metadata['frame_ids'], dict):
        raise ValueError('camera frame identity mapping required')
    if set(arrays) != {'model_state14', 'model_history', *CAMS}:
        raise ValueError('exact model-domain state, history and three RGB images required')
    _finite(arrays['model_state14'], (14,), 'model state')
    history = arrays['model_history']
    if history.ndim != 2 or history.shape[1] != 14 or len(history) > (0 if model_family(model_id)in('mixed366','wam')else 60):
        raise ValueError('model history must have shape (0..60,14)')
    _finite(history, history.shape, 'model history')
    for name in CAMS:
        image = arrays[name]
        if model_family(model_id)=='wam':
            if image.dtype!=np.uint16 or image.shape!=(256,256,3):raise ValueError('WAM input requires256x256x3 WAN BF16 bit patterns')
            values=(image.astype(np.uint32)<<16).view(np.float32)
            if not np.isfinite(values).all()or np.any(np.abs(values)>1):raise ValueError('WAM input has invalid BF16 image values')
        elif image.dtype != np.uint8 or image.shape != ((480,640,3)if model_family(model_id)=='mixed366'else(224,224,3)):
            raise ValueError('preprocessed model RGB must be uint8 (224,224,3)')
    if model_family(model_id)=='wam':
        from .wam import bf16_values
        if not np.array_equal(arrays['model_state14'],bf16_values(arrays['model_state14'])):
            raise ValueError('WAM state requires effective BF16 normalized values')


class ModelDomainAdapter:
    """FrozenBaseClient adapter for inputs already converted by the device host."""
    def model_input(self, observation):
        return (np.asarray(observation['model_state14'], dtype=np.float32), observation['images'])


class ModelService:
    """One serial, explicitly prepared FrozenBaseClient, owned by this service."""
    def __init__(self, client, *, model_id=None, catalog=None, client_factory=None, catalog_rows=None,
                 client_refresh=None):
        self.client = client
        self.model_id = model_id or getattr(client, 'model_id', MODEL_ID)
        self.model_bundle = getattr(client, 'bundle', None)
        model_spec(self.model_id, self.model_bundle)
        self.catalog, self.client_factory = catalog, client_factory
        self.catalog_rows = catalog_rows
        self.client_refresh = client_refresh
        if catalog is not None and not callable(client_factory):
            raise ValueError('catalog switching requires an owned client factory')
        if getattr(client, 'model_id', self.model_id) != self.model_id:
            raise ValueError('service model identity differs from worker deployment')
        self.base_log_path = getattr(client, 'log_path', None)
        self.prepare_attempts = 0
        self.worker_logs = []
        self.lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.instance_id=secrets.token_hex(16)
        self.fact_revision=0;self.fact_signature=None
        self.value = dict(status='not_loaded', loaded=False, busy=False, worker_identity=None,
                          error=None, current_request_id=None, last_request_id=None,
                          prepare_attempts=0, worker_logs=[])

    def _set(self, **fields):
        with self.state_lock:
            self.value.update(fields)

    def models(self):
        """Expose selected checkpoint identities, never model-host artifact paths."""
        if self.catalog_rows is not None:
            rows = self.catalog_rows()
        else:
            catalog = self.catalog() if callable(self.catalog) else self.catalog
            rows = [dict(model_id=(value or {}).get('model_id',name), status='ready', selectable=True, manifest=value)
                    for name,value in (catalog or {self.model_id:self.model_bundle}).items()]
        result = []
        for row in rows:
            item = {key:deepcopy(row[key]) for key in ('model_id','backend','label','epoch','step','status','reason','selectable') if key in row}
            manifest = row.get('derived_manifest', row.get('manifest'))
            if item.get('selectable'):
                spec = model_spec(row['model_id'], manifest)
                item.update(label=spec['label'], epoch=spec['epoch'], step=spec['step'],
                    model_descriptor=model_descriptor(spec['id'],manifest),
                    bundle_digest=model_bundle_digest(spec['id'],manifest))
                item['backend']=spec.get('backend','torch')
                for key in ('inference_options','inference_capabilities'):
                    if key in spec:item[key]=deepcopy(spec[key])
            result.append(item)
        return result

    def status(self):
        with self.state_lock:
            result = deepcopy(self.value)
        if callable(getattr(self.client,'worker_status',None)):
            worker=self.client.worker_status()
            result.update(worker_pid=worker['worker_pid'],worker_exit_code=worker['worker_exit_code'],worker_alive=worker['worker_alive'])
        else:
            worker = getattr(getattr(self.client, 'client', None), 'process', None)
            result.update(worker_pid=getattr(worker, 'pid', None),worker_exit_code=worker.poll() if worker is not None else None)
        result.update(protocol=PROTOCOL, model_id=self.model_id, service_pid=os.getpid(),
                      bundle_digest=model_bundle_digest(self.model_id,self.model_bundle),
                      action_contract=action_contract(self.model_id,self.model_bundle))
        if native_runtime(self.model_bundle):result['backend']=self.model_bundle['backend']
        if result['worker_exit_code'] is not None or result['loaded'] and result.get('worker_alive')is False:
            result.update(loaded=False, status='error', error=result.get('error') or 'owned model worker exited')
        from .state_contract import fact
        signature=(result['status'],result['loaded'],result['model_id'],result['bundle_digest'],result['worker_pid'],
            result['worker_exit_code'],(result.get('warmup')or{}).get('completed'),result.get('error'))
        with self.state_lock:
            if signature!=self.fact_signature:
                self.fact_signature=signature;self.fact_revision+=1
            result['fact']=fact('model','ModelService',self.instance_id,self.fact_revision,time.time_ns(),
                dependencies=dict(model_id=result['model_id'],worker_pid=result['worker_pid']),
                valid=result['status']=='ready'and result['loaded']is True,reason=result.get('error'))
        path = getattr(self.client, 'log_path', None)
        result['stderr_path'], result['stderr_tail'] = path, ''
        if path:
            try:
                with Path(path).open('rb') as stream:
                    stream.seek(0, 2);stream.seek(max(0, stream.tell()-8192))
                    result['stderr_tail'] = stream.read(8192).decode(errors='replace')
            except OSError:
                pass
        return result

    def prepare(self, metadata):
        requested = metadata.get('model_id') if isinstance(metadata, dict) else None
        _identity(metadata, requested)
        catalog = self.catalog() if callable(self.catalog) else self.catalog
        if catalog is not None:
            if not isinstance(catalog, dict):raise ValueError('trusted checkpoint catalog must be a mapping')
            choices=[value for key,value in catalog.items() if (value or {}).get('model_id',key)==requested]
            if 'expected_bundle_digest'in metadata:
                choices=[value for value in choices if model_bundle_digest(requested,value)==metadata['expected_bundle_digest']]
            if len(choices)!=1:raise ValueError('catalog model/backend is absent, ambiguous, or bundle identity changed')
            selected = validate_bundle(choices[0])
            if selected.get('model_id', MODEL_ID) != requested:raise ValueError('catalog entry model identity differs')
        elif requested != self.model_id:
            raise ValueError('model differs from this fixed deployment')
        else:selected = self.model_bundle
        keys={'protocol','model_id','conversion_id','expected_bundle_digest'}
        if native_runtime(selected):
            keys.add('backend')
            if set(metadata)!=keys or metadata['backend']!=selected['backend']:raise ValueError('explicit prepared backend identity required')
        elif set(metadata)!=keys:raise ValueError('prepare requires complete model and bundle identity')
        if ('expected_bundle_digest' in metadata
                and metadata['expected_bundle_digest'] != model_bundle_digest(requested,selected)):
            raise ValueError('selected checkpoint bundle identity changed')
        if not self.lock.acquire(blocking=False):
            raise ValueError('model worker busy; preparation was not queued')
        try:
            candidate = self.client
            if requested != self.model_id or selected != self.model_bundle:
                candidate = self.client_factory(selected)
            elif self.client_refresh is not None and not self.status()['loaded']:
                candidate = self.client_refresh(self.client)
            if candidate is not self.client:
                if getattr(candidate, 'model_id', None) != requested:
                    candidate.close();raise ValueError('catalog client differs from selected model')
                self.client.close()
                self.client, self.model_id, self.model_bundle = candidate, requested, selected
                self.base_log_path = getattr(candidate, 'log_path', None)
                self._set(status='not_loaded', loaded=False, worker_identity=None, error=None, warmup=None)
            if not self.status()['loaded']:
                if getattr(self.client, 'client', None) is not None:
                    self.client.close()  # Explicit prepare after an independently exited owned worker.
                self.prepare_attempts += 1
                if self.base_log_path:
                    self.client.log_path = str(self.base_log_path)+(f'.attempt-{self.prepare_attempts}' if self.prepare_attempts > 1 else '')
                    if Path(self.client.log_path).exists():
                        self.client.log_path += '.restart-'+secrets.token_hex(8)
                    self.worker_logs.append(self.client.log_path)
                self._set(status='loading', busy=True, error=None, prepare_attempts=self.prepare_attempts,
                          worker_logs=list(self.worker_logs))
                identity = self.client.prepare()
                validate_worker_identity(identity, self.model_id, bundle=self.model_bundle)
                start=time.monotonic()
                warm_request=dict(request_id='warmup',seed=0,applied_history=np.empty((0,14),np.float32),
                    observation=dict(source_time=0.,model_state14=np.zeros(14,np.float32),
                        images={name:np.zeros((480,640,3)if model_family(self.model_id)=='mixed366'else(224,224,3),np.uint8) for name in CAMS}))
                if model_family(self.model_id)=='mixed366':warm_request['task']='coffee'
                if model_family(self.model_id)=='wam':warm_request['observation']['images']={name:np.zeros((256,256,3),np.uint16)for name in CAMS}
                warm=self.client.infer(warm_request)
                prediction=_finite(warm['action14'],tuple(action_contract(self.model_id,self.model_bundle)['shape']),'warmup model action')
                self._set(status='ready', loaded=True, worker_identity=deepcopy(identity),
                    warmup=dict(completed=True,wall_s=time.monotonic()-start,seed=0,
                        input='zero model state/RGB and empty history; no device command',
                        output_sha256=hashlib.sha256(prediction.tobytes()).hexdigest(),
                        timing=deepcopy(warm.get('audit',{}).get('timing'))))
            return dict(**metadata, worker_identity=self.status()['worker_identity'])
        except Exception as error:
            self.client.close()
            self._set(status='error', loaded=False, error=str(error))
            raise
        finally:
            self._set(busy=False);self.lock.release()

    def infer(self, payload):
        metadata, arrays = decode_packet(payload, {'model_state14', 'model_history', *CAMS}, self.model_id)
        _validate_request(metadata, arrays, self.model_id, self.model_bundle)
        if not self.lock.acquire(blocking=False):
            raise ValueError('model worker busy; inference was not queued')
        try:
            _identity(metadata, self.model_id)
            if (model_family(self.model_id) in ('base566','mixed366','wam')
                    and metadata['expected_bundle_digest'] != model_bundle_digest(self.model_id,self.model_bundle)):
                raise ValueError('selected checkpoint bundle identity changed')
        except Exception:
            self.lock.release()
            raise  # A stale request must never close a newly selected model.
        try:
            if not self.status()['loaded']:
                raise ValueError('explicit model preparation required')
            self._set(busy=True, current_request_id=metadata['request_id'])
            request = dict(request_id=metadata['request_id'], seed=metadata['seed'],
                observation=dict(model_state14=arrays['model_state14'], images={name: arrays[name] for name in CAMS},
                    source_time=metadata['source_time'], frame_ids=metadata['frame_ids']),
                applied_history=arrays['model_history'])
            if model_family(self.model_id)=='mixed366':request['task']=metadata['task']
            started = time.monotonic()
            result = self.client.infer(request)
            if result.get('request_id') != metadata['request_id']:
                raise ValueError('worker response identity mismatch')
            action = _finite(result['action14'], tuple(action_contract(self.model_id,self.model_bundle)['shape']), 'model action')
            response = {key: metadata[key] for key in ('protocol', 'model_id', 'conversion_id', 'request_id')}
            if 'expected_bundle_digest' in metadata:response['expected_bundle_digest'] = metadata['expected_bundle_digest']
            if 'backend'in metadata:response['backend']=metadata['backend']
            response['audit'] = result.get('audit', {})
            if model_family(self.model_id)=='mixed366':
                response['audit']=dict(response['audit'],task=metadata['task'])
            encoded = encode_packet(response, dict(action14=action))
            self._set(last_request_id=metadata['request_id'], last_infer_wall_s=time.monotonic()-started)
            return encoded
        except Exception as error:
            # A failed/timeout pipe is not reused: no late frame can become the next result.
            self.client.close()
            self._set(status='error', loaded=False, error=str(error))
            raise
        finally:
            self._set(busy=False, current_request_id=None);self.lock.release()

    def close(self):
        with self.lock:
            self.client.close();self._set(status='closed', loaded=False, busy=False)

    def unload(self,metadata):
        if not self.lock.acquire(blocking=False):raise ValueError('model worker busy; unload was not queued')
        try:
            _identity(metadata,self.model_id)
            keys={'protocol','model_id','conversion_id','expected_bundle_digest'}
            if native_runtime(self.model_bundle):keys.add('backend')
            if (set(metadata)!=keys or metadata['expected_bundle_digest']!=model_bundle_digest(self.model_id,self.model_bundle)
                    or native_runtime(self.model_bundle)and metadata['backend']!=self.model_bundle['backend']):
                raise ValueError('unload requires the current model/backend identity')
            self.client.close()
            if callable(getattr(self.client,'worker_status',None)) and self.client.worker_status()['worker_alive']:
                raise ValueError('model worker exit is not confirmed')
            self._set(status='not_loaded',loaded=False,busy=False,worker_identity=None,warmup=None,error=None)
            return dict(**metadata,unloaded=True,worker_exited=True)
        finally:self.lock.release()


def serve_model(service, *, token_file, host='127.0.0.1', port=0, forwarded_ports=()):
    if host != '127.0.0.1' or type(port) is not int or not 0 <= port <= 65535:
        raise ValueError('model service binds only explicit IPv4 loopback')
    if (not isinstance(forwarded_ports,(tuple,list))
            or any(type(value) is not int or not 1<=value<=65535 for value in forwarded_ports)):
        raise ValueError('forwarded ports must be explicitly configured integers from 1 to 65535')
    forwarded_ports=tuple(forwarded_ports)
    token = read_token(token_file)
    from .model_session import PinnedModelService,PROTOCOL as SESSION_PROTOCOL
    if not isinstance(service,PinnedModelService):
        if service.status().get('session_protocol')is not None:
            raise ValueError('use the canonical model-session service; parallel wrapper registries are forbidden')
        # All listeners for one model object share the same capability registry.
        service=service.__dict__.setdefault('_workbench_model_sessions',PinnedModelService(service))
    class Handler(BaseHTTPRequestHandler):
        timeout = 5
        def log_message(self, *args):
            pass
        def reply(self, code, value, kind='application/json'):
            raw = value if isinstance(value, bytes) else json.dumps(value, allow_nan=False).encode()
            self.close_connection = True
            self.send_response(code);self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(raw)));self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'close');self.end_headers()
            try:self.wfile.write(raw)
            except OSError:pass
        def valid_host(self):
            hosts=self.headers.get_all('Host', [])
            return len(hosts)==1 and hosts[0] in self.server.allowed_hosts
        def do_GET(self):
            if not self.valid_host():return self.reply(403, {'error':'invalid Host'})
            if self.path.startswith('/v1/')and hasattr(service,'tool_catalog'):
                auth=self.headers.get_all('Authorization',[])
                if len(auth)!=1 or not secrets.compare_digest(auth[0].encode(),('Bearer '+token).encode()):
                    return self.reply(403,{'error':'model authorization required'})
                try:
                    if self.path=='/v1/catalog':return self.reply(200,service.tool_catalog())
                    if re.fullmatch('/v1/jobs/[a-f0-9]{24}',self.path):return self.reply(200,service.tool_job(self.path.rsplit('/',1)[-1]))
                except ValueError as error:return self.reply(409,{'error':str(error)})
                return self.reply(404,{'error':'not found'})
            if self.path == '/models':
                auth = self.headers.get_all('Authorization', [])
                if len(auth) != 1 or not secrets.compare_digest(auth[0].encode(), ('Bearer '+token).encode()):
                    return self.reply(403, {'error':'model authorization required'})
                try:return self.reply(200, dict(protocol=PROTOCOL, models=service.models()))
                except Exception as error:return self.reply(409, {'error':str(error)})
            if self.path not in ('/healthz', '/status'):return self.reply(404, {'error':'not found'})
            self.reply(200, service.status())
        def do_POST(self):
            if not self.valid_host():return self.reply(403, {'error':'invalid Host'})
            auth = self.headers.get_all('Authorization', [])
            if len(auth) != 1 or not secrets.compare_digest(auth[0].encode(), ('Bearer '+token).encode()):
                return self.reply(403, {'error':'model authorization required'})
            tool=self.path in ('/v1/conversions','/v1/infer')and hasattr(service,'tool_start')
            if not tool and self.path not in ('/prepare', '/infer', '/unload'):return self.reply(404, {'error':'not found'})
            lengths = self.headers.get_all('Content-Length', [])
            if (self.headers.get_all('Transfer-Encoding', []) or len(lengths) != 1 or not lengths[0].isascii()
                    or not lengths[0].isdigit() or not 0 < int(lengths[0]) <= MAX_PACKET):
                return self.reply(413, {'error':'bounded Content-Length required'})
            try:
                raw = self.rfile.read(int(lengths[0]))
                if len(raw) != int(lengths[0]):raise ValueError('incomplete model request')
                if tool:return self.reply(202,service.tool_start(self.path.rsplit('/',1)[-1],json.loads(raw)))
                if self.path in ('/prepare','/unload'):
                    if len(raw) > 4096:raise ValueError('bounded prepare identity required')
                    return self.reply(200, (service.prepare if self.path=='/prepare' else service.unload)(json.loads(raw)))
                self.reply(200, service.infer(raw), 'application/x-npz')
            except (ValueError, KeyError, TypeError) as error:self.reply(409, {'error':str(error)})
            except Exception as error:self.reply(503, {'error':str(error)})
    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    server.model_service=service
    server.forwarded_ports=tuple(sorted(set(forwarded_ports)))
    server.allowed_hosts={f'{name}:{value}' for name in ('127.0.0.1','localhost')
                          for value in (server.server_port,*server.forwarded_ports)}
    return server


class RemoteBaseClient:
    """Device-side conversion and bounded transport; close never stops a remote service."""
    def __init__(self, endpoint, *, token_file, adapter, conversion_identity,
                 startup_timeout_s=600., inference_timeout_s=120., model_id=MODEL_ID, model_bundle=None, task=None):
        model_spec(model_id, model_bundle)
        self.model_id = model_id
        if task is not None and task not in ('coffee','lid_handoff'):raise ValueError('registered training task required')
        self.task=task
        self.model_bundle = deepcopy(model_bundle)
        self.bundle_digest = model_bundle_digest(model_id,model_bundle)
        value = urlsplit(endpoint)
        if (value.scheme != 'http' or value.hostname not in ('127.0.0.1', 'localhost') or value.username or value.password
                or value.path not in ('', '/') or value.query or value.fragment or value.port is None or value.port < 1):
            raise ValueError('explicit http://127.0.0.1:<port> model endpoint or configured loopback tunnel required')
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in (startup_timeout_s, inference_timeout_s)):
            raise ValueError('positive finite model timeouts required')
        self.host, self.port, self.adapter = value.hostname, value.port, adapter
        self.token = read_token(token_file)
        self.conversion_id = conversion_digest(conversion_identity, model_id)
        self.conversion_identity = deepcopy(conversion_identity)
        self.startup_timeout, self.timeout = startup_timeout_s, inference_timeout_s
        self.bind_control_policy(None)
        self.identity = None

    def bind_control_policy(self,policy):
        from .execution_settings import Guard,control_parameters,defaults,policy_hash
        if policy is None:
            parameters=control_parameters(defaults('inference'),business='inference')
            policy=dict(revision=None,policy_hash=policy_hash(parameters),parameters=parameters)
        self._control_policy=deepcopy(policy)
        self._transport_guard=Guard.bind(policy,dict(infer_timeout_s=self.timeout))
        self._status_identity=dict(protocol=PROTOCOL,model_id=self.model_id,bundle_digest=self.bundle_digest)
        if native_runtime(self.model_bundle):self._status_identity['backend']=self.model_bundle['backend']
        self._status_guard=Guard.bind(policy,dict(expected_identity=self._status_identity))

    def _request(self, path, body=None, *, timeout=None, kind='application/json',guard_receipts=None):
        guard=self._transport_guard;started=time.monotonic()
        if path=='/infer'and timeout:
            from .execution_settings import Guard
            guard=Guard.bind(self._control_policy,dict(infer_timeout_s=timeout))
        limit=guard.evaluate(dict(rule_id='G09',signal_id='model_http',observed=0.,source_time=started,checked_time=started),'model_call')['limit'] if path=='/infer'else self.timeout
        connection = HTTPConnection(self.host, self.port, timeout=timeout or limit)
        try:
            connection.request('POST' if body is not None else 'GET', path, body=body,
                headers={'Authorization':'Bearer '+self.token, 'Content-Type':kind, 'Connection':'close'})
            response = connection.getresponse();raw = response.read(MAX_PACKET+1)
            if len(raw) > MAX_PACKET:raise ValueError('model response exceeds bound')
            if response.status != 200:
                try:error = json.loads(raw).get('error', 'model request failed')
                except (ValueError, AttributeError):error = 'invalid model error response'
                raise RuntimeError(f'model HTTP {response.status}: {error}')
            if path=='/infer':
                now=time.monotonic();decision=guard.evaluate(dict(rule_id='G09',signal_id='model_http',observed=now-started,
                    source_time=started,checked_time=now),'model_call')
                if guard_receipts is not None:guard_receipts.append(decision)
                if decision['verdict']=='reject':
                    error=ValueError('G09: model HTTP result exceeded transport timeout');error.guard_decision=decision;raise error
            return raw
        except TimeoutError as error:
            if path=='/infer':
                now=time.monotonic();error.guard_decision=guard.evaluate(dict(rule_id='G09',signal_id='model_http',
                    observed=dict(elapsed_s=now-started,timed_out=True),source_time=started,checked_time=now),'model_call')
            raise
        finally:
            connection.close()

    def status(self):
        value = json.loads(self._request('/status'))
        now=time.monotonic()
        decision=self._status_guard.evaluate(dict(rule_id='G01',signal_id='model_status',
            observed={key:value.get(key)for key in self._status_identity},source_time=now,checked_time=now),'model_status')
        if decision['verdict']=='reject':
            error=(ValueError('remote model status identity mismatch')if value.get('protocol')!=PROTOCOL or value.get('model_id')!=self.model_id else
                ModelSelectionChanged('常驻模型模式已切换；请重新选择 checkpoint 并核验当前工作区'))
            error.guard_decision=decision;raise error
        return value

    def models(self):
        value = json.loads(self._request('/models', timeout=5.))
        if value.get('protocol') != PROTOCOL or not isinstance(value.get('models'), list):
            raise ValueError('invalid checkpoint catalog response')
        for row in value['models']:
            if not isinstance(row, dict):raise ValueError('invalid checkpoint catalog row')
            if row.get('selectable'):
                model_spec(row.get('model_id'), row.get('model_descriptor'))
        return deepcopy(value['models'])

    list_models = models

    def prepare(self, *, ready_only=False):
        if ready_only:
            value=self.status()
            if value.get('status')!='ready' or value.get('loaded') is not True or (value.get('warmup') or {}).get('completed') is not True:
                raise ValueError('previously warm model is no longer ready; explicit preparation required')
            identity=value.get('worker_identity') or {}
            validate_worker_identity(identity,self.model_id,bundle=self.model_bundle)
            self.identity=deepcopy(identity)
            return deepcopy(identity)
        status=json.loads(self._request('/status'))
        if status.get('protocol')!=PROTOCOL:raise ValueError('model HTTP protocol version mismatch; preparation refused')
        metadata = dict(protocol=PROTOCOL, model_id=self.model_id, conversion_id=self.conversion_id,expected_bundle_digest=self.bundle_digest)
        if native_runtime(self.model_bundle):metadata['backend']=self.model_bundle['backend']
        value = json.loads(self._request('/prepare', json.dumps(metadata).encode(), timeout=self.startup_timeout))
        _identity(value, self.model_id)
        identity = value.get('worker_identity') or {}
        if (value['conversion_id'] != self.conversion_id or value.get('expected_bundle_digest') != self.bundle_digest
                or native_runtime(self.model_bundle)and value.get('backend')!=self.model_bundle['backend']):
            raise ValueError('remote model or conversion identity mismatch')
        validate_worker_identity(identity, self.model_id, bundle=self.model_bundle)
        self.identity = deepcopy(identity)
        return deepcopy(identity)

    def infer(self, request, *, prepared_input=None):
        if self.identity is None:raise ValueError('explicit remote model preparation required')
        if model_family(self.model_id)=='mixed366':
            if self.task is None or request.get('task',self.task)!=self.task:raise ValueError('request differs from bound training task')
            request=dict(request,task=self.task)
        payload = request_packet(request, self.adapter, self.conversion_id, self.model_id, self.model_bundle, prepared_input=prepared_input)
        decisions=[]
        metadata, arrays = decode_packet(self._request('/infer', payload, kind='application/x-npz',guard_receipts=decisions), {'action14'}, self.model_id)
        keys = {'protocol', 'model_id', 'conversion_id', 'request_id', 'audit'}
        if model_family(self.model_id) in ('base566','mixed366','wam'):keys.add('expected_bundle_digest')
        if native_runtime(self.model_bundle):keys.add('backend')
        if (set(metadata) != keys or 'expected_bundle_digest' in metadata and metadata['expected_bundle_digest'] != self.bundle_digest
                or native_runtime(self.model_bundle)and metadata['backend']!=self.model_bundle['backend']
                or metadata['conversion_id'] != self.conversion_id or metadata['request_id'] != request['request_id']):
            raise ValueError('remote result request or conversion identity mismatch')
        action = _finite(arrays['action14'], tuple(action_contract(self.model_id,self.model_bundle)['shape']), 'model action')
        audit = deepcopy(metadata['audit'])
        if not isinstance(audit, dict):raise ValueError('model audit object required')
        from .model_session import _CAPABILITY
        pinned=_CAPABILITY.fullmatch(request['request_id'])
        audit['transport'] = dict(protocol=PROTOCOL, model_id=self.model_id, conversion_id=self.conversion_id,
            request_id=pinned[2]if pinned else request['request_id'], conversion_identity=deepcopy(self.conversion_identity))
        audit['transport']['guard_decisions']=decisions
        return dict(request_id=request['request_id'], action14=action, audit=audit)

    def close(self):
        self.identity = None

    def pin(self,session_id,*,auto_renew=True):
        from .model_session import PinnedModelClient
        pinned=PinnedModelClient(self,session_id,auto_renew=auto_renew)
        pinned.acquire()
        return pinned
