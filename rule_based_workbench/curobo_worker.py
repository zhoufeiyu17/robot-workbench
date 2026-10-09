"""Owned persistent planner; offline stdin or formal simulation Unix socket.

Requests: {"input": "/.../planning-input.json", "source": "/.../source.json",
           "output": "/.../new-directory", "speed_scale": 0.3}.
The output root is for offline candidates, never a live workbench state directory.
"""
import argparse
from contextlib import contextmanager, redirect_stdout
import hashlib
import json
import math
import os
import select
import socket
import signal
from pathlib import Path
import sys
import time

MAX_MESSAGE_BYTES = 65536


def allowed_path(value, roots, *, exists=True):
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise ValueError('absolute path string required')
    path = Path(value).resolve(strict=exists)
    if not any(path.is_relative_to(root) for root in roots):
        raise ValueError(f'path outside allowed roots: {path}')
    return path


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def reject_constant(token):
    raise ValueError(f'nonfinite JSON constant: {token}')


def request_paths(line, read_roots, output_root):
    if len(line.encode('utf-8')) > MAX_MESSAGE_BYTES:
        raise ValueError('request exceeds 65536 bytes')
    message = json.loads(line, object_pairs_hook=unique_object, parse_constant=reject_constant)
    required = {'input', 'source', 'output'}
    if not isinstance(message, dict) or not required <= message.keys() or message.keys()-required-{'speed_scale','physical_binding','environment'}:
        raise ValueError('request requires input/source/output, optional speed_scale/physical_binding')
    speed = message.get('speed_scale', .3)
    if type(speed) not in (int, float) or not math.isfinite(speed) or not .1 <= speed <= 1.:
        raise ValueError('speed_scale must be finite and between 0.1 and 1.0')
    paths = {key: allowed_path(message[key], read_roots) for key in ('input', 'source')}
    if not all(path.is_file() for path in paths.values()):
        raise ValueError('input and source must be files')
    if 'physical_binding' in message:
        paths['physical_binding']=allowed_path(message['physical_binding'],read_roots)
        if not paths['physical_binding'].is_file():raise ValueError('physical binding must be a file')
    # load() also reads this sibling; a symlink must not escape the read roots.
    sidecar = allowed_path(str(paths['input'].with_name('result.json')), read_roots)
    if not sidecar.is_file():
        raise ValueError('geometry result.json must be a file')
    output = allowed_path(message['output'], (output_root,), exists=False)
    if output.exists() or Path(message['output']).is_symlink():
        raise FileExistsError(f'output already exists; refusing overwrite: {output}')
    from real_contract import namespace
    environment = namespace(message.get('environment','physical'))['environment']
    return dict(**paths, output=output, speed_scale=float(speed), **({'environment':environment} if environment=='fake' else {}))


@contextmanager
def progress_to_stderr():
    """Route Python and native-extension stdout away from the JSON protocol."""
    import ctypes
    sys.stdout.flush()
    saved = os.dup(1)
    try:
        os.dup2(2, 1)
        with redirect_stdout(sys.stderr):
            yield
    finally:
        sys.stderr.flush()
        ctypes.CDLL(None).fflush(None)
        os.dup2(saved, 1)
        os.close(saved)


def emit(value):
    print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)


class Worker:
    def __init__(self, native_code, assets, *, formal=False):
        import torch
        import curobo
        if curobo.__version__!='0.7.8':raise ValueError('worker requires cuRobo 0.7.8')
        self.native_code,self.assets=native_code,assets
        self.formal=formal
        self.engine_cache={}
        self.loaded_geometry=None

    def warmup(self,input_path=None,source_path=None):
        started=time.perf_counter()
        if input_path is None:
            return dict(warmup_seconds=0.,model_initialized=False,requires_geometry_input=True)
        from playground_geometry import load
        from motiongen_plan import gripper_geometry,initial_motion_state,prepare_model
        source=json.loads(source_path.read_text())
        planner,observation,data=load(input_path,source,self.native_code,self.assets)
        from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
        from robot_sim.specs.action import ac_one_dual_arm_zero_home_action_spec
        urdf=ac_one_asset_spec(self.assets).urdf_path
        grip=gripper_geometry(planner,urdf);opening=grip['open_travel_mm']
        state,_=initial_motion_state(planner,data,dict(source,synthetic_fixture=True),ac_one_dual_arm_zero_home_action_spec(),opening)
        prepared=prepare_model(planner,observation,data,urdf,dict(state,left_joint7=opening/2000,left_joint8=opening/2000),self.engine_cache)
        return dict(warmup_seconds=time.perf_counter()-started,model_initialized=True,model_key=self.engine_cache['key'],
            backend='curobo_motiongen',policy_planned=False)

    def load_geometry(self,request,parent,provenance):
        from playground_geometry import load
        # Code/assets are frozen for this worker lifetime. The sidecar is also
        # a consumed dependency, so even unchanged geometry cannot reuse a
        # previously accepted export after its completion receipt changes.
        key=(provenance['input_sha256'],provenance['source_sha256'],
            hashlib.sha256(request['input'].with_name('result.json').read_bytes()).hexdigest(),str(self.native_code),str(self.assets))
        cached=getattr(self,'loaded_geometry',None)
        reused=cached is not None and cached[0]==key
        if not reused:self.loaded_geometry=(key,load(request['input'],parent,self.native_code,self.assets))
        return (*self.loaded_geometry[1],reused)

    def plan(self,request):
        from motiongen_plan import plan_loaded,write_new
        output=request['output'];output.mkdir(parents=True,exist_ok=False)
        began=time.perf_counter();updates=0
        report=dict(type='result',status='failed',passed=False,offline_candidate=not self.formal,output=str(output),backend='curobo_motiongen',
            speed_scale=request['speed_scale'],gpu_ik=True,online_cpu_mesh_audit=False)
        try:
            parent=json.loads(request['source'].read_text())
            binding=json.loads(request['physical_binding'].read_text()) if request.get('physical_binding') else None
            provenance={key+'_sha256':hashlib.sha256(request[key].read_bytes()).hexdigest() for key in ('input','source')}
            provenance.update(parent_source_sha256=provenance['source_sha256'],planning_input_sha256=provenance['input_sha256'],parent_source_schema=parent['schema_version'])
            if self.formal:
                from real_contract import is_device_source
                physical=is_device_source(parent)
                if physical:
                    from real_contract import validate_real_source
                    from joint_space import validate_initial_state
                    validate_real_source(parent,binding,environment=request.get('environment','physical'))
                    validate_initial_state(parent.get('initial_state'),parent['calibration_profile'])
                if (not physical and (parent.get('schema_version')!='dual_gpu_pipeline.v1' or parent.get('lane')!='simulated-real')
                        or any(parent.get(key) for key in ('offline_candidate','synthetic_fixture','test_fixture'))):
                    raise ValueError('formal worker requires a non-fixture simulation source')
                if not physical and binding is not None:raise ValueError('physical binding cannot relabel a simulation source')
                source=parent
            else:
                from verify_motiongen_chain import synthetic_source,seal_offline
                source=synthetic_source(parent,provenance)
            planner,observation,data,reused_geometry=self.load_geometry(request,parent,provenance)
            from robot_sim.asset_cfg.robots.ac_one_spec import ac_one_asset_spec
            def diagnostic(name,value):
                nonlocal updates
                if name=='motiongen-receipts-latest':name=f'motiongen-receipts-{updates:03d}.json';updates+=1
                write_new(output/name,value)
            base,plan,evidence=plan_loaded(planner,observation,data,source,ac_one_asset_spec(self.assets).urdf_path,
                speed_scale=request['speed_scale'],diagnostic=diagnostic,engine_cache=self.engine_cache,
                physical_binding=binding if self.formal else None,
                **({'environment':request['environment']} if request.get('environment') else {}))
            if self.formal and (base.get('offline_candidate') or plan.get('offline_candidate')):
                raise ValueError('formal worker cannot publish an offline candidate')
            if getattr(self,'compute_resource',None):
                from local_execution import stamp_plan
                stamp_plan(base,self.compute_resource);stamp_plan(plan,self.compute_resource)
                evidence['compute_resource']=dict(self.compute_resource,logical_cuda_index=0)
            write_new(output/'base-plan.json',base if self.formal else seal_offline(base,provenance))
            write_new(output/'plan.json',plan if self.formal else seal_offline(plan,provenance))
            if not self.formal:write_new(output/'planning-evidence.json',evidence)
            report.update(passed=True,status='motiongen-plan-ready',input_ticks=len(base['ticks']),output_ticks=len(plan['ticks']),timeline_digest=plan['timeline_digest'],
                geometry_inputs_reused=reused_geometry,
                engine_reused=evidence['engine_reused'],full_hot_planning_wall_seconds=evidence['full_hot_planning_wall_seconds'],
                input_sha256=provenance['input_sha256'],source_sha256=provenance['source_sha256'])
        except Exception as error:
            report['error']=f'{type(error).__name__}: {error}'
            report['fatal_backend_error']=isinstance(error,(RuntimeError,MemoryError))
        report['total_wall_seconds']=time.perf_counter()-began
        durable=dict(report)
        if self.formal and report['passed']:
            durable.update(**evidence,hardware_access=False,elapsed_seconds=report['total_wall_seconds'])
        write_new(output/'result.json',durable)
        return report


def process_start_ticks():
    return Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19]


def serve(worker, spec, read_roots, output_root, ready):
    """Sequential requests; stdin EOF and a bounded lifetime own this process."""
    from motiongen_plan import write_new
    from planner_service import validate_spec
    validate_spec(spec,private=True)
    root=Path(spec['root']); root.mkdir(mode=0o700,exist_ok=False)
    endpoint=Path(spec['socket'])
    state=dict(**spec,status='ready',pid=os.getpid(),
        spec_sha256=hashlib.sha256(json.dumps(spec,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
        process_start_ticks=process_start_ticks(),
        model_initialized=False)
    def status(**values):
        state.update(values)
        temporary=root/'status.partial'
        temporary.write_text(json.dumps(state,allow_nan=False))
        os.chmod(temporary,0o600);temporary.replace(root/'status.json')
        emit(dict(type='service-state',service={k:v for k,v in state.items() if k!='owner_token'}))
    deadline=time.monotonic()+spec['max_lifetime_seconds']
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    try:
        server.bind(str(endpoint)); os.chmod(endpoint,0o600); server.listen(1)
        status(); emit(dict(**ready,service={k:v for k,v in state.items() if k!='owner_token'}))
        while time.monotonic()<deadline:
            available,_,_=select.select([server,sys.stdin],[],[],max(0.,min(1.,deadline-time.monotonic())))
            if sys.stdin in available and not os.read(sys.stdin.fileno(),1):break
            if server not in available:continue
            connection,_=server.accept()
            with connection:
                connection.settimeout(30)
                response=dict(type='result',status='failed',passed=False)
                try:
                    with connection.makefile('rb') as stream:line=stream.readline(MAX_MESSAGE_BYTES+1)
                    if len(line)>MAX_MESSAGE_BYTES:raise ValueError('oversized service request')
                    message=json.loads(line,object_pairs_hook=unique_object,parse_constant=reject_constant)
                    if not isinstance(message,dict) or set(message)!={'owner_token','request'} or message['owner_token']!=spec['owner_token']:
                        raise ValueError('planner request ownership mismatch')
                    request=request_paths(json.dumps(message['request']),read_roots,output_root)
                    status(status='busy')
                    with progress_to_stderr():response=worker.plan(request)
                except Exception as error:response['error']=f'{type(error).__name__}: {error}'
                finally:status(status='ready',model_initialized=bool(worker.engine_cache.get('engine')))
                try:connection.sendall((json.dumps(response,allow_nan=False)+'\n').encode())
                except (BrokenPipeError,ConnectionResetError):
                    # The completed or failed request remains in its unique run;
                    # a disconnected caller cannot cause automatic replanning.
                    pass
                if response.get('fatal_backend_error'):break
    finally:
        status(status='stopped'); server.close()
        if endpoint.exists():endpoint.unlink()


def request_service(endpoint, owner_token, request, *, timeout=600):
    """A client has no Torch/Isaac imports and cannot launch or restart a worker."""
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(endpoint)
        payload=json.dumps(dict(owner_token=owner_token,request=request),allow_nan=False)+'\n'
        if len(payload.encode())>MAX_MESSAGE_BYTES:raise ValueError('oversized service request')
        connection.sendall(payload.encode())
        with connection.makefile('rb') as stream:line=stream.readline(MAX_MESSAGE_BYTES+1)
        if not line or len(line)>MAX_MESSAGE_BYTES:raise ValueError('missing/oversized planner response')
        result=json.loads(line,object_pairs_hook=unique_object,parse_constant=reject_constant)
        if (not isinstance(result,dict) or not isinstance(result.get('output'),str)
                or Path(result['output']).resolve()!=Path(request['output']).resolve()):
            raise ValueError('planner result request identity differs')
        return result


def self_check():
    """Path/protocol boundary regression, with no Torch, CUDA or FCL import."""
    import tempfile
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory).resolve()
        reads, outputs, outside = (root/name for name in ('reads', 'outputs', 'outside'))
        for path in (reads, outputs, outside):
            path.mkdir()
        for name in ('planning-input.json', 'source.json', 'result.json'):
            (reads/name).write_text('{}')
        (outside/'secret.json').write_text('{}')
        (reads/'escape.json').symlink_to(outside/'secret.json')
        (outputs/'escape').symlink_to(outside, target_is_directory=True)
        (outputs/'existing').mkdir()
        message = dict(input=str(reads/'planning-input.json'), source=str(reads/'source.json'),
                       output=str(outputs/'new'), speed_scale=.3)
        good = request_paths(json.dumps(message), (reads,), outputs)
        assert good['output'] == outputs/'new' and not good['output'].exists()
        bad = [dict(message, input='relative'), dict(message, source=str(reads/'escape.json')),
               dict(message, output=str(outputs/'escape'/'new')), dict(message, output=str(outputs/'existing')),
               dict(message, output=str(outputs/'..'/'outside'/'new')), dict(message, speed_scale=True),
               dict(message, speed_scale=.09), dict(message, extra=1), [], {'input': message['input']}]
        lines = [json.dumps(value) for value in bad]
        lines += ['{"input":"one","input":"two"}', json.dumps(message).replace('0.3', 'NaN'),
                  ' '*(MAX_MESSAGE_BYTES+1)]
        for line in lines:
            try:
                request_paths(line, (reads,), outputs)
            except (ValueError, OSError):
                pass
            else:
                raise AssertionError(f'unsafe message accepted: {line[:100]}')
        assert not good['output'].exists(), 'validation must not create output'
    print('offline worker path/protocol self-check passed; no CUDA imported')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-check', action='store_true')
    parser.add_argument('--service-spec-file',type=Path)
    parser.add_argument('--spec-sha256')
    parser.add_argument('--request',type=json.loads)
    for name in ('native-code', 'assets', 'allow-output-root', 'warmup-input', 'warmup-source'):
        parser.add_argument('--'+name)
    parser.add_argument('--allow-read-root', action='append')
    args = parser.parse_args()
    args.service_spec=None
    if args.self_check:
        self_check()
        return 0
    if args.request is not None:
        if not args.service_spec_file or not args.spec_sha256:parser.error('request requires private service spec and SHA')
        from planner_service import load_private_spec
        spec=load_private_spec(args.service_spec_file,args.spec_sha256)
        result=request_service(spec['socket'],spec['owner_token'],args.request)
        emit(result)
        return 0 if result.get('passed') is True and result.get('status')=='motiongen-plan-ready' else 1
    if any(getattr(args, name) is None for name in ('native_code', 'assets', 'allow_read_root', 'allow_output_root')):
        parser.error('native-code/assets/allow-read-root/allow-output-root are required')
    if bool(args.warmup_input) != bool(args.warmup_source):
        parser.error('warmup-input and warmup-source must be supplied together')
    started = time.perf_counter()
    try:
        roots = tuple(allowed_path(value, (Path('/'),)) for value in args.allow_read_root)
        output_root = allowed_path(args.allow_output_root, (Path('/'),))
        native_code, assets = (allowed_path(value, roots) for value in (args.native_code, args.assets))
        if not all(path.is_dir() for path in (*roots, output_root, native_code, assets)):
            raise ValueError('configured roots/native-code/assets must be existing directories')
        if args.service_spec_file:
            from planner_service import load_private_spec
            args.service_spec=load_private_spec(allowed_path(str(args.service_spec_file),(output_root,)),args.spec_sha256)
        warm_input = warm_source = None
        if args.warmup_input:
            warm_input, warm_source = (allowed_path(value, roots) for value in (args.warmup_input, args.warmup_source))
            allowed_path(str(warm_input.with_name('result.json')), roots)
        with progress_to_stderr():
            worker = Worker(native_code, assets,formal=args.service_spec is not None)
            worker.compute_resource=(args.service_spec or {}).get('compute_resource')
            warmup = worker.warmup(warm_input, warm_source)
        ready=dict(type='ready',passed=True,offline_candidate=args.service_spec is None,startup_seconds=time.perf_counter()-started,**warmup)
        if args.service_spec is not None:
            if tuple(Path(p).resolve() for p in args.service_spec['allowed_read_roots'])!=roots or Path(args.service_spec['allowed_output_root']).resolve()!=output_root:
                raise ValueError('service and command path boundaries differ')
            def stop_service(signum,_frame):raise SystemExit(128+signum)
            signal.signal(signal.SIGTERM,stop_service)
            return serve(worker,args.service_spec,roots,output_root,ready) or 0
        emit(ready)
    except Exception as error:
        emit(dict(type='ready', passed=False, offline_candidate=True, error=f'{type(error).__name__}: {error}'))
        return 1
    while True:
        line = sys.stdin.readline(MAX_MESSAGE_BYTES+1)
        if not line:
            return 0
        try:
            request = request_paths(line, roots, output_root)
            with progress_to_stderr():
                result = worker.plan(request)
        except Exception as error:
            result = dict(type='result', passed=False, offline_candidate=True, error=f'{type(error).__name__}: {error}')
        emit(result)
        if len(line.encode('utf-8')) > MAX_MESSAGE_BYTES:
            return 1  # Do not treat a truncated oversized message as another request.


if __name__ == '__main__':
    raise SystemExit(main())
