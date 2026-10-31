"""Private observed-scene reset/export service. No plant handle, plans, or motion RPC."""
import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import socket
import sys
import time
from types import SimpleNamespace

from curobo_worker import (MAX_MESSAGE_BYTES,allowed_path,unique_object,reject_constant,
    progress_to_stderr,emit,process_start_ticks,request_service)
from planner_service import load_private_spec


def request_paths(line,read_roots,output_root):
    if len(line.encode())>MAX_MESSAGE_BYTES:raise ValueError('oversized scene request')
    value=json.loads(line,object_pairs_hook=unique_object,parse_constant=reject_constant)
    if not isinstance(value,dict) or set(value)!={'source','suite','physical_binding','output'}:
        raise ValueError('scene request requires source/suite/physical_binding/output only')
    result={key:allowed_path(value[key],read_roots) for key in ('source','suite','physical_binding')}
    if any(not path.is_file() for path in result.values()):raise ValueError('scene inputs must be files')
    result['output']=allowed_path(value['output'],(output_root,),exists=False)
    if result['output'].exists() or Path(value['output']).is_symlink():raise FileExistsError('scene output exists')
    return result


def reset_suite(source,suite,binding,output):
    """Name this isolated reset without changing the validated external episode."""
    from integration import _digest
    identity=dict(source_episode_id=suite['episodes'][0]['episode_id'],
        input_digest=_digest(dict(source=source,suite=suite,physical_binding=binding)))
    identity['reset_episode_id']='scene-reset-'+_digest(dict(input_digest=identity['input_digest'],
        output=str(Path(output).resolve())))[:32]
    result=deepcopy(suite);result['episodes'][0]['episode_id']=identity['reset_episode_id']
    return result,identity


class Worker:
    def __init__(self,options,registration=None):
        self.options=options;self.app=self.env=self.runtime=self.planner=None
        self.model_key=None;self.control=self.acceptance=self.materials=None
        self.registration=registration

    def prepare(self,request):
        from integration import _digest
        from playground_sim import validate_inputs,setup,reset_scene,first_frame,plan_episode,write
        started=time.monotonic();output=request['output'];output.mkdir(parents=True,exist_ok=False)
        report=dict(status='failed',passed=False,output=str(output),task_outcome='unverified',
            model_initialized=False,policy_planned=False,motion_requested=False)
        try:
            source=json.loads(request['source'].read_text());suite=json.loads(request['suite'].read_text())
            binding=json.loads(request['physical_binding'].read_text())
            validate_inputs(source,binding,suite,environment=self.options.environment)
            internal_suite,reset_identity=reset_suite(source,suite,binding,output)
            report.update(reset_identity)
            if hashlib.sha256(self.options.finger_envelope.read_bytes()).hexdigest()!=self.options.finger_envelope_sha256:
                raise ValueError('pinned native finger envelope changed')
            key=_digest(dict(calibration=source['calibration_profile'],machine_asset=suite['episodes'][0]['machine_asset'],
                scene_asset_set=(source.get('scene') or {}).get('asset_set'),
                finger_envelope_sha256=self.options.finger_envelope_sha256,assets=str(self.options.assets),
                cup_contact_preset=self.options.cup_contact_preset,environment=self.options.environment))
            if self.model_key is not None and self.model_key!=key:
                raise ValueError('scene model/calibration changed; explicitly prepare a new service')
            args=SimpleNamespace(**vars(self.options));args.output=output;args.mode='geometry-inputs'
            args.source=request['source'];args.suite=request['suite'];args.hardware_binding=request['physical_binding']
            args.run_id=source['run_id'];args.plan=None;args.execution_timeline=None;args.speed_scale=.3
            args.scene_reset_identity=reset_identity
            reused=self.env is not None
            if not reused:
                self.app,self.env,self.runtime,episode,self.control,self.acceptance,info,self.materials=setup(args,source,internal_suite,None)
                self.model_key=key
                if self.registration:self.registration('ready')
            else:episode,info=reset_scene(args,source,internal_suite,self.env,self.runtime,self.acceptance)
            reset_done=time.monotonic()
            first_frame(args,source,self.env,self.runtime,self.materials)
            render_done=time.monotonic()
            self.planner=plan_episode(args,source,self.runtime,episode,self.control,self.acceptance,info,planner=self.planner)
            report.update(status='geometry-inputs-ready',passed=True,model_initialized=True,model_key=key,
                scene_reused=reused,source_digest=_digest(source),worker_identity=self.identity,
                timings_seconds=dict(reset=reset_done-started,render=render_done-reset_done,
                    export=time.monotonic()-render_done,total=time.monotonic()-started))
        except Exception as error:
            report.update(error=f'{type(error).__name__}: {error}',fatal_backend_error=isinstance(error,(RuntimeError,MemoryError)))
        write(output/'result.json',report)
        return report

    def close(self):
        if self.app is not None:
            from isaac_lane import close_runtime
            close_runtime(self.env,self.app)


def serve(worker,spec,spec_sha256):
    roots=tuple(Path(p).resolve() for p in spec['allowed_read_roots']);output_root=Path(spec['allowed_output_root']).resolve()
    root=Path(spec['root']);endpoint=Path(spec['socket'])
    if not root.is_dir() or endpoint.exists():raise ValueError('owned scene scope unavailable')
    state=dict(**{k:v for k,v in spec.items() if k!='owner_token'},spec_sha256=spec_sha256,
        status='ready',pid=os.getpid(),process_start_ticks=process_start_ticks(),model_initialized=False)
    worker.identity=dict(pid=state['pid'],process_start_ticks=state['process_start_ticks'],identity=spec['identity'])
    def status(**values):
        state.update(values);temporary=root/'status.partial'
        temporary.write_text(json.dumps(state,allow_nan=False));temporary.chmod(0o600);temporary.replace(root/'status.json')
        emit(dict(type='service-state',service=state))
    deadline=None if spec.get('persistent_service') else time.monotonic()+spec['max_lifetime_seconds']
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    try:
        server.bind(str(endpoint));endpoint.chmod(0o600);server.listen(1);status()
        while deadline is None or time.monotonic()<deadline:
            available,_,_=select.select([server,sys.stdin],[],[],1. if deadline is None else max(0.,min(1.,deadline-time.monotonic())))
            if sys.stdin in available and not os.read(sys.stdin.fileno(),1):break
            if server not in available:continue
            connection,_=server.accept()
            with connection:
                response=dict(status='failed',passed=False)
                try:
                    connection.settimeout(30)
                    with connection.makefile('rb') as stream:line=stream.readline(MAX_MESSAGE_BYTES+1)
                    if len(line)>MAX_MESSAGE_BYTES:raise ValueError('oversized scene request')
                    value=json.loads(line,object_pairs_hook=unique_object,parse_constant=reject_constant)
                    if not isinstance(value,dict) or set(value)!={'owner_token','request'} or value['owner_token']!=spec['owner_token']:
                        raise ValueError('scene ownership mismatch')
                    request=request_paths(json.dumps(value['request']),roots,output_root)
                    status(status='busy')
                    with progress_to_stderr():response=worker.prepare(request)
                except Exception as error:response['error']=f'{type(error).__name__}: {error}'
                finally:status(status='ready',model_initialized=worker.model_key is not None,model_key=worker.model_key)
                try:connection.sendall((json.dumps(response,allow_nan=False)+'\n').encode())
                except (BrokenPipeError,ConnectionResetError):pass
                if response.get('fatal_backend_error'):break
    finally:
        status(status='stopped');server.close();endpoint.unlink(missing_ok=True)
        with progress_to_stderr():worker.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=json.loads)
    parser.add_argument('--service-spec-file',type=Path,required=True);parser.add_argument('--spec-sha256',required=True)
    for name in ('assets','native-code','finger-envelope','output'):parser.add_argument('--'+name,type=Path)
    parser.add_argument('--finger-envelope-sha256');parser.add_argument('--gpu',type=int);parser.add_argument('--instance')
    parser.add_argument('--environment',choices=('physical','fake'),default='physical')
    parser.add_argument('--execution-json');parser.add_argument('--cup-contact-preset',type=json.loads)
    args=parser.parse_args();spec=load_private_spec(args.service_spec_file,args.spec_sha256,kind='scene')
    if args.request is not None:
        result=request_service(spec['socket'],spec['owner_token'],args.request)
        emit(result);return 0 if result.get('passed') is True else 1
    roots=tuple(Path(p).resolve() for p in spec['allowed_read_roots'])
    for name in ('assets','native_code'):
        value=getattr(args,name)
        if value is None:parser.error('scene worker assets/native-code required')
        path=allowed_path(str(value),roots)
        if not path.is_dir():raise ValueError('scene '+name+' must be an authorized directory')
        expected=(spec.get('runtime_paths') or {}).get(name)
        if expected is not None and path!=Path(expected).resolve(strict=True):
            raise ValueError('scene '+name+' differs from frozen runtime')
        setattr(args,name,path)
    # A startup asset may be authorized as one file outside the IPC directories.
    # Its exact spec pin does not grant RPC access to that file or its parent.
    pin=spec['finger_envelope']
    if args.finger_envelope is None:parser.error('scene worker finger-envelope required')
    envelope=Path(pin['path']).resolve(strict=True)
    if (args.finger_envelope.resolve(strict=True)!=envelope or not envelope.is_file()
            or args.finger_envelope_sha256!=pin['sha256']
            or hashlib.sha256(envelope.read_bytes()).hexdigest()!=pin['sha256']):
        raise ValueError('scene finger envelope differs from exact startup pin')
    args.finger_envelope=envelope
    if args.output is None or args.gpu is None or not args.instance:parser.error('owned output/GPU/instance required')
    resource=spec.get('compute_resource')
    if resource and (args.gpu!=resource['gpu_index'] or args.instance!=resource['instance_id']):
        raise ValueError('scene GPU/instance differs from frozen resource')
    args.output=allowed_path(str(args.output),(Path(spec['allowed_output_root']).resolve(),),exists=False)
    args.output.mkdir(parents=True,exist_ok=False)
    registration=None
    if args.execution_json:
        from local_execution import register_scene
        registration=register_scene(args,__file__)
    def stop(signum,_):raise SystemExit(128+signum)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        return serve(Worker(args,registration),spec,args.spec_sha256) or 0
    finally:
        if registration:registration('stopped')


if __name__=='__main__':raise SystemExit(main())
