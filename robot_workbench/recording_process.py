"""Pure CPU recording child; the controller keeps only bounded events and refs.

Private stdin/stdout pipes carry JSON, never camera pixels or compiled arrays.
The existing recorder still owns validation, codecs, manifests and failure rules.
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import queue
import secrets
import select
import signal
import subprocess
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1]
LIMIT=4*1024*1024


def _encode(value,limit=LIMIT):
    raw=json.dumps(value,allow_nan=False,separators=(',',':'))+'\n'
    if len(raw.encode())>limit:raise ValueError('bounded recording IPC message required')
    return raw


def _timeline(value):
    if value is None:return None
    ref=value.get('execution_timeline_ref')
    if not isinstance(ref,dict):raise ValueError('isolated recording requires execution_timeline_ref, not control arrays')
    return dict(ref)


def _environment():
    from shutil import which
    from launch_environment import clean_environment, HOST_PATH
    paths=[ROOT,ROOT/'rule_based_workbench',ROOT/'calibration',ROOT/'calibration/infra']
    # Keep the deployment-selected codecs, not its Python/CUDA runtime paths.
    # Falling back to the system FFmpeg can silently select an older encoder.
    codec_dirs=[str(Path(found).absolute().parent) for name in ('ffmpeg','ffprobe') if (found:=which(name))]
    return clean_environment(dict(PYTHONPATH=os.pathsep.join(map(str,paths)),CUDA_VISIBLE_DEVICES='',
        PATH=os.pathsep.join(dict.fromkeys([*codec_dirs,*HOST_PATH.split(os.pathsep)])),
        OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1'))


def export_playback_process(raw_root,output,*,environment,python=None):
    """Export/verify in a CPU child so large raw indexes never enter Owner Python."""
    executable=python or os.environ.get('WORKBENCH_CPU_PYTHON') or sys.executable
    if not Path(executable).is_absolute():raise ValueError('absolute CPU Python interpreter required')
    with (Path(raw_root).parent/('playback-worker-'+secrets.token_hex(6)+'.log')).open('xb') as log:
        child=subprocess.Popen([executable,'-u','-B','-m',__name__,'export'],stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=log,text=True,env=_environment(),start_new_session=True)
        try:
            result,_=child.communicate(_encode(dict(raw_root=str(raw_root),output=str(output),environment=environment)),timeout=600)
            if child.returncode or len(result)>LIMIT:raise ValueError('isolated playback export failed; inspect '+log.name)
            return json.loads(result)
        except BaseException:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(5)
            raise


class ProcessRecordingSession:
    def __init__(self,root,*,plan_ref=None,source=None,binding=None,session_kind='frozen',
                 camera_channels=None,camera_directories_path=None,camera_directories=None,
                 on_fault=lambda reason:None,preparation_active=lambda:True,python=None,
                 execution_timeline=None,execution_owner=None,execution_instance_id=None,
                 defer_execution=True,preparation_deadline_monotonic=None,environment=None,
                 storage_format='segments',persistent_service=False,max_wall_time_seconds=None,
                 resident_deadline_wall_ns=None):
        from real_recording import RecordingEventQueue
        self.root=Path(root)
        if not self.root.is_absolute():raise ValueError('absolute recording output required')
        if session_kind not in ('frozen','streaming'):raise ValueError('known recording session kind required')
        if session_kind=='frozen' and not isinstance(plan_ref,dict):raise ValueError('verified plan_ref required')
        self.execution_owner=deepcopy(execution_owner)
        self.execution_instance_id=execution_instance_id or (source or {}).get('execution_instance_id')
        self.environment=environment or (source or {}).get('environment') or (plan_ref or {}).get('environment')
        self.python=python or os.environ.get('WORKBENCH_CPU_PYTHON') or sys.executable
        if not Path(self.python).is_absolute():raise ValueError('absolute CPU Python interpreter required')
        self.camera_channels=camera_channels;self.camera_service=None
        descriptor=None;expected_instance=None
        if camera_channels is not None:
            services={id(channel.service):channel.service for channel in camera_channels.values()}
            if len(services)!=1:raise ValueError('one shared camera service required')
            self.camera_service=next(iter(services.values()))
            descriptor=str(self.camera_service.descriptor_path or Path(next(iter(self.camera_service.directories.values()))).parent/'camera-service.json')
            expected_instance=self.camera_service.descriptor['instance_id']
        elif camera_directories_path is None and camera_directories is None:raise ValueError('recording camera source required')
        options=dict(environment=self.environment,execution_owner=self.execution_owner,
            execution_instance_id=self.execution_instance_id,defer_execution=defer_execution,
            preparation_deadline_monotonic=preparation_deadline_monotonic or time.monotonic()+30,
            storage_format=storage_format,persistent_service=persistent_service,
            max_wall_time_seconds=max_wall_time_seconds,resident_deadline_wall_ns=resident_deadline_wall_ns)
        self.setup=dict(schema='recording_process.v1',root=str(self.root),kind=session_kind,plan_ref=deepcopy(plan_ref),
            source=deepcopy(source),binding=deepcopy(binding),camera_descriptor=descriptor,
            expected_camera_instance=expected_instance,camera_directories_path=str(camera_directories_path) if camera_directories_path else None,
            camera_directories=camera_directories,execution_timeline_ref=_timeline(execution_timeline),options=options)
        self.preparation_active=preparation_active;self.on_fault=on_fault;self.defer_execution=defer_execution
        self.preparation_deadline_monotonic=options['preparation_deadline_monotonic']
        self.prepared,self.ready,self.done,self.cancelled,self.execution_bound=(threading.Event() for _ in range(5))
        self.events=RecordingEventQueue(max_bytes=64*1024**2)
        self.lock=threading.RLock();self.error=self.seal_error=self.manifest=self.controller_result=None
        self.terminal=None;self.lease_tokens=set();self.channel_tokens={};self.process=None;self.worker_pid=None
        self.clock=None;self.cache=dict(ready=False,done=False,error=None,storage_format=storage_format)
        self._finished=threading.Event();self._forced=None;self._fault_notified=False
        self.thread=threading.Thread(target=self._bridge,name='recording-process-bridge',daemon=True);self.thread.start()

    def _error(self,reason):
        with self.lock:
            self.error=self.error or str(reason);self.cancelled.set()
            first=not self._fault_notified;self._fault_notified=True
        if first:self.on_fault(self.error)

    def _enqueue(self,value):
        try:self.events.put_nowait(value)
        except queue.Full:
            self._error('bounded recording process event queue overflow')
            self._forced=dict(action='fail',reason=self.error)
            raise ValueError(self.error)

    def preparation_current(self):
        return (not self.done.is_set() and not self.cancelled.is_set() and self.preparation_active()
                and (self.prepared.is_set() or time.monotonic()<self.preparation_deadline_monotonic))

    def lease_active(self,lease):
        return (self.preparation_current() and lease.get('run_id') in self.lease_tokens
            and lease.get('execution_instance_id')==self.execution_instance_id and lease.get('execution_owner')==self.execution_owner)

    def bind_execution(self,timeline,*,execution_instance_id,execution_owner):
        with self.lock:
            if (not self.preparation_current() or not self.prepared.is_set() or self.execution_bound.is_set()
                    or execution_instance_id!=self.execution_instance_id or execution_owner!=self.execution_owner):
                raise ValueError('one current recording timeline binding required')
            self._enqueue(dict(action='bind',reference=_timeline(timeline),execution_instance_id=execution_instance_id,execution_owner=execution_owner))
            self.execution_bound.set()

    def submit(self,kind,**fields):
        with self.lock:
            if self.error or self.terminal or self.done.is_set():raise ValueError('recording is closing or failed')
            if kind not in ('command','feedback','native_window'):raise ValueError('camera pixels must go directly to the recording child')
            self._enqueue(dict(action='event',kind=kind,fields=fields))

    def append_command(self,**fields):self.execution_bound.set();return self.submit('command',**fields)
    def append_feedback(self,**fields):return self.submit('feedback',**fields)
    def start(self):return self
    def status(self):return self.snapshot()

    def finish(self,status,reason=None):
        if status not in ('completed','failed','stopped'):raise ValueError('explicit terminal recording status required')
        with self.lock:
            if self.terminal is not None:return
            self.terminal=(status,reason,time.monotonic());self.controller_result=dict(status=status,reason=reason)
            if status!='completed':self.cancelled.set()
            self._enqueue(dict(action='finish',status=status,reason=reason))

    def fail(self,reason):
        self._error(reason);self._forced=dict(action='fail',reason=str(reason))

    def snapshot(self):
        with self.lock:value=deepcopy(self.cache)
        value.update(error=self.error or value.get('error'),done=self.done.is_set(),worker_pid=self.worker_pid,
            controller_result=self.controller_result or value.get('controller_result'),directory=str(self.root),
            transport='isolated_cpu_recording_process',parent_event_queue=self.events.snapshot())
        return value

    def _attach_tokens(self,tokens):
        if self.camera_channels is None or tokens==self.channel_tokens:return True
        for name,token in tokens.items():
            lease=self.camera_channels[name].current_lease()
            if lease is None:return False  # Child may already have sealed after this status sample.
            if (lease.get('run_id')!=token or lease.get('execution_owner')!=self.execution_owner
                    or lease.get('execution_instance_id')!=self.execution_instance_id):
                raise ValueError('recording child lease differs from current Owner')
        with self.camera_service.binding_lock:
            if self.camera_service.descriptor['instance_id']!=self.setup['expected_camera_instance']:raise ValueError('camera binding changed during recording preparation')
            for name,token in tokens.items():
                channel=self.camera_channels[name]
                if channel.cached_token not in (None,token):raise ValueError('camera client belongs to another recording')
                channel.cached_token=token;self.camera_service.known_tokens.add(token)
        self.channel_tokens=dict(tokens);self.lease_tokens=set(tokens.values())
        return True

    def _sender(self):
        try:
            while not self._finished.is_set():
                if self._forced is not None:
                    value=self._forced;self._forced=None
                else:
                    try:value=self.events.get(timeout=.05)
                    except queue.Empty:
                        if self.terminal is None and not self.preparation_current():
                            if self.prepared.is_set():self.finish('stopped','recording Owner revoked')
                            else:self.fail('recording preparation expired or Owner revoked')
                        continue
                self._write(value,512*1024)
        except (OSError,ValueError,TypeError) as error:
            if not self._finished.is_set():self._error(error)
        finally:
            try:self.process.stdin.close()
            except OSError:pass

    def _write(self,value,limit=LIMIT):
        payload=_encode(value,limit).encode();deadline=time.monotonic()+5;fd=self.process.stdin.fileno()
        while payload:
            if time.monotonic()>=deadline:raise TimeoutError('recording child command pipe stalled')
            if not select.select([], [fd], [], .1)[1]:continue
            try:count=os.write(fd,payload)
            except BlockingIOError:continue
            payload=payload[count:]

    def _bridge(self):
        sender=None
        try:
            self.root.parent.mkdir(parents=True,exist_ok=True)
            log=self.root.parent/('recording-worker-'+secrets.token_hex(6)+'.log')
            with log.open('xb') as stream:
                self.process=subprocess.Popen([self.python,'-u','-B','-m','robot_workbench.recording_process'],
                    stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=stream,text=True,bufsize=1,env=_environment(),start_new_session=True)
            self.worker_pid=self.process.pid
            os.set_blocking(self.process.stdin.fileno(),False)
            self._write(self.setup)
            sender=threading.Thread(target=self._sender,name='recording-process-events',daemon=True);sender.start()
            while True:
                line=self.process.stdout.readline(LIMIT+1)
                if not line:break
                if len(line)>LIMIT or not line.endswith('\n'):raise ValueError('recording status exceeds IPC bound')
                value=json.loads(line)
                if value.get('worker_pid')!=self.worker_pid:raise ValueError('recording worker identity changed')
                state=value['status']
                attached=True
                if value.get('prepared') and not state.get('done'):attached=self._attach_tokens(value['channel_tokens'])
                with self.lock:
                    self.cache=state;self.clock=value.get('clock');self.manifest=value.get('manifest')
                    self.seal_error=state.get('seal_error')
                if self.camera_channels is None:self.lease_tokens=set(value.get('lease_tokens',[]))
                if value.get('prepared') and attached:self.prepared.set()
                if state.get('ready'):self.ready.set()
                if state.get('error'):self._error(state['error']);self._finished.set()
                if state.get('done'):break
            self._finished.set()
            if sender is not None:sender.join(1)
            code=self.process.wait(timeout=5)
            if code or not self.cache.get('done'):self._error('recording child exited before a complete terminal receipt; inspect '+str(log))
        except Exception as error:
            self._error(error)
        finally:
            self._finished.set()
            if self.process is not None:
                try:self.process.stdin.close()
                except (OSError,ValueError):pass
                if sender is not None:sender.join(1)
                if self.process.poll() is None:
                    # Only this pure CPU recording session and its codecs; never
                    # the shared camera service, controller or another job.
                    try:self.process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        self._error('recording CPU child did not seal after EOF; retained incomplete prefix')
                        try:os.killpg(self.process.pid,signal.SIGTERM)
                        except ProcessLookupError:pass
                        try:self.process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(self.process.pid,signal.SIGKILL);self.process.wait(timeout=5)
                self.process.stdout.close()
            if self.camera_channels is not None:
                for name,channel in self.camera_channels.items():
                    try:
                        if self.camera_service.descriptor['instance_id']!=self.setup['expected_camera_instance']:
                            raise ValueError('camera service instance changed before recording cleanup')
                        token=self.channel_tokens.get(name);lease=channel.current_lease()
                        # A crash after begin but before its status reply may have
                        # left a partial lease. Recover only this exact execution.
                        if lease and lease.get('execution_owner')==self.execution_owner and lease.get('execution_instance_id')==self.execution_instance_id:
                            token=lease['run_id']
                        if token is not None:channel.end(token)
                    except (OSError,ValueError) as error:self._error('recording lease cleanup unknown: '+str(error))
            self.done.set()


def _worker():
    sys.path[:0]=[str(ROOT/'rule_based_workbench'),str(ROOT/'calibration'),str(ROOT/'calibration/infra')]
    from contextlib import redirect_stdout
    from real_recording import RecordingSession,StreamingRecordingSession
    from robot_workbench.control_plan import load_artifact
    from robot_workbench.rgbd_service import CommonCameraService
    setup=json.loads(sys.stdin.readline(LIMIT+1));output=sys.stdout;session=None;active=threading.Event();active.set()
    if setup.get('schema')!='recording_process.v1':raise ValueError('known recording IPC setup required')
    options=setup['options'];owner=options['execution_owner'];ref=setup.get('plan_ref')
    try:
        if ref:
            if ref.get('origin')!=dict(owner_pid=owner['owner_pid'],runtime_nonce=owner['runtime_nonce'],epoch=owner['command_epoch']):
                raise ValueError('recording plan reference belongs to another Owner/epoch')
            source=load_artifact(ref['source']);plan=load_artifact(ref['plan']);binding=load_artifact(ref['binding']) if ref.get('binding') else {}
        else:source=setup['source'];plan=None;binding=setup['binding']
        if setup.get('camera_descriptor'):
            cameras=CommonCameraService.connect(setup['camera_descriptor'])
            if cameras.descriptor['instance_id']!=setup['expected_camera_instance']:raise ValueError('recording camera service changed')
            camera_options=dict(camera_channels=cameras.channels)
        else:
            directories=json.loads(Path(setup['camera_directories_path']).read_text()) if setup.get('camera_directories_path') else setup['camera_directories']
            camera_options=dict(camera_directories=directories)
        with redirect_stdout(sys.stderr):
            if setup['kind']=='streaming':
                session=StreamingRecordingSession(setup['root'],source,binding,execution_owner=owner,
                    preparation_active=active.is_set,max_wall_time_seconds=options.get('max_wall_time_seconds') or 600,
                    storage_format=options['storage_format'],**camera_options)
            else:
                timeline=load_artifact(setup['execution_timeline_ref']) if setup.get('execution_timeline_ref') else None
                session=RecordingSession(setup['root'],source,plan,binding,preparation_active=active.is_set,
                    execution_timeline=timeline,**options,**camera_options)
        def commands():
            try:
                while not session.done.is_set():
                    line=sys.stdin.readline(512*1024+1)
                    if not line:session.finish('stopped','recording parent pipe closed');active.clear();return
                    if len(line)>512*1024 or not line.endswith('\n'):raise ValueError('recording command exceeds bound')
                    value=json.loads(line)
                    with redirect_stdout(sys.stderr):
                        if value['action']=='event':
                            if setup['kind']=='streaming':getattr(session,'append_'+value['kind'])(**value['fields'])
                            else:session.submit(value['kind'],**value['fields'])
                        elif value['action']=='bind':session.bind_execution(load_artifact(value['reference']),execution_instance_id=value['execution_instance_id'],execution_owner=value['execution_owner'])
                        elif value['action']=='finish':session.finish(value['status'],value.get('reason'))
                        elif value['action']=='fail':session.fail(value['reason']);active.clear()
                        else:raise ValueError('unknown recording command')
            except Exception as error:session.fail(error);active.clear()
        command_thread=threading.Thread(target=commands,daemon=True);command_thread.start()
        for signum in (signal.SIGTERM,signal.SIGINT):signal.signal(signum,lambda *_:session.finish('stopped','recording worker termination requested'))
        while True:
            state=session.snapshot();manifest=session.manifest
            value=dict(worker_pid=os.getpid(),status=state,prepared=session.prepared.is_set(),clock=session.clock,
                channel_tokens=dict(session.channel_tokens),lease_tokens=sorted(session.lease_tokens),
                manifest={key:item for key,item in manifest.items() if key!='files'} if manifest else None)
            output.write(_encode(value));output.flush()
            if session.done.wait(.05):
                if state.get('done'):break
        session.thread.join(2)
        # Parent closes its command pipe after observing done. Let the input
        # reader release Python's buffered stdin lock before interpreter exit.
        command_thread.join(2)
        if command_thread.is_alive():os._exit(1)
    except Exception as error:
        if session is not None:
            session.fail(error)
            if not session.done.wait(30):
                # If the parent vanished, no supervisor remains to reap our
                # codec children. Abort only this recorder's owned writers.
                if session.recorder is not None:session.recorder.abort()
                session.done.wait(5)
        try:
            output.write(_encode(dict(worker_pid=os.getpid(),status=dict(done=True,ready=False,error=str(error)),prepared=False,channel_tokens={})))
            output.flush()
        except BrokenPipeError:pass


if __name__=='__main__':
    if sys.argv[1:]==['export']:
        from contextlib import redirect_stdout
        from real_recording import export_playback
        request=json.loads(sys.stdin.readline(LIMIT+1));output=sys.stdout
        with redirect_stdout(sys.stderr):result=export_playback(request['raw_root'],None,request['output'],environment=request['environment'])
        output.write(_encode(result));output.flush()
    else:_worker()
