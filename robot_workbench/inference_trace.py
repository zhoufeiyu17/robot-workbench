"""Per-run inference evidence. Disk/JSON work stays outside the control process."""
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import queue
import re
import threading
import time
import weakref
from types import SimpleNamespace


def clean(value):
    if isinstance(value, dict):
        return {k:clean(v) for k,v in value.items() if k.lower() not in
                {'token','csrf','authorization','password','maintenance_token'}}
    if isinstance(value, (tuple,list)):return [clean(v) for v in value]
    if hasattr(value, 'tolist'):return clean(value.tolist())
    if isinstance(value,float) and not math.isfinite(value):return str(value)
    return value


def write_json(path, value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_suffix(path.suffix+'.tmp')
    with tmp.open('w') as stream:
        json.dump(clean(value),stream,ensure_ascii=False,allow_nan=False,indent=2)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    tmp.replace(path)


def compact(event, value):
    """Keep measured values and native outputs, not repeated static SDK metadata."""
    if event!='feedback':return deepcopy(value)
    raw=value['raw'];native=raw.get('controller_telemetry')or{}
    return dict(source_time=value['source_time'],device_state14=list(value['device_state14']),
        sample_time_s=raw.get('sample_time_s'),sample_wall_ns=raw.get('sample_wall_ns'),
        qpos=deepcopy(raw.get('qpos')),qvel=deepcopy(raw.get('qvel')),
        sdk_timestamps=deepcopy(raw.get('sdk_timestamps')),
        raw_state={side:{k:deepcopy(v) for k,v in arm.items() if k in
            ('qpos','qvel','effort','sdk_feedback_time_s','host_read_monotonic_ns','host_read_wall_ns','validity')}
            for side,arm in (raw.get('raw_state')or{}).get('arms',{}).items()},
        native_output={k:deepcopy(native[k]) for k in ('sample_time_s','sample_wall_ns','arms','observations') if k in native},
        feedback_read_timing={k:v for k,v in (raw.get('feedback_read_timing')or{}).items()
            if k in ('guard_call_s','diagnostics_s','raw_snapshot_s')})


def _writer(root, messages, ready, failed, parent):
    os.environ['CUDA_VISIBLE_DEVICES']=''
    root=Path(root);root.mkdir(parents=True,exist_ok=True);runs={}
    ready.set()
    def finish(identity, terminal, dropped=0):
        row=runs.get(identity)
        if row is None:return
        row['stream'].flush();os.fsync(row['stream'].fileno());row['stream'].close()
        row.update(closed=True,terminal=terminal)
        write_json(row['directory']/'trace-manifest.json',dict(schema='workbench.inference_trace.v1',
            execution_instance_id=identity,status='incomplete' if dropped else 'complete',
            terminal=terminal,event_counts=dict(row['counts']),dropped_records=dropped,
            file='trace.jsonl',sha256=row['hash'].hexdigest(),writer_pid=os.getpid(),
            first_recorded_wall_ns=row['first'],last_recorded_wall_ns=row['last'],
            units=dict(qpos='SDK radians; jaws SDK metres',device_state14='joint radians; jaw millimetres',
                       qvel='SDK rad/s; jaw m/s',native_output='pre-CAN SDK output; not firmware ACK'),
            acceleration='derive from timestamped velocity; not an independently measured sensor value'))
    try:
        while True:
            try:item=messages.get(timeout=.5)
            except queue.Empty:
                if os.getppid()!=parent:break
                continue
            if item is None:break
            identity,event,value,stamp,dropped=item
            if identity not in runs:
                directory=root/('inference-'+identity);directory.mkdir(parents=True,exist_ok=True)
                stream=(directory/'trace.jsonl').open('xb',buffering=64*1024)
                runs[identity]=dict(directory=directory,stream=stream,hash=hashlib.sha256(),counts=Counter(),first=stamp,last=stamp)
                write_json(directory/'trace-manifest.json',dict(schema='workbench.inference_trace.v1',
                    execution_instance_id=identity,status='recording',writer_pid=os.getpid(),
                    note='Missing terminal receipt means this recording is incomplete.'))
            row=runs[identity]
            late=row.get('closed',False)
            if late:row['stream']=(row['directory']/'trace.jsonl').open('ab',buffering=64*1024)
            blob=(json.dumps(dict(event=event,recorded_wall_ns=stamp,data=clean(value)),
                ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n').encode()
            row['stream'].write(blob);row['hash'].update(blob);row['counts'][event]+=1;row['last']=stamp
            if sum(row['counts'].values())%50==0:row['stream'].flush()
            if event=='created':write_json(row['directory']/'configuration.json',value)
            if event=='finish':finish(identity,value,dropped)
            elif late:finish(identity,row['terminal'],dropped)
    except BaseException:
        failed.set()
        raise
    finally:
        for identity,row in runs.items():
            if not row.get('closed'):finish(identity,dict(status='incomplete',reason='writer closed without run terminal'),1)


class InferenceTrace:
    def __init__(self, root, capacity=8192):
        self.root=Path(root)
        if not self.root.is_absolute() or self.root.is_symlink():raise ValueError('absolute owned trace root required')
        ctx=multiprocessing.get_context('spawn')
        self.messages=ctx.Queue(maxsize=capacity);self.ready=ctx.Event();self.failed=ctx.Event()
        self.dropped=Counter();self.closed=False;self.terminals=[]
        self.process=ctx.Process(target=_writer,args=(str(self.root),self.messages,self.ready,self.failed,os.getpid()),daemon=True)
        self.process.start()
        if not self.ready.wait(10):raise RuntimeError('inference trace writer did not become ready')

    def record(self, identity, event, value):
        if not isinstance(identity,str) or not re.fullmatch('[A-Za-z0-9_-]{1,128}',identity):
            raise ValueError('bounded inference trace identity required')
        if self.closed or self.failed.is_set() or not self.process.is_alive():
            self.dropped[identity]+=1;return False
        item=(identity,event,compact(event,value),time.time_ns(),self.dropped[identity])
        try:self.messages.put_nowait(item);return True
        except queue.Full:
            self.dropped[identity]+=1
            if event=='finish':
                # Never make Stop wait for a congested recorder.
                def terminal():
                    try:self.messages.put((*item[:4],self.dropped[identity]),timeout=10)
                    except queue.Full:self.failed.set()
                thread=threading.Thread(target=terminal,daemon=True);self.terminals.append(thread);thread.start()
            return False

    def snapshot(self):
        return dict(writer_pid=self.process.pid,alive=self.process.is_alive(),failed=self.failed.is_set(),
                    dropped_records=sum(self.dropped.values()),root=str(self.root))

    def close(self):
        self.closed=True
        for thread in self.terminals:thread.join(11)
        self.messages.put(None,timeout=10);self.process.join(10)
        if self.process.is_alive():raise RuntimeError('inference trace writer exit unconfirmed')
        self.messages.close();self.messages.join_thread()


class CaptureRun:
    """Business adapter: one CaptureSession for control, model artifacts and video."""
    def __init__(self,identity,directory,configuration,policy,cameras,environment,inputs,still_current,publish,*,business_kind='inference',
                 ros_sources=None,ros_watermarks=None):
        if business_kind not in ('inference','calibration','collection','research'):raise ValueError('known capture business required')
        self.execution_instance_id=identity;self.directory=Path(directory);self.config=deepcopy(configuration)
        self.business_kind=business_kind
        self.policy=deepcopy(policy);self.cameras=cameras;self.environment=environment;self.inputs=inputs
        if (ros_sources is None)!=(ros_watermarks is None):raise ValueError('ROS sources and completion watermarks must be supplied together')
        self.ros_sources,self.ros_watermarks=ros_sources,ros_watermarks
        self.still_current,self.publish=still_current,publish
        self.prepared=threading.Event();self.done=threading.Event()
        reference=weakref.ref(self)
        def alive():
            capture=reference()
            if capture is None:return False
            if not capture.done.is_set():return True
            session=capture.session
            return bool(session is not None and (
                session.process is not None and session.process.poll()is None
                or session.thread is not None and session.thread.is_alive()))
        self.thread=SimpleNamespace(is_alive=alive,join=lambda timeout=None,done=self.done:done.wait(timeout))
        self.lock=threading.RLock();self.session=None;self.error=None;self.manifest=None
        self.controller_result=None;self.root=self.directory/'capture';self.control_sequence=0;self.models={}
        self._finishing=False
        self.sealed=False
        self.guard_decisions=[]

    def is_alive(self):return self.thread.is_alive()
    def join(self,timeout=None):self.done.wait(timeout)

    @property
    def ingress_closed(self):
        session=self.session
        if session is None:return False
        with session.lock:return session.cache.get('ingress_closed')is True

    def _fail(self,error):
        with self.lock:self.error=self.error or str(error)

    def prepare(self,value):
        from .operations import prepare_capture
        try:
            owner=value['execution_owner'];clock=value.get('control_clock_id') or self.config.get('clock',{}).get('source_clock_id') or 'host_monotonic'
            if self.ros_sources is not None:
                sources=self.ros_sources()
            else:
                sources=self._legacy_sources(owner,clock)
            self.session,result=prepare_capture(self.directory/'capture',request_id=self.execution_instance_id,
                run_id=self.execution_instance_id,business_kind=self.business_kind,environment=self.environment,
                policy=self.policy,source_descriptors=sources,still_current=self.still_current,execution_owner=owner,
                execution_context=self.config.get('execution_context'))
            if not result['ready']:raise ValueError(result.get('reason') or 'capture preparation was not ready')
            self.session.start();self.root=self.session.directory
        except Exception as error:self._fail(error)
        finally:self.prepared.set()

    def _legacy_sources(self,owner,clock):
        sources=[dict(stream_id=name,kind=kind,source_instance_id=identity,source_clock_id=clock,
                      units=unit,coordinate_frame=frame,nominal_hz=rate,required=required)
                 for name,kind,identity,unit,frame,rate,required in [
                     ('control','control',owner['runtime_nonce'],'SDK rad; jaw m; original event units','device',100.,True)]]
        if self.inputs is not None:
            sources.append(dict(stream_id='model',kind='model',source_instance_id=self.config['model_id'],
                source_clock_id=clock,units='original model input/output',coordinate_frame='training',nominal_hz=None,required=False))
        try:sources.extend(self.cameras.capture_sources())
        except Exception as error:
            self._fail(error)
            sources.extend(dict(stream_id='video-'+camera,kind='video',source_instance_id=None,
                source_clock_id=None,units='rgb8+depth16le',coordinate_frame=camera,nominal_hz=30.,required=True)
                for camera in ('base_0','left_wrist_0','right_wrist_0'))
        return sources

    def record(self,name,value):
        if name=='prepare':self.prepare(value)
        if name=='finish':self.finish(value['status'],value.get('reason'),value);return
        if self.ros_sources is not None:return  # Payloads arrive directly via ROS; lifecycle IPC carries no data.
        sequence=value['capture_sequence']
        with self.lock:self.control_sequence=max(self.control_sequence,sequence)
        if self.session is None:self._fail('capture session unavailable');return
        result=self.session.try_publish('control',sequence,value['capture_source_time'],dict(event=name,data=clean(value)))
        if not result['accepted']:self._fail('control evidence gap: '+result['reason'])
        state=self.session.snapshot()
        if state.get('diagnostics_error'):self._fail(state['diagnostics_error'])
        if state.get('source_errors'):self._fail(next(iter(state['source_errors'].values())))

    def model_begin(self,request_id,source_time):
        with self.lock:
            if request_id not in self.models:self.models[request_id]=dict(sequence=len(self.models)+1,source_time=source_time,future=None)

    def model_end(self,request_id,future):
        with self.lock:
            if request_id in self.models:self.models[request_id]['future']=future

    def snapshot(self):
        with self.lock:
            state=self.session.snapshot() if self.session else {}
            outcome='incomplete'if self.error else (self.manifest or{}).get('recording_outcome','incomplete'if self.sealed else None)
            return dict(state,ready=bool(state.get('ready')and not self.error),done=self.sealed or self.done.is_set(),error=self.error or state.get('error') or state.get('diagnostics_error'),
                        directory=str(self.root),manifest=self.manifest,controller_result=self.controller_result,
                        guard_decisions=deepcopy(self.guard_decisions),recording_outcome=outcome)

    def finish(self,status,reason=None,value=None):
        with self.lock:
            if self._finishing:return
            self._finishing=True;self.controller_result=dict(status=status,reason=reason)
        threading.Thread(target=self._seal,args=(deepcopy(value or {}),),daemon=True,name='capture-run-seal').start()

    def _seal(self,value):
        try:
            if self.ros_sources is not None:
                self._seal_ros(value);return
            self.prepared.wait(10)
            deadline=time.monotonic()+3
            with self.lock:models=deepcopy({k:{n:v for n,v in row.items() if n!='future'} for k,row in self.models.items()})
            for request_id,row in models.items():
                while True:
                    with self.lock:future=self.models[request_id]['future']
                    if future is not None or time.monotonic()>=deadline:break
                    time.sleep(.02)
                if future is None or future is False:self._fail('model artifact completion missing: '+request_id);continue
                try:future.result(timeout=30)
                except Exception as error:self._fail(error);continue
                directory=self.inputs.root/request_id
                required=['input.npz','request.json']
                if not all((directory/name).is_file() for name in required) or not any((directory/name).is_file() for name in ('output.npz','prediction-error.json')):
                    self._fail('model input/output artifact missing: '+request_id);continue
                files={}
                for path in directory.iterdir():
                    if path.is_file() and path.suffix in ('.npz','.json'):
                        digest=hashlib.sha256()
                        with path.open('rb') as stream:
                            for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
                        files[path.name]=dict(payload_ref=dict(path=str(path.absolute()),size_bytes=path.stat().st_size,sha256=digest.hexdigest()))
                if self.session is not None:
                    result=self.session.try_publish('model',row['sequence'],row['source_time'],dict(request_id=request_id,files=files))
                    if not result['accepted']:self._fail('model evidence gap: '+result['reason'])
            if self.session is not None:
                finals={key:0 for key in self.session.last_sequence}
                final_control=value.get('final_capture_sequence')
                if final_control is None:
                    self._fail('control final sequence unavailable');final_control=self.control_sequence+1
                finals['control']=max(self.control_sequence,final_control)
                if 'model'in finals:finals['model']=len(models)
                endpoint=value.get('finished_wall_ns',time.time_ns())/1e9 if self.environment=='physical' else value.get('source_time',0.)
                endpoints={key:dict(source_clock_id=source['source_clock_id'],source_time=endpoint)
                           for key,source in self.session.camera_sources.items()}
                self.session.finish(finals,self.controller_result,**({'source_endpoints':endpoints} if endpoints else {}))
                if not self.session.done.wait(90):raise TimeoutError('capture session seal not confirmed')
                if self.session.process is not None and self.session.process.poll()not in(None,0):
                    if self.session.thread is not None:self.session.thread.join(2)
                    self.session.recover()  # Data-only prefix sealing after the controller terminal, never motion recovery.
                    if not self.session.done.wait(90):raise TimeoutError('capture session recovery seal not confirmed')
                path=self.session.directory/'manifest.json'
                if path.is_file():self.manifest=json.loads(path.read_text())
                state=self.session.snapshot()
                if state.get('error'):self._fail(state['error'])
        except Exception as error:
            self._fail(error)
            if self.ros_sources is not None and self.session is not None:
                # Data-only prefix sealing after a failed boundary; never leave
                # an orphan writer or present an unproven boundary as complete.
                try:
                    if self.session.terminal is None:
                        self.session.finish(dict(self.session.last_sequence),
                            dict(self.controller_result or {},recording_error=self.error),source_endpoints={
                                key:dict(source_clock_id=row['source_clock_id'],source_time=0.)
                                for key,row in self.session.camera_sources.items()})
                    self.session.done.wait()
                    path=self.session.directory/'manifest.json'
                    if path.is_file():self.manifest=json.loads(path.read_text())
                except Exception as sealing_error:self.error+='; '+str(sealing_error)
        finally:
            try:self._seal_decisions(value)
            except Exception as error:self._fail(error)
            self.sealed=True
            try:self.publish(self)
            except Exception as error:self._fail(error)
            self.publish=lambda capture:None;self.still_current=lambda:False
            self.done.set()

    def _seal_ros(self,value):
        if not self.prepared.wait(10)or self.session is None:raise ValueError('ROS capture preparation did not complete')
        if not isinstance(value.get('ros_final_seq'),dict):raise ValueError('ROS terminal source watermarks are missing')
        deadline=time.monotonic()+5.
        while True:
            try:finals=dict(value['ros_final_seq'],**self.ros_watermarks());break
            except ValueError:
                if time.monotonic()>=deadline:raise
                time.sleep(.01)
        finals={key:finals[key]for key in self.session.last_sequence}
        endpoints={key:dict(source_clock_id=source['source_clock_id'],source_time=
            value['finished_wall_ns']/1e9 if source['source_clock_id']=='host_wall'else value['source_time'])
            for key,source in self.session.camera_sources.items()}
        self.session.finish(finals,self.controller_result,source_endpoints=endpoints)
        # Source closure is bounded by the writer. Verification/export runs in
        # this background thread and may outlive an observer's timeout; only
        # the resident client's terminal result ends this Run's seal tracking.
        self.session.done.wait()
        state=self.session.snapshot()
        if state.get('error'):self._fail(state['error'])
        path=self.session.directory/'manifest.json'
        if not path.is_file():raise ValueError('ROS capture manifest is missing')
        self.manifest=json.loads(path.read_text())
        self.models={key:{}for key in self.manifest.get('model_inputs',{})}

    def _seal_decisions(self,value):
        from .execution_settings import Guard
        guard=Guard.bind(self.policy,{})
        manifest=self.manifest or {};streams=manifest.get('streams',{})
        counts=dict(G33=len(manifest.get('issues',[]))+int(bool(self.error))+int(not manifest),
            G36=sum(row.get('coverage_failed_conditions',row.get('missing',0)+int(bool(row.get('tail_error'))))for row in streams.values())+int(not streams),
            G37=int(manifest.get('control_terminal')is None)+sum(row.get('final_sequence')is None for row in streams.values()))
        checked=time.monotonic();source=value.get('source_time',checked)
        revision=manifest.get('seal_revision');ref=f'manifest-r{revision:06d}.json'if type(revision)is int else'manifest.json'
        self.guard_decisions=[guard.evaluate(dict(rule_id=rule,signal_id=self.execution_instance_id,
            run_id=self.execution_instance_id,observed=dict(failed_conditions=count,receipt_ref=ref),
            source_time=source,checked_time=checked),'seal')for rule,count in counts.items()]

    def close(self):
        if not self._finishing:self.finish('unknown','business closed before the control terminal receipt')
        if not self.done.wait(35) and self.session is not None:self.session.close()


class RunArtifacts:
    """Business-side request receipts and model-file links, including failed Runs."""
    def __init__(self, root, input_evidence, publish):
        self.root=Path(root);self.inputs=input_evidence;self.publish=publish
        self.writer=ThreadPoolExecutor(max_workers=1,thread_name_prefix='infer-artifact-seal')
        self.pending=0;self.error=None;self.lock=threading.Lock()
        self._submit(self._recover_requests)

    def _recover_requests(self):
        for path in self.root.glob('request-*/debug-receipt.json'):
            value=json.loads(path.read_text())
            if value.get('status')!='running':continue
            value.update(status='interrupted',artifact_status='incomplete',
                error='Business process restarted without a terminal receipt; controller outcome is unknown.',
                directory=str(path.parent),trace={'status':'incomplete'},missing_files=['terminal receipt'])
            write_json(path,value);self.publish(value,path)

    def _submit(self, function, *args):
        with self.lock:self.pending+=1
        future=self.writer.submit(function,*args)
        def done(result):
            with self.lock:
                self.pending-=1
                if result.exception():self.error=str(result.exception())
        future.add_done_callback(done)
        return future

    def begin(self, job, body):
        value=dict(job_id=job['id'],request=clean(deepcopy(body)),started_wall_ns=job['started_wall_ns'],
                   execution_instance_id=None,model_requests=[],status='running')
        # This preparation gate runs before any motion, never on Owner.tick or
        # the model callback. Failed storage cannot silently erase a Run.
        self._submit(self._request_receipt,deepcopy(value)).result(timeout=3)
        return value

    def _request_receipt(self, value):
        write_json(self.root/('request-'+value['job_id'])/'debug-receipt.json',value)

    def bind(self, handle, identity, configuration):
        if handle is None:return
        handle['execution_instance_id']=identity
        self._submit(write_json,self.root/('inference-'+identity)/'run-request.json',
                     dict(application_job_id=handle['job_id'],configuration=clean(deepcopy(configuration))))

    def model_request(self, handle, identity):
        if handle is not None:handle['model_requests'].append(identity)

    def finish(self, handle, job, runtime):
        if handle is None:return
        receipt=deepcopy(handle);receipt.update(status=job['status'],finished_wall_ns=job.get('finished_wall_ns'),
            error=job.get('error'),result=clean(job.get('result')),final_runtime=clean(deepcopy(runtime)))
        self._submit(self._finish,receipt)

    def _finish(self, receipt):
        # Order after raw input/output writes, without blocking model/Owner/Stop.
        deadline=time.monotonic()+3
        while time.monotonic()<deadline and any(not ((self.inputs.root/k/'output.npz').exists()
                or (self.inputs.root/k/'prediction-error.json').exists()) for k in receipt['model_requests']):
            time.sleep(.02)
        self.inputs.writer.submit(lambda:None).result(timeout=30)
        identity=receipt['execution_instance_id']
        directory=self.root/('inference-'+identity if identity else 'request-'+receipt['job_id'])
        directory.mkdir(parents=True,exist_ok=True)
        missing=[]
        for request_id in receipt['model_requests']:
            source=self.inputs.root/request_id;target=directory/'model'/request_id
            target.mkdir(parents=True,exist_ok=True)
            for name in ('input.npz','request.json','output.npz','audit.json','chunk.json','pipeline-timing.json',
                         'input-write.json','output-write.json','prediction-error.json'):
                src=source/name;dst=target/name
                if src.is_file():
                    if not dst.exists():os.link(src,dst)
                elif name in ('input.npz','request.json'):missing.append(request_id+'/'+name)
            if not (source/'output.npz').exists() and not (source/'prediction-error.json').exists():
                missing.append(request_id+'/output-or-error')
        trace_path=directory/'trace-manifest.json'
        if identity:
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                if trace_path.exists():
                    trace=json.loads(trace_path.read_text())
                    if trace.get('status')!='recording':break
                time.sleep(.02)
            else:trace=dict(status='incomplete',reason='terminal trace not confirmed')
        else:trace=dict(status='not_started',reason='Run ended before an Owner execution was created')
        receipt.update(trace=trace,missing_files=missing,input_evidence=self.inputs.snapshot(),
            artifact_status='complete' if not missing and trace['status'] in ('complete','not_started')
                and not self.inputs.snapshot()['error'] else 'incomplete',
            directory=str(directory),schema='workbench.inference_debug.v1',
            original_images='Only the RGB frames actually supplied to the model; not a continuous camera video.')
        path=directory/'debug-receipt.json';write_json(path,receipt)
        self._request_receipt(receipt)
        self.publish(receipt,path)

    def snapshot(self):
        with self.lock:return dict(pending=self.pending,error=self.error)

    def close(self):self.writer.shutdown(wait=True)
