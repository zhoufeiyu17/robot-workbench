"""Resident-only C install preparation and main-thread D task observation."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import secrets
import subprocess
import threading
import time
import weakref


def publish_task_status(owner,value):
    with owner.lock:owner.native_task_status=dict(value)


def write_task_receipt(root,owner,receipt):
    """Existing resident writer calls this; no main-thread file hashing."""
    identity=receipt['execution_timeline_digest'];instance=receipt['execution_instance_id']
    if (not re.fullmatch('[0-9a-f]{64}',identity) or
            not re.fullmatch('[0-9a-f]{32}|[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}',instance)):
        raise ValueError('task receipt requires a content digest and unique accepted execution instance')
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    path=root/(instance+'.json');raw=(json.dumps(receipt,allow_nan=False,indent=2)+'\n').encode()
    with path.open('xb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
    digest=hashlib.sha256(raw).hexdigest()
    with owner.lock:
        if (getattr(owner,'native_task_status',{}).get('execution_timeline_digest')==identity
                and owner.native_task_status.get('execution_instance_id')==instance
                and owner.native_task_status.get('runtime_nonce')==receipt['runtime_nonce']
                and owner.native_task_status.get('observer_id')==receipt.get('observer_id')):
            owner.native_task_status.update(task_outcome=receipt['task_outcome'],task_qualified=receipt['task_qualified'],
                receipt_path=str(path),receipt_sha256=digest,message='原生任务观测已封存')
    return dict(execution_timeline_digest=identity,execution_instance_id=instance,receipt_path=str(path),receipt_sha256=digest)


class NativeTaskObserver:
    """Compilation runs in existing background paths; native reads stay on main."""
    def __init__(self,native,backend,fixture,submit_receipt):
        self.native,self.backend,self.fixture=native,backend,deepcopy(fixture)
        self.submit_receipt=submit_receipt
        self.prepared={};self.lock=threading.Lock();self.active=None;self.seen=weakref.WeakSet()

    def attach(self,owner):
        original_validate=owner.source_validator;original_start=owner.start_motion
        def validate(plan,source):
            result=original_validate(plan,source)
            from integration import _digest
            # Actual validation still authorizes motion. Task preparation cannot
            # introduce a new coffee acceptance gate into either business.
            try:
                from robot_workbench.physx_task import prepare_task
                item={'prepared':prepare_task(plan,source,self.fixture),'error':None}
            except Exception as error:item={'prepared':None,'error':str(error)}
            item['source_digest']=_digest(source)
            with self.lock:
                self.prepared[plan['timeline_digest']]=item
                while len(self.prepared)>4:self.prepared.pop(next(iter(self.prepared)))
            return result
        def start(*args,**kwargs):
            result=original_start(*args,**kwargs);runner=owner.runner
            if runner is None or not hasattr(runner,'frozen'):return result
            with self.lock:item=self.prepared.get(runner.frozen.plan['timeline_digest'])
            original_compile=runner.compile_runtime
            def compile_runtime(q):
                timeline=original_compile(q)
                try:
                    if item is None or item['error']:raise ValueError((item or {}).get('error') or 'task plan was not prepared at review')
                    from robot_workbench.physx_task import prepare_task_execution
                    runner.native_task_prepared=dict(prepared=item['prepared'],
                        execution=prepare_task_execution(item['prepared'],timeline),error=None,
                        source_digest=item['source_digest'])
                except Exception as error:runner.native_task_prepared=dict(error=str(error),
                    source_digest=(item or {}).get('source_digest'))
                return timeline
            runner.compile_runtime=compile_runtime
            return result
        owner.source_validator=validate;owner.start_motion=start

    def before_advance(self,owner):
        """Called before physics. A late observer invalidates evidence, not motion."""
        runner=getattr(owner,'runner',None)
        timeline=getattr(runner,'runtime',None)
        if not timeline or not hasattr(runner,'frozen'):return
        identity=timeline['timeline_digest']
        instance=getattr(owner,'execution_instance_id',None)
        if runner in self.seen:return
        pending=self.backend.pending_contexts
        applied=self.backend.applied_contexts
        def belongs(row):
            return (isinstance(row,dict) and row.get('execution_id')==identity
                    and row.get('execution_instance_id')==instance)
        if not any(belongs(row) for row in [*pending.values(),*applied.values()]):return
        self.seen.add(runner)
        status=dict(execution_timeline_digest=identity,source_clock_id=self.backend.source_clock_id,
            execution_instance_id=instance,
            runtime_nonce=owner.runtime_nonce,physics_step=self.backend.physics_step,task_outcome='running',
            task_qualified=False,receipt_path=None,receipt_sha256=None,message='读取同一 plant 的原生接触与任务状态')
        status['observer_id']=secrets.token_hex(16)
        monitor=None;error=None
        try:
            if any(belongs(row) for row in applied.values()):
                raise ValueError('task observer was not ready before the first physics application')
            item=getattr(runner,'native_task_prepared',None)
            if not item or item.get('error'):raise ValueError((item or {}).get('error') or 'background task preparation unavailable')
            from robot_workbench.physx_task import CoffeeTaskMonitor
            monitor=CoffeeTaskMonitor(self.native,self.backend,item['prepared'],timeline,
                command_epoch=owner.epoch,runtime_nonce=owner.runtime_nonce,prepared_execution=item['execution'])
        except Exception as exception:
            error=str(exception);status.update(task_outcome='unverifiable',message=error)
        self.active=dict(owner=owner,runner=runner,monitor=monitor,status=status,error=error)
        publish_task_status(owner,status)

    def after_advance(self,owner,*,physics_advanced):
        item=self.active
        if item is None:return
        monitor=item['monitor'];runner=item['runner'];same=owner is item['owner'] and owner.runner is runner
        if physics_advanced and monitor is not None:
            monitor.observe_current()
            item['status'].update(task_outcome=monitor.outcome,physics_step=self.backend.physics_step,
                message=(monitor.first_failure or {}).get('reason') or '原生任务观测进行中')
            publish_task_status(item['owner'],item['status'])
        state=runner.status
        completed=state=='READY' and runner.frozen.status=='completed' and runner.frozen.final_feedback_qualified
        if not (completed or state in ('FAULT','STOPPED') or not same):return
        outcome='completed' if completed else 'failed' if state=='FAULT' else 'stopped'
        feedback=deepcopy(getattr(item['owner'],'current_feedback',{}))
        if monitor is not None:receipt=monitor.finish(outcome,feedback)
        else:receipt=dict(item['status'],schema_version='owner_physx_native_task.v1',motion_outcome=outcome,
            environment='fake',simulation_only=True,task_outcome='unverifiable',task_qualified=False,
            source_digest=(getattr(runner,'native_task_prepared',{}) or {}).get('source_digest'),
            parent_timeline_digest=runner.runtime['parent_timeline_digest'],reason=item['error'],hardware_qualified=False)
        receipt['runtime_nonce']=item['owner'].runtime_nonce
        receipt['execution_instance_id']=item['status']['execution_instance_id']
        receipt['observer_id']=item['status']['observer_id']
        status=dict(item['status'],task_outcome='pending_receipt',task_qualified=False,
                    receipt_path=None,receipt_sha256=None,message='运动已结束，等待原生任务观测落盘')
        publish_task_status(item['owner'],status)
        try:self.submit_receipt(item['owner'],receipt)
        except Exception as error:
            publish_task_status(item['owner'],dict(status,task_outcome='unverifiable',message='原生任务证据未入队：'+str(error)))
        self.active=None


def load_collection_contract(path,output,gpu,duration):
    """Validate the explicit enablement document before importing Isaac."""
    from local_execution import TARGET
    value=json.loads(Path(path).read_bytes());resources=value['resources'];details=value['collection_setup']
    if (value.get('schema_version')!='anyverse.dsw-codex-experiment.v1'
            or not value.get('identity',{}).get('session_id') or gpu!=1
            or resources.get('gpu_uuid')!=TARGET['gpu_uuid'] or resources.get('cuda_visible_devices')!=[1]
            or resources.get('max_workers')!=4 or resources.get('max_cpu_threads',0)<4
            or type(resources.get('service_lifetime_seconds')) is not int
            or resources['service_lifetime_seconds']<=0
            or not duration+45<=resources.get('max_wall_time_minutes',0)*60):
        raise ValueError('explicit same-Session GPU1 four-process resident contract and cleanup reserve required')
    if not any(Path(output).resolve().is_relative_to(Path(root).resolve()) for root in value['filesystem']['writable_roots']):
        raise ValueError('resident output is outside the collection contract')
    if set(details)!={'profile_template','detection_runtime_root','plant_max_memory_mib','min_free_memory_mib'}:
        raise ValueError('explicit collection template/runtime and bounded memory fields required')
    for key in ('profile_template','detection_runtime_root'):
        p=Path(details[key])
        if not p.is_absolute() or not p.exists():raise ValueError('collection input missing: '+key)
    for key in ('plant_max_memory_mib','min_free_memory_mib'):
        if type(details[key]) is not int or details[key]<=0:raise ValueError('positive collection memory budget required')
    return value


class CollectionPreparation:
    """One installation's owned conversion/services; notification never blocks C."""
    def __init__(self,*,contract,output,base,device,application,ports,directories,control_url,token_file,deadline_wall_ns):
        self.contract,self.output,self.base=deepcopy(contract),Path(output),base
        self.device,self.application=device,application
        self.ports,self.directories,self.control_url,self.token_file=ports,directories,control_url,token_file
        self.deadline_wall_ns=deadline_wall_ns
        self.closed=threading.Event();self.notifications=queue.Queue(maxsize=1)
        self.identity=None;self.parts=None;self.process=None;self.error=None
        self.thread=threading.Thread(target=self._run,daemon=True,name='resident-collection-setup');self.thread.start()

    def notify(self,snapshot):
        if self.closed.is_set() or not snapshot or snapshot.get('verified') is not True:return
        value=deepcopy(snapshot)
        try:self.notifications.put_nowait(value)
        except queue.Full:
            try:self.notifications.get_nowait()
            except queue.Empty:pass
            self.notifications.put_nowait(value)

    def _status(self,status,**fields):
        from probe_owner_physx import save
        save(self.output/'collection-setup-status.json',dict(status=status,wall_ns=time.time_ns(),**fields))

    def _close_parts(self):
        if self.parts:
            parts=self.parts;self.parts=None;errors=[]
            parts.playground.cancel_event.set()
            parts.workflow.closed.set()
            job=parts.playground.thread
            if job is not None:job.join(timeout=30)
            for manager in (parts.detector,parts.planner):
                try:manager.close()
                except Exception as error:errors.append(str(error))
            if job is not None and job.is_alive():errors.append('collection job did not exit within cleanup budget')
            if errors:raise RuntimeError('; '.join(errors))

    def _prepare(self,snapshot):
        from local_execution import TARGET,gpu_check
        from robot_workbench.collection_setup import prepare_collection_setup,finalize_collection_setup,construct_collection
        from probe_owner_physx import save
        root=self.output;active=root/'calibrations/hardware/active'
        expected=dict(head_id=snapshot['head_id'],wrist_id=snapshot['wrist_id'],
            manifest_sha256=hashlib.sha256((active/'manifest.json').read_bytes()).hexdigest())
        if expected==self.identity:return
        if self.parts:
            state=self.device.snapshot(self.device.runtime.holder)
            if state.get('automatic_enabled') or (state.get('recording') or {}).get('done') is False:
                raise ValueError('new installation preparation waits for the current frozen D run to finish')
            self._close_parts()
        details=self.contract['collection_setup'];pid=os.getpid();proc=Path('/proc')/str(pid)
        script=str(Path(__file__).resolve().parents[1]/'scripts/serve_physx_device.py')
        execution=dict(TARGET,session_id=self.contract['identity']['session_id'],max_processes=4,
            min_free_memory_mib=details['min_free_memory_mib'],plant=dict(session_id=self.contract['identity']['session_id'],pid=pid,
                process_start_ticks=(proc/'stat').read_text().rsplit(')',1)[1].split()[19],scope=str(root),
                script=script,script_sha256=hashlib.sha256(Path(script).read_bytes()).hexdigest(),
                argv=[row.decode() for row in (proc/'cmdline').read_bytes().split(b'\0') if row],
                max_memory_mib=details['plant_max_memory_mib']))
        output=root/('collection-'+expected['manifest_sha256'][:16]+'-'+str(time.time_ns()))
        self._status('preparing_profile',installed=expected,output=str(output))
        detector_entry=Path(__file__).resolve().parents[1]/'detector_adapter'/'isaac_rgbd_detection.py'
        if not detector_entry.is_file():raise FileNotFoundError('frozen detector adapter missing: '+str(detector_entry))
        pipeline_config={**self.base['pipeline_config'],'detection_script':str(detector_entry)}
        setup=prepare_collection_setup(run_root=root,calibration_config=root/'calibration-config.json',
            capture_root=root/'calibration-workbench',expected_install=expected,registration=root/'registration.json',
            profile_template=details['profile_template'],base_pipeline_config=pipeline_config,execution=execution,
            contract_template=self.contract,detection_runtime_root=details['detection_runtime_root'],
            control_url=self.control_url,token_file=self.token_file,producer_ports=self.ports,producer_directories=self.directories,
            trusted_binding_path=root/'collection-binding.json',local_recording_root=root/'collection-raw',
            resident_deadline_wall_ns=self.deadline_wall_ns,output=output)
        budget=min(600.,(self.deadline_wall_ns-time.time_ns())/1e9-60)
        if budget<=0:raise TimeoutError('resident deadline leaves no conversion/cleanup budget')
        if self.closed.is_set():raise InterruptedError('resident preparation cancelled before conversion')
        gpu_check({'execution':execution})
        conversion=setup['conversion'];wb=Path(script).parents[1]/'rule_based_workbench'
        command=['python3','-B',str(wb/'owned_process.py'),'--timeout',str(budget),'--',*conversion['argv']]
        self._status('converting_planning_assets',output=str(output),deadline_wall_ns=self.deadline_wall_ns)
        with (output/'conversion.log').open('xb') as log:
            self.process=subprocess.Popen(command,env={**os.environ,**conversion['env']},stdout=log,stderr=subprocess.STDOUT)
            while self.process.poll() is None:
                if self.closed.wait(.2):
                    self.process.terminate();self.process.wait(timeout=30);raise InterruptedError('resident conversion cancelled')
            if self.process.returncode:raise RuntimeError('owned asset conversion failed; inspect '+str(output/'conversion.log'))
        self.process=None
        if self.closed.is_set():raise InterruptedError('resident preparation cancelled')
        configured=finalize_collection_setup(setup['setup_path'])
        if self.closed.is_set():raise InterruptedError('resident preparation cancelled before worker startup')
        save(root/'collection-binding.json',json.loads(Path(configured['initial_trusted_binding_source']).read_bytes()))
        self.parts=construct_collection(output/'configured.json',self.device,self.application)
        self.device.workflows['collection']=self.parts.workflow
        self.parts.detector.start(self.parts.pipeline_config,self.parts.worker_state_directory)
        # The detector owns its GPU PID only after its private status is
        # published. Do not race the planner admission against that startup.
        while True:
            state=self.parts.detector.status()
            if state.get('status')=='ready':break
            if state.get('status') in ('failed','stopped'):
                raise RuntimeError('private detector did not become ready: '+str(state.get('error') or state))
            if self.closed.wait(.2) or time.time_ns()>=self.deadline_wall_ns-45_000_000_000:
                raise InterruptedError('resident detector startup cancelled or deadline reached')
        self.parts.planner.start(self.parts.pipeline_config,self.parts.worker_state_directory)
        self.identity=expected
        self._status('workers_starting',installed=expected,configured=str(output/'configured.json'),
                     movement_started=False,requires_explicit_handoff=True)

    def _run(self):
        while not self.closed.is_set():
            try:snapshot=self.notifications.get(timeout=.1)
            except queue.Empty:continue
            try:self._prepare(snapshot)
            except Exception as error:
                self.error=str(error);self._status('failed',error=str(error),movement_started=False)
            finally:self.notifications.task_done()

    def close(self):
        self.closed.set()
        self.thread.join(timeout=35)
        if self.thread.is_alive():raise TimeoutError('owned collection preparation has not finished cleanup')
        self._close_parts()
