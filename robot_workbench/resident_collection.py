"""Resident-only C install preparation and main-thread D task observation."""
from copy import deepcopy
import hashlib
import json
import math
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
                receipt_path=str(path),receipt_sha256=digest,message='原生任务观测已封存',
                endpoint_observation=deepcopy(receipt.get('endpoint_observation')))
    return dict(execution_timeline_digest=identity,execution_instance_id=instance,receipt_path=str(path),receipt_sha256=digest)


class NativeTaskObserver:
    """Compilation runs in existing background paths; native reads stay on main."""
    def __init__(self,native,backend,fixture,submit_receipt):
        self.native,self.backend,self.fixture=native,backend,deepcopy(fixture)
        self.submit_receipt=submit_receipt
        self.prepared={};self.lock=threading.Lock();self.active=None;self.seen=weakref.WeakSet()

    def attach(self,owner):
        original_validate=owner.source_validator;original_start=owner.start_motion
        if getattr(owner,'isolated_plans',False):
            owner.native_task_fixture=deepcopy(self.fixture)
            def start_isolated(*args,**kwargs):
                result=original_start(*args,**kwargs);runner=owner.runner
                if runner is None or not hasattr(runner,'frozen'):return result
                original_compile=runner.compile_runtime
                def compile_isolated(q):
                    timeline=original_compile(q)
                    item=timeline.pop('native_task',dict(error='isolated task preparation unavailable'))
                    if not item.get('error'):
                        from robot_workbench.physx_task import PlantTargets
                        item['execution']=dict(source_timeline=timeline,timeline=timeline,
                            targets=PlantTargets(timeline['commands'],item['prepared']['fixture']))
                    runner.native_task_prepared=item
                    return timeline
                runner.compile_runtime=compile_isolated
                return result
            owner.start_motion=start_isolated
            return
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
            item['recovery_parent_binding']=json.loads(Path(owner.hardware_binding).read_bytes()) if owner.hardware_binding else None
            with self.lock:
                self.prepared[plan['timeline_digest']]=item
                while len(self.prepared)>4:self.prepared.pop(next(iter(self.prepared)))
            return result
        def start(*args,**kwargs):
            result=original_start(*args,**kwargs);runner=owner.runner
            if runner is None or not hasattr(runner,'frozen'):return result
            with self.lock:item=self.prepared.get(runner.frozen.plan['timeline_digest'])
            runner.recovery_parent_binding=deepcopy((item or {}).get('recovery_parent_binding'))
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
                message=(monitor.first_failure or {}).get('reason') or '原生任务观测进行中',
                first_failure={key:monitor.first_failure.get(key) for key in ('reason','detail','physics_step',
                    'absolute_physics_step','applied_context','contact_speed_m_s','contact_speed_limit_m_s')}
                    if monitor.first_failure else None)
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
    from local_execution import contract_compute_resource,validate_shared_model
    value=json.loads(Path(path).read_bytes());resources=value['resources'];details=value['collection_setup']
    target=contract_compute_resource(value)
    if 'shared_model' in value:validate_shared_model(value['shared_model'])
    from launch_environment import persistent_service
    persistent=persistent_service(resources)
    budget=resources.get('max_wall_time_minutes')
    # duration=0 is the existing setup-only path; it starts no resident lifetime.
    lifetime_ok=((duration is None or type(duration) in (int,float) and duration==0)
        and type(budget) in (int,float) and math.isfinite(budget) and 0<budget<=120) if persistent else (
        type(resources.get('service_lifetime_seconds')) is int and resources['service_lifetime_seconds']>0
        and type(duration) in (int,float) and math.isfinite(duration) and duration>=0
        and type(budget) in (int,float) and math.isfinite(budget) and duration+45<=budget*60)
    from robot_workbench.service import loopback_browser_hosts
    loopback_browser_hosts(value.get('browser_ports',[]))
    if (value.get('schema_version')!='anyverse.dsw-codex-experiment.v1'
            or not value.get('identity',{}).get('session_id') or type(gpu) is not int or gpu!=target['gpu_index']
            or resources.get('max_workers') not in (4,5,6) or resources.get('max_cpu_threads',0)<4
            or not lifetime_ok):
        raise ValueError('explicit same-Session approved GPU four/five/six-process resident contract and cleanup reserve required')
    if resources['max_workers'] in (5,6):
        preview=value.get('preview_setup')
        keys={'port','max_memory_mib','auxiliary_max_memory_mib'}|({'scene_max_memory_mib'} if resources['max_workers']==6 else set())
        if (not isinstance(preview,dict) or set(preview)!=keys
                or type(preview['port']) is not int or not 0<=preview['port']<=65535
                or any(type(preview[key]) is not int or preview[key]<=0 for key in keys-{'port'})):
            raise ValueError('five roles require explicit preview port and preview/auxiliary memory budgets')
    elif value.get('preview_setup') is not None:raise ValueError('preview cannot enter the four-role contract')
    if not any(Path(output).resolve().is_relative_to(Path(root).resolve()) for root in value['filesystem']['writable_roots']):
        raise ValueError('resident output is outside the collection contract')
    if 'camera_spool_root' in value:
        spool=Path(value['camera_spool_root'])
        if (not spool.is_absolute() or spool.exists() or spool.is_symlink() or not spool.parent.is_dir()
                or Path(output).resolve().is_relative_to(spool.resolve())
                or spool.resolve() not in {Path(root).resolve() for root in value['filesystem']['writable_roots']}):
            raise ValueError('camera spool requires a new exclusive directory explicitly declared writable')
    if set(details)!={'profile_template','detection_runtime_root','plant_max_memory_mib','min_free_memory_mib'}:
        raise ValueError('explicit collection template/runtime and bounded memory fields required')
    for key in ('profile_template','detection_runtime_root'):
        p=Path(details[key])
        if not p.is_absolute() or not p.exists():raise ValueError('collection input missing: '+key)
    for key in ('plant_max_memory_mib','min_free_memory_mib'):
        if type(details[key]) is not int or details[key]<=0:raise ValueError('positive collection memory budget required')
    return value


def require_collection_control(device):
    runtime=device.runtime
    if runtime is None:raise ValueError('数采准备需要先通过运维接入控制；不会自动连接设备')
    if runtime.holder!='collection':raise ValueError('数采准备需要由共用执行准入交接给 collection')
    state=device.snapshot('collection');age=state.get('sample_age_s')
    if (state.get('connected') is not True or state.get('feedback_healthy') is not True
            or state.get('stationary') is not True or state.get('automatic_enabled') is not False
            or type(age) not in (int,float) or not 0<=age<=.1):
        raise ValueError('数采准备需要当前新鲜、健康、静止的控制反馈')


class CollectionPreparation:
    """One installation's owned conversion/services; notification never blocks C."""
    def __init__(self,*,contract,output,base,device,application,ports,directories,control_url,token_file,deadline_wall_ns,installed_snapshot=None,diagnostic_source=None):
        from run_pipeline import isolate_control_config_validation
        isolate_control_config_validation()
        self.contract,self.output,self.base=deepcopy(contract),Path(output),base
        self.device,self.application=device,application
        self.ports,self.directories,self.control_url,self.token_file=ports,directories,control_url,token_file
        self.deadline_wall_ns=deadline_wall_ns
        from launch_environment import persistent_service
        self.persistent_service=persistent_service(contract.get('resources',{}))
        if (deadline_wall_ns is None)!=self.persistent_service:
            raise ValueError('resident deadline must match the explicit persistent service contract')
        self.installed_snapshot=installed_snapshot
        self.diagnostic_source=deepcopy(diagnostic_source);self.diagnostic_invalidated=False
        self.lock=threading.Lock();self.busy=False;self.running=False;self.last_install=None
        self.started=None;self.stage_started=None;self.stage_key=None
        self.value=dict(status='awaiting_calibration',error=None,attempt=0,installed=None,
            message='等待有效标定安装；安装后请在服务运维准备数采资产与常驻服务。')
        if self.diagnostic_source:
            from robot_workbench.calibration_profile import diagnostic_metadata
            self.value.update(diagnostic_origin=diagnostic_metadata(self.diagnostic_source),fresh_C_completed=False)
        self.closed=threading.Event();self.notifications=queue.Queue(maxsize=1)
        self.identity=None;self.parts=None;self.process=None;self.error=None
        self.request_generation=0;self.request_app_generation=None
        self.collection_prepare_operation=None
        self._operation_current=self._lifecycle_admission=None
        self.thread=threading.Thread(target=self._run,daemon=True,name='resident-collection-setup');self.thread.start()

    def notify(self,snapshot,*,prepare=False):
        if prepare:self._require_operation()
        if self.closed.is_set():return
        if not snapshot or snapshot.get('verified') is not True:
            self.application.invalidate_collection_preparation(self.device)
            with self.lock:
                self.request_generation+=1;self.last_install=deepcopy(snapshot)
                self.value.update(status='awaiting_calibration',installed=deepcopy(snapshot),error=None)
            return
        if self.diagnostic_source:
            from robot_workbench.calibration_profile import diagnostic_snapshot
            try:matches=snapshot==diagnostic_snapshot(self.diagnostic_source)
            except (ValueError,OSError):matches=False
            if not matches:
                self.diagnostic_invalidated=True;self.closed.set()
                with self.lock:self.busy=self.running
                self.application.invalidate_collection_preparation(self.device)
                self.device.diagnostic_mesh_candidates=[]
                self.device.diagnostic_calibration.update(invalidated=True)
                self._status('failed',error='复用标定诊断已因新 C 安装失效；请启动独立新运行',movement_started=False)
                return
        value=deepcopy(snapshot)
        with self.lock:
            if value!=self.last_install or prepare:self.request_generation+=1
            if prepare:self.request_app_generation=self.application.generations[self.device.environment]
            self.last_install=value
            if self.running and self.value.get('installed')==value:return
            if not prepare:
                if self.value.get('installed')!=value or self.value['status']=='awaiting_calibration':
                    self.value.update(status='awaiting_prepare',installed=value,error=None,worker=None,
                        message='标定已安装；请在服务运维准备数采资产与常驻服务。')
                return
            try:self.notifications.put_nowait(value)
            except queue.Full:
                try:self.notifications.get_nowait();self.notifications.task_done()
                except queue.Empty:pass
                self.notifications.put_nowait(value)
            self.busy=True
            if not self.running:self.value.update(status='queued',installed=deepcopy(value),error=None,worker=None,
                message='已安装标定保留，正在排队准备数采环境。')

    def snapshot(self):
        with self.lock:value=deepcopy(self.value);value['busy']=self.busy
        if value['busy'] and self.started is not None:
            progress=value.setdefault('progress',{}) or {}
            value['progress']=progress
            progress.update(elapsed_s=time.monotonic()-self.started,
                stage_elapsed_s=time.monotonic()-(self.stage_started or self.started))
            for row in progress.get('stages',[]):
                if row.get('status')=='running':row['elapsed_s']=progress['stage_elapsed_s']
        if self.diagnostic_invalidated:value.update(status='failed',diagnostic_invalidated=True)
        remaining=None if self.deadline_wall_ns is None else max(0.,(self.deadline_wall_ns-time.time_ns())/1e9)
        reason=('驻留服务已结束' if self.closed.is_set() else '驻留服务剩余时间不足，不能延长期限' if remaining is not None and remaining<=60
                else '数采环境正在准备，请等待当前请求结束' if value['busy'] else
                '当前没有可复用的有效安装组' if not (value.get('installed') or {}).get('verified') else None)
        value.update(can_retry=value['status'] in ('awaiting_prepare','failed','ready') and reason is None,
            retry_reason=reason,
            resident_deadline_wall_ns=self.deadline_wall_ns,remaining_wall_s=remaining,
            persistent_service=self.persistent_service,
            installed_calibration_preserved=bool((value.get('installed') or {}).get('verified')),
            movement_started=False)
        return value

    def _require_idle(self):
        runtime=self.device.runtime
        state=self.device.snapshot(runtime.holder if runtime else 'collection')
        if state.get('automatic_enabled') or (state.get('recording') or {}).get('done') is False:
            raise ValueError('当前动作或录制尚未结束，不能重新准备数采环境')
        if self.parts:
            with self.parts.playground.lock:busy=self.parts.playground.state.get('busy')
            if busy:raise ValueError('当前数采任务尚未结束，不能替换其准备环境')

    def retry(self):
        if self.collection_prepare_operation is None:
            raise ValueError('数采环境准备需要运维 Operation；请从统一运维入口发起')
        return self.collection_prepare_operation({})

    def environment_status(self):
        value=self.snapshot()
        result={key:value.get(key) for key in ('status','busy','error','installed','installation_identity',
            'warmup','worker','progress','wall_ns','can_retry','retry_reason','service_steps')}
        with self.lock:parts=self.parts
        services={}
        if parts is not None:
            with parts.playground.lock:
                state=parts.playground.state
                for key in ('warmup','progress','error'):result[key]=deepcopy(state.get(key))
                business_ready=bool((state.get('environment') or {}).get('ready') and
                    (state.get('warmup') or {}).get('status')=='ready')
            for name in ('detector','planner','scene'):
                manager=getattr(parts,name,None)
                if manager is not None:
                    try:services[name]=manager.status()
                    except Exception as error:services[name]=dict(status='unknown',error=str(error))
        services_ready=bool(services) and all(row.get('status') in ('ready','busy') for row in services.values())
        ready=bool(parts is not None and value['status']=='ready' and services_ready and business_ready and all(
            name not in ('planner','scene') or row.get('model_initialized') is True for name,row in services.items()))
        result.update(services=services,declared_roles=list(services),services_ready=services_ready,
            prepared=value['status']=='ready',ready=ready,needs_warmup=not ready)
        return result

    def _require_operation(self):
        if (not getattr(self.device,'maintenance',False) or self._operation_current is None
                or not self._operation_current()):raise ValueError('数采准备需要当前有效的运维维护窗口')

    def _require_control(self):require_collection_control(self.device)

    def _lifecycle_call(self,action,callback,*args,**kwargs):
        if not action.startswith('stop:'):self._require_operation()
        if self._lifecycle_admission is None or self._lifecycle_admission(action) is not True:
            raise ValueError('数采服务生命周期缺少运维准入：'+action)
        return callback(*args,**kwargs)

    def validate_asset_request(self,asset_pair_id=None,expected_catalog_digest=None):
        if asset_pair_id is not None or expected_catalog_digest is not None:
            raise ValueError('当前数采部署未登记可选择的场景资产')

    def _configure_asset_request(self,asset_pair_id,expected_catalog_digest):
        self.validate_asset_request(asset_pair_id,expected_catalog_digest)

    def operations_prepare(self,*,still_current,lifecycle_admission,service_role=None,restart=False,
            asset_pair_id=None,expected_catalog_digest=None):
        if not callable(still_current) or not callable(lifecycle_admission):raise ValueError('数采运维回调缺失')
        if type(restart)is not bool or (service_role is None and restart) or (service_role is not None and
                (service_role not in ('detector','planner','scene') or not restart)):
            raise ValueError('单角色操作需要固定服务角色与 restart=true')
        with self.lock:
            if self._operation_current is not None:raise ValueError('数采运维操作正在进行')
            self._operation_current,self._lifecycle_admission=still_current,lifecycle_admission
        try:
            self._require_operation()
            self._require_control()
            if service_role is not None and (asset_pair_id is not None or expected_catalog_digest is not None):
                raise ValueError('单角色重启不能同时更换场景资产')
            if service_role is None:self._configure_asset_request(asset_pair_id,expected_catalog_digest)
            if service_role is not None:return self._restart_role(service_role)
            result=self._retry()
            while True:
                with self.lock:busy=self.busy
                if not busy:break
                if not still_current():
                    self.request_generation+=1
                    if self.parts:self.parts.playground.cancel_event.set()
                self.closed.wait(.02)
            self._require_operation()
            value=self.snapshot()
            if value['status']!='ready':raise ValueError(value.get('error') or '数采能力尚未就绪')
            steps=deepcopy(self.value.get('service_steps',[]));environment=self.environment_status()
            if not environment['services_ready']:raise ValueError('常驻服务尚未全部核验就绪')
            return dict(result,services_ready=True,ready=environment['ready'],needs_warmup=environment['needs_warmup'],
                reused=bool(steps) and all(row['outcome']=='reused' for row in steps),
                service_steps=steps,environment=environment)
        finally:
            with self.lock:self._operation_current=self._lifecycle_admission=None

    def _retry(self):
        self._require_operation()
        with self.lock:
            if self.closed.is_set() or self.busy:raise ValueError('数采准备已关闭或正在进行，不能并发重试')
            if self.value['status'] not in ('awaiting_prepare','failed','ready'):
                raise ValueError('当前没有可初始化的数采环境')
            if self.deadline_wall_ns is not None and time.time_ns()>=self.deadline_wall_ns-60_000_000_000:
                raise ValueError('原驻留期限剩余不足，不能延长或重启设备')
            self.busy=True;self.value.update(status='verifying_installation',error=None,worker=None)
        try:
            self._require_idle()
            current=self.installed_snapshot() if self.installed_snapshot is not None else None
            if not current or current.get('verified') is not True:
                with self.lock:self.value['installed']=deepcopy(current)
                raise ValueError('当前安装组不再有效，请检查标定安装；不会自动重新采集')
            self.notify(current,prepare=True)
            return dict(queued=True,message='准备数采资产与常驻服务；业务初始化与预热由数采页明确发起。')
        except Exception as error:
            with self.lock:self.busy=False;self.value.update(status='failed',error=str(error))
            raise

    def _status(self,status,**fields):
        from scripts.probe_owner_physx import save
        with self.lock:
            if self.diagnostic_invalidated:
                status='failed';fields.update(error='复用标定诊断已失效；不能切换来源或自动转为冷启动',diagnostic_invalidated=True)
            if status=='failed':fields.setdefault('failed_stage',self.value['status'])
            stage=(status,(fields.get('progress') or {}).get('label'),(fields.get('worker') or {}).get('stage'))
            if stage!=self.stage_key:self.stage_key=stage;self.stage_started=time.monotonic()
            self.value.update(status=status,wall_ns=time.time_ns(),**fields)
            value=deepcopy(self.value)
        save(self.output/'collection-setup-status.json',value)

    def _close_parts(self):
        if self.parts:
            parts=self.parts;self.parts=None;self.identity=None;errors=[]
            parts.playground.cancel_event.set()
            parts.workflow.closed.set()
            monitor=getattr(parts.workflow,'thread',None)
            if monitor is not None:monitor.join(timeout=5)
            job=parts.playground.thread
            if job is not None:job.join(timeout=30)
            for name,manager in (('detector',parts.detector),('planner',parts.planner),('scene',getattr(parts,'scene',None))):
                if manager is None:continue
                try:self._lifecycle_call('stop:'+name,manager.close)
                except Exception as error:errors.append(str(error))
            if job is not None and job.is_alive():errors.append('collection job did not exit within cleanup budget')
            if monitor is not None and monitor.is_alive():errors.append('old collection review monitor has not exited')
            if errors:
                self.parts=parts  # A retry must finish retiring this scope before replacing it.
                raise RuntimeError('; '.join(errors))

    def installation(self,snapshot):
        if self.diagnostic_source:
            from robot_workbench.calibration_profile import diagnostic_snapshot
            if self.diagnostic_invalidated or snapshot!=diagnostic_snapshot(self.diagnostic_source):
                raise ValueError('warm diagnostic source invalidated')
            return deepcopy(self.diagnostic_source['expected_install'])
        manifest=self.output/'calibrations/hardware/active/manifest.json'
        return dict(head_id=snapshot['head_id'],wrist_id=snapshot['wrist_id'],
            manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())

    def request_fence(self,snapshot,expected,*,app_generation=None):
        generation=self.request_generation
        if app_generation is None:
            app_generation=(self.application.generations[self.device.environment] if self.request_app_generation is None
                else self.request_app_generation)
        def current():
            try:
                return (not self.closed.is_set() and generation==self.request_generation
                    and self._operation_current is not None and self._operation_current()
                    and app_generation==self.application.generations[self.device.environment]
                    and (self.installed_snapshot is None or snapshot==self.installed_snapshot())
                    and expected==self.installation(snapshot))
            except (OSError,ValueError,KeyError):return False
        current.fence=(generation,app_generation)
        return current

    def _spawn_gate(self,role,current):
        fence=getattr(current,'fence',(self.request_generation,self.application.generations[self.device.environment]))
        def launch(create):
            # Only Popen and its local handle registration belong under this
            # lock. Hashing, SSH/GPU waits, joins and model loading stay outside.
            with self.application.lock:
                self._require_operation()
                if self.closed.is_set() or fence!=(self.request_generation,self.application.generations[self.device.environment]):
                    raise InterruptedError('计算服务启动代次已撤销')
                if self._lifecycle_admission('start:'+role)is not True:raise InterruptedError('计算服务启动准入已撤销')
                return create()  # Manager stores its Popen handle before returning.
        return launch

    def _verify_parts(self,current):
        from .collection_setup import prepare_collection_attachment_process
        if not current():raise InterruptedError('数采准备身份已变化')
        checked=prepare_collection_attachment_process(self.parts.configured_path,self.identity,still_current=current)
        if checked['attachment_sha256']!=self.parts.attachment_identity:
            raise ValueError('派生资产或准备配置身份已变化；未重启现有服务')

    def _invalidate_warmup(self):
        self.application.invalidate_collection_preparation(self.device)
        playground=self.parts.playground
        with playground.lock:
            playground.state.update(can_execute=False,plan={'validated':False},
                warmup=dict(status='invalidated',message='计算服务已变化；服务就绪后仍需预热数采。'))
            playground.persist()

    def _ensure_role(self,role,current,*,restart=False):
        manager=getattr(self.parts,role,None)
        if manager is None:raise ValueError('当前准备配置不包含计算角色：'+role)
        state=manager.status();started=False
        def progress(label,**_):self._status('recovering_'+role,worker=dict(name=role,**manager.status()),message=label)
        try:
            if not current():raise InterruptedError('计算服务操作已撤销')
            before=manager.probe() if state.get('status')=='ready' else None
            reused=before is not None and not restart
            if not reused:
                self._invalidate_warmup()
                self._status('recovering_'+role,worker=dict(name=role,**state),message='仅准备计算服务：'+role)
                gate=self._spawn_gate(role,current)
                def launch(create):
                    def owned_create():
                        nonlocal started
                        started=True
                        return create()
                    return gate(owned_create)
                kwargs=dict(launch=launch)
                if not hasattr(manager,'config'):
                    started=True  # A fresh service also owns its startup reader.
                    self._lifecycle_call('start:'+role,manager.start,self.parts.pipeline_config,self.parts.worker_state_directory,**kwargs)
                    deadline=time.monotonic()+240
                    while manager.status().get('status')!='ready':
                        if not current():raise InterruptedError('计算服务启动已撤销')
                        state=manager.status()
                        if state.get('status') in ('failed','unknown','stopped') or time.monotonic()>deadline:
                            raise RuntimeError(state.get('error') or '计算服务未启动就绪：'+role)
                        self.closed.wait(.1)
                else:
                    self._lifecycle_call('recover:'+role,manager.recover,progress=progress,
                        timeout=240,restart=restart,active=current,**kwargs)
            if not current():raise InterruptedError('计算服务核验已撤销')
            proof=before if reused else manager.probe()
            if reused and proof.get('needs_warmup'):
                with self.parts.playground.lock:
                    state=self.parts.playground.state
                    invalidate=(state.get('warmup') or {}).get('status')=='ready' or (state.get('plan') or {}).get('validated') is True
                if invalidate:self._invalidate_warmup()
            if restart and before and proof['service'].get('pid')==before['service'].get('pid') and proof['service'].get('process_start_ticks')==before['service'].get('process_start_ticks'):
                raise RuntimeError('计算服务进程身份未变化，不能宣称已重启')
            row=dict(role=role,outcome='reused' if reused else 'restarted' if restart else 'recovered',
                evidence=proof,needs_warmup=proof.get('needs_warmup',False),
                retirement=deepcopy(getattr(manager,'last_retirement',None)))
            with self.lock:self.value.setdefault('service_steps',[]).append(row)
            return row
        except Exception as error:
            row=dict(role=role,outcome='unknown',error=str(error),
                service_state={k:manager.status().get(k) for k in ('status','pid','process_start_ticks','model_initialized')})
            with self.lock:self.value.setdefault('service_steps',[]).append(row)
            # A cancelled/failed start owns only this role. Healthy sibling
            # workers and the installed asset objects remain available.
            if started:
                try:self._lifecycle_call('stop:'+role,manager.close)
                except Exception as cleanup_error:row['cleanup_error']=str(cleanup_error)
            raise

    def _reuse_parts(self,expected,current):
        if expected!=self.identity or self.parts is None:return False
        self._require_idle();self._verify_parts(current)
        with self.lock:self.value['service_steps']=[]
        for role in ('detector','planner','scene'):
            if getattr(self.parts,role,None)is not None:self._ensure_role(role,current)
        with self.application.lock:
            self._require_operation();self.device.workflows['collection']=self.parts.workflow
        self._services_prepared()
        return True

    def _restart_role(self,role):
        self._require_idle()
        snapshot=self.installed_snapshot() if self.installed_snapshot is not None else None
        expected=self.installation(snapshot)
        if self.parts is None or self.identity!=expected:raise ValueError('先完成当前安装组的资产准备；单角色重启不会全量初始化')
        with self.application.lock:
            self._require_operation()
            current=self.request_fence(snapshot,expected,app_generation=self.application.generations[self.device.environment])
        self._verify_parts(current)
        with self.lock:self.busy=True;self.value['service_steps']=[]
        try:
            row=self._ensure_role(role,current,restart=True)
            self._status('awaiting_prepare',warmup=dict(status='invalidated'),error=None,
                message='计算服务已重启，数采仍待预热；其他健康服务保留。')
            return dict(service_role=role,service_ready=True,ready=False,needs_warmup=True,
                service_step=row,environment=self.environment_status())
        except Exception as error:
            self._status('failed',error=str(error));raise
        finally:
            with self.lock:self.busy=False

    def _prepare(self,snapshot):
        self._require_operation()
        from local_execution import gpu_check
        from robot_workbench.collection_setup import prepare_collection_setup,finalize_collection_setup
        root=self.output;expected=self.installation(snapshot);current=self.request_fence(snapshot,expected)
        if not current():raise InterruptedError('preparation request or installation changed')
        if self._reuse_parts(expected,current):return
        if self.parts:
            self._require_idle()
            self.application.invalidate_collection_preparation(self.device)
            self._close_parts()
        details=self.contract['collection_setup']
        script=str(Path(__file__).resolve().parents[1]/'scripts/serve_physx_device.py')
        from robot_workbench.preview_service import resident_execution
        execution=resident_execution(self.contract,root)
        # Persistent service does not make an individual preparation unbounded.
        prepare_deadline=time.time_ns()+int(min(self.contract['resources']['max_wall_time_minutes']*60,7200)*1e9)
        if self.deadline_wall_ns is not None:prepare_deadline=min(prepare_deadline,self.deadline_wall_ns-45_000_000_000)
        output=root/('collection-'+expected['manifest_sha256'][:16]+'-'+str(time.time_ns()))
        self._status('preparing_profile',installed=deepcopy(snapshot),installation_identity=expected,output=str(output),error=None)
        detector_entry=Path(__file__).resolve().parents[1]/'detector_adapter'/'isaac_rgbd_detection.py'
        if not detector_entry.is_file():raise FileNotFoundError('frozen detector adapter missing: '+str(detector_entry))
        pipeline_config={**self.base['pipeline_config'],'detection_script':str(detector_entry)}
        setup=prepare_collection_setup(run_root=root,calibration_config=root/'calibration-config.json',
            capture_root=root/'calibration-workbench',expected_install=expected,registration=root/'registration.json',
            profile_template=details['profile_template'],base_pipeline_config=pipeline_config,execution=execution,
            contract_template=self.contract,detection_runtime_root=details['detection_runtime_root'],
            control_url=self.control_url,token_file=self.token_file,producer_ports=self.ports,producer_directories=self.directories,
            trusted_binding_path=root/'collection-binding.json',local_recording_root=root/'collection-raw',
            resident_deadline_wall_ns=self.deadline_wall_ns,output=output,
            **({'diagnostic_source':self.diagnostic_source} if self.diagnostic_source else {}))
        budget=min(600.,(prepare_deadline-time.time_ns())/1e9-15)
        if budget<=0:raise TimeoutError('resident deadline leaves no conversion/cleanup budget')
        if not current():raise InterruptedError('resident preparation cancelled before conversion')
        gpu_check({'execution':execution})
        conversion=setup['conversion'];wb=Path(script).parents[1]/'rule_based_workbench'
        from launch_environment import owned_command,clean_environment,clean_command
        command=owned_command(wb,clean_command(conversion['argv'],conversion['env']),budget)
        self._status('converting_planning_assets',output=str(output),deadline_wall_ns=self.deadline_wall_ns)
        with (output/'conversion.log').open('xb') as log:
            self.process=subprocess.Popen(command,env=clean_environment(),stdout=log,stderr=subprocess.STDOUT)
            while self.process.poll() is None:
                if self.closed.wait(.2) or not current():
                    self.process.terminate();self.process.wait(timeout=30);raise InterruptedError('resident conversion cancelled')
            if self.process.returncode:raise RuntimeError('owned asset conversion failed; inspect '+str(output/'conversion.log'))
        self.process=None
        if not current():raise InterruptedError('resident preparation cancelled')
        configured=finalize_collection_setup(setup['setup_path'])
        if not current() or time.time_ns()>=prepare_deadline:
            raise InterruptedError('resident preparation cancelled or deadline reached before worker startup')
        self._activate_configured(configured,output/'configured.json',expected,prepare_deadline,still_current=current)

    def _activate_configured(self,configured,configured_path,expected,prepare_deadline,*,still_current=None):
        """Shared asset attachment and resident-service startup; business warmup is separate."""
        from contextlib import nullcontext
        from robot_workbench.collection_setup import construct_collection,prepare_collection_attachment_process
        from scripts.probe_owner_physx import save
        def current():
            return (not self.closed.is_set() and time.time_ns()<prepare_deadline
                and (still_current is None or still_current()))
        def require_current():
            if not current():raise InterruptedError('preparation cancelled or installation changed')
        # The large frozen contract and its source pins are checked in a CPU
        # process before taking the application/installation locks.
        validated=prepare_collection_attachment_process(configured_path,expected,still_current=current)
        if configured!=validated['configured']:raise ValueError('prepared collection configuration changed before attachment')
        if configured.get('control_owner_reference')is not None:
            from .collection_setup import trusted_binding_location,trusted_recording_location
            trusted_binding_location(configured['trusted_binding_path'],self.output,configured['control_owner_reference'])
            trusted_recording_location(validated['bridge_config']['local_recording_root'],self.output,configured['control_owner_reference'])
        calibration=self.device.workflows.get('calibration')
        # Installation notification takes this same lock before revoking reviews.
        # Stop revokes the application generation; it never waits for model loading.
        with getattr(calibration,'install_lock',nullcontext()):
            with self.application.lock:
                require_current()
                save(Path(configured['trusted_binding_path']),validated['installed_binding'])
                self.parts=construct_collection(configured_path,self.device,self.application,validated=validated)
                self.identity=expected  # Validated asset scope, not service readiness.
        current.fence=getattr(still_current,'fence',(self.request_generation,self.application.generations[self.device.environment]))
        self._status('initializing_detector',worker=None)
        with self.lock:self.value['service_steps']=[]
        for name in ('detector','planner','scene'):
            manager=getattr(self.parts,name,None)
            if manager is None:continue
            require_current()
            self._ensure_role(name,current)
        with getattr(calibration,'install_lock',nullcontext()):
            with self.application.lock:
                require_current()
                self.device.workflows['collection']=self.parts.workflow
                self.identity=expected
        self._status('initializing_planner',configured=str(configured_path),error=None)
        self._services_prepared()

    def _services_prepared(self):
        self._require_operation()
        with self.parts.playground.lock:warmup=deepcopy(self.parts.playground.state.get('warmup') or {'status':'not_started'})
        self._status('ready',error=None,movement_started=False,warmup=warmup,
            message='数采资产与常驻服务已准备；请在数采页初始化与预热。',requires_explicit_handoff=True)

    def _run(self):
        while not self.closed.is_set():
            try:snapshot=self.notifications.get(timeout=.1)
            except queue.Empty:continue
            with self.lock:
                self.running=True;self.busy=True
                self.started=self.stage_started=time.monotonic();self.stage_key=None
                self.error=None
                for key in ('failed_stage','cleanup_error','status_log_error'):self.value.pop(key,None)
                self.value.update(status='preparing_profile',installed=deepcopy(snapshot),error=None,worker=None,attempt=self.value['attempt']+1)
            previous_parts=self.parts
            try:self._prepare(snapshot)
            except Exception as error:
                self.error=str(error)
                try:self._status('failed',error=str(error),movement_started=False)
                except OSError as log_error:
                    with self.lock:self.value['status_log_error']=str(log_error)
                if self.parts is not None and self.parts is not previous_parts and self.identity is None:
                    try:self._close_parts()
                    except Exception as cleanup_error:
                        with self.lock:self.value['cleanup_error']=str(cleanup_error)
            finally:
                self.notifications.task_done()
                with self.lock:self.running=False;self.busy=not self.notifications.empty()

    def close(self,*,lifecycle_admission=None):
        self.closed.set()
        self.thread.join(timeout=35)
        if self.thread.is_alive():raise TimeoutError('owned collection preparation has not finished cleanup')
        prior=self._lifecycle_admission
        if lifecycle_admission is not None:self._lifecycle_admission=lifecycle_admission
        try:self._close_parts()
        finally:self._lifecycle_admission=prior
