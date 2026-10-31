"""Three explicit Playground operations, with isolated artifacts and no auto-resume."""
from copy import deepcopy
import inspect
import json
import math
from pathlib import Path
from threading import Event, RLock, Thread
import time
from uuid import uuid4
from real_contract import namespace, owner_matches_environment


class Playground:
    def __init__(self, directory, bridge, initialization, external_busy=lambda: False, *, recover_planner=None):
        self.directory = Path(directory).resolve()
        self.bridge, self.initialization, self.external_busy = bridge, initialization, external_busy
        self.recover_planner = recover_planner
        self.managed_lifecycle=False
        self.lifecycle_admission=None
        self.lock = RLock()
        self.cancel_event = Event()
        self.thread = None
        self.started = None
        self.stage_started = None
        self.state = dict(status='idle', busy=False, environment={'ready': False, 'checks': []},
                          warmup={'status':'not_started','message':'点击初始化与预热。'},
                          plan={'validated': False}, can_execute=False, videos=[], cameras={}, progress={})
        saved = self.directory / 'state.json'
        if saved.is_file():
            self.state.update(json.loads(saved.read_text()))
            self.state.update(status='stopped', busy=False, can_execute=False,
                              message='服务已重启；环境将按实时连接重新核验，不会自动执行动作。')
            self.state['plan']['validated'] = False
            self.state['warmup']={'status':'invalidated','message':'服务已重启，需要重新初始化与预热。'}

    @property
    def busy(self):
        with self.lock:
            return self.state['busy']

    def persist(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        path = self.directory / 'state.json'
        temporary = path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(self.state, ensure_ascii=False, allow_nan=False))
        temporary.replace(path)

    def candidate(self):
        candidate = self.bridge.config.get('playground') if self.bridge else None
        if not isinstance(candidate, dict) or candidate.get('enabled') is not True:
            raise ValueError('Playground candidate 尚未配置；不会使用旧工作台轨迹作为替代')
        return deepcopy(candidate)

    def assisted_input(self,config,request_id):
        """Only the server-configured, explicitly named one-off physical source."""
        from integration import _digest
        from real_contract import validate_real_source
        if (self.bridge.config.get('environment','physical')!='physical' or not isinstance(request_id,str)
                or not request_id.strip() or len(request_id)>256):
            raise ValueError('辅助定位需要本次明确的真机请求身份')
        configured=config.get('operator_assisted_directory')
        if not isinstance(configured,str) or not Path(configured).is_absolute():
            raise ValueError('服务端尚未配置本次辅助定位工件')
        directory=Path(configured)
        if directory.is_symlink() or not directory.is_dir():raise ValueError('辅助定位目录无效')
        values=[]
        for name in ('source.json','hardware-binding.json'):
            path=directory/name
            if path.is_symlink() or not path.is_file() or path.stat().st_size>4*1024*1024:
                raise ValueError('辅助定位工件缺失或过大')
            values.append(json.loads(path.read_text()))
        source,binding=values
        receipt=((source.get('scene') or {}).get('source') or {}).get('assistance_receipt') or {}
        if source.get('adapter')!='operator_assisted_rgbd' or receipt.get('request_id')!=request_id:
            raise ValueError('辅助定位请求身份与服务端本次工件不一致')
        validate_real_source(source,binding,environment='physical')
        return directory,_digest(source)

    def checkpoint(self,plan,*,execution_pending=False,execution_output=None):
        self.directory.mkdir(parents=True,exist_ok=True)
        path=self.directory/'prepared-plan.json'
        temporary=path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(dict(schema_version='playground_prepared_plan.v1',plan=plan,
            execution_pending=execution_pending,execution_output=str(execution_output) if execution_output else None),
            ensure_ascii=False,allow_nan=False))
        temporary.replace(path)

    def restore_prepared(self):
        """Explicit local checkpoint recovery; no acquisition, warmup, or motion."""
        import playground_pipeline as pipeline
        from real_observation import read_initial_sample
        from joint_space import state_near
        if self.bridge.config.get('environment','physical')!='physical':
            raise ValueError('冻结计划恢复目前仅用于明确的真机接入')
        with self.lock:
            if self.state['busy'] or self.external_busy():raise ValueError('已有任务正在使用工作台')
            self.cancel_event.clear()
            self.started=self.stage_started=time.monotonic()
            self.state.update(busy=True,can_execute=False,action='plan',
                progress=dict(label='核验本服务冻结计划与当前实测起点',stage_status='running'))
        try:
            plan=pipeline.restore_plan(self.candidate(),self.directory,self.bridge)
            if not isinstance(plan,dict) or plan.get('validated') is not True:
                raise ValueError('没有可恢复的未执行冻结计划；不会自动预热或重新观测')
            source_path=self.artifact(plan['source_path'],self.directory/'jobs')
            source=json.loads(source_path.read_text());initial=source['initial_state']
            current,evidence=read_initial_sample(self.bridge.owner,environment='physical')
            disabled=initial.get('disabled_grippers',[])
            if (evidence.get('disabled_grippers',[])!=disabled or not state_near(current,
                    initial['measured_reported'],joint_rad=.01,disabled_grippers=disabled)):
                raise ValueError('当前实测姿态或禁用夹爪配置已改变，不能恢复旧计划；请重新观测规划')
            image=self.artifact(plan['first_frame_path'],self.directory/'jobs')
            self.validate_first_frame(image)
            output=source_path.parent
            plan.update(output=str(output),first_frame_url=f'/api/playground/media/frame?v={output.name}')
            with self.lock:
                if self.cancel_event.is_set():raise InterruptedError('冻结计划恢复已中断')
                self.state.update(status='ready',plan=plan,error=None,output=str(output),
                    message='已恢复冻结计划并核对当前起点；未预热模型、未重新观测、未执行动作。请重新审阅后执行。',
                    environment=dict(ready=True,scope='frozen_plan_and_current_owner_only',checks=[dict(
                        key='frozen_plan',label='冻结文件、标定绑定与当前实测起点',status='ready')]))
                self.state['progress'].update(stage_status='completed')
            return deepcopy(plan)
        except BaseException as error:
            with self.lock:
                self.state.update(status='failed',error=str(error),can_execute=False,
                    message='冻结计划未恢复；不会自动预热、重新观测或执行动作。')
                self.state['plan']['validated']=False
                self.state['progress'].update(stage_status='failed')
            raise
        finally:
            with self.lock:
                self.state['busy']=False
                self.persist()

    def health(self):
        if not self.bridge:
            return dict(connected=False, owner={}, cameras={})
        return self.bridge.status()

    def snapshot(self):
        health = self.health()
        detector = self.initialization()
        with self.lock:
            value = deepcopy(self.state)
        warmup=value.setdefault('warmup',{'status':'not_started','message':'点击初始化与预热。'})
        for name,label in (('planner','规划'),('scene','场景')):
            component=detector.get(name)
            if (warmup.get('status')=='ready' and component and
                    (component.get('status') not in ('ready','busy') or component.get('model_initialized') is not True)):
                warmup.update(status='invalidated',message=label+'进程或模型已失效，需要重新初始化与预热。')
        prepared_environment_ready = value['environment']['ready']
        owner = health.get('owner', {})
        environment = self.bridge.config.get('environment', 'physical') if self.bridge else 'physical'
        spec = namespace(environment)
        control_ready = bool(health.get('connected') and owner_matches_environment(owner, environment)
                             and owner.get('holder', 'collection') == 'collection'
                             and owner.get('feedback_healthy') and owner.get('state') in
                             ('READY','STOPPED_DAMPING_HOLD','MANUAL_TAKEOVER')
                             and (owner.get('sample_age_s') is None or type(owner['sample_age_s']) in (int,float)
                                  and 0 <= owner['sample_age_s'] <= .1)
                             and owner.get('automatic_enabled') is False)
        value['cameras'] = health.get('cameras', {})
        value.update(device_environment=environment, environment_label=spec['label'])
        dynamic = [dict(key='control', label=spec['label']+'控制', status='ready' if control_ready else 'pending',
                        message=owner.get('error', '')),
                   dict(key='cameras', label='三路 RGB-D', status='ready' if
                        all(value['cameras'].get(key, {}).get('ready') is True for key in
                            ('base_0', 'left_wrist_0', 'right_wrist_0')) else 'pending'),
                   dict(key='detector', label='检测器', status='ready' if detector.get('status') in
                        ('ready', 'busy') else 'pending', message=detector.get('message', ''))]
        checks = [c for c in value['environment']['checks'] if c['key'] not in ('control', 'cameras', 'detector')]
        value['environment']['checks'] = checks + dynamic
        value['environment']['control_connected'] = health.get('connected') is True
        value['environment']['ready'] = bool(value['environment']['ready'] and warmup.get('status')=='ready' and
                                               all(c['status'] == 'ready' for c in checks + dynamic))
        safe = (control_ready and owner.get('stationary') is True and owner.get('automatic_enabled') is False
                and not owner.get('acquisition_pending')
                and (owner.get('allowed_operations') or {}).get('preview', {}).get('allowed', True) is True
                and owner.get('pedal', {}).get('input_healthy') is True
                and owner.get('pedal', {}).get('physically_down') is False)
        value['can_execute'] = bool(not value['busy'] and not self.external_busy() and safe
                                    and prepared_environment_ready and all(c['status']=='ready' for c in checks)
                                    and value['plan'].get('validated') is True)
        value['capture_ready'] = bool(owner.get('recording_configured') and all(
            value['cameras'].get(key, {}).get('ready') is True for key in ('base_0','left_wrist_0','right_wrist_0')))
        value['capture_warning'] = None if value['capture_ready'] else '图像或录制状态待确认；动作独立运行，数据完整性以封存校验为准。'
        if value['plan'].get('validated') is True and not safe and not value['busy']:
            value['message'] = '计划已验证；控制器尚未允许执行（' + str(owner.get('state', '未知')) + '），请检查真机连接与踏板状态。'
        if self.started is not None and value['busy']:
            value['progress']['elapsed_s'] = time.monotonic() - self.started
            if value['progress'].get('stage_status') == 'running':
                value['progress']['stage_elapsed_s'] = time.monotonic() - self.stage_started
            for row in value['progress'].get('stages', []):
                if row.get('status') == 'running':
                    row['elapsed_s'] = value['progress']['stage_elapsed_s']
        if not self.bridge or not self.bridge.config.get('playground', {}).get('enabled'):
            value.update(can_execute=False, message='Playground candidate 尚未配置；请等待部署。')
            value['environment']['ready'] = False
        return value

    def progress(self, label, **fields):
        with self.lock:
            for key in ('errors', 'recording', 'physical_outcome', 'task_outcome', 'native_task', 'simulation', 'dataset'):
                if key in fields:
                    self.state[key] = deepcopy(fields.pop(key))
            if self.state.get('action') == 'execute' and 'errors' in self.state:
                self.state['error'] = '; '.join(f'{key}: {value}' for key, value in self.state['errors'].items()) or None
            outcome = self.state.get('physical_outcome')
            failed = self.state.get('action') == 'execute' and outcome is not None and outcome != 'completed'
            if failed and not self.state.get('error'):
                self.state['error'] = f'真机执行结果：{outcome}；控制器未提供具体原因'
            if self.state['progress'].get('label') != label:
                self.stage_started = time.monotonic()
                self.state['progress'].update(stage_status='running', stage_estimate_s=None)
            self.state['progress'].update(fields)
            self.state['progress'].update(label=label, elapsed_s=time.monotonic() - self.started,
                                          stage_elapsed_s=time.monotonic() - self.stage_started)
            self.state['message'] = ('真机执行未完成；' if failed else '') + label
            self.persist()

    def call(self, fn, *args):
        parameters=inspect.signature(fn).parameters
        options={}
        if 'progress' in parameters:options['progress']=self.progress
        if 'cancel' in parameters:options['cancel']=self.cancel_event.is_set
        return fn(*args,**options)

    def invalidate_checkpoint(self):
        path=self.directory/'prepared-plan.json'
        if path.is_file():
            saved=json.loads(path.read_text());saved['invalidated_by_interrupt']=True
            temporary=path.with_suffix('.json.tmp')
            temporary.write_text(json.dumps(saved,ensure_ascii=False,allow_nan=False))
            temporary.replace(path)

    def interrupt(self,payload):
        if payload!={'action':'interrupt'}:raise ValueError('中断请求只接受 action')
        with self.lock:
            busy=self.state['busy'];active=self.state.get('action')
        if busy and active=='execute':
            owner=self.bridge.owner()
            if owner.get('automatic_enabled'):
                self.bridge.control({'action':'stop'})
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    owner=self.bridge.owner()
                    if not owner.get('automatic_enabled'):break
                    time.sleep(.1)
                else:raise ValueError('真机停止尚未确认；继续保持中断请求')
        live_ready=self.snapshot()['environment']['ready']
        with self.lock:
            if self.state['busy']:
                self.cancel_event.set()
                self.state['message']='正在中断当前阶段，等待任务进程退出。'
                self.persist()
                return dict(status='stopping')
            self.invalidate_checkpoint()
            ready=live_ready
            self.state.update(status='ready' if ready else 'stopped',plan={'validated':False},
                              can_execute=False,error=None,
                              message='当前阶段已中断；环境准备保持就绪，可重新场景重建及规划。' if ready
                              else '当前阶段已中断；控制或检测环境尚未就绪，恢复后可重新规划。')
            self.state['progress'].update(stage_status='interrupted',next_label='场景重建及规划' if ready else '环境准备')
            self.persist()
        return dict(status='ready' if ready else 'stopped')

    def start(self, payload, *, lifecycle_admission=None):
        if payload.get('action')=='interrupt':return self.interrupt(payload)
        if set(payload) - {'action', 'speed_scale', 'arms_supported','operator_assisted_request_id'}:
            raise ValueError('Playground 请求包含未知字段')
        action, speed = payload.get('action'), payload.get('speed_scale', .3)
        if action not in ('prepare', 'plan', 'execute'):
            raise ValueError('未知 Playground 操作')
        if self.managed_lifecycle and action=='prepare':
            if lifecycle_admission is None or lifecycle_admission('warmup:collection') is not True:
                raise ValueError('初始化与预热需要当前业务准入；不会启动服务或接入设备')
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or not .3 <= speed <= 1.7:
            raise ValueError('运动速度必须在 30% 至 170% 之间')
        config = self.candidate()
        assisted=None
        if 'operator_assisted_request_id' in payload:
            if action!='plan':raise ValueError('辅助定位身份只用于规划，不能准备或执行动作')
            request_id=payload['operator_assisted_request_id']
            directory,digest=self.assisted_input(config,request_id)
            from real_observation import read_initial_sample
            read_initial_sample(self.bridge.owner,environment='physical')  # Never auto-connect an assisted request.
            assisted=dict(request_id=request_id,directory=str(directory),source_digest=digest)
        fresh = self.snapshot()
        with self.lock:
            if self.state['busy'] or self.external_busy():
                raise ValueError('已有运行正在使用工作台；请等待结束')
            if action=='prepare' and fresh.get('warmup',{}).get('status')=='ready' and fresh['environment']['ready']:
                return dict(status='ready',action=action,output=self.state.get('output',str(self.directory)),reused=True)
            if action == 'plan' and not fresh['environment']['ready'] and (self.managed_lifecycle or assisted is None):
                raise ValueError('请先完成环境准备')
            if action == 'execute' and not fresh['can_execute']:
                raise ValueError('当前环境、控制状态或本轮计划尚不允许执行')
            if (action == 'prepare' and self.bridge.config.get('environment','physical')=='physical'
                    and not self.health().get('connected') and payload.get('arms_supported') is not True):
                raise ValueError('连接控制会使能电机，需要确认当前双臂已受支撑')
            output = self.directory / 'jobs' / uuid4().hex
            self.cancel_event.clear()
            output.mkdir(parents=True, exist_ok=False)
            plan = deepcopy(self.state['plan'])
            plan['speed_scale'] = speed
            if action=='execute':
                self.checkpoint(plan,execution_pending=True,execution_output=output)
            self.started = time.monotonic()
            self.stage_started = self.started
            previous_time = self.state.get('timing_history', {}).get(action)
            estimate = previous_time if action != 'execute' and previous_time else {'prepare': 180, 'plan': 300,
                       'execute': plan.get('duration_s', 70) + 120}[action]
            self.state.update(status='running', busy=True, can_execute=False, error=None, failure=None, action=action,output=str(output),
                              message={'prepare': '正在准备环境', 'plan': '正在重建场景与规划', 'execute': '正在执行与录制'}[action],
                              progress={'label': {'prepare': '准备环境', 'plan': '准备观测', 'execute': '校验执行输入'}[action],
                                        'elapsed_s': 0, 'stage_elapsed_s': 0, 'stage_status': 'running',
                                        'estimate_s': estimate, 'estimate_source': '上次成功耗时' if previous_time and action != 'execute'
                                        else '粗估基线（非完成承诺）', 'stages': [], 'next_label': ''})
            self.state.pop('dataset',None)
            if action == 'plan':
                self.state.update(plan={'validated': False}, videos=[])
            elif action=='execute':
                for key in ('errors', 'recording', 'simulation', 'physical_outcome', 'task_outcome', 'native_task', 'execution_instance_id', 'execution_scope', 'execution_wait', 'simulation_scope'):
                    self.state.pop(key, None)
                self.state['plan']['validated'] = False  # A click consumes this plan, including failed/mid-run attempts.
                self.state['videos'] = []
            if action == 'prepare':
                self.lifecycle_admission=lifecycle_admission
                self.state['environment'] = {'ready': False, 'checks': []}
                self.state['warmup']={'status':'running','message':'正在初始化与预热，本次服务只需完成一次。'}
            self.persist()
            self.thread = Thread(target=self.run, args=(action, config, output, plan, speed,
                                                        payload.get('arms_supported') is True,assisted), daemon=True)
            self.thread.start()
            return dict(status='accepted', action=action,output=str(output))

    def finish_interrupt(self):
        self.invalidate_checkpoint()
        recovery_error=None
        if self.recover_planner and not self.managed_lifecycle:
            try:self.recover_planner(progress=self.progress)
            except Exception as error:recovery_error=str(error)
        ready=not recovery_error and self.snapshot()['environment']['ready']
        with self.lock:
            if self.state.get('warmup',{}).get('status')=='running':
                self.state['warmup']={'status':'invalidated','message':'初始化已中断；已完成的健康缓存保留，点击初始化与预热继续。'}
            if recovery_error:self.state['environment']['ready']=False
            self.state.update(status='failed' if recovery_error else 'ready' if ready else 'stopped',plan={'validated':False},
                              can_execute=False,error=recovery_error,
                              message='规划服务恢复失败：'+recovery_error if recovery_error else
                              '当前阶段已中断；环境准备保持就绪，可重新场景重建及规划。' if ready
                              else '当前阶段已中断；控制或检测环境尚未就绪，恢复后可重新规划。')

    def prepare_services(self,pipeline,config):
        if self.cancel_event.is_set():raise InterruptedError('环境准备已中断')
        if self.managed_lifecycle and (self.lifecycle_admission is None or self.lifecycle_admission('warmup:collection') is not True):
            raise ValueError('数采业务预热准入已撤销')
        state=self.initialization()
        if self.recover_planner and (state.get('status') in ('failed','stopped','unknown') or
                any((state.get(name) or {}).get('status') in ('failed','stopped','unknown') for name in ('planner','scene'))):
            if self.managed_lifecycle:
                raise ValueError('计算服务在准备期间失效；请从运维重新准备，不在业务预热中自动重启')
            self.recover_planner(progress=self.progress)
        checks=self.call(pipeline.prepare_environment,config)
        if isinstance(checks,dict):checks=checks['checks']
        if not checks or any(c.get('status')!='ready' for c in checks):
            raise ValueError('DSW 与仿真预检尚未全部就绪：'+json.dumps(checks,ensure_ascii=False))
        with self.lock:self.state['environment']['checks']=checks
        deadline=time.monotonic()+240
        while self.initialization().get('status')!='ready':
            if self.cancel_event.is_set():raise InterruptedError('环境准备已中断')
            state=self.initialization()
            if state.get('status') in ('failed','disabled','stopped') or time.monotonic()>deadline:
                raise ValueError('检测器未就绪：'+str(state.get('error') or state.get('message')))
            self.progress(state.get('message') or '等待检测器加载',next_label='核对当前控制与三路相机')
            time.sleep(.5)

    def run(self, action, config, output, plan, speed, supported,assisted=None):
        try:
            import playground_pipeline as pipeline
            if action == 'prepare':
                self.prepare_services(pipeline,config)
                if not self.health().get('connected'):
                    if self.managed_lifecycle:raise ValueError('控制未接入；请通过运维接入，数采准备不会连接设备')
                    if not supported and self.bridge.config.get('environment','physical')=='physical':
                        raise ValueError('控制接入缺少当前支撑确认')
                    self.progress('接入真机控制与三路相机', next_label='确认环境就绪')
                    self.bridge.control({'action': 'acquire', 'confirmation': 'ARMS_SUPPORTED_CONNECT'})
                deadline = time.monotonic() + 30
                while True:
                    if self.cancel_event.is_set():raise InterruptedError('环境准备已中断')
                    with self.lock:
                        self.state['environment']['ready'] = True
                    if all(row['status']=='ready' for row in self.snapshot()['environment']['checks']):
                        break
                    if time.monotonic() > deadline:
                        raise ValueError('控制或三路 RGB-D 接入尚未完成；请检查服务状态')
                    time.sleep(.5)
                planner=self.initialization().get('planner')
                warmup=self.call(pipeline.warmup,config,output,self.bridge) if planner else {'model_initialized':True}
                if not isinstance(warmup,dict) or warmup.get('model_initialized') is not True:
                    raise ValueError('规划服务进程存在，但模型尚未完成预热')
                with self.lock:
                    self.state['warmup']={**warmup,'status':'ready',
                        'message':'初始化与预热完成；移动杯子、咖啡机或中断复位后可直接重新观测规划。',
                        'elapsed_s':time.monotonic()-self.started}
                restored=self.call(pipeline.restore_plan,config,self.directory,self.bridge)
                if restored is not None and (not isinstance(restored,dict) or restored.get('validated') is not True):
                    raise ValueError('恢复计划未通过校验')
                result = dict(status='ready',plan=restored or {**self.state['plan'],'validated':False},
                              message='环境准备完成，已恢复上次计划；请核对现场后点击 action 执行。' if restored
                              else '环境准备完成，可以重建场景与规划。')
            elif action == 'plan':
                if assisted is None:
                    plan = self.call(pipeline.plan, config, output, self.bridge, speed)
                else:
                    from integration import _digest
                    from real_observation import read_initial_sample
                    directory,digest=self.assisted_input(config,assisted['request_id'])
                    if digest!=assisted['source_digest'] or str(directory)!=assisted['directory']:
                        raise ValueError('辅助定位工件在请求后已改变，请重新提交明确请求')
                    read_initial_sample(self.bridge.owner,environment='physical')
                    if not self.snapshot()['environment']['ready']:
                        if self.managed_lifecycle:raise ValueError('数采能力缺失；请通过运维准备后重新规划')
                        with self.lock:
                            self.state['warmup']={'status':'running','message':'为本次辅助定位准备模型；不会自动接入或执行机械臂。'}
                        self.prepare_services(pipeline,config)
                    read_initial_sample(self.bridge.owner,environment='physical')
                    plan=self.call(pipeline.plan_assisted,config,output,self.bridge,speed,directory)
                    if _digest(json.loads(Path(plan['source_path']).read_text()))!=assisted['source_digest']:
                        raise ValueError('规划返回的场景不属于本次辅助定位请求')
                if not isinstance(plan, dict) or plan.get('validated') is not True or not plan.get('timeline_digest'):
                    raise ValueError('规划未返回本轮路径验证证据')
                for key in ('source_path', 'plan_path', 'hardware_binding'):
                    self.artifact(plan.get(key), output)
                image = self.artifact(plan.get('first_frame_path'), output) if plan.get('first_frame_path') else None
                if image:
                    self.validate_first_frame(image)
                plan.update(output=str(output), first_frame_url=(f'/api/playground/media/frame?v={output.name}' if image else None))
                self.checkpoint(plan)
                if assisted is not None:
                    with self.lock:
                        self.state['environment']['ready']=True
                        self.state['warmup']=dict(status='ready',model_initialized=True,
                            message='本次辅助定位的场景与规划模型已就绪；尚未执行机械臂。',
                            elapsed_s=time.monotonic()-self.started,operator_assisted_request_id=assisted['request_id'])
                result = dict(status='ready', plan=plan, message='规划完成；请核对首帧与现场后点击 action 执行。')
            else:
                from playground_execution_process import execute
                result = self.call(execute, config, output, self.bridge.config, plan)
                if not isinstance(result, dict) or result.get('status') not in ('completed', 'failed', 'stopped'):
                    raise ValueError('执行未返回明确终态；禁止自动续跑')
                # Recorder loss can report zero commands after completed motion.
                # A returned execution result never refunds the consumed plan.
                videos = []
                for item in result.get('videos', []):
                    path = self.artifact(item['path'], output)
                    videos.append(dict(label=item['label'], path=str(path),
                                       url=f'/api/playground/media/video-{len(videos)}?v={output.name}'))
                result['videos'] = videos
            with self.lock:
                self.state.update(result)
        except InterruptedError:
            self.finish_interrupt()
        except Exception as error:
            if self.cancel_event.is_set():
                self.finish_interrupt()
                return
            with self.lock:
                self.state.update(status='failed', error=str(error), can_execute=False,
                                  failure=deepcopy(getattr(error,'failure',None)),
                                  message='本步骤失败；不会自动重试或发起后续动作。')
                self.state['plan']['validated'] = False
                if action == 'execute':
                    phase=output/'execution-phase.json'
                    if phase.is_file() and json.loads(phase.read_text())==dict(phase='pre_action',timeline_digest=plan['timeline_digest']):
                        plan['validated']=True
                        self.checkpoint(plan,execution_output=output)
                        self.state.update(plan=plan,message='执行准备失败，尚未请求真机动作；计划已保留，可修复后手动重试。')
                if action == 'prepare':
                    self.state['environment']['ready'] = False
                    failure=deepcopy(getattr(error,'preparation_failure',None)) or dict(
                        stage='unknown',producer_stage='unknown',native_phase='unknown',native_status='unknown',
                        native_reason='unknown',error=str(error),native_result_path=None,job_output=str(output))
                    self.state['warmup']={'status':'failed','message':str(error),'failure':failure}
                elif assisted is not None and self.state.get('warmup',{}).get('status')=='running':
                    self.state['warmup']={'status':'failed','message':str(error)}
                    self.state['environment']['ready']=False
        finally:
            with self.lock:
                self.state['busy'] = False
                interrupted=self.cancel_event.is_set()
                successful = self.state['status'] in ('ready', 'completed') and not interrupted
                elapsed = time.monotonic() - self.started
                self.state['progress'].update(elapsed_s=elapsed, stage_elapsed_s=time.monotonic() - self.stage_started,
                                               stage_status='interrupted' if interrupted else 'completed' if successful else self.state['status'],
                                               next_label='场景重建及规划' if interrupted and self.state['environment'].get('ready') else
                                               {'prepare': '场景重建及规划', 'plan': '核对首帧后点击 action 真机执行',
                                                'execute': '独立回放本轮视频'}[action] if successful else '检查失败原因后重新操作',
                                               percent=100 if successful else 0)
                if action!='plan' and self.state['plan'].get('validated') is True:
                    self.state['progress']['next_label']='核对现场后点击 action 真机执行'
                if successful:
                    self.state.setdefault('timing_history', {})[action] = elapsed
                self.persist()
                self.cancel_event.clear()

    @staticmethod
    def validate_first_frame(image):
        data=image.read_bytes()
        if not ((data.startswith(b'\x89PNG\r\n\x1a\n') and data[12:16]==b'IHDR'
                 and len(data)>=24 and int.from_bytes(data[16:20],'big')>0
                 and int.from_bytes(data[20:24],'big')>0 and b'IEND' in data[-16:])
                or (data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9'))):
            raise ValueError('首帧不是有效的 PNG/JPEG 产物')

    def artifact(self, value, root=None):
        if not isinstance(value, str):
            raise ValueError('本轮产物路径缺失')
        path = Path(value).resolve()
        if not path.is_relative_to(Path(root or self.directory).resolve()) or not path.is_file() or path.stat().st_size == 0:
            raise ValueError('本轮产物不存在、为空或不在本轮输出目录')
        return path

    def media(self, name, revision=None):
        with self.lock:
            if name == 'frame':
                value = self.state['plan'].get('first_frame_path')
                expected = Path(self.state['plan'].get('output', '')).name
            elif name.startswith('video-') and name[6:].isdigit():
                index = int(name[6:])
                item = self.state['videos'][index] if index < len(self.state['videos']) else {}
                value = item.get('path')
                expected = item.get('url', '').rsplit('v=', 1)[-1]
            else:
                value = None
                expected = None
        if revision != expected:
            return None
        return self.artifact(value) if value else None
