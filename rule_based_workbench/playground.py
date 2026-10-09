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
        self.lock = RLock()
        self.cancel_event = Event()
        self.thread = None
        self.started = None
        self.stage_started = None
        self.state = dict(status='idle', busy=False, environment={'ready': False, 'checks': []},
                          plan={'validated': False}, can_execute=False, videos=[], cameras={}, progress={})
        saved = self.directory / 'state.json'
        if saved.is_file():
            self.state.update(json.loads(saved.read_text()))
            self.state.update(status='stopped', busy=False, can_execute=False,
                              message='服务已重启；环境将按实时连接重新核验，不会自动执行动作。')
            self.state['plan']['validated'] = False

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

    def checkpoint(self,plan,*,execution_pending=False,execution_output=None):
        self.directory.mkdir(parents=True,exist_ok=True)
        path=self.directory/'prepared-plan.json'
        temporary=path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(dict(schema_version='playground_prepared_plan.v1',plan=plan,
            execution_pending=execution_pending,execution_output=str(execution_output) if execution_output else None),
            ensure_ascii=False,allow_nan=False))
        temporary.replace(path)

    def health(self):
        if not self.bridge:
            return dict(connected=False, owner={}, cameras={})
        return self.bridge.status()

    def snapshot(self):
        health = self.health()
        detector = self.initialization()
        with self.lock:
            value = deepcopy(self.state)
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
        value['environment']['ready'] = bool(value['environment']['ready'] and
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
        value['capture_warning'] = None if value['capture_ready'] else '录制或相机未就绪；动作可独立执行，数据将标为未完成'
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
            for key in ('errors', 'recording', 'physical_outcome', 'task_outcome', 'native_task', 'simulation'):
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

    def start(self, payload):
        if payload.get('action')=='interrupt':return self.interrupt(payload)
        if set(payload) - {'action', 'speed_scale', 'arms_supported'}:
            raise ValueError('Playground 请求包含未知字段')
        action, speed = payload.get('action'), payload.get('speed_scale', .3)
        if action not in ('prepare', 'plan', 'execute'):
            raise ValueError('未知 Playground 操作')
        if isinstance(speed, bool) or not isinstance(speed, (int, float)) or not math.isfinite(speed) or not .1 <= speed <= 1:
            raise ValueError('运动速度必须在 10% 至 100% 之间')
        config = self.candidate()
        fresh = self.snapshot()
        with self.lock:
            if self.state['busy'] or self.external_busy():
                raise ValueError('已有运行正在使用工作台；请等待结束')
            if action == 'plan' and not fresh['environment']['ready']:
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
            self.state.update(status='running', busy=True, can_execute=False, error=None, action=action,
                              message={'prepare': '正在准备环境', 'plan': '正在重建场景与规划', 'execute': '正在执行与录制'}[action],
                              progress={'label': {'prepare': '准备环境', 'plan': '准备观测', 'execute': '校验执行输入'}[action],
                                        'elapsed_s': 0, 'stage_elapsed_s': 0, 'stage_status': 'running',
                                        'estimate_s': estimate, 'estimate_source': '上次成功耗时' if previous_time and action != 'execute'
                                        else '粗估基线（非完成承诺）', 'stages': [], 'next_label': ''})
            if action == 'plan':
                self.state.update(plan={'validated': False}, videos=[])
            elif action=='execute':
                for key in ('errors', 'recording', 'simulation', 'physical_outcome', 'task_outcome', 'native_task', 'execution_instance_id', 'execution_scope', 'execution_wait', 'simulation_scope'):
                    self.state.pop(key, None)
                self.state['plan']['validated'] = False  # A click consumes this plan, including failed/mid-run attempts.
                self.state['videos'] = []
            if action == 'prepare':
                self.state['environment'] = {'ready': False, 'checks': []}
            self.persist()
            self.thread = Thread(target=self.run, args=(action, config, output, plan, speed,
                                                        payload.get('arms_supported') is True), daemon=True)
            self.thread.start()
            return dict(status='accepted', action=action)

    def finish_interrupt(self):
        self.invalidate_checkpoint()
        recovery_error=None
        if self.recover_planner:
            try:self.recover_planner(progress=self.progress)
            except Exception as error:recovery_error=str(error)
        ready=not recovery_error and self.snapshot()['environment']['ready']
        with self.lock:
            if recovery_error:self.state['environment']['ready']=False
            self.state.update(status='failed' if recovery_error else 'ready' if ready else 'stopped',plan={'validated':False},
                              can_execute=False,error=recovery_error,
                              message='规划服务恢复失败：'+recovery_error if recovery_error else
                              '当前阶段已中断；环境准备保持就绪，可重新场景重建及规划。' if ready
                              else '当前阶段已中断；控制或检测环境尚未就绪，恢复后可重新规划。')

    def run(self, action, config, output, plan, speed, supported):
        try:
            import playground_pipeline as pipeline
            if action == 'prepare':
                if self.recover_planner and self.initialization().get('planner',{}).get('status') in ('failed','stopped'):
                    self.recover_planner(progress=self.progress)
                checks = self.call(pipeline.prepare_environment, config)
                if isinstance(checks, dict):
                    checks = checks['checks']
                if not checks or any(c.get('status') != 'ready' for c in checks):
                    raise ValueError('DSW 与仿真预检尚未全部就绪：' + json.dumps(checks, ensure_ascii=False))
                with self.lock:
                    self.state['environment']['checks'] = checks
                deadline = time.monotonic() + 240
                while self.initialization().get('status') != 'ready':
                    if self.cancel_event.is_set():raise InterruptedError('环境准备已中断')
                    state = self.initialization()
                    if state.get('status') in ('failed', 'disabled', 'stopped') or time.monotonic() > deadline:
                        raise ValueError('检测器未就绪：' + str(state.get('error') or state.get('message')))
                    self.progress(state.get('message') or '等待检测器加载', next_label='接入控制并确认三路相机')
                    time.sleep(.5)
                if not self.health().get('connected'):
                    if not supported and self.bridge.config.get('environment','physical')=='physical':
                        raise ValueError('控制接入缺少当前支撑确认')
                    self.progress('接入真机控制与三路相机', next_label='确认环境就绪')
                    self.bridge.control({'action': 'acquire', 'confirmation': 'ARMS_SUPPORTED_CONNECT'})
                deadline = time.monotonic() + 30
                while True:
                    if self.cancel_event.is_set():raise InterruptedError('环境准备已中断')
                    with self.lock:
                        self.state['environment']['ready'] = True
                    if self.snapshot()['environment']['ready']:
                        break
                    if time.monotonic() > deadline:
                        raise ValueError('控制或三路 RGB-D 接入尚未完成；请检查服务状态')
                    time.sleep(.5)
                restored=self.call(pipeline.restore_plan,config,self.directory,self.bridge)
                if restored is not None and (not isinstance(restored,dict) or restored.get('validated') is not True):
                    raise ValueError('恢复计划未通过校验')
                result = dict(status='ready',plan=restored or {**self.state['plan'],'validated':False},
                              message='环境准备完成，已恢复上次计划；请核对现场后点击 action 执行。' if restored
                              else '环境准备完成，可以重建场景与规划。')
            elif action == 'plan':
                plan = self.call(pipeline.plan, config, output, self.bridge, speed)
                if not isinstance(plan, dict) or plan.get('validated') is not True or not plan.get('timeline_digest'):
                    raise ValueError('规划未返回本轮路径验证证据')
                for key in ('source_path', 'plan_path', 'hardware_binding'):
                    self.artifact(plan.get(key), output)
                image = self.artifact(plan.get('first_frame_path'), output) if plan.get('first_frame_path') else None
                if image:
                    data = image.read_bytes()
                    if not ((data.startswith(b'\x89PNG\r\n\x1a\n') and data[12:16] == b'IHDR'
                             and len(data) >= 24 and int.from_bytes(data[16:20], 'big') > 0
                             and int.from_bytes(data[20:24], 'big') > 0 and b'IEND' in data[-16:])
                            or (data.startswith(b'\xff\xd8') and data.endswith(b'\xff\xd9'))):
                        raise ValueError('首帧不是有效的 PNG/JPEG 产物')
                plan.update(output=str(output), first_frame_url=(f'/api/playground/media/frame?v={output.name}' if image else None))
                self.checkpoint(plan)
                result = dict(status='ready', plan=plan, message='规划完成；请核对首帧与现场后点击 action 执行。')
            else:
                result = self.call(pipeline.execute, config, output, self.bridge, plan)
                if not isinstance(result, dict) or result.get('status') not in ('completed', 'failed', 'stopped'):
                    raise ValueError('执行未返回明确终态；禁止自动续跑')
                command_count=((result.get('recording') or {}).get('manifest') or {}).get('command_count')
                if type(command_count) is int and command_count==0:
                    plan['validated']=True
                    self.checkpoint(plan,execution_output=output)
                    result.update(plan=plan,message=(result.get('message') or '本轮未执行动作')+'；确认本轮未发送控制目标，计划已保留，请手动点击 action 重试。')
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
