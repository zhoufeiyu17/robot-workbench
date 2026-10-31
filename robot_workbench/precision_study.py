"""Explicit control-study buttons; all motion still uses the shared Owner."""
from copy import deepcopy
import hashlib
import gzip
import io
import json
from pathlib import Path
import secrets
import threading
import time
from .execution_settings import study_defaults,study_caps,study_presets,validate_study
from .precision_metrics import SPATIAL_CONTRACT

SPEEDS=(.5,1.,1.5,2.)
NUMERIC_MEMORY_LIMIT=64*1024**2


class _NumericBuffer(io.BytesIO):
    """Two append-only gzip streams share one strict compressed-byte budget."""
    def __init__(self,budget):
        super().__init__();self.budget=budget
    def write(self,data):
        if self.budget[0]+len(data)>NUMERIC_MEMORY_LIMIT:
            raise OSError('research numeric memory budget exceeded; evidence incomplete')
        count=super().write(data);self.budget[0]+=count;return count


def save(path,value):
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2)+'\n')
    temporary.replace(path)


class PrecisionStudy:
    def __init__(self,device,config):
        self.device=device;self.config=deepcopy(config);self.lock=threading.RLock();self.review=None
        if (config.get('schema')!='workbench.precision_study.v1' or config.get('controller_profile')!='x5_control50_damping_v1'
                or not config.get('references')
                or not set(config['references']) <= {str(v) for v in SPEEDS}):
            raise ValueError('declared research execution baseline and pinned supported-speed references required')
        for ref in config['references'].values():
            if (set(ref)!={'path','sha256','size_bytes'}or not Path(ref['path']).is_absolute()
                    or len(ref['sha256'])!=64 or type(ref['size_bytes'])is not int or not 0<ref['size_bytes']<=2*1024**2):
                raise ValueError('bounded immutable precision reference required')
        self.root=Path(config['root']);self.root.mkdir(parents=True,exist_ok=True)
        stored=json.loads((self.root/'settings.json').read_text())if(self.root/'settings.json').exists()else dict(revision=0,settings=study_defaults())
        self.settings=validate_study(stored['settings']);self.revision=stored['revision']
        self.history=json.loads((self.root/'index.json').read_text())if(self.root/'index.json').exists()else []
        self.selected=None
        self.latest=json.loads((self.root/'latest.json').read_text())if(self.root/'latest.json').exists()else None
        if self.latest and self.latest.get('status')in('submitted','running'):
            self.latest.update(status='result_unknown',reason='业务进程已变化；请核对当前Owner，禁止自动重试')
            save(self.root/'latest.json',self.latest)
        self.thread=None

    def _reason(self,state,business):
        if self.thread is not None and self.thread.is_alive():return '研究记录正在执行或收尾'
        if not state.get('connected')or not state.get('feedback_healthy'):return '先在服务运维接入设备并核对健康反馈'
        if state.get('holder')!=business:return '先交接到当前业务，再准备研究动作'
        if state.get('target_interpreter',{}).get('controller_profile')!=self.config['controller_profile']:return '当前执行基线与研究模块声明不匹配'
        speed=state.get('speed_protection')or{}
        if speed.get('enabled')is not True or not speed.get('monitored'):
            return '本次研究要求峰值保护开启且正在监测（默认0.8 rad/s）'
        if not state.get('idle_readiness',{}).get('allowed'):return state.get('idle_readiness',{}).get('reason')or '当前不满足空闲准入'
        return None

    @staticmethod
    def binding(state,business):
        return dict(runtime_nonce=state.get('runtime_nonce'),command_epoch=state.get('command_epoch'),business=business,
            settings_revision=(state.get('execution_settings')or{}).get('revision'),
            calibration_revision=state.get('calibration_revision'))

    def snapshot(self,state,business):
        with self.lock:
            reason=self._reason(state,business);review=self.review
            current=bool(review and review['binding']==self.binding(state,business)and time.monotonic()<review['deadline'])
            return dict(configured=True,profile=self.config['controller_profile'],default_speed_limit_rad_s=.8,
                settings=deepcopy(self.settings),settings_revision=self.revision,settings_caps=study_caps(),settings_presets=study_presets(),
                settings_allowed=not(self.thread and self.thread.is_alive())and not state.get('automatic_enabled'),
                history=deepcopy(self.history),selected=deepcopy(self.selected or self.latest),
                runtime_facts=dict(controller=deepcopy(state.get('target_interpreter')),effective_speed_protection=deepcopy(state.get('speed_protection')),
                    execution_error_settings=deepcopy(state.get('execution_settings')),
                    source_prefix_s=[0,50],trajectory_speed_choices=sorted(map(float,self.config['references']))),
                source_prefix_s=[0,50],recording='仅数值记录，无相机、视频或模型调用',prepare_allowed=reason is None,reason=reason,
                execute_allowed=reason is None and current,
                review={k:deepcopy(review[k])for k in ('id','kind','speed','duration_s','summary','feedforward_artifact')}if current else None,
                latest=deepcopy(self.latest),mechanical_stop_acceptance='待用户操作后依据新反馈验收')

    def update_settings(self,settings,revision):
        value=validate_study(settings)
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('研究仍在执行或收尾，不能改参')
            if type(revision)is not int or revision!=self.revision:raise ValueError('研究配置版本已变化，请重新读取')
            self.revision+=1;self.settings=value;self.review=None
            save(self.root/'settings.json',dict(revision=self.revision,settings=value))
        return dict(message='研究配置已保存；旧研究审批失效，未改电机增益，下一次新动作才应用。')

    def select(self,identity):
        if not isinstance(identity,str)or len(identity)!=24 or any(c not in'0123456789abcdef'for c in identity):raise ValueError('明确的研究记录身份必需')
        if not any(r['id']==identity for r in self.history)and(self.latest or{}).get('id')!=identity:raise ValueError('研究记录不在当前索引')
        with self.lock:self.selected=json.loads((self.root/identity/'result.json').read_text())
        return dict(message='已选择研究记录；未执行动作。')

    def notes(self,identity,text,revision):
        if not isinstance(text,str)or len(text.encode())>16*1024:raise ValueError('中间结论限16KiB文本')
        with self.lock:
            self.select(identity)
            row=self.selected
            if type(revision)is not int or revision!=row.get('notes_revision',0):raise ValueError('结论版本已变化，请重新读取')
            row.update(notes=text,notes_revision=revision+1,notes_wall_ns=time.time_ns());save(self.root/identity/'result.json',row)
            if(self.latest or{}).get('id')==identity:self.latest=deepcopy(row);save(self.root/'latest.json',self.latest)
        return dict(message='中间结论已保存到本次研究记录。')

    def prepare(self,business,kind,speed,feedforward_artifact=None):
        if (kind not in ('start','trajectory','home')or type(speed)not in(int,float)
                or str(float(speed)) not in self.config['references']):
            raise ValueError('研究仅接受起点、当前设备已登记倍率轨迹或回零')
        state=self.device.snapshot(business);reason=self._reason(state,business)
        if reason:raise ValueError(reason)
        binding=self.binding(state,business)
        with self.lock:settings=deepcopy(self.settings);settings_revision=self.revision
        if any(flow.snapshot().get('busy')for flow in self.device.workflows.values()):raise ValueError('其他业务任务仍在执行')
        artifact=self.config['references'][str(float(speed))]
        if kind=='trajectory':command=dict(action='preview_workbench',reference_artifact=artifact)
        else:
            from arx_env.joint_head_jaw_mapping import command_gap_mm,feedback_gaps,command_max_mm
            from .control_reference import load
            if kind=='start':
                reference=load(artifact,state['joint_limits']);q=reference.sample([0.])[0][0]
                q12=[*q[:6],*q[7:13]];gaps=[command_gap_mm(side,float(q[slot]))for side,slot in(('left',6),('right',13))]
            else:q12=[0.]*12;gaps=feedback_gaps(state['calibration_feedback']['qpos'])
            velocity=settings['alignment_speed_rad_s'if kind=='start'else'home_speed_rad_s']
            command=dict(action='preview_workbench',waypoints=[dict(q12=q12,speeds12=[velocity]*12,
                jaw_gap_mm=[min(command_max_mm(side),max(0.,float(g)))for side,g in zip(('left','right'),gaps)],jaw_speed_mm_s=settings['jaw_speed_mm_s'])])
        if feedforward_artifact is not None:
            if kind!='trajectory':
                raise ValueError('研究前馈只允许用于固定轨迹')
            from .control_plan import _read_file
            pin=feedforward_artifact
            if not isinstance(pin,dict) or set(pin)!={'path','sha256','size_bytes'}:
                raise ValueError('exact immutable feedforward artifact required')
            raw=_read_file(pin['path'],128*1024)
            if len(raw)!=pin['size_bytes'] or hashlib.sha256(raw).hexdigest()!=pin['sha256']:
                raise ValueError('feedforward artifact size or SHA256 differs')
            provenance=json.loads(raw).get('provenance',{})
            if provenance.get('study_settings')!=settings or provenance.get('speed_multiplier')!=speed:
                raise ValueError('研究前馈的增益、积分配置或速度与当前研究不一致')
            native_api=(state.get('target_interpreter')or{}).get('native_reference_api')or'legacy_recompute'
            if provenance.get('native_reference_api','legacy_recompute')!=native_api:
                raise ValueError('研究前馈的原生执行接口与当前设备不一致，需要重新资格')
            command['feedforward_artifact']=deepcopy(pin)
        command['study_settings']=settings
        preview=self.device.submit(business,command)
        current=self.device.snapshot(business)
        if binding!=self.binding(current,business):raise ValueError('准备期间控制身份或设置已变化；未执行')
        summary={'start':'低速回到前50秒素材起点','home':'低速双臂回零，保留夹爪目标','trajectory':f'前50秒固定轨迹 · {speed:g}×'}[kind]
        if feedforward_artifact is not None:summary+=' · 有界前馈'+('+积分'if settings['integral_enabled']else'（无残差积分）')
        with self.lock:
            if self.revision!=settings_revision:raise ValueError('研究配置在准备期间已变化；未执行')
            self.review=dict(id=secrets.token_hex(16),kind=kind,speed=speed,summary=summary,
                duration_s=preview['total_motion_duration_s'],deadline=time.monotonic()+min(30.,preview.get('expires_in_s',30.)),
                binding=binding,token=preview['token'],plan_sha256=preview['plans'][0]['sha256'],artifact=artifact,
                prepared_reference_id=preview.get('prepared_reference_id'),
                feedforward_artifact=deepcopy(feedforward_artifact),
                settings=settings,settings_revision=settings_revision)
        return dict(message=summary+'已准备；尚未下发运动，请点击执行当前审阅。')

    def execute(self,business,review_id):
        state=self.device.snapshot(business);reason=self._reason(state,business)
        if reason:raise ValueError(reason)
        import shutil
        if shutil.disk_usage(self.root).free<1024**3:raise ValueError('研究数值记录至少需要1GiB可用空间')
        with self.lock:
            review=self.review
            if (not review or review['id']!=review_id or review['binding']!=self.binding(state,business)
                    or review['settings_revision']!=self.revision or time.monotonic()>=review['deadline']):raise ValueError('研究审批已失效，请重新准备')
            self.review=None  # Consume before confirmation, including an unknown reply.
            directory=self.root/secrets.token_hex(12);directory.mkdir()
            self.latest=dict(id=directory.name,status='submitted',kind=review['kind'],speed=review['speed'],
                summary=review['summary'],duration_s=review['duration_s'],binding=review['binding'],
                plan_sha256=review['plan_sha256'],reference_artifact=review['artifact'],started_wall_ns=time.time_ns(),
                feedforward_artifact=deepcopy(review['feedforward_artifact']),
                prepared_reference_id=review.get('prepared_reference_id'),minimum_reference_start_s=state.get('control_time_s'),
                camera_calls=0,model_calls=0,automatic_home=False)
            self.latest.update(settings=deepcopy(review['settings']),settings_revision=review['settings_revision'],
                execution_settings=deepcopy(state.get('execution_settings')),controller_profile=self.config['controller_profile'],
                native_reference_api=(state.get('target_interpreter')or{}).get('native_reference_api'),
                scoring_model={k:deepcopy(self.config.get(k))for k in('urdf','urdf_sha256','zero_offsets_rad','tcp_m')},
                measurement_contract=SPATIAL_CONTRACT,
                source_path_geometry=deepcopy(self.config.get('source_path_geometry')),
                notes='',notes_revision=0)
            self.selected=None
            save(self.root/'latest.json',self.latest);save(directory/'result.json',self.latest)
            runtime=self.device.runtime
            channel=getattr(runtime,'channel',None)
            cursor=channel.call('events',expected_nonce=runtime.nonce,after=2**63-1)['latest']if channel else None
            try:
                self.device.submit(business,dict(action='confirm',token=review['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
            except Exception:
                self.latest.update(status='result_unknown',reason='提交结果未确认；未重试，请核对Owner')
                save(self.root/'latest.json',self.latest);save(directory/'result.json',self.latest)
                if self.device.runtime is runtime:self.device.request_stop()
                raise
            self.thread=threading.Thread(target=self._monitor,args=(runtime,business,review,directory,cursor),daemon=True,name='precision-numeric-recording')
            self.thread.start()
        return dict(message='本次研究动作已提交一次；不会自动回零、重试或清故障。')

    def _monitor(self,runtime,business,review,directory,cursor):
        last_sample=None;peak=0.;count=0;seen=False;ended=None;channel=getattr(runtime,'channel',None)
        streamed=channel is not None and review['kind']=='trajectory'
        next_state=0.;state=None;state_reads=0;event_reads=0
        budget=[0];buffers={name:_NumericBuffer(budget)for name in ('samples','events')}
        try:
            if streamed and (not review.get('prepared_reference_id')or self.latest['minimum_reference_start_s'] is None):
                raise ValueError('研究事件必须绑定本次准备参考与执行起始时间')
            # Drain Owner events without synchronous disk writes. Only completed
            # monitoring (or explicit failure/Stop handling) reaches the archive.
            with io.TextIOWrapper(gzip.GzipFile(fileobj=buffers['samples'],mode='wb',mtime=0),encoding='utf-8')as samples, \
                    io.TextIOWrapper(gzip.GzipFile(fileobj=buffers['events'],mode='wb',mtime=0),encoding='utf-8')as events:
                deadline=time.monotonic()+review['duration_s']+15.
                while time.monotonic()<deadline:
                    if self.device.runtime is not runtime:raise ValueError('控制实例已变化；结果未确认')
                    fresh_state=not streamed or time.monotonic()>=next_state
                    if fresh_state:
                        state=self.device.snapshot(business);next_state=time.monotonic()+.1;state_reads+=1
                        if state.get('runtime_nonce')!=review['binding']['runtime_nonce']:raise ValueError('Owner身份已变化')
                    if channel:
                        batch=channel.call('events',expected_nonce=runtime.nonce,after=cursor)
                        event_reads+=1
                        if batch['incomplete']:raise ValueError('Owner事件丢失，记录不完整')
                        for row in batch['events']:
                            if row['event']in('control_study_feedback','control_study_native_window','target_interpreter_bound','sdk_call_latency','owner_loop_trace','fault_diagnostic','fault','stop'):
                                events.write(json.dumps(row,allow_nan=False)+'\n')
                            if streamed and row['event']=='control_study_feedback':
                                value=row['value'];feedback=value['raw'];stamp=feedback.get('sample_time_s')
                                if (value.get('reference_id')==review['prepared_reference_id']
                                        and value.get('reference_started_s')is not None
                                        and value['reference_started_s']>=self.latest['minimum_reference_start_s']
                                        and feedback.get('sample_wall_ns',0)>=self.latest['started_wall_ns']
                                        and stamp is not None and stamp!=last_sample):
                                    last_sample=stamp;count+=1
                                    peak=max(peak,max(abs(feedback['qvel'][i])for i in(*range(6),*range(7,13))))
                                    samples.write(json.dumps(dict(feedback=feedback,reference_id=value['reference_id'],
                                        reference_started_s=value['reference_started_s'],source='owner_feedback_event'),allow_nan=False)+'\n')
                        cursor=batch['latest']if not batch['events']else batch['events'][-1]['sequence']
                    feedback=state.get('calibration_feedback')or{};stamp=feedback.get('sample_time_s')
                    trip=(state.get('speed_protection')or{}).get('last_trip')or{}
                    if (trip.get('command_epoch')==review['binding']['command_epoch']
                            and trip.get('sample_wall_ns',0)>=self.latest['started_wall_ns']):
                        peak=max(peak,trip.get('peak_rad_s',0.))
                    if not streamed and stamp is not None and stamp!=last_sample:
                        last_sample=stamp;count+=1
                        speed=max(abs(feedback['qvel'][i])for i in(*range(6),*range(7,13)));peak=max(peak,speed)
                        samples.write(json.dumps(dict(feedback=feedback,motion=state.get('motion'),speed_protection=state.get('speed_protection')),allow_nan=False)+'\n')
                    motion=state.get('motion')or{};seen=seen or motion.get('plan_sha256')==review['plan_sha256']
                    with self.lock:
                        self.latest.update(status='running'if seen else 'submitted',feedback_samples=count,feedback_peak_rad_s=peak)
                    fault='FAULT'in state.get('state','')or not state.get('connected')
                    done=seen and motion.get('status')in('READY','STOPPED','FAULT')
                    if fresh_state and (fault or done or seen and state.get('command_epoch')!=review['binding']['command_epoch']):
                        if ended is None:ended=time.monotonic()
                        # Retain post-stop feedback to distinguish SDK return from stationarity.
                        if time.monotonic()-ended>=1.:
                            status='fault'if fault else 'completed'if motion.get('status')=='READY'else 'stopped'
                            with self.lock:self.latest.update(status=status,reason=state.get('reason'),
                                final_state=state.get('state'),stationary=state.get('stationary'),speed_protection=state.get('speed_protection'))
                            break
                    time.sleep(.01)
                else:raise TimeoutError('执行完成未确认；已请求Stop，不重试')
        except Exception as error:
            with self.lock:self.latest.update(status='result_unknown',reason=str(error))
            # Includes alignment/home failures; main-trajectory submission is not a prerequisite.
            try:
                if self.device.runtime is runtime and runtime.nonce==review['binding']['runtime_nonce']:self.device.request_stop()
            except Exception as stop_error:
                with self.lock:self.latest['stop_error']=str(stop_error)
        finally:
            archive_error=None
            for name,buffer in buffers.items():
                try:(directory/(name+'.jsonl.gz')).write_bytes(buffer.getvalue())
                except OSError as error:archive_error=str(error)
                finally:buffer.close()
            with self.lock:
                self.latest['numeric_storage']=dict(mode='bounded_memory_then_archive',compressed_bytes=budget[0],
                    limit_bytes=NUMERIC_MEMORY_LIMIT,archive_error=archive_error)
                if archive_error:self.latest.update(status='result_unknown',reason='numeric archive failed: '+archive_error)
                self.latest.update(feedback_samples=count,feedback_peak_rad_s=peak,finished_wall_ns=time.time_ns())
                self.latest['monitor_reads']=dict(full_state=state_reads,events=event_reads,feedback_source='owner_events'if streamed else'state_snapshot')
                save(directory/'result.json',self.latest);save(self.root/'latest.json',self.latest)
                self.history=[{k:self.latest[k]for k in('id','status','summary','speed','settings_revision','finished_wall_ns')}]+[r for r in self.history if r['id']!=self.latest['id']]
                self.history=self.history[:20];save(self.root/'index.json',self.history)

    def analyze(self):
        with self.lock:
            if self.thread and self.thread.is_alive():raise ValueError('请等待数值记录收尾')
            selected=self.selected or self.latest
            if not selected:raise ValueError('尚无本模块的研究记录')
            directory=self.root/selected['id']
        from .recording_process import _environment
        import os,subprocess,sys
        request=dict(directory=str(directory),config=self.config)
        result=subprocess.run([os.environ.get('WORKBENCH_CPU_PYTHON')or sys.executable,'-B','-m',__name__,'--analyze'],
            input=json.dumps(request),capture_output=True,text=True,env=_environment(),timeout=90)
        if result.returncode:raise ValueError('数值分析失败：'+result.stderr[-700:])
        report=json.loads(result.stdout)
        with self.lock:
            current=json.loads((directory/'result.json').read_text());current['analysis']=report;save(directory/'result.json',current)
            if(self.selected or{}).get('id')==current['id']:self.selected=deepcopy(current)
            if(self.latest or{}).get('id')==current['id']:self.latest=deepcopy(current);save(self.root/'latest.json',self.latest)
        return dict(message='数值分析完成；故障或数据缺口不会作为精度通过。')


def analyze(directory,config):
    import numpy as np
    from .control_reference import decode
    from .motion_reference import JOINTS
    from .precision_metrics import spatial_metrics,vibration_metrics
    result=json.loads((directory/'result.json').read_text());rows=[];origin=None
    with gzip.open(directory/'events.jsonl.gz','rt')as stream:
        for line in stream:
            row=json.loads(line);value=row['value']
            if row['event']=='control_study_feedback':
                if result.get('prepared_reference_id') and (value.get('reference_id')!=result['prepared_reference_id']
                        or value.get('reference_started_s')is None
                        or value['reference_started_s']<result['minimum_reference_start_s']
                        or value['raw'].get('sample_wall_ns',0)<result['started_wall_ns']):continue
                rows.append(value)
    # The first verifying feedback may precede runner.started. Keep its real
    # time as pre-roll, but bind the trajectory to the first actual start.
    origin=next((r['reference_started_s']for r in rows if r.get('reference_started_s')is not None),None)
    report=dict(metric='同时间关节反馈FK三维TCP距离，无时间平移；非独立空间计量',passed=False,
        execution_status=result['status'],feedback_peak_rad_s=result['feedback_peak_rad_s'],samples=len(rows))
    report['requested_motion_gains']=result.get('settings',{}).get('gains')
    report['native_reference_api']=result.get('native_reference_api')
    native=next((r['raw']['controller_telemetry']for r in rows if(r['raw'].get('controller_telemetry')or{}).get('status')=='observed'),None)
    report['observed_motion_gains']=({side:{k:native['arms'][side][k]for k in('kp','kd')}for side in('left','right')}if native else None)
    if result['kind']!='trajectory'or origin is None:
        report['reason']='起点/回零或尚未进入固定轨迹；保留故障及峰值数据，不作全段精度验收';return report
    pin=result['reference_artifact'];raw=Path(pin['path']).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=pin['sha256']:raise ValueError('研究参考文件已变化')
    reference=decode(raw,np.tile([-10.,10.],(14,1)),verify=False)
    all_times=np.asarray([r['raw']['sample_time_s']-origin for r in rows]);valid=(all_times>=0)&(all_times<=reference.duration_s)
    all_actual=np.asarray([r['raw']['qpos']for r in rows])[:,JOINTS]
    times=all_times[valid];actual=all_actual[valid]
    if len(times)<2:report['reason']='有效轨迹样本不足；故障保留，不作精度通过';return report
    expected=np.vstack([reference.sample(times[i:i+32])[0][:,JOINTS]for i in range(0,len(times),32)])
    from arx_env.joint_head_isaac_fit import Model
    scoring=result.get('scoring_model')or config
    if scoring.get('urdf_sha256')and hashlib.sha256(Path(scoring['urdf']).read_bytes()).hexdigest()!=scoring['urdf_sha256']:raise ValueError('FK模型文件与本次试验快照不符')
    model=Model(Path(scoring['urdf']),[f'left_joint{i}'for i in range(1,7)]+[f'right_joint{i}'for i in range(11,17)])
    offset=np.asarray(scoring['zero_offsets_rad']);tcp=np.asarray(scoring['tcp_m']);errors=[]
    for a,b in zip(actual,expected):
        fa,fb=model.frames(a+offset),model.frames(b+offset);dist=[]
        for link in('left_link6','right_link16'):
            x,y=fa[link],fb[link];dist.append(float(np.linalg.norm(x[:3,:3]@tcp+x[:3,3]-y[:3,:3]@tcp-y[:3,3])*1000))
        errors.append(dist)
    error=np.asarray(errors);coverage=bool(len(times)>2 and times[0]<.03 and reference.duration_s-times[-1]<.03 and np.max(np.diff(times))<.03)
    report.update(coverage_passed=coverage,duration_s=reference.duration_s,
        joint_peak_deg=np.max(abs(np.degrees(actual-expected)),axis=0).tolist(),
        tcp_peak_mm=dict(left=float(np.max(error[:,0])),right=float(np.max(error[:,1]))),
        tolerance_mm=result.get('settings',{}).get('tcp_tolerance_mm',1.),
        passed=bool(result['status']=='completed'and coverage and np.max(error)<=result.get('settings',{}).get('tcp_tolerance_mm',1.)))
    all_expected=np.vstack([reference.sample(all_times[i:i+32])[0][:,JOINTS]for i in range(0,len(all_times),32)])
    report['smoothness']=vibration_metrics(all_times,all_actual-all_expected)
    contract=result.get('measurement_contract','same_time.v1')
    report.update(measurement_contract=contract,same_time_tcp_peak_mm=report['tcp_peak_mm'],
        same_time_precision_passed=report['passed'],smoothness_accepted=None)
    if contract==SPATIAL_CONTRACT:
        def paired_tcp(q):
            frames=model.frames(q+offset)
            return [frames[link][:3,:3]@tcp+frames[link][:3,3]for link in('left_link6','right_link16')]
        # Include terminal feedback: arriving late is permitted by spatial_path.v2.
        observed=np.asarray([paired_tcp(q)for q in all_actual[all_times>=0]])
        grid=reference.breaks
        prepared=np.vstack([reference.sample(grid[i:i+32])[0][:,JOINTS]for i in range(0,len(grid),32)])
        path=np.asarray([paired_tcp(q)for q in prepared])
        pin=result.get('source_path_geometry')
        if not pin:
            report.update(passed=False,precision_passed=False,
                reason='缺少本次冻结的原始空间路径制品；不以准备后路径替代原路径作通过判断')
            return report
        from .control_plan import _read_file
        raw=_read_file(pin['path'],2*1024**2)
        if len(raw)!=pin['size_bytes']or hashlib.sha256(raw).hexdigest()!=pin['sha256']:
            raise ValueError('原始空间路径制品已变化')
        source=json.loads(raw)
        if (source.get('schema')!='workbench.research_path_geometry.v1'
                or source.get('urdf_sha256')!=scoring.get('urdf_sha256')
                or source.get('zero_offsets_rad')!=scoring.get('zero_offsets_rad')
                or source.get('tcp_m')!=scoring.get('tcp_m')
                or source.get('source_range_s')!=[0.,50.]):
            raise ValueError('原始空间路径与本次评分模型/源区间不匹配')
        original=np.asarray(source['paired_tcp_m'],float)
        geometric=spatial_metrics(observed,original,report['tolerance_mm'])
        preparation=spatial_metrics(path,original,report['tolerance_mm'])
        sampling_covered=bool(all_times[0]<.03 and np.max(np.diff(all_times))<.03)
        report.update(metric=geometric['metric'],spatial_path=geometric,prepared_vs_original_path=preparation,
            precision_passed=bool(result['status']=='completed'and sampling_covered
                and geometric['sampled_path_checks_passed']and preparation['sampled_path_checks_passed']),
            acceptance_scope='sampled path precision only; smoothness and continuous path qualification separate')
        report['passed']=report['precision_passed']
    return report


if __name__=='__main__':
    import sys
    if sys.argv[1:]!=['--analyze']:raise SystemExit('CPU numeric analysis only')
    request=json.load(sys.stdin);print(json.dumps(analyze(Path(request['directory']),request['config']),ensure_ascii=False))
