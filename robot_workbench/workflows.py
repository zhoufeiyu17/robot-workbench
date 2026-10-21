"""Thin adapters around existing business workflows; no substitute solvers/planners."""
from contextlib import nullcontext
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import parse_qs, unquote, urlencode, urlsplit


CALIBRATION_STAGES=('head.plan','head.capture','head.solve','wrist.plan','wrist.capture','wrist.solve','install')
CALIBRATION_TOOLS=('configure_calibration','plan_batch','preview_batch','fit_source','fit_batch')
HISTORY_ACTIONS=('delete_preview','delete_confirm','delete_cancel')
COLLECTION_TOOLS=('collection_prepare','collection_plan')


def calibration_stage(state):
    report=state.get('pipeline') or {};review=report.get('review') or {};sub=report.get('subprogress') or {}
    if isinstance(report.get('installed'),dict) and report['installed'].get('verified') is True:
        return 'calibration.complete',7
    if review.get('kind') in ('head','wrist'):
        return review['kind']+'.review',1 if review['kind']=='head' else 4
    phase=report.get('phase') or report.get('failed_phase') or ''
    if phase.startswith('发布并安装'):return 'install',6
    for prefix,kind in (('头部','head'),('腕半场','wrist')):
        if phase.startswith(prefix):
            stage='solve' if '求解' in phase else 'capture' if '采集' in phase or '审阅' in phase else 'plan' if '规划' in phase else None
            if stage:return kind+'.'+stage,CALIBRATION_STAGES.index(kind+'.'+stage)
    steps=report.get('steps') or []
    if report.get('pairs') or report.get('installed'):return 'install',6
    kind=(state.get('workflow') or {}).get('kind','head')
    status=state.get('status')
    if kind in ('head','wrist'):
        stage={'PLANNING':'plan','CAPTURING':'capture','FITTING':'solve'}.get(status)
        if stage:return kind+'.'+stage,CALIBRATION_STAGES.index(kind+'.'+stage)
    index=sub.get('stage')
    if type(index) is int and 0<=index<7:return CALIBRATION_STAGES[index],index
    if any(row.get('half')=='head' and row.get('step') in ('solve','head') for row in steps):return 'wrist.plan',3
    if any(row.get('half')=='head' and row.get('step')=='capture' for row in steps):return 'head.solve',2
    if any(row.get('half')=='head' and row.get('step')=='plan' for row in steps):return 'head.capture',1
    return ('head.plan',0) if report.get('running') else ('calibration.idle',0)


def media_url(environment,business,result_id):
    return '/api/result-media?'+urlencode(dict(environment=environment,business=business,result_id=result_id))


def result_file(root,path):
    root=Path(root).resolve();path=Path(path)
    if path.is_symlink() or not path.resolve().is_relative_to(root) or not path.is_file() or path.stat().st_size==0:
        raise ValueError('回放文件不存在或超出当前工作台结果目录')
    return path.resolve()


def collection_live_view(value,runtime):
    """Join cached evidence only after the complete current execution binding matches."""
    binding=value.get('execution_binding') or {};native=runtime.get('native_task') or {}
    frozen=runtime.get('frozen') or {};timeline=runtime.get('execution_timeline') or {}
    accepted=runtime.get('execution_calibration') or {}
    def same(first,*others):return isinstance(first,str) and bool(first) and all(first==item for item in others)
    if not (value.get('stage_id')=='collection.execute' and value.get('status')=='running' and value.get('busy') is True and value.get('id')
            and runtime.get('holder')=='collection' and accepted.get('business')=='collection'
            and same(binding.get('device_id'),runtime.get('device_id'))
            and same(binding.get('environment'),runtime.get('environment'))
            and same(binding.get('execution_instance_id'),runtime.get('execution_instance_id'),accepted.get('execution_instance_id'),native.get('execution_instance_id'))
            and same(binding.get('runtime_nonce'),runtime.get('runtime_nonce'),native.get('runtime_nonce'))
            and same(binding.get('source_clock_id'),native.get('source_clock_id'),(runtime.get('calibration_feedback') or {}).get('source_clock_id'))
            and same(binding.get('timeline_digest'),frozen.get('timeline_digest'),timeline.get('parent_timeline_digest'))
            and same(binding.get('execution_timeline_digest'),frozen.get('execution_timeline_digest'),timeline.get('timeline_digest'),native.get('execution_timeline_digest'))
            and type(runtime.get('command_epoch')) is int and accepted.get('command_epoch')==runtime['command_epoch']):return value
    outcomes=value.setdefault('outcomes',{})
    def replace(name,placeholders,result):
        previous=outcomes.get(name);status=previous.get('status') if isinstance(previous,dict) else previous
        if result and (status is None or status in placeholders):outcomes[name]=result
    motion=({'FAULT':'failed','STOPPED':'stopped'}.get(runtime.get('motion_status')) or
        {'running':'running','awaiting_final_feedback':'verifying','completed':'completed','failed':'failed','cancelled':'stopped'}.get(frozen.get('status')))
    replace('motion',('未执行','unknown','not_started','running','executing','verifying'),
        dict(status=motion,message=frozen.get('reason') or frozen.get('stop_error')) if motion else None)
    recording=runtime.get('recording') or {};validation=(recording.get('manifest') or {}).get('validation') or {}
    error=runtime.get('recording_error') or recording.get('error') or recording.get('seal_error') or recording.get('diagnostics_error')
    data=('invalid' if error or validation.get('status')=='failed' else 'valid' if validation.get('status')=='passed' else
          'pending' if recording.get('done') else 'recording' if recording.get('ready') else None)
    replace('data',('未录制','unknown','not_started','pending','recording'),dict(status=data,message=error) if data else None)
    task=native.get('task_outcome')
    replace('task',('未验证','unverified','unknown','not_started','pending','running'),
        dict(status='failed',message=native['first_failure']) if native.get('first_failure') else
        dict(status='pending',message='等待原生任务回执核验') if task in ('success','pending_receipt') else
        dict(status=task,message=native.get('message')) if task else None)
    current,total=frozen.get('submitted_command_ticks'),frozen.get('execution_tick_count')
    if type(current) is int and type(total) is int and total==timeline.get('command_count') and 0<=current<=total and total>0:
        value['progress']=dict(current=current,total=total,scope='motion')
    collection=value.setdefault('collection',{})
    collection.update(live_binding_verified=True,recording=deepcopy(recording),native_task=deepcopy(native))
    return value


class CalibrationWorkflow:
    def environment_status(self):
        with self.lock:
            return {**{key:deepcopy(self.value.get(key)) for key in ('busy','status','stage_id')},
                'busy':bool(self.value.get('busy') or self.active and self.value.get('status') not in ('failed','stopped','completed')),
                'generation':self.generation}

    def __init__(self, app, device, service):
        self.app,self.device,self.service=app,device,service
        self.lock=threading.Lock()
        self.install_lock=threading.Lock()
        self.closed=threading.Event()
        self.active=False
        self.operation=None
        self.tool_catalog={}
        self.deletion=None
        self.preview_key=None
        self.preview_context=None;self.preview_failure=None;self.preview_pending=None;self.generation=0
        self.installed_id=None
        self.app.on_calibration_installed=self.calibration_installed
        self.catalog=[]
        self.replay=[];self.replay_context=None;self.media_files={}
        self.arrival_statistics={'error':'正在读取标定到位统计'}
        self.value=dict(status='connecting',busy=False,stage_id='connecting',title='标定',
                        message='正在读取标定工作流',primary_action=None,outcomes={})
        self.last_catalog=0.
        self.thread=threading.Thread(target=self.monitor,daemon=True,name='calibration-view')
        self.thread.start()

    def monitor(self):
        while not self.closed.is_set():
            try:self.refresh()
            except Exception as error:
                with self.lock:self.value.update(message=str(error),view_error=str(error))
            self.closed.wait(.25)

    def refresh(self):
        state=self.app.runtime_state()
        report=state.get('pipeline') or {}
        live_run=getattr(self.app,'pipeline_run_id',None)
        report_run=report.get('run_id')
        if live_run and report_run and live_run!=report_run:
            # A new job may be queued before it replaces the prior report. No
            # previous failure, successful install or fraction belongs to it.
            report={'run_id':live_run,'running':True}
            state={**state,'pipeline':report,'status':'PIPELINE','reason':None}
        run_id=live_run or report.get('run_id')
        review=report.get('review')
        key=(run_id,json.dumps(review,sort_keys=True))
        with self.lock:
            if getattr(self,'preview_context',None)!=key:
                self.generation=getattr(self,'generation',0)+1;self.preview_context=key
                self.preview_key=None;self.preview_failure=None;self.preview_pending=None
        if self.active and review:self.publish_workflow_review(key,review)
        sub=report.get('subprogress') or {}
        running=bool(report.get('running'))
        busy=running or state.get('status') in ('PLANNING','CAPTURING','FITTING','RUNNING')
        stage_id,completed=calibration_stage(state)
        status='stopped' if report.get('stopped') else 'failed' if report.get('aborted') or state.get('status')=='REJECTED' else 'running' if running else 'completed' if completed==7 else 'idle'
        working=running and not review and status=='running' and completed<7
        capture_batch=None;completion=None
        with self.app.lock if hasattr(self.app,'lock') else self.lock:
            active=deepcopy(getattr(self.app,'active_capture',None))
            if (run_id and isinstance(active,dict) and active.get('pipeline_run_id')==run_id
                    and stage_id.startswith(str(active.get('kind'))+'.')
                    and isinstance(active.get('batch_id'),str)):
                capture_batch=active['batch_id']
                if state.get('selected_batch')==capture_batch==getattr(self.app,'selected_batch',None):
                    completion=deepcopy((getattr(self.app,'batch',None) or {}).get('completion'))
        if stage_id.endswith('.capture') and capture_batch and (working or status=='failed'):
            current=completion.get('accepted',completion.get('captured')) if isinstance(completion,dict) else active.get('captured')
            total=active.get('expected_count')
            if type(current) is int and type(total) is int and 0<=current<=total and total>0:
                sub=dict(stage=completed,fraction=current/total,completed=current,total=total,label='已确认采集图像')
        measured=((working or status=='failed') and sub.get('stage')==completed and type(sub.get('fraction')) in (int,float)
                  and math.isfinite(sub['fraction']))
        fraction=max(0.,min(.99,sub['fraction'])) if measured else 0.
        progress=dict(indeterminate=True,unit='当前阶段') if working and not measured else None
        if measured:
            counted=all(type(sub.get(key)) in (int,float) and math.isfinite(sub[key]) for key in ('completed','total'))
            progress=(dict(current=sub['completed'],total=sub['total'],unit=sub.get('label',''))
                if counted and 0<=sub['completed']<=sub['total'] and sub['total']>0 else
                dict(current=fraction,total=1,unit=sub.get('label','阶段工作量'),kind='fraction'))
        primary=dict(id='generate_plan',label='生成完整标定计划',allowed=not busy,reason='标定任务正在进行' if busy else None)
        if review:primary=dict(id='generate_plan',label='重新生成本批执行预览',allowed=True,reason=None)
        message=('等待审阅本批姿态并确认执行。' if review else
            '本轮已停止；已完成的采集与结果保留，重新执行前需要重新审阅。' if status=='stopped' else
            '先生成计划，再审阅本批并执行。' if status=='idle' and not busy else
            '本轮标定未通过；请查看本阶段的失败原因。' if status=='failed' else
            {'head.plan':'正在生成头部采集计划。','head.capture':'正在逐点采集头部图像。',
             'head.solve':'正在使用已采集图像求解并验收头部标定。','wrist.plan':'正在生成双腕采集计划。',
             'wrist.capture':'正在逐点采集双腕图像。','wrist.solve':'正在使用贴纸观测求解并验收双腕标定。',
             'install':'正在校验并安装本轮头部与双腕结果。',
             'calibration.complete':'头部与双腕标定已完成，安装组已核验。'}.get(stage_id,'先生成计划，再审阅本批并执行。'))
        value=dict(id=run_id,status=status,busy=busy,
            stage_id=stage_id,title=report.get('phase') or report.get('failed_phase') or '头部与双腕标定',
            message=report.get('aborted') or state.get('reason') or message,
            progress=progress,primary_action=primary,outcomes={},capture_batch_id=capture_batch,
            failed_stage=stage_id if status=='failed' else None,
            overall_progress=dict(current=completed+fraction,total=7,unit='阶段',completed_stages=completed,
                indeterminate=working and not measured,stage_ids=list(CALIBRATION_STAGES)),
            details=[{'label':'当前模型','value':self.app.config.get('asset_geometry_id','未知')},
                     {'label':'运行身份','value':run_id or '尚未生成'}])
        if capture_batch:value['details'].append({'label':'采集批次','value':capture_batch})
        value['pipeline']={key:deepcopy(report[key]) for key in
            ('run_id','phase','failed_phase','aborted','stopped','steps','subprogress','head_capture','wrist_capture') if key in report}
        value['quality']=dict(counts=deepcopy(state.get('counts') or {}),completion=deepcopy(completion),
            results=[{key:deepcopy(row[key]) for key in ('half','step','round','result_id','metrics','pair_status','acceptance','reused','adopted') if key in row}
                     for row in report.get('steps',[]) if row.get('metrics') is not None],
            **{key:deepcopy(report[key]) for key in ('pairs','best_pair','installed') if key in report})
        if status=='failed' and isinstance(completion,dict):
            value['outcomes']['data']='valid' if completion.get('passed') is True else 'invalid'
        value['manual_queue']=self.manual_queue_state()
        if time.monotonic()-self.last_catalog>5:
            groups={row['id']:row for row in self.app.calibration_groups()}
            rows=[dict(row,kind='head') for row in self.app.calibration_results()]+[dict(row,kind='wrist') for row in self.app.wrist_results()]
            catalog=[]
            for row in rows:
                item=dict(id=row['id'],label=row.get('version') or row['id'][:12],kind=row['kind'],
                    status=row.get('pair_status') or row.get('status','candidate'))
                item['details']={key:deepcopy(row[key]) for key in ('version','environment','robot_source','simulation_only',
                    'batch','file','objective','acceptance','optimizer','metrics_rectified_px','mesh_metrics','left','right','capture_coverage') if key in row}
                source=row.get('batch') if row['kind']=='head' else str(Path(row.get('file','')).parent)
                item['raw_deleted']=bool(source and hasattr(self.app,'deleted_batch') and self.app.deleted_batch(source))
                if row.get('version'):
                    item['download_url']='/api/calibration-file?'+urlencode(dict(environment=self.device.environment,result_id=row['id']))
                if row['kind']=='head':
                    group=groups.get(row['id']) or {}
                    item['details']['group']=deepcopy(group)
                    item['install']=dict(allowed=bool(group.get('installable')),reason=group.get('reason') or '尚无通过安装门槛的匹配双腕',
                        head_result_id=row['id'],wrist_result_id=(group.get('wrist') or {}).get('id'),installed=bool(group.get('installed')))
                elif hasattr(self.app,'wrist_head_id'):
                    item['details']['head_result_id']=self.app.wrist_head_id(row)
                catalog.append(item)
            history=self.app.history()
            catalog += [dict(id='capture:'+row['id'],label=row['id'],kind='capture',status=row.get('status','unknown'),details=deepcopy(row)) for row in history]
            installed=self.app.installed_pair()
            self.arrival_statistics=self.app.arrival_state() if hasattr(self.app,'arrival_state') else {'error':'标定到位统计入口不可用'}
            identity=self.install_identity(installed)
            if self.installed_id is None:
                with self.install_lock:
                    if self.installed_id is None:self.installed_id=identity
            elif identity!=self.installed_id and self.device.runtime:
                self.calibration_installed(installed)
            heads=[dict(id=row['id'],label=row.get('version') or row['id'][:12]) for row in rows if row['kind']=='head'
                and (groups.get(row['id'],{}).get('head') or {}).get('eligible') is True]
            with self.lock:
                self.catalog=catalog
                self.tool_catalog=dict(heads=heads,batches=deepcopy(history),installed=deepcopy(installed))
            self.last_catalog=time.monotonic()
        operation=getattr(self,'operation',None)
        if operation and not running:
            phase=operation['action'];kind=operation['kind']
            status='failed' if state.get('status')=='REJECTED' else 'stopped' if state.get('status')=='STOPPED' else 'running' if busy else 'awaiting_review' if phase=='review' else 'completed'
            titles={'plan':'独立标定批次规划','review':'独立标定批次待审阅','capture':'独立标定批次采集','fit':'指定批次重新求解'}
            value.update(id=operation['id'],status=status,stage_id=f'{kind}.{phase}',title=titles[phase],
                message=state.get('reason') or (operation.get('message') if busy or phase=='review' else '本次操作已结束；采集完整性与标定质量请分别查看批次及结果详情。'),
                overall_progress=None,progress={'indeterminate':True} if busy else None,
                capture_batch_id=operation['id'] if phase=='capture' else None,
                failed_stage=f'{kind}.{phase}' if status=='failed' else None,
                outcomes={'task':status},details=[{'label':'求解来源','value':state.get('selected_batch') or '尚未选择'}])
            if phase in ('plan','review'):
                queue_rows=value['manual_queue'].get('waypoints')
                value['primary_action']=dict(id='preview_batch',label='生成本批采集预览',allowed=not busy and bool(queue_rows),
                    reason='当前标定任务正在进行' if busy else None if queue_rows else '请先生成标定采样计划')
            elif phase=='capture':
                completion=(getattr(self.app,'batch',None) or {}).get('completion') if state.get('selected_batch')==operation['id'] else None
                value['outcomes']['data']='valid' if completion and completion.get('passed') is True else 'invalid' if completion and completion.get('passed') is False else 'pending'
                value['primary_action']=dict(id='fit_batch',label='求解本批采集数据',allowed=not busy and bool(completion and completion.get('passed')),
                    reason='采集尚未完整结束' if busy or not completion or not completion.get('passed') else None)
        queue=value['manual_queue']
        with self.app.lock if hasattr(self.app,'lock') else self.lock:
            plan=deepcopy(getattr(self.app,'plan',None))
        value['calibration_tools']=dict(available=True,workflow=state.get('workflow') or {'kind':'head','head_result_id':None},
            fit_source_id=state.get('selected_batch'),counts=state.get('counts') or {},plan=plan,
            queue_revision=queue.get('revision'),waypoints=queue.get('waypoints',[]),
            preview_message='下方按执行顺序列出冻结的采样点与过渡点；查看图像只预览，不执行动作。',
            **deepcopy(getattr(self,'tool_catalog',{})))
        with self.lock:
            if self.preview_context!=key:return
            value.update(replay=deepcopy(self.replay),replay_context=deepcopy(self.replay_context))
            value['arrival_statistics']=deepcopy(getattr(self,'arrival_statistics',{'error':'标定到位统计尚未读取'}))
            self.value=value

    def snapshot(self):
        with self.lock:
            value=deepcopy(self.value)
            failure=deepcopy(getattr(self,'preview_failure',None)) if getattr(self,'active',False) else None
        motion_preflight=getattr(self.app,'motion_preflight',None)
        if motion_preflight:
            value['manual_preflight']={name:deepcopy(motion_preflight[name]) for name in
                ('request_id','stage','status','started_at_s','elapsed_s','phase_timings','error') if name in motion_preflight}
            if motion_preflight.get('status')=='checking':
                value['manual_preflight']['elapsed_s']=max(0.,time.time()-motion_preflight['started_at_s'])
        preflight=getattr(self.app,'control_preflight_failure',None)
        if preflight and preflight.get('run_id')==getattr(self.app,'pipeline_run_id',None):
            value.setdefault('diagnostics',[]).append(dict(label='最近一次控制准备拒绝',value=deepcopy(preflight)))
        if failure:
            value.update(status='failed',failed_stage=failure['stage_id'],view_error=failure['error'],review_error=failure,
                message='本批执行预览生成失败：'+failure['error']+'；请点击“重新生成本批执行预览”。',
                primary_action=dict(id='generate_plan',label='重新生成本批执行预览',allowed=True,reason=None),
                diagnostics=[*value.get('diagnostics',[]),dict(label='审阅预览发布失败',value=f"运行 {failure['run_id']} · {failure['stage_id']} · {failure['error']}")])
        value['deletion_preview']=self.deletion_preview()
        value['deletion_result']=deepcopy(getattr(self,'deletion_result',None))
        return value

    def publish_workflow_review(self,key,review,*,explicit=False):
        with self.lock:
            if not self.active or self.preview_context!=key:return
            if self.preview_pending is not None:
                if explicit:raise ValueError('本批执行预览仍在生成，请等待当前请求结束')
                return
            if not explicit and (self.preview_key==key or self.preview_failure):return
            self.generation+=1;attempt=(key,self.generation);self.preview_pending=attempt;self.preview_key=None
            revision=self.manual_revision()
        try:
            preview=self.app.preview_calibration()
            summary=f"{review.get('label','本批')}：{preview.get('samples',0)} 次采样，纯运动 {preview.get('total_motion_duration_s',0):.1f} 秒；请审阅全部姿态"
            with getattr(self.app,'review_lock',nullcontext()),self.lock:
                current=(getattr(self.app,'pipeline_run_id',None),json.dumps(getattr(self.app,'pipeline_review',None),sort_keys=True))
                if (not self.active or self.preview_context!=key or self.preview_pending!=attempt
                        or current!=key or self.manual_revision()!=revision):
                    raise ValueError('本批审阅已撤销或运行已变化，请重新生成预览')
                self.service.add_review(self.device,'calibration',dict(preview,workflow_generation=attempt[1]),'execute',summary)
                self.preview_key=key;self.preview_failure=None;self.preview_pending=None
            return dict(message=summary,review_published=True)
        except Exception as error:
            with self.lock:
                if self.active and self.preview_context==key and self.preview_pending==attempt:
                    self.preview_pending=None
                    current=(getattr(self.app,'pipeline_run_id',None),json.dumps(getattr(self.app,'pipeline_review',None),sort_keys=True))
                    if current==key:
                        self.preview_failure=dict(run_id=key[0],stage_id=review.get('kind','calibration')+'.review',
                            error=type(error).__name__+': '+str(error),wall_ns=time.time_ns())
            if explicit:raise

    def deletion_preview(self):
        with self.lock:pending=deepcopy(getattr(self,'deletion',None))
        if not pending:return None
        value=pending['view'];record=getattr(self.app,'delete_tokens',{}).get(value['id'])
        value['valid']=bool(record and record[2]>time.monotonic())
        value['reason']=None if value['valid'] else '删除预览已过期或被替换，请重新预览范围'
        return value

    def delete_history(self,action,body):
        if action=='delete_cancel':
            with self.lock:self.deletion=None
            return {'message':'已取消本次删除确认，原始批次未删除'}
        if action=='delete_preview':
            with self.lock:self.deletion=None;self.deletion_result=None
            scope=body.get('scope','batch')
            if scope=='all':request={'scope':'all'}
            elif scope=='batch':
                identity=body.get('result_id')
                if not isinstance(identity,str) or not identity.startswith('capture:'):raise ValueError('请选择结果列表中的原始采集批次')
                request={'id':identity[8:]}
            else:raise ValueError('删除范围必须为单批或全部原始批次')
            preview=self.app.action('/api/history-delete-preview',request)
            images=sum(Path(name).suffix.lower() in ('.png','.jpg','.jpeg','.webp','.tif','.tiff','.bmp')
                for batch in preview['batches'] for name,_,_ in self.app.deletion_inventory(batch)[1])
            view={key:deepcopy(value) for key,value in preview.items() if key not in ('id','token')}
            view.update(id=preview['token'],root=str(Path(self.app.output).resolve()),image_files=images,
                detail='仅删除上列本地原始批次；其他目录不在本次删除范围内。',
                preserved_installed=self.app.installed_pair(),
                preserved=['已登记标定版本及依赖快照','当前 active 安装组','关节到位统计日志与汇总','上列删除范围以外的目录'])
            with self.lock:self.deletion=dict(request=request,view=view)
            return {'message':'删除范围已核验，请检查结果页的具体预览后明确确认'}
        with self.lock:pending=deepcopy(getattr(self,'deletion',None))
        if not pending or body.get('delete_preview_id')!=pending['view']['id'] or body.get('confirmed') is not True:
            raise ValueError('必须明确确认当前页面展示的删除范围')
        try:
            result=self.app.action('/api/history-delete',{**pending['request'],'token':pending['view']['id']})
        finally:
            with self.lock:self.deletion=None
        removed=set(result.get('removed',[]))
        with self.lock:
            self.deletion_result=deepcopy(result)
            self.catalog=[row for row in self.catalog if row['id'] not in {'capture:'+batch for batch in removed}]
            if (self.replay_context or {}).get('batch_id') in removed:
                self.replay=[];self.replay_context=None;self.media_files={}
                self.value.update(replay=[],replay_context=None)
        self.last_catalog=0.
        return result

    @staticmethod
    def install_identity(snapshot):
        raw=json.dumps(snapshot,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()
        return 'installed-calibration-'+hashlib.sha256(raw).hexdigest()

    def calibration_installed(self,snapshot):
        identity=self.install_identity(snapshot)
        with self.install_lock:
            if self.installed_id==identity:return
            if self.install_identity(self.app.installed_pair())!=identity:
                raise ValueError('安装组在通知前已变化，请重新核验当前安装身份')
            if self.device.runtime:self.device.runtime.notify_calibration_installed(identity)
            self.service.invalidate(self.device.environment)
            self.installed_id=identity
            self.last_catalog=0.

    def results(self):
        with self.lock:return deepcopy(self.catalog)

    def download(self,identity):
        if not any(row['id']==identity and row['kind'] in ('head','wrist') for row in self.results()):
            raise ValueError('所选标定版本不存在')
        return self.app.calibration_store_record(identity)

    def manual_revision(self):
        return getattr(self.app,'review_generation',0)

    def manual_queue_state(self):
        if not hasattr(self.app,'queue') or not hasattr(self.app,'lock'):
            return dict(available=False,reason='尚未配置标定动作队列')
        with self.app.lock:
            return dict(available=True,revision=self.manual_revision(),waypoints=deepcopy(self.app.queue))

    def manual_queue(self,body):
        operation=body.get('operation')
        if type(body.get('queue_revision')) is not int or body['queue_revision']!=self.manual_revision():
            raise ValueError('人工队列已变化，请刷新后重新操作')
        if operation=='add':
            waypoint=body.get('waypoint')
            if not isinstance(waypoint,dict):raise ValueError('人工路径点必需')
            result=self.app.action('/api/queue',waypoint)
        elif operation in ('remove','clear'):
            with self.app.mode_lock:
                if body['queue_revision']!=self.manual_revision():raise ValueError('人工队列已变化，请刷新后重新操作')
                result=self.app.action('/api/queue-'+operation,{'index':body.get('index')} if operation=='remove' else {})
        else:raise ValueError('未知人工队列操作')
        queue=self.manual_queue_state()
        with self.lock:self.value['manual_queue']=queue
        return result

    def manual_preview(self,body,*,queue=False):
        from arx_env.joint_head_real_motion import DEFAULT_VELOCITY
        if body.get('active_arm') is not None:
            from arx_env.joint_head_real_motion import ARM_SLICES
            side=body['active_arm']
            if queue or side not in ARM_SLICES or body.get('jaw_gap_mm') is not None:
                raise ValueError('single-arm preview requires one arm, no queue or jaw target')
            q=self.app.geometry._q(body.get('q12')).tolist()
            speeds=body.get('speeds12',[DEFAULT_VELOCITY]*12)
            a=0 if side=='left' else 6
            if (not isinstance(speeds,list) or len(speeds)!=12
                    or any(type(v) not in (int,float) or not math.isfinite(v) or not 0<v<=.5 for v in speeds)
                    or len(set(speeds[a:a+6]))!=1):
                raise ValueError('single-arm preview requires one shared arm speed in (0, 0.5]')
            return self.app.control_request('motion-preview',dict(active_arm=side,q_active=q[a:a+6],max_velocity=speeds[a]))
        if queue:
            current=self.manual_queue_state()
            if not current.get('available') or body.get('queue_revision')!=current['revision']:
                raise ValueError('人工队列已变化或不可用，请刷新')
            rows=current['waypoints']
        else:
            rows=[dict(q=body.get('q12'),speeds=body.get('speeds12',[DEFAULT_VELOCITY]*12),
                       jaw_gap_mm=body.get('jaw_gap_mm'),jaw_speed_mm_s=body.get('jaw_speed_mm_s',10.))]
        # Reuse C path/seed policy and the shared Owner compiler; do not send each
        # waypoint separately or hold the page lock during geometry preflight.
        return self.app.action('/api/hardware-trajectory-preview',{'waypoints':rows})

    def mesh(self):
        remote=self.app.remote or {}
        return dict(requested=self.app.mesh_rendering_enabled,
                    applied=remote.get('mesh_rendering_enabled') is True)

    def allowed(self, action):
        state=self.snapshot()
        if action in HISTORY_ACTIONS:
            if action=='delete_cancel':return dict(allowed=bool(state.get('deletion_preview')),reason=None)
            busy=state.get('busy') or getattr(self.app,'pipeline_running',False) or getattr(self.app,'importing',False) or getattr(self.app,'pending_capture',None) or getattr(self.app,'status',None) in ('RUNNING','PLANNING','CAPTURING','FITTING') or (getattr(self.app,'remote',None) or {}).get('status') in ('EXECUTING','VALIDATING','RUNNING','SETTLING','AWAITING_CAPTURE')
            if busy:return dict(allowed=False,reason='采集、导入或求解任务进行中，禁止删除原始批次')
            preview=state.get('deletion_preview')
            if action=='delete_confirm' and not (preview and preview.get('valid')):
                return dict(allowed=False,reason=(preview or {}).get('reason') or '请先预览具体删除范围')
            return dict(allowed=True,reason=None)
        if action=='preview_batch' and not getattr(self.app,'queue',[]):return dict(allowed=False,reason='请先生成标定采样计划')
        if action=='fit_batch' and not getattr(self.app,'selected_batch',None):return dict(allowed=False,reason='请先明确设置求解来源')
        if action=='install' and (not self.device.runtime or self.device.runtime.holder!='calibration'):
            return dict(allowed=False,reason='安装前请接入设备并显式交接给标定业务')
        yes=(action in ('mesh','select_result') or action=='cancel' and state.get('busy')
             or action in CALIBRATION_TOOLS and not state.get('busy')
             or action=='generate_plan' and (not state.get('busy') or bool(getattr(self.app,'pipeline_review',None)))
             or action=='install' and not state.get('busy')
             or action=='execute' and bool(getattr(self.app,'pipeline_review',None) or getattr(self.app,'calibration_previews',None)))
        return dict(allowed=yes,reason=None if yes else '当前标定阶段不允许此操作')

    def media(self,result_id):
        with self.lock:path=self.media_files.get(result_id)
        return result_file(self.app.output,path) if path else None

    def select_result(self,body):
        identity=body.get('result_id')
        with self.lock:previous_context=deepcopy(self.replay_context or {})
        if previous_context.get('result_id')!=identity:previous_context={}
        heads={row['id']:row for row in self.app.calibration_results()}
        wrists={row['id']:row for row in self.app.wrist_results()}
        if identity in heads:kind,row='head',heads[identity];batch=row['batch']
        elif identity in wrists:kind,row='wrist',wrists[identity];batch=str(Path(row['file']).parent)
        elif isinstance(identity,str) and identity.startswith('capture:') and identity[8:] in {r['id'] for r in self.app.history()}:
            kind,row,batch='capture',{},identity[8:]
        else:raise ValueError('所选标定结果或采集批次不存在')
        raw_deleted=hasattr(self.app,'deleted_batch') and self.app.deleted_batch(batch)
        view={'frames':[]} if raw_deleted else self.app.review_batch(batch)  # Replay never changes solver input.
        frames=view.get('frames') or [];index=body.get('frame_index',previous_context.get('frame_index',0))
        if type(index) is not int or index<0 or frames and index>=len(frames):raise ValueError('回放帧索引无效')
        image_urls=frames[index].get('image_urls',{}) if frames else {}
        camera=body.get('camera',previous_context.get('camera','left' if kind=='wrist' and 'left' in image_urls else 'head'))
        if camera not in ('head','left','right'):raise ValueError('请选择头部或左右腕相机')
        overlay=body.get('overlay',previous_context.get('overlay',False))
        if type(overlay) is not bool:raise ValueError('overlay 必须是布尔值')
        # review_batch owns the same-batch/dependency boundary. Quality controls
        # installation, while historical overlay keeps its existing parameter checks.
        review_results=view.get('results')
        if review_results is None:
            review_results=[dict(r,kind='head') for r in heads.values() if r.get('batch')==batch]
            review_results += [dict(r,kind='wrist') for r in wrists.values() if str(Path(r.get('file','')).parent)==batch]
        candidates=[dict(id=r['id'],kind=r.get('kind','wrist'),label=r.get('version') or r['id'],
            status=r.get('pair_status') or r.get('status','candidate')) for r in review_results
            if (r.get('kind')=='head')==(camera=='head')]
        candidate_ids={r['id'] for r in candidates}
        overlay_id=body.get('candidate_id')
        if overlay_id is not None and (not isinstance(overlay_id,str) or overlay_id not in candidate_ids):
            raise ValueError('叠加候选必须匹配当前批次与相机')
        if overlay_id is None and not (kind=='wrist' and camera=='head') and previous_context.get('candidate_id') in candidate_ids:
            overlay_id=previous_context['candidate_id']
        if overlay_id is None:
            overlay_id=identity if identity in candidate_ids else None
            if kind=='wrist' and camera=='head':
                dependency=self.app.wrist_head_id(row)
                overlay_id=dependency if dependency in candidate_ids else None
            elif overlay_id is None and kind=='capture' and candidates:overlay_id=candidates[0]['id']
        overlay_available=bool(frames) and not raw_deleted and camera in image_urls and overlay_id is not None
        if overlay:
            if not overlay_available:raise ValueError('当前相机需要匹配的头部或双腕结果')
            image=self.app.historical_overlay(dict(batch_id=batch,frame_index=index,camera=camera,result_id=overlay_id))['image_url']
        else:image=image_urls.get(camera)
        replay=[];files={}
        if image:
            url=urlsplit(image)
            if url.scheme or url.netloc or not url.path.startswith('/artifacts/'):raise ValueError('回放图像必须来自当前工作台产物')
            path=result_file(self.app.output,Path(self.app.output)/unquote(url.path.removeprefix('/artifacts/')))
            media_id=hashlib.sha256(str(path.relative_to(Path(self.app.output).resolve())).encode()).hexdigest()
            files[media_id]=path
            replay=[dict(id=media_id,label=f'{camera} · 第 {index+1} 帧'+(' · mesh' if overlay else ' · 原图'),
                kind='replay',url=media_url(self.device.environment,'calibration',media_id),ready=True)]
        # Selection is useful for replay even when the optional live renderer is absent.
        message='已打开只读回放；未配置实时 mesh 渲染器'
        previous=(self.replay_context or {}).get('result_id')
        if self.snapshot().get('busy') or getattr(self.app,'pipeline_running',False):message='当前任务仍在进行；已打开只读回放，未变更实时叠加选择'
        elif previous==identity or 'candidate_id' in body:message='已更新只读回放'
        elif kind=='wrist' or kind=='head' and getattr(self.app,'live',None):
            reply=self.app.action('/api/wrist-select' if kind=='wrist' else '/api/calibration',{'result_id':identity})
            message=reply.get('message','已选择标定结果')
        context=dict(result_id=identity,batch_id=batch,frame_index=index,frame_count=len(frames),camera=camera,
            cameras=sorted(image_urls),overlay=overlay,overlay_available=overlay_available,
            candidate_id=overlay_id,candidates=candidates,overlay_result_id=overlay_id if overlay_available else None,
            frame={key:deepcopy(frames[index][key]) for key in
                ('id','name','image','split','valid','valid_for_fit','detections','detections_by_camera') if key in frames[index]} if frames else None,
            frames=[{key:deepcopy(frame[key]) for key in ('id','name','image','split','valid','valid_for_fit') if key in frame} for frame in frames],
            message='原始批次已删除；版本参数与依赖快照保留，无法回放原始图像' if raw_deleted else '本帧没有所选相机图像' if frames and not image else '尚无采集帧' if not frames else '')
        with self.lock:
            self.media_files=files;self.replay=replay;self.replay_context=context
            self.value.update(replay=deepcopy(replay),replay_context=deepcopy(context))
        return {'message':message,'replay_context':context}

    def action(self, action, body):
        if not self.allowed(action)['allowed']:raise ValueError(self.allowed(action)['reason'])
        if action in HISTORY_ACTIONS:return self.delete_history(action,body)
        if action=='generate_plan':
            self.active=True
            self.operation=None
            if getattr(self.app,'pipeline_review',None):
                review=deepcopy(self.app.pipeline_review)
                key=(getattr(self.app,'pipeline_run_id',None),json.dumps(review,sort_keys=True))
                with self.lock:
                    if getattr(self,'preview_context',None)!=key:
                        self.generation=getattr(self,'generation',0)+1;self.preview_context=key
                        self.preview_key=None;self.preview_failure=None;self.preview_pending=None
                return self.publish_workflow_review(key,review,explicit=True)
            mode=body.get('mode','full')
            if mode not in ('full','wrist'):raise ValueError('请选择完整标定或仅重做双腕')
            if mode=='wrist' and not body.get('head_result_id'):raise ValueError('仅重做双腕需要明确指定已通过的头标定')
            self.invalidate();self.active=True
            return self.app.action('/api/calibration-pipeline',dict(mode=mode,**({'head_result_id':body['head_result_id']} if mode=='wrist' else {})))
        if action=='execute':
            pipeline=bool(getattr(self.app,'pipeline_running',False))
            result=self.app.action('/api/calibration-confirm',dict(token=body['token'],confirmation='START_REVIEWED_SINGLE_ARM',physical_clearance=True))
            if not pipeline:self.operation=dict(id=result.get('batch_id') or secrets.token_hex(12),action='capture',kind=self.app.workflow['kind'],message=result['message'])
            return result
        if action in CALIBRATION_TOOLS:
            self.active=False
            kind=self.app.workflow['kind']
            if action=='configure_calibration':
                kind=body.get('kind')
                if kind=='wrist' and not body.get('head_result_id'):raise ValueError('双腕标定需要明确指定已通过的头标定')
                result=self.app.action('/api/workflow',dict(kind=kind,head_result_id=body.get('head_result_id')))
                self.operation=None
            elif action=='fit_source':
                result=self.app.action('/api/fit-source',{'id':body.get('batch_id')})
                self.app.invalidate_reviews();self.operation=None
            else:
                if action=='fit_batch' and body.get('batch_id')!=getattr(self.app,'selected_batch',None):
                    raise ValueError('求解来源已变化，请核验当前批次后再求解')
                if action=='preview_batch':
                    if body.get('queue_revision')!=self.manual_revision():raise ValueError('标定计划已变化，请刷新后重新审阅')
                    preview=self.app.action('/api/calibration-preview',{})
                    summary=f"独立{'双腕' if kind=='wrist' else '头部'}标定批次：{preview.get('samples',0)} 个采样点，纯运动 {preview['total_motion_duration_s']:.1f} 秒；每点保存成功并确认采样后继续。"
                    result=dict(preview=preview,summary=summary,message=summary)
                else:
                    path=('/api/wrist-plan' if kind=='wrist' else '/api/auto-plan') if action=='plan_batch' else ('/api/wrist-fit' if kind=='wrist' else '/api/fit')
                    result=self.app.action(path,{})
                self.operation=dict(id=secrets.token_hex(12),action={'plan_batch':'plan','preview_batch':'review','fit_batch':'fit'}[action],kind=kind,message=result.get('message',''))
            self.last_catalog=0.
            return result
        if action=='cancel':return self.cancel()
        if action=='mesh':return self.app.action('/api/overlay-rendering',{'enabled':body.get('enabled')})
        if action=='select_result':return self.select_result(body)
        if action=='install':
            group=next((row for row in self.app.calibration_groups() if row['id']==body.get('result_id')),None)
            if not group or not group.get('wrist'):raise ValueError('请选择含头部与匹配双腕的完整标定组')
            if not group.get('installable') and not group.get('installed'):raise ValueError(group.get('reason') or '该组尚不能安装')
            result=self.app.action('/api/calibration-install',dict(head_result_id=group['id'],wrist_result_id=group['wrist']['id']))
            self.calibration_installed(self.app.installed_pair())
            self.last_catalog=0.
            return result
        raise ValueError('未知标定操作')

    def invalidate(self):
        with self.lock:
            self.active=False;self.operation=None;self.generation=getattr(self,'generation',0)+1
            self.preview_context=None;self.preview_key=None;self.preview_failure=None;self.preview_pending=None
        self.app.invalidate_reviews()

    def cancel(self):
        self.invalidate()
        self.app.stop.set()
        with self.service.lock:self.service.reviews.pop((self.device.environment,'calibration'),None)
        return {'message':'标定工作流已请求取消，已完成阶段保留'}


class CollectionWorkflow:
    def __init__(self, playground, device, service):
        self.playground,self.device,self.service=playground,device,service
        self.playground.managed_lifecycle=True
        self.lock=threading.Lock();self.closed=threading.Event()
        self.value=dict(status='connecting',busy=False,stage_id='connecting',title='数采',primary_action=None,outcomes={})
        self.catalog=[];self.review_key=None;self.speed_scale=None
        self.generation=0
        self.media_files={}
        self._quality_cache={}
        self.thread=threading.Thread(target=self.monitor,daemon=True,name='collection-view');self.thread.start()

    def environment_status(self):
        """Read existing service facts without building a business review."""
        with self.playground.lock:
            state=self.playground.state
            result={key:deepcopy(state.get(key)) for key in ('status','busy','error','warmup','progress')}
            environment_ready=bool((state.get('environment') or {}).get('ready'))
        initialization=self.playground.initialization()
        services={'detector':{k:deepcopy(v) for k,v in initialization.items() if k not in ('planner','scene')}}
        for name in ('planner','scene'):
            if initialization.get(name):services[name]=initialization[name]
        services_ready=all(item.get('status') in ('ready','busy') for item in services.values())
        ready=services_ready and environment_ready and (result['warmup'] or {}).get('status')=='ready' and all(
            name=='detector' or item.get('model_initialized') is True for name,item in services.items())
        return dict(result,ready=bool(ready),needs_warmup=not ready,services=services,
            declared_roles=list(services),services_ready=services_ready)

    def operations_prepare(self,*,still_current,lifecycle_admission):
        from .resident_collection import require_collection_control
        if (not callable(still_current) or not callable(lifecycle_admission)
                or not getattr(self.device,'maintenance',False) or not still_current()):
            raise ValueError('数采准备需要当前有效的运维维护窗口')
        require_collection_control(self.device)
        def admission(action):
            return (getattr(self.device,'maintenance',False) and
                (action.startswith('stop:') or still_current()) and lifecycle_admission(action) is True)
        services=self.playground.initialization()
        unavailable=services.get('status') in ('failed','stopped','unknown') or any(
            (services.get(role) or {}).get('status') in ('failed','stopped','unknown') for role in ('planner','scene'))
        hook=getattr(self.playground,'operations_prepare_services',None)
        preparation=None
        if callable(hook):
            def launch(create):
                with self.service.lock:
                    if admission('start:collection')is not True:raise InterruptedError('运维服务启动已撤销')
                    return create()
            if admission('start:collection')is not True:raise InterruptedError('运维服务准备已撤销')
            preparation=hook(still_current=still_current,launch=launch)
            if not isinstance(preparation,dict) or not isinstance(preparation.get('service_steps'),list):
                raise ValueError('固定配置运维hook未提供准备证据')
            if unavailable or any(row.get('needs_warmup') or row.get('outcome')!='reused' for row in preparation['service_steps']):
                with self.playground.lock:self.playground.state['warmup']={'status':'invalidated'}
        elif unavailable:raise ValueError('当前固定配置未装配受控运维准备hook，不支持自动恢复')
        if not still_current():raise ValueError('数采准备运维操作已撤销')
        value=self.environment_status()
        if not value['services_ready']:raise ValueError('数采服务尚未准备完成；未启动业务预热')
        return dict(services_ready=True,ready=value['ready'],needs_warmup=value['needs_warmup'],environment=value,
            service_steps=(preparation or {}).get('service_steps',[]),service_preparation=preparation)

    def warmup_reason(self):
        preparation=getattr(self.device,'collection_preparation',None)
        if preparation is not None and (getattr(preparation,'parts',None) is None or preparation.snapshot().get('busy')):
            return '请先在服务运维准备当前数采资产与常驻服务'
        if preparation is not None and preparation.closed.is_set():return '数采服务生命周期已结束，请在服务运维核验'
        state=preparation.environment_status() if preparation is not None else self.environment_status()
        if state.get('services_ready') is not True:return '数采计算服务未就绪，请在服务运维准备或恢复对应服务'
        from .resident_collection import require_collection_control
        try:require_collection_control(self.device)
        except ValueError as error:return str(error)
        return None

    def business_warmup(self,body):
        """Business jobs may initialize models, never own service lifecycle or SDK acquisition."""
        reason=self.warmup_reason()
        if reason:raise ValueError(reason)
        speed=body.get('speed_scale',.3)
        if type(speed) not in (int,float) or not math.isfinite(speed) or not .3<=speed<=1.7:
            raise ValueError('运动速度必须在 30% 至 170% 之间')
        generation=self.service.generations[self.device.environment];runtime=self.device.runtime
        preparation=getattr(self.device,'collection_preparation',None)
        def current():
            return (not self.closed.is_set() and not getattr(self.device,'maintenance',False)
                and self.device.runtime is runtime and generation==self.service.generations[self.device.environment])
        if preparation is not None:
            snapshot=preparation.installed_snapshot()
            if not snapshot or snapshot.get('verified')is not True:raise ValueError('当前安装组无效，请在服务运维核验')
            identity=preparation.installation(snapshot)
            if preparation.parts is None or identity!=preparation.identity:
                raise ValueError('当前安装资产尚未准备，请在服务运维重新核验')
            active=current
            def current():
                return (active() and not preparation.closed.is_set() and preparation.parts is not None and preparation.parts.workflow is self
                    and snapshot==preparation.installed_snapshot() and identity==preparation.identity)
            preparation._verify_parts(current)
            for name in ('detector','planner','scene'):
                manager=getattr(preparation.parts,name,None)
                if manager is not None:
                    if manager.status().get('status')!='ready':raise ValueError(name+' 服务未就绪，请在服务运维恢复')
                    manager.probe()
        if not current():raise InterruptedError('业务预热上下文已变化')
        result=self.playground.start(dict(action='prepare',speed_scale=speed),
            lifecycle_admission=lambda step:step=='warmup:collection' and current())
        return result if result.get('reused') else self.accepted(result,'prepare')

    def monitor(self):
        while not self.closed.is_set():
            with self.lock:generation=self.generation
            try:self.refresh()
            except Exception as error:
                with self.lock:
                    if generation==self.generation:self.value.update(view_error=str(error),message=str(error))
            self.closed.wait(.5)

    def refresh(self):
        with self.lock:generation=self.generation
        state=self.playground.snapshot();plan=state.get('plan') or {};progress=state.get('progress') or {}
        native=state.get('native_task') or {}
        instance=progress.get('execution_instance_id') or state.get('execution_instance_id')
        execution_binding=(dict(execution_instance_id=instance,device_id=self.device.id,environment=self.device.environment,
            timeline_digest=plan.get('timeline_digest'),**{key:native.get(key) for key in
            ('runtime_nonce','source_clock_id','execution_timeline_digest')})
            if instance and native.get('execution_instance_id')==instance else None)
        ready=state.get('environment',{}).get('ready')
        busy=state.get('busy',False)
        speed=getattr(self,'speed_scale',None) or plan.get('speed_scale',.3)
        title=progress.get('label') or '数采任务'
        if not busy:title={'completed':'本次数采流程已完成','failed':'本次数采流程失败','stopped':'本次数采流程已停止'}.get(state.get('status'),title)
        recording=state.get('recording') or {}
        validation=(recording.get('manifest') or {}).get('validation') or {}
        data_error=recording.get('error') or recording.get('seal_error') or recording.get('diagnostics_error')
        data_outcome=(dict(status='invalid',message=data_error) if data_error else
            {'passed':'valid','failed':'invalid'}.get(validation.get('status')) or recording.get('status') or
            ('pending' if recording.get('done') else 'recording' if recording.get('ready') else '未录制'))
        prepare_reason='数采任务正在进行' if busy else self.warmup_reason()
        plan_reason=prepare_reason or '数采服务已就绪；请在本页完成业务环境准备（初始化与预热）' if not ready else None
        primary=dict(id='generate_plan',label='采集新观测并生成计划',
                     allowed=not busy and ready,reason=prepare_reason if busy else plan_reason)
        if plan.get('validated') and state.get('can_execute') and not busy:
            primary.update(label='重新预览冻结计划',allowed=True)
        value=dict(id=state.get('output'),status=state.get('status','idle'),busy=busy,
            stage_id='collection.'+(state.get('action') if state.get('action') in ('prepare','plan','execute') else 'idle'),title=title,
            message=state.get('error') or state.get('capture_warning') or state.get('message',''),
            failure=deepcopy(state.get('failure')) if state.get('status')=='failed' else None,
            primary_action=primary,progress=None,speed_scale=speed,execution_binding=execution_binding,outcomes=dict(motion=state.get('physical_outcome','未执行'),
                data=data_outcome,
                simulation=state.get('simulation','未验证'),task=state.get('task_outcome','未验证')),
            replan_action=dict(id='generate_plan',label='重新观测并规划',allowed=not busy,reason='数采任务正在进行' if busy else None) if plan.get('validated') else None,
            details=[{'label':'已耗时','value':f"{progress.get('elapsed_s',0):.1f} 秒"},
                     {'label':'计划身份','value':plan.get('timeline_digest','尚未生成')}])
        actions={name:dict(id=action,label=label,allowed=not busy and ready if name=='plan' else
                          not busy and bool(state.get('can_execute')) if name=='execute' else not busy,
                          reason='数采任务正在进行' if busy else plan_reason if name=='plan' and not ready else
                          '请先生成并审阅可执行计划' if name=='execute' and not state.get('can_execute') else None)
                 for name,action,label in (('prepare','collection_prepare','准备数采环境'),
                     ('plan','collection_plan','采集新观测并生成计划'),('execute','execute','审阅并执行本批'))}
        actions['prepare'].update(label='初始化与预热',allowed=prepare_reason is None,reason=prepare_reason)
        relations=dict(plan={key:deepcopy(plan[key]) for key in ('output','timeline_digest','plan_path','duration_s','speed_scale') if key in plan},
                       source={},calibration={})
        for key in ('source_path','hardware_binding'):
            if not plan.get(key):continue
            try:
                artifact=json.loads(result_file(self.playground.directory,plan[key]).read_text())
                if key=='source_path':
                    relations['source']=dict(path=plan[key],**{name:artifact[name] for name in ('run_id','scene_digest','source_mode','environment') if name in artifact})
                    profile=artifact.get('calibration_profile') or {}
                    if profile.get('profile_id'):relations['calibration']['profile_id']=profile['profile_id']
                else:relations['calibration'].update({name:artifact[name] for name in ('robot_id','camera_serials','calibration_profile_digest','environment') if name in artifact})
            except (OSError,ValueError,TypeError) as error:relations.setdefault('errors',{})[key]=str(error)
        value['collection']=dict(actions=actions,timing=deepcopy(progress),relations=relations,
            warmup=deepcopy(state.get('warmup') or {'status':'not_started','message':'点击初始化与预热。'}),
            environment=deepcopy(state.get('environment') or {}),recording=deepcopy(recording),native_task=deepcopy(native),
            simulation=deepcopy(state.get('simulation')),errors=deepcopy(state.get('errors') or {}),
            execution_wait=deepcopy(progress.get('execution_wait') or state.get('execution_wait')),
            capture_ready=state.get('capture_ready'),capture_warning=state.get('capture_warning'))
        diagnostics=[]
        if state.get('environment',{}).get('checks'):diagnostics.append(dict(label='数采环境检查',value=state['environment']['checks']))
        if progress.get('stages'):
            diagnostics.append(dict(label='数采分阶段耗时',value=[{key:row[key] for key in ('label','status','elapsed_s','estimate_s') if key in row} for row in progress['stages']]))
        manifest=recording.get('manifest') or {}
        if manifest:diagnostics.append(dict(label='本轮录制封存',value={key:manifest[key] for key in ('capture_status','validation','command_count','feedback_count','camera_frames','execution_timeline_digest') if key in manifest}))
        if native:
            detail={key:native[key] for key in ('execution_instance_id','source_clock_id','execution_timeline_digest','task_outcome','task_qualified','receipt_verified','receipt_path','scope') if key in native}
            failure=native.get('first_failure')
            if failure:detail['first_failure']={key:failure[key] for key in ('reason','detail','physics_step','absolute_physics_step') if key in failure}
            diagnostics.append(dict(label='本轮原生任务回执',value=detail))
        simulation=state.get('simulation')
        if isinstance(simulation,dict):diagnostics.append(dict(label='本轮辅助回放',value={key:simulation[key] for key in ('status','scope','same_plant','task_success_evidence','timeline_digest','error') if key in simulation}))
        value['diagnostics']=diagnostics
        catalog=[];files={}
        if plan.get('plan_path') and (plan.get('validated') or state.get('action')=='execute'):
            try:
                path=result_file(self.playground.directory,plan['plan_path'])
                identity=hashlib.sha256((str(path)+'\0'+str(plan.get('timeline_digest'))).encode()).hexdigest();files[identity]=path
                value['trajectory']=dict(id=identity,url=media_url(self.device.environment,'collection',identity),
                    timeline_digest=plan.get('timeline_digest'),speed_scale=plan.get('speed_scale',.3),duration_s=plan.get('duration_s'))
                if plan.get('first_frame_path'):
                    try:
                        path=result_file(self.playground.directory,plan['first_frame_path'])
                        identity=hashlib.sha256(str(path).encode()).hexdigest();files[identity]=path
                        value['trajectory']['first_frame_url']=media_url(self.device.environment,'collection',identity)
                    except (OSError,ValueError) as error:value['trajectory']['first_frame_error']=str(error)
            except (OSError,ValueError) as error:value['trajectory']=dict(error=str(error))
        for row in state.get('videos',[]):
            url=urlsplit(row['url'])
            path=self.playground.media(url.path.rsplit('/',1)[-1],parse_qs(url.query).get('v',[None])[0])
            if path is None:continue
            path=result_file(self.playground.directory,path)
            identity=hashlib.sha256(row['url'].encode()).hexdigest();files[identity]=path
            replay_identity={}
            for prefix,role in (('source-','source'),('sim-','target')):
                camera=path.stem.removeprefix(prefix)
                if path.stem.startswith(prefix) and camera in ('base_0','left_wrist_0','right_wrist_0'):
                    replay_identity=dict(camera=camera,role=role)
            from .recording_quality import view_for_playback
            quality=view_for_playback(path.parent,video_ready=True,
                controller_outcome=(recording.get('manifest') or {}).get('controller_outcome'),cache=getattr(self,'_quality_cache',None))
            catalog.append(dict(id=identity,label=row.get('label','录制视频'),kind='recording',status=state.get('status','unknown'),
                run_id=state.get('output'),recording_quality=quality,
                media=dict(id=identity,label=row.get('label','录制视频'),kind='replay',ready=True,mime_type='video/mp4',
                    url=media_url(self.device.environment,'collection',identity),**replay_identity)))
        with self.lock:
            if generation!=self.generation:return
            self.value=value;self.catalog=catalog;self.media_files=files

    def accepted(self,result,action):
        """Publish the accepted task before its short HTTP request completes.

        Snapshot stays cached; remote health reads never enter this lock. A
        refresh already in flight belongs to the preceding generation.
        """
        with self.lock:
            self.generation+=1;self.review_key=None
            self.value.update(id=result['output'],status='running',busy=True,stage_id='collection.'+action,
                title={'prepare':'准备数采环境','plan':'采集新观测并生成计划','execute':'执行与录制'}[action],
                message='本轮任务已启动，正在更新阶段进度。',failure=None,progress={'indeterminate':True},execution_binding=None,outcomes={},replan_action=None,
                primary_action=dict(id='generate_plan',label='采集新观测并生成计划',allowed=False,reason='数采任务正在进行'))
            self.value.pop('view_error',None)
        with self.service.lock:self.service.reviews.pop((self.device.environment,'collection'),None)
        return result

    def snapshot(self):
        with self.lock:return deepcopy(self.value)
    def results(self):
        with self.lock:return deepcopy(self.catalog)
    def media(self,result_id):
        with self.lock:path=self.media_files.get(result_id)
        return result_file(self.playground.directory,path) if path else None
    def allowed(self,action):
        value=self.snapshot()
        if action in COLLECTION_TOOLS:
            if value.get('busy'):return dict(allowed=False,reason='数采任务正在进行')
            if action=='collection_prepare':
                reason=self.warmup_reason()
                return dict(allowed=reason is None,reason=reason)
            return deepcopy(value.get('collection',{}).get('actions',{}).get(
                'prepare' if action=='collection_prepare' else 'plan',dict(allowed=False,reason='正在读取数采环境状态')))
        with self.playground.lock:busy=self.playground.state.get('busy',False)
        yes=action=='cancel' and busy or action in ('generate_plan','execute') and not busy
        return dict(allowed=bool(yes),reason=None if yes else '当前数采阶段不允许此操作')
    def action(self,action,body):
        restore=body.get('restore_frozen_plan',False)
        if ('restore_frozen_plan' in body and (action!='generate_plan' or type(restore) is not bool)
                or restore and (body.get('replan') or 'operator_assisted_request_id' in body)):
            raise ValueError('恢复冻结计划只用于明确的重新审阅，不能同时请求重新规划')
        assisted=body.get('operator_assisted_request_id')
        if 'operator_assisted_request_id' in body:
            if (action!='collection_plan' or self.device.environment!='physical'
                    or not isinstance(assisted,str) or not assisted.strip() or len(assisted)>256):
                raise ValueError('辅助定位身份仅用于本次真机数采规划')
            if self.playground.snapshot().get('busy'):raise ValueError('数采任务正在进行')
        elif action!='collection_prepare' and not self.allowed(action)['allowed']:raise ValueError(self.allowed(action)['reason'])
        if action=='collection_prepare':
            return self.business_warmup(body)
        if action in ('generate_plan',*COLLECTION_TOOLS):
            if restore:self.playground.restore_prepared()
            state=self.playground.snapshot()
            plan=state.get('plan') or {}
            if restore and (plan.get('validated') is not True or not state.get('can_execute')):
                raise ValueError('冻结计划已核验，但当前控制状态不允许审阅执行；不会自动预热或重新观测')
            speed=body.get('speed_scale',getattr(self,'speed_scale',None) or plan.get('speed_scale',.3))
            if type(speed) not in (int,float) or not math.isfinite(speed) or not .3<=speed<=1.7:
                raise ValueError('运动速度必须在 30% 至 170% 之间')
            if action in COLLECTION_TOOLS:
                if not state.get('environment',{}).get('ready'):
                    raise ValueError(self.warmup_reason() or '数采服务已就绪；请在本页完成业务环境准备（初始化与预热）')
                stage='plan'
                result=self.playground.start(dict(action=stage,speed_scale=speed,
                    **({'operator_assisted_request_id':assisted} if assisted is not None else {})))
                with self.lock:self.speed_scale=speed
                return self.accepted(result,stage)
            replan=body.get('replan',False)
            if type(replan) is not bool:raise ValueError('replan 必须是布尔值')
            if not replan and plan.get('validated') and state.get('can_execute'):
                owner=self.device.snapshot('collection')
                with self.lock:
                    self.speed_scale=speed
                    self.generation+=1
                    self.review_key=(plan.get('timeline_digest'),plan.get('output'),speed)
                    generation=self.generation
                return dict(preview=dict(token=plan['timeline_digest'],command_epoch=owner['command_epoch'],expires_in_s=30,
                    workflow_generation=generation,speed_scale=speed),
                    summary=(plan.get('summary') or '重新审阅冻结路径，不重新观测')+f'；执行速度 {speed*100:g}%，静止等待不缩放')
            with self.service.lock:self.service.reviews.pop((self.device.environment,'collection'),None)
            if not state.get('environment',{}).get('ready'):
                raise ValueError(self.warmup_reason() or '数采服务已就绪；请在本页完成业务环境准备（初始化与预热）')
            stage='plan'
            result=self.playground.start(dict(action=stage,speed_scale=speed))
            with self.lock:self.speed_scale=speed
            return self.accepted(result,stage)
        if action=='execute':
            plan=self.playground.snapshot().get('plan') or {}
            if plan.get('timeline_digest')!=body['token']:raise ValueError('本轮计划身份已变化，请重新审阅')
            speed=body.get('speed_scale',getattr(self,'speed_scale',None) or plan.get('speed_scale',.3))
            if type(speed) not in (int,float) or not math.isfinite(speed) or not .3<=speed<=1.7:
                raise ValueError('运动速度必须在 30% 至 170% 之间')
            return self.accepted(self.playground.start({'action':'execute','speed_scale':speed}),'execute')
        if action=='cancel':return self.cancel()
        raise ValueError('未知数采操作')
    def invalidate(self):
        try:
            self.playground.invalidate_checkpoint()
            with self.playground.lock:self.playground.state['plan']={'validated':False}
        finally:
            with self.lock:self.generation+=1;self.review_key=None
    def cancel(self):
        try:result=self.playground.start({'action':'interrupt'})
        finally:
            with self.lock:self.generation+=1;self.review_key=None
            with self.service.lock:self.service.reviews.pop((self.device.environment,'collection'),None)
        return dict(result,message='数采已请求中断；动作停止与数据收尾独立显示')
