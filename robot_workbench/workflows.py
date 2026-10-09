"""Thin adapters around existing business workflows; no substitute solvers/planners."""
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


class CalibrationWorkflow:
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
        review=report.get('review')
        key=(getattr(self.app,'pipeline_run_id',None),str(review))
        if self.active and review and key!=self.preview_key:
            self.preview_key=key
            preview=self.app.preview_calibration()
            summary=f"{review.get('label','本批')}：{preview.get('samples',0)} 次采样，纯运动 {preview.get('total_motion_duration_s',0):.1f} 秒；请审阅全部姿态"
            self.service.add_review(self.device,'calibration',preview,'execute',summary)
        sub=report.get('subprogress') or {}
        running=bool(report.get('running'))
        busy=running or state.get('status') in ('PLANNING','CAPTURING','FITTING','RUNNING')
        stage_id,completed=calibration_stage(state)
        status='stopped' if report.get('stopped') else 'failed' if report.get('aborted') or state.get('status')=='REJECTED' else 'running' if running else 'completed' if completed==7 else 'idle'
        working=running and not review and status=='running' and completed<7
        measured=(working and sub.get('stage')==completed and type(sub.get('fraction')) in (int,float)
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
        value=dict(id=getattr(self.app,'pipeline_run_id',None),status=status,busy=busy,
            stage_id=stage_id,title=report.get('phase') or '头部与双腕标定',
            message=report.get('aborted') or state.get('reason') or message,
            progress=progress,primary_action=primary,outcomes={},
            overall_progress=dict(current=completed+fraction,total=7,unit='阶段',completed_stages=completed,
                indeterminate=working and not measured,stage_ids=list(CALIBRATION_STAGES)),
            details=[{'label':'当前模型','value':self.app.config.get('asset_geometry_id','未知')},
                     {'label':'运行身份','value':getattr(self.app,'pipeline_run_id',None) or '尚未生成'}])
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
            value.update(replay=deepcopy(self.replay),replay_context=deepcopy(self.replay_context))
            value['arrival_statistics']=deepcopy(getattr(self,'arrival_statistics',{'error':'标定到位统计尚未读取'}))
            self.value=value

    def snapshot(self):
        with self.lock:value=deepcopy(self.value)
        value['deletion_preview']=self.deletion_preview()
        value['deletion_result']=deepcopy(getattr(self,'deletion_result',None))
        return value

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
        heads={row['id']:row for row in self.app.calibration_results()}
        wrists={row['id']:row for row in self.app.wrist_results()}
        if identity in heads:kind,row='head',heads[identity];batch=row['batch']
        elif identity in wrists:kind,row='wrist',wrists[identity];batch=str(Path(row['file']).parent)
        elif isinstance(identity,str) and identity.startswith('capture:') and identity[8:] in {r['id'] for r in self.app.history()}:
            kind,row,batch='capture',{},identity[8:]
        else:raise ValueError('所选标定结果或采集批次不存在')
        raw_deleted=hasattr(self.app,'deleted_batch') and self.app.deleted_batch(batch)
        view={'frames':[]} if raw_deleted else self.app.review_batch(batch)  # Replay never changes solver input.
        frames=view.get('frames') or [];index=body.get('frame_index',0)
        if type(index) is not int or index<0 or frames and index>=len(frames):raise ValueError('回放帧索引无效')
        image_urls=frames[index].get('image_urls',{}) if frames else {}
        camera=body.get('camera','left' if kind=='wrist' and 'left' in image_urls else 'head')
        if camera not in ('head','left','right'):raise ValueError('请选择头部或左右腕相机')
        overlay=body.get('overlay',False)
        if type(overlay) is not bool:raise ValueError('overlay 必须是布尔值')
        overlay_available=bool(frames) and not raw_deleted and (kind=='head' and camera=='head' or kind=='wrist' and camera in ('left','right'))
        if overlay:
            if not overlay_available:raise ValueError('当前相机需要匹配的头部或双腕结果')
            image=self.app.historical_overlay(dict(batch_id=batch,frame_index=index,camera=camera,result_id=identity))['image_url']
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
        elif previous==identity:message='已更新只读回放'
        elif kind=='wrist' or kind=='head' and getattr(self.app,'live',None):
            reply=self.app.action('/api/wrist-select' if kind=='wrist' else '/api/calibration',{'result_id':identity})
            message=reply.get('message','已选择标定结果')
        context=dict(result_id=identity,batch_id=batch,frame_index=index,frame_count=len(frames),camera=camera,
            cameras=sorted(image_urls),overlay=overlay,overlay_available=overlay_available,
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
                preview=self.app.preview_calibration()
                return dict(preview=preview,summary=f"本批 {preview.get('samples',0)} 次采样，纯运动 {preview['total_motion_duration_s']:.1f} 秒")
            mode=body.get('mode','full')
            if mode not in ('full','wrist'):raise ValueError('请选择完整标定或仅重做双腕')
            if mode=='wrist' and not body.get('head_result_id'):raise ValueError('仅重做双腕需要明确指定已通过的头标定')
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
        self.active=False
        self.operation=None
        self.app.invalidate_reviews()

    def cancel(self):
        self.invalidate()
        self.app.stop.set()
        with self.service.lock:self.service.reviews.pop((self.device.environment,'calibration'),None)
        return {'message':'标定工作流已请求取消，已完成阶段保留'}


class CollectionWorkflow:
    def __init__(self, playground, device, service):
        self.playground,self.device,self.service=playground,device,service
        self.lock=threading.Lock();self.closed=threading.Event()
        self.value=dict(status='connecting',busy=False,stage_id='connecting',title='数采',primary_action=None,outcomes={})
        self.catalog=[];self.review_key=None
        self.media_files={}
        self.thread=threading.Thread(target=self.monitor,daemon=True,name='collection-view');self.thread.start()

    def monitor(self):
        while not self.closed.is_set():
            try:self.refresh()
            except Exception as error:
                with self.lock:self.value.update(view_error=str(error),message=str(error))
            self.closed.wait(.5)

    def refresh(self):
        state=self.playground.snapshot();plan=state.get('plan') or {};progress=state.get('progress') or {}
        ready=state.get('environment',{}).get('ready')
        busy=state.get('busy',False)
        primary=dict(id='generate_plan',label='采集新观测并生成计划' if ready else '准备数采环境',
                     allowed=not busy,reason='数采任务正在进行' if busy else None)
        if plan.get('validated') and state.get('can_execute'):
            key=(plan.get('timeline_digest'),plan.get('output'),plan.get('speed_scale'))
            if key!=self.review_key:
                owner=self.device.snapshot('collection')
                self.service.add_review(self.device,'collection',dict(token=plan['timeline_digest'],
                    command_epoch=owner['command_epoch'],expires_in_s=30), 'execute',
                    plan.get('summary') or f"冻结轨迹 {plan.get('duration_s',0):.2f} 秒；当前起点变化时使用慢初始化段")
                self.review_key=key
            primary.update(label='重新预览冻结计划',allowed=True)
        value=dict(id=state.get('output'),status=state.get('status','idle'),busy=busy,
            stage_id='collection.'+(state.get('action') if state.get('action') in ('prepare','plan','execute') else 'idle'),title=progress.get('label') or '数采任务',
            message=state.get('error') or state.get('capture_warning') or state.get('message',''),
            primary_action=primary,progress=None,outcomes=dict(motion=state.get('physical_outcome','未执行'),
                data=(state.get('recording') or {}).get('status','未录制'),
                simulation=state.get('simulation','未验证'),task=state.get('task_outcome','未验证')),
            replan_action=dict(id='generate_plan',label='重新观测并规划',allowed=not busy,reason='数采任务正在进行' if busy else None) if plan.get('validated') else None,
            details=[{'label':'已耗时','value':f"{progress.get('elapsed_s',0):.1f} 秒"},
                     {'label':'计划身份','value':plan.get('timeline_digest','尚未生成')}])
        catalog=[];files={}
        for row in state.get('videos',[]):
            url=urlsplit(row['url'])
            path=self.playground.media(url.path.rsplit('/',1)[-1],parse_qs(url.query).get('v',[None])[0])
            if path is None:continue
            path=result_file(self.playground.directory,path)
            identity=hashlib.sha256(row['url'].encode()).hexdigest();files[identity]=path
            catalog.append(dict(id=identity,label=row.get('label','录制视频'),kind='recording',status=state.get('status','unknown'),
                media=dict(id=identity,label=row.get('label','录制视频'),kind='replay',ready=True,mime_type='video/mp4',
                    url=media_url(self.device.environment,'collection',identity))))
        with self.lock:self.value=value;self.catalog=catalog;self.media_files=files

    def snapshot(self):
        with self.lock:return deepcopy(self.value)
    def results(self):
        with self.lock:return deepcopy(self.catalog)
    def media(self,result_id):
        with self.lock:path=self.media_files.get(result_id)
        return result_file(self.playground.directory,path) if path else None
    def allowed(self,action):
        value=self.snapshot()
        yes=action=='cancel' and value.get('busy') or action in ('generate_plan','execute') and not value.get('busy')
        return dict(allowed=bool(yes),reason=None if yes else '当前数采阶段不允许此操作')
    def action(self,action,body):
        if not self.allowed(action)['allowed']:raise ValueError(self.allowed(action)['reason'])
        if action=='generate_plan':
            state=self.playground.snapshot()
            plan=state.get('plan') or {}
            replan=body.get('replan',False)
            if type(replan) is not bool:raise ValueError('replan 必须是布尔值')
            if not replan and plan.get('validated') and state.get('can_execute'):
                owner=self.device.snapshot('collection')
                return dict(preview=dict(token=plan['timeline_digest'],command_epoch=owner['command_epoch'],expires_in_s=30),
                    summary=plan.get('summary') or f"冻结轨迹 {plan.get('duration_s',0):.2f} 秒；重新审阅，不重新规划")
            self.review_key=None
            with self.service.lock:self.service.reviews.pop((self.device.environment,'collection'),None)
            return self.playground.start(dict(action='plan' if state.get('environment',{}).get('ready') else 'prepare'))
        if action=='execute':
            plan=self.playground.snapshot().get('plan') or {}
            if plan.get('timeline_digest')!=body['token']:raise ValueError('本轮计划身份已变化，请重新审阅')
            return self.playground.start({'action':'execute'})
        if action=='cancel':return self.cancel()
        raise ValueError('未知数采操作')
    def invalidate(self):
        self.review_key=None
        self.playground.invalidate_checkpoint()
        with self.playground.lock:self.playground.state['plan']={'validated':False}
    def cancel(self):
        result=self.playground.start({'action':'interrupt'})
        self.review_key=None
        with self.service.lock:self.service.reviews.pop((self.device.environment,'collection'),None)
        return dict(result,message='数采已请求中断；动作停止与数据收尾独立显示')
