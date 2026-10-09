const BUSINESS = {calibration: '标定', collection: '数采'};
const ENVIRONMENT = {fake: 'Fake · 开发设备', physical: 'Physical · 物理设备'};
const CALIBRATION_TOOLS=['configure_calibration','plan_batch','preview_batch','fit_source','fit_batch'];
const DISPLAY_ACTIONS=['preview_target','preview_overview','preview_mesh','preview_recover'];
const STATUS = {unknown:'未知',idle:'空闲',ready:'就绪',running:'进行中',executing:'执行中',planning:'规划中',capturing:'采集中',recording:'录制中',fitting:'求解中',completed:'已完成',complete:'完整',passed:'通过',failed:'失败',invalid:'无效',stopped:'已停止',stopping:'正在停止',pending:'等待中',unavailable:'不可用',awaiting_review:'等待审阅',awaiting_confirmation:'等待确认',not_started:'尚未开始',not_applicable:'不适用',valid:'有效',damping:'阻尼保持',position:'位置保持',manual_damping:'手动阻尼保持'};
const textValue = value => value == null ? '未知' : typeof value === 'object' ? JSON.stringify(value, null, 2) : String(value);
const statusText = value => ({unconfigured:'业务服务未配置',connecting:'正在连接',candidate:'候选结果',quality_failed:'质量未通过',incomplete_left:'左腕未通过',incomplete_right:'右腕未通过',incomplete_both:'双腕未通过'}[String(value).toLowerCase()] || STATUS[String(value).toLowerCase()] || textValue(value));
const outcomeStatus = value => typeof value === 'object' && value ? value.status : value;
const executing = value => ['running','executing','settling','verifying','awaiting_capture'].includes(String(value).toLowerCase());

export function frameAgeSeconds(media, state, elapsedMs = 0) {
  if (!Number.isFinite(media?.source_wall_ns) || !Number.isFinite(state?.server_wall_ns)) return null;
  return Math.max(0, (state.server_wall_ns - media.source_wall_ns) / 1e9 + elapsedMs / 1000);
}

export function countedProgress(value) {
  return value && Number.isFinite(value.current) && Number.isFinite(value.total) && value.total > 0 && value.current >= 0 && value.current <= value.total ? value : null;
}

export function actionPermission(state, action, options = {}) {
  const operation = state?.workflow?.primary_action?.id === action && action !== 'stop'
    ? state.workflow.primary_action : state?.runtime?.allowed_operations?.[action];
  if (!state?.csrf) return {allowed:false, reason:'尚未核验当前请求身份'};
  if (!operation || operation.allowed !== true) return {allowed:false, reason:operation?.reason || '后端尚未允许此操作'};
  if (options.pending && action !== 'stop') return {allowed:false, reason:'操作请求正在处理中；停止仍可独立请求'};
  if (options.fresh === false && action !== 'stop') return {allowed:false, reason:'状态已过期或连接中断，等待重新核验'};
  if (['execute','manual_execute'].includes(action)) {
    if (!state.workflow?.review?.id) return {allowed:false, reason:'尚无本次执行审阅，请重新生成计划'};
    if ((state.device?.environment === 'physical' || state.workflow.review.physical_clearance_required) && !options.clearance)
      return {allowed:false, reason:'请先勾选确认本条轨迹与现场净空'};
  }
  if (action === 'connect' && state.device?.environment === 'physical' && !options.supported)
    return {allowed:false, reason:'请先确认当前双臂已可靠支撑'};
  return {allowed:true, reason:''};
}

export function safeMediaUrl(value, origin) {
  if (typeof value !== 'string' || !value) return null;
  try { const url = new URL(value, origin); return url.origin === new URL(origin).origin && ['http:','https:'].includes(url.protocol) ? url.href : null; }
  catch { return null; }
}

export function verifyPreviewFrame(media,metadata,environment){
  if(!metadata||metadata.environment!==environment||metadata.view!==media.preview_view||metadata.revision!==media.revision)
    throw Error('预览帧身份或修订不匹配，等待当前请求的画面');
  if(media.renderer_id&&metadata.renderer_id!==media.renderer_id)throw Error('渲染服务身份已变化，丢弃旧画面');
  if(media.preview_view!=='mesh'&&metadata.view_revision!==media.view_revision)throw Error('观察视角修订不匹配，丢弃旧画面');
  if(media.preview_view==='mesh'&&(metadata.camera!==media.camera||metadata.candidate_id!==media.candidate_id))
    throw Error('叠加相机或标定候选不匹配，丢弃旧画面');
  if(!Number.isFinite(metadata.source_wall_ns)||metadata.source_wall_ns<=0)throw Error('预览帧缺少可核验的源时间');
  return metadata;
}

export function createWorkbench({document:doc, fetch:request, location, performance:clock, URL:urls, crypto, setInterval, clearInterval, setTimeout, clearTimeout}) {
  const el = id => doc.getElementById(id);
  const setText = (id, value) => { el(id).textContent = textValue(value); };
  let context = {environment:'fake', business:'calibration'}, tab = 'calibration';
  let state = null, generation = 0, revision = -1, receivedAt = 0, online = false, polling = null;
  let requestSequence = 0, displayedRequest = 0, reviewKey = '', manualKey = '', replayKey = '', stopped = false;
  let waitingForStateNotice=true;
  const pending = new Map(), mediaSlots = new Map(), timers = [], invalidatedReviews = new Set();
  let resultActions=[],queueActions=[],replayControlsKey='',manualQueueKey='';
  let calibrationKey='',calibrationPlanKey='',resultsKey='';
  let previewView='target',meshCamera='head',meshCandidate='',meshOptionsKey='',meshStateKey='',previewRenderer=null;
  let overview={yaw:-Math.PI/2,pitch:.4,zoom:1,pan_y:0},overviewDirty=false,overviewSending=false,overviewPointer=null;
  const displayRequests=new Map(),previewFloors=new Map(),previewErrors=new Map(),planPreviewButtons=[];
  const jointInputs=role=>[...el('joint-inputs').querySelectorAll('input')].filter(input=>input.dataset.role===role);
  const key = () => context.environment + '/' + context.business;
  const options = () => ({fresh:online && clock.now()-receivedAt <= 3000, pending:[...pending.values()].some(p=>p.context === key()), clearance:el('clearance').checked, supported:el('supported').checked});
  const permission = action => ['execute','manual_execute'].includes(action)&&invalidatedReviews.has(key()+'/'+state?.workflow?.review?.id)
    ? {allowed:false,reason:'此审阅已失效；等待后端生成新的执行审阅'} : actionPermission(state, action, {...options(),...(DISPLAY_ACTIONS.includes(action)?{pending:false}:{})});
  const make = (tag, content, className) => { const node = doc.createElement(tag); if (content !== undefined) node.textContent = textValue(content); if(className)node.className=className; return node; };
  function notice(message, active = false, waitingForState = false) { waitingForStateNotice=waitingForState;setText('request-status',message); el('request-status').classList.toggle('pending',active); }
  function gate(id, action, extraReason = '') {
    const p = permission(action), reason = extraReason || p.reason;
    el(id).disabled = !!reason || !p.allowed;
    el(id).setAttribute('aria-disabled',String(el(id).disabled));
    setText(id+'-reason',reason);
  }
  function renderGates() {
    const primary = state?.workflow?.primary_action;
    setText('primary',primary?.label || '等待可用操作');
    gate('primary',primary?.id || 'unknown',primary ? '' : '等待后端提供当前阶段主动作');
    for (const action of ['handoff','stop','connect','cancel','recover']) gate(action,action);
    if ([...pending.values()].some(p=>p.action==='stop' && p.context===key())) {
      el('stop').disabled=true; setText('stop-reason','停止请求已发送，等待停止回执');
    }
    const holder = state?.runtime?.holder;
    const target = holder === context.business ? (context.business === 'calibration' ? 'collection' : 'calibration') : context.business;
    setText('handoff','交接给'+BUSINESS[target]); el('handoff').dataset.target = target;
    const preview=state?.preview,displayBusy=view=>displayRequests.has(view)||displayRequests.has('recovery');
    gate('mesh',preview?'preview_mesh':'mesh',preview&&displayBusy('mesh')?'叠加设置正在更新':preview&&permission('preview_mesh').allowed&&!meshCandidate&&!preview.mesh?.requested?'请选择与相机匹配的标定候选':'');
    gate('manual-preview','manual_preview',manualReady() ? '' : '尚无完整关节定义、限位与实测反馈');
    gate('show-target','preview_target',displayBusy('target')?'目标图像请求正在处理':manualReady()?'':'尚无完整关节定义、限位与实测反馈');
    const manualDisabled = el('manual-preview').disabled&&el('show-target').disabled;
    for(const input of el('joint-inputs').querySelectorAll('input')) input.disabled=manualDisabled;
    for(const id of ['jaw-left','jaw-right','jaw-speed','manual-speed','apply-manual-speed','use-feedback','zero-preview']) el(id).disabled=manualDisabled;
    const queue=state?.manual?.queue;
    gate('queue-add','manual_queue',manualReady()?'':'等待有效的实测姿态与关节定义');
    gate('queue-preview','manual_queue_preview',queue?.available&&queue.waypoints?.length?'':queue?.reason||'标定动作队列为空');
    gate('queue-clear','manual_queue',queue?.available&&queue.waypoints?.length?'':queue?.reason||'标定动作队列为空');
    for(const button of queueActions)button.disabled=!permission('manual_queue').allowed;
    el('support-label').hidden = context.environment !== 'physical' || state?.runtime?.connected === true;
    el('connect').hidden = primary?.id === 'connect';
    setText('select-result-reason',context.business==='calibration'?permission('select_result').reason:'');
    for(const row of resultActions){const p=permission(row.action);row.button.disabled=!p.allowed||row.allowed===false;row.reason.textContent=row.allowed===false?row.ruleReason:p.reason;}
    const replay=state?.workflow?.replay_context;
    gate('replay-open','select_result',replay?.frame_count>0?'':'当前批次尚无可回放帧');
    const cameraChanged=!!replay&&el('replay-camera').value!==replay.camera;
    el('replay-overlay').disabled=cameraChanged||!replay?.overlay_available||!permission('select_result').allowed;
    setText('replay-overlay-reason',cameraChanged?'先查看所选相机，再启用匹配结果的叠加':replay?.overlay_available?'':replay?'历史叠加需要与当前相机匹配的头部或双腕结果':'');
    el('collection-tools').hidden=!state?.workflow?.replan_action;
    gate('replan','generate_plan',state?.workflow?.replan_action?.allowed===false?(state.workflow.replan_action.reason||'当前不能重新规划'):'');
    const tools=state?.workflow?.calibration_tools,selectedKind=el('calibration-kind').value,selectedHead=el('calibration-head').value;
    const dirty=selectedKind!==tools?.workflow?.kind || selectedKind==='wrist'&&selectedHead!==tools?.workflow?.head_result_id;
    const reason=!tools?.available?'标定服务未配置':dirty?'先应用标定类型与头部依赖':'';
    gate('configure-calibration','configure_calibration',selectedKind==='wrist'&&!selectedHead?'请选择已通过的头部结果':'');
    gate('wrist-redo','generate_plan',!tools?.available?'标定服务未配置':state?.workflow?.busy?'当前标定任务正在进行':!selectedHead?'请选择已通过的头部结果':'');
    gate('plan-batch','plan_batch',reason);
    gate('preview-batch','preview_batch',reason||(!tools?.waypoints?.length?'请先生成标定采样计划':''));
    gate('apply-fit-source','fit_source',reason||(!el('fit-source').value?'请选择当前类型的采集批次':''));
    gate('clear-fit-source','fit_source',reason);
    gate('fit-batch','fit_batch',reason||(!tools?.fit_source_id?'请先明确设置求解来源':''));
    el('calibration-kind').disabled=!permission('configure_calibration').allowed;
    el('calibration-head').disabled=!permission('configure_calibration').allowed;
    el('fit-source').disabled=!permission('fit_source').allowed;
    gate('overview-reset','preview_overview',displayBusy('overview')?'视角正在更新':'');
    for(const id of ['overview-preset','overview-zoom'])el(id).disabled=!permission('preview_overview').allowed||displayRequests.has('recovery');
    for(const id of ['mesh-camera','mesh-candidate'])el(id).disabled=!permission('preview_mesh').allowed||displayBusy('mesh');
    for(const button of planPreviewButtons)button.disabled=!permission('preview_target').allowed||displayBusy('target');
    gate('preview-recover','preview_recover',!preview?.recovery?.allowed?(preview?.recovery?.reason||'当前没有后端允许的渲染恢复操作'):displayRequests.has('recovery')?'渲染服务恢复请求正在处理':'');
  }
  function manualReady() {
    const m=state?.manual;
    return m?.joint_names?.length===12 && m.positions?.length===12 && m.positions.every(Number.isFinite) && m.limits?.length===12 && m.limits.every(v=>Array.isArray(v)&&v.length===2&&v.every(Number.isFinite)&&v[0]<=v[1]) && m.jaw_gaps_mm?.length===2 && m.jaw_gaps_mm.every(Number.isFinite);
  }
  function renderManual() {
    const m=state?.manual, signature=JSON.stringify([key(),m?.joint_names,m?.limits]);
    if(signature===manualKey)return; manualKey=signature;
    el('joint-inputs').replaceChildren();
    if(!manualReady())return;
    m.joint_names.forEach((name,index)=>{
      const label=make('label',name+' · rad'), input=make('input'); input.type='number';input.step='0.01';input.id='joint-'+index;
      input.dataset.role='position';
      input.min=m.limits[index][0];input.max=m.limits[index][1];input.value=m.positions[index];input.disabled=true;input.setAttribute('aria-label',name+' 目标 rad');
      input.addEventListener('input',editManual);
      const speed=make('input');speed.type='number';speed.min='0.001';speed.step='0.01';speed.value=m.default_joint_speed_rad_s??.2;speed.dataset.role='speed';speed.setAttribute('aria-label',name+' 速度 rad/s');speed.addEventListener('input',editManual);
      label.append(input,make('small','速度 · rad/s'),speed);el('joint-inputs').append(label);
    });
    el('jaw-left').value=m.jaw_gaps_mm[0];el('jaw-right').value=m.jaw_gaps_mm[1];
    el('manual-speed').value=m.default_joint_speed_rad_s??.2;el('jaw-speed').value=m.default_jaw_speed_mm_s??10;
  }
  function editManual(){
    if(state?.workflow?.primary_action?.id==='manual_execute'&&state.workflow.review?.id)invalidatedReviews.add(key()+'/'+state.workflow.review.id);
    if(state?.preview){previewErrors.set('target','人工姿态草稿已改变，请重新查看目标图像。');renderMedia();}
    clearReview();renderGates();
  }
  function renderQueue(){
    const queue=state?.manual?.queue,signature=JSON.stringify([key(),queue]);
    if(signature===manualQueueKey)return;manualQueueKey=signature;queueActions=[];
    const list=el('manual-queue-list');list.replaceChildren();
    setText('queue-status',queue?.available?`${queue.waypoints?.length??0} 个路径点 · 队列修订 ${queue.revision}`:queue?.reason||'未配置标定动作队列');
    for(const [index,row]of(queue?.waypoints||[]).entries()){
      const item=make('li'),q=row.q||row.q12||[],speeds=row.speeds||row.speeds12||[];
      item.append(make('strong',row.name||'路径点 '+(index+1)),make('p',`左臂 rad：${q.slice(0,6).join(' / ')}；右臂 rad：${q.slice(6).join(' / ')}`),
        make('p',`关节速度 rad/s：${speeds.join(' / ')}；夹爪 mm：${(row.jaw_gap_mm||[]).join(' / ')}；夹爪速度 ${row.jaw_speed_mm_s??'未知'} mm/s`));
      const remove=make('button','移除路径点 '+(index+1));remove.addEventListener('click',()=>send('manual_queue',{operation:'remove',index,queue_revision:queue.revision}));queueActions.push(remove);item.append(remove);list.append(item);
    }
  }
  function renderArrival(){
    const report=state?.arrival_statistics,body=el('arrival-rows');body.replaceChildren();
    if(!report||report.error){setText('arrival-status',report?.error||'标定到位统计尚不可用');setText('arrival-path','');return;}
    setText('arrival-status',`${report.robot_id||'设备身份未知'} · ${report.robot_source==='simulated'?'Fake 反馈':report.robot_source==='physical'?'物理反馈':'来源未知'} · ${report.total_count??'未知'} 个有效到位样本`);
    for(const side of ['left','right'])for(const [index,row]of(report.joints?.[side]||[]).entries()){
      const tr=make('tr');for(const value of [(side==='left'?'左':'右')+' J'+(index+1),row.count??'未知',...['mean_rad','std_rad','rms_rad','abs_p95_rad','abs_max_rad'].map(k=>Number.isFinite(row[k])?(row[k]*180/Math.PI).toFixed(3):'—')])tr.append(make('td',value));body.append(tr);
    }
    setText('arrival-path',`原始日志：${report.raw_path||'未提供'} · 汇总：${report.summary_path||'未提供'}；磁盘保留 rad，不随图像批次删除。`);
  }
  function renderDetails(id, rows) {
    const container=el(id);container.replaceChildren();
    for(const row of Array.isArray(rows)?rows:[]) {
      const entry=make('div'); entry.append(make('dt',typeof row==='object'?(row.label||row.key||'详情'):'详情'),make('dd',typeof row==='object'?(row.value??row.message??row.status):row));container.append(entry);
    }
  }
  function renderCalibration(){
    const tools=state?.workflow?.calibration_tools;
    el('calibration-tools').hidden=context.business!=='calibration';
    setText('calibration-tools-status',tools?.available?`当前类型：${tools.workflow?.kind==='wrist'?'双腕':'头部'}；头部依赖：${tools.workflow?.head_result_id||'不需要'}`:'标定服务未配置，分步规划、采集和求解暂不可用。');
    const signature=JSON.stringify([key(),tools?.workflow,tools?.heads,tools?.batches,tools?.fit_source_id]);
    if(signature!==calibrationKey){
      calibrationKey=signature;
      const select=(id,rows,value,placeholder)=>{const box=el(id);box.replaceChildren();for(const row of [{id:'',label:placeholder},...rows]){const option=make('option',row.label||row.id);option.value=row.id;box.append(option);}box.value=value||'';};
      el('calibration-kind').value=tools?.workflow?.kind||'head';
      select('calibration-head',tools?.heads||[],tools?.workflow?.head_result_id,'选择已通过的头部结果');
      select('fit-source',(tools?.batches||[]).filter(row=>(row.kind||'head')===(tools?.workflow?.kind||'head')).map(row=>({...row,label:row.id+' · '+(row.completion?.captured??'未知')+' 帧'})),tools?.fit_source_id,'选择当前类型的采集批次');
    }
    setText('fit-source-state','当前求解来源：'+(tools?.fit_source_id||'尚未选择')+'。浏览结果与历史回放不会改变求解来源。');
    const waypoints=tools?.waypoints||[],capturing=waypoints.filter(row=>row.capture!==false),counts=tools?.counts||{};
    setText('calibration-plan-summary',`${waypoints.length} 个路径点 · ${capturing.length} 个采样点 · ${waypoints.length-capturing.length} 个过渡点；当前来源：${counts.captured??counts.frames??'未知'} 帧，求解有效 ${counts.valid??'未知'}，无效 ${counts.failed??'未知'}。`);
    setText('calibration-preview-message',state?.preview?(state.preview.connected===true?'可逐姿态查看目标图像；查看只改变显示，不执行采集或机器人动作。':state.preview.status==='unconfigured'?'当前环境尚未配置目标渲染服务，重连画面不能代替环境配置。':state.preview.error||'目标渲染服务尚未连接，可在设备诊断中查看图像恢复指引。'):tools?.preview_message||'逐姿态图像预览尚未接入。');
    const planKey=JSON.stringify([key(),waypoints]);
    if(planKey!==calibrationPlanKey){
      calibrationPlanKey=planKey;const list=el('calibration-waypoints');list.replaceChildren();planPreviewButtons.length=0;
      for(const [index,row]of waypoints.entries()){
        const li=make('li'),button=make('button','查看此姿态图像 · 不执行');button.addEventListener('click',()=>showTarget(row.q,row.jaw_gap_mm));planPreviewButtons.push(button);li.append(make('strong',`${index+1}. ${row.capture===false?'仅过渡':'采样'} · ${row.name||row.id||row.kind||'姿态'}`),make('p',`关节 rad：${(row.q||[]).join(' / ')}；速度 rad/s：${(row.speeds||[]).join(' / ')}；夹爪 mm：${(row.jaw_gap_mm||[]).join(' / ')}`),button);list.append(li);
      }
      const coverage=el('calibration-coverage');coverage.replaceChildren();
      for(let cell=0;cell<9;cell++){const count=capturing.filter(row=>Array.isArray(row.cell)&&row.cell[0]*3+row.cell[1]===cell).length;coverage.append(make('div',`视野 ${Math.floor(cell/3)+1}行 ${cell%3+1}列 · ${count} 点`));}
    }
    el('calibration-coverage').hidden=tools?.workflow?.kind==='wrist'||!waypoints.some(row=>Array.isArray(row.cell));
  }
  function clearReview() { el('clearance').checked=false; }
  function renderPreview(){
    const preview=state?.preview,mesh=preview?.mesh||state?.mesh||{};
    if(preview?.renderer_id!==previewRenderer){previewRenderer=preview?.renderer_id;previewFloors.clear();previewErrors.clear();}
    const meshKey=JSON.stringify([key(),mesh.camera,mesh.candidate_id]);
    if(meshKey!==meshStateKey&&!displayRequests.has('mesh')){meshStateKey=meshKey;meshCamera=mesh.camera||meshCamera;meshCandidate=mesh.candidate_id||'';}
    el('preview-view').value=previewView;el('overview-controls').hidden=previewView!=='overview';
    el('media-preview').classList.toggle('overview',previewView==='overview');
    el('mesh-controls').hidden=!preview;el('media-mesh').hidden=!preview;
    el('mesh-camera').value=meshCamera;
    const choices=(state?.results||[]).filter(row=>row.kind===(meshCamera==='head'?'head':'wrist'));
    const optionsKey=JSON.stringify([key(),meshCamera,meshCandidate,choices.map(row=>[row.id,row.label,row.status])]);
    if(optionsKey!==meshOptionsKey){meshOptionsKey=optionsKey;const box=el('mesh-candidate');box.replaceChildren();for(const row of [{id:'',label:'选择标定候选'},...choices]){const option=make('option',row.label+(row.status?' · '+statusText(row.status):''));option.value=row.id;box.append(option);}box.value=choices.some(row=>row.id===meshCandidate)?meshCandidate:'';}
    setText('mesh-state','请求：'+(mesh.requested===true?'开启':mesh.requested===false?'关闭':'未知')+' · 后端：'+(mesh.applied===true?'已开启':mesh.applied===false?'已关闭':'未知'));
    el('mesh').checked=mesh.requested===true;
    const protocol=state?.device?.id==='protocol-fake';
    setText('preview-status',(protocol?'当前为协议 Fake（L1）：只验证控制协议与页面，不提供 PhysX 相机图像。完整 Fake 需要物理仿真、相机与渲染服务。 ':'')+(!preview||preview.status==='unconfigured'?'目标预览渲染服务尚未配置。':(preview.connected===true?'渲染服务已连接':preview.connected===false?'渲染服务未连接':'渲染连接尚未核验')+(preview.error?' · '+preview.error:'')));
    const rows=[{label:'渲染连接',value:!preview||preview.status==='unconfigured'?'尚未配置':preview.connected===true?'已连接':preview.connected===false?'未连接':'未知'},
      {label:'当前渲染原因',value:preview?.error||preview?.status||'尚无渲染状态回执'}];
    if(protocol)rows.unshift({label:'设备资格',value:'协议 Fake（L1）不提供 PhysX 相机图像；缺少这类图像属于未配置完整仿真，不能通过重连浏览器补齐。'});
    for(const media of state?.media||[])if(media.error||media.ready===false)rows.push({label:media.label||media.id||'图像通道',value:media.error||'当前画面尚未就绪'});
    for(const [view,frame]of Object.entries(preview?.frames||{}))if(frame?.error)rows.push({label:{target:'目标姿态',overview:'整机视角',mesh:'Mesh 叠加'}[view]||view,value:frame.error});
    if(preview?.recovery?.message)rows.push({label:'恢复指引',value:preview.recovery.message});
    renderDetails('image-recovery-details',rows);
    const commands=preview?.recovery?.commands;el('image-recovery-commands').hidden=!commands;
    setText('image-recovery-commands',Array.isArray(commands)?commands.join('\n'):commands||'');
  }
  function render() {
    const s=state, r=s?.runtime||{}, w=s?.workflow||{};
    setText('device-name',s?.device?.label||s?.device?.id||'机器人工作台');
    setText('holder',BUSINESS[r.holder] || (r.holder ? String(r.holder) : '未分配 / 未知'));
    setText('connection','连接：'+(r.connected===true?'已连接':r.connected===false?'未连接':'未知'));
    setText('feedback','反馈：'+(r.feedback_healthy===true?'健康':r.feedback_healthy===false?'不可用':'未知'));
    const hold=typeof r.hold==='object'&&r.hold?Object.entries(r.hold).map(([side,value])=>({left:'左',right:'右'}[side]||side)+' '+statusText(value)).join(' / '):statusText(r.hold);
    setText('hold','保持：'+hold);setText('motion','动作：'+(r.motion_status==='no_automatic_task'?'无自动任务':statusText(r.motion_status)));
    el('motion').title=r.motion_status==='no_automatic_task'?'当前没有自动动作任务；此状态不代表物理静止，仍需核对设备反馈。':'';
    setText('stop-state',r.stop_receipt?(r.stop_receipt.stop_hold_return_confirmed===true?'停止：保持调用已确认':'停止：已受理，尚未确认'):'');
    setText('task-title','当前任务 · '+BUSINESS[context.business]);setText('task-status',statusText(w.status));
    setText('run-identity','Run：'+textValue(w.id));setText('stage-title',w.title||'等待任务状态');setText('stage-id',w.stage_id||'—');
    setText('task-message',w.request?.status==='failed'?'请求失败：'+textValue(w.request.error):w.request?.status==='running'?'正在处理请求：'+textValue(w.request.action)+'。'+(w.message||''):w.message||'尚未取得当前任务回执。');
    const p=countedProgress(w.progress);el('task-progress').hidden=!p&&w.progress?.indeterminate!==true;
    if(p){el('task-progress').max=p.total;el('task-progress').value=p.current;}
    else el('task-progress').removeAttribute('value');
    setText('progress-label',p?(p.kind==='fraction'?`${(100*p.current/p.total).toFixed(1)}% · ${p.unit||'当前阶段'} · 后端工作量`:`${p.current} / ${p.total} ${p.unit||''} · 后端实际计数`):'尚无可计数进度；不估算完成比例');
    const overall=countedProgress(w.overall_progress);el('overall-panel').hidden=!overall;
    if(overall){el('overall-progress').max=overall.total;if(overall.indeterminate)el('overall-progress').removeAttribute('value');else el('overall-progress').value=overall.current;setText('overall-label',`${overall.indeterminate?'当前阶段工作量未知':'整体 '+(100*overall.current/overall.total).toFixed(1)+'%'} · ${overall.completed_stages??overall.current} / ${overall.total} 阶段已完成 · 阶段等权，不代表剩余时间`);}
    renderDetails('task-details',w.details);
    const nextReviewKey=JSON.stringify([key(),w.id,w.revision,w.review?.id]);
    if(nextReviewKey!==reviewKey){reviewKey=nextReviewKey;clearReview();}
    el('review-panel').hidden=!w.review;
    setText('review-summary',w.review?.summary||'尚无执行摘要');setText('review-id','审阅身份：'+textValue(w.review?.id));
    el('clearance-label').hidden=!w.review || !(context.environment==='physical'||w.review.physical_clearance_required);
    const outcomes=w.outcomes||{};
    for(const name of ['motion','data','simulation','task']) {
      const outcome=outcomes[name],value=outcomeStatus(outcome),target=el('outcome-'+name);
      target.textContent=statusText(value)+(outcome?.message?' · '+outcome.message:'');
      target.className=['failed','invalid'].includes(value)?'tone-bad':['passed','completed','complete','valid'].includes(value)?'tone-good':executing(value)?'tone-active':'';
    }
    const dataFailed=['failed','invalid'].includes(outcomeStatus(outcomes.data));
    const alert=[dataFailed&&executing(r.motion_status)?'录制失败，动作仍在执行；本轮数据无效。可停止任务或继续观察，不会自动重跑。':'',
      r.stop_receipt&&r.stop_receipt.stop_hold_return_confirmed!==true?'停止已受理，但保持调用尚未确认。请核对设备反馈与停止详情。':'',
      r.service_error?'设备服务异常：'+r.service_error:'',
      !online&&s?'状态连接中断；旧画面与状态仅作参考，等待重新核验。':s&&options().fresh===false?'状态已超过 3 秒未核验；操作已停用，停止请求仍独立可用。':''].filter(Boolean).join(' ');
    el('alert').hidden=!alert;setText('alert',alert);
    renderManual();renderQueue();renderArrival();renderCalibration();renderResults();renderPreview();renderDetails('diagnostics-list',s?.diagnostics);
    setText('stop-details',r.stop_receipt||'尚无停止回执；请求受理不代表停止已确认');setText('raw-state',s||'尚未核验');
    for(const id of ['results-context','diagnostics-context','footer-context'])setText(id,ENVIRONMENT[context.environment]+' · '+BUSINESS[context.business]);
    renderMedia();renderGates();renderAge();
  }
  function renderAge() {
    if(!state){setText('state-age','状态尚未核验');return;}
    const age=Math.max(0,(clock.now()-receivedAt)/1000);
    setText('state-age',(online?'最近状态':'连接中断 · 最后状态')+' '+age.toFixed(1)+' 秒前');
    for(const slot of mediaSlots.values())updateMediaLabel(slot);
  }
  function updateMediaLabel(slot) {
    const m=slot.media, age=frameAgeSeconds(m,state,clock.now()-receivedAt), stale=age===null||age>3||!online||!!slot.error;
    slot.card.classList.toggle('stale',stale);
    slot.card.querySelector('.media-meta').textContent=[ENVIRONMENT[context.environment],{actual:'实际画面',target:'目标预览',overview:'整机视角',mesh:'实际图像上的 mesh',replay:'录制回放'}[m?.kind]||'来源未知',m?.preview_view==='mesh'&&m.candidate_id?'候选 '+m.candidate_id.slice(0,12):'',age===null?'源时间未知':(age>3?'画面冻结 · ':'源帧龄 ')+age.toFixed(1)+' s',slot.error||m?.error].filter(Boolean).join(' · ');
  }
  function cancelMediaRequest(slot) {
    slot.controller?.abort();slot.controller=null;slot.version++;
    clearTimeout(slot.timer);clearTimeout(slot.timeout);slot.timer=null;slot.timeout=null;
  }
  function clearSlot(slot) {
    cancelMediaRequest(slot);if(slot.blob)urls.revokeObjectURL(slot.blob);slot.blob=null;
    slot.streamKey=null;slot.media=null;slot.etag=null;slot.headerFrame=null;slot.headerSource=null;
    slot.image.hidden=true;slot.image.removeAttribute('src');slot.empty.hidden=false;slot.error='';
  }
  function clearMedia() {for(const slot of mediaSlots.values())clearSlot(slot);}
  function previewMedia(view){
    const preview=state?.preview,frame=preview?.frames?.[view]||{},mesh=preview?.mesh||{},expected=preview?.revisions?.[view];
    let error=previewErrors.get(view)||frame.error||preview?.error||'';
    if(displayRequests.has(view)||displayRequests.has('recovery')||(previewFloors.has(view)&&expected<previewFloors.get(view)))error='等待本次只读预览的匹配画面';
    if(view!=='mesh'&&(displayRequests.has('overview')||(previewFloors.has('overview')&&preview?.revisions?.overview<previewFloors.get('overview'))))error=previewErrors.get('overview')||'等待当前观察视角的匹配画面';
    if(preview?.connected===false)error=preview.error||'渲染服务未连接，请查看设备诊断中的图像恢复指引';
    if(preview?.status==='unconfigured')error='当前环境尚未配置目标渲染服务；重新连接画面不能代替环境配置。';
    if(view==='mesh'&&!mesh.requested)error='实时 mesh 叠加已关闭；原始相机画面保持独立。';
    if(view==='mesh'&&mesh.requested&&(mesh.camera!==meshCamera||mesh.candidate_id!==meshCandidate))error='等待当前相机与所选候选的匹配画面';
    if(frame.revision!==undefined&&frame.revision!==expected)error='渲染结果仍属于旧修订，等待当前请求';
    if(view!=='mesh'&&frame.view_revision!==undefined&&frame.view_revision!==preview?.revisions?.overview)error='观察视角已更新，等待匹配的新画面';
    return {...frame,id:'preview-'+view,kind:view,preview_view:view,revision:expected,view_revision:preview?.revisions?.overview,renderer_id:preview?.renderer_id,
      ...(view==='mesh'?{camera:mesh.camera,candidate_id:mesh.candidate_id}:{}),
      ready:frame.ready===true&&Number.isInteger(expected)&&!error,error:error||(!frame.ready?'尚无此视图的画面，请先提交只读预览请求':''),poll_hz:frame.poll_hz||10};
  }
  function renderMedia() {
    const actual=(state?.media||[]).filter(m=>m.kind==='actual'), used=new Set();
    const slots=[['head',['head','base_0']],['left',['left','left_wrist_0']],['right',['right','right_wrist_0']]];
    const named=new Set(slots.flatMap(([,aliases])=>aliases));
    for(const [id,aliases]of slots){const media=actual.find(m=>aliases.includes(m.id))||actual.find(m=>!named.has(m.id)&&!used.has(m.id));if(media)used.add(media.id);loadMedia(id,media);}
    loadMedia('preview',state?.preview?previewMedia(previewView):(state?.media||[]).find(m=>m.kind==='target'));
    loadMedia('mesh',state?.preview?previewMedia('mesh'):null);
  }
  function loadMedia(id,m) {
    const card=el('media-'+id);
    let slot=mediaSlots.get(id);
    if(!slot){slot={card,image:card.querySelector('img'),empty:card.querySelector('.empty'),version:0};mediaSlots.set(id,slot);}
    if(doc.hidden || !m){clearSlot(slot);slot.empty.textContent=doc.hidden?'页面后台，画面订阅已暂停':state?.device?.id==='protocol-fake'?'协议 Fake（L1）不提供 PhysX 相机图像；完整 Fake 需要配置仿真与图像服务。':'尚无当前环境的可核验画面';updateMediaLabel(slot);return;}
    const url=safeMediaUrl(m.url,location.href);
    const streamKey=JSON.stringify([key(),m.id,m.kind,m.candidate_revision??null,m.revision??null,m.view_revision??null,m.preview_view,m.renderer_id,m.camera,m.candidate_id,url]);
    if(slot.streamKey!==streamKey){clearSlot(slot);slot.streamKey=streamKey;slot.url=url;}
    slot.media={...m,...(slot.headerFrame!==null?{frame_id:slot.headerFrame}:{}),...(slot.headerSource!==null?{source_wall_ns:slot.headerSource}:{})};
    if(!m.ready||!url){cancelMediaRequest(slot);slot.error=m.error||(!url?'尚无有效的同源画面地址':'画面尚未就绪');slot.empty.textContent=slot.error;if(m.preview_view&&slot.blob){urls.revokeObjectURL(slot.blob);slot.blob=null;slot.image.removeAttribute('src');}if(!slot.blob){slot.image.hidden=true;slot.empty.hidden=false;}updateMediaLabel(slot);return;}
    updateMediaLabel(slot);
    if(!slot.controller&&!slot.timer)void nextMediaFrame(slot,slot.version);
  }
  async function nextMediaFrame(slot,version) {
    if(version!==slot.version||stopped||doc.hidden||!slot.media?.ready)return;
    const started=clock.now(),controller=new AbortController(),at=generation;slot.controller=controller;
    let blobUrl,failed=false,timedOut=false;
    slot.timeout=setTimeout(()=>{timedOut=true;controller.abort();},2500);
    try {
      const response=await request(slot.url,{signal:controller.signal,cache:'no-store',headers:slot.etag?{'If-None-Match':slot.etag}:{}});
      if(version!==slot.version||at!==generation||controller.signal.aborted)return;
      if(response.status===304){slot.error='';return;}
      if(!response.ok)throw Error('画面接口 '+response.status);
      const metadata=slot.media.preview_view?verifyPreviewFrame(slot.media,JSON.parse(response.headers?.get('X-Frame-Metadata')||'null'),context.environment):null;
      const blob=await response.blob();
      if(version!==slot.version||at!==generation||controller.signal.aborted)return;
      const source=metadata?.source_wall_ns??response.headers?.get('X-Source-Wall-Ns'),frame=metadata?.frame_id??response.headers?.get('X-Frame-Id');
      const sourceNs=source===null||source===undefined||source===''?null:Number(source);
      blobUrl=urls.createObjectURL(blob);const decoded=make('img');decoded.src=blobUrl;
      if(decoded.decode)await decoded.decode();
      if(version!==slot.version||at!==generation||controller.signal.aborted){urls.revokeObjectURL(blobUrl);blobUrl=null;return;}
      // Only a decoded source frame establishes liveness; HTTP arrival and 304 do not.
      if(Number.isFinite(sourceNs)){slot.headerSource=sourceNs;slot.media.source_wall_ns=sourceNs;}
      if(frame!==null&&frame!==undefined){slot.headerFrame=frame;slot.media.frame_id=frame;}
      slot.etag=response.headers?.get('ETag')||null;slot.image.src=blobUrl;
      if(slot.blob)urls.revokeObjectURL(slot.blob);slot.blob=blobUrl;blobUrl=null;slot.error='';slot.image.hidden=false;slot.empty.hidden=true;
    } catch(error) {
      if(blobUrl)urls.revokeObjectURL(blobUrl);
      if(version===slot.version&&at===generation&&(!controller.signal.aborted||timedOut)){
        failed=true;slot.error=timedOut?'画面请求超时':'画面读取失败：'+error.message;
        if(!slot.blob){slot.empty.textContent=slot.error;slot.empty.hidden=false;slot.image.hidden=true;}
      }
    } finally {
      if(version===slot.version){
        clearTimeout(slot.timeout);slot.timeout=null;slot.controller=null;updateMediaLabel(slot);
        if(!stopped&&!doc.hidden&&at===generation&&slot.media?.ready){
          const hz=Number.isFinite(slot.media.poll_hz)&&slot.media.poll_hz>0?Math.min(60,slot.media.poll_hz):25;
          slot.timer=setTimeout(()=>{slot.timer=null;void nextMediaFrame(slot,version);},failed?250:Math.max(0,1000/hz-(clock.now()-started)));
        }
      }
    }
  }
  function renderResults() {
    const installed=state?.workflow?.calibration_tools?.installed;
    el('installed-group').hidden=context.business!=='calibration';
    renderDetails('installed-details',installed?Object.entries(installed).map(([label,value])=>({label:({head_id:'头部结果',wrist_id:'双腕结果',head_version:'头部版本',wrist_version:'双腕版本',verified:'active 文件校验',problems:'校验问题',path:'安装目录',geometry_id:'几何身份',installed_at:'安装时间',acceptance:'验收依据',notes:'说明',environment:'环境'}[label]||label),value:label==='verified'?(value===true?'通过':value===false?'未通过':'未知'):value})):[{label:'当前安装组',value:'尚未安装'}]);
    const box=el('results-list');
    const results=state?.results||[];
    const signature=JSON.stringify([key(),results]);
    if(signature!==resultsKey){resultsKey=signature;box.replaceChildren();resultActions=[];
    if(!results.length)box.append(make('p','当前业务尚无结果回执。','muted'));
    const addAction=(container,label,action,body,rule={})=>{const cell=make('div',undefined,'action-cell'),button=make('button',label),reason=make('p','', 'action-reason');button.addEventListener('click',()=>send(action,body));cell.append(button,reason);container.append(cell);resultActions.push({button,reason,action,allowed:rule.allowed,ruleReason:rule.reason||'此结果当前不可安装'});};
    for(const result of results){
      const row=make('article',undefined,'result-row'),label=make('div'),actions=make('div',undefined,'result-actions');
      label.append(make('h3',result.label||result.id),make('p',({head:'头部标定',wrist:'双腕标定',capture:'采集批次',recording:'录制视频'}[result.kind]||result.kind||'结果')+' · '+statusText(result.status)+' · '+result.id));
      if(result.kind!=='recording')addAction(actions,'查看此结果','select_result',{result_id:result.id});
      else actions.append(make('p','下方独立回放','muted'));
      if(result.install){const install=result.install;label.append(make('p','安装组：头部 '+String(install.head_result_id||result.id).slice(0,12)+' + 双腕 '+String(install.wrist_result_id||'尚无').slice(0,12)));addAction(actions,install.installed?'当前有效标定':'安装这组头部与双腕','install',{result_id:result.id},install);}
      if(result.details){
        const details=make('details'),list=make('dl',undefined,'detail-list');details.append(make('summary','质量指标、依赖与来源'));
        for(const [field,value]of Object.entries(result.details)){const entry=make('div');entry.append(make('dt',({metrics_rectified_px:'贴纸像素误差',mesh_metrics:'Mesh 留出指标',acceptance:'验收结论',optimizer:'求解收敛',left:'左腕结果',right:'右腕结果',head_result_id:'依赖头部结果',group:'头部与双腕组',batch:'来源批次',version:'版本',environment:'环境',robot_source:'设备来源',simulation_only:'仅仿真',capture_coverage:'采集覆盖',file:'结果文件',objective:'求解目标',completion:'采集完整性'}[field]||field)),make('dd',value));list.append(entry);}
        details.append(list);label.append(details);
      }
      const download=safeMediaUrl(result.download_url,location.href);
      if(download){const link=make('a','下载版本 JSON');link.href=download;link.download='';actions.append(link);}
      row.append(label,actions);box.append(row);
    }
    }
    const view=state?.workflow?.replay_context,viewKey=JSON.stringify([key(),view]);
    el('replay-controls').hidden=!view;
    if(view&&viewKey!==replayControlsKey){
      replayControlsKey=viewKey;el('replay-camera').replaceChildren();
      for(const camera of view.cameras||[]){const option=make('option',{head:'头部',left:'左腕',right:'右腕'}[camera]||camera);option.value=camera;el('replay-camera').append(option);}
      el('replay-camera').value=view.camera;el('replay-index').value=view.frame_index;el('replay-index').max=Math.max(0,view.frame_count-1);el('replay-overlay').checked=view.overlay===true;
      setText('replay-context',`${view.batch_id} · 第 ${view.frame_count?view.frame_index+1:0} / ${view.frame_count} 帧${view.message?' · '+view.message:''}`);
    }
    const allMedia=[...(state?.media||[]),...(state?.workflow?.replay||[]),...results.map(r=>r.media).filter(Boolean)];
    const media=[...new Map(allMedia.filter(m=>m.kind==='replay').map(m=>[m.id||m.url,m])).values()],nextKey=JSON.stringify([key(),media]);
    if(nextKey===replayKey)return;replayKey=nextKey;const replay=el('replay-media');replay.replaceChildren();
    for(const m of media){const url=safeMediaUrl(m.url,location.href);if(!m.ready||!url)continue;const figure=make('figure'),video=/\.(mp4|webm)(\?|$)/i.test(url)||m.mime_type?.startsWith('video/');const node=make(video?'video':'img');node.src=url;if(video){node.controls=true;node.preload='metadata';node.playsInline=true;}else node.alt=m.label||'录制回放';figure.append(make('figcaption',(m.label||'录制回放')+' · 回放'),node);replay.append(figure);}
  }
  async function poll() {
    if(stopped||polling)return;
    const controller=new AbortController(),at=generation,requestedContext={...context};polling=controller;
    try {
      const query=new URLSearchParams(requestedContext),response=await request('/api/state?'+query,{cache:'no-store',signal:controller.signal});
      if(!response.ok)throw Error('状态接口 '+response.status);const incoming=await response.json();
      if(at!==generation||controller.signal.aborted)return;
      if(incoming.schema!=='robot-workbench-ui.v1'||!Number.isFinite(incoming.revision)||incoming.device?.environment!==context.environment)throw Error('状态契约或设备环境不匹配');
      if(incoming.revision<revision)return;
      revision=incoming.revision;state=incoming;receivedAt=clock.now();online=true;
      if(waitingForStateNotice)notice('设备与任务状态已更新。');
      render();
    } catch(error) {if(at===generation&&!controller.signal.aborted){online=false;clearReview();notice('状态核验失败：'+error.message);render();}}
    finally{if(polling===controller)polling=null;}
  }
  function barrier() {
    generation++;polling?.abort();polling=null;clearReview();
    for(const slot of mediaSlots.values())cancelMediaRequest(slot);
  }
  async function sendDisplay(action,extra={}){
    const allowed=permission(action),view={preview_target:'target',preview_overview:'overview',preview_mesh:'mesh',preview_recover:'recovery'}[action];
    if(!allowed.allowed){notice(allowed.reason);return;}
    if(displayRequests.has(view)||displayRequests.has('recovery'))return;
    if(action==='preview_recover'&&state?.preview?.recovery?.allowed!==true){notice(state?.preview?.recovery?.reason||'当前后端未允许恢复渲染服务');return;}
    const requestContext=key(),controller=new AbortController(),sequence=++requestSequence;
    displayedRequest=sequence;displayRequests.set(view,controller);previewErrors.delete(view);
    const affected=view==='recovery'?['target','overview','mesh']:[view];
    for(const name of affected){const slot=mediaSlots.get(name==='mesh'?'mesh':'preview');if(slot)clearSlot(slot);if(view!=='recovery')previewFloors.set(name,(state.preview?.revisions?.[name]??0)+1);}
    renderMedia();renderGates();notice(view==='recovery'?'已请求恢复本会话的渲染服务；机器人控制保持独立。':'正在提交只读显示请求，不执行机器人动作。',true);
    try{
      const response=await request('/api/actions/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':state.csrf},
        body:JSON.stringify({environment:context.environment,business:context.business,request_id:crypto.randomUUID(),...extra}),signal:controller.signal});
      const result=await response.json();if(!response.ok)throw Error(result.error||result.message||'显示请求被拒绝');
      if(requestContext!==key()||controller.signal.aborted)return;
      if(view==='recovery'){previewFloors.clear();previewErrors.clear();}
      else{if(!Number.isInteger(result.client_revision))throw Error('后端未返回可核验的预览修订');previewFloors.set(view,result.client_revision);}
      if(displayedRequest===sequence)notice(result.message||'只读显示请求已受理，等待对应修订的画面。');
    }catch(error){if(requestContext===key()&&!controller.signal.aborted){previewErrors.set(view,error.message);if(displayedRequest===sequence)notice('只读显示请求失败：'+error.message);}}
    finally{if(displayRequests.get(view)===controller)displayRequests.delete(view);if(requestContext===key()){renderMedia();renderGates();void poll();}}
  }
  function showTarget(q12,jaw_gap_mm){
    if(!Array.isArray(q12)||q12.length!==12||!q12.every(Number.isFinite)||!Array.isArray(jaw_gap_mm)||jaw_gap_mm.length!==2||!jaw_gap_mm.every(value=>Number.isFinite(value)&&value>=0&&value<=83)){notice('目标图像需要12个有效关节位置及0–83 mm夹爪开度');return;}
    previewView='target';el('preview-view').value=previewView;el('overview-controls').hidden=true;
    return sendDisplay('preview_target',{q12,jaw_gap_mm});
  }
  async function sendOverview(){
    overviewDirty=true;if(overviewSending)return;overviewSending=true;
    try{while(overviewDirty){overviewDirty=false;await sendDisplay('preview_overview',{...overview});}}
    finally{overviewSending=false;}
  }
  function setOverviewPreset(){
    Object.assign(overview,({front:{yaw:-Math.PI/2,pitch:.4},left:{yaw:-Math.PI/2-.7,pitch:.45},right:{yaw:-Math.PI/2+.7,pitch:.45},top:{yaw:-Math.PI/2,pitch:1.5}}[el('overview-preset').value]||{yaw:-Math.PI/2,pitch:.4}),{zoom:1,pan_y:0});
    el('overview-zoom').value=1;void sendOverview();
  }
  function sendMesh(){
    if(!state?.preview)return send('mesh',{enabled:el('mesh').checked});
    if(el('mesh').checked&&!meshCandidate){notice('请选择与当前相机匹配的标定候选');el('mesh').checked=state.preview.mesh?.requested===true;renderGates();return;}
    return sendDisplay('preview_mesh',{enabled:el('mesh').checked,camera:meshCamera,candidate_id:meshCandidate||null});
  }
  async function reconnectImages(){
    clearMedia();await poll();renderMedia();notice(online?'已重新打开浏览器画面订阅；连接与源帧状态请查看图像诊断。':'状态连接仍未恢复，请按图像诊断中的后端指引检查。');
  }
  async function send(action,extra={}) {
    const allowed=permission(action);
    if(!allowed.allowed){notice(allowed.reason);renderGates();return;}
    if(action==='stop'&&[...pending.values()].some(p=>p.action==='stop'&&p.context===key()))return;
    const requestId=crypto.randomUUID?crypto.randomUUID():Array.from(crypto.getRandomValues(new Uint32Array(4)),n=>n.toString(16).padStart(8,'0')).join('');
    const body={environment:context.environment,business:context.business,request_id:requestId,...extra};
    if(action==='preview_batch'&&body.queue_revision===undefined)body.queue_revision=state?.workflow?.calibration_tools?.queue_revision;
    if(action==='fit_batch'&&body.batch_id===undefined)body.batch_id=state?.workflow?.calibration_tools?.fit_source_id;
    if(['execute','manual_execute'].includes(action)){body.review_id=state.workflow.review.id;body.physical_clearance=context.environment==='physical'||state.workflow.review.physical_clearance_required?el('clearance').checked:true;}
    if(action==='connect'&&context.environment==='physical')body.arms_supported=el('supported').checked;
    const csrf=state.csrf,sequence=++requestSequence,requestContext=key(),controller=new AbortController();
    if(state.workflow?.review?.id&&['stop','cancel','handoff','generate_plan','manual_preview','manual_queue','manual_queue_preview','manual_execute','execute','recover','install',...CALIBRATION_TOOLS].includes(action))
      invalidatedReviews.add(key()+'/'+state.workflow.review.id);
    barrier();const at=generation;displayedRequest=sequence;pending.set(sequence,{action,context:requestContext,controller});
    notice(action==='stop'?'正在请求停止；等待后端停止回执。':'请求已发送，等待后端受理；尚未确认完成。',true);renderGates();
    try {
      const response=await request('/api/actions/'+encodeURIComponent(action),{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:JSON.stringify(body),signal:controller.signal});
      const result=await response.json();if(!response.ok)throw Error(result.message||result.error||result.reason||'请求被拒绝');
      if(requestContext===key()&&sequence===displayedRequest)notice(result.message||(action==='stop'?'停止请求已受理；以停止回执为准。':'请求已受理；以任务回执为准。'));
    } catch(error) {if(requestContext===key()&&sequence===displayedRequest&&!controller.signal.aborted)notice('请求结果未确认：'+error.message+'；不会自动重试，请核对状态。');}
    finally {pending.delete(sequence);if(requestContext===key()){el('supported').checked=false;renderGates();if(at===generation)void poll();}}
  }
  function switchContext(next) {
    if(next.environment===context.environment&&next.business===context.business)return;
    barrier();for(const p of pending.values())p.controller.abort();pending.clear();clearMedia();
    for(const controller of displayRequests.values())controller.abort();displayRequests.clear();previewFloors.clear();previewErrors.clear();
    previewView='target';meshCamera='head';meshCandidate='';meshStateKey='';meshOptionsKey='';overviewDirty=false;overviewPointer=null;
    context=next;state=null;revision=-1;online=false;reviewKey='';manualKey='';manualQueueKey='';replayKey='';replayControlsKey='';el('supported').checked=false;
    notice('正在核验'+ENVIRONMENT[context.environment]+' / '+BUSINESS[context.business]+'；浏览不会交接控制权。',false,true);render();void poll();
  }
  function selectTab(next) {
    tab=next;for(const button of doc.querySelectorAll('[data-tab]')){if(button.dataset.tab===next)button.setAttribute('aria-current','page');else button.removeAttribute('aria-current');}
    el('work-pane').hidden=!BUSINESS[next];el('results-pane').hidden=next!=='results';el('diagnostics-pane').hidden=next!=='diagnostics';
    el('image-recovery').hidden=next!=='diagnostics';
    if(BUSINESS[next])switchContext({...context,business:next});
  }
  function manualPreview(queueAdd=false) {
    try {
      if(!manualReady())throw Error('关节定义或实测反馈尚未完整');
      const inputs=jointInputs('position'),q12=inputs.map(input=>Number(input.value));
      if(inputs.length!==12||inputs.some((input,i)=>input.value===''||!Number.isFinite(q12[i])||q12[i]<Number(input.min)||q12[i]>Number(input.max)))throw Error('关节目标必须是各自限位内的有限数值');
      const values=[el('jaw-left').value,el('jaw-right').value],jaw_gap_mm=values.map(Number);
      if(values.some((value,i)=>value===''||!Number.isFinite(jaw_gap_mm[i])||jaw_gap_mm[i]<0||jaw_gap_mm[i]>83))throw Error('夹爪目标必须在 0–83 mm 之间');
      const speedInputs=jointInputs('speed'),speeds12=speedInputs.map(input=>Number(input.value)),jaw_speed_mm_s=Number(el('jaw-speed').value);
      if(speeds12.length!==12||speedInputs.some((input,i)=>input.value===''||!Number.isFinite(speeds12[i])||speeds12[i]<=0)||!el('jaw-speed').value||!Number.isFinite(jaw_speed_mm_s)||jaw_speed_mm_s<=0)throw Error('关节与夹爪速度必须为有限正数');
      el('manual-error').hidden=true;
      if(queueAdd)void send('manual_queue',{operation:'add',queue_revision:state.manual.queue?.revision,waypoint:{q:q12,speeds:speeds12,jaw_gap_mm,jaw_speed_mm_s}});
      else void send('manual_preview',{q12,speeds12,jaw_gap_mm,jaw_speed_mm_s});
    } catch(error){setText('manual-error',error.message);el('manual-error').hidden=false;}
  }
  function start() {
    el('environment').addEventListener('change',()=>switchContext({...context,environment:el('environment').value}));
    for(const button of doc.querySelectorAll('[data-tab]'))button.addEventListener('click',()=>selectTab(button.dataset.tab));
    el('primary').addEventListener('click',()=>{if(state?.workflow?.primary_action)void send(state.workflow.primary_action.id);});
    el('replan').addEventListener('click',()=>send('generate_plan',{replan:true}));
    for(const id of ['calibration-kind','calibration-head','fit-source'])el(id).addEventListener('change',renderGates);
    el('configure-calibration').addEventListener('click',()=>send('configure_calibration',{kind:el('calibration-kind').value,head_result_id:el('calibration-kind').value==='wrist'?el('calibration-head').value:null}));
    el('wrist-redo').addEventListener('click',()=>send('generate_plan',{mode:'wrist',head_result_id:el('calibration-head').value}));
    el('plan-batch').addEventListener('click',()=>send('plan_batch'));
    el('preview-batch').addEventListener('click',()=>send('preview_batch',{queue_revision:state?.workflow?.calibration_tools?.queue_revision}));
    el('apply-fit-source').addEventListener('click',()=>send('fit_source',{batch_id:el('fit-source').value}));
    el('clear-fit-source').addEventListener('click',()=>send('fit_source',{batch_id:null}));
    el('fit-batch').addEventListener('click',()=>send('fit_batch'));
    el('replay-open').addEventListener('click',()=>{
      const view=state?.workflow?.replay_context,index=Number(el('replay-index').value);
      if(!view||el('replay-index').value===''||!Number.isInteger(index)||index<0||index>=view.frame_count){notice('请选择当前批次内的有效帧');return;}
      void send('select_result',{result_id:view.result_id,frame_index:index,camera:el('replay-camera').value,overlay:el('replay-overlay').checked});
    });
    el('replay-camera').addEventListener('change',()=>{el('replay-overlay').checked=false;renderGates();});
    for(const action of ['stop','connect','cancel','recover'])el(action).addEventListener('click',()=>send(action));
    el('handoff').addEventListener('click',()=>send('handoff',{target_business:el('handoff').dataset.target}));
    el('preview-view').addEventListener('change',()=>{previewView=el('preview-view').value==='overview'?'overview':'target';renderPreview();renderMedia();renderGates();if(previewView==='overview'&&!state?.preview?.frames?.overview?.ready)void sendOverview();});
    el('show-target').addEventListener('click',()=>{
      const joints=jointInputs('position'),jaws=[el('jaw-left'),el('jaw-right')];
      if([...joints,...jaws].some(input=>String(input.value).trim()==='')||joints.some(input=>Number(input.value)<Number(input.min)||Number(input.value)>Number(input.max))){notice('请填写限位内的完整目标姿态与夹爪开度');return;}
      return showTarget(joints.map(input=>Number(input.value)),jaws.map(input=>Number(input.value)));
    });
    el('overview-preset').addEventListener('change',setOverviewPreset);
    el('overview-reset').addEventListener('click',()=>{el('overview-preset').value='front';setOverviewPreset();});
    el('overview-zoom').addEventListener('input',()=>{overview.zoom=Math.max(.4,Math.min(4,Number(el('overview-zoom').value)));void sendOverview();});
    const overviewImage=el('media-preview').querySelector('img');
    overviewImage.addEventListener('pointerdown',event=>{if(previewView!=='overview'||!permission('preview_overview').allowed)return;overviewPointer={x:event.clientX,y:event.clientY};overviewImage.setPointerCapture?.(event.pointerId);});
    overviewImage.addEventListener('pointermove',event=>{if(!overviewPointer||previewView!=='overview')return;overview.yaw=((overview.yaw-(event.clientX-overviewPointer.x)*.006+3*Math.PI)%(2*Math.PI))-Math.PI;overview.pan_y=Math.max(-2,Math.min(2,overview.pan_y+(event.clientY-overviewPointer.y)/(overviewImage.clientHeight||240)));overviewPointer={x:event.clientX,y:event.clientY};void sendOverview();});
    for(const event of ['pointerup','pointercancel'])overviewImage.addEventListener(event,()=>{overviewPointer=null;});
    overviewImage.addEventListener('wheel',event=>{if(previewView!=='overview'||!permission('preview_overview').allowed)return;event.preventDefault();overview.zoom=Math.max(.4,Math.min(4,overview.zoom*Math.exp(-event.deltaY*.001)));el('overview-zoom').value=overview.zoom;void sendOverview();},{passive:false});
    el('mesh-camera').addEventListener('change',()=>{meshCamera=el('mesh-camera').value;meshCandidate='';el('mesh').checked=false;renderPreview();el('mesh').checked=false;renderMedia();void sendMesh();});
    el('mesh-candidate').addEventListener('change',()=>{meshCandidate=el('mesh-candidate').value;renderMedia();void sendMesh();});
    el('mesh').addEventListener('change',sendMesh);
    el('reconnect-images').addEventListener('click',reconnectImages);
    el('preview-recover').addEventListener('click',()=>sendDisplay('preview_recover'));
    for(const id of ['clearance','supported'])el(id).addEventListener('change',renderGates);
    for(const id of ['jaw-left','jaw-right','jaw-speed'])el(id).addEventListener('input',editManual);
    el('manual-preview').addEventListener('click',()=>manualPreview());
    el('queue-add').addEventListener('click',()=>manualPreview(true));
    el('queue-preview').addEventListener('click',()=>send('manual_queue_preview',{queue_revision:state?.manual?.queue?.revision}));
    el('queue-clear').addEventListener('click',()=>send('manual_queue',{operation:'clear',queue_revision:state?.manual?.queue?.revision}));
    el('apply-manual-speed').addEventListener('click',()=>{const speed=Number(el('manual-speed').value);if(!Number.isFinite(speed)||speed<=0){notice('统一关节速度必须为正数');return;}for(const input of jointInputs('speed'))input.value=speed;editManual();});
    el('use-feedback').addEventListener('click',()=>{if(!manualReady())return;jointInputs('position').forEach((input,i)=>input.value=state.manual.positions[i]);el('jaw-left').value=state.manual.jaw_gaps_mm[0];el('jaw-right').value=state.manual.jaw_gaps_mm[1];editManual();});
    el('zero-preview').addEventListener('click',()=>{for(const input of jointInputs('position'))input.value=0;el('jaw-left').value=83;el('jaw-right').value=83;editManual();});
    doc.addEventListener('visibilitychange',()=>{if(doc.hidden)clearMedia();else{renderMedia();void poll();}});
    render();void poll();timers.push(setInterval(poll,1000),setInterval(()=>{renderAge();renderGates();if(state&&options().fresh===false)render();},250));
  }
  function destroy(){stopped=true;barrier();for(const p of pending.values())p.controller.abort();pending.clear();for(const controller of displayRequests.values())controller.abort();displayRequests.clear();overviewDirty=false;clearMedia();timers.forEach(clearInterval);}
  return {start,poll,send,switchContext,selectTab,destroy,get state(){return state;},get context(){return {...context};}};
}

if(typeof window!=='undefined'&&typeof document!=='undefined'){
  const app=createWorkbench({document,fetch:window.fetch.bind(window),location:window.location,performance:window.performance,URL:window.URL,crypto:window.crypto,setInterval:window.setInterval.bind(window),clearInterval:window.clearInterval.bind(window),setTimeout:window.setTimeout.bind(window),clearTimeout:window.clearTimeout.bind(window)});
  app.start();window.addEventListener('pagehide',()=>app.destroy());
}
