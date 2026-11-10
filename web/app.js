import {requestId as newRequestId} from './session.js';
import {mountPhysicalEndpoint} from './physical_endpoint.js';

// The manager substitutes this at load time. A later status refresh must not
// upgrade an already loaded page's version; direct legacy services omit it.
const CLIENT_DEPLOYMENT_SHA='__WORKBENCH_DEPLOYMENT_SHA__';

// Select by the recorded head dependency, never by unrelated latest versions.
export function matchingMeshCandidate(rows,camera,currentId,rememberedId){
  if(rows.some(row=>row.kind==='calibration'))return rows.find(row=>row.id===currentId
    ||Object.values(row.components||{}).includes(currentId))?.id||rows.find(row=>row.mesh_selectable!==false)?.id||'';
  const byId=id=>rows.find(row=>row.id===id),headId=row=>row?.kind==='head'?row.id:row?.details?.head_result_id;
  const current=byId(currentId),dependency=headId(current),kind=camera==='head'?'head':'wrist';
  if(current&&!dependency)return current.kind===kind?current.id:'';
  const choices=rows.filter(row=>row.kind===kind&&row.mesh_selectable!==false&&(!dependency||headId(row)===dependency));
  return choices.find(row=>row.id===rememberedId)?.id||choices.find(row=>row.id===currentId)?.id||choices[0]?.id||'';
}

export function meshGroups(state){
  const physical=state?.device?.environment==='physical',seen=new Set();
  return (state?.preview?.mesh_calibrations||[]).filter(row=>row?.kind==='calibration'&&row.components?.head&&row.components?.wrist
    &&(!physical||row.details?.environment==='hardware'&&row.details?.robot_source==='physical'&&row.details?.simulation_only===false)
    &&!seen.has(row.id)&&seen.add(row.id)).map(row=>({...row,mesh_installed:row.install?.installed===true,
      label:(row.install?.installed?'当前安装 · ':'')+(row.label||'完整标定组')}));
}

export function meshRows(state){
  const source=Array.isArray(state?.preview?.mesh_candidates)?state.preview.mesh_candidates:
    [...(state?.results||[]),...(state?.preview?.diagnostic_candidates||[])];
  const physical=state?.device?.environment==='physical',seen=new Set();
  const rows=source.filter(row=>row&&['head','wrist'].includes(row.kind)&&typeof row.id==='string'&&row.id
    &&(!physical||row.details?.environment==='hardware'&&row.details?.robot_source==='physical'&&row.details?.simulation_only===false)
    &&!seen.has(row.id)&&seen.add(row.id));
  if(!physical)return rows;
  const installedWrists=new Set(rows.filter(row=>row.kind==='head'&&row.install?.installed===true).map(row=>row.install?.wrist_result_id));
  return rows.map(row=>{
    const installed=row.kind==='head'?row.install?.installed===true:installedWrists.has(row.id);
    return {...row,mesh_installed:installed,label:(installed?'当前安装 · ':'')+(row.kind==='head'?'头部 ':'双腕 ')+(row.details?.version||row.label||row.id.slice(0,12))};
  }).sort((a,b)=>Number(b.mesh_installed)-Number(a.mesh_installed));
}

const BUSINESS = {calibration: '标定', collection: '数采', inference:'推理 / Infer',research:'控制研究'};
const INFERENCE_BACKENDS={pi:[['torch','Torch'],['triton','Triton']],wam:[['torch','Torch'],['anyinfer','SO 算子加速']]};
const ENVIRONMENT = {fake: 'Fake · 开发设备', physical: 'Physical · 物理设备'};
export function recordingTaskTime(wallNs,timeZone){
  if(!Number.isFinite(wallNs)||wallNs<=0)return null;
  const date=new Date(wallNs/1e6);if(!Number.isFinite(date.getTime()))return null;
  const parts=Object.fromEntries(new Intl.DateTimeFormat('zh-CN',{month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit',hourCycle:'h23',...(timeZone?{timeZone}:{})}).formatToParts(date).map(row=>[row.type,row.value]));
  return `${parts.month}-${parts.day} ${parts.hour}:${parts.minute}`;
}
const CALIBRATION_TOOLS=['configure_calibration','plan_batch','preview_batch','fit_source','fit_batch'];
const DISPLAY_ACTIONS=['preview_target','preview_overview','preview_mesh','preview_recover'];
const STAGE_LABELS={'head.plan':'头部规划','head.review':'头部轨迹审阅','head.capture':'头部采集','head.solve':'头部求解与验收','head.fit':'头部重新求解','wrist.plan':'双腕规划','wrist.review':'双腕轨迹审阅','wrist.capture':'双腕采集','wrist.solve':'双腕求解与验收','wrist.fit':'双腕重新求解','install':'安装标定','calibration.idle':'准备标定','calibration.complete':'标定完成','collection.prepare':'准备数采环境','collection.plan':'观测与规划','collection.execute':'执行与录制','collection.idle':'准备数采','manual.review':'人工轨迹待审阅','manual.rejected':'人工动作未执行','connecting':'正在连接'};
Object.assign(STAGE_LABELS,{'collection.return_review':'恢复场景 · 待确认','collection.return_execute':'正在恢复场景','collection.return_complete':'场景恢复已结束'});
Object.assign(STAGE_LABELS,{'inference.prepare':'加载与预热推理模型','inference.review':'本轮推理待确认','inference.execute':'连续推理与录制','inference.complete':'本轮推理结束','inference.idle':'准备推理'});
const degrees=value=>value*180/Math.PI;
const radians=value=>Number((Number(value)*Math.PI/180).toPrecision(15));
const seconds=value=>Number.isFinite(value)?value.toFixed(1)+' 秒':'尚无记录';
const STATUS = {unknown:'未知',idle:'空闲',ready:'就绪',model_ready:'模型已就绪',running:'进行中',executing:'执行中',verifying:'核验中',settling:'等待稳定',awaiting_capture:'等待采图',planning:'规划中',capturing:'采集中',recording:'录制中',fitting:'求解中',completed:'已完成',complete:'完整',passed:'通过',failed:'失败',rejected:'未执行',expired:'确认已过期',invalid:'无效',stopped:'已停止',stopping:'正在停止',pending:'等待中',unavailable:'不可用',awaiting_review:'等待审阅',awaiting_confirmation:'等待确认',not_started:'尚未开始',not_applicable:'不适用',valid:'有效',damping:'阻尼保持',position:'位置保持',manual_damping:'手动阻尼保持'};
const reasonText=value=>({'motion or capture is still active':'当前动作或采集尚未结束','holding/control state is not confirmed':'保持/控制状态尚未确认，请查看设备诊断'}[value]||value);
const textValue = value => value == null ? '未知' : typeof value === 'object' ? JSON.stringify(value, null, 2) : String(value);
const statusText = value => ({unconfigured:'业务服务未配置',connecting:'正在连接',candidate:'候选结果',diagnostic_readonly:'只读诊断',quality_failed:'质量未通过',incomplete_left:'左腕未通过',incomplete_right:'右腕未通过',incomplete_both:'双腕未通过'}[String(value).toLowerCase()] || STATUS[String(value).toLowerCase()] || textValue(value));
const outcomeStatus = value => typeof value === 'object' && value ? value.status : value;
const executing = value => ['running','executing','settling','verifying','awaiting_capture'].includes(String(value).toLowerCase());

export function frameAgeSeconds(media, state, elapsedMs = 0) {
  if (!Number.isFinite(media?.source_wall_ns) || !Number.isFinite(state?.server_wall_ns)) return null;
  return Math.max(0, (state.server_wall_ns - media.source_wall_ns) / 1e9 + elapsedMs / 1000);
}

export function countedProgress(value) {
  return value && Number.isFinite(value.current) && Number.isFinite(value.total) && value.total > 0 && value.current >= 0 && value.current <= value.total ? value : null;
}

export function inferenceChoice(row){return row?.runtime_schema?row.bundle_digest:row?.id||row?.model_id||'';}
export function inferenceFamily(row){return String(row?.id||row?.model_id||'').startsWith('wam-')?'wam':'pi';}
export function inferenceModes(rows,backend,family,denoiseSteps){return(rows||[]).filter(row=>(row.backend||'torch')===backend&&(!family||inferenceFamily(row)===family)
  &&(!row.inference_options||row.inference_options.denoise_steps===(denoiseSteps??row.inference_capabilities?.denoise_steps?.default)));}

export function supersedingModelPreparation(state,environment){
  const failed=state?.workflow?.request,info=state?.workflow?.inference||{},model=info.model||{};
  if(failed?.status!=='failed'||!['inference_refresh','inference_select','inference_prepare'].includes(failed.action)
    ||!Number.isFinite(failed.finished_wall_ns)||info.model_ready!==true||model.status!=='ready'||model.error)return null;
  const id=info.selected_model_id||model.id,digest=info.bundle_digest||model.bundle_digest;
  return [...environment?.recent_jobs||[],environment?.operations?.prepare_model?.latest_job].filter(Boolean).find(job=>job.status==='succeeded'&&job.verification?.ok===true
    &&job.request?.action==='prepare_model'&&Number.isFinite(job.finished_at_ns??job.updated_at_ns)
    &&(job.finished_at_ns??job.updated_at_ns)>failed.finished_wall_ns
    &&job.request.expected_target?.environment===state.device?.environment
    &&job.request.expected_target?.device_id===state.device?.id
    &&job.request.expected_target?.backend_instance===state.server_instance_id
    &&job.request.expected_target?.manager_instance===environment.target?.manager_instance
    &&id&&job.request.parameters?.model_id===id&&digest&&job.request.parameters?.expected_bundle_digest===digest)||null;
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
  if (['connect','release','recover','inference_select','inference_prepare'].includes(action))
    return {allowed:false,reason:'请通过统一服务运维执行此环境操作'};
  if(action==='inference_run'&&state.runtime?.connected!==true)
    return {allowed:false,reason:'设备控制未接入；请先完成接入 / 检查，Run 不会连接设备'};
  if(action==='delete_confirm'){
    if(!state.workflow?.deletion_preview?.id||state.workflow.deletion_preview.valid!==true)return {allowed:false,reason:state.workflow?.deletion_preview?.reason||'请先预览具体删除范围'};
    if(!options.deleteClearance)return {allowed:false,reason:'请先勾选确认本次预览中的删除范围'};
  }
  return {allowed:true, reason:''};
}

export function handoffPermission(state,target,options={}) {
  const gate=actionPermission(state,'handoff',options);
  if(!gate.allowed)return gate;
  if(!Object.hasOwn(BUSINESS,target))return {allowed:false,reason:'请选择标定、数采或推理'};
  if(state.runtime?.connected!==true)return {allowed:false,reason:'请先在服务运维接入设备'};
  if(state.workflow?.busy===true)return {allowed:false,reason:'请先结束当前业务任务，再交接控制权'};
  if(state.runtime.holder===target)return {allowed:false,reason:'该业务已持有控制权'};
  return gate;
}

export function safeMediaUrl(value, origin) {
  if (typeof value !== 'string' || !value) return null;
  try { const url = new URL(value, origin); return url.origin === new URL(origin).origin && ['http:','https:'].includes(url.protocol) ? url.href : null; }
  catch { return null; }
}

export function sceneAssetPair(catalog,machine,cup){
  return catalog?.pairs?.find(row=>row.machine===machine&&row.cup===cup)||null;
}

export function routePhysicalRequest(value, options, state, origin) {
  const url=new URL(value,origin),route=state?.gateway_route;
  if(url.origin!==new URL(origin).origin||!url.pathname.startsWith('/api/')||url.pathname==='/api/state'||!route)return value;
  let body;try{body=typeof options?.body==='string'?JSON.parse(options.body):null;}catch{}
  const environment=body?.environment||url.searchParams.get('environment');
  if(environment!=='physical')return value;
  if(!/^[0-9a-f]{32}$/.test(route.route_id)||route.prefix!=='/device-physical/'+route.route_id||route.device_id!==state.device?.id)
    throw Error('真机服务绑定身份无效，请重新核验页面');
  return route.prefix+url.pathname+url.search;
}

export function verifyPreviewFrame(media,metadata,environment){
  const matchingRevision=media.preview_view==='target'&&Number.isInteger(media.minimum_revision)
    ?Number.isInteger(metadata?.revision)&&metadata.revision>=media.minimum_revision&&metadata.revision<=media.revision
    :metadata?.revision===media.revision;
  if(!metadata||metadata.environment!==environment||metadata.view!==media.preview_view||!matchingRevision)
    throw Error('预览帧身份或修订不匹配，等待当前请求的画面');
  if(media.server_instance_id&&metadata.server_instance_id!==media.server_instance_id)throw Error('预览服务实例已变化，丢弃旧画面');
  if(media.display_context&&metadata.display_context!==media.display_context)throw Error('安装或控制上下文已变化，丢弃旧画面');
  if(media.renderer_id&&metadata.renderer_id!==media.renderer_id)throw Error('渲染服务身份已变化，丢弃旧画面');
  if(media.preview_view!=='mesh'&&metadata.view_revision!==media.view_revision)throw Error('观察视角修订不匹配，丢弃旧画面');
  if(media.preview_view==='mesh'&&(metadata.camera!==media.camera||metadata.candidate_id!==media.candidate_id))
    throw Error('叠加相机或标定候选不匹配，丢弃旧画面');
  if(!Number.isFinite(metadata.source_wall_ns)||metadata.source_wall_ns<=0)throw Error('预览帧缺少可核验的源时间');
  return metadata;
}

export function previewAuditSummary(window){
  const dispatched=window.inputs.filter(row=>Number.isFinite(row.submitted_ms)),undispatched=window.inputs.filter(row=>!Number.isFinite(row.submitted_ms));
  const delays=dispatched.map(row=>!row.cancelled&&!row.error&&Number.isFinite(row.presented_ms)&&row.presented_ms>=row.input_ms?row.presented_ms-row.input_ms:Infinity).sort((a,b)=>a-b);
  const percentile=p=>delays.length?delays[Math.ceil(delays.length*p)-1]:null;
  const measured=value=>Number.isFinite(value)?value:null;
  const p95=percentile(.95),p99=percentile(.99),maximum=delays.at(-1);
  return {raw_inputs:window.inputs.length,dispatched_inputs:dispatched.length,coalesced_inputs:undispatched.filter(row=>row.coalesced).length,
    other_undispatched_inputs:undispatched.filter(row=>!row.coalesced).length,
    matched_inputs:delays.filter(Number.isFinite).length,unmatched_inputs:delays.filter(value=>!Number.isFinite(value)).length,
    cancelled_requests:dispatched.filter(row=>row.cancelled).length,failed_requests:dispatched.filter(row=>row.error).length,
    new_presentations:window.frames.filter(row=>Number.isInteger(row.input_id)&&row.presented_ms<60000).length,duplicate_frames:window.duplicates,
    unattributed_frames:window.frames.filter(row=>!Number.isInteger(row.input_id)).length,
    new_display_fps:window.status==='completed'?window.frames.filter(row=>Number.isInteger(row.input_id)&&row.presented_ms<60000).length/60:null,
    latency_p95_ms:measured(p95),latency_p99_ms:measured(p99),latency_max_ms:measured(maximum),
    latency_p95_unbounded:p95===Infinity,latency_p99_unbounded:p99===Infinity,latency_max_unbounded:maximum===Infinity};
}

export function createWorkbench({document:doc, fetch:fetchRequest, location, performance:clock, URL:urls, crypto, setInterval, clearInterval, setTimeout, clearTimeout,requestAnimationFrame,cancelAnimationFrame,storage,sessionMonitor,environmentAccess}) {
  const el = id => doc.getElementById(id);
  const setText = (id, value) => { el(id).textContent = textValue(value); };
  const query=new URL(location.href).searchParams,requestedBusiness=query.get('business');
  let context = {environment:query.get('environment')==='physical'?'physical':'fake',
    business:query.get('view')==='control-research'?'research':Object.keys(BUSINESS).includes(requestedBusiness)?requestedBusiness:'calibration'}, tab = context.business;
  const operatorText=value=>{
    let text=textValue(value).replace(/fake\s*同\s*plant/gi,'当前仿真设备').replace(/\bL3\s*辅助/g,'辅助').replace(/\bL3\b/g,'辅助回放').replace(/\bplant\b/gi,'仿真设备');
    if(context.environment==='fake')text=text.replace(/真机/g,'开发设备');
    return text;
  };
  let state = null, generation = 0, revision = -1, receivedAt = 0, online = false, polling = null;
  let policyKey='',policySnapshot=null,policyFields=[],policyLoading=false,policyDirty=false;
  const request=(url,options)=>fetchRequest(routePhysicalRequest(url,options,state,location.href),
    options?.method==='POST'&&/^[a-f0-9]{64}$/.test(CLIENT_DEPLOYMENT_SHA)
      ? {...options,headers:{...options.headers,'X-Workbench-Deployment':CLIENT_DEPLOYMENT_SHA}} : options);
  let serverInstance,serverRoute=null,retiredServerInstances=new Set();const environmentIdentities=new Map();
  let requestSequence = 0, displayedRequest = 0, reviewKey = '', manualKey = '', replayKey = '', stopped = false;
  let waitingForStateNotice=true,requestFailureNoticeKey=null,activeFailureNoticeKey=null;
  const pending = new Map(), mediaSlots = new Map(), timers = [], invalidatedReviews = new Set();
  let resultActions=[],queueActions=[],replayControlsKey='',manualQueueKey='';
  let manualIntent=null,manualExpiryMessage='';
  let calibrationKey='',calibrationPlanKey='',resultsKey='';
  let inferenceSeedKey='',inferenceCatalogKey='',inferenceRefreshKey='',inferenceBackendKey='',inferenceRunIntent=null,inferenceSelection=null;
  let sceneAssetCatalogKey='',sceneAssetDraft={machine:'',cup:''};
  let executionSettingsKey='',executionSettingsRevision=null,executionSettingsDirty=false;
  let speedProtectionKey='',speedProtectionRevision=null,speedProtectionDirty=false;
  let precisionSettingsKey='',precisionSettingsRevision=null,precisionSettingsDirty=false,precisionNotesKey='',precisionNotesDirty=false,precisionHistoryKey='';
  let speedKey='',wasReturnTask=false,trajectoryKey='',trajectory=null,trajectoryRequest=null;
  let replayVideos=[],pairedCamera='',pairedPosition=0,pairedStarted=0,pairedPlaying=false,pairedRate=1,pairVersion=0;
  const historySelections=new Map();let historyRun='';
  let resultsCursor=null,resultsBrowsing=false;
  let recordingFiles=[],recordingFilesContext='',recordingFilesBusy=false,recordingDeleteIntent=null;
  const recordingSelection=new Set();
  let deletionKey='';const confirmedDeletions=new Set();
  let previewView='target',meshCamera='head',meshCandidate='',meshAnchor='',meshOptionsKey='',meshStateKey='',previewRenderer=null;
  let meshCatalog=null,meshCatalogRequest=null;
  const meshCatalogIdentity=value=>JSON.stringify([value?.server_instance_id,value?.gateway_route?.route_id,value?.device?.id,
    value?.runtime?.calibration_revision,value?.runtime?.calibration_snapshot_id]);
  let overview={yaw:-Math.PI/2,pitch:.4,zoom:1,pan_y:0},overviewDirty=false,overviewSending=false,overviewPointer=null;
  let overviewFlight=null;
  let nativeViewer=null,nativeLoading=null,nativeKey=null,nativeError='',nativePendingPose=null;
  const nativeBinding=()=>state?.preview?.webgl_model?JSON.stringify([context.environment,state.device?.id,state.preview.webgl_model.model_id]):null;
  function nativeLabel(){
    if(nativeViewer)el('media-preview').querySelector('.media-meta').textContent=ENVIRONMENT[context.environment]+' · 本地 WebGL 目标预览 · '+(overview.top_view?'左：交互3D；右：固定顶部视角':'鼠标拖动旋转；滚轮缩放')+' · 不代表实测或运动';
  }
  function syncNativeViewer(){
    const binding=nativeBinding();if(binding===nativeKey)return;
    nativeLoading?.abort();nativeLoading=null;nativeViewer?.destroy();nativeViewer=null;nativeKey=binding;nativeError='';nativePendingPose=null;
    el('media-preview').classList.toggle('gpu',false);
    if(!binding||typeof doc.createElement('canvas').getContext!=='function')return;
    const controller=new AbortController();nativeLoading=controller;const modelRef=state.preview.webgl_model;
    void import('./target_viewer.js').then(module=>module.createTargetViewer({container:el('media-preview').querySelector('.image-box'),modelRef,request,signal:controller.signal,onFrame:nativeLabel})).then(viewer=>{
      if(controller.signal.aborted||binding!==nativeBinding()){viewer.destroy();return;}
      nativeLoading=null;nativeViewer=viewer;cancelTarget('已切换本地GPU目标显示');cancelOverview();
      const pose=nativePendingPose||displayPose();nativePendingPose=null;viewer.setPose(pose.q12,pose.jaw_gap_mm);
      if(overview.yaw===-Math.PI/2)overview.yaw=.4;
      viewer.setView(overview);el('media-preview').classList.toggle('gpu',true);el('media-preview').classList.toggle('split',overview.top_view===true);
      renderMedia();nativeLabel();
    }).catch(error=>{if(!controller.signal.aborted&&binding===nativeBinding()){nativeLoading=null;nativeError=error.message;notice('本地GPU目标显示不可用，保留CPU预览：'+error.message);}});
  }
  const drafts=new Map(),meshChoices=new Map();let autoPreviewTimer=null,planIndex=0,observationCamera='all';
  const displayRequests=new Map(),previewFloors=new Map(),previewErrors=new Map(),planPreviewButtons=[];
  const targetFlights=[];let targetPending=null,presentedTarget=null;
  const auditEnabled=new URL(location.href).searchParams.get('preview-audit')==='1';let audit=null,auditDownload=null;
  const jointInputs=role=>[...el('joint-inputs').querySelectorAll('input')].filter(input=>input.dataset.role===role);
  const jawRange=(i,feedback=false)=>(feedback?state?.manual?.jaw_feedback_ranges_mm:state?.manual?.jaw_command_ranges_mm)?.[i]||(feedback?[-2,89]:[0,83]);
  const validJawDraft=(value,i=0)=>['string','number'].includes(typeof value)&&String(value).trim()!==''&&Number.isFinite(Number(value))&&Number(value)>=jawRange(i)[0]&&Number(value)<=jawRange(i)[1];
  function displayPose(){
    const inputs=jointInputs('position'),q12=inputs.map(input=>radians(input.value));
    const noise=state?.manual?.position_noise_rad??0;
    if(inputs.length!==12||inputs.some((input,i)=>String(input.value).trim()===''||!Number.isFinite(q12[i])||q12[i]<state.manual.limits[i][0]-noise-1e-12||q12[i]>state.manual.limits[i][1]+noise+1e-12))throw Error('关节目标超出限位及允许的位置噪声范围');
    const values=[el('jaw-left').value,el('jaw-right').value],jaw_gap_mm=values.map(Number);
    if(values.some((value,i)=>!validJawDraft(value,i)))throw Error('夹爪目标必须在当前设备标定的命令范围内');
    return {q12,jaw_gap_mm};
  }
  const key = () => context.environment + '/' + context.business;
  const returnTask=()=>context.business==='collection'&&state?.workflow?.collection?.return_task;
  const activeReturnTask=()=>returnTask()&&state?.workflow?.stage_id?.startsWith('collection.return_')&&!['completed','failed','stopped','cancelled'].includes(returnTask().status);
  function returnOffsets(){
    const raw=['return-offset-x','return-offset-y'].map(id=>el(id).value),values=raw.map(Number);
    if(raw.some((value,index)=>String(value).trim()===''||!Number.isInteger(values[index])||Math.abs(values[index])>50))throw Error('杯位偏移须填写 −50～50 mm 的整数');
    return values;
  }
  function returnOffsetsMatch(values){try{return JSON.stringify(returnOffsets())===JSON.stringify(values);}catch{return false;}}
  const returnPositionSummary=()=>[['固定原始杯位',returnTask()?.origin_position_m],['本次计划目标',returnTask()?.goal_position_m]]
    .filter(([,position])=>Array.isArray(position)&&position.length===3&&position.every(Number.isFinite))
    .map(([label,position])=>label+' ['+position.map(value=>(value*1000).toFixed(1)).join(', ')+'] mm（场景坐标）').join('；');
  const historyKey=()=>JSON.stringify([state?.device?.id,key()]);
  const draftKey=()=>state?.device?.id?'robot-workbench-draft.v1/'+encodeURIComponent(state.device.id)+'/'+key():null;
  function readDraft(){
    const id=draftKey();if(!id)return null;
    try{return drafts.get(id)||JSON.parse(storage?.getItem(id)||'null');}catch{return null;}
  }
  function saveDraft(){
    const id=draftKey();if(!id||jointInputs('position').length!==12)return;
    // Only editable drafts and viewing preferences survive; no review or execution identity is stored.
    const value={positions:jointInputs('position').map(input=>input.value),speeds:jointInputs('speed').map(input=>input.value),
      jaw:[el('jaw-left').value,el('jaw-right').value],jawSpeed:el('jaw-speed').value,uniformSpeed:el('manual-speed').value,
      collectionSpeed:el('collection-speed').value,observationCamera,meshChoices:Object.fromEntries(meshChoices)};
    drafts.set(id,value);try{storage?.setItem(id,JSON.stringify(value));}catch{}
  }
  function restoreDraft(){
    const draft=readDraft();if(!draft||typeof draft!=='object')return;
    for(const [role,values]of [['position',draft.positions],['speed',draft.speeds]])if(Array.isArray(values)&&values.length===12)
      jointInputs(role).forEach((input,index)=>{if(typeof values[index]==='string'||typeof values[index]==='number')input.value=values[index];});
    if(Array.isArray(draft.jaw)&&draft.jaw.length===2)draft.jaw.forEach((value,i)=>{
      if(state.manual.disabled_grippers?.includes(['left','right'][i])&&!validJawDraft(value,i))return;
      el('jaw-'+['left','right'][i]).value=value;
    });
    if(draft.jawSpeed!==undefined)el('jaw-speed').value=draft.jawSpeed;
    if(draft.uniformSpeed!==undefined)el('manual-speed').value=draft.uniformSpeed;
    if(Number(draft.collectionSpeed)>=30&&Number(draft.collectionSpeed)<=170){
      el('collection-speed').value=draft.collectionSpeed;setText('collection-speed-value',draft.collectionSpeed+'%');
      if(context.business==='collection'&&!activeReturnTask()&&state?.workflow?.review?.id&&Number(draft.collectionSpeed)!==100*(state.workflow.review.speed_scale??state.workflow.speed_scale??.3))
        invalidatedReviews.add(key()+'/'+state.workflow.review.id);
    }
    observationCamera=['all','head','left','right'].includes(draft.observationCamera)?draft.observationCamera:'all';
    meshChoices.clear();for(const [camera,id]of Object.entries(draft.meshChoices||{}))if(['head','left','right','all'].includes(camera)&&typeof id==='string')meshChoices.set(camera,id);
    syncJointSliders();
    if(state?.workflow?.primary_action?.id==='manual_execute'&&state.workflow.review?.id)
      invalidatedReviews.add(key()+'/'+state.workflow.review.id);
  }
  function syncJointSliders(){jointInputs('slider').forEach((input,index)=>input.value=jointInputs('position')[index].value);}
  const options = () => ({fresh:online && !environmentAccess?.exited && clock.now()-receivedAt <= 3000, pending:[...pending.values()].some(p=>p.context === key()), clearance:el('manual-execution-dialog').open?el('manual-clearance').checked:el('clearance').checked,deleteClearance:el('delete-clearance').checked});
  const permission = action => action==='execute'&&activeReturnTask()&&(!returnOffsetsMatch(state?.workflow?.review?.offset_xy_mm??returnTask().offset_xy_mm)||manualIntent?.action!=='return_cup'||manualIntent?.reviewId!==state?.workflow?.review?.id)
    ? {allowed:false,reason:'请按当前偏移重新点击“恢复场景”，并在本次弹窗中确认'} : ['execute','manual_execute'].includes(action)&&invalidatedReviews.has(key()+'/'+state?.workflow?.review?.id)
    ? {allowed:false,reason:'此审阅已失效；等待后端生成新的执行审阅'} : action==='delete_confirm'&&confirmedDeletions.has(key()+'/'+state?.workflow?.deletion_preview?.id)
    ? {allowed:false,reason:'本次删除确认已提交，请查看删除回执；旧预览不能重复确认'} : actionPermission(state, action, {...options(),...(DISPLAY_ACTIONS.includes(action)?{pending:false}:{})});
  const make = (tag, content, className) => { const node = doc.createElement(tag); if (content !== undefined) node.textContent = textValue(content); if(className)node.className=className; return node; };
  const targetBinding=()=>JSON.stringify([key(),previewView,serverInstance,state?.preview?.renderer_id,state?.preview?.display_context,state?.preview?.revisions?.overview,
    state?.runtime?.runtime_nonce,state?.runtime?.command_epoch,state?.runtime?.calibration_revision,state?.runtime?.calibration_snapshot_id,state?.runtime?.holder,state?.runtime?.execution_instance_id]);
  const overviewBinding=()=>JSON.stringify([key(),serverInstance,state?.preview?.renderer_id,state?.preview?.display_context]);
  function cancelOverview(){clearTimeout(overviewFlight?.timer);overviewFlight=null;overviewDirty=false;}
  const auditBinding=()=>JSON.stringify([key(),previewView,serverInstance,state?.preview?.renderer_id,state?.preview?.revisions?.overview,
    state?.runtime?.runtime_nonce,state?.runtime?.command_epoch,state?.runtime?.calibration_revision,state?.runtime?.calibration_snapshot_id,
    state?.runtime?.holder,state?.runtime?.execution_instance_id,state?.preview?.mesh,state?.manual?.limits,globalThis.innerWidth,globalThis.innerHeight,globalThis.devicePixelRatio]);
  function auditStartReason(){
    if(audit?.active)return '采集进行中；改变显示模式、安装或服务实例会中止本次证据';
    if(!auditEnabled||typeof requestAnimationFrame!=='function')return '仅显式开启诊断的浏览器可采集呈现证据';
    if(doc.hidden||el('work-pane').hidden)return '请将标定或数采工作区保持在前台';
    if(!permission('preview_target').allowed)return permission('preview_target').reason;
    if(!state?.preview?.connected||!options().fresh||!manualReady())return '等待健康渲染服务、最新状态和完整姿态定义';
    try{displayPose();}catch(error){return '请先设置合法的显示姿态：'+error.message;}
    const mesh=state.preview.mesh;
    if(typeof mesh?.requested!=='boolean'||mesh.applied!==mesh.requested)return '等待 mesh 开关的后端生效状态一致';
    if(displayRequests.size)return '等待当前只读显示请求结束';
    if(targetFlights.length||targetPending)return '等待当前目标呈现或结束';
    return '';
  }
  function renderAudit(){
    if(!auditEnabled)return;
    if(audit?.active&&audit.binding!==auditBinding())finishAudit('环境、安装、Owner、renderer、mesh 模式或测量视口已改变');
    if(audit?.active&&(!online||!options().fresh||!state?.preview?.connected))finishAudit('状态或渲染连接失去有效性');
    const w=audit?.window;
    setText('audit-status',audit?.active?`第 ${audit.windows.length} / 3 窗 · ${Math.min(60,(clock.now()-w.started_ms)/1000).toFixed(1)} 秒 · 输入 ${w.inputs.length} · 已呈现 ${w.frames.length}；尾部保留2秒等待窗口`:
      audit?`采集${audit.status==='completed'?'结束':'中止'}：${audit.reason||'3个60秒窗口已记录'}。延迟统计覆盖实际发送的全部目标请求；未发送的合并输入单列，已发送但未上屏的请求保留在尾部。`:'当前模式连续采集60秒×3；仅发送只读目标，不执行动作。');
    setText('audit-summary',audit&&!audit.active?JSON.stringify(audit.windows.map((value,index)=>({window:index+1,status:value.status,...previewAuditSummary(value)})),null,2):'');
  }
  function closeAuditWindow(reason){
    const w=audit.window;w.status=reason?'interrupted':'completed';w.ended_ms=clock.now();
    for(const row of w.inputs)if(row.presented_ms===undefined&&!row.error)row.cancelled=row.cancelled||reason||'未在尾部等待窗口内呈现';
    if(targetFlights.some(job=>job.auditRequest?.window===w)||targetPending?.auditRequest?.window===w)cancelTarget('诊断窗口已结束');
    for(const id of w.rafIds)cancelAnimationFrame?.(id);w.rafIds.clear();
  }
  function finishAudit(reason){
    if(!audit?.active)return;
    clearTimeout(audit.timer);if(audit.window?.status==='running')closeAuditWindow(reason);
    audit.active=false;audit.status=reason?'interrupted':'completed';audit.reason=reason||null;
    el('preview-audit').open=true;renderAudit();
  }
  function beginAuditWindow(){
    const w={started_ms:clock.now(),status:'running',inputs:[],frames:[],duplicates:0,seen:new Set(),byRevision:new Map(),painted:new Map(),rafIds:new Set()};
    const media=mediaSlots.get('preview')?.media;if(media?.frame_id)w.seen.add(JSON.stringify([media.renderer_id,media.frame_id]));
    audit.window=w;audit.windows.push(w);tickAudit();
  }
  function tickAudit(){
    if(!audit?.active)return;renderAudit();if(!audit.active)return;
    const w=audit.window,elapsed=clock.now()-w.started_ms;
    if(elapsed>=62000){closeAuditWindow(null);if(audit.windows.length===3){finishAudit(null);renderGates();return;}beginAuditWindow();return;}
    if(elapsed<60000){
      if(w.inputs.length>=2400||w.frames.length>=4000){finishAudit('达到有界样本上限');return;}
      const q12=[...audit.base.q12],value=audit.center+audit.amplitude*Math.sin(w.inputs.length*.21);q12[audit.joint]=value;
      const row={input_id:w.inputs.length,input_ms:elapsed,target_joint_value_rad:value};w.inputs.push(row);
      void sendDisplay('preview_target',{q12,jaw_gap_mm:audit.base.jaw_gap_mm},{session:audit,window:w,row});
    }
    audit.timer=setTimeout(tickAudit,1000/30);
  }
  function startAudit(){
    const reason=auditStartReason();if(reason){notice(reason);return;}
    const m=state.manual,base=displayPose(),joint=m.limits.findIndex(([lo,hi],i)=>Math.min(base.q12[i]-lo,hi-base.q12[i])>.00001);if(joint<0){notice('没有可用于只读小幅变化的显示关节范围');return;}
    const [lo,hi]=m.limits[joint],center=base.q12[joint],amplitude=Math.min(.005,(center-lo)/2,(hi-center)/2);
    if(auditDownload){urls.revokeObjectURL(auditDownload);auditDownload=null;}el('audit-download').hidden=true;
    previewView='target';
    audit={active:true,status:'running',binding:auditBinding(),windows:[],joint,amplitude,center,
      mode:state.preview.mesh.requested?'mesh_on':'mesh_off',base,base_source:'explicit manual display inputs',
      context:{origin:new URL(location.href).origin,device:{id:state.device.id,label:state.device.label,environment:state.device.environment},server_instance_id:serverInstance,renderer_id:state.preview.renderer_id,
        runtime_nonce:state.runtime.runtime_nonce,calibration_snapshot_id:state.runtime.calibration_snapshot_id,
        calibration_revision:state.runtime.calibration_revision,command_epoch:state.runtime.command_epoch,
        mesh:{...state.preview.mesh},viewport:{width:globalThis.innerWidth||null,height:globalThis.innerHeight||null,device_pixel_ratio:globalThis.devicePixelRatio||null},
        user_agent:globalThis.navigator?.userAgent||'unavailable',hardware_concurrency:globalThis.navigator?.hardwareConcurrency||null},
      started_wall_iso:new Date().toISOString()};
    previewView='target';el('preview-audit').open=false;
    el('media-preview').scrollIntoView?.({block:'center'});beginAuditWindow();renderGates();
  }
  function targetPresentation(slot,metadata){
    const session=audit,w=session?.active?session.window:null;
    if(!metadata?.frame_id||metadata.view!=='target'||typeof requestAnimationFrame!=='function')return;
    const version=slot.version,blob=slot.blob,binding=targetBinding(),presenting=slot.presenting;
    const next=callback=>{const id=requestAnimationFrame(()=>{w?.rafIds.delete(id);callback();});w?.rafIds.add(id);};
    next(()=>next(()=>{
      if(slot.version!==version||slot.blob!==blob||doc.hidden||el('work-pane').hidden)return;
      if(presenting&&!targetFlights.includes(presenting))return;
      const rect=slot.image.getBoundingClientRect?.();if(!rect||rect.width<=0||rect.height<=0||rect.bottom<=0||rect.right<=0||rect.top>=(globalThis.innerHeight||Infinity)||rect.left>=(globalThis.innerWidth||Infinity))return;
      if(binding!==targetBinding()||!online||!options().fresh)return;
      if(w&&audit===session&&session.active&&session.window===w&&session.binding===auditBinding()){
        const identity=JSON.stringify([metadata.renderer_id,metadata.frame_id]);
        if(w.seen.has(identity))w.duplicates++;
        else{
          w.seen.add(identity);
          const frame={frame_id:metadata.frame_id,revision:metadata.revision,view_revision:metadata.view_revision,source_wall_ns:metadata.source_wall_ns,presented_ms:clock.now()-w.started_ms};
          const row=w.byRevision.get(metadata.revision);if(row&&!row.cancelled&&!row.error){frame.input_id=row.input_id;if(row.presented_ms===undefined){row.presented_ms=frame.presented_ms;row.frame_id=frame.frame_id;}}
          if(!w.painted.has(metadata.revision))w.painted.set(metadata.revision,frame);w.frames.push(frame);
        }
      }
      const job=targetFlights[0];
      if(job?.revision===metadata.revision&&slot.presenting===job){
        clearTimeout(job.timer);targetFlights.shift();slot.presenting=null;presentedTarget={binding,revision:metadata.revision};
        presentNextTarget(slot);void dispatchTarget();kickTargetMedia();
      }
    }));
  }
  function exportAudit(){
    if(!audit||audit.active)return;
    const result={schema:'preview_browser_presentation_proxy.v1',clock:'browser performance.now milliseconds',
      presentation_endpoint:'decoded matching image retained through two requestAnimationFrame callbacks while visible; not display photons',
      measurement_population:'dispatched display targets',
      latency_population:'all dispatched targets, measured from the selected original input time including queue delay; unpresented/cancelled/failed requests are +Infinity, non-finite percentiles serialized as null with unbounded flags; never-dispatched inputs are reported separately',
      count_relationship:'raw_inputs = dispatched_inputs + coalesced_inputs + other_undispatched_inputs; dispatched_inputs = matched_inputs + unmatched_inputs',
      qualification:'browser measurement only; join with frozen run contract and measurement-host CPU/GPU load evidence; no automatic pass',
      mode:audit.mode,status:audit.status,reason:audit.reason,context:audit.context,started_wall_iso:audit.started_wall_iso,
      profile:{input_hz:30,window_seconds:60,windows:3,tail_wait_seconds:2,dispatch_policy:'shared product FIFO: at most three unpresented targets, one HTTP POST at a time; only the latest unsent input may be replaced',target_window:3,duplicate_policy:'identical full frame identity keeps the existing decoded Blob and presentation callbacks',base:audit.base,base_source:audit.base_source,joint_index:audit.joint,center_rad:audit.center,amplitude_rad:audit.amplitude},
      thresholds:{new_display_fps:audit.mode==='mesh_off'?20:10,latency_p95_ms:audit.mode==='mesh_off'?200:300},
      windows:audit.windows.map((w,index)=>({index:index+1,status:w.status,started_performance_ms:w.started_ms,ended_performance_ms:w.ended_ms,
        summary:previewAuditSummary(w),inputs:w.inputs,frames:w.frames}))};
    if(auditDownload)urls.revokeObjectURL(auditDownload);auditDownload=urls.createObjectURL(new Blob([JSON.stringify(result,null,2)],{type:'application/json'}));
    const link=el('audit-download');link.href=auditDownload;link.download='preview-browser-'+audit.mode+'.json';link.hidden=false;
  }
  function notice(message, active = false, waitingForState = false,failureKey=null) { activeFailureNoticeKey=failureKey;waitingForStateNotice=waitingForState;setText('request-status',operatorText(message)); el('request-status').classList.toggle('pending',active); }
  function gate(id, action, extraReason = '') {
    const p = permission(action), reason = operatorText(reasonText(extraReason || p.reason)||'');
    el(id).disabled = !!reason || !p.allowed;
    el(id).setAttribute('aria-disabled',String(el(id).disabled));
    setText(id+'-reason',reason);
  }
  function renderGuidance(){
    const r=state?.runtime||{},w=state?.workflow||{},primary=w.primary_action,opts=options();
    const show=(title,message,target,label)=>{
      for(const [id,value]of [['context-guidance-title',title],['context-guidance-message',message],['context-guidance-next',label]]){
        const text=operatorText(reasonText(value));if(el(id).textContent!==text)el(id).textContent=text;
      }
      const link=el('context-guidance-next');link.dataset.target=target;
      link.href=target==='operations-link'?(el('operations-link').href||'/operations'):'#'+target;
    };
    const business=BUSINESS[context.business],stage=STAGE_LABELS[w.stage_id]||w.title||'当前任务';
    if(environmentAccess?.exited)return show(environmentAccess.exitState==='closed'?'工作台已退出':'正在关闭工作台入口','本工作台资源已退出。需要再次使用时，请重新运行启动脚本。','physical-endpoint-panel','查看退出状态');
    if(!state)return show(business+' · 状态待核验',waitingForStateNotice?'正在读取当前业务状态；切换栏目不会连接设备或交接控制权。':'当前无法读取工作台状态；请先到服务运维核验后端。','operations-link','查看服务运维');
    if(!opts.fresh||!state.csrf)return show('当前状态待重新核验','连接中断或状态已过期；旧连接、控制权与任务信息不能作为当前可执行依据。','operations-link','查看服务运维');
    if(['FAULT','FAULT_STOP_UNCONFIRMED','EMERGENCY'].includes(String(r.state).toUpperCase()))
      return show('控制故障仍锁定',r.reason||r.last_fault_diagnostic?.error||primary?.reason||'反馈恢复健康不代表可以执行；请先核对设备诊断。','diagnostics-pane','查看设备诊断');
    if(r.release_receipt&&r.release_receipt.status!=='released')
      return show('设备释放尚未确认',r.release_receipt.message||'请查看释放回执；未确认断开前不要重新连接。','diagnostics-pane','查看释放诊断');
    if(state.maintenance)return show('服务维护中',state.maintenance.error||primary?.reason||'维护尚未结束；请在服务运维查看进度与恢复条件。','operations-link','查看维护状态');
    if(w.status==='unconfigured')return show(business+'业务尚未配置',w.message||primary?.reason||'请先在服务运维核验当前业务后端。','operations-link','查看服务运维');
    if(w.busy===true||w.request?.status==='running'||opts.pending)
      return show(business+' · '+stage,w.message||primary?.reason||'当前请求仍在处理；请查看任务回执，停止入口保持独立。','task-panel','查看当前任务');
    if(String(r.state).toUpperCase()==='CONNECTING')return show('设备正在连接',primary?.reason||w.message||'正在等待连接和反馈回执；请勿重复发起连接。','task-panel','查看连接进度');
    if(w.review?.id&&['execute','manual_execute'].includes(primary?.id)){
      if(primary.id==='manual_execute')return show('人工动作已有审阅回执','请回到人工规划工具核验本次请求；已有回执不会自动执行。','manual-tools','查看人工动作入口');
      return show(business+' · 待审阅确认',permission(primary.id).reason||'请在原执行入口核对本次摘要并确认；查看提示不会执行动作。',
        context.business==='collection'?(activeReturnTask()?'return-cup':'collection-execute'):'primary','查看本次审阅入口');
    }
    if(context.business==='inference'){
      const info=w.inference||{},model=info.model||{},run=permission('inference_run');
      const ready=info.model_ready===true||model.model_ready===true,holder=BUSINESS[r.holder]||r.holder;
      if(run.allowed){
        const connection='设备控制已接入；本轮确认只申请推理执行准入，不连接或恢复设备';
        return show(ready?'模型已就绪 · 等待本轮确认':'后端允许进入 Run 审阅',connection+'。','inference-run','前往 Run 入口');
      }
      if(ready)return show(r.connected===false?'模型已就绪 · 设备尚未连接':'模型已就绪 · 本轮尚不可开始',
        (r.connected===true&&r.holder!=='inference'?'当前控制权在'+(holder||'未知业务')+'；':'')+run.reason,'inference-run','查看 Run 条件');
      return show(model.status==='loading'?'推理模型正在准备':info.model_ready===false||model.loaded===false?'推理模型尚待准备':'推理模型状态待核验',
        model.error||run.reason||primary?.reason||model.message||'先在模型选择区核验当前模型；模型准备不会连接设备或执行动作。','inference-checkpoint','查看模型选择与准备');
    }
    if(r.connected===false)return show(business+' · 设备控制未接入','先完成接入 / 检查；浏览、规划和 Run 不会连接或恢复设备。','environment-connect','查看接入条件');
    if(r.connected!==true)return show('设备连接状态未知','请到服务运维核对最后事实与中断恢复条件。','operations-link','查看服务运维');
    if(context.business==='collection'){
      const collection=w.collection||{},warmup=collection.warmup||{status:typeof collection.environment?.ready==='boolean'?(collection.environment.ready?'ready':'not_started'):'unknown'};
      if(warmup.status!=='ready')
        return show(warmup.status==='failed'?'数采初始化失败':permission('collection_prepare').allowed?'数采业务待预热':'数采环境'+(warmup.status==='running'?'正在准备':['not_started','invalidated'].includes(warmup.status)?'待准备':'状态待核验'),
          permission('collection_prepare').reason||primary?.reason||warmup.message||'请在初始化与预热入口核验当前准备条件。','collection-prepare','查看数采环境准备');
    }
    const next=primary?permission(primary.id):{allowed:false,reason:'后端尚未提供下一步操作'};
    const target=context.business==='collection'&&['generate_plan','collection_plan'].includes(primary?.id)?'collection-plan':'primary';
    return show(business+' · '+stage,next.reason||w.message||(primary?.label?'下一步请在原入口核对“'+primary.label+'”。':'等待后端提供当前阶段与下一步。'),primary?target:'task-panel',next.allowed?'查看下一步入口':'查看当前阻碍');
  }
  const trajectoryBinding=()=>JSON.stringify([key(),serverInstance,state?.workflow?.id,state?.workflow?.trajectory?.id]);
  function renderTrajectory(){
    const view=state?.workflow?.trajectory,binding=trajectoryBinding();
    el('trajectory-panel').hidden=context.business!=='collection'||!view;
    if(binding!==trajectoryKey){
      trajectoryRequest?.abort();trajectoryRequest=null;trajectoryKey=binding;trajectory=null;
      el('trajectory-view').hidden=true;el('trajectory-tick').value=0;
      setText('trajectory-status',view?.error||'点击加载当前冻结轨迹；只读，不执行。');
      const url=safeMediaUrl(view?.first_frame_url,location.href);el('trajectory-first-frame').hidden=!url;
      if(url)el('trajectory-first-frame').src=url;else el('trajectory-first-frame').removeAttribute('src');
    }
    el('trajectory-load').disabled=!view?.url||!!trajectoryRequest;
  }
  async function loadTrajectory(){
    const view=state?.workflow?.trajectory,url=safeMediaUrl(view?.url,location.href);if(!url)return;
    const binding=trajectoryBinding(),controller=new AbortController();trajectoryRequest?.abort();trajectoryRequest=controller;
    setText('trajectory-status','正在读取当前冻结轨迹…');renderTrajectory();
    try{
      const response=await request(url,{cache:'no-store',signal:controller.signal});if(!response.ok)throw Error('轨迹接口 '+response.status);
      const value=await response.json();if(binding!==trajectoryBinding()||controller.signal.aborted)return;
      const ticks=value?.ticks;
      if(value?.timeline_digest!==view.timeline_digest||!Array.isArray(ticks)||!ticks.length||ticks.some((row,index)=>
        row.tick!==index||!Number.isFinite(row.time_s)||row.time_s<0||index&&row.time_s<=ticks[index-1].time_s||
        ['left','right'].some(side=>!Array.isArray(row.target?.[side+'_arm_joints'])||row.target[side+'_arm_joints'].length!==6||
          !row.target[side+'_arm_joints'].every(Number.isFinite)||!Number.isFinite(row.target[side+'_gripper']))))throw Error('轨迹身份、序列或目标字段无效');
      trajectory=value;el('trajectory-view').hidden=false;el('trajectory-tick').max=ticks.length-1;
      const stages=(Array.isArray(value.stages)?value.stages:[]).filter(row=>Number.isInteger(row.start_tick)&&row.start_tick>=0&&row.start_tick<ticks.length);
      el('trajectory-stage').replaceChildren();for(const row of stages){const option=make('option',row.name);option.value=row.start_tick;el('trajectory-stage').append(option);}
      setText('trajectory-status',`${ticks.length} 个目标 · 父计划速度 ${100*(view.speed_scale??.3)}% · ${view.timeline_digest}`);
      renderTrajectoryTick();renderTrajectoryPlots();
    }catch(error){if(binding===trajectoryBinding()&&!controller.signal.aborted)setText('trajectory-status','只读轨迹加载失败：'+error.message);}
    finally{if(trajectoryRequest===controller){trajectoryRequest=null;renderTrajectory();}}
  }
  function renderTrajectoryTick(){
    if(!trajectory)return;const index=Math.max(0,Math.min(trajectory.ticks.length-1,Number(el('trajectory-tick').value)||0)),row=trajectory.ticks[index];
    setText('trajectory-position',`tick ${index} / ${trajectory.ticks.length-1} · ${row.time_s.toFixed(3)} s`);
    setText('trajectory-target',['left','right'].map(side=>`${side==='left'?'左臂':'右臂'} · rad：${row.target[side+'_arm_joints'].map(value=>value.toFixed(5)).join(', ')}\n夹爪金属内间距：${row.target[side+'_gripper'].toFixed(3)} mm`).join('\n'));
  }
  function renderTrajectoryPlots(){
    if(!trajectory)return;const ticks=trajectory.ticks,side=el('trajectory-arm').value==='right'?'right':'left';
    const draw=(id,series,height)=>{
      const svg=el(id),all=series.flat(),low=all.reduce((a,b)=>Math.min(a,b),Infinity),high=all.reduce((a,b)=>Math.max(a,b),-Infinity),span=high-low||1,end=ticks.at(-1).time_s||1;
      svg.replaceChildren();series.forEach((values,index)=>{const line=doc.createElementNS('http://www.w3.org/2000/svg','polyline');
        line.setAttribute('points',values.map((value,i)=>`${10+ticks[i].time_s/end*580},${height-10-(value-low)/span*(height-20)}`).join(' '));
        line.setAttribute('fill','none');line.setAttribute('stroke',['#356894','#b05d35','#568348','#805ba6','#b18a22','#498886'][index]);line.setAttribute('stroke-width','1.5');svg.append(line);});
      svg.setAttribute('aria-label',`${id==='trajectory-joints'?'六关节 rad':'左右夹爪 mm'} · 范围 ${low.toFixed(3)} 至 ${high.toFixed(3)} · 0 至 ${end.toFixed(3)} 秒`);
    };
    draw('trajectory-joints',Array.from({length:6},(_,i)=>ticks.map(row=>row.target[side+'_arm_joints'][i])),160);
    draw('trajectory-jaws',['left','right'].map(side=>ticks.map(row=>row.target[side+'_gripper'])),120);
  }
  const pair=()=>replayVideos.filter(row=>(context.business==='calibration'||row.run_id===historyRun)&&row.camera===pairedCamera&&['source','target'].includes(row.role)).map(row=>row.node);
  const pairDuration=()=>Math.max(0,...pair().map(node=>Number.isFinite(node.duration)?node.duration:0));
  const pairTime=()=>Math.min(pairDuration(),pairedPosition+(pairedPlaying?(clock.now()-pairedStarted)/1000*pairedRate:0));
  function pausePair(){pairVersion++;pairedPosition=pairTime();pairedPlaying=false;for(const node of pair())node.pause?.();setText('collection-replay-toggle','播放两侧');}
  function seekPair(value){pairedPosition=Math.max(0,Math.min(pairDuration(),value));pairedStarted=clock.now();for(const node of pair())if(Number.isFinite(node.duration))node.currentTime=Math.min(pairedPosition,node.duration);renderPairClock();}
  async function playPair(){
    if(pair().length!==2||pair().some(node=>!Number.isFinite(node.duration)))return;
    if(pairedPlaying){pausePair();return;}if(pairTime()>=pairDuration())seekPair(0);
    const version=pairVersion;pairedStarted=clock.now();pairedPlaying=true;setText('collection-replay-toggle','暂停两侧');
    try{await Promise.all(pair().filter(node=>node.currentTime<node.duration).map(node=>{node.playbackRate=pairedRate;return node.play();}));}
    catch(error){if(version===pairVersion){pausePair();setText('collection-replay-status','回放播放失败：'+error.message);}}
  }
  function renderPairClock(){
    const nodes=pair(),ready=nodes.length===2&&nodes.every(node=>Number.isFinite(node.duration)&&node.duration>0),position=pairTime();
    el('collection-replay-toggle').disabled=!ready;el('collection-replay-time').disabled=!ready;el('collection-replay-time').max=pairDuration();el('collection-replay-time').value=position;
    if(ready)setText('collection-replay-status',`${position.toFixed(2)} / ${pairDuration().toFixed(2)} s · 各自首帧对齐，短视频保留末帧；L3 辅助回放不证明实际任务成功。`);
    else if(nodes.length===1)setText('collection-replay-status','此相机仅有单侧视频，可独立回放；不补造另一侧结果。');
    if(pairedPlaying){const version=pairVersion;for(const node of nodes){if(position>=node.duration){node.pause();node.currentTime=node.duration;}else{if(Math.abs(node.currentTime-position)>.12)node.currentTime=position;if(node.paused)void node.play().catch(error=>{if(version===pairVersion){pausePair();setText('collection-replay-status','回放播放失败：'+error.message);}});}}if(position>=pairDuration())pausePair();}
  }
  function selectPair(){
    pausePair();pairedCamera=el('collection-replay-camera').value;pairedPosition=0;
    const paired=pair().length===2;for(const row of replayVideos){row.figure.hidden=!!row.camera&&row.camera!==pairedCamera;row.node.controls=!paired||row.camera!==pairedCamera;}
    seekPair(0);
  }
  function manualPreviewNotice(){
    const request=state?.workflow?.request;
    if(!['manual_preview','manual_queue_preview'].includes(request?.action)||!['running','failed'].includes(request.status))return null;
    const name=request.action==='manual_preview'?'单姿态':'整队',running=request.status==='running';
    const end=running?state.server_wall_ns:request.finished_wall_ns;
    const elapsed=Number.isFinite(end)&&Number.isFinite(request.started_wall_ns)?` · 已耗时 ${Math.max(0,(end-request.started_wall_ns)/1e9).toFixed(1)} 秒`:'';
    const stage=state.workflow.manual_preflight?.stage;
    const title=running?`正在检查${name}动作${stage?' · '+stage:''}`:`${name}执行前检查失败`;
    return {status:request.status,title,message:running?`${title}${elapsed}。尚未执行；检查通过后将弹出本次动作确认。`:`${title}${elapsed}：${operatorText(request.error)}。未下发本次运动；请根据原因处理后重新点击执行。`};
  }
  const manualBinding=()=>JSON.stringify([key(),serverInstance,state?.device?.id,state?.runtime?.runtime_nonce,state?.runtime?.command_epoch,state?.runtime?.calibration_revision,state?.runtime?.holder]);
  function dismissManualExecution(){
    if(manualIntent?.reviewId)invalidatedReviews.add(manualIntent.context+'/'+manualIntent.reviewId);
    manualIntent=null;el('manual-clearance').checked=false;
    if(el('manual-execution-dialog').open)el('manual-execution-dialog').close();
  }
  function renderManualExecution(){
    const intent=manualIntent,w=state?.workflow,review=w?.review;
    if(!intent)return;
    if(intent.binding!==manualBinding()||
        intent.queueRevision!==undefined&&intent.queueRevision!==state?.manual?.queue?.revision||
        w?.request?.id===intent.jobId&&w.request.status==='failed'){
      dismissManualExecution();return;
    }
    // A slow check has no presented approval to revoke. Wait for a fresh,
    // matching receipt; an already presented review still expires on stale state.
    if(!options().fresh){if(intent.reviewId)dismissManualExecution();return;}
    if(!intent.jobId)return;
    if(w?.request?.id===intent.jobId&&w.request.status==='running'){
      el('manual-confirm').disabled=true;return;
    }
    const executeAction=intent.action==='return_cup'?'execute':'manual_execute';
    const matching=review?.request_job_id===intent.jobId&&w.primary_action?.id===executeAction;
    if(matching&&intent.action==='return_cup'&&(!returnOffsetsMatch(intent.offsets)||!returnOffsetsMatch(review.offset_xy_mm??returnTask()?.offset_xy_mm))){dismissManualExecution();return;}
    if(matching&&intent.expiresAt===undefined&&Number.isFinite(review.expires_at_wall_ns))intent.expiresAt=review.expires_at_wall_ns;
    const remaining=Number.isFinite(intent.expiresAt)&&Number.isFinite(state.server_wall_ns)?(intent.expiresAt-state.server_wall_ns)/1e6-(clock.now()-receivedAt):null;
    if(remaining!==null&&remaining<=0){
      manualExpiryMessage=intent.action==='return_cup'?'本次确认已过期，场景尚未恢复；请重新点击“恢复场景”检查并确认。':'本次确认已过期，动作未执行；重新点击执行重新检查。';
      dismissManualExecution();notice(manualExpiryMessage);return;
    }
    if(intent.reviewId&&(!matching||review.id!==intent.reviewId)){dismissManualExecution();return;}
    if(!matching)return;
    const gate=actionPermission(state,executeAction,{...options(),pending:false,clearance:true});
    if(!gate.allowed||invalidatedReviews.has(key()+'/'+review.id)){dismissManualExecution();return;}
    if(!intent.reviewId){
      intent.reviewId=review.id;
      setText('manual-confirm-title',intent.action==='return_cup'?'恢复场景':intent.action==='manual_preview'?'执行到目标姿态':'执行整队');
      setText('manual-confirm-summary',operatorText(review.summary||returnTask()?.summary));
      const duration=review.total_motion_duration_s??(intent.action==='return_cup'?returnTask()?.duration_s:undefined);
      setText('manual-confirm-duration',Number.isFinite(duration)?'预计运动用时：'+seconds(duration)+(context.environment==='fake'?'（仿真时间，实际等待可能更长）':''):'预计运动用时：见本次执行摘要');
      const jaws=review.waypoint_jaw_gap_mm||intent.jaws||[];
      setText('manual-confirm-jaws',intent.action==='return_cup'?`相对固定原始杯位：前后 ${intent.offsets[0]} mm，左右 ${intent.offsets[1]} mm。`+returnPositionSummary():(review.waypoint_jaw_gap_mm?'冻结夹爪目标':'本次请求夹爪目标')+' · '+jaws.map((row,i)=>`${jaws.length>1?'点 '+(i+1)+'：':''}左 ${textValue(row?.[0])} / 右 ${textValue(row?.[1])} mm`).join('；'));
      el('manual-clearance').checked=false;
      el('manual-clearance-label').hidden=!(context.environment==='physical'||review.physical_clearance_required);
      el('manual-execution-dialog').showModal();
    }
    setText('manual-confirm-expiry',remaining===null?'有效期以控制器审阅为准；不会自动续签。':`本次确认剩余 ${Math.ceil(remaining/1000)} 秒 · 到期后需重新点击执行，不会自动续签。`);
    const allowed=permission(executeAction);el('manual-confirm').disabled=!allowed.allowed;setText('manual-confirm-reason',allowed.reason);
  }
  function renderGates() {
    renderControlPolicy();
    renderSceneAssets();
    renderExecutionSettings();
    renderPrecisionStudy();
    const primary = (state?.workflow?.business_task||state?.workflow)?.primary_action;
    setText('primary',primary?.label || '等待可用操作');
    gate('primary',primary?.id || 'unknown',primary ? '' : '等待后端提供当前阶段主动作');
    for (const action of ['stop','cancel']) gate(action,action);
    const handoff=handoffPermission(state,el('handoff-target').value,options());
    el('handoff').disabled=!handoff.allowed;
    el('handoff-target').disabled=!permission('handoff').allowed;
    setText('handoff-reason',handoff.reason||'保持设备连接，不自动执行动作');
    if(environmentAccess?.state?.target?.device_id&&environmentAccess.state.csrf){el('stop').disabled=environmentAccess.exited||environmentAccess.stopping;setText('stop-reason',environmentAccess.exited?'工作台已退出':environmentAccess.stopping?'独立停止请求已发送，等待回执':'独立停止通道；不等同硬件急停');}
    if ([...pending.values()].some(p=>p.action==='stop' && p.context===key())) {
      el('stop').disabled=true; setText('stop-reason','停止请求已发送，等待停止回执');
    }
    const preview=state?.preview,displayBusy=view=>displayRequests.has(view)||displayRequests.has('recovery');
    gate('mesh',preview?'preview_mesh':'mesh',preview&&displayBusy('mesh')?'叠加设置正在更新':preview&&permission('preview_mesh').allowed&&!meshCandidate&&!preview.mesh?.requested?'请选择与相机匹配的标定候选':'');
    gate('manual-preview','manual_preview',feedbackReady() ? '' : '尚无完整关节定义、限位与健康实测反馈');
    renderManualExecution();
    const manualNotice=manualPreviewNotice(),manualReview=state?.workflow?.primary_action?.id==='manual_execute'&&state?.workflow?.review;
    const manualRejection=!manualIntent&&state?.workflow?.stage_id==='manual.rejected'?operatorText(state.workflow.message):'';
    setText('manual-request-status',manualRejection||manualExpiryMessage||manualNotice?.message||(manualIntent?.reviewId?'请在弹窗中核对本次动作并确认；关闭不会执行。':manualIntent?'正在提交执行前检查，尚未执行。':manualReview?'本页没有待确认动作。点击执行入口将重新检查；不会自动执行已有审阅。':'拖动只更新目标画面。点击执行后检查动作，并在弹窗中确认一次。'));
    el('manual-request-status').className=manualRejection||manualExpiryMessage||manualNotice?.status==='failed'?'tone-bad':'hint';
    gate('show-target','preview_target',displayRequests.has('recovery')?'渲染服务正在恢复':manualReady()?'':'尚无完整模型关节定义与限位');
    const manualDisabled = el('manual-preview').disabled&&el('show-target').disabled;
    for(const input of el('joint-inputs').querySelectorAll('input')) input.disabled=manualDisabled;
    for(const id of ['jaw-left','jaw-right','jaw-speed','manual-speed','apply-manual-speed','use-feedback','zero-preview']) el(id).disabled=manualDisabled;
    el('use-feedback').disabled=manualDisabled||!feedbackReady();
    for(const side of state?.manual?.disabled_grippers||[])if(['left','right'].includes(side))el('jaw-'+side).disabled=true;
    const queue=state?.manual?.queue;
    gate('queue-add','manual_queue',feedbackReady()?'':'等待有效的实测姿态与关节定义');
    gate('queue-preview','manual_queue_preview',queue?.available&&queue.waypoints?.length?'':queue?.reason||'标定动作队列为空');
    gate('queue-clear','manual_queue',queue?.available&&queue.waypoints?.length?'':queue?.reason||'标定动作队列为空');
    for(const button of queueActions)button.disabled=!permission('manual_queue').allowed;
    setText('select-result-reason',context.business==='calibration'?permission('select_result').reason:'');
    for(const row of resultActions){const p=permission(row.action);row.button.disabled=!p.allowed||row.allowed===false;row.reason.textContent=row.allowed===false?row.ruleReason:p.reason;}
    const replay=state?.workflow?.replay_context;
    gate('replay-open','select_result',replay?.frame_count>0?'':'当前批次尚无可回放帧');
    const cameraChanged=!!replay&&el('replay-camera').value!==replay.camera;
    el('replay-overlay').disabled=cameraChanged||!replay?.overlay_available||!permission('select_result').allowed;
    setText('replay-overlay-reason',cameraChanged?'先查看所选相机，再启用匹配结果的叠加':replay?.overlay_available?'':replay?'历史叠加需要与当前相机匹配的头部或双腕结果':'');
    el('collection-tools').hidden=!state?.workflow?.replan_action;
    el('collection-speed-tools').hidden=context.business!=='collection'||!state?.workflow?.stage_id?.startsWith('collection.');
    el('collection-speed').disabled=!!activeReturnTask()||state?.workflow?.busy===true||options().fresh===false;
    gate('review-speed','generate_plan',activeReturnTask()?'数采速度草稿不参与本次场景恢复':state?.workflow?.trajectory?.url?'':'请先生成可审阅的冻结计划');
    gate('replan','generate_plan',state?.workflow?.replan_action?.allowed===false?(state.workflow.replan_action.reason||'当前不能重新规划'):'');
    for(const [id,action]of [['collection-prepare','collection_prepare'],['collection-plan','collection_plan'],['collection-execute','execute']])gate(id,action);
    for(const [id,action]of [['inference-prepare','prepare_model']]){
      const allowed=environmentAccess?.permission(action)||{allowed:false,reason:'请先核验统一运维服务'};el(id).disabled=!allowed.allowed;setText(id+'-reason',allowed.allowed?'由统一运维执行并核验准备结果':allowed.reason);
    }
    let inferenceError=inferenceBackendReason();try{inferenceSeed();inferenceDuration();}catch(error){inferenceError=error.message;}
    if(inferenceParameterUpdate)inferenceError='正在应用参数并重新准备控制；Stop仍可使用';
    gate('inference-preview','inference_preview',inferenceError);
    gate('inference-refresh','inference_refresh');gate('inference-run','inference_run',inferenceError);
    gate('inference-arm','inference_run',inferenceInfo().operator_session?'控制会话已准备':inferenceError);
    el('inference-disarm').disabled=!inferenceInfo().operator_session||!options().fresh||options().pending;
    gate('inference-hold','inference_hold');gate('inference-home','inference_home');gate('inference-teach','inference_teach');
    el('inference-home-confirm').disabled=!el('inference-home-clearance').checked||!permission('inference_home').allowed
      ||operatorHomeIdentity!==operatorIdentity();
    el('inference-checkpoint').disabled=!!inferenceParameterUpdate||!!inferenceSelection?.submitting||environmentAccess?.permission('prepare_model').allowed!==true;
    el('inference-backend').disabled=!!inferenceParameterUpdate||state?.workflow?.busy===true||!!inferenceSelection?.submitting||!!environmentAccess?.state?.active_job||!options().fresh;
    el('inference-family').disabled=el('inference-backend').disabled;
    el('inference-seed').disabled=state?.workflow?.busy===true||options().fresh===false||options().pending||!!inferenceParameterUpdate;
    el('inference-duration').disabled=state?.workflow?.busy===true&&!preparedParameterHold()||options().fresh===false||options().pending||!!inferenceParameterUpdate||!!state?.workflow?.inference?.finite_chunk_count;
    renderInferenceDenoise();
    renderInferenceRun();
    let offsetError='';try{returnOffsets();}catch(error){offsetError=error.message;}gate('return-cup','return_cup',offsetError);
    for(const id of ['return-offset-x','return-offset-y'])el(id).disabled=!options().fresh||returnTask()?.status==='running'&&state?.workflow?.busy===true;
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
    for(const button of planPreviewButtons)button.disabled=!permission('preview_target').allowed||displayRequests.has('recovery');
    const planLength=state?.workflow?.calibration_tools?.waypoints?.length||0,canBrowse=permission('preview_target').allowed&&!displayRequests.has('recovery');
    el('calibration-plan-select').disabled=!canBrowse||!planLength;
    el('calibration-plan-prev').disabled=!canBrowse||planIndex<=0;
    el('calibration-plan-next').disabled=!canBrowse||planIndex>=planLength-1;
    el('replay-candidate').disabled=!replay?.candidates?.length||!permission('select_result').allowed;
    el('preview-recover').href=el('operations-link').href;
    gate('delete-all','delete_preview',(state?.results||[]).some(row=>row.kind==='capture')?'':'当前没有可删除的原始批次');
    gate('delete-confirm','delete_confirm');gate('delete-cancel','delete_cancel');
    el('preview-audit').hidden=!auditEnabled;
    const auditReason=auditStartReason();el('audit-start').disabled=!!auditReason;setText('audit-start-reason',auditReason);
    el('audit-stop').disabled=!audit?.active;el('audit-export').disabled=!audit||audit.active;
    renderGuidance();
  }
  function manualReady() {
    const m=state?.manual;
    return m?.joint_names?.length===12 && m.limits?.length===12 && m.limits.every(v=>Array.isArray(v)&&v.length===2&&v.every(Number.isFinite)&&v[0]<=v[1]);
  }
  function feedbackReady() {
    const m=state?.manual;
    return manualReady() && state?.runtime?.connected===true && state.runtime.feedback_healthy===true && options().fresh && m.positions?.length===12 && m.positions.every(Number.isFinite) && m.jaw_gaps_mm?.length===2 && m.jaw_gaps_mm.every((value,i)=>m.disabled_grippers?.includes(['left','right'][i])||Number.isFinite(value)&&value>=jawRange(i,true)[0]&&value<=jawRange(i,true)[1]);
  }
  function fillFeedbackInputs(){
    if(!feedbackReady())return;
    jointInputs('position').forEach((input,i)=>input.value=degrees(state.manual.positions[i]));
    state.manual.jaw_gaps_mm.forEach((value,i)=>{
      const input=el('jaw-'+['left','right'][i]);
      if(Number.isFinite(value))input.value=Math.max(jawRange(i)[0],Math.min(jawRange(i)[1],value));
      else if(state.manual.disabled_grippers?.includes(['left','right'][i])&&!validJawDraft(input.value,i))input.value=jawRange(i)[1];
    });
    syncJointSliders();
  }
  function renderManual() {
    for(const [i,side]of ['left','right'].entries()){el('jaw-'+side).min=jawRange(i)[0];el('jaw-'+side).max=jawRange(i)[1];}
    setText('zero-preview',jawRange(0)[1]===83&&jawRange(1)[1]===83?'填入回零目标 · 夹爪83mm':`填入回零目标 · 左${jawRange(0)[1]} / 右${jawRange(1)[1]}mm`);
    const m=state?.manual, signature=JSON.stringify([draftKey(),m?.joint_names,m?.limits]);
    for(const [index,node]of [...el('joint-inputs').querySelectorAll('small')].filter(node=>node.dataset.role==='actual').entries())
      node.textContent=Number.isFinite(m?.positions?.[index])?'实测 '+degrees(m.positions[index]).toFixed(2)+'°':'实测未知';
    if(signature===manualKey)return;
    el('joint-inputs').replaceChildren();manualKey='';
    if(!manualReady())return;
    if(state?.runtime?.connected!==true||feedbackReady())manualKey=signature;
    const speedDetails=make('details',undefined,'joint-speeds');speedDetails.append(make('summary','逐关节速度 · rad/s'));
    m.joint_names.forEach((name,index)=>{
      const label=make('div',undefined,'joint-control'),title=make('label',name+' · 目标 °'),input=make('input'); input.type='number';input.step='0.05';input.id='joint-'+index;title.htmlFor=input.id;
      input.dataset.role='position';
      input.min=degrees(m.limits[index][0]);input.max=degrees(m.limits[index][1]);input.disabled=true;input.setAttribute('aria-label',name+' 目标角度 °');
      const slider=make('input');slider.type='range';slider.min=input.min;slider.max=input.max;slider.step='any';slider.dataset.role='slider';slider.setAttribute('aria-label',name+' 目标角度滑块');
      slider.addEventListener('input',()=>{input.value=slider.value;editManual();});
      input.addEventListener('input',()=>{slider.value=input.value;editManual();});
      const actual=make('small',Number.isFinite(m.positions?.[index])?'实测 '+degrees(m.positions[index]).toFixed(2)+'°':'实测未知','joint-feedback');actual.dataset.role='actual';
      const speed=make('input');speed.type='number';speed.min='0.001';speed.step='0.01';speed.value=m.default_joint_speed_rad_s??.2;speed.dataset.role='speed';speed.setAttribute('aria-label',name+' 速度 rad/s');speed.addEventListener('input',editManual);
      const speedLabel=make('label',name+' · rad/s','joint-speed');speedLabel.append(speed);speedDetails.append(speedLabel);
      title.append(actual);label.append(title,slider,input);el('joint-inputs').append(label);
    });
    el('joint-inputs').append(speedDetails);
    if(state?.runtime?.connected!==true){
      jointInputs('position').forEach((input,i)=>input.value=degrees(Math.max(m.limits[i][0],Math.min(m.limits[i][1],0))));
      el('jaw-left').value=jawRange(0)[1];el('jaw-right').value=jawRange(1)[1];syncJointSliders();
    }else fillFeedbackInputs();
    el('manual-speed').value=m.default_joint_speed_rad_s??.2;el('jaw-speed').value=m.default_jaw_speed_mm_s??10;
    restoreDraft();
  }
  function editManual(){
    finishAudit('人工姿态草稿改变');
    dismissManualExecution();
    if(state?.workflow?.primary_action?.id==='manual_execute'&&state.workflow.review?.id)invalidatedReviews.add(key()+'/'+state.workflow.review.id);
    saveDraft();clearReview();renderGates();
    clearTimeout(autoPreviewTimer);const binding=targetBinding();
    autoPreviewTimer=setTimeout(()=>{
      autoPreviewTimer=null;if(stopped||doc.hidden||tab!==context.business||binding!==targetBinding()||!permission('preview_target').allowed)return;
      try{const pose=displayPose();el('manual-error').hidden=true;void showTarget(pose.q12,pose.jaw_gap_mm);}
      catch(error){setText('manual-error',error.message);el('manual-error').hidden=false;}
    },33);
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
  const executionFieldMap={'arrival-jaw-tolerance':'arrival_jaw_mm','tracking-joint-tolerance':'tracking_joint_rad',
    'tracking-jaw-tolerance':'tracking_jaw_mm','tracking-duration':'tracking_duration_s'};
  const arrivalInputs=()=>[...el('arrival-tolerance-inputs').querySelectorAll('input')];
  const precisionNumeric={'precision-alignment-speed':'alignment_speed_rad_s','precision-home-speed':'home_speed_rad_s',
    'precision-jaw-speed':'jaw_speed_mm_s','precision-tcp-tolerance':'tcp_tolerance_mm'};
  const precisionGainInputs=()=>[...el('precision-gain-inputs').querySelectorAll('input')];
  function precisionSettingsPayload(){
    const saved=state?.precision_study?.settings;if(!saved)throw Error('等待研究配置');
    const result=JSON.parse(JSON.stringify(saved));
    const number=input=>{const n=Number(input.value);if(String(input.value).trim()===''||!Number.isFinite(n)||n<0||n===0&&input.dataset.gain!=='kd'||Number.isFinite(Number(input.max))&&n>Number(input.max))throw Error('KD可为0；其他研究数值须为有限正数，且均不超过显示上界');return n;};
    for(const input of precisionGainInputs())result.gains[input.dataset.side][input.dataset.gain][Number(input.dataset.joint)]=number(input);
    for(const[id,name]of Object.entries(precisionNumeric))result[name]=number(el(id));
    result.integral_enabled=el('precision-integral').checked;return result;
  }
  function fillPrecisionSettings(settings){
    const grid=el('precision-gain-inputs'),caps=state.precision_study.settings_caps;grid.replaceChildren();
    for(const side of ['left','right'])for(let joint=0;joint<6;joint++)for(const gain of ['kp','kd']){
      const cap=caps.gains[side][gain][joint],label=make('label',`${side==='left'?'左':'右'} J${joint+1} ${gain.toUpperCase()} · ≤${cap}`),input=make('input');
      input.type='number';input.min=gain==='kd'?'0':'0.001';input.max=String(cap);input.step= gain==='kp'?'1':'0.1';input.value=settings.gains[side][gain][joint];
      Object.assign(input.dataset,{side,gain,joint});input.addEventListener('input',()=>{precisionSettingsDirty=true;renderPrecisionStudy();});label.append(input);grid.append(label);
    }
    for(const[id,name]of Object.entries(precisionNumeric))el(id).value=settings[name];
    el('precision-integral').checked=settings.integral_enabled;
  }
  function renderPrecisionStudy(){
    const view=state?.precision_study;
    el('precision-study-tools').hidden=tab!=='research';
    setText('precision-context',`独立控制研究栏目 · 操作身份：${BUSINESS[context.business]} · 当前持权：${BUSINESS[state?.runtime?.holder]||'未接入'}。切换栏目不会交接控制权。`);
    const settingsKey=JSON.stringify([key(),serverInstance,state?.device?.id]);
    if(settingsKey!==precisionSettingsKey){precisionSettingsKey=settingsKey;precisionSettingsDirty=false;precisionSettingsRevision=null;}
    if(view?.settings&&(!precisionSettingsDirty||precisionSettingsRevision===null)&&precisionSettingsRevision!==view.settings_revision){
      precisionSettingsRevision=view.settings_revision;precisionSettingsDirty=false;fillPrecisionSettings(view.settings);
    }
    const configAllowed=permission('precision_settings'),stale=!!view&&precisionSettingsRevision!==view.settings_revision;
    let invalid='',draft;if(view?.settings)try{draft=precisionSettingsPayload();}catch(error){invalid=error.message;}
    const presets=view?.settings_presets||[],config=el('precision-config'),presetsKey=JSON.stringify([settingsKey,presets]);
    if(config.dataset.presets!==presetsKey){
      config.dataset.presets=presetsKey;config.replaceChildren();
      const custom=make('option','自定义 / 当前配置');custom.value='';config.append(custom);
      for(const row of presets){const option=make('option',row.label);option.value=row.id;config.append(option);}
    }
    config.value=presets.find(row=>draft&&JSON.stringify(row.settings)===JSON.stringify(draft))?.id||'';
    config.disabled=!view?.settings||!configAllowed.allowed;
    for(const input of [...precisionGainInputs(),...Object.keys(precisionNumeric).map(el),el('precision-integral')])input.disabled=!view?.settings||!configAllowed.allowed;
    el('precision-settings-save').disabled=!view?.settings||!configAllowed.allowed||!precisionSettingsDirty||stale||!!invalid;
    el('precision-settings-reset').disabled=!view?.settings;
    const savedConfig=presets.find(row=>JSON.stringify(row.settings)===JSON.stringify(view?.settings))?.label||'自定义';
    setText('precision-settings-status',!view?'当前设备尚未配置研究模块。':stale?'研究配置版本已变化，请重新读取；草稿未应用。':invalid?invalid:`已保存版本 ${view.settings_revision??'未知'} · 实际准备使用 ${savedConfig} · ${precisionSettingsDirty?'草稿未保存，请先保存或重新读取':'输入框与保存值一致'}；保存值在下一次新动作中才应用。`);
    const prepare=permission('precision_preview'),execute=permission('precision_execute'),analyze=permission('precision_analyze');
    const configReason=!view?.settings?'研究配置尚未读取，请等待核验':stale?'研究配置版本已变化，请先重新读取':invalid|| (precisionSettingsDirty?'研究配置草稿尚未保存；请先保存研究配置或重新读取，不能沿用旧配置执行':'');
    for(const id of ['precision-start','precision-05','precision-10','precision-15','precision-20','precision-home'])el(id).disabled=!view||!prepare.allowed||!!configReason;
    const speeds=view?.runtime_facts?.trajectory_speed_choices||[.5,1,1.5,2];
    for(const [id,speed]of [['precision-05',.5],['precision-10',1],['precision-15',1.5],['precision-20',2]]){
      el(id).hidden=!speeds.includes(speed);el(id).disabled||=!speeds.includes(speed);
    }
    el('precision-execute').disabled=!view||!execute.allowed||!!configReason;el('precision-analyze').disabled=!view||!analyze.allowed;
    el('precision-stop').disabled=!view||!permission('stop').allowed;
    const review=view?.review;
    setText('precision-review',review?`${review.summary} · 预计 ${Number(review.duration_s).toFixed(2)} 秒；点击“执行当前研究审阅”才运动。`:'尚无当前有效审阅；准备不会下发运动。');
    const latest=view?.latest,selected=view?.selected||latest,job=view?.request;
    const executionTrip=record=>{const trip=record?.speed_protection?.last_trip;return trip&&trip.command_epoch===record.binding?.command_epoch&&trip.sample_wall_ns>=record.started_wall_ns?trip:null;};
    const latestTrip=executionTrip(latest),observedPeak=latestTrip?Math.max(latest?.feedback_peak_rad_s||0,latestTrip.peak_rad_s):latest?.feedback_peak_rad_s;
    const protection=selected?.speed_protection?{...selected.speed_protection}:null;
    if(protection){delete protection.last_trip;protection.current_execution_trip=executionTrip(selected);protection.historical_last_trip=protection.current_execution_trip?null:selected.speed_protection.last_trip;}
    setText('precision-status',job?.status==='running'?'研究请求处理中：'+job.action:job?.status==='failed'?'研究请求失败：'+operatorText(job.error):latest?`${latest.summary} · ${latest.status} · 已记录 ${latest.feedback_samples??0} 样本 · 已观测峰值 ${observedPeak??'待记录'} rad/s`:'等待显式操作；不会接入设备或执行动作。');
    setText('precision-reason',configReason||view?.reason||'0.8 rad/s保护开启；停止独立可用，恢复不会续跑旧动作。');
    setText('precision-report',selected?JSON.stringify({analysis:selected.analysis,status:selected.status,reason:selected.reason,
      final_state:selected.final_state,stationary:selected.stationary,speed_protection:protection,
      frozen_settings:selected.settings,frozen_execution_settings:selected.execution_settings},null,2):'');
    const history=view?.history||[],historyKey=JSON.stringify([settingsKey,history,selected?.id]);
    if(historyKey!==precisionHistoryKey){precisionHistoryKey=historyKey;const box=el('precision-history');box.replaceChildren();for(const row of history){const option=make('option',`${row.summary} · 配置v${row.settings_revision} · ${row.status}`);option.value=row.id;box.append(option);}box.value=selected?.id||history[0]?.id||'';}
    el('precision-select').disabled=!permission('precision_select').allowed||!history.length;
    const notesKey=JSON.stringify([settingsKey,selected?.id,selected?.notes_revision]);
    if(notesKey!==precisionNotesKey){precisionNotesKey=notesKey;precisionNotesDirty=false;el('precision-notes').value=selected?.notes||'';}
    el('precision-notes').disabled=!selected;el('precision-notes-save').disabled=!permission('precision_notes').allowed||!selected||!precisionNotesDirty;
    setText('precision-record-identity',selected?`记录 ${selected.id} · 配置v${selected.settings_revision} · 结论v${selected.notes_revision||0}`:'尚无本模块试验记录。');
    setText('precision-notes-status',precisionNotesDirty?'结论草稿尚未保存。':'结论随选中试验保存，不改写执行参数或历史数值。');
    setText('precision-runtime-facts',view?.runtime_facts?JSON.stringify(view.runtime_facts,null,2):'尚未读取当前控制事实。');
  }
  const speedProtectionAvailable=settings=>typeof settings?.speed_protection?.enabled==='boolean'&&Number.isFinite(settings.speed_protection.limit_rad_s)&&settings.speed_protection.limit_rad_s>0;
  function speedProtectionPayload(){
    const raw=el('speed-protection-limit').value,limit=Number(raw);
    if(String(raw).trim()===''||!Number.isFinite(limit)||limit<=0)throw Error('速度阈值须为有限正数（rad/s）');
    const view=state?.runtime?.execution_settings;
    if(view?.business!==context.business||!speedProtectionAvailable(view?.settings))throw Error('尚未读取实际速度保护设置');
    return {...view.settings,speed_protection:{enabled:el('speed-protection-enabled').checked,limit_rad_s:limit}};
  }
  function renderControlPolicy(){
    const identity=JSON.stringify([key(),serverInstance,state?.device?.id,state?.runtime?.runtime_nonce]);
    if(identity!==policyKey){
      policyKey=identity;policySnapshot=null;policyFields=[];policyLoading=false;policyDirty=false;
      el('control-policy-fields').replaceChildren();setText('control-policy-status','尚未读取当前业务实际策略。');
    }
    const currentRevision=state?.runtime?.execution_settings?.revision;
    const stale=!!policySnapshot&&Number.isInteger(currentRevision)&&currentRevision!==policySnapshot.revision;
    const allowed=permission('control_policy'),editable=!!policySnapshot&&policySnapshot.update_allowed&&allowed.allowed&&!policyLoading&&!stale;
    el('control-policy-tools').hidden=!BUSINESS[tab];
    el('control-policy-load').disabled=!state?.csrf||!options().fresh||options().pending||policyLoading;
    el('control-policy-save').disabled=!editable||!policyDirty;
    for(const field of policyFields)for(const input of field.inputs)input.disabled=!editable;
    setText('control-policy-reason',policyLoading?'正在读取实际策略。':!policySnapshot?'读取不会执行动作。':
      stale?'当前策略版本已变化；草稿未应用，请重新读取后修改。':!allowed.allowed?allowed.reason:!policySnapshot.update_allowed?policySnapshot.update_reason||'当前策略不可修改。':
      policyDirty?'草稿尚未应用；提交时核验读取版本，冲突时保留草稿。':'输入框与读取版本一致。');
  }
  function fillControlPolicy(value){
    if(!value||!Number.isInteger(value.revision)||!value.parameters||!Array.isArray(value.editable_paths))throw Error('统一策略回执不完整');
    const names={measured:'实测速度保护',tracking:'跟踪误差',arrival:'到位误差',planning:'轨迹规划',observation:'模型观测',model:'模型供给',execution:'执行',recording:'记录',
      enabled:'启用',limit_rad_s:'关节速度上限 rad/s',joint_rad:'关节误差 rad',jaw_mm:'夹爪误差 mm',duration_s:'持续时间 s',
      tolerances_rad:'各关节到位误差 rad',velocity_limit_rad_s:'关节速度上限 rad/s',acceleration_limit_rad_s2:'关节加速度上限 rad/s²',
      jaw_velocity_limit_mm_s:'夹爪速度上限 mm/s',maximum_age_s:'观测最大年龄 s',state_max_age_s:'配对反馈最大年龄 s',max_camera_span_s:'三相机最大跨度 s',reference_supply_timeout_s:'等待新观测窗口 s',
      execute_steps:'每段执行步数',prepare_drift_joint_rad:'准备期间关节漂移 rad',prepare_drift_jaw_mm:'准备期间夹爪漂移 mm',
      event_queue_bytes:'事件队列预算 bytes',message_bytes:'单条消息上限 bytes',purpose_profile:'录制用途',fake_error:'仿真误差',
      joint_bias_max_rad:'关节偏置上限 rad',seed:'随机种子'};
    policySnapshot=value;policyDirty=false;policyFields=[];const container=el('control-policy-fields');container.replaceChildren();
    for(const path of value.editable_paths){
      const parts=path.split('.'),original=parts.reduce((row,key)=>row?.[key],value.parameters);
      if(original===undefined)throw Error('策略字段缺少实际值：'+path);
      const text=parts.map(part=>names[part]||part).join(' · '),array=Array.isArray(original);
      const label=doc.createElement(array?'fieldset':'label');let parent=label;
      if(array){const legend=doc.createElement('legend');legend.textContent=text;parent=doc.createElement('div');parent.className='arrival-tolerance-grid';label.append(legend,parent);}
      else label.textContent=text;
      label.title='来源：'+(value.origins?.[path]||'未声明');const inputs=[];
      for(const [index,entry] of (Array.isArray(original)?original:[original]).entries()){
        const input=doc.createElement(path==='recording.purpose_profile'?'select':'input');
        if(path==='recording.purpose_profile'){
          for(const [key,text] of [['','请选择用途'],['data_required','数据完整性必需'],['infer_debug','Infer 调试允许缺口']]){
            const option=doc.createElement('option');option.value=key;option.textContent=text;input.append(option);
          }
          input.value=entry??'';
        }else if(typeof entry==='boolean'){input.type='checkbox';input.checked=entry;}
        else{input.type='number';input.step=path==='execution.execute_steps'||path.endsWith('_bytes')||path.endsWith('.seed')?'1':'any';input.value=String(entry);}
        const axis=(index<6?'左':'右')+' J'+(index%6+1);
        input.setAttribute('aria-label',text+(array?' · '+axis:''));
        input.addEventListener('input',()=>{policyDirty=true;renderControlPolicy();});
        if(array){const item=doc.createElement('label');item.textContent=axis;item.append(input);parent.append(item);}else parent.append(input);
        inputs.push(input);
      }
      policyFields.push({path,original,inputs});container.append(label);
    }
    setText('control-policy-status',`${BUSINESS[context.business]} · 版本 ${value.revision} · 策略 ${value.policy_hash} · 下一 Run 生效`);
    renderControlPolicy();
  }
  async function loadControlPolicy(){
    renderControlPolicy();if(el('control-policy-load').disabled)return;
    const identity=policyKey;policyLoading=true;renderControlPolicy();
    try{
      const response=await request('/api/control-policy?'+new URLSearchParams(context));const value=await response.json();
      if(!response.ok)throw Error(value.message||value.error||'无法读取统一策略');
      if(!stopped&&identity===policyKey)fillControlPolicy(value);
    }catch(error){if(!stopped&&identity===policyKey)setText('control-policy-status','读取失败：'+error.message);}
    finally{if(!stopped&&identity===policyKey){policyLoading=false;renderControlPolicy();}}
  }
  function controlPolicyChanges(){
    const changes={};
    for(const {path,original,inputs} of policyFields){
      const values=inputs.map(input=>{
        if(input.type==='checkbox')return input.checked;
        if(path==='recording.purpose_profile'){if(!input.value)throw Error('请选择录制用途');return input.value;}
        const number=Number(input.value);if(!String(input.value).trim()||!Number.isFinite(number))throw Error('参数必须为有限数值');return number;
      });
      const value=Array.isArray(original)?values:values[0];if(JSON.stringify(value)!==JSON.stringify(original))changes[path]=value;
    }
    if(!Object.keys(changes).length)throw Error('没有参数变更');return changes;
  }
  function renderSpeedProtection(reset=false){
    const view=state?.runtime?.execution_settings,settings=view?.business===context.business?view.settings:null;
    const valid=state?.runtime?.connected===true&&speedProtectionAvailable(settings);
    const signature=JSON.stringify([key(),serverInstance,state?.device?.id,state?.runtime?.runtime_nonce]);
    if(signature!==speedProtectionKey){speedProtectionKey=signature;speedProtectionDirty=false;speedProtectionRevision=null;}
    if(valid&&(reset||speedProtectionRevision===null||!speedProtectionDirty&&speedProtectionRevision!==view.revision)){
      speedProtectionDirty=false;speedProtectionRevision=view.revision;
      el('speed-protection-enabled').checked=settings.speed_protection.enabled;el('speed-protection-limit').value=settings.speed_protection.limit_rad_s;
    }
    const allowed=permission('execution_settings'),stale=valid&&speedProtectionRevision!==view.revision;
    for(const id of ['speed-protection-enabled','speed-protection-limit'])el(id).disabled=!valid||!allowed.allowed;
    let invalid='';if(valid)try{speedProtectionPayload();}catch(error){invalid=error.message;}
    setText('speed-protection-effective',valid?`当前生效 · ${BUSINESS[context.business]} · 版本 ${view.revision} · ${settings.speed_protection.enabled?'开启':'关闭'} · 阈值 ${settings.speed_protection.limit_rad_s} rad/s`:'实际设置未核验：设备未连接或当前后端未提供速度保护。默认值不代表当前真机已启用。');
    el('speed-protection-apply').disabled=!valid||!allowed.allowed||!speedProtectionDirty||stale||!!invalid;
    el('speed-protection-reset').disabled=!valid||options().pending||!options().fresh;
    setText('speed-protection-draft',!valid?'等待实际设置。':stale?'生效版本已变化；草稿未应用，请重新读取后修改。':invalid?'草稿未应用：'+invalid:speedProtectionDirty?'有未应用的草稿；保护仍使用上方当前生效值。':'输入框与当前生效设置一致。');
    setText('speed-protection-reason',!allowed.allowed?allowed.reason:'保存仅影响当前业务并撤销旧审批，不执行动作、不清除故障。');
  }
  function executionSettingsPayload(){
    const number=(node,label,zero=false)=>{const value=Number(node.value);if(String(node.value).trim()===''||!Number.isFinite(value)||(zero?value<0:value<=0))throw Error(label+'须为有限'+(zero?'非负':'正')+'数');return value;};
    const arrival=arrivalInputs();if(arrival.length!==12)throw Error('等待当前 12 关节设置');
    const settings={arrival_enabled:!el('arrival-observe-only').checked,arrival_tolerances_rad:arrival.map(input=>number(input,'到位阈值')),
      tracking_enabled:!el('tracking-observe-only').checked};
    for(const [id,name]of Object.entries(executionFieldMap))settings[name]=number(el(id),'误差阈值或持续时间',name==='tracking_duration_s');
    const seed=number(el('fake-error-seed'),'随机种子',true);if(!Number.isInteger(seed)||seed>4294967295)throw Error('随机种子须为 0～4294967295 的整数');
    const bias=number(el('fake-error-max'),'随机偏置幅度',true);if(bias>.05)throw Error('随机偏置幅度须在 0～0.05 rad 内');
    settings.fake_error={enabled:context.environment==='fake'&&el('fake-error-enabled').checked,joint_bias_max_rad:bias,seed};
    const saved=state?.runtime?.execution_settings?.settings;
    if(!speedProtectionAvailable(saved))throw Error('尚未读取实际速度保护设置');
    settings.speed_protection={...saved.speed_protection};
    return settings;
  }
  function renderExecutionSettings(reset=false){
    renderSpeedProtection();
    const view=state?.runtime?.execution_settings,settings=view?.business===context.business?view.settings:null;
    const signature=JSON.stringify([key(),serverInstance,state?.device?.id,state?.runtime?.runtime_nonce]);
    const valid=settings&&Array.isArray(settings.arrival_tolerances_rad)&&settings.arrival_tolerances_rad.length===12&&speedProtectionAvailable(settings);
    if(signature!==executionSettingsKey){executionSettingsDirty=false;executionSettingsRevision=null;executionSettingsKey=signature;}
    if(valid&&(reset||executionSettingsRevision===null||!executionSettingsDirty&&executionSettingsRevision!==view.revision)){
      executionSettingsDirty=false;executionSettingsRevision=view.revision;
      el('arrival-observe-only').checked=!settings.arrival_enabled;el('tracking-observe-only').checked=!settings.tracking_enabled;
      const grid=el('arrival-tolerance-inputs');grid.replaceChildren();
      for(const [index,value]of settings.arrival_tolerances_rad.entries()){
        const label=make('label',(index<6?'左':'右')+' J'+(index%6+1)+' · rad'),input=make('input');input.type='number';input.min='0.000001';input.step='0.001';input.value=value;
        input.addEventListener('input',()=>{executionSettingsDirty=true;renderExecutionSettings();});label.append(input);grid.append(label);
      }
      for(const [id,name]of Object.entries(executionFieldMap))el(id).value=settings[name];
      el('fake-error-enabled').checked=settings.fake_error.enabled;el('fake-error-max').value=settings.fake_error.joint_bias_max_rad;el('fake-error-seed').value=settings.fake_error.seed;
    }
    const scope=BUSINESS[context.business],policy=enabled=>enabled?'超差保护':'仅记录';
    const summary=valid?`${scope} · 到位${policy(settings.arrival_enabled)} · 跟踪${policy(settings.tracking_enabled)}${context.environment==='fake'?' · 随机误差'+(settings.fake_error.enabled?'开启':'关闭'):''}`:`${scope} · 连接后读取当前业务设置`;
    setText('execution-settings-summary',summary);
    setText('execution-settings-effective',valid?`当前生效 · 版本 ${view.revision} · ${summary}。到位 12 关节 ${settings.arrival_tolerances_rad.map(value=>Number(value).toPrecision(4)).join(' / ')} rad，夹爪 ${settings.arrival_jaw_mm} mm；跟踪 ${settings.tracking_joint_rad} rad / ${settings.tracking_jaw_mm} mm / ${settings.tracking_duration_s} s。`+(view.legacy_passive_guards?' 未自定义时保留原被动轴保护（2° / 2 mm）；应用后所有轴按本页跟踪设置执行。':''):'设备未连接或当前后端未提供执行误差设置。');
    el('fake-error-settings').hidden=context.environment!=='fake';
    const allowed=permission('execution_settings'),stale=valid&&executionSettingsRevision!==view.revision;
    let invalid='';if(valid)try{executionSettingsPayload();}catch(error){invalid=error.message;}
    const fieldIds=['arrival-observe-only','tracking-observe-only',...Object.keys(executionFieldMap),'fake-error-enabled','fake-error-max','fake-error-seed'];
    for(const input of [...arrivalInputs(),...fieldIds.map(el)])input.disabled=!valid||!allowed.allowed;
    if(view?.fake_error_available===false)for(const id of ['fake-error-enabled','fake-error-max','fake-error-seed'])el(id).disabled=true;
    setText('fake-error-capability',context.environment==='fake'&&view?.fake_error_available===false?'当前后端不支持物理仿真误差注入；此项需要完整 Fake 服务。':'');
    el('execution-settings-apply').disabled=!valid||!allowed.allowed||!executionSettingsDirty||stale||!!invalid;
    el('execution-settings-reset').disabled=!valid||options().pending||!options().fresh;
    setText('execution-settings-draft',stale?'生效版本已变化；当前草稿尚未应用，请重新读取生效设置后修改。':invalid?'草稿未应用：'+invalid:executionSettingsDirty?'有未应用的草稿；上方“当前生效”仍是本次实际执行口径。':'输入框与当前生效设置一致。');
    setText('execution-settings-reason',!allowed.allowed?allowed.reason:!executionSettingsDirty?'修改后点击应用；应用不会执行动作。':'应用会撤销原执行审批，下一次动作须重新审阅。');
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
      planIndex=Math.min(planIndex,Math.max(0,waypoints.length-1));el('calibration-plan-select').replaceChildren();
      for(const [index,row]of waypoints.entries()){
        const title=`${index+1}. ${row.capture===false?'仅过渡':'采样'} · ${row.name||row.id||row.kind||'姿态'}`;
        const option=make('option',title);option.value=index;el('calibration-plan-select').append(option);
        const li=make('li'),button=make('button','查看此姿态图像 · 不执行');button.addEventListener('click',()=>browsePlan(index));planPreviewButtons.push(button);li.append(make('strong',title),make('p',`关节 °：${(row.q||[]).map(value=>degrees(value).toFixed(1)).join(' / ')}；速度 rad/s：${(row.speeds||[]).join(' / ')}；夹爪 mm：${(row.jaw_gap_mm||[]).join(' / ')}`),button);list.append(li);
      }
      const coverage=el('calibration-coverage');coverage.replaceChildren();
      for(let cell=0;cell<9;cell++){const count=capturing.filter(row=>Array.isArray(row.cell)&&row.cell[0]*3+row.cell[1]===cell).length;coverage.append(make('div',`视野 ${Math.floor(cell/3)+1}行 ${cell%3+1}列 · ${count} 点`));}
    }
    el('calibration-plan-select').value=planIndex;
    setText('calibration-plan-position',waypoints.length?`姿态 ${planIndex+1} / ${waypoints.length} · 查看只更新图像，不执行动作`:'先生成计划，再逐姿态审阅');
    el('calibration-coverage').hidden=tools?.workflow?.kind==='wrist'||!waypoints.some(row=>Array.isArray(row.cell));
    const workflow=state?.workflow||{},pipeline=workflow.pipeline||{},steps=el('calibration-pipeline-steps');steps.replaceChildren();
    const milestones=['head.plan','head.capture','head.solve','wrist.plan','wrist.capture','wrist.solve','install'];
    const current=workflow.stage_id?.replace('.review','.capture').replace('.fit','.solve');
    const stageFor=step=>step.step==='install'?'install':step.step==='head'?'head.solve':step.step==='wrist'?'wrist.solve':step.half+'.'+step.step;
    for(const [index,stage]of milestones.entries()){
      const step=(pipeline.steps||[]).filter(row=>stageFor(row)===stage).at(-1)||{},metrics=step.metrics||{};
      const done=!!step.step||index<(workflow.overall_progress?.completed_stages??0)||stage==='install'&&workflow.quality?.installed?.verified===true;
      const active=current===stage&&!done,failed=workflow.status==='failed'&&workflow.failed_stage===stage;
      const row=make('li',undefined,'workflow-step'+(active?' is-current':''));row.append(make('strong',STAGE_LABELS[stage]));
      const descriptions=[];
      if(Number.isFinite(step.waypoints))descriptions.push(`${step.waypoints} 个路径点 / ${step.capture_points??'未知'} 个采样点`);
      if(step.counts)descriptions.push(`已采 ${step.counts.captured??'未知'} 帧 · 有效 ${step.counts.valid??'未知'} · 失败 ${step.counts.failed??0}`);
      if(Number.isFinite(metrics.holdout_marker_rms_px))descriptions.push('留出贴纸误差 '+metrics.holdout_marker_rms_px.toFixed(3)+' px');
      if(Number.isFinite(metrics.holdout_mesh_px))descriptions.push('Mesh 留出误差 '+metrics.holdout_mesh_px.toFixed(2)+' px');
      if(Number.isFinite(metrics.holdout_mesh_coverage))descriptions.push('Mesh 覆盖 '+(metrics.holdout_mesh_coverage*100).toFixed(1)+'%');
      for(const side of ['left','right'])if(Number.isFinite(metrics[side]?.holdout_rms_px))descriptions.push((side==='left'?'左腕':'右腕')+'留出误差 '+metrics[side].holdout_rms_px.toFixed(3)+' px');
      if(step.pair_status)descriptions.push('成对状态：'+statusText(step.pair_status));
      const status=done?'已完成':failed?'未通过':active?statusText(workflow.status):'待开始';
      row.append(make('p',status+(descriptions.length?' · '+descriptions.join(' · '):step.message?' · '+step.message:'')));steps.append(row);
    }
    const quality=state?.workflow?.quality||{},qualityCounts=quality.counts||counts;
    setText('calibration-quality-summary',`当前来源：${qualityCounts.captured??qualityCounts.frames??0} 帧 · 求解有效 ${qualityCounts.valid??'未知'} · 无效 ${qualityCounts.failed??'未知'}。`+(quality.completion?`采集完整性：${quality.completion.passed===true?'通过':quality.completion.passed===false?'未通过':'尚未核验'}。`:'')+(pipeline.aborted?'中断原因：'+pipeline.aborted: pipeline.stopped?'已停止；已保存数据保留，继续前须重新审阅。':''));
  }
  function browsePlan(index){
    const rows=state?.workflow?.calibration_tools?.waypoints||[];if(!rows.length)return;
    planIndex=Math.max(0,Math.min(rows.length-1,Number(index)||0));renderCalibration();renderGates();
    return showTarget(rows[planIndex].q,rows[planIndex].jaw_gap_mm);
  }
  let inferenceStepsDirty=false,inferenceStepsRevision=null,inferenceStepsContext=null;
  function renderInferenceSteps(reset=false){
    const value=state?.inference_execution,settingContext=JSON.stringify([context.environment,serverInstance]);
    el('inference-execution-settings').hidden=context.business!=='inference'||tab!=='inference';
    if(reset||settingContext!==inferenceStepsContext){inferenceStepsDirty=false;inferenceStepsRevision=null;inferenceStepsContext=settingContext;}
    if(value&&!inferenceStepsDirty){el('inference-execute-steps').value=String(value.execute_steps);inferenceStepsRevision=value.revision;}
    const input=el('inference-execute-steps'),steps=Number(input.value),horizon=value?.action_horizon??60,stale=value&&inferenceStepsRevision!==value.revision;
    input.max=String(horizon);
    const gate=preparedParameterHold()?{allowed:!inferenceParameterUpdate,reason:'正在应用参数并重新准备'}:permission('inference_execution_settings');
    input.disabled=!value||!gate.allowed;
    el('inference-steps-apply').disabled=!value||!gate.allowed||!inferenceStepsDirty||stale||!Number.isInteger(steps)||steps<1||steps>horizon;
    el('inference-steps-reset').disabled=!value;
    setText('inference-steps-effective',value?`下一轮生效：执行 ${value.execute_steps} / ${horizon} 步（${(value.execute_steps/30).toFixed(2)} 秒动作）。`+
      (stale?'设置已被更新；请重新读取后修改。':inferenceStepsDirty?'草稿未保存。':'')+(!gate.allowed?' '+gate.reason:''):'当前环境未配置推理服务。');
  }
  function inferenceSeed(){
    const raw=el('inference-seed').value,value=Number(raw);
    if(String(raw).trim()===''||!Number.isInteger(value)||value<0||value>4294967295)throw Error('推理随机种子须为 0～4294967295 的整数');
    return value;
  }
  function inferenceDuration(){
    const raw=el('inference-duration').value,value=Number(raw);
    if(String(raw).trim()===''||!Number.isFinite(value)||value<=0)throw Error('请输入大于0的有限运行秒数');
    if(inferenceInfo().finite_chunk_count&&value!==.5)throw Error('当前验证每轮固定15步、模型源时长0.5秒');
    return value;
  }
  const inferenceInfo=()=>state?.workflow?.inference||{};
  let inferenceParameterUpdate=null;
  function preparedParameterHold(){
    const info=inferenceInfo(),mode=info.normal_mode||{};
    return !!info.operator_session&&mode.mode==='HOLD'&&mode.runner_status==='RUNNING'&&!mode.transition_stage
      &&options().fresh&&!options().pending&&!environmentAccess?.state?.active_job
      &&state?.runtime?.feedback_healthy===true&&state?.runtime?.stationary===true;
  }
  async function renewPreparedParameters(apply=async()=>true,{model=null}={}){
    if(inferenceParameterUpdate)return false;
    if(!preparedParameterHold())return apply();
    let seed,duration;try{seed=inferenceSeed();duration=inferenceDuration();}catch(error){notice(error.message);return;}
    const session=inferenceInfo().operator_session;
    const update={context:key(),server:serverInstance,nonce:state.runtime.runtime_nonce,run:session.run_id,
      model:model?.id||model?.model_id||inferenceModelId(),digest:model?.bundle_digest||inferenceDigest(),seed,duration,
      trick:el('inference-stall-enabled').checked,stop:null,stopBound:false,
      applicationGeneration:inferenceInfo().run_context?.application_generation};
    inferenceParameterUpdate=update;
    const stopIdentity=()=>state?.runtime?.stop_receipt?.accepted_monotonic_s??null;
    const check=()=>{
      if(stopped||inferenceParameterUpdate!==update||key()!==update.context||serverInstance!==update.server
          ||state?.runtime?.runtime_nonce!==update.nonce||state?.runtime?.recoverable_fault
          ||update.stopBound&&stopIdentity()!==update.stop
          ||!update.stopBound&&inferenceInfo().run_context?.application_generation!==update.applicationGeneration)
        throw Error('控制上下文或Stop已撤销参数更新，不会重新准备');
    };
    async function waitFor(predicate){
      const deadline=clock.now()+(inferenceInfo().operator_controls?.timeout_s??90)*1000;
      while(true){
        check();await poll();check();
        if(options().fresh&&predicate())return;
        if(clock.now()>=deadline)throw Error('参数更新等待超时；设备保持，未自动重试');
        await new Promise(resolve=>setTimeout(resolve,100));
      }
    }
    try{
      if(!await send('cancel',{},'parameter-renewal'))throw Error('结束旧会话的请求未确认');
      await waitFor(()=>state.workflow?.id===update.run&&inferenceInfo().recording_ingress_closed===true
        &&!state.workflow.busy&&state.runtime.stationary&&state.runtime.feedback_healthy);
      update.stop=stopIdentity();update.stopBound=true;notice('旧Run已结束取流；正在应用参数，录像在后台封存。',true);
      check();await environmentAccess?.refresh?.();check();
      if(!await apply())throw Error('参数应用未确认，未重新准备控制');
      if(model)await waitFor(()=>{
        const jobs=[environmentAccess?.state?.active_job,...environmentAccess?.state?.recent_jobs||[]];
        const failed=jobs.find(row=>row?.request?.action==='prepare_model'&&row.request.parameters?.expected_bundle_digest===update.digest
          &&['failed','cancelled'].includes(row.status));
        if(failed)throw Error(failed.error||'模型准备未通过');
        return inferenceDigest()===update.digest&&inferenceInfo().model_ready===true&&!environmentAccess?.state?.active_job;
      });
      await waitFor(()=>permission('inference_run').allowed===true);
      check();
      if(inferenceModelId()!==update.model||inferenceDigest()!==update.digest)throw Error('所选模型已变化，未重新准备控制');
      const result=await send('inference_run',{prepare_only:true,physical_clearance:true,scene_reset:true,
        model_id:update.model,expected_bundle_digest:update.digest,expected_context:inferenceInfo().run_context,
        state_wall_ns:state.server_wall_ns,seed:update.seed,duration_s:update.duration,wam_stall_recovery:update.trick},'parameter-renewal');
      if(!result)throw Error('新控制会话的准备请求未确认');
      await waitFor(()=>{
        const request=state.workflow?.request;
        if(request?.id===result.job_id&&['failed','cancelled'].includes(request.status))throw Error(request.error||'新会话准备失败');
        const current=inferenceInfo().operator_session;
        return !!current&&current.run_id!==update.run&&current.duration_s===update.duration
          &&current.seed===update.seed&&current.expected_bundle_digest===update.digest&&permission('inference_run').allowed===true;
      });
      notice('参数已应用，控制与踏板已重新准备；SDK连接保持。');return true;
    }catch(error){notice('参数更新未完成：'+error.message);return false;}
    finally{if(inferenceParameterUpdate===update)inferenceParameterUpdate=null;renderGates();renderInferenceSteps();}
  }
  let inferenceDenoiseContext=null,inferenceDenoiseDirty=false;
  let operatorClientId=null,operatorFocusPending=false,operatorFocusActive=false,operatorFocusDesired=false,operatorFocusLease=null;
  let operatorHomeIdentity=null,operatorHomeContext=null;
  const operatorIdentity=()=>JSON.stringify([key(),state?.server_instance_id,state?.runtime?.runtime_nonce,state?.runtime?.command_epoch,state?.runtime?.holder]);
  async function operatorFocus(active){
    active=active??(context.business==='inference'&&tab==='inference'&&!doc.hidden&&!el('work-pane').hidden);
    operatorFocusDesired=active;
    if(operatorFocusPending||!state?.csrf||active&&!inferenceInfo().operator_controls?.pedal_source_descriptor)return;
    if(!active&&!operatorFocusActive)return;
    operatorClientId??=newRequestId(crypto);operatorFocusPending=true;
    const lease=active?{environment:context.environment,runtime_nonce:state.runtime?.runtime_nonce,epoch:state.runtime?.command_epoch}:operatorFocusLease;
    try{
      const response=await request('/api/actions/inference_focus',{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':state.csrf},
        body:JSON.stringify({...lease,business:'inference',request_id:newRequestId(crypto),client_id:operatorClientId,active})});
      if(response.ok){operatorFocusActive=active;operatorFocusLease=active?lease:null;}
    }catch{}finally{operatorFocusPending=false;if(active!==operatorFocusDesired)void operatorFocus(operatorFocusDesired);}
  }
  const inferenceDenoiseValue=()=>Number(el('inference-denoise-steps').value);
  function inferenceDenoiseRow(){
    const info=inferenceInfo(),rows=info.models||[],backend=inferenceBackend();
    return rows.find(row=>(row.id||row.model_id)===info.selected_model_id&&row.backend===backend
      &&row.inference_options?.denoise_steps===inferenceDenoiseValue());
  }
  function renderInferenceDenoise(){
    const info=inferenceInfo(),wam=inferenceFamilySelection()==='wam',backend=inferenceBackend();
    const row=(info.models||[]).find(row=>(row.id||row.model_id)===info.selected_model_id&&row.backend===backend)
      ||(info.backend===backend&&info.inference_capabilities?info:null)
      ||(info.models||[]).find(row=>inferenceFamily(row)==='wam'&&row.backend===backend);
    const bounds=row?.inference_capabilities?.denoise_steps,input=el('inference-denoise-steps');
    const identity=JSON.stringify([key(),serverInstance,info.selected_model_id,backend]);
    const actual=info.backend===backend?info.inference_options?.denoise_steps:undefined;
    if(identity!==inferenceDenoiseContext){inferenceDenoiseContext=identity;inferenceDenoiseDirty=false;}
    if(!inferenceDenoiseDirty&&bounds)input.value=String(actual??bounds.default);
    if(actual===inferenceDenoiseValue())inferenceDenoiseDirty=false;
    el('inference-denoise-tools').hidden=!wam;
    if(bounds){input.min=String(bounds.minimum);input.max=String(bounds.maximum);}
    input.disabled=!wam||!bounds||!!state?.workflow?.busy&&!preparedParameterHold()||!options().fresh||!!environmentAccess?.state?.active_job||!!inferenceParameterUpdate;
    const desired=inferenceDenoiseRow();
    el('inference-denoise-apply').disabled=input.disabled||!desired||environmentAccess?.permission('prepare_model').allowed!==true&&!preparedParameterHold()
      ||desired.bundle_digest===info.bundle_digest&&info.model_ready===true;
    setText('inference-denoise-status',!bounds?'当前模型包未声明去噪配置能力。':
      `模型包范围 ${bounds.minimum}–${bounds.maximum}，默认 ${bounds.default}。当前已绑定：${actual??'尚未绑定'} 步。`+
      (inferenceDenoiseDirty?'草稿未应用；应用后重新准备模型，下一 Run 生效。':'应用配置会重新准备模型，不执行动作。'));
  }
  async function prepareInferenceRow(row){
    if(!row?.selectable)return;
    dismissInferenceRun();clearReview();const id=row.id||row.model_id;
    const selection=inferenceSelection={id,bundle_digest:row.bundle_digest,key:JSON.stringify([key(),serverInstance,inferenceFamilySelection(),inferenceBackend()]),submitting:true};renderInference();renderGates();
    try{return await prepareEnvironment('prepare_model',{model_id:id,expected_bundle_digest:row.bundle_digest});}
    catch(error){notice('模型准备请求未确认：'+error.message);}
    finally{selection.submitting=false;if(!stopped&&inferenceSelection===selection){renderInference();renderGates();}}
  }
  const inferenceModelId=()=>inferenceInfo().selected_model_id||inferenceInfo().model?.id;
  const inferenceDigest=()=>inferenceInfo().bundle_digest||inferenceInfo().model?.bundle_digest;
  const inferenceBackend=()=>el('inference-backend').value||'torch';
  const inferenceFamilySelection=()=>el('inference-family').value==='wam'?'wam':'pi';
  const inferenceBackendLabel=()=>INFERENCE_BACKENDS[inferenceFamilySelection()].find(([id])=>id===inferenceBackend())?.[1]||inferenceBackend();
  function setInferenceBackends(preferred){
    const select=el('inference-backend'),rows=INFERENCE_BACKENDS[inferenceFamilySelection()];select.replaceChildren();
    for(const [id,label]of rows){const option=make('option',label);option.value=id;select.append(option);}
    const bound=inferenceFamily({id:inferenceModelId()})===inferenceFamilySelection()?(inferenceInfo().backend||inferenceInfo().model?.backend):null;
    select.value=rows.some(([id])=>id===preferred)?preferred:rows.some(([id])=>id===bound)?bound:rows[0][0];
  }
  const inferenceBackendReason=()=>{
    if(inferenceFamilySelection()!==inferenceFamily({id:inferenceModelId()})||inferenceBackend()!==(inferenceInfo().backend||inferenceInfo().model?.backend||'torch'))
      return '请先选择并加载 '+(inferenceFamilySelection()==='wam'?'WAM':'π')+' / '+inferenceBackendLabel()+' 的 checkpoint';
    if(inferenceFamilySelection()==='wam'&&inferenceDenoiseDirty&&inferenceDenoiseValue()!==inferenceInfo().inference_options?.denoise_steps)
      return '请先应用去噪配置并等待模型准备完成';
    return '';
  };
  const inferenceRunBinding=()=>JSON.stringify([key(),serverInstance,state?.device?.id,inferenceInfo().run_context,
    inferenceModelId(),inferenceDigest(),inferenceFamilySelection(),inferenceBackend(),el('inference-seed').value,el('inference-duration').value,el('inference-stall-enabled').checked]);
  function dismissInferenceRun(){
    inferenceRunIntent=null;el('inference-run-clearance').checked=false;el('inference-run-confirm').disabled=true;
    if(el('inference-run-dialog').open)el('inference-run-dialog').close();
  }
  function renderInferenceRun(){
    if(!inferenceRunIntent)return;
    if(inferenceRunIntent.binding!==inferenceRunBinding()||inferenceBackendReason()||!options().fresh||!permission('inference_run').allowed){
      dismissInferenceRun();return;
    }
    el('inference-run-confirm').disabled=!el('inference-run-clearance').checked;
  }
  let inferenceRunOpening=false;
  async function requestInferenceRun(){
    if(inferenceRunOpening)return;
    inferenceRunOpening=true;
    try{
      if(preparedParameterHold()&&Number(el('inference-duration').value)!==inferenceInfo().operator_session.duration_s
          &&!await renewPreparedParameters())return;
      openInferenceRun();
    }finally{inferenceRunOpening=false;}
  }
  function openInferenceRun(prepareOnly=false){
    if(inferenceBackendReason()){notice(inferenceBackendReason());return;}
    const gate=permission('inference_run');if(!gate.allowed){notice(gate.reason);return;}
    let seed,duration;try{seed=inferenceSeed();duration=inferenceDuration();}catch(error){notice(error.message);return;}
    const value=inferenceInfo();
    if(!value.run_context||!inferenceModelId()||!inferenceDigest()){notice('等待当前模型和控制上下文核验');return;}
    const steps=state?.inference_execution?.execute_steps??value.execute_steps??15,maxSegment=Number((steps/30*4).toFixed(3));
    inferenceRunIntent={binding:inferenceRunBinding(),model_id:inferenceModelId(),expected_bundle_digest:inferenceDigest(),
      expected_context:JSON.parse(JSON.stringify(value.run_context)),seed,duration_s:duration};
    if(inferenceFamilySelection()==='wam')inferenceRunIntent.wam_stall_recovery=el('inference-stall-enabled').checked;
    if(prepareOnly)inferenceRunIntent.prepare_only=true;
    setText('inference-run-title',prepareOnly?'准备控制与踏板':'开始本轮推理');
    setText('inference-run-summary',`${value.model?.label||inferenceModelId()} · 随机种子 ${seed} · ${duration} 秒。`+
      (value.finite_chunk_count?`连续${value.finite_chunk_count}轮，每轮完整执行15步模型轨迹，源时长0.5秒；可减速至最多${value.reference_planning?.max_execution_duration_s??2}秒。每轮重新观测和推理，等待另计；完成后Stop，异常即停。`:
       value.normal_modes_enabled&&value.reference_planning?`本轮最长运行 ${duration} 秒，包含观测、推理、准备、段间等待与运动；每段完整${steps}点，下一整段放不下时保持并正常结束。可随时Stop。`:duration===.5?'取得实时预测后执行0.5秒并自动Stop；执行节奏可能受速度/加速度约束而放慢。':`每次执行 ${steps} / ${state?.inference_execution?.action_horizon??60} 步。`)+
      '设备控制须已接入；本轮仅申请推理执行准入，不连接或恢复设备。');
    if(prepareOnly)setText('inference-run-summary',`准备当前位置保持与录制服务；只录制推理和拖动阶段，等待时间不计入有效时长。模型 ${value.model?.label||inferenceModelId()}，种子 ${seed}，每轮最长 ${duration} 秒。准备完成后从保持状态双踩踏板1或点击 Run 开始推理。保持中应用参数会自动结束旧Run取流并重新准备。`);
    if(inferenceRunIntent.wam_stall_recovery)setText('inference-run-summary',el('inference-run-summary').textContent+' 已启用WAM静止恢复，触发时顶部输入黑屏并逐块随机采样，实际输入和种子随本轮录制。');
    setText('inference-run-details',prepareOnly?'确认后建立当前位置保持并预热录制；MODEL／TEACH 片段会合成连续回放。等待后续 Run 或双踩踏板1启动模型。':
      '确认后核验当前控制与业务依赖，按所选时长开始推理。停止按钮始终可用。');
    setText('inference-run-clearance-label',prepareOnly?'工作区可安全保持。我确认准备控制与踏板，后续双踩踏板1可启动模型。':
      '场景已复位、工作区无人。我允许本轮模型执行。');
    setText('inference-run-confirm',prepareOnly?'确认并准备':'确认并开始');
    el('inference-run-clearance').checked=false;el('inference-run-confirm').disabled=true;
    el('inference-run-dialog').showModal();
  }
  function renderInference(){
    const value=state?.workflow?.inference||{},model=value.model||{},metrics=value.metrics||{},outcome=value.outcome||{},audit=value.audit||{};
    const background=value.background_recordings||[];
    const activity=value.operator_session?state?.runtime?.recording?.recording_activity:null;
    const activityText=activity?`${activity.active_mode?'正在录制 '+activity.active_mode:'等待中，不录制'} · 有效录制 ${Number(activity.effective_duration_s).toFixed(1)} 秒`:'';
    setText('inference-recording-status',[activityText,...background.map(row=>`Run ${row.run_id}：${row.ingress_closed?'取流已关闭，后台校验与导出中':'正在确认尾部落盘'}${row.error?' · '+row.error:''}`)].filter(Boolean).join('；'));
    const steps=state?.inference_execution?.execute_steps??value.execute_steps??15,maxSegment=Number((steps/30*4).toFixed(3));
    const operator=value.operator_task;
    const modelActive=value.normal_mode?.mode==='MODEL'||['RESUMING','MODEL_ADMITTED'].includes(value.normal_mode?.transition_stage)||
      state?.workflow?.request?.action==='inference_run'&&state.workflow.request.status==='running';
    const doubleWindow=value.operator_controls?.double_press_window_s;
    const doubleHint=Number.isFinite(doubleWindow)?`${doubleWindow}秒内双踩踏板1启动Infer`:'双踩踏板1启动Infer';
    setText('inference-teach',modelActive?'停止推理并位置保持 · 踏板2':operator?.kind==='teach'&&operator.status==='running'?'完成拖动并位置保持 · 踏板2':'重力补偿拖动 · 踏板2');
    setText('inference-operator-status',modelActive?'推理中单踩踏板1或2停止并保持，踏板3无动作。双踩1的第一脚停止，第二脚忽略。':operator?.request_id&&['running','failed'].includes(operator.status)?
      `${operator.kind==='teach'?'人工起点':operator.kind==='hold'?'停止保持':'回零'}：${({waiting_inference_hold:'等待保持与录制准备',preparing_home:'准备轨迹及录制',homing:'执行中',native_homing:'SDK原生回零中，等待完成',holding:'已完成并保持',entering_teach:'进入重力补偿',teaching:'可人工拖动并手动开合夹爪，再踩踏板2保持当前位置与开口',locking_start:'制动并锚定当前位置',sealing_start:'保持已确认，正在封存定位过程',start_ready:'当前位置已保持，可从这里开始下一轮Infer',terminated:'已中断'})[operator.phase]||operator.status} · ${operator.error||operator.run_id||'准备保持与录制'} · SDK保留`:
      value.operator_session?`控制与录制已准备。保持后${doubleHint}，单踩1保持，踏板2切换拖动与保持，踏板3原生回零。`:
      `先在网页准备控制与录制，完成保持后${doubleHint}。踏板响应当前前台推理页面及控制归属。`);
    const mixed=['mixed366-nohistory','coffee999-xnorm'].some(prefix=>String(value.selected_model_id||model.id||'').startsWith(prefix));
    setText('inference-task',String(value.selected_model_id||model.id||'').startsWith('wam-')?'任务上下文：随 checkpoint 绑定的训练文本缓存':value.task?'当前控机任务：'+({coffee:'咖啡制作',lid_handoff:'盖盖与交接'}[value.task]||value.task)+'（训练原文 prompt）':'');
    setText('inference-duration-help',value.finite_chunk_count?`每轮0.5秒指模型源轨迹的15步，完整执行可减速至${value.reference_planning?.max_execution_duration_s??2}秒；共${value.finite_chunk_count}轮，推理等待另计。`:
      value.normal_modes_enabled&&value.reference_planning?`本轮最长运行时长是MODEL控制阶段的总秒数，包含观测、推理、准备、等待与运动。每段完整${steps}点，可减速至最多${maxSegment}秒；剩余预算不足一整段时保持并结束。`:mixed?'0.5秒模式先保持并取得实时预测，再执行0.5秒，随后自动停止。当前实机验收为咖啡任务空场短时输出；整段任务效果未验收。':'设备源时间；包含首次预测等待和保持，实际下发数量单独记录');
    const backendKey=JSON.stringify([key(),!!state]);
    if(backendKey!==inferenceBackendKey){
      inferenceBackendKey=backendKey;const triton=new URL(location.href).searchParams.get('infer_mode')==='triton';
      el('inference-family').value=triton?'pi':inferenceFamily({id:value.selected_model_id||model.id});
      setInferenceBackends(triton?'triton':value.backend||model.backend||'torch');
    }
    el('triton-conversion-tools').hidden=inferenceFamilySelection()!=='pi';
    setText('inference-backend-status',inferenceBackendReason()||(String(value.selected_model_id||model.id).startsWith('wam-')?
      'WAM 使用原生关节与标定夹爪坐标，外参不参与转换；命令开度保留各侧 2 mm 裕量。执行前核验本次设备绑定。':
      'Torch 与 Triton 共用当前任务、设备映射、执行审阅及 Stop；切换模式后重新选择 checkpoint。'));
    const seedKey=JSON.stringify([key(),serverInstance,state?.device?.id,state?.runtime?.runtime_nonce,value.selected_model_id||model.id,value.operator_session?.run_id]);
    if(seedKey!==inferenceSeedKey){
      inferenceSeedKey=seedKey;el('inference-seed').value=Number.isInteger(value.seed)?value.seed:0;
      const duration=value.operator_session?.duration_s;
      el('inference-duration').value=typeof duration==='number'&&Number.isFinite(duration)&&duration>0?String(duration):value.normal_modes_enabled&&value.reference_planning?'120':mixed?'0.5':'120';
    }
    if(value.finite_chunk_count)el('inference-duration').value='0.5';
    const active=environmentAccess?.state?.active_job,modelOperation=active?.request?.action==='prepare_model'?active:null;
    const modelError=model.status==='error'&&!modelOperation,modelUnknown=!options().fresh||model.status==='unknown'&&!modelOperation;
    const modelStatus=modelUnknown?'当前状态未知':modelError?'加载失败':model.status==='loading'||modelOperation?'加载中':model.status==='selection_changed'?'需重新选择':model.loaded===true?'已加载':'尚未加载';
    setText('inference-model','推理模型：'+(model.label||model.id||'尚未选择'));
    setText('inference-model-status',modelStatus);el('inference-model-status').className='badge'+(modelError?' tone-bad':!modelUnknown&&model.loaded===true&&model.status!=='loading'?' tone-good':'');
    setText('inference-model-message',model.error||(value.normal_mode?.runner_status==='READY'?'本轮已结束并保持；下一轮需重新确认。':model.message)||'加载与预热不执行动作。每轮开始前须核对当前设备、模型和执行摘要。');
    setText('inference-prepare',model.loaded?'1. 核验已加载模型':'1. 加载与预热模型');
    const selectionKey=JSON.stringify([key(),serverInstance,inferenceFamilySelection(),inferenceBackend()]);
    if(inferenceSelection?.key!==selectionKey)inferenceSelection=null;
    const preparing=model.status==='loading'||!!modelOperation||!!inferenceSelection&&(inferenceSelection.submitting||environmentAccess?.permission('prepare_model').allowed!==true);
    if(!preparing)inferenceSelection=null;
    const managedModel=environmentAccess?.state?.model;
    const prepared=managedModel?(managedModel.ready===true||managedModel.model_ready===true):(value.model_ready===true||model.loaded===true);
    renderInferenceDenoise();
    const stall=el('inference-stall-enabled'),stallInfo=value.wam_stall_recovery;
    el('inference-stall-tools').hidden=inferenceFamilySelection()!=='wam';
    if(value.operator_session)stall.checked=value.operator_session.wam_stall_recovery===true;
    stall.disabled=!!state?.workflow?.busy||!!value.operator_session||!options().fresh;
    const decision=stallInfo?.last_decision;
    setText('inference-stall-status',decision?`已观察 ${decision.observed_steps} / ${decision.config.window_steps} 个模型步。`+
      (decision.stalled?`静止恢复中 · ${decision.head_black?'顶部黑屏':'真实画面'} · 实际随机种子已记录`:'常规画面与配置种子。'):
      '连续3秒模型步静止后，交替使用2块顶部黑屏和1块真实画面，并为每块重新采样种子。检测到运动后恢复常规推理。夹爪静止阈值为各侧标定行程的5%。');
    const models=inferenceModes(value.models,inferenceBackend(),inferenceFamilySelection(),inferenceDenoiseValue()),placeholder=(!prepared||!!inferenceBackendReason())&&!preparing;
    const selectedId=inferenceSelection?.id||modelOperation?.request?.parameters?.model_id||managedModel?.selected_model_id||value.selected_model_id||model.id||'';
    const selectedDigest=inferenceSelection?.bundle_digest||modelOperation?.request?.parameters?.expected_bundle_digest||managedModel?.bundle_digest||value.bundle_digest||model.bundle_digest;
    const selected=inferenceChoice(models.find(row=>(row.id||row.model_id)===selectedId&&(!row.runtime_schema||row.bundle_digest===selectedDigest)));
    const catalogKey=JSON.stringify([selectionKey,models,selected,placeholder]);
    if(catalogKey!==inferenceCatalogKey){
      inferenceCatalogKey=catalogKey;const select=el('inference-checkpoint');select.replaceChildren();
      if(!models.length){const option=make('option','当前家族 / 后端尚无可用 checkpoint');option.value='';select.append(option);}
      else if(placeholder){const option=make('option','选择checkpoint以准备');option.value='';option.disabled=true;select.append(option);}
      for(const [index,row]of models.entries()){
        const option=make('option',(row.label||row.id||row.model_id||'未完成 checkpoint')+(row.selectable?'':' · '+(row.reason||statusText(row.status))));
        option.value=inferenceChoice(row)||'unavailable-'+index;option.disabled=row.selectable!==true;select.append(option);
      }
      select.value=placeholder?'':selected;
    }
    const unavailable=[...new Set((value.models||[]).filter(row=>!row.selectable)
      .map(row=>`${row.label||row.id||'checkpoint'} / ${row.backend||'torch'} · ${row.reason||statusText(row.status)}`))];
    setText('inference-catalog-status',value.catalog_error||
      (value.catalog_root?'checkpoint 目录：'+value.catalog_root+' · ':'')+`${models.filter(row=>row.selectable).length} 个可选择 · 目录自动刷新`+
      (prepared&&!selected&&!placeholder?' · 当前已加载配置保留，目录中已无同一版本':'')+
      (unavailable.length?' · '+unavailable.join(' | '):''));
    const refreshKey=JSON.stringify([key(),serverInstance]);
    if(context.business==='inference'&&inferenceRefreshKey!==refreshKey&&permission('inference_refresh').allowed){
      inferenceRefreshKey=refreshKey;void send('inference_refresh');
    }
    const finite=value.finite_chunks;
    const rows=[{label:'当前控制权',value:BUSINESS[state?.runtime?.holder]||'未持权'},
      value.finite_chunk_count?{label:'完整模型片段',value:`${finite?.completed_count??0} / ${value.finite_chunk_count} 轮；每轮15步`}:
      {label:'设备源时间',value:seconds(metrics.elapsed_source_s)+` / ${value.duration_s??120} 秒；剩余 `+seconds(metrics.remaining_source_s)},
      {label:'本轮墙钟耗时',value:seconds(metrics.elapsed_wall_s)},
      {label:'模型调用 / 已返回结果',value:textValue(metrics.queries)+' / '+textValue(metrics.returned_predictions??metrics.predictions)},
      {label:'已接受 / 已丢弃预测',value:textValue(metrics.accepted_predictions??metrics.predictions)+' / '+textValue(metrics.discarded_predictions??0)},
      {label:'最近丢弃原因',value:metrics.last_discard_reason==='stale_observation'?'观测已过期，未执行该预测':metrics.last_discard_reason==='revoked_generation'?'执行代次已撤销，未执行该预测':metrics.last_discard_reason||'无'},
      {label:'最近观测年龄 / 模型请求总耗时',value:seconds(metrics.last_observation_age_source_s)+' / '+seconds(metrics.last_model_wall_s??metrics.last_infer_wall_s)},
      value.normal_modes_enabled&&value.reference_planning?
        {label:'当前执行通路',value:'共享 Owner 原生窗口；实际命令与反馈见本轮录制'}:
        {label:value.prepared_reference_execution?'原生参考窗口 · 已提交 / 后续反馈确认':'已提交 / 已实际应用命令',value:textValue(metrics.submitted??metrics.submitted_commands)+' / '+textValue(metrics.completed??metrics.completed_commands)},
      {label:'最近一次推理耗时',value:Number.isFinite(metrics.last_infer_wall_s)?metrics.last_infer_wall_s.toFixed(3)+' 秒':'尚无记录'},
      {label:'本轮随机种子',value:value.seed??'尚未准备'}];
    if(value.finite_chunk_count)rows.splice(2,0,
      {label:'已完成片段耗时 / 累计轨迹执行',value:seconds(finite?.completed_execution_s)+' / '+seconds(finite?.active_execution_s)},
      {label:'当前段推理等待',value:seconds(finite?.waiting_elapsed_s??finite?.current?.waiting_s)},
      {label:'累计推理等待',value:seconds(finite?.total_waiting_s)},
      {label:'各段完整执行时长',value:finite?.completed?.length?finite.completed.map(row=>seconds(row.execution_duration_s)).join(' / '):'尚无完整片段'});
    if(mixed&&value.input_evidence)rows.push({label:'原始推理证据',value:value.input_evidence.error||`待写 ${value.input_evidence.pending} 项，已写 ${value.input_evidence.completed} 项`});
    if(value.observation_pairing)rows.push({label:'当前缓存三相机时间跨度',value:(value.observation_pairing.camera_span_ns/1e6).toFixed(1)+' ms'});
    renderDetails('inference-metrics',rows);
    const outcomeLabel=outcome.success===true?'成功：末段稳定托盘支撑已确认':outcome.success===false?'未成功：末段稳定托盘支撑未满足':'未知：尚无本轮末段稳定托盘支撑结论';
    setText('inference-outcome',outcomeLabel+(outcome.reason?' · '+operatorText(outcome.reason):''));
    el('inference-outcome').className=outcome.success===true?'tone-good':outcome.success===false?'tone-bad':'muted';
    const problems=Array.isArray(audit.problems)?audit.problems:null,counts=audit.counts||{},count=kind=>counts[kind]??(problems?problems.filter(row=>row.kind===kind).length:'尚无记录');
    setText('inference-audit-summary',`仅记录 · 位置超限 ${count('position')} · 速度超限 ${count('velocity')} · 加速度超限 ${count('acceleration')} · 位置投影 ${audit.projection_count??(Array.isArray(audit.projections)?audit.projections.length:'尚无记录')}；不执行碰撞检查，不将审计结果当作任务成功。`);
    setText('inference-audit',Object.keys(audit).length?audit:'本轮尚无目标审计记录。');
  }
  function renderSceneAssets(){
    const catalog=state?.scene_assets,identity=JSON.stringify([context.environment,catalog?.catalog_digest]);
    if(sceneAssetCatalogKey!==identity){
      sceneAssetCatalogKey=identity;
      const pair=catalog?.pairs?.find(row=>row.id===(catalog.requested_pair_id||catalog.applied_pair_id));
      for(const kind of ['machine','cup']){
        const select=el('scene-asset-'+kind);select.replaceChildren();
        const rows=(catalog?.assets||[]).filter(row=>row.kind===kind);
        for(const row of rows){const option=make('option',row.label+' · '+row.revision);option.value=row.id;select.append(option);}
        sceneAssetDraft[kind]=pair?.[kind]||rows[0]?.id||'';select.value=sceneAssetDraft[kind];
      }
    }
    for(const kind of ['machine','cup']){
      const select=el('scene-asset-'+kind),row=catalog?.assets?.find(asset=>asset.id===sceneAssetDraft[kind]);
      select.disabled=!catalog||!options().fresh||!!catalog.busy;
      const image=el('scene-asset-'+kind+'-preview');
      const url=row&&safeMediaUrl(row.preview_url,location.href),routed=url&&routePhysicalRequest(url,undefined,state,location.href);
      if(image.dataset.assetUrl!==(routed||'')){
        image.dataset.assetUrl=routed||'';image.hidden=true;
        if(routed){image.alt=row.label+' · 数字资产预览';image.src=routed;}else image.removeAttribute('src');
      }
      setText('scene-asset-'+kind+'-details',row?`${row.asset_id} · ${row.dimensions_m.map(v=>(v*1000).toFixed(1)).join(' × ')} mm`:'尚未登记模型');
    }
    const selected=sceneAssetPair(catalog,sceneAssetDraft.machine,sceneAssetDraft.cup);
    const gate=environmentAccess?.permission('prepare_collection');
    const reason=!catalog?'此部署尚未登记场景资产目录':!options().fresh?'当前状态待核验':
      catalog.busy?'资产与服务正在准备':state?.workflow?.busy?'当前任务尚未结束':
      catalog.can_retry===false?catalog.retry_reason||'当前数采环境不能重新准备':
      !selected?'此咖啡机与杯子组合尚未登记':!selected.can_prepare?selected.reason||'此组合尚未准备好':
      !gate?.allowed?gate?.reason||'统一运维入口尚未接入':null;
    el('scene-asset-apply').disabled=!!reason;
    setText('scene-asset-apply-reason',reason||'应用后准备对应资产与服务；旧位置与未执行计划失效，下轮重新检测。');
    const applied=catalog?.pairs?.find(row=>row.id===catalog.applied_pair_id);
    const names=pair=>['machine','cup'].map(kind=>catalog.assets.find(row=>row.id===pair[kind])?.label||pair[kind]).join(' + ');
    setText('scene-asset-status',!catalog?'场景资产未配置':
      (applied?'已应用配置：'+names(applied):'尚未应用目录中的资产组合')+
      (catalog.requested_pair_id&&catalog.requested_pair_id!==catalog.applied_pair_id?' · 新组合尚未准备成功':'')+
      (catalog.status==='failed'?' · 当前服务准备失败':'')+'。选择框和图片为草稿预览。');
  }
  function renderCollection(){
    const workflow=state?.workflow||{},collection=workflow.collection||{},timing=collection.timing||{},rows=el('collection-stages');rows.replaceChildren();
    const warmup=collection.warmup||{status:typeof collection.environment?.ready==='boolean'?(collection.environment.ready?'ready':'not_started'):'unknown'},fresh=options().fresh;
    const returning=returnTask();
    el('collection-return').hidden=false;
    const returnStage=!!returning&&workflow.stage_id?.startsWith('collection.return_');
    el('collection-execute-action').hidden=returnStage;
    setText('collection-return-status',returning?(fresh?'场景恢复：':'最后确认的场景恢复：')+statusText(returning.status)+(returning.message?' · '+operatorText(returning.message):''):'点击后检查恢复动作，并弹出一次执行确认；不会直接执行。');
    setText('collection-return-timing',returning?[Number.isFinite(returning.duration_s)?'计划源时间 '+seconds(returning.duration_s):'','以固定原始杯位为参考；本次动作与时长以确认摘要为准。保留既有关节与夹爪限值，数采速度草稿不参与恢复。'].filter(Boolean).join(' · '):'');
    setText('collection-return-positions',returnPositionSummary());
    setText('collection-return-source',returning?.parent_plan_id?'父计划：'+returning.parent_plan_id:'');
    el('collection-timing').hidden=returnStage;el('collection-stages').hidden=returnStage;
    const failed=context.business==='collection'&&workflow.status==='failed';
    el('collection-failure').hidden=!failed;
    setText('collection-failure-title',failed?(fresh?'本次'+(returnStage?'场景恢复':'数采')+'未通过':'最后确认的'+(returnStage?'场景恢复':'数采')+'失败（当前状态待核验）'):'');
    setText('collection-failure-message',failed?operatorText(workflow.failure?.message||workflow.message||'后端尚未提供失败原因，请查看运行身份与技术详情。'):'');
    setText('collection-failure-recovery',failed?'处理建议：'+operatorText(workflow.failure?.recovery||'查看下方运行身份与技术详情中的原始错误，修正后重新发起对应步骤。'):'');
    const warmupLabel={not_started:'待准备',running:'进行中',ready:'已就绪',invalidated:'已失效',failed:'失败'}[warmup.status]||'状态未知';
    setText('collection-warmup-status',(fresh?'初始化与预热：':'最后确认的初始化与预热：')+warmupLabel);
    el('collection-warmup-status').className=fresh?(warmup.status==='ready'?'tone-good':['failed','invalidated'].includes(warmup.status)?'tone-bad':''):'';
    setText('collection-warmup-message',operatorText(warmup.message||{
      not_started:'点击上方初始化与预热，完成本次服务所需的准备。',running:'正在准备本次服务，完成后即可观测与规划。',
      ready:'本次服务已预热。移动杯子或咖啡机、中断后复位后，可直接重新观测与规划。',
      invalidated:'准备条件已变化，请重新初始化与预热。',failed:'初始化与预热失败，请检查原因后使用同一按钮重试。'
    }[warmup.status]||'等待后端确认准备状态。')+(fresh?'':' 当前状态待核验，操作暂不可用。'));
    setText('collection-warmup-stage',[warmup.stage_label,Number.isFinite(warmup.elapsed_s)?'本次准备已用 '+seconds(warmup.elapsed_s):''].filter(Boolean).join(' · '));
    setText('collection-prepare',warmup.status==='ready'?'1. 核验已就绪环境':warmup.status==='running'?'1. 初始化与预热中…':['failed','invalidated'].includes(warmup.status)?'1. 重新初始化与预热':'1. 初始化与预热');
    for(const [index,step]of(timing.stages||[]).entries()){
      const item=make('li',undefined,'workflow-step'+(step.status==='running'?' is-current':''));
      item.append(make('strong',operatorText(step.label||'阶段 '+(index+1))),make('p',statusText(step.status)),make('small','实际 '+seconds(step.elapsed_s)+' · 参考 '+seconds(step.estimate_s)));rows.append(item);
    }
    if(!rows.children.length)rows.append(make('li','准备 → 观测与规划 → 审阅执行；阶段记录将在操作后显示。','muted'));
    setText('collection-timing',operatorText((timing.label?'当前处理：'+timing.label+'。':'')+`本次已用 ${seconds(timing.elapsed_s)} · 当前阶段 ${seconds(timing.stage_elapsed_s)} · 参考耗时 ${seconds(timing.stage_estimate_s??timing.estimate_s)}`+(timing.estimate_source?'（'+timing.estimate_source+'）':'')+'。下一步：'+(timing.next_label||'根据上方操作状态继续')+'。参考耗时不是完成倒计时。'));
    const environment=collection.environment||{},checks=el('collection-environment');checks.replaceChildren();
    const checkList=make('div',undefined,'environment-checks');
    const rawChecks=environment.checks||[],checkRows=Array.isArray(rawChecks)?rawChecks:Object.entries(rawChecks).map(([key,value])=>value&&typeof value==='object'?{key,...value}:{key,status:value});
    for(const check of checkRows){const badge=make('span',(check.label||check.key)+' · '+statusText(check.status),'environment-check '+(check.status==='ready'?'tone-good':check.status==='failed'?'tone-bad':''));badge.title=check.message||'';checkList.append(badge);}
    if(checkList.children.length)checks.append(checkList);
    else checks.append(make('p',environment.ready?'环境已准备':'环境尚待检查','hint'));
    const recording=collection.recording||{},native=collection.native_task||{},simulation=collection.simulation||{};
    const failure=native.first_failure,errors=collection.errors||{};
    const evidence=Object.keys(recording).length||Object.keys(native).length||Object.keys(simulation).length||Object.keys(errors).length||collection.execution_wait;
    el('collection-evidence').hidden=!evidence;
    if(evidence){
      const facts=[];
      if(Object.keys(recording).length)facts.push({label:'录制与封存',value:recording.error||recording.seal_error||recording.diagnostics_error||recording.manifest?.validation|| (recording.done?'录制结束，等待完整性核验':recording.ready?'记录已建立，等待本轮封存':'录制尚未就绪')});
      if(native.task_outcome)facts.push({label:'原生任务观测',value:(native.task_outcome==='success'?'观测报告成功；流程结果以上方独立结果为准':statusText(native.task_outcome))});
      if(failure)facts.push({label:'首次失败',value:failure.reason||failure.detail||failure},{label:'失败位置',value:`物理步 ${failure.absolute_physics_step??failure.physics_step??'未知'}`});
      if(Object.keys(simulation).length)facts.push({label:'辅助回放',value:statusText(simulation.status||'pending')+'；仅用于对照，不代替任务成功证据'});
      if(collection.execution_wait)facts.push({label:'当前等待',value:collection.execution_wait});
      if(Object.keys(errors).length)facts.push({label:'本轮异常',value:errors});
      renderDetails('collection-evidence-details',facts);
    }
    const relations=collection.relations||{},target=el('collection-relations'),signature=JSON.stringify([key(),relations]);
    if(target.dataset.signature!==signature){
      target.dataset.signature=signature;target.replaceChildren();const detail=make('details'),list=make('dl',undefined,'detail-list');detail.append(make('summary','观测、计划与标定来源'));
      for(const [name,fields]of Object.entries(relations)){
        for(const [field,value]of Object.entries(fields||{})){const entry=make('div');entry.append(make('dt',({plan:'计划',source:'观测',calibration:'标定',errors:'读取异常'}[name]||name)+' · '+({run_id:'运行',timeline_digest:'轨迹身份',calibration_profile_digest:'标定身份',profile_id:'版本',camera_serials:'相机',robot_id:'设备',duration_s:'计划时长 s',speed_scale:'速度比例',path:'文件',output:'产物目录',plan_path:'计划文件',scene_digest:'场景身份',environment:'环境'}[field]||field)),make('dd',value));list.append(entry);}
      }
      if(!list.children.length)list.append(make('p','尚无可核验记录','muted'));detail.append(list);target.append(detail);
    }
  }
  function clearReview() { el('clearance').checked=false; }
  function renderPreview(){
    const preview=state?.preview,mesh=preview?.mesh||state?.mesh||{};
    if(overviewFlight&&overviewFlight.binding!==overviewBinding())cancelOverview();
    if(targetFlights.some(job=>job.binding!==targetBinding())||targetPending&&targetPending.binding!==targetBinding())cancelTarget('显示上下文已改变');
    if(presentedTarget&&presentedTarget.binding!==targetBinding())presentedTarget=null;
    if(preview?.renderer_id!==previewRenderer){previewRenderer=preview?.renderer_id;previewFloors.clear();previewErrors.clear();}
    const meshKey=JSON.stringify([key(),mesh.camera,mesh.candidate_id]);
    const unifiedMesh=Array.isArray(preview?.mesh_calibrations);
    if(meshKey!==meshStateKey&&!displayRequests.has('mesh')){meshStateKey=meshKey;meshCamera=mesh.camera||meshCamera;meshCandidate=mesh.candidate_id||'';meshAnchor=meshCandidate;if(meshCandidate&&!unifiedMesh)meshChoices.set(meshCamera,meshCandidate);}
    el('overview-controls').hidden=!preview;
    el('media-preview').classList.toggle('overview',!!preview);
    syncNativeViewer();
    el('preview-top-view').disabled=preview?.capabilities?.top_view!==true||displayRequests.has('recovery');
    el('preview-top-view').checked=overview.top_view===true;
    el('mesh-controls').hidden=!preview;el('media-mesh').hidden=!preview;
    el('mesh-camera').value=meshCamera;
    const catalog=unifiedMesh?meshGroups(state):meshRows(state);
    const choices=unifiedMesh?catalog:catalog.filter(row=>row.kind===(meshCamera==='head'?'head':'wrist'));
    if(unifiedMesh)meshCandidate=matchingMeshCandidate(catalog,meshCamera,meshCandidate);
    else if(context.environment==='physical'&&!choices.some(row=>row.id===meshCandidate)){
      meshCandidate=catalog.some(row=>row.id===meshAnchor)?matchingMeshCandidate(catalog,meshCamera,meshAnchor,meshChoices.get(meshCamera)):
        choices.find(row=>row.mesh_installed&&row.mesh_selectable!==false)?.id||choices.find(row=>row.mesh_selectable!==false)?.id||'';
      if(meshCandidate)meshChoices.set(meshCamera,meshCandidate);
    }
    el('media-mesh').classList.toggle('mesh-all',meshCamera==='all');
    const optionsKey=JSON.stringify([key(),unifiedMesh?'group':meshCamera,meshCandidate,choices.map(row=>[row.id,row.label,row.status,row.mesh_selectable,row.mesh_reason])]);
    if(optionsKey!==meshOptionsKey){meshOptionsKey=optionsKey;const box=el('mesh-candidate');box.replaceChildren();for(const row of [{id:'',label:unifiedMesh?'选择完整标定组':'选择标定候选'},...choices]){const option=make('option',row.label+(row.status?' · '+statusText(row.status):''));option.value=row.id;option.disabled=row.mesh_selectable===false;option.title=row.mesh_reason||'';if(row.mesh_reason)option.textContent+=' · '+row.mesh_reason;box.append(option);}box.value=choices.some(row=>row.id===meshCandidate)?meshCandidate:'';}
    setText('mesh-candidate-message',choices.length?(choices.filter(row=>row.mesh_selectable!==false).length+' 个可用候选 / '+choices.length+' 个已保存候选。'+choices.filter(row=>row.mesh_reason).map(row=>row.label+'：'+row.mesh_reason).join('；')):unifiedMesh?'尚无完整头部与双腕标定组。':'没有保存的'+(meshCamera==='head'?'头部':'双腕')+'候选');
    setText('mesh-state','请求：'+(mesh.requested===true?'开启':mesh.requested===false?'关闭':'未知')+' · 后端：'+(mesh.applied===true?'已开启':mesh.applied===false?'已关闭':'未知'));
    el('mesh').checked=mesh.requested===true;
    const protocol=state?.device?.id==='protocol-fake';
    setText('preview-status',(protocol?'当前为协议 Fake（L1）：只验证控制协议与页面，不提供 PhysX 相机图像。完整 Fake 需要物理仿真、相机与渲染服务。 ':'')+(!preview||preview.status==='unconfigured'?'目标预览渲染服务尚未配置。':(preview.connected===true?'渲染服务已连接':preview.connected===false?'渲染服务未连接':'渲染连接尚未核验')+(preview.error?' · '+preview.error:'')));
    const rows=[{label:'渲染连接',value:!preview||preview.status==='unconfigured'?'尚未配置':preview.connected===true?'已连接':preview.connected===false?'未连接':'未知'},
      {label:'当前渲染原因',value:preview?.error||preview?.status||'尚无渲染状态回执'}];
    if(preview?.diagnostic_calibration)rows.unshift({label:'复用标定诊断（不是本轮新安装）',value:preview.diagnostic_calibration});
    if(preview?.mesh_catalog_error)rows.push({label:'Mesh 标定目录',value:preview.mesh_catalog_error});
    if(protocol)rows.unshift({label:'设备资格',value:'协议 Fake（L1）不提供 PhysX 相机图像；缺少这类图像属于未配置完整仿真，不能通过重连浏览器补齐。'});
    for(const media of state?.media||[])if(media.error||media.ready===false)rows.push({label:media.label||media.id||'图像通道',value:media.error||'当前画面尚未就绪'});
    for(const [view,frame]of Object.entries(preview?.frames||{}))if(frame?.error)rows.push({label:{target:'目标姿态',overview:'整机视角',mesh:'Mesh 叠加'}[view]||view,value:frame.error});
    if(preview?.recovery?.message)rows.push({label:'恢复指引',value:preview.recovery.message});
    renderDetails('image-recovery-details',rows);
    const commands=preview?.recovery?.commands;el('image-recovery-commands').hidden=!commands;
    setText('image-recovery-commands',Array.isArray(commands)?commands.join('\n'):commands||'');
  }
  function render() {
    renderAudit();
    renderControlPolicy();
    const s=state, r=s?.runtime||{}, w=s?.workflow?.business_task||s?.workflow||{};
    el('execution-settings-tools').hidden=!BUSINESS[tab]&&tab!=='research';el('speed-protection-tools').hidden=!BUSINESS[tab]&&tab!=='research';el('context-guidance').hidden=['operations','research'].includes(tab);el('request-status').hidden=tab==='operations';
    el('operations-link').dataset.business=context.business;
    el('operations-link').href='#operations-panel';
    el('work-pane').dataset.business=context.business;
    el('task-panel').hidden=context.business==='research';el('business-outcomes').hidden=context.business==='research';
    setText('observation-title',context.business==='calibration'?'调姿预览与相机':context.business==='inference'?'推理实际观测与执行画面':context.business==='research'?'控制研究 · 目标姿态与观察相机':'实际观测与规划场景');
    el('primary-action').hidden=['manual_execute','connect','handoff','release','recover'].includes(w.primary_action?.id)||context.business==='inference'&&['inference_prepare','inference_preview'].includes(w.primary_action?.id)||context.business==='collection'&&(!!w.collection?.actions||!!w.collection?.warmup||!!w.collection?.return_task||['prepare_collection','collection_prepare','return_cup'].includes(w.primary_action?.id));
    el('results-business').value=context.business;
    setText('device-name',s?.device?.label||s?.device?.id||'机器人工作台');
    setText('holder',BUSINESS[r.holder] || (r.holder ? String(r.holder) : '未分配 / 未知'));
    setText('connection','连接：'+(r.connected===true?'已连接':r.connected===false?'未连接':'未知'));
    const controlState=String(r.state||'unknown').toUpperCase(),controlFault=['FAULT','FAULT_STOP_UNCONFIRMED','EMERGENCY'].includes(controlState);
    setText('control-state','控制：'+({READY:'就绪',STOPPED_DAMPING_HOLD:'已停止，阻尼保持',MANUAL_TAKEOVER:'人工接管',FAULT:'故障锁定',FAULT_STOP_UNCONFIRMED:'故障锁定（停止未确认）',EMERGENCY:'紧急故障锁定',DISCONNECTED:'未连接',RELEASED:'已释放',CONNECTING:'连接中'}[controlState]||statusText(r.state)));
    el('control-state').className=controlFault?'tone-bad':'';el('control-state').title=controlState;
    el('control-fault').hidden=!controlFault;
    const fault=r.last_fault_diagnostic||{},faultReasons=[r.reason,fault.error,...(Array.isArray(fault.reasons)?fault.reasons:[])].filter(Boolean).map(textValue);
    setText('control-fault-reason',controlFault?controlState+' · '+([...new Set(faultReasons)].join('；')||'后端尚未提供故障原因，请查看原始状态回执。'):'');
    setText('control-fault-guidance',controlFault?(context.environment==='physical'?'请先可靠支撑双臂并核对服务运维中的设备诊断；':'请先核对设备诊断；')+'请通过中断恢复查看归因和现场条件；释放未确认时不要重试。不会自动续跑旧轨迹。':'');
    setText('feedback','反馈：'+(r.feedback_healthy===true?'健康':r.feedback_healthy===false?'不可用':'未知'));
    const hold=typeof r.hold==='object'&&r.hold?Object.entries(r.hold).map(([side,value])=>({left:'左',right:'右'}[side]||side)+' '+statusText(value)).join(' / '):statusText(r.hold);
    setText('hold','保持：'+hold);setText('motion','动作：'+(r.motion_status==='no_automatic_task'?'无自动任务':statusText(r.motion_status)));
    el('motion').title=r.motion_status==='no_automatic_task'?'当前没有自动动作任务；此状态不代表物理静止，仍需核对设备反馈。':'';
    setText('stop-state',r.stop_receipt?(r.stop_receipt.stop_hold_return_confirmed===true?'停止：保持调用已确认':'停止：已受理，尚未确认'):'');
    setText('release-state',r.release_receipt?({requested:'释放：已受理，等待设备断开',released:'释放：设备连接已断开',unconfirmed:'释放：尚未确认'}[r.release_receipt.status]||'释放：状态待核验')+(r.release_receipt.message?' · '+r.release_receipt.message:''):'');
    const manualNotice=manualPreviewNotice(),idleManualNotice=false;
    const manualExecution=s?.manual?.execution||(/^(manual\.)/.test(s?.workflow?.stage_id||'')?s.workflow:null);
    setText('manual-execution-status',manualNotice?(manualNotice.status==='running'?'执行前检查':'检查失败'):manualExecution?statusText(manualExecution.status):'尚无人工轨迹');
    setText('manual-execution-message',manualNotice?.message||manualExecution?.message||'人工轨迹的预检、审阅与执行回执显示在这里；不会覆盖业务任务状态。');
    renderDetails('manual-execution-details',manualExecution?.details||[]);
    setText('task-title','当前任务 · '+BUSINESS[context.business]);setText('task-status',idleManualNotice?(manualNotice.status==='running'?'执行前检查':'检查失败'):statusText(w.status));
    setText('run-identity','Run：'+textValue(w.id));setText('stage-title',idleManualNotice?manualNotice.title:STAGE_LABELS[w.stage_id]||operatorText(w.title&&/[\u3400-\u9fff]/.test(w.title)?w.title:'等待任务状态'));setText('stage-id',w.stage_id||'—');
    const actionName={generate_plan:'生成计划',collection_prepare:'准备数采环境',collection_plan:'观测与规划',return_cup:'检查场景恢复动作',execute:context.business==='inference'?'执行本轮120秒推理':'执行已审阅轨迹',inference_prepare:'加载与预热模型',inference_preview:'准备本轮推理审阅',manual_preview:'检查人工轨迹',fit_batch:'求解当前批次',plan_batch:'规划当前批次',preview_batch:'审阅当前批次'};
    const workflowMessage=context.business==='collection'&&w.status==='failed'?w.failure?.message||w.message:w.message;
    const resolvedModelRequest=supersedingModelPreparation(s,environmentAccess?.state);
    const taskMessage=w.request?.status==='failed'&&!resolvedModelRequest?[workflowMessage,'本次请求失败：'+textValue(w.request.error)].filter(Boolean).join(' '):w.request?.status==='running'?'正在处理：'+(actionName[w.request.action]||'当前操作')+'。'+(workflowMessage||''):workflowMessage||'尚未取得当前任务回执。';
    setText('task-message',idleManualNotice?manualNotice.message:operatorText(taskMessage));
    const p=countedProgress(w.progress);el('task-progress').hidden=!p&&w.progress?.indeterminate!==true;
    if(p){el('task-progress').max=p.total;el('task-progress').value=p.current;}
    else el('task-progress').removeAttribute('value');
    setText('progress-label',p?(p.scope==='motion'?`${p.current} / ${p.total} 条运动命令已提交 · ${(100*p.current/p.total).toFixed(1)}% · 仅运动指令进度；数据、任务与回放分别核验`:p.kind==='fraction'?`${(100*p.current/p.total).toFixed(1)}% · ${p.unit||'当前阶段'} · 后端工作量`:`${p.current} / ${p.total} ${p.unit||''} · 后端实际计数`):'尚无可计数进度；不估算完成比例');
    const overall=countedProgress(w.overall_progress);el('overall-panel').hidden=!overall;
    if(overall){el('overall-progress').max=overall.total;if(overall.indeterminate)el('overall-progress').removeAttribute('value');else el('overall-progress').value=overall.current;setText('overall-label',`${overall.indeterminate?'当前阶段工作量未知':'整体 '+(100*overall.current/overall.total).toFixed(1)+'%'} · ${overall.completed_stages??overall.current} / ${overall.total} 阶段已完成 · 阶段等权，不代表剩余时间`);}
    renderDetails('task-details',[...(w.details||[]),...(w.title?[{label:'原始阶段说明',value:w.title}]:[]),...(w.message?[{label:'原始消息',value:w.message}]:[]),...(w.request?.error?[{label:'原始请求错误',value:w.request.error}]:[])]);
    const reviewedSpeed=w.review?.speed_scale??w.speed_scale??.3,nextSpeedKey=JSON.stringify([key(),w.id,reviewedSpeed]);
    if(!activeReturnTask()){
      if(nextSpeedKey!==speedKey&&!wasReturnTask){el('collection-speed').value=100*reviewedSpeed;setText('collection-speed-value',el('collection-speed').value+'%');}
      speedKey=nextSpeedKey;
    }
    wasReturnTask=!!activeReturnTask();
    renderTrajectory();
    const nextReviewKey=JSON.stringify([key(),w.id,w.revision,w.review?.id]);
    if(nextReviewKey!==reviewKey){reviewKey=nextReviewKey;clearReview();}
    el('review-panel').hidden=!w.review||w.primary_action?.id==='manual_execute'||!!activeReturnTask();
    setText('review-summary',w.review?.summary||'尚无执行摘要');setText('review-id','审阅身份：'+textValue(w.review?.id));
    el('clearance-label').hidden=!w.review || !(context.environment==='physical'||w.review.physical_clearance_required);
    setText('clearance-copy',context.business==='inference'?`我已核对本轮实际执行 ${w.review?.execution_duration_s??w.review?.duration_s??120} 秒的范围与现场，确认执行；本轮不检查碰撞`:'我已审阅本条轨迹和现场净空，确认可以执行');
    const outcomes=w.outcomes||{};
    for(const name of ['motion','data','simulation','task']) {
      const outcome=outcomes[name],value=outcomeStatus(outcome),target=el('outcome-'+name);
      target.textContent=statusText(value)+(outcome?.message?' · '+operatorText(outcome.message):'');
      target.className=['failed','invalid'].includes(value)?'tone-bad':['passed','completed','complete','valid'].includes(value)?'tone-good':executing(value)?'tone-active':'';
    }
    const dataFailed=['failed','invalid'].includes(outcomeStatus(outcomes.data));
    const alert=[dataFailed&&executing(r.motion_status)?'录制失败，动作仍在执行；本轮数据无效。可停止任务或继续观察，不会自动重跑。':'',
      r.stop_receipt&&r.stop_receipt.stop_hold_return_confirmed!==true?'停止已受理，但保持调用尚未确认。请核对设备反馈与停止详情。':'',
      r.service_error?'设备服务异常：'+r.service_error:'',
      !online&&s?'状态连接中断；旧画面与状态仅作参考，等待重新核验。':s&&options().fresh===false?'状态已超过 3 秒未核验；操作已停用，停止请求仍独立可用。':''].filter(Boolean).join(' ');
    el('alert').hidden=!alert;setText('alert',alert);
    renderManual();renderQueue();renderArrival();renderCalibration();renderCollection();renderInference();renderInferenceSteps();renderResults();renderPreview();renderDetails('business-diagnostics-list',s?.diagnostics);renderDetails('diagnostics-list',[{label:'控制器状态',value:r.state||'未知'},{label:'当前控制原因',value:r.reason||'未报告'},{label:'控制服务通信',value:r.service_error||'未报告服务通信错误'}]);
    el('observation-camera').value=observationCamera;
    el('camera-grid').dataset.camera=observationCamera;
    setText('manual-feedback',(r.connected?(options().fresh?'当前实测':'最后实测（状态待核验）'):'控制未接入 · 模型预览；使用目标草稿，实测未知')+' · 左夹爪 '+(Number.isFinite(s?.manual?.jaw_gaps_mm?.[0])?s.manual.jaw_gaps_mm[0].toFixed(1)+' mm':'未知')+' · 右夹爪 '+(Number.isFinite(s?.manual?.jaw_gaps_mm?.[1])?s.manual.jaw_gaps_mm[1].toFixed(1)+' mm':'未知')+(jawRange(0)[1]===83&&jawRange(1)[1]===83?'；反馈范围 -2–89 mm，命令目标仍为 0–83 mm；输入框始终是目标草稿。':`；命令范围：左0–${jawRange(0)[1]}mm / 右0–${jawRange(1)[1]}mm；输入框始终是目标草稿。`)+(s?.manual?.disabled_grippers?.length?'已禁用夹爪：'+s.manual.disabled_grippers.map(side=>side==='left'?'左':'右').join('、')+'，不发送夹爪控制。':''));
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
    if(slot.gpu){nativeLabel();return;}
    const m=slot.media, age=frameAgeSeconds(m,state,clock.now()-receivedAt),staticTarget=m?.preview_view==='target',preview=state?.preview;
    const currentTarget=preview?.connected===true&&options().fresh&&!!m?.renderer_id&&m.renderer_id===preview.renderer_id
      &&Number.isInteger(m.applied_revision)&&m.applied_revision>=m.minimum_revision&&m.applied_revision<=m.revision&&m.view_revision===preview.revisions?.overview;
    const stale=!online||!!slot.error||(staticTarget?!currentTarget:age===null||age>3);
    const ageLabel=staticTarget?(age===null?'生成时间未知':(m.applied_revision<m.revision?'最新完成目标，较新目标待呈现':'静态目标')+' · 生成于 '+age.toFixed(1)+' s 前'):
      age===null?'源时间未知':(age>3?'画面冻结 · ':'源帧龄 ')+age.toFixed(1)+' s';
    slot.card.classList.toggle('stale',stale);
    slot.card.classList.toggle('updating',!!slot.retained);
    slot.card.querySelector('.media-meta').textContent=[ENVIRONMENT[context.environment],{actual:'实际画面',target:'目标预览',overview:'目标预览',mesh:'实际图像上的 mesh',replay:'录制回放'}[m?.kind]||'来源未知',m?.preview_view==='mesh'&&m.candidate_id?'候选 '+m.candidate_id.slice(0,12):'',slot.retained?'更新中 · 保留上一帧（非当前目标／视角）':ageLabel,slot.error||m?.error].filter(Boolean).join(' · ');
  }
  function cancelMediaRequest(slot) {
    slot.controller?.abort();slot.controller=null;slot.version++;
    clearTimeout(slot.timer);clearTimeout(slot.timeout);slot.timer=null;slot.timeout=null;
  }
  function clearSlot(slot,keepImage=false) {
    cancelMediaRequest(slot);if(!keepImage&&slot.blob)urls.revokeObjectURL(slot.blob);if(!keepImage)slot.blob=null;
    slot.retained=keepImage&&!!slot.blob;
    slot.streamKey=null;slot.sourceKey=null;slot.media=null;slot.etag=null;slot.headerFrame=null;slot.headerSource=null;
    slot.appliedRevision=null;slot.appliedIdentity=null;slot.presenting=null;
    if(!slot.retained){slot.image.hidden=true;slot.image.removeAttribute('src');slot.empty.hidden=false;slot.card.classList.toggle('split',false);}slot.error='';
  }
  function clearMedia() {for(const slot of mediaSlots.values())clearSlot(slot);if(nativeViewer)nativeViewer.canvas.hidden=true;}
  function previewMedia(view){
    const combined=state?.preview,preview=combined?.sources?.[view]||combined,frame=preview?.frames?.[view]||{},mesh=preview?.mesh||{},flight=view==='overview'?overviewFlight:null,knownOverview=view==='overview'&&Number.isInteger(previewFloors.get('overview')),expected=['target','overview'].includes(view)?Math.max(preview?.revisions?.[view]??0,previewFloors.get(view)??0):preview?.revisions?.[view];
    let error=previewErrors.get(view)||frame.error||preview?.error||'';
    if(view!=='target'&&displayRequests.has(view)||displayRequests.has('recovery')||(view!=='target'&&previewFloors.has(view)&&expected<previewFloors.get(view)))error='等待本次只读预览的匹配画面';
    if(view!=='mesh'&&(displayRequests.has('overview')||(previewFloors.has('overview')&&preview?.revisions?.overview<previewFloors.get('overview'))))error=previewErrors.get('overview')||'等待当前观察视角的匹配画面';
    if(preview?.connected===false)error=preview.error||'渲染服务未连接，请查看设备诊断中的图像恢复指引';
    if(preview?.status==='unconfigured')error='当前环境尚未配置目标渲染服务；重新连接画面不能代替环境配置。';
    if(view==='mesh'&&!mesh.requested)error='实时 mesh 叠加已关闭；原始相机画面保持独立。';
    if(view==='mesh'&&mesh.requested&&(mesh.camera!==meshCamera||mesh.candidate_id!==meshCandidate))error='等待当前相机与所选候选的匹配画面';
    if(view!=='target'&&frame.revision!==undefined&&frame.revision!==null&&frame.revision!==expected)error='渲染结果仍属于旧修订，等待当前请求';
    if(view!=='mesh'&&frame.view_revision!==undefined&&frame.view_revision!==preview?.revisions?.overview)error='观察视角已更新，等待匹配的新画面';
    if(knownOverview&&preview?.connected&&!displayRequests.has('recovery')&&!displayRequests.has('overview'))error=previewErrors.get(view)||'';
    return {...frame,id:'preview-'+view,kind:view,preview_view:view,revision:expected,view_revision:view==='overview'?expected:preview?.revisions?.overview,renderer_id:preview?.renderer_id,
      ...(preview?.display_context?{display_context:preview.display_context,server_instance_id:preview.server_instance_id||serverInstance}:{}),
      ...(view==='target'?{minimum_revision:preview?.target_floor??0}:{}),
      ...(view==='mesh'?{camera:mesh.camera,candidate_id:mesh.candidate_id}:{}),
      ready:(knownOverview||frame.ready===true||view==='target'&&targetFlights.some(job=>Number.isInteger(job.revision)))&&Number.isInteger(expected)&&!error,error:error||(!knownOverview&&!frame.ready&&!targetFlights.length?'尚无此视图的画面，请先提交只读预览请求':''),poll_hz:frame.poll_hz||25};
  }
  function renderMedia() {
    const actual=(state?.media||[]).filter(m=>m.kind==='actual'), used=new Set();
    const slots=[['head',['head','base_0']],['left',['left','left_wrist_0']],['right',['right','right_wrist_0']]];
    const named=new Set(slots.flatMap(([,aliases])=>aliases));
    for(const [id,aliases]of slots){const media=actual.find(m=>aliases.includes(m.id))||actual.find(m=>!named.has(m.id)&&!used.has(m.id));if(media)used.add(media.id);loadMedia(id,media);}
    if(nativeViewer&&nativeKey===nativeBinding()&&!doc.hidden){
      let slot=mediaSlots.get('preview');if(slot&&!slot.gpu){clearSlot(slot);slot.gpu=true;}
      nativeViewer.canvas.hidden=false;el('media-preview').querySelector('.empty').hidden=true;
      el('media-preview').classList.toggle('split',overview.top_view===true);el('media-preview').classList.toggle('stale',false);el('media-preview').classList.toggle('updating',false);nativeLabel();
    }else loadMedia('preview',state?.preview?previewMedia(previewView):(state?.media||[]).find(m=>m.kind==='target'));
    loadMedia('mesh',state?.preview?previewMedia('mesh'):null);
  }
  function loadMedia(id,m) {
    const card=el('media-'+id);
    let slot=mediaSlots.get(id);
    if(!slot){slot={card,image:card.querySelector('img'),empty:card.querySelector('.empty'),version:0};mediaSlots.set(id,slot);}
    slot.gpu=false;
    if(doc.hidden || !m){clearSlot(slot);slot.empty.textContent=doc.hidden?'页面后台，画面订阅已暂停':state?.device?.id==='protocol-fake'?'协议 Fake（L1）不提供 PhysX 相机图像；完整 Fake 需要配置仿真与图像服务。':'尚无当前环境的可核验画面';updateMediaLabel(slot);return;}
    const url=safeMediaUrl(m.url,location.href);
    const retainable=['target','overview'].includes(m.preview_view);
    const sourceKey=JSON.stringify([key(),state?.device?.id,serverInstance,m.server_instance_id,m.preview_view==='overview'?'target':m.preview_view,m.renderer_id,m.display_context,state?.runtime?.calibration_revision,state?.runtime?.calibration_snapshot_id]);
    const streamKey=JSON.stringify([key(),m.id,m.kind,m.candidate_revision??null,m.preview_view==='target'?targetBinding():m.revision??null,m.view_revision??null,m.preview_view,m.renderer_id,m.display_context,m.camera,m.candidate_id,url]);
    if(slot.streamKey!==streamKey){clearSlot(slot,retainable&&slot.sourceKey===sourceKey);slot.streamKey=streamKey;slot.sourceKey=sourceKey;slot.url=url;}
    slot.media={...m,...(slot.headerFrame!==null?{frame_id:slot.headerFrame}:{}),...(slot.headerSource!==null?{source_wall_ns:slot.headerSource}:{}),applied_revision:slot.appliedRevision};
    if(!m.ready||!url){cancelMediaRequest(slot);slot.error=m.error||(!url?'尚无有效的同源画面地址':'画面尚未就绪');slot.empty.textContent=slot.error;if(m.preview_view&&slot.blob){if(retainable)slot.retained=true;else{urls.revokeObjectURL(slot.blob);slot.blob=null;slot.image.removeAttribute('src');}}if(!slot.blob){slot.image.hidden=true;slot.empty.hidden=false;}updateMediaLabel(slot);return;}
    updateMediaLabel(slot);
    if(!slot.controller&&!slot.timer)void nextMediaFrame(slot,slot.version);
  }
  const frameIdentity=metadata=>JSON.stringify([metadata.server_instance_id,metadata.display_context,metadata.renderer_id,metadata.view,metadata.view_revision,metadata.revision,metadata.frame_id,metadata.source_wall_ns]);
  function displayFrame(slot,blob,metadata,sourceNs,frame){
    if(Number.isFinite(sourceNs)){slot.headerSource=sourceNs;slot.media.source_wall_ns=sourceNs;}
    if(frame!==null&&frame!==undefined){slot.headerFrame=frame;slot.media.frame_id=frame;}
    if(metadata?.view==='target'){slot.appliedRevision=metadata.revision;slot.media.applied_revision=metadata.revision;slot.appliedIdentity=frameIdentity(metadata);}
    slot.image.src=blob;if(slot.blob&&slot.blob!==blob)urls.revokeObjectURL(slot.blob);slot.blob=blob;
    slot.error='';slot.retained=false;slot.image.hidden=false;slot.empty.hidden=true;targetPresentation(slot,metadata);
    if(metadata&&['target','overview'].includes(metadata.view))slot.card.classList.toggle('split',metadata.top_view===true);
    if(metadata?.view==='overview'&&metadata.revision===overviewFlight?.revision){
      clearTimeout(overviewFlight.timer);overviewFlight=null;
      if(overviewDirty)Promise.resolve().then(()=>{if(!stopped&&!doc.hidden&&previewView==='overview')void sendOverview();});
    }
  }
  function presentNextTarget(slot){
    const job=targetFlights[0];if(slot.presenting||!job?.frame||job.binding!==targetBinding())return;
    const frame=job.frame;job.frame=null;slot.presenting=job;
    displayFrame(slot,frame.blob,frame.metadata,frame.metadata.source_wall_ns,frame.metadata.frame_id);
  }
  function kickTargetMedia(){
    const slot=mediaSlots.get('preview');if(!slot||slot.media?.preview_view!=='target'||slot.controller||!slot.media.ready)return;
    clearTimeout(slot.timer);slot.timer=null;void nextMediaFrame(slot,slot.version);
  }
  async function nextMediaFrame(slot,version) {
    if(version!==slot.version||stopped||doc.hidden||!slot.media?.ready)return;
    const target=slot.media.preview_view==='target',job=target?targetFlights.find(item=>Number.isInteger(item.revision)&&!item.frame&&!item.fetching&&slot.presenting!==item):null;
    if(target&&targetFlights.length&&!job)return;
    let requestUrl=slot.url;
    if(job){
      const url=new URL(slot.url,location.href);for(const [key,value]of Object.entries({revision:job.revision,...job.identity,wait_ms:100}))if(value!==undefined)url.searchParams.set(key,value);
      requestUrl=url.href;job.fetching=true;
    }
    const started=clock.now(),controller=new AbortController(),at=generation;slot.controller=controller;
    slot.fetchJob=job;
    let blobUrl,failed=false,timedOut=false;
    slot.timeout=setTimeout(()=>{timedOut=true;controller.abort();},2500);
    try {
      const response=await request(requestUrl,{signal:controller.signal,cache:'no-store',headers:slot.etag?{'If-None-Match':slot.etag}:{}});
      if(version!==slot.version||at!==generation||controller.signal.aborted)return;
      if(job&&!targetFlights.includes(job))return;
      if(response.status===304){slot.error='';return;}
      if(response.status===409&&['target','overview'].includes(slot.media.preview_view))return;
      if(!response.ok)throw Error('画面接口 '+response.status);
      const expected=job?{...slot.media,...job.identity,revision:job.revision,minimum_revision:job.revision}:slot.media;
      const metadata=slot.media.preview_view?verifyPreviewFrame(expected,JSON.parse(response.headers?.get('X-Frame-Metadata')||'null'),context.environment):null;
      if(metadata?.view==='target'&&(metadata.revision<slot.appliedRevision||slot.headerSource!==null&&metadata.source_wall_ns<slot.headerSource
          ||slot.headerFrame===metadata.frame_id&&slot.appliedRevision!==metadata.revision))throw Error('完成目标帧发生倒退或身份复用，丢弃旧画面');
      const blob=await response.blob();
      if(version!==slot.version||at!==generation||controller.signal.aborted)return;
      if(job&&!targetFlights.includes(job))return;
      if(metadata?.view==='target'&&slot.appliedIdentity===frameIdentity(metadata)){
        slot.error='';
        if(audit?.active)audit.window.duplicates++;
        return;
      }
      const source=metadata?.source_wall_ns??response.headers?.get('X-Source-Wall-Ns'),frame=metadata?.frame_id??response.headers?.get('X-Frame-Id');
      const sourceNs=source===null||source===undefined||source===''?null:Number(source);
      blobUrl=urls.createObjectURL(blob);const decoded=make('img');decoded.src=blobUrl;
      if(decoded.decode)await decoded.decode();
      if(version!==slot.version||at!==generation||controller.signal.aborted||job&&!targetFlights.includes(job)){urls.revokeObjectURL(blobUrl);blobUrl=null;return;}
      // Only a decoded source frame establishes liveness; HTTP arrival and 304 do not.
      slot.etag=response.headers?.get('ETag')||null;
      if(job){
        job.frame={blob:blobUrl,metadata};blobUrl=null;
        if(job.auditRequest)job.auditRequest.row.decoded_ms=clock.now()-job.auditRequest.window.started_ms;
        presentNextTarget(slot);
      }else{displayFrame(slot,blobUrl,metadata,sourceNs,frame);blobUrl=null;}
    } catch(error) {
      if(blobUrl)urls.revokeObjectURL(blobUrl);
      if(version===slot.version&&at===generation&&(!controller.signal.aborted||timedOut)){
        failed=true;slot.error=timedOut?'画面请求超时':'画面读取失败：'+error.message;
        if(!slot.blob){slot.empty.textContent=slot.error;slot.empty.hidden=false;slot.image.hidden=true;}
      }
    } finally {
      if(job)job.fetching=false;
      if(version===slot.version){
        clearTimeout(slot.timeout);slot.timeout=null;slot.controller=null;slot.fetchJob=null;updateMediaLabel(slot);
        if(!stopped&&!doc.hidden&&at===generation&&slot.media?.ready){
          const hz=Number.isFinite(slot.media.poll_hz)&&slot.media.poll_hz>0?Math.min(60,slot.media.poll_hz):25;
          slot.timer=setTimeout(()=>{slot.timer=null;void nextMediaFrame(slot,version);},failed?250:target&&targetFlights.length?0:Math.max(0,1000/hz-(clock.now()-started)));
        }
      }
    }
  }
  const recordingContext=()=>JSON.stringify([key(),serverInstance,state?.device?.id]);
  function renderRecordingFiles(){
    const box=el('recording-files-list');box.replaceChildren();
    for(const row of recordingFiles){
      const label=make('label',null,'check-label'),input=make('input');input.type='checkbox';input.checked=recordingSelection.has(row.run_id);
      input.disabled=recordingFilesBusy||!row.deletable;
      input.addEventListener('change',()=>{if(input.checked)recordingSelection.add(row.run_id);else recordingSelection.delete(row.run_id);renderRecordingFiles();});
      label.append(input,make('span',`${recordingTaskTime(row.started_wall_ns)||'时间未知'} · ${row.run_id.slice(0,8)} · ${statusText(row.state)}${row.reason?' · '+row.reason:''}`));box.append(label);
    }
    el('recording-files-delete').disabled=recordingFilesBusy||!recordingSelection.size||recordingSelection.size>100;
    el('recording-files-delete').textContent=`删除所选任务${recordingSelection.size?'（'+recordingSelection.size+'）':''}`;
    el('recording-files-refresh').disabled=recordingFilesBusy;el('recording-files-select').disabled=recordingFilesBusy||!recordingFiles.some(row=>row.deletable);
  }
  async function loadRecordingFiles(){
    if(context.business!=='inference'||recordingFilesBusy)return;
    const own=recordingContext();recordingFilesBusy=true;renderRecordingFiles();
    setText('recording-files-status','正在读取录制文件列表…');
    try{
      const response=await request('/api/inference-recordings?'+new URLSearchParams(context),{cache:'no-store'}),value=await response.json();
      if(!response.ok)throw Error(value.error||'文件列表读取失败');
      if(own!==recordingContext())return;
      recordingFiles=value.recordings;recordingFilesContext=own;recordingSelection.clear();
      setText('recording-files-status',recordingFiles.length?`共 ${recordingFiles.length} 条录制任务`:'没有录制文件。');
    }catch(error){if(own===recordingContext())setText('recording-files-status',error.message);}
    finally{recordingFilesBusy=false;renderRecordingFiles();}
  }
  function reviewRecordingDeletion(){
    if(recordingFilesBusy||recordingFilesContext!==recordingContext()||!recordingSelection.size||recordingSelection.size>100)return;
    recordingDeleteIntent={context:key(),server:serverInstance,run_ids:[...recordingSelection]};
    const list=el('recording-files-selection');list.replaceChildren();
    for(const id of recordingDeleteIntent.run_ids){const row=recordingFiles.find(item=>item.run_id===id);list.append(make('li',`${recordingTaskTime(row?.started_wall_ns)||'时间未知'} · ${id.slice(0,8)}`));}
    el('recording-files-confirm').showModal();
  }
  async function deleteRecordingFiles(){
    const intent=recordingDeleteIntent;recordingDeleteIntent=null;el('recording-files-confirm').close();
    if(!intent||intent.context!==key()||intent.server!==serverInstance||recordingFilesBusy||!online)return;
    pausePair();recordingFilesBusy=true;renderRecordingFiles();setText('recording-files-status','正在删除所选任务…');
    try{
      const response=await request('/api/actions/inference_delete_recordings',{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':state.csrf},
        body:JSON.stringify({...context,request_id:crypto.randomUUID(),run_ids:intent.run_ids,confirmed:true})}),value=await response.json();
      if(!response.ok)throw Error(value.error||'删除失败');
      if(intent.context!==key()||intent.server!==serverInstance)return;
      recordingFiles=recordingFiles.filter(row=>!value.deleted_run_ids.includes(row.run_id));recordingSelection.clear();
      setText('recording-files-status',`已删除 ${value.deleted_run_ids.length} 条任务。${value.failed.length?'部分文件未删除，请刷新后重试：'+value.failed.map(row=>row.error).join('；'):''}`);
      historySelections.delete(historyKey());resultsCursor=null;void poll();
    }catch(error){if(intent.context===key())setText('recording-files-status','删除结果未确认：'+error.message+'；请刷新列表核对，不会自动重试。');}
    finally{recordingFilesBusy=false;renderRecordingFiles();}
  }
  function renderHistoryPaging(){
    const page=state?.results_page;
    el('inference-history-pages').hidden=!page;
    el('inference-history-older').disabled=resultsBrowsing||!page?.older_cursor;
    el('inference-history-newer').disabled=resultsBrowsing||!page?.newer_cursor;
    el('inference-history-latest').disabled=resultsBrowsing||!resultsCursor;
    setText('inference-history-page-info',resultsBrowsing?'正在读取历史页…':page?`本页 ${page.run_ids.length} 个任务 · 共 ${page.total_runs} 个任务／${page.total_results} 项结果`:'');
  }
  function browseInferenceHistory(direction){
    if(resultsBrowsing||!state?.results_page)return;
    const cursor=state.results_page[direction==='before'?'older_cursor':'newer_cursor'];
    if(direction&&(!Number.isSafeInteger(cursor)||cursor<=0))return;
    pausePair();resultsCursor=direction?{['results_'+direction]:cursor}:null;resultsBrowsing=true;
    polling?.abort();polling=null;renderHistoryPaging();void poll();
  }
  function renderResults() {
    renderHistoryPaging();
    el('recording-files').hidden=context.business!=='inference';
    if(recordingFilesContext&&recordingFilesContext!==recordingContext()){
      recordingFiles=[];recordingFilesContext='';recordingSelection.clear();recordingDeleteIntent=null;
      el('recording-files').open=false;el('recording-files-confirm').close();renderRecordingFiles();setText('recording-files-status','');
    }
    const deletion=state?.workflow?.deletion_preview;
    el('delete-tools').hidden=context.business!=='calibration';el('deletion-preview').hidden=context.business!=='calibration'||!deletion;
    const nextDeletionKey=JSON.stringify([key(),deletion?.id]);
    if(nextDeletionKey!==deletionKey){deletionKey=nextDeletionKey;el('delete-clearance').checked=false;if(deletion)el('deletion-preview').scrollIntoView?.({block:'center'});}
    if(deletion){const installed=deletion.preserved_installed;
      renderDetails('deletion-details',[{label:'删除范围',value:deletion.scope==='all'?'全部本地原始批次':'单个本地原始批次'},{label:'本地根目录',value:deletion.root},
        {label:'批次列表',value:(deletion.batches||[]).join('\n')},{label:'图像文件数',value:deletion.image_files},{label:'全部文件数',value:deletion.files},
        {label:'占用空间',value:Number.isFinite(deletion.bytes)?(deletion.bytes<1024?deletion.bytes+' B':deletion.bytes<1048576?(deletion.bytes/1024).toFixed(2)+' KiB':(deletion.bytes/1048576).toFixed(2)+' MiB'):'未知'},
        {label:'保留的结果版本',value:deletion.preserved_versions?.length?deletion.preserved_versions.join(' / '):'本次范围没有已登记版本'},
        {label:'保留的安装组',value:installed?'头部 '+installed.head_id+' / 双腕 '+installed.wrist_id:'当前没有安装组'},
        {label:'保留内容',value:(deletion.preserved||[]).join('；')},{label:'确认后会清空',value:deletion.clears?.length?deletion.clears.join('；'):'不清空其他批次的求解来源或显示选择'},
        {label:'边界',value:deletion.detail},{label:'预览有效性',value:deletion.valid?'有效；确认时后端仍会重新核对文件清单':deletion.reason||'预览已失效'}]);
    }
    const deletionResult=state?.workflow?.deletion_result;
    setText('deletion-result',(deletionResult?.message||'')+(deletionResult?.overlay_error?'；叠加清理未完成：'+deletionResult.overlay_error:''));
    const installed=state?.workflow?.calibration_tools?.installed;
    el('installed-group').hidden=context.business!=='calibration';
    renderDetails('installed-details',installed?Object.entries(installed).map(([label,value])=>({label:({head_id:'头部结果',wrist_id:'双腕结果',head_version:'头部版本',wrist_version:'双腕版本',verified:'active 文件校验',problems:'校验问题',path:'安装目录',geometry_id:'几何身份',installed_at:'安装时间',acceptance:'验收依据',notes:'说明',environment:'环境'}[label]||label),value:label==='verified'?(value===true?'通过':value===false?'未通过':'未知'):value})):[{label:'当前安装组',value:'尚未安装'}]);
    const box=el('results-list'),allResults=state?.results||[],currentRun=state?.workflow?.id||'__current__';
    const runFor=entry=>entry.run_id||entry.media?.run_id||(state?.results_page?entry.id:currentRun)||currentRun;
    const allMedia=[...(state?.media||[]),...(state?.workflow?.replay||[]),...allResults.map(row=>row.media).filter(Boolean)].filter(row=>row.kind==='replay');
    const captureRow=row=>row.details?.seal_revision!==undefined||row.id?.startsWith('capture-');
    const collection=context.business!=='calibration'||!!state?.results_page,runs=new Set([currentRun,...allMedia.map(runFor),...allResults.map(runFor)]);
    const nextHistoryRun=collection&&runs.has(historySelections.get(historyKey()))?historySelections.get(historyKey()):
      (resultsCursor||context.business!=='inference')&&state?.results_page?.run_ids.length?state.results_page.run_ids[0]:currentRun;
    if(nextHistoryRun!==historyRun)pausePair();historyRun=nextHistoryRun;
    el('collection-history-controls').hidden=!collection;
    const historyLabel=context.business==='inference'?'推理任务':context.business==='collection'?'数采任务':'控制录制任务';
    setText('history-business-label',historyLabel);
    el('collection-history-run').setAttribute('aria-label',historyLabel);
    const runTime=run=>recordingTaskTime((run===currentRun?state?.runtime?.recording?.recording_activity?.started_wall_ns:null)
      ??allResults.find(row=>runFor(row)===run&&Number.isFinite(row.details?.started_wall_ns))?.details.started_wall_ns);
    const runLabel=run=>{const stamp=runTime(run);return stamp?`${stamp} · ${run===currentRun?'当前':historyLabel}`:run===currentRun?'当前任务（尚未录制）':`时间未知 · ${historyLabel}`;};
    const selector=el('collection-history-run'),optionsKey=JSON.stringify([key(),currentRun,[...runs].map(run=>[run,runLabel(run)])]);
    if(selector.dataset.options!==optionsKey){selector.dataset.options=optionsKey;selector.replaceChildren();
      for(const run of runs){const option=make('option',runLabel(run));option.value=run;option.title=run==='__current__'?'尚未生成当前任务':run;selector.append(option);}
    }
    selector.value=historyRun;
    const selectedMedia=[...new Map(allMedia.filter(row=>!collection||context.business==='calibration'&&!captureRow(row)||runFor(row)===historyRun).map(row=>[JSON.stringify([runFor(row),row.id||row.url]),{...row,run_id:runFor(row)}])).values()];
    setText('collection-history-summary',collection?(historyRun===currentRun?'正在查看当前任务':'正在查看历史任务，当前任务状态仍显示在'+BUSINESS[context.business]+'工作区')+' · '+selectedMedia.length+' 项回放；只读浏览，不改变控制权或继续动作。':'');
    el('collection-history-summary').title=historyRun==='__current__'?'尚未生成当前任务':historyRun;
    const results=allResults.filter(row=>!collection||context.business==='calibration'&&!captureRow(row)||runFor(row)===historyRun);
    const signature=JSON.stringify([key(),historyRun,results]);
    if(signature!==resultsKey){resultsKey=signature;box.replaceChildren();resultActions=[];
    if(!results.length)box.append(make('p','当前业务尚无结果回执。','muted'));
    const addAction=(container,label,action,body,rule={})=>{const cell=make('div',undefined,'action-cell'),button=make('button',label),reason=make('p','', 'action-reason');button.addEventListener('click',()=>send(action,typeof body==='function'?body():body));cell.append(button,reason);container.append(cell);resultActions.push({button,reason,action,allowed:rule.allowed,ruleReason:rule.reason||'此结果当前不可安装'});};
    for(const result of results){
      const row=make('article',undefined,'result-row'),label=make('div'),actions=make('div',undefined,'result-actions');
      label.append(make('h3',result.kind==='control_recording'&&runTime(runFor(result))?runLabel(runFor(result)):result.label||result.id),make('p',({calibration:'头部与双腕完整标定',head:'头部中间候选',wrist:'双腕中间候选',capture:'采集批次',recording:'录制视频',control_recording:'控制过程录制',replay:'过程视频'}[result.kind]||result.kind||'结果')+' · '+(result.recording_quality?'旧版判定：':'')+statusText(result.status)));
      if(result.recording_quality){
        const quality=result.recording_quality,dimensions=quality.dimensions||{},list=make('dl',undefined,'detail-list');
        for(const [name,title] of [['motion','控制执行'],['data','数据覆盖'],['video','过程视频'],['training','训练用途']]){
          const item=make('div');item.append(make('dt',title),make('dd',dimensions[name]?.label||'待核验'));list.append(item);
        }
        label.append(list,make('p','原始记录与旧版判定保留；视频可用不代表任务或训练资格通过。','muted'));
        if(quality.quality_error)label.append(make('p','质量报告尚未核验：'+quality.quality_error,'muted'));
        const details=make('details'),selection=[];
        details.append(make('summary','采样区间、缺口与片段筛选'),make('p','区间仅表示单相机已接收帧号连续；像素质量、跨相机同步及训练用途仍需核验。勾选只生成引用清单，不修改原始数据。','muted'));
        for(const [camera,evidence] of Object.entries(quality.cameras||{})){
          details.append(make('h4',evidence.label||camera));
          const sequences=evidence.consecutive_sequences||[],gaps=evidence.discontinuities||[];
          details.append(make('p',`已接收 ${evidence.accepted_pairs??'未知'} 对 · 连续区间 ${evidence.sequence_count??sequences.length} 段 · 缺口 ${evidence.discontinuity_count??gaps.length} 处`));
          for(const span of sequences.slice(0,100)){
            const option=make('label'),check=make('input');check.type='checkbox';
            selection.push({camera,span,check});option.append(check,make('span',`帧索引 ${span.first_index}–${span.last_index} · ${(Number(span.last_time_ns-span.first_time_ns)/1e9).toFixed(3)} 秒`));details.append(option);
          }
          if((evidence.sequence_count??sequences.length)>100)details.append(make('p','页面仅列前 100 段；全部区间见下载的质量报告。','muted'));
          if(gaps.length)details.append(make('pre',JSON.stringify(gaps.slice(0,20),null,2)));
          if((evidence.discontinuity_count??gaps.length)>20)details.append(make('p','页面仅列前 20 处缺口；完整明细见下载的质量报告。','muted'));
          const ingress=evidence.ingress_diagnostics||{};
          details.append(make('p','入口诊断：'+({snapshot_references:'仅引用记录时可见的日志前缀',unsupported:'仿真源不具备硬件 SDK 阶段',not_available:'当前记录没有入口日志引用'}[ingress.status]||'尚未核验'),'muted'));
        }
        const downloadJSON=(name,value)=>{
          const url=urls.createObjectURL(new Blob([JSON.stringify(value,null,2)],{type:'application/json'})),link=make('a');
          link.href=url;link.download=name;link.click();setTimeout(()=>urls.revokeObjectURL(url),1000);
        };
        const report=make('a','下载完整质量报告'),pick=make('button','下载已选片段清单');
        report.href='/api/recording-quality?'+new URLSearchParams({environment:context.environment,business:context.business,result_id:result.id});
        report.download='recording-quality.json';report.hidden=!quality.raw_manifest_sha256;
        pick.disabled=true;
        for(const {check} of selection)check.addEventListener('change',()=>{pick.disabled=!selection.some(row=>row.check.checked);});
        pick.addEventListener('click',()=>downloadJSON('recording-segment-selection.json',{
          schema:'recording_segment_selection.v1',raw_manifest_sha256:quality.raw_manifest_sha256,
          result_id:result.id,environment:context.environment,training_status:'unassessed',raw_modified:false,
          scope:'per-camera accepted-index references; no synchronized multimodal or training validity asserted',
          segments:selection.filter(row=>row.check.checked).map(({camera,span})=>({camera,...span}))}));
        details.append(report,pick);label.append(details);
      }
      if(result.kind!=='recording'&&!captureRow(result)){
        addAction(actions,'查看此结果','select_result',()=>{
          const selected=state?.workflow?.replay_context;
          const sameBatch=selected&&!['capture','calibration'].includes(result.kind)&&(result.details?.batch===selected.batch_id||selected.candidates?.some(candidate=>candidate.id===result.id));
          return sameBatch?{result_id:selected.result_id,candidate_id:result.id,frame_index:selected.frame_index,camera:selected.camera,overlay:selected.overlay===true}:{result_id:result.id};
        });
      }
      else actions.append(make('p',result.kind==='control_recording'?'只读封存产物':'下方独立回放','muted'));
      if(result.kind==='capture')addAction(actions,'预览删除原始批次','delete_preview',{scope:'batch',result_id:result.id});
      if(result.raw_deleted)label.append(make('p','原始批次已删除；已登记版本、参数和依赖快照保留。'));
      if(result.install){const install=result.install;label.append(make('p','安装组：头部 '+String(install.head_result_id||result.id).slice(0,12)+' + 双腕 '+String(install.wrist_result_id||'尚无').slice(0,12)));addAction(actions,install.installed?'当前有效标定':'安装这组头部与双腕','install',{result_id:result.id},install);}
      if(result.details||result.id){
        const details=make('details'),list=make('dl',undefined,'detail-list');details.append(make('summary','质量指标、依赖与来源'));
        for(const [field,value]of Object.entries({id:result.id,...result.details})){const entry=make('div');entry.append(make('dt',({id:'结果身份',metrics_rectified_px:'贴纸像素误差',mesh_metrics:'Mesh 留出指标',acceptance:'验收结论',optimizer:'求解收敛',left:'左腕结果',right:'右腕结果',head_result_id:'依赖头部结果',group:'头部与双腕组',batch:'来源批次',version:'版本',environment:'环境',robot_source:'设备来源',simulation_only:'仅仿真',capture_coverage:'采集覆盖',file:'结果文件',objective:'求解目标',completion:'采集完整性'}[field]||field)),make('dd',value));list.append(entry);}
        details.append(list);label.append(details);
      }
      const download=safeMediaUrl(result.download_url,location.href);
      if(download){const link=make('a',result.kind==='calibration'?'下载完整标定 JSON':result.kind==='control_recording'?'下载本次 Run 封存回执':result.kind==='inference_debug'?'下载本次 Run 产物索引':'下载中间候选 JSON');link.href=download;link.download='';actions.append(link);}
      row.append(label,actions);box.append(row);
    }
    }
    const view=state?.workflow?.replay_context,viewKey=JSON.stringify([key(),view]);
    el('replay-controls').hidden=!view;
    if(view&&viewKey!==replayControlsKey){
      replayControlsKey=viewKey;el('replay-camera').replaceChildren();
      for(const camera of view.cameras||[]){const option=make('option',{head:'头部',left:'左腕',right:'右腕'}[camera]||camera);option.value=camera;el('replay-camera').append(option);}
      const frameLabel=(frame,index)=>`${index+1}. ${frame.name||frame.id||'帧'} · ${{fit:'拟合',holdout:'留出'}[frame.split]||frame.split||'未分组'} · ${(frame.valid_for_fit??frame.valid)===true?'可用于求解':(frame.valid_for_fit??frame.valid)===false?'不可用于求解':'有效性未知'}`;
      el('replay-index').replaceChildren();
      for(let index=0;index<view.frame_count;index++){const option=make('option',frameLabel(view.frames?.[index]||{},index));option.value=index;el('replay-index').append(option);}
      el('replay-camera').value=view.camera;el('replay-index').value=view.frame_index;el('replay-index').max=Math.max(0,view.frame_count-1);el('replay-overlay').checked=view.overlay===true;
      const candidates=el('replay-candidate');candidates.replaceChildren();
      const blank=make('option','选择当前帧的标定候选');blank.value='';candidates.append(blank);
      for(const candidate of view.candidates||[]){const option=make('option',(candidate.label||candidate.id)+(candidate.status?' · '+statusText(candidate.status):''));option.value=candidate.id;candidates.append(option);}
      candidates.value=view.candidate_id||view.overlay_result_id||'';
      const frame=view.frame||{};
      setText('replay-frame-details',frameLabel(frame,view.frame_index)+(frame.detections||frame.detections_by_camera?'\n当前相机检测：'+textValue(frame.detections_by_camera?.[view.camera]??frame.detections):'')+(frame.reason?'\n原因：'+frame.reason:''));
      setText('replay-context',`${view.batch_id} · 第 ${view.frame_count?view.frame_index+1:0} / ${view.frame_count} 帧${view.message?' · '+view.message:''}`);
    }
    const media=selectedMedia,nextKey=JSON.stringify([key(),historyRun,media]);
    if(nextKey===replayKey)return;pausePair();replayKey=nextKey;replayVideos=[];const replay=el('replay-media');replay.replaceChildren();
    for(const m of media){const url=safeMediaUrl(m.url,location.href);if(!m.ready||!url)continue;const figure=make('figure'),video=/\.(mp4|webm)(\?|$)/i.test(url)||m.mime_type?.startsWith('video/');const node=make(video?'video':'img');node.src=url;if(video){node.controls=true;node.preload='metadata';node.playsInline=true;node.addEventListener('loadedmetadata',renderPairClock);replayVideos.push({node,figure,camera:m.camera,role:m.role,run_id:m.run_id});}else node.alt=m.label||'录制回放';figure.append(make('figcaption',(m.label||'录制回放')+' · 回放'),node);replay.append(figure);}
    const cameras=[...new Set(replayVideos.filter(row=>['source','target'].includes(row.role)).map(row=>row.camera))];
    el('collection-replay-controls').hidden=!cameras.length;el('collection-replay-camera').replaceChildren();
    for(const camera of cameras){const option=make('option',{base_0:'头部',left_wrist_0:'左腕',right_wrist_0:'右腕'}[camera]||camera);option.value=camera;el('collection-replay-camera').append(option);}
    if(cameras.length){el('collection-replay-camera').value=cameras.includes(pairedCamera)?pairedCamera:cameras[0];selectPair();}
  }
  function updateMeshCatalog(incoming,requestedContext,at){
    if(Array.isArray(incoming.preview?.mesh_calibrations))return;
    if(requestedContext.environment!=='physical'||incoming.preview?.capabilities?.mesh!==true)return;
    const identity=meshCatalogIdentity(incoming);
    if(Array.isArray(incoming.preview.mesh_candidates)||requestedContext.business==='calibration'){
      meshCatalogRequest?.abort();meshCatalogRequest=null;
      meshCatalog={identity,rows:Array.isArray(incoming.preview.mesh_candidates)?incoming.preview.mesh_candidates:incoming.results||[]};
    }
    if(meshCatalog?.identity===identity){
      incoming.preview.mesh_candidates=meshCatalog.rows;
      if(meshCatalog.error)incoming.preview.mesh_catalog_error=meshCatalog.error;
      return;
    }
    incoming.preview.mesh_candidates=[];
    if(meshCatalogRequest?.identity===identity)return;
    meshCatalogRequest?.abort();const controller=new AbortController();controller.identity=identity;meshCatalogRequest=controller;
    const current=()=>at===generation&&!controller.signal.aborted&&meshCatalogRequest===controller
      &&context.environment==='physical'&&meshCatalogIdentity(state)===identity;
    void (async()=>{
      try{
        const response=await request('/api/state?environment=physical&business=calibration',{cache:'no-store',signal:controller.signal});
        if(!response.ok)throw Error('标定状态接口 '+response.status);
        const value=await response.json();if(!current())return;
        if(value.schema!=='robot-workbench-ui.v1'||value.device?.environment!=='physical'||meshCatalogIdentity(value)!==identity)
          throw Error('标定目录属于已变化的服务或安装，请切到标定栏目重新核验');
        meshCatalog={identity,rows:Array.isArray(value.preview?.mesh_candidates)?value.preview.mesh_candidates:value.results||[]};
      }catch(error){if(!current())return;meshCatalog={identity,rows:[],error:'无法读取真机标定候选：'+error.message};}
      finally{if(meshCatalogRequest===controller)meshCatalogRequest=null;}
      if(at!==generation||controller.signal.aborted||meshCatalogIdentity(state)!==identity)return;
      state.preview.mesh_candidates=meshCatalog.rows;
      if(meshCatalog.error)state.preview.mesh_catalog_error=meshCatalog.error;
      renderPreview();renderMedia();renderGates();
    })();
  }
  async function poll() {
    if(stopped||polling||environmentAccess?.exited)return;
    const controller=new AbortController(),requestedContext={...context};let at=generation;polling=controller;
    try {
      const query=new URLSearchParams({...requestedContext,...resultsCursor}),response=await request('/api/state?'+query,{cache:'no-store',signal:controller.signal});
      if(!response.ok)throw Error('状态接口 '+response.status);const incoming=await response.json();
      if(at!==generation||controller.signal.aborted)return;
      if(incoming.schema!=='robot-workbench-ui.v1'||!Number.isFinite(incoming.revision)||incoming.device?.environment!==context.environment)throw Error('状态契约或设备环境不匹配');
      if(!acceptServerInstance(incoming.server_instance_id??null,incoming.gateway_route?.route_id??null))return;
      at=generation;
      if(incoming.revision<revision)return;
      if(resultsBrowsing){historySelections.delete(historyKey());resultsBrowsing=false;}
      revision=incoming.revision;state=incoming;receivedAt=clock.now();online=true;
      updateMeshCatalog(incoming,requestedContext,at);
      if(waitingForStateNotice)notice('设备与任务状态已更新。');
      const failed=incoming.workflow?.request;
      if(failed?.status==='failed'){
        const failureKey=JSON.stringify([key(),serverInstance,failed.id,failed.finished_wall_ns,failed.error]);
        if(supersedingModelPreparation(incoming,environmentAccess?.state)){
          if(activeFailureNoticeKey===failureKey)notice('所选模型已由更新的运维操作准备并通过核验；此前模型工具错误保留在技术详情。');
        }else if(failureKey!==requestFailureNoticeKey){
          requestFailureNoticeKey=failureKey;
          const cameraFailure=failed.action==='inference_run'&&failed.stage==='reviewing'&&/Connection refused/.test(failed.error||'');
          notice('本次请求失败：'+(cameraFailure?'无法读取三相机 RGB-D，相机服务连接被拒绝。本次未下发模型动作，请先恢复相机连接。':failed.error||'后端未提供原因'),false,false,failureKey);
        }
      }
      render();
    } catch(error) {if(at===generation&&!controller.signal.aborted){online=false;clearReview();notice('状态核验失败：'+error.message);render();}}
    finally{if(polling===controller){polling=null;resultsBrowsing=false;renderHistoryPaging();}}
  }
  function acceptServerInstance(instance,route){
    if(instance!==null&&(typeof instance!=='string'||!instance))throw Error('服务实例身份无效');
    if(serverInstance===undefined){serverInstance=instance;serverRoute=route;return true;}
    if(instance===serverInstance&&route===serverRoute)return true;
    // Legacy snapshots may be read before upgrading, but cannot replace a known instance.
    if(instance===null||retiredServerInstances.has(JSON.stringify([instance,route])))return false;
    manualExpiryMessage='';
    dismissManualExecution();
    dismissInferenceRun();
    finishAudit('服务实例变化');
    cancelTarget('服务实例变化');
    if(serverInstance!==null)retiredServerInstances.add(JSON.stringify([serverInstance,serverRoute]));
    serverInstance=instance;serverRoute=route;generation++;revision=-1;displayedRequest=++requestSequence;
    resultsCursor=null;resultsBrowsing=false;
    meshCatalogRequest?.abort();meshCatalogRequest=null;meshCatalog=null;
    for(const request of pending.values())request.controller.abort();pending.clear();
    for(const controller of displayRequests.values())controller.abort();displayRequests.clear();
    clearMedia();clearReview();invalidatedReviews.clear();confirmedDeletions.clear();previewFloors.clear();previewErrors.clear();
    reviewKey='';manualKey='';manualQueueKey='';calibrationKey='';calibrationPlanKey='';resultsKey='';replayKey='';replayControlsKey='';deletionKey='';
    previewView='target';meshCamera='head';meshCandidate='';meshAnchor='';meshStateKey='';meshOptionsKey='';previewRenderer=null;
    overview={yaw:-Math.PI/2,pitch:.4,zoom:1,pan_y:0};overviewDirty=false;overviewSending=false;overviewPointer=null;
    el('delete-clearance').checked=false;
    notice('工作台服务已重新连接，旧页面审批已清除；请按当前状态重新审阅。');
    return true;
  }
  function barrier() {
    manualExpiryMessage='';
    dismissManualExecution();
    dismissInferenceRun();
    clearTimeout(autoPreviewTimer);autoPreviewTimer=null;
    cancelTarget('页面控制或上下文已改变');
    cancelOverview();
    generation++;polling?.abort();polling=null;clearReview();
    meshCatalogRequest?.abort();meshCatalogRequest=null;
    for(const slot of mediaSlots.values())cancelMediaRequest(slot);
  }
  function cancelTarget(reason){
    for(const job of [...targetFlights,targetPending])if(job){
      const row=job.auditRequest?.row;if(row&&row.presented_ms===undefined)row.cancelled=row.cancelled||reason;
      clearTimeout(job.timer);job.controller?.abort();job.resolve();
      if(job.frame)urls.revokeObjectURL(job.frame.blob);
      if(displayRequests.get('target')===job.controller)displayRequests.delete('target');
    }
    targetFlights.length=0;targetPending=null;
    const slot=mediaSlots.get('preview');if(slot){if(slot.fetchJob)slot.controller?.abort();slot.presenting=null;}
  }
  function retireTarget(job,reason){
    const index=targetFlights.indexOf(job);if(index<0)return;
    targetFlights.splice(index,1);clearTimeout(job.timer);job.controller?.abort();job.resolve();
    if(job.frame)urls.revokeObjectURL(job.frame.blob);
    const row=job.auditRequest?.row;if(row&&row.presented_ms===undefined)row.cancelled=row.cancelled||reason;
    const slot=mediaSlots.get('preview');if(slot){if(slot.fetchJob===job)slot.controller?.abort();if(slot.presenting===job)slot.presenting=null;presentNextTarget(slot);}
    if(displayRequests.get('target')===job.controller)displayRequests.delete('target');
  }
  async function dispatchTarget(){
    if(targetFlights.length>=3||displayRequests.has('target')||displayRequests.has('recovery')||!targetPending)return;
    const slot=mediaSlots.get('preview');if(!targetFlights.length&&slot?.controller&&!slot.fetchJob)cancelMediaRequest(slot);
    const job=targetPending;targetPending=null;targetFlights.push(job);
    if(job.binding!==targetBinding()){cancelTarget('目标输入的上下文已改变');return;}
    job.timer=setTimeout(()=>{
      retireTarget(job,'目标未在2.5秒内完成匹配呈现');void dispatchTarget();kickTargetMedia();renderGates();
    },2500);
    const ack=presentedTarget?.binding===job.binding?{presented_through_revision:presentedTarget.revision}:{};
    await dispatchDisplay('preview_target',{...job.extra,...job.identity,...ack},job.auditRequest,job);job.resolve();
    if(targetFlights.includes(job)&&!Number.isInteger(job.revision))retireTarget(job,'目标请求未获有效修订');
    void dispatchTarget();renderMedia();kickTargetMedia();
  }
  function sendDisplay(action,extra={},auditRequest=null){
    if(action!=='preview_target'){cancelTarget('显示视角、模式或服务正在改变');return dispatchDisplay(action,extra,auditRequest);}
    if(audit?.active&&!auditRequest)finishAudit('手动更改显示');
    previewErrors.delete('target');
    if(targetPending){const row=targetPending.auditRequest?.row;if(row){row.cancelled='较新输入合并，未发送';row.coalesced=true;}targetPending.resolve();}
    return new Promise(resolve=>{targetPending={extra,auditRequest,resolve,binding:targetBinding(),identity:{display_context:state?.preview?.display_context,renderer_id:state?.preview?.renderer_id,view_revision:state?.preview?.revisions?.overview}};void dispatchTarget();renderMedia();});
  }
  async function dispatchDisplay(action,extra={},auditRequest=null,targetJob=null){
    const allowed=permission(action),view={preview_target:'target',preview_overview:'overview',preview_mesh:'mesh',preview_recover:'recovery'}[action];
    if(!allowed.allowed){if(auditRequest)auditRequest.row.error=String(allowed.reason).slice(0,500);notice(allowed.reason);return;}
    if(displayRequests.has(view)||displayRequests.has('recovery')){if(auditRequest)auditRequest.row.cancelled='只读请求通道忙碌，未发送';return;}
    if(action==='preview_recover'&&state?.preview?.recovery?.allowed!==true){notice(state?.preview?.recovery?.reason||'当前后端未允许恢复渲染服务');return;}
    if(audit?.active&&!auditRequest)finishAudit('手动更改显示');
    const requestContext=key(),requestInstance=serverInstance,displayBinding=overviewBinding(),controller=new AbortController(),sequence=++requestSequence;
    if(targetJob)targetJob.controller=controller;
    if(auditRequest)auditRequest.row.submitted_ms=clock.now()-auditRequest.window.started_ms;
    displayedRequest=sequence;displayRequests.set(view,controller);previewErrors.delete(view);
    const affected=view==='recovery'?['target','overview','mesh']:[view];
    for(const name of affected){const slot=mediaSlots.get(name==='mesh'?'mesh':'preview');if(slot&&name!=='target'){if(view==='overview')cancelMediaRequest(slot);else clearSlot(slot);}if(view!=='recovery'&&name!=='target')previewFloors.set(name,(state.preview?.revisions?.[name]??0)+1);}
    renderMedia();renderGates();notice(view==='recovery'?'已请求恢复本会话的渲染服务；机器人控制保持独立。':'正在提交只读显示请求，不执行机器人动作。',true);
    try{
      const response=await request('/api/actions/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':state.csrf},
        body:JSON.stringify({environment:context.environment,business:context.business,request_id:newRequestId(crypto),...extra}),signal:controller.signal});
      const result=await response.json();if(!response.ok)throw Error(result.error||result.message||'显示请求被拒绝');
      if(requestContext!==key()||requestInstance!==serverInstance||controller.signal.aborted)return;
      if(targetJob&&(!targetFlights.includes(targetJob)||targetJob.binding!==targetBinding()||result.display_context&&result.display_context!==state.preview?.display_context||result.renderer_id&&result.renderer_id!==state.preview?.renderer_id||result.view_revision!==undefined&&result.view_revision!==targetJob.identity.view_revision))throw Error('目标响应属于已变化的显示上下文');
      if(view==='recovery'){previewFloors.clear();previewErrors.clear();}
      else{if(!Number.isInteger(result.client_revision))throw Error('后端未返回可核验的预览修订');previewFloors.set(view,result.client_revision);}
      if(view==='overview'){
        if(displayBinding!==overviewBinding())throw Error('观察视角响应属于已变化的显示服务');
        const flight={revision:result.client_revision,binding:displayBinding};overviewFlight=flight;
        flight.timer=setTimeout(()=>{if(overviewFlight===flight){overviewFlight=null;previewErrors.set('overview','视角画面更新超时');if(overviewDirty)void sendOverview();renderMedia();}},2500);
      }
      if(targetJob)targetJob.revision=result.client_revision;
      if(auditRequest&&audit===auditRequest.session&&audit.active){
        const {window,row}=auditRequest;row.revision=result.client_revision;row.acknowledged_ms=clock.now()-window.started_ms;window.byRevision.set(row.revision,row);
        const painted=window.painted.get(row.revision);if(painted&&painted.presented_ms>=row.input_ms){painted.input_id=row.input_id;row.presented_ms=painted.presented_ms;row.frame_id=painted.frame_id;}
      }
      if(displayedRequest===sequence)notice(result.message||'只读显示请求已受理，等待对应修订的画面。');
    }catch(error){if(auditRequest&&!auditRequest.row.cancelled)auditRequest.row.error=String(error.message).slice(0,500);if(requestContext===key()&&requestInstance===serverInstance&&!controller.signal.aborted){previewErrors.set(view,error.message);if(displayedRequest===sequence)notice('只读显示请求失败：'+error.message);}}
    finally{if(displayRequests.get(view)===controller)displayRequests.delete(view);if(requestContext===key()&&requestInstance===serverInstance){if(view!=='target')void poll();renderMedia();renderGates();}}
  }
  function showTarget(q12,jaw_gap_mm){
    if(!Array.isArray(q12)||q12.length!==12||!q12.every(Number.isFinite)||!Array.isArray(jaw_gap_mm)||jaw_gap_mm.length!==2||!jaw_gap_mm.every((value,i)=>validJawDraft(value,i))){notice('目标图像需要12个有效关节位置及当前设备范围内的夹爪开度');return;}
    previewView='target';
    if(nativeViewer&&nativeKey===nativeBinding()){nativeViewer.setPose(q12,jaw_gap_mm);nativeLabel();return Promise.resolve();}
    if(nativeLoading){nativePendingPose={q12:[...q12],jaw_gap_mm:[...jaw_gap_mm]};notice('目标模型正在加载，完成后将显示此姿态；不执行动作。');return Promise.resolve();}
    return sendDisplay('preview_target',{q12,jaw_gap_mm});
  }
  async function sendOverview(){
    if(nativeViewer&&nativeKey===nativeBinding()){cancelOverview();nativeViewer.setView(overview);el('media-preview').classList.toggle('split',overview.top_view===true);nativeLabel();return;}
    if(nativeLoading){notice('目标模型正在加载，完成后将应用此观察视角。');return;}
    if(previewView!=='overview'){cancelTarget('观察视角变化');previewView='overview';renderMedia();renderGates();}
    overviewDirty=true;if(overviewSending||overviewFlight||stopped||doc.hidden)return;overviewSending=true;const requestInstance=serverInstance;
    try{overviewDirty=false;await sendDisplay('preview_overview',{...overview});}
    finally{if(requestInstance===serverInstance)overviewSending=false;}
  }
  function setOverviewPreset(){
    const front=nativeViewer?0:-Math.PI/2;
    Object.assign(overview,({front:{yaw:front,pitch:.4},left:{yaw:front+.7,pitch:.45},right:{yaw:front-.7,pitch:.45},top:{yaw:front,pitch:1.5}}[el('overview-preset').value]||{yaw:front,pitch:.4}),{zoom:1,pan_y:0});
    el('overview-zoom').value=1;void sendOverview();
  }
  function sendMesh(){
    if(!state?.preview)return send('mesh',{enabled:el('mesh').checked});
    if(el('mesh').checked&&!meshCandidate){notice('没有与当前版本关联的标定结果，请先选择匹配的头部／双腕标定；叠加开启设置已保留。');el('mesh').checked=state.preview.mesh?.requested===true;renderGates();return;}
    return sendDisplay('preview_mesh',{enabled:el('mesh').checked,camera:meshCamera,candidate_id:meshCandidate||null});
  }
  async function reconnectImages(){
    const requestInstance=serverInstance;clearMedia();await poll();if(requestInstance!==serverInstance)return;renderMedia();notice(online?'已重新打开浏览器画面订阅；连接与源帧状态请查看图像诊断。':'状态连接仍未恢复，请按图像诊断中的后端指引检查。');
  }
  async function send(action,extra={},settingsEditor=null) {
    if(inferenceParameterUpdate&&settingsEditor!=='parameter-renewal'&&['stop','cancel','handoff','release','inference_hold','inference_teach','inference_home'].includes(action))inferenceParameterUpdate=null;
    if(action==='stop'&&environmentAccess?.state?.target?.device_id&&environmentAccess.state.csrf){dismissManualExecution();dismissInferenceRun();try{await environmentAccess.stop();notice('独立停止请求已返回；实际状态以停止与保持回执为准。');}catch(error){notice('停止结果未知：'+error.message+'；请核对现场和独立停止设备。');}return;}

    const allowed=action==='handoff'?handoffPermission(state,extra.target_business,options()):permission(action);
    if(!allowed.allowed){notice(allowed.reason);renderGates();return;}
    finishAudit('页面操作：'+action);
    if(action==='stop'&&[...pending.values()].some(p=>p.action==='stop'&&p.context===key()))return;
    let requestId;try{requestId=newRequestId(crypto);}catch(error){notice(error.message);return;}
    const body={environment:context.environment,business:context.business,request_id:requestId,...extra};
    if(context.business==='inference'&&['inference_prepare','inference_preview','execute','inference_run'].includes(action))try{if(body.seed===undefined)body.seed=inferenceSeed();if(body.duration_s===undefined)body.duration_s=inferenceDuration();}catch(error){notice(error.message);return;}
    if(action==='return_cup'||action==='execute'&&activeReturnTask())try{body.offset_xy_mm=returnOffsets();}catch(error){notice(error.message);return;}
    if(action==='preview_batch'&&body.queue_revision===undefined)body.queue_revision=state?.workflow?.calibration_tools?.queue_revision;
    if(action==='fit_batch'&&body.batch_id===undefined)body.batch_id=state?.workflow?.calibration_tools?.fit_source_id;
    if(action==='delete_confirm'){body.delete_preview_id=state.workflow.deletion_preview.id;body.confirmed=el('delete-clearance').checked;confirmedDeletions.add(key()+'/'+body.delete_preview_id);}
    if(['execute','manual_execute'].includes(action)){body.review_id=state.workflow.review.id;body.physical_clearance=context.environment==='physical'||state.workflow.review.physical_clearance_required?options().clearance:true;}
    if(context.business==='collection'&&['generate_plan','collection_plan','execute'].includes(action))body.speed_scale=action==='execute'&&activeReturnTask()?(state.workflow.review?.speed_scale??returnTask().parent_speed_scale):Number(el('collection-speed').value)/100;
    const csrf=state.csrf,sequence=++requestSequence,requestContext=key(),requestInstance=serverInstance,controller=new AbortController();
    if(state.workflow?.review?.id&&!(action==='collection_prepare'&&state.workflow.collection?.warmup?.status==='ready')&&['stop','cancel','handoff','release','execution_settings','control_policy','generate_plan','collection_prepare','collection_plan','inference_select','inference_prepare','inference_preview','inference_run','return_cup','manual_preview','manual_queue','manual_queue_preview','manual_execute','execute','recover','install',...CALIBRATION_TOOLS].includes(action))
      invalidatedReviews.add(key()+'/'+state.workflow.review.id);
    barrier();const at=generation;displayedRequest=sequence;pending.set(sequence,{action,context:requestContext,controller});
    if(['manual_preview','manual_queue_preview','return_cup'].includes(action))manualIntent={action,requestId,jobId:null,reviewId:null,context:requestContext,binding:manualBinding(),
      queueRevision:action==='manual_queue_preview'?body.queue_revision:undefined,
      offsets:action==='return_cup'?body.offset_xy_mm:undefined,
      jaws:action==='return_cup'?[]:JSON.parse(JSON.stringify(action==='manual_preview'?[body.jaw_gap_mm]:state.manual.queue.waypoints.map(row=>row.jaw_gap_mm)))};
    notice(action==='stop'?'正在请求停止；等待后端停止回执。':'请求已发送，等待后端受理；尚未确认完成。',true);renderGates();
    try {
      const response=await request('/api/actions/'+encodeURIComponent(action),{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf},body:JSON.stringify(body),signal:controller.signal});
      const result=await response.json();if(!response.ok)throw Error(result.message||result.error||result.reason||'请求被拒绝');
      if(action==='execution_settings'&&requestContext===key()&&requestInstance===serverInstance&&!controller.signal.aborted){
        if(settingsEditor==='speed_protection'){speedProtectionDirty=false;speedProtectionRevision=null;}
        else {executionSettingsDirty=false;executionSettingsRevision=null;}
      }
      if(action==='inference_execution_settings'&&requestContext===key()&&requestInstance===serverInstance&&!controller.signal.aborted){inferenceStepsDirty=false;inferenceStepsRevision=null;}
      if(action==='control_policy'&&requestContext===key()&&requestInstance===serverInstance&&!controller.signal.aborted)fillControlPolicy(result.control_policy);
      if(action==='precision_settings'&&requestContext===key()&&requestInstance===serverInstance&&!controller.signal.aborted){precisionSettingsDirty=false;precisionSettingsRevision=null;}
      if(manualIntent?.requestId===requestId){if(typeof result.job_id==='string')manualIntent.jobId=result.job_id;else dismissManualExecution();}
      if(requestContext===key()&&requestInstance===serverInstance&&sequence===displayedRequest&&!controller.signal.aborted)notice(result.message||(action==='stop'?'停止请求已受理；以停止回执为准。':'请求已受理；以任务回执为准。'));
      if(requestContext===key()&&requestInstance===serverInstance&&!controller.signal.aborted)return result;
    } catch(error) {if(manualIntent?.requestId===requestId)dismissManualExecution();if(requestContext===key()&&requestInstance===serverInstance&&sequence===displayedRequest&&!controller.signal.aborted)notice('请求结果未确认：'+error.message+'；不会自动重试，请核对状态。');}
    finally {pending.delete(sequence);if(requestContext===key()&&requestInstance===serverInstance){renderGates();if(at===generation)void poll();}}
  }
  function prepareEnvironment(action,parameters){
    if(environmentAccess?.permission(action).allowed!==true){notice(environmentAccess?.permission(action).reason||'统一运维服务尚未核验');return;}
    if(state?.workflow?.review?.id&&!(action==='prepare_collection'&&state.workflow.collection?.warmup?.status==='ready')){
      invalidatedReviews.add(key()+'/'+state.workflow.review.id);clearReview();
    }
    dismissInferenceRun();renderGates();return environmentAccess.prepare(action,parameters);
  }
  function switchContext(next) {
    void operatorFocus(false);
    if(next.environment===context.environment&&next.business===context.business)return;
    el('handoff-target').value=next.business;
    saveDraft();meshChoices.clear();observationCamera='all';
    pausePair();
    finishAudit('设备环境或业务变化');
    barrier();for(const p of pending.values())p.controller.abort();pending.clear();clearMedia();
    for(const controller of displayRequests.values())controller.abort();displayRequests.clear();previewFloors.clear();previewErrors.clear();
    previewView='target';meshCamera='head';meshCandidate='';meshAnchor='';meshStateKey='';meshOptionsKey='';overviewDirty=false;overviewPointer=null;
    if(next.environment!==context.environment){
      environmentIdentities.set(context.environment,{serverInstance,serverRoute,retiredServerInstances});
      const identity=environmentIdentities.get(next.environment)||{};
      serverInstance=identity.serverInstance;serverRoute=identity.serverRoute??null;retiredServerInstances=identity.retiredServerInstances||new Set();
    }
    environmentAccess?.setEnvironment(next.environment);
    el('operations-panel').querySelector('[data-workbench-operations]')?.workbenchOperations?.setEnvironment(next.environment);
    context=next;state=null;revision=-1;online=false;reviewKey='';manualKey='';manualQueueKey='';replayKey='';replayControlsKey='';
    resultsCursor=null;resultsBrowsing=false;
    notice('正在核验'+ENVIRONMENT[context.environment]+' / '+BUSINESS[context.business]+'；浏览不会交接控制权。',false,true);render();void poll();
  }
  function openOperations(target='operations-panel'){
    selectTab('operations');
    const node=el(target);node?.scrollIntoView?.({block:'start'});node?.focus?.({preventScroll:true});
  }
  function selectTab(next) {
    if(next!=='inference')void operatorFocus(false);
    if(next==='control-research')next='research';
    if(next==='diagnostics'){openOperations('diagnostics-pane');return;}
    if(next!==tab){dismissManualExecution();dismissInferenceRun();}
    if(next!=='results')pausePair();
    if(next!==context.business)finishAudit('目标图像离开当前可见工作区');
    tab=next;for(const button of doc.querySelectorAll('[data-tab]')){if(button.dataset.tab===next)button.setAttribute('aria-current','page');else button.removeAttribute('aria-current');}
    el('work-pane').hidden=!BUSINESS[next];el('results-pane').hidden=next!=='results';
    el('operations-panel').hidden=next!=='operations';
    el('execution-settings-tools').hidden=!BUSINESS[next]&&next!=='research';
    el('speed-protection-tools').hidden=!BUSINESS[next]&&next!=='research';
    el('precision-study-tools').hidden=next!=='research';
    el('inference-execution-settings').hidden=next!=='inference';
    el('context-guidance').hidden=['operations','research'].includes(next);
    el('request-status').hidden=next==='operations';
    if(BUSINESS[next])switchContext({...context,business:next});
  }
  function manualPreview(queueAdd=false) {
    try {
      if(!feedbackReady())throw Error('关节定义或健康实测反馈尚未完整');
      const {q12,jaw_gap_mm}=displayPose();
      const speedInputs=jointInputs('speed'),speeds12=speedInputs.map(input=>Number(input.value)),jaw_speed_mm_s=Number(el('jaw-speed').value);
      if(speeds12.length!==12||speedInputs.some((input,i)=>input.value===''||!Number.isFinite(speeds12[i])||speeds12[i]<=0)||!el('jaw-speed').value||!Number.isFinite(jaw_speed_mm_s)||jaw_speed_mm_s<=0)throw Error('关节与夹爪速度必须为有限正数');
      el('manual-error').hidden=true;
      if(queueAdd)void send('manual_queue',{operation:'add',queue_revision:state.manual.queue?.revision,waypoint:{q:q12,speeds:speeds12,jaw_gap_mm,jaw_speed_mm_s}});
      else void send('manual_preview',{q12,speeds12,jaw_gap_mm,jaw_speed_mm_s});
    } catch(error){setText('manual-error',error.message);el('manual-error').hidden=false;}
  }
  function start() {
    el('environment').value=context.environment;
    el('handoff-target').value=context.business;
    selectTab(query.get('view')==='operations'?'operations':context.business);
    el('context-guidance-next').addEventListener('click',event=>{
      const id=el('context-guidance-next').dataset.target;
      if(!id)return;
      if(id==='operations-link'){event.preventDefault();openOperations();return;}
      event.preventDefault();
      const target=doc.getElementById(id);if(!target)return;
      if(id==='diagnostics-pane')selectTab('diagnostics');
      else if(tab!==context.business)selectTab(context.business);
      for(let node=target;node;node=node.parentElement)if(node.tagName?.toLowerCase()==='details')node.open=true;
      target.scrollIntoView?.({block:'center'});target.focus?.({preventScroll:true});
    });
    for(const id of ['preview-recover'])el(id).addEventListener('click',event=>{event.preventDefault();openOperations();});
    sessionMonitor?.start();environmentAccess?.subscribe(()=>{if(!stopped){renderInference();renderGates();renderGuidance();}});
    for(const id of ['arrival-observe-only','tracking-observe-only',...Object.keys(executionFieldMap),'fake-error-enabled','fake-error-max','fake-error-seed'])el(id).addEventListener('input',()=>{executionSettingsDirty=true;renderExecutionSettings();});
    el('execution-settings-reset').addEventListener('click',()=>renderExecutionSettings(true));
    el('execution-settings-apply').addEventListener('click',()=>{if(el('execution-settings-apply').disabled)return;try{void send('execution_settings',{settings:executionSettingsPayload(),expected_revision:executionSettingsRevision},'execution_errors');}catch(error){notice(error.message);}});
    for(const id of ['speed-protection-enabled','speed-protection-limit'])el(id).addEventListener('input',()=>{speedProtectionDirty=true;renderSpeedProtection();});
    el('control-policy-load').addEventListener('click',()=>void loadControlPolicy());
    el('control-policy-save').addEventListener('click',()=>{if(el('control-policy-save').disabled)return;try{void send('control_policy',{expected_revision:policySnapshot.revision,changes:controlPolicyChanges()});}catch(error){notice(error.message);}});
    el('speed-protection-reset').addEventListener('click',()=>renderSpeedProtection(true));
    el('speed-protection-apply').addEventListener('click',()=>{if(el('speed-protection-apply').disabled)return;try{void send('execution_settings',{settings:speedProtectionPayload(),expected_revision:speedProtectionRevision},'speed_protection');}catch(error){notice(error.message);}});
    for(const [id,kind,speed]of [['precision-start','start',.5],['precision-05','trajectory',.5],['precision-10','trajectory',1],['precision-15','trajectory',1.5],['precision-20','trajectory',2],['precision-home','home',.5]])
      el(id).addEventListener('click',()=>{if(!el(id).disabled)void send('precision_preview',{kind,speed:kind==='trajectory'?speed:(state?.precision_study?.runtime_facts?.trajectory_speed_choices?.[0]??speed)});});
    el('precision-execute').addEventListener('click',()=>{if(!el('precision-execute').disabled)void send('precision_execute',{review_id:state?.precision_study?.review?.id});});
    el('precision-stop').addEventListener('click',()=>{if(!el('precision-stop').disabled)void send('stop');});
    el('precision-analyze').addEventListener('click',()=>{if(!el('precision-analyze').disabled)void send('precision_analyze');});
    el('precision-refresh').addEventListener('click',()=>void poll());
    for(const id of [...Object.keys(precisionNumeric),'precision-integral'])el(id).addEventListener('input',()=>{precisionSettingsDirty=true;renderPrecisionStudy();});
    el('precision-settings-reset').addEventListener('click',()=>{precisionSettingsDirty=false;precisionSettingsRevision=null;renderPrecisionStudy();});
    el('precision-config').addEventListener('change',()=>{
      const preset=state?.precision_study?.settings_presets?.find(row=>row.id===el('precision-config').value);
      if(preset&&!el('precision-config').disabled){fillPrecisionSettings(preset.settings);precisionSettingsDirty=true;renderPrecisionStudy();}
    });
    el('precision-settings-save').addEventListener('click',()=>{if(!el('precision-settings-save').disabled)try{void send('precision_settings',{settings:precisionSettingsPayload(),expected_revision:precisionSettingsRevision});}catch(error){notice(error.message);}});
    el('precision-select').addEventListener('click',()=>{if(!el('precision-select').disabled)void send('precision_select',{record_id:el('precision-history').value});});
    el('precision-notes').addEventListener('input',()=>{precisionNotesDirty=true;renderPrecisionStudy();});
    el('precision-notes-save').addEventListener('click',()=>{const selected=state?.precision_study?.selected||state?.precision_study?.latest;if(!el('precision-notes-save').disabled&&selected)void send('precision_notes',{record_id:selected.id,text:el('precision-notes').value,expected_revision:selected.notes_revision||0});});
    el('inference-execute-steps').addEventListener('input',()=>{inferenceStepsDirty=true;renderInferenceSteps();});
    el('inference-steps-reset').addEventListener('click',()=>renderInferenceSteps(true));
    el('inference-steps-apply').addEventListener('click',()=>{if(!el('inference-steps-apply').disabled){const settings={execute_steps:Number(el('inference-execute-steps').value),expected_revision:inferenceStepsRevision};void renewPreparedParameters(()=>send('inference_execution_settings',settings,'parameter-renewal'));}});
    el('environment').addEventListener('change',()=>{
      const next=el('environment').value;el('environment').value=context.environment;
      saveDraft();dismissManualExecution();dismissInferenceRun();
      if(environmentAccess?.navigateEnvironment)void environmentAccess.navigateEnvironment(next);
      else notice('请先核验统一运维服务与当前控制连接，再切换环境。');
    });
    doc.addEventListener('physical-endpoint-changed',()=>{
      environmentIdentities.delete('physical');
      if(stopped||context.environment!=='physical')return;
      saveDraft();barrier();for(const job of pending.values())job.controller.abort();pending.clear();clearMedia();
      serverInstance=undefined;serverRoute=null;retiredServerInstances=new Set();state=null;revision=-1;online=false;

      notice('真机服务地址已变化；旧审批已清除，正在核验新绑定。',false,true);render();void poll();
    });
    for(const button of doc.querySelectorAll('[data-tab]'))button.addEventListener('click',()=>button.dataset.tab==='operations'?openOperations():selectTab(button.dataset.tab));
    el('handoff-target').addEventListener('change',renderGates);
    el('handoff').addEventListener('click',()=>{if(!el('handoff').disabled)void send('handoff',{target_business:el('handoff-target').value});});
    el('results-business').addEventListener('change',()=>{const business=el('results-business').value;if(BUSINESS[business])switchContext({...context,business});});
    el('calibration-compare-link').addEventListener('click',()=>{if(context.business!=='calibration')switchContext({...context,business:'calibration'});selectTab('results');});
    el('observation-camera').addEventListener('change',()=>{const value=el('observation-camera').value;if(['all','head','left','right'].includes(value)){observationCamera=value;el('camera-grid').dataset.camera=value;saveDraft();}});
    el('primary').addEventListener('click',()=>{const primary=(state?.workflow?.business_task||state?.workflow)?.primary_action;if(primary&&primary.id!=='manual_execute')void send(primary.id);});
    el('manual-confirm').addEventListener('click',()=>{renderManualExecution();const action=manualIntent?.action==='return_cup'?'execute':'manual_execute';if(el('manual-execution-dialog').open&&manualIntent?.reviewId===state?.workflow?.review?.id&&permission(action).allowed)void send(action);});
    el('manual-dismiss').addEventListener('click',()=>{dismissManualExecution();renderGates();});
    el('manual-execution-dialog').addEventListener('cancel',event=>{event.preventDefault();dismissManualExecution();renderGates();});
    el('manual-execution-dialog').addEventListener('close',()=>{if(!el('manual-execution-dialog').open){dismissManualExecution();renderGates();}});
    el('manual-clearance').addEventListener('change',renderGates);
    el('replan').addEventListener('click',()=>send('generate_plan',{replan:true}));
    el('collection-prepare').addEventListener('click',()=>send('collection_prepare'));
    for(const kind of ['machine','cup']){
      el('scene-asset-'+kind).addEventListener('change',()=>{sceneAssetDraft[kind]=el('scene-asset-'+kind).value;renderSceneAssets();});
      el('scene-asset-'+kind+'-preview').addEventListener('load',()=>{el('scene-asset-'+kind+'-preview').hidden=false;});
      el('scene-asset-'+kind+'-preview').addEventListener('error',()=>{el('scene-asset-'+kind+'-preview').hidden=true;});
    }
    el('scene-asset-apply').addEventListener('click',()=>{
      const catalog=state?.scene_assets,pair=sceneAssetPair(catalog,sceneAssetDraft.machine,sceneAssetDraft.cup);
      if(!el('scene-asset-apply').disabled&&pair)void prepareEnvironment('prepare_collection',
        {asset_pair_id:pair.id,expected_catalog_digest:catalog.catalog_digest});
    });
    el('collection-plan').addEventListener('click',()=>send('collection_plan'));
    el('inference-prepare').addEventListener('click',()=>prepareEnvironment('prepare_model',{model_id:inferenceModelId(),expected_bundle_digest:inferenceDigest()}));
    el('inference-preview').addEventListener('click',()=>send('inference_preview'));
    el('inference-refresh').addEventListener('click',()=>send('inference_refresh'));
    el('inference-checkpoint').addEventListener('change',async()=>{
      if(el('inference-checkpoint').disabled)return;
      dismissInferenceRun();const choice=el('inference-checkpoint').value;
      const row=inferenceModes(inferenceInfo().models,inferenceBackend(),inferenceFamilySelection(),inferenceDenoiseValue()).find(row=>inferenceChoice(row)===choice);
      await prepareInferenceRow(row);
    });
    el('inference-denoise-steps').addEventListener('input',()=>{inferenceDenoiseDirty=true;dismissInferenceRun();clearReview();inferenceCatalogKey='';renderInference();renderGates();});
    el('inference-denoise-apply').addEventListener('click',()=>{if(!el('inference-denoise-apply').disabled){const row=inferenceDenoiseRow();void renewPreparedParameters(()=>prepareInferenceRow(row),{model:row});}});
    el('inference-backend').addEventListener('change',()=>{dismissInferenceRun();clearReview();inferenceSelection=null;inferenceCatalogKey='';renderInference();renderGates();});
    el('inference-family').addEventListener('change',()=>{setInferenceBackends(inferenceBackend());dismissInferenceRun();clearReview();inferenceSelection=null;inferenceCatalogKey='';renderInference();renderGates();});
    el('inference-run').addEventListener('pointerdown',event=>{
      // Keep the duration input focused until this explicit Run intent can
      // join parameter preparation; blur must not consume the first click.
      if(event.button===0&&doc.activeElement===el('inference-duration')&&preparedParameterHold())event.preventDefault();
    });
    el('inference-run').addEventListener('click',()=>void requestInferenceRun());
    el('inference-arm').addEventListener('click',()=>openInferenceRun(true));
    el('inference-disarm').addEventListener('click',()=>{if(!el('inference-disarm').disabled)void send('cancel');});
    const operatorContext=()=>({runtime_nonce:state?.runtime?.runtime_nonce,epoch:state?.runtime?.command_epoch});
    el('inference-hold').addEventListener('click',()=>{if(!el('inference-hold').disabled)void send('inference_hold',operatorContext());});
    el('inference-teach').addEventListener('click',()=>{if(!el('inference-teach').disabled)void send('inference_teach',operatorContext());});
    el('inference-home').addEventListener('click',()=>{if(!el('inference-home').disabled){operatorHomeIdentity=operatorIdentity();operatorHomeContext=operatorContext();el('inference-home-clearance').checked=false;el('inference-home-dialog').showModal();renderGates();}});
    el('inference-home-clearance').addEventListener('change',renderGates);
    el('inference-home-cancel').addEventListener('click',()=>el('inference-home-dialog').close());
    el('inference-home-confirm').addEventListener('click',()=>{if(!el('inference-home-confirm').disabled&&operatorHomeIdentity===operatorIdentity()){el('inference-home-dialog').close();void send('inference_home',{...operatorHomeContext,physical_clearance:true});}});
    el('inference-run-clearance').addEventListener('change',renderGates);
    el('inference-run-confirm').addEventListener('click',()=>{
      renderInferenceRun();if(!inferenceRunIntent||el('inference-run-confirm').disabled)return;
      const {binding,...body}=inferenceRunIntent;
      void send('inference_run',{...body,state_wall_ns:state.server_wall_ns,physical_clearance:true,scene_reset:true});
    });
    el('inference-run-cancel').addEventListener('click',()=>{dismissInferenceRun();renderGates();});
    el('inference-run-dialog').addEventListener('cancel',event=>{event.preventDefault();dismissInferenceRun();renderGates();});
    el('inference-run-dialog').addEventListener('close',()=>{if(!el('inference-run-dialog').open)dismissInferenceRun();});
    el('inference-results').addEventListener('click',()=>selectTab('results'));
    el('inference-seed').addEventListener('input',()=>{dismissInferenceRun();if(context.business==='inference'&&state?.workflow?.review?.id)invalidatedReviews.add(key()+'/'+state.workflow.review.id);clearReview();renderGates();});
    el('inference-duration').addEventListener('change',()=>{if(inferenceRunIntent?.duration_s!==Number(el('inference-duration').value))dismissInferenceRun();if(context.business==='inference'&&state?.workflow?.review?.id)invalidatedReviews.add(key()+'/'+state.workflow.review.id);clearReview();renderGates();if(preparedParameterHold()&&Number(el('inference-duration').value)!==inferenceInfo().operator_session.duration_s)void renewPreparedParameters();});
    el('inference-stall-enabled').addEventListener('change',()=>{dismissInferenceRun();clearReview();renderGates();});
    el('return-cup').addEventListener('click',()=>send('return_cup'));
    for(const id of ['return-offset-x','return-offset-y'])el(id).addEventListener('input',()=>{
      if(manualIntent?.action==='return_cup')dismissManualExecution();
      if(activeReturnTask()&&state?.workflow?.review?.id)invalidatedReviews.add(key()+'/'+state.workflow.review.id);
      clearReview();renderGates();
    });
    el('collection-execute').addEventListener('click',()=>send('execute'));
    el('review-speed').addEventListener('click',()=>{if(!activeReturnTask())return send('generate_plan');});
    el('collection-speed').addEventListener('input',()=>{
      if(activeReturnTask())return;
      setText('collection-speed-value',el('collection-speed').value+'%');
      if(state?.workflow?.review?.id)invalidatedReviews.add(key()+'/'+state.workflow.review.id);
      saveDraft();clearReview();renderGates();
    });
    el('trajectory-load').addEventListener('click',loadTrajectory);
    el('trajectory-tick').addEventListener('input',renderTrajectoryTick);
    el('trajectory-stage').addEventListener('change',()=>{el('trajectory-tick').value=el('trajectory-stage').value;renderTrajectoryTick();});
    el('trajectory-arm').addEventListener('change',renderTrajectoryPlots);
    el('collection-replay-camera').addEventListener('change',selectPair);
    el('collection-history-run').addEventListener('change',()=>{pausePair();historySelections.set(historyKey(),el('collection-history-run').value);renderResults();renderGates();});
    el('inference-history-older').addEventListener('click',()=>browseInferenceHistory('before'));
    el('inference-history-newer').addEventListener('click',()=>browseInferenceHistory('after'));
    el('inference-history-latest').addEventListener('click',()=>browseInferenceHistory(null));
    el('recording-files').addEventListener('toggle',()=>{if(el('recording-files').open)void loadRecordingFiles();});
    el('recording-files-refresh').addEventListener('click',()=>void loadRecordingFiles());
    el('recording-files-select').addEventListener('click',()=>{recordingSelection.clear();for(const row of recordingFiles.filter(row=>row.deletable).slice(0,100))recordingSelection.add(row.run_id);renderRecordingFiles();});
    el('recording-files-delete').addEventListener('click',reviewRecordingDeletion);
    el('recording-files-confirm-delete').addEventListener('click',()=>void deleteRecordingFiles());
    el('recording-files-confirm').addEventListener('cancel',()=>{recordingDeleteIntent=null;});
    el('recording-files-cancel').addEventListener('click',()=>{recordingDeleteIntent=null;el('recording-files-confirm').close();});
    el('collection-replay-toggle').addEventListener('click',playPair);
    el('collection-replay-time').addEventListener('input',()=>seekPair(Number(el('collection-replay-time').value)||0));
    el('collection-replay-speed').addEventListener('change',()=>{const value=Number(el('collection-replay-speed').value);if(![.25,.5,1,2].includes(value))return;pairedPosition=pairTime();pairedStarted=clock.now();pairedRate=value;for(const node of pair())node.playbackRate=value;});
    for(const id of ['calibration-kind','calibration-head','fit-source'])el(id).addEventListener('change',renderGates);
    el('configure-calibration').addEventListener('click',()=>send('configure_calibration',{kind:el('calibration-kind').value,head_result_id:el('calibration-kind').value==='wrist'?el('calibration-head').value:null}));
    el('wrist-redo').addEventListener('click',()=>send('generate_plan',{mode:'wrist',head_result_id:el('calibration-head').value}));
    el('plan-batch').addEventListener('click',()=>send('plan_batch'));
    el('calibration-plan-prev').addEventListener('click',()=>browsePlan(planIndex-1));
    el('calibration-plan-next').addEventListener('click',()=>browsePlan(planIndex+1));
    el('calibration-plan-select').addEventListener('change',()=>browsePlan(el('calibration-plan-select').value));
    el('preview-batch').addEventListener('click',()=>send('preview_batch',{queue_revision:state?.workflow?.calibration_tools?.queue_revision}));
    el('apply-fit-source').addEventListener('click',()=>send('fit_source',{batch_id:el('fit-source').value}));
    el('clear-fit-source').addEventListener('click',()=>send('fit_source',{batch_id:null}));
    el('fit-batch').addEventListener('click',()=>send('fit_batch'));
    const openReplay=()=>{
      const view=state?.workflow?.replay_context,index=Number(el('replay-index').value);
      if(!view||el('replay-index').value===''||!Number.isInteger(index)||index<0||index>=view.frame_count){notice('请选择当前批次内的有效帧');return;}
      const candidate=el('replay-candidate').value;
      return send('select_result',{result_id:view.result_id,frame_index:index,camera:el('replay-camera').value,overlay:el('replay-overlay').checked,...(candidate?{candidate_id:candidate}:{})});
    };
    el('replay-open').addEventListener('click',openReplay);
    el('replay-index').addEventListener('change',openReplay);
    el('replay-candidate').addEventListener('change',openReplay);
    el('replay-overlay').addEventListener('change',openReplay);
    el('replay-camera').addEventListener('change',()=>{el('replay-overlay').checked=false;el('replay-candidate').value='';renderGates();return openReplay();});
    for(const action of ['stop','cancel'])el(action).addEventListener('click',()=>send(action));
    el('control-fault-diagnostics').addEventListener('click',()=>selectTab('diagnostics'));
    el('show-target').addEventListener('click',()=>{
      try{const {q12,jaw_gap_mm}=displayPose();return showTarget(q12,jaw_gap_mm);}
      catch(error){notice(error.message);}
    });
    el('overview-preset').addEventListener('change',setOverviewPreset);
    el('overview-reset').addEventListener('click',()=>{el('overview-preset').value='front';setOverviewPreset();});
    el('overview-zoom').addEventListener('input',()=>{overview.zoom=Math.max(.4,Math.min(4,Number(el('overview-zoom').value)));void sendOverview();});
    const overviewImage=el('media-preview').querySelector('img');
    overviewImage.draggable=false;
    overviewImage.addEventListener('dragstart',event=>event.preventDefault());
    el('preview-top-view').addEventListener('change',()=>{overview.top_view=el('preview-top-view').checked;renderPreview();void sendOverview();});
    overviewImage.addEventListener('pointerdown',event=>{if(!permission('preview_overview').allowed)return;event.preventDefault();overviewPointer={x:event.clientX,y:event.clientY};overviewImage.setPointerCapture?.(event.pointerId);});
    overviewImage.addEventListener('pointermove',event=>{if(!overviewPointer)return;overview.yaw=((overview.yaw-(event.clientX-overviewPointer.x)*.006+3*Math.PI)%(2*Math.PI))-Math.PI;overview.pan_y=Math.max(-2,Math.min(2,overview.pan_y+(event.clientY-overviewPointer.y)/(overviewImage.clientHeight||240)));overviewPointer={x:event.clientX,y:event.clientY};void sendOverview();});
    for(const event of ['pointerup','pointercancel'])overviewImage.addEventListener(event,()=>{overviewPointer=null;});
    overviewImage.addEventListener('wheel',event=>{if(!permission('preview_overview').allowed)return;event.preventDefault();overview.zoom=Math.max(.4,Math.min(4,overview.zoom*Math.exp(-event.deltaY*.001)));el('overview-zoom').value=overview.zoom;void sendOverview();},{passive:false});
    el('mesh-camera').addEventListener('change',()=>{const enabled=el('mesh').checked,current=meshCandidate||meshAnchor;meshCamera=el('mesh-camera').value;if(!Array.isArray(state?.preview?.mesh_calibrations)){meshCandidate=matchingMeshCandidate(meshRows(state),meshCamera,current,meshChoices.get(meshCamera));meshAnchor=meshCandidate||current;if(meshCandidate)meshChoices.set(meshCamera,meshCandidate);}renderPreview();el('mesh').checked=enabled;saveDraft();renderMedia();void sendMesh();});
    el('mesh-candidate').addEventListener('change',()=>{meshCandidate=el('mesh-candidate').value;meshAnchor=meshCandidate;if(!Array.isArray(state?.preview?.mesh_calibrations))meshChoices.set(meshCamera,meshCandidate);saveDraft();renderMedia();void sendMesh();});
    el('mesh').addEventListener('change',sendMesh);
    el('reconnect-images').addEventListener('click',reconnectImages);
    el('audit-start').addEventListener('click',startAudit);
    el('audit-stop').addEventListener('click',()=>{finishAudit('用户结束诊断');renderGates();});
    el('audit-export').addEventListener('click',exportAudit);
    el('delete-all').addEventListener('click',()=>send('delete_preview',{scope:'all'}));
    el('delete-clearance').addEventListener('change',renderGates);
    el('delete-confirm').addEventListener('click',()=>send('delete_confirm'));
    el('delete-cancel').addEventListener('click',()=>send('delete_cancel'));
    for(const id of ['clearance'])el(id).addEventListener('change',renderGates);
    for(const id of ['jaw-left','jaw-right','jaw-speed'])el(id).addEventListener('input',editManual);
    el('manual-preview').addEventListener('click',()=>manualPreview());
    el('queue-add').addEventListener('click',()=>manualPreview(true));
    el('queue-preview').addEventListener('click',()=>send('manual_queue_preview',{queue_revision:state?.manual?.queue?.revision}));
    el('queue-clear').addEventListener('click',()=>send('manual_queue',{operation:'clear',queue_revision:state?.manual?.queue?.revision}));
    el('apply-manual-speed').addEventListener('click',()=>{const speed=Number(el('manual-speed').value);if(!Number.isFinite(speed)||speed<=0){notice('统一关节速度必须为正数');return;}for(const input of jointInputs('speed'))input.value=speed;editManual();});
    el('use-feedback').addEventListener('click',()=>{if(!feedbackReady())return;fillFeedbackInputs();editManual();});
    el('zero-preview').addEventListener('click',()=>{for(const input of jointInputs('position'))input.value=0;syncJointSliders();el('jaw-left').value=jawRange(0)[1];el('jaw-right').value=jawRange(1)[1];editManual();});
    doc.addEventListener('visibilitychange',()=>{void operatorFocus(!doc.hidden&&tab==='inference');if(doc.hidden){finishAudit('页面进入后台');cancelTarget('页面进入后台');cancelOverview();clearMedia();}else{renderMedia();void poll();}});
    render();void poll();timers.push(setInterval(poll,1000),setInterval(()=>{renderAge();renderGates();if(state&&options().fresh===false)render();},250),setInterval(renderPairClock,100));
    timers.push(setInterval(()=>void operatorFocus(),2000));
  }
  function destroy(){void operatorFocus(false);el('operations-panel').querySelector('[data-workbench-operations]')?.workbenchOperations?.destroy();nativeLoading?.abort();nativeViewer?.destroy();nativeViewer=null;environmentAccess?.destroy();sessionMonitor?.destroy();saveDraft();pausePair();trajectoryRequest?.abort();finishAudit('页面关闭');if(auditDownload)urls.revokeObjectURL(auditDownload);stopped=true;barrier();for(const p of pending.values())p.controller.abort();pending.clear();for(const controller of displayRequests.values())controller.abort();displayRequests.clear();overviewDirty=false;clearMedia();timers.forEach(clearInterval);}
  return {start,poll,send,switchContext,selectTab,destroy,get state(){return state;},get context(){return {...context};}};
}

if(typeof window!=='undefined'&&typeof document!=='undefined'){
  let storage;try{storage=window.localStorage;}catch{}
  const environmentAccess=mountPhysicalEndpoint({document,fetch:window.fetch.bind(window),location:window.location,crypto:window.crypto});
  const app=createWorkbench({document,fetch:window.fetch.bind(window),location:window.location,performance:window.performance,URL:window.URL,crypto:window.crypto,setInterval:window.setInterval.bind(window),clearInterval:window.clearInterval.bind(window),setTimeout:window.setTimeout.bind(window),clearTimeout:window.clearTimeout.bind(window),requestAnimationFrame:window.requestAnimationFrame.bind(window),cancelAnimationFrame:window.cancelAnimationFrame.bind(window),storage,environmentAccess});
  app.start();window.addEventListener('pagehide',()=>app.destroy());
}
