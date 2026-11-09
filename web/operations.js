// Acknowledging an event never changes the live fault or enables control.
let alertInstance=0;
export function mountControlAlerts({document:doc,container,acknowledge,recover}={}){
  const make=(tag,text)=>{const node=doc.createElement(tag);if(text!==undefined)node.textContent=text;return node;};
  const root=make('section');root.className='control-alerts';root.setAttribute('aria-label','控制急停事件与当前心跳');
  const warnings=make('div'),current=make('p');current.setAttribute('role','status');current.setAttribute('aria-live','polite');
  const recovery=make('div'),recoverButton=make('button','中断恢复…'),recoverReason=make('p');
  recoverButton.id=`workbench-control-recovery-${++alertInstance}`;
  const recoveryLink=()=>{const link=make('a','查看“中断恢复”');link.href='#'+recoverButton.id;
    link.addEventListener('click',()=>{recoverButton.scrollIntoView?.({block:'center'});recoverButton.focus?.();});return link;};
  recoverButton.addEventListener('click',()=>{if(!recoverButton.disabled)recover();});
  recovery.append(recoverButton,make('p','先核对恢复范围与现场条件；不会续跑中断前的动作。'),recoverReason);
  recovery.hidden=true;recoverButton.disabled=true;root.append(warnings,current,recovery);container.append(root);
  let state=null,pending=false,error='';
  function render(){
    const live=state?.current,heartbeat=live?.heartbeat;
    const needsRecovery=live?.recovery_required===true||live?.active_anomaly===true;
    warnings.replaceChildren();
    for(const event of state?.warnings||[]){
      const box=make('section');box.className='control-stop-warning';box.setAttribute('role','alert');
      box.append(make('strong','控制急停事件 · 待人工确认'),make('p',event.reason||'控制保护已触发'),
        make('p',`${event.wall_ns?new Date(event.wall_ns/1e6).toLocaleString('zh-CN'):'时间见故障回执'}。${event.stop_confirmed===true?'SDK 输出进程退出已确认；机械停止与保持以现场核验为准。':'已触发控制急停；输出中断尚待核验，请检查现场停止入口。'}`),
        make('p','确认只关闭这条告警，不恢复控制、不续跑任务。'));
      const button=make('button','我已知晓，关闭这条告警');button.disabled=pending;
      button.addEventListener('click',async()=>{
        if(pending)return;pending=true;error='';render();
        try{await acknowledge(event.id);state={...state,warnings:state.warnings.filter(row=>row.id!==event.id)};}
        catch(failure){error='确认未保存：'+failure.message;}
        finally{pending=false;render();}
      });box.append(button);
      if(typeof recover==='function')box.append(recoveryLink());
      warnings.append(box);
    }
    if(error)warnings.append(make('p',error));
    const stopped=live?.recovery_required===true&&heartbeat?.status==='not_monitored';
    current.hidden=!needsRecovery&&(live?.active_anomaly===false||heartbeat?.status==='not_monitored');
    current.className=stopped?'control-recovery-required':'control-heartbeat-anomaly';
    current.textContent=stopped?'控制已断开 · SDK 输出进程已退出，等待中断恢复':
      live?.active_anomaly===true?(heartbeat?.status==='lost'?'当前心跳异常 · ':'当前控制链路异常 · ')+(live.reason||'控制链路异常；控制保持急停，等待核验恢复'):
      '当前心跳状态未知 · '+(live?.reason||'等待管理服务核验；无法据此认定已恢复');
    if(!current.hidden&&typeof recover==='function')current.append(make('span',' · '),recoveryLink());
    recovery.hidden=(current.hidden&&!(state?.warnings||[]).length)||typeof recover!=='function';
  }
  return {update(value,{stale=false}={}){state=stale?{...value,current:{active_anomaly:null,reason:'状态读取中断，不能确认心跳已恢复'}}:value;render();},
    setRecoveryGate(gate){recoverButton.disabled=gate?.allowed!==true;recoverReason.textContent=recoverButton.disabled?(gate?.reason||'当前尚不允许恢复，请先核验状态。'):'';},root};
}

// Independent management UI. All effects remain behind the deployment's API gates.
const DEPLOYMENT_SHA='__WORKBENCH_DEPLOYMENT_SHA__';
const STYLE=`
.wb-operations .ops-stop{background:#a33438;color:white;border-color:#a33438}.wb-operations pre{white-space:pre-wrap;overflow-wrap:anywhere;max-height:240px;overflow:auto}.wb-operations .ops-muted{overflow-wrap:anywhere}.wb-operations .ops-profile{padding:15px;margin:16px 0 20px;border:1px solid #e1e8f1;border-radius:9px;background:#f5f8fc}
.wb-operations .ops-guidance{margin:16px 0;padding:16px 18px;border:1px solid #c8d9eb;border-left:4px solid #24619a;border-radius:8px;background:#f5f9ff}.wb-operations .ops-guidance p{margin:5px 0 8px;color:#526882}.wb-operations .ops-guidance a{color:#24619a;font-weight:600}.wb-operations .ops-group{margin:16px 0;border-top:1px solid #e1e8f1;padding-top:16px}.wb-operations .ops-group summary{cursor:pointer;font-size:16px;font-weight:600;padding:5px 0}.wb-operations .ops-group>p{margin-top:5px}
.wb-operations{font:14px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;color:#233650;background:#fff;border:1px solid #d5deea;border-radius:12px;padding:22px;margin:0 0 18px;min-width:0}
.wb-operations>.ops-actions{position:sticky;top:8px;z-index:5;background:#fff;padding:8px 0;border-bottom:1px solid #e1e8f1}.wb-operations *{box-sizing:border-box}.wb-operations [hidden]{display:none!important}.wb-operations h2,.wb-operations h3,.wb-operations p{margin:0}.wb-operations h2{font-size:21px}.wb-operations h3{font-size:15px}.wb-operations button{font:inherit;min-height:40px;border:1px solid #bdccdf;border-radius:7px;padding:8px 13px;background:#f5f8fc;color:#233650;cursor:pointer}.wb-operations button:hover:not(:disabled){background:#e7effa}.wb-operations button:disabled{cursor:not-allowed;opacity:.55}.wb-operations button:focus-visible,.wb-operations input:focus-visible{outline:3px solid #74a6d5;outline-offset:3px}.wb-operations .ops-primary{background:#24619a;color:#fff;border-color:#24619a}.wb-operations .ops-head,.wb-operations .ops-actions{display:flex;gap:12px;align-items:center;flex-wrap:wrap}.wb-operations .ops-head{justify-content:space-between;margin-bottom:12px}.wb-operations .ops-tag{font-size:12px;border:1px solid #c8d5e6;background:#f0f5fc;border-radius:20px;padding:3px 10px}.wb-operations .ops-stop{background:#a33438;color:white;border-color:#a33438}.wb-operations pre{white-space:pre-wrap;overflow-wrap:anywhere;max-height:240px;overflow:auto}.wb-operations .ops-muted{font-size:12px;color:#66758a}.wb-operations .ops-status{padding:10px 0;min-height:42px;overflow-wrap:anywhere}.wb-operations .ops-error{color:#a33438}.wb-operations .ops-good{color:#246d54}.wb-operations .ops-warning{color:#99600e}.wb-operations .ops-health{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:10px;margin:16px 0}.wb-operations .ops-health>div{padding:11px 13px;background:#f5f8fc;border:1px solid #e1e8f1;border-radius:8px}.wb-operations .ops-health strong{display:block;font-weight:500;overflow-wrap:anywhere}.wb-operations .ops-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px;margin:16px 0}.wb-operations .ops-card{display:flex;flex-direction:column;align-items:flex-start;gap:8px;padding:15px;border:1px solid #d5deea;border-radius:9px}.wb-operations .ops-card p{font-size:12px;overflow-wrap:anywhere}.wb-operations .ops-card button{margin-top:auto}.wb-operations .ops-job{padding:14px;background:#f5f8fc;border:1px solid #d5deea;border-radius:9px;margin-top:12px;overflow-wrap:anywhere}.wb-operations .ops-job progress{display:block;width:100%;height:12px;margin:9px 0;accent-color:#24619a}.wb-operations .ops-job pre{font:12px/1.6 ui-monospace,monospace;white-space:pre-wrap;overflow-wrap:anywhere;max-height:190px;overflow:auto;margin:8px 0}.wb-operations .ops-history{margin-top:18px}.wb-operations .ops-history summary{cursor:pointer;font-weight:600;padding:6px 0}.wb-operations dialog{width:min(560px,calc(100% - 32px));border:1px solid #bdccdf;border-radius:12px;padding:24px;color:#233650;box-shadow:0 18px 65px #23365030}.wb-operations dialog::backdrop{background:#14243b70}.wb-operations dialog p{margin:12px 0;white-space:pre-wrap;overflow-wrap:anywhere}.wb-operations .ops-support{display:flex;align-items:flex-start;gap:9px;padding:12px;background:#fff5e3;border-radius:7px;margin:16px 0}.wb-operations .ops-support input{width:18px;height:18px;flex:none;margin-top:3px}.wb-operations dialog .ops-actions{justify-content:flex-end;margin-top:20px}@media(max-width:600px){.wb-operations{padding:16px}.wb-operations .ops-grid{grid-template-columns:1fr}}
`;
let instance=0;
const copy=value=>JSON.parse(JSON.stringify(value));
const canonical=value=>JSON.stringify(value,(_key,item)=>item&&typeof item==='object'&&!Array.isArray(item)?Object.fromEntries(Object.keys(item).sort().map(key=>[key,item[key]])):item);
const same=(a,b)=>canonical(a)===canonical(b);
const describeTarget=target=>target?`${target.environment==='physical'?'Physical · 物理设备':'Fake · 开发设备'} / ${target.id||target.device_id||target.robot_id||'未命名设备'} / ${target.manager_instance||target.manager_instance_id||target.deployment_id||'未提供管理器版本'}`:'尚未核验目标';
const readable=value=>typeof value==='string'?value:JSON.stringify(value,null,2);
export const phaseText=value=>({preparing_exit_home:'准备退出前回零审阅',returning_home:'执行平滑低速回零',waiting_exit_home:'等待回零、静止与录制封存',waiting_recording_seal:'等待录制封存与写入进程退出',verifying_postconditions:'核验实际后置条件',verifying_model_identity:'核验模型与IO身份',stopping_environment:'退出工作台资源',restoring_collection_web:'恢复原数采网页',waiting_collection_web:'核验原数采入口',stopping_current_control:'停止旧控制任务',waiting_control_stop:'等待停止与静止反馈',checking_collection_handoff:'核验数采交接',stopping_collection:'正常结束数采',waiting_collection_exit:'等待数采保存与退出',stopping_collection_cameras:'正常退出数采相机服务',waiting_collection_cameras_exit:'等待数采相机释放设备',stopping_collection_web:'暂停数采网页',waiting_collection_web_exit:'核验数采进程退出',waiting_control_owner:'等待控制服务就绪',recovering_workbench_control:'接入并核验工作台控制',queued:'已排队',preflight:'核验当前条件',run:'执行已确认操作',verify:'核验实际结果',complete:'完成',recovery:'等待核验并结束遗留维护',start_workbench:'启动工作台后端',stop_workbench:'等待旧后端退出',start_model:'启动模型服务',restart_model:'重启模型服务',waiting_model_endpoint:'等待模型接口就绪',warming_model:'加载并预热模型',camera_service:'准备相机服务',start_cameras:'启动相机服务',restart_cameras:'重启相机服务',stop_cameras:'等待相机服务退出',waiting_camera_source:'等待相机真实帧',rebinding_camera_clients:'重新绑定相机消费者',resetting_camera_usb:'复位指定USB相机',restoring_warm_model_binding:'恢复原暖模型绑定',releasing_control:'释放设备控制',verifying_old_maintenance:'核验遗留维护'})[value]||value||'未知';
export const jobText=job=>{
  if(!job)return '尚无操作记录';
  if(job.status==='succeeded')return job.verification?.ok===true?'已完成 · 后置核验通过':'返回成功状态，但后置核验尚未通过';
  return ({queued:'已排队',running:'正在执行',cancel_requested:'已请求取消 · 等待回调与后置核验',cancelled:'已取消 · 回调已确认',waiting_onsite:'等待现场处理',waiting:'等待前置条件',unknown:'结果尚未核验',failed:'操作失败',timeout_unknown:'操作超时 · 状态未知，不能认为服务已退出',interrupted_unknown:'管理器曾中断 · 状态未知，不能认为服务已退出'})[job.status]||`状态：${job.status||'未知'}`;
};

function workspaceURL(value){
  if(typeof value!=='string')return null;
  try{
    const url=new URL(value);
    return ['http:','https:'].includes(url.protocol)&&!url.username&&!url.password?url.href:null;
  }catch{return null;}
}

export function operationsGuidance(state,stale=false){
  const health=state?.health||{},caps=state?.current_capabilities||{},runtime=state?.runtime||health.runtime||{};
  const missing=(state?.dependencies||[]).find(row=>row.category==='onsite'&&['blocked','failed','waiting_onsite'].includes(row.status));
  if(stale||!state)return {title:'当前状态尚未核验',message:'等待管理服务返回新状态；历史结果不能代表服务现在仍然可用。'};
  if(state.active_job)return {title:(state.active_job.label||'维护操作')+' · '+jobText(state.active_job),message:phaseText(state.active_job.phase)+'。当前操作完成并通过核验后，再继续下一步。',current:true};
  const closing=(state.recent_jobs||[]).some(job=>job.status==='succeeded'&&job.verification?.ok===true&&job.result?.manager_exit_pending===true&&job.request?.expected_target?.manager_instance===state.target?.manager_instance);
  if(closing)return {title:'工作台资源已退出',message:'正在关闭管理入口。重新使用时请运行启动脚本。'};
  const exited=(state.recent_jobs||[]).some(job=>job?.request?.action==='exit_environment'&&job.status==='succeeded'&&job.verification?.ok===true);
  const resources=state.services||health.services||{};
  if(resources.control_owner?.lifecycle==='external'&&resources.workbench?.state==='stopped'
      &&resources.workbench?.running===false&&state.operations?.connect_control?.allowed===true)
    return {title:'接入本工作台控制',message:'共享执行服务已就绪。接入本工作台后，可进入推理页运行模型。',action:'connect_control'};
  const consumerExited=(state.recent_jobs||[]).find(job=>job?.request?.action==='exit_environment'&&job.status==='succeeded'&&job.verification?.ok===true&&job.verification?.consumer_detached===true);
  if(consumerExited&&resources.workbench?.state==='stopped'&&resources.workbench?.running===false)
    return {title:'已退出本工作台',message:'本工作台的控制claim和模型绑定已释放，共享控制、相机和模型服务保留；其他消费者可继续使用。没有启动原数采。',url:workspaceURL(consumerExited.result?.next_url)};
  if(exited&&['workbench','control_owner','cameras','model'].every(name=>resources[name]?.state==='stopped'&&resources[name]?.running===false))
    return {title:'工作台环境已退出',message:'工作台控制、相机和模型已核验释放，原数采网页已恢复。请在数采端继续操作；此管理页保留回执，不会自动接回设备。'};
  if(missing)return {title:'等待现场处理 · '+missing.label,message:missing.reason||missing.actual||'请按依赖要求处理现场条件，再重新检查。',action:missing.recommended_action||'status_check'};
  if(state.operations?.reclaim_control?.allowed===true&&(!runtime.connected||state.recovery?.required||['FAULT','FAULT_STOP_UNCONFIRMED','EMERGENCY'].includes(runtime.state)))
    return {title:'清理旧控制并接入工作台',message:state.operations.reclaim_control.impact,action:'reclaim_control'};
  if(state.recovery?.required)return {title:'先处理未完成的维护',message:'上次操作的结果尚未核验。先结束遗留维护，再开始新的操作；不会恢复旧动作。',action:'recover_maintenance'};
  if(runtime.release_receipt?.status==='unconfirmed')return {title:'设备释放结果未知',message:runtime.release_receipt.message||'SDK 关闭尚未确认；请先核对恢复计划和现场条件。',action:'recover_control'};
  if(['FAULT','FAULT_STOP_UNCONFIRMED','EMERGENCY'].includes(runtime.state))return {title:'控制故障仍锁定',message:runtime.reason||'反馈恢复不代表控制恢复；请核对中断恢复的范围与现场条件。',action:'recover_control'};
  if(health.backend_reachable!==true)return {title:health.services?.workbench?.running===true?'工作台进程仍在，接口尚未就绪':'工作台业务服务尚未就绪',message:health.error||'先准备当前环境；准备服务不会连接或控制机器人。',action:health.services?.workbench?.running===true?'status_check':'prepare_environment'};
  if(caps.control_connected===true&&health.runtime?.feedback_healthy===false)return {title:'控制反馈需要处理',message:'服务在线不代表设备可执行。请核对中断恢复与保持证据。',action:'recover_control'};
  if(caps.camera_frames!==true)return {title:'相机观测尚未就绪',message:health.services?.cameras?.error||'还未确认三路新鲜画面。先核对相机服务和观测绑定。',action:health.services?.cameras?.state==='stopped'?'start_cameras':'rebind_cameras'};
  if(health.services?.model&&(caps.model_loaded!==true||health.model_service?.status!=='ready'||health.model_service?.warmup?.completed!==true))return {title:'画面可用，推理模型尚未就绪',message:health.model?.error||'需要推理时准备所选模型。标定和数采仍按各自工作区的条件进行。',action:'prepare_model'};
  if(health.services?.model&&caps.workflow_model_bound!==true)return {title:'模型在运行，工作区尚未准备好',message:health.model?.error||'准备模型以核验并恢复当前工作区绑定；不会自动接入设备控制。',action:'prepare_model'};
  return {title:caps.control_connected===true?'观测可用 · 控制已接入':'观测可用 · 控制未接入',message:'标定、数采与推理各自的依赖状态见下方。Run 只申请执行，不会连接或恢复设备。',workspace:true};
}

export function createOperationClient({fetch:request,crypto=globalThis.crypto,deploymentVersion=DEPLOYMENT_SHA}={}){
  let sequence=0;
  const id=()=>crypto?.randomUUID?.()||`ops-${Date.now().toString(36)}-${++sequence}-${Math.random().toString(36).slice(2)}`;
  async function call(environment,kind,body,csrf){
    const response=await request(kind==='environment'?`/api/environment?environment=${environment}&view=summary`:`/api/operations/${kind}?environment=${environment}`,{
      credentials:'same-origin',cache:'no-store',signal:globalThis.AbortSignal?.timeout?.(kind==='environment'?10000:15000),
      ...(body?{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf,...(/^[a-f0-9]{64}$/.test(deploymentVersion)?{'X-Workbench-Deployment':deploymentVersion}:{})},body:JSON.stringify(body)}:{})});
    const result=await response.json();
    if(!response.ok){const error=Error(result.error?.message||result.error||`管理接口返回 HTTP ${response.status}`);error.rejected=true;throw error;}
    if(kind==='environment'&&(result.schema!=='robot_workbench.environment.v1'||!result.operations||result.target?.environment!==environment||!result.csrf))throw Error('尚未部署受支持的统一运维服务，或返回的目标身份不匹配');
    if(result.projection==='manager_environment_summary'){
      const job=id=>id==null?null:result.jobs[id];
      result.active_job=job(result.active_job);result.recent_jobs=result.recent_jobs.map(job);
      for(const row of Object.values(result.operations))row.latest_job=job(row.latest_job);
      for(const key of ['runtime','services','model','collection','resources','activity','maintenance'])result.health[key]=result[key];
      for(const capability of Object.values(result.capabilities||{}))capability.identity=result.target;
    }
    return result;
  }
  return {id,read:environment=>call(environment,'environment'),submit:(environment,body,csrf,kind='action')=>call(environment,kind,body,csrf),
    stop:(environment,target,csrf)=>call(environment,'stop',{expected_device_id:target.device_id,request_id:id()},csrf),
    compatible:state=>/^[a-f0-9]{64}$/.test(deploymentVersion)&&state?.deployment?.sha256===deploymentVersion};
}

export const capabilityText=value=>({ready:'已核验就绪',blocked:'条件未满足',unknown:'状态未知',unsupported:'不支持',degraded:'能力受限',failed:'核验失败',waiting_onsite:'等待现场处理',active:'正在处理',declared:'部署已声明，待实际核验',operator_confirmation:'操作时确认现场条件',not_applicable:'不适用',requires_review:'执行前需审阅'})[value]||value||'状态未知';

export function environmentSwitchReason(state,{stale=false,pending=false}={}){
  if(stale||!state)return '当前控制与任务状态尚未核验，不能切换接入目标';
  const health=state.health||{},runtime=state.runtime||{},workbench=health.services?.workbench;
  if(pending||state.active_job||health.maintenance||state.maintenance||runtime.maintenance_active||state.recovery?.required)return '当前操作或维护尚未核验结束，请先处理后再切换环境';
  if(runtime.connected===true||health.busy===true)return '请先停止任务并在运维释放当前设备，再切换环境；不会自动释放';
  const stopped=health.backend_reachable===false&&workbench?.state==='stopped'&&workbench.running===false&&state.recovery?.required===false;
  if((runtime.connected!==false||health.busy!==false)&&!stopped)return '当前控制与任务状态尚未核验，不能切换接入目标';
  return '';
}

export function createOperations({document:doc,fetch:request,container,location=globalThis.location,
  crypto=globalThis.crypto,setTimeout:schedule=globalThis.setTimeout,clearTimeout:unschedule=globalThis.clearTimeout,pollMs=2000,deploymentVersion=DEPLOYMENT_SHA}={}){
  if(!container)container=doc.querySelector('[data-workbench-operations]');
  if(!container)return null;
  if(container.workbenchOperations)return container.workbenchOperations;
  const prefix=`workbench-operations-${++instance}`,client=createOperationClient({fetch:request,crypto,deploymentVersion});
  container.classList.add('wb-operations');
  const make=(tag,text,className)=>{const node=doc.createElement(tag);if(text!==undefined)node.textContent=text;if(className)node.className=className;return node;};
  const style=make('style',STYLE);container.append(style);
  const heading=make('div',undefined,'ops-head'),title=make('h2','服务运维'),badge=make('span','等待目标','ops-tag');
  heading.append(title,badge);container.append(heading);
  const identity=make('p','尚未核验目标','ops-muted');container.append(identity);
  container.append(make('p','环境与设备生命周期统一在此管理；全局 Stop 始终独立可达，不等同硬件急停。','ops-muted'));
  const controlAlerts=mountControlAlerts({document:doc,container,recover:()=>begin('recover_control'),acknowledge:event_id=>client.submit(environment,{event_id,environment,device_id:state.target.device_id},state.csrf,'acknowledge-control-warning')});
  const status=make('p','正在读取管理服务…','ops-status');status.setAttribute('role','status');status.setAttribute('aria-live','polite');container.append(status);
  const toolbar=make('div',undefined,'ops-actions'),refreshButton=make('button','刷新状态'),retryButton=make('button','重试同一提交'),stopButton=make('button','Stop · 停止当前设备任务');
  stopButton.className='ops-stop';
  retryButton.hidden=true;toolbar.append(stopButton,refreshButton,retryButton);container.append(toolbar);
  const guidance=make('section',undefined,'ops-guidance'),guidanceTitle=make('h3'),guidanceMessage=make('p'),guidanceLink=make('a');
  guidance.setAttribute('aria-label','当前状态与下一步');guidance.append(guidanceTitle,guidanceMessage,guidanceLink);container.append(guidance);
  const submissionError=make('p',undefined,'ops-error');submissionError.hidden=true;submissionError.setAttribute('role','alert');container.append(submissionError);
  const versionWarning=make('p',undefined,'ops-warning'),reloadButton=make('button','重新加载页面');reloadButton.hidden=true;
  versionWarning.setAttribute('role','alert');container.append(versionWarning,reloadButton);
  const profile=make('details',undefined,'ops-profile ops-history');profile.setAttribute('aria-label','可信部署机器档案');container.append(profile);
  const healthTitle=make('h3','当前服务状态');container.append(healthTitle);
  const health=make('div',undefined,'ops-health'),cards=make('div',undefined,'ops-groups');container.append(health);
  const current=make('section');current.id=prefix+'-current';current.setAttribute('aria-label','当前管理操作');container.append(current,cards);
  const groups=[
    {title:'接入与控制',description:'核对目标、现场条件与控制事实。恢复不续跑旧任务。',actions:['reclaim_control','exit_workbench','exit_environment','connect_control','release_control','recover_control','verify_hold']},
    {title:'依赖与能力',description:'预置、可准备、现场条件分别核验；各业务依赖独立判断。',actions:['prepare_environment','prepare_collection','prepare_model']},
    {title:'服务与资源',description:'按实际影响管理已登记服务；计算角色可单独重启，其他健康服务保留。',actions:['start_cameras','stop_cameras','restart_cameras','rebind_cameras','restart_model','stop_model','restart_detector','restart_planner','restart_scene','restart_workbench','stop_workbench','reset_camera_head','reset_camera_left','reset_camera_right'],collapsed:true},
    {title:'诊断与回执',description:'当前事实与历史操作分开；进程退出、SDK 释放与物理保持分别核验。',actions:['status_check','recover_maintenance']}
  ].map(group=>{const node=make(group.collapsed?'details':'section',undefined,'ops-group'),rows=make('div',undefined,'ops-grid');node.append(make(group.collapsed?'summary':'h3',group.title),make('p',group.description,'ops-muted'),rows);cards.append(node);return {...group,node,rows};});
  const controlFacts=make('div',undefined,'ops-health'),capabilityRows=make('div',undefined,'ops-health'),dependencyRows=make('div'),resourceRows=make('details',undefined,'ops-history'),collectionNotice=make('p',undefined,'ops-warning');
  const dependencySections=new Map(),dependencyEvidence=new Map();
  for(const [category,label]of Object.entries({preinstalled:'预置条件',preparable:'可通过运维准备',onsite:'现场条件'})){
    const section=make('details'),body=make('div');section.open=category!=='preinstalled';
    section.append(make('summary',label),body);dependencyRows.append(section);dependencySections.set(category,body);
  }
  collectionNotice.setAttribute('role','status');
  groups[0].node.append(controlFacts,profile);groups[1].node.append(capabilityRows,collectionNotice,dependencyRows);groups[2].node.append(healthTitle,health,resourceRows);groups[3].node.append(current);
  const emptyCards=make('p','等待管理服务返回可用操作。','ops-muted');cards.append(emptyCards);
  guidanceLink.addEventListener('click',()=>{const next=operationsGuidance(state,stale),group=groups.find(row=>row.actions.includes(next.action));if(group?.collapsed)group.node.open=true;});
  const history=make('details',undefined,'ops-history'),historyTitle=make('summary','历史操作回执（不代表当前服务状态）'),historyRows=make('div');history.append(historyTitle,historyRows);groups[3].node.append(history);
  const dialog=make('dialog');dialog.setAttribute('aria-labelledby',prefix+'-confirm-title');
  const dialogTitle=make('h3','确认本次操作');dialogTitle.id=prefix+'-confirm-title';
  const reviewTarget=make('p',undefined,'ops-muted'),reviewImpact=make('p'),supportLabel=make('label',undefined,'ops-support'),support=make('input');support.type='checkbox';
  supportLabel.append(support,make('span','我已现场确认双臂得到可靠支撑，允许对上面这个目标执行本次操作。'));
  const reviewReason=make('p',undefined,'ops-warning'),dialogActions=make('div',undefined,'ops-actions'),dismiss=make('button','取消'),confirm=make('button','确认执行','ops-primary');
  dismiss.autofocus=true;dialogActions.append(dismiss,confirm);dialog.append(dialogTitle,reviewTarget,reviewImpact,supportLabel,reviewReason,dialogActions);container.append(dialog);
  const environmentSelect=doc.getElementById('environment');
  const query=new URLSearchParams(location?.search||'');
  const queryEnvironment=query.get('environment')==='fake'?'fake':'physical';
  const validBusiness=value=>['calibration','collection','inference','research'].includes(value)?value:'calibration';
  if(environmentSelect&&(container.dataset.operationsStandalone==='true'||['fake','physical'].includes(query.get('environment'))))environmentSelect.value=queryEnvironment;
  let environment=environmentSelect?.value==='fake'?'fake':environmentSelect?.value==='physical'?'physical':queryEnvironment;
  let state=null,stale=true,posting=false,stopping=false,stopNotice='',review=null,lastJob=null,uncertain=null,destroyed=false,timer=null,generation=0,reading=null,sequence=0;
  let pendingHandoff=null,exitVerified=false,entryClosed=false;
  const buttons=new Map(),reasons=new Map();
  const operation=name=>state?.operations?.[name];
  const reasonFor=row=>{const reason=row?.disabled_reason||row?.reason;return reason==='Another management operation is still in flight.'?'另一项维护操作尚未完成，请等待其核验结果。':reason||'当前前置条件不满足';};
  const compatible=()=>client.compatible(state);
  const versionReason=()=>'页面与管理器的部署版本不一致，请重新加载页面；诊断与安全停止仍可使用。';
  const canUse=name=>!exitVerified&&!stale&&!posting&&!uncertain&&(name==='status_check'||compatible())&&operation(name)?.allowed===true&&!!state?.csrf&&state.target?.environment===environment;
  const reviewSignature=row=>({label:row?.label,impact:row?.impact,requires_support:row?.requires_support===true});
  const nextId=client.id;
  function endReview(){review=null;support.checked=false;if(dialog.open)dialog.close();}
  function gates(){
    controlAlerts.setRecoveryGate({allowed:canUse('recover_control'),reason:stale?'管理服务状态不可用，等待重新核验。':!compatible()?versionReason():posting?'本次请求正在提交，请勿重复操作。':uncertain?'有一次提交结果尚未核验，请先核对或重试同一提交。':reasonFor(operation('recover_control'))});
    for(const [name,button]of buttons){
      button.disabled=!canUse(name);
      reasons.get(name).textContent=stale?'管理服务状态不可用，等待重新核验。':name!=='status_check'&&!compatible()?versionReason():posting?'本次请求正在提交，请勿重复操作。':uncertain?'有一次提交结果尚未核验，请先核对或重试同一提交。':operation(name)?.allowed===true?'当前条件允许':reasonFor(operation(name));
    }
    refreshButton.disabled=posting;stopButton.disabled=exitVerified||stopping||!state?.csrf||!state?.target?.device_id;
    retryButton.hidden=!uncertain;
    retryButton.disabled=posting||stale||!same(uncertain?.expected_target,state?.target)||(uncertain?.action!=='status_check'&&!compatible());
    confirm.disabled=!review||posting||stale||!same(review?.target,state?.target)
      ||(review?.kind==='cancel'?!compatible()||state?.active_job?.request?.request_id!==review.request_id:!canUse(review?.action))
      ||(review?.requires_support&&!support.checked);
    if(review)reviewReason.textContent=stale?'管理服务状态尚未重新核验，暂不能执行。':!compatible()?versionReason():
      review.kind==='cancel'?'取消请求需要回调协作；完成前不能认为服务或 SDK 已停止。':
      operation(review.action)?.allowed!==true?reasonFor(operation(review.action)):
      review.requires_support&&!support.checked?'请为本次操作确认双臂已可靠支撑。':'';
  }
  function jobNode(job){
    const node=make('article',undefined,'ops-job'),label=make('strong',job.label||job.request?.action||'服务操作');
    const text=make('p',jobText(job),job.status==='succeeded'&&job.verification?.ok===true?'ops-good':job.status?.includes('unknown')||job.status==='failed'?'ops-warning':'');
    node.append(label,text,make('p',`阶段：${phaseText(job.phase)} · 请求：${job.request?.request_id||job.job_id||'未提供'}`,'ops-muted'));
    node.append(make('p',`记录时间：${job.created_at_ns?new Date(job.created_at_ns/1e6).toLocaleString('zh-CN'):'历史记录未提供'} · 部署版本：${job.request?.expected_target?.deployment_sha256||'历史记录未登记版本'}`,'ops-muted'));
    const bar=make('progress');bar.max=1;if(Number.isFinite(job.progress))bar.value=Math.max(0,Math.min(1,job.progress));bar.setAttribute('aria-label','操作进度');node.append(bar);
    if(job.detail)node.append(make('p',readable(job.detail),'ops-muted'));
    for(const ref of job.detail?.step_refs||[]){
      const match=/^(UC\d{2}|SC\d{2})-S\d{2}$/.exec(ref);if(!match)continue;
      const query=new URLSearchParams(match[1].startsWith('UC')?{uc:match[1],step:ref}:{sc:match[1],substep:ref});
      const link=make('a',`架构步骤 ${ref}（关联，不代表已通过）`);link.href='http://10.17.0.27:18930/#trace?'+query;
      link.target='_blank';link.rel='noopener';node.append(link);
    }
    if(job.error)node.append(make('p',typeof job.error==='string'?job.error:job.error.message||readable(job.error),'ops-error'));
    if(job.result!=null){const details=make('details');details.append(make('summary','查看操作结果'),make('pre',readable(job.result)));node.append(details);}
    if(job.verification){const details=make('details');details.append(make('summary','查看后置核验'),make('pre',readable(job.verification)));node.append(details);}
    if(typeof job.detail_url==='string'&&/^\/api\/operations\/jobs\/[A-Za-z0-9_-]+$/.test(job.detail_url)){const link=make('a','完整操作证据');link.href=job.detail_url+'?environment='+environment;node.append(link);}
    if(job.timed_out&&job.verification?.ok===true)node.append(make('p','该操作曾超时；以上结论来自后续实际核验。','ops-muted'));
    if(state?.active_job?.job_id===job.job_id&&job.finished_at_ns==null){
      if(['prepare_collection','restart_detector','restart_planner','restart_scene'].includes(job.request?.action)){
        const value=state.collection||state.health?.collection||{},worker=value.worker||{},elapsed=value.progress?.elapsed_s;
        const stages={recovering_detector:'只恢复检测服务',recovering_planner:'只恢复规划服务',recovering_scene:'只恢复场景服务',awaiting_prepare:'数采计算待准备或预热',initializing_detector:'正在初始化检测服务',starting_detector:'正在启动检测服务',initializing_planner:'正在初始化规划服务',initializing_scene:'正在初始化场景服务',warming_planner:'正在预热规划服务',warming_scene:'正在预热场景服务',verifying_installation:'正在核验安装标定',building_assets:'正在推导数采资产',ready:'数采准备已就绪',failed:'数采准备失败',running:'数采准备进行中'};
        const known=stages[worker.message]||stages[value.status];
        for(const step of value.service_steps||[]){const role={detector:'检测',planner:'规划',scene:'场景'}[step.role]||step.role;node.append(make('p',`${role}：${({reused:'已核验复用',recovered:'已局部恢复',restarted:'已重启',unknown:'待核验'}[step.outcome]||step.outcome)}${step.needs_warmup?' · 待预热':''}`,'ops-muted'));}
        node.append(make('p',(known||'正在核验数采准备阶段')+(Number.isFinite(elapsed)&&elapsed>=0?` · 已用 ${elapsed.toFixed(1)} 秒`:'')+'；不估算剩余时间。','ops-muted'));
        if((worker.message&&!stages[worker.message])||(value.status&&!stages[value.status])){const detail=make('details');detail.append(make('summary','数采准备原始阶段'),make('pre',readable({status:value.status,worker_message:worker.message})));node.append(detail);}
      }
      if(job.cancellable){const cancel=make('button','请求取消');cancel.disabled=posting||stale||!compatible()||job.cancellation_requested===true;cancel.addEventListener('click',()=>openCancel(job));node.append(cancel);}
      else node.append(make('p','此操作不支持取消；请等待当前阶段完成。不会自动杀死 SDK 进程。','ops-muted'));
    }
    return node;
  }
  function render(){
    controlAlerts.update(state?.control_alerts,{stale});
    const link=doc.getElementById('operations-link');
    const business=validBusiness(link?.dataset.business||query.get('business'));
    const contextQuery='environment='+environment+'&business='+business;
    if(link)link.href=container.dataset.operationsStandalone==='true'?'/operations?'+contextQuery:'#operations-panel';
    const returnLink=doc.getElementById('operations-return');if(returnLink)returnLink.href='/?'+contextQuery;
    badge.textContent=environment==='physical'?'Physical · 物理设备':'Fake · 开发设备';identity.textContent=describeTarget(state?.target);
    versionWarning.hidden=!state||compatible();versionWarning.textContent=versionWarning.hidden?'':versionReason();reloadButton.hidden=versionWarning.hidden;
    const next=operationsGuidance(state,stale);guidanceTitle.textContent=next.title;guidanceMessage.textContent=next.message;
    guidanceLink.hidden=!next.workspace&&!next.action&&!next.current&&!next.url;
    guidanceLink.href=next.url||(next.workspace?'/?'+contextQuery:next.current?'#'+current.id:'#'+prefix+'-action-'+next.action);
    guidanceLink.textContent=next.url?'打开数采工作台':next.workspace?'返回工作台继续':next.current?'查看当前操作进度':'查看“'+(operation(next.action)?.label||'环境检查')+'”';
    profile.replaceChildren();profile.append(make('summary','机器档案与部署版本 · 查看硬件、标定及已知限制'));
    const machine=state?.machine_profile,declared=machine?.declared||{},hardware=declared.hardware||{};
    profile.append(make('p',`设备：${machine?.identity?.device_id||state?.target?.device_id||'未登记'} · 名称：${declared.label||'未登记'} · 主机：${declared.host_name||'未登记'}`,'ops-muted'));
    profile.append(make('p',`机器人：${hardware.robot_model||'型号未登记'} · 序列号：${hardware.robot_serial||'未登记'}。档案不代表现场硬件已核验。`,'ops-muted'));
    for(const [role,camera]of Object.entries(hardware.cameras||{}))profile.append(make('p',`${({head:'头部',left:'左腕',right:'右腕'})[role]||role}相机：${camera.serial} · 内参索引：${camera.intrinsics_index||'未登记'}`,'ops-muted'));
    if(hardware.can_interfaces)profile.append(make('p',`CAN：${Object.entries(hardware.can_interfaces).map(([side,name])=>`${side} ${name}`).join(' / ')}`,'ops-muted'));
    if(hardware.disabled_grippers?.length)profile.append(make('p',`部署停用夹爪：${hardware.disabled_grippers.join(' / ')}`,'ops-warning'));
    const indexNames={configuration_index:'标定配置',library_index:'标定库',active_manifest_index:'安装清单'};
    for(const [name,row]of Object.entries(machine?.calibration_indexes||{}))profile.append(make('p',`${indexNames[name]||name}：${row.path} · ${({file_present:'文件可读',directory_present:'目录存在',unavailable:'不可用',unreadable:'无法读取',present_not_hashed_size_limit:'存在，未核验摘要'})[row.status]||'未核验'}${row.sha256?' · SHA '+row.sha256:''}`,'ops-muted'));
    if(!Object.keys(machine?.calibration_indexes||{}).length)profile.append(make('p','标定索引尚未登记。','ops-muted'));
    for(const row of declared.known_degradations||[])profile.append(make('p',`已声明限制 · ${row.capability}：${row.reason}`,'ops-warning'));
    profile.append(make('p',`启动时登记的部署版本：${state?.deployment?.sha256||'未核验'}`,'ops-muted'));
    healthTitle.textContent=`${stale?'上次核验（当前状态未知）':'当前服务状态'} · ${state?.health?.checked_wall_ns?new Date(state.health.checked_wall_ns/1e6).toLocaleString('zh-CN'):'尚无核验时间'}`;
    health.replaceChildren();
    const names={manager:'管理服务',workbench:'工作台',control_owner:'控制服务',recording:'记录服务',pedal:'踏板输入源',cameras:'相机服务',model:'模型服务',detector:'检测服务',planner:'规划服务',scene:'场景服务',environment:'运行环境'};
    const healthNames={running:'运行中',ready:'已就绪',healthy:'健康',stopped:'已停止',unreachable:'不可达',unknown:'未知',starting:'正在启动',failed:'失败',not_configured:'尚未配置'};
    const healthValues=state?.health?.services?{manager:{status:'running'},...state.health.services,...(state.collection?.services||{})}:state?.health||{};
    if(state?.health?.services?.workbench&&state.health.backend_reachable===false)healthValues.workbench={...healthValues.workbench,
      status:healthValues.workbench.running===true?'进程存活，HTTP 不可达':healthValues.workbench.state,
      reason:state.health.error||healthValues.workbench.reason};
    for(const [name,value]of Object.entries(healthValues)){
      const summary=typeof value==='object'&&value!==null?String(value.status??value.state??'未知'):String(value??'未知');
      const row=make('div');row.append(make('span',names[name]||name,'ops-muted'),make('strong',(stale?'上次：':'')+(healthNames[summary]||summary)));
      if(value&&typeof value==='object'&&(value.error||value.reason&&!['running','ready','healthy'].includes(summary)))row.append(make('p',readable(value.error||value.reason),'ops-muted'));health.append(row);
    }
    if(!health.children.length)health.append(make('p',stale?'管理服务状态尚未核验':'管理服务可达；详细健康信息尚未提供','ops-muted'));
    for(const [key,label]of Object.entries({control_connected:'设备控制连接',model_loaded:'推理模型',camera_frames:'相机观测'})){
      const value=stale?undefined:state?.current_capabilities?.[key],row=make('div');
      const text=value==null?'尚未核验':key==='control_connected'?(value?'已连接':'未连接'):key==='camera_frames'?(value?'三路画面新鲜':'画面未就绪'):value?(state?.health?.model_service?.warmup?.completed?'已预热':'已加载，预热待核验'):'尚未加载';
      row.append(make('span',label,'ops-muted'),make('strong',text));health.append(row);
    }
    controlFacts.replaceChildren();
    const runtime=state?.runtime||state?.health?.runtime||{};
    for(const [label,value]of [['控制接入',runtime.connected===true?'已接入':runtime.connected===false?'未接入':'未知'],['控制持有者',runtime.holder??'未分配'],['Runtime 状态',runtime.state??'未知'],['保持证据',runtime.hold],['SDK 释放回执',runtime.release_receipt]]){
      const row=make('div');row.append(make('span',label,'ops-muted'),make('strong',(stale?'上次：':'')+(value==null?'未提供证据':readable(value))));controlFacts.append(row);
    }
    capabilityRows.replaceChildren();
    const collection=state?.collection||state?.health?.collection||{},failure=collection.warmup?.failure;
    collectionNotice.className=failure?'ops-warning':'ops-muted';
    collectionNotice.hidden=!failure&&(collection.ready===true||collection.services_ready!==true);
    collectionNotice.textContent=(stale?'当前状态未知；上次记录：':'')+(collection.services_ready===true?'常驻服务可用；':'常驻服务尚未全部核验；')+
      (failure?({scene_setup:'任务场景准备失败',model_init:'规划模型初始化失败',detection:'检测失败'}[failure.stage]||'准备失败阶段未知')+
        ' · '+(failure.native_reason&&failure.native_reason!=='unknown'?failure.native_reason:failure.error||'原因未知')+'；':'')+
      (collection.ready===true?'整体数采准备已就绪。':collection.services_ready===true?'业务初始化与预热尚未完成，请到数采页明确发起；不会自动重采或重启。':collection.ready===false?'整体数采准备尚不可用；不会自动重采或重启。':'整体数采准备尚未核验；不会自动重采或重启。');
    if(state?.execution_admission)capabilityRows.append(make('p',state.execution_admission.reason,'ops-muted'));
    for(const [name,label]of Object.entries({observe:'可观察',control:'控制接入',calibration:'标定依赖',collection:'数采依赖',inference:'推理依赖'})){
      const value=state?.capabilities?.[name],row=make('div');row.append(make('span',label,'ops-muted'),make('strong',stale?'当前未知':capabilityText(value?.status)));
      row.append(make('p',value?.reason||(value?.ready===true?'当前依赖已核验；实际执行仍检查本轮占用与授权。':'尚无可核验证据'),'ops-muted'));
      row.append(make('p',`观察时间：${value?.checked_wall_ns?new Date(value.checked_wall_ns/1e6).toLocaleString('zh-CN'):'未提供'} · 支持：${value?.support??'unknown'}`,'ops-muted'));capabilityRows.append(row);
    }
    const visibleEvidence=new Set();
    for(const [category,body]of dependencySections){
      body.replaceChildren();
      const rows=(state?.dependencies||[]).filter(row=>row.category===category);
      for(const value of rows){
        const row=make('article',undefined,'ops-job'),actual=value.actual,structured=actual!==null&&typeof actual==='object';
        const summary=structured?Object.entries({状态:actual.status??actual.state,阶段:actual.stage,说明:actual.message,PID:actual.service_pid??actual.pid,模型初始化:actual.model_initialized})
          .filter(([,item])=>item!==null&&item!==undefined&&typeof item!=='object')
          .map(([label,item])=>`${label}：${String(item).slice(0,240)}`).join(' · ')||'已提供结构化证据，展开查看':readable(actual??'未提供');
        row.append(make('strong',value.label||value.id),make('p',stale?'当前未知；以下为最后记录':capabilityText(value.status)),make('p',`预期：${readable(value.expected??'未声明')} · 实际：${summary}`,'ops-muted'),make('p',value.reason||'','ops-muted'),make('p',`证据时间：${value.checked_wall_ns?new Date(value.checked_wall_ns/1e6).toLocaleString('zh-CN'):'未提供'}`,'ops-muted'));
        if(structured){
          const key=JSON.stringify([environment,category,value.id]);visibleEvidence.add(key);
          let evidence=dependencyEvidence.get(key);
          if(!evidence){const details=make('details'),content=make('pre');details.append(make('summary','查看实际条件原始证据'),content);evidence={details,content};dependencyEvidence.set(key,evidence);}
          evidence.content.textContent=readable(actual);row.append(evidence.details);
        }
        body.append(row);
      }
      if(!rows.length)body.append(make('p','当前部署未声明此类证据。','ops-muted'));
    }
    for(const key of dependencyEvidence.keys())if(!visibleEvidence.has(key))dependencyEvidence.delete(key);
    resourceRows.replaceChildren();resourceRows.append(make('summary','资源声明与实测证据'),make('pre',readable(state?.resources||{status:'unknown',reason:'当前部署未提供资源证据'})));
    for(const group of groups)group.rows.replaceChildren();buttons.clear();reasons.clear();
    for(const name of groups.flatMap(group=>group.actions)){
      const row=operation(name);if(!row)continue;
      const card=make('article',undefined,'ops-card'),button=make('button',row.label||name);button.dataset.operation=name;
      card.id=prefix+'-action-'+name;
      const reason=make('p',stale?'管理服务状态不可用，等待重新核验。':row.allowed===true?'当前条件允许':reasonFor(row),'ops-muted');reason.id=prefix+'-'+name+'-reason';
      button.setAttribute('aria-describedby',reason.id);button.addEventListener('click',()=>begin(name));
      card.append(make('h3',row.label||name),make('p',String(row.impact||'影响说明尚未提供')),reason);
      if(row.requires_support)card.append(make('p','执行前须逐次确认双臂可靠支撑。','ops-warning'));
      if(row.latest_job)card.append(make('p','上次：'+jobText(row.latest_job),'ops-muted'));
      card.append(button);groups.find(group=>group.actions.includes(name)).rows.append(card);buttons.set(name,button);reasons.set(name,reason);
    }
    for(const group of groups)group.node.hidden=false;
    emptyCards.hidden=buttons.size>0;
    current.replaceChildren();const job=state?.active_job,recent=state?.recent_jobs?.[0];
    current.hidden=!job&&!recent;
    current.append(make('h3',job?'当前管理操作':'最近一次维护结果'));
    if(job)current.append(jobNode(job));else{
      current.append(make('p','当前没有管理操作；以下回执不代表当前服务状态。','ops-muted'));
      if(recent)current.append(jobNode(recent));
    }
    historyRows.replaceChildren();for(const row of(state?.recent_jobs||[]).slice(0,6))historyRows.append(jobNode(row));
    history.hidden=!state?.recent_jobs?.length;gates();
  }
  function showReview(value){
    endReview();review=value;support.checked=false;supportLabel.hidden=!value.requires_support;
    dialogTitle.textContent=value.kind==='cancel'?'确认请求取消':`确认：${value.label}`;
    reviewTarget.textContent=describeTarget(value.target);reviewImpact.textContent=value.impact;
    gates();dialog.showModal();
  }
  function begin(name){
    if(!canUse(name))return;
    if(name==='status_check'){void submit({action:name,request_id:nextId(),expected_target:copy(state.target),arms_supported:false});return;}
    const row=operation(name);
    showReview({action:name,label:row.label,impact:row.impact,requires_support:row.requires_support===true,target:copy(state.target),signature:reviewSignature(row)});
  }
  function openCancel(job){
    if(posting||stale||!compatible()||job.cancellation_requested||state?.active_job?.job_id!==job.job_id)return;
    showReview({kind:'cancel',request_id:job.request.request_id,target:copy(state.target),requires_support:false,
      impact:'只请求当前操作协作取消。服务与 SDK 的真实状态仍以回调和后置核验为准。'});
  }
  async function submit(body,kind='action'){
    if(posting||destroyed)return;
    if(kind==='action'&&['exit_workbench','exit_environment'].includes(body.action)){
      pendingHandoff={requestId:body.request_id,action:body.action,environment,deviceId:state.target.device_id,
        managerInstance:state.target.manager_instance,deployment:state.deployment.sha256};
    }
    posting=true;const epoch=generation;endReview();gates();submissionError.hidden=true;status.textContent='正在提交本次操作…';status.className='ops-status';
    try{
      const result=await client.submit(environment,body,state.csrf,kind);
      if(epoch!==generation||destroyed)return;
      lastJob=result.job||result;uncertain=null;status.textContent='请求已接收；执行结果以后置核验为准。';
    }catch(error){
      if(epoch!==generation||destroyed)return;
      if(error.rejected)pendingHandoff=null;
      if(kind==='action'&&!error.rejected)uncertain=copy(body);
      submissionError.textContent=(uncertain?'提交结果未知；先核对进度，重试会沿用同一请求编号。 ':'')+String(error.message||error);submissionError.hidden=false;
    }finally{
      posting=false;if(!destroyed){stale=true;render();void refresh();}
    }
  }
  function followVerifiedHandoff(){
    if(!pendingHandoff||stale||!compatible()||posting||destroyed)return;
    if(pendingHandoff.environment!==environment||pendingHandoff.deviceId!==state.target.device_id
        ||pendingHandoff.managerInstance!==state.target.manager_instance||pendingHandoff.deployment!==state.deployment.sha256){
      pendingHandoff=null;
      return;
    }
    const job=(state.recent_jobs||[]).find(row=>row.request?.request_id===pendingHandoff.requestId);
    if(!job)return;
    if(['failed','cancelled'].includes(job.status)){
      pendingHandoff=null;
      return;
    }
    const proof=job.verification||{};
    const shared=proof.consumer_detached===true&&proof.independent_services_preserved===true&&proof.shared_runtime_preserved===true;
    const released=proof.environment_exited===true&&proof.sdk_release_verified===true&&(pendingHandoff.action==='exit_workbench'||proof.collection_web_ready===true);
    if(job.status!=='succeeded'||proof.ok!==true||!(shared||released))return;
    const url=pendingHandoff.action==='exit_environment'?workspaceURL(job.result?.next_url):null;
    exitVerified=job.result?.manager_exit_pending===true;
    pendingHandoff=null;
    if(exitVerified){status.textContent='工作台资源已退出，正在关闭管理入口…';gates();}
    if(url)location?.assign?.(url);
  }
  async function refresh(){
    if(destroyed||entryClosed)return;
    const epoch=generation;if(reading?.epoch===epoch)return reading.promise;
    if(timer!==null){unschedule(timer);timer=null;}
    const promise=(async()=>{
      try{
        const incoming=await client.read(environment);
        if(epoch!==generation||destroyed)return;
        const changed=state&&(!same(state.target,incoming.target)||state.csrf!==incoming.csrf);
        if(changed){endReview();stopNotice='';lastJob=null;uncertain=null;submissionError.hidden=true;status.textContent='目标或管理器身份已变化，旧确认已撤销。';}
        if(review&&review.kind!=='cancel'&&!same(review.signature,reviewSignature(incoming.operations[review.action]))){endReview();status.textContent='操作影响说明已变化，请重新审阅。';}
        state=incoming;stale=incoming.health?.freshness?.valid===false;
        if(!compatible())endReview();
        if(uncertain){const recovered=(incoming.recent_jobs||[]).find(job=>job.request?.request_id===uncertain.request_id)||[incoming.active_job].find(job=>job?.request?.request_id===uncertain.request_id);if(recovered){lastJob=recovered;uncertain=null;submissionError.hidden=true;status.textContent='已找到刚才的请求回执，继续显示实际进度。';}}
        if(lastJob){const updated=(incoming.recent_jobs||[]).find(job=>job.job_id===lastJob.job_id);if(updated)lastJob=updated;}
        if(!stopNotice&&!review&&!uncertain&&!posting&&!changed){status.textContent='管理服务状态已更新。执行前仍会由后端重新核验。';status.className='ops-status';}
        render();
        followVerifiedHandoff();
      }catch(error){
        if(epoch!==generation||destroyed)return;
        stale=true;entryClosed=exitVerified;status.textContent=entryClosed?'工作台入口已关闭。重新使用时请运行启动脚本。':'无法核验管理服务：'+String(error.message||error)+'。已有服务的运行或停止状态不能据此判断。';status.className=entryClosed?'ops-status':'ops-status ops-error';render();
        if(entryClosed){guidanceTitle.textContent='工作台已退出';guidanceMessage.textContent='需要再次使用时，请重新运行启动脚本。';guidanceLink.hidden=true;}
      }finally{
        if(reading?.epoch===epoch)reading=null;
        if(epoch===generation&&!destroyed&&!entryClosed&&pollMs>0)timer=schedule(()=>void refresh(),pollMs);
      }
    })();
    reading={epoch,promise};return promise;
  }
  function setEnvironment(value){
    const next=value==='fake'?'fake':'physical';if(next===environment)return;
    environment=next;generation++;pendingHandoff=null;exitVerified=false;entryClosed=false;stopNotice='';state=null;stale=true;lastJob=null;uncertain=null;submissionError.hidden=true;endReview();status.textContent='环境已切换，正在重新核验目标…';render();void refresh();
  }
  const onEnvironment=()=>{
    const access=doc.workbenchEnvironmentAccess,next=environmentSelect.value;
    if(access?.navigateEnvironment){environmentSelect.value=environment;void access.navigateEnvironment(next,{page:container.dataset.operationsStandalone==='true'?'/operations':'/'});}
    else setEnvironment(next);
  };
  environmentSelect?.addEventListener('change',onEnvironment);
  support.addEventListener('change',gates);dismiss.addEventListener('click',endReview);
  dialog.addEventListener('cancel',endReview);
  confirm.addEventListener('click',()=>{
    gates();if(confirm.disabled)return;
    const selected=review;
    if(selected.kind==='cancel')void submit({request_id:selected.request_id},'cancel');
    else void submit({action:selected.action,request_id:nextId(),expected_target:copy(selected.target),arms_supported:selected.requires_support&&support.checked});
  });
  stopButton.addEventListener('click',async()=>{
    if(stopping||!state?.csrf||!state?.target?.device_id)return;stopping=true;const at=generation;endReview();gates();
    pendingHandoff=null;
    stopNotice='正在请求独立停止；等待设备回执，不据此推断机械静止。';status.textContent=stopNotice;
    try{const result=await client.stop(environment,state.target,state.csrf);if(at===generation&&!destroyed){stopNotice='停止请求回执（机械静止须另行核验）：'+readable(result);status.textContent=stopNotice;status.className='ops-status';}}
    catch(error){if(at===generation&&!destroyed){stopNotice='停止结果未知：'+error.message+'；请核对现场和独立停止设备。';status.textContent=stopNotice;status.className='ops-status ops-error';}}
    finally{stopping=false;if(!destroyed)gates();}
  });
  refreshButton.addEventListener('click',()=>void refresh());retryButton.addEventListener('click',()=>{if(!retryButton.disabled&&uncertain)void submit(copy(uncertain));});
  reloadButton.addEventListener('click',()=>location?.reload?.());
  const controller={refresh,setEnvironment,get state(){return state;},get environment(){return environment;},
    elements:{container,status,identity,profile,versionWarning,reloadButton,health,cards,current,history,dialog,support,confirm,dismiss,refreshButton,retryButton,stopButton,submissionError,buttons,guidance,guidanceLink,groups,capabilityRows,dependencyRows,controlFacts,resourceRows,collectionNotice},
    destroy(){destroyed=true;generation++;if(timer!==null)unschedule(timer);endReview();environmentSelect?.removeEventListener('change',onEnvironment);delete container.workbenchOperations;}};
  container.workbenchOperations=controller;render();void refresh();return controller;
}

if(typeof document!=='undefined'&&typeof window!=='undefined'){
  for(const container of document.querySelectorAll('[data-workbench-operations]'))createOperations({document,fetch:window.fetch.bind(window),container,location:window.location});
}
