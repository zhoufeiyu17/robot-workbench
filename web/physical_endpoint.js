import {createOperationClient,jobText,phaseText,environmentSwitchReason,mountControlAlerts} from './operations.js';

// Compatibility export: addresses now resolve only through registered deployment profiles.
export function mountPhysicalEndpoint({document:doc,fetch:request,location=globalThis.location,crypto=globalThis.crypto,
  deploymentVersion='__WORKBENCH_DEPLOYMENT_SHA__',container,showControlAlerts=true,setTimeout:schedule=globalThis.setTimeout,clearTimeout:unschedule=globalThis.clearTimeout,pollMs=2000}={}){
  if(doc.workbenchEnvironmentAccess)return doc.workbenchEnvironmentAccess;
  const make=(tag,text)=>{const node=doc.createElement(tag);if(text!==undefined)node.textContent=text;return node;};
  const client=createOperationClient({fetch:request,crypto,deploymentVersion});
  const panel=make('details');panel.id='physical-endpoint-panel';panel.className='panel environment-access';
  panel.append(make('summary','环境接入 · 已登记的机器与计算端'));
  const body=make('div');body.className='manual-body';panel.append(body);
  body.append(make('p','选择已部署的环境档案。IP 用于核对登记身份；端口、路由和凭据由部署档案提供。未知地址不会自动安装服务或接管设备。'));
  const fields=make('div');fields.className='calibration-fields';body.append(fields);
  const profile=make('select');profile.id='environment-profile';
  const address=make('input');address.type='text';address.id='physical-service-address';address.autocomplete='off';address.placeholder='已登记的机器人 IP';
  const compute=make('input');compute.type='text';compute.id='compute-service-address';compute.autocomplete='off';compute.placeholder='本机计算可留空';
  for(const [label,input]of [['部署档案 / 实例',profile],['机器人地址',address],['计算端地址（可选）',compute]]){const node=make('label',label);node.append(input);fields.append(node);}
  const check=make('button','检查登记目标');body.append(check);
  const status=make('p','正在核验统一运维服务…');status.setAttribute('role','status');status.setAttribute('aria-live','polite');body.append(status);
  const retry=make('button','核对 / 重试同一提交');retry.hidden=true;body.append(retry);
  const receipt=make('a','查看操作与诊断回执');body.append(receipt);
  const dialog=make('dialog');dialog.className='environment-review';
  const title=make('h2'),identity=make('p'),impact=make('p'),supportLabel=make('label'),support=make('input');support.type='checkbox';
  supportLabel.append(support,make('span','我已现场确认双臂得到可靠支撑，允许本次操作。'));
  const reason=make('p'),cancel=make('button','取消'),confirm=make('button','确认本次操作');dialog.append(title,identity,impact,supportLabel,reason,cancel,confirm);
  const alertHost=make('div');
  const main=container||doc.querySelector('main');if(main)main.prepend(panel);else doc.body.prepend(panel);
  (main||doc.body).append(dialog); // A closed details ancestor hides a modal while its backdrop blocks the page.
  (main||doc.body).prepend(alertHost);
  const controlAlerts=showControlAlerts?mountControlAlerts({document:doc,container:alertHost,recover:()=>prepare('recover_control'),acknowledge:event_id=>client.submit(environment,{event_id,environment,device_id:state.target.device_id},state.csrf,'acknowledge-control-warning')}):null;
  const environmentSelect=doc.getElementById('environment'),connect=doc.getElementById('environment-connect'),recover=doc.getElementById('environment-recover'),exit=doc.getElementById('environment-exit'),summary=doc.getElementById('environment-operation-status');
  const initial=new URLSearchParams(location?.search||new URL(location?.href||'http://localhost/').search);
  let environment=['fake','physical'].includes(initial.get('environment'))?initial.get('environment'):(environmentSelect?.value==='physical'?'physical':'fake'),state=null,stale=true,busy=false,epoch=0,destroyed=false,timer=null,reading=null,review=null,uncertain=null,lastRequest=null,profileKey='',dirty=false,stopFlight=null,stopNotice='',targetError='';
  const listeners=new Set(),same=(a,b)=>JSON.stringify(a)===JSON.stringify(b),copy=value=>JSON.parse(JSON.stringify(value));
  let lastAction=null,exitVerified=false,entryClosed=false;
  const business=()=>doc.getElementById('operations-link')?.dataset.business||(['calibration','collection','inference','research'].includes(initial.get('business'))?initial.get('business'):'calibration');
  const operation=name=>state?.operations?.[name];
  const compatible=()=>client.compatible(state);
  function permission(name){
    if(exitVerified)return {allowed:false,reason:'工作台已退出'};
    if(stale||!state)return {allowed:false,reason:'尚未部署或无法核验统一运维服务；请查看部署要求'};
    if(!compatible())return {allowed:false,reason:'页面与管理器部署版本不一致，请重新加载页面'};
    if(busy||uncertain)return {allowed:false,reason:uncertain?'上一提交结果未知，请核对或重试同一请求':'正在提交请求'};
    const row=operation(name);return {allowed:row?.allowed===true,reason:row?.disabled_reason||row?.reason||'当前环境不支持此操作'};
  }
  function message(text){status.textContent=text;if(summary)summary.textContent=text;}
  function dismiss(){review=null;support.checked=false;if(dialog.open)dialog.close();}
  function gates(){
    controlAlerts?.setRecoveryGate(permission('recover_control'));
    if(connect)connect.textContent=!stale&&state?.runtime?.connected===true?'设备已接入':'接入设备';
    check.disabled=busy||stale||!state?.csrf;profile.disabled=busy;address.disabled=busy;compute.disabled=busy;
    for(const [button,name]of [[connect,'connect_control'],[recover,'recover_control'],[exit,'exit_workbench']])if(button){const gate=permission(name);button.disabled=!gate.allowed;button.title=gate.allowed?'':gate.reason;}
    retry.hidden=!uncertain;retry.disabled=busy||stale||!compatible()||!same(uncertain?.expected_target,state?.target);
    confirm.disabled=!review||!permission(review.action).allowed||!same(review.target,state?.target)||review.requires_support&&!support.checked;
    reason.textContent=!review?'':!same(review.target,state?.target)?'目标身份已变化，请重新检查':!permission(review.action).allowed?permission(review.action).reason:review.requires_support&&!support.checked?'请确认本次现场支撑条件':'';
    receipt.href=doc.getElementById('operations-panel')?'#operations-panel':'/operations?environment='+environment+'&business='+business();
    receipt.onclick=event=>{const panel=doc.getElementById('operations-panel');if(panel){event?.preventDefault();doc.getElementById('operations-link')?.click?.();}};
    for(const listener of listeners)listener();
  }
  function render(){
    controlAlerts?.update(state?.control_alerts,{stale});
    const rows=state?.profiles||[],key=JSON.stringify([environment,state?.selected_profile_id,rows]);
    if(key!==profileKey){
      profileKey=key;profile.replaceChildren();
      for(const row of rows.filter(row=>row.configured===true)){const option=make('option',row.label||row.id);option.value=row.id;profile.append(option);}
      profile.value=state?.selected_profile_id||'';
      if(!dirty){const selected=rows.find(row=>row.id===profile.value);address.value=selected?.robot_address||'';compute.value=selected?.compute_address||'';}
    }
    const jobs=[state?.active_job,...state?.recent_jobs||[]],job=jobs.find(row=>row?.request?.request_id===lastRequest);
    if(job?.request?.action==='exit_workbench'&&job.status==='succeeded'&&job.verification?.ok===true&&job.result?.manager_exit_pending===true)exitVerified=true;
    if(exitVerified)message(entryClosed?'工作台入口已关闭。重新使用时请运行启动脚本。':'工作台资源已退出，正在关闭管理入口…');
    else if(targetError)message(targetError);
    else if(stopNotice)message(stopNotice);
    else if(job){message((job.label||'环境操作')+' · '+jobText(job)+' · '+phaseText(job.phase)+(job.error?' · '+(job.error.message||String(job.error)):''));}
    else if(!busy&&!uncertain&&!review&&!stale){const caps=state?.capabilities||{};message('观测：'+(caps.observe?.ready===true?'可用':caps.observe?.reason||'待核验')+' · 控制：'+(state.runtime?.connected===true?'已接入':state.runtime?.connected===false?'未接入':'未知')+' · '+(caps[business()]?.reason||'业务依赖按所选工作区单独核验'));}
    gates();
  }
  async function refresh(){
    if(destroyed||entryClosed)return;if(reading?.epoch===epoch)return reading.promise;
    if(timer!==null)unschedule(timer);const at=epoch;
    const promise=(async()=>{
      try{
        const incoming=await client.read(environment);if(at!==epoch||destroyed)return;
        if(state&&(!same(state.target,incoming.target)||state.csrf!==incoming.csrf)){
          dismiss();uncertain=null;stopNotice='';
          if(lastAction!=='exit_workbench'||state.target?.manager_instance!==incoming.target?.manager_instance||state.target?.device_id!==incoming.target?.device_id){lastRequest=null;lastAction=null;exitVerified=false;}
        }
        state=incoming;stale=incoming.health?.freshness?.valid===false;
        if(review){const row=operation(review.action);if(!same(review.signature,[row?.label,row?.impact,row?.requires_support]))dismiss();}
        if(uncertain&&[incoming.active_job,...incoming.recent_jobs||[]].some(row=>row?.request?.request_id===uncertain.request_id))uncertain=null;
        render();return incoming;
      }catch(error){if(at===epoch&&!destroyed){stale=true;controlAlerts?.update(state?.control_alerts,{stale});
        if(exitVerified){entryClosed=true;message('工作台入口已关闭。重新使用时请运行启动脚本。');}
        else message((targetError?targetError+'。':'')+'环境状态未知：'+error.message+'。最后记录不能证明当前可用。');gates();}}
      finally{if(reading?.epoch===at)reading=null;if(at===epoch&&!destroyed&&!entryClosed&&pollMs>0)timer=schedule(()=>void refresh(),pollMs);}
    })();reading={epoch:at,promise};return promise;
  }
  async function submit(body){
    if(busy||destroyed)return;const at=epoch;busy=true;targetError='';dismiss();message('正在提交环境操作；执行结果以后置核验为准。');gates();
    try{const result=await client.submit(environment,body,state.csrf);if(at!==epoch||destroyed)return;lastRequest=body.request_id;lastAction=body.action;uncertain=null;message('操作已受理 · '+jobText(result.job||result));return result;}
    catch(error){if(at===epoch&&!destroyed){if(!error.rejected){uncertain=copy(body);lastRequest=body.request_id;lastAction=body.action;}message((uncertain?'提交结果未知；核对或重试将沿用同一请求编号。 ':'操作未受理：')+error.message);}}
    finally{busy=false;if(at===epoch&&!destroyed){stale=true;controlAlerts?.update(state?.control_alerts,{stale});gates();void refresh();}}
  }
  function prepare(action,parameters){
    const gate=permission(action);if(!gate.allowed){message(gate.reason);return;}
    targetError='';
    const row=operation(action);
    if(row.requires_support||['connect_control','recover_control','release_control','verify_hold','exit_workbench','exit_environment'].includes(action)){
      dismiss();review={action,parameters,target:copy(state.target),requires_support:row.requires_support===true,signature:[row.label,row.impact,row.requires_support]};
      title.textContent=row.label||action;identity.textContent=`${state.target.environment} · ${state.target.device_id||state.target.id||'设备未登记'}`;impact.textContent=row.impact||'请核对操作的实际影响。';supportLabel.hidden=!review.requires_support;gates();dialog.showModal();return;
    }
    return submit({action,request_id:client.id(),expected_target:copy(state.target),arms_supported:false,...(parameters?{parameters}: {})});
  }
  async function resolveTarget(andConnect=false,{page='/'}={}){
    if(busy||stale||!state?.csrf)return;busy=true;dismiss();gates();const at=epoch;
    try{
      const selected=(state.profiles||[]).find(row=>row.id===profile.value);if(!selected)throw Error('请选择已登记的部署档案；同 IP 多实例必须明确选择');
      if(selected.environment!==environment){const reason=environmentSwitchReason(state,{stale,pending:!!uncertain});if(reason)throw Error(reason);}
      const body={profile_id:selected.id,expected_target:copy(state.target),...(address.value.trim()?{robot_address:address.value.trim()}:{}),...(compute.value.trim()?{compute_address:compute.value.trim()}: {})};
      const result=await client.submit(environment,body,state.csrf,'target');if(at!==epoch||destroyed)return;
      if(result.redirect_url){
        const target=new URL(result.redirect_url,location.href),registered=new URL(selected.manager_url,location.href);
        if(!['http:','https:'].includes(target.protocol)||target.username||target.password||target.origin!==registered.origin||target.pathname!=='/'||target.hash
          ||result.environment?.schema!=='robot_workbench.environment.v1'||result.environment?.target?.environment!==selected.environment
          ||result.environment?.target?.device_id!==selected.device_id)throw Error('目标入口与已登记档案不匹配，未导航');
        target.pathname=page==='/operations'?'/operations':'/';
        target.searchParams.set('environment',selected.environment);target.searchParams.set('business',business());
        targetError='';message('目标已核验；将进入登记的管理服务，重新读取身份后再接入控制。');location.assign(target.href);return;
      }
      if(result.schema!=='robot_workbench.environment.v1'||result.target?.environment!==environment||!result.csrf)throw Error('目标核验没有返回匹配的运维身份');
      state=result;stale=result.health?.freshness?.valid===false;dirty=false;profileKey='';targetError='';message('目标已核验；尚未接入设备控制。');render();
      busy=false;if(andConnect)prepare('connect_control');
    }catch(error){if(at===epoch&&!destroyed){targetError='目标未接入：'+error.message;message(targetError);}}
    finally{busy=false;if(at===epoch&&!destroyed)gates();}
  }
  function setEnvironment(next){
    if(next===environment)return;environment=next;epoch++;state=null;stale=true;uncertain=null;lastRequest=null;lastAction=null;exitVerified=false;entryClosed=false;profileKey='';dirty=false;stopNotice='';targetError='';dismiss();message('正在核验所选环境…');gates();void refresh();
  }
  async function navigateEnvironment(next,{page='/'}={}){
    if(next===environment)return;
    const reason=environmentSwitchReason(state,{stale,pending:busy||!!uncertain||!!stopFlight});
    if(reason){targetError=reason;message(reason);return;}
    const rows=(state.profiles||[]).filter(row=>row.environment===next&&row.configured===true);
    if(rows.length!==1){targetError='所选环境没有唯一已登记管理服务，请核验部署档案';message(targetError);return;}
    profile.value=rows[0].id;address.value=rows[0].robot_address||'';compute.value=rows[0].compute_address||'';dirty=true;
    return resolveTarget(false,{page});
  }
  profile.addEventListener('change',()=>{dirty=true;targetError='';const row=state?.profiles?.find(row=>row.id===profile.value);address.value=row?.robot_address||'';compute.value=row?.compute_address||'';dismiss();render();});
  for(const input of [address,compute])input.addEventListener('input',()=>{dirty=true;targetError='';dismiss();render();});
  check.addEventListener('click',()=>void resolveTarget());connect?.addEventListener('click',()=>void resolveTarget(true));recover?.addEventListener('click',()=>prepare('recover_control'));
  exit?.addEventListener('click',()=>prepare('exit_workbench'));
  support.addEventListener('change',gates);cancel.addEventListener('click',dismiss);dialog.addEventListener('cancel',dismiss);
  confirm.addEventListener('click',()=>{gates();if(confirm.disabled)return;const value=review;void submit({action:value.action,request_id:client.id(),expected_target:copy(value.target),arms_supported:value.requires_support&&support.checked,...(value.parameters?{parameters:value.parameters}: {})});});
  retry.addEventListener('click',()=>{if(!retry.disabled&&uncertain)void submit(copy(uncertain));});
  async function stop(){
    if(stopFlight)return stopFlight;if(!state?.csrf||!state?.target?.device_id)throw Error('独立停止目标身份尚未核验');
    const at=epoch;dismiss();targetError='';stopNotice='正在请求独立停止；等待设备回执。';message(stopNotice);
    const flight=client.stop(environment,state.target,state.csrf);stopFlight=flight;gates();
    try{const result=await flight;if(at===epoch&&!destroyed){stopNotice='停止请求回执（机械静止须另行核验）：'+JSON.stringify(result);message(stopNotice);}return result;}
    catch(error){if(at===epoch&&!destroyed){stopNotice='停止结果未知：'+error.message+'；请核对现场与独立停止设备。';message(stopNotice);}throw error;}
    finally{if(stopFlight===flight)stopFlight=null;if(!destroyed)gates();}
  }
  const api={panel,refresh,prepare,permission,setEnvironment,navigateEnvironment,stop,get stopping(){return !!stopFlight;},get exited(){return exitVerified;},get exitState(){return entryClosed?'closed':exitVerified?'closing':null;},get state(){return state;},get registry(){return state;},subscribe(fn){listeners.add(fn);return ()=>listeners.delete(fn);},
    elements:{profile,address,compute,check,status,retry,dialog,support,confirm,cancel},destroy(){destroyed=true;epoch++;if(timer!==null)unschedule(timer);dismiss();listeners.clear();delete doc.workbenchEnvironmentAccess;}};
  doc.workbenchEnvironmentAccess=api;gates();void refresh();return api;
}
