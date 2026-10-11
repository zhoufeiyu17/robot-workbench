const labels={stopped:'开发服务已停止',starting:'正在启动开发服务',ready:'开发服务已就绪',stopping:'正在停止开发服务',expired:'本次服务时间已到期',failed:'开发服务启动或运行失败'};
const guidance={stopped:'可在本次授权的剩余时间内手动启动。已保存的数据保留，旧动作不会自动续跑。',starting:'正在准备仿真、相机与工作台。就绪后可进入；启动不会连接设备或执行动作。',ready:'可以进入工作台。启动服务不等于连接设备，执行任务仍需审阅。',stopping:'正在等待自有进程退出并核验资源清退。已保存的数据保留。',expired:'本次服务不再启动。需要继续使用时，应先取得新的服务时间与资源授权。',failed:'请查看下面的原因。仅当允许启动时才可手动重试；不会自动重启。'};
export function sessionRemaining(state,elapsedMs=0){return Number.isFinite(state?.remaining_wall_s)?Math.max(0,state.remaining_wall_s-elapsedMs/1000):null;}
export function sessionDuration(seconds){if(seconds===null)return '未知';const n=Math.ceil(Math.max(0,seconds));return `${Math.floor(n/3600)?`${Math.floor(n/3600)} 小时 `:''}${Math.floor(n%3600/60)} 分 ${n%60} 秒`;}

export function createSessionMonitor({document:doc,fetch:request,performance:clock,crypto,setInterval,clearInterval,setTimeout,clearTimeout,confirm}){
  const page=!!doc.getElementById('session-management'),el=id=>doc.getElementById(id),put=(id,text)=>{const node=el(id);if(node)node.textContent=text??'';};
  let state=null,receivedAt=0,online=false,ended=false,started=false,polling=null,posting=null,sequence=0,timer=null,missing=false,notice='';
  const retiredInstances=new Set();
  function render(){
    const remaining=sessionRemaining(state,clock.now()-receivedAt),expired=remaining===0,label=labels[state?.status]||'服务状态未知';
    if(!page){
      const link=el('session-link');if(link)link.hidden=!state?.managed||missing;
      put('session-summary',online?`${label} · ${expired?'驻留时间已到，等待清退核验':`剩余 ${sessionDuration(remaining)}`}`:'开发服务状态暂不可读，打开管理页核验');return;
    }
    put('session-profile',state?.profile==='protocol_fake'?'CPU 协议 fake 验收（无GPU/物理图像）。这里不连接真机。':state?.profile==='full_fake'?'管理完整 fake 的仿真、相机与工作台服务。这里不连接真机。':'正在核验开发服务类型。这里不连接真机。');
    put('session-start',state?.profile==='protocol_fake'?'启动协议 fake':state?.profile==='full_fake'?'启动完整 fake':'启动开发服务');
    put('session-state',state?label:missing?'当前入口未提供服务管理':online?'服务状态未知':'正在核验服务…');
    put('session-message',online?state?.message:missing?'这个入口是独立工作台，未配置开发服务管理器。':'无法读取服务管理状态，正在重新核验；暂不允许启动或停止。');
    put('session-stage',state?.stage?.label?`${state.stage.label}${Number.isFinite(state.stage.elapsed_s)?` · 已用 ${sessionDuration(state.stage.elapsed_s)}`:''}`:'');
    put('session-deadline',Number.isFinite(state?.deadline_wall_ns)?new Date(state.deadline_wall_ns/1e6).toLocaleString('zh-CN',{hour12:false}):'等待核验');
    put('session-remaining',remaining===null?'等待核验':expired?(state?.cleanup_verified===true?'驻留时间已到，自有服务进程已退出':'驻留时间已到，等待后台核验清退'):sessionDuration(remaining));
    put('session-guidance',guidance[state?.status]||'状态核验通过后，才可操作开发服务。');
    put('session-error',state?.error);el('session-error').hidden=!state?.error;
    const requestMessage=notice.startsWith('启动请求已受理')&&state?.status==='ready'?'服务已就绪，可以进入工作台。':
      notice.startsWith('停止请求已受理')&&state?.cleanup_verified===true?'停止已完成，已保存的数据保留。':notice;
    put('session-request',requestMessage);
    for(const action of ['start','stop']){
      const permission=state?.allowed?.[action],reason=!online?'等待最新管理状态':posting?'正在确认上一项请求':action==='start'&&expired?'本次驻留已到期，不能再次启动':permission?.allowed===true?'':permission?.reason||'当前状态不允许此操作';
      el(`session-${action}`).disabled=!!reason;put(`session-${action}-reason`,reason);
    }
    el('session-enter').hidden=!online||state?.status!=='ready'||expired;
    // The only supported entry is same-origin; server diagnostics cannot supply a navigation URL.
    el('session-enter').href='/';
    put('session-details',state?JSON.stringify({manager_instance_id:state.manager_instance_id,profile:state.profile,generation:state.generation,status:state.status,current_run:state.current_run,stage:state.stage},null,2):'尚无可核验回执');
  }
  function accept(value){
    if(value?.schema!=='fake-session.v1'||value.managed!==true||!value.manager_instance_id||!Number.isInteger(value.generation)||!labels[value.status]||typeof value.csrf!=='string'||!value.csrf)throw Error('管理状态格式不完整');
    if(retiredInstances.has(value.manager_instance_id))return false;
    if(state?.manager_instance_id===value.manager_instance_id&&value.generation<state.generation)return false;
    if(state&&state.manager_instance_id!==value.manager_instance_id){retiredInstances.add(state.manager_instance_id);if(posting)notice='管理服务已更换，上一请求结果不再适用。请核对当前状态。';}
    state=value;receivedAt=clock.now();online=true;missing=false;render();return true;
  }
  async function poll(force=false){
    if(ended||missing||(polling&&!force))return;
    polling?.controller.abort();const flight={sequence:++sequence,controller:new AbortController()};polling=flight;
    const timeout=setTimeout(()=>flight.controller.abort(),5000);
    try{
      const response=await request('/api/session',{cache:'no-store',credentials:'same-origin',signal:flight.controller.signal});
      if(ended||flight.sequence!==sequence)return;
      if(response.status===404){missing=true;online=false;render();return;}
      if(!response.ok)throw Error(`管理状态请求失败 (${response.status})`);
      const value=await response.json();if(!ended&&flight.sequence===sequence)accept(value);
    }catch(error){if(!ended&&flight.sequence===sequence){online=false;render();}}
    finally{clearTimeout(timeout);if(polling===flight)polling=null;}
  }
  async function send(action){
    if(ended||!page||!online||posting||!['start','stop'].includes(action)||state?.allowed?.[action]?.allowed!==true||(action==='start'&&sessionRemaining(state,clock.now()-receivedAt)===0))return;
    if(action==='stop'&&!confirm('停止开发服务会中断当前 fake 任务，并保留已保存的数据。不会操作真机。确认停止？'))return;
    const instance=state.manager_instance_id,controller=new AbortController();posting=controller;
    ++sequence;polling?.controller.abort();polling=null;
    notice=action==='start'?'正在请求启动开发服务…':'正在请求停止开发服务…';render();
    const timeout=setTimeout(()=>controller.abort(),5000);
    try{
      const response=await request(`/api/session/${action}`,{method:'POST',credentials:'same-origin',headers:{'Content-Type':'application/json','X-CSRF-Token':state.csrf},body:JSON.stringify({manager_instance_id:instance,generation:state.generation,request_id:crypto.randomUUID(),...(action==='stop'?{confirm:true}:{})}),signal:controller.signal});
      const value=await response.json();if(ended||state.manager_instance_id!==instance)return;
      if(!response.ok)throw Error(typeof value.error==='string'?value.error:`请求失败 (${response.status})`);
      if(value.manager_instance_id!==instance)throw Error('管理服务已更换，请核对最新状态后重新操作');
      accept(value);notice=action==='start'?'启动请求已受理，正在等待服务就绪。':'停止请求已受理，正在等待清退核验。';
    }catch(error){if(!ended&&state.manager_instance_id===instance)notice=`请求未确认：${error.message}。正在重新核验状态，不会自动重试。`;}
    finally{clearTimeout(timeout);if(posting===controller)posting=null;if(!ended){render();void poll(true);}}
  }
  function start(){if(started||ended)return;started=true;if(page){el('session-start').addEventListener('click',()=>send('start'));el('session-stop').addEventListener('click',()=>send('stop'));}render();void poll();timer=setInterval(()=>{render();void poll();},1000);}
  function destroy(){ended=true;++sequence;polling?.controller.abort();posting?.abort();clearInterval(timer);}
  return {start,poll,send,destroy,get state(){return state;}};
}

if(typeof window!=='undefined'&&typeof document!=='undefined'&&document.getElementById('session-management')){
  const app=createSessionMonitor({document,fetch:window.fetch.bind(window),performance:window.performance,crypto:window.crypto,setInterval:window.setInterval.bind(window),clearInterval:window.clearInterval.bind(window),setTimeout:window.setTimeout.bind(window),clearTimeout:window.clearTimeout.bind(window),confirm:window.confirm.bind(window)});
  app.start();window.addEventListener('pagehide',()=>app.destroy());
}
