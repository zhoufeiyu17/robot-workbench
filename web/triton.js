// Conversion tools inside the shared inference task. Model execution uses app.js Run.
import {requestId} from './session.js';
const DEPLOYMENT='__WORKBENCH_DEPLOYMENT_SHA__';
const terminal=new Set(['READY','SUCCEEDED','COMPLETED','FAILED','CANCELLED','EXPIRED']);
export function readyModel(row){return row?.status==='READY'&&/^[a-f0-9]{64}$/.test(row.deployment_sha256||'');}
export function mountTriton({document:doc,fetch,crypto,storage,setInterval,clearInterval}){
  const el=id=>doc.getElementById(id);let catalog=null,job=null,busy=false,reading=false,context='',lastReady=null;
  const pending=new Map();
  try{for(const [key,value]of Object.entries(JSON.parse(storage?.getItem('triton-preview-pending')||'{}')))pending.set(key,value);}catch{}
  function remember(){try{storage?.setItem('triton-preview-pending',JSON.stringify(Object.fromEntries(pending)));}catch{}}
  const environment=()=>el('environment').value;
  const visible=()=>el('work-pane').dataset.business==='inference'&&!el('work-pane').hidden;
  const message=text=>el('triton-message').textContent=text;
  async function request(url,body,csrf){
    const controller=new AbortController(),timer=globalThis.setTimeout(()=>controller.abort(),8000);
    try{
      const response=await fetch(url,{signal:controller.signal,cache:'no-store',...(body?{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf,'X-Workbench-Deployment':DEPLOYMENT},body:JSON.stringify(body)}:{})});
      const value=await response.json();if(!response.ok){const error=new Error(typeof value.error==='string'?value.error:value.error?.message||'请求未通过');error.status=response.status;throw error;}return value;
    }finally{globalThis.clearTimeout(timer);}
  }
  function gates(){
    const blocked=busy||job&&!terminal.has(job.status)||pending.has(environment())||!catalog?.available;
    el('triton-source').disabled=!!blocked;
    el('triton-convert').disabled=!!blocked||!catalog.sources.some(x=>x.id===el('triton-source').value&&x.selectable===true);
  }
  async function refresh(){
    if(reading||!visible())return;reading=true;const env=environment();
    try{
      const value=await request('/api/triton/catalog?environment='+encodeURIComponent(env));if(env!==environment())return;
      catalog=value;const select=el('triton-source'),old=select.value;select.replaceChildren();
      if(!value.sources.length){const option=doc.createElement('option');option.value='';option.textContent='暂无登记来源';select.append(option);}
      for(const row of value.sources){const option=doc.createElement('option');option.value=row.id;option.textContent=row.display_name;select.append(option);}
      select.value=value.sources.some(x=>x.id===old)?old:value.sources[0]?.id||'';
      const list=el('triton-model-status');list.replaceChildren();
      for(const row of value.models){const li=doc.createElement('li');const status=row.status==='READY'&&!readyModel(row)?'待核验':row.status;li.textContent=`${row.display_name} · ${status}${row.error||row.reason?' · '+(row.error?.message||row.reason||row.error):''}`;list.append(li);}
      const uncertain=pending.get(env),recovered=(value.recent_jobs||[]).find(x=>x.request_id===uncertain?.request_id);
      if(recovered){job=recovered;pending.delete(env);remember();}
      if(!job)job=[...value.recent_jobs||[]].reverse().find(x=>x.type==='conversions')||null;
      if(job){const next=await request('/api/triton/jobs/'+encodeURIComponent(job.job_id)+'?environment='+encodeURIComponent(env));if(env!==environment())return;job=next;el('triton-job').textContent=`转换任务 ${job.job_id} · ${job.status} · ${job.stage||''}`;}
      if(job?.status==='READY'&&lastReady!==job.job_id){lastReady=job.job_id;if(!el('inference-refresh').disabled)el('inference-refresh').click();}
      message(pending.has(env)?'提交结果待核验，不自动重发。':job?.error?`${job.error.code||'FAILED'}：${job.error.message||'转换失败'}`:!value.configured?value.reason||'转换服务未配置':value.reason|| (job&&!terminal.has(job.status)?'转换进行中，等待完整数值与制品验证。':'转换完成后，在上方选择 Triton checkpoint，沿用同一 Run 审阅与停止流程。'));
    }catch(error){if(env===environment()){catalog=null;message('转换状态不可用：'+error.message+'；未自动重试。');}}
    finally{reading=false;gates();}
  }
  async function convert(){
    gates();if(el('triton-convert').disabled)return;
    let id;try{id=requestId(crypto);}catch(error){message(error.message);return;}
    const env=environment(),body={environment:env,business:'inference',request_id:'triton-'+id,source_id:el('triton-source').value};busy=true;gates();let submitted=false;
    try{
      const state=await request('/api/environment?environment='+encodeURIComponent(env));if(env!==environment()||!visible())throw new Error('页面已切换，未提交');
      pending.set(env,{request_id:body.request_id});remember();submitted=true;
      const reply=await request('/api/triton/conversions',body,state.csrf);if(env!==environment())return;
      pending.delete(env);remember();job=reply;message('转换已提交，正在核验进度。');
    }catch(error){if(error.status>=400&&error.status<500){pending.delete(env);remember();submitted=false;}if(env===environment())message((submitted?'提交结果待核验，不自动重试：':'未提交：')+error.message);}
    finally{busy=false;gates();void refresh();}
  }
  function tick(){
    if(environment()!==context){context=environment();catalog=job=null;lastReady=null;}
    if(visible()){gates();void refresh();}
  }
  el('triton-source').addEventListener('change',gates);el('triton-refresh').addEventListener('click',refresh);el('triton-convert').addEventListener('click',convert);
  const timer=setInterval(tick,2000);tick();return{refresh,destroy(){clearInterval(timer);}};
}
if(typeof window!=='undefined'&&typeof document!=='undefined'){
  let storage;try{storage=window.localStorage;}catch{}
  const panel=mountTriton({document,fetch:window.fetch.bind(window),crypto:window.crypto,storage,setInterval:window.setInterval.bind(window),clearInterval:window.clearInterval.bind(window)});
  window.addEventListener('pagehide',()=>panel.destroy());
}
