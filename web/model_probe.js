// Explicit recorded-input model probes. This module never calls a robot action endpoint.
import {requestId} from './session.js';
const DEPLOYMENT='__WORKBENCH_DEPLOYMENT_SHA__';
export function mountModelProbe({document:doc,fetch,setInterval,clearInterval,crypto}){
 const el=id=>doc.getElementById(id);let catalog=null,job=null,pending=false,reading=false,context='';
 const env=()=>el('environment').value;
 const visible=()=>el('work-pane').dataset.business==='inference'&&!el('work-pane').hidden;
 async function request(url,body,csrf){
  const r=await fetch(url,{cache:'no-store',...(body?{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf,'X-Workbench-Deployment':DEPLOYMENT},body:JSON.stringify(body)}:{})});
  const d=await r.json();if(!r.ok)throw Error(typeof d.error==='string'?d.error:d.error?.message||'模型验证失败');return d;
 }
 function gates(){for(const b of ['torch','triton'])el('model-probe-'+b).disabled=pending||!!job&&!['SUCCEEDED','FAILED'].includes(job.status)||!catalog?.models?.some(x=>x.id===b+'-'+el('model-probe-task').value&&x.selectable);}
 async function refresh(){
  if(reading||!visible())return;reading=true;const current=env();
  try{
   const d=await request('/api/triton/catalog?environment='+current);if(current!==env())return;catalog=d;
   el('model-probe-panel').hidden=d.model_only_probes!==true;
   if(job){job=await request('/api/triton/jobs/'+job.job_id+'?environment='+current);el('model-probe-state').textContent=job.status+' · '+(job.stage||'')+(job.error?' · '+(job.error.message||job.error):'');
    if(job.result){el('model-probe-result').textContent=JSON.stringify(job.result,null,2);el('model-probe-result').hidden=false;}}
  }catch(e){el('model-probe-state').textContent=e.message;}finally{reading=false;gates();}
 }
 async function run(backend){
  gates();if(el('model-probe-'+backend).disabled)return;pending=true;gates();const current=env();
  try{
   const state=await request('/api/environment?environment='+current);if(current!==env())throw Error('页面已切换，未提交');
   const body={environment:current,business:'inference',request_id:'probe-'+requestId(crypto),model_id:backend+'-'+el('model-probe-task').value};
   job=await request('/api/triton/infer',body,state.csrf);el('model-probe-state').textContent='验证已提交 · 使用已保存样本，不执行机器人动作';
  }catch(e){el('model-probe-state').textContent=e.message+'；若提交结果未知，请检查任务回执，不重复点击。';}finally{pending=false;gates();}
 }
 for(const backend of ['torch','triton'])el('model-probe-'+backend).addEventListener('click',()=>run(backend));
 el('model-probe-task').addEventListener('change',gates);
 const timer=setInterval(()=>{if(context!==env()){context=env();catalog=job=null;}void refresh();},2000);void refresh();
 return{destroy(){clearInterval(timer);}};
}
if(typeof window!=='undefined'&&typeof document!=='undefined'){
 const p=mountModelProbe({document,fetch:window.fetch.bind(window),setInterval,clearInterval,crypto:window.crypto});window.addEventListener('pagehide',()=>p.destroy());
}
