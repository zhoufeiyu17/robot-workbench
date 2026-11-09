import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../web/operations.js',import.meta.url),'utf8');
const {createOperations,operationsGuidance,environmentSwitchReason,mountControlAlerts}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));

class Element{
  constructor(tag){this.tag=tag;this.children=[];this.events={};this.dataset={};this.value='';this._text='';this.hidden=false;this.disabled=false;this.checked=false;this.open=false;this.classList={add:name=>{this.className=name;}};}
  set textContent(value){this._text=String(value??'');this.children=[];}get textContent(){return this._text+this.children.map(child=>child.textContent).join(' ');}
  set innerHTML(_value){throw Error('untrusted HTML must never be inserted');}
  append(...nodes){this.children.push(...nodes);}replaceChildren(...nodes){this._text='';this.children=nodes;}
  addEventListener(name,callback){(this.events[name]??=[]).push(callback);}removeEventListener(name,callback){this.events[name]=(this.events[name]||[]).filter(value=>value!==callback);}
  setAttribute(name,value){this[name]=value;}showModal(){this.open=true;}close(){this.open=false;}
  fire(name){for(const callback of this.events[name]||[])callback({target:this,preventDefault(){}});}
}
const settle=async()=>{for(let index=0;index<24;index++)await Promise.resolve();};
const fixture=(environment='physical',manager='manager-v1')=>({schema:'robot_workbench.environment.v1',csrf:'secret-token',target:{environment,id:'cpu-'+environment,device_id:'cpu-'+environment,manager_instance:manager},
  deployment:{sha256:'a'.repeat(64),source_files:12},
  health:{workbench:{status:'unreachable',reason:'业务页面不可达；管理器独立可用'},cameras:{status:'running'}},busy:false,active_job:null,recent_jobs:[],
  operations:{status_check:{label:'核验服务状态',impact:'只读检查',allowed:true},restart_workbench:{label:'重启工作台',impact:'将重启本部署工作台；现有业务会中断',allowed:true,requires_support:true},
    restart_cameras:{label:'重启相机',impact:'会暂时中断三路画面',allowed:false,reason:'正在录制'},prepare_model:{label:'准备模型',impact:'加载当前登记模型',allowed:true},
    restart_detector:{label:'重启检测服务',impact:'只重启检测，数采待预热',allowed:true},
    restart_planner:{label:'重启规划服务',impact:'保留健康场景模型，数采待预热',allowed:true},
    restart_scene:{label:'重启场景服务',impact:'只重启场景，数采待预热',allowed:true},
    unknown_shell:{label:'不得出现的未知操作',allowed:true}}});

{
 const value=fixture();value.recent_jobs=[{request:{action:'exit_environment'},status:'succeeded',verification:{ok:true}}];
 value.services=Object.fromEntries(['workbench','control_owner','cameras','model'].map(name=>[name,{state:'stopped',running:false}]));
 assert.equal(operationsGuidance(value).title,'工作台环境已退出');
 const shared=structuredClone(value);
 shared.recent_jobs[0].verification.consumer_detached=true;
 for(const role of ['control_owner','cameras','model'])shared.services[role]={state:'running',running:true};
 assert.equal(operationsGuidance(shared).title,'已退出本工作台');
 assert.match(operationsGuidance(shared).message,/没有启动原数采/);
 shared.services.workbench={state:'running',running:true};
 assert.notEqual(operationsGuidance(shared).title,'已退出本工作台');
 value.services.model={state:'running',running:true};assert.notEqual(operationsGuidance(value).title,'工作台环境已退出','historical exit cannot override a currently running resource');
}
function harness({environment='physical',standalone=false,query='',crypto={randomUUID:()=> 'request-fixed'}}={}){
  const nodes=[],container=new Element('section'),select=new Element('select'),entry=new Element('a'),back=new Element('a');select.value=environment;
  if(standalone)container.dataset.operationsStandalone='true';
  const doc={getElementById:id=>id==='environment'?select:id==='operations-link'?entry:id==='operations-return'?back:null,querySelector:()=>container,createElement:tag=>{const node=new Element(tag);nodes.push(node);return node;}};
  const requests=[],navigations=[];
  const api=createOperations({document:doc,container,location:{search:query,assign:url=>navigations.push(url)},crypto,pollMs:0,deploymentVersion:'a'.repeat(64),
    fetch:(url,options={})=>new Promise((resolve,reject)=>requests.push({url,options,resolve,reject}))});
  const reply=(index,value,ok=true)=>requests[index].resolve({ok,status:ok?200:409,json:async()=>value});
  const button=name=>api.elements.buttons.get(name);
  const last=()=>requests.length-1;
  return {api,container,select,entry,back,nodes,requests,reply,button,last,doc,navigations};
}

for(const complete of [true,false]){
  const h=harness(),initial=fixture();
  initial.operations.exit_workbench={label:'退出工作台',impact:'正常回零释放后关闭服务',allowed:true};
  h.reply(0,initial);await settle();h.button('exit_workbench').fire('click');h.api.elements.confirm.fire('click');
  const posted=JSON.parse(h.requests[h.last()].options.body);
  h.reply(h.last(),{job_id:'exit-job',status:'queued',request:posted});await settle();
  const finished=structuredClone(initial);finished.target.backend_instance=null;
  finished.recent_jobs=[{job_id:'exit-job',request:posted,status:complete?'succeeded':'unknown',verification:{ok:complete,environment_exited:complete,sdk_release_verified:complete},result:{manager_exit_pending:complete}}];
  h.reply(h.last(),finished);await settle();assert.equal(h.navigations.length,0);
  h.api.refresh();h.requests[h.last()].reject(Error('closed'));await settle();
  assert.match(h.container.textContent,complete?/入口已关闭/:/无法核验管理服务/);h.api.destroy();
}

for(const mode of ['verified','lan-http','unknown','manager-changed','runtime-changed','formal','formal-unreleased']){
  const verified=mode==='verified'||mode==='lan-http'||mode==='formal';
  const destination=mode.startsWith('formal')?'http://10.17.32.5:8080/':'http://10.17.32.5:18080/';
  const h=harness(mode==='lan-http'?{crypto:{getRandomValues:array=>array.fill(7)}}:{}),initial=fixture();
  initial.operations.exit_environment={label:'切换到数采工作台',impact:'归还本工作台控制后打开数采',allowed:true};
  h.reply(0,initial);await settle();
  h.button('exit_environment').fire('click');h.api.elements.confirm.fire('click');
  const posted=JSON.parse(h.requests[h.last()].options.body);
  h.reply(h.last(),{job_id:'switch-job',status:'queued',request:posted});await settle();
  const finished=structuredClone(initial);
  finished.target.backend_instance=null;finished.target.command_epoch=9;
  if(mode==='manager-changed')finished.target.manager_instance='replacement';
  finished.services={workbench:{state:'stopped',running:false}};
  finished.recent_jobs=[{job_id:'switch-job',request:posted,status:mode==='unknown'?'unknown':'succeeded',
    verification:{ok:mode!=='unknown',consumer_detached:true,independent_services_preserved:true,shared_runtime_preserved:mode!=='runtime-changed'},
    result:{next_url:destination}}];
  if(mode.startsWith('formal'))finished.recent_jobs[0].verification={ok:true,environment_exited:true,collection_web_ready:true,sdk_release_verified:mode==='formal'};
  h.reply(h.last(),finished);await settle();
  assert.deepEqual(h.navigations,verified?[destination]:[]);
  const refresh=h.api.refresh();h.reply(h.last(),finished);await refresh;
  assert.equal(h.navigations.length,verified?1:0,'a verified user handoff navigates at most once');
  h.api.destroy();
  const reopened=harness();reopened.reply(0,finished);await settle();
  assert.equal(reopened.navigations.length,0,'historical success must never redirect a newly opened page');
  reopened.api.destroy();
}

{
  const returned=fixture();
  returned.services={workbench:{state:'stopped',running:false},control_owner:{state:'running',lifecycle:'external'}};
  returned.operations.connect_control={allowed:true};
  returned.recent_jobs=[{request:{action:'exit_environment'},status:'succeeded',verification:{ok:true,consumer_detached:true}}];
  assert.equal(operationsGuidance(returned).action,'connect_control','a returned user can attach instead of following a historical exit back to collection');
}

{
  const h=harness(),summary=fixture();
  summary.projection='manager_environment_summary';summary.runtime={connected:false};summary.services={};
  const job={job_id:'job-1',status:'succeeded',phase:'complete',label:'核验',request:{request_id:'read-1'},verification:{ok:true},detail_url:'/api/operations/jobs/job-1'};
  summary.jobs={'job-1':job};summary.recent_jobs=['job-1'];summary.active_job=null;
  for(const operation of Object.values(summary.operations))operation.latest_job='job-1';
  h.reply(0,summary);await settle();
  assert.match(h.api.elements.history.textContent,/核验/);
  assert.equal(h.api.state.recent_jobs[0].job_id,'job-1');
  assert.equal(h.api.state.health.runtime,h.api.state.runtime,'wire aliases share the same restored facts');
  h.api.destroy();
}

{
  const h=harness(),value=fixture();
  value.operations.reclaim_control={label:'清理现有控制并接入',impact:'正常结束数采，保留数据；不强杀、不续跑旧动作',allowed:true,requires_support:true};
  h.reply(0,value);await settle();
  assert(h.button('reclaim_control'),'the supported reclaim action must be visible');
  h.button('reclaim_control').fire('click');
  assert.equal(h.api.elements.confirm.disabled,true);
  assert.match(h.api.elements.dialog.textContent,/正常结束数采/);
  h.api.elements.support.checked=true;h.api.elements.support.fire('change');
  h.api.elements.confirm.fire('click');await settle();
  const payload=JSON.parse(h.requests[h.last()].options.body);
  assert.equal(payload.action,'reclaim_control');assert.equal(payload.arms_supported,true);
  assert.deepEqual(payload.expected_target,value.target);
  h.reply(h.last(),{job_id:'reclaim-cpu',status:'succeeded',phase:'complete',verification:{ok:true},request:payload});await settle();
  h.api.destroy();
}

for(const environment of ['fake','physical']){
  const h=harness({environment}),value=fixture(environment);
  value.collection={services_ready:true,ready:false,declared_roles:['detector','planner'],
    warmup:{status:'failed',failure:{stage:'scene_setup',native_reason:'reset characterization rejected: cup_table_recontact_or_invalid_support'}}};
  h.reply(0,value);await settle();
  assert.equal(h.api.elements.collectionNotice.hidden,false);
  assert.match(h.api.elements.collectionNotice.textContent,/常驻服务可用/);
  assert.match(h.api.elements.collectionNotice.textContent,/任务场景准备失败/);
  assert.match(h.api.elements.collectionNotice.textContent,/业务初始化与预热尚未完成.*数采页/);
  const refresh=h.api.refresh();h.reply(h.last(),{error:'offline'},false);await refresh;
  assert.match(h.api.elements.collectionNotice.textContent,/当前状态未知；上次记录/);
  value.collection={services_ready:true,ready:true,warmup:{status:'ready'}};
  const ready=h.api.refresh();h.reply(h.last(),value);await ready;
  assert.equal(h.api.elements.collectionNotice.hidden,true,'healthy preparation must not show an amber failure notice');
  h.api.destroy();
}

for(const environment of ['fake','physical']){
  const h=harness({environment});h.reply(0,fixture(environment));await settle();
  assert.equal(h.entry.href,'#operations-panel');
  assert.equal(h.api.elements.groups.length,4);assert.equal(h.api.elements.groups[2].node.open,false);
  for(const role of ['detector','planner','scene'])assert(h.button('restart_'+role),'registered role restart stays in the existing advanced group');
  assert.equal(h.requests[0].url,`/api/environment?environment=${environment}&view=summary`);
  assert.equal(h.button('restart_cameras').disabled,true);assert.match(h.api.elements.cards.textContent,/正在录制/);
  assert(!h.button('unknown_shell'),'unknown backend actions must not create buttons');
  h.button('restart_workbench').fire('click');assert.equal(h.api.elements.dialog.open,true);assert.equal(h.api.elements.confirm.disabled,true);
  assert.match(h.api.elements.dialog.textContent,/现有业务会中断/);assert.match(h.api.elements.dialog.textContent,new RegExp('cpu-'+environment));
  h.api.elements.support.checked=true;h.api.elements.support.fire('change');assert.equal(h.api.elements.confirm.disabled,false);
  const refresh=h.api.refresh();h.reply(h.last(),fixture(environment));await refresh;
  assert.equal(h.api.elements.support.checked,true,'ordinary polling preserves the current confirmation draft');
  h.api.elements.confirm.fire('click');h.api.elements.confirm.fire('click');
  const posts=h.requests.filter(row=>row.options.method==='POST');assert.equal(posts.length,1,'double click sends one request');
  assert.equal(posts[0].url,`/api/operations/action?environment=${environment}`);
  assert.equal(posts[0].options.credentials,'same-origin');assert.equal(posts[0].options.headers['X-CSRF-Token'],'secret-token');
  assert.equal(posts[0].options.headers['X-Workbench-Deployment'],'a'.repeat(64));
  const body=JSON.parse(posts[0].options.body);assert.deepEqual(Object.keys(body).sort(),['action','arms_supported','expected_target','request_id']);
  assert.equal(body.arms_supported,true);assert.equal(body.expected_target.environment,environment);assert.equal(h.api.elements.support.checked,false);
  posts[0].resolve({ok:true,json:async()=>({job:{job_id:'j1',label:'重启工作台',request:body,status:'running',phase:'restart',progress:.3}})});await settle();
  h.reply(h.last(),fixture(environment));await settle();
  assert(h.requests.every(row=>!row.url.includes('/api/state')&&!row.url.includes('/api/actions/')),'management must not depend on business runtime state');
  assert(!h.container.textContent.includes('secret-token'),'CSRF never enters rendered content');
  h.api.destroy();
}

{
  const ready={health:{backend_reachable:true,services:{model:{state:'running'}},model_service:{status:'ready',warmup:{completed:true}},runtime:{connected:false}},current_capabilities:{control_connected:false,camera_frames:true,model_loaded:true,workflow_model_bound:true}};
  assert.match(operationsGuidance(ready).title,/观测可用.*未接入/);
  assert.equal(operationsGuidance(ready).workspace,true);
  assert.match(operationsGuidance(ready,true).title,/尚未核验/);
  assert.equal(operationsGuidance({...ready,recovery:{required:true}}).action,'recover_maintenance');
  assert.equal(operationsGuidance({...ready,active_job:{label:'重启相机',phase:'waiting_camera_source'}}).current,true);
  const warming=operationsGuidance({...ready,recovery:{required:true},active_job:{label:'准备当前环境',status:'running',phase:'warming_model'}});
  assert.equal(warming.current,true);assert.equal(warming.action,undefined);assert.match(warming.title,/正在执行/);assert.match(warming.message,/预热模型/);
  assert.match(operationsGuidance({...ready,recovery:{required:true},active_job:{status:'timeout_unknown'}}).title,/状态未知/);
  assert.equal(operationsGuidance({...ready,current_capabilities:{...ready.current_capabilities,workflow_model_bound:false}}).action,'prepare_model');
  assert.equal(operationsGuidance({...ready,health:{...ready.health,services:{cameras:{state:'stopped',error:'左腕首帧超时'}}},current_capabilities:{camera_frames:false}}).message,'左腕首帧超时');
  const h=harness();const value=fixture();value.recovery={required:true};value.operations.recover_maintenance={label:'结束遗留维护',allowed:true};h.reply(0,value);await settle();
  h.api.elements.guidanceLink.fire('click');assert.equal(h.api.elements.guidanceLink.href.endsWith('-action-recover_maintenance'),true);assert.equal(h.requests.length,1,'next-step navigation cannot submit an operation');h.api.destroy();
}

for(const environment of ['fake','physical']){
  const h=harness({environment:environment==='fake'?'physical':'fake',query:'?environment='+environment});
  assert.equal(h.api.environment,environment,'embedded operations must use the explicit page environment before the app initializes');
  assert.equal(h.select.value,environment);assert.equal(h.requests[0].url,`/api/environment?environment=${environment}&view=summary`);
  assert(h.requests.every(row=>row.options.method!=='POST'),'initial environment selection only reads state');h.api.destroy();
}

for(const business of ['inference','collection','constructor']){
  const h=harness({standalone:true,query:'?environment=fake&business='+business});
  const valid=business==='constructor'?'calibration':business;
  assert.equal(h.back.href,'/?environment=fake&business='+valid);
  const state=fixture('fake');state.health={backend_reachable:true};state.current_capabilities={camera_frames:true,control_connected:false};
  h.reply(0,state);await settle();
  assert.equal(h.api.elements.guidanceLink.href,h.back.href);
  h.api.elements.guidanceLink.fire('click');
  assert(h.requests.every(row=>row.options.method!=='POST'&&!row.url.includes('business=')));
  h.entry.dataset.business='inference';
  const refresh=h.api.refresh();h.reply(h.last(),state);await refresh;
  assert.equal(h.entry.href,'/operations?environment=fake&business=inference','standalone rescue page preserves business');
  assert.equal(h.api.elements.dialog.open,false);h.api.destroy();
}

{
  const h=harness({standalone:true,query:'?environment=fake'});assert.equal(h.select.value,'fake');assert.equal(h.api.environment,'fake');
  h.reply(0,fixture('fake'));await settle();h.button('status_check').fire('click');assert.equal(h.api.elements.dialog.open,false);
  assert.equal(JSON.parse(h.requests[1].options.body).action,'status_check');h.api.destroy();
  const defaults=harness({standalone:true,environment:'fake'});assert.equal(defaults.api.environment,'physical');defaults.api.destroy();
}

{
  const h=harness();h.reply(0,fixture());await settle();h.button('restart_workbench').fire('click');h.api.elements.support.checked=true;h.api.elements.support.fire('change');
  const refresh=h.api.refresh();h.reply(h.last(),fixture('physical','manager-v2'));await refresh;
  assert.equal(h.api.elements.dialog.open,false);assert.equal(h.api.elements.support.checked,false);assert.match(h.api.elements.status.textContent,/旧确认已撤销/);
  h.button('restart_workbench').fire('click');h.api.elements.support.checked=true;
  h.select.value='fake';h.select.fire('change');assert.equal(h.api.elements.dialog.open,false);assert.equal(h.api.elements.support.checked,false);
  assert.equal(h.entry.href,'#operations-panel');
  h.reply(h.last(),fixture('fake'));await settle();assert.equal(h.api.environment,'fake');assert.match(h.api.elements.identity.textContent,/cpu-fake/);h.api.destroy();
}

{
  const h=harness();h.select.value='fake';h.select.fire('change');h.reply(1,fixture('fake'));await settle();
  h.reply(0,fixture('physical'));await settle();assert.equal(h.api.state.target.environment,'fake','late old-environment responses cannot replace the new target');h.api.destroy();
}

{
  const h=harness(),initial=fixture();initial.current_capabilities={control_connected:true,model_loaded:true,camera_frames:true};
  h.reply(0,initial);await settle();h.button('restart_workbench').fire('click');h.api.elements.support.checked=true;h.api.elements.support.fire('change');
  const refresh=h.api.refresh();h.requests[h.last()].reject(Error('connection refused'));await refresh;
  assert.equal(h.api.elements.confirm.disabled,true);assert.equal(h.api.elements.dialog.open,true);assert.equal(h.api.elements.support.checked,true);
  assert.equal(h.button('prepare_model').disabled,true);assert.match(h.api.elements.status.textContent,/不能据此判断/);
  assert.match(h.api.elements.health.textContent,/上次：运行中/);assert.doesNotMatch(h.api.elements.health.textContent,/当前可用/);
  const recovery=h.api.refresh();h.reply(h.last(),fixture());await recovery;assert.equal(h.api.elements.confirm.disabled,false);h.api.destroy();
}

{
  const h=harness();h.reply(0,fixture());await settle();h.button('prepare_model').fire('click');h.api.elements.confirm.fire('click');
  const submitted=JSON.parse(h.requests[1].options.body);h.requests[1].reject(Error('response lost'));await settle();
  h.reply(h.last(),fixture());await settle();assert.equal(h.api.elements.retryButton.hidden,false);assert.equal(h.button('prepare_model').disabled,true);
  h.api.elements.retryButton.fire('click');assert.deepEqual(JSON.parse(h.requests[h.last()].options.body),submitted,'ambiguous submission retries the exact id and payload');
  const job={job_id:'lost-request',request:submitted,status:'succeeded',verification:{ok:true},phase:'complete'};
  h.reply(h.last(),job);await settle();const state=fixture();state.recent_jobs=[job];h.reply(h.last(),state);await settle();
  assert.equal(h.api.elements.retryButton.hidden,true);assert.match(h.api.elements.history.textContent,/后置核验通过/);
  assert.match(h.api.elements.current.textContent,/当前没有管理操作/);h.api.destroy();
}

{
  const h=harness();h.reply(0,fixture());await settle();h.button('prepare_model').fire('click');h.api.elements.confirm.fire('click');
  h.reply(1,{error:'<img src=x onerror=alert(1)> rejected'},false);await settle();h.reply(h.last(),fixture());await settle();
  assert.equal(h.api.elements.submissionError.hidden,false);assert.match(h.api.elements.submissionError.textContent,/<img src=x onerror=alert\(1\)>/);
  assert.equal(h.api.elements.retryButton.hidden,true,'explicit rejection does not become an ambiguous retry');h.api.destroy();
}

{
  const h=harness();const state=fixture();state.active_job={job_id:'slow',request:{request_id:'slow-id',action:'restart_workbench'},label:'重启工作台',status:'timeout_unknown',phase:'verify',progress:.6,cancellable:false};
  state.recent_jobs=[{job_id:'old',request:{request_id:'old-id'},status:'interrupted_unknown',phase:'recovery'},
    {job_id:'bad-success',request:{request_id:'bad-id'},status:'succeeded',verification:{ok:false}}];
  h.reply(0,state);await settle();assert.match(h.api.elements.current.textContent,/不能认为服务已退出/);assert.match(h.api.elements.current.textContent,/不支持取消/);
  assert.match(h.api.elements.current.textContent,/阶段：核验实际结果/);
  assert.match(h.api.elements.history.textContent,/管理器曾中断/);assert.match(h.api.elements.history.textContent,/后置核验尚未通过/);
  const refresh=h.api.refresh();state.active_job={...state.active_job,status:'running',cancellable:true};h.reply(h.last(),state);await refresh;
  const cancel=h.api.elements.current.children.find(node=>node.tag==='article').children.find(node=>node.tag==='button');cancel.fire('click');assert.equal(h.api.elements.dialog.open,true);
  h.api.elements.confirm.fire('click');assert.match(h.requests[h.last()].url,/\/cancel\?environment=physical$/);assert.deepEqual(JSON.parse(h.requests[h.last()].options.body),{request_id:'slow-id'});h.api.destroy();
}

{
  const h=harness({crypto:{}});h.reply(0,fixture());await settle();h.button('status_check').fire('click');
  assert.match(JSON.parse(h.requests[1].options.body).request_id,/^ops-[a-z0-9-]+$/);h.api.destroy();
}

const html=await readFile(new URL('../web/operations.html',import.meta.url),'utf8');
for(const environment of ['fake','physical']){
  const h=harness({environment}),state=fixture(environment);
  for(const role of ['head','left','right'])state.operations['reset_camera_'+role]={label:'软件重置'+role+'相机',
    impact:'只复位所选物理设备；三路画面会暂时暂停，机器人SDK保持不变。',allowed:environment==='physical',
    disabled_reason:environment==='fake'?'Fake 没有物理USB相机；此操作不支持仿真':null};
  h.reply(0,state);await settle();
  for(const role of ['head','left','right'])assert.equal(h.button('reset_camera_'+role).disabled,environment==='fake');
  if(environment==='physical'){
    h.button('reset_camera_left').fire('click');assert.equal(h.api.elements.dialog.open,true);
    assert.match(h.api.elements.dialog.textContent,/三路画面会暂时暂停/);h.api.elements.confirm.fire('click');
    const body=JSON.parse(h.requests[1].options.body);assert.equal(body.action,'reset_camera_left');
    assert.equal(body.serial,undefined);assert.equal(body.command,undefined);assert.equal(body.path,undefined);
  }else assert.match(h.api.elements.cards.textContent,/不支持仿真/);
  h.api.destroy();
}
{
  const h=harness();const state=fixture();
  state.machine_profile={identity:{device_id:'declared-machine',environment:'physical'},
    declared:{label:'<img src=x onerror=alert(1)>',host_name:'configured-host',hardware:{robot_model:'A1',robot_serial:'declared-serial',
      cameras:{head:{serial:'camera-123',intrinsics_index:'/trusted/head.json'}},disabled_grippers:['right']},
      known_degradations:[{capability:'right_gripper',reason:'部署已停用'}]},
    calibration_indexes:{active_manifest_index:{path:'/trusted/manifest.json',status:'file_present',sha256:'c'.repeat(64)}}};
  state.deployment.sha256='b'.repeat(64);h.reply(0,state);await settle();
  assert.match(h.api.elements.profile.textContent,/declared-machine/);assert.match(h.api.elements.profile.textContent,/不代表现场硬件已核验/);
  assert.match(h.api.elements.profile.textContent,/camera-123/);assert.match(h.api.elements.profile.textContent,/部署已停用/);
  assert.match(h.api.elements.profile.textContent,/<img src=x onerror=alert\(1\)>/);
  assert.equal(h.button('prepare_model').disabled,true);assert.equal(h.button('status_check').disabled,false);
  assert.equal(h.api.elements.reloadButton.hidden,false);assert.match(h.api.elements.versionWarning.textContent,/版本不一致/);
  h.button('prepare_model').fire('click');assert.equal(h.requests.length,1);
  const refresh=h.api.refresh();state.deployment.sha256='a'.repeat(64);delete state.deployment.files_unchanged;
  h.reply(h.last(),state);await refresh;assert.equal(h.button('prepare_model').disabled,false);
  assert.equal(h.api.elements.versionWarning.hidden,true);h.api.destroy();
}
for(const environment of ['fake','physical']){
  const h=harness({environment});const state=fixture(environment);
  state.operations.recover_maintenance={label:'核验并结束遗留维护',impact:'核验旧动作已结束后解除冻结；不恢复运动',allowed:true};
  h.reply(0,state);await settle();h.button('recover_maintenance').fire('click');
  assert.equal(h.api.elements.dialog.open,true);assert.match(h.api.elements.dialog.textContent,/不恢复运动/);
  assert.equal(h.api.elements.support.checked,false);h.api.elements.confirm.fire('click');
  const body=JSON.parse(h.requests[1].options.body);
  assert.equal(body.action,'recover_maintenance');assert.equal(body.expected_target.environment,environment);assert.equal(body.arms_supported,false);
  h.api.destroy();
}
assert(html.includes('data-workbench-operations'));assert(html.includes('/web/operations.js'));assert(!html.includes('/web/app.js'));
{
  const h=harness(),state=fixture();state.health={backend_reachable:true,services:{model:{state:'running'}}};
  state.runtime={connected:false,holder:null,state:'DISCONNECTED',release_receipt:{status:'unconfirmed',message:'SDK close 未确认'}};
  state.capabilities={observe:{status:'ready',ready:true,reason:'三路新鲜帧',support:'supported',checked_wall_ns:1000000000},control:{status:'blocked',ready:false,reason:'尚未接入'},calibration:{status:'unknown',ready:null,reason:'未核验'},collection:{status:'blocked',ready:false,reason:'planner 未部署'},inference:{status:'degraded',ready:false,reason:'模型 worker 不可达'}};
  state.dependencies=[{id:'sdk',label:'SDK 安装',category:'preinstalled',status:'ready',expected:'登记版本',actual:'已核验',checked_wall_ns:1000000000},
    {id:'planner',label:'规划运行时',category:'preparable',status:'blocked',expected:'已登记服务',actual:{status:'starting',stage:'warming',message:'正在初始化规划',service_pid:321,model_initialized:false,allowed_read_roots:['/trusted/large/runtime/tree'],spec:{payload:'raw evidence'}},reason:'请准备数采'},
    {id:'usb',label:'相机连接',category:'onsite',status:'waiting_onsite',expected:'当前流配置',actual:'首帧超时',reason:'核对 USB 线后重新检查',recommended_action:'status_check'}];
  h.reply(0,state);await settle();assert.equal(h.api.elements.groups.length,4);
  assert.match(h.api.elements.capabilityRows.textContent,/已核验就绪.*条件未满足.*状态未知.*能力受限/s);
  assert.match(h.api.elements.dependencyRows.textContent,/预置条件.*SDK 安装.*可通过运维准备.*规划运行时.*现场条件.*首帧超时/s);
  const sections=h.api.elements.dependencyRows.children;
  const planner=sections.find(node=>node.children[0].textContent==='可通过运维准备').children[1].children.find(node=>node.tag==='article');
  assert.match(planner.children[2].textContent,/状态：starting.*阶段：warming.*说明：正在初始化规划.*PID：321.*模型初始化：false/s);
  assert(!planner.children[2].textContent.includes('allowed_read_roots'),'large runtime evidence must not expand the main condition summary');
  const raw=planner.children.find(node=>node.tag==='details');assert.equal(raw.open,false);assert.equal(raw.children[1].textContent,JSON.stringify(state.dependencies[1].actual,null,2));
  assert.match(h.api.elements.controlFacts.textContent,/SDK close 未确认/);assert.match(h.api.elements.guidance.textContent,/等待现场处理.*USB 线/s);
  assert.equal(sections[0].open,false);assert.equal(sections[1].open,true);assert.equal(sections[2].open,true);
  sections[0].open=true;sections[2].open=false;raw.open=true;
  state.dependencies[1].actual.stage='ready';
  const refresh=h.api.refresh();h.reply(h.last(),state);await refresh;
  assert.equal(h.api.elements.dependencyRows.children[0],sections[0],'polling preserves the disclosure DOM node');
  assert.equal(sections[0].open,true,'polling keeps user-expanded preconditions open');
  assert.equal(sections[2].open,false,'polling keeps user-collapsed conditions closed');
  const updatedRaw=sections[1].children[1].children[0].children.find(node=>node.tag==='details');
  assert.equal(updatedRaw,raw);assert.equal(updatedRaw.open,true);
  assert.equal(updatedRaw.children[1].textContent,JSON.stringify(state.dependencies[1].actual,null,2),'open evidence still refreshes');
  sections[0].open=false;raw.open=false;
  const failedRefresh=h.api.refresh();h.reply(h.last(),{},false);await failedRefresh;
  assert.equal(sections[0].open,false);assert.equal(raw.open,false,'a failed refresh cannot reset disclosure state');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'dependency rendering is observational');h.api.destroy();
}
{
  const h=harness();h.reply(0,fixture());await settle();h.button('prepare_model').fire('click');h.api.elements.confirm.fire('click');
  assert.equal(h.requests.length,2);h.api.elements.stopButton.fire('click');h.api.elements.stopButton.fire('click');
  assert.equal(h.requests.length,3,'independent Stop bypasses operation busy but deduplicates its own submit');
  assert.equal(h.requests[2].url,'/api/operations/stop?environment=physical');
  assert.deepEqual(Object.keys(JSON.parse(h.requests[2].options.body)).sort(),['expected_device_id','request_id']);
  h.requests[2].reject(Error('backend unreachable'));await settle();assert.match(h.api.elements.status.textContent,/停止结果未知/);
  const refresh=h.api.refresh();h.reply(h.last(),fixture());await refresh;assert.match(h.api.elements.status.textContent,/停止结果未知/,'routine polling cannot erase unknown stop evidence');h.api.destroy();
}
{
  const h=harness(),state=fixture();state.recent_jobs=[{job_id:'needs-evidence',request:{request_id:'receipt-only'},status:'unknown',phase:'verify',result:{summary:'保留摘要'},detail_url:'/api/operations/jobs/needs-evidence'},
    {job_id:'unsafe',status:'unknown',detail_url:'javascript:alert(1)'}];
  h.reply(0,state);await settle();assert.match(h.api.elements.history.textContent,/结果尚未核验/);
  assert(h.nodes.some(node=>node.tag==='a'&&node.textContent==='完整操作证据'&&node.href==='/api/operations/jobs/needs-evidence?environment=physical'));
  assert(!h.nodes.some(node=>node.href==='javascript:alert(1)'));assert.equal(h.requests.length,1,'evidence link does not fetch or perform actions before navigation');h.api.destroy();
}
{
  const h=harness({standalone:true,query:'?environment=physical&business=collection'}),calls=[];
  h.doc.workbenchEnvironmentAccess={navigateEnvironment:(next,options)=>calls.push({next,options})};
  h.reply(0,fixture());await settle();h.select.value='fake';h.select.fire('change');
  assert.deepEqual(calls,[{next:'fake',options:{page:'/operations'}}]);assert.equal(h.select.value,'physical');
  assert.equal(h.api.environment,'physical');assert.equal(h.requests.length,1,'mode selection delegates a fenced full navigation, never changes the loaded deployment client');h.api.destroy();
}
{
  const h=harness(),state=fixture();state.active_job={job_id:'collection-current',request:{request_id:'collection-current',action:'prepare_collection'},status:'running',phase:'preparing_collection'};
  state.collection={status:'initializing_detector',progress:{elapsed_s:160.42},worker:{message:'initializing_detector'},services:{detector:{status:'starting'}}};
  h.reply(0,state);await settle();assert.match(h.api.elements.current.textContent,/正在初始化检测服务.*已用 160.4 秒.*不估算剩余时间/s);
  const refresh=h.api.refresh();state.collection.worker.message='native stage x';state.collection.status='future_stage';state.collection.progress.elapsed_s=170;h.reply(h.last(),state);await refresh;
  assert.match(h.api.elements.current.textContent,/正在核验数采准备阶段.*已用 170.0 秒/s);
  assert(h.nodes.some(node=>node.tag==='summary'&&node.textContent==='数采准备原始阶段'));
  assert.match(h.api.elements.current.textContent,/native stage x/);assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.api.destroy();
}
{
  const stopped={runtime:{connected:null},health:{backend_reachable:false,busy:null,services:{workbench:{state:'stopped',running:false,owned:true}}},recovery:{required:false}};
  assert.equal(environmentSwitchReason(stopped),'');
  for(const options of [{stale:true},{pending:true}])assert.notEqual(environmentSwitchReason(stopped,options),'');
  for(const change of [{health:{backend_reachable:true}},{health:{services:{workbench:{state:'running',running:true}}}},
    {health:{services:{workbench:{state:'unknown',running:false}}}},{health:{services:{workbench:{state:'stopped',running:null}}}},
    {health:{services:{workbench:{state:'stopped',running:true}}}},{health:{busy:true}},{runtime:{connected:true}},
    {recovery:{required:true}},{recovery:{}},{active_job:{status:'running'}},{health:{maintenance:{status:'active'}}},
    {maintenance:{status:'active'}},{runtime:{maintenance_active:true}}]){
    const state={...stopped,...change,health:{...stopped.health,...change.health}};
    assert.notEqual(environmentSwitchReason(state),'',JSON.stringify(change));
  }
}
console.log('PASS operations UI: fake/physical routing, explicit impact/support review, independent polling, target fencing, idempotent retry, backend gates, verified outcomes, safe text, cancellation and standalone page.');

// The fault's inline recovery uses the same review as the operation card.
{
 const h=harness(),state=fixture();
 state.operations.recover_control={allowed:true,label:'中断恢复',impact:'恢复新会话，不续跑旧动作',requires_support:true};
 state.control_alerts={warnings:[],current:{active_anomaly:true,heartbeat:{status:'lost'}}};
 h.reply(0,state);await settle();
 const recovery=h.nodes.find(row=>row.tag==='button'&&row.textContent==='中断恢复…');
 assert.equal(recovery.disabled,false);recovery.fire('click');
 assert.equal(h.api.elements.dialog.open,true);assert.equal(h.api.elements.confirm.disabled,true);
 assert.equal(h.api.elements.support.checked,false);assert.match(h.api.elements.dialog.textContent,/中断恢复/);
 assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'opening recovery never reconnects');
 const refresh=h.api.refresh();h.requests[h.last()].reject(Error('offline'));await refresh;
 assert.equal(recovery.disabled,true,'stale state fences the inline recovery too');h.api.destroy();
}

// One-shot event acknowledgement must never hide an ongoing heartbeat fault.
{
 const container=new Element('div'),doc={createElement:tag=>new Element(tag)},acks=[];
 let recoveries=0;
 const alerts=mountControlAlerts({document:doc,container,recover:()=>recoveries++,acknowledge:id=>new Promise(resolve=>acks.push({id,resolve}))});
 const event={id:'fault-a',reason:'SDK owner heartbeat lost',wall_ns:1};
 alerts.update({warnings:[event],current:{active_anomaly:false,heartbeat:{status:'healthy'}}});
 const [warnings,current,recovery]=alerts.root.children;
 const recover=recovery.children[0];
 assert.equal(recovery.hidden,false,'an unacknowledged event retains its recovery index');
 assert.equal(warnings.children.length,1);assert.equal(current.hidden,true,'recovery clears only live anomaly');
 alerts.update({warnings:[event],current:{active_anomaly:true,reason:'heartbeat missing',heartbeat:{status:'lost'}}});
 const index=current.children.find(row=>row.tag==='a');assert.equal(index.href,'#'+recover.id);
 index.fire('click');assert.equal(recoveries,0,'recovery index only navigates; it never acquires control');
 assert(warnings.children[0].children.some(row=>row.tag==='a'&&row.href===index.href));
 assert.equal(recovery.hidden,false);recover.fire('click');assert.equal(recoveries,0,'recovery stays disabled before fresh permission');
 alerts.setRecoveryGate({allowed:true});recover.fire('click');assert.equal(recoveries,1);
 const button=warnings.children[0].children.find(row=>row.tag==='button');button.fire('click');
 assert.equal(acks[0].id,'fault-a');acks[0].resolve({acknowledged:true,motion_resumed:false});await settle();
 assert.equal(warnings.children.length,0);assert.equal(current.hidden,false,'acknowledgement never clears active fault');
 assert.equal(recovery.hidden,false,'acknowledgement never hides recovery');assert.equal(recoveries,1,'acknowledgement never recovers control');
 alerts.update({warnings:[{...event,id:'fault-b'}],current:{active_anomaly:false,heartbeat:{status:'healthy'}}});
 assert.equal(warnings.children.length,1,'a new stop event must reappear');assert.equal(current.hidden,true);
 alerts.update({warnings:[event],current:{active_anomaly:false}},{stale:true});assert.equal(current.hidden,false,'stale is unknown, not recovered');
 alerts.setRecoveryGate({allowed:false,reason:'状态未知'});recover.fire('click');assert.equal(recoveries,1);assert.match(recovery.textContent,/状态未知/);
 alerts.update({warnings:[],current:{active_anomaly:true,reason:'worker exited',heartbeat:{status:'unavailable'}}});assert.match(current.textContent,/当前控制链路异常/);assert.doesNotMatch(current.textContent,/当前心跳异常/);
 alerts.update({warnings:[event],current:{active_anomaly:false,recovery_required:true,heartbeat:{status:'not_monitored'}}});
 assert.equal(current.hidden,false);assert.equal(recovery.hidden,false);
 assert.match(current.textContent,/控制已断开/);assert.doesNotMatch(current.textContent,/当前心跳异常/);
 assert.equal(current.children.find(row=>row.tag==='a').href,'#'+recover.id);
 assert.equal(warnings.children.length,1,'confirmed process exit retains the historical warning');
}

{
 const h=harness(),value=fixture();
 value.active_job={job_id:'trace',request:{request_id:'trace'},label:'退出',status:'unknown',phase:'waiting_exit_home',detail:{step_refs:['UC09-S02','<script>']}};
 h.reply(0,value);await settle();
 const links=h.nodes.filter(node=>node.tag==='a'&&node.textContent.includes('架构步骤'));
 assert.equal(links.length,1);assert.match(links[0].href,/#trace\?uc=UC09&step=UC09-S02$/);
 assert.match(links[0].textContent,/不代表已通过/);h.api.destroy();
}
