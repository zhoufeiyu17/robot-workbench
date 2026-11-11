import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {webcrypto} from 'node:crypto';

const source=await readFile(new URL('../web/app.js',import.meta.url),'utf8');
const sessionSource=await readFile(new URL('../web/session.js',import.meta.url),'utf8');
const sessionModule='data:text/javascript;base64,'+Buffer.from(sessionSource).toString('base64');
const operationsSource=await readFile(new URL('../web/operations.js',import.meta.url),'utf8');
const operationsModule='data:text/javascript;base64,'+Buffer.from(operationsSource).toString('base64');
const endpointSource=await readFile(new URL('../web/physical_endpoint.js',import.meta.url),'utf8');
const endpointModule='data:text/javascript;base64,'+Buffer.from(endpointSource.replace("'./operations.js'",JSON.stringify(operationsModule))).toString('base64');
const html=await readFile(new URL('../web/index.html',import.meta.url),'utf8');
assert(html.indexOf('data-tab="research"')<html.indexOf('data-tab="results"'),'control research precedes playback');
assert.match(html,/<option value="research">控制研究<\/option>/,'research is a real handoff destination');
assert.doesNotMatch(html,/id="preview-view"/,'one preview surface must not expose duplicate pose/view modes');
assert.match(html,/<h3>目标姿态预览<\/h3>/);
assert.match(html,/<h2 id="precision-study-title">控制精度研究<\/h2>/);
assert.match(html,/<details id="precision-settings-details">/,'research settings are collapsed by default');
assert.doesNotMatch(html,/precision-settings-v10/,'config versions must not have dedicated buttons');
assert.match(html,/<select id="precision-config"/,'control configs share one selector');
for(const id of ['observation-panel','manual-tools','media-preview','camera-grid'])
  assert.equal([...html.matchAll(new RegExp('id="'+id+'"','g'))].length,1,'shared workspace component must not be duplicated: '+id);
assert.match(html, /id="control-fault-recovery" href="#environment-recover"/);
assert.match(html, /id="collection-speed" type="range" min="30" max="170"/);
const {inferenceFamily,inferenceModes,inferenceLabel,recordingTaskTime,createWorkbench,actionPermission,handoffPermission,countedProgress,frameAgeSeconds,safeMediaUrl,verifyPreviewFrame,previewAuditSummary,matchingMeshCandidate,meshRows,meshGroups,routePhysicalRequest}=await import('data:text/javascript;base64,'+Buffer.from(source.replace("'./session.js'",JSON.stringify(sessionModule)).replace("'./physical_endpoint.js'",JSON.stringify(endpointModule))).toString('base64'));

assert.equal(inferenceFamily({id:'wam-mem-fixture-step1'}),'wam-mem');
assert.equal(inferenceFamily({id:'wam-fixture-step1'}),'wam');
assert.equal(inferenceLabel({id:'wam-mem-t01-step006486',label:'WAM-MEM · step_006486 · 2步去噪'}),'WAM-MEM T01 · step_006486 · 2步去噪');
assert.equal(inferenceLabel({id:'wam-mem-t02-1-step006076',label:'WAM-MEM · step_006076 · 10步去噪'}),'WAM-MEM T02_1 · step_006076 · 10步去噪');
assert.equal(inferenceLabel({id:'wam-example-step1',label:'WAM example'}),'WAM example');
assert.match(html,/<option value="wam-mem">WAM-MEM<\/option>/);
assert.doesNotMatch(html,/id="inference-memory-fresh"/);
assert.equal(inferenceModes([{id:'wam-mem-fixture-step1',backend:'torch',inference_options:{denoise_steps:2},inference_capabilities:{denoise_steps:{default:2}}}], 'torch','wam-mem',2).length,1);


class Element {
  constructor(tag='div'){this.tagName=tag;this.children=[];this.attributes={};this.dataset={};this.events={};this.value='';this.checked=false;this.hidden=false;this.disabled=false;this.textContent='';this.className='';this.currentTime=0;this.duration=NaN;this.paused=true;this.playbackRate=1;this.open=false;this.classList={toggle:(name,on)=>{const names=new Set(this.className.split(' ').filter(Boolean));if(on)names.add(name);else names.delete(name);this.className=[...names].join(' ');},contains:name=>this.className.split(' ').includes(name)};}
  showModal(){this.open=true;this.showCount=(this.showCount||0)+1;}
  close(){this.open=false;this.events.close?.();}
  play(){this.paused=false;return Promise.resolve();}
  pause(){this.paused=true;}
  append(...children){this.children.push(...children);}
  replaceChildren(...children){this.children=children;}
  setAttribute(name,value){this.attributes[name]=String(value);}
  removeAttribute(name){delete this.attributes[name];if(name==='src')this.src='';}
  addEventListener(name,callback){this.events[name]=callback;}
  querySelectorAll(selector){const matches=node=>selector.startsWith('.')?node.classList.contains(selector.slice(1)):node.tagName===selector;return this.children.flatMap(child=>[...(matches(child)?[child]:[]),...child.querySelectorAll(selector)]);}
  querySelector(selector){return this.querySelectorAll(selector)[0]||null;}
  async decode(){}
  getBoundingClientRect(){return {top:0,left:0,bottom:100,right:100,width:100,height:100};}
}
function documentStub(){
  const nodes=new Map([...html.matchAll(/id="([^"]+)"/g)].map(match=>[match[1],new Element()]));
  for(const id of ['return-offset-x','return-offset-y'])nodes.get(id).value='0';
  for(const id of ['head','left','right','preview','mesh']){const card=nodes.get('media-'+id),image=new Element('img'),empty=new Element('p'),meta=new Element('p');empty.className='empty';meta.className='media-meta';card.append(image,empty,meta);}
  const tabs=['calibration','collection','inference','research','results','operations'].map(tab=>{const button=tab==='operations'?nodes.get('operations-link'):new Element('button');button.dataset.tab=tab;return button;});
  return {hidden:false,nodes,tabs,events:{},getElementById(id){assert(nodes.has(id),'Missing UI node '+id);return nodes.get(id);},createElement(tag){return new Element(tag);},createElementNS(namespace,tag){return new Element(tag);},querySelectorAll(selector){return selector==='[data-tab]'?tabs:[];},addEventListener(name,callback){this.events[name]=callback;}};
}
const response=(value,headers={})=>({ok:true,status:200,headers:{get:key=>headers[key]??null},json:async()=>value,blob:async()=>new Blob(['fixture'])});
const settle=async()=>{for(let i=0;i<8;i++)await Promise.resolve();};
const degrees=value=>value*180/Math.PI;
function snapshot(overrides={}){
  return {schema:'robot-workbench-ui.v1',server_instance_id:'fixture-server',revision:1,server_wall_ns:10e9,csrf:'fixture-only',device:{id:'fixture-device',label:'Fixture device',environment:'fake'},runtime:{connected:true,holder:'calibration',feedback_healthy:true,motion_status:'idle',hold:{left:'position',right:'position'},allowed_operations:Object.fromEntries(['handoff','stop','connect','cancel','recover','mesh','manual_preview','select_result'].map(action=>[action,{allowed:true,reason:''}]))},workflow:{id:'fixture-run',revision:1,status:'planning',stage_id:'head.plan',title:'头部规划',message:'仍在规划',progress:null,primary_action:{id:'generate_plan',label:'生成计划',allowed:true,reason:''},outcomes:{}},media:[],results:[],diagnostics:[],mesh:{requested:false,applied:false},...overrides};
}
function manualSnapshot(){
  const value=snapshot();value.runtime.allowed_operations.manual_queue={allowed:true};value.runtime.allowed_operations.manual_queue_preview={allowed:true};
  value.manual={joint_names:Array.from({length:12},(_,i)=>'J'+i),positions:Array(12).fill(.1),limits:Array.from({length:12},()=>[-1,1]),jaw_gaps_mm:[80,80],default_joint_speed_rad_s:.2,default_jaw_speed_mm_s:10,queue:{available:true,revision:4,waypoints:[]}};
  return value;
}
function harness({audit=false,physical=false,business,view,storage,sessionMonitor,environmentAccess,crypto=webcrypto,create=createWorkbench}={}){
  const document=documentStub(),requests=[],intervals=[],revoked=[],blobs=[],timeouts=new Map(),rafs=new Map();let now=0,timerId=0,rafId=0;
  const app=create({document,storage,sessionMonitor,environmentAccess,location:{href:'http://localhost:8110/?'+new URLSearchParams({...audit?{'preview-audit':'1'}:{},...physical?{environment:'physical'}:{},...business!==undefined?{business}:{},...view?{view}:{}})},performance:{now:()=>now},crypto,URL:{createObjectURL:blob=>{blobs.push(blob);return `blob:fixture-${requests.length}`;},revokeObjectURL:value=>revoked.push(value)},setInterval:fn=>{intervals.push(fn);return intervals.length;},clearInterval:()=>{},setTimeout:(fn,ms)=>{timeouts.set(++timerId,{fn,ms});return timerId;},clearTimeout:id=>timeouts.delete(id),requestAnimationFrame:fn=>{rafs.set(++rafId,fn);return rafId;},cancelAnimationFrame:id=>rafs.delete(id),fetch:(url,options={})=>new Promise(resolve=>requests.push({url,options,resolve}))});
  const fireTimer=predicate=>{const timer=[...timeouts.entries()].find(([,t])=>predicate(t.ms));assert(timer,'matching timer scheduled');timeouts.delete(timer[0]);timer[1].fn();return timer[1].ms;};
  return {app,document,requests,intervals,revoked,blobs,timeouts,advance:ms=>now+=ms,get:id=>document.getElementById(id),reply(index,value,headers){requests[index].resolve(response(value,headers));},fireTimer,nextFrame:()=>fireTimer(ms=>ms!==2500),paint(){const ready=[...rafs.values()];rafs.clear();for(const fn of ready)fn(now);}};
}

{
  const h=harness({physical:true,business:'inference',view:'operations'});
  h.app.start();
  assert.equal(h.app.context.business,'inference');
  assert.equal(h.get('operations-panel').hidden,false);
  assert.equal(h.get('work-pane').hidden,true);
  assert.equal(h.requests.filter(row=>row.options?.method==='POST').length,0);
  h.app.destroy();
}

assert.equal(countedProgress({current:4,total:null}),null);
assert.equal(countedProgress({current:12,total:10}),null);
assert.equal(countedProgress({current:0,total:10}).current,0);
assert.equal(frameAgeSeconds({source_wall_ns:7e9},{server_wall_ns:10e9},250),3.25);
assert.equal(frameAgeSeconds({},snapshot()),null);
assert.equal(recordingTaskTime(Date.parse('2026-11-09T05:37:00Z')*1e6,'Asia/Shanghai'),'11-09 13:37');
assert.equal(recordingTaskTime(null),null);
assert.equal(recordingTaskTime(NaN),null);
assert.equal(safeMediaUrl('https://external.invalid/frame.jpg','http://localhost:8110/'),null);
assert.equal(safeMediaUrl('/media/head.jpg','http://localhost:8110/'),'http://localhost:8110/media/head.jpg');
assert.equal(actionPermission(snapshot(),'generate_plan').allowed,true);
assert.equal(actionPermission(snapshot(),'execute').allowed,false);
const review=snapshot({device:{environment:'physical'},workflow:{primary_action:{id:'execute',allowed:true},review:{id:'review-fixture',physical_clearance_required:true}}});
assert.match(actionPermission(review,'execute').reason,/净空/);
assert.equal(actionPermission(review,'execute',{clearance:true}).allowed,true);
assert.equal(actionPermission(snapshot(),'stop',{pending:true,fresh:false}).allowed,true);
assert.equal(handoffPermission(snapshot(),'collection').allowed,true);
assert.equal(handoffPermission(snapshot(),'calibration').allowed,false);
assert.equal(handoffPermission(snapshot(),'constructor').allowed,false);
assert.equal(handoffPermission(snapshot(),'collection',{pending:true}).allowed,false);
assert.equal(handoffPermission(snapshot(),'collection',{fresh:false}).allowed,false);
assert.equal(handoffPermission({...snapshot(),workflow:{busy:true}},'collection').allowed,false);
assert.equal(handoffPermission({...snapshot(),runtime:{...snapshot().runtime,connected:false}},'collection').allowed,false);

for(const [business,expected]of [['inference','inference'],['collection','collection'],['calibration','calibration'],['invalid','calibration'],['constructor','calibration']]){
  const h=harness({physical:true,business});h.app.start();
  assert.deepEqual(h.app.context,{environment:'physical',business:expected});
  assert.equal(h.document.tabs.find(tab=>tab.dataset.tab===expected).attributes['aria-current'],'page');
  assert.equal(h.requests.length,1);assert.equal(h.requests[0].url,'/api/state?environment=physical&business='+expected);
  const href='#operations-panel';
  assert.equal(h.get('operations-link').href,href);assert.equal(h.get('operations-link').dataset.business,expected);
  assert.equal(h.get('context-guidance-next').href,href);
  const value=snapshot();value.device.environment='physical';h.reply(0,value);await settle();
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0,'initial browse context never connects, hands off or executes');
  h.app.selectTab('collection');
  assert.equal(h.get('operations-link').href,'#operations-panel');
  assert.equal(h.get('operations-link').dataset.business,'collection');
  h.app.selectTab('diagnostics');assert.equal(h.get('operations-link').dataset.business,'collection','auxiliary tabs retain the current business');
  h.app.destroy();
}

async function guidanceFixture(business,update,environment='fake'){
  const h=harness({physical:environment==='physical'});h.app.start();
  if(business!=='calibration')h.app.selectTab(business);
  const value=snapshot();Object.assign(value.runtime,{state:'READY',holder:business});
  Object.assign(value.workflow,{status:'idle',busy:false,stage_id:business+'.idle',message:'',
    primary_action:{id:'generate_plan',label:'生成计划',allowed:true}});
  value.runtime.allowed_operations.inference_refresh={allowed:false,reason:'fixture has no catalog refresh'};
  update?.(value);h.reply(h.requests.length-1,value);await settle();return {h,value};
}
const htmlIds=new Set([...html.matchAll(/id="([^"]+)"/g)].map(match=>match[1]));
assert(htmlIds.has('inference-family'),'model family must precede backend selection');

{
  const h=harness({business:'inference'});h.app.start();
  const wam='wam-e03-step003190';
  const rows=[{id:'pi-fixture',backend:'torch',bundle_digest:'a'.repeat(64),selectable:true},
    {id:'pi-triton',backend:'triton',bundle_digest:'b'.repeat(64),selectable:true},
    {id:wam,runtime_schema:'workbench.wam-e03.v1',backend:'torch',bundle_digest:'c'.repeat(64),selectable:true},
    {id:wam,runtime_schema:'workbench.wam-e03.v1',backend:'anyinfer',bundle_digest:'d'.repeat(64),selectable:true}];
  const value=snapshot();value.runtime.holder='inference';value.runtime.allowed_operations.inference_run={allowed:true};
  value.workflow.inference={selected_model_id:wam,backend:'anyinfer',bundle_digest:'d'.repeat(64),model_ready:true,
    model:{id:wam,backend:'anyinfer',loaded:true,status:'ready'},models:rows};
  h.reply(0,value);await settle();
  const choices=()=>h.get('inference-checkpoint').children.map(row=>row.value);
  assert.equal(h.get('inference-family').value,'wam');
  assert.equal(h.get('inference-backend').value,'anyinfer');
  assert.deepEqual(h.get('inference-backend').children.map(row=>[row.value,row.textContent]),[['torch','Torch'],['anyinfer','SO 算子加速']]);
  assert.deepEqual(choices(),['d'.repeat(64)]);
  h.get('inference-family').value='pi';h.get('inference-family').events.change();
  assert.equal(h.get('inference-backend').value,'torch');assert(choices().includes('pi-fixture'));
  assert(!choices().some(choice=>['c'.repeat(64),'d'.repeat(64)].includes(choice)));
  h.get('inference-backend').value='triton';h.get('inference-backend').events.change();
  h.get('inference-family').value='wam';h.get('inference-family').events.change();assert.equal(h.get('inference-backend').value,'anyinfer','returning to a family selects its loaded backend');
  h.get('inference-family').value='pi';h.get('inference-family').events.change();
  // The same backend name cannot make a different model family executable.
  h.app.poll();const next=structuredClone(value);next.revision=2;next.workflow.inference.backend='torch';next.workflow.inference.model.backend='torch';next.workflow.inference.bundle_digest='c'.repeat(64);
  h.reply(1,next);await settle();assert.equal(h.get('inference-run').disabled,true);
  h.get('inference-backend').value='triton';h.get('inference-backend').events.change();assert(choices().includes('pi-triton'));
  h.get('inference-family').value='wam';h.get('inference-family').events.change();
  assert.equal(h.get('inference-backend').value,'torch');assert.deepEqual(choices(),['c'.repeat(64)]);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'family/backend intent never starts a model or robot');
  h.app.destroy();
}
for(const target of ['operations-link','diagnostics-pane','task-panel','manual-tools','primary','collection-execute','return-cup',
  'inference-run','inference-checkpoint','environment-connect','collection-prepare','collection-plan'])
  assert(htmlIds.has(target),'guidance navigation target must exist in actual index.html: '+target);
function followGuidance(h,target){
  assert(htmlIds.has(target),'navigation target cannot be created only by the DOM fixture');
  let scrolled=false;h.get(target).scrollIntoView=()=>{scrolled=true;};
  const requests=h.requests.length;
  h.get('context-guidance-next').events.click({preventDefault(){}});
  assert.equal(scrolled,true);assert.equal(h.requests.length,requests,'guidance only navigates; it cannot send an action');
  assert.equal(h.get('inference-run-dialog').open,false);assert.equal(h.get('manual-execution-dialog').open,false);
}
{
  const h=harness();h.app.start();assert.match(h.get('context-guidance-title').textContent,/标定.*待核验/);
  h.app.selectTab('collection');assert.match(h.get('context-guidance-title').textContent,/数采.*待核验/);
  assert.match(h.get('context-guidance-message').textContent,/不会连接设备或交接控制权/);
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0);h.app.destroy();
}
for(const connected of [false,true]){
  const {h}=await guidanceFixture('inference',value=>{
    Object.assign(value.runtime,{connected,holder:connected?'calibration':null,state:connected?'READY':'DISCONNECTED'});
    value.runtime.allowed_operations.inference_run={allowed:true};
    value.workflow.inference={model_ready:true,control_ready:false,model:{loaded:true,id:'fixture-model'}};
  });
  assert.match(h.get('context-guidance-title').textContent,connected?/模型已就绪.*等待本轮确认/:/模型已就绪.*尚未连接/);
  assert.match(h.get('context-guidance-message').textContent,connected?/控制已接入.*不连接或恢复/:/控制未接入.*Run 不会连接/);
  assert.equal(h.get('context-guidance-next').href,'#inference-run');followGuidance(h,'inference-run');
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0);
  h.advance(3100);h.intervals[1]();assert.match(h.get('context-guidance-title').textContent,/待重新核验/);
  assert.equal(h.get('context-guidance-next').href,'#operations-panel');h.app.destroy();
}
{
  const {h}=await guidanceFixture('inference',value=>{
    Object.assign(value.runtime,{connected:false,holder:null,state:'DISCONNECTED'});
    value.runtime.allowed_operations.inference_run={allowed:false,reason:'头部相机尚未就绪，请核验当前相机服务'};
    value.workflow.inference={model_ready:true,model:{loaded:true}};
  });
  assert.match(h.get('context-guidance-title').textContent,/模型已就绪.*尚未连接/);
  assert.equal(h.get('context-guidance-message').textContent,'头部相机尚未就绪，请核验当前相机服务');
  followGuidance(h,'inference-run');h.app.destroy();
}
{
  const {h}=await guidanceFixture('inference',value=>{value.workflow.inference={};});
  assert.match(h.get('context-guidance-title').textContent,/状态待核验/);
  assert.doesNotMatch(h.get('context-guidance-title').textContent,/已就绪/);h.app.destroy();
}
for(const [business,update,title,reason,target]of [
  ['calibration',v=>{v.runtime.state='FAULT';v.runtime.reason='CAN反馈丢失，控制仍锁定';},/控制故障仍锁定/,/CAN反馈丢失/,'diagnostics-pane'],
  ['calibration',v=>{v.maintenance={purpose:'restart_model'};v.workflow.primary_action.reason='服务维护中';},/服务维护中/,/服务维护中/,'operations-link'],
  ['calibration',v=>{v.workflow.primary_action={id:'generate_plan',allowed:false,reason:'先应用标定类型与头部依赖'};},/标定/,/先应用标定类型与头部依赖/,'primary'],
  ['collection',v=>{v.workflow.collection={warmup:{status:'not_started',message:'请先安装当前设备的头部与双腕标定'}};v.runtime.allowed_operations.collection_prepare={allowed:false,reason:'请先安装当前设备的头部与双腕标定'};},/数采环境待准备/,/请先安装当前设备/,'collection-prepare'],
  ['calibration',v=>{v.workflow.review={id:'review-for-guidance'};v.workflow.primary_action={id:'execute',allowed:true};},/待审阅确认/,/原执行入口/,'primary'],
  ['collection',v=>{v.workflow.busy=true;v.workflow.stage_id='collection.plan';v.workflow.message='正在处理本次真实观测';},/观测与规划/,/本次真实观测/,'task-panel']
]){
  const {h}=await guidanceFixture(business,update);
  assert.match(h.get('context-guidance-title').textContent,title);assert.match(h.get('context-guidance-message').textContent,reason);
  assert.equal(h.get('context-guidance-next').dataset.target,target);assert.equal(h.get('stop').disabled,false,'guidance preserves independent Stop');
  if(target!=='operations-link')followGuidance(h,target);
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0);h.app.destroy();
}
{
  const {h}=await guidanceFixture('collection',value=>{
    value.workflow.collection={warmup:{status:'failed',message:'准备失败'},environment:{ready:true}};
    value.runtime.allowed_operations.collection_prepare={allowed:false,reason:'请核验初始化失败原因'};
  });
  assert.match(h.get('context-guidance-title').textContent,/数采初始化失败/,'explicit warmup wins over environment fallback, as in renderCollection');
  const requests=h.requests.length,lookup=h.document.getElementById.bind(h.document);
  h.document.getElementById=id=>id==='removed-guidance-target'?null:lookup(id);
  h.get('context-guidance-next').dataset.target='removed-guidance-target';
  assert.doesNotThrow(()=>h.get('context-guidance-next').events.click({preventDefault(){}}));
  assert.equal(h.requests.length,requests);h.app.destroy();
}
for(const environment of ['fake','physical']){
  const {h}=await guidanceFixture('collection',value=>{
    value.device.environment=environment;
    value.workflow.collection={warmup:{status:'invalidated',message:'服务变化后需要业务预热'},environment:{ready:false}};
    value.workflow.primary_action={id:'generate_plan',allowed:false,reason:'数采服务已就绪；请在本页完成业务环境准备（初始化与预热）'};
    value.runtime.allowed_operations.collection_prepare={allowed:true,reason:null};
  },environment);
  assert.equal(h.get('context-guidance-title').textContent,'数采业务待预热');
  assert.match(h.get('context-guidance-message').textContent,/本页.*初始化与预热/);
  assert.doesNotMatch(h.get('context-guidance-message').textContent,/运维/);
  followGuidance(h,'collection-prepare');h.app.destroy();
}

{
  const version='a'.repeat(64),injected=source.replace('__WORKBENCH_DEPLOYMENT_SHA__',version)
    .replace("'./session.js'",JSON.stringify(sessionModule)).replace("'./physical_endpoint.js'",JSON.stringify(endpointModule));
  const versioned=await import('data:text/javascript;base64,'+Buffer.from(injected).toString('base64'));
  for(const [create,expected] of [[createWorkbench,undefined],[versioned.createWorkbench,version]]){
    const h=harness({create});h.app.start();h.reply(0,snapshot());await settle();
    const sent=h.app.send('stop');assert.equal(h.requests[1].options.headers['X-Workbench-Deployment'],expected);
    h.reply(1,{message:'stopped'});await sent;h.app.destroy();
  }
}

{
  let starts=0,stops=0;const h=harness({sessionMonitor:{start:()=>starts++,destroy:()=>stops++}});
  h.app.start();assert.equal(starts,1);h.reply(0,snapshot());await settle();h.app.selectTab('collection');assert.equal(starts,1,'business navigation does not restart development services');h.app.destroy();assert.equal(stops,1,'workbench destruction also stops the independent management monitor');
}

function collectionSnapshot(){
  const value=snapshot(),binding={execution_instance_id:'current-instance',runtime_nonce:'owner',source_clock_id:'clock',
    execution_timeline_digest:'initialized-plan',timeline_digest:'frozen-plan',device_id:value.device.id,environment:'fake'};
  Object.assign(value.workflow,{status:'running',busy:true,stage_id:'collection.execute',execution_binding:binding,
    outcomes:{motion:{status:'running'},data:{status:'recording'},task:{status:'running'},simulation:'未验证'},progress:{current:1270,total:3671,scope:'motion'}});
  Object.assign(value.runtime,{holder:'collection',environment:'fake',device_id:value.device.id,runtime_nonce:'owner',command_epoch:4,
    motion_status:'RUNNING',execution_instance_id:'current-instance',calibration_feedback:{source_clock_id:'clock'},
    execution_calibration:{business:'collection',execution_instance_id:'current-instance',command_epoch:4},
    native_task:{execution_instance_id:'current-instance',runtime_nonce:'owner',source_clock_id:'clock',execution_timeline_digest:'initialized-plan',task_outcome:'running',first_failure:null},
    frozen:{status:'running',timeline_digest:'frozen-plan',execution_timeline_digest:'initialized-plan',send_count:4233,submitted_command_ticks:1270,execution_tick_count:3671},
    execution_timeline:{timeline_digest:'initialized-plan',parent_timeline_digest:'frozen-plan',command_count:3671},
    recording:{ready:true,done:false,error:null,seal_error:null,diagnostics_error:null}});
  return value;
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');
  const value=collectionSnapshot();h.reply(1,value);await settle();
  assert.equal(h.get('outcome-motion').textContent,'进行中');assert.equal(h.get('outcome-data').textContent,'录制中');
  assert.equal(h.get('task-progress').value,1270);assert.equal(h.get('task-progress').max,3671);
  assert.match(h.get('progress-label').textContent,/1270 \/ 3671.*34.6%.*仅运动指令进度/);assert.equal(h.get('overall-panel').hidden,true);
  value.revision++;value.runtime.recording.error='unmatched raw event';value.runtime.frozen.submitted_command_ticks=3666;h.app.poll();h.reply(2,value);await settle();
  assert.equal(h.get('outcome-data').textContent,'录制中');assert.equal(h.get('task-progress').value,1270,'only server workflow receipts determine displayed outcomes and progress');
  value.revision++;value.workflow.outcomes.data={status:'invalid',message:'raw queue overflow'};h.app.poll();h.reply(3,value);await settle();
  assert.match(h.get('outcome-data').textContent,/无效.*raw queue overflow/);assert.match(h.get('alert').textContent,/录制失败，动作仍在执行/);
  value.revision++;value.workflow.status='failed';value.workflow.busy=false;value.workflow.outcomes={motion:'completed',data:'invalid',task:'failed'};
  h.app.poll();h.reply(4,value);await settle();assert.equal(h.get('task-status').textContent,'失败');
  assert.equal(h.get('outcome-task').textContent,'失败');assert.equal(h.get('outcome-motion').textContent,'已完成');h.app.destroy();
}

for(const [status,message,label] of [['running','本轮准备仍在运行','进行中'],['failed','实际任务失败：场景检测错误','失败']]){
  const h=harness();h.app.start();const value=snapshot();
  value.workflow={...value.workflow,status,busy:status==='running',message,request:{status:'failed',error:'重复请求被拒绝'}};
  h.reply(0,value);await settle();
  assert.match(h.get('task-message').textContent,new RegExp(message));
  assert.match(h.get('task-message').textContent,/本次请求失败：重复请求被拒绝/);
  assert.match(h.get('request-status').textContent,/本次请求失败：重复请求被拒绝/,'asynchronous failure is visible without opening details');
  assert.equal(h.get('task-status').textContent,label,'a request error is not the business task outcome');
  h.app.destroy();
}

{
  const h=harness();h.get('request-status').textContent='正在读取设备与任务状态…';h.app.start();
  h.reply(0,snapshot());await settle();assert.equal(h.get('request-status').textContent,'设备与任务状态已更新。');
  const action=h.app.send('generate_plan');h.app.poll();h.reply(2,snapshot({revision:2}));await settle();
  assert.match(h.get('request-status').textContent,/请求已发送/,'a state poll must preserve an in-flight action notice');
  h.requests[1].resolve({ok:false,json:async()=>({error:'planner refused'})});await action;h.reply(3,snapshot({revision:3}));await settle();
  assert.match(h.get('request-status').textContent,/planner refused/,'a state poll must preserve an explicit action failure');
  h.app.selectTab('collection');h.reply(4,snapshot({revision:4}));await settle();
  assert.equal(h.get('request-status').textContent,'设备与任务状态已更新。','context-loading notice also ends after its valid state');h.app.destroy();
}

{
  const h=harness();h.app.start();const value=snapshot();value.runtime.motion_status='no_automatic_task';
  value.workflow={id:'manual-review-1',status:'awaiting_review',stage_id:'manual.review',title:'人工轨迹待审阅',primary_action:{id:'manual_execute',label:'审阅并执行本批',allowed:true},review:{id:'manual-review-1',summary:'本次人工轨迹'}};
  h.reply(0,value);await settle();assert.equal(h.get('motion').textContent,'动作：无自动任务');assert.match(h.get('motion').title,/不代表物理静止/);
  assert.equal(h.get('stage-title').textContent,'人工轨迹待审阅');assert.equal(h.get('stage-id').textContent,'manual.review');assert.equal(h.get('primary').disabled,false);
  h.app.destroy();
}

for(const [action,label] of [['manual_preview','单姿态'],['manual_queue_preview','整队']]){
  const h=harness();h.app.start();const value=manualSnapshot();
  value.workflow={...value.workflow,status:'idle',stage_id:'calibration.idle',title:'准备标定',message:'',
    primary_action:{id:'generate_plan',label:'生成完整标定计划',allowed:false,reason:'当前请求仍在处理'},
    request:{action,status:'running',started_wall_ns:8e9},outcomes:{motion:'not_started',task:'not_started'}};
  h.reply(0,value);await settle();
  assert.equal(h.get('stage-title').textContent,'准备标定');assert.equal(h.get('manual-execution-status').textContent,'执行前检查');
  assert.match(h.get('manual-request-status').textContent,/已耗时 2.0 秒.*尚未执行.*弹出本次动作确认/);
  assert.equal(h.get('manual-execution-dialog').open,false);assert.equal(h.get('primary').disabled,true);
  value.revision++;value.server_wall_ns=11e9;h.app.poll();h.reply(1,value);await settle();
  assert.match(h.get('manual-request-status').textContent,/已耗时 3.0 秒/,'elapsed time uses each fresh server receipt');
  value.revision++;value.workflow.request={...value.workflow.request,status:'failed',finished_wall_ns:150e9,error:'真机：q_target[6] outside SDK limit'};
  h.app.poll();h.reply(2,value);await settle();
  assert.equal(h.get('stage-title').textContent,'准备标定');assert.equal(h.get('manual-execution-status').textContent,'检查失败');
  assert.match(h.get('manual-request-status').textContent,/已耗时 142.0 秒.*开发设备：q_target\[6\] outside SDK limit.*未下发本次运动/);
  assert.equal(h.get('manual-request-status').className,'tone-bad');assert.equal(h.get('outcome-task').textContent,'尚未开始','request failure does not rewrite business outcomes');
  value.revision++;value.workflow.stage_id='head.capture';value.workflow.status='running';
  h.app.poll();h.reply(3,value);await settle();assert.equal(h.get('stage-title').textContent,'头部采集');assert.equal(h.get('task-status').textContent,'进行中','request error cannot mask an active business stage');
  value.revision++;value.workflow={...value.workflow,status:'awaiting_review',stage_id:'manual.review',request:undefined,
    primary_action:{id:'manual_execute',label:'审阅并执行本批',allowed:true},review:{id:'new-manual-review',summary:'本次人工轨迹'}};
  h.app.poll();h.reply(4,value);await settle();
  assert.match(h.get('manual-request-status').textContent,/本页没有待确认动作/);assert.equal(h.get('manual-execution-dialog').open,false);assert.equal(h.get('primary-action').hidden,true);
  assert.equal(h.get('primary').disabled,false);assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0,'status rendering never confirms motion');
  h.get('manual-speed').value=.1;h.get('apply-manual-speed').events.click();
  assert.equal(h.get('manual-execution-dialog').open,false);h.app.destroy();
}

// Shared components stay in the same DOM across business tabs; browsing performs no actions.
for(const business of ['calibration','collection','inference']){
  const h=harness({business});h.app.start();const value=manualSnapshot();
  value.workflow={...value.workflow,stage_id:'manual.execute',status:'executing',message:'人工执行中',
    business_task:{stage_id:business+'.idle',status:'idle',title:'业务任务',message:'业务任务没有启动',primary_action:{id:'generate_plan',label:'生成业务计划',allowed:true}}};
  value.manual.execution={status:'executing',stage_id:'manual.execute',message:'本次人工轨迹执行中'};
  h.reply(0,value);await settle();
  assert.equal(h.get('inference-execution-settings').hidden,business!=='inference');
  assert.equal(h.get('task-message').textContent,'业务任务没有启动');
  assert.equal(h.get('manual-execution-status').textContent,'执行中');
  assert.equal(h.get('manual-execution-message').textContent,'本次人工轨迹执行中');
  const count=h.requests.length;h.get('operations-link').events.click({preventDefault(){}});
  assert.equal(h.get('operations-panel').hidden,false);assert.equal(h.get('work-pane').hidden,true);assert.equal(h.get('execution-settings-tools').hidden,true);assert.equal(h.get('inference-execution-settings').hidden,true);
  assert.equal(h.get('request-status').hidden,true,'business request notices stay outside operations');
  assert.equal(h.app.context.business,business);assert.equal(h.requests.length,count,'opening operations cannot connect, recover, hand off or execute');
  h.app.poll();h.reply(count,value);await settle();assert.equal(h.get('request-status').hidden,true,'a business state refresh cannot leak its notice into operations');
  h.app.selectTab(business);assert.equal(h.get('request-status').hidden,false,'business notices return with the business workspace');
  h.app.destroy();
}
assert.equal([...html.matchAll(/id="manual-tools"/g)].length,1);
assert.equal([...html.matchAll(/id="media-preview"/g)].length,1);
assert(!html.includes('data-tab="site-cameras"'));assert(!html.includes('/web/site_cameras.js'));
assert(!html.includes('data-tab="diagnostics"'));
assert(html.indexOf('id="calibration-queue"')>html.indexOf('id="manual-execution-state"'));

const operationsMarkup=html.slice(html.indexOf('id="operations-panel"'),html.indexOf('<footer>'));
for(const id of ['manual-tools','task-panel','arrival-panel','image-recovery','raw-state'])assert(!operationsMarkup.includes('id="'+id+'"'),'operations must not contain business/view tools: '+id);
assert.equal([...html.matchAll(/data-workbench-operations/g)].length,1);
for(const camera of ['head','left','right','all']){
  const h=harness({physical:true});h.app.start();const value=previewSnapshot();value.device.environment='physical';
  const group={id:'calibration:one-group',kind:'calibration',label:'头部 v5 + 双腕 v6',components:{head:'head-component',wrist:'wrist-component'},mesh_selectable:true,details:{environment:'hardware',robot_source:'physical',simulation_only:false}};
  value.preview.mesh_calibrations=[group];value.preview.mesh={requested:false,applied:false,camera:'head',candidate_id:'wrist-component'};
  h.reply(0,value);await settle();assert.equal(h.get('mesh-candidate').value,group.id);
  const option=h.get('mesh-candidate').children[1];assert.equal(h.get('mesh-candidate').children.length,2,'one calibration group plus placeholder');
  h.get('mesh-camera').value=camera;h.get('mesh-camera').events.change();
  assert.equal(h.get('mesh-candidate').value,group.id);assert.equal(h.get('mesh-candidate').children[1],option,'camera changes preserve the same option node and selection');
  const sent=h.requests.find(row=>row.options.method==='POST'&&row.url.includes('preview_mesh'));
  assert.equal(JSON.parse(sent.options.body).candidate_id,group.id);assert.equal(JSON.parse(sent.options.body).camera,camera);
  h.app.destroy();
}

async function pendingManualExecution({queue=false,physical=false}={}){
  const h=harness();h.app.start();const value=manualSnapshot();
  value.workflow={...value.workflow,status:'idle',stage_id:'calibration.idle'};
  value.manual.queue.waypoints=[{q:Array(12).fill(.1),speeds:Array(12).fill(.2),jaw_gap_mm:[80,81]}];
  h.reply(0,value);await settle();
  if(physical){h.app.switchContext({environment:'physical',business:'calibration'});value.device={...value.device,environment:'physical'};h.reply(h.requests.length-1,value);await settle();}
  h.get(queue?'queue-preview':'manual-preview').events.click();
  const post=h.requests.length-1;assert.equal(h.requests[post].url,'/api/actions/'+(queue?'manual_queue_preview':'manual_preview'));
  const ready=()=>{value.revision++;value.workflow={...value.workflow,status:'awaiting_review',stage_id:'manual.review',request:undefined,
    primary_action:{id:'manual_execute',label:'审阅并执行本批',allowed:true},review:{id:'intent-review',request_job_id:'intent-job',summary:'冻结动作：2个路径点，预计4.25秒',total_motion_duration_s:4.25,waypoint_jaw_gap_mm:[[80,81]],physical_clearance_required:physical}};return value;};
  return {h,value,post,ready};
}
for(const accepted of [false,true]){
  const {h,value,post,ready}=await pendingManualExecution();
  if(accepted){h.reply(post,{job_id:'intent-job'});await settle();}
  h.advance(3100);h.intervals[1]();
  assert.equal(h.get('manual-execution-dialog').open,false,'stale state cannot present or approve a pending check');
  h.get('manual-confirm').events.click();
  assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,0);
  if(!accepted){h.reply(post,{job_id:'intent-job'});await settle();}
  h.reply(h.requests.length-1,ready());await settle();
  assert.equal(h.get('manual-execution-dialog').open,true,'a slow unreviewed check waits for its fresh matching receipt');
  assert.equal(h.get('manual-execution-dialog').showCount,1);
  assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,0,'opening a delayed confirmation never executes');
  h.app.destroy();
}
{
  const {h,value,post,ready}=await pendingManualExecution();h.reply(post,{job_id:'intent-job'});await settle();
  ready().workflow.request={id:'intent-job',action:'manual_preview',status:'running'};
  value.workflow.primary_action={id:'manual_execute',allowed:false,reason:'当前请求仍在处理'};
  h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('manual-execution-dialog').open,false,'a published review must wait until its job has completed');
  h.app.poll();h.reply(h.requests.length-1,ready());await settle();
  assert.equal(h.get('manual-execution-dialog').open,true,'the pending intent survives review publication before job completion');
  assert.equal(h.get('manual-execution-dialog').showCount,1);assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,0);h.app.destroy();
}
for(const physical of [false,true]){
  const {h,value,post,ready}=await pendingManualExecution({physical,queue:physical});
  h.app.poll();h.reply(post+1,ready());await settle();assert.equal(h.get('manual-execution-dialog').open,false,'review arriving before its accepted job cannot open a confirmation');
  h.reply(post,{job_id:'intent-job',message:'accepted'});await settle();
  assert.equal(h.get('manual-execution-dialog').open,true);assert.equal(h.get('manual-execution-dialog').showCount,1);
  assert.match(h.get('manual-confirm-summary').textContent,/冻结动作/);assert.match(h.get('manual-confirm-duration').textContent,/4.3 秒/);assert.match(h.get('manual-confirm-jaws').textContent,/冻结夹爪目标 · 左 80 \/ 右 81 mm/);
  assert.equal(h.get('primary-action').hidden,true);assert.equal(h.get('review-panel').hidden,true,'manual confirmation is not duplicated in the business task card');
  assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,0,'checks and opening the dialog cannot execute');
  assert.equal(h.get('manual-confirm').disabled,physical);
  if(physical){h.get('manual-confirm').events.click();assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,0);h.get('manual-clearance').checked=true;h.get('manual-clearance').events.change();}
  assert.equal(h.get('manual-confirm').disabled,false);h.get('manual-confirm').events.click();
  const execution=h.requests.at(-1);assert.equal(execution.url,'/api/actions/manual_execute');assert.equal(JSON.parse(execution.options.body).review_id,'intent-review');assert.equal(JSON.parse(execution.options.body).physical_clearance,true);
  assert.equal(h.get('manual-execution-dialog').open,false);h.get('manual-confirm').events.click();assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,1,'a confirmation is submitted at most once');
  h.app.destroy();
}
{
  const {h,value,post,ready}=await pendingManualExecution();h.reply(post,{job_id:'intent-job'});await settle();
  ready().workflow.review.expires_at_wall_ns=40e9;h.reply(h.requests.length-1,value);await settle();
  assert.match(h.get('manual-confirm-expiry').textContent,/剩余 30 秒/);
  h.advance(1250);h.intervals[1]();assert.match(h.get('manual-confirm-expiry').textContent,/剩余 29 秒/,'countdown advances with local monotonic time between server polls');
  value.revision++;value.server_wall_ns=38.5e9;h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.match(h.get('manual-confirm-expiry').textContent,/剩余 2 秒/,'a new server receipt updates the same absolute deadline without renewal');
  h.advance(1500);h.intervals[1]();
  assert.equal(h.get('manual-execution-dialog').open,false);assert.match(h.get('manual-request-status').textContent,/本次确认已过期，动作未执行；重新点击执行重新检查/);
  assert.match(h.get('request-status').textContent,/本次确认已过期/);
  h.get('manual-confirm').events.click();h.get('manual-confirm').events.click();
  assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,0,'late or repeated confirmation cannot execute an expired review');
  value.revision++;value.server_wall_ns=40.1e9;h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('manual-execution-dialog').open,false);assert.match(h.get('manual-request-status').textContent,/本次确认已过期/,'expiry explanation survives a later poll with the old review');
  h.get('manual-preview').events.click();assert.equal(h.requests.at(-1).url,'/api/actions/manual_preview');assert.doesNotMatch(h.get('manual-request-status').textContent,/本次确认已过期/,'only an explicit new request clears the expired intent');h.app.destroy();
}
for(const status of ['expired','rejected']){
  const {h,value,post,ready}=await pendingManualExecution();h.reply(post,{job_id:'intent-job'});await settle();h.reply(h.requests.length-1,ready());await settle();
  h.get('manual-confirm').events.click();const execution=h.requests.length-1;h.reply(execution,{message:'确认请求已入队'});await settle();
  value.revision++;value.workflow={...value.workflow,status,stage_id:'manual.rejected',busy:false,review:null,message:status==='expired'?'本次确认已过期，动作未执行；重新点击执行重新检查。':'本次确认被拒绝，动作未执行；请重新检查。',primary_action:{id:'manual_tools',allowed:false},outcomes:{motion:status,task:status}};
  value.runtime.last_confirm_rejection={reason:status};h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('stage-title').textContent,'人工动作未执行');assert.equal(h.get('task-status').textContent,status==='expired'?'确认已过期':'未执行');
  assert.match(h.get('manual-request-status').textContent,/动作未执行/);if(status==='rejected')assert.doesNotMatch(h.get('manual-request-status').textContent,/过期/);
  assert.equal(h.get('manual-execution-dialog').open,false);assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,1,'a queue rejection is never automatically retried');h.app.destroy();
}
for(const invalidate of ['dismiss','escape','input','queue','stop','cancel','tab','environment','instance','permission','expiry','stale']){
  const {h,value,post,ready}=await pendingManualExecution({queue:true});h.reply(post,{job_id:'intent-job'});await settle();h.reply(h.requests.length-1,ready());await settle();
  assert.equal(h.get('manual-execution-dialog').open,true);
  if(invalidate==='dismiss')h.get('manual-dismiss').events.click();
  else if(invalidate==='escape')h.get('manual-execution-dialog').events.cancel({preventDefault(){}});
  else if(invalidate==='input'){h.get('jaw-left').value=70;h.get('jaw-left').events.input();}
  else if(invalidate==='queue')h.get('queue-clear').events.click();
  else if(['stop','cancel'].includes(invalidate))void h.app.send(invalidate);
  else if(invalidate==='tab')h.app.selectTab('results');
  else if(invalidate==='environment')h.app.switchContext({environment:'physical',business:'calibration'});
  else if(invalidate==='stale'){h.advance(3100);h.intervals[1]();}
  else{
    value.revision++;if(invalidate==='instance')value.server_instance_id='replacement-instance';
    if(invalidate==='permission')value.workflow.primary_action.allowed=false;
    if(invalidate==='expiry')value.workflow.review=null;
    h.app.poll();h.reply(h.requests.length-1,value);await settle();
  }
  assert.equal(h.get('manual-execution-dialog').open,false,invalidate+' must dismiss confirmation');
  assert.doesNotMatch(h.get('manual-request-status').textContent,/本次确认已过期/,'non-TTL invalidation must not be mislabeled as expiry');
  h.get('manual-confirm').events.click();assert.equal(h.requests.filter(r=>r.url==='/api/actions/manual_execute').length,0,invalidate+' cannot execute');
  if(['dismiss','escape'].includes(invalidate)){
    value.revision++;h.app.poll();h.reply(h.requests.length-1,value);await settle();assert.equal(h.get('manual-execution-dialog').open,false,'polling cannot reopen a dismissed review');
  }
  h.app.destroy();
}
{
  const {h,value,post,ready}=await pendingManualExecution();h.reply(post,{job_id:'intent-job'});await settle();
  ready().workflow.review.request_job_id='another-job';h.reply(h.requests.length-1,value);await settle();assert.equal(h.get('manual-execution-dialog').open,false,'an unrelated request cannot open confirmation');
  h.get('jaw-left').value=72;h.get('jaw-left').events.input();value.revision++;value.workflow.review.request_job_id='intent-job';
  h.app.poll();h.reply(h.requests.length-1,value);await settle();assert.equal(h.get('manual-execution-dialog').open,false,'an edited pending intent cannot be resurrected by a late matching review');h.app.destroy();
}

{
  const h=harness();h.app.start();assert.match(h.requests[0].url,/environment=fake&business=calibration/);
  h.reply(0,snapshot());await settle();assert.equal(h.get('stage-id').textContent,'head.plan');assert.equal(h.get('task-progress').hidden,true);
  h.app.poll();const old=h.requests.at(-1);h.app.selectTab('collection');assert.equal(old.options.signal.aborted,true);
  h.reply(2,snapshot({revision:5,workflow:{id:'collection-now',revision:5,status:'running',stage_id:'collection.observe',title:'观测中',primary_action:{id:'generate_plan',label:'重建规划',allowed:false,reason:'正在采集新的观测'}}}));await settle();
  old.resolve(response(snapshot({revision:99,workflow:{title:'迟到的标定成功',stage_id:'old.complete'}})));await settle();
  assert.equal(h.get('stage-id').textContent,'collection.observe','old context must never replace current stage');
  assert.equal(h.get('primary').disabled,true);assert.match(h.get('primary-reason').textContent,/新的观测/);
  h.app.poll();h.reply(3,snapshot({revision:4,workflow:{stage_id:'older.plan'}}));await settle();assert.equal(h.get('stage-id').textContent,'collection.observe','lower revision must not overwrite current stage');
  h.app.selectTab('results');assert.equal(h.app.context.business,'collection');assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0,'browsing never hands off or executes');
  h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();
  const slow=h.app.send('generate_plan');assert.match(h.get('request-status').textContent,/请求已发送/);assert.equal(h.get('primary').disabled,true);
  const stop=h.app.send('stop');assert.equal(h.requests.length,3,'STOP sent while generation POST remains unresolved');
  assert.equal(h.requests[2].url,'/api/actions/stop');assert.match(h.get('request-status').textContent,/请求停止/);
  const stopBody=JSON.parse(h.requests[2].options.body);assert.equal(stopBody.environment,'fake');assert(stopBody.request_id);assert.equal(h.requests[2].options.headers['X-CSRF-Token'],'fixture-only');
  h.reply(2,{message:'停止请求已受理'});await stop;await settle();
  h.reply(3,snapshot({revision:3,workflow:{id:'fixture-run',revision:3,status:'stopped',stage_id:'stop.confirmed',title:'已停止',primary_action:{id:'generate_plan',allowed:false,reason:'需要重新审阅'}}}));await settle();
  h.reply(1,{message:'迟到：计划生成完成'});await slow;await settle();assert.equal(h.get('stage-id').textContent,'stop.confirmed');assert.doesNotMatch(h.get('request-status').textContent,/迟到/);
  h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot({media:[{id:'head',label:'头部',kind:'actual',url:'/media/head.jpg',frame_id:1,source_wall_ns:9e9,ready:true}]}));await settle();
  assert.equal(h.requests.length,2);h.reply(1,{});await settle();assert.equal(h.get('media-head').querySelector('img').hidden,false);
  h.advance(2200);h.intervals[1]();assert(h.get('media-head').classList.contains('stale'),'source older than three seconds is grey');assert.match(h.get('media-head').querySelector('.media-meta').textContent,/冻结/);
  h.app.poll();h.reply(2,snapshot({revision:2,server_wall_ns:12e9,media:[{id:'head',kind:'actual',url:'/media/head.jpg',frame_id:2,source_wall_ns:12e9,ready:true}]}));await settle();
  h.nextFrame();const frame=h.requests.at(-1);h.app.switchContext({environment:'physical',business:'calibration'});assert.equal(frame.options.signal.aborted,true,'old media request aborted on context switch');frame.resolve(response({}));await settle();assert.equal(h.get('media-head').querySelector('img').hidden,true,'late media cannot enter new environment');
  h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot({runtime:{connected:true,motion_status:'running',allowed_operations:{stop:{allowed:true},mesh:{allowed:false,reason:'渲染器不支持'}}},workflow:{outcomes:{motion:'running',data:'failed',task:'unknown'}}}));await settle();
  assert.match(h.get('alert').textContent,/录制失败，动作仍在执行/);assert.equal(h.get('outcome-task').textContent,'未知');assert.match(h.get('mesh-reason').textContent,/不支持/);assert.equal(h.get('mesh').checked,false);
  assert.match(html,/开启会占用渲染资源，降低预览性能/);assert.match(h.get('mesh-state').textContent,/请求：关闭 · 后端：已关闭/);
  h.app.destroy();
}
{
  const h=harness();h.app.start();
  const approved=snapshot({workflow:{id:'fixture-run',revision:1,primary_action:{id:'execute',label:'执行已审阅计划',allowed:true},review:{id:'single-use-review',summary:'fixture preview',physical_clearance_required:false}}});
  h.reply(0,approved);await settle();const execution=h.app.send('execute');
  assert.equal(JSON.parse(h.requests[1].options.body).review_id,'single-use-review');assert.equal(JSON.parse(h.requests[1].options.body).physical_clearance,true,'fake approval comes from the explicit execute click');
  h.reply(1,{message:'受理'});await execution;h.reply(2,{...approved,revision:2});await settle();
  assert.equal(h.get('primary').disabled,true,'a cached old review cannot become executable after a mutation');assert.match(h.get('primary-reason').textContent,/审阅已失效/);
  h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot({media:[{id:'head',kind:'actual',url:'/media/head.jpg',frame_id:1,source_wall_ns:10e9,ready:true}]}));await settle();
  assert.equal(h.timeouts.size,1,'only request watchdog, no accumulated frame work');
  h.reply(1,{}, {'ETag':'"frame-2"','X-Frame-Id':'2','X-Source-Wall-Ns':String(10e9)});await settle();
  assert.equal(h.nextFrame(),40,'default 25 Hz completion-based pacing');assert.equal(h.requests[2].options.headers['If-None-Match'],'"frame-2"');
  const previous=h.get('media-head').querySelector('img').src;
  h.requests[2].resolve({ok:false,status:304,headers:{get:()=>null},blob:()=>{throw Error('304 must not decode');}});await settle();
  assert.equal(h.get('media-head').querySelector('img').src,previous);h.advance(3100);h.intervals[1]();assert.match(h.get('media-head').querySelector('.media-meta').textContent,/冻结/,'304 must not refresh source age');
  h.nextFrame();h.requests.at(-1).resolve({ok:false,status:503});await settle();assert.equal(h.get('media-head').querySelector('img').src,previous);assert.match(h.get('media-head').querySelector('.media-meta').textContent,/503/);
  h.document.hidden=true;h.document.events.visibilitychange();assert.equal(h.timeouts.size,0,'hidden document cancels pending frame timer');
  h.app.destroy();
}
{
  const h=harness();h.app.start();const value=snapshot();value.runtime.allowed_operations.install={allowed:true,reason:''};
  value.results=[{id:'head-1',label:'头部候选',kind:'head',status:'quality_failed',install:{allowed:false,reason:'缺少通过安装门槛的双腕',head_result_id:'head-1',wrist_result_id:null}}];
  h.reply(0,value);await settle();let buttons=h.get('results-list').querySelectorAll('button');
  assert.equal(buttons[0].disabled,false,'quality failure never hides read-only view');assert.equal(buttons[1].disabled,true);
  assert(h.get('results-list').querySelectorAll('p').some(p=>p.textContent.includes('安装门槛')));
  value.revision=2;value.results[0].install={allowed:true,head_result_id:'head-1',wrist_result_id:'wrist-1'};
  h.app.poll();h.reply(1,value);await settle();buttons=h.get('results-list').querySelectorAll('button');
  assert.equal(buttons[1].disabled,false);const install=buttons[1].events.click();assert.equal(h.requests[2].url,'/api/actions/install');
  assert.equal(JSON.parse(h.requests[2].options.body).result_id,'head-1');h.reply(2,{message:'安装请求受理'});await install;h.app.destroy();
}
{
  const h=harness();h.app.start();const value=snapshot();
  value.workflow.overall_progress={current:4,total:7,unit:'阶段'};
  value.workflow.replay_context={result_id:'wrist-1',batch_id:'batch/capture',frame_index:0,frame_count:3,camera:'left',cameras:['head','left','right'],overlay_available:true,overlay:false};
  value.workflow.replay=[{id:'frame-1',kind:'replay',ready:true,url:'/api/result-media?business=calibration&result_id=frame-1',label:'左腕回放'}];
  h.reply(0,value);await settle();assert.equal(h.get('overall-progress').value,4);assert.match(h.get('overall-label').textContent,/4 \/ 7/);
  assert.equal(h.get('replay-controls').hidden,false);assert.equal(h.get('replay-media').querySelectorAll('img').length,1);
  h.get('replay-index').value=2;h.get('replay-overlay').checked=true;h.get('replay-open').events.click();
  const payload=JSON.parse(h.requests[1].options.body);assert.equal(payload.result_id,'wrist-1');assert.equal(payload.frame_index,2);assert.equal(payload.camera,'left');assert.equal(payload.overlay,true);
  assert.equal(h.requests[1].url,'/api/actions/select_result');h.app.destroy();
}
{
  const h=harness();h.app.start();const value=snapshot();
  value.workflow.progress={current:.5,total:1,kind:'fraction',unit:'求解工作量'};
  value.workflow.overall_progress={current:2.5,total:7,completed_stages:2,unit:'阶段'};
  h.reply(0,value);await settle();assert.equal(h.get('overall-progress').value,2.5);assert.match(h.get('overall-label').textContent,/35.7%/);assert.match(h.get('progress-label').textContent,/50.0%/);
  value.revision=2;value.workflow.progress={indeterminate:true};value.workflow.overall_progress.indeterminate=true;
  h.app.poll();h.reply(1,value);await settle();assert.equal(h.get('task-progress').hidden,false);assert.match(h.get('overall-label').textContent,/工作量未知/);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=snapshot();
  value.results=[{id:'recording-1',kind:'recording',label:'头部录制',status:'failed',media:{id:'recording-1',kind:'replay',ready:true,mime_type:'video/mp4',url:'/api/result-media?business=collection&result_id=recording-1'}}];
  value.workflow.replan_action={id:'generate_plan',label:'重新观测并规划',allowed:true};
  h.reply(0,value);await settle();assert.equal(h.get('replay-media').querySelectorAll('video').length,1,'retained failed-run video is replayable without a quality gate');
  assert.equal(h.get('results-list').querySelectorAll('button').length,0,'recordings do not call unsupported select_result');
  assert.equal(h.get('collection-tools').hidden,false);const replan=h.get('replan').events.click();assert.equal(JSON.parse(h.requests[1].options.body).replan,true);
  h.reply(1,{message:'重新规划受理'});await replan;h.app.destroy();
}
{
  const h=harness();h.app.start();const value=manualSnapshot();h.reply(0,value);await settle();
  const inputs=h.get('joint-inputs').querySelectorAll('input'),positions=inputs.filter(i=>i.dataset.role==='position'),speeds=inputs.filter(i=>i.dataset.role==='speed');
  assert.equal(positions.length,12);assert.equal(speeds.length,12);
  positions[0].value=degrees(.25);speeds[0].value=.05;h.get('jaw-speed').value=5;h.get('queue-add').events.click();
  const queued=JSON.parse(h.requests[1].options.body);assert.equal(h.requests[1].url,'/api/actions/manual_queue');assert.equal(queued.operation,'add');assert.equal(queued.queue_revision,4);assert.equal(queued.waypoint.q[0],.25);assert.equal(queued.waypoint.speeds[0],.05);assert.equal(queued.waypoint.jaw_speed_mm_s,5);
  h.reply(1,{message:'队列已保存'});await settle();value.revision=2;value.manual.queue={available:true,revision:5,waypoints:[queued.waypoint]};h.reply(2,value);await settle();
  assert.equal(h.get('manual-queue-list').querySelectorAll('li').length,1);assert.equal(h.get('queue-preview').disabled,false);
  h.get('queue-preview').events.click();assert.equal(h.requests[3].url,'/api/actions/manual_queue_preview');assert.equal(JSON.parse(h.requests[3].options.body).queue_revision,5);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=manualSnapshot();
  value.workflow.primary_action={id:'manual_execute',label:'确认人工轨迹',allowed:true};value.workflow.review={id:'old-manual',summary:'old target',physical_clearance_required:false};
  h.reply(0,value);await settle();h.get('manual-speed').value=.12;h.get('apply-manual-speed').events.click();h.get('jaw-speed').value=7;
  assert.equal(h.get('primary').disabled,true,'editing speed revokes the displayed manual approval');
  h.get('zero-preview').events.click();assert.equal(h.requests.length,1,'filling home targets never sends motion');
  assert(h.get('joint-inputs').querySelectorAll('input').filter(i=>i.dataset.role==='position').every(i=>Number(i.value)===0));assert.equal(h.get('jaw-left').value,83);assert.equal(h.get('jaw-right').value,83);
  h.get('manual-preview').events.click();const body=JSON.parse(h.requests[1].options.body);
  assert.deepEqual(body.q12,Array(12).fill(0));assert.deepEqual(body.speeds12,Array(12).fill(.12));assert.deepEqual(body.jaw_gap_mm,[83,83]);assert.equal(body.jaw_speed_mm_s,7);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=snapshot();value.arrival_statistics={error:'未配置原 C 到位统计'};
  h.reply(0,value);await settle();assert.equal(h.get('arrival-rows').children.length,0);assert.match(h.get('arrival-status').textContent,/未配置/);
  value.revision=2;value.arrival_statistics={robot_id:'fake-fixture',robot_source:'simulated',total_count:1,raw_path:'/existing/c/samples.jsonl',summary_path:'/existing/c/summary.json',joints:{left:[{count:1,mean_rad:.001,std_rad:0,rms_rad:.001,abs_p95_rad:.001,abs_max_rad:.001}],right:[]}};
  h.app.poll();h.reply(1,value);await settle();assert.equal(h.get('arrival-rows').children.length,1);assert.equal(h.get('arrival-rows').children[0].children[2].textContent,'0.057');assert.match(h.get('arrival-path').textContent,/existing\/c\/samples/);h.app.destroy();
}
function calibrationSnapshot(){
  const value=snapshot();value.workflow.status='idle';
  for(const action of ['configure_calibration','plan_batch','preview_batch','fit_source','fit_batch','generate_plan'])value.runtime.allowed_operations[action]={allowed:true};
  value.workflow.calibration_tools={available:true,workflow:{kind:'head',head_result_id:null},heads:[{id:'head-accepted',label:'已通过头部'}],batches:[{id:'head-batch',kind:'head',completion:{captured:3}},{id:'wrist-batch',kind:'wrist',completion:{captured:2}}],fit_source_id:'head-batch',queue_revision:7,counts:{captured:3,valid:2,failed:1},waypoints:[{id:'sample',q:Array(12).fill(0),speeds:Array(12).fill(.2),jaw_gap_mm:[80,80],capture:true,cell:[0,0]},{id:'transit',q:Array(12).fill(.01),capture:false}]};
  return value;
}
{
  const h=harness();h.app.start();h.reply(0,calibrationSnapshot());await settle();
  h.get('calibration-kind').value='wrist';h.get('calibration-kind').events.change();assert.equal(h.get('configure-calibration').disabled,true);
  h.get('calibration-head').value='head-accepted';h.get('calibration-head').events.change();assert.equal(h.get('plan-batch').disabled,true,'new draft dependency must be applied explicitly');
  h.get('configure-calibration').events.click();assert.equal(h.requests[1].url,'/api/actions/configure_calibration');assert.equal(JSON.parse(h.requests[1].options.body).head_result_id,'head-accepted');h.app.destroy();
}
for(const [button,path,field,expected]of [['wrist-redo','generate_plan','mode','wrist'],['preview-batch','preview_batch','queue_revision',7],['apply-fit-source','fit_source','batch_id','head-batch'],['clear-fit-source','fit_source','batch_id',null],['fit-batch','fit_batch',null,null],['plan-batch','plan_batch',null,null]]){
  const h=harness();h.app.start();h.reply(0,calibrationSnapshot());await settle();
  if(button==='wrist-redo'){h.get('calibration-head').value='head-accepted';h.get('calibration-head').events.change();}
  assert.equal(h.get(button).disabled,false);h.get(button).events.click();assert.equal(h.requests[1].url,'/api/actions/'+path);
  const payload=JSON.parse(h.requests[1].options.body);if(field)assert.equal(payload[field],expected);
  if(button==='wrist-redo')assert.equal(payload.head_result_id,'head-accepted');
  if(button==='fit-batch')assert.equal(payload.batch_id,'head-batch');
  if(button==='preview-batch')assert(!payload.review_id,'creating a batch preview must never execute');
  h.app.destroy();
}
{
  const h=harness();h.app.start();const value=calibrationSnapshot();
  value.workflow.calibration_tools.installed={head_id:'head-accepted',wrist_id:'wrist-match',verified:false,problems:['active hash mismatch'],path:'/local/active'};
  value.results=[{id:'head-accepted',label:'头部结果',kind:'head',status:'quality_failed',details:{metrics_rectified_px:{holdout:{rms_px:4}},head_result_id:'source-head'},download_url:'/api/calibration-file?environment=fake&result_id=head-accepted'}];
  h.reply(0,value);await settle();assert.match(h.get('calibration-plan-summary').textContent,/1 个采样点 · 1 个过渡点/);assert.equal(h.get('calibration-waypoints').children.length,2);
  assert.equal(h.get('fit-source').children.length,2,'only head batches enter current solver selector');
  assert.equal(h.get('results-list').querySelectorAll('a')[0].href,'http://localhost:8110/api/calibration-file?environment=fake&result_id=head-accepted');
  const details=h.get('results-list').querySelector('details');details.open=true;
  value.revision++;h.app.poll();h.reply(1,value);await settle();assert.equal(h.get('results-list').querySelector('details'),details,'polling must preserve open result details');
  assert(h.get('installed-details').children.some(row=>row.children[1].textContent==='未通过'));
  assert.equal(h.get('results-list').querySelectorAll('button')[0].disabled,false,'quality failure must not gate readonly result view');h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot({workflow:{status:'unconfigured'}}));await settle();
  assert.equal(h.get('task-status').textContent,'业务服务未配置');assert.doesNotMatch(html,/原 C|C 动作|C 后端/);h.app.destroy();
}
function previewSnapshot(){
  const value=manualSnapshot();for(const action of ['preview_target','preview_overview','preview_mesh','preview_recover'])value.runtime.allowed_operations[action]={allowed:true};
  value.preview={connected:true,renderer_id:'renderer-one',status:'ready',revisions:{target:7,overview:3,mesh:4},
    frames:Object.fromEntries(['target','overview','mesh'].map(view=>[view,{ready:false,url:'/api/preview-media?environment=fake&view='+view,revision:{target:7,overview:3,mesh:4}[view]}])),
    mesh:{requested:false,applied:false,camera:'head',candidate_id:null},recovery:{allowed:false,reason:'无需重启'}};
  value.results=[{id:'poor-head',kind:'head',label:'未通过头部',status:'quality_failed'},{id:'other-head',kind:'head',label:'另一个头部',status:'quality_failed'},{id:'poor-wrist',kind:'wrist',label:'未通过双腕',status:'quality_failed',details:{head_result_id:'poor-head'}}];
  return value;
}
for(const [business,disabledDraft]of [['calibration',''],['collection',89],['inference',null],['research',null]]){
  const value=previewSnapshot();value.manual.disabled_grippers=['right'];value.manual.jaw_gaps_mm=[80,null];
  const draft=JSON.stringify({jaw:[80,disabledDraft]});
  const h=harness({business,storage:{getItem:()=>draft,setItem(){}}});h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('jaw-right').disabled,true);assert.equal(Number(h.get('jaw-right').value),83,'unknown disabled jaw gets a display target, never a claimed measurement');
  const input=h.get('joint-inputs').querySelectorAll('input').find(input=>input.dataset.role==='position');
  input.value=degrees(.2);input.events.input();h.fireTimer(ms=>ms===33);await settle();
  const post=h.requests.find(row=>row.options.method==='POST');assert.equal(post.url,'/api/actions/preview_target');
  const body=JSON.parse(post.options.body);assert.equal(body.business,business);assert.equal(body.q12[0],.2);assert.deepEqual(body.jaw_gap_mm,[80,83]);
  h.get('jaw-left').value='90';h.get('show-target').events.click();
  assert.match(h.get('request-status').textContent,/夹爪目标必须/,'invalid enabled targets are still rejected');
  assert(h.requests.filter(row=>row.options.method==='POST').every(row=>row.url==='/api/actions/preview_target'),'editing and previewing cannot execute, recover or change ownership');h.app.destroy();
}
const previewHeaders=(view,revision,extra={})=>({'X-Frame-Metadata':JSON.stringify({environment:'fake',view,revision,view_revision:3,renderer_id:'renderer-one',frame_id:'frame-'+revision,source_wall_ns:10e9,...extra})});
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.frames.target.ready=true;
  h.reply(0,value);await settle();h.reply(1,{},previewHeaders('target',7));await settle();
  const image=h.get('media-preview').querySelector('img'),previous=image.src;
  value.revision++;value.preview.revisions.overview++;value.preview.frames.target.ready=false;
  h.app.poll();h.reply(2,value);await settle();
  assert.equal(image.src,previous);assert.equal(image.hidden,false);assert(!h.revoked.includes(previous));
  assert.match(h.get('media-preview').querySelector('.media-meta').textContent,/保留上一帧/);
  value.revision++;value.preview.renderer_id='replacement-renderer';h.app.poll();h.reply(3,value);await settle();
  assert.equal(image.hidden,true,'a frame must not survive a renderer identity change');assert(h.revoked.includes(previous));h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.frames.target.ready=true;
  h.reply(0,value);await settle();h.reply(1,{},previewHeaders('target',7));await settle();
  assert.equal(h.get('overview-controls').hidden,false,'camera controls are available without a separate view mode');
  const image=h.get('media-preview').querySelector('img'),previous=image.src;
  image.events.pointerdown({clientX:100,clientY:100,pointerId:1,preventDefault(){}});
  image.events.pointermove({clientX:120,clientY:100});image.events.pointerup();
  assert.equal(h.requests[2].url,'/api/actions/preview_overview','the default target preview can rotate directly');
  assert.equal(image.hidden,false);assert.equal(image.src,previous,'camera updates keep the last target image visible');
  assert(h.requests.filter(row=>row.options.method==='POST').every(row=>row.url==='/api/actions/preview_overview'));h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();h.reply(0,value);await settle();
  h.get('overview-reset').events.click();h.reply(1,{client_revision:4});await settle();
  value.revision++;value.preview.revisions.overview=4;value.preview.frames.overview={...value.preview.frames.overview,ready:true,revision:4,view_revision:4};
  h.reply(2,value);await settle();h.reply(3,{},previewHeaders('overview',4,{view_revision:4}));await settle();
  const image=h.get('media-preview').querySelector('img'),previous=image.src;
  assert.equal(image.draggable,false,'native image dragging must not steal the orbit gesture');
  let prevented=0;image.events.dragstart({preventDefault:()=>prevented++});
  image.events.pointerdown({clientX:100,clientY:100,pointerId:1,preventDefault:()=>prevented++});
  image.events.pointermove({clientX:130,clientY:110});image.events.pointerup();assert.equal(prevented,2);
  assert.equal(h.requests[4].url,'/api/actions/preview_overview');assert.notEqual(JSON.parse(h.requests[4].options.body).yaw,-Math.PI/2);
  assert.equal(image.hidden,false);assert.equal(image.src,previous,'dragging retains the last completed view during the POST');
  h.reply(4,{client_revision:5});await settle();value.revision++;value.preview.revisions.overview=5;
  h.reply(5,value);await settle();h.reply(6,{},previewHeaders('overview',4,{view_revision:4}));await settle();assert.equal(image.src,previous,'old frame metadata cannot satisfy the new view');
  h.nextFrame();h.reply(7,{},previewHeaders('overview',5,{view_revision:5}));await settle();
  assert.notEqual(image.src,previous);assert(h.revoked.includes(previous));assert.equal(image.hidden,false);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();h.reply(0,value);await settle();
  h.get('overview-reset').events.click();h.reply(1,{client_revision:4});await settle();
  h.reply(3,{},previewHeaders('overview',4,{view_revision:4}));await settle();
  value.revision++;h.reply(2,value);await settle();h.nextFrame();
  h.reply(4,{},previewHeaders('overview',4,{view_revision:4}));await settle();
  assert.doesNotMatch(h.get('media-preview').querySelector('.media-meta').textContent,/不匹配|读取失败/,'a late state projection cannot regress an acknowledged view revision');
  assert.equal(h.get('media-preview').querySelector('img').hidden,false);h.app.destroy();
}
function physicalMeshSnapshot(){
  const value=previewSnapshot();value.device.environment='physical';value.preview.capabilities={mesh:true};
  Object.assign(value.runtime,{calibration_revision:3,calibration_snapshot_id:'physical-installed-group'});
  const details={environment:'hardware',robot_source:'physical',simulation_only:false};
  value.results=[{id:'simulation-head',kind:'head',label:'v99',details:{...details,robot_source:'simulated',simulation_only:true}},
    {id:'unproven-head',kind:'head',label:'v0'},
    {id:'history-head',kind:'head',label:'v3',status:'quality_failed',details:{...details,version:'v3'}},
    {id:'installed-head',kind:'head',label:'v5',status:'passed',details:{...details,version:'v5'},install:{installed:true,wrist_result_id:'installed-wrist'}},
    {id:'installed-wrist',kind:'wrist',label:'v6',status:'quality_failed',details:{...details,version:'v6',head_result_id:'installed-head'}}];
  value.results.push({...value.results.at(-1)});
  return value;
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.capabilities={top_view:true};h.reply(0,value);await settle();
  assert.equal(h.get('preview-top-view').disabled,false);h.get('preview-top-view').checked=true;h.get('preview-top-view').events.change();
  assert.equal(JSON.parse(h.requests[1].options.body).top_view,true);
  h.reply(1,{client_revision:4});await settle();
  const image=h.get('media-preview').querySelector('img');image.events.pointerdown({clientX:100,clientY:100,pointerId:1,preventDefault(){}});
  image.events.pointermove({clientX:120,clientY:100});
  h.reply(3,{},previewHeaders('overview',4,{view_revision:4,top_view:true}));await settle();
  assert.equal(h.get('media-preview').classList.contains('split'),true);
  let previous=image.src;
  for(let i=0;i<3;i++){
    const post=h.requests.findLastIndex(row=>row.options.method==='POST');h.reply(post,{client_revision:5+i});await settle();
    image.events.pointermove({clientX:140+20*i,clientY:100});
    const frame=h.requests.findLastIndex(row=>row.url.includes('preview-media'));
    h.reply(frame,{},previewHeaders('overview',5+i,{view_revision:5+i,top_view:true}));await settle();
    assert.notEqual(image.src,previous,'continuous drag presents new frames before pointerup');previous=image.src;
  }
  image.events.pointerup();assert(h.requests.filter(row=>row.options.method==='POST').every(row=>row.url.endsWith('/preview_overview')));
  h.app.destroy();
}
{
  const value=physicalMeshSnapshot(),rows=meshRows(value);
  assert.deepEqual(rows.map(row=>row.id),['installed-head','installed-wrist','history-head']);
  assert.equal(rows[0].label,'当前安装 · 头部 v5');assert.equal(rows[1].label,'当前安装 · 双腕 v6');
  assert.equal(rows[2].status,'quality_failed','viewing a historical candidate does not impose the install quality gate');
  value.preview.mesh_candidates=[value.results[2]];
  assert.deepEqual(meshRows(value).map(row=>row.id),['history-head'],'dedicated mesh directory takes precedence');
  const fake=previewSnapshot();assert.deepEqual(meshRows(fake),fake.results,'legacy fake uses its original candidate directory');
}
{
  const h=harness({physical:true});h.app.start();const value=physicalMeshSnapshot();
  value.preview.mesh.candidate_id='simulation-head';h.reply(0,value);await settle();
  assert.equal(h.get('mesh-candidate').value,'installed-head','unusable old selection defaults to the physical installed head');
  assert.deepEqual(h.get('mesh-candidate').children.map(row=>row.value),['','installed-head','history-head']);
  assert.equal(h.get('mesh').checked,false);assert.equal(h.requests.length,1,'loading defaults never enables or starts mesh');
  h.get('mesh-candidate').value='history-head';h.get('mesh-candidate').events.change();
  h.reply(1,{client_revision:5});await settle();value.revision++;value.preview.mesh.candidate_id='history-head';value.preview.revisions.mesh=5;
  h.reply(2,value);await settle();assert.equal(h.get('mesh-candidate').value,'history-head','a valid user-selected historical candidate survives polling');
  h.get('mesh').checked=true;h.get('mesh-camera').value='all';h.get('mesh-camera').events.change();
  assert.equal(h.get('mesh-candidate').value,'','missing historical wrist dependency must not silently use an unrelated installed pair');
  assert.equal(h.requests.length,3,'a missing association never dispatches an unrelated candidate');h.app.destroy();
}
{
  const h=harness({physical:true});h.app.start();const value=physicalMeshSnapshot();
  value.preview.mesh={requested:true,applied:true,camera:'head',candidate_id:'installed-head'};
  h.reply(0,value);await settle();
  for(const [camera,candidate] of [['left','installed-wrist'],['right','installed-wrist'],['all','installed-wrist'],['head','installed-head']]){
    const index=h.requests.length;h.get('mesh-camera').value=camera;h.get('mesh-camera').events.change();
    const sent=JSON.parse(h.requests[index].options.body);assert.equal(sent.candidate_id,candidate);assert.equal(sent.enabled,true);
    h.reply(index,{client_revision:index+10});await settle();value.revision++;
    value.preview.mesh={requested:true,applied:true,camera,candidate_id:candidate};value.preview.revisions.mesh=index+10;
    h.reply(index+1,value);await settle();assert.equal(h.get('mesh').checked,true);
  }
  h.app.destroy();
}
{
  const h=harness({physical:true});h.app.start();h.app.selectTab('collection');
  assert.equal(h.requests[0].options.signal.aborted,true);
  const collection=physicalMeshSnapshot();collection.results=[];collection.workflow.stage_id='collection.idle';
  collection.preview.mesh={requested:true,applied:true,camera:'all',candidate_id:'installed-wrist'};
  h.reply(1,collection);await settle();assert.equal(h.app.state.workflow.stage_id,'collection.idle','main state renders without waiting for the extra catalog');
  const directory=h.requests[2];assert.match(directory.url,/business=calibration/);assert.equal(directory.options.method,undefined);
  h.reply(2,physicalMeshSnapshot());await settle();
  assert.equal(h.get('mesh-candidate').value,'installed-wrist');assert.equal(h.get('mesh').checked,true);
  assert.deepEqual(h.app.state.results,[],'the calibration catalog does not replace collection results');
  assert.equal(h.app.state.preview.mesh_candidates.length,6);
  h.app.poll();h.reply(3,{...collection,revision:2,preview:{...collection.preview,mesh_candidates:undefined}});await settle();
  assert.equal(h.requests.length,4,'same service/install identity reuses the catalog');
  h.app.selectTab('calibration');h.reply(4,physicalMeshSnapshot());await settle();
  h.app.selectTab('collection');h.reply(5,{...collection,revision:3,preview:{...collection.preview,mesh_candidates:undefined}});await settle();
  assert.equal(h.requests.length,6,'business changes reuse the same physical directory');
  assert.equal(h.get('mesh-candidate').value,'installed-wrist');assert.equal(h.get('mesh').checked,true);h.app.destroy();
}
for(const changed of ['server','route','calibration','environment']){
  const h=harness({physical:true});h.app.start();h.app.selectTab('collection');
  const value=physicalMeshSnapshot();value.results=[];
  h.reply(1,value);await settle();const old=h.requests[2];
  if(changed==='environment'){
    h.app.switchContext({environment:'fake',business:'collection'});h.reply(3,previewSnapshot());await settle();
  }else{
    const newer=physicalMeshSnapshot();newer.results=[];newer.revision=2;
    if(changed==='server')newer.server_instance_id='new-physical-server';
    if(changed==='route')newer.gateway_route={route_id:'a'.repeat(32)};
    if(changed==='calibration')newer.runtime.calibration_revision++;
    h.app.poll();h.reply(3,newer);await settle();
    assert.equal(h.requests.length,5,'new identity gets its own catalog request');
    const result={...newer,results:physicalMeshSnapshot().results.map(row=>({...row,id:'new-'+row.id}))};
    h.reply(4,result);await settle();
  }
  assert.equal(old.options.signal.aborted,true,'old catalog request is aborted on '+changed+' change');
  old.resolve(response(physicalMeshSnapshot()));await settle();
  assert(!h.get('mesh-candidate').children.some(row=>row.value==='installed-head'),'late catalog cannot cross '+changed+' identity');
  h.app.destroy();
}
{
  const h=harness({physical:true});h.app.start();h.app.selectTab('collection');
  const value=physicalMeshSnapshot();value.results=[];h.reply(1,value);await settle();
  h.reply(2,{...physicalMeshSnapshot(),server_instance_id:'unrelated-calibration-server'});await settle();
  assert.equal(h.get('mesh-candidate').value,'');assert.match(h.app.state.preview.mesh_catalog_error,/已变化的服务或安装/);
  assert.equal(h.app.state.server_instance_id,'fixture-server','catalog lookup cannot change the accepted main service identity');
  h.app.poll();h.reply(3,{...value,revision:2,preview:{...value.preview,mesh_candidates:undefined}});await settle();
  assert.equal(h.requests.length,4,'a failed lookup is not retried on every status poll; calibration view can refresh it');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.frames.target.ready=true;value.preview.frames.target.source_wall_ns=4e9;
  h.reply(0,value);await settle();h.reply(1,{},previewHeaders('target',7,{source_wall_ns:4e9}));await settle();
  assert.equal(h.get('media-preview').classList.contains('stale'),false,'a matching static target remains valid after its original frame is three seconds old');
  assert.match(h.get('media-preview').querySelector('.media-meta').textContent,/静态目标 · 生成于 6.0 s 前/);
  h.advance(3100);h.intervals[1]();assert(h.get('media-preview').classList.contains('stale'),'a static target still expires when state is not refreshed');
  value.revision=2;value.server_wall_ns=14e9;h.app.poll();h.reply(2,value);await settle();
  assert.equal(h.get('media-preview').classList.contains('stale'),false,'fresh matching state revalidates the static target without repainting');
  assert.match(h.get('media-preview').querySelector('.media-meta').textContent,/生成于 10.0 s 前/);assert.equal(h.app.state.preview.frames.target.source_wall_ns,4e9,'state freshness must not rewrite the original generation time');
  h.app.poll();h.requests[3].resolve({ok:false,status:503});await settle();assert(h.get('media-preview').classList.contains('stale'),'state connection failure greys a static target');
  value.revision=3;value.preview.connected=false;h.app.poll();h.reply(4,value);await settle();assert(h.get('media-preview').classList.contains('stale'),'renderer disconnection also invalidates a static target');h.app.destroy();
}
{
  const media={preview_view:'mesh',revision:3,renderer_id:'one',camera:'left',candidate_id:'poor'};
  const meta={view:'mesh',revision:3,renderer_id:'one',environment:'fake',camera:'left',candidate_id:'poor',source_wall_ns:10e9};
  assert.equal(verifyPreviewFrame(media,meta,'fake'),meta);
  for(const change of [{revision:2},{view:'target'},{environment:'physical'},{renderer_id:'old'},{camera:'right'},{candidate_id:'other'},{source_wall_ns:null}])assert.throws(()=>verifyPreviewFrame(media,{...meta,...change},'fake'));
  const target={...media,preview_view:'target',view_revision:9},targetMeta={...meta,view:'target',view_revision:9};
  assert.equal(verifyPreviewFrame(target,targetMeta,'fake'),targetMeta);
  assert.throws(()=>verifyPreviewFrame(target,{...targetMeta,view_revision:8},'fake'),/观察视角修订/);
  const bound={...target,minimum_revision:2,revision:5,server_instance_id:'server',display_context:'installed-current'};
  const completed={...targetMeta,revision:3,server_instance_id:'server',display_context:'installed-current'};
  assert.equal(verifyPreviewFrame(bound,completed,'fake'),completed,'same-context completed targets may trail the requested revision');
  for(const change of [{revision:1},{revision:6},{server_instance_id:'old-server'},{display_context:'old-install'},{renderer_id:'old-renderer'}])assert.throws(()=>verifyPreviewFrame(bound,{...completed,...change},'fake'));
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.display_context='queue-context';h.reply(0,value);await settle();
  const positions=h.get('joint-inputs').querySelectorAll('input').filter(row=>row.dataset.role==='position');
  h.get('show-target').events.click();assert.equal(h.get('show-target').disabled,false,'the shared latest-input queue remains available while HTTP is in flight');
  positions[0].value=degrees(.2);positions[0].events.input();h.get('show-target').events.click();positions[0].value=degrees(.3);positions[0].events.input();h.get('show-target').events.click();
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'ordinary target edits use the bounded presentation queue');
  h.reply(1,{client_revision:8});await settle();assert.equal(JSON.parse(h.requests[3].options.body).q12[0],.3,'only the latest unsent ordinary target is dispatched');
  h.reply(3,{client_revision:9});await settle();positions[0].value=degrees(.4);h.get('show-target').events.click();h.reply(4,{client_revision:10});await settle();
  positions[0].value=degrees(.5);h.get('show-target').events.click();positions[0].value=degrees(.6);h.get('show-target').events.click();
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,3,'three slots include accepted, downloading and awaiting presentation');
  const headers=revision=>previewHeaders('target',revision,{display_context:'queue-context',server_instance_id:'fixture-server'});
  assert.match(h.requests[2].url,/revision=8/);h.reply(2,{},headers(8));await settle();const firstImage=h.get('media-preview').querySelector('img').src;
  h.fireTimer(ms=>ms===0);assert.match(h.requests[5].url,/revision=9/);h.reply(5,{},headers(9));await settle();
  h.fireTimer(ms=>ms===0);assert.match(h.requests[6].url,/revision=10/);h.reply(6,{},headers(10));await settle();
  assert.equal(h.get('media-preview').querySelector('img').src,firstImage,'prefetched revisions cannot replace the frame before its two RAF callbacks');
  h.paint();h.advance(16);h.paint();await settle();assert.notEqual(h.get('media-preview').querySelector('img').src,firstImage);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,4);
  const next=JSON.parse(h.requests[7].options.body);assert.equal(next.q12[0],.6);assert.equal(next.presented_through_revision,8);assert.equal(next.display_context,'queue-context');assert.equal(next.view_revision,3);
  assert.equal(h.requests.filter(row=>row.url.startsWith('/api/state')).length,1,'target ACKs do not trigger full business state polls');
  h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.display_context='installed-a';value.preview.target_floor=0;
  h.reply(0,value);await settle();h.get('show-target').events.click();h.get('show-target').events.click();h.app.poll();
  value.revision=2;value.runtime.calibration_revision=2;value.preview.display_context='installed-b';value.preview.target_floor=8;
  h.reply(2,value);await settle();assert.equal(h.requests[1].options.signal.aborted,true,'installation changes cancel the ordinary target flight and pending input');
  h.reply(1,{client_revision:8,display_context:'installed-a',renderer_id:'renderer-one'});await settle();
  assert.equal(h.get('media-preview').querySelector('img').hidden,true);assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'old pending target is never replayed into the new installed context');h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();
  value.workflow.review={id:'approved-motion',summary:'already reviewed',physical_clearance_required:true};value.workflow.primary_action={id:'manual_execute',allowed:true};
  h.reply(0,value);await settle();h.get('clearance').checked=true;
  const request=h.get('show-target').events.click();assert.equal(h.requests[1].url,'/api/actions/preview_target');
  assert.deepEqual(JSON.parse(h.requests[1].options.body).q12,Array(12).fill(.1));assert.equal(h.get('clearance').checked,true,'read-only display must not revoke a motion review');
  h.reply(1,{client_revision:8,token:null});await request;await settle();
  assert.equal(h.requests.length,3,'exact frame fetch uses the ACK without a global state poll');
  assert.match(h.requests[2].url,/revision=8/);h.reply(2,{},previewHeaders('target',7));await settle();
  assert.equal(h.get('media-preview').querySelector('img').hidden,true,'an exact request cannot be satisfied with another completed revision');
  h.nextFrame();h.reply(3,{},previewHeaders('target',8));await settle();const completedImage=h.get('media-preview').querySelector('img').src;
  h.paint();h.advance(16);h.paint();await settle();h.reply(4,{},previewHeaders('target',7));await settle();assert.equal(h.get('media-preview').querySelector('img').src,completedImage,'completed revisions must never move backwards');
  assert.match(h.get('media-preview').querySelector('.media-meta').textContent,/倒退/);
  h.nextFrame();h.reply(5,{},previewHeaders('target',8));await settle();assert.equal(h.get('media-preview').querySelector('img').src,completedImage);assert.equal(h.get('media-preview').classList.contains('stale'),false,'a verified duplicate clears a transient fetch error without replacing the image');
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,1,'target image never invokes motion preview or confirm');h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,previewSnapshot());await settle();
  h.get('show-target').events.click();const stop=h.app.send('stop');
  assert.equal(h.requests[1].url,'/api/actions/preview_target');assert.equal(h.requests[2].url,'/api/actions/stop','slow renderer must never delay the independent stop request');
  h.reply(2,{message:'停止已受理，等待保持回执'});await stop;h.reply(1,{client_revision:8,message:'迟到的目标显示回执'});await settle();
  assert.doesNotMatch(h.get('request-status').textContent,/迟到的目标/);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.mesh={requested:true,applied:true,camera:'head',candidate_id:'poor-head'};value.preview.frames.mesh.ready=true;
  h.reply(0,value);await settle();const old=h.requests[1];
  assert(h.get('mesh-candidate').children.some(row=>row.value==='poor-head'),'quality failed candidates remain selectable');
  h.get('mesh-candidate').value='other-head';const change=h.get('mesh-candidate').events.change();assert.equal(h.requests[2].url,'/api/actions/preview_mesh');assert.equal(old.options.signal.aborted,true);
  old.resolve(response({},previewHeaders('mesh',4,{camera:'head',candidate_id:'poor-head'})));await settle();assert.equal(h.get('media-mesh').querySelector('img').hidden,true,'late previous candidate never reappears');
  h.reply(2,{client_revision:5});await change;await settle();value.revision=2;value.preview.mesh.candidate_id='other-head';value.preview.revisions.mesh=5;value.preview.frames.mesh.revision=5;h.reply(3,value);await settle();
  h.reply(4,{},previewHeaders('mesh',5,{camera:'head',candidate_id:'poor-head'}));await settle();assert.equal(h.get('media-mesh').querySelector('img').hidden,true,'metadata candidate must match current selection');
  h.nextFrame();h.reply(5,{},previewHeaders('mesh',5,{camera:'head',candidate_id:'other-head'}));await settle();assert.equal(h.get('media-mesh').querySelector('img').hidden,false);
  h.get('mesh').checked=false;h.get('mesh').events.change();assert.equal(JSON.parse(h.requests[6].options.body).enabled,false);assert.equal(h.get('media-mesh').querySelector('img').hidden,true);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();h.reply(0,value);await settle();
  h.get('overview-reset').events.click();assert.equal(h.requests[1].url,'/api/actions/preview_overview');
  h.get('overview-zoom').value=1.5;h.get('overview-zoom').events.input();h.get('overview-zoom').value=2;h.get('overview-zoom').events.input();assert.equal(h.requests.length,2,'overview requests are serialized and coalesced');
  h.reply(1,{client_revision:4});await settle();assert.match(h.requests[3].url,/preview-media/);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'the next view waits for a rendered frame, not just its HTTP ACK');
  h.reply(3,{},previewHeaders('overview',4,{view_revision:4}));await settle();
  assert.equal(h.requests[4].url,'/api/actions/preview_overview');assert.equal(JSON.parse(h.requests[4].options.body).zoom,2);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.connected=false;value.preview.error='renderer process unavailable';value.preview.recovery={allowed:false,reason:'后端未允许',message:'检查本会话渲染日志',commands:['review renderer.log']};
  h.reply(0,value);await settle();h.app.selectTab('diagnostics');assert.equal(h.get('image-recovery').hidden,false);assert.equal(h.get('preview-recover').href,'#operations-panel');
  h.get('preview-recover').events.click({preventDefault(){}});assert.equal(h.get('operations-panel').hidden,false,'service recovery opens the unique operations workspace without a business action');
  const reconnect=h.get('reconnect-images').events.click();assert.match(h.requests[1].url,/\/api\/state/);h.reply(1,{...value,revision:2});await reconnect;
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0,'browser recovery only rereads state and frames');
  assert.equal(h.get('image-recovery-commands').textContent,'review renderer.log');
  value.preview.recovery.allowed=true;value.revision=3;h.app.poll();h.reply(2,value);await settle();assert.equal(typeof h.get('preview-recover').events.click,'function');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot({device:{id:'protocol-fake',environment:'fake'}}));await settle();
  assert.match(h.get('preview-status').textContent,/协议 Fake（L1）/);assert.match(h.get('media-head').querySelector('.empty').textContent,/不提供 PhysX 相机图像/);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.results=[];value.preview.connected=false;
  value.runtime.allowed_operations.preview_mesh={allowed:false,reason:'渲染服务尚未配置'};
  h.reply(0,value);await settle();assert.equal(h.get('mesh').disabled,true);
  assert.equal(h.get('mesh-reason').textContent,'渲染服务尚未配置');h.app.destroy();
}
function deletionSnapshot(){
  const value=calibrationSnapshot();for(const action of ['delete_preview','delete_confirm','delete_cancel'])value.runtime.allowed_operations[action]={allowed:true};
  value.results=[{id:'capture:head-batch',kind:'capture',label:'头部批次',status:'completed'}];return value;
}
{
  const h=harness();h.app.start();const value=deletionSnapshot();h.reply(0,value);await settle();
  const buttons=h.get('results-list').querySelectorAll('button');assert.equal(buttons[1].textContent,'预览删除原始批次');
  const previewRequest=buttons[1].events.click();let body=JSON.parse(h.requests[1].options.body);
  assert.equal(h.requests[1].url,'/api/actions/delete_preview');assert.equal(body.result_id,'capture:head-batch');assert.equal(body.scope,'batch');
  h.reply(1,{message:'请核对范围',job_id:'delete-preview-job'});await previewRequest;
  value.revision=2;value.workflow.deletion_preview={id:'legacy-preview-one',scope:'all',root:'/fixture/results',batches:['head-batch'],image_files:2,files:5,bytes:4096,preserved_versions:['H001','W001'],preserved_installed:{head_id:'installed-head',wrist_id:'installed-wrist'},preserved:['关节到位统计日志与汇总','当前 active 安装组'],clears:['所选求解批次','实时叠加选择'],detail:'仅本地批次',valid:true};
  h.reply(2,value);await settle();assert.equal(h.get('deletion-preview').hidden,false);assert.equal(h.get('delete-confirm').disabled,true);
  assert(h.get('deletion-details').querySelectorAll('dd').some(row=>row.textContent.includes('关节到位统计')));
  assert(h.get('deletion-details').querySelectorAll('dd').some(row=>row.textContent.includes('所选求解批次')));
  h.get('delete-clearance').checked=true;h.get('delete-clearance').events.change();assert.equal(h.get('delete-confirm').disabled,false);
  value.revision=3;value.workflow.deletion_preview.valid=false;value.workflow.deletion_preview.reason='预览已过期';h.app.poll();h.reply(3,value);await settle();assert.equal(h.get('delete-confirm').disabled,true);
  value.revision=4;value.workflow.deletion_preview={...value.workflow.deletion_preview,id:'legacy-preview-two',valid:true,reason:null};h.app.poll();h.reply(4,value);await settle();assert.equal(h.get('delete-clearance').checked,false,'new scope preview always needs a new explicit confirmation');
  h.get('delete-clearance').checked=true;h.get('delete-clearance').events.change();const confirmation=h.get('delete-confirm').events.click();
  body=JSON.parse(h.requests[5].options.body);assert.equal(h.requests[5].url,'/api/actions/delete_confirm');assert.equal(body.delete_preview_id,'legacy-preview-two');assert.equal(body.confirmed,true);assert.equal(body.token,undefined,'token is never entered through a generic input');assert.equal(body.scope,undefined,'confirmation cannot expand the server-owned scope');
  h.reply(5,{message:'删除请求已受理',job_id:'delete-job'});await confirmation;value.revision=5;h.reply(6,value);await settle();assert.equal(h.get('delete-confirm').disabled,true,'old cached preview must not allow a second confirmation');
  value.revision=6;value.workflow.deletion_preview=null;value.workflow.deletion_result={message:'已删除原始批次，标定版本保留'};value.results=[];h.app.poll();h.reply(7,value);await settle();assert.equal(h.get('deletion-preview').hidden,true);assert.match(h.get('deletion-result').textContent,/标定版本保留/);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=deletionSnapshot();h.reply(0,value);await settle();
  assert.equal(h.get('delete-all').disabled,false,'an idle selected fit source does not add a new all-delete gate');
  h.get('delete-all').events.click();assert.equal(JSON.parse(h.requests[1].options.body).scope,'all');assert.equal(h.requests[1].url,'/api/actions/delete_preview');h.app.destroy();
}
{
  const h=harness();h.app.start();const value=deletionSnapshot();value.runtime.allowed_operations.delete_preview={allowed:false,reason:'采集任务进行中'};value.runtime.allowed_operations.delete_confirm={allowed:false,reason:'采集任务进行中'};
  value.workflow.deletion_preview={id:'pending-delete',valid:true,batches:['head-batch'],preserved:[]};h.reply(0,value);await settle();h.get('delete-clearance').checked=true;h.get('delete-clearance').events.change();
  assert.equal(h.get('delete-all').disabled,true);assert.equal(h.get('delete-confirm').disabled,true);h.get('delete-cancel').events.click();assert.equal(h.requests[1].url,'/api/actions/delete_cancel');assert.equal(h.requests.filter(row=>row.url==='/api/actions/delete_confirm').length,0);h.app.destroy();
}
{
  const h=harness();h.app.start();const old=snapshot({server_instance_id:'server-old',revision:80,csrf:'old-csrf'});
  old.workflow={id:'old-run',status:'awaiting_review',stage_id:'manual.review',primary_action:{id:'manual_execute',allowed:true},review:{id:'old-review',physical_clearance_required:true}};
  old.media=[{id:'head',kind:'actual',url:'/api/media/head',ready:true,frame_id:'old-frame',source_wall_ns:10e9}];
  h.reply(0,old);await settle();h.get('clearance').checked=true;
  const actionIndex=h.requests.length,oldAction=h.app.send('manual_execute');h.app.poll();
  const refreshIndex=h.requests.length-1;h.reply(refreshIndex,{...old,revision:81});await settle();const oldFrame=h.requests.at(-1);
  h.get('clearance').checked=true;h.get('delete-clearance').checked=true;h.app.poll();
  const fresh=snapshot({server_instance_id:'server-new',revision:1,csrf:'new-csrf'});
  fresh.workflow={id:'new-run',status:'awaiting_review',stage_id:'new.manual.review',primary_action:{id:'manual_execute',allowed:true},review:{id:'new-review',physical_clearance_required:true}};
  fresh.media=[{...old.media[0],frame_id:'new-frame'}];h.reply(h.requests.length-1,fresh);await settle();const freshFrame=h.requests.at(-1);
  assert.equal(h.app.state.server_instance_id,'server-new');assert.equal(h.app.state.revision,1,'new instance accepts a reset revision counter');
  assert.equal(h.get('clearance').checked,false);assert.equal(h.get('delete-clearance').checked,false);assert.equal(h.get('primary').disabled,true,'old approvals cannot authorize the new instance');
  assert.equal(h.requests[actionIndex].options.signal.aborted,true);assert.equal(oldFrame.options.signal.aborted,true);
  oldFrame.resolve(response({}, {'X-Frame-Id':'old-frame','X-Source-Wall-Ns':String(10e9)}));h.reply(actionIndex,{message:'迟到的旧动作已完成'});await oldAction;await settle();
  assert.equal(h.get('media-head').querySelector('img').hidden,true);assert.doesNotMatch(h.get('request-status').textContent,/迟到/);
  freshFrame.resolve(response({}, {'X-Frame-Id':'new-frame','X-Source-Wall-Ns':String(10e9)}));await settle();assert.equal(h.get('media-head').querySelector('img').hidden,false);
  h.app.poll();h.reply(h.requests.length-1,{...old,revision:9999});await settle();assert.equal(h.app.state.server_instance_id,'server-new','a retired instance can never return with a larger revision');assert.equal(h.app.state.csrf,'new-csrf');
  h.app.poll();h.reply(h.requests.length-1,{...fresh,revision:0,workflow:{stage_id:'stale-new'}});await settle();assert.equal(h.get('stage-id').textContent,'new.manual.review','same-instance stale revisions remain rejected');
  const stopIndex=h.requests.length,stop=h.app.send('stop');assert.equal(h.requests[stopIndex].options.headers['X-CSRF-Token'],'new-csrf');h.reply(stopIndex,{message:'new stop accepted'});await stop;h.app.destroy();
}
{
  const h=harness();h.app.start();const old=previewSnapshot();old.server_instance_id='display-old';old.revision=100;old.preview.frames.target.ready=true;
  h.reply(0,old);await settle();const oldFrame=h.requests[1],displayIndex=h.requests.length,oldDisplay=h.get('show-target').events.click();
  h.app.poll();const fresh=previewSnapshot();fresh.server_instance_id='display-new';fresh.csrf='display-new-csrf';fresh.preview.revisions.target=1;fresh.preview.frames.target={...fresh.preview.frames.target,ready:true,revision:1};
  h.reply(h.requests.length-1,fresh);await settle();const newFrame=h.requests.at(-1);
  assert.equal(h.requests[displayIndex].options.signal.aborted,true);oldFrame.resolve(response({},previewHeaders('target',7)));
  h.reply(displayIndex,{client_revision:999,message:'old preview'});await oldDisplay;await settle();assert.equal(h.get('media-preview').querySelector('img').hidden,true);
  newFrame.resolve(response({},previewHeaders('target',1)));await settle();assert.equal(h.get('media-preview').querySelector('img').hidden,false,'old display response must not restore its revision floor');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'server restart must not replay an old action');h.app.destroy();
}
{
  const inputs=Array.from({length:30},(_,input_id)=>({input_id,input_ms:input_id*33,...(input_id<20?{submitted_ms:input_id*33+40,presented_ms:input_id*33+150}:{coalesced:true,cancelled:'coalesced'})}));
  const window={status:'completed',duplicates:2,inputs,frames:[{input_id:0,presented_ms:150},{input_id:0,presented_ms:61000}]};
  const summary=previewAuditSummary(window);
  assert.equal(summary.raw_inputs,30);assert.equal(summary.dispatched_inputs,20);assert.equal(summary.coalesced_inputs,10);assert.equal(summary.other_undispatched_inputs,0);
  assert.equal(summary.new_display_fps,1/60);assert.equal(summary.matched_inputs,20);assert.equal(summary.unmatched_inputs,0);
  assert.equal(summary.latency_p95_ms,150,'latency starts at the selected original input, including its 40 ms queue delay');assert.equal(summary.latency_p95_unbounded,false,'never-dispatched coalesced samples are not timed-out requests');
  for(const input of inputs.slice(18,20)){delete input.presented_ms;input.cancelled='after dispatch';}
  const missing=previewAuditSummary(window);assert.equal(missing.matched_inputs,18);assert.equal(missing.unmatched_inputs,2);assert.equal(missing.cancelled_requests,2);
  assert.equal(missing.latency_p95_ms,null);assert.equal(missing.latency_p95_unbounded,true);assert.equal(missing.latency_max_unbounded,true,'dispatched but unpresented requests must remain in the tail');
  assert.equal(summary.passed,undefined,'the diagnostic must not silently declare qualification');
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.frames.target.ready=true;h.reply(0,value);await settle();h.reply(1,{},previewHeaders('target',7));await settle();
  assert.equal(h.get('preview-audit').hidden,true,'normal UI does not show instrumentation');assert.equal(h.nextFrame(),40,'target fetch pacing must not impose a 10 Hz ceiling on the approved 20 FPS profile');h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();h.reply(0,value);await settle();
  assert.equal(h.get('preview-audit').hidden,false);assert.equal(h.get('audit-start').disabled,false);h.get('audit-start').events.click();
  assert.equal(h.requests[1].url,'/api/actions/preview_target');h.advance(5);h.reply(1,{client_revision:8});await settle();
  h.advance(10);h.reply(2,{},previewHeaders('target',8));await settle();h.paint();h.advance(16);h.paint();
  const displayed=h.get('media-preview').querySelector('img').src,blobCount=h.blobs.length;
  h.reply(3,{},previewHeaders('target',8));await settle();assert.equal(h.get('media-preview').querySelector('img').src,displayed);assert.equal(h.blobs.length,blobCount,'identical source frames never decode into a new Blob');
  h.get('audit-stop').events.click();h.get('audit-export').events.click();const evidence=JSON.parse(await h.blobs.at(-1).text()),window=evidence.windows[0];
  assert.match(evidence.presentation_endpoint,/two requestAnimationFrame/);assert.match(evidence.presentation_endpoint,/not display photons/);
  assert.equal(evidence.measurement_population,'dispatched display targets');
  assert.equal(window.summary.matched_inputs,1);assert.equal(window.summary.duplicate_frames,1);assert.equal(window.inputs[0].presented_ms,31);assert.equal(window.inputs[0].acknowledged_ms,5);
  assert.equal(window.summary.latency_max_ms,31);assert.equal(window.summary.new_display_fps,null,'interrupted windows never count as a completed 60 s test');
  assert.equal(window.frames[0].source_wall_ns,10e9);assert(!JSON.stringify(evidence).includes('fixture-only'),'evidence must not include CSRF');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1);assert.equal(h.get('audit-download').hidden,false);h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();value.preview.frames.target.ready=true;h.reply(0,value);await settle();
  const latest=h.requests[1];h.get('audit-start').events.click();assert.equal(latest.options.signal.aborted,true,'starting a FIFO cancels an older unversioned latest fetch');
  h.reply(2,{client_revision:8});await settle();latest.resolve(response({},previewHeaders('target',8)));await settle();
  assert.equal(h.get('media-preview').querySelector('img').hidden,true,'late latest pixels cannot bypass the exact-revision FIFO');
  h.reply(3,{},previewHeaders('target',8));await settle();h.paint();h.advance(16);h.paint();
  h.get('audit-stop').events.click();h.get('audit-export').events.click();const evidence=JSON.parse(await h.blobs.at(-1).text());
  assert.equal(evidence.windows[0].summary.matched_inputs,1);assert.equal(evidence.windows[0].summary.unmatched_inputs,0);h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();h.reply(0,previewSnapshot());await settle();h.get('audit-start').events.click();h.reply(1,{client_revision:8});await settle();
  h.advance(34);h.fireTimer(ms=>Math.abs(ms-1000/30)<.01);h.reply(3,{client_revision:9});await settle();
  h.advance(34);h.fireTimer(ms=>Math.abs(ms-1000/30)<.01);h.reply(4,{client_revision:10});await settle();
  h.reply(2,{},previewHeaders('target',8));await settle();h.fireTimer(ms=>ms===0);
  h.requests[5].resolve({ok:false,status:503});await settle();h.paint();h.advance(16);h.paint();
  h.get('audit-stop').events.click();h.get('audit-export').events.click();const evidence=JSON.parse(await h.blobs.at(-1).text());
  assert.equal(evidence.windows[0].summary.matched_inputs,1,'a later prefetch error cannot cancel the valid head frame presentation');
  assert.equal(evidence.windows[0].summary.unmatched_inputs,2);assert.equal(evidence.windows[0].summary.latency_p95_unbounded,true,'unfinished dispatched siblings remain in the latency population');h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();value.preview.frames.target.ready=true;h.reply(0,value);await settle();
  h.reply(1,{},previewHeaders('target',7));await settle();h.get('audit-start').events.click();h.reply(2,{client_revision:8});await settle();
  h.reply(3,{},previewHeaders('target',8,{frame_id:'frame-7'}));await settle();h.paint();h.advance(16);h.paint();
  h.get('audit-stop').events.click();h.get('audit-export').events.click();const evidence=JSON.parse(await h.blobs.at(-1).text());
  assert.match(h.get('media-preview').querySelector('.media-meta').textContent,/身份复用/,'a reused source frame is rejected even if its revision metadata changed');
  assert.equal(evidence.windows[0].summary.matched_inputs,0);assert.equal(evidence.windows[0].summary.latency_p95_unbounded,true);h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();value.manual.positions[0]=1;value.manual.positions[1]=.9999;h.reply(0,value);await settle();
  h.get('audit-start').events.click();h.get('audit-stop').events.click();h.get('audit-export').events.click();const evidence=JSON.parse(await h.blobs.at(-1).text());
  assert.equal(evidence.profile.joint_index,1,'a joint exactly at its limit is not moved by recentering the scripted profile');
  assert.equal(evidence.profile.center_rad,.9999);assert(evidence.profile.amplitude_rad<=.00005);h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();h.reply(0,value);await settle();h.get('audit-start').events.click();
  h.advance(34);h.fireTimer(ms=>Math.abs(ms-1000/30)<.01);h.advance(34);h.fireTimer(ms=>Math.abs(ms-1000/30)<.01);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'input generation keeps one target request in flight');
  h.app.poll();value.revision=2;value.runtime.calibration_revision=99;h.reply(h.requests.length-1,value);await settle();
  assert.match(h.get('audit-status').textContent,/中止/);assert.equal(h.requests[1].options.signal.aborted,true);h.get('audit-export').events.click();
  const evidence=JSON.parse(await h.blobs.at(-1).text()),summary=evidence.windows[0].summary;assert.equal(summary.raw_inputs,3);assert.equal(summary.dispatched_inputs,1);assert.equal(summary.coalesced_inputs,1);assert.equal(summary.other_undispatched_inputs,1);assert.equal(summary.cancelled_requests,1);assert.equal(summary.unmatched_inputs,1);assert.equal(summary.latency_p95_unbounded,true);h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();h.reply(0,value);await settle();h.get('audit-start').events.click();
  for(let i=0;i<3;i++){h.advance(62000);h.app.poll();value.revision++;h.reply(h.requests.length-1,value);await settle();h.fireTimer(ms=>Math.abs(ms-1000/30)<.01);await settle();}
  assert.match(h.get('audit-status').textContent,/采集结束/);h.get('audit-export').events.click();const evidence=JSON.parse(await h.blobs.at(-1).text());
  assert.equal(evidence.windows.length,3);assert.equal(evidence.profile.window_seconds,60);assert.equal(evidence.profile.tail_wait_seconds,2);assert.equal(evidence.status,'completed');
  assert(evidence.windows.every(row=>row.summary.new_display_fps===0&&row.summary.latency_p95_unbounded));
  assert(h.requests.filter(row=>row.options.method==='POST').every(row=>row.url==='/api/actions/preview_target'));h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();value.manual.positions[0]=1.000000001;h.reply(0,value);await settle();
  assert.equal(h.get('audit-start').disabled,true,'raw feedback outside the command domain is not silently clamped');
  h.get('zero-preview').events.click();assert.equal(h.get('audit-start').disabled,false);h.get('audit-start').events.click();
  h.get('audit-stop').events.click();h.get('audit-export').events.click();const evidence=JSON.parse(await h.blobs.at(-1).text());
  assert.deepEqual(evidence.profile.base.q12,Array(12).fill(0));assert.equal(evidence.profile.base_source,'explicit manual display inputs');
  assert.equal(value.manual.positions[0],1.000000001,'display diagnostics do not rewrite measured feedback');h.app.destroy();
}
{
  const originalWidth=globalThis.innerWidth;globalThis.innerWidth=729;
  const h=harness({audit:true});h.app.start();h.reply(0,previewSnapshot());await settle();h.get('audit-start').events.click();
  globalThis.innerWidth=730;h.advance(34);h.fireTimer(ms=>Math.abs(ms-1000/30)<.01);
  assert.match(h.get('audit-status').textContent,/测量视口已改变/);assert.equal(h.requests[1].options.signal.aborted,true);h.app.destroy();
  if(originalWidth===undefined)delete globalThis.innerWidth;else globalThis.innerWidth=originalWidth;
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();
  value.manual.jaw_command_ranges_mm=[[0,80],[0,89]];value.manual.jaw_feedback_ranges_mm=[[-3,83],[-3,92]];
  value.manual.jaw_gaps_mm=[80,89];h.reply(0,value);await settle();
  assert.equal(Number(h.get('jaw-left').value),80);assert.equal(Number(h.get('jaw-right').value),89);
  assert.equal(Number(h.get('jaw-left').max),80);assert.equal(Number(h.get('jaw-right').max),89);
  h.get('zero-preview').events.click();assert.equal(Number(h.get('jaw-right').value),89);
  h.get('jaw-left').value=81;h.get('show-target').events.click();assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0);
  h.get('use-feedback').events.click();assert.equal(Number(h.get('jaw-right').value),89,'readback must not close an ARX13 jaw to the old 83mm limit');h.app.destroy();
}
{
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();value.manual.jaw_gaps_mm=[83.00000236797332,-.000002];h.reply(0,value);await settle();
  assert.equal(h.get('jaw-left').value,83);assert.equal(h.get('jaw-right').value,0);assert.equal(h.get('audit-start').disabled,false);
  assert.deepEqual(h.app.state.manual.jaw_gaps_mm,[83.00000236797332,-.000002],'command fields must not mutate measured feedback');
  h.get('jaw-left').value='83.1';h.get('show-target').events.click();assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'an explicit command outside 0–83 is rejected, not clamped');
  value.manual.jaw_gaps_mm=[-2,87];h.get('use-feedback').events.click();assert.equal(h.get('jaw-left').value,0);assert.equal(h.get('jaw-right').value,83);
  assert.deepEqual(value.manual.jaw_gaps_mm,[-2,87]);h.get('audit-start').events.click();assert.deepEqual(JSON.parse(h.requests[1].options.body).jaw_gap_mm,[0,83]);h.app.destroy();
  for(const accepted of [85.019,87,88.5,89]){
    const valid=harness({audit:true});valid.app.start();const observed=previewSnapshot();observed.manual.jaw_gaps_mm=[accepted,80];valid.reply(0,observed);await settle();
    assert.equal(valid.get('use-feedback').disabled,false);assert.equal(valid.get('audit-start').disabled,false);assert.equal(valid.get('jaw-left').value,83);
    assert.equal(valid.app.state.manual.jaw_gaps_mm[0],accepted);assert.match(valid.get('manual-feedback').textContent,/反馈范围 -2–89 mm，命令目标仍为 0–83 mm/);valid.app.destroy();
  }
  for(const invalid of [-2.00001,89.000001,NaN,Infinity]){
    const rejected=harness({audit:true});rejected.app.start();const bad=previewSnapshot();bad.manual.jaw_gaps_mm=[invalid,80];rejected.reply(0,bad);await settle();
    assert.equal(rejected.get('use-feedback').disabled,true);assert.equal(rejected.get('audit-start').disabled,true);assert.equal(rejected.get('jaw-left').value,'','invalid feedback is never silently converted into a target');
    assert(Object.is(rejected.app.state.manual.jaw_gaps_mm[0],invalid));
    bad.revision=2;bad.manual.jaw_gaps_mm=[83.00000236797332,80];rejected.app.poll();rejected.reply(1,bad);await settle();
    assert.equal(rejected.get('jaw-left').value,83,'valid recovered feedback initializes the previously unavailable command fields');rejected.app.destroy();
  }
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.manual.position_noise_rad=.005;value.manual.positions[0]=1.004;
  h.reply(0,value);await settle();h.get('show-target').events.click();
  assert.equal(JSON.parse(h.requests[1].options.body).q12[0],1.004,'raw boundary input reaches backend for recorded normalization');
  assert.equal(h.app.state.manual.positions[0],1.004);h.app.destroy();
  const bad=harness();bad.app.start();value.manual.positions[0]=1.006;bad.reply(0,value);await settle();bad.get('show-target').events.click();
  assert.equal(bad.requests.filter(row=>row.options.method==='POST').length,0,'beyond-tolerance position stays rejected');bad.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');
  const value=collectionSnapshot();Object.assign(value.workflow,{busy:false,status:'ready',speed_scale:.3,review:{id:'speed-30',speed_scale:.3},primary_action:{id:'execute',allowed:true},trajectory:{id:'plan-one',timeline_digest:'frozen-plan',url:'/api/result-media?result_id=plan-one',speed_scale:.3}});
  value.runtime.allowed_operations.generate_plan={allowed:true};h.reply(1,value);await settle();
  h.get('collection-speed').value='170';h.get('collection-speed').events.input();assert.equal(h.get('primary').disabled,true);
  const update=h.get('review-speed').events.click();assert.equal(JSON.parse(h.requests[2].options.body).speed_scale,1.7);
  h.reply(2,{message:'新速度已审阅'});await update;value.revision++;value.workflow.speed_scale=1.7;value.workflow.review={id:'speed-170',speed_scale:1.7};h.reply(3,value);await settle();
  assert.equal(h.get('primary').disabled,false);assert.equal(h.get('collection-speed-value').textContent,'170%');
  const target={left_arm_joints:Array(6).fill(0),right_arm_joints:Array(6).fill(.1),left_gripper:65,right_gripper:83};
  const plan={timeline_digest:'frozen-plan',ticks:[{tick:0,time_s:0,target},{tick:1,time_s:.033,target}],stages:[{name:'close',start_tick:1}]};
  const load=h.get('trajectory-load').events.click();h.reply(4,plan);await load;
  assert.equal(h.get('trajectory-view').hidden,false);assert.equal(h.get('trajectory-joints').children.length,6);assert.equal(h.get('trajectory-jaws').children.length,2);
  h.get('trajectory-stage').value='1';h.get('trajectory-stage').events.change();assert.match(h.get('trajectory-position').textContent,/tick 1.*0.033/);assert.match(h.get('trajectory-target').textContent,/65.000 mm/);
  const stale=h.get('trajectory-load').events.click();h.app.poll();value.revision++;value.workflow.trajectory={...value.workflow.trajectory,id:'plan-two'};h.reply(6,value);await settle();
  assert.equal(h.requests[5].options.signal.aborted,true);h.reply(5,plan);await stale;assert.equal(h.get('trajectory-view').hidden,true);
  const bad=h.get('trajectory-load').events.click();h.reply(7,{...plan,timeline_digest:'another-plan'});await bad;
  assert.match(h.get('trajectory-status').textContent,/身份/);assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'trajectory inspection never issues motion');h.app.destroy();
}
async function pendingSceneRestore(offsets=[20,-10],reviewOffsets=offsets,planningWaitMs=0){
  const saved=new Map(),h=harness({storage:{getItem:key=>saved.get(key),setItem:(key,value)=>saved.set(key,value)}});
  h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');
  const value=manualSnapshot();value.runtime.holder='collection';
  Object.assign(value.runtime.allowed_operations,{return_cup:{allowed:true},collection_plan:{allowed:true},generate_plan:{allowed:true}});
  Object.assign(value.workflow,{status:'ready',busy:false,stage_id:'collection.idle',speed_scale:.97,collection:{},review:null});
  h.reply(1,value);await settle();h.get('collection-speed').value='106';h.get('collection-speed').events.input();
  offsets.forEach((n,i)=>{const input=h.get(i?'return-offset-y':'return-offset-x');input.value=String(n);input.events.input();});
  assert.equal(h.get('return-cup').disabled,false);assert.equal(h.get('collection-return').hidden,false);
  const request=h.get('return-cup').events.click(),post=h.requests.length-1;
  assert.equal(h.requests[post].url,'/api/actions/return_cup');const body=JSON.parse(h.requests[post].options.body);
  assert.equal(body.speed_scale,undefined,'scene recovery does not submit the normal speed draft');assert.deepEqual(body.offset_xy_mm,offsets);
  assert.equal(h.get('manual-execution-dialog').open,false,'generation request does not confirm movement');
  if(planningWaitMs){h.advance(planningWaitMs);h.intervals[1]();assert.equal(h.get('manual-execution-dialog').open,false);value.server_wall_ns+=planningWaitMs*1e6;}
  h.reply(post,{job_id:'restore-job',message:'恢复计划已生成'});await request;
  value.revision++;Object.assign(value.workflow,{id:'return-one',status:'awaiting_review',stage_id:'collection.return_review',
    primary_action:{id:'execute',allowed:true},review:{id:'return-review',request_job_id:'restore-job',speed_scale:.97,offset_xy_mm:reviewOffsets,expires_at_wall_ns:value.server_wall_ns+30e9,physical_clearance_required:false,summary:'恢复到固定原始杯位附近'},
    collection:{return_task:{status:'awaiting_review',message:'请确认本次恢复',offset_xy_mm:offsets,origin_position_m:[.7,.05,.051],goal_position_m:[.7+offsets[0]/1000,.05+offsets[1]/1000,.051],duration_s:37.5667,parent_speed_scale:.97,parent_plan_id:'successful-parent-plan'}},
    trajectory:{id:'return-path',url:'/api/result-media?result_id=return-path',speed_scale:.97}});
  h.reply(h.requests.length-1,value);await settle();return {h,value};
}
{
  const {h,value}=await pendingSceneRestore();
  assert.equal(h.get('stage-title').textContent,'恢复场景 · 待确认');assert.equal(h.get('primary-action').hidden,true);
  assert.equal(h.get('collection-execute-action').hidden,true);assert.equal(h.get('review-panel').hidden,true,'scene recovery has one confirmation entry');
  assert.equal(h.get('manual-execution-dialog').open,true);assert.equal(h.get('manual-execution-dialog').showCount,1);
  assert.equal(h.get('manual-confirm-title').textContent,'恢复场景');assert.match(h.get('manual-confirm-jaws').textContent,/固定原始杯位.*20 mm.*-10 mm/);
  assert.match(h.get('manual-confirm-jaws').textContent,/本次计划目标 \[720.0, 40.0, 51.0\] mm/);
  assert.match(h.get('collection-return-positions').textContent,/固定原始杯位 \[700.0, 50.0, 51.0\].*本次计划目标 \[720.0, 40.0, 51.0\]/);
  assert.equal(h.get('collection-speed').value,'106');assert.equal(h.get('collection-speed').disabled,true);assert.equal(h.get('review-speed').disabled,true);
  assert.match(h.get('collection-return-timing').textContent,/37.6 秒.*固定原始杯位.*数采速度草稿不参与/);
  assert.equal(h.get('collection-return-source').textContent,'父计划：successful-parent-plan');assert.equal(h.get('collection-stages').hidden,true);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'a ready plan opens confirmation without executing');
  const before=h.requests.length;h.get('review-speed').events.click();assert.equal(h.requests.length,before);
  h.get('manual-confirm').events.click();const executePost=h.requests.length-1;
  assert.equal(h.requests[executePost].url,'/api/actions/execute');const body=JSON.parse(h.requests[executePost].options.body);
  assert.equal(body.speed_scale,.97);assert.equal(body.review_id,'return-review');assert.deepEqual(body.offset_xy_mm,[20,-10]);
  h.reply(executePost,{message:'恢复执行已受理'});await settle();
  value.revision++;value.workflow.status='running';value.workflow.stage_id='collection.return_execute';value.workflow.busy=true;value.workflow.collection.return_task.status='running';
  h.reply(h.requests.length-1,value);await settle();assert.equal(h.get('stage-title').textContent,'正在恢复场景');assert.equal(h.get('collection-speed').value,'106');
  assert.equal(h.get('return-offset-x').disabled,true,'offset draft is locked during motion');
  value.revision++;Object.assign(value.workflow,{status:'completed',busy:false,stage_id:'collection.return_complete',review:null,primary_action:{id:'execute',allowed:false}});
  value.workflow.collection.return_task.status='completed';h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('collection-speed').disabled,false);assert.equal(h.get('collection-speed').value,'106','completed restoration unlocks and preserves the normal draft even while its receipt remains');
  value.revision++;Object.assign(value.workflow,{id:'normal-after-return',status:'ready',stage_id:'collection.plan',primary_action:{id:'collection_plan',allowed:true}});
  h.app.poll();h.reply(h.requests.length-1,value);await settle();assert.equal(h.get('collection-stages').hidden,false);
  h.get('collection-speed').value='106';h.get('collection-speed').events.input();
  const normalPlan=h.get('collection-plan').events.click(),normalPost=h.requests.length-1;
  assert.equal(JSON.parse(h.requests[normalPost].options.body).speed_scale,1.06,'historical return metadata cannot lock normal planning or change its speed');
  assert.equal(JSON.parse(h.requests[normalPost].options.body).offset_xy_mm,undefined);
  h.reply(normalPost,{message:'普通数采计划请求已受理'});await normalPlan;h.app.destroy();
}
for(const invalidate of ['offset','stale','dismiss','tab','instance']){
  const {h,value}=await pendingSceneRestore([0,0]);
  if(invalidate==='offset'){h.get('return-offset-x').value='1';h.get('return-offset-x').events.input();}
  if(invalidate==='stale'){h.advance(3100);h.intervals[1]();assert.equal(h.get('return-cup').disabled,true);}
  if(invalidate==='dismiss')h.get('manual-dismiss').events.click();
  if(invalidate==='tab'){h.app.selectTab('calibration');h.reply(h.requests.length-1,manualSnapshot());await settle();h.app.selectTab('collection');h.reply(h.requests.length-1,value);await settle();}
  if(invalidate==='instance'){value.server_instance_id='new-instance';value.revision=1;h.app.poll();h.reply(h.requests.length-1,value);await settle();}
  assert.equal(h.get('manual-execution-dialog').open,false,invalidate+' revokes the presented restore approval');
  h.get('manual-confirm').events.click();await h.app.send('execute');
  assert.equal(h.requests.filter(row=>row.url==='/api/actions/execute').length,0,invalidate+' cannot execute an old recovery plan');h.app.destroy();
}
{
  const {h}=await pendingSceneRestore([10,5],[0,0]);assert.equal(h.get('manual-execution-dialog').open,false,'a receipt for another offset cannot open confirmation');
  h.get('manual-confirm').events.click();await h.app.send('execute');assert.equal(h.requests.filter(row=>row.url==='/api/actions/execute').length,0);h.app.destroy();
}
{
  const {h,value}=await pendingSceneRestore([0,0],[0,0],120000);
  assert.equal(h.get('manual-execution-dialog').open,true,'a two-minute plan can still open its new matching confirmation');
  assert.match(h.get('manual-confirm-expiry').textContent,/30 秒/);
  value.revision++;value.server_wall_ns+=30.001e9;h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('manual-execution-dialog').open,false,'waiting for planning cannot extend the actual 30-second credential');
  assert.match(h.get('request-status').textContent,/重新点击“恢复场景”/);h.get('manual-confirm').events.click();
  assert.equal(h.requests.filter(row=>row.url==='/api/actions/execute').length,0);h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');const value=manualSnapshot();
  value.runtime.allowed_operations.return_cup={allowed:true};value.workflow.stage_id='collection.idle';h.reply(1,value);await settle();
  for(const invalid of ['51','-51','','1.5','NaN']){
    h.get('return-offset-x').value=invalid;h.get('return-offset-x').events.input();assert.equal(h.get('return-cup').disabled,true);await h.get('return-cup').events.click();
    assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'invalid offset cannot reach the backend');
  }
  h.get('return-offset-x').value='50';h.get('return-offset-y').value='-50';h.get('return-offset-x').events.input();assert.equal(h.get('return-cup').disabled,false);
  h.advance(3100);h.intervals[1]();assert.equal(h.get('return-cup').disabled,true);h.get('return-cup').events.click();assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
  const physical=snapshot({device:{environment:'physical'}});physical.runtime.allowed_operations.return_cup={allowed:true};
  assert.equal(actionPermission(physical,'return_cup').allowed,true);
  physical.runtime.allowed_operations.return_cup={allowed:false,reason:'当前真机还原计划尚未完成观测与路径检查'};
  assert.equal(actionPermission(physical,'return_cup').allowed,false);
}
{
  const h=harness();h.app.start();const value=snapshot();
  value.results=[['source','base_0'],['target','base_0'],['source','left_wrist_0'],['source','right_wrist_0']].map(([role,camera],i)=>({id:String(i),kind:'recording',media:{id:String(i),kind:'replay',ready:true,mime_type:'video/mp4',url:'/api/result-media?id='+i,role,camera}}));
  h.reply(0,value);await settle();h.app.selectTab('results');const videos=h.get('replay-media').querySelectorAll('video');
  videos.forEach((node,i)=>node.duration=[2,4,3,5][i]);videos[0].events.loadedmetadata();
  assert.equal(h.get('collection-replay-toggle').disabled,false);await h.get('collection-replay-toggle').events.click();
  h.advance(1000);h.intervals[2]();assert.equal(videos[0].currentTime,1);assert.equal(videos[1].currentTime,1);
  h.get('collection-replay-speed').value='.5';h.get('collection-replay-speed').events.change();h.advance(2000);h.intervals[2]();
  assert.equal(videos[0].currentTime,2);assert.equal(videos[0].paused,true);assert.equal(videos[1].currentTime,2);assert.equal(videos[1].paused,false);
  h.get('collection-replay-time').value='1';h.get('collection-replay-time').events.input();await settle();assert.equal(videos[0].paused,false,'seeking back resumes a shorter video');
  assert.equal(h.get('replay-media').children[2].hidden,true);
  h.get('collection-replay-left').checked=true;h.get('collection-replay-left').events.change();
  assert.equal(h.get('replay-media').children[2].hidden,false);assert.equal(videos[2].currentTime,1);
  assert.equal(h.get('collection-replay-toggle').disabled,false);assert.equal(videos[2].controls,false);
  assert.equal(h.get('replay-media').children[3].hidden,true);
  h.get('collection-replay-right').checked=true;h.get('collection-replay-right').events.change();
  assert.equal(h.get('replay-media').children[3].hidden,false);
  assert.equal(videos[3].currentTime,1);
  await h.get('collection-replay-toggle').events.click();assert(videos.every(node=>!node.paused));
  h.get('collection-replay-left').checked=false;h.get('collection-replay-left').events.change();
  assert.equal(videos[2].paused,true);assert.equal(h.get('replay-media').children[2].hidden,true);
  let rejectOldPlay;videos[0].play=()=>new Promise((resolve,reject)=>{rejectOldPlay=reject;});
  const oldPlay=h.get('collection-replay-toggle').events.click();h.app.selectTab('calibration');rejectOldPlay(Error('old playback failed'));
  await oldPlay;assert.doesNotMatch(h.get('collection-replay-status').textContent,/old playback failed/,'late play errors cannot replace a different view');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}
{
  const h=harness({physical:true});h.app.start();const value=snapshot({device:{id:'physical-fixture',environment:'physical'}});
  value.runtime.allowed_operations.release={allowed:true};value.runtime.allowed_operations.recover={allowed:true};
  value.runtime.release_receipt={status:'unconfirmed',message:'SDK 尚未断开'};h.reply(0,value);await settle();
  for(const action of ['connect','handoff','release','recover'])await h.app.send(action);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'business page cannot call lifecycle writes even when legacy permissions allow them');
  assert.match(h.get('release-state').textContent,/尚未确认.*SDK/);
  for(const id of ['connect','release','recover'])assert(!htmlIds.has(id),'old lifecycle entry removed');h.app.destroy();
}
{
  const h=harness({physical:true});h.app.start();h.reply(0,snapshot({device:{id:'physical-fixture',environment:'physical'}}));await settle();
  assert.equal(h.get('handoff').disabled,true,'same holder needs no handoff');
  h.get('handoff-target').value='collection';h.get('handoff-target').events.change();
  assert.equal(h.get('handoff').disabled,false);h.get('handoff').events.click();await settle();
  const writes=h.requests.filter(row=>row.options.method==='POST');
  assert.equal(writes.length,1);assert.equal(writes[0].url,'/api/actions/handoff');
  assert.equal(JSON.parse(writes[0].options.body).target_business,'collection');
  assert.equal(h.app.context.business,'calibration','handoff does not navigate or execute');
  h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');
  const value=collectionSnapshot();Object.assign(value.workflow,{busy:false,status:'ready',speed_scale:.3,review:{id:'expiring',speed_scale:.3}});
  h.reply(1,value);await settle();h.get('collection-speed').value='50';h.get('collection-speed').events.input();
  value.revision++;delete value.workflow.review;h.app.poll();h.reply(2,value);await settle();
  assert.equal(h.get('collection-speed').value,'50','review expiry must not overwrite an unsent local speed draft');
  assert.equal(h.get('collection-speed-value').textContent,'50%');h.app.destroy();
}
{
  const h=harness();h.app.start();const value=snapshot();
  value.runtime.state='FAULT_STOP_UNCONFIRMED';value.runtime.reason='SDK feedback age 0.1996s exceeds bound';value.runtime.last_fault_diagnostic={reasons:['feedback_age'],error:'original control failure'};
  value.runtime.allowed_operations.handoff={allowed:false,reason:'holding/control state is not confirmed'};
  h.reply(0,value);await settle();assert.equal(h.get('feedback').textContent,'反馈：健康');
  assert.match(h.get('control-state').textContent,/故障锁定/);assert.equal(h.get('control-fault').hidden,false);
  assert.match(h.get('control-fault-reason').textContent,/SDK feedback age.*original control failure.*feedback_age/);
  const count=h.requests.length;h.get('control-fault-diagnostics').events.click();assert.equal(h.get('diagnostics-pane').hidden,false);assert.equal(h.requests.length,count,'diagnostic navigation must not recover or reconnect');
  value.revision++;value.runtime.state='READY';h.app.poll();h.reply(count,value);await settle();assert.equal(h.get('control-fault').hidden,true,'historical diagnostic must not imply a current fault');h.app.destroy();
  const physical=harness();physical.app.start();physical.app.switchContext({environment:'physical',business:'calibration'});value.device.environment='physical';value.runtime.state='FAULT';physical.reply(1,value);await settle();
  assert.match(physical.get('control-fault-guidance').textContent,/可靠支撑双臂.*中断恢复.*未确认时不要重试/);physical.app.destroy();
}
{
  const saved=new Map(),storage={getItem:key=>saved.get(key),setItem:(key,value)=>saved.set(key,value)};
  const h=harness({storage});h.app.start();const value=previewSnapshot();
  value.workflow.primary_action={id:'manual_execute',allowed:true};value.workflow.review={id:'never-save-this-review',physical_clearance_required:true};
  h.reply(0,value);await settle();h.get('clearance').checked=true;
  const positions=()=>h.get('joint-inputs').querySelectorAll('input').filter(node=>node.dataset.role==='position');
  const slider=h.get('joint-inputs').querySelectorAll('input').find(node=>node.dataset.role==='slider');
  assert.equal(slider.step,'any','non-integral degree limits must not shift slider targets onto a min-anchored step grid');assert.equal(positions()[0].step,'0.05');
  slider.value=30;slider.events.input();assert.equal(Number(positions()[0].value),30);
  assert.equal(h.get('primary').disabled,true,'changing a draft revokes its existing motion review');
  assert.equal(h.get('clearance').checked,false);h.fireTimer(ms=>ms===33);
  assert.equal(h.requests[1].url,'/api/actions/preview_target');
  assert(Math.abs(JSON.parse(h.requests[1].options.body).q12[0]-Math.PI/6)<1e-14,'display degrees are sent as radians');
  assert(h.requests.filter(row=>row.options.method==='POST').every(row=>row.url.endsWith('/preview_target')),'automatic editing never requests a motion preview or executes');
  assert.equal(saved.size,1);assert.doesNotMatch([...saved.values()][0],/never-save-this-review|token|csrf|command_epoch/);
  h.app.selectTab('collection');const collection=previewSnapshot();h.reply(2,collection);await settle();
  assert(Math.abs(Number(positions()[0].value)-degrees(.1))<1e-12,'another business starts with its own draft');
  h.app.selectTab('calibration');value.revision++;value.manual.positions[0]=.2;h.reply(3,value);await settle();
  assert.equal(Number(positions()[0].value),30,'returning to calibration preserves the editable target');
  assert.match(h.get('joint-inputs').querySelectorAll('small').find(node=>node.dataset.role==='actual').textContent,/11\.46/,'live feedback still refreshes alongside the preserved target');
  h.app.destroy();
  const reload=harness({storage});reload.app.start();const restored=previewSnapshot();restored.workflow.primary_action={id:'manual_execute',allowed:true};restored.workflow.review={id:'server-kept-old-review'};reload.reply(0,restored);await settle();
  assert.equal(Number(reload.get('joint-inputs').querySelectorAll('input').find(node=>node.dataset.role==='position').value),30);
  assert.equal(reload.get('primary').disabled,true,'restored targets must be reviewed again even if the server retained an older review');
  assert.equal(reload.requests.filter(row=>row.options.method==='POST').length,0,'restoring a draft never sends any command');reload.app.destroy();
  const other=harness({storage});other.app.start();const otherDevice=previewSnapshot();otherDevice.device.id='another-device';other.reply(0,otherDevice);await settle();
  assert(Math.abs(Number(other.get('joint-inputs').querySelectorAll('input').find(node=>node.dataset.role==='position').value)-degrees(.1))<1e-12,'drafts never cross device identities');other.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,previewSnapshot());await settle();
  const position=h.get('joint-inputs').querySelectorAll('input').find(node=>node.dataset.role==='position');
  position.value=90;position.events.input();h.fireTimer(ms=>ms===33);
  assert.match(h.get('manual-error').textContent,/超出限位/);assert.equal(h.requests.length,1,'invalid drafts cannot reach the renderer');
  position.value=10;position.events.input();h.app.selectTab('results');h.fireTimer(ms=>ms===33);
  assert.equal(h.requests.length,1,'leaving the work pane cancels pending automatic display');h.app.destroy();
}
for(const business of ['calibration','collection','inference','research']){
  const h=harness({business,physical:true});h.app.start();const value=previewSnapshot();
  value.device.environment='physical';
  Object.assign(value.runtime,{connected:false,feedback_healthy:false,holder:null,state:'DISCONNECTED'});
  Object.assign(value.runtime.allowed_operations,{manual_preview:{allowed:false,reason:'请先连接设备'},manual_execute:{allowed:false},manual_queue:{allowed:false}});
  Object.assign(value.manual,{positions:null,jaw_gaps_mm:[null,null]});h.reply(0,value);await settle();
  const inputs=role=>h.get('joint-inputs').querySelectorAll('input').filter(node=>node.dataset.role===role);
  assert.equal(inputs('slider').length,12);assert(inputs('slider').every(input=>!input.disabled),'offline model sliders remain editable');
  assert.equal(Number(inputs('position')[0].value),0);assert.equal(Number(h.get('jaw-left').value),83);
  assert.equal(h.get('use-feedback').disabled,true);assert.equal(h.get('manual-preview').disabled,true);assert.equal(h.get('show-target').disabled,false);
  assert.match(h.get('manual-feedback').textContent,/控制未接入.*模型预览.*实测未知/);
  assert.equal(value.manual.positions,null,'neutral model pose is only a target draft');
  inputs('slider')[0].value=15;inputs('slider')[0].events.input();h.fireTimer(ms=>ms===33);
  const body=JSON.parse(h.requests[1].options.body);assert.equal(body.business,business);assert.equal(h.requests[1].url,'/api/actions/preview_target');
  assert(Math.abs(body.q12[0]-Math.PI/12)<1e-14);assert.deepEqual(body.jaw_gap_mm,[83,83]);
  assert(h.requests.filter(row=>row.options.method==='POST').every(row=>row.url==='/api/actions/preview_target'),'offline editing never connects, transfers ownership or moves the robot');h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();h.reply(0,value);await settle();
  const inputs=role=>h.get('joint-inputs').querySelectorAll('input').filter(node=>node.dataset.role===role);
  inputs('position')[0].value=12;inputs('position')[0].events.input();
  const offline={...value,revision:2,manual:{...value.manual,positions:null,jaw_gaps_mm:[null,null]},runtime:{...value.runtime,connected:false,feedback_healthy:false,holder:null}};
  h.app.poll();h.reply(1,offline);await settle();
  assert.equal(inputs('position').length,12);assert.equal(Number(inputs('position')[0].value),12,'disconnect preserves the draft and model controls');
  assert.equal(h.get('use-feedback').disabled,true);assert.match(h.get('joint-inputs').querySelectorAll('small').find(node=>node.dataset.role==='actual').textContent,/实测未知/);
  value.revision=3;value.manual.positions[0]=.2;h.app.poll();h.reply(2,value);await settle();
  assert.equal(Number(inputs('position')[0].value),12,'reconnecting does not replace the offline target with measured feedback');
  assert.equal(h.get('use-feedback').disabled,false);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=manualSnapshot();h.reply(0,value);await settle();
  const inputs=role=>h.get('joint-inputs').querySelectorAll('input').filter(node=>node.dataset.role===role);
  inputs('position')[0].value=12;inputs('position')[0].events.input();
  inputs('speed')[0].value=.12;inputs('speed')[0].events.input();h.get('jaw-left').value=72;h.get('jaw-left').events.input();
  h.app.poll();h.reply(1,{...value,revision:2,manual:null,runtime:{...value.runtime,connected:false}});await settle();
  assert.equal(inputs('position').length,0);assert.equal(h.get('manual-preview').disabled,true);
  value.revision=3;value.manual.positions[0]=.2;h.app.poll();h.reply(2,value);await settle();
  assert.equal(inputs('position').length,12,'reconnecting with the same joint definitions rebuilds cleared controls');
  assert.equal(Number(inputs('position')[0].value),12,'reconnection preserves the editable target draft');
  assert.equal(Number(inputs('speed')[0].value),.12);assert.equal(Number(h.get('jaw-left').value),72);
  assert.match(h.get('joint-inputs').querySelectorAll('small').find(node=>node.dataset.role==='actual').textContent,/11\.46/);
  assert.equal(h.get('manual-preview').disabled,false);assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'rebuilding controls never issues motion');h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();
  value.workflow.calibration_tools={available:true,workflow:{kind:'head'},waypoints:[{name:'参考',q:Array(12).fill(0),jaw_gap_mm:[80,80]},{name:'留出姿态',capture:true,q:Array(12).fill(.2),jaw_gap_mm:[80,80]}]};
  value.workflow.pipeline={steps:[{half:'head',step:'capture',counts:{captured:34,valid:32,failed:2}},{half:'head',step:'solve',metrics:{holdout_marker_rms_px:.7,holdout_mesh_px:3,holdout_mesh_coverage:.8}}]};
  value.workflow.quality={counts:{captured:34,valid:32,failed:2},completion:{passed:true}};
  h.reply(0,value);await settle();assert.equal(h.get('calibration-plan-select').children.length,2);assert.equal(h.get('calibration-pipeline-steps').children.length,7);assert.match(h.get('calibration-quality-summary').textContent,/34 帧.*有效 32.*通过/);
  h.get('calibration-plan-next').events.click();assert.match(h.get('calibration-plan-position').textContent,/姿态 2 \/ 2/);
  assert.equal(h.requests[1].url,'/api/actions/preview_target');assert.deepEqual(JSON.parse(h.requests[1].options.body).q12,Array(12).fill(.2));assert.equal(h.get('calibration-plan-next').disabled,true);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=snapshot();
  value.workflow.replay_context={result_id:'capture:batch/capture',batch_id:'batch/capture',frame_index:1,frame_count:2,camera:'left',cameras:['head','left'],overlay:true,overlay_available:true,candidate_id:'wrist-a',
    candidates:[{id:'wrist-a',label:'腕部 v1'},{id:'wrist-b',label:'腕部 v2',status:'quality_failed'}],
    frame:{id:'holdout-1',name:'抬腕留出',split:'holdout',valid_for_fit:false,detections:{ids:[30]}},frames:[{id:'fit-1',split:'fit',valid_for_fit:true},{id:'holdout-1',name:'抬腕留出',split:'holdout',valid_for_fit:false}]};
  h.reply(0,value);await settle();assert.match(h.get('replay-index').children[1].textContent,/抬腕留出.*留出.*不可用于求解/);
  assert.match(h.get('replay-frame-details').textContent,/当前相机检测.*\n/s);
  h.get('replay-candidate').value='wrist-b';h.get('replay-candidate').events.change();const body=JSON.parse(h.requests[1].options.body);
  assert.equal(body.result_id,'capture:batch/capture');assert.equal(body.candidate_id,'wrist-b');assert.equal(body.frame_index,1);assert.equal(body.camera,'left');assert.equal(body.overlay,true);
  assert.equal(h.requests[1].url,'/api/actions/select_result','candidate comparison is read-only and keeps batch, frame and camera');h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('results');h.get('results-business').value='collection';h.get('results-business').events.change();
  assert.equal(h.get('results-pane').hidden,false);assert.equal(h.app.context.business,'collection');assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'switching result business does not transfer control');
  const value=snapshot();value.runtime.allowed_operations.collection_prepare={allowed:true};value.runtime.allowed_operations.collection_plan={allowed:false,reason:'先准备环境'};
  value.workflow.collection={timing:{elapsed_s:12,stage_elapsed_s:4,stage_estimate_s:8,next_label:'核验三路相机',stages:[{label:'加载环境',status:'completed',elapsed_s:8,estimate_s:10},{label:'相机检查',status:'running',elapsed_s:4,estimate_s:8}]},environment:{ready:false,checks:{cameras:'waiting'}},relations:{plan:{timeline_digest:'private-plan'}}};
  h.reply(1,value);await settle();assert.equal(h.get('collection-stages').children.length,2);assert.match(h.get('collection-timing').textContent,/12\.0 秒.*4\.0 秒.*8\.0 秒.*核验三路相机/);
  assert.equal(h.get('collection-plan').disabled,true);assert.match(h.get('collection-plan-reason').textContent,/先准备环境/);
  h.get('collection-prepare').events.click();
  const request=h.requests.find(row=>row.options.method==='POST');assert.equal(request.url,'/api/actions/collection_prepare','business warmup uses the business endpoint without a management operation');h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');
  const value=snapshot();value.runtime.state='READY';
  Object.assign(value.workflow,{status:'failed',stage_id:'collection.plan',message:'target_joint_acceleration_limit: 6.36775 > 5',
    failure:{kind:'motion_limit',message:'当前路径在 123% 速度下超过关节加速度上限。',recovery:'请降低速度后重新规划。',details:{speed_scale:1.23}},
    collection:{timing:{label:'规划与碰撞检查',stages:[{label:'规划与碰撞检查',status:'failed'}]}}});
  h.reply(1,value);await settle();
  assert.equal(h.get('collection-failure').hidden,false);assert.equal(h.get('collection-failure-title').textContent,'本次数采未通过');
  assert.match(h.get('collection-failure-message').textContent,/当前路径.*123%.*加速度上限/);
  assert.match(h.get('collection-failure-recovery').textContent,/降低速度后重新规划/);
  assert.match(h.get('task-message').textContent,/加速度上限/);assert.doesNotMatch(h.get('task-message').textContent,/target_joint/);
  assert.match(h.get('task-details').children.map(node=>node.children.map(child=>child.textContent).join(' ')).join(' '),/target_joint_acceleration_limit/,'raw failure stays available in technical details');
  assert.equal(h.get('control-fault').hidden,true,'planning failure does not manufacture a controller fault');
  h.advance(3100);h.intervals[1]();assert.match(h.get('collection-failure-title').textContent,/最后确认.*当前状态待核验/);
  value.revision++;delete value.workflow.failure;value.workflow.message='杯子检测未通过';h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('collection-failure-message').textContent,'杯子检测未通过');assert.match(h.get('collection-failure-recovery').textContent,/原始错误/);
  assert.doesNotMatch(h.get('collection-failure-recovery').textContent,/降低速度/,'generic failures do not retain old recovery guidance');
  for(const status of ['running','ready']){
    value.revision++;value.workflow.status=status;h.app.poll();h.reply(h.requests.length-1,value);await settle();
    assert.equal(h.get('collection-failure').hidden,true);assert.equal(h.get('collection-failure-message').textContent,'');assert.equal(h.get('collection-failure-recovery').textContent,'');
  }
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'failure guidance never initiates recovery or motion');h.app.destroy();
}
for(const [status,label,button]of [['not_started','待准备','初始化与预热'],['running','进行中','初始化与预热中'],['ready','已就绪','核验已就绪环境'],['invalidated','已失效','重新初始化与预热'],['failed','失败','重新初始化与预热']]){
  const preparations=[],access={permission:()=>({allowed:status!=='running',reason:status==='running'?'正在预热':''}),prepare:action=>{if(status!=='running')preparations.push(action);},subscribe(){},setEnvironment(){},destroy(){}};
  const h=harness({environmentAccess:access});h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');
  const value=snapshot();value.runtime.allowed_operations.collection_prepare={allowed:status!=='running',reason:status==='running'?'正在预热':''};
  value.workflow.primary_action={id:'prepare_collection',allowed:true,label:'旧后台准备入口'};
  value.workflow.collection={warmup:{status,message:'后端准备说明',stage_label:'核验规划服务',elapsed_s:12,generation:'service-one'}};
  h.reply(1,value);await settle();
  assert.equal(h.get('collection-warmup-status').textContent,'初始化与预热：'+label);assert.match(h.get('collection-warmup-message').textContent,/后端准备说明/);
  assert.match(h.get('collection-warmup-stage').textContent,/核验规划服务.*12.0 秒/);assert.match(h.get('collection-prepare').textContent,new RegExp(button));
  assert.equal(h.get('primary-action').hidden,true,'one preparation button replaces the separate background-setup button');
  h.get('collection-prepare').events.click();
  assert.deepEqual(preparations,[],'business warmup never delegates to operations lifecycle');
  const posts=h.requests.filter(r=>r.options.method==='POST');assert.equal(posts.length,status==='running'?0:1);
  if(posts.length){
    assert.equal(posts[0].url,'/api/actions/collection_prepare');h.reply(2,{job_id:'business-warmup',message:'accepted'});await settle();
    h.reply(h.requests.length-1,value);await settle();
  }
  if(status==='ready'){
    value.revision++;h.app.poll();h.reply(h.requests.length-1,value);await settle();
    assert.equal(h.get('collection-warmup-status').textContent,'初始化与预热：已就绪','idempotent verification does not invent a new cold preparation');
    assert.equal(h.get('collection-prepare').disabled,false);
    for(const taskStatus of ['stopped','idle','failed']){
      value.revision++;value.workflow.status=taskStatus;value.workflow.review=null;h.app.poll();h.reply(h.requests.length-1,value);await settle();
      assert.equal(h.get('collection-warmup-status').textContent,'初始化与预热：已就绪','task outcome is separate from resident initialization');
    }
    h.advance(3100);h.intervals[1]();assert.equal(h.get('collection-warmup-status').textContent,'最后确认的初始化与预热：已就绪');
    assert.match(h.get('collection-warmup-message').textContent,/当前状态待核验/);assert.equal(h.get('collection-prepare').disabled,true,'business warmup requires fresh business facts');
  }
  h.app.destroy();
}
for(const status of ['ready','invalidated']){
  const prepared=[],access={permission:()=>({allowed:true}),prepare:action=>prepared.push(action),subscribe(){},setEnvironment(){},destroy(){}};
  const h=harness({environmentAccess:access});h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');
  const value=snapshot();value.runtime.allowed_operations.collection_prepare={allowed:true};value.runtime.allowed_operations.execute={allowed:true};
  Object.assign(value.workflow,{primary_action:{id:'execute',allowed:true},review:{id:'retained-review',physical_clearance_required:false},collection:{warmup:{status}}});
  h.reply(1,value);await settle();h.get('collection-prepare').events.click();await settle();
  h.reply(2,{job_id:'business-warmup',message:'accepted'});await settle();
  value.revision++;h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('collection-execute').disabled,status!=='ready','ready verification preserves the server review; cold preparation still invalidates old reviews');
  assert.deepEqual(prepared,[]);assert.deepEqual(h.requests.filter(r=>r.options.method==='POST').map(r=>r.url),['/api/actions/collection_prepare'],'verification never auto-confirms a retained plan');h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();
  assert.match(h.get('collection-warmup-status').textContent,/状态未知/,'missing initialization evidence cannot imply readiness');
  const value=snapshot({revision:2});value.workflow.collection={environment:{ready:true}};h.app.poll();h.reply(1,value);await settle();
  assert.equal(h.get('collection-warmup-status').textContent,'初始化与预热：已就绪','older backends retain their explicit environment readiness');
  assert.match(h.get('collection-warmup-message').textContent,/移动杯子或咖啡机、中断后复位后，可直接重新观测与规划/);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.mesh={requested:true,applied:true,camera:'head',candidate_id:'poor-head'};
  h.reply(0,value);await settle();h.get('observation-camera').value='left';h.get('observation-camera').events.change();assert.equal(h.get('camera-grid').dataset.camera,'left');
  for(const [index,camera,candidate] of [[1,'left','poor-wrist'],[3,'right','poor-wrist'],[5,'all','poor-wrist'],[7,'head','poor-head']]){
    h.get('mesh-camera').value=camera;h.get('mesh-camera').events.change();
    assert.deepEqual(JSON.parse(h.requests[index].options.body).candidate_id,candidate);
    assert.equal(JSON.parse(h.requests[index].options.body).enabled,true,'switches preserve enabled mesh');
    h.reply(index,{client_revision:index+4});await settle();value.revision++;value.preview.revisions.mesh=index+4;
    value.preview.mesh={requested:true,applied:false,camera,candidate_id:candidate};h.reply(index+1,value);await settle();
  }
  const rows=[...value.results,{id:'other-wrist',kind:'wrist',details:{head_result_id:'other-head'}}];
  assert.equal(matchingMeshCandidate(rows,'all','poor-head','other-wrist'),'poor-wrist','unrelated remembered candidate cannot override the dependency');
  assert.equal(matchingMeshCandidate(rows,'head','other-wrist','poor-head'),'other-head');
  assert.equal(matchingMeshCandidate(rows.filter(row=>row.id!=='poor-wrist'),'all','poor-head',''),'', 'missing association is not replaced by an unrelated candidate');
  assert(h.requests.filter(row=>row.options.method==='POST').every(row=>row.url.endsWith('/preview_mesh')),'camera selection never routes to install or execution');h.app.destroy();
}
{
  const h=harness();h.app.start();const value=snapshot();
  value.workflow.stage_id='collection.execute';value.workflow.title='fake 同 plant · L3辅助回放';value.workflow.message='真机回放准备中';
  value.workflow.collection={timing:{label:'fake 同 plant · L3辅助回放',next_label:'检查真机视频'}};
  h.reply(0,value);await settle();assert.equal(h.get('stage-title').textContent,'执行与录制');
  assert.match(h.get('task-message').textContent,/开发设备回放准备中/);assert.doesNotMatch(h.get('collection-timing').textContent,/fake|plant|L3|真机/);
  assert(h.get('task-details').querySelectorAll('dd').some(node=>node.textContent==='真机回放准备中'),'original backend wording remains available in technical details');h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('collection');
  const value=snapshot(),current='/runs/current-job',historical='/runs/old-job';value.workflow.id=current;
  const video=(role,run)=>({id:role,kind:'replay',mime_type:'video/mp4',ready:true,camera:'base_0',role,url:'/videos/'+(run?'old':'current')+'-'+role+'.mp4',...(run?{run_id:run}:{})});
  value.media=['source','target'].map(role=>video(role));
  value.results=['source','target'].map(role=>({id:role,kind:'recording',status:'failed',run_id:historical,label:role,media:video(role,historical),details:{started_wall_ns:Date.parse('2026-11-09T05:37:00Z')*1e6},
    recording_quality:{raw_manifest_sha256:'a'.repeat(64),dimensions:{motion:{label:'控制执行完成'},data:{label:'存在采样缺口'},video:{label:'视频可回放'},training:{label:'训练用途待筛选'}},
      cameras:{base_0:{label:'头部',accepted_pairs:3,consecutive_sequences:[{first_index:0,last_index:1,first_time_ns:100,last_time_ns:200},{first_index:2,last_index:2,first_time_ns:400,last_time_ns:400}],
        discontinuities:[{reason:'accepted_counter_discontinuity',before_index:1,after_index:2}],ingress_diagnostics:{status:'not_available'}}}}}));
  h.reply(1,value);await settle();h.app.selectTab('results');
  assert.equal(h.get('collection-history-run').children.length,2);assert.equal(h.get('collection-history-run').value,current);
  assert.match(h.get('collection-history-run').children[1].textContent,/11-09/);assert.equal(h.get('collection-history-run').children[1].value,historical);
  let videos=h.get('replay-media').querySelectorAll('video');assert.equal(videos.length,2);assert(videos.every(node=>node.src.includes('current-')));
  for(const node of videos){node.duration=3;node.events.loadedmetadata();}await h.get('collection-replay-toggle').events.click();assert(videos.every(node=>!node.paused));
  const before=h.requests.length;h.get('collection-history-run').value=historical;h.get('collection-history-run').events.change();
  assert(videos.every(node=>node.paused),'switching runs stops playback from the old pair');
  videos=h.get('replay-media').querySelectorAll('video');assert.equal(videos.length,2,'same camera and media ids across runs never form a four-video pair');assert(videos.every(node=>node.src.includes('old-')));
  assert.equal(h.get('results-list').querySelectorAll('article').length,2);assert.match(h.get('collection-history-summary').textContent,/正在查看历史任务.*2 项回放/);assert.equal(h.get('collection-history-summary').title,historical);
  const qualityLabels=h.get('results-list').querySelectorAll('dd').map(node=>node.textContent);
  for(const label of ['控制执行完成','存在采样缺口','视频可回放','训练用途待筛选'])assert(qualityLabels.includes(label));
  const selectors=h.get('results-list').querySelectorAll('input');assert.equal(selectors.length,4);
  const pick=h.get('results-list').querySelectorAll('button').find(node=>node.textContent==='下载已选片段清单');
  assert.equal(pick.disabled,true);selectors[0].checked=true;selectors[0].events.change();assert.equal(pick.disabled,false);
  selectors[0].checked=false;selectors[0].events.change();assert.equal(pick.disabled,true);
  for(const node of videos){node.duration=4;node.events.loadedmetadata();}assert.equal(h.get('collection-replay-toggle').disabled,false);await h.get('collection-replay-toggle').events.click();assert(videos.every(node=>!node.paused));
  assert.equal(h.requests.length,before,'history browsing and local playback send no device actions');
  value.revision++;value.workflow.id='/runs/newly-generated-job';h.app.poll();h.reply(before,value);await settle();
  assert.equal(h.get('collection-history-run').value,historical,'a new current job cannot reset an existing history selection');assert.equal(h.get('replay-media').querySelectorAll('video')[0],videos[0],'unchanged history playback is not rebuilt on status refresh');
  h.app.selectTab('calibration');h.reply(before+1,snapshot());await settle();h.app.selectTab('collection');h.reply(before+2,value);await settle();
  assert.equal(h.get('collection-history-run').value,historical,'history selection survives switching business context');h.app.destroy();
}
for(const crypto of [webcrypto,{getRandomValues:values=>webcrypto.getRandomValues(values)}]){
  const h=harness({crypto});h.app.start();h.reply(0,previewSnapshot());await settle();h.get('show-target').events.click();
  assert.equal(h.requests[1].url,'/api/actions/preview_target','preview reaches the backend with secure-context or HTTP crypto');
  const previewId=JSON.parse(h.requests[1].options.body).request_id;
  assert.match(previewId,/^[\da-f]{8}-[\da-f]{4}-4[\da-f]{3}-[89ab][\da-f]{3}-[\da-f]{12}$/);
  const stop=h.app.send('stop');assert.equal(h.requests[2].url,'/api/actions/stop');
  const stopId=JSON.parse(h.requests[2].options.body).request_id;
  assert.match(stopId,/^[\da-f]{8}-[\da-f]{4}-4[\da-f]{3}-[89ab][\da-f]{3}-[\da-f]{12}$/);assert.notEqual(stopId,previewId);
  h.reply(2,{message:'停止已受理'});await stop;h.app.destroy();
}
{
  const h=harness({crypto:{}});h.app.start();h.reply(0,previewSnapshot());await settle();h.get('show-target').events.click();await settle();
  assert.match(h.get('request-status').textContent,/缺少安全随机数/);await h.app.send('stop');
  assert.match(h.get('request-status').textContent,/缺少安全随机数/);assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}
function executionSettingsSnapshot(business='calibration'){
  const value=snapshot();value.runtime.allowed_operations.execution_settings={allowed:true};
  value.runtime.execution_settings={revision:2,business,settings:{arrival_enabled:business==='calibration',
    arrival_tolerances_rad:Array.from({length:12},(_,i)=>.01+i*.001),arrival_jaw_mm:2,
    tracking_enabled:business==='calibration',tracking_joint_rad:.08,tracking_jaw_mm:4,tracking_duration_s:.2,
    fake_error:{enabled:false,joint_bias_max_rad:0,seed:0},speed_protection:{enabled:true,limit_rad_s:.8}}};return value;
}
function researchSettings(){return {gains:{left:{kp:[240,210,210,90,90,60],kd:[5,5,5,3,3,2.1]},right:{kp:[80,70,70,80,30,20],kd:[2,2,2,1.6,1,.7]}},
  integral_enabled:true,alignment_speed_rad_s:.1,home_speed_rad_s:.1,jaw_speed_mm_s:10,tcp_tolerance_mm:1};}
assert(html.indexOf('id="speed-protection-tools"')<html.indexOf('id="execution-settings-tools"'));
for(const business of ['calibration','collection','inference']){
  const h=harness({business});h.app.start();const value=executionSettingsSnapshot(business);value.runtime.holder=business;
  h.reply(0,value);await settle();
  assert.equal(h.get('speed-protection-tools').hidden,false);assert.equal(h.get('speed-protection-enabled').checked,true);
  assert.equal(Number(h.get('speed-protection-limit').value),.8);assert.match(h.get('speed-protection-effective').textContent,/当前生效.*版本 2.*开启.*0.8 rad\/s/);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);
  for(const limit of ['',0,-.1,'NaN','Infinity']){
    h.get('speed-protection-limit').value=limit;h.get('speed-protection-limit').events.input();
    assert.equal(h.get('speed-protection-apply').disabled,true);assert.match(h.get('speed-protection-draft').textContent,/有限正数/);
  }
  h.get('speed-protection-limit').value='.9';h.get('speed-protection-limit').events.input();
  h.get('speed-protection-enabled').checked=false;h.get('speed-protection-enabled').events.input();
  h.get('speed-protection-apply').events.click();
  const body=JSON.parse(h.requests[1].options.body);assert.equal(body.expected_revision,2);
  assert.equal(body.business,business);assert.deepEqual(body.settings.speed_protection,{enabled:false,limit_rad_s:.9});
  assert.equal(h.requests[1].url,'/api/actions/execution_settings');
  assert.equal(h.get('manual-execution-dialog').open,false,'saving a limit never starts a motion');
  h.reply(1,{message:'saved'});await settle();value.revision++;value.runtime.execution_settings.revision++;value.runtime.execution_settings.settings=body.settings;
  h.reply(2,value);await settle();assert.match(h.get('speed-protection-effective').textContent,/关闭.*0.9 rad\/s/);
  h.app.selectTab('results');assert.equal(h.get('speed-protection-tools').hidden,true);
  h.app.selectTab('operations');assert.equal(h.get('speed-protection-tools').hidden,true);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=executionSettingsSnapshot();h.reply(0,value);await settle();
  h.get('tracking-joint-tolerance').value='.12';h.get('tracking-joint-tolerance').events.input();
  h.get('speed-protection-limit').value='.9';h.get('speed-protection-limit').events.input();h.get('speed-protection-apply').events.click();
  const speedBody=JSON.parse(h.requests[1].options.body);
  assert.equal(speedBody.settings.tracking_joint_rad,.08,'speed save preserves effective errors, not their draft');
  h.reply(1,{message:'saved'});await settle();value.revision++;value.runtime.execution_settings.revision++;value.runtime.execution_settings.settings=speedBody.settings;
  h.reply(2,value);await settle();
  assert.equal(h.get('tracking-joint-tolerance').value,'.12');assert.equal(h.get('execution-settings-apply').disabled,true);
  assert.match(h.get('execution-settings-draft').textContent,/版本已变化/);
  h.get('speed-protection-limit').value='.7';h.get('speed-protection-limit').events.input();
  h.get('execution-settings-reset').events.click();assert.equal(h.get('speed-protection-limit').value,'.7','error reset preserves speed draft');
  h.get('tracking-joint-tolerance').value='.11';h.get('tracking-joint-tolerance').events.input();h.get('execution-settings-apply').events.click();
  const errorBody=JSON.parse(h.requests[3].options.body);assert.equal(errorBody.settings.speed_protection.limit_rad_s,.9,'error save preserves effective speed, not its draft');
  h.reply(3,{message:'saved'});await settle();value.revision++;value.runtime.execution_settings.revision++;value.runtime.execution_settings.settings=errorBody.settings;
  h.reply(4,value);await settle();assert.equal(h.get('speed-protection-limit').value,'.7');assert.equal(h.get('speed-protection-apply').disabled,true);
  assert.match(h.get('speed-protection-draft').textContent,/版本已变化/);
  h.get('speed-protection-reset').events.click();assert.equal(Number(h.get('speed-protection-limit').value),.9);h.app.destroy();
}
for(const unavailable of ['disconnected','old-backend','not-holder']){
  const h=harness();h.app.start();const value=executionSettingsSnapshot();
  if(unavailable==='disconnected')value.runtime.connected=false;
  if(unavailable==='old-backend')delete value.runtime.execution_settings.settings.speed_protection;
  if(unavailable==='not-holder')value.runtime.allowed_operations.execution_settings={allowed:false,reason:'请先交接控制权限'};
  h.reply(0,value);await settle();assert.equal(h.get('speed-protection-enabled').disabled,true);assert.equal(h.get('speed-protection-apply').disabled,true);
  if(unavailable!=='not-holder')assert.match(h.get('speed-protection-effective').textContent,/实际设置未核验/);
  else assert.match(h.get('speed-protection-reason').textContent,/交接/);
  h.app.destroy();
}
{
  const h=harness({business:'research'});h.app.start();const value=executionSettingsSnapshot('research');value.runtime.holder='collection';
  Object.assign(value.runtime.allowed_operations,{precision_preview:{allowed:false,reason:'请先交接控制权限'},precision_execute:{allowed:false},precision_analyze:{allowed:false}});
  const settings=researchSettings();
  value.precision_study={configured:true,reason:'请先交接控制权限',latest:null,review:null,settings,settings_caps:settings,settings_revision:0};h.reply(0,value);await settle();
  assert.equal(h.app.context.business,'research');assert.equal(h.get('work-pane').hidden,false);
  assert.equal(h.get('precision-study-tools').hidden,false);assert.equal(h.get('task-panel').hidden,true);
  assert.equal(h.get('business-outcomes').hidden,true);assert.equal(h.get('precision-execute').disabled,true);
  assert.equal(h.get('manual-tools').hidden,false);assert.equal(h.get('observation-panel').hidden,false);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'opening research never transfers control');
  value.revision++;value.runtime.holder='research';value.runtime.allowed_operations.precision_preview={allowed:true};h.app.poll();h.reply(1,value);await settle();
  h.get('precision-20').events.click();const body=JSON.parse(h.requests[2].options.body);
  assert.equal(h.requests[2].url,'/api/actions/precision_preview');assert.equal(body.business,'research');
  assert.equal(body.kind,'trajectory');assert.equal(body.speed,2);
  h.reply(2,{message:'prepared'});await settle();value.revision++;value.precision_study.review={id:'one-study-review',summary:'固定片段 2×',duration_s:25.5};
  value.runtime.allowed_operations.precision_execute={allowed:true};h.reply(3,value);await settle();
  assert.match(h.get('precision-review').textContent,/2×.*25.50/);h.get('precision-execute').events.click();
  assert.equal(h.requests[4].url,'/api/actions/precision_execute');assert.equal(JSON.parse(h.requests[4].options.body).review_id,'one-study-review');
  h.app.selectTab('results');assert.equal(h.get('precision-study-tools').hidden,true);h.app.destroy();
}

{
  const h=harness({business:'research'});h.app.start();const value=executionSettingsSnapshot('research');
  value.runtime.holder='research';Object.assign(value.runtime.allowed_operations,{precision_preview:{allowed:true},precision_execute:{allowed:false},precision_analyze:{allowed:false}});
  const settings=researchSettings(),caps=researchSettings();for(const side of ['left','right'])settings.gains[side].kd=[0,0,0,0,0,0];
  value.precision_study={configured:true,settings,settings_caps:caps,settings_revision:0,runtime_facts:{trajectory_speed_choices:[1.5]}};
  h.reply(0,value);await settle();
  for(const id of ['precision-05','precision-10','precision-20']){assert.equal(h.get(id).hidden,true);assert.equal(h.get(id).disabled,true);}
  assert.equal(h.get('precision-15').hidden,false);assert.equal(h.get('precision-start').disabled,false,'zero KD remains valid, matching the server');h.get('precision-start').events.click();
  assert.equal(JSON.parse(h.requests[1].options.body).speed,1.5,'alignment uses an available reference even when 0.5x is absent');h.app.destroy();
}

{
  const h=harness({business:'collection',view:'control-research'});h.app.start();
  assert.equal(h.document.tabs.find(t=>t.dataset.tab==='research').attributes['aria-current'],'page');
  const value=executionSettingsSnapshot('research');value.runtime.connected=false;
  const config=researchSettings();
  const v8=JSON.parse(JSON.stringify(config));v8.gains.left.kd=[2,2,2,1,1,.7].map(v=>1.5*Math.sqrt(2)*v);
  value.precision_study={configured:true,settings:config,settings_caps:config,settings_revision:0,history:[],review:null,latest:null,
    settings_presets:[{id:'v8',label:'V8',settings:v8},{id:'v10',label:'V10',settings:config}]};
  value.runtime.allowed_operations.precision_settings={allowed:true};h.reply(0,value);await settle();
  const gains=()=>h.get('precision-gain-inputs').querySelectorAll('input');assert.equal(gains().length,24);
  gains()[6].value='70';gains()[6].events.input();h.get('precision-integral').checked=false;h.get('precision-integral').events.input();
  h.app.poll();h.reply(1,value);await settle();assert.equal(gains()[6].value,'70','poll must preserve the gain draft');
  h.get('precision-settings-save').events.click();const body=JSON.parse(h.requests[2].options.body);
  assert.equal(h.requests[2].url,'/api/actions/precision_settings');assert.equal(body.settings.gains.left.kp[3],70);assert.equal(body.settings.integral_enabled,false);
  assert.equal(body.expected_revision,0);assert.equal(h.requests.filter(x=>/connect|precision_execute/.test(x.url)).length,0,'saving gains must not connect or move');
  h.reply(2,{message:'saved'});await settle();value.revision++;value.precision_study={...value.precision_study,settings:body.settings,settings_revision:1};
  Object.assign(value.runtime,{connected:true,holder:'research'});
  Object.assign(value.runtime.allowed_operations,{precision_preview:{allowed:true},precision_execute:{allowed:true}});
  value.precision_study.review={id:'saved-config-review',summary:'已保存配置的审阅',duration_s:1};
  h.reply(3,value);await settle();assert.match(h.get('precision-settings-status').textContent,/版本 1/);
  assert.equal(h.get('precision-start').disabled,false);assert.equal(h.get('precision-execute').disabled,false);
  assert.equal(h.get('precision-settings-details').open,false);
  const count=h.requests.length,stored=JSON.stringify(value.precision_study.settings);
  const selector=h.get('precision-config');assert.deepEqual(selector.children.map(option=>option.value),['','v8','v10']);
  selector.value='v8';selector.events.change();assert.equal(selector.value,'v8');
  assert.equal(Number(gains()[1].value),v8.gains.left.kd[0]);assert.equal(h.get('precision-settings-details').open,false);
  assert.match(h.get('precision-settings-status').textContent,/实际准备使用 自定义.*草稿未保存/);
  assert.equal(h.get('precision-start').disabled,true);assert.equal(h.get('precision-execute').disabled,true);
  h.get('precision-start').events.click();h.get('precision-execute').events.click();assert.equal(h.requests.length,count,'unsaved config cannot silently use old saved gains');
  selector.value='v10';selector.events.change();assert.equal(selector.value,'v10');
  assert.equal(Number(gains()[1].value),5);
  assert.equal(h.requests.length,count,'filling a preset neither saves nor applies gains');
  assert.equal(JSON.stringify(value.precision_study.settings),stored,'preset editing cannot rewrite saved settings');
  assert.match(h.get('precision-settings-status').textContent,/草稿未保存/);
  gains()[6].value='91';gains()[6].events.input();assert.equal(h.get('precision-settings-save').disabled,true);
  h.app.selectTab('calibration');assert.equal(h.get('precision-study-tools').hidden,true);h.app.destroy();
}
{
  const h=harness({business:'research'});h.app.start();const value=executionSettingsSnapshot('research'),settings=researchSettings();
  const trip={joint:'left_J5',command_epoch:15,sample_wall_ns:100,peak_rad_s:1.28};
  const latest={id:'fixture-record',summary:'2x',status:'fault',reason:'reference rejected',started_wall_ns:200,binding:{command_epoch:17},feedback_peak_rad_s:.626,speed_protection:{last_trip:trip}};
  value.precision_study={configured:true,settings,settings_caps:settings,settings_revision:0,latest,review:null};h.reply(0,value);await settle();
  let report=JSON.parse(h.get('precision-report').textContent);
  assert.equal(report.speed_protection.current_execution_trip,null);assert.deepEqual(report.speed_protection.historical_last_trip,trip);
  assert.match(h.get('precision-status').textContent,/已观测峰值 0.626/);
  latest.binding.command_epoch=15;latest.started_wall_ns=0;value.revision++;h.app.poll();h.reply(1,value);await settle();
  report=JSON.parse(h.get('precision-report').textContent);assert.deepEqual(report.speed_protection.current_execution_trip,trip);
  assert.equal(report.speed_protection.historical_last_trip,null);assert.match(h.get('precision-status').textContent,/已观测峰值 1.28/);h.app.destroy();
}
{
  const h=harness();h.app.start();assert.equal(h.get('execution-settings-apply').disabled,true);
  const value=executionSettingsSnapshot();value.runtime.execution_settings.legacy_passive_guards=true;h.reply(0,value);await settle();
  const inputs=()=>h.get('arrival-tolerance-inputs').querySelectorAll('input');
  assert.deepEqual(inputs().map(node=>node.value),value.runtime.execution_settings.settings.arrival_tolerances_rad);
  assert.match(h.get('execution-settings-effective').textContent,/当前生效.*版本 2.*标定.*超差保护/);
  assert.match(h.get('execution-settings-effective').textContent,/原被动轴保护（2° \/ 2 mm）；应用后所有轴按本页跟踪设置执行/);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);
  h.get('arrival-observe-only').checked=true;h.get('arrival-observe-only').events.input();
  h.get('tracking-observe-only').checked=true;h.get('tracking-observe-only').events.input();
  h.get('fake-error-enabled').checked=true;h.get('fake-error-enabled').events.input();
  h.get('fake-error-max').value='.004';h.get('fake-error-max').events.input();
  h.get('fake-error-seed').value='42';h.get('fake-error-seed').events.input();
  inputs()[2].value='.027';inputs()[2].events.input();
  value.revision++;h.app.poll();h.reply(1,value);await settle();
  assert.equal(inputs()[2].value,'.027','status polling must preserve unapplied per-joint draft');
  assert.match(h.get('execution-settings-draft').textContent,/未应用/);
  assert.match(h.get('execution-settings-effective').textContent,/到位超差保护/,'draft toggles do not misreport effective policy');
  assert.equal(h.get('execution-settings-apply').disabled,false);h.get('execution-settings-apply').events.click();
  assert.equal(h.requests[2].url,'/api/actions/execution_settings');const body=JSON.parse(h.requests[2].options.body);
  assert.equal(body.expected_revision,2);assert.equal(body.business,'calibration');assert.equal(body.settings.arrival_enabled,false);assert.equal(body.settings.tracking_enabled,false);
  assert.deepEqual(body.settings.arrival_tolerances_rad,value.runtime.execution_settings.settings.arrival_tolerances_rad.map((v,i)=>i===2?.027:v));
  assert.deepEqual(body.settings.fake_error,{enabled:true,joint_bias_max_rad:.004,seed:42});
  assert.equal(h.get('manual-execution-dialog').open,false,'applying policy does not ask for or confirm a motion');
  h.reply(2,{message:'执行误差设置已生效'});await settle();
  value.revision++;value.runtime.execution_settings={...value.runtime.execution_settings,revision:3,settings:body.settings,legacy_passive_guards:false};
  h.reply(3,value);await settle();assert.match(h.get('execution-settings-summary').textContent,/到位仅记录.*跟踪仅记录.*随机误差开启/);
  assert.doesNotMatch(h.get('execution-settings-effective').textContent,/原被动轴保护/);
  assert.equal(h.get('execution-settings-apply').disabled,true);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=executionSettingsSnapshot();h.reply(0,value);await settle();
  h.get('tracking-joint-tolerance').value='.12';h.get('tracking-joint-tolerance').events.input();
  value.revision++;value.runtime.execution_settings.revision++;h.app.poll();h.reply(1,value);await settle();
  assert.equal(h.get('tracking-joint-tolerance').value,'.12');assert.equal(h.get('execution-settings-apply').disabled,true);
  assert.match(h.get('execution-settings-draft').textContent,/版本已变化/);
  h.get('execution-settings-reset').events.click();assert.equal(h.get('tracking-joint-tolerance').value,.08);
  h.get('fake-error-max').value='.051';h.get('fake-error-max').events.input();assert.equal(h.get('execution-settings-apply').disabled,true);
  assert.match(h.get('execution-settings-draft').textContent,/0～0.05/);
  h.app.selectTab('collection');const collection=executionSettingsSnapshot('collection');collection.runtime.allowed_operations.execution_settings={allowed:false,reason:'请先交接控制权限'};
  h.reply(2,collection);await settle();assert.equal(h.get('arrival-observe-only').checked,true);assert.equal(h.get('tracking-observe-only').checked,true);
  assert.equal(h.get('execution-settings-apply').disabled,true);assert.match(h.get('execution-settings-reason').textContent,/交接/);
  collection.revision++;collection.runtime.allowed_operations.execution_settings={allowed:true};collection.runtime.execution_settings.fake_error_available=false;
  h.app.poll();h.reply(3,collection);await settle();assert.equal(h.get('fake-error-enabled').disabled,true);
  assert.match(h.get('fake-error-capability').textContent,/需要完整 Fake/);
  h.app.switchContext({...h.app.context,environment:'physical'});
  const physical=executionSettingsSnapshot('collection');physical.device.environment='physical';h.reply(4,physical);await settle();
  assert.equal(h.get('fake-error-settings').hidden,true);h.app.destroy();
}
{
  const h=harness({business:'inference'}),value=snapshot();
  value.inference_execution={execute_steps:30,revision:0,allowed:true,action_horizon:60,control_hz:30};
  value.runtime.allowed_operations.inference_execution_settings={allowed:true};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-execution-settings').hidden,false);
  assert.equal(h.get('inference-execute-steps').value,'30');
  h.get('inference-execute-steps').value='45';h.get('inference-execute-steps').events.input();
  assert.equal(h.get('inference-steps-apply').disabled,false);
  h.app.poll();h.reply(1,value);await settle();
  assert.equal(h.get('inference-execute-steps').value,'45','poll preserves draft');
  h.get('inference-steps-apply').events.click();
  const sent=h.requests.find(row=>row.options.method==='POST');
  assert.equal(sent.url,'/api/actions/inference_execution_settings');
  const body=JSON.parse(sent.options.body);assert.equal(body.business,'inference');assert.equal(body.execute_steps,45);assert.equal(body.expected_revision,0);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'saving does not run or connect');
  h.app.destroy();
}
{
  const h=harness({business:'inference'}),value=snapshot();
  value.inference_execution={execute_steps:15,revision:0,allowed:true,action_horizon:32,control_hz:30};
  value.runtime.allowed_operations.inference_execution_settings={allowed:true};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-execute-steps').max,'32');
  assert.match(h.get('inference-steps-effective').textContent,/15 \/ 32/);
  h.get('inference-execute-steps').value='33';h.get('inference-execute-steps').events.input();
  assert.equal(h.get('inference-steps-apply').disabled,true);
  h.get('inference-execute-steps').value='32';h.get('inference-execute-steps').events.input();
  assert.equal(h.get('inference-steps-apply').disabled,false);
  h.get('inference-steps-apply').events.click();
  assert.equal(JSON.parse(h.requests.find(row=>row.options.method==='POST').options.body).execute_steps,32);
  h.app.destroy();
}
function inferenceSnapshot(){
  const value=snapshot();value.runtime.holder='inference';value.runtime.allowed_operations.inference_prepare={allowed:true};value.runtime.allowed_operations.inference_preview={allowed:true};value.runtime.allowed_operations.execute={allowed:true};
  value.workflow={id:'inference-round-1',revision:1,status:'ready',busy:false,stage_id:'inference.idle',message:'推理环境就绪',primary_action:{id:'inference_preview',label:'准备本轮推理',allowed:true},outcomes:{},
    inference:{model:{id:'base379-010-29450',label:'Base379 · epoch 010 / step 29450',loaded:true},duration_s:40,seed:7,
      metrics:{elapsed_source_s:12.4,remaining_source_s:27.6,elapsed_wall_s:31.2,queries:4,predictions:3,submitted:372,completed:370,last_infer_wall_s:.42},
      outcome:{success:null},audit:{problems:[{kind:'velocity',axis:1,value:4}],projections:[{axis:2,before:2.1,after:2}]}}};
  return value;
}
{
  const h=harness({business:'inference'});h.app.start();const value=inferenceSnapshot();
  value.runtime.connected=false;value.runtime.allowed_operations.inference_run={allowed:true};
  value.workflow.inference={...value.workflow.inference,model_ready:true,run_context:{id:'legacy-disconnected'},selected_model_id:'chosen',bundle_digest:'a'.repeat(64)};
  h.reply(0,value);await settle();assert.equal(h.get('inference-run').disabled,true);
  h.get('inference-run').events.click();h.get('inference-run-clearance').checked=true;h.get('inference-run-confirm').events.click();
  await h.app.send('inference_run');assert.equal(h.get('inference-run-dialog').open,false);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'disconnected Run cannot submit any SDK or business execution even with legacy allowed:true');h.app.destroy();
}
function oneRunSnapshot(){
  const value=inferenceSnapshot(),id='base566-epoch007-step58982',digest='a'.repeat(64);
  Object.assign(value.runtime,{connected:true,holder:'inference'});
  Object.assign(value.runtime.allowed_operations,{inference_run:{allowed:true},inference_select:{allowed:true},inference_refresh:{allowed:false}});
  Object.assign(value.workflow.inference,{selected_model_id:id,bundle_digest:digest,model_ready:true,
    run_context:{server_instance_id:value.server_instance_id,device_id:value.device.id,connected:true,workflow_generation:1},
    models:[{id,label:'Base566 epoch007',selectable:true,status:'ready',bundle_digest:digest},
      {id:'base566-epoch005-step42130',label:'Base566 epoch005',selectable:true,status:'ready',bundle_digest:'b'.repeat(64)},
      {id:'pending',label:'epoch008',selectable:false,status:'downloading',reason:'下载中'}],
    model:{id,label:'Base566 epoch007',loaded:true,bundle_digest:digest}});
  return value;
}
{
  const h=harness({business:'inference'}),value=oneRunSnapshot(),info=value.workflow.inference;
  info.selected_model_id='wam-fixture-step42';info.backend='torch';info.model.id=info.selected_model_id;
  info.normal_modes_enabled=true;info.reference_planning={};info.wam_stall_recovery={enabled:false};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-stall-enabled').checked,false);
  assert.equal(h.get('inference-stall-tools').hidden,false);
  h.get('inference-stall-enabled').checked=true;h.get('inference-stall-enabled').events.change();
  h.get('inference-run').events.click();
  assert.match(h.get('inference-run-summary').textContent,/已启用WAM静止恢复/);
  h.get('inference-run-clearance').checked=true;h.get('inference-run-clearance').events.change();
  h.get('inference-run-confirm').events.click();
  assert.equal(JSON.parse(h.requests.find(row=>row.options.method==='POST').options.body).wam_stall_recovery,true);
  info.operator_session={wam_stall_recovery:true};value.revision++;
  h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('inference-stall-enabled').checked,true);
  assert.equal(h.get('inference-stall-enabled').disabled,true);
  h.app.destroy();
}
{
  const h=harness({business:'inference'}),value=oneRunSnapshot();
  value.workflow.inference.models.push({id:'wam-new-step77',backend:'unsupported',selectable:false,
    label:'New checkpoint',reason:'unsupported model family or backend'});
  h.app.start();h.reply(0,value);await settle();
  assert.match(h.get('inference-catalog-status').textContent,/目录自动刷新/);
  assert.match(h.get('inference-catalog-status').textContent,/New checkpoint.*unsupported model family or backend/);
  const original=value.workflow.inference.selected_model_id;
  value.workflow.inference.models.push({id:'base566-epoch009-step70000',backend:'torch',selectable:true,
    label:'newly discovered checkpoint',bundle_digest:'d'.repeat(64)});
  value.revision++;h.app.poll();h.reply(1,value);await settle();
  assert.ok(h.get('inference-checkpoint').children.some(row=>row.textContent==='newly discovered checkpoint'));
  assert.equal(h.get('inference-checkpoint').value,original);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);
  value.workflow.inference.models=[];
  value.revision++;h.app.poll();h.reply(2,value);await settle();
  assert.match(h.get('inference-catalog-status').textContent,/当前已加载配置保留/);
  assert.equal(h.get('inference-run').disabled,false);
  h.app.destroy();
}
{
  const calls=[];let listener=()=>{},allowed=true,finish;
  const access={state:{model:{selected_model_id:'base566-epoch007-step58982',ready:false},active_job:null},
    permission:()=>({allowed}),subscribe(fn){listener=fn;},setEnvironment(){},destroy(){},
    prepare(action,parameters){calls.push({action,parameters});allowed=false;listener();return new Promise(resolve=>finish=resolve);}};
  const h=harness({business:'inference',environmentAccess:access}),value=oneRunSnapshot(),id=value.workflow.inference.selected_model_id;
  value.runtime.connected=false;value.runtime.allowed_operations.inference_run={allowed:false,reason:'先选择checkpoint'};
  value.workflow.inference.model_ready=false;value.workflow.inference.model.loaded=false;value.workflow.inference.model.status='not_loaded';
  value.workflow.inference.models=value.workflow.inference.models.slice(0,1);value.workflow.inference.catalog_error='模型目录接口尚未启动';
  h.app.start();h.reply(0,value);await settle();
  const select=h.get('inference-checkpoint');
  assert.equal(select.value,'');assert.equal(select.children.length,2);
  assert.equal(select.children[0].textContent,'选择checkpoint以准备');assert.equal(select.children[0].disabled,true);
  assert.equal(select.children[1].value,id,'unreachable catalog adds no invented checkpoint');
  const choose=next=>{if(select.value!==next){select.value=next;return select.events.change();}};
  choose(id);select.events.change();assert.equal(calls.length,1,'one human choice submits exactly one operation');
  assert.equal(select.value,id);assert.equal(select.disabled,true);
  value.revision++;h.app.poll();h.reply(1,value);await settle();assert.equal(select.value,id,'a stale business poll preserves the in-flight selection');
  access.state.active_job={request:{action:'prepare_model',parameters:{model_id:id}},status:'running',phase:'warming_model'};
  finish();await settle();listener();
  value.revision++;value.workflow.inference.model.status='loading';value.workflow.inference.model.message='正在预热当前 checkpoint';
  h.app.poll();h.reply(2,value);await settle();assert.equal(select.value,id);assert.match(h.get('inference-model-message').textContent,/正在预热/);
  assert.equal(access.state.active_job.phase,'warming_model');assert.equal(calls.length,1);
  access.state.active_job=null;allowed=true;value.workflow.inference.model.status='error';value.workflow.inference.model.error='预热失败';
  value.revision++;h.app.poll();h.reply(3,value);await settle();listener();
  assert.equal(select.value,'','failed preparation restores a real re-selection opportunity');assert.equal(select.disabled,false);
  choose(id);assert.equal(calls.length,2,'the same sole checkpoint can be explicitly retried');
  access.state.model.ready=true;allowed=true;finish();await settle();listener();
  value.workflow.inference.model_ready=true;value.workflow.inference.model.loaded=true;value.workflow.inference.model.status='ready';value.workflow.inference.model.error=null;
  value.revision++;h.app.poll();h.reply(4,value);await settle();assert.equal(select.value,id,'prepared model remains selected');
  assert.equal(h.get('inference-run').disabled,true);h.get('inference-run').events.click();
  assert.equal(h.get('inference-run-dialog').open,false);
  assert(calls.every(row=>row.action==='prepare_model'));assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'selection and Run never auto-connect through business actions');
  h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('inference');
  const value=oneRunSnapshot();h.reply(1,value);await settle();
  assert.equal(h.get('inference-checkpoint').value,value.workflow.inference.selected_model_id);
  assert.equal(h.get('inference-checkpoint').children[2].disabled,true);
  h.get('inference-duration').value='2';h.get('inference-duration').events.change();
  h.get('inference-run').events.click();assert.equal(h.get('inference-run-dialog').open,true);
  assert.match(h.get('inference-run-summary').textContent,/2 秒/);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'opening Run is not authorization');
  assert.equal(h.get('inference-run-confirm').disabled,true);
  h.get('inference-run-clearance').checked=true;h.get('inference-run-clearance').events.change();
  h.get('inference-run-confirm').events.click();h.get('inference-run-confirm').events.click();
  const sent=h.requests.filter(row=>row.options.method==='POST');assert.equal(sent.length,1);
  assert.equal(sent[0].url,'/api/actions/inference_run');const body=JSON.parse(sent[0].options.body);
  assert.equal(body.seed,7);assert.equal(body.duration_s,2);assert.equal(body.arms_supported,undefined);
  assert.equal(body.physical_clearance,true);assert.equal(body.scene_reset,true);
  assert.equal(body.expected_bundle_digest,'a'.repeat(64));assert.deepEqual(body.expected_context,value.workflow.inference.run_context);
  assert.equal(h.get('inference-run-dialog').open,false);
  assert.equal(sent.some(row=>/actions\/(connect|handoff|execute)$/.test(row.url)),false,'Run contains no control connection or handoff request');
  h.app.destroy();
}
{
  const calls=[],access={permission:()=>({allowed:true}),prepare:(action,parameters)=>calls.push({action,parameters}),subscribe(){},setEnvironment(){},destroy(){}};
  const h=harness({environmentAccess:access});h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('inference');
  const value=oneRunSnapshot();h.reply(1,value);await settle();
  h.get('inference-run').events.click();h.get('inference-duration').value='2';h.get('inference-duration').events.change();
  assert.equal(h.get('inference-run-dialog').open,false,'duration change revokes unified confirmation');
  h.get('inference-run').events.click();h.get('inference-seed').value='8';h.get('inference-seed').events.input();
  assert.equal(h.get('inference-run-dialog').open,false,'seed change revokes unified confirmation');
  h.get('inference-run').events.click();h.advance(3100);h.intervals[1]();
  assert.equal(h.get('inference-run-dialog').open,false,'stale state revokes unified confirmation');
  h.app.poll();const poll=h.requests.length-1;value.revision++;h.reply(poll,value);await settle();
  h.get('inference-run').events.click();h.get('inference-checkpoint').value='base566-epoch005-step42130';
  h.get('inference-checkpoint').events.change();assert.equal(h.get('inference-run-dialog').open,false);
  assert.deepEqual(calls,[{action:'prepare_model',parameters:{model_id:'base566-epoch005-step42130',expected_bundle_digest:'b'.repeat(64)}}]);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'selection cannot use inference_select or connect');
  h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('inference');
  const value=oneRunSnapshot();h.reply(1,value);await settle();h.get('inference-run').events.click();
  h.app.poll();value.revision++;value.server_instance_id='new-server';h.reply(2,value);await settle();
  assert.equal(h.get('inference-run-dialog').open,false,'server restart revokes unified confirmation');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('inference');
  assert.match(h.requests[1].url,/business=inference/);assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'navigation does not transfer control or prepare a model');
  const value=inferenceSnapshot();value.workflow.inference.duration_s=1;h.reply(1,value);await settle();assert.equal(h.get('work-pane').dataset.business,'inference');
  assert.match(h.get('inference-model').textContent,/Base379/);assert.equal(h.get('inference-model-status').textContent,'已加载');
  assert.match(h.get('inference-outcome').textContent,/未知/);assert.match(h.get('inference-audit-summary').textContent,/速度超限 1.*位置投影 1.*不执行碰撞检查/);
  const metricText=h.get('inference-metrics').children.flatMap(row=>row.children.map(node=>node.textContent)).join(' ');
  assert.match(metricText,/\/ 1 秒/,'short diagnostic reports its actual duration; next Run uses 120 seconds');
  assert.match(metricText,/12.4 秒.*27.6 秒.*31.2 秒.*4 \/ 3.*372 \/ 370.*0.420 秒/);
  assert.equal(h.get('primary-action').hidden,true,'prepare action has one visible entry');
  h.get('inference-preview').events.click();assert.equal(h.requests[2].url,'/api/actions/inference_preview');h.reply(2,{job_id:'preview-job'});await settle();
  value.revision++;value.workflow.status='awaiting_review';value.workflow.stage_id='inference.review';
  value.workflow.primary_action={id:'execute',allowed:true,label:'确认执行本轮 40 秒推理'};value.workflow.review={id:'inference-review-1',seed:7,duration_s:120,summary:'整轮120秒，仅记录越限，不检查碰撞'};
  h.reply(3,value);await settle();assert.equal(h.get('review-panel').hidden,false);assert.equal(h.get('primary-action').hidden,false);assert.equal(h.get('primary').disabled,false);
  assert.equal(h.requests.filter(row=>row.url==='/api/actions/execute').length,0,'preparing and rendering the round review never starts it');
  h.get('inference-seed').value='8';h.get('inference-seed').events.input();assert.equal(h.get('primary').disabled,true,'changing seed revokes prior local review');
  h.get('inference-preview').events.click();h.reply(4,{job_id:'next-preview'});await settle();
  value.revision++;value.workflow.review={...value.workflow.review,id:'inference-review-2',seed:8};value.workflow.inference.seed=8;
  h.reply(5,value);await settle();h.get('primary').events.click();assert.equal(h.requests[6].url,'/api/actions/execute');
  const body=JSON.parse(h.requests[6].options.body);assert.equal(body.seed,8);assert.equal(body.duration_s,120);assert.equal(body.physical_clearance,true);
  h.reply(6,{job_id:'inference-running'});await settle();value.revision++;value.workflow.status='running';value.workflow.busy=true;value.workflow.review=null;value.workflow.primary_action=null;value.workflow.stage_id='inference.execute';h.reply(7,value);await settle();
  assert.equal(h.get('inference-seed').disabled,true);assert.equal(h.requests.filter(row=>row.url==='/api/actions/execute').length,1,'a running round has no per-chunk confirmation');
  value.revision++;value.workflow.status='completed';value.workflow.busy=false;value.workflow.inference.outcome={success:true,reason:'stable_tray_support'};
  value.results=[{id:'infer-head',kind:'recording',status:'valid',run_id:value.workflow.id,media:{id:'infer-head',kind:'replay',camera:'base_0',role:'source',ready:true,mime_type:'video/mp4',url:'/api/result-media?business=inference&result_id=infer-head'}}];
  h.app.poll();h.reply(8,value);await settle();assert.match(h.get('inference-outcome').textContent,/成功：末段稳定托盘支撑已确认/);
  h.get('inference-results').events.click();assert.equal(h.get('results-pane').hidden,false);assert.equal(h.get('results-business').value,'inference');assert.equal(h.get('replay-media').querySelectorAll('video').length,1);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,3);h.app.destroy();
}
{
  const h=harness({business:'inference'});h.app.start();
  const value=snapshot({workflow:{id:'current-run',status:'completed',busy:false,outcomes:{},primary_action:null},
    results_page:{schema:'inference_results_page.v1',run_ids:['current-run','older-a'],total_runs:30,total_results:90,page_run_limit:12,older_cursor:19,newer_cursor:null},
    results:[{id:'current-video',run_id:'current-run',kind:'recording',media:{id:'current-video',kind:'replay',ready:true,mime_type:'video/mp4',url:'/api/result-media?result_id=current-video'}}]});
  h.reply(0,value);await settle();h.app.selectTab('results');
  assert.equal(h.get('inference-history-pages').hidden,false);assert.match(h.get('inference-history-page-info').textContent,/共 30 个任务／90 项结果/);
  assert.equal(h.get('inference-history-newer').disabled,true);
  h.get('inference-history-older').events.click();assert.match(h.requests[1].url,/results_before=19/);
  const older={...value,revision:2,results_page:{...value.results_page,run_ids:['older-b'],older_cursor:7,newer_cursor:18},
    results:[{id:'old-video',run_id:'older-b',kind:'recording',media:{id:'old-video',kind:'replay',ready:true,mime_type:'video/mp4',url:'/api/result-media?result_id=old-video'}}]};
  h.reply(1,older);await settle();
  assert.equal(h.get('collection-history-run').value,'older-b','paging selects a task on that page, not the current run outside it');
  assert.equal(h.get('results-list').querySelectorAll('article').length,1);
  h.app.poll();assert.match(h.requests[2].url,/results_before=19/,'ordinary polling retains the stable page cursor');
  h.get('inference-history-newer').events.click();assert(h.requests[2].options.signal.aborted);
  assert.match(h.requests[3].url,/results_after=18/);
  h.reply(2,{...older,revision:999});h.reply(3,{...value,revision:3});await settle();
  assert.equal(h.app.state.revision,3,'aborted old page cannot replace the newer response');
  h.get('inference-history-latest').events.click();assert(!h.requests[4].url.includes('results_'));
  h.reply(4,{...value,revision:4});await settle();
  assert.equal(h.get('collection-history-run').value,'current-run');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'paging never transfers control or starts tasks');
  h.app.switchContext({environment:'fake',business:'calibration'});assert(!h.requests[5].url.includes('results_'));
  h.app.destroy();
}
{
  const route={route_id:'a'.repeat(32),prefix:'/device-physical/'+'a'.repeat(32),device_id:'physical-cpu'};
  const value=snapshot({device:{id:'physical-cpu',environment:'physical'},gateway_route:route});
  assert.equal(routePhysicalRequest('/api/actions/stop',{body:JSON.stringify({environment:'physical'})},value,'http://localhost:8110/'),route.prefix+'/api/actions/stop');
  assert.equal(routePhysicalRequest('/api/state?environment=physical',{},value,'http://localhost:8110/'),'/api/state?environment=physical');
  assert.equal(routePhysicalRequest('/api/actions/stop',{body:JSON.stringify({environment:'fake'})},value,'http://localhost:8110/'),'/api/actions/stop');
  assert.equal(routePhysicalRequest(route.prefix+'/api/media/head?environment=physical',{},value,'http://localhost:8110/'),route.prefix+'/api/media/head?environment=physical');
  const h=harness();h.app.start();h.reply(0,snapshot({server_instance_id:'fake-server'}));await settle();
  h.app.switchContext({environment:'physical',business:'calibration'});value.server_instance_id='physical-server';h.reply(1,value);await settle();
  assert.equal(h.app.state.server_instance_id,'physical-server');
  h.app.switchContext({environment:'fake',business:'calibration'});h.reply(2,snapshot({server_instance_id:'fake-server',revision:2}));await settle();
  assert.equal(h.app.state.server_instance_id,'fake-server','returning to another environment must not retire that live server');
  h.app.switchContext({environment:'physical',business:'calibration'});value.revision++;h.reply(3,value);await settle();
  h.app.send('stop');assert.equal(h.requests[4].url,route.prefix+'/api/actions/stop');
  assert.equal(JSON.parse(h.requests[4].options.body).environment,'physical');h.reply(4,{message:'stop accepted'});await settle();h.app.destroy();
}
{
  const h=harness();h.app.start();const value=manualSnapshot();
  value.manual.disabled_grippers=['left'];value.manual.jaw_gaps_mm=[null,72.];
  h.reply(0,value);await settle();h.get('jaw-left').value='65';
  assert.equal(h.get('jaw-left').disabled,true);assert.equal(h.get('jaw-right').disabled,false);
  assert.match(h.get('manual-feedback').textContent,/左夹爪 未知/);
  assert.match(h.get('manual-feedback').textContent,/已禁用夹爪：左/);
  h.get('use-feedback').events.click();
  assert.equal(h.get('jaw-left').value,'65','missing feedback cannot overwrite the draft with a fake zero');
  assert.equal(Number(h.get('jaw-right').value),72);h.app.destroy();
}
{
  const calls=[],access={navigateEnvironment:next=>calls.push(next),permission:()=>({allowed:false}),subscribe(){},setEnvironment(){},destroy(){}};
  const h=harness({environmentAccess:access,business:'collection'});h.app.start();const value=snapshot();h.reply(0,value);await settle();
  const count=h.requests.length;h.get('environment').value='physical';h.get('environment').events.change();
  assert.deepEqual(calls,['physical']);assert.equal(h.app.context.environment,'fake');assert.equal(h.get('environment').value,'fake');
  assert.equal(h.requests.length,count,'environment switch cannot reuse current scripts against a different manager');h.app.destroy();
}
{
  const env={target:{manager_instance:'current-manager'},recent_jobs:[]},access={state:env,permission:()=>({allowed:true}),subscribe(){},setEnvironment(){},destroy(){}};
  const h=harness({business:'inference',environmentAccess:access});h.app.start();const value=oneRunSnapshot();
  value.workflow.message='所选模型已就绪';value.workflow.inference.model.status='ready';value.workflow.inference.model.error=null;
  value.workflow.request={id:'old-model-tool',action:'inference_refresh',status:'failed',finished_wall_ns:20,error:'Connection refused'};
  const originalRequest=JSON.parse(JSON.stringify(value.workflow.request));h.reply(0,value);await settle();
  assert.match(h.get('task-message').textContent,/本次请求失败.*Connection refused/,'model_ready alone cannot hide a failed tool');
  const job={status:'succeeded',verification:{ok:true},finished_at_ns:30,request:{action:'prepare_model',
    expected_target:{manager_instance:'current-manager',backend_instance:value.server_instance_id,environment:'fake',device_id:value.device.id},
    parameters:{model_id:value.workflow.inference.selected_model_id,expected_bundle_digest:value.workflow.inference.bundle_digest}}};
  env.recent_jobs=[job];value.revision++;h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.doesNotMatch(h.get('task-message').textContent,/本次请求失败/);assert.match(h.get('request-status').textContent,/更新的运维操作.*通过核验/);
  assert.match(h.get('task-details').children.flatMap(row=>row.children.map(node=>node.textContent)).join(' '),/Connection refused/);assert.deepEqual(value.workflow.request,originalRequest);
  for(const alter of [j=>j.status='unknown',j=>j.verification.ok=false,j=>j.finished_at_ns=10,
    j=>j.request.parameters.model_id='another-model',j=>j.request.expected_target.backend_instance='old-backend',
    j=>j.request.expected_target.manager_instance='old-manager']){
    const invalid=JSON.parse(JSON.stringify(job));alter(invalid);env.recent_jobs=[invalid];value.revision++;h.app.poll();h.reply(h.requests.length-1,value);await settle();
    assert.match(h.get('task-message').textContent,/本次请求失败.*Connection refused/);
  }
  env.recent_jobs=[job];value.workflow.request={...originalRequest,action:'inference_run',finished_wall_ns:20};value.revision++;h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.match(h.get('task-message').textContent,/本次请求失败/,'new model preparation cannot hide an execution failure');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}
{
  const h=harness({business:'inference'}),value=oneRunSnapshot();
  value.workflow.id='timeline-run';value.workflow.busy=false;
  value.results=[{id:'capture-timeline-run-receipt',run_id:'timeline-run',kind:'control_recording',status:'completed'},
    {id:'timeline-video',run_id:'timeline-run',kind:'replay',media:{id:'timeline-video',run_id:'timeline-run',
      kind:'replay',camera:'base_0',role:'source',ready:true,mime_type:'video/mp4',url:'/api/result-media?result_id=timeline-video'}}];
  h.app.start();h.reply(0,value);await settle();h.app.selectTab('results');await settle();
  const metadata=h.requests.findIndex(row=>row.url.startsWith('/api/inference-replay-metadata?'));
  assert(metadata>=0);
  const common={model_id:'wam-mem-fixture-step1',model_label:'WAM-MEM · step_1 · 2步去噪 · 32步输出',backend:'torch',execute_steps:8};
  h.reply(metadata,{run_id:'timeline-run',segments:[{...common,start_s:0,end_s:2,mode:'MODEL',denoise_steps:2},
    {...common,start_s:2,end_s:4,mode:'TEACH',denoise_steps:2},
    {...common,start_s:4,end_s:6,mode:'MODEL',denoise_steps:10}]});await settle();
  const video=h.get('replay-media').querySelectorAll('video')[0];video.duration=6;video.events.loadedmetadata();
  assert.match(h.get('replay-current-configuration').textContent,/去噪 2 步.*执行 8 点/);
  assert.match(h.get('replay-current-configuration').textContent,/WAM-MEM FIXTURE/);
  assert.doesNotMatch(h.get('replay-current-configuration').textContent,/2步去噪|32步输出/);
  assert.match(h.get('replay-media').querySelectorAll('figcaption')[0].textContent,/头部/);
  h.get('collection-replay-time').value='3';h.get('collection-replay-time').events.input();
  assert.match(h.get('replay-current-configuration').textContent,/人工接管/);
  h.get('collection-replay-time').value='5';h.get('collection-replay-time').events.input();
  assert.match(h.get('replay-current-configuration').textContent,/去噪 10 步/);
  assert.equal(h.get('replay-configuration-segments').querySelectorAll('button').length,3);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}
console.log('PASS unified web: inference workspace/reviews/metrics/media, execution error settings/drafts/gates, per-run history pairing, server-owned outcomes, workspace drafts, degree sliders, readonly auto-preview, calibration comparison, explicit collection stages and failure guidance, HTTP request IDs, reviewed speed, replay, release, per-environment server identities and bound physical endpoint routing.');

// The catalog contains trusted IDs, while selection alone remains read-only.
{
  const calls=[],access={permission:()=>({allowed:true}),subscribe(){},setEnvironment(){},destroy(){},
    prepare:(action,parameters)=>{calls.push({action,parameters});return Promise.resolve();}};
  const h=harness({business:'collection',environmentAccess:access});h.app.start();
  const value=snapshot();value.scene_assets={catalog_digest:'a'.repeat(64),can_retry:true,busy:false,status:'ready',
    assets:[{id:'machine',kind:'machine',label:'咖啡机',asset_id:'coffee',revision:'v1',dimensions_m:[.23,.43,.345],preview_url:'/api/scene-assets/preview?environment=fake&asset_id=machine&catalog_digest='+ 'a'.repeat(64)},
      {id:'open',kind:'cup',label:'无盖杯',asset_id:'cup-open',revision:'v1',dimensions_m:[.079,.079,.1105],preview_url:'/api/scene-assets/preview?environment=fake&asset_id=open&catalog_digest='+ 'a'.repeat(64)},
      {id:'lid',kind:'cup',label:'带盖杯',asset_id:'cup-lid',revision:'v1',dimensions_m:[.082,.082,.124],preview_url:'/api/scene-assets/preview?environment=fake&asset_id=lid&catalog_digest='+ 'a'.repeat(64)}],
    pairs:[{id:'machine-open',machine:'machine',cup:'open',can_prepare:true},{id:'machine-lid',machine:'machine',cup:'lid',can_prepare:true}]};
  h.reply(0,value);await settle();const requests=h.requests.length;
  h.get('scene-asset-cup').value='lid';h.get('scene-asset-cup').events.change();
  assert.equal(h.requests.length,requests);assert.deepEqual(calls,[],'draft selection cannot prepare GPU services');
  assert.match(h.get('scene-asset-cup-preview').src,/asset_id=lid/);
  assert.match(h.get('scene-asset-cup-details').textContent,/124\.0/);
  assert.equal(h.get('scene-asset-apply').disabled,false);
  h.get('scene-asset-apply').events.click();await settle();
  assert.deepEqual(calls,[{action:'prepare_collection',parameters:{asset_pair_id:'machine-lid',expected_catalog_digest:'a'.repeat(64)}}]);
  assert.equal(h.requests.length,requests);h.app.destroy();
}

{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('inference');
  const value=oneRunSnapshot(),info=value.workflow.inference;
  info.finite_chunk_count=3;info.reference_planning={max_execution_duration_s:2};
  info.finite_chunks={completed_count:1,completed_execution_s:.69,active_execution_s:.2,waiting_elapsed_s:.1,total_waiting_s:.35,
    current:{waiting_s:.25},completed:[{execution_duration_s:.69}]};
  info.input_evidence={pending:2,completed:4,error:null};
  info.observation_pairing={camera_span_ns:12000000};
  h.reply(1,value);await settle();
  assert.equal(h.get('inference-duration').value,'0.5');
  assert.equal(h.get('inference-duration').disabled,true);
  assert.match(h.get('inference-duration-help').textContent,/完整执行可减速至2秒/);
  assert.match(h.get('inference-duration-help').textContent,/共3轮/);
  h.get('inference-run').events.click();
  assert.match(h.get('inference-run-summary').textContent,/连续3轮/);
  assert.match(h.get('inference-run-summary').textContent,/每轮完整执行15步/);
  assert.match(h.get('inference-run-summary').textContent,/最多2秒/);
  const text=h.get('inference-metrics').children.flatMap(row=>row.children.map(node=>node.textContent)).join(' ');
  assert.match(text,/1 \/ 3 轮/);assert.match(text,/当前段推理等待/);
  assert.match(text,/累计推理等待/);
  assert.doesNotMatch(text,/剩余/,'finite source horizon must not masquerade as the wall/execution budget');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);
  h.app.destroy();
}

{
  const h=harness({business:'inference'}),value=oneRunSnapshot(),info=value.workflow.inference;
  info.normal_modes_enabled=true;info.reference_planning={max_execution_duration_s:2};
  info.finite_chunk_count=null;info.execute_steps=15;
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-duration').value,'120');
  assert.equal(h.get('inference-duration').disabled,false);
  assert.match(h.get('inference-duration-help').textContent,/本轮最长运行时长.*观测.*推理.*等待/);
  h.get('inference-duration').value='40';h.get('inference-duration').events.change();
  h.get('inference-run').events.click();
  assert.match(h.get('inference-run-summary').textContent,/最长运行 40 秒/);
  assert.match(h.get('inference-run-summary').textContent,/每段完整15点/);
  assert.doesNotMatch(h.get('inference-run-summary').textContent,/共4轮|连续4轮/);
  h.app.destroy();
}

{
  const h=harness(),value=snapshot();value.runtime.allowed_operations.control_policy={allowed:true};
  const policy={revision:4,policy_hash:'a'.repeat(64),update_allowed:true,origins:{},
    editable_paths:['measured.limit_rad_s','execution.execute_steps'],
    parameters:{measured:{limit_rad_s:1},execution:{execute_steps:15}}};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.requests.filter(row=>row.url.startsWith('/api/control-policy')).length,0,'policy reads are explicit');
  h.get('control-policy-load').events.click();let read=h.requests.findIndex(row=>row.url.startsWith('/api/control-policy'));
  h.reply(read,policy);await settle();
  assert.match(h.get('control-policy-status').textContent,/版本 4/);
  const input=h.get('control-policy-fields').children[0].children[0];
  input.value='1.1';input.events.input();assert.equal(h.get('control-policy-save').disabled,false);
  h.get('control-policy-save').events.click();
  const write=h.requests.findIndex(row=>row.url==='/api/actions/control_policy');
  const body=JSON.parse(h.requests[write].options.body);
  assert.equal(body.expected_revision,4);assert.deepEqual(body.changes,{'measured.limit_rad_s':1.1});
  assert.equal(h.get('control-policy-save').disabled,true,'save cannot overlap another request');
  h.reply(write,{control_policy:{...policy,revision:5,parameters:{...policy.parameters,measured:{limit_rad_s:1.1}}}});await settle();
  assert.match(h.get('control-policy-status').textContent,/版本 5/);
  assert.equal(h.get('control-policy-save').disabled,true);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'parameter editing never starts a control Run');
  h.app.destroy();
}

{
  const h=harness(),value=snapshot();value.runtime.allowed_operations.control_policy={allowed:true};
  h.app.start();h.reply(0,value);await settle();h.get('control-policy-load').events.click();
  const read=h.requests.findIndex(row=>row.url.startsWith('/api/control-policy'));
  h.reply(read,{revision:4,policy_hash:'a'.repeat(64),update_allowed:true,origins:{},
    editable_paths:['measured.limit_rad_s'],parameters:{measured:{limit_rad_s:1}}});await settle();
  const input=h.get('control-policy-fields').children[0].children[0];input.value='1.1';input.events.input();
  h.get('control-policy-save').events.click();
  const write=h.requests.find(row=>row.url==='/api/actions/control_policy');
  write.resolve({...response({message:'control policy revision changed'}),ok:false,status:409});await settle();
  assert.equal(input.value,'1.1','revision conflict must preserve the draft');
  assert.match(h.get('request-status').textContent,/revision changed/);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'conflict must never retry automatically');
  h.app.destroy();
}
{
  const h=harness(),value=snapshot();value.runtime.allowed_operations.control_policy={allowed:true};
  h.app.start();h.reply(0,value);await settle();h.get('control-policy-load').events.click();
  const read=h.requests.findIndex(row=>row.url.startsWith('/api/control-policy'));
  h.app.switchContext({environment:'fake',business:'inference'});
  h.reply(read,{revision:4,policy_hash:'a'.repeat(64),update_allowed:true,origins:{},
    editable_paths:['measured.limit_rad_s'],parameters:{measured:{limit_rad_s:1}}});await settle();
  assert.equal(h.get('control-policy-fields').children.length,0,'a retired business reply must not populate the new editor');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);
  h.app.destroy();
}

{
 const h=harness();h.app.start();const value=snapshot();
 value.results_page={run_ids:['control-run'],total_runs:1,total_results:2,older_cursor:null,newer_cursor:null};
 value.results=[{id:'legacy-head',kind:'head',label:'已保留的标定候选',status:'valid'},
  {id:'capture-control-receipt',run_id:'control-run',kind:'control_recording',label:'本次控制封存',status:'completed',details:{seal_revision:1},download_url:'/api/result-media?result_id=capture-control-receipt'}];
 h.reply(0,value);await settle();
 assert.equal(h.get('collection-history-run').value,'control-run');
 const text=node=>[node.textContent,...node.children.map(text)].join(' '),rendered=text(h.get('results-list'));
 assert.match(rendered,/本次控制封存/);assert.match(rendered,/已保留的标定候选/);
 assert.match(rendered,/下载本次 Run 封存回执/);h.app.destroy();
}

{
  const calls=[],access={state:{model:{ready:true,model_ready:true}},permission:()=>({allowed:true}),
    prepare:(action,parameters)=>calls.push({action,parameters}),subscribe(){},setEnvironment(){},destroy(){}};
  const h=harness({business:'inference',environmentAccess:access}),value=oneRunSnapshot(),info=value.workflow.inference;
  const id='wam-fixture-step42',capabilities={denoise_steps:{minimum:1,maximum:10,default:2}},digest=n=>String(n).padStart(64,'0');
  info.models=[1,2,10].map(n=>({id,backend:'anyinfer',runtime_schema:'workbench.wam.v2',selectable:true,status:'ready',
    label:`WAM · ${n}步去噪`,bundle_digest:digest(n),inference_options:{denoise_steps:n},inference_capabilities:capabilities}));
  Object.assign(info,{selected_model_id:id,backend:'anyinfer',bundle_digest:digest(2),inference_options:{denoise_steps:2},
    inference_capabilities:capabilities,finite_chunk_count:null,normal_modes_enabled:true,reference_planning:{max_execution_duration_s:2}});
  Object.assign(info.model,{id,backend:'anyinfer',bundle_digest:digest(2)});
  value.inference_execution={execute_steps:15,revision:2,action_horizon:32,allowed:true,control_hz:30};
  value.runtime.allowed_operations.inference_execution_settings={allowed:true};
  access.state.model={ready:true,selected_model_id:id,backend:'anyinfer',bundle_digest:digest(2)};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-execute-steps').disabled,false);assert.equal(h.get('inference-execute-steps').max,'32');
  assert.equal(h.get('inference-duration').disabled,false);assert.equal(h.get('inference-duration').value,'120');
  assert.equal(h.get('inference-denoise-tools').hidden,false);assert.equal(h.get('inference-denoise-steps').value,'2');
  assert.equal(h.get('inference-denoise-steps').min,'1');assert.equal(h.get('inference-denoise-steps').max,'10');
  assert.equal(h.get('inference-checkpoint').children.length,1,'runtime variants do not duplicate the checkpoint menu');
  h.get('inference-denoise-steps').value='10';h.get('inference-denoise-steps').events.input();
  assert.equal(h.get('inference-run').disabled,true,'unapplied denoise draft cannot run the old configuration');
  assert.equal(h.get('inference-denoise-apply').disabled,false);h.get('inference-denoise-apply').events.click();await settle();
  assert.deepEqual(calls,[{action:'prepare_model',parameters:{model_id:id,expected_bundle_digest:digest(10)}}]);
  assert.equal(h.get('inference-denoise-steps').value,'10');
  value.revision++;info.bundle_digest=digest(10);info.model.bundle_digest=digest(10);info.inference_options={denoise_steps:10};
  access.state.model.bundle_digest=digest(10);h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('inference-denoise-apply').disabled,true);assert.equal(h.get('inference-run').disabled,false);
  for(const invalid of ['0','11','1.5']){
    h.get('inference-denoise-steps').value=invalid;h.get('inference-denoise-steps').events.input();
    assert.equal(h.get('inference-denoise-apply').disabled,true);
  }
  assert.equal(calls.length,1,'invalid values and polling never prepare a model');h.app.destroy();
}

{
  const h=harness({business:'inference'}),value=oneRunSnapshot();
  Object.assign(value.runtime,{runtime_nonce:'operator-owner',command_epoch:7});
  Object.assign(value.runtime.allowed_operations,{inference_hold:{allowed:true},inference_home:{allowed:true}});
  value.workflow.inference.operator_controls={home_velocity_rad_s:.1,timeout_s:60,focus_timeout_s:8};
  h.app.start();h.reply(0,value);await settle();
  h.get('inference-home').events.click();assert.equal(h.get('inference-home-dialog').open,true);
  assert.equal(h.get('inference-home-confirm').disabled,true);
  h.get('inference-home-clearance').checked=true;h.get('inference-home-clearance').events.change();
  assert.equal(h.get('inference-home-confirm').disabled,false);
  value.revision++;value.runtime.command_epoch++;h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('inference-home-confirm').disabled,true,'an old Home confirmation must not adopt a new epoch');
  h.get('inference-home-confirm').events.click();assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0);
  h.get('inference-home-cancel').events.click();h.get('inference-hold').events.click();
  const hold=h.requests.find(r=>r.url==='/api/actions/inference_hold');assert(hold);
  assert.equal(JSON.parse(hold.options.body).epoch,8);assert.equal(JSON.parse(hold.options.body).runtime_nonce,'operator-owner');
  h.app.destroy();
}
{
  const h=harness({business:'inference'}),value=oneRunSnapshot();
  value.workflow.inference.operator_session={run_id:'prepared'};
  value.workflow.inference.operator_controls={double_press_window_s:.6};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('manual-tools').hidden,true);
  assert.equal(h.get('media-preview').hidden,true);
  assert.equal(h.get('overview-controls').hidden,true);
  assert(!h.requests.some(row=>row.url.includes('/api/preview/target')));
  assert.match(h.get('inference-operator-status').textContent,/0.6秒内双踩踏板1/);
  assert.equal(h.get('inference-run-dialog').open,false);
  const running=oneRunSnapshot();running.workflow.inference.normal_mode={mode:'MODEL'};
  running.revision=value.revision+1;h.app.poll();h.reply(h.requests.length-1,running);await settle();
  assert.match(h.get('inference-teach').textContent,/停止推理并位置保持/);
  assert.match(h.get('inference-operator-status').textContent,/第二脚忽略/);
  assert.equal(h.get('inference-run-dialog').open,false,'pedal-driven state updates must not open a web confirmation');
  h.app.destroy();
}
{
  const h=harness({business:'inference'}),value=oneRunSnapshot();
  Object.assign(value.runtime,{runtime_nonce:'pedal-owner',command_epoch:2});
  value.workflow.inference.operator_controls={pedal_source_descriptor:'/registered/source.json'};
  h.app.start();h.reply(0,value);await settle();
  for(const interval of h.intervals)interval();
  const focus=h.requests.find(r=>r.url==='/api/actions/inference_focus');assert(focus);
  assert.equal(JSON.parse(focus.options.body).active,true);
  h.document.hidden=true;h.document.events.visibilitychange();
  focus.resolve(response({active:true}));await settle();
  const leases=h.requests.filter(r=>r.url==='/api/actions/inference_focus');assert.equal(leases.length,2);
  assert.equal(JSON.parse(leases[1].options.body).active,false,'blur during an in-flight focus must revoke when it returns');
  assert.equal(JSON.parse(leases[1].options.body).client_id,JSON.parse(focus.options.body).client_id);
  h.app.destroy();
}

{
  assert.match(html,/<input id="inference-duration" type="number"[^>]*step="any"/);
  const h=harness({business:'inference'}),value=oneRunSnapshot();
  Object.assign(value.workflow.inference,{normal_modes_enabled:true,reference_planning:{max_execution_duration_s:2}});
  h.app.start();h.reply(0,value);await settle();
  for(const duration of ['5','5.125']){
    h.get('inference-duration').value=duration;h.get('inference-duration').events.change();
    assert.equal(h.get('inference-run').disabled,false);h.get('inference-run').events.click();
    assert.match(h.get('inference-run-summary').textContent,new RegExp('最长运行 '+duration+' 秒'));
    h.get('inference-run-cancel').events.click();
  }
  for(const duration of ['', '0', '-1','Infinity','NaN']){
    h.get('inference-duration').value=duration;h.get('inference-duration').events.change();
    assert.equal(h.get('inference-run').disabled,true);
  }
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}

{
  const h=harness({business:'inference'}),value=oneRunSnapshot();
  Object.assign(value.runtime,{runtime_nonce:'teach-owner',command_epoch:7});
  value.runtime.allowed_operations.inference_teach={allowed:true};
  value.runtime.allowed_operations.inference_run={allowed:false,reason:'请先完成当前位置保持'};
  value.workflow.busy=true;
  value.workflow.inference.operator_task={request_id:'manual-start',kind:'teach',status:'running',phase:'teaching'};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-teach').disabled,false);
  assert.match(h.get('inference-teach').textContent,/完成拖动并位置保持/);
  assert.match(h.get('inference-operator-status').textContent,/可人工拖动/);
  assert.equal(h.get('inference-run').disabled,true);
  h.get('inference-teach').events.click();
  const request=h.requests.find(row=>row.url==='/api/actions/inference_teach');assert(request);
  const body=JSON.parse(request.options.body);assert.equal(body.runtime_nonce,'teach-owner');assert.equal(body.epoch,7);
  assert.equal(h.requests.filter(row=>row.url==='/api/actions/inference_run').length,0);
  h.app.destroy();
}

{
  const h=harness({business:'inference'}),value=oneRunSnapshot();
  h.app.start();h.reply(0,value);await settle();
  h.get('inference-arm').events.click();
  assert.equal(h.get('inference-run-confirm').textContent,'确认并准备');
  assert.match(h.get('inference-run-details').textContent,/等待后续 Run 或双踩踏板1/);
  assert.equal(h.get('inference-run-dialog').open,true);
  assert.match(h.get('inference-run-title').textContent,/准备控制与踏板/);
  h.get('inference-run-clearance').checked=true;h.get('inference-run-clearance').events.change();
  h.get('inference-run-confirm').events.click();
  const request=h.requests.find(row=>row.url==='/api/actions/inference_run');assert(request);
  assert.equal(JSON.parse(request.options.body).prepare_only,true);
  h.app.destroy();
}

{
  const h=harness({business:'inference'}),value=oneRunSnapshot(),info=value.workflow.inference;
  Object.assign(info,{normal_modes_enabled:true,reference_planning:{},duration_s:600,operator_session:{run_id:'prepared600',duration_s:600}});
  h.app.start();h.reply(0,value);await settle();assert.equal(h.get('inference-duration').value,'600');
  h.get('inference-duration').value='10';value.revision++;h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('inference-duration').value,'10','same-session polling keeps an unsubmitted duration draft');
  info.operator_session={run_id:'next-prepared',duration_s:30};value.revision++;h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('inference-duration').value,'30','a new prepared session shows its actual frozen duration');h.app.destroy();
}

{
  const h=harness({business:'inference'}),value=oneRunSnapshot(),info=value.workflow.inference;
  info.reference_planning={velocity_limit_rad_s:2};value.runtime.execution_settings={revision:4};
  value.runtime.allowed_operations.control_policy={allowed:true};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-parameter-settings').hidden,false);
  assert.equal(h.get('inference-planning-speed').value,'2');
  for(const invalid of ['', '0', '-1', 'Infinity']){
    h.get('inference-planning-speed').value=invalid;h.get('inference-planning-speed').events.input();
    assert.equal(h.get('inference-planning-apply').disabled,true);
  }
  h.get('inference-planning-speed').value='1.5';h.get('inference-planning-speed').events.input();
  assert.equal(h.get('inference-planning-apply').disabled,false);
  value.revision++;value.runtime.execution_settings.revision=5;info.reference_planning.velocity_limit_rad_s=1;
  h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('inference-planning-speed').value,'1.5');
  assert.equal(h.get('inference-planning-apply').disabled,true);
  assert.match(h.get('inference-planning-effective').textContent,/策略版本已变化/);
  h.get('inference-planning-reset').events.click();assert.equal(h.get('inference-planning-speed').value,'1');
  value.revision++;value.runtime.allowed_operations.control_policy={allowed:false,reason:'MODEL active'};
  info.operator_session={run_id:'active'};info.normal_mode={mode:'MODEL',runner_status:'RUNNING'};
  h.app.poll();h.reply(h.requests.length-1,value);await settle();
  assert.equal(h.get('inference-planning-speed').disabled,true);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);
  h.app.selectTab('results');assert.equal(h.get('inference-parameter-settings').hidden,true);h.app.destroy();
}

for(const recording of [false,true]){
  const h=harness({business:'inference'}),value=oneRunSnapshot(),info=value.workflow.inference;
  value.workflow.busy=true;value.runtime.allowed_operations.cancel={allowed:true};
  info.operator_session={run_id:'test-ending'};info.recording_enabled=recording;
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-disarm').textContent,recording?'结束控制会话并封存':'结束控制会话');
  h.get('inference-disarm').events.click();await settle();
  const request=h.requests.find(row=>row.options.method==='POST');
  assert(request.url.endsWith('/cancel'));assert.equal(JSON.parse(request.options.body).end_test,true);
  h.app.destroy();
}

for(const change of ['steps','memory-steps','planning','memory-planning','duration','denoise','memory-denoise','stop','duration-run','recording-run']){
  const value=oneRunSnapshot(),info=value.workflow.inference,old='prepared-old',digest='a'.repeat(64),nextDigest='b'.repeat(64);
  Object.assign(value.runtime,{runtime_nonce:'connected-owner',command_epoch:7,stationary:true,stop_receipt:{accepted_monotonic_s:1}});
  value.workflow.id=old;value.workflow.busy=true;
  value.inference_execution={execute_steps:15,revision:2,action_horizon:32,allowed:false};
  value.runtime.execution_settings={...value.runtime.execution_settings,revision:2};
  value.runtime.allowed_operations.control_policy={allowed:false,reason:'Run active'};
  value.runtime.allowed_operations.inference_execution_settings={allowed:false,reason:'Run active'};
  Object.assign(info,{selected_model_id:'wam-fixture-step42',backend:'torch',normal_modes_enabled:true,reference_planning:{velocity_limit_rad_s:2},
    operator_controls:{timeout_s:30},normal_mode:{mode:'HOLD',runner_status:'RUNNING'},
    operator_session:{run_id:old,duration_s:120,seed:0,model_id:'wam-fixture-step42',expected_bundle_digest:digest},
    inference_options:{denoise_steps:2},inference_capabilities:{denoise_steps:{minimum:1,maximum:10,default:2}},
    models:[2,10].map(n=>({id:'wam-fixture-step42',backend:'torch',selectable:true,status:'ready',bundle_digest:n===2?digest:nextDigest,
      inference_options:{denoise_steps:n},inference_capabilities:{denoise_steps:{minimum:1,maximum:10,default:2}}}))});
  Object.assign(info.model,{id:info.selected_model_id,backend:'torch'});
  if(change.startsWith('memory-')){
    const model='wam-mem-fixture-step42';info.selected_model_id=info.model.id=info.operator_session.model_id=model;
    for(const row of info.models)row.id=model;
  }
  let closed=false,renewed=false,modelApplied=false,managementRefreshes=0;const posts=[];
  const access={state:{active_job:null,recent_jobs:[],target:{backend_instance:value.server_instance_id,runtime_nonce:value.runtime.runtime_nonce,command_epoch:-1}},permission:()=>({allowed:closed}),subscribe(){},setEnvironment(){},destroy(){},
    async refresh(){if(++managementRefreshes>=2)this.state.target.command_epoch=value.runtime.command_epoch;},
    prepare(action,parameters){assert(closed);assert(managementRefreshes>=2,'wait for the manager to observe the post-cancel epoch');assert.equal(action,'prepare_model');assert.equal(parameters.expected_bundle_digest,nextDigest);
      modelApplied=true;info.bundle_digest=info.model.bundle_digest=nextDigest;info.inference_options={denoise_steps:10};return {job:{id:'model-update'}};}};
  const h=harness({business:'inference',environmentAccess:access});h.app.start();h.reply(0,structuredClone(value));await settle();
  assert.equal(h.get('inference-execute-steps').disabled,false);assert.equal(h.get('inference-duration').disabled,false);
  assert.equal(h.get('inference-seed').disabled,true,'prepared seed stays frozen; only the requested controls renew the Run');
  assert.equal(h.get('inference-denoise-steps').disabled,false);
  if(change==='steps'||change==='memory-steps'){
    assert.equal(h.get('inference-execute-steps').min,'1');
    h.get('inference-execute-steps').value=change==='memory-steps'?'8':'32';
    h.get('inference-execute-steps').events.input();assert.equal(h.get('inference-steps-apply').disabled,false);
    h.get('inference-steps-apply').events.click();
  }
  else if(change.endsWith('planning')){
    assert.equal(h.get('inference-planning-speed').value,'2');
    h.get('inference-planning-speed').value='1.5';h.get('inference-planning-speed').events.input();
    assert.equal(h.get('inference-planning-apply').disabled,false);h.get('inference-planning-apply').events.click();
  }
  else if(change.endsWith('denoise')){h.get('inference-denoise-steps').value='10';h.get('inference-denoise-steps').events.input();h.get('inference-denoise-apply').events.click();}
  else if(change==='recording-run'){
    h.get('inference-recording-enabled').checked=true;h.get('inference-recording-enabled').events.change();
    h.get('inference-run').events.click();
  }else if(change==='duration-run'){
    h.get('inference-duration').value='5';h.document.activeElement=h.get('inference-duration');let prevented=false;
    h.get('inference-run').events.pointerdown({button:0,preventDefault(){prevented=true;}});assert(prevented);
    h.get('inference-run').events.click();h.get('inference-run').events.click();
  }else{h.get('inference-duration').value='5';h.get('inference-duration').events.change();}
  if(change==='stop')void h.app.send('stop');
  let cursor=1;
  for(let turn=0;turn<80;turn++){
    while(cursor<h.requests.length){
      const index=cursor++,request=h.requests[index];
      if(request.options.method==='POST'){
        const action=request.url.split('/').at(-1),body=JSON.parse(request.options.body);posts.push(action);
        if(action==='cancel'){
          assert.equal(body.end_test,undefined,'parameter renewal preserves the test memory');
          closed=true;value.workflow.busy=false;value.runtime.command_epoch=9;info.operator_session=null;info.recording_ingress_closed=true;
          info.normal_mode={mode:'HOLD',runner_status:'STOPPED'};value.runtime.stop_receipt={accepted_monotonic_s:2};
          value.runtime.allowed_operations.inference_execution_settings={allowed:true};value.inference_execution.allowed=true;
          value.runtime.allowed_operations.control_policy={allowed:true};
          info.background_recordings=[{run_id:old,ingress_closed:true,state:'sealing'}];
        }else if(action==='inference_execution_settings'){
          assert(closed);assert.equal(body.expected_revision,2);assert.equal(body.execute_steps,change==='memory-steps'?8:32);
          value.inference_execution.execute_steps=body.execute_steps;value.inference_execution.revision=3;
        }else if(action==='control_policy'){
          assert(closed);assert.equal(body.expected_revision,2);
          assert.deepEqual(body.changes,{'planning.velocity_limit_rad_s':1.5});
          info.reference_planning.velocity_limit_rad_s=1.5;value.runtime.execution_settings.revision=3;
        }else if(action==='inference_run'){
          assert(closed);assert.equal(body.prepare_only,true);assert.equal(body.duration_s,change.startsWith('duration')?5:120);
          assert.equal(body.expected_bundle_digest,change.endsWith('denoise')?nextDigest:digest);
          assert.equal(body.recording_enabled,change==='recording-run');
          renewed=true;value.workflow.id='prepared-new';value.workflow.busy=true;
          info.recording_enabled=body.recording_enabled;
          info.operator_session={run_id:'prepared-new',duration_s:body.duration_s,seed:body.seed,expected_bundle_digest:body.expected_bundle_digest,recording_enabled:body.recording_enabled};
          info.normal_mode={mode:'HOLD',runner_status:'RUNNING'};
        }
        h.reply(index,action==='control_policy'?{control_policy:{revision:3,parameters:{planning:{velocity_limit_rad_s:1.5}},
          editable_paths:['planning.velocity_limit_rad_s'],update_allowed:true}}:{message:'accepted',job_id:'renewal'});
      }else{value.revision++;h.reply(index,structuredClone(value));}
    }
    await settle();
    const timer=[...h.timeouts.entries()].find(([,t])=>t.ms===100);
    if(timer){h.timeouts.delete(timer[0]);h.advance(100);timer[1].fn();await settle();}
  }
  assert.equal(posts[0],'cancel');assert.equal(renewed,change!=='stop',h.get('request-status').textContent);
  assert.equal(modelApplied,change.endsWith('denoise'));assert.equal(h.get('inference-run-dialog').open,change==='duration-run'||change==='recording-run');
  if(change==='duration-run'){
    assert.match(h.get('inference-run-title').textContent,/开始本轮推理/);
    assert.match(h.get('inference-run-summary').textContent,/5 秒/);
    h.get('inference-duration').events.change();assert.equal(h.get('inference-run-dialog').open,true,'the delayed blur of the already-applied value must not discard the retained Run intent');
    h.get('inference-run-clearance').checked=true;h.get('inference-run-clearance').events.change();
    h.get('inference-run-confirm').events.click();
    const body=JSON.parse(h.requests.at(-1).options.body);assert.equal(body.prepare_only,undefined);assert.equal(body.duration_s,5);
    assert.equal(posts.filter(x=>x==='cancel').length,1,'the explicit click prepares once, then asks for the ordinary Run confirmation');
  }
  assert.match(h.get('inference-recording-status').textContent,/prepared-old.*后台/);
  if(change==='stop')assert(!posts.includes('inference_execution_settings'));
  h.app.destroy();
}

{
  const choice='continue';
  const h=harness({business:'inference'}),value=oneRunSnapshot(),info=value.workflow.inference;
  const id='wam-mem-fixture-step1',digest='c'.repeat(64),caps={denoise_steps:{minimum:2,maximum:2,default:2}};
  info.models=[{id,backend:'torch',selectable:true,status:'ready',label:'WAM-MEM',bundle_digest:digest,inference_options:{denoise_steps:2},inference_capabilities:caps}];
  Object.assign(info,{selected_model_id:id,backend:'torch',bundle_digest:digest,inference_options:{denoise_steps:2},inference_capabilities:caps,memory:{empty:false,observations:8,manual_observations:3},normal_mode:{mode:'HOLD'}});
  Object.assign(info.model,{id,backend:'torch',bundle_digest:digest});
  value.runtime.allowed_operations.inference_memory_clear={allowed:true};
  h.app.start();h.reply(0,value);await settle();
  assert.equal(h.get('inference-recording-enabled').checked,false,'recording is opt-in even for a ready memory model');
  assert.equal(h.get('inference-run').hidden,true);assert.equal(h.get('inference-memory-'+choice).hidden,false);
  assert.match(h.get('inference-memory-status').textContent,/8.*3/);
  if(choice==='continue'){h.get('inference-recording-enabled').checked=true;h.get('inference-recording-enabled').events.change();}
  h.get('inference-memory-'+choice).events.click();await settle();
  assert.equal(h.get('inference-run-dialog').open,true);
  assert.match(h.get('inference-run-title').textContent,/保留记忆/);
  assert.match(h.get('inference-run-details').textContent,/已开启录制/);
  h.get('inference-run-clearance').checked=true;h.get('inference-run-clearance').events.change();h.get('inference-run-confirm').events.click();
  const body=JSON.parse(h.requests.at(-1).options.body);
  assert.equal(body.memory_start,choice);assert.equal(body.recording_enabled,choice==='continue');h.app.destroy();
}

{
  const h=harness({business:'inference',crypto:{getRandomValues:values=>webcrypto.getRandomValues(values)}});h.app.start();h.reply(0,snapshot());await settle();h.app.selectTab('results');
  h.get('recording-files').open=true;h.get('recording-files').events.toggle();
  const read=h.requests.at(-1);assert.match(read.url,/api\/inference-recordings/);
  read.resolve(response({recordings:[{run_id:'saved-one',started_wall_ns:1794280000000000000,state:'complete',deletable:true},{run_id:'saved-two',state:'incomplete',deletable:true},{run_id:'active',state:'recording',deletable:false,reason:'录制尚未结束'}]}));await settle();
  assert.equal(h.get('recording-files-list').querySelectorAll('input')[2].disabled,true);
  h.get('recording-files-select').events.click();h.get('recording-files-delete').events.click();
  assert.equal(h.get('recording-files-confirm').open,true);assert.equal(h.get('recording-files-selection').children.length,2);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'selection and preview never delete');
  h.get('recording-files-cancel').events.click();assert.equal(h.get('recording-files-confirm').open,false);
  h.get('recording-files-delete').events.click();h.get('recording-files-confirm-delete').events.click();
  const deletion=h.requests.at(-1);assert.match(deletion.url,/inference_delete_recordings$/);
  assert.deepEqual(JSON.parse(deletion.options.body).run_ids,['saved-one','saved-two']);assert.equal(JSON.parse(deletion.options.body).confirmed,true);
  deletion.resolve(response({deleted_run_ids:['saved-one','saved-two'],failed:[]}));await settle();
  assert.match(h.get('recording-files-status').textContent,/已删除 2 条/);assert.equal(h.get('recording-files-list').children.length,1);
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,1,'deletion never starts control or automatically retries');
  h.app.destroy();
}
