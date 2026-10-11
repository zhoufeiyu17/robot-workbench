import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {webcrypto} from 'node:crypto';

const source=await readFile(new URL('../web/app.js',import.meta.url),'utf8');
const html=await readFile(new URL('../web/index.html',import.meta.url),'utf8');
const {createWorkbench,actionPermission,countedProgress,frameAgeSeconds,safeMediaUrl,verifyPreviewFrame,previewAuditSummary}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));

class Element {
  constructor(tag='div'){this.tagName=tag;this.children=[];this.attributes={};this.dataset={};this.events={};this.value='';this.checked=false;this.hidden=false;this.disabled=false;this.textContent='';this.className='';this.currentTime=0;this.duration=NaN;this.paused=true;this.playbackRate=1;this.classList={toggle:(name,on)=>{const names=new Set(this.className.split(' ').filter(Boolean));if(on)names.add(name);else names.delete(name);this.className=[...names].join(' ');},contains:name=>this.className.split(' ').includes(name)};}
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
  for(const id of ['head','left','right','preview','mesh']){const card=nodes.get('media-'+id),image=new Element('img'),empty=new Element('p'),meta=new Element('p');empty.className='empty';meta.className='media-meta';card.append(image,empty,meta);}
  const tabs=['calibration','collection','results','diagnostics'].map(tab=>{const button=new Element('button');button.dataset.tab=tab;return button;});
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
function harness({audit=false,storage,sessionMonitor}={}){
  const document=documentStub(),requests=[],intervals=[],revoked=[],blobs=[],timeouts=new Map(),rafs=new Map();let now=0,timerId=0,rafId=0;
  const app=createWorkbench({document,storage,sessionMonitor,location:{href:'http://localhost:8110/'+(audit?'?preview-audit=1':'')},performance:{now:()=>now},crypto:webcrypto,URL:{createObjectURL:blob=>{blobs.push(blob);return `blob:fixture-${requests.length}`;},revokeObjectURL:value=>revoked.push(value)},setInterval:fn=>{intervals.push(fn);return intervals.length;},clearInterval:()=>{},setTimeout:(fn,ms)=>{timeouts.set(++timerId,{fn,ms});return timerId;},clearTimeout:id=>timeouts.delete(id),requestAnimationFrame:fn=>{rafs.set(++rafId,fn);return rafId;},cancelAnimationFrame:id=>rafs.delete(id),fetch:(url,options={})=>new Promise(resolve=>requests.push({url,options,resolve}))});
  const fireTimer=predicate=>{const timer=[...timeouts.entries()].find(([,t])=>predicate(t.ms));assert(timer,'matching timer scheduled');timeouts.delete(timer[0]);timer[1].fn();return timer[1].ms;};
  return {app,document,requests,intervals,revoked,blobs,timeouts,advance:ms=>now+=ms,get:id=>document.getElementById(id),reply(index,value,headers){requests[index].resolve(response(value,headers));},fireTimer,nextFrame:()=>fireTimer(ms=>ms!==2500),paint(){const ready=[...rafs.values()];rafs.clear();for(const fn of ready)fn(now);}};
}

assert.equal(countedProgress({current:4,total:null}),null);
assert.equal(countedProgress({current:12,total:10}),null);
assert.equal(countedProgress({current:0,total:10}).current,0);
assert.equal(frameAgeSeconds({source_wall_ns:7e9},{server_wall_ns:10e9},250),3.25);
assert.equal(frameAgeSeconds({},snapshot()),null);
assert.equal(safeMediaUrl('https://external.invalid/frame.jpg','http://localhost:8110/'),null);
assert.equal(safeMediaUrl('/media/head.jpg','http://localhost:8110/'),'http://localhost:8110/media/head.jpg');
assert.equal(actionPermission(snapshot(),'generate_plan').allowed,true);
assert.equal(actionPermission(snapshot(),'execute').allowed,false);
const review=snapshot({device:{environment:'physical'},workflow:{primary_action:{id:'execute',allowed:true},review:{id:'review-fixture',physical_clearance_required:true}}});
assert.match(actionPermission(review,'execute').reason,/净空/);
assert.equal(actionPermission(review,'execute',{clearance:true}).allowed,true);
assert.equal(actionPermission(snapshot(),'stop',{pending:true,fresh:false}).allowed,true);

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
  value.results=[{id:'poor-head',kind:'head',label:'未通过头部',status:'quality_failed'},{id:'other-head',kind:'head',label:'另一个头部',status:'quality_failed'},{id:'poor-wrist',kind:'wrist',label:'未通过双腕',status:'quality_failed'}];
  return value;
}
const previewHeaders=(view,revision,extra={})=>({'X-Frame-Metadata':JSON.stringify({environment:'fake',view,revision,view_revision:3,renderer_id:'renderer-one',frame_id:'frame-'+revision,source_wall_ns:10e9,...extra})});
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
  h.get('preview-view').value='overview';h.get('preview-view').events.change();assert.equal(h.requests[1].url,'/api/actions/preview_overview');
  h.get('overview-zoom').value=1.5;h.get('overview-zoom').events.input();h.get('overview-zoom').value=2;h.get('overview-zoom').events.input();assert.equal(h.requests.length,2,'overview requests are serialized and coalesced');
  h.reply(1,{client_revision:4});await settle();assert.equal(h.requests[3].url,'/api/actions/preview_overview');assert.equal(JSON.parse(h.requests[3].options.body).zoom,2);h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.connected=false;value.preview.error='renderer process unavailable';value.preview.recovery={allowed:false,reason:'后端未允许',message:'检查本会话渲染日志',commands:['review renderer.log']};
  h.reply(0,value);await settle();h.app.selectTab('diagnostics');assert.equal(h.get('image-recovery').hidden,false);assert.equal(h.get('preview-recover').disabled,true);
  await h.get('preview-recover').events.click();assert.equal(h.requests.length,1,'disabled recovery must not send a restart');
  const reconnect=h.get('reconnect-images').events.click();assert.match(h.requests[1].url,/\/api\/state/);h.reply(1,{...value,revision:2});await reconnect;
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0,'browser recovery only rereads state and frames');
  assert.equal(h.get('image-recovery-commands').textContent,'review renderer.log');
  value.preview.recovery.allowed=true;value.revision=3;h.app.poll();h.reply(2,value);await settle();assert.equal(h.get('preview-recover').disabled,false);
  h.get('preview-recover').events.click();assert.equal(h.requests[3].url,'/api/actions/preview_recover');h.app.destroy();
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
  const h=harness({audit:true});h.app.start();const value=previewSnapshot();value.manual.jaw_gaps_mm=[83.00000236797332,-.000002];h.reply(0,value);await settle();
  assert.equal(h.get('jaw-left').value,83);assert.equal(h.get('jaw-right').value,0);assert.equal(h.get('audit-start').disabled,false);
  assert.deepEqual(h.app.state.manual.jaw_gaps_mm,[83.00000236797332,-.000002],'command fields must not mutate measured feedback');
  h.get('jaw-left').value='84';h.get('show-target').events.click();assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0,'an explicit command outside 0–83 is rejected, not clamped');
  value.manual.jaw_gaps_mm=[-2,85];h.get('use-feedback').events.click();assert.equal(h.get('jaw-left').value,0);assert.equal(h.get('jaw-right').value,83);
  assert.deepEqual(value.manual.jaw_gaps_mm,[-2,85]);h.get('audit-start').events.click();assert.deepEqual(JSON.parse(h.requests[1].options.body).jaw_gap_mm,[0,83]);h.app.destroy();
  for(const invalid of [-2.00001,85.00001,NaN,Infinity]){
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
  h.get('collection-speed').value='50';h.get('collection-speed').events.input();assert.equal(h.get('primary').disabled,true);
  const update=h.get('review-speed').events.click();assert.equal(JSON.parse(h.requests[2].options.body).speed_scale,.5);
  h.reply(2,{message:'新速度已审阅'});await update;value.revision++;value.workflow.speed_scale=.5;value.workflow.review={id:'speed-50',speed_scale:.5};h.reply(3,value);await settle();
  assert.equal(h.get('primary').disabled,false);assert.equal(h.get('collection-speed-value').textContent,'50%');
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
{
  const h=harness();h.app.start();const value=snapshot();
  value.results=[['source','base_0'],['target','base_0'],['source','left_wrist_0']].map(([role,camera],i)=>({id:String(i),kind:'recording',media:{id:String(i),kind:'replay',ready:true,mime_type:'video/mp4',url:'/api/result-media?id='+i,role,camera}}));
  h.reply(0,value);await settle();h.app.selectTab('results');const videos=h.get('replay-media').querySelectorAll('video');
  videos.forEach((node,i)=>node.duration=[2,4,3][i]);videos[0].events.loadedmetadata();
  assert.equal(h.get('collection-replay-toggle').disabled,false);await h.get('collection-replay-toggle').events.click();
  h.advance(1000);h.intervals[2]();assert.equal(videos[0].currentTime,1);assert.equal(videos[1].currentTime,1);
  h.get('collection-replay-speed').value='.5';h.get('collection-replay-speed').events.change();h.advance(2000);h.intervals[2]();
  assert.equal(videos[0].currentTime,2);assert.equal(videos[0].paused,true);assert.equal(videos[1].currentTime,2);assert.equal(videos[1].paused,false);
  h.get('collection-replay-time').value='1';h.get('collection-replay-time').events.input();await settle();assert.equal(videos[0].paused,false,'seeking back resumes a shorter video');
  h.get('collection-replay-camera').value='left_wrist_0';h.get('collection-replay-camera').events.change();
  assert.equal(h.get('collection-replay-toggle').disabled,true);assert.equal(videos[2].controls,true);assert.match(h.get('collection-replay-status').textContent,/仅有单侧/);
  h.get('collection-replay-camera').value='base_0';h.get('collection-replay-camera').events.change();
  let rejectOldPlay;videos[0].play=()=>new Promise((resolve,reject)=>{rejectOldPlay=reject;});
  const oldPlay=h.get('collection-replay-toggle').events.click();h.app.selectTab('calibration');rejectOldPlay(Error('old playback failed'));
  await oldPlay;assert.doesNotMatch(h.get('collection-replay-status').textContent,/old playback failed/,'late play errors cannot replace a different view');
  assert.equal(h.requests.filter(row=>row.options.method==='POST').length,0);h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();h.app.switchContext({environment:'physical',business:'calibration'});
  const value=snapshot({device:{id:'physical-fixture',environment:'physical'}});value.runtime.allowed_operations.release={allowed:true};
  h.reply(1,value);await settle();assert.equal(h.get('release').disabled,true);h.get('release-supported').checked=true;h.get('release-supported').events.change();
  assert.equal(h.get('release').disabled,false);const release=h.get('release').events.click();assert.equal(h.requests[2].url,'/api/actions/release');assert.equal(JSON.parse(h.requests[2].options.body).arms_supported,true);
  h.reply(2,{message:'释放已受理'});await release;value.revision++;value.runtime.release_receipt={status:'unconfirmed',message:'SDK 尚未断开'};value.runtime.allowed_operations.release={allowed:false};value.runtime.allowed_operations.connect={allowed:false};
  h.reply(3,value);await settle();assert.match(h.get('release-state').textContent,/尚未确认.*SDK/);assert.equal(h.get('release-supported').checked,false);
  assert.equal(h.requests.filter(row=>row.url==='/api/actions/connect').length,0);h.app.destroy();
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
  assert.match(h.get('handoff-reason').textContent,/保持\/控制状态尚未确认/);
  const count=h.requests.length;h.get('control-fault-diagnostics').events.click();assert.equal(h.get('diagnostics-pane').hidden,false);assert.equal(h.requests.length,count,'diagnostic navigation must not recover or reconnect');
  value.revision++;value.runtime.state='READY';h.app.poll();h.reply(count,value);await settle();assert.equal(h.get('control-fault').hidden,true,'historical diagnostic must not imply a current fault');h.app.destroy();
  const physical=harness();physical.app.start();physical.app.switchContext({environment:'physical',business:'calibration'});value.device.environment='physical';value.runtime.state='FAULT';physical.reply(1,value);await settle();
  assert.match(physical.get('control-fault-guidance').textContent,/可靠支撑双臂.*确认断开后再连接.*未确认时不要重试/);physical.app.destroy();
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
  h.get('collection-prepare').events.click();assert.equal(h.requests[2].url,'/api/actions/collection_prepare');h.app.destroy();
}
{
  const h=harness();h.app.start();const value=previewSnapshot();value.preview.mesh={requested:true,applied:true,camera:'head',candidate_id:'poor-head'};
  h.reply(0,value);await settle();h.get('observation-camera').value='left';h.get('observation-camera').events.change();assert.equal(h.get('camera-grid').dataset.camera,'left');
  h.get('mesh-camera').value='left';h.get('mesh-camera').events.change();assert.equal(JSON.parse(h.requests[1].options.body).candidate_id,null);
  h.reply(1,{client_revision:5});await settle();value.revision++;value.preview.revisions.mesh=5;value.preview.mesh={requested:false,applied:false,camera:'left',candidate_id:null};h.reply(2,value);await settle();
  h.get('mesh-candidate').value='poor-wrist';h.get('mesh-candidate').events.change();assert.equal(JSON.parse(h.requests[3].options.body).candidate_id,'poor-wrist');
  h.reply(3,{client_revision:6});await settle();value.revision++;value.preview.revisions.mesh=6;value.preview.mesh.candidate_id='poor-wrist';h.reply(4,value);await settle();
  h.get('mesh-camera').value='head';h.get('mesh-camera').events.change();assert.equal(JSON.parse(h.requests[5].options.body).candidate_id,'poor-head','switching cameras restores its own remembered candidate');
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
  value.results=['source','target'].map(role=>({id:role,kind:'recording',run_id:historical,label:role,media:video(role,historical)}));
  h.reply(1,value);await settle();h.app.selectTab('results');
  assert.equal(h.get('collection-history-run').children.length,2);assert.equal(h.get('collection-history-run').value,current);
  let videos=h.get('replay-media').querySelectorAll('video');assert.equal(videos.length,2);assert(videos.every(node=>node.src.includes('current-')));
  for(const node of videos){node.duration=3;node.events.loadedmetadata();}await h.get('collection-replay-toggle').events.click();assert(videos.every(node=>!node.paused));
  const before=h.requests.length;h.get('collection-history-run').value=historical;h.get('collection-history-run').events.change();
  assert(videos.every(node=>node.paused),'switching runs stops playback from the old pair');
  videos=h.get('replay-media').querySelectorAll('video');assert.equal(videos.length,2,'same camera and media ids across runs never form a four-video pair');assert(videos.every(node=>node.src.includes('old-')));
  assert.equal(h.get('results-list').querySelectorAll('article').length,2);assert.match(h.get('collection-history-summary').textContent,/正在查看历史任务.*2 项回放/);assert.equal(h.get('collection-history-summary').title,historical);
  for(const node of videos){node.duration=4;node.events.loadedmetadata();}assert.equal(h.get('collection-replay-toggle').disabled,false);await h.get('collection-replay-toggle').events.click();assert(videos.every(node=>!node.paused));
  assert.equal(h.requests.length,before,'history browsing and local playback send no device actions');
  value.revision++;value.workflow.id='/runs/newly-generated-job';h.app.poll();h.reply(before,value);await settle();
  assert.equal(h.get('collection-history-run').value,historical,'a new current job cannot reset an existing history selection');assert.equal(h.get('replay-media').querySelectorAll('video')[0],videos[0],'unchanged history playback is not rebuilt on status refresh');
  h.app.selectTab('calibration');h.reply(before+1,snapshot());await settle();h.app.selectTab('collection');h.reply(before+2,value);await settle();
  assert.equal(h.get('collection-history-run').value,historical,'history selection survives switching business context');h.app.destroy();
}
console.log('PASS unified web: per-run history pairing, server-owned outcomes, workspace drafts, degree sliders, readonly auto-preview, calibration comparison, explicit collection stages, reviewed speed, replay, release and instance/media guards.');
