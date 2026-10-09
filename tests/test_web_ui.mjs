import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {webcrypto} from 'node:crypto';

const source=await readFile(new URL('../web/app.js',import.meta.url),'utf8');
const html=await readFile(new URL('../web/index.html',import.meta.url),'utf8');
const {createWorkbench,actionPermission,countedProgress,frameAgeSeconds,safeMediaUrl,verifyPreviewFrame}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));

class Element {
  constructor(tag='div'){this.tagName=tag;this.children=[];this.attributes={};this.dataset={};this.events={};this.value='';this.checked=false;this.hidden=false;this.disabled=false;this.textContent='';this.className='';this.classList={toggle:(name,on)=>{const names=new Set(this.className.split(' ').filter(Boolean));if(on)names.add(name);else names.delete(name);this.className=[...names].join(' ');},contains:name=>this.className.split(' ').includes(name)};}
  append(...children){this.children.push(...children);}
  replaceChildren(...children){this.children=children;}
  setAttribute(name,value){this.attributes[name]=String(value);}
  removeAttribute(name){delete this.attributes[name];if(name==='src')this.src='';}
  addEventListener(name,callback){this.events[name]=callback;}
  querySelectorAll(selector){const matches=node=>selector.startsWith('.')?node.classList.contains(selector.slice(1)):node.tagName===selector;return this.children.flatMap(child=>[...(matches(child)?[child]:[]),...child.querySelectorAll(selector)]);}
  querySelector(selector){return this.querySelectorAll(selector)[0]||null;}
  async decode(){}
}
function documentStub(){
  const nodes=new Map([...html.matchAll(/id="([^"]+)"/g)].map(match=>[match[1],new Element()]));
  for(const id of ['head','left','right','preview','mesh']){const card=nodes.get('media-'+id),image=new Element('img'),empty=new Element('p'),meta=new Element('p');empty.className='empty';meta.className='media-meta';card.append(image,empty,meta);}
  const tabs=['calibration','collection','results','diagnostics'].map(tab=>{const button=new Element('button');button.dataset.tab=tab;return button;});
  return {hidden:false,nodes,tabs,events:{},getElementById(id){assert(nodes.has(id),'Missing UI node '+id);return nodes.get(id);},createElement(tag){return new Element(tag);},querySelectorAll(selector){return selector==='[data-tab]'?tabs:[];},addEventListener(name,callback){this.events[name]=callback;}};
}
const response=(value,headers={})=>({ok:true,status:200,headers:{get:key=>headers[key]??null},json:async()=>value,blob:async()=>new Blob(['fixture'])});
const settle=async()=>{for(let i=0;i<8;i++)await Promise.resolve();};
function snapshot(overrides={}){
  return {schema:'robot-workbench-ui.v1',revision:1,server_wall_ns:10e9,csrf:'fixture-only',device:{id:'fixture-device',label:'Fixture device',environment:'fake'},runtime:{connected:true,holder:'calibration',feedback_healthy:true,motion_status:'idle',hold:{left:'position',right:'position'},allowed_operations:Object.fromEntries(['handoff','stop','connect','cancel','recover','mesh','manual_preview','select_result'].map(action=>[action,{allowed:true,reason:''}]))},workflow:{id:'fixture-run',revision:1,status:'planning',stage_id:'head.plan',title:'头部规划',message:'仍在规划',progress:null,primary_action:{id:'generate_plan',label:'生成计划',allowed:true,reason:''},outcomes:{}},media:[],results:[],diagnostics:[],mesh:{requested:false,applied:false},...overrides};
}
function manualSnapshot(){
  const value=snapshot();value.runtime.allowed_operations.manual_queue={allowed:true};value.runtime.allowed_operations.manual_queue_preview={allowed:true};
  value.manual={joint_names:Array.from({length:12},(_,i)=>'J'+i),positions:Array(12).fill(.1),limits:Array.from({length:12},()=>[-1,1]),jaw_gaps_mm:[80,80],default_joint_speed_rad_s:.2,default_jaw_speed_mm_s:10,queue:{available:true,revision:4,waypoints:[]}};
  return value;
}
function harness(){
  const document=documentStub(),requests=[],intervals=[],revoked=[],timeouts=new Map();let now=0,timerId=0;
  const app=createWorkbench({document,location:{href:'http://localhost:8110/'},performance:{now:()=>now},crypto:webcrypto,URL:{createObjectURL:()=>`blob:fixture-${requests.length}`,revokeObjectURL:value=>revoked.push(value)},setInterval:fn=>{intervals.push(fn);return intervals.length;},clearInterval:()=>{},setTimeout:(fn,ms)=>{timeouts.set(++timerId,{fn,ms});return timerId;},clearTimeout:id=>timeouts.delete(id),fetch:(url,options={})=>new Promise(resolve=>requests.push({url,options,resolve}))});
  return {app,document,requests,intervals,revoked,timeouts,advance:ms=>now+=ms,get:id=>document.getElementById(id),reply(index,value,headers){requests[index].resolve(response(value,headers));},nextFrame(){const timer=[...timeouts.entries()].find(([,t])=>t.ms!==2500);assert(timer,'frame timer scheduled');timeouts.delete(timer[0]);timer[1].fn();return timer[1].ms;}};
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
  positions[0].value=.25;speeds[0].value=.05;h.get('jaw-speed').value=5;h.get('queue-add').events.click();
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
}
{
  const h=harness();h.app.start();const value=previewSnapshot();
  value.workflow.review={id:'approved-motion',summary:'already reviewed',physical_clearance_required:true};value.workflow.primary_action={id:'manual_execute',allowed:true};
  h.reply(0,value);await settle();h.get('clearance').checked=true;
  const request=h.get('show-target').events.click();assert.equal(h.requests[1].url,'/api/actions/preview_target');
  assert.deepEqual(JSON.parse(h.requests[1].options.body).q12,Array(12).fill(.1));assert.equal(h.get('clearance').checked,true,'read-only display must not revoke a motion review');
  h.reply(1,{client_revision:8,token:null});await request;await settle();
  value.revision=2;value.preview.frames.target.ready=true;h.reply(2,value);await settle();
  assert.equal(h.requests.length,3,'older state revision cannot start an image request for a newer target');
  value.revision=3;value.preview.revisions.target=8;value.preview.frames.target.revision=8;h.app.poll();h.reply(3,value);await settle();
  assert.match(h.requests[4].url,/view=target/);h.reply(4,{},previewHeaders('target',7));await settle();
  assert.equal(h.get('media-preview').querySelector('img').hidden,true);assert.match(h.get('media-preview').querySelector('.media-meta').textContent,/修订不匹配/);
  h.nextFrame();h.reply(5,{},previewHeaders('target',8));await settle();assert.equal(h.get('media-preview').querySelector('img').hidden,false);
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
console.log('PASS unified web: review/stop gates, calibration tools, target and mesh frame identities, display-only actions, coalesced overview, recovery isolation and truthful L1 availability.');
