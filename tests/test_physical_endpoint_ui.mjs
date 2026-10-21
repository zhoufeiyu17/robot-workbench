import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const operations=await readFile(new URL('../web/operations.js',import.meta.url),'utf8');
const moduleUrl='data:text/javascript;base64,'+Buffer.from(operations).toString('base64');
const source=await readFile(new URL('../web/physical_endpoint.js',import.meta.url),'utf8');
const {mountPhysicalEndpoint}=await import('data:text/javascript;base64,'+Buffer.from(source.replace("'./operations.js'",JSON.stringify(moduleUrl))).toString('base64'));
class Element{
  constructor(tag){this.tag=tag;this.children=[];this.events={};this.dataset={};this.value='';this.textContent='';this.hidden=false;this.disabled=false;this.checked=false;this.open=false;}
  append(...values){this.children.push(...values);}prepend(...values){this.children.unshift(...values);}
  replaceChildren(...values){this.children=values;}addEventListener(name,callback){this.events[name]=callback;}setAttribute(name,value){this[name]=value;}
  showModal(){this.open=true;}close(){this.open=false;}fire(name){return this.events[name]?.({target:this,preventDefault(){}});}
}
const settle=async()=>{for(let i=0;i<24;i++)await Promise.resolve();};
const fixture=(environment='physical',manager='manager-one')=>({schema:'robot_workbench.environment.v1',csrf:'manager-token',target:{device_id:'device-'+environment,environment,manager_instance:manager},
  deployment:{sha256:'a'.repeat(64),files_unchanged:true},runtime:{connected:false},health:{busy:false,maintenance:null},selected_profile_id:environment,
  capabilities:{observe:{ready:true},control:{ready:false,reason:'未接入'},inference:{ready:false,reason:'未准备模型'}},
  profiles:['physical','fake'].map(id=>({id,device_id:'device-'+id,label:id+' registered',environment:id,configured:true,robot_address:id==='physical'?'10.1.2.3':undefined,
    compute_address:id==='fake'?'registered.dsw.invalid':undefined,manager_url:id==='physical'?'http://physical.invalid/':'http://fake.invalid/'})),
  operations:{connect_control:{allowed:true,label:'接入控制',impact:'准备登记依赖并连接设备，不开始业务动作',requires_support:environment==='physical'},recover_control:{allowed:true,label:'中断恢复',impact:'恢复新鲜会话，不续跑旧轨迹',requires_support:environment==='physical'},prepare_model:{allowed:true,label:'准备所选模型',impact:'不连接控制'},prepare_collection:{allowed:true,label:'准备数采',impact:'不连接控制'}},active_job:null,recent_jobs:[]});
function harness(environment='physical'){
  const nodes=[],main=new Element('main'),select=new Element('select'),entry=new Element('a');select.value=environment;entry.dataset.business='inference';
  const connect=new Element('button'),recover=new Element('button'),summary=new Element('p');
  const doc={body:main,getElementById:id=>({environment:select,'operations-link':entry,'environment-connect':connect,'environment-recover':recover,'environment-operation-status':summary})[id]||nodes.find(n=>n.id===id),querySelector:()=>main,createElement:tag=>{const n=new Element(tag);nodes.push(n);return n;}};
  const requests=[],navigated=[];let sequence=0;
  const api=mountPhysicalEndpoint({document:doc,fetch:(url,options={})=>new Promise((resolve,reject)=>requests.push({url,options,resolve,reject})),location:{href:'http://physical.invalid/?business=inference',search:'?business=inference',assign:url=>navigated.push(url)},crypto:{randomUUID:()=>`request-${++sequence}`},deploymentVersion:'a'.repeat(64),pollMs:0});
  return {api,requests,navigated,connect,recover,summary,doc,reply(index,value,ok=true){requests[index].resolve({ok,status:ok?200:409,json:async()=>value});}};
}
for(const environment of ['physical','fake']){
  const h=harness(environment);assert.equal(h.requests[0].url,'/api/environment?environment='+environment+'&view=summary');h.reply(0,fixture(environment));await settle();
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0,'read and navigation never connect or prepare');
  h.api.prepare('prepare_model',{model_id:'chosen',expected_bundle_digest:'b'.repeat(64)});h.api.prepare('prepare_model',{model_id:'chosen'});
  assert.equal(h.requests.length,2,'repeated selection cannot duplicate an in-flight submit');
  const body=JSON.parse(h.requests[1].options.body);assert.equal(body.action,'prepare_model');assert.equal(body.arms_supported,false);assert.equal(body.parameters.model_id,'chosen');assert.equal(h.api.elements.dialog.open,false);
  h.requests[1].reject(Error('reply lost'));await settle();h.reply(2,fixture(environment));await settle();
  assert.equal(h.api.elements.retry.hidden,false);h.api.elements.retry.fire('click');assert.deepEqual(JSON.parse(h.requests[3].options.body),body,'ambiguous retry preserves id, target and model digest');
  h.reply(3,{job:{status:'running'}});await settle();const next=fixture(environment);next.recent_jobs=[{request:body,status:'failed',phase:'verify',error:{message:'model worker exited'}}];h.reply(4,next);await settle();
  assert.match(h.summary.textContent,/操作失败.*model worker exited/);assert(!h.requests.some(r=>r.url.includes('/api/actions/')||r.url.includes('/api/session/')));h.api.destroy();
}
{
  const h=harness();h.reply(0,fixture());await settle();h.connect.fire('click');
  assert.equal(h.requests[1].url,'/api/operations/target?environment=physical');
  assert.deepEqual(JSON.parse(h.requests[1].options.body),{profile_id:'physical',expected_target:fixture().target,robot_address:'10.1.2.3'});
  assert.equal(h.api.elements.dialog.open,false,'identity resolves before any control confirmation');
  h.reply(1,fixture());await settle();assert.equal(h.api.elements.dialog.open,true);assert.equal(h.api.elements.confirm.disabled,true);
  h.api.elements.support.checked=true;h.api.elements.support.fire('change');h.api.elements.confirm.fire('click');h.api.elements.confirm.fire('click');
  assert.equal(h.requests.length,3);const body=JSON.parse(h.requests[2].options.body);assert.equal(body.action,'connect_control');assert.equal(body.arms_supported,true);
  assert.equal(body.parameters,undefined);h.api.destroy();
}
{
  const h=harness();h.reply(0,fixture());await settle();h.recover.fire('click');assert.equal(h.api.elements.dialog.open,true);
  assert.equal(h.api.panel.open,false);
  assert(!h.api.panel.children.includes(h.api.elements.dialog),'top recovery modal must be outside the collapsed access panel');
  h.api.elements.cancel.fire('click');assert.equal(h.api.elements.dialog.open,false);
  h.recover.fire('click');assert.equal(h.api.elements.dialog.open,true,'cancelled recovery leaves the page usable');
  h.api.refresh();h.reply(1,fixture('physical','replacement'));await settle();assert.equal(h.api.elements.dialog.open,false);assert.equal(h.api.elements.support.checked,false);
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0,'manager change revokes confirmation without replay');h.api.destroy();
}
{
  const h=harness();h.reply(0,fixture());await settle();h.api.elements.address.value='10.9.9.9';h.api.elements.address.fire('input');h.connect.fire('click');
  h.reply(1,{error:'地址未登记或尚未部署受支持服务'},false);await settle();assert.match(h.summary.textContent,/未登记或尚未部署/);assert.equal(h.requests.length,2);assert.equal(h.api.elements.dialog.open,false);
  h.api.refresh();h.reply(2,fixture());await settle();assert.match(h.summary.textContent,/未登记或尚未部署/,'normal polling preserves failed address checks');
  h.api.elements.address.value='10.1.2.3';h.api.elements.address.fire('input');assert.doesNotMatch(h.summary.textContent,/未登记或尚未部署/);
  h.api.elements.check.fire('click');h.reply(3,fixture());await settle();h.api.refresh();h.reply(4,fixture());await settle();
  assert.doesNotMatch(h.summary.textContent,/目标未接入/);assert.equal(h.api.state.runtime.connected,false);h.api.destroy();
}
{
  const h=harness('fake');h.reply(0,fixture('fake'));await settle();h.api.elements.compute.value='203.0.113.7';h.api.elements.compute.fire('input');h.api.elements.check.fire('click');
  assert.equal(JSON.parse(h.requests[1].options.body).compute_address,'203.0.113.7');
  h.reply(1,{error:'计算地址未匹配登记档案'},false);await settle();h.api.refresh();h.reply(2,fixture('fake'));await settle();
  assert.match(h.summary.textContent,/计算地址未匹配/);assert.equal(h.api.elements.compute.value,'203.0.113.7');
  h.api.refresh();h.requests[3].reject(Error('poll offline'));await settle();assert.match(h.summary.textContent,/计算地址未匹配.*环境状态未知/);
  h.api.refresh();h.reply(4,fixture('fake'));await settle();assert.match(h.summary.textContent,/计算地址未匹配/);
  h.api.elements.check.fire('click');h.reply(5,fixture('fake'));await settle();assert.doesNotMatch(h.summary.textContent,/计算地址未匹配/,'successful explicit recheck clears its prior failure');
  h.api.elements.check.fire('click');h.reply(6,{error:'再次未匹配'},false);await settle();
  h.api.prepare('prepare_model');assert.doesNotMatch(h.summary.textContent,/再次未匹配/,'a new operation replaces the target-check error');
  assert.equal(JSON.parse(h.requests[7].options.body).action,'prepare_model');assert.equal(h.api.state.runtime.connected,false);
  h.api.destroy();
}
for(const malicious of [false,true]){
  const h=harness();h.reply(0,fixture());await settle();h.api.elements.profile.value='fake';h.api.elements.profile.fire('change');h.api.elements.check.fire('click');
  h.reply(1,{redirect_url:malicious?'https://unregistered.invalid/':'http://fake.invalid/',environment:fixture('fake'),controller_acquired:false});await settle();
  if(malicious){assert.equal(h.navigated.length,0);assert.match(h.summary.textContent,/不匹配/);}
  else{assert.equal(h.navigated[0],'http://fake.invalid/?environment=fake&business=inference');assert.equal(h.requests.length,2,'cross-manager result only navigates; it cannot write remote target on current origin');}
  h.api.destroy();
}
{
  const h=harness();h.api.setEnvironment('fake');h.reply(1,fixture('fake'));await settle();h.reply(0,fixture());await settle();assert.equal(h.api.state.target.environment,'fake');
  h.api.refresh();h.requests[2].reject(Error('connection refused'));await settle();assert.match(h.summary.textContent,/状态未知.*不能证明当前可用/);assert.equal(h.connect.disabled,true);h.api.destroy();
}
{
  const h=harness();const state=fixture();h.reply(0,state);await settle();h.api.prepare('prepare_model');
  const stop=h.api.stop();assert.equal(h.requests[2].url,'/api/operations/stop?environment=physical','independent stop bypasses in-flight preparation');
  assert.deepEqual(JSON.parse(h.requests[2].options.body),{expected_device_id:'device-physical',request_id:'request-2'});
  h.requests[2].reject(Error('backend unreachable'));await assert.rejects(stop);assert.match(h.summary.textContent,/停止结果未知/);
  h.api.destroy();
  const stale=harness();state.deployment.sha256='b'.repeat(64);stale.reply(0,state);await settle();const pending=stale.api.stop();assert.equal(stale.requests[1].url,'/api/operations/stop?environment=physical');stale.reply(1,{stop_hold_return_confirmed:false});await pending;stale.api.destroy();
}
for(const connected of [true,null]){
  const h=harness();const state=fixture();state.runtime.connected=connected;h.reply(0,state);await settle();
  await h.api.navigateEnvironment('fake');assert.equal(h.requests.length,1);assert.equal(h.navigated.length,0);
  assert.match(h.summary.textContent,connected===true?/先停止任务.*释放/:/尚未核验/);h.api.destroy();
}
{
  const h=harness();h.reply(0,fixture());await settle();const change=h.api.navigateEnvironment('fake',{page:'/operations'});
  assert.equal(h.requests[1].url,'/api/operations/target?environment=physical');assert.equal(JSON.parse(h.requests[1].options.body).profile_id,'fake');
  h.reply(1,{redirect_url:'http://fake.invalid/?environment=fake',environment:fixture('fake'),controller_acquired:false});await change;
  assert.equal(h.navigated[0],'http://fake.invalid/operations?environment=fake&business=inference');assert.equal(h.api.state.target.environment,'physical');
  assert.equal(h.requests.filter(row=>row.url.includes('/action?')).length,0,'switching is only target discovery and full reload, no control operation');h.api.destroy();
}
for(const entry of ['navigate','profile-check']){
  const h=harness('fake'),state=fixture('fake');state.runtime={};state.recovery={required:false};
  state.health={backend_reachable:false,busy:null,services:{workbench:{state:'stopped',running:false,owned:true}}};
  h.reply(0,state);await settle();let change;
  if(entry==='navigate')change=h.api.navigateEnvironment('physical',{page:'/operations'});
  else{h.api.elements.profile.value='physical';h.api.elements.profile.fire('change');h.api.elements.check.fire('click');}
  assert.equal(h.requests[1].url,'/api/operations/target?environment=fake');
  h.reply(1,{redirect_url:'http://physical.invalid/?environment=physical',environment:fixture('physical'),controller_acquired:false});
  await change;await settle();assert.equal(h.navigated.length,1);assert.equal(h.api.elements.dialog.open,false);
  assert(h.requests.every(row=>!row.url.includes('/action?')&&!row.url.includes('/stop?')),'verified stopped switching never connects, starts or releases control');h.api.destroy();
}
for(const workbench of [{state:'running',running:true},{state:'unknown',running:false},{state:'stopped',running:null}]){
  const h=harness('fake'),state=fixture('fake');state.runtime={};state.recovery={required:false};
  state.health={backend_reachable:false,busy:null,services:{workbench}};h.reply(0,state);await settle();
  await h.api.navigateEnvironment('physical');assert.equal(h.requests.length,1);assert.equal(h.navigated.length,0);
  h.api.elements.profile.value='physical';h.api.elements.profile.fire('change');h.api.elements.check.fire('click');await settle();
  assert.equal(h.requests.length,1,'both profile resolution and dropdown navigation reject live-unreachable or unconfirmed shutdown');h.api.destroy();
}
assert(!source.includes('/api/physical-endpoint'),'new UI has no second endpoint binding writer');
assert(!source.includes('18868'),'no guessed port default');
console.log('PASS environment access: registered profiles, explicit control confirmation, same-request retry, target fencing, independent Stop, unknown state and safe cross-manager navigation.');
