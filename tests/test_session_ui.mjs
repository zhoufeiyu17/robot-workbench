import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {webcrypto} from 'node:crypto';

const source=await readFile(new URL('../web/session.js',import.meta.url),'utf8');
const html=await readFile(new URL('../web/session.html',import.meta.url),'utf8');
const {createSessionMonitor,sessionRemaining,sessionDuration}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
const settle=async()=>{for(let n=0;n<10;n++)await Promise.resolve();};
const snapshot=(patch={})=>({schema:'fake-session.v1',managed:true,profile:'full_fake',manager_instance_id:'manager-a',generation:0,csrf:'manager-token',status:'stopped',message:'服务已停止',error:null,deadline_wall_ns:1791726000000000000,remaining_wall_s:60,stage:{id:'stopped',label:'已停止',elapsed_s:2},current_run:{id:'fake-run'},allowed:{start:{allowed:true,reason:''},stop:{allowed:false,reason:'服务已经停止'}},workbench_url:'/',...patch});
const response=(value,status=200)=>({ok:status>=200&&status<300,status,json:async()=>value});
function harness({page=true}={}){
  const ids=page?[...html.matchAll(/id="([^"]+)"/g)].map(m=>m[1]):['session-link','session-summary'];
  const nodes=new Map(ids.map(id=>[id,{textContent:'',hidden:true,disabled:false,events:{},addEventListener(name,fn){this.events[name]=fn;}}]));
  const requests=[],intervals=new Map(),timeouts=new Map(),confirmations=[];let now=0,timer=0,confirmed=false;
  const app=createSessionMonitor({document:{getElementById:id=>nodes.get(id)||null},fetch:(url,options={})=>new Promise((resolve,reject)=>requests.push({url,options,resolve,reject})),performance:{now:()=>now},crypto:webcrypto,confirm:message=>{confirmations.push(message);return confirmed;},setInterval:(fn,ms)=>{intervals.set(++timer,{fn,ms});return timer;},clearInterval:id=>intervals.delete(id),setTimeout:(fn,ms)=>{timeouts.set(++timer,{fn,ms});return timer;},clearTimeout:id=>timeouts.delete(id)});
  return {app,requests,intervals,timeouts,confirmations,get:id=>nodes.get(id),advance:ms=>now+=ms,confirm:value=>confirmed=value,reply:(index,value,status)=>requests[index].resolve(response(value,status)),tick:()=>{for(const item of intervals.values())item.fn();}};
}
assert.equal(sessionRemaining(snapshot(),2000),58);assert.equal(sessionRemaining(snapshot(),61000),0);assert.equal(sessionRemaining({}),null);
assert.equal(sessionDuration(3601),'1 小时 0 分 1 秒');assert.equal(sessionDuration(null),'未知');

{
  const h=harness();h.app.start();h.reply(0,snapshot({profile:'protocol_fake'}));await settle();
  assert.match(h.get('session-profile').textContent,/CPU 协议 fake 验收（无GPU\/物理图像）/);assert.equal(h.get('session-start').textContent,'启动协议 fake');
  h.app.poll();h.reply(1,snapshot({profile:'full_fake'}));await settle();assert.match(h.get('session-profile').textContent,/完整 fake/);assert.equal(h.get('session-start').textContent,'启动完整 fake');h.app.destroy();
}

{
  const h=harness();h.app.start();h.app.start();assert.equal(h.requests.length,1);assert.equal([...h.intervals.values()][0].ms,1000);
  h.reply(0,snapshot());await settle();assert.equal(h.get('session-start').disabled,false);assert.equal(h.get('session-stop').disabled,true);assert.match(h.get('session-stop-reason').textContent,/已经停止/);
  assert.equal(h.requests.filter(r=>r.options.method==='POST').length,0,'loading never starts services');
  h.advance(1000);h.tick();assert.equal(h.requests.length,2);h.reply(1,snapshot({status:'stopped',allowed:{start:{allowed:false,reason:'GPU 仍由其他会话占用'},stop:{allowed:false,reason:'未启动'}}}));await settle();
  assert.equal(h.get('session-start').disabled,true);assert.match(h.get('session-start-reason').textContent,/其他会话/);await h.app.send('start');assert.equal(h.requests.length,2);
  h.app.destroy();assert.equal(h.intervals.size,0);
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();
  const promise=h.app.send('start');assert.equal(h.get('session-start').disabled,true);const request=h.requests[1],body=JSON.parse(request.options.body);
  assert.equal(request.url,'/api/session/start');assert.equal(request.options.headers['X-CSRF-Token'],'manager-token');assert.equal(body.manager_instance_id,'manager-a');assert.equal(body.generation,0);assert.equal(typeof body.request_id,'string');assert.equal(body.confirm,undefined);
  h.reply(1,snapshot({generation:1,status:'starting',allowed:{start:{allowed:false,reason:'正在启动'},stop:{allowed:true}}}));await promise;
  assert.equal(h.app.state.status,'starting');assert.match(h.get('session-request').textContent,/已受理/);assert.equal(h.requests[2].url,'/api/session');
  h.reply(2,snapshot({generation:1,status:'ready',workbench_url:'https://untrusted.invalid/',allowed:{start:{allowed:false,reason:'已经就绪'},stop:{allowed:true}}}));await settle();
  assert.equal(h.get('session-enter').hidden,false);assert.equal(h.get('session-enter').href,'/');assert.match(h.get('session-request').textContent,/服务已就绪/);
  await h.app.send('stop');assert.equal(h.confirmations.length,1);assert.match(h.confirmations[0],/中断当前 fake.*保留已保存/);assert.equal(h.requests.length,3,'cancel leaves services alone');
  h.confirm(true);const stop=h.app.send('stop');assert.equal(JSON.parse(h.requests[3].options.body).confirm,true);assert.equal(JSON.parse(h.requests[3].options.body).generation,1);
  h.reply(3,snapshot({generation:2,status:'stopping',allowed:{start:{allowed:false,reason:'正在清退'},stop:{allowed:false,reason:'正在清退'}}}));await stop;assert.equal(h.app.state.status,'stopping');
  h.reply(4,snapshot({generation:2,status:'stopped',cleanup_verified:true}));await settle();assert.match(h.get('session-request').textContent,/停止已完成/);h.app.poll();
  h.app.destroy();assert.equal(h.requests.at(-1).options.signal.aborted,true);
}
{
  const h=harness();h.app.start();h.reply(0,snapshot());await settle();const action=h.app.send('start');
  h.reply(1,{error:'<script>启动条件发生变化</script>'},409);await action;assert.match(h.get('session-request').textContent,/<script>启动条件发生变化<\/script>/);assert.equal(h.requests.filter(r=>r.options.method==='POST').length,1);
  h.reply(2,snapshot({status:'failed',error:'<img onerror=bad>',allowed:{start:{allowed:false,reason:'清退尚未确认'},stop:{allowed:true}}}));await settle();assert.equal(h.get('session-error').textContent,'<img onerror=bad>');assert.equal(h.get('session-start').disabled,true);
  h.app.poll();h.requests.at(-1).reject(Error('network unavailable'));await settle();assert.match(h.get('session-message').textContent,/无法读取.*重新核验/);assert.equal(h.get('session-stop').disabled,true);h.app.destroy();
}
{
  const h=harness();h.app.start();const old=h.requests[0];h.app.poll(true);assert.equal(old.options.signal.aborted,true);
  h.reply(1,snapshot({manager_instance_id:'manager-b',generation:3}));await settle();old.resolve(response(snapshot({status:'ready'})));await settle();assert.equal(h.app.state.manager_instance_id,'manager-b');
  const action=h.app.send('start');h.app.poll(true);h.reply(3,snapshot({manager_instance_id:'manager-c',generation:0}));await settle();h.reply(2,snapshot({manager_instance_id:'manager-b',generation:4,status:'ready'}));await action;assert.equal(h.app.state.manager_instance_id,'manager-c');assert.match(h.get('session-request').textContent,/管理服务已更换/);
  h.reply(4,snapshot({manager_instance_id:'manager-b',generation:5,status:'ready'}));await settle();assert.equal(h.app.state.manager_instance_id,'manager-c','a retired manager never replaces the current one');
  h.app.poll();h.reply(5,snapshot({manager_instance_id:'manager-c',generation:4,status:'starting'}));await settle();h.app.poll();h.reply(6,snapshot({manager_instance_id:'manager-c',generation:3,status:'stopped'}));await settle();assert.equal(h.app.state.status,'starting');h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,snapshot({status:'ready',remaining_wall_s:1,allowed:{start:{allowed:true},stop:{allowed:true}}}));await settle();h.advance(1500);h.tick();
  assert.match(h.get('session-state').textContent,/已就绪/,'browser countdown does not assert process death');assert.match(h.get('session-remaining').textContent,/等待后台核验清退/);assert.equal(h.get('session-start').disabled,true);assert.equal(h.get('session-enter').hidden,true);assert.equal(h.get('session-stop').disabled,false);
  h.reply(1,snapshot({status:'expired',remaining_wall_s:0,cleanup_verified:true,allowed:{start:{allowed:false,reason:'时间已到'},stop:{allowed:false,reason:'已清退'}}}));await settle();assert.match(h.get('session-state').textContent,/已到期/);assert.match(h.get('session-remaining').textContent,/自有服务进程已退出/);assert.match(h.get('session-guidance').textContent,/新的服务时间与资源授权/);h.app.destroy();
}
{
  const h=harness({page:false});h.app.start();h.reply(0,{},404);await settle();assert.equal(h.get('session-link').hidden,true);h.tick();assert.equal(h.requests.length,1,'standalone API 404 quietly disables management polling');h.app.destroy();
  const managed=harness({page:false});managed.app.start();managed.reply(0,snapshot({status:'ready'}));await settle();assert.equal(managed.get('session-link').hidden,false);assert.match(managed.get('session-summary').textContent,/已就绪/);
  managed.app.poll();managed.requests[1].reject(Error('temporary outage'));await settle();assert.equal(managed.get('session-link').hidden,false);assert.match(managed.get('session-summary').textContent,/暂不可读/);managed.app.destroy();
}
assert(!source.includes('innerHTML'),'server diagnostics are rendered as text');
console.log('session UI: lifecycle refresh, permission reasons, explicit start/stop, stale responses, expiry and standalone isolation passed');
