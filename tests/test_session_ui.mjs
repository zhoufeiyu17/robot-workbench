import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {webcrypto} from 'node:crypto';
const source=await readFile(new URL('../web/session.js',import.meta.url),'utf8');
const html=await readFile(new URL('../web/session.html',import.meta.url),'utf8');
const {createSessionMonitor,sessionRemaining,sessionDuration,requestId}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
const settle=async()=>{for(let i=0;i<12;i++)await Promise.resolve();};
const fixture=(patch={})=>({schema:'fake-session.v1',managed:true,profile:'full_fake',manager_instance_id:'manager-a',generation:0,csrf:'legacy-token',status:'stopped',message:'服务已停止',error:null,deadline_wall_ns:1791726000000000000,remaining_wall_s:60,stage:{label:'已停止'},current_run:{id:'old-run'},allowed:{start:{allowed:true},stop:{allowed:true}},...patch});
function harness(page=true){
  const ids=page?[...html.matchAll(/id="([^"]+)"/g)].map(m=>m[1]):['session-link','session-summary'];
  const nodes=new Map(ids.map(id=>[id,{textContent:'',hidden:true,events:{},addEventListener(name,fn){this.events[name]=fn;}}]));
  const requests=[],intervals=new Map();let now=0,seq=0;
  const app=createSessionMonitor({document:{getElementById:id=>nodes.get(id)||null},fetch:(url,options={})=>new Promise((resolve,reject)=>requests.push({url,options,resolve,reject})),performance:{now:()=>now},crypto:webcrypto,
    setInterval:fn=>{intervals.set(++seq,fn);return seq;},clearInterval:id=>intervals.delete(id),setTimeout:()=>++seq,clearTimeout(){},confirm:()=>assert.fail('read-only session cannot ask for lifecycle confirmation')});
  return {app,requests,nodes,get:id=>nodes.get(id),advance:ms=>now+=ms,tick:()=>{for(const fn of intervals.values())fn();},reply(index,value,status=200){requests[index].resolve({ok:status===200,status,json:async()=>value});}};
}
assert.equal(sessionRemaining(fixture(),2000),58);assert.equal(sessionRemaining(fixture(),61000),0);assert.equal(sessionRemaining({}),null);
assert.equal(sessionDuration(3601),'1 小时 0 分 1 秒');assert.equal(sessionDuration(null),'未知');
{
  const h=harness();h.app.start();h.app.start();assert.equal(h.requests.length,1);h.reply(0,fixture());await settle();
  assert.equal(h.get('session-operations').href,'/operations?environment=fake&business=calibration');assert.match(h.get('session-guidance').textContent,/统一到服务运维.*只显示/);
  await h.app.send('start');await h.app.send('stop');assert.equal(h.requests.length,1,'legacy compatibility entry has no lifecycle write even when old server allows it');
  assert.match(h.get('session-request').textContent,/只读/);assert(!h.nodes.has('session-start')&&!h.nodes.has('session-stop'));
  h.app.poll();h.reply(1,fixture({status:'ready',workbench_url:'https://untrusted.invalid/'}));await settle();assert.equal(h.get('session-enter').href,'/');assert.equal(h.get('session-enter').hidden,false);
  h.app.poll();h.requests[2].reject(Error('network down'));await settle();assert.match(h.get('session-message').textContent,/无法读取/);assert.equal(h.get('session-enter').hidden,true);h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,fixture());await settle();h.app.poll();const old=h.requests[1];h.app.poll(true);assert.equal(old.options.signal.aborted,true);
  h.reply(2,fixture({manager_instance_id:'new-manager',generation:3,status:'ready'}));await settle();h.reply(1,fixture());await settle();assert.equal(h.app.state.manager_instance_id,'new-manager');
  h.app.poll();h.reply(3,fixture({manager_instance_id:'manager-a',generation:8}));await settle();assert.equal(h.app.state.manager_instance_id,'new-manager');
  h.app.poll();h.reply(4,fixture({manager_instance_id:'new-manager',generation:2,status:'stopped'}));await settle();assert.equal(h.app.state.status,'ready');h.app.destroy();
}
{
  const h=harness();h.app.start();h.reply(0,fixture({status:'ready',remaining_wall_s:1}));await settle();h.advance(1500);h.tick();
  assert.match(h.get('session-state').textContent,/已就绪/);assert.match(h.get('session-remaining').textContent,/等待后台核验清退/);assert.equal(h.get('session-enter').hidden,true);
  h.reply(1,fixture({status:'expired',remaining_wall_s:0,cleanup_verified:true}));await settle();assert.match(h.get('session-remaining').textContent,/自有服务进程已退出/);h.app.destroy();
  const persistent=harness();persistent.app.start();persistent.reply(0,fixture({persistent_service:true,deadline_wall_ns:null,remaining_wall_s:null}));await settle();assert.match(persistent.get('session-remaining').textContent,/手动停止/);await persistent.app.send('start');assert.equal(persistent.requests.length,1);persistent.app.destroy();
}
{
  const h=harness(false);h.app.start();h.reply(0,{},404);await settle();assert.equal(h.get('session-link').hidden,true);h.tick();assert.equal(h.requests.length,1);h.app.destroy();
}
assert.equal(requestId({getRandomValues:values=>values.fill(255)}),'ffffffff-ffff-4fff-bfff-ffffffffffff');assert.throws(()=>requestId({}),/缺少安全随机数/);
assert(!source.includes('innerHTML'));assert(!source.includes('/api/session/${action}'));assert(!html.includes('id="session-start"'));
console.log('PASS session compatibility: read-only status, unified operations link, no legacy writer, stale identity fencing, expiry evidence and secure request identities.');
