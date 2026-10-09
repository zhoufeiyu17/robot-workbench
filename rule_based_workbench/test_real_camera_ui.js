// CPU browser shim: one bounded frame request, cancellation, pacing and stale-response isolation.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const script=fs.readFileSync(__dirname+'/index.html','utf8').match(/<script>([\s\S]*)<\/script>/)[1];new Function(script);
const nodes=new Map(),get=id=>{if(!nodes.has(id))nodes.set(id,{dataset:{},value:'base_0',decode:async()=>{}});return nodes.get(id);};
let tick=0,nonce=0,blob=0;const timers=new Map(),requests=[],revoked=[],events={};
const ctx=vm.createContext({workbenchMode:'real',realCameraTimer:null,realCameraRequest:null,realCameraGeneration:0,
 realCameraKey:null,realCameraBlob:null,realCameraPaused:false,realSnapshot:{cameras:{base_0:{ready:true},left_wrist_0:{ready:true},right_wrist_0:{ready:true}}},
 $:get,text:()=>{},performance:{now:()=>tick},AbortController,URL:{createObjectURL:()=>`blob:${++blob}`,revokeObjectURL:u=>revoked.push(u)},
 setTimeout:(fn,ms)=>{timers.set(++nonce,{fn,ms});return nonce;},clearTimeout:id=>timers.delete(id),
 fetch:(url,options)=>new Promise((resolve,reject)=>{requests.push({url,options,resolve});options.signal.addEventListener('abort',()=>reject(Error('aborted')));}),
 document:{hidden:false,addEventListener:(name,fn)=>events[name]=fn},window:{addEventListener:(name,fn)=>events[name]=fn},
 clearComparisonImage:()=>ctx.stopRealCamera()});
vm.runInContext(script.slice(script.indexOf('function stopRealCamera()'),script.indexOf('async function refreshReal()')),ctx);
const settle=async()=>{for(let i=0;i<8;i++)await Promise.resolve();};
const respond=i=>requests[i].resolve({ok:true,blob:async()=>({})});
(async()=>{
 ctx.updateRealCamera();ctx.updateRealCamera();assert.equal(requests.length,1);assert.match(requests[0].url,/base_0\/frame.jpg$/);
 respond(0);await settle();assert.equal(get('#source-image').src,'blob:1');assert.equal(timers.size,1);assert.equal([...timers.values()][0].ms,33);
 const next=[...timers.values()][0];timers.clear();next.fn();assert.equal(requests.length,2);
 get('#source-camera').value='left_wrist_0';ctx.updateRealCamera();assert(requests[1].options.signal.aborted);assert.equal(requests.length,3);
 respond(1);await settle();assert.equal(get('#source-image').src,'blob:1','cancelled old camera cannot overwrite');
 respond(2);await settle();assert.equal(get('#source-image').dataset.camera,'left_wrist_0');assert.equal(get('#source-image').src,'blob:2');
 events.pagehide();assert.equal(timers.size,0);ctx.updateRealCamera();assert.equal(requests.length,3);
 events.pageshow();assert.equal(requests.length,4);ctx.document.hidden=true;events.visibilitychange();assert(requests[3].options.signal.aborted);await settle();assert.equal(timers.size,0);
 ctx.document.hidden=false;events.visibilitychange();assert.equal(requests.length,5);
 ctx.workbenchMode='dual_dsw';ctx.stopRealCamera();respond(4);await settle();assert.equal(timers.size,0);assert.equal(get('#source-image').src,'blob:2');
 assert(revoked.includes('blob:1')&&revoked.includes('blob:2'));
 console.log('PASS bounded real preview: 33ms pacing, single in-flight, camera/mode cancellation, stale-frame isolation, visibility/page lifecycle, blob cleanup');
})().catch(e=>{console.error(e);process.exitCode=1;});
