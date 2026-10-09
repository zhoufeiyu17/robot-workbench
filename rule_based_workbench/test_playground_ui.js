// CPU only: node rule_based_workbench/test_playground_ui.js
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const html=fs.readFileSync(__dirname+'/index.html','utf8'),script=html.match(/<script>([\s\S]*)<\/script>/)[1];
new Function(script);
const nodes=new Map(),events={},requests=[],posts=[],timers=new Map();let nonce=0,blob=0;
function node(){return {value:'',dataset:{},children:[],hidden:false,open:true,checked:false,
 append(...items){this.children.push(...items)},replaceChildren(){this.children=[]},removeAttribute(k){delete this[k]},
 setAttribute(k,v){this[k]=v},addEventListener(){},decode:async()=>{}};}
const get=id=>{if(!nodes.has(id))nodes.set(id,node());return nodes.get(id)};
get('#pg-speed').value='30';
const ctx=vm.createContext({modeCsrf:'test-csrf',$:get,text:(id,s)=>get('#'+id).textContent=s,
 performance:{now:()=>1000},secondsLabel:s=>Math.ceil(s)+' 秒',location:{href:'http://localhost:8119/',origin:'http://localhost:8119'},
 URL:class extends URL{static createObjectURL(){return 'blob:'+(++blob)}static revokeObjectURL(){}},AbortController,
 document:{hidden:false,createElement:node,addEventListener:(name,fn)=>events[name]=fn},window:{addEventListener:(name,fn)=>events[name]=fn},
 setTimeout:(fn,ms)=>{timers.set(++nonce,{fn,ms});return nonce},clearTimeout:id=>timers.delete(id),
 fetch:(url,options)=>new Promise((resolve,reject)=>{requests.push({url,options,resolve});options.signal.addEventListener('abort',()=>reject(Error('aborted')))}),
 post:async(url,payload)=>posts.push({url,payload})});
vm.runInContext(script.slice(script.indexOf('// Playground uses'),script.indexOf('pgRefresh();setInterval(pgRefresh')),ctx);
const ready={status:'ready',environment:{ready:true,checks:[{key:'control',label:'真机控制',status:'ready'}]},plan:{validated:true},cameras:{}};
function render(s){ctx.state=s;vm.runInContext('pgRender(state)',ctx)}
const settle=async()=>{for(let i=0;i<12;i++)await Promise.resolve()};
(async()=>{
 vm.runInContext('pgButtons()',ctx);assert(get('#pg-execute').disabled);assert(get('#pg-prepare').disabled);
 render({environment:{ready:false,checks:[]}});assert(get('#pg-prepare').disabled);get('#pg-supported').checked=true;vm.runInContext('pgButtons()',ctx);assert(!get('#pg-prepare').disabled);
 render(ready);assert(get('#pg-execute').disabled,'can_execute absent never enables execution');assert(!get('#pg-plan').disabled);assert(get('#pg-support-label').hidden);
 render({...ready,can_execute:true});assert(!get('#pg-execute').disabled);assert.equal(posts.length,0,'polling never commands motion');
 render({...ready,can_execute:true,busy:true});assert(get('#pg-execute').disabled);assert(get('#pg-speed').disabled);
 assert(!get('#pg-interrupt').disabled,'running stage must be interruptible');
 render({...ready,can_execute:true,plan:{validated:false}});assert(get('#pg-execute').disabled);
 render({...ready,busy:true,progress:{label:'规划',elapsed_s:100,stage_elapsed_s:8,stage_status:'running',estimate_s:300,estimate_source:'粗估基线',next_label:'校验产物'}});
 assert(get('#pg-timing').textContent.includes('本段已用 8 秒'));
 assert(get('#pg-timing').textContent.includes('本次总用时 100 秒'));
 assert(get('#pg-timing').textContent.includes('预计还需 200 秒'));
 assert(get('#pg-timing').textContent.includes('下一步：校验产物'));
 assert(get('#pg-timing').textContent.includes('粗估基线'));
 assert(get('#pg-progress').value>30 && get('#pg-progress').value<34);
 render({...ready,busy:true,progress:{elapsed_s:400,estimate_s:300}});
 assert.equal(get('#pg-progress').value,95);assert(get('#pg-timing').textContent.includes('已超过预估'));
 assert.equal(vm.runInContext("pgMediaUrl('https://elsewhere.example/frame.jpg')",ctx),null);
 assert.equal(vm.runInContext("pgMediaUrl('javascript:alert(1)')",ctx),null);
 render({...ready,can_execute:true,plan:{validated:true,first_frame_url:'/api/playground/frame.jpg'},videos:[{label:'本轮真机',url:'/api/real.mp4'}]});
 assert.equal(get('#pg-sim').src,'http://localhost:8119/api/playground/frame.jpg');assert(get('#pg-sim').hidden,'not evidence until load event');get('#pg-sim').onload();assert(!get('#pg-sim').hidden);
 assert.equal(get('#pg-videos').children.length,1);
 render({...ready,can_execute:true});assert(get('#pg-sim').hidden);assert.equal(get('#pg-sim').src,undefined,'new scene without frame cannot show old frame');
 const action=vm.runInContext("pgAction('execute')",ctx);assert(get('#pg-execute').disabled);await settle();
 assert.equal(posts.length,1);assert.equal(posts[0].payload.speed_scale,.3);assert.equal(posts[0].payload.action,'execute');
 await vm.runInContext("pgAction('execute')",ctx);assert.equal(posts.length,1,'no duplicate request while awaiting authoritative state');
 requests[0].resolve({ok:true,json:async()=>({...ready,busy:true,can_execute:false})});await action;assert(get('#pg-execute').disabled);
 // A read begun before a click must not re-enable execution with pre-action state.
 render({...ready,can_execute:true});const oldRead=vm.runInContext('pgRefresh()',ctx);const secondAction=vm.runInContext("pgAction('execute')",ctx);await settle();
 requests[1].resolve({ok:true,json:async()=>({...ready,can_execute:true})});await oldRead;await secondAction;assert(get('#pg-execute').disabled);
 // Only ready cameras poll; one in-flight frame per camera, and hiding cancels it.
 render({...ready,cameras:{base_0:{ready:true}}});assert.equal(requests[2].url,'/api/playground/camera/base_0/frame.jpg');
 vm.runInContext('pgUpdateCameras()',ctx);assert.equal(requests.length,3);
 requests[2].resolve({ok:true,blob:async()=>({})});await settle();assert(!get('#pg-base_0').hidden);assert.equal([...timers.values()][0].ms,33);
 const next=[...timers.values()][0];timers.clear();next.fn();assert.equal(requests.length,4);
 ctx.document.hidden=true;events.visibilitychange();assert(requests[3].options.signal.aborted);await settle();assert(get('#pg-base_0').hidden);assert.equal(timers.size,0);
 render({...ready,status:'failed',plan:{validated:false}});assert(!get('#pg-interrupt').disabled);
 const interrupt=vm.runInContext("pgAction('interrupt')",ctx);await settle();
 assert.equal(posts.at(-1).payload.action,'interrupt');assert.equal(Object.keys(posts.at(-1).payload).length,1);
 requests.at(-1).resolve({ok:true,json:async()=>({...ready,plan:{validated:false}})});await interrupt;
 assert.equal((html.match(/<button id="pg-/g)||[]).length,4);
 console.log('PASS Playground gates, explicit execution, speed, stale-state rejection, media identity, video rendering and bounded camera lifecycle');
})().catch(e=>{console.error(e);process.exitCode=1});
