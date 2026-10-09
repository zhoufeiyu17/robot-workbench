// CPU-only rendering checks; no browser or remote collection.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const html = fs.readFileSync(__dirname + '/index.html', 'utf8');
const nodes = new Map();
function node() { return {textContent:'', hidden:false, children:[], style:{}, append(n){this.children.push(n)}, replaceChildren(){this.children=[]}, removeAttribute(k){delete this[k]}}; }
const context = {performance:{now:()=>0}, setInterval(){}, document:{createElement:node}, $:id=>{if(!nodes.has(id))nodes.set(id,node());return nodes.get(id)}, text:(id,s)=>context.$('#'+id).textContent=s, tag(){}};
vm.createContext(context);
vm.runInContext(html.slice(html.indexOf('function runtimeStageLabel'), html.indexOf('let initializationSnapshot')) + html.slice(html.indexOf('function secondsLabel'),html.indexOf('function updateInitialization')),context);
function render(status, stages){context.job={directory:'fixture',runtime:{status,pipeline_finished:status!=='running',elapsed_s:700,stages}};vm.runInContext('updateRuntime(job)',context);}
const stages=[{key:'observe',label:'观测',status:'completed',elapsed_s:70,estimate_s:73,basis:'历史实测'}, {key:'replay',label:'双侧执行与录制',status:'running',elapsed_s:600,estimate_s:383,basis:'历史实测'}, {key:'verify',label:'校验',status:'pending',elapsed_s:null,estimate_s:3,basis:'粗估'}];
render('running',stages);
assert(context.$('#runtime-bar').value<100);
assert(context.$('#runtime-current').textContent.includes('超出预估'));
assert(context.$('#runtime-time').textContent.includes('当前阶段已用 10 分 0 秒'));
assert(context.$('#runtime-next').textContent.includes('校验'));
render('failed',stages.map(s=>({...s,status:s.status==='running'?'failed':s.status})));
assert(context.$('#runtime-bar').value<100);
assert(context.$('#runtime-time').textContent.includes('计时已停止'));
render('completed',stages.map(s=>({...s,status:'completed'})));
assert.equal(context.$('#runtime-bar').value,100);
assert(context.$('#runtime-next').textContent.includes('回放对比'));
console.log('Runtime UI overrun, failure, completion and next-stage checks: PASS');
