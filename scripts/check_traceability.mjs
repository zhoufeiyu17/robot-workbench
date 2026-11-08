import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';
import {tracePaths,affectedSteps,sharedUsages,selectTrace,traceSvg,initTraceability,sharedState,mapFileUrl} from '../web/architecture/traceability.mjs';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const catalog=JSON.parse(fs.readFileSync(path.join(root,'web/architecture/catalog.json')));
const map=JSON.parse(fs.readFileSync(path.join(root,'web/architecture/development-map.json')));
function validate(c) {
  assert.equal(c.schema_version,2);
  const shared=new Map(c.shared_steps.map(s=>[s.id,s]));
  const contracts=new Map(c.interfaces.map(i=>[i.id,i]));
  const components=new Set([...c.components,...c.supporting].map(m=>m.id));
  const bindings=c.traceability.bindings;
  const allIds=new Set(),used=new Set();
  function checkStep(s,localIds) {
    assert(!allIds.has(s.id),'duplicate stable step ID');allIds.add(s.id);
    assert(s.title||s.trigger);assert(s.inputs&&s.completion);
    assert(s.bindings.length||s.sc_refs.length,'unmapped behaviour '+s.id);
    if(s.next)assert(s.next==='complete'||localIds.has(s.next));
    if(s.loop_to)assert(localIds.has(s.loop_to)&&s.loop_condition);
    if(s.from_steps)assert(s.from_steps.length&&s.from_steps.every(id=>localIds.has(id)));
    for(const id of s.bindings){assert(bindings[id],id);used.add(id);}
    for(const id of s.sc_refs) {
      assert(shared.has(id),'unknown shared process');
      for(const child of s.sc_steps?.[id]||[])assert(shared.get(id).steps.some(row=>row.id===child));
    }
    assert(tracePaths(c,s).length,'step has no resolved path');
  }
  for(const u of c.use_cases) {
    const local=new Set(u.main_flow.map(s=>s.id));
    for(const s of [...u.main_flow,...u.alternatives])checkStep(s,local);
    const refs=[...new Set([...u.main_flow,...u.alternatives].flatMap(s=>s.sc_refs))].sort();
    assert.deepEqual(refs,u.shared_steps);
  }
  for(const sc of c.shared_steps) {
    assert(sc.inputs.length&&sc.preconditions.length&&sc.completion&&sc.cancel&&sc.retry);
    assert(components.has(sc.orchestrator));
    assert(sc.acceptance.length>=2&&sc.progress.owner&&sc.progress.note);
    const local=new Set(sc.steps.map(s=>s.id));
    for(const s of [...sc.steps,...sc.failures]) {
      checkStep({...s,inputs:s.inputs||sc.inputs.join('、'),completion:s.completion||s.resulting_state},local);
    }
    const expected=new Set([...sc.steps,...sc.failures].flatMap(s=>s.bindings));
    const actual=new Set(sc.participants.flatMap(p=>p.bindings));
    assert.deepEqual(actual,expected);
    for(const p of sc.participants)for(const id of p.bindings)assert.equal(bindings[id].component,p.component);
    assert(sharedUsages(c,sc.id).length,'unused SC');
    assert.equal(new Set(sc.acceptance.map(a=>a.id)).size,sc.acceptance.length);
    for(const a of sc.acceptance)assert(a.given&&a.when&&a.then);
    if(sc.progress.state==='verified')assert(sc.progress.evidence&&sc.progress.verified_fingerprint);
  }
  for(const [id,b] of Object.entries(bindings)) {
    assert(used.has(id),'unused implementation binding '+id);
    const contract=contracts.get(b.interface);assert(contract,'unknown interface');
    assert([...(contract.producers||[contract.producer]),...(contract.consumers||[contract.consumer])].includes(b.component),id+' wrong endpoint');
    assert(b.role&&['planned','partial','implemented'].includes(b.status));
    assert(b.files.length||b.status==='planned'&&b.gap,'missing implementation or explicit gap');
    if(b.status!=='implemented')assert(b.gap,'pending behaviour lacks explanation');
    for(const file of b.files){assert(map.files[file],'unmapped source '+file);assert(fs.existsSync(path.join(root,file)));}
  }
  return allIds.size;
}
const count=validate(catalog);
const bad=structuredClone(catalog);bad.use_cases[0].main_flow[0].bindings=[];bad.use_cases[0].main_flow[0].sc_refs=[];
assert.throws(()=>validate(bad),/unmapped behaviour/);
const invalidEndpoint=structuredClone(catalog);invalidEndpoint.traceability.bindings['capture-request'].component='sdk';
assert.throws(()=>validate(invalidEndpoint));
const sample=selectTrace(catalog,'#trace?uc=UC05&step=UC05-S02');
assert.equal(sample.step.sc_refs[0],'SC05');
assert(sample.paths.some(p=>p.binding.id==='capture-guard'&&p.binding.files.includes('robot_workbench/execution_settings.py')));
assert.deepEqual(new Set(sample.paths.map(p=>p.binding.component)),new Set(['lifecycle','input','evidence','guard']));
const bindOnly=selectTrace(catalog,'#trace?uc=UC17&step=UC17-S02');
assert(bindOnly.paths.every(p=>p.origin==='SC05-S05'||p.origin==='SC05-E03'));
const file='robot_workbench/recording_process.py';
assert(affectedSteps(catalog,file).some(p=>p.uc.id==='UC05'&&p.step.id==='UC05-S02'));
assert(affectedSteps(catalog,file).some(p=>p.uc.id==='UC20'));
assert(!affectedSteps(catalog,'robot_workbench/gc_timing.py').length);
assert.equal(selectTrace(catalog,'#trace?sc=SC05&substep=SC05-S03').step.id,'SC05-S03');
assert.equal(selectTrace(catalog,'#trace?file=missing.py').step,null);
assert.equal(sharedState({progress:{state:'verified',fingerprint:'a',verified_fingerprint:'b',evidence:{matches_file:true}}}),'stale');
assert.equal(sharedState({progress:{state:'verified',fingerprint:'a',verified_fingerprint:'a',evidence:{matches_file:false}}}),'stale');
const svg=traceSvg(catalog,sample);
assert(svg.includes('execution_settings.py')&&svg.includes('SC05-S03')&&svg.includes('I11'));
const missing=structuredClone(sample);missing.paths[0].binding.files=[];
assert(traceSvg(catalog,missing).includes('目标实现待定位'));
assert(svg.includes(mapFileUrl(file)));

const html=fs.readFileSync(path.join(root,'web/architecture/index.html'),'utf8');
const js=fs.readFileSync(path.join(root,'web/architecture/traceability.mjs'),'utf8');
const elements=new Map([...html.matchAll(/\bid="([^"]+)"/g)].map(m=>[m[1],{innerHTML:'',textContent:'',value:''}]));
for(const m of js.matchAll(/(?<!\.)get\('([^']+)'\)/g))assert(elements.has(m[1]),m[1]);
let hash='#trace?uc=UC05&step=UC05-S02';const listeners=[];
const win={location:{get hash(){return hash;},set hash(v){hash=v;listeners.forEach(fn=>fn());}},addEventListener(name,fn){assert.equal(name,'hashchange');listeners.push(fn);}};
initTraceability(catalog,{get:id=>elements.get(id)},win);
assert(elements.get('trace-shared').innerHTML.includes('SC05-A04'));
elements.get('trace-sc').value='SC07';elements.get('trace-sc').onchange();
assert(elements.get('trace-detail').innerHTML.includes('SC07'));
assert(elements.get('trace-related').innerHTML.includes('UC09'));
win.location.hash='#trace?file='+encodeURIComponent(file);
assert(elements.get('trace-file').innerHTML.includes(file));
assert(elements.get('trace-related').innerHTML.includes('UC20'));
win.location.hash='#trace?file=missing.py';assert.equal(elements.get('trace-bindings').innerHTML,'');
elements.get('trace-uc').value='UC05';elements.get('trace-uc').onchange();
assert(elements.get('trace-detail').innerHTML.includes('UC05-S02'));
assert(html.includes('data-view="trace"'));
const mapJs=fs.readFileSync(path.join(root,'web/architecture/development-map.mjs'),'utf8');
assert(mapJs.includes('affectedSteps(catalog,node.id)')&&mapJs.includes('traceUrl({file:node.id})'));

const result=spawnSync('python3',['-B','-c',String.raw`
import importlib.util,json,tempfile
from pathlib import Path
s=importlib.util.spec_from_file_location('map','scripts/development_map.py');m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
with tempfile.TemporaryDirectory() as tmp:
 root=Path(tmp);(root/'web/architecture').mkdir(parents=True)
 row={'id':'SC01','steps':[{'bindings':['b']}],'failures':[],'acceptance':[{'id':'SC01-A01'},{'id':'SC01-A02'}],'progress':{'state':'planned'}}
 c={'revision':'test','shared_steps':[row],'interfaces':[],'traceability':{'bindings':{'b':{'files':[],'interface':'I01','status':'planned'}}}}
 data={'files':{},'changes':[]}
 try:m.update_shared(root,data,c,['SC01'],'verified','passed','','owner',None)
 except ValueError:pass
 else:raise AssertionError('pending implementation must prevent verification')
 c['traceability']['bindings']['b']['status']='implemented'
 f=m.shared_fingerprint(c,data,row)
 def receipt(cases,identity=f):
  (root/'receipt.json').write_text(json.dumps({'source_fingerprint':identity,'scope':'CPU integration','cases':cases}))
 receipt({'SC01-A01':'passed'})
 try:m.update_shared(root,data,c,['SC01'],'verified','passed','','owner','receipt.json')
 except ValueError:pass
 else:raise AssertionError('missing acceptance case')
 receipt({'SC01-A01':'passed','SC01-A02':'passed'},'wrong')
 try:m.update_shared(root,data,c,['SC01'],'verified','passed','','owner','receipt.json')
 except ValueError:pass
 else:raise AssertionError('wrong source identity')
 receipt({'SC01-A01':'passed','SC01-A02':'passed'})
 m.update_shared(root,data,c,['SC01'],'verified','passed','','owner','receipt.json')
 assert row['progress']['verified_fingerprint']==f
 row['steps'][0]['completion']='changed behaviour'
 m.refresh_shared(root,data,c)
 assert row['progress']['fingerprint']!=f
 (root/'receipt.json').write_text('{}');m.refresh_shared(root,data,c)
 assert not row['progress']['evidence']['matches_file']
print('PASS SC evidence identity, all-case coverage, pending implementation, changed behaviour and receipt invalidation')
`],{cwd:root,encoding:'utf8'});
assert.equal(result.status,0,result.stdout+result.stderr);console.log(result.stdout.trim());
console.log(`PASS traceability: ${catalog.use_cases.length} UCs, ${count} UC/SC steps, ${catalog.shared_steps.length} SCs, ${Object.keys(catalog.traceability.bindings).length} bindings; exact roles, reverse navigation and explicit gaps`);
