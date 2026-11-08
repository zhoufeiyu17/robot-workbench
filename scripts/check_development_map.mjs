import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawnSync} from 'node:child_process';
import {states,effectiveState,counts,aggregate,route,url,nodesFor,edgesFor,graphSvg,readSnapshot,bootMap} from '../web/architecture/development-map.mjs';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const catalog=JSON.parse(fs.readFileSync(path.join(root,'web/architecture/catalog.json')));
const data=JSON.parse(fs.readFileSync(path.join(root,'web/architecture/development-map.json')));
const modules=[...catalog.components,...catalog.supporting];
const ids=new Set(data.domains.map(d=>d.id));
assert.equal(ids.size,data.domains.length);
assert.equal(data.schema_version,1);
const mapped=new Set(data.domains.flatMap(d=>d.files));
assert.deepEqual([...mapped].sort(),Object.keys(data.files).sort());
for(const file of fs.readdirSync(path.join(root,'robot_workbench')).filter(f=>f.endsWith('.py')))assert(mapped.has('robot_workbench/'+file));
for(const domain of data.domains) {
  assert(modules.some(m=>m.id===domain.module));
  assert(catalog.phases.some(p=>p.id===domain.phase));
  assert(domain.files.length&&domain.note&&domain.next&&domain.owner);
}
for(const [file,row]of Object.entries(data.files)) {
  assert(fs.existsSync(path.join(root,file)),file);
  assert(row.source.exists);
  assert(/^[a-f0-9]{64}$/.test(row.source.sha256));
  for(const imported of row.source.imports)assert(mapped.has(imported));
}
assert(data.files['robot_workbench/recording_process.py'].source.imports.includes('rule_based_workbench/rgbd_segments.py'));
for(const row of [...data.domains,...Object.values(data.files)]) {
  assert(states[row.state]&&row.owner&&row.updated_at);
  assert(/^[a-f0-9]{64}$/.test(row.fingerprint));
  if(row.state==='verified')assert(row.evidence?.path&&row.verified_fingerprint);
}
assert.deepEqual(route('#level=files&module=guard'),{level:'files',module:'guard',domain:'',node:''});
assert.equal(route('#level=class').level,'modules');
assert.equal(route(url({level:'files',node:'robot_workbench/runtime.py'})).node,'robot_workbench/runtime.py');
assert.equal(effectiveState({state:'verified',verified_fingerprint:'a',fingerprint:'b'}),'stale');
assert.equal(effectiveState({state:'verified',verified_fingerprint:'a',fingerprint:'a',evidence:{matches_file:false}}),'stale');
assert.equal(effectiveState({state:'active',source:{exists:false}}),'blocked');
assert.equal(aggregate([{state:'verified'},{state:'blocked'}]),'blocked');
assert.deepEqual(counts([{state:'active'},{state:'planned'}]),{active:1,planned:1});
for(const level of ['modules','domains','files']) {
  const view={level,module:'',domain:''},nodes=nodesFor(catalog,data,view);
  assert(nodes.length);
  const svg=graphSvg(nodes,edgesFor(catalog,data,view,nodes,nodes[0].id),nodes[0].id);
  for(const node of nodes)assert(svg.includes(`data-node="${node.id}"`));
  assert(svg.includes('tabindex="0"'));
  assert.equal(new Set(nodes.map(n=>n.id)).size,nodes.length);
}
const guardFiles=nodesFor(catalog,data,{level:'files',module:'guard'});
assert.equal(guardFiles.filter(f=>f.id==='robot_workbench/runtime.py').length,1);
assert(graphSvg([{id:'x',title:'<script>',subtitle:'',state:'planned'}],[],'x').includes('&lt;script&gt;'));
await assert.rejects(readSnapshot(async()=>({ok:false,status:503})),/503/);
await assert.rejects(readSnapshot(async()=>({ok:true,json:async()=>({schema_version:2})})),/格式/);

// Small DOM substitute exercises normal UI entry points and polling without a browser.
const html=fs.readFileSync(path.join(root,'web/architecture/development-map.html'),'utf8');
const js=fs.readFileSync(path.join(root,'web/architecture/development-map.mjs'),'utf8');
const elements=new Map();
for(const match of html.matchAll(/\bid="([^"]+)"/g)) {
  assert(!elements.has(match[1]));
  elements.set(match[1],{innerHTML:'',textContent:'',hidden:false,value:'',scrollTop:0,scrollLeft:0,
    querySelector:()=>({style:{}}),querySelectorAll:()=>[]});
}
for(const match of js.matchAll(/(?<!\.)get\('([^']+)'\)/g))assert(elements.has(match[1]),match[1]);
const levelButtons=['modules','domains','files'].map(level=>({dataset:{level},classList:{toggle(){}},setAttribute(){}}));
elements.get('map-levels').querySelectorAll=()=>levelButtons;
elements.get('map-zoom').value='1';
let hash='#level=modules',poll,fail=false,live=structuredClone(data);
const listeners={};
const win={location:{get hash(){return hash;},set hash(value){hash=value;listeners.hashchange?.();}},
  addEventListener(name,fn){listeners[name]=fn;},setInterval(fn,ms){assert.equal(ms,15000);poll=fn;return 1;},clearInterval(){}};
const doc={getElementById:id=>elements.get(id),hidden:false};
const fetcher=async name=>({ok:!fail,status:fail?503:200,json:async()=>name==='catalog.json'?catalog:structuredClone(live)});
await bootMap(doc,win,fetcher);
assert(elements.get('map-graph').innerHTML.includes('data-node="guard"'));
elements.get('map-levels').onclick({target:{closest:()=>levelButtons[1]}});
assert(hash.includes('level=domains'));
assert(elements.get('map-graph').innerHTML.includes('guard-policy'));
elements.get('map-module').value='guard';elements.get('map-module').onchange();
assert(hash.includes('module=guard'));
elements.get('map-levels').onclick({target:{closest:()=>levelButtons[2]}});
elements.get('map-search').value='execution_settings.py';elements.get('map-search').oninput();
assert(elements.get('map-summary').textContent.startsWith('显示 1 个节点'));
elements.get('map-graph').onclick({target:{closest:()=>({dataset:{node:'robot_workbench/execution_settings.py'}})}});
assert(elements.get('map-detail').innerHTML.includes('源码身份'));
let prevented=false;
elements.get('map-graph').onkeydown({key:'Enter',preventDefault(){prevented=true;},target:{closest:()=>({dataset:{node:'robot_workbench/execution_settings.py'}})}});
assert(prevented);
live.files['robot_workbench/execution_settings.py'].state='blocked';live.revision++;
poll();await new Promise(resolve=>setImmediate(resolve));
assert(elements.get('map-revision').textContent.includes('r'+live.revision));
assert(elements.get('map-detail').innerHTML.includes('阻塞'));
assert(hash.includes('execution_settings.py'));
fail=true;poll();await new Promise(resolve=>setImmediate(resolve));
assert.equal(elements.get('map-error').hidden,false);
assert(elements.get('map-detail').innerHTML.includes('阻塞'));
fail=false;await elements.get('map-refresh').onclick();assert.equal(elements.get('map-error').hidden,true);
elements.get('map-status').value='active';elements.get('map-status').onchange();
assert(elements.get('map-graph').innerHTML.includes('没有节点'));
assert(!js.includes('/api/')&&!js.includes('/prepare')&&!js.includes('/infer'));
const architectureHtml=fs.readFileSync(path.join(root,'web/architecture/index.html'),'utf8');
assert(architectureHtml.includes('data-view="progress">开发 Map · 实施进度</button>'));
assert(architectureHtml.match(/id="view-progress"[\s\S]*?<iframe[^>]+src="development-map.html\?embedded=1"[^>]+title=/));
assert(html.includes("classList.add('map-embedded')"));
assert(html.includes('href="./#progress" target="_top"'));

const python=spawnSync('python3',['-B','-c',String.raw`
import importlib.util,json,tempfile
from pathlib import Path
spec=importlib.util.spec_from_file_location('devmap','scripts/development_map.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
with tempfile.TemporaryDirectory() as tmp:
 root=Path(tmp);(root/'robot_workbench').mkdir()
 (root/'robot_workbench/a.py').write_text('from .b import value\n')
 (root/'robot_workbench/b.py').write_text('value=1\n')
 paths=['robot_workbench/a.py','robot_workbench/b.py']
 def facts():return m.source_facts(root,paths,{'root':'snapshot'})
 f=facts();assert f[paths[0]]['imports']==[paths[1]]
 data={'files':{p:{'source':f[p]} for p in paths},'domains':[],'changes':[]}
 first=m.fingerprint(data['files'],[paths[0]])
 data['files'][paths[0]]['fingerprint']=first
 (root/'receipt.json').write_text('{"passed":true}')
 m.update(root,data,'file',[paths[0]],'verified','passed','','owner','receipt.json')
 assert data['files'][paths[0]]['verified_fingerprint']==first
 (root/paths[1]).write_text('value=2\n')
 f=facts()
 for p in paths:data['files'][p]['source']=f[p]
 assert m.fingerprint(data['files'],[paths[0]])!=first
 try:m.update(root,data,'file',[paths[0]],'verified','passed','','owner',None)
 except ValueError:pass
 else:raise AssertionError('verified without evidence')
 try:m.local_path(root,'../outside')
 except ValueError:pass
 else:raise AssertionError('path escape')
 m.atomic_json(root/'data.json',data);assert json.loads((root/'data.json').read_text())==data
 (root/paths[1]).unlink();assert not facts()[paths[1]]['exists']
 (root/'rule_based_workbench').mkdir()
 (root/'rule_based_workbench/rgbd_segments.py').write_text('value=1\n')
 (root/'robot_workbench/recorder.py').write_text('from rgbd_segments import value\n')
 alias_paths=['robot_workbench/recorder.py','rule_based_workbench/rgbd_segments.py']
 aliases=m.source_facts(root,alias_paths,{'root':'snapshot'})
 assert aliases[alias_paths[0]]['imports']==[alias_paths[1]]
 old=m.fingerprint({p:{'source':aliases[p]} for p in alias_paths},[alias_paths[0]])
 (root/'rule_based_workbench/rgbd_segments.py').write_text('value=2\n')
 aliases=m.source_facts(root,alias_paths,{'root':'snapshot'})
 assert old!=m.fingerprint({p:{'source':aliases[p]} for p in alias_paths},[alias_paths[0]])
print('PASS source inventory, relative imports, transitive validation invalidation, evidence requirement, path boundary and atomic writes')
`],{cwd:root,encoding:'utf8'});
assert.equal(python.status,0,python.stdout+python.stderr);
console.log(python.stdout.trim());
console.log(`PASS development Map: ${modules.length} modules, ${data.domains.length} domains, ${mapped.size} unique files; hierarchy, filters, keyboard, detail navigation, polling updates, error retention and recovery`);
