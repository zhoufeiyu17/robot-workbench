import {affectedSteps,tracePaths,traceUrl,sharedState} from './traceability.mjs?v=trace1';

export const states = {
  baseline: {label:'已有基线', color:'#708299'},
  planned: {label:'待开发', color:'#8a73b0'},
  active: {label:'开发中', color:'#356cce'},
  blocked: {label:'阻塞', color:'#bf4646'},
  implemented: {label:'已实现 · 待验收', color:'#b77816'},
  verified: {label:'已通过所列验证', color:'#208265'},
  stale: {label:'源码变化 · 待重验', color:'#cc652e'}
};
const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const time = value => value ? new Date(value).toLocaleString('zh-CN',{hour12:false}) : '尚未记录';
export function effectiveState(row) {
  if(row.source?.exists===false&&row.state!=='planned')return 'blocked';
  return row.state==='verified' && (row.verified_fingerprint!==row.fingerprint||row.evidence?.matches_file===false) ? 'stale' : row.state;
}
export function counts(rows) {
  const result={};
  for(const row of rows) {const state=effectiveState(row);result[state]=(result[state]||0)+1;}
  return result;
}
export function aggregate(rows) {
  const totals=counts(rows);
  return ['blocked','active','stale','implemented','planned','baseline','verified'].find(s=>totals[s]) || 'planned';
}
export function route(hash) {
  const params=new URLSearchParams(hash.replace(/^#/,''));
  return {level:['modules','domains','files'].includes(params.get('level'))?params.get('level'):'modules',
    module:params.get('module')||'',domain:params.get('domain')||'',node:params.get('node')||''};
}
export function url(values) {
  return '#'+new URLSearchParams(Object.entries(values).filter(([,value])=>value)).toString();
}
const components = catalog => [...catalog.components,...catalog.supporting];
const filePaths = domains => [...new Set(domains.flatMap(d=>d.files))];
export function nodesFor(catalog,data,view) {
  const domains=data.domains.filter(d=>(!view.module||d.module===view.module)&&(!view.domain||d.id===view.domain));
  if(view.level==='modules') return components(catalog).filter(m=>!view.module||m.id===view.module).map(m=>{
    const children=data.domains.filter(d=>d.module===m.id);
    const state=aggregate(children),total=counts(children)[state]||0;
    const label=total===children.length?states[state].label:`${states[state].label} · ${total}/${children.length} 子域`;
    return {...m,id:m.id,title:m.name,state,state_label:label,children,
      paths:filePaths(children),subtitle:`${children.length} 个职责域 · ${filePaths(children).length} 个文件`};
  });
  if(view.level==='domains') return domains.map(d=>({...d,title:d.name,children:d.files.map(p=>data.files[p]),paths:d.files,
    subtitle:`${d.phase} · ${d.files.length} 个文件`}));
  return filePaths(domains).sort().map(path=>({...data.files[path],id:path,title:path.split('/').at(-1),paths:[path],children:[],
    subtitle:path.slice(0,path.lastIndexOf('/'))}));
}
export function edgesFor(catalog,data,view,nodes,selected) {
  const visible=new Set(nodes.map(n=>n.id)),edges=new Map();
  const add=(from,to)=>{if(from!==to&&visible.has(from)&&visible.has(to))edges.set(from+'>'+to,{from,to});};
  if(view.level==='modules') {
    for(const contract of catalog.interfaces) for(const from of contract.producers||[contract.producer])
      for(const to of contract.consumers||[contract.consumer]) add(from,to);
  } else if(view.level==='files') {
    for(const node of nodes) for(const path of data.files[node.id].source.imports) add(node.id,path);
  } else {
    for(const from of nodes) for(const path of from.files) for(const imported of data.files[path].source.imports)
      for(const to of nodes) if(to.files.includes(imported)&&!to.files.includes(path))add(from.id,to.id);
  }
  // Show the selected node's neighbours so dense import graphs remain readable.
  return [...edges.values()].filter(e=>e.from===selected||e.to===selected);
}
export function graphSvg(nodes,edges,selected) {
  if(!nodes.length)return '<p class="empty">当前筛选下没有节点</p>';
  const width=900,columns=3,cellWidth=300,cellHeight=150,height=Math.ceil(nodes.length/columns)*cellHeight+16;
  const positions=new Map(nodes.map((node,i)=>[node.id,{x:(i%columns)*cellWidth+12,y:Math.floor(i/columns)*cellHeight+12}]));
  let svg=`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${width} ${height}" role="group" aria-label="可选择的开发状态节点" font-family="system-ui, PingFang SC, sans-serif"><defs><marker id="map-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0 L8 4 L0 8" fill="#8b9fb9"/></marker></defs>`;
  for(const edge of edges) {
    const a=positions.get(edge.from),b=positions.get(edge.to);
    const upward=b.y<a.y,ax=a.x+138,ay=upward?a.y:a.y+112,bx=b.x+138,by=b.y<=a.y?b.y+112:b.y;
    svg+=`<path d="M${ax} ${ay} V${ay+(upward?-13:13)} H${bx} V${by}" fill="none" stroke="#9aadc5" stroke-width="1.3" marker-end="url(#map-arrow)"/>`;
  }
  for(const node of nodes) {
    const {x,y}=positions.get(node.id),state=states[effectiveState(node)],title=node.title,label=node.state_label||state.label;
    // Long file names wrap at underscores without losing the full title in details.
    const split=title.length>29?Math.max(title.lastIndexOf('_',28)+1,24):title.length;
    const lines=[title.slice(0,split),title.slice(split)];
    svg+=`<g data-node="${esc(node.id)}" tabindex="0" role="button" aria-label="${esc(title+' '+label)}" aria-pressed="${node.id===selected}"><title>${esc(title)}</title><rect x="${x}" y="${y}" width="276" height="112" rx="9" fill="${node.id===selected?'#eef4ff':'#fff'}" stroke="${node.id===selected?'#3466ba':'#d7e0ed'}" stroke-width="${node.id===selected?2:1}"/><rect x="${x}" y="${y}" width="5" height="112" rx="2" fill="${state.color}"/><text x="${x+16}" y="${y+25}" font-size="${title.includes('.')?12:15}" fill="#243b56">${esc(lines[0])}</text>${lines[1]?`<text x="${x+16}" y="${y+42}" font-size="12" fill="#243b56">${esc(lines[1])}</text>`:''}<text x="${x+16}" y="${y+62}" font-size="10" fill="#6f829a">${esc(node.subtitle.length>39?node.subtitle.slice(0,36)+'…':node.subtitle)}</text><circle cx="${x+20}" cy="${y+87}" r="3" fill="${state.color}"/><text x="${x+30}" y="${y+91}" font-size="11" fill="${state.color}">${esc(label)}</text></g>`;
  }
  return svg+'</svg>';
}

export async function readSnapshot(fetcher) {
  const response=await fetcher('development-map.json',{cache:'no-store'});
  if(!response.ok)throw Error(`HTTP ${response.status}`);
  const data=await response.json();
  if(data.schema_version!==1||!Array.isArray(data.domains)||!data.files||!Number.isInteger(data.revision))throw Error('开发快照格式不兼容');
  return data;
}

export async function bootMap(doc=document,win=window,fetcher=fetch) {
  const get=id=>doc.getElementById(id);
  let catalog,data,view=route(win.location.hash),busy=false;
  const badge=row=>{const s=states[effectiveState(row)];return `<span class="map-status" style="color:${s.color};border-color:${s.color}">${esc(row.state_label||s.label)}</span>`;};
  function navigate(next) {win.location.hash=url(next);}
  function renderDetail(node) {
    if(!node){get('map-detail').innerHTML='<h2>选择一个节点</h2><p>图中颜色表示本轮开发状态，详情展示证据、下一步和源码。</p>';return;}
    let body=`${badge(node)}<h2>${esc(node.title)}</h2>`;
    if(view.level==='modules') {
      body+=`<p>${esc(node.current||node.responsibility)}</p><div class="actions"><a href="${url({level:'domains',module:node.id})}">进入职责域 →</a><a href="${url({level:'files',module:node.id})}">查看源码文件</a></div><p>模块状态按子域汇总。文件数已去重。</p>`;
    } else {
      body+=`<p>${esc(node.note)}</p><h3>下一步</h3><p>${esc(node.next||'等待下一阶段安排')}</p><dl><dt>负责</dt><dd>${esc(node.owner||'待分配')}</dd><dt>状态更新</dt><dd>${time(node.updated_at)}</dd></dl>`;
      if(view.level==='domains')body+=`<div class="actions"><a href="${url({level:'files',module:node.module,domain:node.id})}">进入源码文件 →</a></div><p>阶段 ${node.phase}。子域状态由目标验收结果维护，文件状态分别记录。</p>`;
      if(node.evidence)body+=`<h3>验证回执</h3><code>${esc(node.evidence.path)}</code><p>证据 SHA ${esc(node.evidence.sha256.slice(0,16))}</p>`;
      if(effectiveState(node)==='stale')body+='<p class="rule-note">源码或已映射依赖发生变化，历史通过记录待重验。</p>';
    }
    if(node.children.length)body+=`<div class="map-counts">${Object.entries(counts(node.children)).map(([state,count])=>`<span style="color:${states[state].color}">${states[state].label} ${count}</span>`).join('')}</div>`;
    if(view.level==='files') {
      const source=node.source,members=data.domains.filter(d=>d.files.includes(node.id));
      body+=`<h3>源码身份</h3><code>${esc(node.id)}</code><p>${source.exists?'文件存在':'文件缺失'} · ${node.id.endsWith('.py')?'Python 模块':'配套源码'}</p><code>${esc(source.sha256||'无摘要')}</code><h3>所属职责域</h3>${members.map(d=>`<a class="map-link" href="${url({level:'domains',module:d.module,node:d.id})}">${esc(d.name)}</a>`).join('')}<h3>历史部署比对</h3><p>${time(data.deployment_snapshot.checked_at)}</p>${source.deployed_snapshots.length?source.deployed_snapshots.map(d=>`<p>${esc(d.scope)} · ${d.matches_local?'与快照同源码':'与快照存在差异'}<br><code>${d.sha256.slice(0,16)}</code></p>`).join(''):'<p>该文件没有已登记的部署源码快照。</p>'}`;
      const importedBy=Object.keys(data.files).filter(p=>data.files[p].source.imports.includes(node.id));
      for(const [label,paths]of [['引用的已映射文件',source.imports],['引用此文件',importedBy]])body+=`<h3>${label} · ${paths.length}</h3>${paths.map(p=>`<a class="map-link" href="${url({level:'files',node:p})}">${esc(p)}</a>`).join('')||'<p>无已识别静态导入</p>'}`;
    } else {
      const module=view.level==='modules'?node.id:node.module;
      body+=`<h3>关联用户用例</h3><p>${catalog.use_cases.filter(c=>c.modules.includes(module)).map(c=>esc(c.id+' '+c.title)).join('<br>')}</p><a href="./#usecases" target="_top">查看用例流程</a><h3>关联接口</h3><p>${catalog.interfaces.filter(i=>[...(i.producers||[i.producer]),...(i.consumers||[i.consumer])].includes(module)).map(i=>esc(i.id+' '+i.name)).join('<br>')}</p><a href="./#interfaces" target="_top">查看接口契约</a>`;
    }
    const paths=new Set(node.paths);
    const extensions=(catalog.model_extensions||[]).filter(plan=>node.id===plan.module||node.module===plan.module||plan.steps.some(step=>step.files.some(file=>paths.has(file))));
    if(extensions.length)body+='<h3>已登记的模型扩展</h3>'+extensions.map(plan=>`<p>${esc(plan.title)} · 设计审阅中</p><a class="map-link" href="./#models" target="_top">查看WAM任务图、接口差异与验收用例 →</a>`).join('');
    const related=catalog.shared_steps.filter(sc=>tracePaths(catalog,{id:sc.id,sc_refs:[sc.id],bindings:[]}).some(p=>p.binding.files.some(file=>paths.has(file))));
    body+='<h3>支撑的共享子流程</h3>'+related.map(sc=>`<a class="map-link" href="./${traceUrl({sc:sc.id})}" target="_top">${sc.id} ${esc(sc.name)} · ${states[sharedState(sc)]?.label||sharedState(sc)}</a>`).join('');
    if(view.level==='files') {
      const impacted=affectedSteps(catalog,node.id);
      body+=`<h3>关联用例步骤 · ${impacted.length}</h3><a class="map-link" href="./${traceUrl({file:node.id})}" target="_top">查看此文件的全部行为追溯 →</a>`;
    }
    get('map-detail').innerHTML=body;
  }
  function render() {
    if(!catalog||!data)return;
    if(!components(catalog).some(m=>m.id===view.module))view.module='';
    if(!data.domains.some(d=>d.id===view.domain&&(!view.module||d.module===view.module)))view.domain='';
    get('map-module').value=view.module;
    get('map-levels').querySelectorAll('button').forEach(b=>{b.classList.toggle('active',b.dataset.level===view.level);b.setAttribute('aria-pressed',String(b.dataset.level===view.level));});
    let nodes=nodesFor(catalog,data,view);
    const query=get('map-search').value.toLowerCase().trim(),filter=get('map-status').value;
    nodes=nodes.filter(n=>(!filter||effectiveState(n)===filter||n.children.some(c=>effectiveState(c)===filter))&&(!query||[n.title,n.id,n.note,n.next,...n.paths].join(' ').toLowerCase().includes(query)));
    const selected=nodes.find(n=>n.id===view.node)||nodes[0];
    const scrollTop=get('map-graph').scrollTop,scrollLeft=get('map-graph').scrollLeft;
    get('map-graph').innerHTML=graphSvg(nodes,edgesFor(catalog,data,view,nodes,selected?.id),selected?.id);
    get('map-graph').scrollTop=scrollTop;get('map-graph').scrollLeft=scrollLeft;
    const svg=get('map-graph').querySelector('svg');if(svg)svg.style.width=(Number(get('map-zoom').value)*100)+'%';
    const labels={modules:'功能模块',domains:'职责子域',files:'源码文件'};
    get('map-title').textContent=labels[view.level];
    get('map-edge-note').textContent=view.level==='modules'?'连线显示选中模块关联的设计接口，箭头沿生产者 → 消费者。选择节点后可从详情进入下一层。':'连线显示选中节点的已识别 Python 导入关系，箭头沿引用方 → 被引用方。完整跨范围依赖见文件详情。';
    const module=components(catalog).find(m=>m.id===view.module),domain=data.domains.find(d=>d.id===view.domain);
    get('map-breadcrumb').innerHTML=`<a href="#level=modules">全部功能模块</a>${module?`<span>›</span><a href="${url({level:'domains',module:module.id})}">${esc(module.name)}</a>`:''}${domain?`<span>›</span><a href="${url({level:'files',module:domain.module,domain:domain.id})}">${esc(domain.name)}</a>`:''}<span>› ${labels[view.level]}</span>`;
    get('map-summary').textContent=`显示 ${nodes.length} 个节点 · ${filePaths(nodes.map(n=>({files:n.paths}))).length} 个独立文件 · 全图 ${data.domains.length} 个职责域 / ${Object.keys(data.files).length} 个文件${data.unmapped_files.length?' · 待归属 '+data.unmapped_files.join(', '):''}`;
    get('map-revision').textContent=`已发布快照 r${data.revision} · 更新于 ${time(data.updated_at)}`;
    get('map-scope').textContent=data.scope;
    get('map-changes').innerHTML=[...data.changes].reverse().slice(0,8).map(c=>`<div class="map-change"><span>${time(c.at)}</span><span style="color:${states[c.state].color}">${states[c.state].label}</span><span>${esc(c.note)}<small>${c.ids.map(esc).join(' · ')}</small></span></div>`).join('');
    renderDetail(selected);
  }
  async function refresh() {
    if(busy)return;
    busy=true;
    try {
      const next=await readSnapshot(fetcher);
      if(next.architecture_revision!==catalog.revision){
        const response=await fetcher('catalog.json',{cache:'no-store'});
        if(!response.ok)throw Error(`架构目录 HTTP ${response.status}`);
        const nextCatalog=await response.json();
        if(nextCatalog.revision!==next.architecture_revision)throw Error('架构与开发快照版本不同，请稍后刷新');
        catalog=nextCatalog;
        get('map-module').innerHTML='<option value="">全部模块</option>'+components(catalog).map(m=>`<option value="${m.id}">${esc(m.name)}</option>`).join('');
      }
      if(!data||data.revision!==next.revision){data=next;render();}
      get('map-error').hidden=true;
      get('map-sync').textContent=`读取成功 ${new Date().toLocaleTimeString('zh-CN')} · 每 15 秒刷新`;
    } catch(error) {
      get('map-error').hidden=false;
      get('map-error').textContent=`快照读取失败 · ${error.message}。${data?'正在显示上次成功读取的快照。':'可点击立即刷新重试。'}`;
      get('map-sync').textContent='更新失败，当前显示可能滞后';
    } finally {busy=false;}
  }
  try {
    const response=await fetcher('catalog.json',{cache:'no-store'});if(!response.ok)throw Error(`HTTP ${response.status}`);catalog=await response.json();
  } catch(error) {get('map-error').hidden=false;get('map-error').textContent=`架构目录读取失败 · ${error.message}，请重新加载页面。`;return;}
  get('map-module').innerHTML='<option value="">全部模块</option>'+components(catalog).map(m=>`<option value="${m.id}">${esc(m.name)}</option>`).join('');
  get('map-status').innerHTML='<option value="">全部状态</option>'+Object.entries(states).map(([id,s])=>`<option value="${id}">${s.label}</option>`).join('');
  get('map-legend').innerHTML=Object.values(states).map(s=>`<span><i style="background:${s.color}"></i>${s.label}</span>`).join('');
  get('map-levels').onclick=e=>{const b=e.target.closest('[data-level]');if(b)navigate({...view,level:b.dataset.level,node:'',domain:b.dataset.level==='modules'?'':view.domain});};
  get('map-module').onchange=()=>navigate({level:view.level,module:get('map-module').value});
  get('map-status').onchange=render;get('map-search').oninput=render;get('map-zoom').onchange=render;
  function select(event) {const node=event.target.closest('[data-node]');if(node)navigate({...view,node:node.dataset.node});}
  get('map-graph').onclick=select;
  get('map-graph').onkeydown=event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();select(event);}};
  get('map-refresh').onclick=refresh;
  win.addEventListener('hashchange',()=>{view=route(win.location.hash);render();});
  await refresh();
  const timer=win.setInterval(()=>{if(!doc.hidden)refresh();},15000);
  win.addEventListener('pagehide',()=>win.clearInterval(timer),{once:true});
}
if(typeof document!=='undefined')bootMap();
