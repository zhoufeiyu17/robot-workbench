const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const modules = catalog => [...catalog.components,...catalog.supporting];
const allSteps = useCase => [...useCase.main_flow,...useCase.alternatives];
const bindingLabels={planned:'待实现',partial:'已有基础 · 待归并',implemented:'已实现'};
export const traceUrl = values => '#trace?'+new URLSearchParams(Object.entries(values).filter(([,v])=>v)).toString();
export const mapFileUrl = file => '#progress?file='+encodeURIComponent(file);

export function tracePaths(catalog, step) {
  const paths=(step.bindings||[]).map(id=>({origin:step.id,shared:step.id.startsWith('SC')?step.id.split('-')[0]:null,binding:catalog.traceability.bindings[id]}));
  for(const id of step.sc_refs||[]) {
    const shared=catalog.shared_steps.find(s=>s.id===id);
    const selected=step.sc_steps?.[id];
    const mains=shared.steps.filter(s=>!selected||selected.includes(s.id));
    const branches=shared.failures.filter(s=>s.from_steps.some(from=>mains.some(m=>m.id===from)));
    for(const child of [...mains,...branches])for(const binding of child.bindings)
      paths.push({origin:child.id,shared:id,binding:catalog.traceability.bindings[binding]});
  }
  return paths;
}

export function affectedSteps(catalog, file) {
  return catalog.use_cases.flatMap(uc=>allSteps(uc).filter(step=>tracePaths(catalog,step).some(p=>p.binding.files.includes(file))).map(step=>({uc,step})));
}

export function sharedUsages(catalog, id) {
  return catalog.use_cases.flatMap(uc=>allSteps(uc).filter(step=>step.sc_refs.includes(id)).map(step=>({uc,step})));
}

export function selectTrace(catalog, hash) {
  const query=new URLSearchParams(hash.split('?')[1]||'');
  const file=query.get('file')||'';
  const shared=catalog.shared_steps.find(s=>s.id===query.get('sc'));
  if(shared) {
    const child=[...shared.steps,...shared.failures].find(s=>s.id===query.get('substep'));
    const step=child||{id:shared.id,title:shared.name,sc_refs:[shared.id],bindings:[]};
    return {shared,step,file,paths:tracePaths(catalog,step),uc:null};
  }
  const matches=file?affectedSteps(catalog,file):[];
  if(file&&!matches.length)return {file,paths:[],step:null,uc:null,shared:null};
  const requested=catalog.use_cases.find(u=>u.id===query.get('uc'));
  const uc=requested||matches[0]?.uc||catalog.use_cases.find(u=>u.id==='UC05');
  const candidates=file?matches.filter(m=>m.uc.id===uc.id).map(m=>m.step):allSteps(uc);
  const step=candidates.find(s=>s.id===query.get('step'))||candidates.find(s=>s.id==='UC05-S02')||candidates[0];
  return {uc,step,shared:null,file,paths:step?tracePaths(catalog,step):[]};
}

function wrapped(text, width=24) {
  const lines=[];
  while(text.length>width) {
    const space=text.lastIndexOf(' ',width);
    const cut=space>width/2?space:width;
    lines.push(text.slice(0,cut));text=text.slice(cut).trimStart();
  }
  if(text)lines.push(text);
  return lines;
}

export function traceSvg(catalog, selection) {
  const unique=new Map(selection.paths.map(p=>[p.origin+'>'+p.binding.id,p]));
  const rows=[...unique.values()];
  if(!rows.length)return '<p class="empty">这个范围暂时没有步骤映射。请查看待实现说明或选择其他用例。</p>';
  const component=id=>modules(catalog).find(m=>m.id===id)?.name||id;
  const heights=rows.map(({binding:b})=>Math.max(110,45+b.files.length*22,50+wrapped(b.role,18).length*18));
  const height=80+heights.reduce((a,b)=>a+b,0);
  let svg=`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1140 ${height}" role="group" aria-label="步骤到接口、组件职责和源码文件的追溯图" font-family="system-ui, PingFang SC, sans-serif"><defs><marker id="trace-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0 L8 4 L0 8" fill="#829bb9"/></marker></defs><rect x="8" y="6" width="1120" height="45" rx="7" fill="#eaf1ff"/><text x="24" y="34" font-size="14" fill="#2d527e">${esc(selection.step.id+' · '+(selection.step.title||selection.step.trigger||''))}</text>`;
  let y=70;
  rows.forEach(({origin,shared,binding:b},index)=>{
    const h=heights[index],color=b.status==='planned'?'#9867aa':b.status==='implemented'?'#208265':'#a67929';
    svg+=`<rect x="8" y="${y}" width="1120" height="${h-10}" rx="8" fill="#fff" stroke="#dce4ef"/><text x="22" y="${y+26}" font-size="12" fill="#3565a5">${esc(origin)}</text><text x="22" y="${y+48}" font-size="10" fill="#75869c">${shared?'共享子流程步骤':'用例直接行为'}</text><text x="22" y="${y+71}" font-size="10" fill="${color}">${bindingLabels[b.status]}</text>`;
    const contract=catalog.interfaces.find(i=>i.id===b.interface);
    svg+=`<text x="215" y="${y+26}" font-size="13" fill="#385f90">${b.interface}</text>`;
    wrapped(contract.name,24).forEach((line,i)=>{svg+=`<text x="215" y="${y+48+i*17}" font-size="10" fill="#637b97">${esc(line)}</text>`;});
    svg+=`<text x="468" y="${y+26}" font-size="13" fill="#2b4c72">${esc(component(b.component))}</text>`;
    wrapped(b.role,18).forEach((line,i)=>{svg+=`<text x="468" y="${y+48+i*18}" font-size="11" fill="#637b97">${esc(line)}</text>`;});
    for(const [a,z]of [[173,204],[420,454],[699,738]])svg+=`<path d="M${a} ${y+38} H${z}" stroke="#829bb9" fill="none" marker-end="url(#trace-arrow)"/>`;
    if(!b.files.length)svg+=`<text x="752" y="${y+30}" font-size="12" fill="#9867aa">目标实现待定位</text>`;
    b.files.forEach((file,i)=>{svg+=`<a href="${mapFileUrl(file)}" aria-label="查看${esc(file)}"><text x="752" y="${y+30+i*22}" font-size="11" fill="#3565a5">${esc(file.split('/').at(-1))}</text></a>`;});
    y+=h;
  });
  return svg+'</svg>';
}

const list = items => `<ul>${(items||[]).map(text=>`<li>${esc(text)}</li>`).join('')}</ul>`;
const stateNames={planned:'待开发',active:'开发中',blocked:'阻塞',implemented:'已实现 · 待验收',verified:'已通过所列验收',stale:'变更后待重验'};
export function sharedState(shared) {
  const p=shared.progress;
  return p.state==='verified'&&(!p.evidence?.matches_file||p.verified_fingerprint!==p.fingerprint)?'stale':p.state;
}

export function initTraceability(catalog, {get}, win=window) {
  const name=id=>modules(catalog).find(m=>m.id===id)?.name||id;
  const useLink=(uc,step,file='')=>`<a class="case-chip" href="${traceUrl({uc:uc.id,step:step.id,file})}">${esc(uc.id+' / '+step.id+' '+(step.title||step.trigger))}</a>`;
  function render() {
    const selection=selectTrace(catalog,win.location.hash),{uc,shared,step,file,paths}=selection;
    get('trace-uc').value=uc?.id||'';get('trace-sc').value=shared?.id||'';
    const total=catalog.use_cases.reduce((n,u)=>n+allSteps(u).length,0);
    get('trace-summary').textContent=`${catalog.use_cases.length} 个 UC · ${total} 个主／异常步骤 · ${catalog.shared_steps.length} 个 SC · ${Object.keys(catalog.traceability.bindings).length} 项实现角色映射`;
    get('trace-file').innerHTML=file?`当前反向查询 <code>${esc(file)}</code> <a href="#trace">清除文件筛选</a>`:'选择用例步骤或共享子流程，沿接口定位实现。';
    const entries=shared?[...shared.steps,...shared.failures]:uc?allSteps(uc):[];
    get('trace-steps').innerHTML=entries.map(s=>{
      const href=shared?traceUrl({sc:shared.id,substep:s.id}):traceUrl({uc:uc.id,step:s.id,file});
      const matches=!file||tracePaths(catalog,s).some(p=>p.binding.files.includes(file));
      return `<a href="${href}" class="trace-step ${s.id===step?.id?'selected':''} ${matches?'':'outside-filter'}"><small>${s.id} · ${s.trigger?'异常分支':'主流程'}</small><span>${esc(s.title||s.trigger)}</span></a>`;
    }).join('');
    get('trace-related').innerHTML=file?`<h3>受影响的用例步骤</h3><div class="case-chips">${affectedSteps(catalog,file).map(({uc,step})=>useLink(uc,step,file)).join('')||'尚无映射'}</div>`:shared?`<h3>引用这个 SC 的具体步骤</h3><div class="case-chips">${sharedUsages(catalog,shared.id).map(({uc,step})=>useLink(uc,step)).join('')}</div>`:'';
    get('trace-graph').innerHTML=traceSvg(catalog,selection);
    if(!step){get('trace-detail').innerHTML='<p>该文件尚无行为映射，开发Map中的源码状态仍可独立查看。</p>';get('trace-shared').innerHTML='';get('trace-bindings').innerHTML='';return;}
    get('trace-detail').innerHTML=`<h2>${esc(step.id+' '+(step.title||step.trigger||shared?.name))}</h2>${step.condition?`<p class="rule-note">执行条件 · ${esc(step.condition)}</p>`:''}<p><b>输入</b> ${esc(step.inputs||shared?.inputs.join('、'))}</p><p><b>完成条件</b> ${esc(step.completion||step.resulting_state||shared?.completion||'按各共享子步骤及异常契约完成')}</p>${step.from_steps?`<p><b>触发位置</b> ${step.from_steps.map(esc).join('、')}</p><p><b>后续处理</b> ${esc(step.next_action)}</p>`:''}${step.next?`<p><b>后继</b> ${esc(step.next)}</p>`:''}${step.loop_to?`<p><b>循环</b> ${esc(step.loop_condition)} → ${step.loop_to}</p>`:''}<div class="case-chips">${(step.sc_refs||[]).map(id=>`<a class="case-chip" href="${traceUrl({sc:id})}">${id} ${esc(catalog.shared_steps.find(s=>s.id===id).name)}</a>`).join('')}</div>`;
    const selectedShared=shared||catalog.shared_steps.find(s=>s.id===step.sc_refs?.[0]);
    get('trace-shared').innerHTML=selectedShared?sharedHtml(selectedShared):'';
    const bindings=[...new Map(paths.map(p=>[p.binding.id,p.binding])).values()];
    get('trace-bindings').innerHTML=bindings.map(b=>`<tr><td><a href="#interfaces?contract=${b.interface}">${b.interface}</a><br><code>${b.id}</code></td><td>${esc(name(b.component))}<p>${esc(b.role)}</p></td><td>${b.files.map(p=>`<a class="trace-source" href="${mapFileUrl(p)}">${esc(p)}</a>`).join('')||'<span class="badge review">目标实现待定位</span>'}</td><td>${bindingLabels[b.status]}<p>${esc(b.gap)}</p></td></tr>`).join('');
  }
  function sharedHtml(sc) {
    const state=sharedState(sc);
    return `<h2>${sc.id} ${esc(sc.name)}</h2><span class="badge ${state==='verified'?'confirmed':'review'}">${stateNames[state]}</span><p>${esc(sc.purpose)}</p><p><b>完成条件</b> ${esc(sc.completion)}</p><p>编排负责组件 · ${esc(name(sc.orchestrator))}</p><div class="detail-columns"><div><h3>输入与前置</h3>${list(sc.inputs)}${list(sc.preconditions)}<h3>取消与重试</h3><p>${esc(sc.cancel)}</p><p>${esc(sc.retry)}</p></div><div><h3>参与组件及职责</h3>${sc.participants.map(p=>`<p><b>${esc(name(p.component))}</b><br>${p.bindings.map(id=>esc(catalog.traceability.bindings[id].role)).join('、')}</p>`).join('')}</div></div><h3>共享子步骤</h3><div class="case-chips">${sc.steps.map(s=>`<a class="case-chip" href="${traceUrl({sc:sc.id,substep:s.id})}">${s.id} ${esc(s.title)}</a>`).join('')}</div><h3>失败处理</h3>${sc.failures.map(f=>`<p><a href="${traceUrl({sc:sc.id,substep:f.id})}">${f.id} ${esc(f.trigger)}</a><br>${esc(f.resulting_state)} · ${esc(f.next_action)}</p>`).join('')}<h3>本 SC 的集成验收</h3><div class="acceptance-grid">${sc.acceptance.map(a=>`<article><code>${a.id}</code><p><b>Given</b> ${esc(a.given)}</p><p><b>When</b> ${esc(a.when)}</p><p><b>Then</b> ${esc(a.then)}</p></article>`).join('')}</div><p class="muted">进度依据 · ${esc(sc.progress.note)}${sc.progress.next?`<br>下一步 · ${esc(sc.progress.next)}`:''}${sc.progress.evidence?`<br>回执 <code>${esc(sc.progress.evidence.path)}</code><br>验证范围 · ${esc(sc.progress.evidence.scope||'按原回执说明')}`:''}</p>`;
  }
  get('trace-uc').innerHTML='<option value="">选择用户用例</option>'+catalog.use_cases.map(u=>`<option value="${u.id}">${u.id} ${esc(u.title)}</option>`).join('');
  get('trace-sc').innerHTML='<option value="">选择共享子流程</option>'+catalog.shared_steps.map(s=>`<option value="${s.id}">${s.id} ${esc(s.name)}</option>`).join('');
  get('trace-uc').onchange=()=>{win.location.hash=traceUrl({uc:get('trace-uc').value});};
  get('trace-sc').onchange=()=>{win.location.hash=traceUrl({sc:get('trace-sc').value});};
  win.addEventListener('hashchange',()=>{if(win.location.hash.startsWith('#trace'))render();});
  render();
}
