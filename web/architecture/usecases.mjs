const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

export function relatedCases(catalog, moduleId) {
  return catalog.use_cases.filter(useCase => useCase.modules.includes(moduleId));
}

export function changeScope(catalog, changeId) {
  const change = catalog.change_examples.find(row => row.id === changeId);
  if (!change) throw new Error('Unknown change example');
  return {
    ...change,
    useCases: catalog.use_cases.filter(row => change.cases.includes(row.id)),
    validation: catalog.validation_tiers.filter(row => change.tiers.includes(row.id))
  };
}

function actor(x, y, label) {
  return `<g stroke="#536b87" stroke-width="1.6" fill="none"><circle cx="${x}" cy="${y}" r="12"/><path d="M${x} ${y+12} V${y+51} M${x-23} ${y+28} H${x+23} M${x} ${y+51} L${x-20} ${y+78} M${x} ${y+51} L${x+20} ${y+78}"/></g><text x="${x}" y="${y+102}" text-anchor="middle" fill="#415b78" font-size="13">${esc(label)}</text>`;
}

export function useCaseSvg(catalog, group, selectedId) {
  const cases = catalog.use_cases.filter(row => row.group === group);
  const selected = cases.find(row => row.id === selectedId) || cases[0];
  const shared = catalog.shared_steps.filter(step => selected.shared_steps.includes(step.id));
  const caseRows = Math.ceil(cases.length / 2);
  const sharedLabelY = 104 + (caseRows - 1) * 100 + 95;
  const sharedRows = Math.ceil(shared.length / 4);
  const footerY = sharedLabelY + 70 + Math.max(0,sharedRows - 1) * 86 + 65;
  const height = footerY + 55;
  let svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1100 ${height}" role="img" aria-label="${esc(group)}用例图" font-family="system-ui, PingFang SC, Microsoft Yahei, sans-serif"><defs><marker id="uc-dependency" markerWidth="9" markerHeight="8" refX="8" refY="4" orient="auto"><path d="M0 0 L8 4 L0 8" fill="none" stroke="#8ba2c0"/></marker></defs>`;
  svg += `<rect x="200" y="18" width="770" height="${height-40}" rx="12" fill="#f8fafc" stroke="#b7c7dc"/><text x="225" y="48" fill="#536d8c" font-size="14">工作台系统边界 · 用户目标与复用行为</text>`;
  const positions = new Map();
  cases.forEach((row, index) => {
    const x = index % 2 ? 755 : 415;
    const y = 104 + Math.floor(index / 2) * 100;
    positions.set(row.id, {x,y});
    const active = row.id === selected.id;
    svg += `<g data-use-case="${row.id}" role="button" tabindex="0" aria-label="${esc(row.title)}"><ellipse cx="${x}" cy="${y}" rx="147" ry="35" fill="${active?'#e8efff':'white'}" stroke="${active?'#3565c9':'#9eb0c8'}" stroke-width="${active?2:1}"/><text x="${x}" y="${y-7}" text-anchor="middle" fill="#7690b2" font-size="11">${row.id}</text><text x="${x}" y="${y+14}" text-anchor="middle" fill="#23405e" font-size="14">${esc(row.title)}</text></g>`;
  });
  const position = positions.get(selected.id);
  const mainActor = catalog.actors.find(row => row.id === selected.actor);
  svg += actor(92, position.y - 38, mainActor.name);
  const actorPath = position.x < 600
    ? `M92 ${position.y} H${position.x-147}`
    : `M92 ${position.y} H180 V${position.y-48} H${position.x} V${position.y-35}`;
  svg += `<path d="${actorPath}" stroke="#536b87" fill="none"/>`;
  if (group==='记录') {
    svg += actor(1044, position.y - 38, '控制与相机源');
    svg += `<path d="${position.x>600?`M${position.x+147} ${position.y} H1044`:`M${position.x} ${position.y+35} V${position.y+55} H1018 V${position.y} H1044`}" stroke="#536b87" fill="none"/>`;
  } else if (['UC09','UC10'].includes(selected.id)) {
    svg += actor(1044, position.y - 38, '正式数采平台');
    svg += `<path d="${position.x>600?`M${position.x+147} ${position.y} H1044`:`M${position.x} ${position.y+35} V${position.y+55} H1018 V${position.y} H1044`}" stroke="#536b87" fill="none"/>`;
  } else if (selected.modules.includes('sdk') || selected.id === 'UC12') {
    svg += actor(1044, position.y - 38, '机器人与相机');
    svg += `<path d="${position.x>600?`M${position.x+147} ${position.y} H1044`:`M${position.x} ${position.y+35} V${position.y+55} H1018 V${position.y} H1044`}" stroke="#536b87" fill="none"/>`;
  }
  svg += `<text x="225" y="${sharedLabelY}" fill="#7188a5" font-size="12">共享行为引用 · 具体主流程与异常步骤见追溯</text>`;
  const columns=Math.min(4,Math.max(1,shared.length));
  shared.forEach((step, index) => {
    const x = 220 + (index % columns + .5) * 730 / columns;
    const y = sharedLabelY + 70 + Math.floor(index / columns) * 86;
    const rx = Math.min(87,730 / columns / 2 - 6);
    svg += `<a href="#trace?sc=${step.id}"><ellipse cx="${x}" cy="${y}" rx="${rx}" ry="32" fill="#eef3fa" stroke="#9bb0cd"/><text x="${x}" y="${y-6}" text-anchor="middle" fill="#7890ae" font-size="10">${step.id}</text><text x="${x}" y="${y+12}" text-anchor="middle" fill="#486484" font-size="11">${esc(step.name)}</text></a>`;
    svg += `<path d="M${position.x} ${position.y+35} V${position.y+46} H960 V${y-50} H${x} V${y-33}" fill="none" stroke="#9db0ca" stroke-dasharray="4 4" marker-end="url(#uc-dependency)"/>`;
  });
  if (shared.length) svg += `<text x="832" y="${sharedLabelY+14}" fill="#7890b0" font-size="10">步骤引用</text>`;
  const extensions = catalog.use_case_relations.filter(row => row.type==='extend' && (row.to === selected.id || row.from === selected.id));
  const extensionText = extensions.map(row => `${row.from} «extend» ${row.to} · ${row.condition}`).join('  /  ');
  svg += `<text x="225" y="${footerY}" fill="#69809d" font-size="11">${esc(extensionText || '主流程、失败后状态与验收case见下方用例详情')}</text>`;
  return svg + '</svg>';
}

export function operationsSvg() {
  const nodes = [
    ['queued','已记录请求',30,30],['checking','检查前置',245,30],['running','执行步骤',460,30],['verifying','核验后置',675,30],['succeeded','确认成功',890,30],
    ['failed','确认失败',245,210],['unknown','结果未知',675,210],['cancelling','请求取消',460,365],['cancelled','确认取消',890,365]
  ];
  let svg = '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1100 500" role="img" aria-label="运维操作状态图" font-family="system-ui, PingFang SC, Microsoft Yahei, sans-serif"><defs><marker id="op-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0 L8 4 L0 8" fill="#8197b3"/></marker></defs>';
  for (const [id,label,x,y] of nodes) {
    const color = id==='unknown'?'#bf7b13':id==='failed'?'#ba4545':id==='succeeded'||id==='cancelled'?'#21845c':'#3565c9';
    svg += `<rect x="${x}" y="${y}" width="175" height="68" rx="10" fill="#f6f8fc" stroke="${color}"/><text x="${x+87}" y="${y+27}" text-anchor="middle" fill="${color}" font-size="13">${id}</text><text x="${x+87}" y="${y+49}" text-anchor="middle" fill="#405d7e" font-size="13">${label}</text>`;
  }
  const edge = (d,label,x,y,dash=false) => `<path d="${d}" fill="none" stroke="#8197b3" ${dash?'stroke-dasharray="5 4"':''} marker-end="url(#op-arrow)"/><text x="${x}" y="${y}" fill="#758ba7" font-size="11">${esc(label)}</text>`;
  svg += edge('M205 64 H239','',0,0)+edge('M420 64 H454','',0,0)+edge('M635 64 H669','',0,0)+edge('M850 64 H884','',0,0);
  svg += edge('M332 98 V204','前置不满足',342,156);
  svg += edge('M547 98 V244 H669','发出动作后失联 / 超时',454,191);
  svg += edge('M802 98 V204','核验不足',812,158);
  svg += edge('M717 210 V104','只读核验原请求',584,141,true);
  svg += edge('M547 98 H446 V398 H454','显式取消',450,333);
  svg += edge('M635 399 H884','取消后置条件已确认',678,449);
  svg += edge('M635 387 H762 V285','取消结果待核验',773,331,true);
  svg += '<text x="35" y="478" fill="#617b9c" font-size="12">Stop 独立优先处理。unknown 先核验原 operation_id，再决定可继续或可重复的步骤。</text>';
  return svg + '</svg>';
}

function list(items) {
  return `<ul>${items.map(item=>`<li>${esc(item)}</li>`).join('')}</ul>`;
}
function caseButton(row) {
  return `<button class="case-chip" data-use-case="${row.id}">${row.id} ${esc(row.title)}</button>`;
}

export function initUseCases(catalog, {get, openView}) {
  let group='运维';
  let selectedId='UC09';
  const moduleName=id=>[...catalog.components,...catalog.supporting].find(row=>row.id===id)?.name||id;
  const useCase=id=>catalog.use_cases.find(row=>row.id===id);
  get('usecase-summary').textContent=`${catalog.goals.length} 个用户目标 · ${catalog.use_cases.length} 个用例 · ${catalog.journeys.length} 条典型旅程 · ${catalog.use_cases.reduce((n,row)=>n+row.acceptance.length,0)} 项目标验收case`
  get('goals').innerHTML=catalog.goals.map(goal=>`<article class="goal-card"><span class="rule-id">${goal.id}</span><h3>${esc(goal.title)}</h3><p>${esc(goal.outcome)}</p><div class="case-chips">${goal.cases.map(id=>caseButton(useCase(id))).join('')}</div></article>`).join('');
  get('journeys').innerHTML=catalog.journeys.map(journey=>`<article class="journey"><h3>${esc(journey.name)}</h3><div class="journey-route">${journey.cases.map(id=>caseButton(useCase(id))).join('<span class="route-arrow">→</span>')}</div><p class="muted">${esc(journey.success)}</p></article>`).join('');
  get('layers').innerHTML=catalog.layers.map(layer=>`<article class="layer"><span class="rule-id">${layer.id}</span><div><h3>${esc(layer.name)}</h3><p>${esc(layer.responsibility)}</p><div class="layer-tags">${layer.contains.map(name=>`<code>${esc(name)}</code>`).join(' ')}</div></div></article>`).join('');
  function selectCase(id) {
    const row=useCase(id);
    if (!row) return;
    selectedId=id;group=row.group;
    get('case-groups').querySelectorAll('button').forEach(button=>button.classList.toggle('active',button.dataset.caseGroup===group));
    get('case-select').innerHTML=catalog.use_cases.filter(item=>item.group===group).map(item=>`<option value="${item.id}">${item.id} ${esc(item.title)}</option>`).join('');
    get('case-select').value=id;
    get('usecase-graph').innerHTML=useCaseSvg(catalog,group,id);
    get('case-detail').innerHTML=`<div class="detail-head"><span class="rule-id">${row.id}</span><h2>${esc(row.title)}</h2><span class="badge review">设计待确认</span></div><p class="muted">参与者 · ${esc(catalog.actors.find(actor=>actor.id===row.actor).name)}　目标 · ${row.goal}</p><p>${esc(row.current)}</p><div class="detail-columns"><div><h3>前置条件</h3>${list(row.preconditions)}<h3>用户主流程</h3><ol>${row.main_flow.map(step=>`<li><a href="#trace?uc=${row.id}&step=${step.id}">${step.id}</a> ${esc(step.title)}</li>`).join('')}</ol></div><div><h3>成功后的状态</h3>${list(row.success_postconditions)}<h3>实际影响</h3>${list(row.effects)}<h3>涉及的模块与契约</h3><p>${row.modules.map(moduleName).map(esc).join(' · ')}</p><div class="case-chips">${row.interfaces.map(id=>`<button class="contract-chip" data-contract="${id}">${id} ${esc(catalog.interfaces.find(item=>item.id===id).name)}</button>`).join('')}</div><p class="muted">关联规则 · ${row.rules.join(' / ')}</p></div></div><h3>异常分支与下一步</h3><div class="table-scroll"><table><thead><tr><th>触发情况</th><th>此时系统状态</th><th>允许的后续处理</th></tr></thead><tbody>${row.alternatives.map(branch=>`<tr><td><a href="#trace?uc=${row.id}&step=${branch.id}">${branch.id}</a> ${esc(branch.trigger)}</td><td>${esc(branch.resulting_state)}</td><td>${esc(branch.next_action)}</td></tr>`).join('')}</tbody></table></div><h3 class="subheading">可执行验收case</h3><div class="acceptance-grid">${row.acceptance.map(test=>`<article><span class="badge ${test.kind==='success'?'review':'partial'}">${test.kind==='success'?'主流程标准':'失败分支标准'}</span><p><b>Given</b> ${esc(test.given)}</p><p><b>When</b> ${esc(test.when)}</p><p><b>Then</b> ${esc(test.then)}</p></article>`).join('')}</div><p class="muted">上述条目是目标验收条件。现有源码依据 ${row.evidence.map(id=>`${catalog.sources[id].path}:${catalog.sources[id].line}`).map(esc).join(' · ')}</p>`;
    get('usecase-graph').querySelectorAll('[data-use-case]').forEach(node=>node.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();selectCase(node.dataset.useCase);}}));
  }
  get('case-groups').innerHTML=[...new Set(catalog.use_cases.map(row=>row.group))].map(name=>`<button data-case-group="${esc(name)}">${esc(name)}</button>`).join('');
  get('case-groups').querySelectorAll('button').forEach(button=>button.onclick=()=>selectCase(catalog.use_cases.find(row=>row.group===button.dataset.caseGroup).id));
  get('case-select').onchange=()=>selectCase(get('case-select').value);
  get('main').addEventListener('click',event=>{
    const target=event.target.closest('[data-use-case]');
    if(target){const inDiagram=!!target.closest('#usecase-graph');selectCase(target.dataset.useCase);if(!inDiagram){openView('usecases');get('case-detail').scrollIntoView({block:'start'});}}
    const contract=event.target.closest('[data-contract]');
    if(contract){openView('interfaces');const entry=get('contract-'+contract.dataset.contract);if(entry){entry.open=true;entry.scrollIntoView({block:'start'});}}
  });
  selectCase(selectedId);
  window.addEventListener('hashchange',()=>{if(location.hash.startsWith('#usecases?')){const id=new URLSearchParams(location.hash.split('?')[1]).get('uc');if(id)selectCase(id);}});
  get('operations-graph').innerHTML=operationsSvg();
  get('operation-transitions').innerHTML=catalog.operation_design.transitions.map(([from,to])=>`<tr><td><code>${from}</code> → <code>${to}</code></td><td>${esc(catalog.operation_design.transition_reasons[from+'>'+to])}</td></tr>`).join('');
  get('operation-principles').innerHTML=catalog.operation_design.principles.map(text=>`<li>${esc(text)}</li>`).join('');
  get('state-facts').innerHTML=catalog.operation_design.facts.map(fact=>`<tr><td><code>${fact.name}</code></td><td>${esc(fact.definition)}</td><td>${esc(moduleName(fact.owner))}</td></tr>`).join('');
  get('operation-receipt').textContent=JSON.stringify({schema:'workbench.operation.proposal.v1',use_case_id:'UC09',operation_id:'<operation-id>',request_id:'<idempotency-key>',outcome:'failed',completed_steps:['stop_verified','sdk_release_verified'],failed_step:'collector_web_ready',state:{sdk:'released',collector_web:'failed'},allowed_next_actions:['inspect_collector_web','retry_verified_web_start_step'],evidence_refs:['<release-receipt>','<web-start-error>']},null,2);
  const modules=[...catalog.components,...catalog.supporting];
  get('case-matrix').innerHTML=modules.map(module=>`<tr><td>${esc(module.name)}</td><td><div class="case-chips">${relatedCases(catalog,module.id).map(caseButton).join('')}</div></td><td>${catalog.interfaces.filter(i=>[...(i.producers||[i.producer]),...(i.consumers||[i.consumer])].includes(module.id)).map(i=>`<code>${i.id}</code>`).join(' ')||'通过所属用例的契约组合'}</td></tr>`).join('');
  get('change-select').innerHTML=catalog.change_examples.map(change=>`<option value="${change.id}">${esc(change.title)}</option>`).join('');
  function renderChange() {
    const change=changeScope(catalog,get('change-select').value);
    get('change-detail').innerHTML=`<div class="case-chips">${change.useCases.map(caseButton).join('')}</div><p class="muted">接口 ${change.interfaces.join(' / ')}　规则 ${change.rules.join(' / ')||'按展示内容引用'}</p><h3>需要的验证层级</h3><div class="validation-grid">${change.validation.map(tier=>`<article><span class="rule-id">${tier.id}</span><h4>${esc(tier.name)}</h4><p>${esc(tier.scope)}</p></article>`).join('')}</div><h3>本次检查依据</h3>${list(change.checks)}<p class="rule-note">证据复用 · ${esc(change.reuse)}</p>`;
  }
  get('change-select').onchange=renderChange;renderChange();
  return {selectCase};
}
