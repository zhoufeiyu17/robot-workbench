import {initRecording, recordingPolicyFor} from './recording.mjs?v=20261105-v3';
import {initUseCases, relatedCases} from './usecases.mjs?v=trace1';
import {initTraceability} from './traceability.mjs?v=trace1';
export function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>"']/g, character => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[character]));
}

export function modelExtensionSelection(plan,family='wam',steps=15) {
  const horizon=family==='wam'?plan.candidate.horizon:60;
  if(!['wam','pi05'].includes(family)||!Number.isInteger(steps)||steps<1||steps>horizon)throw Error(`执行步数须为1至${horizon}的整数`);
  return {family,horizon,steps,sourceSeconds:steps/30,maximumSeconds:steps/30*4};
}

export function modelExtensionSvg(plan) {
  const colors={verified:'#21845c',active:'#bf7b13',planned:'#3565c9'};
  const labels={verified:'来源已核对',active:'进行中',planned:'待开发／验收'};
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1100 145" role="img" aria-label="WAM开发任务图">${plan.steps.map((s,i)=>{
    const x=10+i*183,color=colors[s.state];
    return `${i?`<path d="M${x-15} 60h10" stroke="#8194ae"/>`:''}<g><rect x="${x}" y="12" width="168" height="105" rx="10" fill="#fff" stroke="${color}"/><text x="${x+12}" y="37" font-size="12" fill="${color}">${escapeHtml(s.id)}</text><text x="${x+12}" y="64" font-size="14" fill="#243b56">${escapeHtml(s.title)}</text><text x="${x+12}" y="94" font-size="12" fill="${color}">${labels[s.state]}</text></g>`;
  }).join('')}</svg>`;
}

export function modelExtensionHtml(plan,family='wam') {
  if(!plan)return '<p>暂无新增模型设计。</p>';
  const esc=escapeHtml,c=plan.candidate,h=family==='wam'?c.horizon:60,age=family==='wam'?(c.maximum_observation_age_s??.5):(c.pi05_maximum_observation_age_s??.5);
  return `<div class="notice"><strong>${esc(plan.title)} · ${plan.status==='approved'?'方案已确认，待实现验收':'设计候选'}</strong><p>${esc(plan.summary)}</p><p>已有运行入口与本页草案独立；此页不发送模型或机器人操作。</p></div>
  <div class="panel"><h2>开发任务图</h2><div class="graph">${modelExtensionSvg(plan)}</div><div class="table-scroll"><table><thead><tr><th>阶段</th><th>完成条件</th><th>文件定位</th></tr></thead><tbody>${plan.steps.map(s=>`<tr><td>${esc(s.id)} ${esc(s.title)}</td><td>${esc(s.note)}</td><td>${s.files.map(p=>`<a href="#progress?file=${encodeURIComponent(p)}"><code>${esc(p)}</code></a>`).join('<br>')}${(s.proposed_files||[]).map(p=>`<p>拟新增：<code>${esc(p)}</code></p>`).join('')}</td></tr>`).join('')}</tbody></table></div></div>
  <div class="panel"><h2>模型选择与执行长度 · UI草案</h2><div class="parameter-grid"><label>模型模式<select id="model-design-family"><option value="wam" ${family==='wam'?'selected':''}>WAM · ${esc(c.model)} · ${c.denoise_steps}步去噪</option><option value="pi05" ${family==='pi05'?'selected':''}>π0.5 · 保留现有模型配置</option></select></label><label>每个动作片段执行步数<input id="model-design-steps" type="number" min="1" max="${h}" step="1" value="15"></label></div><p>模型输出 ${h} 步；执行步数是独立选择。默认执行15步。观测／结果新鲜度上限 ${age} 秒；实时反馈与运动保护独立保持。</p><p id="model-design-duration">15步源时长0.500s；单段最长2.000s。</p><p id="model-design-error" role="status"></p><p>WAM文本：${esc(c.prompt)}。资格：未完成模型验收。</p><button disabled>运行 · 仅设计预览</button></div>
  <div class="panel"><h2>已核验来源与待闭合事实</h2><p>${esc(plan.source.host)} · <code>${esc(plan.source.root+'/'+plan.source.bundle)}</code></p><p>头部｜左腕｜右腕；14个有效机器人维度映射到80维网络输入。2步合成基准约260ms只作来源参考，实际输入首次/热调用仍待验。</p><ul>${plan.pending.map(p=>`<li>${esc(p)}</li>`).join('')}</ul></div>
  <div class="panel"><h2>接口新增／修改候选</h2><div class="table-scroll"><table><thead><tr><th>契约 / 字段</th><th>现状 → 候选</th><th>消费者及用途</th><th>传递时机</th></tr></thead><tbody>${plan.interfaces.map(i=>`<tr><td>${esc(i.id)}<br><code>${esc(i.field)}</code></td><td>${esc(i.before)}<br>→ ${esc(i.after)}</td><td>${esc(i.consumer)}</td><td>${esc(i.frequency)}</td></tr>`).join('')}</tbody></table></div></div>
  <div class="panel"><h2>验收用例 · 定义完成，尚未执行WAM模型</h2><div class="table-scroll"><table><thead><tr><th>ID</th><th>场景</th><th>通过条件</th></tr></thead><tbody>${plan.cases.map(c=>`<tr><td>${esc(c.id)}</td><td>${esc(c.title)}<p>${(c.use_cases||[]).map(id=>`<a href="#trace?uc=${encodeURIComponent(id)}">${esc(id)}</a>`).join(" · ")}</p></td><td>${esc(c.acceptance)}</td></tr>`).join('')}</tbody></table></div></div>`;
}

function fieldContractHtml(contract) {
  if (!contract.field_contracts) return '';
  return `<h3>字段消费者与传递时机</h3><div class="table-scroll"><table><thead><tr><th>字段</th><th>消费者与用途</th><th>传递时机</th><th>校验</th></tr></thead><tbody>${contract.field_contracts.map(row=>`<tr><td><code>${escapeHtml(row.name)}</code></td><td>${escapeHtml(row.consumer)}<p>${escapeHtml(row.purpose)}</p></td><td>${escapeHtml(row.frequency)}</td><td>${escapeHtml(row.validation)}</td></tr>`).join('')}</tbody></table></div>`;
}

export function validateDraft(values, parameters) {
  const errors = [];
  for (const parameter of parameters) {
    const value = values[parameter.key];
    if (!Number.isFinite(value) || value <= 0) errors.push(`${parameter.label}须为正数`);
    if (parameter.integer && !Number.isInteger(value)) errors.push(`${parameter.label}须为整数`);
    if (parameter.min !== undefined && value < parameter.min) errors.push(`${parameter.label}至少${parameter.min}`);
    if (parameter.max !== undefined && value > parameter.max) errors.push(`${parameter.label}最多${parameter.max}`);
  }
  if (values.planning_speed > values.measured_speed) errors.push('规划速度须小于或等于实测速度保护值');
  return [...new Set(errors)];
}

export function buildDraft(values, options, revision, executionContract) {
  return {
    schema: 'workbench.control-policy.proposal.v1',
    design_revision: revision,
    status: 'draft_for_review',
    effective_from: 'next_run_after_explicit_commit',
    requires_device_capability_validation: true,
    planning: {joint_velocity_rad_s: values.planning_speed, joint_acceleration_rad_s2: values.acceleration, jaw_velocity_mm_s: values.jaw_speed},
    measured: {joint_velocity_rad_s: values.measured_speed},
    execution: {steps: values.steps, model_hz: executionContract.model_hz, source_duration_s: values.steps / executionContract.model_hz, max_duration_factor: executionContract.max_duration_factor, max_segment_duration_s: values.steps / executionContract.model_hz * executionContract.max_duration_factor, run_budget_s: values.run_seconds},
    inference: {observation_wait_ms: values.observation_wait, result_window_ms: values.result_window, model_call_timeout_ms: values.call_timeout, deadline_rule: 'min(observation_source+result_window,call_started+model_call_timeout)'},
    tracking: {mode: options.tracking, joint_rad: values.tracking_joint, jaw_mm: values.tracking_jaw, duration_ms: values.tracking_hold},
    arrival: {mode: options.arrival, joint_rad: values.arrival_joint, jaw_mm: values.arrival_jaw},
    recording: {...options.recordingPolicy},
    observation: {pairing: {mode: options.pairing, rgb_error_ms: values.pair_rgb, state_error_ms: values.pair_state}}
  };
}

const colors = {guard:'#ba4545',input:'#bf7b13',infer:'#21845c',translation:'#bf7b13',evidence:'#bf7b13'};
function node(id,x,y,w,h,title,sub,status,color) {
  return `<g data-component="${id}" tabindex="0" role="button" aria-label="查看${escapeHtml(title)}"><rect x="${x}" y="${y}" width="${w}" height="${h}" rx="11" fill="#fff" stroke="${color}" stroke-width="1.5"/><rect x="${x}" y="${y}" width="5" height="${h}" rx="2" fill="${color}"/><text x="${x+18}" y="${y+29}" font-size="16" font-weight="650" fill="#263c58">${escapeHtml(title)}</text><text x="${x+18}" y="${y+51}" font-size="11" fill="#667b95">${escapeHtml(sub)}</text><text x="${x+18}" y="${y+73}" font-size="11" fill="${color}">${escapeHtml(status)}</text></g>`;
}
function svgStart(label,height=385) {
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1100 ${height}" role="img" aria-label="${label}" font-family="system-ui, PingFang SC, Microsoft Yahei, sans-serif"><defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8" fill="#8194ae"/></marker></defs>`;
}
export function componentSvg(catalog,target=true) {
  const status = id => target ? '设计待确认' : catalog.statuses[catalog.components.find(row=>row.id===id).status].label;
  const color = id => target ? '#3565c9' : colors[id];
  let svg=svgStart('五个公共模块及并行记录数据流',565);
  svg+='<rect x="10" y="8" width="1080" height="545" rx="14" fill="#f7f9fc"/>';
  svg+=node('guard',228,24,665,85,target?'公用控制门槛 · Control Guard':'门槛分散在多个执行点',target?'唯一策略版本 · 统一裁决 · 录制用途策略':'Workflow / Runner / Runtime / 解释器 / SDK 各自校验',status('guard'),color('guard'));
  svg+='<text x="32" y="53" fill="#677d99" font-size="13">应用层用例编排</text><text x="32" y="75" fill="#8b9bb0" font-size="11">参数与录制会话</text><path d="M146 65 H220" stroke="#8194ae" fill="none" marker-end="url(#arrow)"/>';
  for(const x of [145,445,755])svg+=`<path d="M${x<228?265:x} 109 L${x} 154" stroke="#9cb2d9" stroke-dasharray="4 4" fill="none"/>`;
  svg+=node('input',25,157,250,94,'相机与 state 管理','ObservationBundle + 全流帧源',status('input'),color('input'));
  svg+=node('infer',330,157,230,94,'共享 infer 服务','ModelRequest → ActionChunk',status('infer'),color('infer'));
  svg+=node('translation',615,157,280,94,'轨迹翻译与执行','准备器 + 唯一 Owner / SDK 出口',status('translation'),color('translation'));
  svg+='<rect x="955" y="166" width="119" height="75" rx="10" fill="#e8edf5"/><text x="975" y="198" fill="#425b7a" font-size="15">SDK / CAN</text><text x="969" y="219" fill="#647d9a" font-size="10">设备能力与原生保护</text>';
  for(const [a,b]of [[275,324],[560,609],[895,949]])svg+=`<path d="M${a} 202 H${b}" stroke="#8194ae" fill="none" marker-end="url(#arrow)"/>`;
  svg+='<path d="M1015 242 V292 H146 V258" stroke="#7297aa" fill="none" marker-end="url(#arrow)"/><text x="476" y="285" font-size="11" fill="#5c7b92">ExecutionFeedback · 同时刻反馈</text>';
  svg+=node('evidence',160,396,785,104,target?'中间产物采集与保存 · 独立进程':'记录能力分布在多个业务实现',target?'全程状态 / 指令 / 连续视频 / 模型与业务事件 → 异步编码、写盘、封存':'ProcessRecordingSession / InferenceTrace / InferenceEvidence → 通用服务待归并',status('evidence'),color('evidence'));
  svg+='<path d="M210 251 V335 H260 V389" stroke="#8a9fb8" fill="none" marker-end="url(#arrow)"/><text x="180" y="355" font-size="11" fill="#6b819c">连续视频 / 状态源</text>';
  svg+='<path d="M445 251 V389" stroke="#8a9fb8" fill="none" marker-end="url(#arrow)"/><text x="455" y="355" font-size="11" fill="#6b819c">输入 / 输出引用</text>';
  svg+='<path d="M755 251 V389" stroke="#8a9fb8" fill="none" marker-end="url(#arrow)"/><text x="765" y="355" font-size="11" fill="#6b819c">控制与业务事件</text>';
  svg+='<path d="M893 65 H1080 V370 H900 V389" stroke="#a1afc3" fill="none" stroke-dasharray="4 4" marker-end="url(#arrow)"/>';
  svg+='<path d="M945 449 H1092 V125 H900 V100" stroke="#6c94be" fill="none" stroke-dasharray="4 4" marker-end="url(#arrow)"/><text x="952" y="426" font-size="10" fill="#6b819c">健康 → Guard</text>';
  svg+='<text x="198" y="532" font-size="12" fill="#617b9b">数据面有界发布 / 共享帧引用；控制面管理CaptureSession与用途策略；资源和性能独立核验。</text>';
  return svg+'</svg>';
}
export function flowSvg() {
  const labels=[['绑定上下文','设备 · 模型 · policy_hash'],['取得观测','三相机 + 实测 state'],['模型预测','原始60点 + 有效期限'],['翻译准备','前N点 + 动态/刹车证明'],['下发短窗口','唯一Owner + SDK回执'],['监测实际执行','速度 · 跟踪 · 反馈年龄'],['核验段末','输出完成 + 实际到位'],['进入下一段 / 结束','新观测或HOLD / Stop']];
  let s=svgStart('控制与独立记录的全生命周期信息流',440);
  labels.forEach(([title,sub],i)=>{const col=i<4?i:7-i,x=24+col*270,y=i<4?30:202;s+=`<rect x="${x}" y="${y}" width="240" height="88" rx="10" fill="#f3f6fb" stroke="#d4deeb"/><text x="${x+15}" y="${y+26}" font-size="11" fill="#6e8cba">0${i+1}</text><text x="${x+15}" y="${y+49}" font-size="15" fill="#294461">${title}</text><text x="${x+15}" y="${y+71}" font-size="10" fill="#71849c">${sub}</text>`;});
  for(const x of[264,534,804])s+=`<path d="M${x} 74 H${x+24}" stroke="#8194ae" marker-end="url(#arrow)"/>`;
  s+='<path d="M954 120 V195" stroke="#8194ae" marker-end="url(#arrow)"/>';
  for(const x of[834,564,294])s+=`<path d="M${x} 246 H${x-24}" stroke="#8194ae" marker-end="url(#arrow)"/>`;
  s+='<rect x="235" y="144" width="620" height="32" rx="7" fill="#e9effc"/><text x="285" y="165" font-size="12" fill="#3863a5">各阶段统一返回 GuardDecision · 阶段 / 原因 / 实际值 / 门槛 / 处理动作</text>';
  s+='<path d="M144 290 V337 H954 V290 M414 290 V337 M684 290 V337" stroke="#9daec5" stroke-dasharray="4 4" fill="none"/><path d="M550 337 V352" stroke="#8194ae" marker-end="url(#arrow)"/><rect x="160" y="358" width="780" height="64" rx="10" fill="#eaf2ff" stroke="#9cb9e6"/><text x="195" y="385" font-size="14" fill="#305b96">独立记录进程 · 全流视频 / 状态 / 指令 / 模型 / 业务事件</text><text x="195" y="406" font-size="11" fill="#6c85a8">执行前准备会话，各阶段并行采集，finish提交序列屏障后异步封存</text>';
  return s+'</svg>';
}

async function boot() {
  const get=id=>document.getElementById(id);
  let catalog;
  try {const response=await fetch('catalog.json',{cache:'no-store'});if(!response.ok)throw Error(`HTTP ${response.status}`);catalog=await response.json();}
  catch(error){get('loading').textContent=`架构目录读取失败 · ${error.message}`;return;}
  const esc=escapeHtml;
  const badge=status=>`<span class="badge ${status}">${esc(catalog.statuses[status]?.label||status)}</span>`;
  const name=id=>[...catalog.components,...catalog.supporting].find(row=>row.id===id)?.name||id;
  const sourceHtml=keys=>(keys||[]).map(key=>{const s=catalog.sources[key];return `<details class="source"><summary>依据 · ${esc(s.path)}:${s.line}</summary><p>${esc(s.scope)} · SHA256 ${s.sha256.slice(0,16)}…</p><pre>${esc(s.excerpt)}</pre></details>`;}).join('');
  const time=new Date(catalog.checked_at).toLocaleString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false});
  get('snapshot-text').textContent=catalog.runtime_snapshot.summary;
  get('snapshot-time').textContent=`只读核验快照 ${time} CST`;
  get('revision').textContent=catalog.revision;
  const modelPlan=catalog.model_extensions?.[0];
  get('model-extension-design').innerHTML=modelExtensionHtml(modelPlan);
  get('model-extension-design').addEventListener('change',event=>{
    if(event.target.id==='model-design-family')get('model-extension-design').innerHTML=modelExtensionHtml(modelPlan,event.target.value);
    else if(event.target.id==='model-design-steps'){
      try {
        const value=modelExtensionSelection(modelPlan,get('model-design-family').value,Number(event.target.value));
        get('model-design-duration').textContent=`${value.steps}步源时长${value.sourceSeconds.toFixed(3)}s；单段最长${value.maximumSeconds.toFixed(3)}s。`;
        get('model-design-error').textContent='';
      }catch(error){get('model-design-error').textContent=error.message;}
    }
  });
  const speedDefaults=['planning_speed','measured_speed'].map(key=>catalog.parameters.find(row=>row.key===key)?.value??'未声明').join(' / ');
  get('metrics').innerHTML=[[catalog.components.length,'核心模块','目标边界待确认'],[catalog.gates.length,'已梳理规则','每项有执行点与来源'],[catalog.interfaces.length,'接口契约','当前协议 + 目标设计'],[esc(speedDefaults),'规划 / 实测速度 rad/s','目录默认值 · 实际值以设备回读为准']].map(([value,label,sub])=>`<div class="metric"><strong>${value}</strong><span>${label}</span><small>${sub}</small></div>`).join('');
  get('legend').innerHTML=Object.entries(catalog.statuses).filter(([id])=>id!=='confirmed').map(([,s])=>`<span><i style="background:${s.color}"></i>${esc(s.label)}</span>`).join('');
  let selected='guard',target=true;
  function selectComponent(id) {
    selected=id;
    const c=catalog.components.find(row=>row.id===id);if(!c)return;
    get('component-cards').innerHTML=catalog.components.map(row=>`<button class="component-card ${selected===row.id?'selected':''}" data-id="${row.id}" style="border-top-color:${catalog.statuses[row.status].color}">${badge(row.status)}<strong>${esc(row.name)}</strong><small>${esc(row.summary)}</small></button>`).join('');
    get('component-detail').innerHTML=`<div class="detail-head"><h2>${esc(c.name)}</h2>${badge(c.status)}<span class="muted">${esc(c.en)}</span></div><p>${esc(c.current)}</p><div class="detail-columns"><div><h3>目标职责</h3><ul>${c.responsibilities.map(s=>`<li>${esc(s)}</li>`).join('')}</ul><h3>边界</h3><ul>${c.boundaries.map(s=>`<li>${esc(s)}</li>`).join('')}</ul></div><div><h3>验收条件</h3><ul>${c.acceptance.map(s=>`<li>${esc(s)}</li>`).join('')}</ul><p class="muted">现有实现 · ${c.files.map(s=>`<code>${esc(s)}</code>`).join(' ')}</p>${sourceHtml(c.sources)}</div></div>`;
    get('component-detail').insertAdjacentHTML('beforeend', `<h3 class="subheading">服务的用户用例</h3><div class="case-chips">${relatedCases(catalog,c.id).map(row=>`<button class="case-chip" data-use-case="${row.id}">${row.id} ${esc(row.title)}</button>`).join('')}</div>`);
    get('component-cards').querySelectorAll('button').forEach(b=>b.addEventListener('click',()=>selectComponent(b.dataset.id)));
  }
  function renderGraph() {
    get('component-graph').innerHTML=componentSvg(catalog,target);
    get('target-mode').classList.toggle('active',target);get('current-mode').classList.toggle('active',!target);
    get('component-graph').querySelectorAll('[data-component]').forEach(node=>{node.addEventListener('click',()=>selectComponent(node.dataset.component));node.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();selectComponent(node.dataset.component);}});});
  }
  get('target-mode').onclick=()=>{target=true;renderGraph();};get('current-mode').onclick=()=>{target=false;renderGraph();};renderGraph();selectComponent('guard');
  get('supporting').innerHTML=catalog.supporting.map(c=>`<article>${badge(c.status)}<h3>${esc(c.name)}</h3><p>${esc(c.responsibility)}</p></article>`).join('');
  get('decisions').innerHTML=catalog.decisions.map(d=>`<article class="decision"><div class="decision-code">${d.id}</div><div><h3>${esc(d.title)} ${badge(d.status||'review')}</h3><p>${esc(d.recommendation)}</p><p class="muted">${esc(d.reason)}</p></div></article>`).join('');
  get('flow-graph').innerHTML=flowSvg();
  get('lifecycle').innerHTML=catalog.lifecycle.map(row=>`<tr>${['phase','input','check','failure'].map(k=>`<td>${esc(row[k])}</td>`).join('')}</tr>`).join('');
  const categories={identity:'身份与版本',freshness:'输入新鲜度',quality:'质量与精度',budget:'执行预算',transport:'传输时限',start:'起点与静止',dynamics:'动力学',safety:'实时保护',position:'位置边界',contract:'协议完整性',recording:'记录完整性'};
  get('rule-module').insertAdjacentHTML('beforeend',[...catalog.components,...catalog.supporting].filter(c=>catalog.gates.some(g=>g.site===c.id)).map(c=>`<option value="${c.id}">${esc(c.name)}</option>`).join(''));
  get('rule-category').insertAdjacentHTML('beforeend',Object.entries(categories).map(([k,v])=>`<option value="${k}">${v}</option>`).join(''));
  get('rule-enforcement').insertAdjacentHTML('beforeend',[...new Set(catalog.gates.map(g=>g.enforcement))].map(v=>`<option>${esc(v)}</option>`).join(''));
  function renderRules() {
    const query=get('rule-search').value.trim().toLowerCase();
    const rows=catalog.gates.filter(g=>(!query||JSON.stringify(g).toLowerCase().includes(query))&&(!get('rule-module').value||g.site===get('rule-module').value)&&(!get('rule-category').value||g.category===get('rule-category').value)&&(!get('rule-enforcement').value||g.enforcement===get('rule-enforcement').value));
    get('rule-count').textContent=`${rows.length} / ${catalog.gates.length} 条规则`;
    get('rules').innerHTML=rows.length?rows.map(g=>`<article class="rule"><div class="rule-head"><span class="rule-id">${g.id}</span><strong>${esc(g.title)}</strong><span class="badge ${g.enforcement.includes('未启用')?'missing':g.enforcement.includes('诊断')?'partial':'neutral'}">${esc(g.enforcement)}</span><span class="muted">${esc(categories[g.category])}</span></div><div class="rule-grid"><div><small>当前值与来源范围</small><p>${esc(g.current)}</p><p class="muted">${esc(g.profile)}</p></div><div><small>检查阶段 / 执行点</small><p>${esc(g.phases.join(' · '))}</p><p class="muted">${esc(name(g.site))}</p></div><div><small>触发后的处理</small><p>${esc(g.action)}</p></div></div>${g.note?`<p class="rule-note">${esc(g.note)}</p>`:''}<p class="muted">目标字段 <code>${esc(g.policy_key)}</code></p>${sourceHtml(g.sources)}</article>`).join(''):'<div class="empty">没有符合筛选条件的规则</div>';
  }
  for(const id of ['rule-search','rule-module','rule-category','rule-enforcement'])get(id).addEventListener('input',renderRules);renderRules();
  get('interfaces').innerHTML=catalog.interfaces.map(i=>`<details class="contract" id="contract-${i.id}"><summary><span class="rule-id">${i.id}</span>${esc(i.name)}<small>${esc((i.producers||[i.producer]).map(name).join(' / '))} → ${esc((i.consumers||[i.consumer]).map(name).join(' / '))}</small></summary><div class="contract-body"><dl><dt>调用与传输</dt><dd>${esc(i.transport)}</dd><dt>当前协议</dt><dd>${esc(i.current)}</dd><dt>目标设计</dt><dd>${esc(i.target)}</dd><dt>时钟</dt><dd>${esc(i.clock)}</dd><dt>单位与坐标</dt><dd>${esc(i.units)}</dd><dt>失败与取消</dt><dd>${esc(i.failure)}</dd><dt>版本兼容</dt><dd>${esc(i.compatibility)}</dd></dl><h3>目标字段</h3><ul>${i.fields.map(f=>`<li>${esc(f)}</li>`).join('')}</ul>${fieldContractHtml(i)}${sourceHtml(i.sources)}</div></details>`).join('');
  get('decision-example').textContent=JSON.stringify({rule_id:'observation.max_age',phase:'before_execution',verdict:'reject',severity:'recoverable',action:'retain_hold',observed:526,limit:500,unit:'ms',policy_revision:7,policy_hash:'<frozen-hash>',run_id:'<run-id>',observation_id:'<observation-id>',reason:'结果已超过本次观测有效期限'},null,2);
  function renderParameterFields() {
    const groups=[...new Set(catalog.parameters.map(p=>p.group))];
    get('parameter-form').innerHTML=groups.map(group=>`<fieldset><legend>${esc(group)}</legend><div class="parameter-grid">${catalog.parameters.filter(p=>p.group===group).map(p=>`<label class="parameter-field">${esc(p.label)}<div class="input-unit"><input name="${p.key}" type="number" step="${p.integer?'1':'any'}" value="${p.value}" ${p.min!==undefined?`min="${p.min}"`:''} ${p.max!==undefined?`max="${p.max}"`:''} aria-label="${esc(p.label)}"><span>${esc(p.unit)}</span></div><small>${p.source} · ${esc(p.policy)}</small></label>`).join('')}</div></fieldset>`).join('');
  }
  function values() {return Object.fromEntries(catalog.parameters.map(p=>[p.key,Number(get('parameter-form').elements.namedItem(p.key).value)]));}
  get('recording-purpose').innerHTML=Object.entries(catalog.recording_design.purpose_profiles).map(([id,p])=>`<option value="${id}" ${id==='infer_debug'?'selected':''}>${esc(p.label)}</option>`).join('');
  let latestDraft;
  function updateDraft() {
    const data=values(),errors=validateDraft(data,catalog.parameters);
    latestDraft=buildDraft(data,{tracking:get('tracking-enabled').value,arrival:get('arrival-enabled').value,pairing:get('pairing-mode').value,recordingPolicy:recordingPolicyFor(catalog,get('recording-purpose').value)},catalog.revision,catalog.execution_contract);
    get('draft-errors').className=errors.length?'errors':'valid';get('draft-errors').textContent=errors.length?errors.join(' · '):'草案字段关系检查通过，设备能力与实际下发接口待后续阶段实现。';get('export-draft').disabled=!!errors.length;
    get('derived').innerHTML=`<div>源轨迹时长<strong>${Number.isFinite(data.steps)?latestDraft.execution.source_duration_s.toFixed(3):'—'} s</strong>N ÷ 30</div><div>单段最长时长<strong>${Number.isFinite(data.steps)?latestDraft.execution.max_segment_duration_s.toFixed(3):'—'} s</strong>源时长 × 4</div><div>零速供给等待上界<strong>${Number.isFinite(data.observation_wait+data.result_window)?data.observation_wait+data.result_window:'—'} ms</strong>观测等待 + 结果有效窗</div>`;
    get('recording-policy-preview').textContent=latestDraft.recording.failure_profile==='data_required'?'该用途要求记录完整，录制异常由Guard请求受控结束。':'该调试用途允许控制继续，记录异常明确标记缺口与不完整。';
    get('draft-json').textContent=JSON.stringify(latestDraft,null,2);
  }
  renderParameterFields();get('parameter-form').addEventListener('submit',e=>e.preventDefault());get('parameter-form').addEventListener('input',updateDraft);
  for(const id of ['tracking-enabled','arrival-enabled','pairing-mode','recording-purpose'])get(id).addEventListener('change',updateDraft);
  get('reset-draft').onclick=()=>{renderParameterFields();get('tracking-enabled').value='disabled';get('arrival-enabled').value='disabled';get('pairing-mode').value='observe';get('recording-purpose').value='infer_debug';updateDraft();};
  get('export-draft').onclick=()=>{if(validateDraft(values(),catalog.parameters).length)return;const blob=new Blob([JSON.stringify(latestDraft,null,2)+'\n'],{type:'application/json'});const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='control-policy-review-draft.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};updateDraft();
  get('progress').innerHTML=catalog.phases.map(p=>`<article class="phase" style="border-top-color:${catalog.statuses[p.status].color}"><span class="rule-id">${p.id}</span> ${badge(p.status)}<h3>${esc(p.name)}</h3><p class="owner">负责 · ${esc(p.owner)}</p><ul>${p.items.map(i=>`<li>${esc(i)}</li>`).join('')}</ul></article>`).join('');
  get('maintenance').innerHTML=catalog.maintenance.workflow.map(v=>`<li>${esc(v)}</li>`).join('');
  get('baseline').innerHTML=`<p>分支 <code>${esc(catalog.baseline.branch)}</code> · HEAD <code>${catalog.baseline.head.slice(0,12)}</code></p><p>实际部署源码 <code>${esc(catalog.baseline.deployed_root)}</code></p><p class="rule-note">部署源码与本地工作树仍待归并。门槛快照引用本次读取的部署源码和真实Run回执。</p><p>最后封存 Run <code>${catalog.baseline.latest_run.execution_id.slice(0,12)}</code>，执行 ${catalog.baseline.latest_run.execute_steps} 步，执行设置 revision ${catalog.baseline.latest_run.execution_settings.revision}。</p>`;
  get('runtime-json').textContent=JSON.stringify(catalog.runtime_snapshot.units,null,2);
  function view(route) {
    let [id,query='']=route.split('?');
    if(!document.getElementById('view-'+id))id='usecases';
    document.querySelectorAll('.view').forEach(v=>v.classList.toggle('active',v.id==='view-'+id));
    document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===id));
    history.replaceState(null,'','#'+id+(query?'?'+query:''));
    const params=new URLSearchParams(query);
    if(id==='progress'&&params.get('file'))get('development-map-frame').src='development-map.html?embedded=1#level=files&node='+encodeURIComponent(params.get('file'));
    if(id==='interfaces'&&params.get('contract')) {
      const entry=document.getElementById('contract-'+params.get('contract'));
      if(entry){entry.open=true;entry.scrollIntoView({block:'start'});return;}
    }
    window.scrollTo(0,0);
  }
  initUseCases(catalog,{get,openView:view});
  initRecording(catalog,get);
  initTraceability(catalog,{get});
  document.querySelectorAll('[data-view]').forEach(b=>b.onclick=()=>{location.hash='#'+b.dataset.view;});view(location.hash.slice(1)||'usecases');
  window.addEventListener('hashchange',()=>view(location.hash.slice(1)||'usecases'));
  get('loading').hidden=true;get('app').hidden=false;
}
if (typeof document !== 'undefined') boot();
