const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));

export function recordingPolicyFor(catalog, purpose) {
  const profiles = catalog.recording_design.purpose_profiles;
  if (!Object.prototype.hasOwnProperty.call(profiles, purpose)) throw new Error('Unknown recording purpose');
  const profile = profiles[purpose];
  return {
    purpose,
    failure_profile: profile.failure_profile,
    failure_action: profile.failure_action,
    require_ready_before_start: profile.require_ready_before_start,
    stream_scope: 'all_samples_of_declared_streams',
    resource_profile: 'device_budget_to_be_frozen'
  };
}

export function recordingCapacity(example, bufferSeconds=2) {
  const rawBytesPerSecond = example.camera_count * example.width * example.height * example.channels * example.fps;
  return {rawBytesPerSecond, bufferMiB: rawBytesPerSecond * bufferSeconds / 1024 ** 2};
}

export function recordingSvg() {
  let svg='<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1100 555" role="img" aria-label="独立记录服务数据流" font-family="system-ui, PingFang SC, Microsoft Yahei, sans-serif"><defs><marker id="rec-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0 L8 4 L0 8" fill="#8299b5"/></marker></defs>';
  const box=(x,y,w,h,title,sub,fill='#f3f6fb',stroke='#c4d1e3')=>`<rect x="${x}" y="${y}" width="${w}" height="${h}" rx="10" fill="${fill}" stroke="${stroke}"/><text x="${x+15}" y="${y+28}" font-size="14" fill="#294968">${title}</text><text x="${x+15}" y="${y+50}" font-size="11" fill="#6d839f">${sub}</text>`;
  const arrow=(path,label,x,y,dashed=false)=>`<path d="${path}" fill="none" stroke="#8299b5" ${dashed?'stroke-dasharray="4 4"':''} marker-end="url(#rec-arrow)"/><text x="${x}" y="${y}" font-size="11" fill="#6c86a7">${label}</text>`;
  ['标定工作流','规则数采工作流','infer 工作流','控制研究 / 其他控制'].forEach((label,i)=>{svg+=box(20,28+i*93,190,68,label,'统一会话 + 业务阶段标记');svg+=arrow(`M210 ${62+i*93} H235 V198 H259`,'',0,0);});
  svg+=box(265,145,220,98,'共同控制事件发布口','固定上限事件 / 源序列 / 时间戳');
  svg+='<text x="280" y="221" font-size="11" fill="#416d8c">状态、指令、窗口与策略事件</text>';
  svg+=arrow('M485 193 H527','',0,0);
  svg+='<text x="283" y="132" font-size="11" fill="#6c86a7">有界非阻塞发布</text><rect x="535" y="96" width="340" height="245" rx="14" fill="#eaf2ff" stroke="#3565c9" stroke-width="2"/><text x="560" y="127" fill="#2e59a5" font-size="17">记录与产物服务 · 独立进程</text><text x="560" y="153" fill="#627fa7" font-size="12">CaptureSession / StreamRegistry</text>';
  svg+=box(555,174,300,65,'采集 / 编码 / 分片写盘','独立worker，CPU/内存/IO预算','#fff','#b9cce9');
  svg+=box(555,252,300,66,'序列核验 / 索引 / 异步封存','final_seq屏障与可恢复journal','#fff','#b9cce9');
  svg+=box(265,330,220,68,'唯一相机源分发','完整连续帧 + 共享FrameRef');
  svg+=arrow('M485 364 H510 V270 H527','',0,0);
  svg+=box(265,425,220,68,'模型产物与上下文','原始输入/输出引用 + 调用耗时');
  svg+=arrow('M485 459 H520 V310 H527','',0,0);
  svg+=box(925,170,155,92,'统一产物目录','视频 / 状态 / 指令');svg+='<text x="940" y="242" font-size="11" fill="#6d839f">模型 / 索引 / 终态</text>';
  svg+=arrow('M875 211 H919','',0,0);
  svg+=box(925,355,155,80,'Guard / 运维','用途策略与健康状态');svg+=arrow('M810 341 V394 H919','RecordingHealth',760,375,true);
  svg+=box(535,425,340,68,'控制面 · 应用层管理会话','prepare → bind → start → finish → sealed','#f7f9fc','#c4d1e3');
  svg+=arrow('M705 425 V348','',0,0,true);
  svg+='<text x="25" y="535" font-size="12" fill="#607c9d">源按声明频率全量发布；相机采集实例唯一；每流序列、时间、缺口和提交范围可追溯。</text>';
  return svg+'</svg>';
}

export function initRecording(catalog, get) {
  const design=catalog.recording_design;
  const capacity=recordingCapacity(design.capacity_example);
  const list=items=>`<ul>${items.map(item=>`<li>${esc(item)}</li>`).join('')}</ul>`;
  get('recording-content').innerHTML=`
    <div class="notice"><strong>第五个公共模块 · 中间产物采集与保存</strong><p>${esc(design.process_model)}</p><span class="badge review">服务设计待实现</span> <span class="badge confirmed">按用途故障策略已确认</span></div>
    <div class="panel"><h2>跨业务并行采集</h2><p>${esc(design.coverage_contract)}</p><div class="graph">${recordingSvg()}</div><div class="case-chips">${['UC17','UC18','UC19','UC20','UC21'].map(id=>`<button class="case-chip" data-use-case="${id}">${id} ${esc(catalog.use_cases.find(row=>row.id===id).title)}</button>`).join('')}</div></div>
    <div class="panel"><h2>统一采集的数据范围</h2><div class="table-scroll"><table><thead><tr><th>数据流</th><th>来源与范围</th><th>频率与完整性</th><th>契约</th></tr></thead><tbody>${design.streams.map(stream=>`<tr><td>${esc(stream.name)}</td><td><strong>${esc(stream.source)}</strong><p>${esc(stream.scope)}</p></td><td>${esc(stream.rate)}</td><td><button class="contract-chip" data-contract="${stream.contract}">${stream.contract}</button></td></tr>`).join('')}</tbody></table></div></div>
    <div class="panel"><h2>一次接入的统一生命周期</h2><div class="table-scroll"><table><thead><tr><th>阶段</th><th>调用</th><th>可核验结果</th></tr></thead><tbody>${design.protocol.map(row=>`<tr><td>${esc(row.stage)}</td><td><code>${esc(row.call)}</code></td><td>${esc(row.result)}</td></tr>`).join('')}</tbody></table></div><p class="muted">高频数据面与会话控制面分别设计，开始/结束由应用层编排，执行期间消费已绑定的发布句柄。</p></div>
    <div class="panel"><h2>控制时延与资源隔离</h2>${list(design.isolation)}<p class="rule-note">容量示例 · 三路640×480 RGB、各30fps约 ${Math.round(capacity.rawBytesPerSecond/1000000*100)/100} MB/s。源帧驻留两秒约 ${capacity.bufferMiB.toFixed(1)} MiB。${esc(design.capacity_example.note)}</p><div class="table-scroll"><table><thead><tr><th>验收项</th><th>测量内容</th><th>目标条件</th></tr></thead><tbody>${design.performance.map(row=>`<tr><td>${esc(row.item)}</td><td>${esc(row.measure)}</td><td>${esc(row.acceptance)}</td></tr>`).join('')}</tbody></table></div></div>
    <div class="panel"><h2>已确认的用途故障策略</h2><p class="muted">数采/标定要求完整，infer调试可继续控制并明确标记缺口。具体控制动作由公共Guard与执行器负责。${esc(design.approved_fault_policy.scope)}</p><div class="table-scroll"><table><thead><tr><th>用途</th><th>开始前</th><th>运行中故障</th><th>产物结果</th></tr></thead><tbody>${design.failure_profiles.map(profile=>`<tr><td><code>${profile.id}</code><p>${esc(profile.purpose)}</p></td><td>${esc(profile.before)}</td><td>${esc(profile.during)}</td><td>${esc(profile.artifact)}</td></tr>`).join('')}</tbody></table></div></div>
    <div class="panel"><h2>产物与恢复</h2><pre>${esc(design.artifact_layout.join('\n'))}</pre><p>${esc(design.migration)}</p><p class="muted">封存成功需要各源final_seq与已提交范围对应，异常范围、控制结果与记录结果独立可查。</p></div>`;
}
