// Independent camera view. Device selection never submits an Owner operation.
export function startSiteCameras({document: doc, fetch: request, location, URL: urls = URL}) {
  const nav = doc.querySelector('nav.tabs');
  if (!nav) return;
  let tab = nav.querySelector('[data-tab="site-cameras"]');
  if (!tab) { tab = doc.createElement('button'); tab.dataset.tab = 'site-cameras'; tab.textContent = '现场相机'; nav.append(tab); }
  const style = doc.createElement('style');
  style.textContent = `
    #site-cameras-pane{padding:24px;margin:20px 0;background:#fff;border:1px solid #d5deea;border-radius:12px;color:#233650;font:14px/1.6 system-ui,sans-serif}
    #site-cameras-pane [hidden]{display:none!important}#site-cameras-pane h2{margin:0;font-size:22px}#site-cameras-pane p{margin:10px 0}
    .site-camera-toolbar,.site-camera-form{display:flex;flex-wrap:wrap;align-items:end;gap:12px}.site-camera-toolbar label,.site-camera-form label{display:grid;gap:4px}
    #site-cameras-pane input,#site-cameras-pane select,#site-cameras-pane button{font:inherit;padding:8px 12px;border:1px solid #cbd6e4;border-radius:6px;background:#f7f9fc;color:#233650;min-height:40px}
    #site-cameras-pane button{cursor:pointer}#site-cameras-pane input[type=checkbox]{min-height:initial}#site-cameras-pane input:focus-visible,#site-cameras-pane select:focus-visible,#site-cameras-pane button:focus-visible{outline:3px solid #74a6d5;outline-offset:2px}
    .site-camera-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px;margin-top:20px}.site-camera-grid[data-focus=single]{grid-template-columns:minmax(0,1fr);max-width:960px}.site-camera-card{border:1px solid #d5deea;border-radius:10px;overflow:hidden;position:relative}
    .site-camera-card h3{font-size:16px;padding:12px 16px;margin:0;background:#f7f9fc}.site-camera-image{position:relative;background:#192b40;aspect-ratio:4/3;display:grid;place-items:center;overflow:hidden}
    .site-camera-image img{width:100%;height:100%;object-fit:contain;position:absolute}.site-camera-image span{color:white;z-index:1;padding:10px;background:#192b40c9;text-align:center}
    .site-camera-card.stale img{opacity:.35}.site-camera-meta{padding:10px 14px;font-size:13px;color:#61738a;min-height:54px}
    #site-camera-state{padding:12px 16px;background:#f0f5fa;border-radius:7px}#site-camera-state.bad{background:#fff0ee;color:#a43436}
    #site-camera-message{color:#a43436}#site-cameras-pane details{margin-top:16px;padding:12px;background:#f7f9fc;border-radius:8px}#site-cameras-pane summary{cursor:pointer;margin-bottom:8px}
    #site-cameras-pane .note{color:#61738a;font-size:13px}body.site-cameras-active #execution-settings-tools{display:none!important}
    body.site-cameras-standalone{margin:0;background:#f0f3f8}body.site-cameras-standalone main{max-width:1300px;padding:20px;margin:auto}body.site-cameras-standalone nav{display:flex;gap:20px;align-items:center;font:16px system-ui}
    @media(max-width:850px){.site-camera-grid{grid-template-columns:1fr}#site-cameras-pane{padding:16px}.site-camera-toolbar>*{max-width:100%}}
  `;
  doc.head.append(style);
  const pane = doc.createElement('section'); pane.id = 'site-cameras-pane'; pane.hidden = true;
  pane.setAttribute('aria-label', '多 ARX 现场相机监控');
  pane.innerHTML = `
    <h2>现场相机 · ARX 实时视野</h2>
    <p class="note">独立相机服务。可单独启动并连接图像进程；切换监控设备不会切换顶部控制设备或交接机械臂控制权。</p>
    <div class="site-camera-toolbar">
      <label>监控设备<select id="site-camera-device" aria-label="监控 ARX 设备"></select></label>
      <label>查看范围<select id="site-camera-focus"><option value="all">头部与双腕</option><option value="base_0_rgb">头部相机</option><option value="left_wrist_0_rgb">左腕相机</option><option value="right_wrist_0_rgb">右腕相机</option></select></label>
      <button id="site-camera-start" disabled>启动并连接图像进程</button><button id="site-camera-refresh">重新读取</button><button id="site-camera-pause">暂停画面</button><button id="site-camera-edit">编辑此设备</button>
    </div>
    <p id="site-camera-state" role="status" aria-live="polite">正在读取设备列表…</p>
    <div class="site-camera-grid"></div>
    <p class="note">每秒获取最新预览；源帧龄来自设备的相机状态。显示陈旧或离线时，保留的旧图会变暗并明确标注，不代表现场当前状态。</p>
    <details id="site-camera-config"><summary>添加或修改监控设备</summary>
      <form class="site-camera-form" id="site-camera-form">
        <label>设备名称<input id="site-camera-label" required maxlength="60" placeholder="例如 ARX6"></label>
        <label>服务器内网 IP<input id="site-camera-address" required placeholder="10.17.x.x" inputmode="decimal"></label>
        <label>只读预览桥端口<input id="site-camera-port" type="number" min="1024" max="65535" value="18873" required></label>
        <button type="submit" id="site-camera-save">保存设备</button><button type="button" id="site-camera-new">新增设备</button><button type="button" id="site-camera-remove" hidden>移出监控列表</button>
      </form><p class="note">对应 ARX 需已部署相机服务。保存地址不启动设备；上方启动按钮仅管理图像进程，不接管机械臂或结束其他采集任务。</p>
    </details><p id="site-camera-message" role="status"></p>
  `;
  nav.after(pane);
  const el = id => doc.getElementById(id);
  const keys = ['base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb'];
  const cards = new Map();
  for (const [index, key] of keys.entries()) {
    const card = doc.createElement('article'); card.className = 'site-camera-card stale';
    const title = doc.createElement('h3'); title.textContent = ['头部相机', '左腕相机', '右腕相机'][index];
    const visual = doc.createElement('div'); visual.className = 'site-camera-image';
    const image = doc.createElement('img'); image.alt = title.textContent + '现场画面'; image.hidden = true;
    const overlay = doc.createElement('span'); overlay.textContent = '等待实时画面';
    const meta = doc.createElement('div'); meta.className = 'site-camera-meta'; meta.textContent = '尚未核验';
    visual.append(image, overlay); card.append(title, visual, meta); pane.querySelector('.site-camera-grid').append(card);
    cards.set(key, {card, image, overlay, meta, blob: null, received: null});
  }
  let devices = [], selected = '', source = '', csrf = '', editing = null;
  let active = false, paused = false, generation = 0, timer = null, flight = null, starting = false;
  const requested = new URLSearchParams(location.search).get('arx');
  function stale(message, clear = false) {
    for (const slot of cards.values()) {
      slot.card.classList.add('stale'); slot.overlay.hidden = false; slot.overlay.textContent = message;
      if (!clear) slot.meta.textContent = slot.received ? '上次图像接收：' + new Date(slot.received).toLocaleTimeString() : '尚未接收此设备画面';
      if (clear) { if (slot.blob) urls.revokeObjectURL(slot.blob); slot.blob = null; slot.image.removeAttribute('src'); slot.image.hidden = true; slot.received = null; slot.meta.textContent = '尚未接收此设备画面'; }
    }
  }
  function cancel() { generation++; clearTimeout(timer); timer = null; flight?.abort(); flight = null; }
  function state(message, bad = false) { el('site-camera-state').textContent = message; el('site-camera-state').classList.toggle('bad', bad); }
  function choose(id) {
    cancel(); selected = id; source = devices.find(x => x.id === id)?.source || ''; stale('等待当前设备画面', true);
    el('site-camera-start').disabled=true;
    el('site-camera-device').value = selected; el('site-camera-edit').disabled = !selected;
    if (active && !paused) void poll();
  }
  async function catalog(preferred) {
    const response = await request('/api/site-cameras', {cache: 'no-store'});
    if (!response.ok) throw Error('现场相机服务尚未配置或不可达（HTTP ' + response.status + '）');
    const value = await response.json(); devices = value.devices; csrf = value.csrf;
    el('site-camera-device').replaceChildren();
    if (!devices.length) { const option = doc.createElement('option'); option.textContent = '尚未添加 ARX'; option.value = ''; el('site-camera-device').append(option); }
    for (const device of devices) { const option = doc.createElement('option'); option.value = device.id; option.textContent = `${device.label} · ${device.address}:${device.port}`; el('site-camera-device').append(option); }
    choose(devices.some(x => x.id === preferred) ? preferred : devices.some(x => x.id === selected) ? selected : devices[0]?.id || '');
    if (!selected) { state('尚未配置监控设备，请添加 ARX 地址。'); el('site-camera-config').open = true; }
  }
  async function poll() {
    if (!active || paused || doc.hidden || !selected || flight) return;
    const at = generation, deviceId = selected, controller = new AbortController(); flight = controller;
    const valid = () => at === generation && deviceId === selected && active && !controller.signal.aborted;
    try {
      const response = await request(`/api/site-cameras/${deviceId}/streams`, {cache: 'no-store', signal: controller.signal});
      if (!response.ok) throw Error('设备状态读取失败（HTTP ' + response.status + '）');
      const view = await response.json(); if (!valid()) return;
      const lifecycle=view.lifecycle;
      el('site-camera-start').disabled=starting||!lifecycle?.available||lifecycle.phase==='starting';
      el('site-camera-start').textContent=starting||lifecycle?.phase==='starting'?'图像进程启动中…':'启动并连接图像进程';
      if (view.device.source !== source) { source = view.device.source; stale('设备地址已更新，等待新画面', true); }
      if (!view.reachable) throw Error('网络或预览服务连接中断：' + (view.error || '相机预览桥不可达'));
      const live = view.streams.filter(x => x.status === 'live').length;
      const sourceStopped = view.streams.some(x => x.source_state === 'stopped' || x.source_state === 'error');
      const sourceError = view.streams.some(x => x.source_state === 'error');
      const sourceStarting = view.streams.some(x => x.source_state === 'starting');
      const sourceNote = lifecycle?.phase==='starting'?'图像进程正在启动，等待三路首帧':lifecycle?.phase==='stopped'?'图像进程未启动 · 点击启动并连接图像进程':sourceError ? '源相机服务故障 · 网络连接正常' : sourceStopped ? '源相机已停止 · 网络连接正常' : sourceStarting ? '源相机正在启动' : `实时 ${live}/3`;
      state(`${view.device.label} · ${view.device.address} · ${sourceNote} · 状态核验 ${new Date().toLocaleTimeString()}`, live !== 3 || sourceStopped);
      if(lifecycle?.error)el('site-camera-message').textContent='图像进程启动失败：'+lifecycle.error;
      el('site-camera-start').title=lifecycle?.available?'仅管理图像进程；已运行时直接连接，不重启采集':'此设备尚未部署独立图像启动入口';
      const focus = el('site-camera-focus').value;
      await Promise.all(view.streams.map(async row => {
        const slot = cards.get(row.key); if (!slot) return;
        slot.card.hidden = focus !== 'all' && focus !== row.key;
        if (slot.card.hidden) return;
        slot.meta.textContent = `${view.device.label} · ${row.age_s === null ? '源帧龄未知' : '源帧龄 ' + row.age_s.toFixed(2) + ' 秒'}${row.fps === null ? '' : ' · 源预览约 ' + row.fps.toFixed(1) + ' FPS'}`;
        if (row.status !== 'live' || sourceStopped) { slot.card.classList.add('stale'); slot.overlay.hidden = false; slot.overlay.textContent = lifecycle?.phase==='stopped'?'图像进程尚未启动':lifecycle?.phase==='starting'?'图像进程启动中，等待首帧':sourceError ? '源相机服务故障' : sourceStopped ? '源相机已停止' : row.source_state === 'starting' ? '相机正在启动' : row.status === 'missing' ? '此相机未发布' : '源画面未更新'; return; }
        let blob;
        try {
          const frameSource = source;
          const frame = await request(`/api/site-cameras/${deviceId}/frame/${row.key}.jpg?source=${encodeURIComponent(frameSource)}`, {cache: 'no-store', signal: controller.signal});
          if (!frame.ok) throw Error('图像读取失败（HTTP ' + frame.status + '）');
          if (frame.headers.get('X-Camera-Source') !== frameSource || !frame.headers.get('Content-Type')?.startsWith('image/jpeg')) throw Error('图像来源已变化，等待重新核验');
          blob = urls.createObjectURL(await frame.blob());
          const decoded = doc.createElement('img'); decoded.src = blob; await decoded.decode();
          if (!valid() || source !== frameSource) { urls.revokeObjectURL(blob); return; }
          const old = slot.blob; slot.blob = blob; slot.image.src = blob; slot.image.hidden = false;
          if (old) urls.revokeObjectURL(old);
          slot.received = Date.now(); slot.overlay.hidden = true; slot.card.classList.remove('stale');
        } catch (error) {
          if (blob && blob !== slot.blob) urls.revokeObjectURL(blob);
          if (!valid()) return;
          slot.card.classList.add('stale'); slot.overlay.hidden = false; slot.overlay.textContent = error.message;
        }
      }));
    } catch (error) {
      if (valid()) { state(error.message + '。请核对设备地址及只读预览桥，或点击重新读取。', true); stale('当前画面不可用 · 旧图仅作参考'); }
    } finally {
      if (flight === controller) flight = null;
      if (valid() && !paused) timer = setTimeout(poll, 1000);
    }
  }
  function open(on) {
    active = on; pane.hidden = !on; doc.body.classList.toggle('site-cameras-active', on);
    cancel();
    if (on) { tab.setAttribute('aria-current', 'page'); if (!devices.length) void catalog(requested).catch(error => state(error.message, true)); else if (!paused) void poll(); }
  }
  for (const button of nav.querySelectorAll('[data-tab]')) button.addEventListener('click', () => open(button === tab));
  el('site-camera-device').addEventListener('change', () => choose(el('site-camera-device').value));
  el('site-camera-start').addEventListener('click',async()=>{
    const deviceId=selected,expectedSource=source;
    if(starting||!deviceId)return;
    starting=true;cancel();el('site-camera-start').disabled=true;el('site-camera-message').textContent='';
    try{
      const response=await request(`/api/site-cameras/${deviceId}/start`,{method:'POST',headers:{'Content-Type':'application/json','X-Site-Camera-Token':csrf},body:JSON.stringify({source:expectedSource})});
      const result=await response.json();if(!response.ok)throw Error(result.error||'启动图像进程失败');
      if(deviceId===selected&&expectedSource===source){paused=false;el('site-camera-pause').textContent='暂停画面';state(result.message||'图像进程启动中，等待首帧');}
    }catch(error){if(deviceId===selected&&expectedSource===source)el('site-camera-message').textContent=error.message;}
    finally{starting=false;if(deviceId===selected&&expectedSource===source){el('site-camera-start').disabled=false;void poll();}}
  });
  el('site-camera-focus').addEventListener('change', () => { cancel(); pane.querySelector('.site-camera-grid').dataset.focus = el('site-camera-focus').value === 'all' ? 'all' : 'single'; for (const [key, slot] of cards) slot.card.hidden = el('site-camera-focus').value !== 'all' && el('site-camera-focus').value !== key; void poll(); });
  el('site-camera-pause').addEventListener('click', () => { paused = !paused; cancel(); el('site-camera-pause').textContent = paused ? '恢复画面' : '暂停画面'; if (paused) { state('已暂停刷新；下方是上次接收画面。'); stale('刷新已暂停'); } else void poll(); });
  el('site-camera-refresh').addEventListener('click', () => { cancel(); void catalog(selected).catch(error => state(error.message, true)); });
  function edit(device) {
    editing = device?.id || null; el('site-camera-label').value = device?.label || ''; el('site-camera-address').value = device?.address || ''; el('site-camera-port').value = device?.port || 18873;
    el('site-camera-remove').hidden = !editing; el('site-camera-config').open = true; el('site-camera-message').textContent = '';
  }
  el('site-camera-edit').addEventListener('click', () => edit(devices.find(x => x.id === selected)));
  el('site-camera-new').addEventListener('click', () => edit(null));
  async function save(operation, device) {
    const response = await request('/api/site-cameras', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Site-Camera-Token': csrf}, body: JSON.stringify({operation, device})});
    const result = await response.json(); if (!response.ok) throw Error(result.error || '保存失败');
    await catalog(operation === 'save' ? result.devices.find(x => x.address === device.address && x.port === device.port)?.id : undefined);
    edit(null); el('site-camera-config').open = false;
    el('site-camera-message').textContent = operation === 'remove' ? '已移出监控列表；设备与采集服务继续独立运行。' : '监控配置已保存。';
  }
  el('site-camera-form').addEventListener('submit', event => {
    event.preventDefault(); el('site-camera-save').disabled = true;
    const device = {label: el('site-camera-label').value.trim(), address: el('site-camera-address').value.trim(), port: Number(el('site-camera-port').value)};
    if (editing) device.id = editing;
    void save('save', device).catch(error => el('site-camera-message').textContent = error.message).finally(() => el('site-camera-save').disabled = false);
  });
  el('site-camera-remove').addEventListener('click', () => { if (editing) void save('remove', {id: editing}).catch(error => el('site-camera-message').textContent = error.message); });
  doc.addEventListener('visibilitychange', () => { cancel(); if (!doc.hidden && active && !paused) void poll(); });
  const standalone = location.pathname === '/site-cameras';
  if (standalone) doc.body.classList.add('site-cameras-standalone');
  if (standalone || new URLSearchParams(location.search).get('view') === 'site-cameras') tab.click();
  return {open, destroy() { cancel(); stale('已关闭', true); }};
}

if (typeof document !== 'undefined') startSiteCameras({document, fetch: window.fetch.bind(window), location: window.location});
