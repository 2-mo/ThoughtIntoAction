// Independent device cards. A slow/disconnected feed cannot retain a live badge.
const el = id => document.getElementById(id);
const cards = new Map();
let latest = {}, receivedAt = 0, selected = null;
const streams = [['rgb', 'RGB', 'preview'], ['depth', '深度', 'depth_preview']];

function createCard(id) {
  const node = document.createElement('article');
  node.className = 'camera-card'; node.dataset.cameraId = id;
  node.innerHTML = `<header class="camera-card-heading"><div><h3></h3><span class="camera-model"></span></div><span class="camera-badge" role="status">未连接</span><button class="camera-toggle" type="button" aria-pressed="false" disabled>开启</button></header>
    <div class="camera-streams">${streams.map(([key, title]) => `<button class="camera-stream" data-stream="${key}" type="button"><span class="camera-stream-label">${title}</span><span class="camera-zoom">放大 ↗</span><img alt="" hidden><span class="camera-placeholder">等待画面</span>${key === 'depth' ? '<span class="camera-depth-scale">0.2 m <i></i> 2 m</span>' : ''}</button>`).join('')}</div>
    <div class="camera-card-footer"><p class="camera-status" role="status">等待相机数据</p><p class="camera-color-settings">RGB 参数：等待反馈</p><p class="camera-meta"></p></div><p class="camera-control-error" role="alert" hidden></p>`;
  if (id === 'wrist_rgb') {
    // Stable identifiers for the existing wrist recording and monitoring tools.
    node.querySelector('.camera-toggle').id = 'camera-toggle';
    node.querySelector('.camera-status').id = 'camera-status';
    node.querySelector('.camera-control-error').id = 'camera-control-error';
    for (const [key, prefix] of [['rgb', 'camera'], ['depth', 'depth']]) {
      node.querySelector(`[data-stream="${key}"] img`).id = prefix + '-image';
      node.querySelector(`[data-stream="${key}"] .camera-placeholder`).id = prefix + '-empty';
    }
  }
  const card = {node, busy: false};
  node.querySelector('.camera-toggle').onclick = async () => {
    card.busy = true;
    const error = node.querySelector('.camera-control-error'); error.hidden = true;
    draw();
    try {
      const response = await fetch('/api/camera', {method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({camera_id: id, enabled: latest[id]?.camera?.enabled !== true}), signal: AbortSignal.timeout(8000)});
      const result = await response.json();
      if (!response.ok || result.error) throw Error(result.error || '相机开关失败');
    } catch (exc) {error.textContent = exc.message; error.hidden = false;}
    finally {card.busy = false; draw();}
  };
  node.querySelectorAll('.camera-stream').forEach(button => {
    button.onclick = () => {
      selected = {id, key: button.dataset.stream};
      drawDialog(); el('camera-dialog').showModal();
    };
  });
  cards.set(id, card); el('camera-grid').append(node);
  return card;
}

function streamLive(feed, key) {
  const preview = feed?.[key];
  return !serviceError && !feed?.camera?.error && feed?.camera?.enabled !== false &&
    Number.isFinite(preview?.age_s) && preview.age_s >= 0 &&
    preview.age_s + (performance.now() - receivedAt) / 1000 < .5 && !!preview.jpeg_base64;
}
function displayImage(img, placeholder, feed, key, emptyText) {
  const live = streamLive(feed, key);
  img.hidden = !live; placeholder.hidden = live;
  placeholder.textContent = emptyText;
  if (live) {
    const source = 'data:image/jpeg;base64,' + feed[key].jpeg_base64;
    if (img.getAttribute('src') !== source) img.src = source;
  } else img.removeAttribute('src');
  return live;
}

let recording = false, serviceError = '';
function colorSettings(meta, full = false) {
  const c = meta?.rgb_controls;
  if (!c) return 'RGB 参数：未提供';
  const value = (name, scale = 1) => Number.isFinite(c[name]) ? String(Number((c[name] * scale).toFixed(2))) : '—';
  const exposure = Number.isFinite(meta.rgb_exposure_unit_us) ? value('exposure', meta.rgb_exposure_unit_us / 1000) + ' ms' : value('exposure') + '（设备单位）';
  const summary = [`白平衡 ${c.enable_auto_white_balance === 1 ? '自动' : value('white_balance') + ' K'}`,
    `曝光 ${c.enable_auto_exposure === 1 ? '自动' : exposure}`, `增益 ${c.enable_auto_exposure === 1 ? '自动' : value('gain')}`];
  if (full) {
    summary.push(...[['亮度','brightness'],['对比度','contrast'],['饱和度','saturation'],['Gamma','gamma'],['色调','hue'],['锐度','sharpness'],['背光补偿','backlight_compensation']].map(([label,key]) => `${label} ${value(key)}`));
    summary.push('电源频率 ' + ({0:'关闭',1:'50 Hz',2:'60 Hz',3:'自动'}[c.power_line_frequency] || '—'));
  }
  return summary.join(' · ');
}
function draw() {
  let online = 0;
  for (const [id, {node, busy}] of cards) {
    const feed = latest[id] || {}, camera = feed.camera || {}, meta = camera.metadata || {};
    const label = camera.label || (id === 'wrist_rgb' ? '腕部相机' : id);
    const model = (meta.model || camera.model || '').replace(/^RealSense\s+/i, '');
    const live = streamLive(feed, 'preview');
    if (live) online++;
    const enabled = camera.enabled !== false;
    const status = serviceError || (!enabled ? '相机已关闭' : camera.error ||
      (live ? 'RGB 正常' + (streamLive(feed, 'depth_preview') ? ' · 深度正常' : ' · 等待深度') : '等待新画面'));
    node.dataset.state = serviceError ? 'offline' : !enabled ? 'off' : live ? 'live' : camera.error ? 'error' : 'waiting';
    node.querySelector('h3').textContent = label;
    node.querySelector('.camera-model').textContent = [model, meta.fps ? `${meta.fps} FPS` : ''].filter(Boolean).join(' · ');
    node.querySelector('.camera-badge').textContent = ({live: '在线', off: '已关闭', offline: '未连接', error: '连接异常', waiting: '等待画面'})[node.dataset.state];
    const button = node.querySelector('.camera-toggle');
    button.setAttribute('aria-pressed', String(camera.enabled === true));
    button.setAttribute('aria-label', `${enabled ? '关闭' : '开启'}${label}`);
    button.disabled = busy || !!serviceError || (recording && enabled) || camera.switch_protocol !== 1;
    button.textContent = busy ? '切换中…' : enabled ? '关闭' : '开启';
    button.title = recording && enabled ? '录制期间不能关闭相机' : `${enabled ? '停止' : '开启'}这台相机的 RGB 和深度采集`;
    for (const [key, title, source] of streams) {
      const wrap = node.querySelector(`[data-stream="${key}"]`);
      const img = wrap.querySelector('img'); img.alt = `${label} · ${title}`;
      wrap.setAttribute('aria-label', `放大${label}的${title}画面`);
      displayImage(img, wrap.querySelector('.camera-placeholder'), feed, source,
        serviceError ? '相机服务未连接' : !enabled ? '采集已关闭' : camera.error ? '设备暂不可用 · 自动重连' : '等待新画面');
    }
    node.querySelector('.camera-status').textContent = status;
    const settings = node.querySelector('.camera-color-settings');
    settings.textContent = colorSettings(meta);
    settings.title = colorSettings(meta, true);
    const details = meta.width ? `${meta.width} × ${meta.height} · 采集 ${meta.fps} Hz · 预览 ${camera.preview_fps || 5} Hz` : '';
    node.querySelector('.camera-meta').textContent = [details, (meta.serial || camera.serial) ? `SN ${meta.serial || camera.serial}` : '', meta.usb_type ? `USB ${meta.usb_type}` : ''].filter(Boolean).join(' · ');
    node.title = [label, status, node.querySelector('.camera-meta').textContent, '点击画面放大'].join('\n');
  }
  el('camera-count').textContent = `${online} / ${cards.size} 在线`;
  el('camera-service-status').textContent = serviceError || (document.body.dataset.depth === 'true' ? '点击画面放大 · 深度为原生视角，未对齐 RGB' : '点击画面放大');
  el('camera-service-status').dataset.error = String(!!serviceError);
  if (selected) drawDialog();
}

function drawDialog() {
  if (!selected) return;
  const feed = latest[selected.id] || {};
  const key = selected.key === 'rgb' ? 'preview' : 'depth_preview';
  const name = `${feed.camera?.label || selected.id} · ${selected.key === 'rgb' ? 'RGB' : '深度'}`;
  el('camera-dialog-title').textContent = name;
  el('camera-dialog-image').alt = name;
  const live = displayImage(el('camera-dialog-image'), el('camera-dialog-empty'), feed, key,
    serviceError || feed.camera?.error || '等待新画面');
  el('camera-dialog-status').textContent = live ? selected.key === 'depth' ? '0.2–2 m · 黑色为无效深度 · 未对齐 RGB' : colorSettings(feed.camera?.metadata, true) : '画面不可用';
}

export function renderCameras(data) {
  serviceError = data.camera_error || '';
  const feeds = data.cameras || (data.camera ? {wrist_rgb: data} : null);
  recording = !!data.recording;
  receivedAt = performance.now();
  if (feeds) {
    latest = feeds;
    for (const id of cards.keys()) if (!Object.hasOwn(feeds, id)) {cards.get(id).node.remove(); cards.delete(id);}
    for (const id of Object.keys(feeds)) if (!cards.has(id)) createCard(id);
  } else {
    serviceError ||= '相机服务未连接';
    latest = Object.fromEntries([...cards.keys()].map(id => [id, {camera: {...latest[id]?.camera, enabled: undefined, error: serviceError}}]));
  }
  draw();
}

el('camera-dialog-close').onclick = () => el('camera-dialog').close();
el('camera-dialog').addEventListener('close', () => {selected = null; el('camera-dialog-image').removeAttribute('src');});
createCard('wrist_rgb');
renderCameras({camera_error: '等待相机服务连接'});
const watchdog = setInterval(() => {
  const expired = [...cards].some(([id, {node}]) => streams.some(([key, , source]) =>
    node.querySelector(`[data-stream="${key}"] img`).hasAttribute('src') && !streamLive(latest[id], source)));
  if (expired) draw();
}, 100);
window.addEventListener('pagehide', () => clearInterval(watchdog), {once: true});
