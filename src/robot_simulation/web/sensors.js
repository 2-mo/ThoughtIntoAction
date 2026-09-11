// Read-only session feed, independently polled so camera IPC never delays arm rendering.
const byId = (id) => document.getElementById(id);
let closed = false;
window.addEventListener('pagehide', () => { closed = true; });
const inputPanel = document.querySelector('.input-panel');
const stateLabels = { live: '输入在线', offline: '无输入', stale: '已过期', partial: '单侧输入' };
let inputDeadline = 0;
function inputState(state, detail = '') {
  inputPanel.dataset.state = state;
  byId('input-status').textContent = stateLabels[state];
  byId('input-status').title = detail;
}
function validHand(hand) {
  return hand?.pose?.length === 7 && hand.pose.every(Number.isFinite) &&
    Math.hypot(...hand.pose.slice(3)) > .1;
}
function renderHand(side, hand, feedback = {}, state = hand ? 'live' : 'offline') {
  const root = byId(side + '-hand');
  root.dataset.state = state;
  root.setAttribute('aria-label', (side === 'left' ? '左手柄：' : '右手柄：') + stateLabels[state]);
  if (!root.querySelector('.controller-shell')) {
    const keys = side === 'left' ? ['X', 'Y'] : ['A', 'B'];
    const ring = (key, content, label) => `<span class="hold-ring" data-hold="${key}" role="progressbar" aria-label="${label}" aria-valuemin="0" aria-valuemax="100" aria-valuenow="0">${content}</span>`;
    const stick = `<span class="controller-stick" ${side === 'right' ? 'data-key="R3"' : ''}><i></i></span>`;
    root.innerHTML = `<div class="controller-shell"><div class="hand-signal" aria-hidden="true"><span class="signal-mark"></span></div><div class="controller-face"><div class="key-unit">${side === 'right' ? ring('stick', stick, '摇杆长按进度') : stick}<small class="stick-label">摇杆</small></div>${keys.map(key => `<div class="key-unit">${key === 'B' ? ring('B', `<span class="controller-key" data-key="B">B</span>`, 'B 回零长按进度') : `<span class="controller-key" data-key="${key}">${key}</span>`}<small>${({A:'预抓取/采集', B:'预抓取/回零', X:'网页回放', Y:'真机回放'})[key]}</small></div>`).join('')}</div><div class="meters"></div></div><p class="controller-pose"></p>`;
    const labels = [['trigger', 'Trigger'], ['grip', 'Grip']];
    root.querySelector('.meters').innerHTML = labels.map(([key, label]) => `<label class="controller-meter">${label}<output data-meter="${key}" aria-label="${label}">0%</output></label>`).join('');
  }
  root.querySelectorAll('[data-key]').forEach(node => { node.dataset.pressed = String(!!hand?.buttons?.[node.dataset.key]); });
  root.querySelectorAll('[data-meter]').forEach(node => {
    const key = node.dataset.meter;
    const value = hand ? (hand[key] || 0) : 0;
    node.textContent = hand ? Math.round(value * 100) + '%' : '—';
    node.title = hand ? feedback[key]?.fired ? '长按已触发' : Math.round(value * 100) + '%' : '数据未连接';
  });
  root.querySelectorAll('[data-hold]').forEach(node => {
    const key = node.dataset.hold === 'stick' ? ((feedback.down?.progress || 0) > 0 ? 'down' : 'R3') : node.dataset.hold;
    const value = hand ? Math.max(0, Math.min(1, feedback[key]?.progress || 0)) : 0;
    node.style.setProperty('--hold', `${value * 360}deg`);
    node.setAttribute('aria-valuenow', String(Math.round(value * 100)));
    node.dataset.active = String(value > 0);
    node.title = !hand ? '数据未连接' : feedback[key]?.fired ? '长按已触发' : `${Math.round(value * 100)}%`;
    if (node.dataset.hold === 'stick') root.querySelector('.stick-label').textContent = key === 'down' ? '下拨切换' : '按住结束任务';
  });
  const axes = hand?.axes || [0, 0];
  root.querySelector('.controller-stick i').style.transform = `translate(${axes[0] * 10}px, ${-axes[1] * 10}px)`;
  root.querySelector('.controller-pose').textContent = hand ? 'XYZ ' + hand.pose.slice(0, 3).map(v => v.toFixed(2)).join(' / ') + ' m' : '数据未连接';
}
function clearInput(message, state = 'offline') {
  inputDeadline = 0;
  inputState(state, message);
  for (const side of ['left', 'right']) renderHand(side, null, {}, state);
  byId('button-response').textContent = '';
  byId('button-response').title = message;
}
function renderCollection(data = {}) {
  const collection = data.collection;
  const recording = data.recording;
  const last = data.last_episode;
  const error = collection?.error || recording?.error || data.runtime_error;
  const state = error ? 'ERROR' : collection?.state || (recording ? 'RECORDING' : data.session_id ? 'DISABLED' : 'OFFLINE');
  const labels = {REPLAY_POSITIONING: '真机回放 · 前往起点', REPLAYING: '真机回放 · 执行中', REPLAY_LOADING: '正在加载回放', PAUSED: '任务采集已暂停', IDLE: '任务采集未开启', POSITIONING: '前往预抓取位', READY: '准备就绪', RECORDING: '● 采集中', BUSY: '正在处理', HOME_WAIT: '等待保存 / 松开 Grip', HOMING: '正在回零', ERROR: '采集异常', DISABLED: '按键采集未启用', OFFLINE: '未连接'};
  document.querySelector('.collection-panel').dataset.state = state;
  const put = (id, text) => { byId(id).textContent = text; byId(id).title = text; };
  put('collection-state', labels[state] || state);
  put('control-state', collection?.control_stopping ? '平滑停车中' : collection?.control_enabled === true ? '控制已启用' : collection?.control_enabled === false ? '待机 · 未启用控制' : '控制状态未连接');
  byId('control-state').dataset.enabled = String(collection?.control_enabled === true);
  put('collection-count', String(data.collection_count ?? '—'));
  byId('collection-count').title = '本轮任务，自上次重置以来完整保存且未丢弃的条数；不代表任务成功';
  put('collection-task', recording?.task || collection?.task || '等待采集会话');
  const seconds = recording?.duration_s;
  put('collection-duration', Number.isFinite(seconds) ? Math.floor(seconds / 60).toString().padStart(2, '0') + ':' + Math.floor(seconds % 60).toString().padStart(2, '0') : '—');
  put('collection-frames', String(recording?.counts?.frame ?? '—'));
  put('collection-samples', String(recording?.counts?.state ?? '—'));
  put('collection-episode', '当前 Episode：' + (recording?.id || '—'));
  const drops = Object.values(recording?.dropped || {}).reduce((a, b) => a + b, 0);
  put('collection-health', error || (recording ? `写入队列 ${recording.queue_size ?? 0} · 丢弃样本 ${drops}` : state === 'IDLE' ? 'A 开始采集 · B 前往预抓取位' : state === 'PAUSED' ? 'A 恢复并开始采集 · B 前往预抓取位' : state === 'READY' ? 'A 开始下一条 · B 前往预抓取位' : state === 'OFFLINE' ? '等待采集会话' : '等待采集操作完成'));
  const dataset = data.task_dataset;
  if (dataset) {
    byId('collection-health').title = dataset.path || '';
    if (dataset.state === 'saving') put('collection-health', '正在打包任务 · ' + dataset.episodes + ' 条');
    else if (dataset.state === 'complete') put('collection-health', '可上传 · ' + dataset.episodes + ' 条 · ' + dataset.path.split('/').pop());
    else if (dataset.state === 'failed') put('collection-health', '任务打包失败：' + dataset.error);
  }
  const result = last ? (last.lifecycle !== 'complete' ? '未完整保存' : last.outcome === 'discarded' ? '已丢弃' : '已保存') : '—';
  put('collection-last', '上一条：' + result + (last ? ' · ' + last.id : ''));
}
async function refresh() {
  try {
    const response = await fetch('/api/sensors', { cache: 'no-store', signal: AbortSignal.timeout(1500) });
    if (!response.ok) throw new Error('监看接口未连接');
    const data = await response.json();
    window.dispatchEvent(new CustomEvent('teleop-preview', { detail: data }));
    for (const [key, id] of [['arm', 'arm-latency'], ['quest_usb', 'input-latency']]) {
      const value = data.connections?.[key]?.duration_ms;
      byId(id).textContent = !Object.hasOwn(data, 'connections') ? '需重启监看' :
        Number.isFinite(value) && value >= 0 ? value.toFixed(1) + ' ms' : '未测';
    }
    renderCollection(data);
    const safety = data.table_safety;
    byId('table-safety').hidden = !safety;
    if (safety) byId('table-safety').textContent = data.state_age_s == null || data.state_age_s >= .5 ? '桌面限制状态已过期' :
      '末端最低 Z ' + (safety.minimum_z_m * 100).toFixed(1) + ' cm · ' +
      (safety.blocked ? '已拦截越界路径，请抬高手柄或取消预设' : '桌面软限位已启用');
    const rates = data.loop_rates || {};
    const rateLive = data.state_age_s != null && data.state_age_s < .5;
    const rateText = key => {
      const rate = rates[key];
      if (!rate) return '—';
      const observed = rateLive && Number.isFinite(rate.observed_hz) ? rate.observed_hz.toFixed(1) : '—';
      return observed + '/' + rate.configured_hz;
    };
    byId('loop-rates').textContent = 'IK ' + rateText('ik') + ' Hz · 服务 ' + rateText('arm_service') + ' Hz';
    const limit = data.joint_limit;
    byId('motion-joints').textContent = limit?.age_s < .5 && limit.joints.length ?
      '限位 J' + limit.joints.join('/J') + ' · 反向移动可恢复' : '关节快照（悬停查看）';
    const diagnostic = data.motion_diagnostics;
    const diagnosticLive = diagnostic && diagnostic.age_s != null && diagnostic.age_s < .5;
    byId('motion-diagnostics').textContent = diagnosticLive ?
      'IK 步长 ' + diagnostic.peak_ik_step.toFixed(4) + ' · 积压 ' + diagnostic.ik_backlog.toFixed(4) +
      ' · 跟踪 ' + diagnostic.command_tracking_error.toFixed(4) : '—';
    byId('motion-joints').title = diagnosticLive ?
      ['q_actual', 'q_des', 'q_safe'].map(k => k + ': ' + diagnostic[k].map(v => v.toFixed(4)).join(' ')).join('\n') : '无新诊断数据';
    const depth = data.depth_preview || {};
    const depthLive = !data.camera?.error && depth.age_s != null && depth.age_s < .5 && depth.jpeg_base64;
    byId('depth-image').hidden = !depthLive;
    byId('depth-empty').hidden = !!depthLive;
    if (depthLive) byId('depth-image').src = 'data:image/jpeg;base64,' + depth.jpeg_base64;
    else byId('depth-image').removeAttribute('src');
    const preview = data.preview || {};
    const cameraLive = !data.camera?.error && preview.age_s != null && preview.age_s < .5 && preview.jpeg_base64;
    byId('camera-image').hidden = !cameraLive;
    byId('camera-empty').hidden = !!cameraLive;
    if (cameraLive) byId('camera-image').src = 'data:image/jpeg;base64,' + preview.jpeg_base64;
    else byId('camera-image').removeAttribute('src');
    byId('camera-status').textContent = data.error || data.camera?.error ||
      'RGB：' + (cameraLive ? '正常' : '暂无新画面') + ' · 深度：' + (depthLive ? '正常' : '暂无新画面');
    byId('camera-status').title = byId('camera-status').textContent;
    const input = data.input;
    if (!input || input.error || data.input_error || !Number.isFinite(data.input_age_s)) {
      clearInput(data.input_error || input?.error || '等待手柄新帧');
    } else if (data.input_age_s >= .5 || !Number.isFinite(data.state_age_s) || data.state_age_s >= .5) {
      clearInput('手柄输入数据已过期', 'stale');
    } else {
      const hands = Object.fromEntries(['left', 'right'].map(side => [side, validHand(input.hands?.[side]) ? input.hands[side] : null]));
      const count = Object.values(hands).filter(Boolean).length;
      inputState(count === 2 ? 'live' : count === 1 ? 'partial' : 'offline', '依据手柄输入帧的新鲜度；不代表机械臂控制已启用');
      inputDeadline = count ? performance.now() + 1000 * (.5 - Math.max(data.input_age_s, data.state_age_s)) : 0;
      for (const side of ['left', 'right']) renderHand(side, hands[side], data.button_feedback || {});
      byId('button-response').textContent = data.button_feedback?.Y || '需重启遥操以显示实际按键响应和长按进度';
    }
  } catch {
    window.dispatchEvent(new CustomEvent('teleop-preview', { detail: null }));
    renderCollection();
    byId('table-safety').textContent = '桌面限制状态未连接';
    byId('loop-rates').textContent = 'IK — Hz · 服务 — Hz';
    byId('arm-latency').textContent = byId('input-latency').textContent = '未测';
    byId('motion-joints').textContent = '关节快照（悬停查看）';
    byId('depth-image').hidden = true;
    byId('depth-image').removeAttribute('src');
    byId('depth-empty').hidden = false;
    byId('motion-diagnostics').textContent = '—';
    byId('motion-joints').title = '无新诊断数据';
    byId('camera-image').hidden = true;
    byId('camera-image').removeAttribute('src');
    byId('camera-empty').hidden = false;
    byId('camera-status').textContent = '监看接口未连接';
    clearInput('未连接');
  } finally {
    if (!closed) setTimeout(refresh, document.hidden ? 1000 : 200);
  }
}
clearInput('等待手柄新帧');
refresh();
// A stalled HTTP request must not leave the last green frame on screen.
const inputWatchdog = setInterval(() => {
  if (inputDeadline && performance.now() >= inputDeadline) clearInput('手柄输入数据已过期', 'stale');
}, 100);
window.addEventListener('pagehide', () => clearInterval(inputWatchdog));

// Keep operating guidance available on hover without taking dashboard space.
document.querySelectorAll('.hint, .collection-controls').forEach(note => {
  const section = note.closest('section');
  if (section) section.title = [section.title, note.textContent.trim()].filter(Boolean).join('\n');
});
