// Phone status and explicit ownership commands share the independent receiver.
const byId = id => document.getElementById(id);
const panel = document.querySelector('.phone-panel');
let state = {}, pending = false, closed = false, received = 0, serviceMessage = false;
const put = (id, value) => { byId('phone-' + id).textContent = value; };
const message = (text, error = false) => {
  put('message', text);
  byId('phone-message').dataset.error = String(error);
};
function render(value) {
  state = value;
  const fresh = value.connected === true;
  panel.dataset.control = value.enabled ? value.active ? 'active' : 'enabled' : 'idle';
  byId('phone-control-banner').hidden = !value.enabled;
  byId('phone-control-banner').textContent = value.active ? '● 手机遥操已开启 · 正在跟随' : '● 手机遥操已开启 · 松手保持';
  panel.dataset.state = value.estop || (value.error && value.available) ? 'error' : fresh
    ? value.tracking === 'normal' ? 'live' : value.tracking === 'limited' ? 'warning' : 'error' : 'offline';
  put('status', !value.available ? '接收服务未启动' : !fresh ? '等待手机连接' : value.enabled ? '已接管' : '仅监看');
  put('endpoint', value.addresses?.length ? 'App 服务器 ' + value.addresses.map(host => `${host}:${value.port}`).join(' / ') : '手机与电脑使用可互通的网络 · UDP 5005');
  put('tracking', fresh ? ({normal: '正常', limited: '受限', unavailable: '无追踪'}[value.tracking] || '—') : '—');
  put('fps', fresh && Number.isFinite(value.fps) ? `${value.fps.toFixed(1)} Hz` : '—');
  put('age', fresh && Number.isFinite(value.age_s) ? `${(value.age_s * 1000).toFixed(0)} ms` : '—');
  put('control', value.enabled ? '手机持有' : '仅监看');
  put('clutch', fresh ? value.clutch ? '按住' : '松开' : '—');
  put('gripper', fresh && Number.isFinite(value.gripper) ? `${(value.gripper * 100).toFixed(0)}%` : '—');
  put('position', fresh && value.position ? `XYZ ${value.position.map(v => v.toFixed(3)).join(' / ')} m` : 'XYZ —');
  put('mapping', Number.isFinite(value.position_scale) ? `相对控制 · 位移 ${value.position_scale.toFixed(2)}× · 旋转 ${Number.isFinite(value.rotation_scale) ? value.rotation_scale.toFixed(2) + '×' : '—'}` : '相对控制');
  byId('phone-start').disabled = pending || value.available;
  byId('phone-enable').disabled = pending || !fresh || value.enabled || value.estop || value.clutch || value.tracking !== 'normal' || !value.kernel_runtime_id;
  byId('phone-disable').disabled = pending || !value.enabled;
  if (value.error && value.available) { message(value.error, true); serviceMessage = true; }
  else if (value.estop) { message('手机停止已锁存；松开 Clutch 后在手机点恢复。', true); serviceMessage = true; }
  else if (fresh && value.tracking !== 'normal') { message('追踪受限：请保持相机画面清晰，观察有纹理的环境。'); serviceMessage = true; }
  else if (value.target_limited) { message('目标跟随误差较大，超出部分已丢弃；可反向移动或松开重新定位。'); serviceMessage = true; }
  else if (serviceMessage || byId('phone-message').dataset.error !== 'true') {
    message(value.enabled ? '按住跟随，上滑开夹爪、下滑闭夹爪；松手保持。' : '仅监看；可在手机、终端或本页开始遥操。');
    serviceMessage = false;
  }
  window.dispatchEvent(new CustomEvent('phone-preview', {detail: value}));
}
async function poll() {
  try {
    const response = await fetch('/api/phone', {cache: 'no-store', signal: AbortSignal.timeout(1500)});
    if (!response.ok) throw new Error('手机状态接口未连接');
    render(await response.json());
    received = performance.now();
  } catch (error) {
    render({available: false});
    message(error.message, true);
  } finally { if (!closed) setTimeout(poll, 300); }
}
for (const action of ['start', 'enable', 'disable']) {
  byId('phone-' + action).addEventListener('click', async () => {
    pending = true; render(state);
    try {
      const response = await fetch('/api/phone', {method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({action, runtime_id: state.kernel_runtime_id}), signal: AbortSignal.timeout(6000)});
      const result = await response.json();
      if (!response.ok || result.error) throw new Error(result.error || '操作未完成');
      message({start: '接收已启动。打开 RobotTeleop，填写上方地址。', enable: '手机已接管。先松开，再按住 Clutch 移动。', disable: '已停止手机遥操，继续监看输入。'}[action]);
    } catch (error) { message(error.message, true); }
    finally { pending = false; render(state); }
  });
}
const expiry = setInterval(() => {
  if (received && performance.now() - received > 1800) render({available: false});
}, 300);
window.addEventListener('pagehide', () => { closed = true; clearInterval(expiry); });
poll();
