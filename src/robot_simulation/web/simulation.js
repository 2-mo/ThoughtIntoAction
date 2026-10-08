// Backend selection stays in the terminal; both modes share the same dashboard.
const meta = await fetch('/api/model').then(r => r.json());
document.body.dataset.simulation = String(Boolean(meta.simulation));
document.title = meta.simulation ? 'QuestArm · MuJoCo 遥操' : 'QuestArm · 真机';
if (meta.simulation) {
  document.getElementById('camera-title').textContent = '相机 · 实物参考';
  const heading = document.createElement('div');
  heading.id = 'simulation-heading';
  heading.innerHTML = '<strong>MuJoCo · Quest 遥操</strong><span id="simulation-status">等待连接…</span><span>右摇杆下拨 1 秒启用 · Grip 跟随 · Trigger 夹爪</span>';
  document.querySelector('.collection-panel').prepend(heading);
  let stopped = false;
  window.addEventListener('pagehide', () => {stopped = true;});
  async function poll() {
    try {
      const state = await fetch('/api/simulation', {cache:'no-store'}).then(r => r.json());
      const status = document.getElementById('simulation-status');
      status.textContent = state.message || '等待 Quest';
      status.dataset.state = state.state;
    } catch { document.getElementById('simulation-status').textContent = '连接中…'; }
    if (!stopped) setTimeout(poll, 500);
  }
  poll();
}
