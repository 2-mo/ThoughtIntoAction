import {mode, replayFrame, setMode, setReplay} from './display-state.js';
const el = id => document.getElementById(id);
const panel = el('dataset-replay');
let closed = false, timer, generation = null, commandError = null;
async function listEpisodes() {
  try {
    const result = await (await fetch('/api/replay?list=1', {cache: 'no-store'})).json();
    if (result.error) throw Error(result.error);
    const selected = el('replay-episodes').value;
    el('replay-episodes').replaceChildren(...(result.episodes || []).map(episode => {
      const option = document.createElement('option'); option.value = episode.id;
      option.textContent = episode.id; option.title = episode.task || ''; return option;
    }));
    if ([...el('replay-episodes').options].some(o => o.value === selected)) el('replay-episodes').value = selected;
  } catch (error) { el('replay-status').textContent = '数据列表不可用：' + error.message; }
}
function open() { setMode('replay'); listEpisodes(); render(); }
el('open-replay').onclick = open;
el('view-live').onclick = () => { setMode('live'); render(); };
async function command(action, position_s, episode_id, speed) {
  try {
    const response = await fetch('/api/replay', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({action, position_s, episode_id, speed}), signal: AbortSignal.timeout(6000)});
    if (!response.ok) throw Error(`回放请求失败（HTTP ${response.status}）`);
    const result = await response.json(); if (result.error) throw Error(result.error); commandError = null;
  } catch (error) { commandError = error.message; el('replay-status').textContent = error.message; }
}
el('replay-load').onclick = () => command('load', undefined, el('replay-episodes').value);
el('replay-last').onclick = () => command('load_last');
el('replay-play').onclick = () => command('play');
el('replay-pause').onclick = () => command('pause');
el('replay-seek').onchange = event => command('seek', Number(event.target.value));
el('replay-speed').onchange = event => command('speed', undefined, undefined, Number(event.target.value));
function render() {
  const reviewing = mode === 'replay', f = replayFrame;
  panel.hidden = !reviewing;
  el('recorded-camera').hidden = !reviewing;
  el('camera-title').textContent = reviewing ? 'D435i · 录制 RGB' : 'D435i · RGB';
  el('view-live').setAttribute('aria-pressed', String(!reviewing));
  el('open-replay').setAttribute('aria-pressed', String(reviewing));
  for (const id of ['replay-play', 'replay-pause', 'replay-seek', 'replay-speed']) el(id).disabled = !f;
  if (f) {
    el('replay-status').textContent = commandError || `${f.episode} · ${f.playing ? '播放中' : '暂停 / 已结束'}`;
    el('replay-time').textContent = `${f.position_s.toFixed(2)} / ${f.duration_s.toFixed(2)} s`;
    el('replay-seek').max = f.duration_s;
    if (document.activeElement !== el('replay-seek')) el('replay-seek').value = f.position_s;
    if (document.activeElement !== el('replay-speed')) el('replay-speed').value = f.speed || 1;
  }
  const pixels = f?.image_base64 || f?.jpeg_base64;
  el('replay-video').hidden = !pixels;
  el('replay-video-empty').hidden = !!pixels;
  el('replay-video-empty').textContent = f ? '该条未录制 RGB' : '等待回放数据';
  if (pixels) el('replay-video').src = `data:${f.image_mime || 'image/jpeg'};base64,${pixels}`;
  else el('replay-video').removeAttribute('src');
  el('replay-video-status').textContent = f ? '录制 RGB · ' + 'MCAP' : '录制 RGB';
  el('replay-video-status').title = '视频按接收时间与机械臂样本对齐；未录制深度';
}
async function poll() {
  try {
    const response = await fetch('/api/replay', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    const data = await response.json(); if (!response.ok || data.error) throw Error(data.error || '接口不可用');
    setReplay(data.replay || null);
    const real = data.robot_replay;
    el('robot-replay-state').textContent = '真机回放：' + (real?.error || ({positioning: '前往起点', playing: '正在执行', loading: '加载数据', idle: '未运行'}[real?.phase] || '未运行'));
    if (data.replay && generation !== data.replay.generation) { generation = data.replay.generation; open(); }
    render();
  } catch (error) {
    setReplay(null); render();
    el('replay-status').textContent = '回放数据未连接：' + error.message;
    el('robot-replay-state').textContent = '真机回放：状态未连接';
  } finally { if (!closed) timer = setTimeout(poll, document.hidden ? 1000 : mode === 'replay' ? 50 : 400); }
}
el('depth-toggle').onclick = () => {
  const on = el('depth-toggle').getAttribute('aria-pressed') !== 'true';
  el('depth-toggle').setAttribute('aria-pressed', String(on));
  document.body.dataset.depth = String(on);
};
el('replay-video').onclick = () => {
  const wrap = el('recorded-camera');
  if (document.fullscreenElement) document.exitFullscreen?.();
  else wrap.requestFullscreen?.();
};
const timelineSize = new ResizeObserver(() => {
  panel.closest('.stage').style.setProperty('--replay-space', `${panel.getBoundingClientRect().height + 44}px`);
});
timelineSize.observe(panel);
window.addEventListener('pagehide', () => {closed = true; clearTimeout(timer); timelineSize.disconnect();}, {once: true});
setMode('live'); render(); poll();
