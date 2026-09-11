// Read-only telemetry stays visible even when WebGL or the robot is unavailable.
const byId = id => document.getElementById(id);
const jointRows = Array.from({length: 6}, (_, index) => {
  const row = document.createElement('div');
  row.className = 'position-row';
  row.innerHTML = `<span class="position-name">J${index + 1}</span><div class="position-track"><i class="position-zero" style="left:50%"></i><i class="position-fill target" hidden></i><i class="position-fill actual" hidden></i></div><span class="position-values"><b>—</b><b>—</b></span>`;
  byId('joints').append(row);
  return row;
});
const torqueRows = [...jointRows, null].map((_, index) => {
  const label = index < 6 ? `J${index + 1}` : '夹爪';
  const row = document.createElement('div');
  row.className = 'torque-row';
  row.innerHTML = `<span>${label}<small class="torque-range">—</small></span><svg viewBox="0 0 200 24" preserveAspectRatio="none" role="img" aria-label="${label} 力矩，最近五秒"><path class="torque-grid" d="M0 3H200 M0 12H200 M0 21H200 M50 0V24 M100 0V24 M150 0V24"/><path class="torque-line"/></svg><b>—</b>`;
  byId('torques').append(row);
  return row;
});
let limits, torqueLimits, state = null, received = 0, history = [], streamId, lastSample;
const finite = Number.isFinite;
function showPositions(live) {
  byId('joints').classList.toggle('feedback-inactive', !live);
  const missing = !state?.actual || state?.stream?.status === 'waiting';
  jointRows.forEach((row, index) => {
    const extent = limits ? Math.max(Math.abs(limits.lower[index]), Math.abs(limits.upper[index])) : null;
    [state?.actual?.q[index], state?.target?.q[index]].forEach((value, side) => {
      const valid = !missing && finite(value);
      row.querySelectorAll('b')[side].textContent = valid ? `${(value * 180 / Math.PI).toFixed(2)}°` : '—';
      const fill = row.querySelector(side ? '.target' : '.actual');
      fill.hidden = !valid || !finite(extent) || extent <= 0;
      if (!fill.hidden) {
        const end = 50 + 50 * Math.max(-1, Math.min(1, value / extent));
        fill.style.left = `${Math.min(50, end)}%`;
        fill.style.width = `${Math.abs(end - 50)}%`;
      }
    });
  });
  byId('eef-position').querySelectorAll('b').forEach((cell, index) => {
    const value = missing ? null : state?.actual?.eef?.[12 + index];
    cell.textContent = finite(value) ? value.toFixed(4) : '—';
  });
  byId('eef-position').classList.toggle('feedback-inactive', !live);
  const grip = missing ? null : state?.gripper;
  const gripRow = byId('gripper');
  gripRow.classList.toggle('feedback-inactive', !live);
  gripRow.dataset.state = finite(grip?.actual) ? (live ? 'live' : 'stale') : 'waiting';
  [grip?.actual, grip?.target].forEach((value, side) => {
    const valid = finite(value) && finite(grip?.max) && grip.max > 0;
    gripRow.querySelectorAll('b')[side].textContent = valid ? `${(value * 1000).toFixed(1)}mm` : '—';
    const group = byId(side ? 'gripper-target' : 'gripper-actual');
    group.replaceChildren();
    if (!valid) return;
    const gap = 52 * Math.max(0, Math.min(1, value / grip.max));
    const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    path.setAttribute('d', `M${76-gap} 9V26H${80-gap} M${84+gap} 9V26H${80+gap}`);
    group.append(path);
  });
}
function render() {
  const now = performance.now();
  const live = state?.stream?.status === 'live' && now - received + (state.stream.age_s || 0) * 1000 < 1000;
  showPositions(live);
  history = history.filter(sample => now - sample.time <= 5000);
  byId('torque-scale').textContent = 'N·m · 5 s';
  const values = live ? [...(state?.actual_torque || Array(6).fill(null)), state?.gripper_actual_torque] : null;
  byId('torques').dataset.state = values?.some(finite) ? 'live' : 'waiting';
  torqueRows.forEach((row, index) => {
    const scale = torqueLimits?.[index];
    const validScale = finite(scale) && scale > 0;
    row.querySelector('.torque-range').textContent = validScale ? `±${scale}` : '—';
    row.title = validScale ? `固定纵轴 ±${scale} N·m，最大力矩量程，零点居中` : '未配置最大力矩';
    row.dataset.state = finite(values?.[index]) ? 'live' : 'waiting';
    row.querySelector('b').textContent = finite(values?.[index]) ? values[index].toFixed(2) : '—';
    let previous;
    const d = history.map(sample => {
      if (!validScale || !finite(sample.values[index])) { previous = undefined; return ''; }
      const x = 200 * (1 - (now - sample.time) / 5000);
      const y = 12 - Math.max(-1, Math.min(1, sample.values[index] / scale)) * 9;
      const command = previous === undefined || sample.time - previous > 500 ? 'M' : 'L';
      previous = sample.time;
      return `${command}${x.toFixed(1)} ${y.toFixed(1)}`;
    }).join(' ');
    row.querySelector('.torque-line').setAttribute('d', d);
  });
}
let stopped = false;
async function poll() {
  try {
    if (!limits) {
      const response = await fetch('/api/model', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
      if (response.ok) {
        const model = await response.json();
        limits = model.joint_limits; torqueLimits = model.torque_limits;
      }
    }
    const response = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    if (!response.ok) throw new Error('Feedback unavailable');
    state = await response.json();
    received = performance.now();
    if (state.stream?.stream_id !== streamId) {
      history = []; lastSample = undefined; streamId = state.stream?.stream_id;
    }
    const values = [...(state.actual_torque || Array(6).fill(null)), state.gripper_actual_torque];
    if (state.stream?.status === 'live' && state.stream.sequence !== lastSample &&
        values.some(finite)) {
      lastSample = state.stream.sequence;
      history.push({time: received - (state.stream.age_s || 0) * 1000, values});
    }
  } catch {
    if (state) state = {...state, stream: {...state.stream, status: 'error'}};
  }
  if (!stopped) setTimeout(poll, document.hidden ? 500 : 100);
}
render();
const timer = setInterval(render, 100);
window.addEventListener('pagehide', () => {stopped = true; clearInterval(timer);});
poll();
