import {mode, replayFrame, displayedState} from './display-state.js';
// Read-only telemetry stays visible even when WebGL or the robot is unavailable.
const byId = id => document.getElementById(id);
const jointRows = Array.from({length: 6}, (_, index) => {
  const row = document.createElement('div');
  row.className = 'position-row';
  row.innerHTML = `<span class="position-name">J${index + 1}</span><div class="position-track"><i class="position-zero" style="left:50%"></i><i class="position-fill target" hidden></i><i class="position-fill actual" hidden></i></div><span class="position-values"><b>—</b><b>—</b></span>`;
  byId('joints').append(row);
  return row;
});
const velocityRows = jointRows.map((_, index) => {
  const row = document.createElement('div');
  row.className = 'position-row';
  row.innerHTML = `<span class="position-name">J${index + 1}</span><svg class="velocity-chart" viewBox="0 0 200 24" preserveAspectRatio="none" aria-label="J${index + 1} 速度，最近五秒"><path class="torque-grid" d="M0 12H200 M50 0V24 M100 0V24 M150 0V24"/><path class="velocity-line target"/><path class="velocity-line actual"/></svg><span class="position-values"><b>—</b><b>—</b></span>`;
  byId('velocities').append(row); return row;
});
const torqueRows = [...jointRows, null].map((_, index) => {
  const label = index < 6 ? `J${index + 1}` : '夹爪';
  const row = document.createElement('div');
  row.className = 'torque-row';
  row.innerHTML = `<span>${label}<small class="torque-range">—</small></span><svg viewBox="0 0 200 24" preserveAspectRatio="none" role="img" aria-label="${label} 力矩，最近五秒"><path class="torque-grid" d="M0 3H200 M0 12H200 M0 21H200 M50 0V24 M100 0V24 M150 0V24"/><path class="torque-line"/></svg><b>—</b>`;
  byId('torques').append(row);
  return row;
});
const motorRows = Array.from({length: 7}, (_, i) => {
  const row = document.createElement('div');
  row.className = 'motor-row'; row.setAttribute('role', 'row');
  row.innerHTML = `<span>${i < 6 ? `J${i + 1}` : '夹爪'}</span>${'<b>—</b>'.repeat(6)}`;
  byId('motor-feedback').append(row);
  return row;
});
function showMotors(live) {
  const data = state?.motor_diagnostics;
  const order = [...(data?.joint_order || []), data?.gripper_id];
  motorRows.forEach((row, i) => {
    const motor = live ? data?.motors?.find(m => m.motor_id === order[i]) : null;
    const fresh = motor?.feedback_status === 'live';
    const fault = Number.isInteger(motor?.fault_code) && motor.fault_code !== 0;
    row.dataset.state = fault ? 'fault' : motor?.valid === true ? 'live' : 'missing';
    const numbers = [motor?.current_a, motor?.temperature_c, motor?.rotor_temperature_c, motor?.bus_voltage_v];
    row.querySelectorAll('b').forEach((cell, column) => {
      if (column < 4) cell.textContent = fresh && finite(numbers[column]) ? numbers[column].toFixed(column === 0 ? 3 : 1) : '—';
      else if (column === 4) cell.textContent = Number.isInteger(motor?.fault_code) ? String(motor.fault_code) : '—';
      else cell.textContent = motor?.valid === true ? '有效' : motor?.valid === false ? '无效' : '—';
    });
    row.title = !motor ? '暂无电机反馈' : `CAN ID ${motor.motor_id} · ${motor.motor_type || ''} · ${motor.feedback_status}；— 表示未提供反馈`;
  });
}
let limits, torqueLimits, velocityLimits, state = null, received = 0, history = [], streamId, lastSample;
const finite = Number.isFinite;
let previousQuaternion = null, quaternionStream;
function showPositions(live) {
  byId('joints').classList.toggle('feedback-inactive', !live);
  const missing = !state?.actual || state?.stream?.status === 'waiting';
  jointRows.forEach((row, index) => {
    const extent = limits ? Math.max(Math.abs(limits.lower[index]), Math.abs(limits.upper[index])) : null;
    [state?.actual?.q?.[index], state?.target?.q?.[index]].forEach((value, side) => {
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
  if (quaternionStream !== state?.stream?.stream_id) {
    previousQuaternion = null;
    quaternionStream = state?.stream?.stream_id;
  }
  const quaternion = missing ? null : state?.actual?.eef_quaternion;
  let displayedQuaternion = null;
  if (Array.isArray(quaternion) && quaternion.length === 4 && quaternion.every(finite)) {
    const norm = Math.hypot(...quaternion);
    if (norm > 1e-9) {
      displayedQuaternion = quaternion.map(v => v / norm);
      if (previousQuaternion && displayedQuaternion.reduce((dot, v, i) => dot + v * previousQuaternion[i], 0) < 0)
        displayedQuaternion = displayedQuaternion.map(v => -v);
      previousQuaternion = displayedQuaternion;
    }
  }
  byId('eef-quaternion').querySelectorAll('b').forEach((cell, i) => {
    const value = displayedQuaternion?.[i];
    cell.textContent = finite(value) ? (Math.abs(value) < .00005 ? 0 : value).toFixed(4) : '—';
  });
  byId('eef-quaternion').classList.toggle('feedback-inactive', !live);
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
  const now = mode === 'replay' ? (replayFrame?.position_s || 0) * 1000 : performance.now();
  if (mode === 'replay') { state = displayedState(null); history = replayFrame?.torque_history || []; }
  const live = state?.stream?.status === 'live' && (mode === 'replay' || now - received + (state.stream.age_s || 0) * 1000 < 1000);
  showPositions(live);
  showMotors(live);
  velocityRows.forEach((row, index) => {
    const scale = velocityLimits?.[index];
    row.title = Number.isFinite(scale) ? `URDF 速度量程 ±${(scale * 180 / Math.PI).toFixed(1)} °/s；深色实际，浅色指令` : '速度量程未配置';
    [state?.actual_vel?.[index], state?.target_vel?.[index]].forEach((v, side) => {
      const valid = live && finite(v);
      row.querySelectorAll('b')[side].textContent = valid ? (v * 180 / Math.PI).toFixed(1) : '—';
      const segments = [];
      let previous;
      const y = v => 12 - 10 * Math.max(-1, Math.min(1, v / scale));
      const valueAt = sample => sample[side ? 'target_velocity' : 'velocity']?.[index];
      for (const sample of history) {
        const value = valueAt(sample);
        if (!finite(value) || !finite(scale) || scale <= 0) {previous = null; continue;}
        if (sample.time < now - 5000) {previous = sample; continue;}
        const connected = previous && sample.time - previous.time <= 500;
        if (connected && previous.time < now - 5000) {
          const fraction = (now - 5000 - previous.time) / (sample.time - previous.time);
          segments.push(`M0 ${y(valueAt(previous) + fraction * (value - valueAt(previous))).toFixed(2)}`);
        }
        segments.push(`${connected ? 'L' : 'M'}${(200 * (sample.time - now + 5000) / 5000).toFixed(2)} ${y(value).toFixed(2)}`);
        previous = sample;
      }
      row.querySelector(side ? '.target' : '.actual').setAttribute('d', live ? segments.join(' ') : '');
    });
  });
  const speeds = [state?.eef_velocity?.speed, state?.eef_velocity?.angular_speed, state?.gripper_actual_vel];
  byId('eef-velocity').querySelectorAll('b').forEach((cell, i) => {
    cell.textContent = live && finite(speeds[i]) ? (speeds[i] * (i === 1 ? 180 / Math.PI : 1000)).toFixed(1) : '—';
  });
  byId('eef-velocity').title = '末端速度由反馈关节速度和 URDF 雅可比计算，坐标系与末端位置一致';
  const windowStart = now - 5000;
  // Keep one point before the boundary so the trace can be clipped continuously.
  while (history.length > 1 && history[1].time <= windowStart) history.shift();
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
    const segments = [];
    const yFor = value => 12 - Math.max(-1, Math.min(1, value / scale)) * 9;
    for (const sample of history) {
      const value = sample.values[index];
      if (!validScale || !finite(value)) { previous = undefined; continue; }
      if (sample.time < windowStart) { previous = sample; continue; }
      const connected = previous && sample.time - previous.time <= 500;
      if (connected && previous.time < windowStart) {
        const fraction = (windowStart - previous.time) / (sample.time - previous.time);
        const boundary = previous.values[index] + fraction * (value - previous.values[index]);
        segments.push(`M0.0 ${yFor(boundary).toFixed(1)}`);
      }
      const x = 200 * (sample.time - windowStart) / 5000;
      segments.push(`${connected ? 'L' : 'M'}${x.toFixed(1)} ${yFor(value).toFixed(1)}`);
      previous = sample;
    }
    const d = segments.join(' ');
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
        limits = model.joint_limits; torqueLimits = model.torque_limits; velocityLimits = model.velocity_limits;
      }
    }
    const response = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    if (!response.ok) throw new Error('Feedback unavailable');
    const liveState = await response.json();
    state = displayedState(liveState);
    received = performance.now();
    if (state.stream?.stream_id !== streamId) {
      history = []; lastSample = undefined; streamId = state.stream?.stream_id;
    }
    const values = [...(state.actual_torque || Array(6).fill(null)), state.gripper_actual_torque];
    if (mode !== 'replay' && state.stream?.status === 'live' && state.stream.sequence !== lastSample &&
        (values.some(finite) || state.actual_vel?.some(finite))) {
      lastSample = state.stream.sequence;
      history.push({time: received - (state.stream.age_s || 0) * 1000, values,
        velocity: state.actual_vel, target_velocity: state.target_vel});
    }
  } catch {
    if (state) state = {...state, stream: {...state.stream, status: 'error'}};
  }
  if (!stopped) setTimeout(poll, document.hidden ? 500 : 100);
}
window.addEventListener('view-mode', () => {state = null; history = []; streamId = undefined; lastSample = undefined; render();});
render();
const timer = setInterval(render, 100);
window.addEventListener('pagehide', () => {stopped = true; clearInterval(timer);});
poll();
