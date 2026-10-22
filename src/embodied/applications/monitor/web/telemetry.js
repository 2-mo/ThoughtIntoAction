import {mode, replayFrame, displayedState} from './display-state.js';
// Read-only telemetry stays visible even when WebGL or the robot is unavailable.
const byId = id => document.getElementById(id);
byId('open-telemetry-details').onclick = () => byId('telemetry-details').showModal();
byId('telemetry-details-close').onclick = () => byId('telemetry-details').close();
let jointCount = 0, hasGripper = false, modelViewerId;
let commandRows = [], jointRows = [], velocityRows = [], torqueRows = [], motorRows = [];
function configureModel(model) {
  const names = model.data_contract?.joint_names || model.robot?.joint_names || [];
  jointCount = names.length;
  hasGripper = model.data_contract ? !!model.data_contract.gripper : jointCount > 0;
  for (const id of ['command-feedback', 'joints', 'velocities', 'torques', 'motor-feedback']) byId(id).replaceChildren();
  byId('gripper').hidden = byId('gripper-velocity').hidden = !hasGripper;
  commandRows = Array.from({length: jointCount + (hasGripper ? 1 : 0)}, (_, i) => {
    const row = document.createElement('tr');
    row.innerHTML = `<th>${i < jointCount ? `J${i + 1}` : '夹爪'}</th>${'<td>—</td>'.repeat(6)}`;
    byId('command-feedback').append(row);
    return row;
  });
  jointRows = Array.from({length: jointCount}, (_, index) => {
    const row = document.createElement('div');
    row.className = 'position-row';
    row.innerHTML = `<span class="position-name">J${index + 1}</span><div class="position-track"><i class="position-zero" style="left:50%"></i><i class="position-fill target" hidden></i><i class="position-fill actual" hidden></i></div><span class="position-values"><b>—</b><b>—</b></span>`;
    byId('joints').append(row);
    return row;
  });
  velocityRows = jointRows.map((_, index) => {
    const row = document.createElement('div');
    row.className = 'position-row';
    row.innerHTML = `<span class="position-name">J${index + 1}<small class="velocity-range">—</small><small class="chart-clipped" hidden>超范围</small></span><svg class="velocity-chart" viewBox="0 0 200 24" preserveAspectRatio="none" role="img" aria-label="J${index + 1} 速度，最近五秒"><path class="torque-grid" d="M50 0V24 M100 0V24 M150 0V24"/><path class="chart-zero" d="M0 12H200"/><path class="velocity-line actual"/><path class="velocity-line trend"/><path class="velocity-line target"/></svg><span class="position-values"><b>—</b><b>—</b></span>`;
    byId('velocities').append(row); return row;
  });
  torqueRows = [...jointRows, ...(hasGripper ? [null] : [])].map((_, index) => {
    const label = index < jointCount ? `J${index + 1}` : '夹爪';
    const row = document.createElement('div');
    row.className = 'torque-row';
    row.innerHTML = `<span>${label}<small class="torque-range">—</small><small class="chart-clipped" hidden>超范围</small></span><svg viewBox="0 0 200 24" preserveAspectRatio="none" role="img" aria-label="${label} 力矩，最近五秒"><path class="torque-grid" d="M0 3H200 M0 21H200 M50 0V24 M100 0V24 M150 0V24"/><path class="chart-zero" d="M0 12H200"/><path class="torque-line"/></svg><b>—</b>`;
    byId('torques').append(row);
    return row;
  });
  motorRows = Array.from({length: jointCount + (hasGripper ? 1 : 0)}, (_, i) => {
    const row = document.createElement('div');
    row.className = 'motor-row'; row.setAttribute('role', 'row');
    row.innerHTML = `<span>${i < jointCount ? `J${i + 1}` : '夹爪'}</span>${Array.from({length:3}, (_, c) => `<span class="motor-cell"><b>—</b><span class="motor-track${c === 0 ? ' bipolar' : ''}"><i hidden></i></span></span>`).join('')}${'<b class="motor-detail">—</b>'.repeat(3)}`;
    byId('motor-feedback').append(row);
    return row;
  });

  jointRows.forEach((row, i) => {row.dataset.jointName = names[i];});
}
function showCommands(live) {
  const data = live ? state?.command_details : null;
  commandRows.forEach((row, i) => {
    const values = i < jointCount
      ? [data?.submission?.pos?.[i], ...['sdk_cmd_pos', 'sdk_cmd_vel', 'sdk_cmd_torque', 'kp', 'kd'].map(k => data?.[k]?.[i])]
      : [data?.submission?.gripper_pos, ...['gripper_sdk_cmd_pos', 'gripper_sdk_cmd_vel', 'gripper_sdk_cmd_torque', 'gripper_kp', 'gripper_kd'].map(k => data?.[k])];
    row.querySelectorAll('td').forEach((cell, j) => {cell.textContent = finite(values[j]) ? values[j].toFixed(4) : '—';});
  });
  byId('command-feedback-status').textContent = data ? `${mode === 'replay' ? '录制样本' : '实时'}${hasGripper ? ' · 夹爪模式 ' + (data.gripper_mode || '—') : ''}` : '当前数据源未提供';
  byId('command-timestamps').textContent = 'SDK 时间戳（s） · ' + [['反馈', 'feedback_timestamp'], ['指令', 'command_timestamp'], ['控制器', 'controller_timestamp']]
    .map(([label, key]) => `${label} ${finite(data?.[key]) ? data[key].toFixed(6) : '—'}`).join(' · ');
}
function showMotors(live) {
  const data = state?.motor_diagnostics;
  const order = [...(data?.joint_order || []), ...(hasGripper ? [data?.gripper_id] : [])];
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
    row.querySelectorAll('.motor-track i').forEach((bar, column) => {
      const value = numbers[column];
      bar.hidden = !fresh || !finite(value);
      const end = column === 0 ? 50 + Math.max(-1, Math.min(1, value / 30)) * 50 : Math.max(0, Math.min(100, value));
      bar.style.left = `${column === 0 ? Math.min(50, end) : 0}%`;
      bar.style.width = `${column === 0 ? Math.abs(end - 50) : end}%`;
    });
    row.title = !motor ? '暂无电机反馈' : `CAN ID ${motor.motor_id} · ${motor.motor_type || ''} · ${motor.feedback_status}；— 表示未提供反馈`;
  });
}
let limits, torqueLimits, velocityLimits, state = null, received = 0, history = [], streamId, lastSample;
const finite = Number.isFinite;
let sampleClock = null;
function clearHistory() { history = []; lastSample = undefined; sampleClock = null; }
function sampleTime(stream, receipt) {
  const stamp = stream.sample_monotonic_ns / 1e6;
  const fallback = receipt - (finite(stream.age_s) ? Math.max(0, stream.age_s) * 1000 : 0);
  if (!finite(stamp) || stamp <= 0) return fallback;
  if (!sampleClock || stamp < sampleClock.last) {
    history = [];
    sampleClock = {server: stamp, browser: fallback};
  }
  sampleClock.last = stamp;
  return sampleClock.browser + stamp - sampleClock.server;
}
function withVelocityTrend(sample, previous) {
  const dt = sample.time - previous?.time;
  const alpha = dt > 0 && dt <= 500 ? -Math.expm1(-dt / 120) : 1;
  return {...sample, trend_velocity: (sample.velocity || []).map((value, index) => {
    const before = previous?.trend_velocity?.[index];
    return finite(value) ? (alpha < 1 && finite(before) ? before + alpha * (value - before) : value) : null;
  })};
}
const chartTime = () => mode === 'replay' ? (replayFrame?.position_s || 0) * 1000 : performance.now();
const feedbackLive = now => state?.stream?.status === 'live' &&
  (mode === 'replay' || now - received + (state.stream.age_s || 0) * 1000 < 1000);
let previousQuaternion = null, quaternionStream;
function showPositions(live) {
  byId('joints').classList.toggle('feedback-inactive', !live);
  const missing = !state?.actual || state?.stream?.status === 'waiting';
  jointRows.forEach((row, index) => {
    const extent = limits ? Math.max(Math.abs(limits.lower[index]), Math.abs(limits.upper[index])) : null;
    const actual = state?.actual?.q?.[index], target = state?.target?.q?.[index];
    row.title = !missing && finite(actual) && finite(target)
      ? `J${index + 1} · 目标 − 实际：${((target - actual) * 180 / Math.PI).toFixed(2)}°` : '暂无关节反馈';
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
    const valid = finite(value);
    const validRange = finite(grip?.max) && grip.max > 0;
    gripRow.querySelectorAll('b')[side].textContent = valid ? `${(value * 1000).toFixed(1)}mm` : '—';
    const group = byId(side ? 'gripper-target' : 'gripper-actual');
    group.replaceChildren();
    if (!valid || !validRange) return;
    const gap = 52 * Math.max(0, Math.min(1, value / grip.max));
    const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
    path.setAttribute('d', `M${76-gap} 9V26H${80-gap} M${84+gap} 9V26H${80+gap}`);
    group.append(path);
  });
}
function render() {
  const now = chartTime();
  if (mode === 'replay') {
    state = displayedState(null);
    // Rebuild only from this replay window, so seeking cannot retain a live or
    // later recording's filter state. Recorded samples remain untouched.
    history = [];
    for (const sample of replayFrame?.torque_history || []) history.push(withVelocityTrend(sample, history.at(-1)));
  }
  const live = feedbackLive(now);
  if (!live) clearHistory();
  showPositions(live);
  showMotors(live);
  showCommands(live);
  velocityRows.forEach((row, index) => {
    const scale = velocityLimits?.[index] / Number(byId('velocity-zoom').value);
    const validScale = finite(scale) && scale > 0;
    row.querySelector('.velocity-range').textContent = validScale ? `±${(scale * 180 / Math.PI).toFixed(1)}` : '—';
    [state?.actual_vel?.[index], state?.target_vel?.[index]].forEach((v, side) => {
      const valid = live && finite(v);
      row.querySelectorAll('b')[side].textContent = valid ? (v * 180 / Math.PI).toFixed(1) : '—';
    });
  });
  const speeds = [state?.eef_velocity?.speed, state?.eef_velocity?.angular_speed];
  byId('eef-velocity').querySelectorAll('b').forEach((cell, i) => {
    cell.textContent = live && finite(speeds[i]) ? (speeds[i] * (i === 1 ? 180 / Math.PI : 1000)).toFixed(1) : '—';
  });
  byId('eef-velocity').title = '末端速度由反馈关节速度和 URDF 雅可比计算，坐标系与末端位置一致';
  byId('gripper-velocity').querySelectorAll('b').forEach((cell, i) => {
    const value = i ? state?.command_details?.gripper_sdk_cmd_vel : state?.gripper_actual_vel;
    cell.textContent = live && finite(value) ? (value * 1000).toFixed(1) : '—';
  });
  byId('torque-scale').textContent = 'N·m · 5 s';
  const values = live ? [...(state?.actual_torque || Array(jointCount).fill(null)), ...(hasGripper ? [state?.gripper_actual_torque] : [])] : null;
  byId('torques').dataset.state = values?.some(finite) ? 'live' : 'waiting';
  torqueRows.forEach((row, index) => {
    const scale = torqueLimits?.[index] / Number(byId('torque-zoom').value);
    const validScale = finite(scale) && scale > 0;
    row.querySelector('.torque-range').textContent = validScale ? `±${scale}` : '—';
    row.title = validScale ? `固定纵轴 ±${scale} N·m，零点居中；超范围仅表示曲线超出当前显示范围，数值保持原值` : '未配置最大力矩';
    row.dataset.state = finite(values?.[index]) ? 'live' : 'waiting';
    row.querySelector('b').textContent = finite(values?.[index]) ? values[index].toFixed(2) : '—';
  });
  drawCharts();
}
function trace(valueAt, scale, now, amplitude = 10, digits = 2) {
  const start = now - 5000, segments = [];
  const y = value => (12 - amplitude * Math.max(-1, Math.min(1, value / scale))).toFixed(digits);
  let previous;
  for (const sample of history) {
    const value = valueAt(sample);
    if (!finite(scale) || scale <= 0 || !finite(value)) { previous = null; continue; }
    if (sample.time < start) { previous = sample; continue; }
    const connected = previous && sample.time > previous.time && sample.time - previous.time <= 500;
    if (connected && previous.time < start) {
      const fraction = (start - previous.time) / (sample.time - previous.time);
      segments.push(`M${(0).toFixed(digits)} ${y(valueAt(previous) + fraction * (value - valueAt(previous)))}`);
    }
    segments.push(`${connected ? 'L' : 'M'}${(200 * (sample.time - start) / 5000).toFixed(digits)} ${y(value)}`);
    previous = sample;
  }
  return segments.join(' ');
}
function drawCharts() {
  const now = chartTime(), live = feedbackLive(now), start = now - 5000;
  // Retain one boundary sample for continuous scrolling without invented data.
  while (history.length > 1 && history[1].time <= start) history.shift();
  velocityRows.forEach((row, index) => {
    const scale = velocityLimits?.[index] / Number(byId('velocity-zoom').value);
    const samples = history.filter(sample => sample.time >= start);
    const raw = samples.map(sample => sample.velocity?.[index]).filter(finite);
    const peak = raw.length ? Math.max(...raw.map(Math.abs)) : null;
    let clipped = finite(scale) && scale > 0 && [state?.actual_vel?.[index], state?.target_vel?.[index],
      ...raw, ...samples.map(sample => sample.target_velocity?.[index])].some(value => finite(value) && Math.abs(value) > scale);
    for (const [name, field] of [['actual', 'velocity'], ['trend', 'trend_velocity'], ['target', 'target_velocity']])
      row.querySelector('.' + name).setAttribute('d', live ? trace(sample => sample[field]?.[index], scale, now) : '');
    row.querySelector('.chart-clipped').hidden = !live || !clipped;
    row.title = `原始瞬时数值不滤波；淡线为原始反馈，亮线为120 ms平滑趋势，橙色虚线为指令。最近5秒已收到样本的原始峰值：${live && finite(peak) ? (peak * 180 / Math.PI).toFixed(1) + ' °/s' : '—'}；不代表控制全采样峰值。超范围仅表示超出当前图表量程。`;
  });
  torqueRows.forEach((row, index) => {
    const scale = torqueLimits?.[index] / Number(byId('torque-zoom').value);
    const value = index < jointCount ? state?.actual_torque?.[index] : state?.gripper_actual_torque;
    const clipped = finite(scale) && scale > 0 && (finite(value) && Math.abs(value) > scale ||
      history.some(sample => sample.time >= start && finite(sample.values?.[index]) && Math.abs(sample.values[index]) > scale));
    row.querySelector('.torque-line').setAttribute('d', live ? trace(sample => sample.values?.[index], scale, now, 9, 1) : '');
    row.querySelector('.chart-clipped').hidden = !live || !clipped;
  });
}
for (const id of ['velocity-zoom', 'torque-zoom']) byId(id).addEventListener('change', render);
let stopped = false;
async function poll() {
  try {
    if (!limits) {
      const response = await fetch('/api/model', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
      if (response.ok) {
        const model = await response.json();
        limits = model.joint_limits; torqueLimits = model.torque_limits; velocityLimits = model.velocity_limits;
        configureModel(model); modelViewerId = model.viewer_id;
      }
    }
    const response = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    if (!response.ok) throw new Error('Feedback unavailable');
    const liveState = await response.json();
    if (modelViewerId && liveState.viewer_id !== modelViewerId) {
      limits = null; state = null; clearHistory(); throw Error('模型已切换，等待新描述');
    }
    state = displayedState(liveState);
    received = performance.now();
    if (state.stream?.stream_id !== streamId) {
      clearHistory(); streamId = state.stream?.stream_id;
    }
    const values = [...(state.actual_torque || Array(jointCount).fill(null)), ...(hasGripper ? [state.gripper_actual_torque] : [])];
    if (mode !== 'replay' && state.stream?.status === 'live' && state.stream.sequence !== lastSample &&
        (values.some(finite) || state.actual_vel?.some(finite))) {
      lastSample = state.stream.sequence;
      const time = sampleTime(state.stream, received);
      history.push(withVelocityTrend({time, values, velocity: state.actual_vel,
        target_velocity: state.target_vel}, history.at(-1)));
    }
  } catch {
    if (state) state = {...state, stream: {...state.stream, status: 'error'}};
  }
  if (!stopped) setTimeout(poll, document.hidden ? 500 : 100);
}
window.addEventListener('view-mode', () => {state = null; clearHistory(); streamId = undefined; render();});
render();
const timer = setInterval(render, 100);
let animation;
function animate() { drawCharts(); if (!stopped) animation = requestAnimationFrame(animate); }
animation = requestAnimationFrame(animate);
window.addEventListener('pagehide', () => {stopped = true; clearInterval(timer); cancelAnimationFrame(animation);});
poll();
