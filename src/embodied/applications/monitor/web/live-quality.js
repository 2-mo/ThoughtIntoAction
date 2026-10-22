const el = id => document.getElementById(id);
const number = (value, scale = 1) => Number.isFinite(value) ? (value * scale).toFixed(2) : '—';
let data = {}, received = 0;
export function renderLiveQuality(value = {}) {
  data = value; received = performance.now(); draw();
}
function draw() {
  const elapsed = (performance.now() - received) / 1000;
  const motion = data.motion_quality || {};
  const fresh = motion.valid && Number.isFinite(motion.feedback_age_s) && motion.feedback_age_s + elapsed < .5;
  const value = key => fresh ? motion[key] : null;
  el('open-live-quality').textContent = `实测 ${number(value('peak_joint_speed_rad_s'), 180 / Math.PI)} °/s · 质量`;
  el('quality-valid').textContent = fresh ? '反馈有效 · 采样值' : '反馈不可用 / 已过期';
  el('quality-speed').textContent = number(value('peak_joint_speed_rad_s'), 180 / Math.PI);
  el('quality-error').textContent = number(value('target_feedback_error_rad'), 180 / Math.PI);
  el('quality-period').textContent = number(value('cycle_mean_ms'));
  el('quality-jitter').textContent = number(value('cycle_std_ms'));
  el('quality-period-max').textContent = number(value('cycle_max_ms'));
  el('quality-age').textContent = number(Number.isFinite(motion.feedback_age_s) ? motion.feedback_age_s + elapsed : null, 1000);
  el('quality-clock').textContent = `反馈 ${motion.feedback_monotonic_ns ?? '—'} ns · 命令 ${motion.command_monotonic_ns ?? '—'} ns · 命令晚于反馈 ${number(motion.command_after_feedback_ms)} ms。偏差未作时间对齐，不能当作同一时刻的跟踪误差。`;
  el('quality-period-note').textContent = `最近 ${motion.cycle_samples ?? 0} 个连续周期 · 监看跳过 ${motion.monitor_skipped_ticks ?? '—'} tick · 发布丢样 ${motion.producer_dropped ?? '—'} · observer 缺口 ${motion.observer_gaps ?? '—'}。周期统计不包含跳过的区间，不代表电机内部伺服抖动。`;
  el('quality-joint-speeds').textContent = fresh && motion.joint_velocity_rad_s ?
    motion.joint_velocity_rad_s.map((v, i) => `J${i + 1} ${number(v, 180 / Math.PI)} °/s`).join(' · ') : '关节实测速度：—';
  el('live-quality-cameras').replaceChildren(...Object.entries(data.camera_quality || {}).map(([id, camera]) => {
    const age = Number.isFinite(camera.frame_age_s) ? camera.frame_age_s + elapsed : null;
    const valid = camera.valid && age !== null && age < .5;
    const row = document.createElement('tr');
    [id, valid ? '有效' : '不可用 / 过期', number(age, 1000),
      number(valid && fresh ? camera.robot_receipt_delta_ms : null), camera.source_missing_frames ?? '未提供',
      camera.frame_number ?? '—'].forEach((text, index) => {
      const cell = document.createElement(index ? 'td' : 'th'); cell.textContent = String(text); row.append(cell);
    });
    return row;
  }));
  el('live-quality-empty').hidden = Object.keys(data.camera_quality || {}).length > 0;
}
el('open-live-quality').onclick = () => {draw(); el('live-quality-dialog').showModal();};
el('close-live-quality').onclick = () => el('live-quality-dialog').close();
const timer = setInterval(draw, 100);
window.addEventListener('pagehide', () => clearInterval(timer));
draw();
