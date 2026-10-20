const el = id => document.getElementById(id);
const labels = {checking: '检查中', passed: '通过', failed: '有问题'};
let episode = null;
const number = (value, scale = 1) => Number.isFinite(value) ? (value * scale).toFixed(1) : '—';
export function renderQuality(last) {
  episode = last;
  const button = el('collection-last'), quality = last?.quality;
  button.disabled = !last;
  button.dataset.quality = quality?.status || '';
  const saved = last?.lifecycle !== 'complete' ? '未完整保存' : last?.outcome === 'discarded' ? '已丢弃' : '已保存';
  button.textContent = last ? `上一条：${saved} · 质量${labels[quality?.status] || '未检查'} · 详情` : '上一条：—';
  button.title = last ? `${last.id} · 点击查看质量详情` : '暂无录制';
  if (el('quality-details').open) renderDetails();
}
function renderDetails() {
  const report = episode?.quality;
  el('quality-episode').textContent = episode?.id || '暂无录制';
  el('quality-summary').textContent = `数据质量：${labels[report?.status] || '未检查'}`;
  const rows = report ? [['机械臂', report.state_count, report.state_hz, report.state_max_gap_s, null, null],
    ...Object.entries(report.cameras || {}).map(([key, cam]) => [
      {wrist_rgb: '腕部 RGB', exterior_rgb: '外部 RGB'}[key] || key,
      cam.frame_count, cam.frame_hz, cam.max_gap_s, cam.missing_source_frames, cam.receive_time_alignment_max_s])] : [];
  el('quality-streams').replaceChildren(...rows.map(values => {
    const row = document.createElement('tr');
    values.forEach((value, index) => {
      const cell = document.createElement(index === 0 ? 'th' : 'td');
      cell.textContent = index === 0 ? value : index === 1 || index === 4 ? (value ?? '—') : number(value, index >= 3 ? 1000 : 1);
      row.append(cell);
    });
    return row;
  }));
  const messages = [
    ...(report?.errors || []).map(text => ['error', text]),
    ...(report?.warnings || []).map(text => ['warning', text]),
    ...Object.entries(report?.camera_pair_alignment || {}).map(([key, value]) => ['note',
      `${key} 与腕部接收时间偏差：P99 ${number(value.p99_s, 1000)} ms · 最大 ${number(value.max_s, 1000)} ms`])];
  el('quality-messages').replaceChildren(...messages.map(([kind, text]) => {
    const p = document.createElement('p'); p.dataset.kind = kind; p.textContent = text; return p;
  }));
}
el('collection-last').onclick = () => {renderDetails(); el('quality-details').showModal();};
el('quality-close').onclick = () => el('quality-details').close();
