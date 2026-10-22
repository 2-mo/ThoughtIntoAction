const el = id => document.getElementById(id);
let snapshot = {}, annotations = null, busy = false;
const states = {running: '执行中', started: '开始', completed: '执行完成', failed: '失败',
  needs_review: '待复核', approved: '复核通过', rejected: '复核退回'};
const label = value => states[value] || value || '—';
function cell(text, header = false) {
  const node = document.createElement(header ? 'th' : 'td'); node.textContent = String(text ?? '—'); return node;
}
function options(id, rows, title, previous) {
  el(id).replaceChildren(...rows.map(row => Object.assign(document.createElement('option'), {value: row.id, textContent: title(row)})));
  if (rows.some(row => row.id === previous)) el(id).value = previous;
}
function taskTimeline() {
  const task = snapshot.tasks?.find(item => item.task_id === el('review-task').value);
  el('task-result').textContent = task ? `${task.task_name} · ${label(task.status)}${task.error ? ' · ' + task.error : ''}` : '当前内核尚无任务记录';
  el('task-timeline').replaceChildren(...(task?.phases || []).map(phase => {
    const row = document.createElement('tr');
    row.title = `主机单调时间 ${phase.host_monotonic_ns ?? '—'} ns`;
    [phase.phase, label(phase.status), Number.isFinite(phase.elapsed_s) ? phase.elapsed_s.toFixed(3) + ' s' : '—',
      phase.operation_id, phase.error || JSON.stringify(phase.result ?? null)].forEach((text, i) => row.append(cell(text, !i)));
    return row;
  }));
}
function annotationTimeline() {
  el('review-episode').disabled = busy;
  el('review-refresh').disabled = busy;
  const review = annotations?.review;
  el('annotation-status').textContent = annotations ?
    `${label(annotations.annotation_status)} · 版本 ${annotations.revision} · 相机 ${annotations.camera_id} · 来源 ${annotations.input_kind} · 标注者 ${annotations.annotator}${review ? ' · ' + JSON.stringify(review) : ''}` : '此 Episode 尚无技能标注；先通过标注接口生成派生标注。';
  el('annotation-timeline').replaceChildren(...(annotations?.segments || []).map(segment => {
    const row = document.createElement('tr');
    row.title = `主机单调时间 ${segment.start_monotonic_ns ?? '—'}–${segment.end_monotonic_ns ?? '—'} ns`;
    [segment.phase, `${segment.start_frame}–${segment.end_frame}`, segment.object || '—',
      Number.isFinite(segment.confidence) ? segment.confidence : '—', (segment.evidence || []).join('；')].forEach((text, i) => row.append(cell(text, !i)));
    return row;
  }));
  for (const id of ['annotation-approve', 'annotation-reject']) el(id).disabled = busy || !annotations;
}
async function load() {
  if (busy) return;
  busy = true; annotationTimeline();
  try {
    const selected = el('review-episode').value;
    const response = await fetch('/api/review' + (selected ? '?episode_id=' + encodeURIComponent(selected) : ''), {cache: 'no-store', signal: AbortSignal.timeout(5000)});
    const payload = await response.json();
    if (!response.ok || payload.error) throw Error(payload.error || '时间轴读取失败');
    snapshot = payload; annotations = payload.annotations;
    options('review-task', (snapshot.tasks || []).map(t => ({...t, id: t.task_id})), t => `${t.task_name} · ${label(t.status)}`, el('review-task').value);
    options('review-episode', snapshot.episodes || [], e => `${e.id} · ${e.task || ''}`, selected);
    // A newly selected first episode needs its own annotation read.
    if (!selected && el('review-episode').value) {
      busy = false; return await load();
    }
    taskTimeline(); el('review-error').textContent = '';
  } catch (error) {snapshot = {}; annotations = null; taskTimeline(); el('review-error').textContent = error.message;}
  finally {busy = false; annotationTimeline();}
}
async function review(decision) {
  if (!annotations || busy) return;
  const reviewer = el('annotation-reviewer').value.trim();
  if (!reviewer) {el('review-error').textContent = '填写复核者名称'; return;}
  busy = true; annotationTimeline();
  try {
    const response = await fetch('/api/annotations/review', {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({episode_id: el('review-episode').value, reviewer, decision,
        expected_revision: annotations.revision, note: el('annotation-note').value.trim()}), signal: AbortSignal.timeout(5000)});
    const result = await response.json();
    if (!response.ok || result.error) throw Error(result.error || '复核失败');
    annotations = result.annotations; el('review-error').textContent = '复核已保存为新版本；原始录制未改变。';
  } catch (error) {el('review-error').textContent = `${error.message}；如版本已变化，请刷新后重新核对。`;}
  finally {busy = false; annotationTimeline();}
}
el('open-task-review').onclick = () => {el('task-review-dialog').showModal(); load();};
el('close-task-review').onclick = () => el('task-review-dialog').close();
el('review-refresh').onclick = load;
el('review-task').onchange = taskTimeline;
el('review-episode').onchange = () => {annotations = null; annotationTimeline(); load();};
el('annotation-approve').onclick = () => review('approved');
el('annotation-reject').onclick = () => review('rejected');

let refreshingTasks = false, closed = false;
export async function refreshTasks() {
  if (busy || refreshingTasks || closed || !el('task-review-dialog').open) return;
  refreshingTasks = true;
  try {
    const response = await fetch('/api/review?tasks_only=1', {cache: 'no-store', signal: AbortSignal.timeout(3000)});
    const result = await response.json();
    if (!response.ok || result.error) throw Error(result.error || '任务进度刷新失败');
    snapshot.tasks = result.tasks;
    options('review-task', (snapshot.tasks || []).map(t => ({...t, id: t.task_id})),
      t => `${t.task_name} · ${label(t.status)}`, el('review-task').value);
    taskTimeline();
    el('task-refresh-status').textContent = '阶段自动刷新 · 每秒一次';
  } catch (error) {el('task-refresh-status').textContent = `阶段更新中断：${error.message}`;}
  finally {refreshingTasks = false;}
}
const taskTimer = setInterval(refreshTasks, 1000);
window.addEventListener('pagehide', () => {closed = true; clearInterval(taskTimer);});
