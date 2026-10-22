// Offline DOM stubs exercise states/actions; no HTTP listener or browser required.
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const root = new URL('../../src/embodied/applications/monitor/web/', import.meta.url);
const html = await readFile(new URL('index.html', root), 'utf8');
class Node {
  value = ''; textContent = ''; children = []; disabled = false;
  append(child) {this.children.push(child);}
  replaceChildren(...children) {this.children = children; if (this.select) this.value = children[0]?.value || '';}
  showModal() {this.open = true;}
  close() {this.open = false;}
}
const nodes = new Map([...html.matchAll(/id="([^"]+)"/g)].map(match => [match[1], new Node()]));
for (const id of ['review-episode', 'review-task']) nodes.get(id).select = true;
globalThis.document = {
  getElementById(id) {assert.ok(nodes.has(id), `Missing HTML element: ${id}`); return nodes.get(id);},
  createElement() {return new Node();},
};
globalThis.window = {addEventListener() {}};
globalThis.setInterval = () => 0;
const module = async name => import('data:text/javascript;base64,' + Buffer.from(await readFile(new URL(name, root), 'utf8')).toString('base64'));
const quality = await module('live-quality.js');
quality.renderLiveQuality({motion_quality: {valid: true, feedback_age_s: 0, peak_joint_speed_rad_s: 0,
  joint_velocity_rad_s: [0], cycle_samples: 0}, camera_quality: {cam: {valid: true, frame_age_s: 0}}});
assert.match(nodes.get('open-live-quality').textContent, /0.00/);
assert.equal(nodes.get('quality-jitter').textContent, '—');
quality.renderLiveQuality({motion_quality: {valid: true, feedback_age_s: 1, peak_joint_speed_rad_s: 9}});
assert.equal(nodes.get('quality-speed').textContent, '—');
const annotations = {revision: 2, annotation_status: 'needs_review', camera_id: 'cam', annotator: 'operator',
  input_kind: 'manual', segments: [{phase: 'grasp', start_frame: 1, end_frame: 5, evidence: ['contact'], confidence: .8}]};
const payload = {tasks: [{task_id: 'task-1', task_name: 'pick', status: 'failed', phases: [
  {phase: 'verify_grasp', status: 'failed', elapsed_s: 1, operation_id: 'op-1'}]}],
  episodes: [{id: 'episode-1', task: 'pick'}]};
let posted, annotationReply;
globalThis.fetch = async (url, options = {}) => {
  if (options.method === 'POST') {
    posted = JSON.parse(options.body);
    return {ok: true, json: async () => ({annotations: {...annotations, revision: 3, annotation_status: 'approved'}})};
  }
  if (url.includes('episode_id=')) return new Promise(resolve => {annotationReply = resolve;});
  return {ok: true, json: async () => ({...payload, annotations: null})};
};
const reviewModule = await module('task-review.js');
nodes.get('open-task-review').onclick();
await new Promise(resolve => setImmediate(resolve));
assert.equal(typeof annotationReply, 'function');
assert.equal(nodes.get('review-episode').disabled, true, 'Episode selection stays locked during second fetch');
assert.equal(nodes.get('review-refresh').disabled, true);
assert.equal(nodes.get('annotation-approve').disabled, true);
annotationReply({ok: true, json: async () => ({...payload, annotations})});
await new Promise(resolve => setImmediate(resolve));
assert.equal(nodes.get('review-episode').disabled, false);
assert.equal(nodes.get('task-timeline').children.length, 1);
assert.equal(nodes.get('annotation-timeline').children[0].children[0].textContent, 'grasp');
assert.equal(nodes.get('annotation-approve').disabled, false);
nodes.get('annotation-reviewer').value = 'tester';
await nodes.get('annotation-approve').onclick();
assert.equal(posted.expected_revision, 2);
assert.equal(posted.episode_id, 'episode-1');
assert.match(nodes.get('annotation-status').textContent, /版本 3/);
nodes.get('annotation-note').value = 'review draft';
payload.tasks[0].phases[0].status = 'completed';
await reviewModule.refreshTasks();
assert.equal(nodes.get('task-timeline').children[0].children[1].textContent, '执行完成');
assert.match(nodes.get('annotation-status').textContent, /版本 3/);
assert.equal(nodes.get('annotation-note').value, 'review draft');
const collection = await module('collection-view.js');
const active = {collection: {state: 'RECORDING'}, recording: {live: true, heartbeat_age_s: .2, heartbeat_ttl_s: 2,
  counts: {state: 10}, camera_counts: {front: 3, top: 0, side: 5}, queue_size: 0}};
const metrics = collection.collectionMetrics(active);
assert.equal(metrics.frames, '3 / 0 / 5');
assert.equal(metrics.frameTitle, 'front: 3 · top: 0 · side: 5');
assert.equal(metrics.progress, '写盘队列 0 / — · 丢弃 —');
assert.equal(collection.expireRecording(active, 2).collection.state, 'STALE');
assert.equal(active.collection.state, 'RECORDING');
console.log('Offline quality/review/live collection UI states passed');
