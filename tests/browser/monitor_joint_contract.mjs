// Exercise telemetry row creation with a seven-axis, gripper-free runtime contract.
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const root = new URL('../../src/embodied/applications/monitor/web/', import.meta.url);
const html = await readFile(new URL('index.html', root), 'utf8');
class Node {
  children = []; innerHTML = ''; textContent = ''; value = ''; dataset = {}; style = {}; hidden = false;
  classList = {toggle() {}, add() {}};
  selected = new Map();
  append(value) {this.children.push(value);}
  replaceChildren(...children) {this.children = children;}
  setAttribute() {}
  addEventListener() {}
  querySelector(key) {if (!this.selected.has(key)) this.selected.set(key, new Node()); return this.selected.get(key);}
  querySelectorAll(key) {
    const token = key === 'b' ? '<b[ >]' : key === 'td' ? '<td[ >]' : null;
    const count = token ? [...this.innerHTML.matchAll(new RegExp(token, 'g'))].length : key === '.motor-track i' ? 3 : 0;
    return Array.from({length: count}, (_, i) => this.querySelector(key + i));
  }
}
const nodes = new Map([...html.matchAll(/id="([^"]+)"/g)].map(match => [match[1], new Node()]));
// The static gripper display has two value cells even while hidden.
nodes.get('gripper').innerHTML = '<b>—</b><b>—</b>';
for (const id of ['velocity-zoom', 'torque-zoom']) nodes.get(id).value = '1';
globalThis.document = {hidden: false,
  getElementById(id) {assert.ok(nodes.has(id), `Missing element: ${id}`); return nodes.get(id);},
  createElement() {return new Node();}, createElementNS() {return new Node();}};
globalThis.window = {addEventListener() {}};
let render;
globalThis.setInterval = callback => {render = callback; return 0;};
globalThis.setTimeout = () => 0;
const q = Array.from({length: 7}, (_, i) => .1 * (i + 1));
const model = {viewer_id: 'seven', robot: {joint_names: q.map((_, i) => `joint${i + 1}`)},
  data_contract: {joint_names: q.map((_, i) => `joint${i + 1}`), gripper: null},
  joint_limits: {lower: q.map(() => -3), upper: q.map(() => 3)},
  velocity_limits: q.map(() => 2), torque_limits: q.map(() => 20)};
const state = {viewer_id: 'seven', actual: {q}, target: {q}, actual_vel: q,
  stream: {status: 'live', age_s: 0, sequence: 1, stream_id: 'run'}};
globalThis.fetch = async url => ({ok: true, json: async () => url === '/api/model' ? model : state});
const source = (await readFile(new URL('telemetry.js', root), 'utf8')).replace(
  /^import .*display-state\.js';$/m, "const mode = 'live', replayFrame = null, displayedState = value => value;");
await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
await new Promise(resolve => setImmediate(resolve));
render();
for (const id of ['joints', 'velocities', 'command-feedback', 'torques', 'motor-feedback'])
  assert.equal(nodes.get(id).children.length, 7, `${id} must retain every declared axis without a gripper row`);
assert.equal(nodes.get('joints').children[6].dataset.jointName, 'joint7');
assert.equal(nodes.get('joints').children[6].querySelectorAll('b')[0].textContent, '40.11°');
assert.equal(nodes.get('velocities').children[6].querySelectorAll('b')[0].textContent, '40.1');
assert.equal(nodes.get('gripper').hidden, true);
assert.equal(nodes.get('gripper-velocity').hidden, true);
console.log('Seven-axis gripper-free telemetry DOM passed');
