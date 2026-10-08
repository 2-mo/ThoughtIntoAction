// Isolated viewer lifecycle: a real-mode observer (no controller) and native MuJoCo.
import assert from 'node:assert/strict';
import {execFile} from 'node:child_process';
import {promisify} from 'node:util';
import {mkdtemp, writeFile, rm, readdir, readFile} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import net from 'node:net';
import {chromium} from '../.cache/viewer-validation/node_modules/playwright/index.mjs';
const exec = promisify(execFile);
const folder = await mkdtemp(tmpdir() + '/questarm-viewer-mode-');
const freePort = () => new Promise(resolve => {
  const socket = net.createServer();
  socket.listen(0, '127.0.0.1', () => {const port = socket.address().port; socket.close(() => resolve(port));});
});
const port = await freePort();
const feedbackPort = await freePort();
const config = folder + '/config.toml';
await writeFile(config, `[viewer_service]\nport=${port}\ncamera=false\nprepare_quest=false\nsocket="${folder}/viewer.sock"\n[paths]\nlog_dir="${folder}"\ndataset_dir="${folder}/data"\n[runtime]\nsocket="${folder}/teleop.sock"\n[camera_service]\nsocket="${folder}/camera.sock"\n[arm_service]\nsocket="${folder}/arm.sock"\n[telemetry]\nendpoint="tcp://127.0.0.1:${feedbackPort}"\n`);
const env = {...process.env, QUEST_TELEOP_CONFIG:config, PYTHONPATH:'src'};
const command = action => exec('./run.sh', ['viewer-control', action], {env, timeout:40000});
async function select(mode) {
  await writeFile(folder + '/workbench-mode.txt', mode + '\n');
  await command('start');
}
let browser;
const url = `http://127.0.0.1:${port}/`;
try {
  await select('real');
  browser = await chromium.launch({headless:true, env:{...process.env, DISPLAY:'', LD_LIBRARY_PATH:''}, args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader','--disable-gpu-sandbox']});
  const page = await browser.newPage({viewport:{width:1920,height:1080}});
  async function until(check, timeout=25000) {
    const deadline = performance.now()+timeout;
    while (!await page.evaluate(check)) {
      if (performance.now()>deadline) throw Error('State condition timed out: '+String(check));
      await page.waitForTimeout(50);
    }
  }
  const errors = []; page.on('pageerror', error => errors.push(error.message));
  await page.goto(url);
  await page.waitForFunction(() => document.body.dataset.monitorReady === 'true');
  const before = await page.evaluate(() => fetch('/api/model').then(r=>r.json()));
  const gap = await page.evaluate(() => document.querySelector('.display-panel').getBoundingClientRect().top - document.querySelector('.orientation-panel').getBoundingClientRect().bottom);
  assert.ok(gap > 150, `Leave natural space in Quest card: ${gap}`);
  const firstStatus = JSON.parse((await exec('.venv/bin/python', ['-c', 'import json;from robot_simulation.control import status;print(json.dumps(status()))'], {env})).stdout);
  await select('simulation');
  await page.waitForFunction(() => document.body.dataset.simulation === 'true' && document.body.dataset.monitorReady === 'true');
  await until(async () => (await fetch('/api/simulation').then(r=>r.json())).state === 'running', 25000);
  assert.equal(page.url(), url);
  assert.equal(await page.locator('.hands').isVisible(), true);
  assert.equal(await page.locator('.display-panel').isVisible(), true);
  assert.equal(await page.locator('.simulation-actions').count(), 0);
  const sim = await page.evaluate(() => fetch('/api/model').then(r=>r.json()));
  assert.notEqual(before.viewer_id, sim.viewer_id);
  assert.ok(sim.simulation);
  const clipped = await page.evaluate(() => [...document.querySelectorAll('main, .stage, .input-panel, .camera-panel, .telemetry-group')].filter(e=>e.scrollHeight>e.clientHeight+2).map(e=>e.className));
  assert.deepEqual(clipped, []);
  await command('sim-pregrasp');
  await until(async () => {
    const state = await fetch('/api/state').then(r=>r.json());
    const sensors = await fetch('/api/sensors').then(r=>r.json());
    if (sensors.collection?.error) throw Error(sensors.collection.error);
    return state.actual?.q && Math.abs(state.actual.q[1]-1.5)<.03 && sensors.collection?.state !== 'POSITIONING';
  }, 20000);
  await command('sim-record');
  await until(async () => (await fetch('/api/sensors').then(r=>r.json())).recording?.counts.state > 30);
  await command('sim-save');
  const episodes = await readdir(folder+'/data/mujoco');
  assert.equal(episodes.length, 1);
  const manifest = JSON.parse(await readFile(folder+'/data/mujoco/'+episodes[0]+'/episode.json'));
  assert.equal(manifest.lifecycle, 'complete');
  assert.equal(manifest.runtime.parameters.backend, 'sim');
  assert.ok(manifest.counts.state > 30, JSON.stringify({counts:manifest.counts, reason:manifest.stop_reason, error:manifest.error}));
  assert.equal(manifest.camera, null);
  await command('sim-pause');
  await until(async () => (await fetch('/api/simulation').then(r=>r.json())).state === 'paused');
  await command('sim-resume');
  await until(async () => (await fetch('/api/simulation').then(r=>r.json())).state === 'running', 25000);
  const runtime = await page.evaluate(() => fetch('/api/simulation').then(r=>r.json()));
  const started = performance.now();
  await select('real');
  const switchedMs = performance.now()-started;
  await page.waitForFunction(() => document.body.dataset.simulation === 'false' && document.body.dataset.monitorReady === 'true');
  assert.equal(page.url(), url);
  assert.equal(await page.locator('#simulation-heading').count(), 0);
  await select('simulation');
  await page.waitForFunction(() => document.body.dataset.simulation === 'true');
  const restored = await page.evaluate(() => fetch('/api/simulation').then(r=>r.json()));
  assert.equal(restored.runtime_id, runtime.runtime_id, 'Retain the MuJoCo runtime across selection');
  const finalStatus = JSON.parse((await exec('.venv/bin/python', ['-c', 'import json;from robot_simulation.control import status;print(json.dumps(status()))'], {env})).stdout);
  assert.equal(firstStatus.pid, finalStatus.pid, 'Viewer process must survive selection');
  assert.ok(switchedMs < 2000, `Switch must not park/restart: ${switchedMs}`);
  assert.deepEqual(errors, []);
  console.log(`Warm switch: ${switchedMs.toFixed(0)} ms; same viewer PID and MuJoCo runtime.`);
  console.log('PASS: same URL real→simulation→real, auto reload, preset motion, original-tick recording, teleop pause/resume, Quest whitespace and 1920×1080 layout.');
} finally {
  if (browser) await browser.close();
  await command('stop');
  await rm(folder, {recursive:true, force:true});
}
