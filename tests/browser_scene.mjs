// Optional: npm install --prefix .cache/viewer-validation playwright@1.55.1
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import { mkdir, mkdtemp, writeFile, rm } from 'node:fs/promises';
import { createServer } from 'node:net';
import { fileURLToPath } from 'node:url';
import { chromium } from '../.cache/viewer-validation/node_modules/playwright/index.mjs';

const root = fileURLToPath(new URL('..', import.meta.url));
const output = `${root}/outputs/logs`;
await mkdir(output, { recursive: true });
const temp = await mkdtemp(`${root}/.cache/monitor-browser-`);
const portProbe = createServer();
portProbe.listen(0, '127.0.0.1');
await once(portProbe, 'listening');
const endpoint = `tcp://127.0.0.1:${portProbe.address().port}`;
await new Promise((resolve) => portProbe.close(resolve));
const config = `${temp}/local.toml`;
await writeFile(config, `[arm_safety]\ntable_enabled = false\n[telemetry]\nendpoint = "${endpoint}"\n[arm_service]\nsocket = "${temp}/arm.sock"\n[runtime]\nsocket = "${temp}/teleop.sock"\n[camera_service]\nsocket = "${temp}/camera.sock"\n`);
const hosts = [];
function start(script, args = []) {
  const child = spawn(`${root}/${script}`, args, { cwd: '/tmp', stdio: ['pipe', 'pipe', 'pipe'],
    env: { ...process.env, QUEST_TELEOP_CONFIG: config } });
  hosts.push(child);
  let log = '';
  const result = once(child, 'exit');
  child.stderr.on('data', (data) => { log += data; });
  child.stdout.on('data', (data) => { log += data; });
  function wait(pattern) {
    return new Promise((resolve, reject) => {
      const timer = setInterval(() => {
        const match = log.match(pattern);
        if (match) { clearInterval(timer); clearTimeout(timeout); resolve(match); }
        else if (child.exitCode !== null) { clearInterval(timer); clearTimeout(timeout); reject(new Error(log)); }
      }, 50);
      const timeout = setTimeout(() => { clearInterval(timer); reject(new Error(`Startup timeout: ${log}`)); }, 20000);
    });
  }
  return { child, result, wait, log: () => log };
}
const urlPattern = /Scene \(Three\.js[^\n]*\): (http:\/\/127\.0\.0\.1:\d+\/)/;
const browser = await chromium.launch({ executablePath: process.env.SCENE_TEST_BROWSER || undefined,
  headless: true, env: { ...process.env, DISPLAY: '', LD_LIBRARY_PATH: '' },
  args: ['--no-sandbox', '--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--disable-gpu-sandbox'] });
const errors = [];
let monitorStopped = false;
try {
  const page = await browser.newPage({ viewport: { width: 1440, height: 960 } });
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => {
    if (message.type() !== 'error') return;
    if (monitorStopped && /net::ERR_(CONNECTION_REFUSED|CONNECTION_RESET|EMPTY_RESPONSE|ABORTED|SOCKET_NOT_CONNECTED)/.test(message.text())) return;
    errors.push(message.text());
  });
  await page.route('**/*', async (route) => {
    if (!route.request().url().startsWith('http://127.0.0.1:')) {
      errors.push(`Unexpected external request: ${route.request().url()}`);
      await route.abort();
    } else await route.continue();
  });
  const monitor = start('启动仿真.sh', ['--port', '0', '--no-camera', '--no-connections']);
  const url = (await monitor.wait(urlPattern))[1];
  await page.goto(url);
  await page.waitForSelector('body[data-monitor-ready="true"]');
  await page.waitForFunction(() => document.getElementById('camera-status').textContent.includes('未连接'));
  const sample = { camera: { error: null }, preview: {
    age_s: .03, jpeg_base64: '' }, input_age_s: .02, input: { hands: {} } };
  sample.state_age_s = .02;
  sample.table_safety = {minimum_z_m: 0, blocked: true};
  sample.loop_rates = {ik: {observed_hz: 48.2, configured_hz: 50}, arm_service: {observed_hz: 46.7, configured_hz: 50}};
  sample.collection_count = 12;
  sample.collection = {state: "RECORDING", task: "Place object", error: null};
  sample.recording = {id: 'episode-test', task: 'Place object', duration_s: 65, counts: {frame: 1950, state: 6500}, queue_size: 2, dropped: {frame: 0, state: 0}};
  sample.connections = {arm: {duration_ms: 2.5}, quest_usb: {duration_ms: 4.1}};
  sample.timing = { arm_command: {age_s: .02, duration_ms: 2.5}, input_processing: {age_s: .03, duration_ms: 4.1} };
  sample.joint_limit = { age_s: .03, joints: [2, 3] };
  sample.motion_diagnostics = { age_s: .03, peak_ik_step: .01, ik_backlog: .27,
    command_tracking_error: .04, q_actual: [0, 0, 0, 0, 0, 0],
    q_des: [0, 0, 0, 0, 0, 0], q_safe: [0, 0, 0, 0, 0, 0] };
  sample.preview.jpeg_base64 = await page.evaluate(() => {
    const c = document.createElement('canvas'); c.width = 320; c.height = 240;
    const ctx = c.getContext('2d'); ctx.fillStyle = '#235366'; ctx.fillRect(0, 0, 320, 240);
    ctx.fillStyle = 'white'; ctx.fillText('Test RGB', 120, 120);
    return c.toDataURL('image/jpeg').split(',')[1];
  });
  sample.depth_preview = { ...sample.preview };
  for (const side of ['left', 'right']) sample.input.hands[side] = {
    pose: [.1, -.2, .3, 0, 0, 0, 1], trigger: .25, grip: .75,
    buttons: side === 'left' ? { X: true, Y: false } : { A: true, B: false }
  };
  let delayInput = false;
  await page.route('**/api/sensors', async route => {
    if (delayInput) await new Promise(resolve => setTimeout(resolve, 900));
    return route.fulfill({ json: sample });
  });
  sample.button_feedback = {B: {progress: .6, fired: false}, R3: {progress: .2}, down: {progress: .4}, Y: 'Y 已识别'};
  sample.collection.control_enabled = false;
  await page.waitForFunction(() => document.getElementById('control-state').textContent === '待机 · 未启用控制');
  assert.equal(await page.locator('#collection-state').textContent(), '● 采集中');
  assert.equal(await page.locator('#loop-rates').textContent(), 'IK 48.2/50 Hz · 服务 46.7/50 Hz');
  assert.ok((await page.locator('#table-safety').textContent()).includes('已拦截'));
  sample.table_safety.blocked = false;
  await page.waitForFunction(() => document.getElementById('table-safety').textContent.includes('软限位已启用'));
  sample.collection.control_enabled = true;
  await page.waitForFunction(() => document.getElementById('control-state').textContent === '控制已启用');
  sample.collection.control_enabled = false;
  await page.waitForFunction(() => document.getElementById('control-state').textContent === '待机 · 未启用控制');
  await page.waitForFunction(() => !document.getElementById('camera-image').hidden && !document.getElementById('depth-image').hidden &&
    document.querySelector('#right-hand [data-meter=grip]').textContent === '75%' &&
    document.getElementById('motion-diagnostics').textContent.includes('0.2700') &&
    document.getElementById('motion-joints').textContent.includes('限位 J2/J3') &&
    document.getElementById('arm-latency').textContent === '2.5 ms' && document.getElementById('input-latency').textContent === '4.1 ms' &&
    document.getElementById('collection-state').textContent === '● 采集中' && document.getElementById('collection-duration').textContent === '01:05' && document.getElementById('collection-frames').textContent === '1950' && document.getElementById('collection-count').textContent === '12');
  assert.equal(await page.locator('#right-hand [data-hold=B]').getAttribute('aria-valuenow'), '60');
  assert.equal(await page.locator('#button-response').textContent(), 'Y 已识别');
  const inputPanel = page.locator('.input-panel');
  await page.waitForFunction(() => document.querySelector('.input-panel').dataset.state === 'live');
  const liveColor = await inputPanel.locator('.input-badge').evaluate(e => getComputedStyle(e).color);
  await inputPanel.screenshot({path: output + '/quest-input-live.png'});
  const left = sample.input.hands.left;
  delete sample.input.hands.left;
  await page.waitForFunction(() => document.querySelector('.input-panel').dataset.state === 'partial');
  assert.equal(await page.locator('#left-hand').getAttribute('data-state'), 'offline');
  assert.equal(await page.locator('#right-hand').getAttribute('data-state'), 'live');
  await inputPanel.screenshot({path: output + '/quest-input-partial.png'});
  sample.input.hands.left = left;
  sample.input_age_s = .5;
  await page.waitForFunction(() => document.querySelector('.input-panel').dataset.state === 'stale');
  assert.notEqual(await inputPanel.locator('.input-badge').evaluate(e => getComputedStyle(e).color), liveColor);
  assert.equal(await page.locator('#right-hand [data-key=A]').getAttribute('data-pressed'), 'false');
  await inputPanel.screenshot({path: output + '/quest-input-stale.png'});
  sample.input.error = 'disconnected';
  await page.waitForFunction(() => document.querySelector('.input-panel').dataset.state === 'offline');
  await inputPanel.screenshot({path: output + '/quest-input-offline.png'});
  delete sample.input.error;
  sample.input_age_s = .02;
  await page.waitForFunction(() => document.querySelector('.input-panel').dataset.state === 'live');
  delayInput = true;
  await page.waitForFunction(() => document.querySelector('.input-panel').dataset.state === 'stale');
  delayInput = false;
  await page.waitForFunction(() => document.querySelector('.input-panel').dataset.state === 'live');

  for (const [width, height] of [[1440, 960], [1366, 768], [1024, 600]]) {
    await page.setViewportSize({ width, height });
    await page.waitForTimeout(250);
    assert.equal(await page.evaluate(() => {
      const all = [document.documentElement, document.body, document.querySelector('aside'), document.querySelector('.collection-panel'), document.querySelector('.collection-controls')];
      return all.some(e => e.scrollWidth > e.clientWidth + 1);
    }), false, 'Overflow at ' + width + 'x' + height);
    for (const id of ['collection-count', 'collection-state', 'collection-duration', 'collection-last', 'collection-health', 'camera-image', 'depth-image', 'left-hand', 'right-hand', 'joints', 'eef-position', 'state-source', 'motion-diagnostics', 'arm-latency', 'input-latency']) {
      const box = await page.locator('#' + id).boundingBox();
      assert.ok(box && box.height > 0 && box.x >= 0 && box.x + box.width <= width + 1 && box.y >= 0 && box.y + box.height <= height + 1, id + ' clipped');
    }
    await page.screenshot({ path: output + '/monitor-sensors-' + width + '.png' });
  }
  sample.recording = null;
  sample.collection.state = 'PAUSED';
  await page.waitForFunction(() => document.getElementById('collection-state').textContent === '任务采集已暂停');
  sample.collection.state = 'IDLE';
  sample.recording = null;
  await page.waitForFunction(() => document.getElementById('collection-state').textContent === '任务采集未开启' && document.getElementById('collection-health').textContent.includes('预抓取位'));
  sample.collection_count = 0;
  sample.recording = null;
  sample.collection.state = 'READY';
  sample.last_episode = {id: 'episode-test', lifecycle: 'complete', outcome: 'discarded'};
  await page.waitForFunction(() => document.getElementById('collection-state').textContent === '准备就绪' && document.getElementById('collection-count').textContent === '0' && document.getElementById('collection-frames').textContent === '—' && document.getElementById('collection-last').textContent.includes('已丢弃'));
  sample.collection.error = '测试写盘失败';
  await page.waitForFunction(() => document.getElementById('collection-state').textContent === '采集异常' && document.getElementById('collection-health').textContent === '测试写盘失败');
  sample.collection.error = null;
  sample.depth_preview.age_s = 2; sample.preview.age_s = 2; sample.input_age_s = 2; sample.motion_diagnostics.age_s = 2; sample.joint_limit.age_s = 2; sample.timing.arm_command.age_s = 2; sample.timing.input_processing.age_s = 2; sample.connections = {};
  await page.waitForFunction(() => document.getElementById('camera-image').hidden && document.getElementById('depth-image').hidden &&
    document.querySelector('.input-panel').dataset.state === 'stale' &&
    document.querySelector('#right-hand [data-meter=grip]').textContent === '—' && document.querySelector('#right-hand [data-key=A]').dataset.pressed === 'false' &&
    document.getElementById('motion-diagnostics').textContent === '—' && document.getElementById('arm-latency').textContent === '未测' && document.getElementById('input-latency').textContent === '未测');
  await page.setViewportSize({ width: 1440, height: 960 });
  // Attitude preview uses the Cartesian task, and never leaves stale ghosts visible.
  const identity = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
  sample.state_age_s = .01; sample.input_age_s = .01;
  sample.orientation_preview = {basis: [[0,0,-1],[-1,0,0],[0,1,0]], input_ready: true, active: true, target: identity};
  await page.unroute('**/api/sensors');
  await page.route('**/api/sensors', route => {
    sample.input.device_timestamp_ns = Date.now() * 1000000;
    return route.fulfill({json: sample});
  });
  const previewState = await page.evaluate(async () => (await (await fetch('/api/model')).json()).reference);
  previewState.actual.eef = identity;
  previewState.stream = {status: 'live', source: {}, stream_id: 'preview-test'};
  await page.route('**/api/state', route => route.fulfill({json: previewState}));
  await page.waitForFunction(() => document.getElementById('orientation-canvas').dataset.handVisible === 'true' &&
    document.getElementById('orientation-error').textContent.endsWith('0.0°'));
  sample.orientation_preview.target = [0,1,0,0, -1,0,0,0, 0,0,1,0, 0,0,0,1];
  await page.waitForFunction(() => document.getElementById('orientation-error').textContent.endsWith('90.0°'));
  await page.screenshot({path: output + '/monitor-orientation.png'});
  sample.orientation_preview.active = false;
  await page.waitForFunction(() => document.getElementById('orientation-canvas').dataset.targetVisible === 'false' &&
    document.getElementById('orientation-canvas').dataset.handVisible === 'true');
  sample.state_age_s = 2;
  await page.waitForFunction(() => document.getElementById('orientation-canvas').dataset.handVisible === 'false');
  const replayFixture = {replay: {episode: 'review-test', generation: 1, playing: true,
    duration_s: 2, position_s: .5, q: [0, 0, 0, 0, 0, 0], gripper_m: .04,
    jpeg_base64: sample.preview.jpeg_base64, pose: previewState.actual}, robot_replay: {phase: 'idle'}};
  const replayActions = [];
  await page.route('**/api/replay', async route => {
    if (route.request().method() === 'POST') {
      const command = route.request().postDataJSON(); replayActions.push(command);
      if (command.action === 'pause') replayFixture.replay.playing = false;
      if (command.action === 'seek') replayFixture.replay.position_s = command.position_s;
      return route.fulfill({json: {ok: true}});
    }
    return route.fulfill({json: replayFixture});
  });
  await page.waitForFunction(() => !document.getElementById('dataset-replay').hidden && document.getElementById('replay-status').textContent.includes('review-test'));
  await page.waitForFunction(() => document.getElementById('replay-video').naturalWidth > 0);
  await page.click('#replay-pause');
  await page.waitForFunction(() => document.getElementById('replay-status').textContent.includes('暂停'));
  await page.locator('#replay-seek').evaluate(input => { input.value = '1.5'; input.dispatchEvent(new Event('change')); });
  await page.waitForFunction(() => document.getElementById('replay-time').textContent.startsWith('1.50'));
  assert.ok(replayActions.some(command => command.action === 'seek' && command.position_s === 1.5));
  await page.screenshot({path: output + '/dataset-replay.png'});
  assert.equal(await page.locator('details').count(), 0);
  assert.equal(await page.locator('#dataset-replay').isVisible(), true);
  await page.unroute('**/api/replay');
  await page.unroute('**/api/state');
  await page.waitForFunction(() => document.getElementById('connection').dataset.state === 'waiting');
  await page.unroute('**/api/sensors');
  await page.setViewportSize({ width: 1440, height: 960 });

  assert.equal(await page.locator('#connection').getAttribute('data-state'), 'waiting');
  await page.screenshot({ path: `${output}/monitor-waiting.png` });
  await page.evaluate(() => { document.documentElement.dataset.session = 'same-page'; });

  const service = start('run.sh', ['arm-service', '--backend', 'simulated']);
  await service.wait(/Arm service ready:/);
  await page.waitForFunction(() => document.getElementById('state-source').textContent.includes('阻尼待机')
    && document.getElementById('connection').dataset.state === 'connected');
  const terminal = start('run.sh', ['control-arm']);
  await terminal.wait(/Commands \(rad\/m\)/);
  assert.ok(!terminal.log().includes('Scene ('), 'Control unexpectedly started a viewer');
  await page.waitForFunction(() => document.getElementById('connection').dataset.state === 'connected');
  const canvas = page.locator('#robot-canvas');
  await page.waitForTimeout(200);
  assert.equal(await canvas.getAttribute('data-feedback'), 'live');
  assert.equal(await canvas.getAttribute('data-source'), 'virtual');
  const initial = await canvas.screenshot();
  const firstStream = await page.evaluate(async () => (await (await fetch('/api/state')).json()).stream.stream_id);
  terminal.child.stdin.write('q 0.15 1.5 1.45 -1.52 0 0\n');
  await page.waitForFunction(() => document.querySelector('.joint-row span:nth-child(2)')?.textContent === '8.59');
  assert.notDeepEqual(await canvas.screenshot(), initial, 'Arm rendering did not move');
  await page.screenshot({ path: `${output}/monitor-terminal.png` });
  await page.locator('#show-table').uncheck();
  const noTable = await canvas.screenshot();
  await page.locator('#show-table').check();
  assert.notDeepEqual(await canvas.screenshot(), noTable, 'Table toggle did not change the scene');
  await page.setViewportSize({ width: 760, height: 900 });
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
  await page.setViewportSize({ width: 1440, height: 960 });
  terminal.child.stdin.end('exit\n');
  assert.deepEqual(await terminal.result, [0, null], terminal.log());
  await page.waitForFunction(() => document.getElementById('state-source').textContent.includes('阻尼待机')
    && document.getElementById('connection').dataset.state === 'connected');
  assert.equal(monitor.child.exitCode, null, 'Monitor exited with controller');

  const quest = start('run.sh', ['control-arm', '--input', 'idle']);
  await quest.wait(/Control client ready:/);
  await page.waitForFunction(() => document.getElementById('state-source').textContent.includes('idle')
    && document.getElementById('connection').dataset.state === 'connected');
  assert.equal(page.url(), url);
  assert.equal(await page.evaluate(() => document.documentElement.dataset.session), 'same-page');
  const nextStream = await page.evaluate(async () => (await (await fetch('/api/state')).json()).stream.stream_id);
  assert.equal(firstStream, nextStream, "Control replacement restarted the arm service");
  await page.screenshot({ path: `${output}/monitor-pose-input.png` });

  monitorStopped = true;
  monitor.child.kill('SIGINT');
  assert.deepEqual(await monitor.result, [0, null], monitor.log());
  assert.equal(quest.child.exitCode, null, 'Closing monitor stopped controller');
  const laterMonitor = start('启动仿真.sh', ['--port', '0', '--no-camera', '--no-connections']);
  await page.goto((await laterMonitor.wait(urlPattern))[1]);
  monitorStopped = false;
  await page.waitForFunction(() => document.getElementById('connection').dataset.state === 'connected');
  assert.equal(await page.evaluate(async () => (await (await fetch('/api/state')).json()).stream.stream_id), nextStream);
  quest.child.kill('SIGINT');
  assert.deepEqual(await quest.result, [0, null], quest.log());
  await page.waitForFunction(() => document.getElementById('state-source').textContent.includes('阻尼待机')
    && document.getElementById('connection').dataset.state === 'connected');
  const crashed = start('run.sh', ['control-arm']);
  await crashed.wait(/Commands \(rad\/m\)/);
  crashed.child.kill('SIGKILL');
  await crashed.result;
  await page.waitForFunction(() => document.getElementById('state-source').textContent.includes('阻尼待机')
    && document.getElementById('connection').dataset.state === 'connected');
  service.child.kill('SIGINT');
  assert.deepEqual(await service.result, [0, null], service.log());
  await page.waitForFunction(() => document.getElementById('connection').dataset.state === 'stale');
  assert.equal(await canvas.getAttribute('data-feedback'), 'stale');
  await page.screenshot({path: output + '/arm-feedback-stale.png'});
  const restartedService = start('run.sh', ['arm-service', '--backend', 'simulated']);
  await restartedService.wait(/Arm service ready:/);
  await page.waitForFunction(() => document.getElementById('connection').dataset.state === 'connected');
  assert.notEqual(await page.evaluate(async () => (await (await fetch('/api/state')).json()).stream.stream_id), nextStream);
  restartedService.child.kill('SIGINT');
  assert.deepEqual(await restartedService.result, [0, null], restartedService.log());
  monitorStopped = true;
  laterMonitor.child.kill('SIGINT');
  assert.deepEqual(await laterMonitor.result, [0, null], laterMonitor.log());
  await writeFile(`${output}/arm-service-browser-cli.log`, [service, monitor, terminal, quest, laterMonitor, crashed, restartedService].map((x) => x.log()).join('\n'));
  assert.deepEqual(errors, []);
  console.log('PASS: standby feedback before control, terminal motion, controller replacement, SIGKILL watchdog, independent monitor restart, service restart and stale status.');
} finally {
  for (const child of hosts) if (child.exitCode === null && child.signalCode === null) child.kill('SIGINT');
  await browser.close();
  await rm(temp, { recursive: true, force: true });
}
