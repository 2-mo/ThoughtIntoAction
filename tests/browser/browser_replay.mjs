// Isolated read-only scene with recorded fixtures; no camera or device services.
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {chromium} from '../../.cache/viewer-validation/node_modules/playwright/index.mjs';
const server = spawn('.venv/bin/python', ['-u', '-c', `from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from embodied.applications.monitor.scene import Scene
import threading
scene = Scene(X5PinocchioModel())
threading.Event().wait()`], {env:{...process.env, PYTHONPATH:'src'}, stdio:['ignore','pipe','pipe']});
let browser;
try {
  const url = await new Promise((resolve, reject) => {
    let log = '';
    const timeout = setTimeout(() => reject(Error('Scene startup timeout: '+log)), 15000);
    server.stdout.on('data', chunk => {log += chunk; const match = log.match(/http:\/\/127\.0\.0\.1:\d+\//); if (match) {clearTimeout(timeout); resolve(match[0]);}});
    server.once('exit', code => {clearTimeout(timeout); reject(Error('Scene exited: '+code));});
  });
  const model = await (await fetch(url+'api/model')).json();
  browser = await chromium.launch({headless:true, env:{...process.env, DISPLAY:'', LD_LIBRARY_PATH:''}, args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader','--disable-gpu-sandbox']});
  const page = await browser.newPage({viewport:{width:1920,height:1080}});
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  let frame = null;
  const reference = model.reference.actual;
  const live = {...model.reference, actual_vel:[.1,0,0,0,0,0], target_vel:[.2,0,0,0,0,0], gripper_actual_vel:.001,
    eef_velocity:{speed:.1,angular_speed:.2}, stream:{status:'live',age_s:0,sequence:1,stream_id:'live',source:{kind:'virtual'}}};
  await page.route('**/api/state', route => route.fulfill({json:live}));
  const sensors = {last_episode:{id:'dual-fixture',lifecycle:'complete',outcome:'unknown',quality:{status:'checking'}}};
  await page.route('**/api/sensors', route => route.fulfill({json:sensors}));
  const actions = [];
  await page.route('**/api/replay*', async route => {
    if (route.request().method() === 'POST') {
      const command = route.request().postDataJSON(); actions.push(command.action);
      if (command.action === 'load') frame = {pose:reference, q:reference.q, target_q:null, gripper_m:.04, gripper_max:.082,
        sample_index:2, sample_count:10, generation:1, position_s:1, duration_s:5, playing:false,
        episode:'fixture', record_format:'mcap', actual_vel:[.5,0,0,0,0,0], gripper_actual_vel:.002,
        eef_velocity:{speed:.2,angular_speed:.3}, actual_torque:[1,0,0,0,0,0],
        torque_history:[{time:0,values:[0,0,0,0,0,0,null]},{time:1000,values:[1,0,0,0,0,0,null]}]};
      if (command.action === 'seek') frame.position_s = command.position_s;
      if (command.action === 'speed') frame.speed = command.speed;
      return route.fulfill({json:{ok:true}});
    }
    return route.fulfill({json:route.request().url().includes('list=1') ? {episodes:[{id:'fixture'}]} : {replay:frame}});
  });
  await page.goto(url);
  await page.waitForSelector('body[data-ready="true"]', {timeout:15000}).catch(async error => {throw Error(error.message + '\n' + errors.join('\n') + '\n' + await page.locator('#loading').textContent());});
  await page.waitForFunction(() => document.querySelector('#velocities b').textContent === '5.7');
  await page.waitForFunction(() => document.querySelector('#collection-last').textContent.includes('检查中'));
  sensors.last_episode.quality = {status:'failed',state_count:240,state_hz:240,state_max_gap_s:.005,
    cameras:{wrist_rgb:{frame_count:30,frame_hz:30,max_gap_s:.034,missing_source_frames:0,receive_time_alignment_max_s:.004},
      exterior_rgb:{frame_count:28,frame_hz:28,max_gap_s:.10,missing_source_frames:2,receive_time_alignment_max_s:.005}},
    errors:['exterior_rgb: Missing source camera frames'], warnings:['任务结果尚未确认']};
  await page.waitForFunction(() => document.querySelector('#collection-last').textContent.includes('有问题'));
  await page.click('#collection-last');
  assert.equal(await page.locator('#quality-details').isVisible(), true);
  assert.equal(await page.locator('#quality-streams tr').count(), 3);
  assert.match(await page.locator('#quality-messages').textContent(), /Missing source camera frames/);
  await page.click('#quality-close');
  assert.equal(await page.evaluate(() => document.documentElement.scrollHeight <= innerHeight && document.documentElement.scrollWidth <= innerWidth),true);
  assert.equal(await page.locator('#replay-canvas').count(), 0);
  assert.equal(await page.locator('#dataset-replay').isVisible(), false);
  await page.click('#open-replay');
  await page.waitForFunction(() => document.querySelector('#replay-episodes').options.length === 1);
  await page.click('#replay-load');
  await page.waitForFunction(() => document.querySelector('#robot-canvas').dataset.source === 'replay');
  await page.waitForFunction(() => document.querySelector('#velocities b').textContent === '28.6');
  assert.equal(await page.locator('#camera-grid').isVisible(), false);
  assert.equal(await page.locator('#recorded-camera').isVisible(), true);
  assert.equal(await page.locator('#replay-video-empty').textContent(), '该条未录制 RGB');
  const jpeg = await page.evaluate(() => {
    const canvas = document.createElement('canvas'); canvas.width=64; canvas.height=48;
    const context=canvas.getContext('2d');context.fillStyle='#498071';context.fillRect(0,0,64,48);
    return canvas.toDataURL('image/jpeg').split(',')[1];
  });
  frame.image_base64 = jpeg; frame.image_mime='image/jpeg';
  frame.cameras = {wrist_rgb:{image_base64:jpeg,image_mime:'image/jpeg'},exterior_rgb:{image_base64:jpeg,image_mime:'image/jpeg'}};
  await page.waitForFunction(() => !document.querySelector('#replay-exterior-video').hidden && document.querySelector('#replay-exterior-video').naturalWidth===64);
  assert.equal(await page.locator('#recorded-exterior').isVisible(),true);
  assert.equal(await page.locator('#recorded-grid').getAttribute('data-count'),'2');
  frame.cameras.exterior_rgb.image_base64 = null;
  await page.waitForFunction(() => !document.querySelector('#replay-exterior-video').hasAttribute('src'));
  assert.equal(await page.locator('#replay-exterior-empty').isVisible(),true);
  frame.cameras.exterior_rgb.image_base64 = jpeg;
  const trace = await page.locator('.torque-line').first().getAttribute('d');
  await page.waitForTimeout(350);
  assert.equal(await page.locator('.torque-line').first().getAttribute('d'), trace);
  await page.selectOption('#replay-speed', '0.5');
  await page.locator('#replay-seek').fill('3');
  await page.locator('#replay-seek').dispatchEvent('change');
  await page.waitForFunction(() => document.querySelector('#replay-time').textContent.startsWith('3.00'));
  await page.screenshot({path:'.cache/viewer-validation/unified-replay.png'});
  await page.click('#view-live');
  await page.waitForFunction(() => document.querySelector('#robot-canvas').dataset.source === 'virtual');
  await page.waitForFunction(() => document.querySelector('#velocities b').textContent === '5.7');
  assert.equal(await page.locator('#recorded-camera').isVisible(), false);
  assert(actions.every(a => ['load','seek','speed'].includes(a)));
  assert.deepEqual(errors, []);
  console.log('PASS: dual RGB replay, missing-frame clearing, quality details, 1920x1080 single screen, legacy replay and shared telemetry.');
} finally {await browser?.close(); server.kill('SIGTERM');}
