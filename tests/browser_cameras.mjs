// Isolated multi-camera UI: no devices opened, all camera controls intercepted.
import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {chromium} from '../.cache/viewer-validation/node_modules/playwright/index.mjs';
const server = spawn('.venv/bin/python', ['-u', '-B', '-c', `from robot_core.model import RobotModel
from robot_simulation.scene import Scene
import threading
scene = Scene(RobotModel())
threading.Event().wait()`], {env:{...process.env, PYTHONPATH:'src'}, stdio:['ignore','pipe','pipe']});
let browser;
try {
  const url = await new Promise((resolve, reject) => {
    let log = '';
    const timeout = setTimeout(() => reject(Error('Scene startup timeout: '+log)), 15000);
    server.stderr.on('data', chunk => {log += chunk;});
    server.stdout.on('data', chunk => {log += chunk; const match = log.match(/http:\/\/127\.0\.0\.1:\d+\//); if (match) {clearTimeout(timeout); resolve(match[0]);}});
    server.once('exit', code => {clearTimeout(timeout); reject(Error('Scene exited: '+code+' '+log));});
  });
  browser = await chromium.launch({headless:true, env:{...process.env, DISPLAY:'', LD_LIBRARY_PATH:''}, args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader','--disable-gpu-sandbox']});
  const page = await browser.newPage({viewport:{width:1920,height:1080}});
  const errors = []; page.on('pageerror', error => errors.push(error.message));
  const jpeg = await page.evaluate(() => {
    const c = document.createElement('canvas'); c.width=640; c.height=480;
    const ctx=c.getContext('2d'); ctx.fillStyle='#315d65'; ctx.fillRect(0,0,640,480);
    return c.toDataURL('image/jpeg').split(',')[1];
  });
  const feed = (id, model) => ({camera:{id,label:id === 'wrist_rgb' ? '腕部相机' : '外部相机',model,serial:id,
    enabled:true,switch_protocol:1,error:null,metadata:{serial:id,model,width:640,height:480,fps:30,rgb_exposure_unit_us:100,rgb_controls:{enable_auto_white_balance:0,white_balance:4600,enable_auto_exposure:0,exposure:166,gain:64,contrast:50,saturation:64,gamma:300}}},
    preview:{age_s:.01,jpeg_base64:jpeg},depth_preview:{age_s:.01,jpeg_base64:jpeg}});
  const sample = {cameras:{wrist_rgb:feed('wrist_rgb','D435I'), exterior_rgb:feed('exterior_rgb','D435')}};
  const controls = []; let lag = false, serviceDown = false;
  await page.route('**/api/sensors', async route => {
    const data=structuredClone(sample);
    if (lag) await new Promise(resolve => setTimeout(resolve,900));
    await route.fulfill({json:serviceDown ? {camera_error:'相机服务未连接'} : data});
  });
  await page.route('**/api/camera', route => {
    const data = route.request().postDataJSON(); controls.push(data);
    const camera=sample.cameras[data.camera_id].camera;
    camera.enabled=data.enabled; camera.error=data.enabled ? null : '相机已关闭';
    return route.fulfill({json:{ok:true}});
  });
  await page.route('**/api/replay*', route => route.fulfill({json:{}}));
  await page.goto(url);
  await page.waitForFunction(() => document.querySelectorAll('.camera-card[data-state="live"]').length === 2);
  const wrist=page.locator('[data-camera-id="wrist_rgb"]'), exterior=page.locator('[data-camera-id="exterior_rgb"]');
  assert.equal(await page.locator('.camera-stream[data-stream="rgb"]:visible').count(), 2);
  assert.equal(await wrist.locator('.camera-color-settings').textContent(), '白平衡 4600 K · 曝光 16.6 ms · 增益 64');
  assert.equal(await wrist.locator('.camera-color-settings').isVisible(), true);
  assert.equal(await page.locator('.camera-stream[data-stream="depth"]:visible').count(), 0);
  await page.click('#depth-toggle');
  assert.equal(await page.locator('.camera-stream[data-stream="depth"]:visible').count(), 2);
  assert.equal(await page.locator('.camera-stream[data-stream="rgb"]:visible').count(), 0);
  await exterior.locator('[data-stream="depth"]').click();
  assert.equal(await page.locator('#camera-dialog').isVisible(), true);
  assert.equal(await page.locator('#camera-dialog-title').textContent(), '外部相机 · 深度');
  await page.keyboard.press('Escape');
  assert.equal(await page.locator('#camera-dialog').isVisible(), false);
  await page.click('#depth-toggle');
  await exterior.locator('.camera-toggle').click();
  await page.waitForFunction(() => document.querySelector('[data-camera-id="exterior_rgb"]').dataset.state === 'off');
  assert.deepEqual(controls, [{camera_id:'exterior_rgb',enabled:false}]);
  assert.equal(await wrist.getAttribute('data-state'), 'live');
  assert.equal(await exterior.locator('img[src]').count(), 0);
  await exterior.locator('.camera-toggle').click();
  await page.waitForFunction(() => document.querySelectorAll('.camera-card[data-state="live"]').length === 2);
  sample.recording={id:'test'};
  await page.waitForFunction(() => [...document.querySelectorAll('.camera-toggle')].every(button => button.disabled));
  delete sample.recording;
  sample.cameras.exterior_rgb.camera.error='设备断开';
  await page.waitForFunction(() => document.querySelector('[data-camera-id="exterior_rgb"]').dataset.state === 'error');
  assert.equal(await wrist.getAttribute('data-state'), 'live');
  assert.equal(await exterior.locator('img[src]').count(), 0);
  sample.cameras.exterior_rgb.camera.error=null;
  await page.waitForFunction(() => document.querySelectorAll('.camera-card[data-state="live"]').length === 2);
  await wrist.locator('[data-stream="rgb"]').click();
  lag=true;
  await page.waitForFunction(() => document.getElementById('camera-dialog-image').hidden);
  assert.equal(await page.locator('.camera-card[data-state="live"]').count(), 0);
  await page.keyboard.press('Escape'); lag=false;
  await page.waitForFunction(() => document.querySelectorAll('.camera-card[data-state="live"]').length === 2);
  for (const [width,height] of [[1920,1080]]) {
    await page.setViewportSize({width,height});
    await page.waitForTimeout(150);
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth + 1), false, `Horizontal overflow at ${width}`);
    assert.equal(await page.evaluate(() => document.documentElement.scrollHeight > innerHeight + 1), false, `Vertical overflow at ${width}x${height}`);
    assert.equal(await page.locator('.camera-stream[data-stream="rgb"]:visible').count(), 2);
    assert.equal(await page.locator('.camera-card').evaluateAll(nodes => nodes.some(node => node.scrollWidth > node.clientWidth + 1)), false);
    const overlaps = await page.evaluate(() => {
      const nodes=[...document.querySelectorAll('.stage, .camera-panel, .input-panel, .telemetry-group, .collection-panel')];
      return nodes.some((a,i) => nodes.slice(i+1).some(b => {
        const x=a.getBoundingClientRect(), y=b.getBoundingClientRect();
        return x.left<y.right-1 && x.right>y.left+1 && x.top<y.bottom-1 && x.bottom>y.top+1;
      }));
    });
    assert.equal(overlaps,false,`Dashboard overlap at ${width}`);
    const clipped = await page.evaluate(() => [...document.querySelectorAll('.stage, .camera-panel, .camera-card, .input-panel, .telemetry-group, .telemetry-group > section, .collection-panel')].filter(node => {
      const box=node.getBoundingClientRect();
      return box.top < -1 || box.left < -1 || box.right > innerWidth + 1 || box.bottom > innerHeight + 1 ||
        node.scrollHeight > node.clientHeight + 1 || node.scrollWidth > node.clientWidth + 1;
    }).map(node => ({name:node.className,height:node.clientHeight,scroll:node.scrollHeight,width:node.clientWidth,scrollWidth:node.scrollWidth})));
    assert.deepEqual(clipped,[],`Clipped dashboard content at ${width}x${height}`);
    await page.screenshot({path:`outputs/logs/multicamera-test-${width}.png`,fullPage:true});
  }
  serviceDown=true;
  await page.waitForFunction(() => document.querySelectorAll('.camera-card[data-state="offline"]').length === 2);
  assert.equal(await page.locator('.camera-card img[src]').count(),0);
  assert.deepEqual(errors,[]);
  console.log('PASS: independent cameras, RGB/depth switch, zoom, targeted controls, recording exclusion, stale handling, 1920×1080 layout and RGB readbacks.');
} finally {await browser?.close(); server.kill('SIGTERM');}
