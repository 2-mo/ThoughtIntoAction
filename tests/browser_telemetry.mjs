// Telemetry must work without WebGL, including empty and stale states.
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {chromium} from '../.cache/viewer-validation/node_modules/playwright/index.mjs';
const root = new URL('../src/robot_simulation/web/', import.meta.url);
const browser = await chromium.launch({headless:true, args:['--no-sandbox']});
try {
  const page = await browser.newPage();
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  const html = (await readFile(new URL('index.html', root), 'utf8')).replace(/<script\b[^>]*>[\s\S]*?<\/script>/g, '').replace(/<link\b[^>]*>/g, '');
  await page.setContent(html);
  await page.addStyleTag({path:new URL('scene.css', root).pathname});
  await page.evaluate(() => {
    window.fixture = {stream:{status:'waiting'}, actual:null};
    window.fetch = async url => ({ok:true, json:async () => url.includes('model') ?
      {torque_limits:[30,40,30,15,10,10,1.5],joint_limits:{lower:[-3,-1,-2,-3,-2,-3],upper:[3,3,2,1,2,3]}} : structuredClone(window.fixture)});
  });
  const shared = 'data:text/javascript;base64,' + Buffer.from(await readFile(new URL('display-state.js', root), 'utf8')).toString('base64');
  await page.addScriptTag({type:'module', content:(await readFile(new URL('telemetry.js', root), 'utf8')).replace('./display-state.js', shared)});
  await page.waitForFunction(() => document.querySelectorAll('.torque-row').length === 7);
  assert.equal(await page.locator('#joints .position-row').count(), 6);
  assert.equal(await page.locator('.gripper-placeholder').isVisible(), true);
  assert.equal(await page.locator('.torque-line').first().getAttribute('d'), '');
  await page.evaluate(() => {
    window.fixture = {actual:{q:[0,1,-1,0,0,0],eef_quaternion:[0,0,0,1],eef:[1,0,0,0,0,1,0,0,0,0,1,0,.0962,-.001,.1552,1]},target:{q:[0,2,-.5,0,0,0]},
      gripper:{actual:.02,target:.06,max:.08}, actual_torque:[1,-2,0,.5,-.5,3], gripper_actual_torque:.35,
      stream:{status:'live',age_s:0,stream_id:'test',sequence:1}};
  });
  await page.waitForFunction(() => document.querySelector('#gripper').dataset.state === 'live');
  await page.waitForFunction(() => document.querySelector('.torque-row:last-child b').textContent === '0.35');
  assert.equal(await page.locator('.torque-row:last-child .torque-range').textContent(), '±1.5');
  await page.evaluate(() => {
    window.fixture.motor_diagnostics = {joint_order:[1,2,4,5,6,7],gripper_id:8,
      motors:[{motor_id:1,current_a:1.234,temperature_c:32,valid:true,feedback_status:'live',fault_code:0},
              {motor_id:5,current_a:null,temperature_c:35,rotor_temperature_c:33,valid:false,feedback_status:'live',fault_code:8}]};
  });
  await page.waitForFunction(() => document.querySelector('#motor-feedback b').textContent === '1.234');
  assert.equal(await page.locator('#motor-feedback .motor-row').nth(3).getAttribute('data-state'), 'fault');
  assert.equal(await page.locator('#motor-feedback .motor-row').nth(3).locator('b').first().textContent(), '—');
  assert.equal(await page.locator('#motor-feedback .motor-row').nth(6).locator('b').first().textContent(), '—');
  await page.evaluate(() => {window.fixture.stream.sequence++; window.fixture.actual_torque[0] = 2;});
  await page.waitForFunction(() => document.querySelector('.torque-line').getAttribute('d').includes('L'));
  await page.waitForFunction(() => document.querySelector('#eef-position b').textContent === '0.0962');
  await page.waitForFunction(() => document.querySelector('#eef-quaternion div:last-child b').textContent === '1.0000');
  await page.evaluate(() => {window.fixture.actual.eef_quaternion = [0,0,0,-1]; window.fixture.stream.sequence++;});
  await page.waitForTimeout(250);
  assert.equal(await page.locator('#eef-quaternion div:last-child b').textContent(), '1.0000');

  await page.evaluate(() => {
    document.querySelectorAll('#motion-diagnostics b').forEach((b,i) => {b.textContent = ['0.0000','0.0189','0.0189'][i];});
    document.getElementById('state-source').textContent = '真机反馈 · quest';
    document.getElementById('loop-rates').textContent = 'IK 48.2/50 Hz · 服务 46.7/50 Hz';
    document.getElementById('arm-latency').textContent = '0.4 ms';
    document.getElementById('input-latency').textContent = '4.2 ms';
  });
  for (const [width,height] of [[1440,960],[1366,768],[1024,600]]) {
    await page.setViewportSize({width,height});
    assert.ok(await page.locator('.torque-row svg').first().evaluate(e => e.getBoundingClientRect().height) >= 40);
    assert.equal(await page.locator('.metric-grid > div').evaluateAll(cells => cells.some(c => c.scrollWidth > c.clientWidth + 1)), false, 'Numeric tile overflow');
    const overlap = await page.evaluate(() => {
      const sections = [...document.querySelectorAll('.telemetry-group > section')];
      return sections.flatMap(a => sections.filter(b => a !== b).map(b => {
        const x=a.getBoundingClientRect(), y=b.getBoundingClientRect();
        return x.left < y.right-1 && x.right > y.left+1 && x.top < y.bottom-1 && x.bottom > y.top+1;
      })).some(Boolean) || sections.some(s => s.scrollHeight > s.clientHeight+1);
    });
    if (overlap) console.log(await page.evaluate(() => [...document.querySelectorAll('.telemetry-group > section')].map(s => ({name:s.className,height:s.clientHeight,scroll:s.scrollHeight,top:s.getBoundingClientRect().top,bottom:s.getBoundingClientRect().bottom})))) ;
    await page.screenshot({path:new URL(`../outputs/logs/telemetry-layout-${width}.png`,import.meta.url).pathname});
    assert.equal(overlap, false, `Telemetry content overlaps at ${width}x${height}`);
  }
  assert.equal(await page.locator('.torque-range').first().textContent(), '±30');
  await page.waitForTimeout(5100);
  assert.equal(await page.locator('.torque-line').first().getAttribute('d'), '', 'Samples must leave the five-second window');
  await page.evaluate(() => {
    window.traceFeed = setInterval(() => {window.fixture.stream.sequence++;}, 80);
  });
  await page.waitForTimeout(5400);
  for (let i = 0; i < 8; i++) {
    assert.match(await page.locator('.torque-line').first().getAttribute('d'), /^M0\.0 /,
      'A full live trace must stay anchored at the five-second boundary');
    await page.waitForTimeout(110);
  }
  await page.evaluate(() => {clearInterval(window.traceFeed);});
  await page.evaluate(() => {window.fixture.stream.status='stale';});
  await page.waitForFunction(() => document.querySelector('.torque-row b').textContent === '—');
  assert.equal(await page.locator('#motor-feedback b').first().textContent(), '—');
  await page.evaluate(() => {window.fixture={stream:{status:'waiting'},actual:null};});
  await page.waitForFunction(() => document.querySelector('#gripper').dataset.state === 'waiting');
  assert.equal(await page.locator('.gripper-placeholder').isVisible(), true);
  assert.equal(await page.locator('.torque-line').first().getAttribute('d'), '');
  assert.deepEqual(errors, []);
  console.log('PASS: placeholders without WebGL, gripper opening, torque history, stale feedback and three layouts.');
} finally {await browser.close();}
