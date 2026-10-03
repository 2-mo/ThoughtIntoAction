// Telemetry must work without WebGL, including empty and stale states.
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {chromium} from '../.cache/viewer-validation/node_modules/playwright/index.mjs';
const root = new URL('../src/robot_simulation/web/', import.meta.url);
const browser = await chromium.launch({headless:true, args:['--no-sandbox']});
try {
  const page = await browser.newPage({viewport:{width:1920,height:1080}});
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  const html = (await readFile(new URL('index.html', root), 'utf8')).replace(/<script\b[^>]*>[\s\S]*?<\/script>/g, '').replace(/<link\b[^>]*>/g, '');
  await page.setContent(html);
  await page.addStyleTag({path:new URL('scene.css', root).pathname});
  await page.addStyleTag({path:new URL('cameras.css', root).pathname});
  const hands = (await readFile(new URL('sensors.js', root), 'utf8')).split('function clearInput')[0].replace(/^import .*;$/m, '');
  await page.addScriptTag({type:'module', content:hands + `
renderHand('left', null); renderHand('right', null);`});
  await page.evaluate(() => {
    window.fixture = {stream:{status:'waiting'}, actual:null};
    window.fetch = async url => ({ok:true, json:async () => url.includes('model') ?
      {velocity_limits:[3,3,3,3,3,3],torque_limits:[30,40,30,15,10,10,1.5],joint_limits:{lower:[-3,-1,-2,-3,-2,-3],upper:[3,3,2,1,2,3]}} : structuredClone(window.fixture)});
  });
  const shared = 'data:text/javascript;base64,' + Buffer.from(await readFile(new URL('display-state.js', root), 'utf8')).toString('base64');
  await page.addScriptTag({type:'module', content:(await readFile(new URL('telemetry.js', root), 'utf8')).replace('./display-state.js', shared)});
  await page.waitForFunction(() => document.querySelectorAll('.torque-row').length === 7);
  assert.equal(await page.locator('#joints .position-row').count(), 6);
  assert.equal(await page.locator('details.diagnostics-panel').count(), 0);
  assert.equal(await page.locator('#motion-diagnostics').isVisible(), false);
  await page.click('#open-telemetry-details');
  assert.equal(await page.locator('#motion-diagnostics').isVisible(), true);
  await page.click('#telemetry-details-close');
  assert.equal(await page.locator('.telemetry-group #eef-position').count(), 0);
  assert.equal(await page.locator('.gripper-placeholder').isVisible(), true);
  assert.equal(await page.locator('.torque-line').first().getAttribute('d'), '');
  await page.evaluate(() => {
    window.fixture = {actual:{q:[0,1,-1,0,0,0],eef_quaternion:[0,0,0,1],eef:[1,0,0,0,0,1,0,0,0,0,1,0,.0962,-.001,.1552,1]},target:{q:[0,2,-.5,0,0,0]},
      actual_vel:[.2,-.4,0,.1,0,.3],target_vel:[.3,-.3,0,.2,0,.4],
      gripper_actual_vel:.001, command_details:{gripper_sdk_cmd_vel:.002, gripper_sdk_cmd_pos:.03, gripper_sdk_cmd_torque:.3, gripper_kp:4, gripper_kd:.5, submission:{gripper_pos:.04}},
      gripper:{actual:.02,target:.06,max:.08}, actual_torque:[1,-2,0,.5,-.5,3], gripper_actual_torque:.35,
      stream:{status:'live',age_s:0,stream_id:'test',sequence:1}};
  });
  await page.waitForFunction(() => document.querySelector('#gripper').dataset.state === 'live');
  await page.waitForFunction(() => document.querySelector('.torque-row:last-child b').textContent === '0.35');
  assert.equal(await page.locator('.torque-row:last-child .torque-range').textContent(), '±1.5');
  assert.deepEqual(await page.locator('#gripper-velocity b').allTextContents(), ['1.0', '2.0']);
  assert.deepEqual(await page.locator('#command-feedback tr:last-child td').allTextContents(), ['0.0400', '0.0300', '0.0020', '0.3000', '4.0000', '0.5000']);
  await page.evaluate(() => {
    window.fixture.motor_diagnostics = {joint_order:[1,2,4,5,6,7],gripper_id:8,
      motors:[{motor_id:1,current_a:1.234,temperature_c:32,valid:true,feedback_status:'live',fault_code:0},
              {motor_id:5,current_a:null,temperature_c:35,rotor_temperature_c:33,valid:false,feedback_status:'live',fault_code:8}]};
  });
  await page.waitForFunction(() => document.querySelector('#motor-feedback b').textContent === '1.234');
  assert.equal(await page.locator('#motor-feedback .motor-row').nth(3).getAttribute('data-state'), 'fault');
  assert.equal(await page.locator('#motor-feedback .motor-row').nth(3).locator('b').first().textContent(), '—');
  assert.equal(await page.locator('#motor-feedback .motor-row').nth(6).locator('b').first().textContent(), '—');
  assert.equal(await page.locator('#motor-feedback .motor-row').first().locator('.motor-track i').nth(1).evaluate(e => e.style.width), '32%');
  assert.equal(await page.locator('#motor-feedback .motor-row').nth(3).locator('.motor-track i').first().isVisible(), false);
  assert.equal(await page.locator('.input-panel #show-table').count(), 1);
  assert.equal(await page.locator('.input-panel #arm-latency').count(), 1);
  await page.evaluate(() => {window.fixture.stream.sequence++; window.fixture.actual_torque[0] = 2;});
  await page.waitForFunction(() => document.querySelector('.torque-line').getAttribute('d').includes('L'));
  await page.waitForFunction(() => document.querySelector('.velocity-line.actual').getAttribute('d').includes('L'));
  const lastY = d => Number([...d.matchAll(/[ML][-\d.]+ ([-\d.]+)/g)].at(-1)[1]);
  const fullScaleY = lastY(await page.locator('.torque-line').first().getAttribute('d'));
  await page.selectOption('#torque-zoom', '10');
  assert.equal(await page.locator('.torque-range').first().textContent(), '±3');
  assert.ok(lastY(await page.locator('.torque-line').first().getAttribute('d')) < fullScaleY,
    'Zoom must enlarge the plotted signal without changing its value');
  assert.equal(await page.locator('.torque-row b').first().textContent(), '2.00');
  await page.selectOption('#velocity-zoom', '10');
  assert.equal(await page.locator('.velocity-range').first().textContent(), '±17.2');
  await page.evaluate(() => {
    window.fixture.actual_torque[0] = 8;
    window.fixture.actual_vel[0] = 1;
    window.fixture.stream.sequence++;
  });
  await page.waitForFunction(() => document.querySelector('.torque-row b').textContent === '8.00');
  assert.equal(await page.locator('#velocities .position-values b').first().textContent(), '57.3');
  assert.equal(await page.locator('#velocities .chart-clipped').first().isVisible(), true);
  assert.equal(await page.locator('#torques .chart-clipped').first().isVisible(), true);
  await page.selectOption('#velocity-zoom', '1');
  await page.selectOption('#torque-zoom', '1');
  assert.equal(await page.locator('#velocities .chart-clipped').first().isVisible(), false);
  assert.equal(await page.locator('#torques .chart-clipped').first().isVisible(), false);
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
  for (const [width,height] of [[1920,1080]]) {
    await page.setViewportSize({width,height});
    assert.ok(await page.locator('.torque-row svg').first().evaluate(e => e.getBoundingClientRect().height) >= 24);
    assert.equal(await page.locator('.metric-grid > div').evaluateAll(cells => cells.some(c => c.getClientRects().length && c.scrollWidth > c.clientWidth + 1)), false, 'Numeric tile overflow');
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
    assert.equal(await page.locator('.input-panel').evaluate(e => e.scrollHeight > e.clientHeight + 1), false, 'Upper-right controls must remain visible');
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth + 1), false,
      'Responsive telemetry must not overflow horizontally');
    assert.equal(await page.evaluate(() => document.documentElement.scrollHeight > innerHeight + 1), false,
      'The complete dashboard must fit one screen');
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
  console.log('PASS: telemetry, fixed-axis zoom, unclipped values, stale feedback and 1920×1080 layout.');
} finally {await browser.close();}
