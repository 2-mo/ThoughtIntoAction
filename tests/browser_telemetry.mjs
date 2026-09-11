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
      {joint_limits:{lower:[-3,-1,-2,-3,-2,-3],upper:[3,3,2,1,2,3]}} : structuredClone(window.fixture)});
  });
  await page.addScriptTag({type:'module', content:await readFile(new URL('telemetry.js', root), 'utf8')});
  await page.waitForFunction(() => document.querySelectorAll('.torque-row').length === 6);
  assert.equal(await page.locator('.position-row').count(), 6);
  assert.equal(await page.locator('.gripper-placeholder').isVisible(), true);
  assert.equal(await page.locator('.torque-line').first().getAttribute('d'), '');
  await page.evaluate(() => {
    window.fixture = {actual:{q:[0,1,-1,0,0,0]},target:{q:[0,2,-.5,0,0,0]},
      gripper:{actual:.02,target:.06,max:.08}, actual_torque:[1,-2,0,.5,-.5,3],
      stream:{status:'live',age_s:0,stream_id:'test',sequence:1}};
  });
  await page.waitForFunction(() => document.querySelector('#gripper').dataset.state === 'live');
  await page.evaluate(() => {window.fixture.stream.sequence++; window.fixture.actual_torque[0] = 2;});
  await page.waitForFunction(() => document.querySelector('.torque-line').getAttribute('d').includes('L'));
  for (const [width,height] of [[1440,960],[1366,768],[1024,600]]) {
    await page.setViewportSize({width,height});
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
  await page.evaluate(() => {window.fixture.stream.status='stale';});
  await page.waitForFunction(() => document.querySelector('.torque-row b').textContent === '—');
  await page.evaluate(() => {window.fixture={stream:{status:'waiting'},actual:null};});
  await page.waitForFunction(() => document.querySelector('#gripper').dataset.state === 'waiting');
  assert.equal(await page.locator('.gripper-placeholder').isVisible(), true);
  assert.equal(await page.locator('.torque-line').first().getAttribute('d'), '');
  assert.deepEqual(errors, []);
  console.log('PASS: placeholders without WebGL, gripper opening, torque history, stale feedback and three layouts.');
} finally {await browser.close();}
