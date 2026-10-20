// Controller cards follow the backend button layout; legacy real-arm pages keep theirs.
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
  const hands = (await readFile(new URL('sensors.js', root), 'utf8')).split('function clearInput')[0].replace(/^import .*;$/gm, '');
  const layout = {
    left: {stick: 'L3', holds: ['stick', 'Y'], keys: {X: '录制 / 保存', Y: '预抓取 / 长按作废'}},
    right: {stick: null, holds: [], keys: {A: '长按切换模式', B: 'A+B 回 Home'}},
  };
  await page.addScriptTag({type:'module', content: hands + `
window.render = (side, buttons, feedback, layout) =>
  renderHand(side, {pose: [0, 0, 0, 0, 0, 0, 1], grip: 0, trigger: 0, axes: [0, 0], buttons}, feedback, undefined, layout);`});
  await page.waitForFunction(() => typeof window.render === 'function', null, {timeout: 3000}).catch(() => { throw new Error('module failed: ' + errors.join('; ')); });
  await page.evaluate(([layout]) => {
    window.render('left', {X: true, L3: true}, {Y: {progress: .5}}, layout);
    window.render('right', {A: true}, {}, layout);
  }, [layout]);
  assert.deepEqual(await page.locator('#left-hand .controller-key').allTextContents(), ['X', 'Y']);
  assert.deepEqual(await page.locator('#left-hand small').allTextContents(), ['按住结束任务', '录制 / 保存', '预抓取 / 长按作废']);
  assert.equal(await page.locator('#left-hand [data-key="X"]').getAttribute('data-pressed'), 'true');
  assert.equal(await page.locator('#left-hand [data-key="L3"]').getAttribute('data-pressed'), 'true');
  assert.equal(await page.locator('#left-hand [data-hold="Y"]').getAttribute('aria-valuenow'), '50');
  assert.equal(await page.locator('#left-hand [data-hold="stick"]').count(), 1);
  assert.equal(await page.locator('#right-hand .hold-ring').count(), 0);
  assert.equal(await page.locator('#right-hand [data-key="A"]').getAttribute('data-pressed'), 'true');
  // Without a backend layout the legacy real-arm card returns.
  await page.evaluate(() => { window.render('right', {B: true}, {B: {progress: .25}}); });
  assert.deepEqual(await page.locator('#right-hand small').allTextContents(), ['按住结束任务', '预抓取/采集', '预抓取/回零']);
  assert.equal(await page.locator('#right-hand [data-hold="B"]').getAttribute('aria-valuenow'), '25');
  assert.deepEqual(errors, []);
  console.log('PASS: simulation and legacy controller layouts');
} finally {
  await browser.close();
}
