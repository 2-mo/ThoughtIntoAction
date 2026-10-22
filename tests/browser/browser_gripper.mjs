// Isolated renderer: original mesh, measured jaw gap, wrist transforms and replay.
// No device service is started and no robot commands are sent.
import assert from 'node:assert/strict';
import {readFile, mkdir} from 'node:fs/promises';
import {spawn} from 'node:child_process';
import {chromium} from '../../.cache/viewer-validation/node_modules/playwright/index.mjs';

const server = spawn('.venv/bin/python', ['-u', '-B', '-c', `from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from embodied.applications.monitor.scene import Scene
import threading
scene = Scene(X5PinocchioModel())
threading.Event().wait()`], {env:{...process.env, PYTHONPATH:'src'}, stdio:['ignore','pipe','pipe']});
let browser;
try {
  const url = await new Promise((resolve, reject) => {
    let log = '';
    const timer = setTimeout(() => reject(Error('Scene startup timeout: '+log)), 15000);
    server.stdout.on('data', chunk => {
      log += chunk;
      const match = log.match(/http:\/\/127\.0\.0\.1:\d+\//);
      if (match) {clearTimeout(timer); resolve(match[0]);}
    });
    server.stderr.on('data', chunk => {log += chunk;});
    server.once('exit', code => {clearTimeout(timer); reject(Error(`Scene exited ${code}: ${log}`));});
  });
  const model = await (await fetch(url+'api/model')).json();
  const index = model.meshes.findIndex(mesh => mesh.link === 'link6');
  const reference = model.reference.actual;
  let live = {...structuredClone(model.reference), gripper:{actual:0, target:0, max:.082},
    stream:{status:'live', age_s:0, sequence:1, stream_id:'fixture', source:{kind:'virtual'}}};
  let replay = null;
  browser = await chromium.launch({headless:true, env:{...process.env, DISPLAY:'', LD_LIBRARY_PATH:''},
    args:['--no-sandbox','--use-gl=angle','--use-angle=swiftshader','--enable-unsafe-swiftshader','--disable-gpu-sandbox']});
  const page = await browser.newPage({viewport:{width:1920,height:1080}});
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  const source = await readFile(new URL('../../src/embodied/applications/monitor/web/scene.js', import.meta.url), 'utf8');
  // Inspect rendered objects without adding a production debug API.
  await page.route('**/scene.js', route => route.fulfill({contentType:'text/javascript', body:source.replace(
    'scene.add(actualModel, targetModel);',
    'scene.add(actualModel, targetModel); window.rigTest = {actualMeshes, targetMeshes, camera, controls, THREE};')}));
  await page.route('**/api/state', route => route.fulfill({json:live}));
  await page.route('**/api/sensors', route => route.fulfill({json:{}}));
  await page.route('**/api/replay*', route => route.fulfill({json:{replay, episodes:[]}}));
  await page.goto(url);
  await page.waitForSelector('body[data-ready="true"]', {timeout:15000});
  await page.waitForFunction(i => window.rigTest?.actualMeshes[i]?.userData.gripperOpening === 0, index);
  const total = await page.evaluate(i => {
    const root = rigTest.actualMeshes[i];
    window.fixedVertices = root.geometry.attributes.position.array.slice();
    return root.geometry.attributes.position.count + root.children.reduce((n, child) => n + child.geometry.attributes.position.count, 0);
  }, index);
  assert.equal(total, 50666 * 3, 'Splitting must retain every original triangle');
  const readGap = () => page.evaluate(i => {
    const {THREE, actualMeshes, targetMeshes} = rigTest;
    const mesh = actualMeshes[i], [positive, negative] = mesh.children;
    mesh.updateMatrixWorld(true);
    const innerSurface = (part, positive) => {
      const p = part.geometry.attributes.position;
      let y = positive ? Infinity : -Infinity;
      for (let v=0; v<p.count; v++) if (p.getX(v) > .12)
        y = positive ? Math.min(y, p.getY(v)) : Math.max(y, p.getY(v));
      return y + part.position.y;
    };
    const plus = new THREE.Vector3(.14, innerSurface(positive, true), 0).applyMatrix4(mesh.matrixWorld);
    const minus = new THREE.Vector3(.14, innerSurface(negative, false), 0).applyMatrix4(mesh.matrixWorld);
    return {gap:plus.distanceTo(minus), delta:plus.sub(minus).toArray(), matrix:mesh.matrix.toArray(),
      fixed:fixedVertices.every((v,j) => v === mesh.geometry.attributes.position.array[j]),
      ghost:targetMeshes[i].visible && targetMeshes[i].parent.visible,
      bodyVisible:mesh.material.visible, ghostBodyVisible:targetMeshes[i].material.visible};
  }, index);
  await page.evaluate(i => {
    const {THREE,camera,controls,actualMeshes} = rigTest;
    const wrist = actualMeshes[i].matrix;
    const center = new THREE.Vector3(.095,0,0).applyMatrix4(wrist);
    controls.target.copy(center);
    camera.position.copy(new THREE.Vector3(.30,-.28,.22).applyMatrix4(wrist));
    controls.update();
  }, index);
  await mkdir(new URL('../../outputs/logs/', import.meta.url), {recursive:true});
  for (const width of [0, .041, .082]) {
    live.gripper = {actual:width, target:width, max:.082};
    await page.waitForFunction(({i,width}) => rigTest.actualMeshes[i].userData.gripperOpening === width, {i:index,width});
    const result = await readGap();
    assert.ok(Math.abs(result.gap-width) < 1e-6, `Measured tip gap must equal ${width} m`);
    assert.equal(result.fixed, true, 'Housing geometry must stay unchanged');
    assert.equal(result.bodyVisible, true);
    assert.equal(result.ghost, false, 'Coincident target stays hidden');
    assert.deepEqual(result.matrix, reference.meshes[index]);
    await page.locator('#robot-canvas').screenshot({path:new URL(`../../outputs/logs/gripper-${Math.round(width*1000)}mm.png`,import.meta.url).pathname});
  }
  // A gripper-only target must show moving fingers without overlaying its housing.
  live.gripper = {actual:.02, target:.07, max:.082};
  await page.waitForFunction(i => rigTest.targetMeshes[i].userData.gripperOpening === .07, index);
  assert.equal((await readGap()).ghost, true);
  assert.equal((await readGap()).ghostBodyVisible, false);
  await page.uncheck('#show-target');
  assert.equal((await readGap()).ghost, false);
  await page.check('#show-target');
  // Use a rotated/translated wrist to catch motion applied in world coordinates.
  const rotated = await page.evaluate(matrix => {
    const {THREE} = rigTest;
    return new THREE.Matrix4().fromArray(matrix).multiply(new THREE.Matrix4().makeRotationX(Math.PI/2)).toArray();
  }, reference.meshes[index]);
  live.actual.meshes[index] = rotated;
  await page.waitForFunction(({i,m}) => rigTest.actualMeshes[i].matrix.elements.every((v,j) => v === m[j]), {i:index,m:rotated});
  const rotatedGap = await readGap();
  assert.ok(Math.abs(rotatedGap.gap-.02) < 1e-6);
  rotatedGap.delta.forEach((v,j) => assert.ok(Math.abs(v - rotated[4+j]*.02) < 1e-6));
  // Recorded width wins over the current live width, then live mode restores it.
  replay = {pose:reference, q:reference.q, gripper_m:.055, gripper_max:.082, generation:1,
    sample_index:1, sample_count:2, position_s:0, duration_s:1, episode:'gripper-fixture', playing:false};
  await page.waitForFunction(i => rigTest.actualMeshes[i].userData.gripperOpening === .055, index);
  assert.ok(Math.abs((await readGap()).gap-.055) < 1e-6);
  await page.click('#view-live');
  await page.waitForFunction(i => rigTest.actualMeshes[i].userData.gripperOpening === .02, index);
  live.gripper = null;
  await page.waitForFunction(i => rigTest.actualMeshes[i].userData.gripperOpening === null, index);
  assert.ok(Math.abs((await readGap()).gap-.060285814106464386) < 1e-6,
    'Missing width uses reference geometry, not a false closed jaw');
  const foreignMesh = await page.evaluate(async url => {
    const {splitGripperGeometry} = await import('/gripper.js');
    const {STLLoader} = await import('three/addons/loaders/STLLoader.js');
    const bytes = await (await fetch(url)).arrayBuffer();
    const geometry = new STLLoader().parse(bytes);
    new Uint8Array(bytes)[0] ^= 1;
    return await splitGripperGeometry(bytes, geometry, [1,1,1]) === null;
  }, model.meshes[index].url);
  assert.equal(foreignMesh, true, 'Unrecognized meshes must not inherit the split');
  assert.deepEqual(errors, []);
  console.log('PASS: original geometry, 0/41/82 mm gaps, wrist rotation, target ghost, replay and missing feedback.');
} finally {
  await browser?.close();
  server.kill('SIGTERM');
}
