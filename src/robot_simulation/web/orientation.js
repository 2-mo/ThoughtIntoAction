// Read-only attitude comparison. Each viewport shares one camera, no robot commands.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const el = id => document.getElementById(id);
const canvas = el('orientation-canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const camera = new THREE.PerspectiveCamera(40, 1, .01, 10);
camera.up.set(0, 0, 1);
camera.position.set(.32, -.5, .3);
const controls = new OrbitControls(camera, canvas);
controls.enablePan = false;
controls.enableZoom = false;
const scenes = [], models = [];
for (const [i, color] of [0x55dcec, 0xffb84c, 0xe5edf2].entries()) {
  const scene = new THREE.Scene();
  const model = new THREE.Group();
  const material = new THREE.MeshBasicMaterial({ color, wireframe: false });
  function box(x, y, z, a, b, c) {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(a, b, c), material);
    mesh.position.set(x, y, z); model.add(mesh);
  }
  if (i === 0) {
    box(-.035, 0, 0, .10, .05, .025);
    box(-.065, 0, -.04, .025, .025, .08);
  } else {
    box(-.065, 0, 0, .025, .09, .035);
    for (const y of [-.035, .035]) box(-.022, y, 0, .08, .012, .025);
  }
  model.add(new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), .13, color, .025, .015));
  box(0, 0, .035, .009, .009, .07);
  model.visible = false;
  scene.add(model, new THREE.AxesHelper(.065));
  scenes.push(scene); models.push(model);
}
// Canonical model +X is controller forward (-Z), +Z is controller up (+Y).
// C maps controller-local to Quest-world; K maps Quest-world to robot base.
const toolToController = new THREE.Matrix4().set(0,-1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1);
let teleop = null, arm = null, inputAt = 0, armAt = 0;
window.addEventListener('teleop-preview', e => { teleop = e.detail; inputAt = performance.now(); });
window.addEventListener('arm-preview', e => { arm = e.detail; armAt = performance.now(); });
function setPose(model, values) {
  if (!Array.isArray(values) || values.length !== 16 || !values.every(Number.isFinite)) return false;
  model.quaternion.setFromRotationMatrix(new THREE.Matrix4().fromArray(values));
  return true;
}
let deviceStamp = null, deviceAt = 0;
function update() {
  const now = performance.now();
  const preview = teleop?.orientation_preview;
  const input = teleop?.input;
  if (input?.device_timestamp_ns !== deviceStamp) {
    deviceStamp = input?.device_timestamp_ns; deviceAt = now;
  }
  const fresh = teleop && teleop.state_age_s != null && teleop.state_age_s < .5 && now - inputAt < 700;
  const hand = input?.hands?.right;
  const handLive = fresh && !input?.error && teleop.input_age_s != null && teleop.input_age_s < .5 &&
    now - deviceAt < 500 && deviceStamp > 0 && preview?.input_ready && hand?.pose?.length === 7 && hand.pose.every(Number.isFinite);
  const k = preview?.basis;
  models[0].visible = false;
  if (handLive && Array.isArray(k) && k.length === 3 && k.every(row => Array.isArray(row) && row.length === 3 && row.every(Number.isFinite))) {
    const q = new THREE.Quaternion(...hand.pose.slice(3));
    if (q.lengthSq() > 1e-8) {
      const basis = new THREE.Matrix4().set(...k[0],0, ...k[1],0, ...k[2],0, 0,0,0,1);
      const mapped = basis.multiply(new THREE.Matrix4().makeRotationFromQuaternion(q.normalize()));
      if (!preview.native_basis) mapped.multiply(toolToController);
      models[0].quaternion.setFromRotationMatrix(mapped);
      models[0].visible = true;
    }
  }
  models[1].visible = !!(handLive && preview?.active && setPose(models[1], preview.target));
  models[2].visible = !!(arm?.stream?.status === 'live' && now - armAt < 500 && setPose(models[2], arm.actual?.eef));
  el('orientation-status').textContent = !fresh ? '等待遥操新数据（更新后需重启遥操）' : !preview ? '需重启遥操以启用姿态对照' : !k ? '当前模式尚无方向参考' : !handLive ? '手柄跟踪暂停或数据过期' : preview.active ? 'Grip 按住 · 正在显示遥操目标' : 'Grip 松开 / 待机 · 仅显示手柄与实际朝向';
  const degrees = (a, b) => (a.angleTo(b) * 180 / Math.PI).toFixed(1) + '°';
  el('orientation-error').textContent = '目标与实际角度差：' + (models[1].visible && models[2].visible ? degrees(models[1].quaternion, models[2].quaternion) : '—');
  canvas.dataset.handVisible = String(models[0].visible);
  canvas.dataset.targetVisible = String(models[1].visible);
  canvas.dataset.actualVisible = String(models[2].visible);
}
const resize = new ResizeObserver(() => {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (w && h) { renderer.setSize(w, h, false); camera.aspect = w / 3 / h; camera.updateProjectionMatrix(); }
});
resize.observe(canvas);
renderer.setScissorTest(true);
renderer.setAnimationLoop(() => {
  update(); controls.update();
  const w = canvas.clientWidth / 3, h = canvas.clientHeight;
  scenes.forEach((scene, i) => {
    renderer.setViewport(i * w, 0, w, h); renderer.setScissor(i * w, 0, w, h);
    renderer.render(scene, camera);
  });
});
window.addEventListener('pagehide', () => {
  renderer.setAnimationLoop(null); resize.disconnect(); controls.dispose();
  const materials = new Set();
  scenes.forEach(scene => scene.traverse(node => {
    node.geometry?.dispose(); if (node.material) materials.add(node.material);
  }));
  materials.forEach(material => material.dispose()); renderer.dispose();
}, { once: true });
