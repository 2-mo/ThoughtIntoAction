// Read-only attitude comparison. Each viewport shares one camera, no robot commands.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
const el = id => document.getElementById(id);
const canvas = el('orientation-canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
const camera = new THREE.PerspectiveCamera(40, 1, .01, 10);
camera.up.set(0, 0, 1);
camera.position.set(.3, -.44, .26);
const controls = new OrbitControls(camera, canvas);
controls.enablePan = false;
controls.enableZoom = false;
const scenes = [], models = [];
// Look (todo U-05): translucent device with glowing edges, long RGB axes through its origin.
const AXIS_LENGTH = .15;
const AXES = [[new THREE.Vector3(1, 0, 0), 0xff5a4a], [new THREE.Vector3(0, 1, 0), 0x86e33f], [new THREE.Vector3(0, 0, 1), 0x45aaff]];
function axisRod(direction, color, radius, opacity) {
  const rod = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 2 * AXIS_LENGTH, 10, 1, true),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false, blending: THREE.AdditiveBlending }));
  rod.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
  return rod;
}
function deviceAxes() {
  const group = new THREE.Group();
  for (const [direction, color] of AXES) {
    group.add(axisRod(direction, color, .0016, 1), axisRod(direction, color, .0055, .16));
    const tip = new THREE.Mesh(new THREE.SphereGeometry(.0045, 12, 8), new THREE.MeshBasicMaterial({ color }));
    tip.position.copy(direction).multiplyScalar(AXIS_LENGTH);
    group.add(tip);
  }
  return group;
}
function glowingDevice(color, kind) {
  const group = new THREE.Group();
  const fill = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: .13, depthWrite: false, side: THREE.DoubleSide });
  const edge = new THREE.LineBasicMaterial({ color, transparent: true, opacity: .95 });
  const halo = new THREE.LineBasicMaterial({ color, transparent: true, opacity: .3, depthWrite: false, blending: THREE.AdditiveBlending });
  function part(geometry, position, rotation = [0, 0, 0]) {
    const edges = new THREE.EdgesGeometry(geometry, 25);
    const outline = new THREE.LineSegments(edges, halo);
    outline.scale.setScalar(1.08);
    for (const node of [new THREE.Mesh(geometry, fill), new THREE.LineSegments(edges, edge), outline]) {
      node.position.set(...position); node.rotation.set(...rotation); group.add(node);
    }
  }
  if (kind === 'controller') {
    part(new THREE.BoxGeometry(.105, .044, .03), [-.042, 0, 0]);                         // face along controller forward
    part(new THREE.BoxGeometry(.034, .036, .085), [-.083, 0, -.045], [0, -.35, 0]);      // grip
    part(new THREE.TorusGeometry(.034, .007, 8, 32), [.03, 0, .006], [0, Math.PI / 2, 0]); // tracking ring
  } else if (kind === 'phone') {
    part(new THREE.BoxGeometry(.012, .07, .13), [0, 0, 0]);
    part(new THREE.BoxGeometry(.004, .05, .1), [.008, 0, 0]);
  } else {
    part(new THREE.BoxGeometry(.032, .1, .042), [-.072, 0, 0]);                          // palm
    for (const y of [-.034, .034]) part(new THREE.BoxGeometry(.086, .013, .028), [-.016, y, 0]); // fingers
    part(new THREE.CylinderGeometry(.012, .012, .05, 16), [-.112, 0, 0], [0, 0, Math.PI / 2]);  // flange
  }
  group.add(deviceAxes());
  group.visible = false;
  return group;
}
function referenceRing() {
  const ring = new THREE.Mesh(new THREE.RingGeometry(.125, .127, 64),
    new THREE.MeshBasicMaterial({ color: 0x9fb3bf, transparent: true, opacity: .14, side: THREE.DoubleSide, depthWrite: false }));
  ring.position.z = -.075;
  return ring;
}
for (const [i, color] of [0x55dcec, 0xffb84c, 0xe5edf2].entries()) {
  const scene = new THREE.Scene();
  const model = glowingDevice(color, i === 0 ? 'controller' : 'gripper');
  scene.add(model, referenceRing());
  scenes.push(scene); models.push(model);
}
const phoneModel = glowingDevice(0x55dcec, 'phone');
scenes[0].add(phoneModel);
// Canonical model +X is controller forward (-Z), +Z is controller up (+Y).
// C maps controller-local to Quest-world; K maps Quest-world to robot base.
const toolToController = new THREE.Matrix4().set(0,-1,0,0, 0,0,1,0, -1,0,0,0, 0,0,0,1);
let teleop = null, phone = null, arm = null, inputAt = 0, phoneAt = 0, armAt = 0;
window.addEventListener('teleop-preview', e => { teleop = e.detail; inputAt = performance.now(); });
window.addEventListener('phone-preview', e => { phone = e.detail; phoneAt = performance.now(); });
window.addEventListener('arm-preview', e => { arm = e.detail; armAt = performance.now(); });
function setQuaternion(model, values) {
  if (!Array.isArray(values) || values.length !== 4 || !values.every(Number.isFinite)) return false;
  const q = new THREE.Quaternion(...values);
  if (q.lengthSq() < 1e-8) return false;
  model.quaternion.copy(q.normalize());
  return true;
}
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
  phoneModel.visible = false;
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
  const phoneFresh = now - phoneAt < 1800 && phone?.connected && phone.age_s < .5;
  // Ownership wins over the other connected input. Never reuse a phone target
  // from a different robot run, or mix ARKit world orientation with base TCP.
  const phoneSelected = now - phoneAt < 1800 && (phone?.enabled || (phoneFresh && !handLive));
  if (phoneSelected) {
    const phonePreview = phone.orientation_preview;
    const sameRun = phone.runtime_id && phone.runtime_id === arm?.stream?.stream_id;
    const live = phoneFresh && sameRun && phone.tracking === 'normal' && phone.active && phonePreview?.frame === 'base_tcp';
    models[0].visible = false;
    phoneModel.visible = !!(live && setQuaternion(phoneModel, phonePreview.input_quaternion));
    models[1].visible = !!(live && setQuaternion(models[1], phonePreview.target_quaternion));
    el('orientation-source').textContent = '手机 · 基座坐标系';
    el('orientation-input-label').textContent = '手机映射 · 青';
    el('orientation-status').textContent = !phoneFresh ? '手机姿态过期' : !phone.enabled ? '手机仅监看 · 开始遥操后建立参考'
      : !sameRun ? '等待当前 robot 反馈' : phone.tracking !== 'normal' ? '手机追踪受限 · 目标已隐藏'
      : phone.active ? 'Clutch 按住 · 显示相对映射与执行目标' : 'Clutch 松开 · 按住后建立相对参考';
  } else {
    el('orientation-source').textContent = handLive ? 'Quest · 基座坐标系' : '等待控制源';
    el('orientation-input-label').textContent = '手柄映射 · 青';
    el('orientation-status').textContent = !fresh ? '等待手机或 Quest 遥操输入' : !preview ? '等待姿态对照数据'
      : !k ? '当前模式尚无方向参考' : !handLive ? '手柄跟踪暂停或数据过期'
      : preview.active ? 'Grip 按住 · 正在显示遥操目标' : 'Grip 松开 · 仅显示手柄与实际朝向';
  }
  canvas.dataset.source = phoneSelected ? 'phone' : handLive ? 'quest' : 'none';
  const degrees = (a, b) => (a.angleTo(b) * 180 / Math.PI).toFixed(1) + '°';
  el('orientation-error').textContent = '目标与实际角度差：' + (models[1].visible && models[2].visible ? degrees(models[1].quaternion, models[2].quaternion) : '—');
  canvas.dataset.handVisible = String(models[0].visible || phoneModel.visible);
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
