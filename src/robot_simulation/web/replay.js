import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
const el = id => document.getElementById(id);
const panel = el('dataset-replay');
let closed = false, timer, generation = null, frame = null, renderer, controls, resize;
const meshes = [];
let initializePromise, commandError = null;
async function initialize() {
  const manifest = await (await fetch('/api/model', {cache: 'no-store'})).json();
  const scene = new THREE.Scene(); scene.background = new THREE.Color(0x10171c);
  renderer = new THREE.WebGLRenderer({canvas: el('replay-canvas'), antialias: true});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  const camera = new THREE.PerspectiveCamera(40, 1, .01, 10);
  camera.up.set(0, 0, 1); camera.position.set(.8, -.9, .65);
  controls = new OrbitControls(camera, el('replay-canvas')); controls.target.set(.15, 0, .2); controls.update();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x444444, 3), new THREE.AxesHelper(.15));
  const loader = new STLLoader();
  await Promise.all(manifest.meshes.map(async (spec, i) => {
    const geometry = await loader.loadAsync(spec.url); geometry.scale(...spec.scale);
    const mesh = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({color: 0x70bfd7, roughness: .55}));
    mesh.matrixAutoUpdate = false; mesh.visible = false; meshes[i] = mesh; scene.add(mesh);
  }));
  resize = new ResizeObserver(() => {
    const {width, height} = el('replay-canvas').parentElement.getBoundingClientRect();
    if (width && height) { renderer.setSize(width, height, false); camera.aspect = width / height; camera.updateProjectionMatrix(); }
  }); resize.observe(el('replay-canvas').parentElement);
  renderer.setAnimationLoop(() => {
    if (closed || panel.hidden) return;
    meshes.forEach((mesh, i) => { mesh.visible = !!frame?.pose?.meshes?.[i]; if (mesh.visible) mesh.matrix.fromArray(frame.pose.meshes[i]); });
    controls.update(); renderer.render(scene, camera);
  });
}
async function listEpisodes() {
  try {
    const result = await (await fetch('/api/replay?list=1', {cache: 'no-store'})).json();
    const selected = el('replay-episodes').value;
    el('replay-episodes').replaceChildren(...(result.episodes || []).map(episode => {
      const option = document.createElement('option'); option.value = episode.id;
      option.textContent = episode.id + (episode.task ? ' · ' + episode.task : ''); return option;
    }));
    if (selected) el('replay-episodes').value = selected;
  } catch { /* The main replay status reports unavailable sessions. */ }
}
function open() {
  panel.hidden = false;
  listEpisodes();
  initializePromise ||= initialize().catch(error => { el('replay-status').textContent = '3D 回放加载失败：' + error.message; });
}
el('open-replay').onclick = open;
async function command(action, position_s, episode_id) {
  try {
    const response = await fetch('/api/replay', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({action, position_s, episode_id}), signal: AbortSignal.timeout(6000)});
    const result = await response.json(); if (result.error) throw Error(result.error); commandError = null;
  } catch (error) { commandError = error.message; el('replay-status').textContent = error.message; }
}
el('close-replay').onclick = () => { panel.hidden = true; command('pause'); };
el('replay-load').onclick = () => command('load', undefined, el('replay-episodes').value);
el('replay-last').onclick = () => command('load_last');
el('replay-play').onclick = () => command('play');
el('replay-pause').onclick = () => command('pause');
el('replay-seek').onchange = event => command('seek', Number(event.target.value));
async function poll() {
  try {
    const response = await fetch('/api/replay', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
    const data = await response.json(); if (data.error) throw Error(data.error);
    frame = data.replay;
    const real = data.robot_replay;
    el('robot-replay-state').textContent = '真机回放：' + (real?.error || ({positioning: '前往起点', playing: '正在执行', loading: '加载数据', idle: '未运行'}[real?.phase] || '未运行'));
    if (frame) {
      if (generation !== frame.generation) { generation = frame.generation; open(); }
      el('replay-status').textContent = commandError || frame.episode + ' · ' + (frame.playing ? '播放中' : '暂停 / 已结束');
      el('replay-time').textContent = frame.position_s.toFixed(2) + ' / ' + frame.duration_s.toFixed(2) + ' s';
      el('replay-seek').max = frame.duration_s;
      if (document.activeElement !== el('replay-seek')) el('replay-seek').value = frame.position_s;
      el('replay-joints').textContent = frame.q.map((q, i) => 'J' + (i + 1) + ' ' + (q * 180 / Math.PI).toFixed(1) + '°').join(' · ') + ' · 夹爪 ' + (frame.gripper_m * 1000).toFixed(1) + ' mm';
      const pixels = frame.image_base64 || frame.jpeg_base64;
      el('replay-video-status').textContent = pixels ? '录制 RGB 原图' : '该条没有 RGB 视频';
      if (pixels) el('replay-video').src = 'data:' + (frame.image_mime || 'image/jpeg') + ';base64,' + pixels;
      else el('replay-video').removeAttribute('src');
    }
  } catch (error) {
    frame = null; el('replay-status').textContent = '回放数据未连接：' + error.message;
    el('replay-video').removeAttribute('src'); el('replay-joints').textContent = '关节与夹爪数据：—';
    el('robot-replay-state').textContent = '真机回放：状态未连接';
  } finally { if (!closed) timer = setTimeout(poll, document.hidden ? 1000 : panel.hidden ? 400 : 33); }
}
window.addEventListener('pagehide', () => { closed = true; clearTimeout(timer); renderer?.setAnimationLoop(null); controls?.dispose(); resize?.disconnect(); meshes.forEach(mesh => {mesh.geometry.dispose(); mesh.material.dispose();}); renderer?.dispose(); }, {once: true});
poll();
