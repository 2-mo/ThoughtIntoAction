// Display treatment adapted from arx5_test's scene; no desktop kinematics or controls.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';

const element = (id) => document.getElementById(id);
let showArmConnection = () => {};
const status = (message, state) => {
  element('connection').textContent = state === 'connected' ? message : ({waiting: '未连接', stale: '已过期', error: '已断开'}[state] || message);
  element('connection').title = message;
  element('connection').dataset.state = state;
  showArmConnection(state);
};

async function readJSON(url) {
  const response = await fetch(url, { cache: 'no-store', signal: AbortSignal.timeout(2000) });
  if (!response.ok) throw new Error(`${url}: HTTP ${response.status}`);
  return response.json();
}

async function initialize() {
  const canvas = element('robot-canvas');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.1;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x191b1e);
  scene.fog = new THREE.FogExp2(0x191b1e, 0.17);
  const environment = new RoomEnvironment();
  const pmrem = new THREE.PMREMGenerator(renderer);
  const environmentTarget = pmrem.fromScene(environment, 0.04);
  scene.environment = environmentTarget.texture;
  scene.environmentIntensity = 0.72;
  pmrem.dispose();
  environment.dispose();

  const camera = new THREE.PerspectiveCamera(39, 1, 0.01, 12);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.25;
  controls.maxDistance = 4;
  controls.maxPolarAngle = Math.PI * 0.93;
  function resetView() {
    camera.position.set(-0.85, -1.05, 0.85);
    controls.target.set(0.16, 0, 0.23);
    controls.update();
  }
  resetView();
  element('reset-view').addEventListener('click', resetView);
  const resize = new ResizeObserver(() => {
    const { width, height } = canvas.parentElement.getBoundingClientRect();
    if (!width || !height) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  });
  resize.observe(canvas.parentElement);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x171717, 2.9));
  const key = new THREE.DirectionalLight(0xffffff, 3.4);
  key.position.set(0.45, -0.55, 1.2);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  Object.assign(key.shadow.camera, { near: 0.05, far: 4, left: -1.5, right: 1.5, top: 1.5, bottom: -1.5 });
  key.shadow.bias = -0.00025;
  key.shadow.normalBias = 0.002;
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xa9b8b2, 1.05);
  fill.position.set(-0.75, 0.45, 0.45);
  scene.add(fill);

  const ground = new THREE.GridHelper(4, 80, 0x505258, 0x303237);
  ground.rotation.x = Math.PI / 2;
  ground.position.z = -0.043;
  ground.material.transparent = true;
  ground.material.opacity = 0.32;
  scene.add(ground);
  const axes = new THREE.Group();
  const baseAxes = new THREE.AxesHelper(0.115);
  baseAxes.position.z = 0.002;
  const actualAxes = new THREE.AxesHelper(0.08);
  actualAxes.matrixAutoUpdate = false;
  axes.add(baseAxes, actualAxes);
  scene.add(axes);

  // A visual work surface only. No physical table transform or camera extrinsic is assumed.
  const textureLoader = new THREE.TextureLoader();
  const textures = await Promise.all(['diff', 'nor_gl'].map(async kind => {
    const texture = await textureLoader.loadAsync(`/textures/oak_veneer_01/oak_veneer_01_${kind}_2k.jpg`);
    texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
    texture.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
    texture.colorSpace = kind === 'diff' ? THREE.SRGBColorSpace : THREE.NoColorSpace;
    return texture;
  }));
  const tableWidth = 1.2, tableDepth = 0.9, tableThickness = 0.04;
  const cornerRadius = 0.06, tableBevel = 0.004;
  // Round the tabletop in plan independently from its thin, softly beveled edge.
  const halfWidth = tableWidth / 2 - tableBevel, halfDepth = tableDepth / 2 - tableBevel;
  const radius = cornerRadius - tableBevel;
  const outline = new THREE.Shape();
  outline.moveTo(-halfWidth + radius, -halfDepth);
  outline.lineTo(halfWidth - radius, -halfDepth);
  outline.absarc(halfWidth - radius, -halfDepth + radius, radius, -Math.PI / 2, 0, false);
  outline.lineTo(halfWidth, halfDepth - radius);
  outline.absarc(halfWidth - radius, halfDepth - radius, radius, 0, Math.PI / 2, false);
  outline.lineTo(-halfWidth + radius, halfDepth);
  outline.absarc(-halfWidth + radius, halfDepth - radius, radius, Math.PI / 2, Math.PI, false);
  outline.lineTo(-halfWidth, -halfDepth + radius);
  outline.absarc(-halfWidth + radius, -halfDepth + radius, radius, Math.PI, Math.PI * 1.5, false);
  outline.closePath();
  const tableGeometry = new THREE.ExtrudeGeometry(outline, {
    depth: tableThickness - 2 * tableBevel, bevelEnabled: true,
    bevelThickness: tableBevel, bevelSize: tableBevel, bevelSegments: 5, curveSegments: 20, steps: 1,
  });
  tableGeometry.translate(0, 0, -(tableThickness - 2 * tableBevel) / 2);
  // A pale satin veneer: subtle pores, even roughness and a soft protective finish.
  const tabletopMaterial = new THREE.MeshPhysicalMaterial({
    color: 0xe7dfd3, map: textures[0], normalMap: textures[1],
    normalScale: new THREE.Vector2(0.045, 0.045), roughness: 0.62, metalness: 0,
    clearcoat: 0.16, clearcoatRoughness: 0.55,
  });
  // A quiet matching edge avoids stretching the top grain down the thin side wall.
  const edgeMaterial = new THREE.MeshPhysicalMaterial({
    color: 0xc5b397, roughness: 0.72, metalness: 0, clearcoat: 0.08, clearcoatRoughness: 0.65,
  });
  const table = new THREE.Mesh(tableGeometry, [tabletopMaterial, edgeMaterial]);
  table.position.set(0.42, 0, -tableThickness / 2);
  table.receiveShadow = true;
  scene.add(table);

  const manifest = await readJSON('/api/model');
  if (manifest.matrix_order !== 'column-major') throw new Error('未知的模型变换格式');
  element('model-name').textContent = manifest.model_file;
  element('eef-name').textContent = `${manifest.robot.eef_frame} / m`;
  element('model-id').textContent = `模型 SHA256 ${manifest.robot.urdf_sha256.slice(0, 12)}`;
  const actualModel = new THREE.Group();
  const targetModel = new THREE.Group();
  actualModel.visible = targetModel.visible = false;
  const loader = new STLLoader();
  const actualMeshes = [], targetMeshes = [];
  await Promise.all(manifest.meshes.map(async (spec, index) => {
    const geometry = await loader.loadAsync(spec.url);
    geometry.scale(...spec.scale);
    geometry.computeVertexNormals();
    // The Desktop scene uses black housings and a brushed-metal link3.
    const metal = spec.link === 'link3';
    const material = new THREE.MeshStandardMaterial({
      color: metal ? 0xc8d0d5 : 0x171a1d, emissive: 0x070807, emissiveIntensity: 0.08,
      metalness: metal ? 0.86 : 0.08, roughness: metal ? 0.25 : 0.68, side: THREE.DoubleSide,
    });
    const actual = new THREE.Mesh(geometry, material);
    const target = new THREE.Mesh(geometry, new THREE.MeshBasicMaterial({ color: 0xffb84c,
      transparent: true, opacity: 0.23, depthWrite: false, side: THREE.DoubleSide }));
    actual.name = target.name = spec.name;
    actual.matrixAutoUpdate = target.matrixAutoUpdate = false;
    actual.castShadow = actual.receiveShadow = true;
    actual.userData.online = { color: material.color.clone(), metalness: material.metalness, roughness: material.roughness };
    actualMeshes[index] = actual;
    targetMeshes[index] = target;
    actualModel.add(actual);
    targetModel.add(target);
  }));
  // Place the entire base on the flat part of the near edge, just beyond the roundover.
  // Reference mesh transforms include the URDF visual origin; robot coordinates stay unchanged.
  const baseBounds = new THREE.Box3();
  manifest.meshes.forEach((spec, index) => {
    if (spec.link !== 'base_link') return;
    const geometry = actualMeshes[index].geometry;
    geometry.computeBoundingBox();
    baseBounds.union(geometry.boundingBox.clone().applyMatrix4(
      new THREE.Matrix4().fromArray(manifest.reference.actual.meshes[index])));
  });
  if (!baseBounds.isEmpty()) {
    table.position.set(baseBounds.min.x - tableBevel - 0.003 + tableWidth / 2,
      (baseBounds.min.y + baseBounds.max.y) / 2, baseBounds.min.z - tableThickness / 2);
  }
  scene.add(actualModel, targetModel);
  const paths = new THREE.Group();
  scene.add(paths);
  const pathObjects = new Map();
  const pathSignatures = new Map();
  let latest = null;
  let feedbackLive = false, armDeadline = 0, armVisualState = null;
  function displayOptions() {
    // Coincident ghosts wash out the real materials; show only a distinct target.
    targetModel.visible = feedbackLive && Boolean(latest?.target_visible) && element('show-target').checked;
    paths.visible = feedbackLive && element('show-path').checked;
    table.visible = element('show-table').checked;
    axes.visible = element('show-axes').checked;
  }
  showArmConnection = state => {
    if (state === armVisualState) return;
    armVisualState = state;
    feedbackLive = state === 'connected';
    canvas.dataset.feedback = feedbackLive ? 'live' : state;
    actualMeshes.forEach(mesh => {
      const original = mesh.userData.online;
      mesh.material.color.copy(feedbackLive ? original.color : new THREE.Color(0x384148));
      mesh.material.metalness = feedbackLive ? original.metalness : 0;
      mesh.material.roughness = feedbackLive ? original.roughness : 1;
      mesh.castShadow = feedbackLive;
    });
    displayOptions();
  };
  showArmConnection('waiting');
  for (const id of ['show-target', 'show-path', 'show-table', 'show-axes']) {
    element(id).addEventListener('change', displayOptions);
  }
  function applyState(state) {
    latest = state;
    // Pinocchio supplies world matrices including URDF visual origins. There is
    // no hardcoded joint chain, Euler conversion or feedback smoothing in JS.
    actualMeshes.forEach((mesh, index) => mesh.matrix.fromArray(state.actual.meshes[index]));
    targetMeshes.forEach((mesh, index) => {
      const matrix = state.target.meshes[index];
      mesh.matrix.fromArray(matrix);
      // Even with a distinct target, the base and upstream links can coincide.
      // Suppress those ghosts too, avoiding tinted surfaces and z-fighting.
      mesh.visible = matrix.some((value, entry) => Math.abs(value - state.actual.meshes[index][entry])
        > (entry >= 12 && entry <= 14 ? 0.001 : 0.005));
    });
    actualAxes.matrix.fromArray(state.actual.eef);
    actualModel.visible = true;
    for (const [name, line] of pathObjects) {
      if (!(name in state.paths)) {
        paths.remove(line); line.geometry.dispose(); line.material.dispose();
        pathObjects.delete(name); pathSignatures.delete(name);
      }
    }
    for (const [name, path] of Object.entries(state.paths)) {
      const signature = JSON.stringify(path);
      if (pathSignatures.get(name) === signature) continue;
      const old = pathObjects.get(name);
      if (old) { paths.remove(old); old.geometry.dispose(); old.material.dispose(); pathObjects.delete(name); }
      if (path.points.length > 1) {
        const geometry = new THREE.BufferGeometry().setFromPoints(path.points.map((p) => new THREE.Vector3(...p)));
        const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({ color: path.color }));
        paths.add(line);
        pathObjects.set(name, line);
      }
      pathSignatures.set(name, signature);
    }
    displayOptions();
    canvas.dataset.sequence = String(state.sequence);
    document.body.dataset.ready = 'true';
    element('loading').hidden = true;
  }
  let stopped = false;
  let timer;
  async function poll() {
    try {
      const state = await readJSON('/api/state');
      window.dispatchEvent(new CustomEvent('arm-preview', { detail: state }));
      if (stopped) return;
      if (state.actual) applyState(state);
      else applyState(manifest.reference);
      const source = state.stream.source || {};
      const kind = { hardware: '真机反馈', virtual: '虚拟机械臂', replay: '数据回放' }[source.kind] || '机械臂状态';
      element('state-source').textContent = state.stream.stream_id
        ? `${kind} · ${source.input || source.controller || '状态发布端'}${source.mode === 'damping' ? ' · 阻尼待机' : ''}` : '参考姿态 · 无机械臂反馈';
      canvas.dataset.source = source.kind || 'unknown';
      if (state.stream.status === 'live' && state.actual) {
        armDeadline = performance.now() + 1000 * Math.max(0, 1 - (state.stream.age_s || 0));
        status(({hardware: '真机在线', virtual: '仿真在线', replay: '回放'}[source.kind] || '反馈在线'), 'connected');
      }
      else if (state.stream.status === 'waiting') {
        status('参考姿态 · 无机械臂反馈', 'waiting');
        element('loading').hidden = true;
      } else if (state.stream.status === 'stale') status('状态已过期 · 保留最后画面', 'stale');
      else status(`状态不匹配：${state.stream.message}`, 'error');
      document.body.dataset.monitorReady = 'true';
    } catch {
      window.dispatchEvent(new CustomEvent('arm-preview', { detail: null }));
      if (!stopped) status('连接中断 · 保留最后画面', 'error');
    }
    if (!stopped) timer = setTimeout(poll, document.hidden ? 500 : 50);
  }
  renderer.setAnimationLoop(() => {
    if (feedbackLive && performance.now() >= armDeadline) status('机械臂反馈已过期，保留最后姿态', 'stale');
    controls.update();
    renderer.render(scene, camera);
  });
  canvas.addEventListener('webglcontextlost', (event) => {
    event.preventDefault();
    stopped = true;
    clearTimeout(timer);
    status('3D 显示已中断，请刷新页面', 'error');
    renderer.setAnimationLoop(null);
  });
  window.addEventListener('pagehide', () => {
    stopped = true;
    clearTimeout(timer);
    renderer.setAnimationLoop(null);
    resize.disconnect();
    controls.dispose();
    environmentTarget.dispose();
    const geometries = new Set(), materials = new Set();
    scene.traverse((node) => {
      if (node.geometry) geometries.add(node.geometry);
      for (const material of (Array.isArray(node.material) ? node.material : [node.material])) {
        if (material) materials.add(material);
      }
    });
    geometries.forEach((geometry) => geometry.dispose());
    materials.forEach((material) => material.dispose());
    textures.forEach(texture => texture.dispose());
    renderer.dispose();
  }, { once: true });
  window.addEventListener('pageshow', (event) => { if (event.persisted) location.reload(); });
  await poll();
}

initialize().catch((error) => {
  console.error(error);
  status('场景加载失败', 'error');
  element('loading').textContent = `场景加载失败：${error.message}。请检查终端输出后刷新。`;
});
