import * as THREE from './assets/vendor/three/three.module.min.js';

// A diagram you can inspect, rather than a decorative spinning object. This
// scene has no connection to the runtime, devices, or a control endpoint.
const MODULES = [
  { id: 'inputs', title: '交互输入', subtitle: 'VR · 手机 · Python', color: 0xaca8ff, position: [-2.8, 0.2, 0.35] },
  { id: 'control', title: '统一控制', subtitle: 'Robot API · 运行内核', color: 0xc6ccff, position: [0, 1.22, 0] },
  { id: 'sensors', title: '视觉感知', subtitle: '可替换的传感器', color: 0x79d6cf, position: [-0.15, 0.2, 2.55] },
  { id: 'data', title: '数据采集', subtitle: '记录 · 回放 · 导出', color: 0xb6d6ff, position: [2.8, 0.2, 0.15] },
  { id: 'policy', title: '策略执行', subtitle: '观测 · 状态 → 动作', color: 0xd7bfff, position: [0.1, 0.2, -2.55] },
];
const FLOWS = {
  control: { nodes: ['inputs', 'control'], links: ['input-control'] },
  data: { nodes: ['sensors', 'control', 'data'], links: ['sensor-data', 'control-data'] },
  policy: { nodes: ['sensors', 'policy', 'control'], links: ['sensor-policy', 'control-policy', 'policy-control'] },
};

export function createPlatformScene(container, { onSelect, reducedMotion = false } = {}) {
  const canvas = document.createElement('canvas');
  const context = canvas.getContext('webgl2', { alpha: true, antialias: true, powerPreference: 'low-power' });
  if (!context) throw new Error('WebGL unavailable');
  const renderer = new THREE.WebGLRenderer({ canvas, context, alpha: true, antialias: true, powerPreference: 'low-power' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
  renderer.setClearColor(0x000000, 0);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  canvas.className = 'scene-canvas';
  canvas.tabIndex = 0;
  canvas.setAttribute('role', 'img');
  canvas.setAttribute('aria-label', '平台模块三维示意图。拖动或使用方向键调整视角，按 R 复位。模块可通过下方按钮选择。');
  canvas.style.cssText = 'display:block;width:100%;height:100%;touch-action:pan-y;outline-offset:-4px;';

  const labels = document.createElement('div');
  labels.className = 'scene-labels';
  labels.style.cssText = 'position:absolute;inset:0;pointer-events:none;overflow:hidden;';
  const styles = document.createElement('style');
  styles.textContent = `@layer scene-base {
    .scene-label{position:absolute;left:0;top:0;transform:translate(-50%,-100%);pointer-events:auto;
      padding:7px 11px;border:1px solid #7c88b744;border-radius:7px;background:#101321e8;
      color:#e5e8ff;font-family:inherit;font-size:12px;font-weight:500;line-height:1.4;text-align:left;white-space:nowrap;cursor:pointer;
      box-shadow:0 5px 18px #0002;transition:border-color .18s,background .18s;}
    .scene-label:hover,.scene-label:focus-visible{border-color:#bbc3ff;outline:2px solid #bbc3ff;outline-offset:3px;}
    .scene-label[data-active="true"]{border-color:#9aafff88;background:#181c32f2;}
    .scene-label strong{display:block;font-size:12px;font-weight:550;letter-spacing:.035em;}
    .scene-label small{display:block;margin-top:2px;font-size:9px;color:#939fb9;letter-spacing:.035em;}
    .scene-label strong:before{content:'';display:inline-block;width:5px;height:5px;margin-right:6px;
      border-radius:50%;background:var(--node-color);vertical-align:2px;}
    @media(max-width:600px){.scene-label{padding:5px 7px}.scene-label strong{font-size:10px}.scene-label small{font-size:8px}}
  }
  `;
  container.append(styles, canvas, labels);

  const scene = new THREE.Scene();
  const camera = new THREE.OrthographicCamera(-5, 5, 5, -5, 0.1, 60);
  const assembly = new THREE.Group();
  scene.add(assembly);
  scene.add(new THREE.AmbientLight(0xbed0ff, 2.0));
  const keyLight = new THREE.DirectionalLight(0xd9e1ff, 3.2);
  keyLight.position.set(3, 8, 5);
  scene.add(keyLight);
  const fillLight = new THREE.DirectionalLight(0x7764e8, 2);
  fillLight.position.set(-6, 3, -4);
  scene.add(fillLight);

  const disposers = [];
  const nodes = new Map();
  const wires = [];
  const hitTargets = [];
  const target = new THREE.Vector3(0, 0.35, 0);
  const projection = new THREE.Vector3();
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const orbitDefault = { azimuth: 0.65, elevation: 0.7 };
  let orbit = { ...orbitDefault };
  let parallax = { x: 0, y: 0 };
  let drag = null;
  let width = 1;
  let height = 1;
  let flow = 'control';
  let selectedNode = 'control';
  let paused = false;
  let visible = true;
  let disposed = false;
  let frameId = 0;
  let frames = 0;
  let elapsed = 0;
  let previousTime = 0;

  function listen(element, name, handler, options) {
    element.addEventListener(name, handler, options);
    disposers.push(() => element.removeEventListener(name, handler, options));
  }

  function line(points, color, opacity = 1) {
    const geometry = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(...p)));
    return new THREE.Line(geometry, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
  }

  function box(parent, position, dimensions, color, opacity = 1, nodeId) {
    const geometry = new THREE.BoxGeometry(...dimensions);
    const material = new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.35, transparent: opacity < 1, opacity });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(...position);
    parent.add(mesh);
    const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geometry), new THREE.LineBasicMaterial({ color: 0x7f8cbd, transparent: true, opacity: 0.45 }));
    mesh.add(edges);
    if (nodeId) {
      mesh.userData.nodeId = nodeId;
      hitTargets.push(mesh);
    }
    return mesh;
  }

  // A fixed engineering grid provides spatial context; no particle backdrop.
  const grid = new THREE.GridHelper(8.5, 17, 0x3d4566, 0x262e48);
  grid.position.y = -0.46;
  grid.material.transparent = true;
  grid.material.opacity = 0.36;
  assembly.add(grid);
  assembly.add(line([[-4.25, -0.45, -4.25], [4.25, -0.45, -4.25], [4.25, -0.45, 4.25], [-4.25, -0.45, 4.25], [-4.25, -0.45, -4.25]], 0x596185, 0.23));

  // The central stack represents contracts, motion/runtime, and the public API.
  const core = new THREE.Group();
  assembly.add(core);
  box(core, [0, -0.2, 0], [2.3, 0.16, 1.7], 0x242c49, 1, 'control');
  box(core, [0, 0.28, 0], [2.15, 0.15, 1.55], 0x454e79, 0.85, 'control');
  box(core, [0, 0.77, 0], [2.0, 0.14, 1.4], 0x7078b3, 0.8, 'control');
  box(core, [0, 0.96, 0], [0.82, 0.22, 0.62], 0x8f98d5, 1, 'control');
  for (const x of [-0.87, 0.87]) {
    for (const z of [-0.57, 0.57]) {
      assembly.add(line([[x, -0.12, z], [x, 0.69, z]], 0xadbaff, 0.6));
    }
  }
  for (let i = 0; i < 6; i += 1) {
    assembly.add(line([[-0.86, 0.855, -0.42 + i * 0.16], [-0.56, 0.855, -0.42 + i * 0.16]], 0xc5ccff, 0.65));
    assembly.add(line([[0.56, 0.855, -0.42 + i * 0.16], [0.86, 0.855, -0.42 + i * 0.16]], 0xc5ccff, 0.65));
  }

  for (const item of MODULES) {
    const group = new THREE.Group();
    group.position.set(...item.position);
    if (item.id !== 'control') {
      assembly.add(group);
      box(group, [0, -0.27, 0], [1.15, 0.13, 0.93], 0x242e4c, 1, item.id);
      box(group, [0, -0.16, 0], [0.95, 0.07, 0.74], item.color, 0.36, item.id);
      if (item.id === 'inputs') {
        box(group, [-0.19, 0.08, 0], [0.22, 0.38, 0.22], 0x7468ac, 1, item.id);
        box(group, [0.19, 0.08, 0], [0.22, 0.38, 0.22], 0x7468ac, 1, item.id);
        for (const x of [-0.19, 0.19]) {
          const ring = new THREE.Mesh(new THREE.TorusGeometry(0.15, 0.018, 6, 24), new THREE.MeshBasicMaterial({ color: item.color }));
          ring.position.set(x, 0.32, 0);
          ring.rotation.x = -Math.PI / 2.8;
          group.add(ring);
        }
      } else if (item.id === 'sensors') {
        box(group, [0, 0.02, 0], [0.66, 0.28, 0.3], 0x477979, 1, item.id);
        for (const x of [-0.18, 0.18]) {
          const lens = new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.085, 0.06, 20), new THREE.MeshBasicMaterial({ color: item.color }));
          lens.rotation.x = Math.PI / 2;
          lens.position.set(x, 0.03, 0.18);
          group.add(lens);
        }
      } else if (item.id === 'data') {
        for (let level = 0; level < 3; level += 1) {
          box(group, [0, level * 0.13 - 0.02, 0], [0.68, 0.08, 0.46], level === 2 ? 0x8dadd8 : 0x4c6488, 1, item.id);
          group.add(line([[-0.24, level * 0.13, 0.235], [0.15, level * 0.13, 0.235]], item.color, 0.85));
        }
      } else if (item.id === 'policy') {
        const positions = [[-0.3, 0.04, 0.14], [0, 0.27, -0.08], [0.3, 0.04, 0.14], [0, 0.04, -0.3]];
        for (const position of positions) {
          const cell = box(group, position, [0.13, 0.13, 0.13], item.color, 1, item.id);
          cell.rotation.y = Math.PI / 4;
        }
        for (const [a, b] of [[0, 1], [1, 2], [2, 3], [3, 0], [1, 3]]) group.add(line([positions[a], positions[b]], item.color, 0.65));
      }
    }
    const button = document.createElement('button');
    button.type = 'button';
    button.className = 'scene-label';
    button.dataset.node = item.id;
    button.style.setProperty('--node-color', `#${item.color.toString(16).padStart(6, '0')}`);
    const title = document.createElement('strong');
    title.textContent = item.title;
    const subtitle = document.createElement('small');
    subtitle.textContent = item.subtitle;
    button.append(title, subtitle);
    labels.append(button);
    listen(button, 'click', () => select(item.id));
    nodes.set(item.id, { ...item, button, anchor: new THREE.Vector3(...item.position).add(new THREE.Vector3(0, item.id === 'control' ? 0.12 : 0.7, 0)) });
  }

  function wire(id, points, color) {
    const curve = new THREE.CatmullRomCurve3(points.map(p => new THREE.Vector3(...p)), false, 'centripetal');
    const geometry = new THREE.BufferGeometry().setFromPoints(curve.getPoints(40));
    const material = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.2 });
    assembly.add(new THREE.Line(geometry, material));
    const signals = [];
    for (let i = 0; i < 2; i += 1) {
      const dot = new THREE.Mesh(new THREE.SphereGeometry(0.036, 8, 6), new THREE.MeshBasicMaterial({ color }));
      assembly.add(dot);
      signals.push(dot);
    }
    wires.push({ id, curve, material, signals, active: false });
  }
  wire('input-control', [[-2.2, 0.16, 0.35], [-1.5, 0.16, 0.35], [-1.2, 0.45, 0.05], [-0.86, 0.48, 0]], 0xb2a6ff);
  wire('sensor-data', [[0.4, 0.1, 2.55], [1.45, 0.1, 2.55], [2.8, 0.1, 1.65], [2.8, 0.1, 0.6]], 0x83d9d1);
  wire('control-data', [[1.0, 0.48, 0], [1.45, 0.48, 0], [1.8, 0.16, 0.15], [2.24, 0.16, 0.15]], 0xb6d6ff);
  wire('sensor-policy', [[-0.15, 0.1, 2.1], [-1.55, 0.1, 1.65], [-1.55, 0.1, -1.65], [-0.45, 0.1, -2.55]], 0x83d9d1);
  wire('control-policy', [[0.6, 0.32, -0.7], [0.6, 0.32, -1.3], [0.6, 0.12, -1.7], [0.5, 0.12, -2.1]], 0xb6d6ff);
  wire('policy-control', [[0.1, 0.16, -2.1], [0.1, 0.16, -1.55], [0.1, 0.48, -1.1], [0.1, 0.48, -0.7]], 0xc9b5fb);

  function select(id) {
    if (!nodes.has(id) || disposed) return;
    selectedNode = id;
    nodes.forEach(node => node.button.setAttribute('aria-pressed', String(node.id === id)));
    onSelect?.(id);
    requestFrame();
  }

  function updateCamera() {
    const radius = 14;
    const azimuth = orbit.azimuth + parallax.x;
    const elevation = orbit.elevation + parallax.y;
    camera.position.set(radius * Math.sin(azimuth) * Math.cos(elevation), radius * Math.sin(elevation), radius * Math.cos(azimuth) * Math.cos(elevation));
    camera.lookAt(target);
    camera.updateMatrixWorld();
  }

  function render() {
    updateCamera();
    for (const item of wires) {
      item.signals.forEach((dot, index) => {
        dot.visible = item.active;
        if (item.active) dot.position.copy(item.curve.getPointAt((elapsed * 0.18 + index * 0.5) % 1));
      });
    }
    renderer.render(scene, camera);
    nodes.forEach(node => {
      projection.copy(node.anchor).project(camera);
      const x = (projection.x * 0.5 + 0.5) * width;
      const y = (-projection.y * 0.5 + 0.5) * height;
      node.button.style.left = `${Math.min(width - node.button.offsetWidth / 2 - 5, Math.max(node.button.offsetWidth / 2 + 5, x))}px`;
      node.button.style.top = `${Math.max(node.button.offsetHeight + 5, Math.min(height - 8, y))}px`;
    });
    container.dataset.sceneFrames = String(++frames);
  }

  function canRender() { return !disposed && visible && !document.hidden; }
  function animating() { return canRender() && !paused && !reducedMotion; }
  function tick(now) {
    frameId = 0;
    if (!canRender()) return;
    if (animating() && previousTime) elapsed += Math.min((now - previousTime) / 1000, 0.05);
    previousTime = now;
    render();
    if (animating()) frameId = requestAnimationFrame(tick);
    container.dataset.sceneAnimating = String(animating());
  }
  function requestFrame() {
    if (canRender() && !frameId) frameId = requestAnimationFrame(tick);
  }
  function syncAnimation() {
    if (frameId) cancelAnimationFrame(frameId);
    frameId = 0;
    previousTime = 0;
    container.dataset.sceneAnimating = String(animating());
    requestFrame();
  }
  function resize() {
    if (disposed) return;
    width = Math.max(container.clientWidth, 1);
    height = Math.max(container.clientHeight, 1);
    const aspect = width / height;
    const halfHeight = Math.max(3.65, 4.15 / aspect);
    camera.left = -halfHeight * aspect;
    camera.right = halfHeight * aspect;
    camera.top = halfHeight;
    camera.bottom = -halfHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
    requestFrame();
  }
  function setFlow(id) {
    if (!FLOWS[id] || disposed) return;
    flow = id;
    container.dataset.sceneFlow = flow;
    wires.forEach(item => {
      item.active = FLOWS[flow].links.includes(item.id);
      item.material.opacity = item.active ? 0.8 : 0.15;
    });
    nodes.forEach(node => {
      node.button.dataset.active = String(FLOWS[flow].nodes.includes(node.id));
      node.button.setAttribute('aria-pressed', String(node.id === selectedNode));
    });
    requestFrame();
  }
  function resetView() {
    orbit = { ...orbitDefault };
    parallax = { x: 0, y: 0 };
    requestFrame();
  }

  listen(canvas, 'pointerdown', event => {
    if (event.button !== 0) return;
    drag = { id: event.pointerId, x: event.clientX, y: event.clientY, moved: false, azimuth: orbit.azimuth, elevation: orbit.elevation };
    canvas.setPointerCapture(event.pointerId);
  });
  listen(canvas, 'pointermove', event => {
    if (drag && drag.id === event.pointerId) {
      const dx = event.clientX - drag.x;
      const dy = event.clientY - drag.y;
      drag.moved ||= Math.abs(dx) + Math.abs(dy) > 5;
      orbit.azimuth = drag.azimuth - dx * 0.005;
      orbit.elevation = Math.max(0.35, Math.min(1.2, drag.elevation + dy * 0.004));
      parallax = { x: 0, y: 0 };
      requestFrame();
    } else if (event.pointerType === 'mouse' && !reducedMotion && !paused) {
      const rect = canvas.getBoundingClientRect();
      parallax = { x: ((event.clientX - rect.left) / width - 0.5) * 0.04, y: ((event.clientY - rect.top) / height - 0.5) * 0.025 };
      requestFrame();
    }
  });
  listen(canvas, 'pointerup', event => {
    if (!drag || drag.id !== event.pointerId) return;
    if (!drag.moved) {
      const rect = canvas.getBoundingClientRect();
      pointer.set((event.clientX - rect.left) / width * 2 - 1, -(event.clientY - rect.top) / height * 2 + 1);
      raycaster.setFromCamera(pointer, camera);
      const hit = raycaster.intersectObjects(hitTargets, false)[0];
      if (hit) select(hit.object.userData.nodeId);
    }
    drag = null;
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  });
  listen(canvas, 'pointercancel', () => { drag = null; });
  listen(canvas, 'pointerleave', () => {
    if (!drag) { parallax = { x: 0, y: 0 }; requestFrame(); }
  });
  listen(canvas, 'keydown', event => {
    if (!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'r', 'R'].includes(event.key)) return;
    event.preventDefault();
    event.stopPropagation();
    if (event.key.toLowerCase() === 'r') resetView();
    if (event.key === 'ArrowLeft') orbit.azimuth -= 0.12;
    if (event.key === 'ArrowRight') orbit.azimuth += 0.12;
    if (event.key === 'ArrowUp') orbit.elevation = Math.min(1.2, orbit.elevation + 0.08);
    if (event.key === 'ArrowDown') orbit.elevation = Math.max(0.35, orbit.elevation - 0.08);
    requestFrame();
  });
  listen(document, 'visibilitychange', syncAnimation);
  const resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(container);
  const intersectionObserver = new IntersectionObserver(entries => {
    visible = entries[0].isIntersecting;
    syncAnimation();
  }, { threshold: 0 });
  intersectionObserver.observe(container);
  setFlow(flow);
  resize();
  syncAnimation();

  return {
    setFlow,
    setPaused(value) { paused = Boolean(value); syncAnimation(); },
    setReducedMotion(value) {
      reducedMotion = Boolean(value);
      if (reducedMotion) parallax = { x: 0, y: 0 };
      syncAnimation();
    },
    resetView,
    dispose() {
      if (disposed) return;
      disposed = true;
      if (frameId) cancelAnimationFrame(frameId);
      disposers.forEach(remove => remove());
      resizeObserver.disconnect();
      intersectionObserver.disconnect();
      const geometries = new Set();
      const materials = new Set();
      scene.traverse(object => {
        if (object.geometry) geometries.add(object.geometry);
        if (object.material) (Array.isArray(object.material) ? object.material : [object.material]).forEach(material => materials.add(material));
      });
      geometries.forEach(geometry => geometry.dispose());
      materials.forEach(material => material.dispose());
      renderer.dispose();
      renderer.forceContextLoss();
      canvas.remove();
      labels.remove();
      styles.remove();
      container.dataset.sceneAnimating = 'false';
    },
  };
}
