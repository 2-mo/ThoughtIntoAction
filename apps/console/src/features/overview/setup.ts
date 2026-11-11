// @ts-nocheck
// Device setup: robots, cameras and how each camera is mounted.
// The page owns no state beyond the document it edits; saving validates on the
// backend and reports what has to restart.
import { withToken } from "../../api/control-protocol";

const ROBOT_ID = /^[a-z][a-z0-9_-]{0,31}$/;
const CAMERA_ID = /^[a-z][a-z0-9_]*_rgb$/;

let document_ = null;
let discovery = { cameras: [], can_adapters: [], can_interfaces: [], vendors: {} };
let running = { kernel: false, camera: false };
let busy = false;

const el = (id) => document.getElementById(id);

async function api(path, options = {}) {
  const response = await fetch(withToken(path), { cache: "no-store", ...options });
  const text = await response.text();
  let data = null;
  if (text) {
    try {
      data = JSON.parse(text);
    } catch {
      throw new Error("后端未运行（静态预览）");
    }
  }
  if (!response.ok) throw new Error(data?.detail || `HTTP ${response.status}`);
  return data;
}

function message(text, kind = "info") {
  const node = el("setupMessage");
  if (!node) return;
  node.textContent = text;
  node.dataset.kind = kind;
}

function escape(value) {
  return String(value ?? "").replace(/[&<>"']/g, (c) => (
    { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]
  ));
}

function vendorOptions(selected) {
  return Object.entries(discovery.vendors || {}).map(([key, vendor]) => {
    const disabled = vendor.implemented ? "" : " disabled";
    const suffix = vendor.implemented ? "" : "（待接入）";
    return `<option value="${key}"${key === selected ? " selected" : ""}${disabled}>${escape(vendor.label)}${suffix}</option>`;
  }).join("");
}

function modelOptions(vendor, selected) {
  const models = discovery.vendors?.[vendor]?.models || {};
  return Object.entries(models).map(([key, spec]) =>
    `<option value="${key}"${key === selected ? " selected" : ""}>${escape(spec.label)}</option>`
  ).join("");
}

function serialOptions(selected) {
  const found = (discovery.cameras || []).filter((camera) => camera.serial);
  return `<option value="">选择已探测相机…</option>` + found.map((camera) =>
    `<option value="${escape(camera.serial)}"${camera.serial === selected ? " selected" : ""}>`
    + `${escape(camera.model || camera.name)} · ${escape(camera.serial)}</option>`
  ).join("");
}

function canOptions(selected) {
  const names = new Set((discovery.can_interfaces || []).map((item) => item.name));
  for (const adapter of discovery.can_adapters || []) if (adapter.interface) names.add(adapter.interface);
  if (selected) names.add(selected);
  return [...names].sort().map((name) => {
    const state = (discovery.can_interfaces || []).find((item) => item.name === name)?.state;
    return `<option value="${escape(name)}"${name === selected ? " selected" : ""}>${escape(name)}${state ? ` · ${state}` : ""}</option>`;
  }).join("") || `<option value="${escape(selected || "can0")}" selected>${escape(selected || "can0")}</option>`;
}

function armOptions(selected) {
  return (document_?.robots || []).map((robot) =>
    `<option value="${escape(robot.id)}"${robot.id === selected ? " selected" : ""}>${escape(robot.id)}（${escape(robot.side)}）</option>`
  ).join("");
}

function renderRobots() {
  const host = el("setupRobots");
  if (!host) return;
  host.innerHTML = (document_.robots || []).map((robot, index) => `
    <div class="setup-row" data-robot="${index}">
      <label><span>ID</span><input data-field="id" value="${escape(robot.id)}" ${document_.robots.length === 1 ? "readonly" : ""}></label>
      <label><span>厂商</span><select data-field="vendor">${vendorOptions(robot.vendor)}</select></label>
      <label><span>型号</span><select data-field="model">${modelOptions(robot.vendor, robot.model)}</select></label>
      <label><span>臂侧</span><select data-field="side" ${document_.robots.length === 1 ? "disabled" : ""}>
        <option value="left"${robot.side === "left" ? " selected" : ""}>left</option>
        <option value="right"${robot.side === "right" ? " selected" : ""}>right</option>
      </select></label>
      <label><span>CAN</span><select data-field="can_interface">${canOptions(robot.can_interface)}</select></label>
      <button type="button" class="setup-remove" data-remove-robot="${index}" title="移除这台机械臂">移除</button>
    </div>`).join("");
  host.querySelectorAll("[data-robot]").forEach((row) => {
    const index = Number(row.dataset.robot);
    row.querySelectorAll("[data-field]").forEach((input) => {
      input.addEventListener("change", () => {
        const robot = document_.robots[index];
        if (!robot) return;
        robot[input.dataset.field] = input.value;
        if (input.dataset.field === "vendor") {
          robot.model = Object.keys(discovery.vendors?.[robot.vendor]?.models || {})[0] || robot.model;
          renderRobots();
        }
        if (input.dataset.field === "id") renderCameras();
      });
    });
    row.querySelector("[data-remove-robot]")?.addEventListener("click", () => {
      if (document_.robots.length <= 1) return message("至少保留一台机械臂", "warning");
      document_.robots.splice(index, 1);
      if (document_.robots.length === 1) document_.robots[0].side = "single";
      renderRobots();
      renderCameras();
    });
  });
}

function renderCameras() {
  const host = el("setupCameras");
  if (!host) return;
  host.innerHTML = `<div class="setup-camera-head"><span>相机 ID</span><span>名称</span><span>已探测设备</span><span>型号</span><span>分辨率 / 帧率</span><span>安装</span><span>绑定</span><span></span></div>`
    + (document_.cameras || []).map((camera, index) => `
    <div class="setup-row setup-camera-row" data-camera="${index}">
      <input data-field="id" value="${escape(camera.id)}" aria-label="相机 ID">
      <input data-field="label" value="${escape(camera.label || "")}" aria-label="相机名称">
      <select data-field="serial" aria-label="已探测设备">${serialOptions(camera.serial)}</select>
      <select data-field="model" aria-label="型号">
        <option value="D405"${camera.model === "D405" ? " selected" : ""}>D405</option>
        <option value="D435i"${camera.model === "D435i" ? " selected" : ""}>D435i</option>
        <option value=""${!camera.model ? " selected" : ""}>不限</option>
      </select>
      <span class="setup-resolution">
        <input data-field="width" type="number" min="1" value="${camera.width || 640}" aria-label="宽度">
        ×
        <input data-field="height" type="number" min="1" value="${camera.height || 480}" aria-label="高度">
        @
        <input data-field="fps" type="number" min="1" value="${camera.fps || 30}" aria-label="帧率">
      </span>
      <select data-field="mount" aria-label="安装位置">
        <option value="wrist"${camera.mount === "wrist" ? " selected" : ""}>腕部</option>
        <option value="fixed"${camera.mount === "fixed" ? " selected" : ""}>外部固定</option>
      </select>
      <select data-field="arm" aria-label="绑定机械臂" ${camera.mount === "wrist" ? "" : "disabled"}>${armOptions(camera.arm)}</select>
      <button type="button" class="setup-remove" data-remove-camera="${index}" title="移除这台相机">移除</button>
    </div>`).join("");
  host.querySelectorAll("[data-camera]").forEach((row) => {
    const index = Number(row.dataset.camera);
    row.querySelectorAll("[data-field]").forEach((input) => {
      const apply = () => {
        const camera = document_.cameras[index];
        if (!camera) return;
        const field = input.dataset.field;
        camera[field] = ["width", "height", "fps"].includes(field) ? Number(input.value) : input.value;
        if (field === "mount") {
          camera.arm = camera.mount === "wrist" ? (document_.robots[0]?.id || "") : null;
          renderCameras();
        }
      };
      input.addEventListener("change", apply);
    });
    row.querySelector("[data-remove-camera]")?.addEventListener("click", () => {
      document_.cameras.splice(index, 1);
      renderCameras();
    });
  });
}

function renderRestarts(restarts = []) {
  const host = el("setupRestarts");
  if (!host) return;
  if (!restarts.length) {
    host.hidden = true;
    host.innerHTML = "";
    return;
  }
  host.hidden = false;
  host.innerHTML = `<p class="op-hint">配置已保存，以下服务需要重启后生效：</p>` + restarts.map((item) =>
    `<div class="op-row"><div class="op-info"><strong>${escape(item.label)}</strong><span>${escape(item.reason)}</span></div>`
    + `<div class="op-actions"><button type="button" data-restart="${escape(item.kind)}">重启${escape(item.label)}</button></div></div>`
  ).join("");
  host.querySelectorAll("[data-restart]").forEach((button) => {
    button.addEventListener("click", () => restart(button.dataset.restart, button));
  });
}

async function restart(kind, button) {
  if (busy) return;
  busy = true;
  button.disabled = true;
  try {
    await fetch(withToken(`/api/v1/operations/${kind}/stop`), { method: "POST" });
    await new Promise((resolve) => setTimeout(resolve, 1200));
    const result = await api(`/api/v1/operations/${kind}/start`, {
      method: "POST",
      headers: { "content-type": "application/json" },
      body: JSON.stringify(kind === "kernel" ? { profile: "real-dual-quest", confirm: true } : {}),
    });
    message(`${kind === "kernel" ? "内核" : "相机服务"}已重启（${result.state || "running"}）`, "success");
  } catch (error) {
    message(`重启失败：${error.message || error}`, "error");
  } finally {
    busy = false;
    button.disabled = false;
  }
}

function validateLocal() {
  const robots = document_.robots || [];
  const ids = new Set();
  for (const robot of robots) {
    if (!ROBOT_ID.test(robot.id || "")) return `机械臂 ID 不合法：${robot.id || "空"}`;
    if (ids.has(robot.id)) return `机械臂 ID 重复：${robot.id}`;
    ids.add(robot.id);
  }
  if (robots.length === 2) {
    const sides = new Set(robots.map((robot) => robot.side));
    if (!sides.has("left") || !sides.has("right") || sides.size !== 2) return "双臂需要一台 left、一台 right";
  }
  const cameras = new Set();
  for (const camera of document_.cameras || []) {
    if (!CAMERA_ID.test(camera.id || "")) return `相机 ID 必须以 _rgb 结尾：${camera.id || "空"}`;
    if (cameras.has(camera.id)) return `相机 ID 重复：${camera.id}`;
    cameras.add(camera.id);
    if (camera.mount === "wrist" && !ids.has(camera.arm)) return `${camera.id} 需要绑定到已配置的机械臂`;
  }
  return null;
}

async function save() {
  const problem = validateLocal();
  if (problem) return message(problem, "error");
  busy = true;
  el("setupSave").disabled = true;
  try {
    const result = await api("/api/v1/setup", {
      method: "PUT",
      headers: { "content-type": "application/json" },
      body: JSON.stringify(document_),
    });
    document_ = result.setup;
    renderRobots();
    renderCameras();
    message("已保存。", "success");
    renderRestarts(result.restarts || []);
  } catch (error) {
    message(`保存失败：${error.message || error}`, "error");
  } finally {
    busy = false;
    el("setupSave").disabled = false;
  }
}

async function refreshDiscovery() {
  try {
    discovery = await api("/api/v1/setup/devices?refresh=1");
  } catch {
    // Discovery is best effort; keep the last result.
  }
}

export async function initSetup() {
  if (!el("setupCard")) return;
  el("setupSave")?.addEventListener("click", save);
  el("setupRefresh")?.addEventListener("click", async () => {
    await refreshDiscovery();
    renderRobots();
    renderCameras();
    message("已重新探测设备。");
  });
  el("setupAddRobot")?.addEventListener("click", () => {
    if (document_.robots.length >= 2) return message("目前最多支持两台机械臂", "warning");
    const used = new Set(document_.robots.map((robot) => robot.side));
    document_.robots.push({
      id: document_.robots.length ? "arm2" : "arm1",
      vendor: "arx", model: "x5",
      side: used.has("left") ? "right" : "left",
      can_interface: (discovery.can_interfaces || [])[document_.robots.length]?.name || `can${document_.robots.length}`,
    });
    if (document_.robots.length === 2) {
      document_.robots[0].side = used.has("left") ? "right" : "left";
      document_.robots[1].side = document_.robots[0].side === "left" ? "right" : "left";
    }
    renderRobots();
    renderCameras();
  });
  el("setupAddCamera")?.addEventListener("click", () => {
    const free = (discovery.cameras || []).find((camera) =>
      !document_.cameras.some((item) => item.serial === camera.serial));
    const suffix = document_.cameras.length + 1;
    document_.cameras.push({
      id: `camera${suffix}_rgb`,
      label: free?.model || "相机",
      driver: "realsense",
      model: free?.model || "D405",
      serial: free?.serial || "",
      width: 640, height: 480, fps: 30,
      enabled: true, mount: "fixed", arm: null,
    });
    renderCameras();
  });

  try {
    const [setup, devices] = await Promise.all([api("/api/v1/setup"), api("/api/v1/setup/devices")]);
    document_ = setup.setup || setup.seed;
    discovery = devices;
    running = setup.running || running;
    renderRobots();
    renderCameras();
    renderRestarts(setup.restarts || []);
    message(setup.setup ? "已载入保存的配置。" : "尚未保存过配置，下面是根据当前环境生成的初始值。");
  } catch (error) {
    message(`读取设备配置失败：${error.message || error}`, "error");
  }
}
