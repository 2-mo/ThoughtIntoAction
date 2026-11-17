// @ts-nocheck
// 接入配置：一套机械臂 + 相机的组合，命名保存后一键载入。
// The page edits one draft; saving stores it in the library, loading applies it
// to the document the kernel and camera service already read.
import { withToken } from "../../api/control-protocol";

let library = [];
let activeSlug = null;
let draft = null;
let discovery = { cameras: [], can_adapters: [], can_interfaces: [], vendors: {} };
let images = {};
let previewErrors = {};
let busy = false;

const MAX_CAMERAS = 4;

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

// ------------------------------------------------------------------ draft ⇄ document

function cameraId(role, index, cameras) {
  if (role === "left") return "wrist_rgb";
  if (role === "right") return "wrist_second_rgb";
  const nth = cameras.slice(0, index).filter((camera) => camera.role === "exterior").length;
  return nth === 0 ? "exterior_rgb" : `exterior_${nth + 1}_rgb`;
}

function defaultLabel(role, index, cameras) {
  if (role === "left") return cameras.filter((camera) => camera.role !== "exterior").length > 1 ? "腕部相机" : "腕部相机";
  if (role === "right") return "腕部相机 2";
  const nth = cameras.slice(0, index).filter((camera) => camera.role === "exterior").length;
  return nth === 0 ? "外部相机" : `外部相机 ${nth + 1}`;
}

function connectionKind() {
  return discovery.vendors?.[draft.vendor]?.connection || "can";
}

export function setupConnection() {
  return draft ? connectionKind() : "can";
}

function controllerFor(side) {
  if (side === "single") return draft.swapHands ? "left" : "right";
  if (!draft.swapHands) return side;
  return side === "left" ? "right" : "left";
}

function robotEntry(side, index) {
  const base = { id: index === 0 ? "arm1" : "arm2", vendor: draft.vendor, model: draft.model, side,
                 controller: controllerFor(side) };
  return connectionKind() === "ethernet"
    ? { ...base, host: draft.hosts[index] || "" }
    : { ...base, can_interface: draft.cans[index] || (index === 0 ? "can1" : "can0") };
}

function toDocument() {
  const arms = Number(draft.arms);
  const robots = arms === 2
    ? [robotEntry("left", 0), robotEntry("right", 1)]
    : [robotEntry("single", 0)];
  const cameras = draft.cameras.map((camera, index) => {
    const mount = camera.role === "exterior" ? "fixed" : "wrist";
    return {
      id: cameraId(camera.role, index, draft.cameras),
      label: camera.label || defaultLabel(camera.role, index, draft.cameras),
      enabled: true,
      driver: "realsense",
      model: camera.model || "D435i",
      serial: camera.serial || "",
      width: camera.width || 640,
      height: camera.height || 480,
      fps: camera.fps || 30,
      mount,
      arm: mount === "wrist" ? (camera.role === "right" ? "arm2" : "arm1") : null,
    };
  });
  return { version: 1, robots, cameras };
}

function fromDocument(document) {
  const robots = document.robots || [];
  const arms = robots.length === 2 ? 2 : 1;
  const cameras = (document.cameras || []).map((camera) => ({
    serial: camera.serial || "",
    role: camera.mount === "wrist"
      ? (camera.arm === "arm2" || String(camera.id).includes("second") ? "right" : "left")
      : "exterior",
    model: camera.model,
    label: camera.label,
    width: camera.width,
    height: camera.height,
    fps: camera.fps,
  }));
  return {
    vendor: robots[0]?.vendor || "arx",
    model: robots[0]?.model || "x5",
    arms,
    cans: arms === 2
      ? [robots[0]?.can_interface || "can1", robots[1]?.can_interface || "can0"]
      : [robots[0]?.can_interface || "can0"],
    hosts: robots.map((robot) => robot.host || ""),
    // A rig whose XR channels are crossed stores the opposite hand per arm.
    swapHands: robots.length === 2 && robots.some((robot) => robot.controller && robot.controller !== robot.side),
    cameras,
  };
}

// ------------------------------------------------------------------ options

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

function canOptions(selected) {
  const names = new Set((discovery.can_interfaces || []).map((item) => item.name));
  for (const adapter of discovery.can_adapters || []) if (adapter.interface) names.add(adapter.interface);
  if (selected) names.add(selected);
  const options = [...names].sort().map((name) => {
    const state = (discovery.can_interfaces || []).find((item) => item.name === name)?.state;
    return `<option value="${escape(name)}"${name === selected ? " selected" : ""}>${escape(name)}${state ? ` · ${state}` : ""}</option>`;
  }).join("");
  return options || `<option value="${escape(selected || "can0")}" selected>${escape(selected || "can0")}</option>`;
}

function serialOptions(selected) {
  const found = (discovery.cameras || []).filter((camera) => camera.serial);
  const known = found.some((camera) => camera.serial === selected);
  const assigned = new Set(draft.cameras.map((camera) => camera.serial).filter(Boolean));
  const options = found.map((camera) => {
    const taken = assigned.has(camera.serial) && camera.serial !== selected;
    return `<option value="${escape(camera.serial)}"${camera.serial === selected ? " selected" : ""}${taken ? " disabled" : ""}>`
      + `${escape(camera.model || camera.name)} · ${escape(camera.serial)}${taken ? "（已用）" : ""}</option>`;
  }).join("");
  const extra = selected && !known
    ? `<option value="${escape(selected)}" selected>${escape(selected)}（未探测到）</option>` : "";
  return `<option value=""${selected ? "" : " selected"}>选择相机…</option>${extra}${options}`;
}

// ------------------------------------------------------------------ render

function renderLibrary() {
  const host = el("setupLibraryList");
  if (!host) return;
  if (!library.length) {
    host.innerHTML = `<p class="op-hint">还没有保存的配置，先设好下面几项再保存。</p>`;
    return;
  }
  host.innerHTML = library.map((entry) => `
    <span class="setup-chip${entry.slug === activeSlug ? " active" : ""}">
      <button type="button" class="setup-chip-load" data-load="${escape(entry.slug)}" title="载入这个配置">
        <strong>${escape(entry.name)}</strong>
        <small>${entry.robots === 2 ? "双臂" : "单臂"} · ${entry.cameras} 相机</small>
      </button>
      <button type="button" class="setup-chip-del" data-del="${escape(entry.slug)}" title="删除">×</button>
    </span>`).join("");
  host.querySelectorAll("[data-load]").forEach((button) => {
    button.addEventListener("click", (event) => {
      // The strip lives inside <summary>; a chip click loads, it does not toggle.
      event.preventDefault();
      event.stopPropagation();
      applyEntry(button.dataset.load);
    });
  });
  host.querySelectorAll("[data-del]").forEach((button) => {
    button.addEventListener("click", (event) => {
      event.preventDefault();
      event.stopPropagation();
      removeEntry(button.dataset.del);
    });
  });
}

function renderArm() {
  const vendor = el("setupVendor");
  const model = el("setupModel");
  const arms = el("setupArms");
  if (!vendor) return;
  vendor.innerHTML = vendorOptions(draft.vendor);
  model.innerHTML = modelOptions(draft.vendor, draft.model);
  const counts = discovery.vendors?.[draft.vendor]?.models?.[draft.model]?.arm_counts || [1, 2];
  if (!counts.includes(Number(draft.arms))) {
    draft.arms = counts[0];
    // A single-arm rig has no right wrist; that camera becomes an exterior one.
    if (Number(draft.arms) === 1) {
      draft.cameras.forEach((camera) => { if (camera.role === "right") camera.role = "exterior"; });
    }
  }
  arms.innerHTML = counts.map((count) =>
    `<option value="${count}"${count === Number(draft.arms) ? " selected" : ""}>${count === 1 ? "单臂" : "双臂"}</option>`
  ).join("");
  const ethernet = connectionKind() === "ethernet";
  const host = el("setupCans");
  host.innerHTML = Array.from({ length: draft.arms }, (_, index) => {
    const label = draft.arms === 1 ? "" : index === 0 ? "左臂 " : "右臂 ";
    if (ethernet) {
      return `<label><span>${label}IP</span>
        <input data-host="${index}" value="${escape(draft.hosts[index] || "")}" placeholder="192.168.1.${index === 0 ? "19" : "18"}"></label>`;
    }
    return `<label><span>${label}CAN</span>
      <select data-can="${index}">${canOptions(draft.cans[index])}</select></label>`;
  }).join("");
  host.querySelectorAll("[data-can]").forEach((select) => {
    select.addEventListener("change", () => {
      draft.cans[Number(select.dataset.can)] = select.value;
    });
  });
  host.querySelectorAll("[data-host]").forEach((input) => {
    input.addEventListener("change", () => {
      draft.hosts[Number(input.dataset.host)] = input.value.trim();
    });
  });
  if (el("setupSwapHands")) el("setupSwapHands").checked = Boolean(draft.swapHands);
}

function renderCameraCount() {
  const select = el("setupCameraCount");
  if (!select) return;
  const total = Math.max(MAX_CAMERAS, draft.cameras.length);
  select.innerHTML = Array.from({ length: total }, (_, index) =>
    `<option value="${index + 1}"${index + 1 === draft.cameras.length ? " selected" : ""}>${index + 1}</option>`
  ).join("");
}

function shotFor(camera) {
  const image = camera.serial ? images[camera.serial] : null;
  if (image) return `<img src="data:image/jpeg;base64,${image}" alt="${escape(camera.serial)}">`;
  const error = camera.serial ? previewErrors[camera.serial] : null;
  const hint = !camera.serial ? "先选相机" : error ? "无画面" : "读取中…";
  return `<span class="setup-shot-empty" title="${escape(error || "")}">${hint}</span>`;
}

function renderCameras() {
  const host = el("setupCameras");
  if (!host) return;
  const roles = draft.arms === 2
    ? [["left", "左腕"], ["right", "右腕"], ["exterior", "外部"]]
    : [["left", "腕部"], ["exterior", "外部"]];
  host.innerHTML = draft.cameras.map((camera, index) => `
    <div class="setup-camera-card" data-camera="${index}" data-role="${camera.role}">
      <div class="setup-shot">${shotFor(camera)}</div>
      <select data-field="serial" aria-label="相机">${serialOptions(camera.serial)}</select>
      <select data-field="role" aria-label="安装位置">
        ${roles.map(([value, label]) =>
          `<option value="${value}"${camera.role === value ? " selected" : ""}>${label}</option>`).join("")}
      </select>
    </div>`).join("");
  host.querySelectorAll("[data-camera]").forEach((card) => {
    const index = Number(card.dataset.camera);
    const camera = draft.cameras[index];
    card.querySelectorAll("[data-field]").forEach((select) => {
      select.addEventListener("change", () => {
        if (select.dataset.field === "serial") {
          camera.serial = select.value;
          const found = (discovery.cameras || []).find((item) => item.serial === select.value);
          if (found?.model) camera.model = found.model;
          if (select.value && !images[select.value]) void loadPreviews([select.value]);
        } else {
          camera.role = select.value;
          // One camera per wrist: give up the old owner of this role.
          draft.cameras.forEach((other, otherIndex) => {
            if (otherIndex !== index && other.role === camera.role && camera.role !== "exterior") {
              other.role = "exterior";
            }
          });
        }
        renderCameras();
      });
    });
  });
  renderCameraCount();
}

function renderSummary() {
  const node = el("setupSummary");
  if (!node) return;
  const entry = library.find((item) => item.slug === activeSlug);
  node.textContent = entry ? entry.name : "未保存的配置";
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
  host.innerHTML = `<p class="op-hint">配置已载入，以下服务需要重启后生效：</p>` + restarts.map((item) =>
    `<div class="op-row"><div class="op-info"><strong>${escape(item.label)}</strong><span>${escape(item.reason)}</span></div>`
    + `<div class="op-actions"><button type="button" data-restart="${escape(item.kind)}">重启${escape(item.label)}</button></div></div>`
  ).join("");
  host.querySelectorAll("[data-restart]").forEach((button) => {
    button.addEventListener("click", () => restart(button.dataset.restart, button));
  });
}

function renderAll() {
  renderLibrary();
  renderArm();
  renderCameras();
  renderSummary();
}

// ------------------------------------------------------------------ actions

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
      body: JSON.stringify(kind === "kernel" ? { mode: "collect", confirm: true } : {}),
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
  if (!draft.vendor || !draft.model) return "先选择机械臂厂商与型号";
  if (connectionKind() === "ethernet") {
    const hosts = draft.hosts.slice(0, draft.arms);
    for (const host of hosts) {
      if (!/^\d{1,3}(\.\d{1,3}){3}$/.test(host || "")) return `机械臂 IP 不合法：${host || "空"}`;
    }
    if (new Set(hosts).size !== hosts.length) return "机械臂 IP 不能重复";
  } else if (draft.arms === 2 && draft.cans[0] === draft.cans[1]) {
    return "双臂的 CAN 接口不能相同";
  }
  const serials = new Set();
  for (const camera of draft.cameras) {
    if (!camera.serial) continue;
    if (serials.has(camera.serial)) return `相机序列号重复：${camera.serial}`;
    serials.add(camera.serial);
  }
  const wrists = draft.cameras.filter((camera) => camera.role !== "exterior");
  if (new Set(wrists.map((camera) => camera.role)).size !== wrists.length) return "每台臂只能有一台腕部相机";
  return null;
}

async function save() {
  const problem = validateLocal();
  if (problem) return message(problem, "error");
  if (busy) return;
  busy = true;
  el("setupSave").disabled = true;
  try {
    const name = (el("setupName").value || "").trim() || "未命名配置";
    // Same name overwrites that entry; a new name adds one beside it.
    const existing = library.find((entry) => entry.name === name);
    const saved = await api("/api/v1/setup/library", {
      method: "POST",
      headers: { "content-type": "application/json" },
      body: JSON.stringify({ name, setup: toDocument(), ...(existing ? { slug: existing.slug } : {}) }),
    });
    message(`已保存「${saved.entry.name}」，正在载入…`, "success");
    busy = false;
    await applyEntry(saved.entry.slug);
  } catch (error) {
    message(`保存失败：${error.message || error}`, "error");
  } finally {
    busy = false;
    el("setupSave").disabled = false;
  }
}

async function applyEntry(slug) {
  if (busy) return;
  busy = true;
  try {
    const result = await api(`/api/v1/setup/library/${encodeURIComponent(slug)}/apply`, { method: "POST" });
    activeSlug = result.active || slug;
    draft = fromDocument(result.setup);
    await refreshLibrary();
    el("setupName").value = result.name || "";
    renderAll();
    message(`已载入「${result.name || slug}」`, "success");
    renderRestarts(result.restarts || []);
    await loadPreviews();
  } catch (error) {
    message(`载入失败：${error.message || error}`, "error");
  } finally {
    busy = false;
  }
}

async function removeEntry(slug) {
  if (busy) return;
  busy = true;
  try {
    await api(`/api/v1/setup/library/${encodeURIComponent(slug)}`, { method: "DELETE" });
    if (activeSlug === slug) activeSlug = null;
    await refreshLibrary();
    renderAll();
    message("已删除。", "success");
  } catch (error) {
    message(`删除失败：${error.message || error}`, "error");
  } finally {
    busy = false;
  }
}

async function refreshLibrary() {
  try {
    const result = await api("/api/v1/setup/library");
    library = result.entries || [];
    activeSlug = result.active || null;
  } catch {
    // Keep the last known list; the page still edits the draft.
  }
}

async function loadPreviews(serials = null, refresh = false) {
  const wanted = serials || [...new Set(draft.cameras.map((camera) => camera.serial).filter(Boolean))];
  if (!wanted.length) return true;
  try {
    const query = new URLSearchParams({ serials: wanted.join(",") });
    if (refresh) query.set("refresh", "1");
    const result = await api(`/api/v1/setup/previews?${query}`);
    images = { ...images, ...(result.images || {}) };
    previewErrors = { ...previewErrors, ...(result.errors || {}) };
    for (const serial of wanted) if (!result.images?.[serial]) delete images[serial];
    renderCameras();
    return Object.keys(result.errors || {}).length === 0;
  } catch (error) {
    previewErrors = Object.fromEntries(wanted.map((serial) => [serial, String(error.message || error)]));
    renderCameras();
    return false;
  }
}

// ------------------------------------------------------------------ wiring

export async function initSetup() {
  if (!el("setupCard")) return;
  el("setupSave")?.addEventListener("click", save);
  el("setupRefresh")?.addEventListener("click", async () => {
    try {
      discovery = await api("/api/v1/setup/devices?refresh=1");
    } catch {
      // Discovery is best effort; keep the last result.
    }
    renderAll();
    message("已重新探测设备。");
    await loadPreviews(null, true);
  });
  el("setupPreview")?.addEventListener("click", async () => {
    message("正在取画面…");
    const ok = await loadPreviews(null, true);
    message(ok ? "" : "有相机取不到画面：鼠标停在格子上看原因；相机服务占用设备时先停掉它。",
      ok ? "info" : "warning");
  });
  el("setupVendor")?.addEventListener("change", () => {
    draft.vendor = el("setupVendor").value;
    draft.model = Object.keys(discovery.vendors?.[draft.vendor]?.models || {})[0] || draft.model;
    if (connectionKind() === "ethernet" && !draft.hosts.filter(Boolean).length) {
      draft.hosts = ["192.168.1.19", "192.168.1.18"];
    }
    // A different vendor is a different setup: never save over the previous name.
    el("setupName").value = "";
    renderAll();
  });
  el("setupModel")?.addEventListener("change", () => {
    draft.model = el("setupModel").value;
  });
  el("setupSwapHands")?.addEventListener("change", () => {
    draft.swapHands = el("setupSwapHands").checked;
  });
  el("setupArms")?.addEventListener("change", () => {
    draft.arms = Number(el("setupArms").value);
    if (draft.arms === 2) {
      draft.cans = [draft.cans[0] || "can1", draft.cans[1] || "can0"];
      if (draft.cans[0] === draft.cans[1]) draft.cans[1] = draft.cans[0] === "can0" ? "can1" : "can0";
      draft.hosts = [draft.hosts[0] || "192.168.1.19", draft.hosts[1] || "192.168.1.18"];
    } else {
      draft.cans = [draft.cans[0] || "can0"];
      draft.hosts = [draft.hosts[0] || "192.168.1.19"];
      draft.cameras.forEach((camera) => { if (camera.role === "right") camera.role = "exterior"; });
    }
    renderAll();
  });
  el("setupCameraCount")?.addEventListener("change", () => {
    const count = Number(el("setupCameraCount").value);
    while (draft.cameras.length < count) {
      const free = (discovery.cameras || []).find((camera) =>
        camera.serial && !draft.cameras.some((item) => item.serial === camera.serial));
      draft.cameras.push({
        serial: free?.serial || "",
        role: "exterior",
        model: free?.model || "D435i",
        width: 640, height: 480, fps: 30,
      });
    }
    draft.cameras.length = count;
    renderCameras();
    void loadPreviews();
  });

  try {
    const [setup, devices] = await Promise.all([
      api("/api/v1/setup"),
      api("/api/v1/setup/devices"),
      refreshLibrary(),
    ]);
    discovery = devices;
    draft = fromDocument(setup.setup || setup.seed);
    renderAll();
    renderRestarts(setup.restarts || []);
    const active = library.find((entry) => entry.slug === activeSlug);
    el("setupName").value = active ? active.name : (library.length ? "" : "当前配置");
    message(setup.setup ? "已载入当前配置。" : "还没有保存过配置，下面是按现有环境生成的值，命名后保存。");
    await loadPreviews();
  } catch (error) {
    message(`读取设备配置失败：${error.message || error}`, "error");
  }
}
