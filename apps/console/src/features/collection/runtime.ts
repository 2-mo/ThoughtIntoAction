// @ts-nocheck
// Kernel-owned collection: configure a session, record one episode per run, watch live status.
import { withToken } from "../../api/control-protocol";
import { listOperations, startOperation, stopOperation, operationLog } from "../../api/operations";
import { deleteEpisode, listEpisodes } from "../../api/collection";

const STATUS_MS = 500;
const OPERATIONS_MS = 1500;
const HISTORY_SECONDS = 60;
const STORAGE_KEY = "tia.collect.session";
const COUNTER_KEY = "tia.collect.counters";
const RAD_TO_DEG = 180 / Math.PI;
const JOINT_COLORS = ["#78f0c7", "#72b7ff", "#f2d06b", "#ff8f88", "#c8a5ff", "#93e36f"];

const ui = Object.fromEntries(
  [
    "sessionTask", "kernelBadge", "controlStatus", "manualLink",
    "collectedCount", "targetCount", "episodeProgress", "sessionLabel",
    "finishedCount", "failedCount", "liveDuration",
    "messageBar", "messageIcon", "message",
    "cameraTitle", "cameraSpec", "cameraStatus", "recordingBadge", "timer",
    "cameraPreviews", "stateSamples", "stateRate",
    "datasetCount", "datasetPath", "episodeList",
    "teleopBadge", "teleopSource", "teleopQuest", "teleopService", "teleopOwner", "teleopHint",
    "teleopLink", "teleopPhase",
    "cameraAge", "gripperValue", "feedbackAge", "armMode",
    "taskInput", "secondsField", "secondsInput", "targetInput",
    "cameraChoices", "rgbSelect", "outputInput",
    "startButton", "stopButton", "collectMessage", "collectLog",
    "formatValue", "dimensionsValue", "horizonValue", "rawRootValue",
    "jointList",
  ].map((id) => [id, document.getElementById(id)]),
);

const statusUrl = withToken("/api/v1/system/status");

let status = null;
let collectOperation = null;
let statusTimer = undefined;
let operationsTimer = undefined;
let tickTimer = undefined;
let statusPending = false;
let operationsPending = false;
let sampleCount = 0;
const history = [];
const countedOperations = new Set();
const runningOperations = new Set();
let counters = loadCounters();
let lastConnected = null;
let settings = loadSettings();

function loadSettings() {
  const fallback = { task: "", mode: "fixed", seconds: 30, target: 50, rgb: "mp4", output: "outputs/datasets/runtime" };
  try {
    return { ...fallback, ...JSON.parse(window.localStorage.getItem(STORAGE_KEY) || "{}") };
  } catch {
    return fallback;
  }
}

function saveSettings() {
  try {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
  } catch {
    /* storage unavailable: settings stay in memory */
  }
}

function loadCounters() {
  try {
    const stored = JSON.parse(window.sessionStorage.getItem(COUNTER_KEY) || "{}");
    return { collected: Number(stored.collected) || 0, failed: Number(stored.failed) || 0 };
  } catch {
    return { collected: 0, failed: 0 };
  }
}

function saveCounters() {
  try {
    window.sessionStorage.setItem(COUNTER_KEY, JSON.stringify(counters));
  } catch {
    /* storage unavailable: counters stay in memory */
  }
}

function selectedMode() {
  return document.querySelector('input[name="collectMode"]:checked')?.value || "fixed";
}

function enabledCameras() {
  return (status?.cameras || []).filter((camera) => camera.enabled !== false);
}

function selectedCameras() {
  return Array.from(document.querySelectorAll('#cameraChoices input[type="checkbox"]:checked')).map(
    (input) => input.value,
  );
}

function setText(node, value) {
  if (node) node.textContent = value;
}

function setBadge(node, state, label) {
  if (!node) return;
  if (node.classList.contains("status")) {
    node.classList.remove("good", "bad", "waiting");
    node.classList.add(state);
  } else {
    node.dataset.state = state;
  }
  node.textContent = label;
}

function setMessage(text, tone = "info") {
  if (!ui.message) return;
  ui.message.textContent = text;
  ui.messageBar.dataset.tone = tone;
  const icon = { info: "fa-circle-info", success: "fa-circle-check", warning: "fa-triangle-exclamation", error: "fa-circle-exclamation" };
  ui.messageIcon.className = `fa-solid ${icon[tone] || icon.info}`;
}

function setCollectMessage(text, tone = "") {
  if (!ui.collectMessage) return;
  ui.collectMessage.textContent = text;
  ui.collectMessage.dataset.tone = tone;
}

function operationRunning() {
  return Boolean(collectOperation && ["starting", "running", "stopping"].includes(collectOperation.state));
}

function terminal(operation) {
  return Boolean(operation && ["stopped", "failed"].includes(operation.state));
}

// Settings form

function renderSettings() {
  ui.taskInput.value = settings.task;
  ui.secondsInput.value = String(settings.seconds);
  ui.targetInput.value = String(settings.target);
  ui.rgbSelect.value = settings.rgb;
  ui.outputInput.value = settings.output;
  const radio = document.querySelector(`input[name="collectMode"][value="${settings.mode}"]`);
  if (radio) radio.checked = true;
  ui.secondsField.hidden = selectedMode() !== "fixed";
  updateTarget();
  renderSessionTask();
}

function readSettings() {
  settings = {
    task: ui.taskInput.value.trim(),
    mode: selectedMode(),
    seconds: Number(ui.secondsInput.value) || 30,
    target: Math.min(Math.max(Number(ui.targetInput.value) || 50, 1), 999),
    rgb: ui.rgbSelect.value,
    output: ui.outputInput.value.trim() || "outputs/datasets/runtime",
  };
  saveSettings();
  updateTarget();
  renderSessionTask();
  updateControls();
}

function renderSessionTask() {
  setText(ui.sessionTask, settings.task || "尚未设置任务描述");
  setText(ui.rawRootValue, settings.output);
}

function updateTarget() {
  setText(ui.targetCount, String(settings.target));
  ui.episodeProgress.max = settings.target;
  ui.episodeProgress.value = Math.min(counters.collected, settings.target);
  setText(ui.collectedCount, String(counters.collected));
  setText(ui.finishedCount, String(counters.collected));
  setText(ui.failedCount, String(counters.failed));
  const remaining = Math.max(settings.target - counters.collected, 0);
  setText(
    ui.sessionLabel,
    counters.collected === 0 ? "尚未开始" : `已完成 ${counters.collected} 条 · 剩余 ${remaining} 条`,
  );
}

let cameraSignature = "";

function renderCameraChoices() {
  const cameras = enabledCameras();
  const signature = `${Boolean(status)}|${cameras.map((camera) => `${camera.id}:${camera.label}`).join(",")}`;
  if (signature === cameraSignature) return;
  cameraSignature = signature;
  if (!cameras.length) {
    ui.cameraChoices.textContent = status ? "没有启用的相机" : "内核未连接，暂无法读取相机";
    return;
  }
  const only = cameras.length === 1;
  ui.cameraChoices.innerHTML = cameras
    .map(
      (camera) => `<label><input type="checkbox" value="${camera.id}" ${only ? "checked disabled" : "checked"}>
        <span>${camera.label || camera.id}<small>${camera.model || camera.id}</small></span></label>`,
    )
    .join("");
  setText(
    ui.cameraTitle,
    cameras.length > 1 ? `${cameras.length} 路相机 · 实时预览` : `${cameras[0].label || cameras[0].id} · 实时预览`,
  );
}


// Teleoperation

function renderTeleop() {
  const teleop = status?.teleop || {};
  const source = String(teleop.source || "cli");
  const input = String(teleop.input || "idle");
  const vr = source === "quest" || input === "quest";
  const owner = status?.owner?.source || null;
  const connected = Boolean(status?.connected);
  const teleopOk = Boolean(teleop.ready && teleop.armed && teleop.permit && status?.phase === "teleop");
  setText(ui.teleopSource, vr ? "VR · Quest" : source === "phone" ? "手机" : "无外部输入");
  setText(ui.teleopQuest, teleop.quest_usb ? "已连接" : "未连接");
  setText(ui.teleopService, teleop.xr_service ? "运行中" : "未启动");
  setText(ui.teleopOwner, owner ? `${owner} · ${status?.phase || "—"}` : "无人控制");
  setText(
    ui.teleopLink,
    `就绪${teleop.ready ? "✓" : "—"} · 激活${teleop.armed ? "✓" : "—"} · 许可${teleop.permit ? "✓" : "—"}`,
  );
  setText(ui.teleopPhase, status?.phase || "—");

  if (!connected) {
    setBadge(ui.teleopBadge, "waiting", "内核未连接");
  } else if (!vr) {
    setBadge(ui.teleopBadge, "waiting", "未启用 VR");
  } else if (teleopOk) {
    setBadge(ui.teleopBadge, "good", "遥操中");
  } else if (teleop.armed && !teleop.permit) {
    setBadge(ui.teleopBadge, "bad", "协调器未开启");
  } else if (teleop.ready) {
    setBadge(ui.teleopBadge, "waiting", "等待握持 Grip");
  } else {
    setBadge(ui.teleopBadge, "waiting", String(teleop.reason || "未就绪").slice(0, 18));
  }

  setText(
    ui.teleopHint,
    !vr
      ? "当前内核没有启用 Quest 输入；VR 采集需用 real-quest 配置启动内核。"
      : teleopOk
        ? "遥操中：Grip 跟随，Trigger 控制夹爪；采集时页面只负责录制与计数。"
        : teleop.armed && !teleop.permit
          ? "已激活但协调器处于停用状态：按 A 会被当成 A+B 关闭跟随，需要重启内核恢复。"
          : "握住右手柄 Grip 接入遥操；接入后 就绪 / 激活 / 许可 三项会同时变为 ✓。",
  );
}


// Dataset

const EPISODES_MS = 3000;
let episodesPending = false;
let pendingDelete = null; // Episode the operator has armed for a second click.

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, (character) => (
    { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[character]
  ));
}

async function refreshEpisodes() {
  if (episodesPending) return;
  episodesPending = true;
  try {
    const data = await listEpisodes(settings.output, 6);
    setText(ui.datasetPath, data.directory);
    setText(ui.datasetCount, data.total ? `${data.total} 条` : "暂无");
    const rows = (data.episodes || []).map((episode) => {
      const done = episode.lifecycle === "complete";
      const label = episode.lifecycle === "complete" ? "完成" : (episode.lifecycle || "未知");
      const armed = pendingDelete === episode.name;
      return `<li class="episode-item ${done ? "" : "bad"}">
        <strong>${escapeHtml(episode.task || episode.name)}</strong>
        <span class="episode-meta">
          <em>${escapeHtml(label)}</em>
          <button type="button" class="episode-delete${armed ? " confirming" : ""}"
            data-episode="${escapeHtml(episode.name)}"
            title="${armed ? "再点一次确认删除，删除后不可恢复" : "删除这条录制"}">${armed ? "确认删除" : "删除"}</button>
        </span>
        <small>${escapeHtml(episode.name)}</small>
        <small>${episode.ticks ?? "—"} tick · ${episode.frames ?? 0} 帧</small>
      </li>`;
    });
    ui.episodeList.innerHTML = rows.length
      ? rows.join("")
      : '<li class="episode-empty">还没有录制的 episode</li>';
  } catch {
    setText(ui.datasetCount, "读取失败");
  } finally {
    episodesPending = false;
  }
}

// Status polling

async function pollStatus() {
  if (statusPending) return;
  statusPending = true;
  try {
    const response = await fetch(statusUrl, { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    status = await response.json();
  } catch {
    status = null;
  } finally {
    statusPending = false;
    renderStatus();
  }
}

function renderStatus() {
  const connected = Boolean(status?.connected);
  setBadge(ui.kernelBadge, connected ? "good" : "waiting", connected ? "内核已连接" : "内核未连接");
  setBadge(ui.controlStatus, connected ? "good" : "waiting", connected ? "就绪" : "内核未连接");
  setText(ui.armMode, connected ? `运行 ${status.runtime_id || "—"} · ${status.phase || "—"}` : "内核未连接，先在总览页启动");

  const q = Array.isArray(status?.q) ? status.q : [];
  if (connected && q.length >= 6) {
    sampleCount += 1;
    history.push({ t: Date.now() / 1000, q });
    const cutoff = Date.now() / 1000 - HISTORY_SECONDS;
    while (history.length && history[0].t < cutoff) history.shift();
    for (const row of jointRows) {
      const value = q[row.offset];
      setText(row.value, Number.isFinite(value) ? `${(value * RAD_TO_DEG).toFixed(1)}°` : "--");
    }
  }
  renderJointRows();
  setText(ui.stateSamples, String(sampleCount));
  setText(ui.stateRate, rateLabel());
  drawAllJoints();

  const gripper = status?.gripper_width;
  const widths = Array.isArray(status?.gripper_widths) ? status.gripper_widths : null;
  const gripperText = widths && widths.length > 1
    ? widths.map((width, index) => {
        const label = status.arms?.[index]?.label ?? `臂${index + 1}`;
        return Number.isFinite(width) ? `${label} ${(width * 1000).toFixed(1)}` : `${label} —`;
      }).join(" · ") + " mm"
    : Number.isFinite(gripper) ? `${(gripper * 1000).toFixed(1)} mm` : "—";
  setText(ui.gripperValue, gripperText);
  const age = status?.feedback_age_ms;
  setText(ui.feedbackAge, Number.isFinite(age) ? `${age.toFixed(0)} ms` : "—");

  const cameraOnline = Boolean(status?.shared_camera);
  const videoOwnsCamera = Boolean(status?.camera_service);
  setBadge(ui.cameraStatus, cameraOnline ? "good" : "waiting", cameraOnline ? "相机服务在线" : "相机服务未启动");
  if (!cameraOnline) {
    for (const card of previewCards) {
      card.state.textContent = "未连接";
      card.empty.textContent = videoOwnsCamera ? "相机被控制台视频占用" : "相机服务未启动";
      card.figure.classList.add("loading");
    }
    setText(ui.cameraAge, "—");
  }
  renderCameraChoices();
  renderPreviewCards();
  ensureStreams();
  renderTeleop();
  updateControls();
  renderOperation();
}

function rateLabel() {
  if (history.length < 2) return "-- Hz";
  const window = history.slice(-5);
  const span = window[window.length - 1].t - window[0].t;
  if (span <= 0) return "-- Hz";
  return `${((window.length - 1) / span).toFixed(1)} Hz`;
}

function jointRange(index) {
  const lower = Array.isArray(status?.limits?.joint_min) ? status.limits.joint_min : [];
  const upper = Array.isArray(status?.limits?.joint_max) ? status.limits.joint_max : [];
  const lo = Number.isFinite(lower[index]) ? lower[index] : -Math.PI;
  const hi = Number.isFinite(upper[index]) ? upper[index] : Math.PI;
  return [lo, Math.max(hi - lo, 1e-6)];
}

function renderJointRows() {
  const arms = Array.isArray(status?.arms) && status.arms.length
    ? status.arms
    : [{ id: "arm1", label: "机械臂", joints: new Array(6).fill(null) }];
  const signature = `${arms.length}|${arms.map((arm) => (arm.joints || []).length).join(",")}`;
  if (signature === jointSignature) return;
  jointSignature = signature;
  const showLabel = arms.length > 1;
  ui.jointList.innerHTML = "";
  ui.jointList.classList.toggle("is-dual", showLabel);
  jointRows = [];
  arms.forEach((arm, armIndex) => {
    const group = document.createElement("div");
    group.className = "joint-arm";
    if (showLabel) {
      const heading = document.createElement("div");
      heading.className = "joint-arm-name";
      heading.textContent = arm.label || arm.id;
      group.append(heading);
    }
    ui.jointList.append(group);
    (arm.joints || []).forEach((_, index) => {
      const row = document.createElement("div");
      row.className = "joint-row";
      const name = document.createElement("span");
      name.className = "joint-name";
      name.textContent = `J${index + 1}`;
      const canvas = document.createElement("canvas");
      canvas.width = 320;
      canvas.height = 28;
      canvas.setAttribute("role", "img");
      canvas.setAttribute("aria-label", `${arm.label || arm.id} J${index + 1} 关节角度曲线`);
      const value = document.createElement("strong");
      value.textContent = "--";
      row.append(name, canvas, value);
      group.append(row);
      jointRows.push({ canvas, value, offset: armIndex * 6 + index });
    });
  });
}

function drawJoint(row) {
  const canvas = row.canvas;
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 10 || rect.height < 10) return;
  const dpr = window.devicePixelRatio || 1;
  const width = Math.round(rect.width * dpr);
  const height = Math.round(rect.height * dpr);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext("2d");
  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  context.clearRect(0, 0, rect.width, rect.height);

  context.strokeStyle = "rgba(255,255,255,.06)";
  context.lineWidth = 1;
  for (let row = 1; row < 4; row += 1) {
    const y = (rect.height * row) / 4;
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(rect.width, y);
    context.stroke();
  }
  if (history.length < 2) return;

  const now = history[history.length - 1].t;
  const span = Math.min(HISTORY_SECONDS, Math.max(10, now - history[0].t));
  const start = now - span;
  const [lo, range] = jointRange(row.offset);
  context.strokeStyle = JOINT_COLORS[row.offset % JOINT_COLORS.length];
  context.lineWidth = 1.5;
  context.beginPath();
  let started = false;
  for (const sample of history) {
    if (sample.t < start) continue;
    const x = ((sample.t - start) / span) * rect.width;
    const value = sample.q[row.offset];
    if (!Number.isFinite(value)) continue;
    const ratio = Math.min(Math.max((value - lo) / range, 0), 1);
    const y = rect.height - ratio * rect.height;
    if (started) context.lineTo(x, y);
    else {
      context.moveTo(x, y);
      started = true;
    }
  }
  context.stroke();
}

function drawAllJoints() {
  for (const row of jointRows) drawJoint(row);
}

// Operations

async function pollOperations() {
  if (operationsPending) return;
  operationsPending = true;
  try {
    const list = await listOperations();
    collectOperation = list.find((operation) => operation.kind === "collect") || null;
  } catch {
    collectOperation = null;
  } finally {
    operationsPending = false;
  }
  await trackOperation();
  renderOperation();
}

async function trackOperation() {
  const operation = collectOperation;
  if (!operation) return;
  if (["starting", "running", "stopping"].includes(operation.state)) {
    runningOperations.add(operation.started_at);
    try {
      const lines = await operationLog("collect", 40);
      ui.collectLog.textContent = lines.slice(-14).join("\n") || "—";
    } catch {
      /* log is best effort */
    }
    return;
  }
  if (!terminal(operation) || countedOperations.has(operation.started_at)) return;
  if (!runningOperations.has(operation.started_at)) return;
  countedOperations.add(operation.started_at);
  if (operation.state === "stopped" && operation.exit_code === 0) {
    counters.collected += 1;
    setCollectMessage(`已保存第 ${counters.collected} 条 episode`, "success");
    setMessage(`第 ${counters.collected} 条已写入 ${settings.output}`, "success");
  } else {
    counters.failed += 1;
    setCollectMessage(`采集失败（退出码 ${operation.exit_code ?? "?"}），查看运行日志`, "error");
    setMessage("采集失败，检查内核与相机服务", "error");
  }
  saveCounters();
  updateTarget();
  void refreshEpisodes();
}

function renderOperation() {
  const running = operationRunning();
  ui.recordingBadge.hidden = !running;
  if (!running) setText(ui.timer, "00:00.0");
  if (running) {
    setMessage(`正在采集第 ${counters.collected + 1} 条 · ${settings.mode === "fixed" ? `${settings.seconds} 秒` : "手动停止"}`);
  } else {
    const connected = Boolean(status?.connected);
    if (connected !== lastConnected) {
      lastConnected = connected;
      setMessage(
        connected ? "内核已就绪，填写任务描述后开始采集" : "内核未连接：先在总览页启动控制内核",
        connected ? "info" : "warning",
      );
    }
  }
  updateControls();
}

function updateControls() {
  const running = operationRunning();
  const connected = Boolean(status?.connected);
  const task = ui.taskInput.value.trim();
  ui.startButton.disabled = running || !connected || !task;
  ui.stopButton.disabled = !running;
  ui.startButton.title = !connected ? "需要运行中的内核" : !task ? "先填写任务描述" : "Space";
}

// Preview

let previewCards = [];
let previewSignature = "";
let jointRows = [];
let jointSignature = "";

function streamUrl(cameraId) {
  const url = new URL(withToken(`/api/v1/collection/camera-stream/${cameraId}.mjpeg`), window.location.href);
  url.searchParams.set("t", String(Date.now()));
  return url.toString();
}

function renderPreviewCards() {
  const cameras = enabledCameras();
  const signature = cameras.map((camera) => `${camera.id}:${camera.label}`).join(",");
  if (signature === previewSignature) return;
  previewSignature = signature;
  ui.cameraPreviews.replaceChildren();
  previewCards = cameras.map((camera) => {
    const figure = document.createElement("figure");
    figure.className = "camera-preview loading";
    const img = document.createElement("img");
    img.alt = `${camera.label || camera.id} 预览`;
    img.decoding = "async";
    const empty = document.createElement("div");
    empty.className = "camera-empty";
    empty.textContent = "相机服务未启动";
    const caption = document.createElement("figcaption");
    const name = document.createElement("strong");
    name.textContent = camera.label || camera.id;
    const state = document.createElement("span");
    state.textContent = "未连接";
    caption.append(name, state);
    figure.append(img, empty, caption);
    img.addEventListener("load", () => {
      figure.classList.remove("loading");
      state.textContent = "实时";
    });
    img.addEventListener("error", () => {
      figure.classList.add("loading");
      state.textContent = "等待画面";
    });
    ui.cameraPreviews.append(figure);
    return { id: camera.id, figure, img, empty, state };
  });
  ensureStreams();
}

function ensureStreams() {
  const online = Boolean(status?.shared_camera);
  let live = 0;
  for (const card of previewCards) {
    if (!online) {
      if (card.img.hasAttribute("src")) card.img.removeAttribute("src");
      card.figure.classList.add("loading");
      card.state.textContent = "未连接";
      continue;
    }
    if (!card.img.hasAttribute("src")) card.img.src = streamUrl(card.id);
    if (!card.figure.classList.contains("loading")) live += 1;
  }
  setText(ui.cameraAge, online ? `${live} / ${previewCards.length} 路在线` : "—");
}

// Actions

async function startCollection() {
  readSettings();
  if (!settings.task) {
    setCollectMessage("请先填写任务描述", "error");
    ui.taskInput.focus();
    return;
  }
  if (!status?.connected) {
    setCollectMessage("没有运行中的内核", "error");
    return;
  }
  const cameras = selectedCameras();
  if (!cameras.length) {
    setCollectMessage("至少选择一台相机", "error");
    return;
  }
  ui.startButton.disabled = true;
  setCollectMessage("正在启动采集…");
  try {
    const payload = {
      task: settings.task,
      mode: settings.mode,
      seconds: settings.mode === "fixed" ? settings.seconds : 0,
      rgb: settings.rgb,
      output: settings.output,
      cameras,
    };
    collectOperation = await startOperation("collect", payload);
    setCollectMessage("采集中…");
    setMessage(`正在采集第 ${counters.collected + 1} 条`);
  } catch (error) {
    setCollectMessage(String(error.message || error), "error");
  } finally {
    updateControls();
    renderOperation();
  }
}

async function stopCollection() {
  ui.stopButton.disabled = true;
  setCollectMessage("正在停止并写入…");
  try {
    collectOperation = await stopOperation("collect");
  } catch (error) {
    setCollectMessage(String(error.message || error), "error");
  } finally {
    updateControls();
    renderOperation();
  }
}

function startTimer() {
  window.clearInterval(tickTimer);
  tickTimer = window.setInterval(() => {
    if (!operationRunning()) return;
    const elapsed = Math.max(0, Date.now() / 1000 - collectOperation.started_at);
    setText(ui.liveDuration, `${elapsed.toFixed(1)} s`);
    const minutes = Math.floor(elapsed / 60);
    const seconds = Math.floor(elapsed % 60);
    const tenths = Math.floor((elapsed * 10) % 10);
    setText(
      ui.timer,
      `${String(minutes).padStart(2, "0")}:${String(seconds).padStart(2, "0")}.${tenths}`,
    );
  }, 100);
}

// Wiring

for (const link of document.querySelectorAll(".header-actions a")) {
  link.href = withToken(link.getAttribute("href") || "/");
}
for (const link of document.querySelectorAll('a[href="/control"]')) {
  link.href = withToken("/control");
}

for (const input of [ui.taskInput, ui.secondsInput, ui.targetInput, ui.rgbSelect, ui.outputInput]) {
  input.addEventListener("change", readSettings);
}
ui.taskInput.addEventListener("input", () => {
  settings.task = ui.taskInput.value.trim();
  saveSettings();
  renderSessionTask();
  updateControls();
});
ui.targetInput.addEventListener("input", readSettings);
for (const radio of document.querySelectorAll('input[name="collectMode"]')) {
  radio.addEventListener("change", () => {
    ui.secondsField.hidden = selectedMode() !== "fixed";
    readSettings();
  });
}
ui.cameraChoices.addEventListener("change", updateControls);
ui.startButton.addEventListener("click", startCollection);
ui.stopButton.addEventListener("click", stopCollection);

document.addEventListener("keydown", (event) => {
  if (event.code !== "Space") return;
  const target = event.target;
  if (target instanceof HTMLElement && ["INPUT", "TEXTAREA", "SELECT", "BUTTON"].includes(target.tagName)) return;
  event.preventDefault();
  if (operationRunning()) stopCollection();
  else if (!ui.startButton.disabled) startCollection();
});

window.addEventListener("resize", drawAllJoints);


renderSettings();
renderStatus();
setCollectMessage("填写任务描述后开始采集");
startTimer();
pollStatus();
pollOperations();
statusTimer = window.setInterval(pollStatus, STATUS_MS);
operationsTimer = window.setInterval(pollOperations, OPERATIONS_MS);
window.setInterval(refreshEpisodes, EPISODES_MS);
refreshEpisodes();

// Two-step delete: the first click arms the row, the second one removes it.
ui.episodeList?.addEventListener("click", async (event) => {
  const button = event.target.closest("button[data-episode]");
  if (!button) return;
  const name = button.dataset.episode;
  if (!name) return;
  if (operationRunning()) {
    setMessage("采集中，先停止采集再删除", "warning");
    return;
  }
  if (pendingDelete !== name) {
    pendingDelete = name;
    void refreshEpisodes();
    return;
  }
  pendingDelete = null;
  button.disabled = true;
  try {
    await deleteEpisode(settings.output, name);
    setMessage(`已删除 ${name}`, "success");
  } catch (error) {
    setMessage(`删除失败：${error.message || error}`, "error");
  } finally {
    await refreshEpisodes();
  }
});
