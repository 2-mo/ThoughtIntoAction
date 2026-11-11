// @ts-nocheck
// Read-only status poller plus whitelisted operations (kernel, camera).
import { withToken } from "../../api/control-protocol";
import { listOperations, startOperation, stopOperation } from "../../api/operations";
import { questStatus, runAction } from "../../api/system";
import { initSetup } from "./setup";

const POLL_MS = 200;
const OPERATIONS_MS = 2000;
const TREND_SECONDS = 60;
const RAD_TO_DEG = 180 / Math.PI;
const JOINT_LABELS = ["J1", "J2", "J3", "J4", "J5", "J6"];
const JOINTS_PER_ARM = 6;
const JOINT_COLORS = ["#ffbd6b", "#79dfac", "#6bb8ff", "#ff8a8a", "#c79bff", "#e7e0a6"];

const ui = Object.fromEntries(
  [
    "overviewConnection", "overviewAge", "overviewPhase", "overviewJoints",
    "overviewGripper", "overviewPose", "overviewRuntime", "overviewOwner",
    "overviewLease", "overviewOperations", "overviewMessage", "overviewTrend",
    "overviewTrendHint", "overviewKernelState", "overviewKernelStart",
    "overviewKernelStop", "overviewKernelProfile", "overviewKernelConfirm",
    "overviewKernelConfirmStart", "overviewKernelCancel", "overviewKernelMessage",
    "overviewCameraState", "overviewCameraStart", "overviewCameraStop",
    "overviewCameraHint", "overviewCameraMessage",
    "wizardProgress", "wizardMessage", "wizardCan", "wizardCanBadge", "wizardCanDetail",
    "wizardCanAction", "wizardQuest", "wizardQuestBadge", "wizardQuestDetail",
    "wizardQuestAction", "wizardKernel", "wizardKernelBadge", "wizardKernelDetail",
    "wizardKernelAction", "wizardTeleop", "wizardTeleopBadge", "wizardTeleopDetail",
    "wizardCamera", "wizardCameraBadge", "wizardCameraDetail", "wizardCameraAction",
  ].map((id) => [id, document.getElementById(id)]),
);

const statusUrl = withToken("/api/v1/system/status");

for (const link of document.querySelectorAll(".overview-links a")) {
  link.href = withToken(link.getAttribute("href") || "/");
}

let timer = undefined;
let operationsTimer = undefined;
let inFlight = false;
let trendLimits = null;
const trend = [];
const operations = new Map();
let kernelLive = false;
let cameraLive = false;
let lastStatus = null;
let quest = null;
let questPending = false;
let wizardBusy = false;

function setText(node, value) {
  if (node) node.textContent = value;
}

function setConnection(state, label) {
  if (!ui.overviewConnection) return;
  ui.overviewConnection.dataset.state = state;
  ui.overviewConnection.textContent = label;
}

function setBadge(node, state, label) {
  if (!node) return;
  node.dataset.state = state;
  node.textContent = label;
}

function jointRange(limits, index) {
  const lower = Array.isArray(limits?.joint_min) ? limits.joint_min : [];
  const upper = Array.isArray(limits?.joint_max) ? limits.joint_max : [];
  const lo = Number.isFinite(lower[index]) ? lower[index] : -Math.PI;
  const hi = Number.isFinite(upper[index]) ? upper[index] : Math.PI;
  return [lo, Math.max(hi - lo, 1e-6)];
}

function renderJoints(arms, limits) {
  if (!ui.overviewJoints) return;
  const groups = Array.isArray(arms) && arms.length
    ? arms
    : [{ id: "arm1", label: "机械臂", joints: [] }];
  ui.overviewJoints.innerHTML = groups.map((arm, armIndex) => {
    const rows = (arm.joints || []).map((value, index) => {
      const offset = armIndex * JOINTS_PER_ARM + index;
      const [lo, span] = jointRange(limits, offset);
      const ratio = Math.min(Math.max((value - lo) / span, 0), 1) * 100;
      return `<div class="overview-joint">
      <span class="overview-joint-label">${JOINT_LABELS[index] ?? index + 1}</span>
      <div class="overview-joint-track"><span style="width:${ratio.toFixed(1)}%"></span></div>
      <strong>${(value * RAD_TO_DEG).toFixed(1)}°</strong>
    </div>`;
    }).join("");
    const heading = groups.length > 1
      ? `<div class="overview-arm-name">${arm.label || arm.id}</div>`
      : "";
    return heading + rows;
  }).join("");
}

function drawTrend() {
  const canvas = ui.overviewTrend;
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

  if (trend.length < 2) {
    setText(ui.overviewTrendHint, "等待反馈");
    return;
  }
  const now = trend[trend.length - 1].t;
  const windowSeconds = Math.min(TREND_SECONDS, Math.max(10, now - trend[0].t));
  const start = now - windowSeconds;
  setText(ui.overviewTrendHint, `${Math.round(windowSeconds)} 秒 · ${trend.length} 个采样`);
  for (let joint = 0; joint < 6; joint += 1) {
    const [lo, span] = jointRange(trendLimits, joint);
    context.strokeStyle = JOINT_COLORS[joint];
    context.lineWidth = 1.5;
    context.beginPath();
    let started = false;
    for (const sample of trend) {
      if (sample.t < start) continue;
      const x = ((sample.t - start) / windowSeconds) * rect.width;
      const ratio = Math.min(Math.max((sample.q[joint] - lo) / span, 0), 1);
      const y = rect.height - ratio * rect.height;
      if (started) context.lineTo(x, y);
      else { context.moveTo(x, y); started = true; }
    }
    context.stroke();
  }
}

function renderDisconnected(reason) {
  setConnection("disconnected", "未连接内核");
  setText(ui.overviewAge, "反馈延迟 —");
  setText(ui.overviewPhase, "—");
  setText(ui.overviewGripper, "—");
  setText(ui.overviewPose, "—");
  setText(ui.overviewRuntime, "—");
  setText(ui.overviewOwner, "—");
  setText(ui.overviewLease, "—");
  setText(ui.overviewOperations, "—");
  if (ui.overviewJoints) ui.overviewJoints.replaceChildren();
  setText(ui.overviewMessage, reason || "未连接内核 · 先启动控制内核");
  drawTrend();
}

function renderCapabilities(data) {
  const cameras = (Array.isArray(data.cameras) ? data.cameras : [])
    .filter((camera) => camera.enabled !== false);
  cameraLive = Boolean(data.camera_service);
  setText(ui.overviewCameraHint, cameras.length
    ? `${cameras.map((camera) => camera.label || camera.id).join(" · ")} · WebRTC`
    : "未配置相机");
}

function renderConnected(data) {
  kernelLive = Boolean(data.connected);
  setConnection("connected", "已连接");
  setText(ui.overviewAge, `反馈延迟 ${Math.round(data.feedback_age_ms)} ms`);
  setText(ui.overviewPhase, data.stopped ? "已锁定" : data.phase);
  if (Array.isArray(data.q)) {
    trendLimits = data.limits || null;
    trend.push({ t: performance.now() / 1000, q: data.q });
    const cutoff = performance.now() / 1000 - TREND_SECONDS;
    while (trend.length && trend[0].t < cutoff) trend.shift();
  }
  renderJoints(data.arms, data.limits);
  const width = Number(data.gripper_width);
  const maxWidth = Number(data.limits?.gripper_max_width);
  setText(ui.overviewGripper, Number.isFinite(width)
    ? `${(width * 1000).toFixed(1)} mm${Number.isFinite(maxWidth) ? ` / ${(maxWidth * 1000).toFixed(0)}` : ""}`
    : "—");
  const pose = Array.isArray(data.pose) ? data.pose : [];
  setText(ui.overviewPose, pose.length >= 3
    ? `${(pose[0] * 1000).toFixed(0)}, ${(pose[1] * 1000).toFixed(0)}, ${(pose[2] * 1000).toFixed(0)} mm`
    : "—");
  setText(ui.overviewRuntime, data.runtime_id || "—");
  setText(ui.overviewOwner, data.owner
    ? [data.owner.source, data.owner.purpose].filter(Boolean).join(" · ") || "已接管"
    : "空闲");
  setText(ui.overviewLease, data.owner && data.lease_remaining_s > 0
    ? `${Number(data.lease_remaining_s).toFixed(1)} s`
    : "无");
  const running = Array.isArray(data.operations) ? data.operations : [];
  setText(ui.overviewOperations, running.length
    ? running.map((operation) => `${operation.operation_id} · ${operation.status}`).join("；")
    : "空闲");
  setText(ui.overviewMessage, "只读监看 · 不申请控制权，不改变机械臂状态");
  drawTrend();
}

function renderOperations() {
  const kernel = operations.get("kernel");
  const ours = kernel && ["starting", "running", "stopping"].includes(kernel.state);
  if (ours) {
    setBadge(ui.overviewKernelState, "on", kernel.state === "stopping" ? "停止中" : "启动中");
  } else if (kernelLive) {
    setBadge(ui.overviewKernelState, "on", "运行中（外部）");
  } else if (kernel && kernel.state === "failed") {
    setBadge(ui.overviewKernelState, "error", "启动失败");
  } else {
    setBadge(ui.overviewKernelState, "off", "未运行");
  }
  if (ui.overviewKernelStart) ui.overviewKernelStart.disabled = kernelLive || Boolean(ours);
  if (ui.overviewKernelStop) ui.overviewKernelStop.disabled = !ours;

  const camera = operations.get("camera");
  const cameraBusy = camera && ["starting", "running", "stopping"].includes(camera.state);
  if (cameraBusy) {
    setBadge(ui.overviewCameraState, "on", camera.state === "stopping" ? "停止中" : "启动中");
  } else if (cameraLive) {
    setBadge(ui.overviewCameraState, "on", "已启动");
  } else if (camera && camera.state === "failed") {
    setBadge(ui.overviewCameraState, "error", "启动失败");
  } else {
    setBadge(ui.overviewCameraState, "off", "未启动");
  }
  if (ui.overviewCameraStart) ui.overviewCameraStart.disabled = cameraLive || Boolean(cameraBusy);
  if (ui.overviewCameraStop) ui.overviewCameraStop.disabled = !cameraBusy;
}

async function refreshOperations() {
  try {
    const list = await listOperations();
    operations.clear();
    for (const operation of list) operations.set(operation.kind, operation);
    renderOperations();
    renderWizard();
  } catch {
    // Backend unavailable: keep the last known state.
  }
}


// Teleop wizard

const QUEST_MS = 3000;

function renderStep(rootId, badgeId, state, badge, detail) {
  const root = ui[rootId];
  if (root) root.dataset.state = state;
  const badgeNode = ui[badgeId];
  if (badgeNode) {
    badgeNode.dataset.state = state;
    badgeNode.textContent = badge;
  }
  const detailNode = ui[`${rootId}Detail`];
  if (detailNode && detail !== undefined) detailNode.textContent = detail;
}

function setWizardMessage(text) {
  setText(ui.wizardMessage, text);
}

function renderWizard() {
  if (!ui.wizardCan) return;
  const data = lastStatus;
  let done = 0;

  const canOk = Boolean(data?.can?.up);
  if (canOk) done += 1;
  renderStep("wizardCan", "wizardCanBadge", canOk ? "done" : "todo",
    canOk ? "已建立" : "未建立", canOk ? "can0 UP" : "需要建立 can0");
  ui.wizardCanAction.hidden = canOk;

  const q = quest || {};
  const questOk = Boolean(q.connected);
  if (questOk) done += 1;
  let questBadge = "未连接";
  if (questOk) questBadge = "已连接";
  else if (q.usb && q.usb !== "ready") questBadge = `USB ${q.usb}`;
  else if (q.usb === "ready" && !q.reverse) questBadge = "未配置转发";
  else if (q.reverse && !q.client) questBadge = "等待 Connect";
  const questDetail = q.error
    ? String(q.error).slice(0, 46)
    : `USB ${q.usb || "?"} · 转发 ${q.reverse ? "已配置" : "未配置"} · 客户端 ${q.client ? "已连接" : "未连接"}`;
  renderStep("wizardQuest", "wizardQuestBadge", questOk ? "done" : (q.usb === "ready" ? "warn" : "todo"),
    questBadge, questDetail);
  ui.wizardQuestAction.hidden = questOk;

  const connected = Boolean(data?.connected);
  const vr = data?.teleop?.source === "quest";
  const kernelOk = connected && vr;
  if (kernelOk) done += 1;
  renderStep("wizardKernel", "wizardKernelBadge", kernelOk ? "done" : (connected ? "warn" : "todo"),
    kernelOk ? "运行中" : (connected ? "无 VR 输入" : "未运行"),
    connected ? `运行 ${data?.runtime_id || ""}`.trim() : "real-dual-quest · 先连接头显");
  ui.wizardKernelAction.hidden = connected;

  const teleop = data?.teleop || {};
  const teleopOk = Boolean(teleop.ready && teleop.armed && teleop.permit && data?.phase === "teleop");
  if (teleopOk) done += 1;
  let teleopBadge = "等待内核";
  let teleopState = "waiting";
  if (connected && teleopOk) { teleopBadge = "遥操中"; teleopState = "done"; }
  else if (connected && teleop.ready) { teleopBadge = "等待握持 Grip"; teleopState = "warn"; }
  else if (connected) { teleopBadge = teleop.reason ? String(teleop.reason).slice(0, 22) : "未就绪"; teleopState = "todo"; }
  renderStep("wizardTeleop", "wizardTeleopBadge", teleopState, teleopBadge,
    `ready ${teleop.ready ? "✓" : "—"} · armed ${teleop.armed ? "✓" : "—"} · permit ${teleop.permit ? "✓" : "—"}`);

  const cameraOk = Boolean(data?.shared_camera);
  if (cameraOk) done += 1;
  const sensor = operations.get("sensor");
  const sensorBusy = sensor && ["starting", "running", "stopping"].includes(sensor.state);
  renderStep("wizardCamera", "wizardCameraBadge", cameraOk ? "done" : "todo",
    cameraOk ? "已启动" : (sensorBusy ? "启动中" : "未启动"),
    cameraOk ? "采集与预览可用" : "采集前需要启动");
  ui.wizardCameraAction.hidden = cameraOk && !sensorBusy;
  ui.wizardCameraAction.textContent = sensorBusy ? "停止相机" : "启动相机";
  ui.wizardCameraAction.disabled = wizardBusy;

  setText(ui.wizardProgress, `${done} / 5`);
}

async function pollQuest() {
  if (questPending) return;
  questPending = true;
  try {
    quest = await questStatus();
  } catch {
    quest = null;
  } finally {
    questPending = false;
    renderWizard();
  }
}

async function manualAction(name, label) {
  if (wizardBusy) return;
  wizardBusy = true;
  setWizardMessage(`${label}…`);
  try {
    const result = await runAction(name);
    setWizardMessage(result.ok ? `${label}完成` : `${label}失败：${String(result.output || "").split("\n").slice(-2).join(" ")}`);
  } catch (error) {
    setWizardMessage(`${label}失败：${error instanceof Error ? error.message : error}`);
  } finally {
    wizardBusy = false;
    void poll();
    void pollQuest();
    renderWizard();
  }
}

if (ui.wizardCanAction) {
  ui.wizardCanAction.addEventListener("click", () => manualAction("can-up", "建立 CAN 接口"));
}
if (ui.wizardQuestAction) {
  ui.wizardQuestAction.addEventListener("click", () => manualAction("quest-prepare", "准备 ADB 转发"));
}
if (ui.wizardKernelAction) {
  ui.wizardKernelAction.addEventListener("click", () => {
    ui.overviewKernelProfile.value = "real-dual-quest";
    ui.overviewKernelStart.click();
  });
}
if (ui.wizardCameraAction) {
  ui.wizardCameraAction.addEventListener("click", async () => {
    if (wizardBusy) return;
    wizardBusy = true;
    const sensor = operations.get("sensor");
    const running = sensor && ["starting", "running", "stopping"].includes(sensor.state);
    setWizardMessage(running ? "正在停止相机服务…" : "正在启动相机服务…");
    try {
      if (running) await stopOperation("sensor");
      else await startOperation("sensor");
      setWizardMessage(running ? "相机服务已停止" : "相机服务已启动");
    } catch (error) {
      setWizardMessage(`相机服务操作失败：${error instanceof Error ? error.message : error}`);
    } finally {
      wizardBusy = false;
      await refreshOperations();
      void poll();
      renderWizard();
    }
  });
}

window.setInterval(pollQuest, QUEST_MS);
void pollQuest();

async function poll() {
  if (inFlight || document.hidden) return;
  inFlight = true;
  try {
    const response = await fetch(statusUrl, { cache: "no-store" });
    if (response.status === 401) {
      renderDisconnected("配对令牌无效 · 请带上页面的 ?token= 参数");
      return;
    }
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    if (!(response.headers.get("content-type") || "").includes("application/json")) {
      throw new Error("后端未运行（静态预览）");
    }
    const data = await response.json();
    lastStatus = data;
    renderCapabilities(data);
    if (!data.connected) {
      kernelLive = false;
      renderDisconnected(data.reason || "未连接内核");
      renderOperations();
      renderWizard();
      return;
    }
    renderConnected(data);
    renderOperations();
    renderWizard();
  } catch (error) {
    lastStatus = null;
    renderDisconnected(`未连接内核 · ${error instanceof Error ? error.message : error}`);
    renderWizard();
  } finally {
    inFlight = false;
  }
}

function setMessage(node, text, tone = "") {
  if (!node) return;
  node.textContent = text;
  node.dataset.tone = tone;
}

async function runKernel(confirm) {
  const profile = ui.overviewKernelProfile?.value || "sim-manual";
  setMessage(ui.overviewKernelMessage, confirm ? "正在启动真机内核…" : "正在启动内核…");
  if (ui.overviewKernelConfirm) ui.overviewKernelConfirm.hidden = true;
  try {
    await startOperation("kernel", confirm ? { profile, confirm: true } : { profile });
    setMessage(ui.overviewKernelMessage, "已提交启动", "success");
  } catch (error) {
    setMessage(ui.overviewKernelMessage, error instanceof Error ? error.message : String(error), "error");
  }
  await refreshOperations();
}

async function runStop(kind, node) {
  setMessage(node, "正在停止…");
  try {
    await stopOperation(kind);
    setMessage(node, "已停止", "success");
  } catch (error) {
    setMessage(node, error instanceof Error ? error.message : String(error), "error");
  }
  await refreshOperations();
}

function wireOperations() {
  if (ui.overviewKernelStart) {
    ui.overviewKernelStart.addEventListener("click", () => {
      const profile = ui.overviewKernelProfile?.value || "sim-manual";
      if (profile.startsWith("real")) {
        if (ui.overviewKernelConfirm) ui.overviewKernelConfirm.hidden = false;
        return;
      }
      void runKernel(false);
    });
  }
  if (ui.overviewKernelConfirmStart) {
    ui.overviewKernelConfirmStart.addEventListener("click", () => void runKernel(true));
  }
  if (ui.overviewKernelCancel) {
    ui.overviewKernelCancel.addEventListener("click", () => {
      if (ui.overviewKernelConfirm) ui.overviewKernelConfirm.hidden = true;
    });
  }
  if (ui.overviewKernelProfile) {
    ui.overviewKernelProfile.addEventListener("change", () => {
      if (ui.overviewKernelConfirm) ui.overviewKernelConfirm.hidden = true;
    });
  }
  if (ui.overviewKernelStop) {
    ui.overviewKernelStop.addEventListener("click", () => void runStop("kernel", ui.overviewKernelMessage));
  }
  if (ui.overviewCameraStart) {
    ui.overviewCameraStart.addEventListener("click", async () => {
      setMessage(ui.overviewCameraMessage, "正在启动相机服务…");
      try {
        await startOperation("camera");
        setMessage(ui.overviewCameraMessage, "已提交启动", "success");
      } catch (error) {
        setMessage(ui.overviewCameraMessage, error instanceof Error ? error.message : String(error), "error");
      }
      await refreshOperations();
    });
  }
  if (ui.overviewCameraStop) {
    ui.overviewCameraStop.addEventListener("click", () => void runStop("camera", ui.overviewCameraMessage));
  }
}

function start() {
  if (timer !== undefined) return;
  void poll();
  void refreshOperations();
  timer = window.setInterval(poll, POLL_MS);
  operationsTimer = window.setInterval(refreshOperations, OPERATIONS_MS);
}

function stop() {
  if (timer !== undefined) {
    window.clearInterval(timer);
    timer = undefined;
  }
  if (operationsTimer !== undefined) {
    window.clearInterval(operationsTimer);
    operationsTimer = undefined;
  }
}

document.addEventListener("visibilitychange", () => (document.hidden ? stop() : start()));
window.addEventListener("resize", drawTrend);
window.addEventListener("pagehide", stop);
wireOperations();
void initSetup();
start();
