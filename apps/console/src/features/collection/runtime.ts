// @ts-nocheck
import {
  controlEndpoint,
  decodeControlEvent,
  encodeCommand,
} from "../../api/control-protocol";

const ui = Object.fromEntries(
  [
    "taskText", "datasetId", "acceptedCount", "targetCount", "remainingCount",
    "rejectedCount", "successRate", "episodeProgress", "sessionLabel", "cameraSpec",
    "cameraStatus", "controlStatus", "armMode", "recordingBadge", "timer",
    "liveDuration", "stateSamples", "stateRate", "cameraFrames", "cameraRate",
    "dropRate", "cameraQueue", "readySummary", "startButton", "stopButton",
    "discardButton", "reviewPanel",
    "validationBadge", "validationSummary", "reviewDuration", "reviewSamples",
    "reviewSync", "reviewStill", "saveButton", "reviewDiscardButton",
    "copyValidationButton", "messageBar", "messageIcon", "message", "copyMessageButton", "manualLink", "formatValue",
    "dimensionsValue", "horizonValue", "rawRootValue", "jointRange", "healthStatus",
    "healthState", "healthCamera", "healthQueue", "healthDrop",
    "exteriorPreview", "wristPreview", "exteriorPreviewStatus", "wristPreviewStatus",
    "jointChart1", "jointChart2", "jointChart3", "jointChart4",
    "jointChart5", "jointChart6", "jointValue1", "jointValue2", "jointValue3",
    "jointValue4", "jointValue5", "jointValue6",
  ].map((id) => [id, document.getElementById(id)]),
);

let controlSocket;
let controlSequence = 0;
const controlRequests = new Map();
let armState = null;
let collectionState = null;
let cameraReady = false;
let cameraCollectionState = null;
let recordingStartedAt = null;
let timerHandle = null;
let statusPollHandle = null;
let reconnectHandle = null;
let pageClosing = false;
let busy = false;
let statePollPending = false;
let lastStatePollAt = 0;
let armPreparePending = false;
let lastArmPrepareAt = 0;
let lastTelemetrySampleAt = 0;
let lastTelemetryStateCount = -1;
let renderedValidationSignature = "";
const telemetryHistory = [];
const JOINT_COLORS = ["#78f0c7", "#72b7ff", "#f2d06b", "#ff8f88", "#c8a5ff", "#93e36f"];
const TELEMETRY_HISTORY_LIMIT = 240;
const CAMERA_PREVIEWS = [
  ["exterior_rgb", "exteriorPreview", "exteriorPreviewStatus"],
  ["wrist_rgb", "wristPreview", "wristPreviewStatus"],
];

function endpoint() {
  return controlEndpoint();
}

function withToken(href) {
  const token = new URLSearchParams(location.search).get("token");
  if (!token) return href;
  const url = new URL(href, location.href);
  url.searchParams.set("token", token);
  return url.toString();
}

function cameraPreviewPhase() {
  if (collectionState?.active) return "采集中";
  if (collectionState?.stage === "pending_review") return "采集后";
  return "采集前";
}

function updateCameraPreviewLabels() {
  const phase = cameraPreviewPhase();
  for (const [, , statusId] of CAMERA_PREVIEWS) {
    if (!ui[statusId].classList.contains("failed")) {
      ui[statusId].textContent = phase;
    }
  }
}

async function refreshCameraPreview(videoKey, imageId, statusId) {
  const now = Date.now();
  const image = ui[imageId];
  const preview = image.closest(".camera-preview");
  const startedAt = number(image.dataset.loadingStartedAt);
  if (image.dataset.loading === "true" && now - startedAt < 5_000) return;
  image.dataset.loading = "true";
  image.dataset.loadingStartedAt = String(now);
  if (!image.dataset.objectUrl) preview?.classList.add("loading");
  ui[statusId].classList.remove("failed");
  let objectUrl = "";
  try {
    const response = await fetch(
      withToken(`/api/v1/collection/camera-preview/${videoKey}.jpg?t=${now}`),
      { cache: "no-store" },
    );
    if (!response.ok) throw new Error(`preview HTTP ${response.status}`);
    objectUrl = URL.createObjectURL(await response.blob());
    const preloaded = new Image();
    await new Promise((resolve, reject) => {
      preloaded.addEventListener("load", resolve, { once: true });
      preloaded.addEventListener("error", reject, { once: true });
      preloaded.src = objectUrl;
    });
    const previousObjectUrl = image.dataset.objectUrl;
    image.dataset.objectUrl = objectUrl;
    image.src = objectUrl;
    if (previousObjectUrl) URL.revokeObjectURL(previousObjectUrl);
    image.dataset.loading = "false";
    preview?.classList.remove("loading", "failed");
    ui[statusId].classList.remove("failed");
    ui[statusId].textContent = cameraPreviewPhase();
  } catch {
    if (objectUrl) URL.revokeObjectURL(objectUrl);
    image.dataset.loading = "false";
    preview?.classList.remove("loading");
    preview?.classList.add("failed");
    ui[statusId].classList.add("failed");
    ui[statusId].textContent = "暂不可用";
  }
}

function refreshCameraPreviews() {
  for (const preview of CAMERA_PREVIEWS) void refreshCameraPreview(...preview);
}

function initializeCameraPreviews() {
  refreshCameraPreviews();
}

ui.manualLink.href = withToken("./");

function setMessage(text, error = false, requestedTone = null) {
  const tone = requestedTone ?? (error
    ? "error"
    : /将以 .*速度自动回零位|正在.*回零位|正在.*回位/.test(text)
      ? "warning"
      : /正在采集|示教中/.test(text)
        ? "recording"
        : /已保存|已恢复|准备就绪|可以摆放|可以开始/.test(text)
          ? "success"
          : "info");
  const signature = `${tone}\u0000${error ? "1" : "0"}\u0000${text}`;
  if (ui.messageBar.dataset.signature === signature) return;
  ui.messageBar.dataset.signature = signature;
  const icons = {
    error: "fa-solid fa-circle-exclamation",
    warning: "fa-solid fa-triangle-exclamation",
    recording: "fa-solid fa-circle",
    success: "fa-solid fa-circle-check",
    info: "fa-solid fa-circle-info",
  };
  ui.message.textContent = text;
  ui.message.title = text;
  ui.message.classList.toggle("error", error);
  ui.messageBar.className = `message-bar ${tone}`;
  ui.messageIcon.className = icons[tone];
  ui.copyMessageButton.hidden = !error || !text;
  ui.copyMessageButton.dataset.copyText = error ? text : "";
}

async function copyText(text, button) {
  if (!text) return;
  try {
    await navigator.clipboard.writeText(text);
  } catch {
    const input = document.createElement("textarea");
    input.value = text;
    input.setAttribute("readonly", "");
    input.style.position = "fixed";
    input.style.opacity = "0";
    document.body.append(input);
    input.select();
    const copied = document.execCommand("copy");
    input.remove();
    if (!copied) throw new Error("浏览器未允许复制");
  }
  const label = button.querySelector("span");
  if (!label) return;
  const original = label.textContent;
  label.textContent = "已复制";
  window.setTimeout(() => { label.textContent = original; }, 1_500);
}

function setStatus(element, text, state) {
  element.textContent = text;
  element.className = `${element.classList.contains("check") ? "check" : "status"} ${state}`;
}

function number(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function updateDatasetUi(next) {
  const accepted = number(next.counts?.accepted);
  const rejected = number(next.counts?.rejected);
  const target = number(next.target_accepted_episodes ?? next.counts?.target, 50);
  const attempted = accepted + rejected;
  ui.acceptedCount.textContent = String(accepted);
  ui.targetCount.textContent = String(target);
  ui.remainingCount.textContent = String(number(next.remaining_episodes, Math.max(0, target - accepted)));
  ui.rejectedCount.textContent = String(rejected);
  ui.successRate.textContent = attempted ? `${(100 * accepted / attempted).toFixed(0)}%` : "--";
  ui.episodeProgress.max = target;
  ui.episodeProgress.value = accepted;
  ui.taskText.textContent = next.task_display ?? ui.taskText.textContent;
  ui.datasetId.textContent = next.dataset?.id ?? "ARX5 GR00T dataset";
  ui.datasetId.title = next.dataset?.id ?? "";
  ui.formatValue.textContent = next.dataset?.format ?? "GR00T-flavored LeRobot v2.1";
  ui.dimensionsValue.textContent = `${next.dataset?.state_dimension ?? 16}D / ${next.dataset?.action_dimension ?? 16}D`;
  ui.horizonValue.textContent = String(next.recording_spec?.action_horizon ?? 16);
  ui.rawRootValue.textContent = next.dataset?.raw_root ?? "/home/r2/work/data/arx5/data-coll";
  ui.rawRootValue.title = next.dataset?.raw_root ?? "";
  const spec = next.recording_spec;
  if (spec) {
    const cameras = Array.isArray(spec.cameras) ? spec.cameras : [];
    const camera = cameras[0];
    ui.cameraSpec.textContent = camera
      ? `双 RGB · ${camera.width}×${camera.height}@${camera.fps} · RGB8 · 自动预热后锁定`
      : `双 RGB · ${spec.camera_fps} FPS`;
    ui.cameraSpec.title = camera
      ? "D455 + D405：预热约 1 秒，录制前分别锁定固定曝光、增益与白平衡"
      : "";
  }
}

function updateCollectionState(next) {
  if (!next) return;
  collectionState = next;
  updateCameraCollectionState(next.camera_state);
  updateDatasetUi(next);
  const accepted = number(next.counts?.accepted);
  const target = number(next.target_accepted_episodes, 50);
  ui.sessionLabel.textContent = next.session_id
    ?? (accepted >= target ? "采集目标已完成" : "等待下一条 episode");
  const review = next.stage === "pending_review";
  ui.reviewPanel.hidden = !review;
  ui.reviewPanel.closest(".control-panel")?.classList.toggle("review-mode", review);
  if (review) {
    const signature = JSON.stringify(next.validation ?? null);
    if (signature !== renderedValidationSignature) {
      renderedValidationSignature = signature;
      renderValidation(next.validation);
    }
  } else {
    renderedValidationSignature = "";
  }
  if (next.active) {
    beginTimer(number(next.state_summary?.duration_s));
  } else {
    endTimer();
  }
  updateLiveMetrics();
  updateCameraPreviewLabels();
  updateButtons();
}

function updateArmState(next) {
  if (!next) return;
  armState = next;
  updateCollectionState(next.collection);
  const modeNames = {
    disconnected: "机械臂未连接",
    damping: "阻尼示教模式",
    active: "关节位置模式",
    fault: "故障阻尼",
  };
  ui.armMode.textContent = modeNames[next.mode] ?? next.mode;
  setStatus(
    ui.controlStatus,
    next.connected ? "控制服务正常" : "控制服务已连接",
    next.connected ? "good" : "waiting",
  );
  updateButtons();
}

function readiness() {
  const recording = collectionState?.active === true;
  const review = collectionState?.stage === "pending_review";
  if (recording) return {
    ready: false,
    state: "recording",
    text: "正在采集 · 完成任务后点确认保存；操作失误点丢弃并回零位",
  };
  if (review) return {
    ready: false,
    state: collectionState?.validation?.ok ? "good" : "bad",
    text: collectionState?.validation?.ok ? "质量检查通过 · 确认任务实际成功后保存" : "质量检查未通过 · 请丢弃并重新采集",
  };
  if (number(collectionState?.remaining_episodes, 1) <= 0) {
    return { ready: false, state: "good", text: "采集目标已完成，可导出数据集" };
  }
  if (!armState?.connected) return {
    ready: false,
    state: "waiting",
    text: "后台正在连接机械臂并进入阻尼示教模式",
  };
  if (armState.mode !== "damping") return { ready: false, state: "waiting", text: "后台正在切换阻尼示教模式" };
  if (!cameraReady) return { ready: false, state: "waiting", text: "主机正在准备 D455 + D405" };
  return { ready: true, state: "good", text: "可以开始 · 摆好黑色和白色方块后按开始采集" };
}

function updateButtons() {
  const recording = collectionState?.active === true;
  const review = collectionState?.stage === "pending_review";
  const ready = readiness();
  const panel = ui.reviewPanel.closest(".control-panel");
  panel?.classList.toggle("recording-mode", recording);
  panel?.classList.toggle("review-mode", review);
  ui.startButton.hidden = recording || review;
  ui.stopButton.hidden = !recording;
  ui.discardButton.hidden = !recording;
  ui.readySummary.className = `ready-summary ${ready.state}`;
  ui.readySummary.querySelector("i").className = ready.state === "good"
    ? "fa-solid fa-circle-check"
    : ready.state === "recording"
      ? "fa-solid fa-circle"
      : ready.state === "bad" ? "fa-solid fa-circle-xmark" : "fa-solid fa-circle-notch fa-spin";
  ui.readySummary.querySelector("span").textContent = ready.text;
  ui.startButton.disabled = !ready.ready || busy || armPreparePending;
  ui.stopButton.disabled = !recording || busy;
  ui.discardButton.disabled = busy || (!recording && !review);
  ui.saveButton.disabled = busy || !review || !collectionState?.validation?.ok;
  ui.reviewDiscardButton.disabled = busy || !review;
}

function formatMetric(value, decimals, unit) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? `${parsed.toFixed(decimals)} ${unit}` : "--";
}

function validationIssueDetails(issue, warning = false) {
  const text = String(issue);
  if (text.includes("D455/D405 p99 alignment")) return {
    title: "双相机时间差偏大，导出时自动对齐",
    detail: text,
    action: "系统会按主机时间戳为每个 D455 帧匹配最近的 D405 帧，不阻止保存。",
  };
  if (text.includes("maximum video gap")) return {
    title: "相机出现瞬时帧间隔，保留有效帧",
    detail: text,
    action: "视频和逐帧时间戳均已保留，导出时继续按真实时间对齐，不阻止保存。",
  };
  if (text.includes("camera queue drop fraction")) return {
    title: "写入队列出现丢帧",
    detail: text,
    action: "系统会跳过缺失帧并保持时间戳连续关系；可保存，后续关注 USB 或磁盘负载。",
  };
  if (text.includes("repeated source frames")) return {
    title: "重复源帧已自动剔除",
    detail: text,
    action: "重复画面不会进入导出视频；系统继续按剩余真实帧计算有效帧率。",
  };
  if (text.includes("source camera skipped")) return {
    title: "相机源帧序号存在跳跃",
    detail: text,
    action: "系统已保留真实时间戳并在导出时重采样；当前数据仍可保存。",
  };
  if (text.includes("source frame sequence reset")) return {
    title: "相机源帧序列中途重置",
    detail: text,
    action: "相机采集链路发生重启，无法保证连续性，请丢弃当前数据。",
  };
  if (text.includes("state alignment")) return {
    title: "图像与机械臂状态时间差偏大",
    detail: text,
    action: "导出时会在图像时间戳处插值机械臂状态，不阻止保存。",
  };
  if (text.includes("still window")) return {
    title: "首尾静止时间偏短",
    detail: text,
    action: "这只是下一条采集的操作建议，不影响当前数据保存。",
  };
  if (/video is missing|recording is not complete|recorder failed|camera stop failed/.test(text)) return {
    title: "相机视频没有完整写入",
    detail: text,
    action: "当前数据缺少训练必需图像，请丢弃；系统会继续检查相机服务和连接。",
  };
  if (/effective(?: source)? frame rate/.test(text)) return {
    title: "相机有效帧率低于训练最低要求",
    detail: text,
    action: "当前图像密度不足，无法可靠训练；请丢弃，系统会继续显示具体相机和实时帧率。",
  };
  if (/state stream unreadable|state recorder failed|fewer than two valid state/.test(text)) return {
    title: "机械臂状态数据不完整",
    detail: text,
    action: "当前数据缺少训练必需状态，请丢弃并检查 CAN 连接。",
  };
  if (/timestamp.*not strictly increasing|invalid host timestamp/.test(text)) return {
    title: "时间戳异常",
    detail: text,
    action: "无法可靠对齐状态和图像，请丢弃当前数据并重启采集服务。",
  };
  if (text.includes("overlap duration")) return {
    title: "双相机与机械臂的共同有效时长不足",
    detail: text,
    action: "当前数据无法形成完整训练片段，请丢弃后重新采集。",
  };
  if (text.includes("gripper position exceeds")) return {
    title: "夹爪反馈超出可接受范围",
    detail: text,
    action: "反馈已超过校准容差，请丢弃并检查夹爪零点或反馈通道。",
  };
  if (text.includes("joint position exceeds")) return {
    title: "关节反馈超出控制器记录范围",
    detail: text,
    action: "请丢弃当前数据并检查控制器限位元数据。",
  };
  return {
    title: warning ? "质量提示" : "数据完整性异常",
    detail: text,
    action: warning ? "当前数据仍可保存；系统会在导出时按时间戳处理。" : "请复制详情后丢弃当前数据。",
  };
}

function appendValidationIssues(container, issues, warning = false) {
  if (!issues.length) return;
  const list = document.createElement("ul");
  list.className = `validation-list ${warning ? "warning" : "bad"}`;
  for (const issue of issues) {
    const explained = validationIssueDetails(issue, warning);
    const item = document.createElement("li");
    const title = document.createElement("strong");
    const detail = document.createElement("span");
    const action = document.createElement("small");
    title.textContent = explained.title;
    detail.textContent = explained.detail;
    action.textContent = explained.action;
    item.append(title, detail, action);
    list.append(item);
  }
  container.append(list);
}

function renderValidation(report) {
  ui.validationSummary.replaceChildren();
  ui.copyValidationButton.hidden = true;
  ui.copyValidationButton.dataset.copyText = "";
  ui.copyValidationButton.querySelector("span").textContent = "复制原因";
  if (!report) {
    setStatus(ui.validationBadge, "检查中", "waiting");
    const pending = document.createElement("span");
    pending.className = "validation-bad";
    pending.textContent = "等待完整性检查结果";
    ui.validationSummary.append(pending);
    return;
  }
  setStatus(ui.validationBadge, report.ok ? "自动检查通过" : "检查未通过", report.ok ? "good" : "bad");
  const errors = Array.isArray(report.errors) ? report.errors.map(String) : [];
  const warnings = Array.isArray(report.warnings) ? report.warnings.map(String) : [];
  if (errors.length || warnings.length) {
    ui.copyValidationButton.hidden = false;
    ui.copyValidationButton.querySelector("span").textContent = errors.length ? "复制原因" : "复制提示";
    ui.copyValidationButton.dataset.copyText = [
      `Episode: ${collectionState?.session_id ?? "unknown"}`,
      ...errors.map((error) => `失败：${error}`),
      ...warnings.map((warning) => `提示：${warning}`),
    ].join("\n");
  }
  const metrics = report.metrics ?? {};
  ui.reviewDuration.textContent = formatMetric(metrics.overlap_duration_s, 1, "s");
  const cameraFrames = metrics.camera_frames ?? {};
  ui.reviewSamples.textContent = `${number(metrics.state_samples)} / ${number(cameraFrames.exterior_rgb)} / ${number(cameraFrames.wrist_rgb)}`;
  ui.reviewSync.textContent = `${number(metrics.p99_state_alignment_ms).toFixed(2)} / ${number(metrics.p99_camera_alignment_ms).toFixed(2)} ms`;
  ui.reviewStill.textContent = `${number(metrics.initial_still_s).toFixed(2)} / ${number(metrics.final_still_s).toFixed(2)} s`;
  if (report.ok) {
    const summary = document.createElement("span");
    const hasStillnessAdvice = warnings.some((warning) => warning.includes("still window"));
    summary.className = warnings.length ? "validation-warning" : "validation-good";
    summary.textContent = hasStillnessAdvice
      ? "数据完整，可保存；首尾静止较短仅作操作提示。"
      : warnings.length
        ? "数据完整，可保存；下列质量项将自动后处理。"
      : "GR00T 原始数据完整；请根据真实场景确认任务成功。";
    ui.validationSummary.append(summary);
    appendValidationIssues(ui.validationSummary, warnings, true);
  } else {
    const summary = document.createElement("strong");
    summary.className = "validation-bad validation-failure-heading";
    summary.textContent = `发现 ${errors.length} 项无法自动修复的问题：`;
    ui.validationSummary.append(summary);
    appendValidationIssues(ui.validationSummary, errors);
  }
}

function updateCameraCollectionState(next) {
  if (!next) return;
  cameraCollectionState = next;
  cameraReady = next.ok !== false && next.ready !== false;
  if (cameraReady) {
    setStatus(
      ui.cameraStatus,
      next.active ? "D455 + D405 正在录制" : "D455 + D405 主机就绪",
      "good",
    );
  } else {
    setStatus(ui.cameraStatus, "D455 + D405 主机异常", "bad");
  }
  updateLiveMetrics();
}

function clamp(value, minimum = 0, maximum = 1) {
  return Math.min(maximum, Math.max(minimum, value));
}

function currentJointDegrees(arm) {
  const radians = arm?.latest_joint_position_rad;
  if (Array.isArray(radians) && radians.length === 6) {
    return radians.map((value) => number(value) * 180 / Math.PI);
  }
  const degrees = armState?.joint_position_deg;
  return Array.isArray(degrees) && degrees.length === 6
    ? degrees.map((value) => number(value))
    : null;
}

function appendTelemetry(sample) {
  if (!collectionState?.active || !sample.joints) return;
  const now = performance.now();
  if (now - lastTelemetrySampleAt < 200 || sample.stateCount === lastTelemetryStateCount) return;
  lastTelemetrySampleAt = now;
  lastTelemetryStateCount = sample.stateCount;
  telemetryHistory.push(sample);
  if (telemetryHistory.length > TELEMETRY_HISTORY_LIMIT) telemetryHistory.shift();
}

function prepareCanvas(canvas) {
  const width = Math.max(1, canvas.clientWidth);
  const height = Math.max(1, canvas.clientHeight);
  const scale = Math.min(2, window.devicePixelRatio || 1);
  const pixelWidth = Math.round(width * scale);
  const pixelHeight = Math.round(height * scale);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const context = canvas.getContext("2d");
  context.setTransform(scale, 0, 0, scale, 0, 0);
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#151918";
  context.fillRect(0, 0, width, height);
  context.font = "12px Inter, system-ui, sans-serif";
  return { context, width, height };
}

function drawJointChart(canvas, jointIndex) {
  const { context, width, height } = prepareCanvas(canvas);
  const plot = { left: 8, right: width - 8, top: 8, bottom: height - 17 };
  const values = telemetryHistory.map((sample) => sample.joints[jointIndex]);
  let minimum = values.length ? Math.min(...values) : -30;
  let maximum = values.length ? Math.max(...values) : 30;
  const center = (minimum + maximum) / 2;
  const span = Math.max(20, maximum - minimum);
  minimum = center - span * 0.65;
  maximum = center + span * 0.65;

  context.strokeStyle = "rgba(255, 255, 255, .08)";
  context.fillStyle = "rgba(238, 244, 241, .52)";
  context.lineWidth = 1;
  for (let index = 0; index <= 2; index += 1) {
    const y = plot.top + (plot.bottom - plot.top) * index / 2;
    context.beginPath();
    context.moveTo(plot.left, y + 0.5);
    context.lineTo(plot.right, y + 0.5);
    context.stroke();
  }
  context.fillText("-60s", plot.left, height - 4);
  context.textAlign = "right";
  context.fillText("现在", plot.right, height - 4);
  context.textAlign = "left";

  if (telemetryHistory.length > 1) {
    const latestTime = telemetryHistory.at(-1).elapsed;
    const windowStart = Math.max(0, latestTime - 60);
    const windowSpan = Math.max(1, latestTime - windowStart);
    context.strokeStyle = JOINT_COLORS[jointIndex];
    context.lineWidth = 2.25;
    context.lineJoin = "round";
    context.lineCap = "round";
    context.beginPath();
    telemetryHistory.forEach((sample, sampleIndex) => {
      const x = plot.left + clamp((sample.elapsed - windowStart) / windowSpan) * (plot.right - plot.left);
      const y = plot.bottom - clamp((sample.joints[jointIndex] - minimum) / (maximum - minimum))
        * (plot.bottom - plot.top);
      if (sampleIndex === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();
  } else {
    context.fillStyle = "rgba(238, 244, 241, .38)";
    context.textAlign = "center";
    context.fillText("等待轨迹", (plot.left + plot.right) / 2, (plot.top + plot.bottom) / 2 + 3);
    context.textAlign = "left";
  }
}

function drawJointCharts() {
  const current = telemetryHistory.at(-1)?.joints;
  ui.jointRange.textContent = current ? "实时更新" : "等待采集";
  JOINT_COLORS.forEach((color, index) => {
    drawJointChart(ui[`jointChart${index + 1}`], index);
    const value = ui[`jointValue${index + 1}`];
    value.textContent = current ? `${current[index].toFixed(1)}°` : "--";
    value.style.color = color;
  });
}

function healthScores(sample) {
  const spec = collectionState?.recording_spec ?? {};
  const targetState = number(spec.arm_rate_hz, 100);
  const targetCamera = number(spec.camera_fps, 30);
  const maximumDrop = number(spec.maximum_frame_drop_fraction, 0.01);
  return [
    clamp(1 - 2 * Math.abs(sample.stateRate / targetState - 1)),
    clamp(1 - 2 * Math.abs(sample.cameraRate / targetCamera - 1)),
    clamp(1 - sample.queueFraction),
    clamp(1 - Math.max(
      sample.dropFraction / Math.max(maximumDrop, 0.0001),
      sample.duplicateSourceFraction / 0.05,
    )),
  ];
}

function healthLevel(score) {
  if (!Number.isFinite(score)) return "waiting";
  if (score >= 0.88) return "good";
  if (score >= 0.68) return "warn";
  return "bad";
}

function setHealthCard(card, value, score) {
  card.className = `health-card ${healthLevel(score)}`;
  card.querySelector("strong").textContent = value;
}

function renderHealthCards(sample) {
  if (!collectionState?.active || !sample) {
    setHealthCard(ui.healthState, "-- Hz", Number.NaN);
    setHealthCard(ui.healthCamera, "-- / -- FPS", Number.NaN);
    setHealthCard(ui.healthQueue, "0 / 0", Number.NaN);
    setHealthCard(ui.healthDrop, "0.00% · 重复 0", Number.NaN);
    ui.healthStatus.textContent = "等待采集";
    return;
  }
  const scores = healthScores(sample);
  if (!sample.cameraOk) scores[1] = 0;
  setHealthCard(ui.healthState, `${sample.stateRate.toFixed(1)} Hz`, scores[0]);
  setHealthCard(
    ui.healthCamera,
    `${sample.exteriorRate.toFixed(1)} / ${sample.wristRate.toFixed(1)} FPS`,
    scores[1],
  );
  setHealthCard(ui.healthQueue, sample.queueText, scores[2]);
  setHealthCard(
    ui.healthDrop,
    `${(100 * sample.dropFraction).toFixed(2)}% · 重复 ${sample.duplicateSourceFrames}`,
    scores[3],
  );
  const recorderFailed = Boolean(sample.armError || sample.exteriorError || sample.wristError);
  const worst = Math.min(...scores);
  ui.healthStatus.textContent = recorderFailed || !sample.cameraOk
    ? "录制异常"
    : sample.belowUsableRate && sample.elapsed >= 3
      ? "帧率过低，可能不可用"
    : worst >= 0.88 ? "质量正常" : "质量提醒（可保存）";
}

function updateLiveGuidance(sample) {
  if (!collectionState?.active || !sample) return;
  const recorderError = sample.armError || sample.exteriorError || sample.wristError;
  if (recorderError || !sample.cameraOk) {
    setMessage(`录制通道异常：${recorderError || cameraCollectionState?.error || "双相机状态异常"}。当前 episode 将无法保存，请直接丢弃。`, true);
    return;
  }
  if (sample.elapsed < 3) return;
  const targetCamera = number(collectionState?.recording_spec?.camera_fps, 30);
  const minimumUsableCameraRate = targetCamera * number(
    collectionState?.recording_spec?.minimum_usable_camera_rate_fraction,
    0.5,
  );
  if (sample.cameraRate < minimumUsableCameraRate) {
    setMessage(`双相机最低有效写入仅 ${sample.cameraRate.toFixed(1)} FPS，低于训练最低要求 ${minimumUsableCameraRate.toFixed(1)} FPS。若持续到结束，本条将无法保存；D455 ${sample.exteriorRate.toFixed(1)}、D405 ${sample.wristRate.toFixed(1)} FPS，重复源帧已自动剔除 ${sample.duplicateSourceFrames} 张。`, true);
    return;
  }
  const maximumDrop = number(collectionState?.recording_spec?.maximum_frame_drop_fraction, 0.01);
  if (sample.queueFraction >= 0.7) {
    setMessage(`写入队列接近满载：D455 ${sample.exteriorQueueText}，D405 ${sample.wristQueueText}。系统会保留有效帧并记录准确时间戳，本条可继续，无需人工中断。`, false, "warning");
    return;
  }
  if (sample.dropFraction > maximumDrop) {
    setMessage(`采集中检测到 ${(100 * sample.dropFraction).toFixed(2)}% 队列丢帧，超过建议值 ${(100 * maximumDrop).toFixed(2)}%。当前数据仍可保存，导出时会自动跳过缺帧并按时间戳对齐。`, false, "warning");
    return;
  }
  if (sample.duplicateSourceFrames > 0) {
    setMessage(`采集中发现 ${sample.duplicateSourceFrames} 张重复源帧，系统已在写入前自动剔除，不会混入训练视频；当前有效帧率仍满足要求。`, false, "warning");
    return;
  }
  const maximumGap = number(collectionState?.recording_spec?.maximum_video_gap_ms, 67);
  if (sample.maximumGapMs > maximumGap) {
    setMessage(`采集中已检测到最大帧间隔 ${sample.maximumGapMs.toFixed(1)} ms，超过建议值 ${maximumGap.toFixed(1)} ms。原始帧和时间戳已保留，导出时会自动对齐；无需中断本条采集。`, false, "warning");
    return;
  }
  const maximumCameraAlignment = number(collectionState?.recording_spec?.maximum_camera_alignment_p99_ms, 20);
  if (sample.latestCameraSkewMs > maximumCameraAlignment) {
    setMessage(`D455 与 D405 当前帧时间差约 ${sample.latestCameraSkewMs.toFixed(1)} ms。系统会在导出时按最近时间戳自动配对，当前数据仍可保存。`, false, "warning");
    return;
  }
  if (sample.cameraRate < targetCamera * 0.75) {
    setMessage(`双相机实际写入 ${sample.exteriorRate.toFixed(1)} / ${sample.wristRate.toFixed(1)} FPS，低于目标 ${targetCamera} FPS。系统已记录该质量项并保留原始时间戳，本条可继续。`, false, "warning");
    return;
  }
  setMessage("正在采集 · 状态、双相机和写入队列正常；完成任务后点确认保存。");
}

function drawTelemetry() {
  drawJointCharts();
}

function updateLiveMetrics() {
  const arm = collectionState?.state_summary ?? {};
  const cameras = cameraCollectionState?.cameras ?? {};
  const exterior = cameras.exterior_rgb ?? cameraCollectionState?.summary ?? {};
  const wrist = cameras.wrist_rgb ?? {};
  const armDuration = number(arm.duration_s);
  const exteriorDuration = number(exterior.duration_s);
  const wristDuration = number(wrist.duration_s);
  const cameraDuration = Math.max(exteriorDuration, wristDuration);
  const elapsed = recordingStartedAt === null
    ? Math.max(armDuration, cameraDuration)
    : Math.max(0, (performance.now() - recordingStartedAt) / 1000);
  const stateCount = number(arm.sample_count);
  const exteriorFrames = number(exterior.frame_count);
  const wristFrames = number(wrist.frame_count);
  const stateRate = armDuration > 0 ? stateCount / armDuration : 0;
  const exteriorRate = number(
    exterior.recording_effective_fps ?? exterior.source_effective_fps,
    exteriorDuration > 0 ? exteriorFrames / exteriorDuration : 0,
  );
  const wristRate = number(
    wrist.recording_effective_fps ?? wrist.source_effective_fps,
    wristDuration > 0 ? wristFrames / wristDuration : 0,
  );
  const cameraRate = Math.min(exteriorRate, wristRate);
  const exteriorQueueCapacity = number(exterior.queue_capacity, 90);
  const wristQueueCapacity = number(wrist.queue_capacity, 90);
  const exteriorQueueDepth = number(exterior.queue_depth);
  const wristQueueDepth = number(wrist.queue_depth);
  const queueFraction = Math.max(
    exteriorQueueDepth / Math.max(1, exteriorQueueCapacity),
    wristQueueDepth / Math.max(1, wristQueueCapacity),
  );
  const dropFraction = Math.max(
    number(exterior.drop_fraction),
    number(wrist.drop_fraction),
  );
  const exteriorDuplicateSourceFrames = number(exterior.duplicate_source_frames);
  const wristDuplicateSourceFrames = number(wrist.duplicate_source_frames);
  const duplicateSourceFrames = exteriorDuplicateSourceFrames + wristDuplicateSourceFrames;
  const duplicateSourceFraction = Math.max(
    number(exterior.duplicate_source_fraction),
    number(wrist.duplicate_source_fraction),
  );
  const maximumGapMs = Math.max(
    number(exterior.maximum_frame_gap_ms),
    number(wrist.maximum_frame_gap_ms),
  );
  const exteriorLastNs = number(exterior.last_host_monotonic_ns, Number.NaN);
  const wristLastNs = number(wrist.last_host_monotonic_ns, Number.NaN);
  const latestCameraSkewMs = Number.isFinite(exteriorLastNs) && Number.isFinite(wristLastNs)
    ? Math.abs(exteriorLastNs - wristLastNs) / 1_000_000
    : 0;
  ui.liveDuration.textContent = `${elapsed.toFixed(1)} s`;
  ui.stateSamples.textContent = String(stateCount);
  ui.stateRate.textContent = armDuration > 0 ? `${stateRate.toFixed(1)} Hz` : "-- Hz";
  ui.cameraFrames.textContent = `${exteriorFrames} / ${wristFrames}`;
  ui.cameraRate.textContent = cameraDuration > 0
    ? `${exteriorRate.toFixed(1)} / ${wristRate.toFixed(1)} FPS`
    : "-- / -- FPS";
  ui.dropRate.textContent = `${(100 * dropFraction).toFixed(2)}% · 重复 ${duplicateSourceFrames}`;
  ui.cameraQueue.textContent = `队列 ${exteriorQueueDepth}/${exteriorQueueCapacity} · ${wristQueueDepth}/${wristQueueCapacity}`;
  const sample = {
    elapsed,
    stateCount,
    stateRate,
    cameraRate,
    exteriorRate,
    wristRate,
    queueFraction,
    queueText: `${exteriorQueueDepth}/${exteriorQueueCapacity} · ${wristQueueDepth}/${wristQueueCapacity}`,
    dropFraction,
    duplicateSourceFrames,
    duplicateSourceFraction,
    cameraOk: cameraCollectionState?.ok !== false,
    armError: arm.error,
    exteriorError: exterior.error,
    wristError: wrist.error,
    exteriorQueueText: `${exteriorQueueDepth}/${exteriorQueueCapacity}`,
    wristQueueText: `${wristQueueDepth}/${wristQueueCapacity}`,
    maximumGapMs,
    latestCameraSkewMs,
    belowUsableRate: cameraRate < number(collectionState?.recording_spec?.camera_fps, 30) * number(
      collectionState?.recording_spec?.minimum_usable_camera_rate_fraction,
      0.5,
    ),
    joints: currentJointDegrees(arm),
  };
  appendTelemetry(sample);
  renderHealthCards(sample);
  updateLiveGuidance(sample);
  drawTelemetry();
}

function resetLiveMetrics() {
  cameraCollectionState = null;
  telemetryHistory.length = 0;
  lastTelemetrySampleAt = 0;
  lastTelemetryStateCount = -1;
  ui.liveDuration.textContent = "0.0 s";
  ui.stateSamples.textContent = "0";
  ui.stateRate.textContent = "-- Hz";
  ui.cameraFrames.textContent = "0 / 0";
  ui.cameraRate.textContent = "-- / -- FPS";
  ui.dropRate.textContent = "0.00% · 重复 0";
  ui.cameraQueue.textContent = "队列 0/90 · 0/90";
  renderHealthCards(null);
  drawTelemetry();
}

function controlCommand(action, params = {}, timeoutMs = 20_000) {
  if (!controlSocket || controlSocket.readyState !== WebSocket.OPEN) {
    return Promise.reject(new Error("控制服务未连接"));
  }
  const id = `groot-${++controlSequence}`;
  return new Promise((resolve, reject) => {
    const timer = window.setTimeout(() => {
      controlRequests.delete(id);
      reject(new Error(`${action} 超时`));
    }, timeoutMs);
    controlRequests.set(id, { resolve, reject, timer });
    controlSocket.send(encodeCommand(action, params, id));
  });
}

async function connectControl() {
  const socket = new WebSocket(endpoint());
  controlSocket = socket;
  socket.addEventListener("open", () => {
    setStatus(ui.controlStatus, "控制服务已连接", "waiting");
  });
  socket.addEventListener("message", (event) => {
    let message;
    try { message = decodeControlEvent(event.data); } catch { return; }
    if (message.state) updateArmState(message.state);
    if (message.type === "hello") updateArmState(message.state);
    if (message.type === "fatal") setMessage(message.message, true);
    const pending = controlRequests.get(String(message.id));
    if (!pending || !["command_result", "state"].includes(message.type)) return;
    controlRequests.delete(String(message.id));
    window.clearTimeout(pending.timer);
    if (message.ok) pending.resolve(message.result ?? message.state);
    else pending.reject(new Error(message.error ?? "控制命令失败"));
  });
  socket.addEventListener("close", () => {
    if (controlSocket === socket) controlSocket = null;
    endTimer();
    setStatus(ui.controlStatus, "控制服务已断开", "bad");
    armState = null;
    updateButtons();
    for (const request of controlRequests.values()) {
      window.clearTimeout(request.timer);
      request.reject(new Error("控制服务连接已断开"));
    }
    controlRequests.clear();
    scheduleControlReconnect();
  });
  await waitSocketOpen(socket, "控制服务");
}

function scheduleControlReconnect() {
  if (pageClosing || reconnectHandle !== null) return;
  reconnectHandle = window.setTimeout(async () => {
    reconnectHandle = null;
    try {
      await connectControl();
      await prepareArm();
      const status = await controlCommand("collection_status", {}, 20_000);
      updateCollectionState(status);
      setMessage(status.active
        ? "手机连接已恢复，后台采集一直保持运行。"
        : "手机连接已恢复，机械臂和双相机状态已同步。");
    } catch {
      scheduleControlReconnect();
    }
  }, 1_500);
}

function waitSocketOpen(socket, label) {
  return new Promise((resolve, reject) => {
    const timer = window.setTimeout(() => reject(new Error(`${label}连接超时`)), 6_000);
    socket.addEventListener("open", () => { window.clearTimeout(timer); resolve(); }, { once: true });
    socket.addEventListener("error", () => { window.clearTimeout(timer); reject(new Error(`${label}连接失败`)); }, { once: true });
  });
}

function maximumEpisodeMs() {
  const configured = number(collectionState?.recording_spec?.maximum_episode_seconds, 60);
  return Math.max(1_000, (configured - 0.75) * 1_000);
}

function beginTimer(initialSeconds = 0) {
  if (timerHandle !== null) return;
  recordingStartedAt = performance.now() - initialSeconds * 1_000;
  ui.recordingBadge.hidden = false;
  timerHandle = window.setInterval(() => {
    const elapsed = performance.now() - recordingStartedAt;
    const seconds = elapsed / 1_000;
    ui.timer.textContent = `${String(Math.floor(seconds / 60)).padStart(2, "0")}:${(seconds % 60).toFixed(1).padStart(4, "0")}`;
    updateLiveMetrics();
    if (elapsed >= maximumEpisodeMs() && collectionState?.active && !busy) {
      setMessage("已接近时长上限，正在自动停止并检查。", true);
      stopEpisode(false);
    }
  }, 100);
}

function endTimer() {
  if (timerHandle !== null) window.clearInterval(timerHandle);
  timerHandle = null;
  recordingStartedAt = null;
  ui.recordingBadge.hidden = true;
}

async function guarded(task) {
  if (busy) return;
  busy = true;
  updateButtons();
  try {
    return await task();
  } catch (error) {
    setMessage(error.message ?? String(error), true);
  } finally {
    busy = false;
    updateButtons();
  }
}

async function startEpisode() {
  return guarded(async () => {
    resetLiveMetrics();
    setMessage("正在创建原始 episode 并启动状态与图像记录…");
    const result = await controlCommand("collection_start");
    updateCollectionState(result);
    refreshCameraPreviews();
    setMessage("正在采集：开头保持约 0.5 秒静止；任务成功后点“确认保存”，失误点“丢弃并回零位”。");
  });
}

async function stopEpisodeTask() {
  const sessionId = collectionState?.session_id;
  if (!sessionId) throw new Error("缺少当前 session_id");
  setMessage("主机正在停止状态与双相机记录，并执行 GR00T 完整性检查…");
  const result = await controlCommand("collection_stop", {}, 25_000);
  endTimer();
  updateCollectionState(result);
  refreshCameraPreviews();
  const cameraError = result.camera_state?.ok === false
    ? new Error(result.camera_state.error ?? "双相机停止异常")
    : null;
  if (cameraError) {
    setMessage(`相机停止异常（${cameraError.message}）；请丢弃当前 episode。`, true);
  }
  return { result, cameraError };
}

async function stopEpisode(saveAfterValidation = false, forcedMessage = null) {
  return guarded(async () => {
    const { result, cameraError } = await stopEpisodeTask();
    if (saveAfterValidation && !cameraError && result.validation?.ok) {
      await acceptAndReturn(result.session_id);
      return;
    }
    if (saveAfterValidation) {
      const errors = Array.isArray(result.validation?.errors)
        ? result.validation.errors.map(String)
        : [];
      const reason = cameraError
        ? `camera stop failed: ${cameraError.message}`
        : `automatic integrity check failed: ${errors.join("; ") || "unknown error"}`;
      const rejected = await rejectStoppedEpisode(result.session_id, reason);
      try {
        await returnToReferenceAfterDecision("当前数据已自动归档到 discarded");
      } catch (error) {
        setMessage(`当前数据已自动丢弃，但自动回位未完成：${error.message}`, true);
        return;
      }
      updateCollectionState(rejected);
      setMessage("自动检查未通过，当前数据已丢弃并回到零位；可以摆放下一组。", true);
      return;
    }
    if (forcedMessage) {
      setMessage(forcedMessage, true);
    } else if (saveAfterValidation && !result.validation?.ok) {
      setMessage("自动检查未通过，未保存；请查看原因后丢弃。", true);
    } else if (!cameraError) {
      setMessage(result.validation?.ok
        ? "自动检查通过。确认真实任务成功后保存，否则丢弃。"
        : "自动检查未通过，请查看原因并丢弃。", !result.validation?.ok);
    }
  });
}

async function returnToReferenceAfterDecision(decisionMessage) {
  if (!armState?.connected) {
    throw new Error(`${decisionMessage}；机械臂未连接，无法自动回零位`);
  }
  if (!Array.isArray(armState.reference_joint_position_deg)) {
    throw new Error(`${decisionMessage}；当前没有参考姿态，无法自动回位`);
  }
  setMessage(`${decisionMessage}。机械臂将以 75% 速度自动回零位，请保持工作区无人和无遮挡物…`);
  await new Promise((resolve) => window.setTimeout(resolve, 1_000));
  if (armState.mode === "damping") {
    await controlCommand("activate", {}, 20_000);
  }
  const motionParams = { speed_percent: 75 };
  await controlCommand("preview_park", motionParams, 30_000);
  await controlCommand("park", motionParams, 90_000);
  await controlCommand("damping", {}, 20_000);
}

async function acceptAndReturn(sessionId) {
  const accepted = await controlCommand("collection_accept", { session_id: sessionId });
  updateCollectionState(accepted);
  const progress = `${accepted.counts?.accepted ?? 0} / ${accepted.target_accepted_episodes ?? 50}`;
  try {
    await returnToReferenceAfterDecision("成功数据已保存");
  } catch (error) {
    setMessage(`成功数据已保存（${progress}），但自动回位未完成：${error.message}`, true);
    return;
  }
  setMessage(`成功数据已保存并回到零位（${progress}）。摆好下一组后继续采集。`);
}

async function rejectStoppedEpisode(sessionId, reason) {
  const rejectParams = { session_id: sessionId, reason };
  for (let attempt = 0; attempt < 10; attempt += 1) {
    try {
      return await controlCommand("collection_reject", rejectParams);
    } catch (error) {
      if (!String(error.message).includes("相机录制仍在写入") || attempt === 9) {
        throw error;
      }
      await new Promise((resolve) => window.setTimeout(resolve, 250));
    }
  }
  throw new Error("自动归档失败");
}

async function discardEpisode() {
  return guarded(async () => {
    const sessionId = collectionState?.session_id;
    if (!sessionId) throw new Error("没有可丢弃的 episode");
    if (collectionState?.active) {
      setMessage("正在结束并归档当前 episode…");
      await controlCommand("collection_stop", {}, 25_000);
      endTimer();
    }
    const result = await rejectStoppedEpisode(sessionId, "operator discarded episode");
    updateCollectionState(result);
    resetLiveMetrics();
    try {
      await returnToReferenceAfterDecision("当前 episode 已丢弃");
    } catch (error) {
      setMessage(`当前 episode 已归档，但自动回位未完成：${error.message}`, true);
      return;
    }
    setMessage("当前 episode 已丢弃并回到零位，可以摆放下一组。");
  });
}

async function saveEpisode() {
  return guarded(async () => {
    if (!collectionState?.validation?.ok) throw new Error("数据完整性检查未通过，不能保存");
    await acceptAndReturn(collectionState?.session_id);
  });
}

ui.startButton.addEventListener("click", startEpisode);
ui.stopButton.addEventListener("click", () => stopEpisode(true));
ui.discardButton.addEventListener("click", discardEpisode);
ui.reviewDiscardButton.addEventListener("click", discardEpisode);
ui.saveButton.addEventListener("click", saveEpisode);
ui.copyValidationButton.addEventListener("click", () => guarded(async () => {
  await copyText(ui.copyValidationButton.dataset.copyText, ui.copyValidationButton);
}));
ui.copyMessageButton.addEventListener("click", () => guarded(async () => {
  await copyText(ui.copyMessageButton.dataset.copyText, ui.copyMessageButton);
}));

window.addEventListener("keydown", (event) => {
  const target = event.target;
  if (event.repeat || event.ctrlKey || event.altKey || event.metaKey
    || target instanceof HTMLInputElement || target instanceof HTMLTextAreaElement
    || target instanceof HTMLSelectElement || target?.isContentEditable) return;
  if (event.code === "Space") {
    if (!ui.stopButton.disabled) {
      event.preventDefault();
      stopEpisode(true);
    } else if (!ui.startButton.disabled) {
      event.preventDefault();
      startEpisode();
    }
  } else if (event.key.toLowerCase() === "s") {
    if (collectionState?.active && !busy) {
      event.preventDefault();
      stopEpisode(true);
    } else if (!ui.saveButton.disabled) {
      event.preventDefault();
      saveEpisode();
    }
  } else if (event.key.toLowerCase() === "x" && (
    !ui.discardButton.disabled || !ui.reviewDiscardButton.disabled
  )) {
    event.preventDefault();
    discardEpisode();
  }
});

window.addEventListener("resize", drawTelemetry);

window.addEventListener("beforeunload", () => {
  pageClosing = true;
  if (reconnectHandle !== null) window.clearTimeout(reconnectHandle);
  if (statusPollHandle !== null) window.clearInterval(statusPollHandle);
  for (const [, imageId] of CAMERA_PREVIEWS) {
    const objectUrl = ui[imageId].dataset.objectUrl;
    if (objectUrl) URL.revokeObjectURL(objectUrl);
  }
  window.removeEventListener("resize", drawTelemetry);
  controlSocket?.close();
});

async function prepareArm() {
  if (
    armPreparePending
    || busy
    || collectionState?.active
    || controlSocket?.readyState !== WebSocket.OPEN
    || (
      armState?.connected
      && armState.mode === "damping"
      && collectionState?.home_calibration?.fixed
    )
  ) return Boolean(armState?.connected && armState.mode === "damping");
  armPreparePending = true;
  lastArmPrepareAt = performance.now();
  updateButtons();
  try {
    setMessage("后台正在准备固定采集零位和阻尼示教模式…");
    const prepared = await controlCommand("collection_prepare", {}, 35_000);
    updateCollectionState(prepared);
    return true;
  } catch (error) {
    setMessage(`机械臂自动准备失败：${error.message ?? String(error)}`, true);
    return false;
  } finally {
    armPreparePending = false;
    updateButtons();
  }
}

async function initialize() {
  updateButtons();
  resetLiveMetrics();
  initializeCameraPreviews();
  try {
    await connectControl();
    const armReady = await prepareArm();
    const status = await controlCommand("collection_status", {}, 20_000);
    updateCollectionState(status);
    if (armReady && cameraReady) {
      setMessage("机械臂和双相机已由主机准备完成，摆好方块后开始采集。");
    } else if (!cameraReady) {
      setMessage(status.camera_state?.error ?? "主机双相机尚未就绪", true);
    }
  } catch (error) {
    if (!cameraReady) setStatus(ui.cameraStatus, "D455 + D405 未连接", "bad");
    setMessage(error.message ?? String(error), true);
  }
  statusPollHandle = window.setInterval(() => {
    const now = performance.now();
    const stateInterval = collectionState?.active ? 200 : 1_000;
    if (
      controlSocket?.readyState === WebSocket.OPEN
      && !statePollPending
      && now - lastStatePollAt >= stateInterval
    ) {
      lastStatePollAt = now;
      statePollPending = true;
      controlCommand("state", {}, 3_000)
        .catch(() => {})
        .finally(() => { statePollPending = false; });
    }
    if (
      !collectionState?.active
      && (!armState?.connected || armState.mode !== "damping")
      && now - lastArmPrepareAt >= 5_000
    ) {
      prepareArm();
    }
  }, 100);
}

initialize();
