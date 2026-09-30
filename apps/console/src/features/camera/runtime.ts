// @ts-nocheck
const MEDIAPIPE_RUNTIME_URL = "/mediapipe/vision_bundle.mjs";
const {
  DrawingUtils,
  FilesetResolver,
  HandLandmarker,
  PoseLandmarker,
} = await import(/* @vite-ignore */ MEDIAPIPE_RUNTIME_URL);
import { serviceEndpoint } from "../../api/control-protocol";
import { PoseTemporalStabilizer } from "../vision/pose-temporal-filter";
import handModelUrl from "@weights/mediapipe/hand_landmarker.task?url";
import poseModelUrl from "@weights/mediapipe/pose_landmarker_heavy.task?url";
import {
  CubePoseFilter,
  estimateCubePose,
  projectImagePolygonToBasePlane,
  projectTableCalibrationEdges,
  projectTablePolygon,
} from "../cube/pose";

const WASM_ROOT = new URL("./mediapipe/wasm/", document.baseURI).href;
const SEND_INTERVAL_MS = 50;
const CAMERA_FRAME_INTERVAL_MS = 1000 / 15;
const CAMERA_MAX_FRAME_INTERVAL_MS = 200;
const INFERENCE_IDLE_RATIO = 1.6;
const CAMERA_RECOVERY_ATTEMPTS = 3;
const CAMERA_RECOVERY_DELAY_MS = 900;
const WEBRTC_FIRST_FRAME_TIMEOUT_MS = 25_000;
const WEBRTC_DIAGNOSTIC_EVENT_LIMIT = 12;
// The camera service tracks cubes independently. Eight updates per second is
// responsive in the simulator without coupling the visual rate to arm motion.
const CUBE_POLL_INTERVAL_MS = 125;
const CUBE_REQUEST_STALE_MS = 400;
const TABLE_EDGE_ALIGN_DELAY_MS = 160;
const CUBE_ROI_STORAGE_KEY = "arx5-cube-workspace-roi-v1";
const CUBE_ROI_VISIBILITY_STORAGE_KEY = "arx5-cube-workspace-visible-v1";
const TABLE_EDGE_GUIDES_STORAGE_KEY = "arx5-table-edge-guides-v1";
const HANDEYE_TRANSFORM_STORAGE_KEY = "arx5-enabled-handeye-transform-v1";
const TABLE_EDGE_ALIGNMENT_STORAGE_KEY = "arx5-table-edge-alignment-v1";
const UPPER_LIMB_LANDMARKS = Object.freeze({
  left: Object.freeze([11, 13, 15]),
  right: Object.freeze([12, 14, 16]),
});

const elements = {
  rgbVideo: document.querySelector("#rgbVideo"),
  depthVideo: document.querySelector("#depthVideo"),
  eyeInHandVideo: document.querySelector("#eyeInHandVideo"),
  eyeInHandDepthVideo: document.querySelector("#eyeInHandDepthVideo"),
  canvas: document.querySelector("#overlay"),
  cubeCanvas: document.querySelector("#cubeOverlay"),
  depthCubeCanvas: document.querySelector("#depthCubeOverlay"),
  depthUnavailable: document.querySelector("#depthUnavailable"),
  depthUnavailableMessage: document.querySelector("#depthUnavailableMessage"),
  poseSideSwitch: document.querySelector("#poseSideSwitch"),
  bridgeUrl: document.querySelector("#bridgeUrl"),
  cameraToggleButton: document.querySelector("#cameraToggleButton"),
  cubePerceptionButton: document.querySelector("#cubePerceptionButton"),
  cubeRoiButton: document.querySelector("#cubeRoiButton"),
  cubeRoiVisibilityButton: document.querySelector("#cubeRoiVisibilityButton"),
  cubeRoiEditor: document.querySelector("#cubeRoiEditor"),
  cubeRoiUndo: document.querySelector("#cubeRoiUndo"),
  cubeRoiCommit: document.querySelector("#cubeRoiCommit"),
  cubeRoiCancel: document.querySelector("#cubeRoiCancel"),
  cubeRoiClear: document.querySelector("#cubeRoiClear"),
  tableEdgeEditButton: document.querySelector("#tableEdgeEditButton"),
  tableEdgeEditor: document.querySelector("#tableEdgeEditor"),
  tableEdgeUndo: document.querySelector("#tableEdgeUndo"),
  tableEdgeCommit: document.querySelector("#tableEdgeCommit"),
  tableEdgeCancel: document.querySelector("#tableEdgeCancel"),
  tableEdgeClear: document.querySelector("#tableEdgeClear"),
  depthHoverMarker: document.querySelector("#depthHoverMarker"),
  depthHoverTooltip: document.querySelector("#depthHoverTooltip"),
  cubeBackgroundButton: document.querySelector("#cubeBackgroundButton"),
  emptyState: document.querySelector("#emptyState"),
  modelStatus: document.querySelector("#modelStatus"),
  cameraStatus: document.querySelector("#cameraStatus"),
  fps: document.querySelector("#fps"),
  poseLatency: document.querySelector("#poseLatency"),
  poseMetric: document.querySelector("#poseMetric"),
  handMetric: document.querySelector("#handMetric"),
  pinchMetric: document.querySelector("#pinchMetric"),
  handPoseBadge: document.querySelector("#handPoseBadge"),
  visualRuntimeMetric: document.querySelector("#visualRuntimeMetric"),
  cubeMetric: document.querySelector("#cubeMetric"),
  cubeColorSwatch: document.querySelector("#cubeColorSwatch"),
  cubeCandidateMetric: document.querySelector("#cubeCandidateMetric"),
  cubePositionIcon: document.querySelector("#cubePositionIcon"),
  grootCollectionLink: document.querySelector("#grootCollectionLink"),
};

function websocketEndpoint(port) {
  return serviceEndpoint(
    port,
    port === 8765
      ? "/api/v1/vision/landmarks"
      : "/api/v1/cameras/d455/webrtc",
  );
}

elements.bridgeUrl.value = websocketEndpoint(8765);
if (elements.grootCollectionLink) {
  const collectionUrl = new URL("/collection", document.baseURI);
  const collectionToken = new URLSearchParams(window.location.search).get("token");
  if (collectionToken) collectionUrl.searchParams.set("token", collectionToken);
  elements.grootCollectionLink.href = collectionUrl.toString();
}

const context = elements.canvas.getContext("2d");
const cubeContext = elements.cubeCanvas.getContext("2d");
const depthCubeContext = elements.depthCubeCanvas.getContext("2d");
const drawing = new DrawingUtils(context);
const poseStabilizer = new PoseTemporalStabilizer();
const cubePoseFilters = new Map();
const cubePoseFilterLastSeen = new Map();
let poseLandmarker;
let handLandmarker;
let poseRecognitionEnabled = false;
let websocket;
let bridgeStateLabel = "未连接";
let bridgeReconnectAfter = -Infinity;
let cameraSignal;
let cameraPeerConnection;
let depthChannel;
let depthRequestSequence = 0;
const depthRequests = new Map();
let cubeRequestSequence = 0;
let cubeRequestPending = null;
let cubeRequestSentAt = -Infinity;
let cubePollTimer = null;
let cubePerceptionEnabled = false;
let cameraDepthAvailable = null;
let cameraDepthError = null;
let cameraDepthRecoveryActive = false;
let cameraServiceOutdated = false;
let cameraIntrinsics = null;
let handeyeTransform = loadCachedHandeyeTransform();
let cubeConfigSignature = null;
let cubeBackgroundState = "missing";
let cubeBackgroundRemaining = 0;
let cubeBackgroundTarget = 24;
let cubeTableRoiActive = false;
let cubeBackgroundRequestPending = false;
let selectedCubeCandidateId = null;
let latestCubeResponse = null;
let latestCubeCandidates = [];
let latestCubeDisplay = null;
let manualCubeRoi = loadManualCubeRoi();
let cubeRoiOverlayVisible = loadCubeRoiOverlayVisibility();
let cubeRoiDraft = [];
let cubeRoiEditing = false;
let cubeRoiDragIndex = null;
let tableEdgeGuides = loadTableEdgeGuides();
let tableEdgeDraft = [];
let tableEdgeEditing = false;
let tableEdgeDragIndex = null;
let tableEdgeAlignTimer = null;
let tableEdgeAlignedGeneration = -1;
let depthHoverLastRequestAt = -Infinity;
let depthHoverRequestGeneration = 0;
const cubeMetricDisplay = {
  source: null,
  pendingSource: null,
  pendingSourceFrames: 0,
  positionMm: null,
  positionUpdatedAt: -Infinity,
  candidateId: null,
  lastPoseAt: -Infinity,
};
const legacyCubeCandidateCenters = new Map();
let nextLegacyCubeCandidateId = 1;
let running = false;
let cameraActive = false;
let cameraGeneration = 0;
let cameraRecoveryPromise = null;
let cameraWebRtcAttemptInFlight = false;
let sequence = 0;
let lastFrameProcessedAt = -Infinity;
let lastSentAt = 0;
let framesSinceReport = 0;
let fpsReportAt = performance.now();
let smoothedPoseLatencyMs = null;
let smoothedInferenceLatencyMs = null;
let renderRequestId = null;
let renderRequestKind = null;

function setPill(element, text, state) {
  const label = element.querySelector(":scope > span");
  if (label) label.textContent = text;
  else {
    const icon = element.querySelector("i");
    element.replaceChildren(...(icon
      ? [icon, document.createTextNode(text)]
      : [document.createTextNode(text)]));
  }
  element.title = text;
  element.setAttribute("aria-label", text);
  element.className = `pill ${state}`;
}

function createWebRtcDiagnostics(generation) {
  return {
    generation,
    startedAtMs: performance.now(),
    events: [],
    tracks: [],
  };
}

function recordWebRtcDiagnostic(diagnostics, event, detail = "") {
  if (!diagnostics) return;
  const elapsedMs = Math.max(0, performance.now() - diagnostics.startedAtMs);
  diagnostics.events.push({ event, detail, elapsedMs });
  if (diagnostics.events.length > WEBRTC_DIAGNOSTIC_EVENT_LIMIT) {
    diagnostics.events.shift();
  }
}

function formatWebRtcElapsed(elapsedMs) {
  return `${Math.round(Math.max(0, Number(elapsedMs) || 0))}ms`;
}

async function webRtcFirstFrameDiagnostics(peer, diagnostics) {
  const summary = {
    ice: peer?.iceConnectionState ?? "unknown",
    connection: peer?.connectionState ?? "unknown",
    signaling: peer?.signalingState ?? "unknown",
    candidatePair: "none",
    inboundTracks: 0,
    bytes: 0,
    frames: 0,
    decoded: 0,
  };
  try {
    const reports = await peer?.getStats?.();
    reports?.forEach?.((report) => {
      if (
        report.type === "candidate-pair"
        && (report.selected === true
          || (report.nominated === true && report.state === "succeeded"))
      ) {
        summary.candidatePair = String(report.state ?? "selected");
      }
      if (
        report.type === "inbound-rtp"
        && (report.kind === "video" || report.mediaType === "video")
      ) {
        summary.inboundTracks += 1;
        summary.bytes += Number(report.bytesReceived) || 0;
        summary.frames += Number(report.framesReceived) || 0;
        summary.decoded += Number(report.framesDecoded) || 0;
      }
    });
  } catch (error) {
    summary.statsError = error?.message || "unavailable";
  }
  const events = diagnostics?.events
    ?.map(({ event, detail, elapsedMs }) => (
      `${formatWebRtcElapsed(elapsedMs)}:${event}${detail ? `=${detail}` : ""}`
    ))
    .join(", ") || "none";
  const tracks = diagnostics?.tracks?.join(",") || "none";
  return {
    ...summary,
    events,
    tracks,
    text: [
      `ICE=${summary.ice}`,
      `Peer=${summary.connection}`,
      `Pair=${summary.candidatePair}`,
      `RTP=${summary.bytes}B/${summary.frames}帧/${summary.decoded}解码`,
      `轨=${tracks}`,
      `时序=${events}`,
      summary.statsError ? `stats=${summary.statsError}` : null,
    ].filter(Boolean).join("；"),
  };
}

function finiteHandeyeTransform(value) {
  return Array.isArray(value) && value.length === 4
    && value.every((row) => Array.isArray(row) && row.length === 4
      && row.every((component) => Number.isFinite(Number(component))));
}

function loadCachedHandeyeTransform() {
  try {
    const cached = JSON.parse(localStorage.getItem(HANDEYE_TRANSFORM_STORAGE_KEY) ?? "null");
    return finiteHandeyeTransform(cached) ? cached : null;
  } catch (error) {
    console.warn("Unable to load cached hand-eye transform", error);
    return null;
  }
}

function cacheHandeyeTransform(transform) {
  try {
    if (finiteHandeyeTransform(transform)) {
      localStorage.setItem(HANDEYE_TRANSFORM_STORAGE_KEY, JSON.stringify(transform));
    } else {
      localStorage.removeItem(HANDEYE_TRANSFORM_STORAGE_KEY);
    }
  } catch (error) {
    console.warn("Unable to cache hand-eye transform", error);
  }
}

function updateEnabledHandeyeTransform(validation) {
  const transform = validation?.enabled === true ? validation.T_base_camera : null;
  if (finiteHandeyeTransform(transform)) {
    handeyeTransform = transform;
    cacheHandeyeTransform(transform);
  } else if (validation?.enabled === false) {
    handeyeTransform = null;
    cacheHandeyeTransform(null);
  }
}

function updateVisualRuntimeTitle() {
  const metric = elements.visualRuntimeMetric;
  if (!metric) return;
  const fps = elements.fps.textContent;
  const latency = elements.poseLatency.textContent;
  const pose = elements.poseMetric.textContent;
  const hand = elements.handMetric.textContent;
  const pinch = elements.pinchMetric.textContent;
  metric.title = poseRecognitionEnabled
    ? `视觉：${fps} FPS · 推理 ${latency} · 姿态 ${pose} · 手 ${hand} · 捏合 ${pinch} · 数据桥 ${bridgeStateLabel}`
    : "视觉识别已关闭";
}

function setHandPoseBadge(text, state = "off") {
  const badge = elements.handPoseBadge;
  if (!badge) return;
  const label = badge.querySelector("span");
  if (label) label.textContent = text;
  badge.className = `hand-pose-badge ${state}`;
  badge.hidden = state === "off";
}

function inferenceFrameIntervalMs() {
  if (!Number.isFinite(smoothedInferenceLatencyMs)) return CAMERA_FRAME_INTERVAL_MS;
  return Math.min(
    CAMERA_MAX_FRAME_INTERVAL_MS,
    Math.max(CAMERA_FRAME_INTERVAL_MS, smoothedInferenceLatencyMs * INFERENCE_IDLE_RATIO),
  );
}

function resetTableEdgeAlign() {
  if (tableEdgeAlignTimer !== null) window.clearTimeout(tableEdgeAlignTimer);
  tableEdgeAlignTimer = null;
  tableEdgeAlignedGeneration = -1;
}

function tableEdgeAlignmentSignature() {
  if (!tableEdgeGuides || !cameraIntrinsics || !handeyeTransform) return null;
  const round = (value, digits) => Number(Number(value).toFixed(digits));
  return JSON.stringify({
    table_edges: tableEdgeGuides.map(([x, y]) => [round(x, 5), round(y, 5)]),
    intrinsics: ["width", "height", "fx", "fy", "ppx", "ppy"].map(
      (key) => round(cameraIntrinsics[key], 3),
    ),
    transform: handeyeTransform.map((row) => row.map((value) => round(value, 5))),
  });
}

function cachedTableEdgeAlignment(signature) {
  if (!signature) return null;
  try {
    const cached = JSON.parse(localStorage.getItem(TABLE_EDGE_ALIGNMENT_STORAGE_KEY) ?? "null");
    const calibration = cached?.calibration;
    if (cached?.signature !== signature
        || ![calibration?.xMm, calibration?.yMm, calibration?.yawDeg]
          .every((value) => Number.isFinite(Number(value)))) return null;
    return calibration;
  } catch (error) {
    console.warn("Unable to load the table-edge alignment", error);
    return null;
  }
}

function cacheTableEdgeAlignment(signature, calibration) {
  if (!signature) return;
  try {
    localStorage.setItem(TABLE_EDGE_ALIGNMENT_STORAGE_KEY, JSON.stringify({
      signature,
      calibration,
    }));
  } catch (error) {
    console.warn("Unable to cache the table-edge alignment", error);
  }
}

function tableMatchesAlignment(table, calibration) {
  if (!table || !calibration) return false;
  const [xMm, yMm] = Array.isArray(table.position_mm) ? table.position_mm.map(Number) : [];
  return Number.isFinite(xMm) && Number.isFinite(yMm)
    && Math.abs(xMm - Number(calibration.xMm)) < 0.2
    && Math.abs(yMm - Number(calibration.yMm)) < 0.2
    && angularDistance(Number(table.yaw_deg), Number(calibration.yawDeg)) < 0.05;
}

function scheduleTableEdgeAlign() {
  if (tableEdgeAlignTimer !== null
      || tableEdgeAlignedGeneration === cameraGeneration
      || !cameraActive
      || !tableEdgeGuides
      || !cameraIntrinsics
      || !handeyeTransform
      || !window.arx5GetSceneCalibration?.()?.table) return;
  tableEdgeAlignTimer = window.setTimeout(() => {
    tableEdgeAlignTimer = null;
    if (tableEdgeAlignedGeneration === cameraGeneration
        || !cameraActive
        || !tableEdgeGuides
        || !cameraIntrinsics
        || !handeyeTransform) return;
    const signature = tableEdgeAlignmentSignature();
    const cached = cachedTableEdgeAlignment(signature);
    tableEdgeAlignedGeneration = cameraGeneration;
    const table = window.arx5GetSceneCalibration?.()?.table;
    if (cached) {
      if (!tableMatchesAlignment(table, cached)) {
        emitTableAutoAlignResult({ ok: true, calibration: cached, cached: true });
      }
      return;
    }
    alignTableFromEdgeGuides({ silent: true, signature });
  }, TABLE_EDGE_ALIGN_DELAY_MS);
}

function selectedSide() {
  return Number(elements.poseSideSwitch.value) < 0 ? "left" : "right";
}

function poseRecognitionMode() {
  const value = Number(elements.poseSideSwitch.value);
  return value < 0 ? "left" : value > 0 ? "right" : "off";
}

function serializeLandmarks(landmarks) {
  return landmarks.map(({ x, y, z, visibility, presence }) => ({
    x, y, z,
    ...(Number.isFinite(visibility) ? { visibility } : {}),
    ...(Number.isFinite(presence) ? { presence } : {}),
  }));
}

function distance3(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function pinchRatio(hand) {
  if (!hand) return null;
  const palmScale = distance3(hand[5], hand[17]);
  return palmScale > 1e-6 ? distance3(hand[4], hand[8]) / palmScale : null;
}

async function loadModels() {
  if (poseLandmarker && handLandmarker) {
    setPill(elements.modelStatus, "视觉模型已加载（本地缓存）", "ok");
    return;
  }
  setPill(elements.modelStatus, "视觉模型加载中", "pending");
  const vision = await FilesetResolver.forVisionTasks(WASM_ROOT);
  const createLocal = (Factory, options, modelPath) => Factory.createFromOptions(
    vision,
    {
      ...options,
      baseOptions: { ...options.baseOptions, modelAssetPath: modelPath },
    },
  );

  [poseLandmarker, handLandmarker] = await Promise.all([
    createLocal(
      PoseLandmarker,
      {
        baseOptions: { delegate: "GPU" },
        runningMode: "VIDEO",
        numPoses: 1,
        minPoseDetectionConfidence: 0.55,
        minPosePresenceConfidence: 0.55,
        minTrackingConfidence: 0.7,
      },
      poseModelUrl,
    ),
    createLocal(
      HandLandmarker,
      {
        baseOptions: { delegate: "GPU" },
        runningMode: "VIDEO",
        numHands: 2,
        minHandDetectionConfidence: 0.6,
        minHandPresenceConfidence: 0.6,
        minTrackingConfidence: 0.6,
      },
      handModelUrl,
    ),
  ]);
  setPill(elements.modelStatus, "视觉模型已加载（本地）", "ok");
}

function setCameraMediaMode(mode) {
  const usingWebRtc = mode === "webrtc";
  const depthUnavailable = usingWebRtc && cameraDepthAvailable === false;
  elements.rgbVideo.hidden = !usingWebRtc;
  elements.depthVideo.hidden = !usingWebRtc || depthUnavailable;
  elements.eyeInHandVideo.hidden = !usingWebRtc;
  elements.eyeInHandDepthVideo.hidden = !usingWebRtc;
  elements.depthVideo.closest(".camera-view")?.toggleAttribute(
    "hidden",
    !usingWebRtc,
  );
  elements.depthUnavailable.hidden = !depthUnavailable;
  elements.rgbVideo.closest(".camera-grid")?.classList.toggle(
    "rgb-only",
    !usingWebRtc,
  );
}

function setDepthAvailability(available, status = {}) {
  cameraDepthAvailable = Boolean(available);
  cameraServiceOutdated = typeof status.source_revision !== "string"
    || typeof status.service_started_at !== "string";
  cameraDepthError = typeof status.depth_error === "string"
    ? status.depth_error
    : null;
  cameraDepthRecoveryActive = status.depth_retry_in_ms != null;
  if (elements.depthUnavailableMessage) {
    elements.depthUnavailableMessage.textContent = cameraServiceOutdated
      ? "D455 服务版本过旧，请重启 Web 服务以启用深度自动恢复"
      : cameraDepthRecoveryActive
        ? "正在自动恢复"
        : "当前为 RGB-only 模式";
  }
  if (elements.depthUnavailable) {
    elements.depthUnavailable.title = cameraDepthError
      ? `深度流错误：${cameraDepthError}`
      : "深度流暂不可用";
  }
  if (!cameraDepthAvailable) hideDepthHoverTooltip();
  setCameraMediaMode(cameraActive ? "webrtc" : null);
  if (cameraActive) {
    setPill(
      elements.cameraStatus,
      cameraServiceOutdated
        ? "D455 服务版本过旧"
        : cameraDepthAvailable
          ? "D455 RGB-D + D405 运行中"
          : `D455 RGB + D405 运行中；${cameraDepthRecoveryActive ? "深度正在恢复" : "深度未启用"}`,
      cameraServiceOutdated
        ? "warning"
        : cameraDepthAvailable
          ? "ok"
          : cameraDepthRecoveryActive
            ? "warning"
            : "pending",
    );
  }
  updateCubeBackgroundUi();
}

function updateCubeBackgroundUi(status = {}) {
  if (status && typeof status.state === "string") {
    cubeBackgroundState = status.state;
    cubeBackgroundRemaining = Number(status.capture_remaining ?? 0);
    cubeBackgroundTarget = Number(status.capture_target ?? cubeBackgroundTarget);
  }
  const button = elements.cubeBackgroundButton;
  if (!button) return;
  // RTCDataChannel uses string states ("connecting", "open", ...), unlike
  // WebSocket.readyState which uses numeric constants.
  const channelReady = depthChannel?.readyState === "open";
  // Let the user click as soon as the camera data channel is ready. If the
  // calibrated tabletop ROI is not available yet, the server returns a clear
  // error instead of leaving a permanently inert control.
  const canCapture = cameraActive && channelReady;
  const capturing = cubeBackgroundState === "capturing" || cubeBackgroundRequestPending;
  // Keep the control clickable when the camera is not ready so the user gets
  // an actionable status message instead of an inert, permanently grey button.
  button.disabled = capturing;
  button.setAttribute("aria-disabled", canCapture ? "false" : "true");
  if (cubeBackgroundState === "capturing") {
    button.innerHTML = '<i class="fa-solid fa-spinner fa-spin" aria-hidden="true"></i><span>采集中 ' + Math.max(0, cubeBackgroundRemaining) + '</span>';
    button.title = `正在采集空桌面背景（${Math.max(0, cubeBackgroundRemaining)}/${cubeBackgroundTarget}）`;
  } else if (cubeBackgroundState === "ready") {
    button.innerHTML = '<i class="fa-solid fa-layer-group" aria-hidden="true"></i><span>重采集背景</span>';
    button.title = "移开方块、手和机械臂后，重新采集空桌面背景";
  } else {
    button.innerHTML = '<i class="fa-solid fa-layer-group" aria-hidden="true"></i><span>空背景（可选）</span>';
    button.title = !cameraActive
      ? "先点击“开启”连接 D455 RGB"
      : !channelReady
        ? "正在等待 D455 数据通道"
        : cubeTableRoiActive
          ? "清空桌面工作区，移开方块、手和机械臂后采集 RGB 空背景"
          : "先启用固定相机标定并完成桌面 ROI，才能采集空背景";
  }
}

function stopCubePolling({ publishLost = true } = {}) {
  if (cubePollTimer !== null) window.clearInterval(cubePollTimer);
  cubePollTimer = null;
  cubeRequestPending = null;
  cubeRequestSentAt = -Infinity;
  cubeConfigSignature = null;
  cubeBackgroundState = "missing";
  cubeBackgroundRemaining = 0;
  cubeTableRoiActive = false;
  cubeBackgroundRequestPending = false;
  cubePoseFilters.clear();
  cubePoseFilterLastSeen.clear();
  selectedCubeCandidateId = null;
  latestCubeResponse = null;
  latestCubeCandidates = [];
  latestCubeDisplay = null;
  cubeMetricDisplay.source = null;
  cubeMetricDisplay.pendingSource = null;
  cubeMetricDisplay.pendingSourceFrames = 0;
  cubeMetricDisplay.positionMm = null;
  cubeMetricDisplay.positionUpdatedAt = -Infinity;
  cubeMetricDisplay.candidateId = null;
  cubeMetricDisplay.lastPoseAt = -Infinity;
  cubeRoiEditing = false;
  cubeRoiDragIndex = null;
  cubeRoiDraft = [];
  tableEdgeEditing = false;
  tableEdgeDragIndex = null;
  tableEdgeDraft = [];
  resetTableEdgeAlign();
  legacyCubeCandidateCenters.clear();
  nextLegacyCubeCandidateId = 1;
  updateCubeRoiUi();
  updateTableEdgeUi();
  cubeContext.clearRect(0, 0, elements.cubeCanvas.width, elements.cubeCanvas.height);
  depthCubeContext.clearRect(
    0, 0, elements.depthCubeCanvas.width, elements.depthCubeCanvas.height,
  );
  updateCubeBackgroundUi();
  if (publishLost) {
    setCubeMetricUnavailable("未检测到方块", "lost");
    window.dispatchEvent(new CustomEvent("arx5-cube-detection", {
      detail: { status: "lost", timestamp: Date.now() / 1000 },
    }));
  }
}

function clearCameraFrames() {
  hideDepthHoverTooltip();
  for (const video of [
    elements.rgbVideo,
    elements.depthVideo,
    elements.eyeInHandVideo,
    elements.eyeInHandDepthVideo,
  ]) {
    if (video.srcObject) {
      for (const track of video.srcObject.getTracks()) track.stop();
    }
    video.srcObject = null;
  }
  setCameraMediaMode(null);
}

function stopCameraTransport() {
  stopCubePolling();
  const signal = cameraSignal;
  cameraSignal = undefined;
  if (signal) {
    if (signal.readyState === WebSocket.OPEN) {
      signal.send(JSON.stringify({ type: "close" }));
    }
    signal.close();
  }
  const peer = cameraPeerConnection;
  cameraPeerConnection = undefined;
  const channel = depthChannel;
  depthChannel = undefined;
  cameraDepthAvailable = null;
  cameraDepthError = null;
  cameraDepthRecoveryActive = false;
  cameraServiceOutdated = false;
  cameraIntrinsics = null;
  for (const pending of depthRequests.values()) {
    window.clearTimeout(pending.timer);
    pending.reject(new Error("D455 深度通道已关闭"));
  }
  depthRequests.clear();
  if (channel) channel.close();
  if (peer) peer.close();
  clearCameraFrames();
}

function setEmptyStateVisible(visible) {
  elements.emptyState.hidden = !visible;
  elements.emptyState.classList.toggle("is-hidden", !visible);
}

function setCameraToggle(active, pending = false) {
  elements.cameraToggleButton.disabled = pending;
  elements.cameraToggleButton.setAttribute("aria-pressed", String(active));
  elements.cameraToggleButton.setAttribute("aria-label", active ? "关闭相机" : "开启相机");
  elements.cameraToggleButton.title = active ? "关闭相机" : "开启相机";
  elements.cameraToggleButton.classList.toggle("primary", !active);
  elements.cameraToggleButton.innerHTML = active
    ? '<i class="fa-solid fa-video-slash" aria-hidden="true"></i><span>关闭</span>'
    : '<i class="fa-solid fa-video" aria-hidden="true"></i><span>开启</span>';
  updateCubeRoiUi();
  updateTableEdgeUi();
  if (active && !pending) scheduleTableEdgeAlign();
}

function updatePoseModeUi(pending = false) {
  const control = elements.poseSideSwitch;
  if (!control) return;
  const mode = poseRecognitionMode();
  const switchElement = control.closest(".vision-pose-switch");
  control.disabled = pending || !cameraActive;
  if (switchElement) {
    switchElement.dataset.mode = mode;
    switchElement.classList.toggle("pending", pending);
  }
  const description = mode === "left"
    ? "当前识别左侧人体与手部姿态；中间档关闭，右档识别右手"
    : mode === "right"
      ? "当前识别右侧人体与手部姿态；中间档关闭，左档识别左手"
      : "人体与手部姿态识别已关闭；向左选择左手，向右选择右手";
  control.setAttribute("aria-label", description);
  if (switchElement) switchElement.title = description;
  if (!cameraActive) setHandPoseBadge("手部姿态：请先开启 D455", "off");
  else if (mode === "off") setHandPoseBadge("手部姿态：关闭", "off");
  else if (pending) setHandPoseBadge("手部模型加载中…", "pending");
  else setHandPoseBadge(`正在查找${mode === "left" ? "左手" : "右手"}…`, "searching");
}

function stopPoseRecognition() {
  running = false;
  stopRenderLoop();
  poseStabilizer.reset();
  smoothedPoseLatencyMs = null;
  smoothedInferenceLatencyMs = null;
  elements.fps.textContent = "—";
  elements.poseLatency.textContent = "—";
  elements.poseMetric.textContent = "—";
  elements.handMetric.textContent = "—";
  elements.pinchMetric.textContent = "—";
  updateVisualRuntimeTitle();
  setPill(elements.modelStatus, "姿态识别已关闭", "pending");
  context.clearRect(0, 0, elements.canvas.width, elements.canvas.height);
}

async function startPoseRecognition() {
  if (!cameraActive || !poseRecognitionEnabled) return;
  ensureBridgeConnected();
  updatePoseModeUi(true);
  try {
    await loadModels();
  } catch (error) {
    console.error(error);
    poseRecognitionEnabled = false;
    elements.poseSideSwitch.value = "0";
    stopPoseRecognition();
    setPill(elements.modelStatus, `视觉模型错误：${error.message}`, "bad");
    updatePoseModeUi(false);
    setHandPoseBadge("手部模型加载失败", "error");
    return;
  }
  if (!cameraActive || !poseRecognitionEnabled) return;
  running = true;
  lastFrameProcessedAt = -Infinity;
  framesSinceReport = 0;
  fpsReportAt = performance.now();
  updatePoseModeUi(false);
  scheduleRenderLoop();
}

function showCameraError(message, generation) {
  if (generation !== cameraGeneration || !cameraActive) return;
  running = false;
  stopRenderLoop();
  poseStabilizer.reset();
  cameraActive = false;
  cameraGeneration += 1;
  stopPoseRecognition();
  stopCameraTransport();
  setCameraToggle(false);
  updatePoseModeUi(false);
  setPill(elements.cameraStatus, `相机错误：${message}`, "bad");
  setEmptyStateVisible(true);
  elements.emptyState.classList.add("error");
  elements.emptyState.textContent = message.includes("WebRTC RGB 首帧超时")
    ? `D455 已启动，但 WebRTC 视频未到达：${message}`
    : `无法启动 D455：${message}`;
}

function recoverCameraTransport(message, generation) {
  if (generation !== cameraGeneration || !cameraActive || cameraRecoveryPromise) return;
  running = false;
  stopRenderLoop();
  poseStabilizer.reset();
  stopPoseRecognition();
  stopCameraTransport();
  setCameraToggle(true, true);
  updatePoseModeUi(true);
  setPill(elements.cameraStatus, `WebRTC 媒体自动恢复：${message}`, "warning");
  setEmptyStateVisible(true);
  elements.emptyState.classList.remove("error");
  elements.emptyState.textContent = "正在仅重建 WebRTC 媒体通道；D455/D405 采集保持运行…";

  cameraRecoveryPromise = (async () => {
    try {
      await waitForCameraRecovery(generation);
      await startWebRtcCameraWithRecovery(generation);
      if (generation !== cameraGeneration || !cameraActive) return;
      setEmptyStateVisible(false);
      setCameraToggle(true);
      updatePoseModeUi(false);
      if (poseRecognitionEnabled) await startPoseRecognition();
      else stopPoseRecognition();
    } catch (error) {
      if (generation === cameraGeneration && cameraActive) {
        showCameraError(cameraErrorMessage(error), generation);
      }
    } finally {
      cameraRecoveryPromise = null;
    }
  })();
}

function cameraErrorMessage(error) {
  const message = error?.message || "WebRTC 连接失败";
  if (message.includes("VIDIOC_S_FMT") || message.includes("Device or resource busy")) {
    return "视频接口自动恢复失败，请检查 D455 USB 连接后重试";
  }
  return message;
}

function recoverableCameraError(error) {
  const message = error?.message || "";
  return message.includes("VIDIOC_S_FMT")
    || message.includes("Device or resource busy")
    || message.includes("采集进程正在自动重启")
    || message.includes("WebRTC 信令连接失败")
    || message.includes("WebRTC 信令提前断开")
    || message.includes("WebRTC RGB 首帧超时")
    || message.includes("WebRTC 媒体连接在 RGB 首帧前断开");
}

function waitForCameraRecovery(generation, delayMs = CAMERA_RECOVERY_DELAY_MS) {
  return new Promise((resolve, reject) => {
    window.setTimeout(() => {
      if (generation === cameraGeneration && cameraActive) resolve();
      else reject(new Error("摄像头连接已取消"));
    }, delayMs);
  });
}

function waitForWebSocketOpen(socket, timeoutMs = 5000) {
  return new Promise((resolve, reject) => {
    const timer = window.setTimeout(() => finish(new Error("WebRTC 信令连接超时")), timeoutMs);
    const finish = (error) => {
      window.clearTimeout(timer);
      socket.removeEventListener("open", onOpen);
      socket.removeEventListener("error", onError);
      socket.removeEventListener("close", onClose);
      if (error) reject(error);
      else resolve();
    };
    const onOpen = () => finish();
    const onError = () => finish(new Error("WebRTC 信令连接失败"));
    const onClose = () => finish(new Error("WebRTC 信令提前断开"));
    socket.addEventListener("open", onOpen);
    socket.addEventListener("error", onError);
    socket.addEventListener("close", onClose);
  });
}

function waitForSignalAnswer(socket, timeoutMs = 12000) {
  return new Promise((resolve, reject) => {
    const timer = window.setTimeout(() => finish(new Error("WebRTC 应答超时")), timeoutMs);
    const finish = (error, answer) => {
      window.clearTimeout(timer);
      socket.removeEventListener("message", onMessage);
      socket.removeEventListener("error", onError);
      socket.removeEventListener("close", onClose);
      if (error) reject(error);
      else resolve(answer);
    };
    const onMessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === "error") {
          finish(new Error(message.error || "D455 WebRTC 服务错误"));
        } else if (message.type === "answer" && typeof message.sdp === "string") {
          finish(null, message);
        }
      } catch (error) {
        finish(error);
      }
    };
    const onError = () => finish(new Error("WebRTC 信令连接失败"));
    const onClose = () => finish(new Error("WebRTC 信令提前断开"));
    socket.addEventListener("message", onMessage);
    socket.addEventListener("error", onError);
    socket.addEventListener("close", onClose);
  });
}

function waitForIceGathering(peer, timeoutMs = 5000) {
  if (peer.iceGatheringState === "complete") return Promise.resolve();
  return new Promise((resolve, reject) => {
    const timer = window.setTimeout(() => finish(new Error("WebRTC ICE 收集超时")), timeoutMs);
    const finish = (error) => {
      window.clearTimeout(timer);
      peer.removeEventListener("icegatheringstatechange", onChange);
      if (error) reject(error);
      else resolve();
    };
    const onChange = () => {
      if (peer.iceGatheringState === "complete") finish();
    };
    peer.addEventListener("icegatheringstatechange", onChange);
  });
}

function waitForVideoFrame(
  video,
  { peer, diagnostics, timeoutMs = WEBRTC_FIRST_FRAME_TIMEOUT_MS } = {},
) {
  return new Promise((resolve, reject) => {
    let callbackId;
    let timer;
    let settled = false;
    const finish = (error) => {
      if (settled) return;
      settled = true;
      window.clearTimeout(timer);
      video.removeEventListener("loadeddata", onLoadedData);
      peer?.removeEventListener?.("connectionstatechange", onPeerStateChange);
      if (callbackId !== undefined && video.cancelVideoFrameCallback) {
        video.cancelVideoFrameCallback(callbackId);
      }
      if (error) reject(error);
      else resolve();
    };
    const reportFailure = (message) => {
      if (settled) return;
      void (async () => {
        const summary = await webRtcFirstFrameDiagnostics(peer, diagnostics);
        console.warn(`ARX5 ${message}`, summary);
        finish(new Error(`${message}（${summary.text}）`));
      })();
    };
    const onPeerStateChange = () => {
      if (["failed", "closed"].includes(peer?.connectionState)) {
        reportFailure("WebRTC 媒体连接在 RGB 首帧前断开");
      }
    };
    const onLoadedData = () => {
      recordWebRtcDiagnostic(diagnostics, "rgb-loadeddata");
      finish();
    };
    timer = window.setTimeout(
      () => reportFailure("WebRTC RGB 首帧超时"),
      timeoutMs,
    );
    peer?.addEventListener?.("connectionstatechange", onPeerStateChange);
    if (video.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA && video.videoWidth > 0) {
      recordWebRtcDiagnostic(diagnostics, "rgb-ready");
      finish();
    } else if (video.requestVideoFrameCallback) {
      callbackId = video.requestVideoFrameCallback(() => {
        recordWebRtcDiagnostic(diagnostics, "rgb-frame-callback");
        finish();
      });
    } else {
      video.addEventListener("loadeddata", onLoadedData, { once: true });
    }
    onPeerStateChange();
  });
}

function preferBrowserH264(transceiver) {
  if (!transceiver.setCodecPreferences || !window.RTCRtpReceiver?.getCapabilities) return;
  const codecs = RTCRtpReceiver.getCapabilities("video")?.codecs;
  if (!codecs) return;
  const h264 = codecs.filter((codec) => codec.mimeType.toLowerCase() === "video/h264");
  const remaining = codecs.filter((codec) => codec.mimeType.toLowerCase() !== "video/h264");
  if (h264.length) transceiver.setCodecPreferences([...h264, ...remaining]);
}

async function startWebRtcCamera(generation) {
  setCameraMediaMode("webrtc");
  const diagnostics = createWebRtcDiagnostics(generation);
  recordWebRtcDiagnostic(diagnostics, "peer-created");
  const peer = new RTCPeerConnection({ iceServers: [] });
  const channel = peer.createDataChannel("depth");
  depthChannel = channel;
  channel.addEventListener("message", (event) => {
    try {
      const response = JSON.parse(event.data);
      if (response.type === "camera_status") {
        cameraIntrinsics = response.intrinsics ?? null;
        setDepthAvailability(response.depth_available === true, response);
        syncCubeDetectionConfig();
        publishCubeWorkAreaProjection();
        return;
      }
      if (response.type === "cube_detection") {
        if (response.id === cubeRequestPending) cubeRequestPending = null;
        if (
          typeof response.depth_available === "boolean"
          && (
            response.depth_available !== cameraDepthAvailable
            || (response.depth_error ?? null) !== cameraDepthError
          )
        ) {
          setDepthAvailability(response.depth_available, response);
        }
        handleCubeDetection(response);
        return;
      }
      if (response.type === "cube_background") {
        cubeBackgroundRequestPending = false;
        updateCubeBackgroundUi(response);
        if (response.ok === false) {
          const error = response.error || "空背景采集失败";
          cubeBackgroundState = "missing";
          updateCubeBackgroundUi();
          if (elements.cubeBackgroundButton) elements.cubeBackgroundButton.title = error;
          setCubeMetricUnavailable(error, "error");
        }
        return;
      }
      const pending = depthRequests.get(response.id);
      if (!pending) return;
      depthRequests.delete(response.id);
      window.clearTimeout(pending.timer);
      if (response.ok) pending.resolve(response);
      else pending.reject(new Error(response.error || "D455 深度无效"));
    } catch (error) {
      console.warn("D455 depth response is invalid", error);
    }
  });
  channel.addEventListener("close", () => {
    if (depthChannel === channel) {
      depthChannel = undefined;
      stopCubePolling();
    }
  });
  channel.addEventListener("open", () => {
    // Match the server to this page's explicit default on every WebRTC
    // connection; otherwise a previous page can leave detection running.
    channel.send(JSON.stringify({ type: "cube_perception", enabled: cubePerceptionEnabled }));
    channel.send(JSON.stringify({ type: "camera_status", id: "camera-status" }));
    channel.send(JSON.stringify({ type: "cube_background_status", id: "cube-background-status" }));
    updateCubePerceptionButton();
    if (cubePerceptionEnabled) {
      requestCubeDetection();
      cubePollTimer = window.setInterval(requestCubeDetection, CUBE_POLL_INTERVAL_MS);
    }
    updateCubeBackgroundUi();
  });
  const rgbTransceiver = peer.addTransceiver("video", { direction: "recvonly" });
  const depthTransceiver = peer.addTransceiver("video", { direction: "recvonly" });
  const eyeInHandTransceiver = peer.addTransceiver("video", { direction: "recvonly" });
  const eyeInHandDepthTransceiver = peer.addTransceiver("video", { direction: "recvonly" });
  preferBrowserH264(rgbTransceiver);
  preferBrowserH264(depthTransceiver);
  preferBrowserH264(eyeInHandTransceiver);
  preferBrowserH264(eyeInHandDepthTransceiver);
  cameraPeerConnection = peer;

  peer.addEventListener("iceconnectionstatechange", () => {
    recordWebRtcDiagnostic(diagnostics, "ice", peer.iceConnectionState);
  });

  peer.addEventListener("track", (event) => {
    let target;
    let label;
    if (event.transceiver === rgbTransceiver) {
      target = elements.rgbVideo;
      label = "D455-RGB";
    }
    else if (event.transceiver === depthTransceiver) {
      target = elements.depthVideo;
      label = "D455-depth";
    }
    else if (event.transceiver === eyeInHandTransceiver) {
      target = elements.eyeInHandVideo;
      label = "D405-RGB";
    }
    else if (event.transceiver === eyeInHandDepthTransceiver) {
      target = elements.eyeInHandDepthVideo;
      label = "D405-depth";
    }
    else if (!elements.rgbVideo.srcObject) {
      target = elements.rgbVideo;
      label = "D455-RGB?";
    }
    else if (!elements.depthVideo.srcObject) {
      target = elements.depthVideo;
      label = "D455-depth?";
    }
    else if (!elements.eyeInHandVideo.srcObject) {
      target = elements.eyeInHandVideo;
      label = "D405-RGB?";
    }
    else {
      target = elements.eyeInHandDepthVideo;
      label = "D405-depth?";
    }
    diagnostics.tracks.push(label);
    recordWebRtcDiagnostic(diagnostics, "track", label);
    target.srcObject = new MediaStream([event.track]);
    target.play().catch((error) => console.warn("WebRTC autoplay failed", error));
  });
  peer.addEventListener("connectionstatechange", () => {
    recordWebRtcDiagnostic(diagnostics, "peer", peer.connectionState);
    if (
      peer === cameraPeerConnection
      && generation === cameraGeneration
      && peer.connectionState === "failed"
      && !cameraWebRtcAttemptInFlight
    ) {
      recoverCameraTransport("WebRTC 视频连接中断", generation);
    }
  });

  const signal = new WebSocket(websocketEndpoint(8767));
  cameraSignal = signal;
  await waitForWebSocketOpen(signal);
  recordWebRtcDiagnostic(diagnostics, "signal-open");
  const offer = await peer.createOffer();
  await peer.setLocalDescription(offer);
  await waitForIceGathering(peer);
  recordWebRtcDiagnostic(diagnostics, "offer-ready");
  const answerPromise = waitForSignalAnswer(signal);
  signal.send(JSON.stringify({ type: "offer", sdp: peer.localDescription.sdp }));
  const answer = await answerPromise;
  recordWebRtcDiagnostic(diagnostics, "answer-received");
  await peer.setRemoteDescription({ type: "answer", sdp: answer.sdp });
  recordWebRtcDiagnostic(diagnostics, "remote-description");
  // RGB is the required path. Depth may be unavailable or intentionally
  // disabled, so it must never prevent the RGB stream from becoming usable.
  await waitForVideoFrame(elements.rgbVideo, { peer, diagnostics });
  if (generation !== cameraGeneration || !cameraActive) {
    throw new Error("摄像头连接已取消");
  }

  signal.addEventListener("close", () => {
    if (
      signal === cameraSignal
      && generation === cameraGeneration
      && cameraActive
    ) {
      recoverCameraTransport("WebRTC 信令已断开", generation);
    }
  });
}

/** Request one robust metric depth patch for later cube targeting. */
function requestDepthSample(x, y, radius = 2) {
  if (!depthChannel || depthChannel.readyState !== "open") {
    return Promise.reject(new Error("D455 深度通道尚未就绪"));
  }
  const id = `depth-${++depthRequestSequence}`;
  return new Promise((resolve, reject) => {
    const timer = window.setTimeout(() => {
      depthRequests.delete(id);
      reject(new Error("D455 深度采样超时"));
    }, 1500);
    depthRequests.set(id, { resolve, reject, timer });
    depthChannel.send(JSON.stringify({ type: "depth_sample", id, x, y, radius }));
  });
}

function releaseCameraForCalibration() {
  if (!depthChannel || depthChannel.readyState !== "open") {
    return Promise.resolve();
  }
  const id = `camera-release-${++depthRequestSequence}`;
  return new Promise((resolve, reject) => {
    const timer = window.setTimeout(() => {
      depthRequests.delete(id);
      reject(new Error("相机服务未能及时释放 D455/D405"));
    }, 10000);
    depthRequests.set(id, { resolve, reject, timer });
    depthChannel.send(JSON.stringify({ type: "camera_release_for_calibration", id }));
  });
}

function normalizedCubeRoi(value) {
  if (!Array.isArray(value) || value.length < 3 || value.length > 24) return null;
  const points = value.map((point) => (
    Array.isArray(point) && point.length === 2 ? point.map(Number) : [NaN, NaN]
  ));
  if (!points.every(([x, y]) => (
    Number.isFinite(x) && Number.isFinite(y) && x >= 0 && x <= 1 && y >= 0 && y <= 1
  ))) return null;
  const doubleArea = Math.abs(points.reduce((sum, [x, y], index) => {
    const [nextX, nextY] = points[(index + 1) % points.length];
    return sum + x * nextY - nextX * y;
  }, 0));
  // Keep very small or nearly collinear polygons from becoming a detection ROI.
  return doubleArea >= 0.00005 ? points : null;
}

function normalizedTableEdgeGuides(value) {
  if (!Array.isArray(value) || value.length !== 3) return null;
  const points = value.map((point) => (
    Array.isArray(point) && point.length === 2 ? point.map(Number) : [NaN, NaN]
  ));
  if (!points.every(([x, y]) => (
    Number.isFinite(x) && Number.isFinite(y) && x >= 0 && x <= 1 && y >= 0 && y <= 1
  ))) return null;
  const firstLength = Math.hypot(
    points[1][0] - points[0][0],
    points[1][1] - points[0][1],
  );
  const secondLength = Math.hypot(
    points[2][0] - points[1][0],
    points[2][1] - points[1][1],
  );
  const cross = Math.abs(
    (points[1][0] - points[0][0]) * (points[2][1] - points[1][1])
      - (points[1][1] - points[0][1]) * (points[2][0] - points[1][0]),
  );
  return firstLength >= 0.02 && secondLength >= 0.02 && cross >= 0.0001 ? points : null;
}

function loadManualCubeRoi() {
  try {
    return normalizedCubeRoi(JSON.parse(localStorage.getItem(CUBE_ROI_STORAGE_KEY) ?? "null"));
  } catch (error) {
    console.warn("Unable to load the cube work area", error);
    return null;
  }
}

function loadCubeRoiOverlayVisibility() {
  try {
    return localStorage.getItem(CUBE_ROI_VISIBILITY_STORAGE_KEY) !== "false";
  } catch (error) {
    console.warn("Unable to load the cube work-area visibility", error);
    return true;
  }
}

function saveCubeRoiOverlayVisibility() {
  try {
    localStorage.setItem(CUBE_ROI_VISIBILITY_STORAGE_KEY, String(cubeRoiOverlayVisible));
  } catch (error) {
    console.warn("Unable to save the cube work-area visibility", error);
  }
}

function loadTableEdgeGuides() {
  try {
    return normalizedTableEdgeGuides(
      JSON.parse(localStorage.getItem(TABLE_EDGE_GUIDES_STORAGE_KEY) ?? "null"),
    );
  } catch (error) {
    console.warn("Unable to load the table-edge guides", error);
    return null;
  }
}

function saveTableEdgeGuides() {
  try {
    if (tableEdgeGuides) {
      localStorage.setItem(TABLE_EDGE_GUIDES_STORAGE_KEY, JSON.stringify(tableEdgeGuides));
    } else {
      localStorage.removeItem(TABLE_EDGE_GUIDES_STORAGE_KEY);
    }
  } catch (error) {
    console.warn("Unable to save the table-edge guides", error);
  }
}

function saveManualCubeRoi() {
  try {
    if (manualCubeRoi) localStorage.setItem(CUBE_ROI_STORAGE_KEY, JSON.stringify(manualCubeRoi));
    else localStorage.removeItem(CUBE_ROI_STORAGE_KEY);
  } catch (error) {
    console.warn("Unable to save the cube work area", error);
  }
}

function cubeRoiDimensions() {
  return {
    width: Math.max(1, Number(cameraIntrinsics?.width) || elements.rgbVideo.videoWidth || 640),
    height: Math.max(1, Number(cameraIntrinsics?.height) || elements.rgbVideo.videoHeight || 480),
  };
}

function manualCubeRoiPixels() {
  if (!manualCubeRoi) return null;
  const { width, height } = cubeRoiDimensions();
  return manualCubeRoi.map(([x, y]) => [x * width, y * height]);
}

function tableEdgeGuidePixels() {
  if (!tableEdgeGuides) return null;
  const { width, height } = cubeRoiDimensions();
  return tableEdgeGuides.map(([x, y]) => [x * width, y * height]);
}

function publishCubeWorkAreaProjection() {
  const sourcePolygon = cubeRoiEditing && cubeRoiDraft.length >= 3
    ? cubeRoiDraft
    : manualCubeRoi;
  const { width, height } = cubeRoiDimensions();
  const pixelPolygon = Array.isArray(sourcePolygon)
    ? sourcePolygon.map(([x, y]) => [x * width, y * height])
    : null;
  const sceneCalibration = window.arx5GetSceneCalibration?.();
  const tableZM = Number(sceneCalibration?.table?.position_mm?.[2]) / 1000;
  let basePolygon = null;
  if (pixelPolygon && cameraIntrinsics && handeyeTransform && Number.isFinite(tableZM)) {
    try {
      basePolygon = projectImagePolygonToBasePlane(
        pixelPolygon,
        cameraIntrinsics,
        handeyeTransform,
        tableZM,
      );
    } catch (error) {
      console.warn("Unable to project the cube work area into BASE", error);
    }
  }
  window.dispatchEvent(new CustomEvent("arx5-cube-work-area", {
    detail: {
      active: Boolean(sourcePolygon) && (cubeRoiOverlayVisible || cubeRoiEditing),
      pixel_polygon: pixelPolygon,
      base_polygon_m: basePolygon,
      table_z_m: Number.isFinite(tableZM) ? tableZM : null,
    },
  }));
}

function publishTableEdgeGuidePreview() {
  const sceneCalibration = window.arx5GetSceneCalibration?.();
  const tableZM = Number(sceneCalibration?.table?.position_mm?.[2]) / 1000;
  let basePoints = null;
  if (tableEdgeEditing && tableEdgeDraft.length === 3
      && cameraIntrinsics && handeyeTransform && Number.isFinite(tableZM)) {
    const { width, height } = cubeRoiDimensions();
    const pixels = tableEdgeDraft.map(([x, y]) => [x * width, y * height]);
    try {
      basePoints = projectImagePolygonToBasePlane(
        pixels,
        cameraIntrinsics,
        handeyeTransform,
        tableZM,
      );
    } catch {
      basePoints = null;
    }
  }
  window.dispatchEvent(new CustomEvent("arx5-table-edge-guides-preview", {
    detail: {
      active: Boolean(tableEdgeEditing && basePoints),
      base_points_m: basePoints,
      drag_index: tableEdgeDragIndex,
    },
  }));
}

function pointInPolygon(point, polygon) {
  let inside = false;
  for (let index = 0, previous = polygon.length - 1; index < polygon.length; previous = index++) {
    const [x, y] = polygon[index];
    const [previousX, previousY] = polygon[previous];
    if ((y > point[1]) !== (previousY > point[1])
        && point[0] < ((previousX - x) * (point[1] - y)) / (previousY - y) + x) {
      inside = !inside;
    }
  }
  return inside;
}

function cubeCandidateInManualRoi(candidate) {
  const polygon = manualCubeRoiPixels();
  const center = candidate?.center_px?.map(Number);
  return !polygon || (Array.isArray(center) && center.length === 2
    && center.every(Number.isFinite) && pointInPolygon(center, polygon));
}

function updateCubeRoiUi() {
  const available = cameraActive && elements.rgbVideo.videoWidth > 0;
  if (elements.cubeRoiButton) {
    elements.cubeRoiButton.disabled = !available;
    elements.cubeRoiButton.classList.toggle("selected", cubeRoiEditing || Boolean(manualCubeRoi));
    elements.cubeRoiButton.setAttribute("aria-pressed", String(cubeRoiEditing));
    elements.cubeRoiButton.title = manualCubeRoi
      ? "调整方块识别工作区边界"
      : "绘制方块识别工作区";
  }
  if (elements.cubeRoiEditor) elements.cubeRoiEditor.hidden = !cubeRoiEditing;
  if (elements.cubeRoiUndo) elements.cubeRoiUndo.disabled = cubeRoiDraft.length === 0;
  if (elements.cubeRoiCommit) {
    elements.cubeRoiCommit.disabled = normalizedCubeRoi(cubeRoiDraft) === null;
  }
  if (elements.cubeRoiClear) elements.cubeRoiClear.disabled = !manualCubeRoi;
  if (elements.cubeRoiVisibilityButton) {
    elements.cubeRoiVisibilityButton.disabled = !manualCubeRoi && !cubeRoiEditing;
    elements.cubeRoiVisibilityButton.setAttribute(
      "aria-pressed",
      String(cubeRoiOverlayVisible),
    );
    elements.cubeRoiVisibilityButton.title = cubeRoiOverlayVisible
      ? "隐藏工作区边界和投影，检测仍继续使用该区域"
      : "显示工作区边界和投影";
    elements.cubeRoiVisibilityButton.innerHTML = cubeRoiOverlayVisible
      ? '<i class="fa-solid fa-eye" aria-hidden="true"></i><span>显示</span>'
      : '<i class="fa-solid fa-eye-slash" aria-hidden="true"></i><span>隐藏</span>';
  }
  elements.cubeCanvas.classList.toggle("roi-editing", cubeRoiEditing || tableEdgeEditing);
  elements.cubeCanvas.classList.toggle(
    "roi-point-dragging",
    (cubeRoiEditing && cubeRoiDragIndex !== null)
      || (tableEdgeEditing && tableEdgeDragIndex !== null),
  );
  publishCubeWorkAreaProjection();
}

function updateTableEdgeUi() {
  const available = cameraActive && elements.rgbVideo.videoWidth > 0;
  if (elements.tableEdgeEditButton) {
    elements.tableEdgeEditButton.disabled = !available;
    elements.tableEdgeEditButton.classList.toggle("selected", tableEdgeEditing);
    elements.tableEdgeEditButton.setAttribute("aria-pressed", String(tableEdgeEditing));
    elements.tableEdgeEditButton.title = tableEdgeGuides
      ? "调整独立桌边标注"
      : "在 RGB 画面中单独标注桌面远边和右边";
  }
  if (elements.tableEdgeEditor) elements.tableEdgeEditor.hidden = !tableEdgeEditing;
  if (elements.tableEdgeUndo) elements.tableEdgeUndo.disabled = tableEdgeDraft.length === 0;
  if (elements.tableEdgeCommit) {
    elements.tableEdgeCommit.disabled = normalizedTableEdgeGuides(tableEdgeDraft) === null;
  }
  if (elements.tableEdgeClear) elements.tableEdgeClear.disabled = !tableEdgeGuides;
  elements.cubeCanvas.classList.toggle("roi-editing", cubeRoiEditing || tableEdgeEditing);
  elements.cubeCanvas.classList.toggle(
    "roi-point-dragging",
    (cubeRoiEditing && cubeRoiDragIndex !== null)
      || (tableEdgeEditing && tableEdgeDragIndex !== null),
  );
  publishTableEdgeGuidePreview();
}

function redrawCubeOverlay() {
  drawCubeDetection(latestCubeDisplay ?? { status: "lost" });
}

function setCubeRoiEditing(active) {
  cubeRoiEditing = Boolean(active && cameraActive);
  if (cubeRoiEditing) {
    tableEdgeEditing = false;
    tableEdgeDragIndex = null;
    tableEdgeDraft = [];
  }
  cubeRoiDragIndex = null;
  cubeRoiDraft = cubeRoiEditing && manualCubeRoi
    ? manualCubeRoi.map(([x, y]) => [x, y])
    : [];
  updateCubeRoiUi();
  updateTableEdgeUi();
  redrawCubeOverlay();
}

function setTableEdgeEditing(active) {
  tableEdgeEditing = Boolean(active && cameraActive);
  if (tableEdgeEditing) {
    cubeRoiEditing = false;
    cubeRoiDragIndex = null;
    cubeRoiDraft = [];
  }
  tableEdgeDragIndex = null;
  tableEdgeDraft = tableEdgeEditing && tableEdgeGuides
    ? tableEdgeGuides.map(([x, y]) => [x, y])
    : [];
  updateCubeRoiUi();
  updateTableEdgeUi();
  redrawCubeOverlay();
}

function commitCubeRoi() {
  const polygon = normalizedCubeRoi(cubeRoiDraft);
  if (!polygon) return;
  manualCubeRoi = polygon;
  saveManualCubeRoi();
  cubeRoiEditing = false;
  cubeRoiDragIndex = null;
  cubeRoiDraft = [];
  cubeConfigSignature = null;
  updateCubeRoiUi();
  syncCubeDetectionConfig();
  publishCubeWorkAreaProjection();
  redrawCubeOverlay();
}

function clearManualCubeRoi() {
  manualCubeRoi = null;
  saveManualCubeRoi();
  cubeRoiEditing = false;
  cubeRoiDragIndex = null;
  cubeRoiDraft = [];
  cubeConfigSignature = null;
  updateCubeRoiUi();
  syncCubeDetectionConfig();
  publishCubeWorkAreaProjection();
  redrawCubeOverlay();
}

function commitTableEdgeGuides() {
  const guides = normalizedTableEdgeGuides(tableEdgeDraft);
  if (!guides) return;
  tableEdgeGuides = guides;
  saveTableEdgeGuides();
  tableEdgeDragIndex = null;
  tableEdgeDraft = guides.map(([x, y]) => [x, y]);
  resetTableEdgeAlign();
  updateCubeRoiUi();
  updateTableEdgeUi();
  redrawCubeOverlay();
  window.dispatchEvent(new CustomEvent("arx5-table-edge-guides-change", {
    detail: { active: true },
  }));
  alignTableFromEdgeGuides();
}

function clearTableEdgeGuides() {
  tableEdgeGuides = null;
  saveTableEdgeGuides();
  tableEdgeEditing = false;
  tableEdgeDragIndex = null;
  tableEdgeDraft = [];
  resetTableEdgeAlign();
  updateCubeRoiUi();
  updateTableEdgeUi();
  redrawCubeOverlay();
  window.dispatchEvent(new CustomEvent("arx5-table-edge-guides-change", {
    detail: { active: false },
  }));
}

function rgbPointFromPointer(event) {
  const width = elements.rgbVideo.videoWidth;
  const height = elements.rgbVideo.videoHeight;
  const rect = elements.rgbVideo.getBoundingClientRect();
  if (width <= 0 || height <= 0 || rect.width <= 0 || rect.height <= 0) return null;
  const scale = Math.min(rect.width / width, rect.height / height);
  const contentWidth = width * scale;
  const contentHeight = height * scale;
  const x = (event.clientX - rect.left - (rect.width - contentWidth) / 2) / scale;
  const y = (event.clientY - rect.top - (rect.height - contentHeight) / 2) / scale;
  if (x < 0 || x > width || y < 0 || y > height) return null;
  return [x / width, y / height];
}

function videoPointFromPointer(event, video) {
  const width = video.videoWidth;
  const height = video.videoHeight;
  const rect = video.getBoundingClientRect();
  if (width <= 0 || height <= 0 || rect.width <= 0 || rect.height <= 0) return null;
  const scale = Math.min(rect.width / width, rect.height / height);
  const contentWidth = width * scale;
  const contentHeight = height * scale;
  const localX = event.clientX - rect.left;
  const localY = event.clientY - rect.top;
  const x = (localX - (rect.width - contentWidth) / 2) / scale;
  const y = (localY - (rect.height - contentHeight) / 2) / scale;
  if (x < 0 || x >= width || y < 0 || y >= height) return null;
  return { x, y, localX, localY };
}

function annotationVertexNear(point, draft) {
  const { width, height } = cubeRoiDimensions();
  const radius = 16;
  let nearest = -1;
  let distanceSquared = radius * radius;
  draft.forEach(([x, y], index) => {
    const deltaX = x * width - point[0] * width;
    const deltaY = y * height - point[1] * height;
    const candidateDistanceSquared = deltaX * deltaX + deltaY * deltaY;
    if (candidateDistanceSquared <= distanceSquared) {
      nearest = index;
      distanceSquared = candidateDistanceSquared;
    }
  });
  return nearest;
}

function clampNormalizedPoint([x, y]) {
  return [Math.min(1, Math.max(0, x)), Math.min(1, Math.max(0, y))];
}

function pointToImageLineDistance(point, line) {
  if (!Array.isArray(line) || line.length !== 2) return Infinity;
  const [[x0, y0], [x1, y1]] = line;
  const length = Math.hypot(x1 - x0, y1 - y0);
  if (length < 1) return Infinity;
  return Math.abs((x1 - x0) * (y0 - point[1]) - (x0 - point[0]) * (y1 - y0)) / length;
}

function closestPointOnImageLine(point, line) {
  if (!Array.isArray(line) || line.length !== 2) return null;
  const [[x0, y0], [x1, y1]] = line;
  const deltaX = x1 - x0;
  const deltaY = y1 - y0;
  const lengthSquared = deltaX * deltaX + deltaY * deltaY;
  if (lengthSquared < 1) return null;
  const amount = ((point[0] - x0) * deltaX + (point[1] - y0) * deltaY) / lengthSquared;
  return [x0 + amount * deltaX, y0 + amount * deltaY];
}

function extendImageLine(line, beforePixels = 0, afterPixels = 0) {
  if (!Array.isArray(line) || line.length !== 2) return line;
  const [[x0, y0], [x1, y1]] = line;
  const length = Math.hypot(x1 - x0, y1 - y0);
  if (length < 1) return line;
  const unitX = (x1 - x0) / length;
  const unitY = (y1 - y0) / length;
  return [
    [x0 - unitX * beforePixels, y0 - unitY * beforePixels],
    [x1 + unitX * afterPixels, y1 + unitY * afterPixels],
  ];
}

function tableCornerExtensionPixels() {
  const { width, height } = cubeRoiDimensions();
  return Math.max(140, Math.min(320, Math.min(width, height) * 0.38));
}

function normalizedAngle(degrees) {
  return ((degrees + 180) % 360 + 360) % 360 - 180;
}

function angularDistance(first, second) {
  return Math.abs(normalizedAngle(first - second));
}

function lineAngleDegrees(start, end) {
  return Math.atan2(end[1] - start[1], end[0] - start[0]) * 180 / Math.PI;
}

function undirectedAngularDistance(first, second) {
  const difference = angularDistance(first, second);
  return Math.min(difference, Math.abs(180 - difference));
}

function tableEdgesFromGuides() {
  const points = tableEdgeGuidePixels();
  if (!points || points.length !== 3) {
    throw new Error("请先单独标注桌面远边和右边");
  }
  const top = {
    start: points[0],
    end: points[1],
    length: Math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1]),
  };
  const side = {
    start: points[1],
    end: points[2],
    length: Math.hypot(points[2][0] - points[1][0], points[2][1] - points[1][1]),
  };
  if (top.length < 18 || side.length < 18) {
    throw new Error("两条桌边标注过短，请拉开端点");
  }
  const { width, height } = cubeRoiDimensions();
  const edgeSeparation = undirectedAngularDistance(
    lineAngleDegrees(top.start, top.end),
    lineAngleDegrees(side.start, side.end),
  );
  const corner = points[1];
  if (edgeSeparation < 8
      || corner[0] < -64 || corner[0] > width + 64
      || corner[1] < -64 || corner[1] > height + 64) {
    throw new Error("两条桌边需要在公共角点形成明显夹角");
  }
  return {
    top: [top.start, corner],
    side: [corner, side.end],
    inliers: 3,
  };
}

function emitTableAutoAlignResult(detail) {
  const applied = detail?.ok === true && detail.calibration
    && window.arx5ApplyTableCalibration?.(detail.calibration) === true;
  window.dispatchEvent(new CustomEvent("arx5-table-auto-align-result", {
    detail: { ...detail, applied },
  }));
}

function alignTableFromEdgeGuides(options = {}) {
  const silent = options?.silent === true;
  const signature = options?.signature ?? tableEdgeAlignmentSignature();
  try {
    const sceneCalibration = window.arx5GetSceneCalibration?.();
    const table = sceneCalibration?.table;
    if (!cameraIntrinsics || !handeyeTransform || !table) {
      throw new Error("需要已启用的固定相机手眼标定");
    }
    const edges = tableEdgesFromGuides();
    const tableZM = Number(table.position_mm?.[2]) / 1000;
    const [topStartBase, cornerBase, sideEndBase] = projectImagePolygonToBasePlane(
      [edges.top[0], edges.top[1], edges.side[1]],
      cameraIntrinsics,
      handeyeTransform,
      tableZM,
    );
    const edgeYawDeg = Math.atan2(
      cornerBase[1] - topStartBase[1],
      cornerBase[0] - topStartBase[0],
    ) * 180 / Math.PI;
    const sideVector = [
      sideEndBase[0] - cornerBase[0],
      sideEndBase[1] - cornerBase[1],
    ];
    const sideLength = Math.hypot(...sideVector);
    if (sideLength < 1e-5) throw new Error("桌面右边标注过短");
    const sideDirection = sideVector.map((value) => value / sideLength);
    const yawCandidates = [edgeYawDeg, edgeYawDeg + 180].map(normalizedAngle);
    const sideAgreement = (candidate) => {
      const radians = candidate * Math.PI / 180;
      return sideDirection[0] * Math.sin(radians) - sideDirection[1] * Math.cos(radians);
    };
    const yawDeg = yawCandidates.reduce((best, candidate) => (
      sideAgreement(candidate) > sideAgreement(best) ? candidate : best
    ));
    if (sideAgreement(yawDeg) < 0.35) {
      throw new Error("请按“远边起点 → 公共角点 → 右边终点”的顺序标注");
    }
    const [widthM, depthM] = table.size_mm.map((value) => Number(value) / 1000);
    const yawRadians = yawDeg * Math.PI / 180;
    const centerX = cornerBase[0] - Math.cos(yawRadians) * widthM / 2
      + Math.sin(yawRadians) * depthM / 2;
    const centerY = cornerBase[1] - Math.sin(yawRadians) * widthM / 2
      - Math.cos(yawRadians) * depthM / 2;
    if (![centerX, centerY, yawDeg].every(Number.isFinite)
        || Math.abs(centerX) > 3 || Math.abs(centerY) > 3) {
      throw new Error("桌边对齐结果超出 3D 场景范围，未应用");
    }
    const calibration = {
      xMm: centerX * 1000,
      yMm: centerY * 1000,
      yawDeg,
    };
    cacheTableEdgeAlignment(signature, calibration);
    redrawCubeOverlay();
    emitTableAutoAlignResult({
      ok: true,
      calibration,
      inliers: edges.inliers,
    });
  } catch (error) {
    const message = error instanceof Error ? error.message : "独立桌边对齐失败";
    if (silent) console.warn("Automatic table-edge alignment skipped", message);
    else emitTableAutoAlignResult({ ok: false, error: message });
  }
}

function updateCubePerceptionButton() {
  const button = elements.cubePerceptionButton;
  if (!button) return;
  button.setAttribute("aria-pressed", String(cubePerceptionEnabled));
  button.title = cubePerceptionEnabled ? "关闭方块感知" : "开启方块感知";
  button.classList.toggle("primary", cubePerceptionEnabled);
  button.querySelector("span").textContent = cubePerceptionEnabled ? "方块开" : "方块";
}

function setCubePerception(enabled) {
  cubePerceptionEnabled = Boolean(enabled);
  stopCubePolling();
  if (depthChannel?.readyState === "open") {
    depthChannel.send(JSON.stringify({ type: "cube_perception", enabled: cubePerceptionEnabled }));
  }
  if (cubePerceptionEnabled) {
    requestCubeDetection();
    cubePollTimer = window.setInterval(requestCubeDetection, CUBE_POLL_INTERVAL_MS);
  } else {
    setCubeMetricUnavailable("方块感知已关闭", "idle");
    window.dispatchEvent(new CustomEvent("arx5-cube-detection", { detail: { status: "disabled", timestamp: Date.now() / 1000 } }));
  }
  updateCubePerceptionButton();
}

function requestCubeDetection() {
  if (!cubePerceptionEnabled || !depthChannel || depthChannel.readyState !== "open") return;
  if (cubeRequestPending !== null
      && performance.now() - cubeRequestSentAt < CUBE_REQUEST_STALE_MS) return;
  syncCubeDetectionConfig();
  const id = `cube-${++cubeRequestSequence}`;
  cubeRequestPending = id;
  cubeRequestSentAt = performance.now();
  depthChannel.send(JSON.stringify({ type: "cube_detection", id }));
}

function expandCubeRoiPolygon(polygon, intrinsics) {
  if (!Array.isArray(polygon) || polygon.length < 3) return polygon;
  const width = Math.max(1, Number(intrinsics?.width) || elements.rgbVideo.videoWidth || 640);
  const height = Math.max(1, Number(intrinsics?.height) || elements.rgbVideo.videoHeight || 480);
  const points = polygon
    .filter((point) => Array.isArray(point) && point.length === 2)
    .map(([x, y]) => [Number(x), Number(y)])
    .filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
  if (points.length < 3) return polygon;
  const paddingX = Math.max(36, width * 0.08);
  const paddingY = Math.max(28, height * 0.07);
  const minX = Math.max(0, Math.min(...points.map(([x]) => x)) - paddingX);
  const maxX = Math.min(width, Math.max(...points.map(([x]) => x)) + paddingX);
  const minY = Math.max(0, Math.min(...points.map(([, y]) => y)) - paddingY);
  const projectedWidth = maxX - minX;
  const projectedCenter = (minX + maxX) / 2;
  if (projectedWidth < width * 0.72
      || projectedCenter < width * 0.25
      || projectedCenter > width * 0.75) {
    return [[0, height * 0.28], [width, height * 0.28], [width, height], [0, height]];
  }
  // The tabletop continues to the bottom edge in the fixed D455 setup. This
  // catches small objects near the front edge even when calibration corners
  // are a little outside the live RGB crop.
  const maxY = height;
  if (maxX - minX < 48 || maxY - minY < 48) return polygon;
  return [[minX, minY], [maxX, minY], [maxX, maxY], [minX, maxY]];
}

function syncCubeDetectionConfig() {
  if (!depthChannel || depthChannel.readyState !== "open") return;
  const sceneCalibration = window.arx5GetSceneCalibration?.();
  let polygon = manualCubeRoiPixels();
  if (!polygon && cameraIntrinsics && handeyeTransform && sceneCalibration?.table) {
    try {
      polygon = projectTablePolygon(
        sceneCalibration.table,
        handeyeTransform,
        cameraIntrinsics,
      );
      polygon = expandCubeRoiPolygon(polygon, cameraIntrinsics);
    } catch (error) {
      console.warn("Unable to project the tabletop ROI", error);
    }
  }
  const signature = polygon
    ? `${manualCubeRoi ? "manual" : "auto"}:${JSON.stringify(
      polygon.map(([x, y]) => [Math.round(x), Math.round(y)]),
    )}`
    : "none";
  if (signature === cubeConfigSignature) return;
  cubeConfigSignature = signature;
  cubePoseFilters.clear();
  cubePoseFilterLastSeen.clear();
  selectedCubeCandidateId = null;
  legacyCubeCandidateCenters.clear();
  nextLegacyCubeCandidateId = 1;
  cubeTableRoiActive = Boolean(polygon);
  cubeBackgroundState = "missing";
  cubeBackgroundRemaining = 0;
  updateCubeBackgroundUi();
  depthChannel.send(JSON.stringify({
    type: "cube_config",
    table_polygon_px: polygon,
  }));
}

function requestCubeBackgroundCapture() {
  if (!depthChannel || depthChannel.readyState !== "open") {
    cubeBackgroundState = "missing";
    updateCubeBackgroundUi();
    if (elements.cubeMetric) {
      setCubeMetricUnavailable("请先点击“开启”连接 D455 RGB 数据通道", "idle");
    }
    return;
  }
  if (cubeBackgroundRequestPending || cubeBackgroundState === "capturing") return;
  cubeBackgroundRequestPending = true;
  cubeBackgroundState = "capturing";
  cubeBackgroundRemaining = 24;
  cubeBackgroundTarget = 24;
  updateCubeBackgroundUi();
  depthChannel.send(JSON.stringify({
    type: "cube_background_capture",
    id: `cube-background-${Date.now()}`,
    frames: 24,
  }));
}

function cubeStatusLabel(status) {
  if (status === "tracking") return "跟踪";
  if (status === "occluded") return "短暂遮挡";
  if (status === "ignored") return "背景忽略";
  if (status === "background_missing") return "待采集空背景";
  if (status === "background_capturing") return "采集空背景";
  if (status === "background_confirming") return "确认中";
  return "丢失";
}

function setCubeMetricUnavailable(title, state = "idle") {
  const metric = elements.cubeMetric.closest(".cube-status");
  metric.classList.remove("has-pose");
  metric.dataset.state = state;
  metric.title = title;
  metric.setAttribute("aria-label", title);
  elements.cubeColorSwatch.hidden = true;
  elements.cubeColorSwatch.removeAttribute("data-color");
  elements.cubeCandidateMetric.hidden = true;
  elements.cubePositionIcon.hidden = true;
  elements.cubeMetric.hidden = true;
  elements.cubeMetric.textContent = "";
}

function cubePoseWithinTable(pose, table, expectedSizeM = 0.03) {
  const position = table?.position_mm?.map((value) => Number(value) / 1000);
  const size = table?.size_mm?.map((value) => Number(value) / 1000);
  const yaw = Number(table?.yaw_deg) * Math.PI / 180;
  if (!Array.isArray(position) || position.length < 2 || !position.every(Number.isFinite)
      || !Array.isArray(size) || size.length < 2 || !size.every(Number.isFinite)
      || !Number.isFinite(yaw)) return true;
  const dx = pose.base_xyz_m[0] - position[0];
  const dy = pose.base_xyz_m[1] - position[1];
  const cosine = Math.cos(yaw);
  const sine = Math.sin(yaw);
  const localX = cosine * dx + sine * dy;
  const localY = -sine * dx + cosine * dy;
  const sizeEstimate = Number(pose.size_estimate_m);
  const expected = Number(expectedSizeM);
  const sizeIsValid = !Number.isFinite(sizeEstimate)
    || !Number.isFinite(expected)
    || (sizeEstimate >= expected * 0.55 && sizeEstimate <= expected * 1.55);
  return Math.abs(localX) <= size[0] / 2 + 0.045
    && Math.abs(localY) <= size[1] / 2 + 0.045
    && sizeIsValid;
}

function drawCubeWorkArea(targetContext, width, height, { dimOutside = true } = {}) {
  if (!cubeRoiOverlayVisible && !cubeRoiEditing) return;
  const normalized = cubeRoiEditing ? cubeRoiDraft : manualCubeRoi;
  if (!Array.isArray(normalized) || normalized.length === 0) return;
  const points = normalized.map(([x, y]) => [x * width, y * height]);
  targetContext.save();
  if (points.length >= 3) {
    targetContext.beginPath();
    if (dimOutside) targetContext.rect(0, 0, width, height);
    targetContext.moveTo(points[0][0], points[0][1]);
    points.slice(1).forEach(([x, y]) => targetContext.lineTo(x, y));
    targetContext.closePath();
    targetContext.fillStyle = dimOutside
      ? (cubeRoiEditing ? "rgba(0, 0, 0, 0.28)" : "rgba(0, 0, 0, 0.42)")
      : "rgba(118, 240, 192, 0.14)";
    targetContext.fill(dimOutside ? "evenodd" : "nonzero");
  }
  targetContext.beginPath();
  targetContext.moveTo(points[0][0], points[0][1]);
  points.slice(1).forEach(([x, y]) => targetContext.lineTo(x, y));
  if (!cubeRoiEditing && points.length >= 3) targetContext.closePath();
  targetContext.strokeStyle = cubeRoiEditing ? "#f5c46b" : "#76f0c0";
  targetContext.lineWidth = cubeRoiEditing ? 2 : 2.5;
  targetContext.setLineDash(cubeRoiEditing ? [7, 5] : []);
  targetContext.stroke();
  targetContext.setLineDash([]);
  points.forEach(([x, y], index) => {
    targetContext.beginPath();
    targetContext.arc(x, y, cubeRoiEditing ? 7 : index === 0 ? 5 : 4, 0, Math.PI * 2);
    targetContext.fillStyle = index === 0 ? "#fff2a8" : "#76f0c0";
    targetContext.fill();
    if (cubeRoiEditing) {
      targetContext.lineWidth = 1.5;
      targetContext.strokeStyle = "#1c251f";
      targetContext.stroke();
    }
  });
  targetContext.restore();
}

function projectedTableEdges() {
  if (!cameraIntrinsics || !handeyeTransform) return null;
  const table = window.arx5GetSceneCalibration?.()?.table;
  if (!table) return null;
  try {
    const projection = projectTableCalibrationEdges(
      table,
      handeyeTransform,
      cameraIntrinsics,
    );
    if (!Array.isArray(projection?.polygon) || projection.polygon.length < 3
        || !Array.isArray(projection.top) || projection.top.length !== 2
        || !Array.isArray(projection.side) || projection.side.length !== 2) return null;
    return projection;
  } catch {
    return null;
  }
}

function drawProjectedTablePlane() {
  if (!tableEdgeEditing) return;
  const projection = projectedTableEdges();
  if (!projection) return;
  cubeContext.save();
  cubeContext.beginPath();
  projection.polygon.forEach(([x, y], index) => {
    if (index === 0) cubeContext.moveTo(x, y);
    else cubeContext.lineTo(x, y);
  });
  cubeContext.closePath();
  cubeContext.fillStyle = "rgba(84, 217, 232, .09)";
  cubeContext.fill();
  cubeContext.strokeStyle = "#54d9e8";
  cubeContext.fillStyle = "#8deaf3";
  cubeContext.lineWidth = 2;
  cubeContext.setLineDash([4, 4]);
  for (const line of [projection.top, projection.side]) {
    cubeContext.beginPath();
    cubeContext.moveTo(line[0][0], line[0][1]);
    cubeContext.lineTo(line[1][0], line[1][1]);
    cubeContext.stroke();
  }
  const extension = tableCornerExtensionPixels();
  const topExtended = extendImageLine(projection.top, extension, extension);
  const sideExtended = extendImageLine(projection.side, extension, extension);
  cubeContext.setLineDash([2, 5]);
  for (const line of [
    [topExtended[0], projection.top[0]],
    [projection.top[1], topExtended[1]],
    [sideExtended[0], projection.side[0]],
    [projection.side[1], sideExtended[1]],
  ]) {
    cubeContext.beginPath();
    cubeContext.moveTo(line[0][0], line[0][1]);
    cubeContext.lineTo(line[1][0], line[1][1]);
    cubeContext.stroke();
  }
  cubeContext.setLineDash([]);
  cubeContext.font = "600 10px system-ui, sans-serif";
  cubeContext.fillText(
    "3D 桌面投影",
    projection.top[0][0],
    projection.top[0][1] - 7,
  );
  cubeContext.restore();
}

function drawTableEdgeGuides() {
  if (!tableEdgeEditing) return;
  const normalized = tableEdgeDraft;
  if (!Array.isArray(normalized) || normalized.length === 0) return;
  const width = elements.cubeCanvas.width;
  const height = elements.cubeCanvas.height;
  const points = normalized.map(([x, y]) => [x * width, y * height]);
  cubeContext.save();
  cubeContext.strokeStyle = "#f5c46b";
  cubeContext.fillStyle = "#fff2a8";
  cubeContext.lineWidth = tableEdgeEditing ? 2 : 2.5;
  cubeContext.setLineDash(tableEdgeEditing ? [7, 5] : []);
  cubeContext.beginPath();
  cubeContext.moveTo(points[0][0], points[0][1]);
  points.slice(1).forEach(([x, y]) => cubeContext.lineTo(x, y));
  cubeContext.stroke();
  if (points.length >= 3) {
    const extension = tableCornerExtensionPixels();
    const top = [points[0], points[1]];
    const side = [points[1], points[2]];
    const topExtended = extendImageLine(top, extension, extension);
    const sideExtended = extendImageLine(side, extension, extension);
    cubeContext.setLineDash([3, 5]);
    for (const line of [
      [topExtended[0], top[0]],
      [points[1], topExtended[1]],
      [sideExtended[0], points[1]],
      [side[1], sideExtended[1]],
    ]) {
      cubeContext.beginPath();
      cubeContext.moveTo(line[0][0], line[0][1]);
      cubeContext.lineTo(line[1][0], line[1][1]);
      cubeContext.stroke();
    }
  }
  cubeContext.setLineDash([]);
  points.forEach(([x, y], index) => {
    cubeContext.beginPath();
    cubeContext.arc(x, y, tableEdgeEditing ? 7 : 4.5, 0, Math.PI * 2);
    cubeContext.fillStyle = index === 1 ? "#fff" : "#fff2a8";
    cubeContext.fill();
    cubeContext.strokeStyle = "#241b0b";
    cubeContext.lineWidth = 1.5;
    cubeContext.stroke();
  });
  cubeContext.font = "600 11px system-ui, sans-serif";
  cubeContext.textBaseline = "bottom";
  cubeContext.fillStyle = "#fff2a8";
  if (points.length >= 2) {
    cubeContext.fillText("桌面远边", (points[0][0] + points[1][0]) / 2, (points[0][1] + points[1][1]) / 2 - 6);
  }
  if (points.length >= 3) {
    cubeContext.fillText("桌面右边", (points[1][0] + points[2][0]) / 2 + 6, (points[1][1] + points[2][1]) / 2);
  }
  cubeContext.restore();
}

function drawCubeRoiMagnifier() {
  const point = tableEdgeEditing && tableEdgeDragIndex !== null
    ? tableEdgeDraft[tableEdgeDragIndex]
    : cubeRoiEditing && cubeRoiDragIndex !== null
      ? cubeRoiDraft[cubeRoiDragIndex]
      : null;
  if (!Array.isArray(point)) return;
  const width = elements.cubeCanvas.width;
  const height = elements.cubeCanvas.height;
  if (width < 1 || height < 1) return;

  const pointX = point[0] * width;
  const pointY = point[1] * height;
  const radius = Math.max(72, Math.min(108, Math.min(width, height) * 0.2));
  const offset = radius + 28;
  const lensX = pointX + offset + radius <= width ? pointX + offset : pointX - offset;
  const lensY = pointY - offset - radius >= 0 ? pointY - offset : pointY + offset;
  const centerX = Math.min(width - radius - 4, Math.max(radius + 4, lensX));
  const centerY = Math.min(height - radius - 4, Math.max(radius + 4, lensY));
  const zoom = 3.2;
  const sourceSize = radius * 2 / zoom;
  const sourceX = Math.min(width - sourceSize, Math.max(0, pointX - sourceSize / 2));
  const sourceY = Math.min(height - sourceSize, Math.max(0, pointY - sourceSize / 2));
  const markerX = centerX - radius + (pointX - sourceX) / sourceSize * radius * 2;
  const markerY = centerY - radius + (pointY - sourceY) / sourceSize * radius * 2;
  const tableProjection = tableEdgeEditing ? projectedTableEdges() : null;
  let edgeDistanceLabel = "";
  if (tableProjection && tableEdgeDraft.length === 3) {
    const corner = tableEdgeDraft[1].map((value, index) => value * [width, height][index]);
    const topDistance = pointToImageLineDistance(corner, tableProjection.top);
    const sideDistance = pointToImageLineDistance(corner, tableProjection.side);
    edgeDistanceLabel = `远 ${topDistance.toFixed(1)}px · 右 ${sideDistance.toFixed(1)}px`;
  } else if (tableEdgeEditing && !tableProjection) {
    edgeDistanceLabel = "3D 投影未就绪";
  }

  cubeContext.save();
  cubeContext.beginPath();
  cubeContext.moveTo(pointX, pointY);
  cubeContext.lineTo(centerX, centerY);
  cubeContext.strokeStyle = "rgba(255, 242, 168, .72)";
  cubeContext.lineWidth = 1.5;
  cubeContext.setLineDash([4, 4]);
  cubeContext.stroke();
  cubeContext.setLineDash([]);

  cubeContext.beginPath();
  cubeContext.arc(centerX, centerY, radius + 4, 0, Math.PI * 2);
  cubeContext.fillStyle = "rgba(2, 5, 4, .9)";
  cubeContext.fill();
  cubeContext.save();
  cubeContext.beginPath();
  cubeContext.arc(centerX, centerY, radius, 0, Math.PI * 2);
  cubeContext.clip();
  try {
    cubeContext.drawImage(
      elements.rgbVideo,
      sourceX,
      sourceY,
      sourceSize,
      sourceSize,
      centerX - radius,
      centerY - radius,
      radius * 2,
      radius * 2,
    );
  } catch {
    cubeContext.fillStyle = "#07100d";
    cubeContext.fillRect(centerX - radius, centerY - radius, radius * 2, radius * 2);
  }
  const lensPoint = ([x, y]) => [
    centerX - radius + (x - sourceX) / sourceSize * radius * 2,
    centerY - radius + (y - sourceY) / sourceSize * radius * 2,
  ];
  const drawLensLine = (line, color, lineWidth, dash = []) => {
    if (!Array.isArray(line) || line.length !== 2) return;
    const [start, end] = line.map(lensPoint);
    cubeContext.beginPath();
    cubeContext.moveTo(start[0], start[1]);
    cubeContext.lineTo(end[0], end[1]);
    cubeContext.strokeStyle = color;
    cubeContext.lineWidth = lineWidth;
    cubeContext.setLineDash(dash);
    cubeContext.stroke();
    cubeContext.setLineDash([]);
  };
  if (tableEdgeEditing) {
    const extension = width + height;
    if (tableProjection) {
      drawLensLine(
        extendImageLine(tableProjection.top, 0, extension),
        "#54d9e8",
        2,
        [4, 3],
      );
      drawLensLine(
        extendImageLine(tableProjection.side, extension, 0),
        "#54d9e8",
        2,
        [4, 3],
      );
    }
    if (tableEdgeDraft.length >= 2) {
      const draftPixels = tableEdgeDraft.map(([x, y]) => [x * width, y * height]);
      drawLensLine(
        extendImageLine([draftPixels[0], draftPixels[1]], 0, extension),
        "#f5c46b",
        3,
      );
      if (draftPixels.length >= 3) {
        drawLensLine(
          extendImageLine([draftPixels[1], draftPixels[2]], extension, 0),
          "#f5c46b",
          3,
        );
      }
      if (tableProjection && draftPixels.length >= 3) {
        const corner = draftPixels[1];
        for (const line of [tableProjection.top, tableProjection.side]) {
          const nearest = closestPointOnImageLine(corner, line);
          if (nearest) drawLensLine([corner, nearest], "#fff", 1.5, [3, 3]);
        }
      }
    }
  } else if (cubeRoiEditing && cubeRoiDragIndex !== null && cubeRoiDraft.length >= 2) {
    const extension = width + height;
    const draftPixels = cubeRoiDraft.map(([x, y]) => [x * width, y * height]);
    const previousIndex = (
      cubeRoiDragIndex + draftPixels.length - 1
    ) % draftPixels.length;
    const nextIndex = (cubeRoiDragIndex + 1) % draftPixels.length;
    for (const neighborIndex of new Set([previousIndex, nextIndex])) {
      if (neighborIndex === cubeRoiDragIndex) continue;
      drawLensLine(
        extendImageLine(
          [draftPixels[neighborIndex], draftPixels[cubeRoiDragIndex]],
          extension,
          extension,
        ),
        "#f5c46b",
        3,
      );
    }
  }
  cubeContext.restore();

  cubeContext.beginPath();
  cubeContext.arc(centerX, centerY, radius, 0, Math.PI * 2);
  cubeContext.strokeStyle = "#fff2a8";
  cubeContext.lineWidth = 3;
  cubeContext.stroke();
  cubeContext.beginPath();
  cubeContext.moveTo(markerX - 11, markerY);
  cubeContext.lineTo(markerX + 11, markerY);
  cubeContext.moveTo(markerX, markerY - 11);
  cubeContext.lineTo(markerX, markerY + 11);
  cubeContext.strokeStyle = "#76f0c0";
  cubeContext.lineWidth = 1.5;
  cubeContext.stroke();
  cubeContext.beginPath();
  cubeContext.arc(markerX, markerY, 3, 0, Math.PI * 2);
  cubeContext.fillStyle = "#fff";
  cubeContext.fill();

  cubeContext.font = "600 11px system-ui, sans-serif";
  cubeContext.textAlign = "center";
  cubeContext.textBaseline = "middle";
  const lensLabel = edgeDistanceLabel || "3.2×";
  const labelWidth = Math.min(radius * 1.8, Math.max(36, cubeContext.measureText(lensLabel).width + 12));
  cubeContext.fillStyle = "#07100d";
  cubeContext.fillRect(centerX - labelWidth / 2, centerY + radius - 16, labelWidth, 18);
  cubeContext.fillStyle = "#fff2a8";
  cubeContext.fillText(lensLabel, centerX, centerY + radius - 7);
  cubeContext.restore();
}

function drawDepthCubeOverlay(detection) {
  const width = elements.depthVideo.videoWidth || elements.rgbVideo.videoWidth || 640;
  const height = elements.depthVideo.videoHeight || elements.rgbVideo.videoHeight || 480;
  if (elements.depthCubeCanvas.width !== width || elements.depthCubeCanvas.height !== height) {
    elements.depthCubeCanvas.width = width;
    elements.depthCubeCanvas.height = height;
  }
  depthCubeContext.clearRect(0, 0, width, height);
  drawCubeWorkArea(depthCubeContext, width, height, { dimOutside: true });
  const candidates = Array.isArray(detection?.candidates) ? detection.candidates : [];
  depthCubeContext.save();
  candidates.forEach((candidate) => {
    if (!Array.isArray(candidate?.center_px)
        || !["tracking", "occluded", "ignored"].includes(candidate.status)) return;
    const selected = candidate.selected === true;
    depthCubeContext.globalAlpha = selected ? 0.96 : 0.56;
    depthCubeContext.strokeStyle = selected ? "#76f0c0" : "#54d9e8";
    depthCubeContext.lineWidth = selected ? 2.4 : 1.4;
    depthCubeContext.setLineDash(selected ? [] : [6, 5]);
    if (Array.isArray(candidate.corners_px) && candidate.corners_px.length === 4) {
      depthCubeContext.beginPath();
      candidate.corners_px.forEach(([x, y], index) => {
        if (index === 0) depthCubeContext.moveTo(x, y);
        else depthCubeContext.lineTo(x, y);
      });
      depthCubeContext.closePath();
      depthCubeContext.stroke();
    }
    const [x, y] = candidate.center_px.map(Number);
    depthCubeContext.setLineDash([]);
    depthCubeContext.beginPath();
    depthCubeContext.arc(x, y, selected ? 5 : 4, 0, Math.PI * 2);
    depthCubeContext.stroke();
  });
  depthCubeContext.restore();
}

function drawCubeDetectionBase(detection) {
  const width = elements.rgbVideo.videoWidth || 640;
  const height = elements.rgbVideo.videoHeight || 480;
  if (elements.cubeCanvas.width !== width || elements.cubeCanvas.height !== height) {
    elements.cubeCanvas.width = width;
    elements.cubeCanvas.height = height;
  }
  cubeContext.clearRect(0, 0, width, height);
  drawCubeWorkArea(cubeContext, width, height);
  drawProjectedTablePlane();
  drawTableEdgeGuides();
  drawDepthCubeOverlay(detection);
  const background = detection.background ?? {};
  const regions = Array.isArray(background.regions) ? background.regions : [];
  cubeContext.save();
  cubeContext.lineWidth = 1.5;
  cubeContext.font = "12px system-ui, sans-serif";
  cubeContext.textBaseline = "top";
  regions.forEach((region) => {
    const bbox = region.bbox_px;
    if (!Array.isArray(bbox) || bbox.length !== 4) return;
    const [x, y, boxWidth, boxHeight] = bbox.map(Number);
    if (![x, y, boxWidth, boxHeight].every(Number.isFinite)) return;
    const selected = region.selected === true;
    const accepted = region.accepted === true;
    cubeContext.strokeStyle = selected ? "#fff2a8" : accepted ? "#54d9e8" : "#a06a73";
    cubeContext.lineWidth = selected ? 2.5 : 1.5;
    cubeContext.setLineDash(selected ? [] : [5, 4]);
    cubeContext.strokeRect(x, y, boxWidth, boxHeight);
    cubeContext.setLineDash([]);
    const label = selected
      ? `BG · selected ${Number(region.score ?? 0).toFixed(2)}`
      : accepted
        ? `BG · geom ${Number(region.score ?? 0).toFixed(2)}`
        : `BG · ${region.rejection ?? "reject"}`;
    cubeContext.fillStyle = selected ? "#fff2a8" : accepted ? "#54d9e8" : "#d28b95";
    cubeContext.fillText(label, Math.max(2, x), Math.max(2, y - 15));
  });
  const candidates = Array.isArray(detection.candidates) ? detection.candidates : [];
  if (candidates.length > 0) {
    candidates.forEach((candidate, index) => {
      if (!Array.isArray(candidate.center_px)
          || !["tracking", "occluded", "ignored"].includes(candidate.status)) return;
      const selected = candidate.selected === true;
      const color = selected ? "#76f0c0" : "#54d9e8";
      cubeContext.save();
      cubeContext.globalAlpha = selected ? 1 : 0.58;
      cubeContext.strokeStyle = color;
      cubeContext.fillStyle = color;
      cubeContext.lineWidth = selected ? 2.5 : 1.5;
      cubeContext.setLineDash(selected ? [] : [6, 5]);
      if (Array.isArray(candidate.corners_px) && candidate.corners_px.length === 4) {
        cubeContext.beginPath();
        candidate.corners_px.forEach(([x, y], cornerIndex) => {
          if (cornerIndex === 0) cubeContext.moveTo(x, y);
          else cubeContext.lineTo(x, y);
        });
        cubeContext.closePath();
        cubeContext.stroke();
      }
      const [centerX, centerY] = candidate.center_px.map(Number);
      cubeContext.setLineDash([]);
      cubeContext.beginPath();
      cubeContext.moveTo(centerX - 5, centerY);
      cubeContext.lineTo(centerX + 5, centerY);
      cubeContext.moveTo(centerX, centerY - 5);
      cubeContext.lineTo(centerX, centerY + 5);
      cubeContext.stroke();
      if (Array.isArray(candidate.axis_px)) {
        const length = Math.max(18, Number(candidate.size_px ?? 20) * 0.7);
        const endX = centerX + Number(candidate.axis_px[0]) * length;
        const endY = centerY + Number(candidate.axis_px[1]) * length;
        cubeContext.beginPath();
        cubeContext.moveTo(centerX, centerY);
        cubeContext.lineTo(endX, endY);
        cubeContext.stroke();
      }
      const colorName = { black: "黑", white: "白", blue: "蓝" }[String(candidate.color ?? "")];
      const colorPrefix = colorName ? `${colorName}色 ` : "";
      const label = selected
        ? `${colorPrefix}方块 ${index + 1} · 已选择`
        : `${colorPrefix}候选 ${index + 1}`;
      cubeContext.fillText(label, Math.max(2, centerX + 8), Math.max(2, centerY - 18));
      cubeContext.restore();
    });
    cubeContext.restore();
    return;
  }
  if (!Array.isArray(detection.center_px)
      || !["tracking", "occluded"].includes(detection.status)) {
    cubeContext.restore();
    return;
  }
  const color = detection.status === "tracking" ? "#76f0c0" : "#f5c46b";
  cubeContext.save();
  cubeContext.lineWidth = 2;
  cubeContext.strokeStyle = color;
  cubeContext.fillStyle = color;
  if (Array.isArray(detection.corners_px) && detection.corners_px.length === 4) {
    cubeContext.beginPath();
    detection.corners_px.forEach(([x, y], index) => {
      if (index === 0) cubeContext.moveTo(x, y);
      else cubeContext.lineTo(x, y);
    });
    cubeContext.closePath();
    cubeContext.stroke();
  }
  const [centerX, centerY] = detection.center_px;
  cubeContext.beginPath();
  cubeContext.moveTo(centerX - 5, centerY);
  cubeContext.lineTo(centerX + 5, centerY);
  cubeContext.moveTo(centerX, centerY - 5);
  cubeContext.lineTo(centerX, centerY + 5);
  cubeContext.stroke();
  if (Array.isArray(detection.axis_px)) {
    const length = Math.max(18, Number(detection.size_px ?? 20) * 0.7);
    const endX = centerX + detection.axis_px[0] * length;
    const endY = centerY + detection.axis_px[1] * length;
    cubeContext.beginPath();
    cubeContext.moveTo(centerX, centerY);
    cubeContext.lineTo(endX, endY);
    cubeContext.stroke();
    cubeContext.beginPath();
    cubeContext.arc(endX, endY, 3, 0, Math.PI * 2);
    cubeContext.fill();
  }
  cubeContext.restore();
  cubeContext.restore();
}

function drawCubeDetection(detection) {
  drawCubeDetectionBase(detection);
  drawCubeRoiMagnifier();
}

function cubeFilter(candidateId) {
  if (!cubePoseFilters.has(candidateId)) {
    cubePoseFilters.set(candidateId, new CubePoseFilter());
  }
  cubePoseFilterLastSeen.set(candidateId, performance.now());
  return cubePoseFilters.get(candidateId);
}

function legacyCandidateId(center) {
  const point = center.map(Number);
  let bestId = null;
  let bestDistance = Infinity;
  for (const [candidateId, previous] of legacyCubeCandidateCenters) {
    const distance = Math.hypot(point[0] - previous[0], point[1] - previous[1]);
    if (distance <= 36 && distance < bestDistance) {
      bestId = candidateId;
      bestDistance = distance;
    }
  }
  const candidateId = bestId ?? `cube-legacy-${nextLegacyCubeCandidateId++}`;
  legacyCubeCandidateCenters.set(candidateId, point);
  return candidateId;
}

function stabilizedCubeMetricSource(pose) {
  const nextSource = pose?.position_source === "rgb_plane+depth_average"
    ? "RGB-D 多帧"
    : "RGB ROI";
  if (cubeMetricDisplay.source === null) {
    cubeMetricDisplay.source = nextSource;
    return nextSource;
  }
  if (nextSource === cubeMetricDisplay.source) {
    cubeMetricDisplay.pendingSource = null;
    cubeMetricDisplay.pendingSourceFrames = 0;
    return cubeMetricDisplay.source;
  }
  if (cubeMetricDisplay.pendingSource !== nextSource) {
    cubeMetricDisplay.pendingSource = nextSource;
    cubeMetricDisplay.pendingSourceFrames = 1;
  } else {
    cubeMetricDisplay.pendingSourceFrames += 1;
  }
  if (cubeMetricDisplay.pendingSourceFrames >= 5) {
    cubeMetricDisplay.source = nextSource;
    cubeMetricDisplay.pendingSource = null;
    cubeMetricDisplay.pendingSourceFrames = 0;
  }
  return cubeMetricDisplay.source;
}

function stabilizedCubeMetricPosition(pose, candidateId) {
  const raw = pose?.base_xyz_m?.slice(0, 2).map((value) => Number(value) * 1000);
  if (!Array.isArray(raw) || !raw.every(Number.isFinite)) return null;
  const now = performance.now();
  const changedCandidate = cubeMetricDisplay.candidateId !== candidateId;
  const previous = cubeMetricDisplay.positionMm;
  if (changedCandidate || !previous
      || Math.hypot(raw[0] - previous[0], raw[1] - previous[1]) >= 10
      || now - cubeMetricDisplay.positionUpdatedAt >= 1000) {
    cubeMetricDisplay.positionMm = raw.map((value) => Math.round(value / 5) * 5);
    cubeMetricDisplay.positionUpdatedAt = now;
    cubeMetricDisplay.candidateId = candidateId;
  }
  return cubeMetricDisplay.positionMm;
}

function updateCubeMetric(detail, candidates) {
  const pose = detail.pose_base;
  const colorNames = { black: "黑色", white: "白色", blue: "蓝色" };
  const colorKey = String(detail.cube_color ?? detail.color ?? "");
  const colorLabel = colorNames[colorKey] ?? "";
  const candidateNumber = Math.max(
    1,
    candidates.findIndex((candidate) => candidate.selected) + 1,
  );
  const candidateLabel = candidates.length > 1
    ? `${candidateNumber}/${candidates.length}`
    : "";
  if (!pose) {
    const reason = detail.table_roi_active
      ? "桌面 ROI 内未检测到 30 mm 方块"
      : "桌面 ROI 尚未就绪";
    const metric = elements.cubeMetric.closest(".cube-status");
    const transientLoss = ["lost", "occluded"].includes(detail.status)
      && metric.classList.contains("has-pose")
      && performance.now() - cubeMetricDisplay.lastPoseAt < 1000;
    if (transientLoss) {
      metric.dataset.state = "occluded";
      const title = `短暂丢失；保留上次稳定坐标；${reason}`;
      metric.title = title;
      metric.setAttribute("aria-label", title);
      return;
    }
    setCubeMetricUnavailable(`${cubeStatusLabel(detail.status)}；${reason}`, detail.status);
    return;
  }
  const sourceLabel = stabilizedCubeMetricSource(pose);
  const positionMm = stabilizedCubeMetricPosition(pose, detail.selected_candidate_id ?? null);
  const metric = elements.cubeMetric.closest(".cube-status");
  metric.classList.add("has-pose");
  metric.dataset.state = detail.status;
  cubeMetricDisplay.lastPoseAt = performance.now();
  elements.cubeColorSwatch.hidden = !colorLabel;
  if (colorLabel) elements.cubeColorSwatch.dataset.color = colorKey;
  else elements.cubeColorSwatch.removeAttribute("data-color");
  elements.cubeCandidateMetric.hidden = !candidateLabel;
  elements.cubeCandidateMetric.textContent = candidateLabel;
  elements.cubePositionIcon.hidden = !positionMm;
  elements.cubeMetric.hidden = !positionMm;
  elements.cubeMetric.textContent = positionMm ? `${positionMm[0]},${positionMm[1]}` : "";
  const [xMm, yMm] = positionMm ?? [NaN, NaN];
  const title = `${colorLabel ? `${colorLabel}方块；` : ""}${sourceLabel}；${cubeStatusLabel(detail.status)}${candidateLabel ? `；候选 ${candidateLabel}` : ""}；置信度 ${(Number(detail.confidence ?? 0) * 100).toFixed(1)}%；BASE x ${Number.isFinite(xMm) ? xMm.toFixed(0) : "—"} mm，y ${Number.isFinite(yMm) ? yMm.toFixed(0) : "—"} mm，yaw ${Number(pose.yaw_deg).toFixed(1)}°；尺寸 30.0 mm${Number.isFinite(pose.depth_m) ? `；深度仅诊断 ${pose.depth_m.toFixed(3)} m` : ""}`;
  metric.title = title;
  metric.setAttribute("aria-label", title);
}

function handleCubeDetection(response) {
  const now = performance.now();
  const detail = { ...response };
  latestCubeResponse = response;
  cameraIntrinsics = detail.intrinsics ?? cameraIntrinsics;
  syncCubeDetectionConfig();
  const sceneCalibration = window.arx5GetSceneCalibration?.();
  let rawCandidates = Array.isArray(response.candidates)
    ? response.candidates.filter((candidate) => Array.isArray(candidate?.center_px))
    : [];
  if (rawCandidates.length === 0 && Array.isArray(detail.color?.candidates)) {
    rawCandidates = detail.color.candidates
      .filter((candidate) => candidate?.accepted === true && Array.isArray(candidate.center_px))
      .map((candidate) => {
        const center = candidate.center_px.map(Number);
        const bbox = Array.isArray(candidate.bbox_px) && candidate.bbox_px.length === 4
          ? candidate.bbox_px.map(Number)
          : [center[0] - 15, center[1] - 15, 30, 30];
        const [x, y, width, height] = bbox;
        return {
          candidate_id: legacyCandidateId(center),
          selected: candidate.selected === true,
          status: "tracking",
          confidence: Math.min(0.96, 0.78 + Number(candidate.score ?? 0) * 0.18),
          timestamp: detail.timestamp,
          center_px: center,
          corners_px: [[x, y], [x + width, y], [x + width, y + height], [x, y + height]],
          axis_px: [1, 0],
          size_px: Math.sqrt(Math.max(1, Number(candidate.area_px ?? width * height))),
          color: candidate.color ?? null,
          depth_sample: candidate.selected === true ? detail.depth_sample : null,
        };
      });
  }
  if (rawCandidates.length === 0 && detail.status !== "lost" && Array.isArray(detail.center_px)) {
    rawCandidates = [{ ...detail, candidate_id: "cube-primary", selected: true }];
  }
  if (manualCubeRoi) rawCandidates = rawCandidates.filter(cubeCandidateInManualRoi);
  // Keep each pose filter across short detector gaps.  Deleting it on every
  // missed frame makes a returning cube jump from a fresh raw pose and reads
  // as visual flicker in the simulator.
  const candidates = rawCandidates.map((rawCandidate) => {
    const candidate = {
      ...rawCandidate,
      candidate_id: String(rawCandidate.candidate_id),
      intrinsics: rawCandidate.intrinsics ?? detail.intrinsics,
      cube_size_m: Number(rawCandidate.cube_size_m ?? detail.cube_size_m ?? 0.03),
    };
    try {
      candidate.pose_base = estimateCubePose(
        candidate,
        handeyeTransform,
        Number(sceneCalibration?.table?.position_mm?.[2]) / 1000,
      );
      if (!cubePoseWithinTable(
        candidate.pose_base,
        sceneCalibration?.table,
        candidate.cube_size_m,
      )) {
        candidate.status = "ignored";
        candidate.pose_error = "桌面 ROI 外的候选已忽略";
        // Keep the estimate for the 3D diagnostic view.  It remains
        // `ignored`, so cube picking stays disabled, but clearing the pose
        // here made a calibration/table-ROI mismatch look like no detection.
      } else {
        candidate.pose_base = cubeFilter(candidate.candidate_id).update(
          candidate.pose_base,
          candidate.status,
        );
      }
    } catch (error) {
      candidate.pose_error = error.message;
      candidate.pose_base = null;
    }
    return candidate;
  });
  for (const [candidateId, lastSeen] of cubePoseFilterLastSeen) {
    if (now - lastSeen > 5000) {
      cubePoseFilterLastSeen.delete(candidateId);
      cubePoseFilters.delete(candidateId);
    }
  }
  if (candidates.length > 0
      && !candidates.some((candidate) => candidate.candidate_id === selectedCubeCandidateId)) {
    selectedCubeCandidateId = candidates.find(
      (candidate) => candidate.candidate_id === response.selected_candidate_id,
    )?.candidate_id ?? candidates.find((candidate) => candidate.status !== "ignored")?.candidate_id ?? null;
  }
  candidates.forEach((candidate) => {
    candidate.selected = candidate.candidate_id === selectedCubeCandidateId;
  });
  const selected = candidates.find((candidate) => candidate.selected) ?? null;
  latestCubeCandidates = candidates;
  detail.candidates = candidates;
  detail.selected_candidate_id = selected?.candidate_id ?? null;
  if (selected) {
    for (const key of [
      "status", "confidence", "timestamp", "center_px", "corners_px", "axis_px",
      "size_px", "color", "depth_sample", "pose_base", "pose_error",
    ]) {
      if (key in selected) detail[key] = selected[key];
      else delete detail[key];
    }
  } else {
    detail.status = "lost";
    detail.pose_base = null;
  }
  // Keep filters through a short all-candidate gap; the next frame can then
  // resume from the last stable pose instead of reappearing at a raw jump.
  const background = detail.background;
  if (background) updateCubeBackgroundUi(background);
  cubeTableRoiActive = detail.table_roi_active === true || cubeTableRoiActive;
  updateCubeBackgroundUi();
  updateCubeMetric(detail, candidates);
  latestCubeDisplay = detail;
  drawCubeDetection(detail);
  window.dispatchEvent(new CustomEvent("arx5-cube-detection", { detail }));
}

async function startWebRtcCameraWithRecovery(generation) {
  let lastError;
  for (let attempt = 1; attempt <= CAMERA_RECOVERY_ATTEMPTS; attempt += 1) {
    try {
      cameraWebRtcAttemptInFlight = true;
      await startWebRtcCamera(generation);
      return;
    } catch (error) {
      lastError = error;
      // This closes only the failed signaling/ICE/RTP peer. The camera server
      // keeps both RealSense pipelines alive for its 12 s idle grace window,
      // while this retry starts in 900 ms; no UVC release or camera restart is
      // requested here.
      stopCameraTransport();
      const retry = recoverableCameraError(error)
        && attempt < CAMERA_RECOVERY_ATTEMPTS
        && generation === cameraGeneration
        && cameraActive;
      if (!retry) throw error;
      setPill(elements.cameraStatus, "WebRTC 视频通道自动恢复中", "warning");
      setEmptyStateVisible(true);
      elements.emptyState.classList.remove("error");
      elements.emptyState.textContent = `正在仅重建 WebRTC 媒体通道（${attempt + 1}/${CAMERA_RECOVERY_ATTEMPTS}）；D455/D405 不重启…`;
      await waitForCameraRecovery(generation);
    } finally {
      cameraWebRtcAttemptInFlight = false;
    }
  }
  throw lastError;
}

async function startCamera() {
  if (cameraActive) return;
  smoothedPoseLatencyMs = null;
  elements.poseLatency.textContent = "—";
  const generation = cameraGeneration + 1;
  cameraGeneration = generation;
  cameraActive = true;
  setCameraToggle(true, true);
  updatePoseModeUi(false);
  setEmptyStateVisible(true);
  elements.emptyState.classList.remove("error");
  elements.emptyState.textContent = "正在连接 D455 RGB-D 与眼在手 D405…";
  stopCameraTransport();
  elements.canvas.width = 640;
  elements.canvas.height = 480;
  try {
    await startWebRtcCameraWithRecovery(generation);
    if (generation !== cameraGeneration || !cameraActive) return;
    setEmptyStateVisible(false);
    if (cameraDepthAvailable === null) {
      setPill(elements.cameraStatus, "D455 RGB + D405 运行中", "ok");
    }
  } catch (error) {
    console.error("RealSense WebRTC unavailable", error);
    showCameraError(cameraErrorMessage(error), generation);
    return;
  }
  setCameraToggle(true);
  updatePoseModeUi(false);
  if (poseRecognitionEnabled) await startPoseRecognition();
  else stopPoseRecognition();
}

function stopCamera() {
  running = false;
  stopRenderLoop();
  smoothedPoseLatencyMs = null;
  elements.poseLatency.textContent = "—";
  poseStabilizer.reset();
  cameraActive = false;
  cameraGeneration += 1;
  stopCameraTransport();
  context.clearRect(0, 0, elements.canvas.width, elements.canvas.height);
  setCameraToggle(false);
  updatePoseModeUi(false);
  setEmptyStateVisible(true);
  elements.emptyState.classList.remove("error");
  elements.emptyState.textContent = "点击“启动摄像头”连接 D455";
  setPill(elements.cameraStatus, "相机已停止", "pending");
}

function selectHand(handResult) {
  const requested = selectedSide();
  for (let i = 0; i < handResult.landmarks.length; i += 1) {
    const category = handResult.handedness[i]?.[0];
    if (category?.categoryName?.toLowerCase() === requested) {
      return {
        landmarks: handResult.landmarks[i],
        world: handResult.worldLandmarks[i],
        handedness: category.categoryName,
        score: category.score,
      };
    }
  }
  return null;
}

function sendResults(now, poseResult, handResult) {
  if (!websocket || websocket.readyState !== WebSocket.OPEN) {
    ensureBridgeConnected();
    return;
  }
  if (now - lastSentAt < SEND_INTERVAL_MS) return;
  const pose = poseResult.landmarks[0] ?? null;
  const poseWorld = poseResult.worldLandmarks[0] ?? null;
  const selectedHand = selectHand(handResult);
  if (!pose && !selectedHand) return;

  sequence += 1;
  const payload = {
    version: 1,
    type: "landmarks",
    seq: sequence,
    client_time_ms: Date.now(),
    selected_side: selectedSide(),
    pose: pose ? serializeLandmarks(pose) : null,
    pose_world: poseWorld ? serializeLandmarks(poseWorld) : null,
    hand: selectedHand ? serializeLandmarks(selectedHand.landmarks) : null,
    hand_world: selectedHand?.world
      ? serializeLandmarks(selectedHand.world)
      : null,
    handedness: selectedHand?.handedness ?? null,
    handedness_score: selectedHand?.score ?? null,
    pinch_ratio: selectedHand ? pinchRatio(selectedHand.landmarks) : null,
  };
  websocket.send(JSON.stringify(payload));
  lastSentAt = now;
}

function drawResults(poseResult, handResult) {
  context.clearRect(0, 0, elements.canvas.width, elements.canvas.height);
  const upperLimbIndices = UPPER_LIMB_LANDMARKS[selectedSide()];
  for (const landmarks of poseResult.landmarks) {
    const upperLimb = upperLimbIndices
      .map((index) => landmarks[index])
      .filter(Boolean);
    drawing.drawLandmarks(upperLimb, { color: "#f4fffb", radius: 3 });
  }
  const selectedHand = selectHand(handResult);
  if (selectedHand) {
    drawing.drawConnectors(
      selectedHand.landmarks,
      HandLandmarker.HAND_CONNECTIONS ?? [],
      { color: "rgba(255, 207, 116, .72)", lineWidth: 2 },
    );
    drawing.drawLandmarks(selectedHand.landmarks, { color: "#fff2d5", radius: 3 });
  }
}

function updateMetrics(now, poseResult, handResult) {
  framesSinceReport += 1;
  if (now - fpsReportAt >= 500) {
    const fps = (framesSinceReport * 1000) / (now - fpsReportAt);
    elements.fps.textContent = fps.toFixed(1);
    framesSinceReport = 0;
    fpsReportAt = now;
  }
  const pose = poseResult.landmarks[0];
  const selectedHand = selectHand(handResult);
  elements.poseMetric.textContent = pose ? "开" : "—";
  const handLabel = selectedHand
    ? (String(selectedHand.handedness).toLowerCase().startsWith("right") ? "R" : "L")
    : "—";
  elements.handMetric.textContent = handLabel;
  const pinch = selectedHand ? pinchRatio(selectedHand.landmarks) : null;
  elements.pinchMetric.textContent = pinch === null ? "—" : pinch.toFixed(2);
  const side = selectedSide() === "left" ? "左手" : "右手";
  setHandPoseBadge(
    selectedHand
      ? `已识别${side}${pinch === null ? "" : ` · 捏合 ${pinch.toFixed(2)}`}`
      : `正在查找${side}…`,
    selectedHand ? "detected" : "searching",
  );
  updateVisualRuntimeTitle();
}

function stopRenderLoop() {
  if (renderRequestId === null) return;
  if (renderRequestKind === "video" && elements.rgbVideo.cancelVideoFrameCallback) {
    elements.rgbVideo.cancelVideoFrameCallback(renderRequestId);
  } else if (renderRequestKind === "animation") {
    cancelAnimationFrame(renderRequestId);
  }
  renderRequestId = null;
  renderRequestKind = null;
}

function scheduleRenderLoop() {
  if (!running || renderRequestId !== null) return;
  if (elements.rgbVideo.requestVideoFrameCallback) {
    renderRequestKind = "video";
    renderRequestId = elements.rgbVideo.requestVideoFrameCallback(renderLoop);
  } else {
    renderRequestKind = "animation";
    renderRequestId = requestAnimationFrame(renderLoop);
  }
}

function renderLoop(now) {
  renderRequestId = null;
  renderRequestKind = null;
  if (!running) return;
  const source = elements.rgbVideo;
  const frameWidth = source.videoWidth;
  const frameHeight = source.videoHeight;
  if (frameWidth <= 0 || frameHeight <= 0) {
    scheduleRenderLoop();
    return;
  }
  if (elements.canvas.width !== frameWidth || elements.canvas.height !== frameHeight) {
    elements.canvas.width = frameWidth;
    elements.canvas.height = frameHeight;
  }
  if (now - lastFrameProcessedAt >= inferenceFrameIntervalMs()) {
    const timestamp = performance.now();
    const inferenceStartedAt = performance.now();
    const rawPoseResult = poseLandmarker.detectForVideo(source, timestamp);
    const poseResult = poseStabilizer.update(rawPoseResult, timestamp);
    const handResult = handLandmarker.detectForVideo(source, timestamp);
    const inferenceLatencyMs = performance.now() - inferenceStartedAt;
    smoothedInferenceLatencyMs = smoothedInferenceLatencyMs === null
      ? inferenceLatencyMs
      : smoothedInferenceLatencyMs * 0.8 + inferenceLatencyMs * 0.2;
    smoothedPoseLatencyMs = smoothedInferenceLatencyMs;
    elements.poseLatency.textContent = `${smoothedInferenceLatencyMs.toFixed(0)}ms`;
    drawResults(poseResult, handResult);
    updateMetrics(now, poseResult, handResult);
    sendResults(now, poseResult, handResult);
    lastFrameProcessedAt = now;
  }
  scheduleRenderLoop();
}

function ensureBridgeConnected() {
  if (websocket?.readyState === WebSocket.CONNECTING
      || websocket?.readyState === WebSocket.OPEN
      || performance.now() < bridgeReconnectAfter) return;
  bridgeStateLabel = "连接中";
  updateVisualRuntimeTitle();
  const socket = new WebSocket(elements.bridgeUrl.value.trim());
  websocket = socket;
  socket.addEventListener("open", () => {
    if (websocket !== socket) return;
    bridgeStateLabel = "已连接";
    updateVisualRuntimeTitle();
  });
  socket.addEventListener("message", (event) => {
    const ack = JSON.parse(event.data);
    if (ack.type !== "ack" || !Number.isInteger(ack.seq)) return;
    if (!ack.accepted) console.warn("bridge rejected frame", ack.error);
  });
  socket.addEventListener("close", () => {
    if (websocket !== socket) return;
    websocket = undefined;
    bridgeStateLabel = "未连接";
    bridgeReconnectAfter = performance.now() + 1500;
    updateVisualRuntimeTitle();
  });
  socket.addEventListener("error", () => {
    if (websocket !== socket) return;
    bridgeStateLabel = "错误";
    updateVisualRuntimeTitle();
  });
}

elements.cameraToggleButton.addEventListener("click", () => {
  if (cameraActive) stopCamera();
  else startCamera();
});
elements.cubeRoiButton?.addEventListener("click", () => {
  setCubeRoiEditing(!cubeRoiEditing);
});
elements.cubeRoiVisibilityButton?.addEventListener("click", () => {
  cubeRoiOverlayVisible = !cubeRoiOverlayVisible;
  saveCubeRoiOverlayVisibility();
  updateCubeRoiUi();
  redrawCubeOverlay();
});
elements.cubeRoiUndo?.addEventListener("click", () => {
  cubeRoiDraft.pop();
  updateCubeRoiUi();
  redrawCubeOverlay();
});
elements.cubeRoiCommit?.addEventListener("click", commitCubeRoi);
elements.cubeRoiCancel?.addEventListener("click", () => setCubeRoiEditing(false));
elements.cubeRoiClear?.addEventListener("click", clearManualCubeRoi);
elements.tableEdgeEditButton?.addEventListener("click", () => {
  setTableEdgeEditing(!tableEdgeEditing);
});
elements.tableEdgeUndo?.addEventListener("click", () => {
  tableEdgeDraft.pop();
  updateTableEdgeUi();
  redrawCubeOverlay();
});
elements.tableEdgeCommit?.addEventListener("click", commitTableEdgeGuides);
elements.tableEdgeCancel?.addEventListener("click", () => setTableEdgeEditing(false));
elements.tableEdgeClear?.addEventListener("click", clearTableEdgeGuides);
elements.cubeCanvas.addEventListener("pointerdown", (event) => {
  if (!cubeRoiEditing && !tableEdgeEditing) return;
  const point = rgbPointFromPointer(event);
  if (!point) return;
  const draft = tableEdgeEditing ? tableEdgeDraft : cubeRoiDraft;
  const vertex = annotationVertexNear(point, draft);
  if (vertex >= 0) {
    if (tableEdgeEditing) tableEdgeDragIndex = vertex;
    else cubeRoiDragIndex = vertex;
    elements.cubeCanvas.setPointerCapture?.(event.pointerId);
    updateCubeRoiUi();
    updateTableEdgeUi();
    redrawCubeOverlay();
    event.preventDefault();
    return;
  }
  if (tableEdgeEditing) {
    if (tableEdgeDraft.length >= 3) return;
    tableEdgeDraft.push(clampNormalizedPoint(point));
  } else {
    if (cubeRoiDraft.length >= 24) return;
    cubeRoiDraft.push(clampNormalizedPoint(point));
  }
  updateCubeRoiUi();
  updateTableEdgeUi();
  redrawCubeOverlay();
});
elements.cubeCanvas.addEventListener("pointermove", (event) => {
  if ((!cubeRoiEditing || cubeRoiDragIndex === null)
      && (!tableEdgeEditing || tableEdgeDragIndex === null)) return;
  const point = rgbPointFromPointer(event);
  if (!point) return;
  if (tableEdgeEditing) {
    tableEdgeDraft[tableEdgeDragIndex] = clampNormalizedPoint(point);
  } else {
    cubeRoiDraft[cubeRoiDragIndex] = clampNormalizedPoint(point);
  }
  updateCubeRoiUi();
  updateTableEdgeUi();
  redrawCubeOverlay();
  event.preventDefault();
});
function finishCubeRoiDrag(event) {
  if (cubeRoiDragIndex === null && tableEdgeDragIndex === null) return;
  cubeRoiDragIndex = null;
  tableEdgeDragIndex = null;
  if (event?.pointerId !== undefined && elements.cubeCanvas.hasPointerCapture?.(event.pointerId)) {
    elements.cubeCanvas.releasePointerCapture(event.pointerId);
  }
  updateCubeRoiUi();
  updateTableEdgeUi();
  redrawCubeOverlay();
}
elements.cubeCanvas.addEventListener("pointerup", finishCubeRoiDrag);
elements.cubeCanvas.addEventListener("pointercancel", finishCubeRoiDrag);

function cubeCandidateAtPoint(point) {
  const selectable = latestCubeCandidates.filter((candidate) => candidate.status !== "ignored");
  const inside = selectable.filter((candidate) => {
    const corners = candidate.corners_px;
    return Array.isArray(corners) && corners.length >= 3 && pointInPolygon(point, corners);
  });
  if (inside.length > 0) {
    return inside.reduce((nearest, candidate) => {
      const distance = Math.hypot(
        point[0] - Number(candidate.center_px?.[0]),
        point[1] - Number(candidate.center_px?.[1]),
      );
      return !nearest || distance < nearest.distance ? { candidate, distance } : nearest;
    }, null)?.candidate ?? null;
  }
  return selectable.reduce((nearest, candidate) => {
    const center = candidate.center_px?.map(Number);
    if (!Array.isArray(center) || center.length !== 2 || !center.every(Number.isFinite)) {
      return nearest;
    }
    const distance = Math.hypot(point[0] - center[0], point[1] - center[1]);
    const radius = Math.max(22, Math.min(60, Number(candidate.size_px || 30) * 0.8));
    return distance <= radius && (!nearest || distance < nearest.distance)
      ? { candidate, distance }
      : nearest;
  }, null)?.candidate ?? null;
}

function selectCubeCandidate(candidateId) {
  const id = String(candidateId ?? "");
  const candidate = latestCubeCandidates.find((item) => (
    item.candidate_id === id && item.status !== "ignored"
  ));
  if (!candidate || !latestCubeResponse) return false;
  selectedCubeCandidateId = id;
  handleCubeDetection(latestCubeResponse);
  return true;
}

const rgbCameraView = elements.rgbVideo.closest(".camera-view");
rgbCameraView?.addEventListener("click", (event) => {
  if (cubeRoiEditing || tableEdgeEditing || !latestCubeResponse) return;
  const normalized = rgbPointFromPointer(event);
  if (!normalized) return;
  const { width, height } = cubeRoiDimensions();
  const candidate = cubeCandidateAtPoint([normalized[0] * width, normalized[1] * height]);
  if (!candidate) return;
  selectCubeCandidate(candidate.candidate_id);
});
rgbCameraView?.addEventListener("pointermove", (event) => {
  if (cubeRoiEditing || tableEdgeEditing) {
    rgbCameraView.classList.remove("cube-selectable");
    return;
  }
  const normalized = rgbPointFromPointer(event);
  const { width, height } = cubeRoiDimensions();
  const candidate = normalized
    ? cubeCandidateAtPoint([normalized[0] * width, normalized[1] * height])
    : null;
  rgbCameraView.classList.toggle("cube-selectable", Boolean(candidate));
});
rgbCameraView?.addEventListener("pointerleave", () => {
  rgbCameraView.classList.remove("cube-selectable");
});

window.addEventListener("arx5-cube-select", (event) => {
  selectCubeCandidate(event.detail?.candidate_id);
});

function hideDepthHoverTooltip() {
  depthHoverRequestGeneration += 1;
  if (elements.depthHoverMarker) elements.depthHoverMarker.hidden = true;
  if (elements.depthHoverTooltip) elements.depthHoverTooltip.hidden = true;
}

function positionDepthHoverTooltip(point) {
  const marker = elements.depthHoverMarker;
  const tooltip = elements.depthHoverTooltip;
  const view = elements.depthVideo.closest(".camera-view");
  if (!tooltip || !view) return;
  if (marker) {
    marker.style.left = `${point.localX}px`;
    marker.style.top = `${point.localY}px`;
    marker.hidden = false;
  }
  const maxX = Math.max(8, view.clientWidth - 72);
  const maxY = Math.max(8, view.clientHeight - 34);
  tooltip.style.left = `${Math.min(maxX, Math.max(8, point.localX + 12))}px`;
  tooltip.style.top = `${Math.min(maxY, Math.max(8, point.localY + 12))}px`;
}

elements.depthVideo.closest(".camera-view")?.addEventListener("pointermove", async (event) => {
  const point = videoPointFromPointer(event, elements.depthVideo);
  if (!point || !cameraActive || cameraDepthAvailable !== true) {
    hideDepthHoverTooltip();
    return;
  }
  const now = performance.now();
  if (now - depthHoverLastRequestAt < 120) return;
  depthHoverLastRequestAt = now;
  positionDepthHoverTooltip(point);
  const generation = ++depthHoverRequestGeneration;
  try {
    const sample = await requestDepthSample(Math.round(point.x), Math.round(point.y), 2);
    if (generation !== depthHoverRequestGeneration) return;
    const depthM = Number(sample.depth_m);
    if (!Number.isFinite(depthM) || depthM <= 0) {
      hideDepthHoverTooltip();
      return;
    }
    elements.depthHoverTooltip.textContent = `${Math.round(depthM * 1000)} mm`;
    elements.depthHoverTooltip.hidden = false;
  } catch {
    if (generation === depthHoverRequestGeneration) hideDepthHoverTooltip();
  }
});
elements.depthVideo.closest(".camera-view")?.addEventListener(
  "pointerleave",
  hideDepthHoverTooltip,
);

window.addEventListener("keydown", (event) => {
  if (!cubeRoiEditing && !tableEdgeEditing) return;
  if (event.key === "Escape") {
    if (tableEdgeEditing) setTableEdgeEditing(false);
    else setCubeRoiEditing(false);
  } else if (event.key === "Enter") {
    if (tableEdgeEditing && normalizedTableEdgeGuides(tableEdgeDraft)) commitTableEdgeGuides();
    else if (cubeRoiEditing && normalizedCubeRoi(cubeRoiDraft)) commitCubeRoi();
  } else if ((event.key === "Backspace" || event.key === "Delete")
      && (tableEdgeEditing ? tableEdgeDraft.length : cubeRoiDraft.length) > 0) {
    event.preventDefault();
    if (tableEdgeEditing) tableEdgeDraft.pop();
    else cubeRoiDraft.pop();
    updateCubeRoiUi();
    updateTableEdgeUi();
    redrawCubeOverlay();
  }
});
elements.poseSideSwitch?.addEventListener("change", async () => {
  if (!cameraActive) {
    elements.poseSideSwitch.value = "0";
    updatePoseModeUi(false);
    return;
  }
  const wasEnabled = poseRecognitionEnabled;
  poseRecognitionEnabled = poseRecognitionMode() !== "off";
  poseStabilizer.reset();
  updatePoseModeUi(poseRecognitionEnabled && !wasEnabled);
  if (poseRecognitionEnabled && !wasEnabled) await startPoseRecognition();
  else if (!poseRecognitionEnabled) {
    stopPoseRecognition();
    updatePoseModeUi(false);
  }
});
window.addEventListener("arx5-teleoperation-mode", (event) => {
  if (!event.detail?.active) return;
  if (elements.poseSideSwitch) elements.poseSideSwitch.value = "0";
  poseRecognitionEnabled = false;
  poseStabilizer.reset();
  stopPoseRecognition();
  updatePoseModeUi(false);
});
elements.cubeBackgroundButton?.addEventListener("click", requestCubeBackgroundCapture);
updatePoseModeUi(false);
updateCubeRoiUi();
updateTableEdgeUi();
updateCubeBackgroundUi();
// The hand-eye workflow temporarily needs exclusive access to the D455.
// Keep this small bridge explicit so the control module can release and
// restore the browser stream without reaching into the camera implementation.
window.arx5StopCamera = stopCamera;
window.arx5ReleaseCameraForCalibration = releaseCameraForCalibration;
window.arx5StartCamera = startCamera;
window.arx5CameraIsActive = () => cameraActive;
window.arx5RequestDepthSample = requestDepthSample;
window.addEventListener("arx5-table-auto-align-request", alignTableFromEdgeGuides);
window.addEventListener("arx5-state", (event) => {
  const validation = event.detail?.handeye_validation;
  updateEnabledHandeyeTransform(validation);
  syncCubeDetectionConfig();
  publishCubeWorkAreaProjection();
});
window.addEventListener("arx5-scene-calibration", () => {
  publishCubeWorkAreaProjection();
  publishTableEdgeGuidePreview();
  redrawCubeOverlay();
});
elements.depthVideo.addEventListener("loadeddata", redrawCubeOverlay);
elements.depthVideo.addEventListener("resize", redrawCubeOverlay);
// Load the cached validation state early. This only opens the control
// WebSocket and never connects or moves the arm; it lets the camera apply the
// tabletop ROI before the first cube frame arrives.
window.arx5RequestControlState?.();
window.setTimeout(startCamera, 250);

elements.cubePerceptionButton?.addEventListener("click", () => setCubePerception(!cubePerceptionEnabled));
