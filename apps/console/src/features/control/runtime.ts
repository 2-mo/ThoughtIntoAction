// @ts-nocheck
import {
  controlEndpoint,
  decodeControlEvent,
  encodeCommand,
  encodeStream,
} from "../../api/control-protocol";

const jointLabels = [
  ["J1", "底座旋转"],
  ["J2", "肩部"],
  ["J3", "肘部"],
  ["J4", "腕部 1"],
  ["J5", "腕部 2"],
  ["J6", "末端旋转"],
];

const ui = {
  armStatus: document.querySelector("#armConnectButton"),
  controlUrl: document.querySelector("#controlUrl"),
  armConnectButton: document.querySelector("#armConnectButton"),
  armModeSwitch: document.querySelector("#armModeSwitch"),
  armModeButtons: [...document.querySelectorAll("[data-arm-mode]")],
  setReferenceButton: document.querySelector("#setReferenceButton"),
  parkButton: document.querySelector("#parkButton"),
  clearFaultButton: document.querySelector("#clearFaultButton"),
  emergencyButton: document.querySelector("#emergencyButton"),
  motionSpeed: document.querySelector("#motionSpeed"),
  motionSpeedValue: document.querySelector("#motionSpeedValue"),
  jointStep: document.querySelector("#jointStep"),
  jointGrid: document.querySelector("#jointGrid"),
  controlMode: document.querySelector("#controlMode"),
  controlBusy: document.querySelector("#controlBusy"),
  feedbackAge: document.querySelector("#feedbackAge"),
  payloadMetric: document.querySelector("#payloadMetric"),
  payloadWeight: document.querySelector("#payloadWeight"),
  payloadProfile: document.querySelector("#payloadProfile"),
  payloadMass: document.querySelector("#payloadMass"),
  payloadCalibrationTool: document.querySelector("#payloadCalibrationTool"),
  payloadCalibrationProfile: document.querySelector("#payloadCalibrationProfile"),
  payloadCalibrationStage: document.querySelector("#payloadCalibrationStage"),
  payloadCalibrationMessage: document.querySelector("#payloadCalibrationMessage"),
  payloadCalibrationConfirmActions: document.querySelector("#payloadCalibrationConfirmActions"),
  payloadFixedButton: document.querySelector("#payloadFixedButton"),
  payloadGraspedButton: document.querySelector("#payloadGraspedButton"),
  payloadSafetyButton: document.querySelector("#payloadSafetyButton"),
  payloadSafetyBackButton: document.querySelector("#payloadSafetyBackButton"),
  payloadCenterReadyButton: document.querySelector("#payloadCenterReadyButton"),
  payloadRunButton: document.querySelector("#payloadRunButton"),
  payloadReleaseButton: document.querySelector("#payloadReleaseButton"),
  payloadCancelButton: document.querySelector("#payloadCancelButton"),
  armGain: document.querySelector("#armGain"),
  gripperGain: document.querySelector("#gripperGain"),
  controlMessage: document.querySelector("#controlMessage"),
  statusbar: document.querySelector(".app-statusbar"),
  gripperWidth: document.querySelector("#gripperWidth"),
  gripperWidthValue: document.querySelector("#gripperWidthValue"),
  gripperActual: document.querySelector("#gripperActual"),
  gripperReference: document.querySelector("#gripperReference"),
  gripperTorque: document.querySelector("#gripperTorque"),
  gripperMinimum: document.querySelector("#gripperMinimum"),
  gripperMoveButton: document.querySelector("#gripperMoveButton"),
  gripperForceButton: document.querySelector("#gripperForceButton"),
  gripperTorqueActual: document.querySelector("#gripperTorqueActual"),
  gripperHint: document.querySelector("#gripperHint"),
  gripperQuickButtons: [...document.querySelectorAll("#gripperQuick button")],
  gripperResetButton: document.querySelector("#gripperResetButton"),
  gripperCalibrationSafetyConfirm: document.querySelector("#gripperCalibrationSafetyConfirm"),
  gripperCalibrationStart: document.querySelector("#gripperCalibrationStart"),
  gripperCalibrationClosedButton: document.querySelector("#gripperCalibrationClosedButton"),
  gripperCalibrationOpenButton: document.querySelector("#gripperCalibrationOpenButton"),
  gripperCalibrationCancelButton: document.querySelector("#gripperCalibrationCancelButton"),
  gripperCalibrationMessage: document.querySelector("#gripperCalibrationMessage"),
  handeyeCalibrationPanel: document.querySelector("#handeyeCalibrationPanel"),
  handeyeCalibrationClose: document.querySelector("#handeyeCalibrationClose"),
  handeyeSafetyConfirm: document.querySelector("#handeyeSafetyConfirm"),
  handeyeReturnReference: document.querySelector("#handeyeReturnReference"),
  handeyeD405Start: document.querySelector("#handeyeD405Start"),
  handeyeD455Start: document.querySelector("#handeyeD455Start"),
  handeyeCalibrationStart: document.querySelector("#handeyeCalibrationStart"),
  handeyeCalibrationCancel: document.querySelector("#handeyeCalibrationCancel"),
  handeyeCalibrationConfirmActions: document.querySelector("#handeyeCalibrationConfirmActions"),
  handeyeD405ReadyButton: document.querySelector("#handeyeD405ReadyButton"),
  handeyeBoardReadyButton: document.querySelector("#handeyeBoardReadyButton"),
  handeyeCenterReadyButton: document.querySelector("#handeyeCenterReadyButton"),
  handeyeRunButton: document.querySelector("#handeyeRunButton"),
  handeyeManualHomeButton: document.querySelector("#handeyeManualHomeButton"),
  handeyeManualSampleButton: document.querySelector("#handeyeManualSampleButton"),
  handeyeManualDoneButton: document.querySelector("#handeyeManualDoneButton"),
  handeyeReviewMoreButton: document.querySelector("#handeyeReviewMoreButton"),
  handeyeReviewAcceptButton: document.querySelector("#handeyeReviewAcceptButton"),
  handeyeRemoveButton: document.querySelector("#handeyeRemoveButton"),
  handeyeCalibrationStage: document.querySelector("#handeyeCalibrationStage"),
  handeyeCalibrationMessage: document.querySelector("#handeyeCalibrationMessage"),
  handeyeCalibrationProgress: document.querySelector("#handeyeCalibrationProgress"),
  handeyeCalibrationResult: document.querySelector("#handeyeCalibrationResult"),
  handeyeQualityPanel: document.querySelector("#handeyeQualityPanel"),
  handeyeQualityStatus: document.querySelector("#handeyeQualityStatus"),
  handeyeQualityMetrics: document.querySelector("#handeyeQualityMetrics"),
  handeyeQualityMessage: document.querySelector("#handeyeQualityMessage"),
  handeyeValidationStage: document.querySelector("#handeyeValidationStage"),
  handeyeValidationMessage: document.querySelector("#handeyeValidationMessage"),
  handeyeValidateButton: document.querySelector("#handeyeValidateButton"),
  handeyeEnableButton: document.querySelector("#handeyeEnableButton"),
  handeyeDisableButton: document.querySelector("#handeyeDisableButton"),
  cameraGrid: document.querySelector(".camera-grid"),
  cameraEmptyState: document.querySelector("#emptyState"),
  handeyePreview: document.querySelector("#handeyePreview"),
  handeyeGuide: document.querySelector("#handeyeGuide"),
  handeyeGuideMessage: document.querySelector("#handeyeGuideMessage"),
  handeyeGuideMetrics: document.querySelector("#handeyeGuideMetrics"),
  cubePickButton: document.querySelector("#cubePickButton"),
  cubeMoveButton: document.querySelector("#cubeMoveButton"),
  cubePlaceButton: document.querySelector("#cubePlaceButton"),
  cubeHomeButton: document.querySelector("#cubeHomeButton"),
  cubeStackButton: document.querySelector("#cubeStackButton"),
};

const SPEED_CONTROLLED_ACTIONS = new Set([
  "park",
  "preview_park",
  "preview_xyz_live",
  "move_xyz",
  "move_joint",
  "move_joint_target",
  "reset_joint",
  "gripper_position",
  "reset_gripper",
]);
const POST_MOTION_MODE_ACTIONS = new Set([
  "move_xyz",
  "move_joint",
  "move_joint_target",
  "reset_joint",
  "gripper_position",
  "reset_gripper",
  "cube_place_execute",
]);

export const MOTION_SPEED_TIERS = Object.freeze([
  1, 2, 3, 4, 5, 10, 25, 50, 75, 100,
]);
const MOTION_PROFILE_REVISION = 6;
const CONTROL_FEEDBACK_INTERVAL_MS = 40;
const MOTION_STATE_UI_INTERVAL_MS = 100;

export function getMotionSpeedIndex() {
  const value = Number(ui.motionSpeed?.value);
  if (!Number.isFinite(value)) return MOTION_SPEED_TIERS.length - 1;
  return Math.max(0, Math.min(
    MOTION_SPEED_TIERS.length - 1,
    Math.round(value),
  ));
}

export function getMotionSpeedPercent() {
  return MOTION_SPEED_TIERS[getMotionSpeedIndex()] ?? 100;
}

export function getMotionSpeedProgress() {
  return getMotionSpeedIndex() / (MOTION_SPEED_TIERS.length - 1);
}

export function getMotionSpeedParams() {
  return { speed_percent: getMotionSpeedPercent() };
}

function withMotionSpeed(action, params) {
  if (!SPEED_CONTROLLED_ACTIONS.has(action)) return params;
  return { ...params, ...getMotionSpeedParams() };
}

function withPostMotionMode(action, params) {
  if (!POST_MOTION_MODE_ACTIONS.has(action)) return params;
  return { ...params, mode_after_motion: selectedArmMode };
}

function websocketEndpoint() {
  return controlEndpoint();
}

ui.controlUrl.value = websocketEndpoint();
if (window.location.protocol === "https:") {
  ui.controlMessage.innerHTML =
    "先在主机运行 <code>./arx5 can-up</code> 和 <code>./arx5 web-lan</code>。";
  ui.controlMessage.title = "先在主机运行 ./arx5 can-up 和 ./arx5 web-lan。";
}

const modeNames = {
  disconnected: "未连接",
  damping: "自由阻尼",
  active: "关节位置",
  fault: "故障阻尼",
  closed: "已关闭",
};

const actionNames = {
  connect: "连接机械臂",
  disconnect: "断开机械臂",
  activate: "启用关节位置控制",
  damping: "进入自由阻尼",
  clear_fault: "清除故障",
  set_reference: "记录参考姿态",
  preview_park: "回参考轨迹预览",
  park: "回参考并安全停止",
  preview_xyz: "XYZ 目标预览",
  preview_xyz_live: "XYZ 实时预览",
  move_xyz: "XYZ 末端移动",
  move_joint: "关节点动",
  move_joint_target: "关节目标移动",
  reset_joint: "单关节回参考",
  gripper_position: "夹爪开度",
  gripper_force: "夹爪限扭矩闭合",
  reset_gripper: "夹爪关闭",
  gripper_calibration_start: "夹爪零点标定",
  gripper_calibration_confirm: "夹爪标定确认",
  gripper_calibration_cancel: "取消夹爪标定",
  payload_calibration_start: "负载标定",
  payload_calibration_confirm: "标定确认",
  payload_calibration_cancel: "取消负载标定",
  handeye_calibration_start: "手眼标定",
  handeye_calibration_confirm: "手眼标定确认",
  handeye_calibration_cancel: "取消手眼标定",
  handeye_validate: "验证手眼标定",
  handeye_enable: "启用手眼标定结果",
  handeye_disable: "停用手眼标定结果",
  cube_pick_preview: "方块抓取预览",
  cube_pick_execute: "方块抓取执行",
  cube_move_preview: "持物移动预览",
  cube_move_execute: "持物移动执行",
  cube_place_preview: "方块放置预览",
  cube_place_execute: "方块放置执行",
  cube_stack_preview: "完整叠放统一规划",
  cube_stack_execute: "完整叠放连续执行",
  emergency: "紧急阻尼",
};

let controlSocket;
let controlSocketReadyPromise = null;
let controlProfileCompatible = false;
let controlProfileMismatch = null;
let controlStateRequestId = null;
let pendingMotionState = null;
let motionStateFrameRequest = null;
let lastMotionStateUiUpdateAt = -Infinity;
let connectionTransition = null;
let connectionTransitionRequestId = null;
let connectionFailed = false;
let requestSequence = 0;
let servoSequence = 0;
let selectedArmMode = "damping";
let modeSelectionRequestId = null;
let teleoperationSequence = 0;
let lastTeleoperationResponseSequence = -1;
let activeTeleoperationSessionId = null;
let armState = {
  connected: false,
  mode: "disconnected",
  busy: false,
  servo_active: false,
  servo_kind: null,
  payload_calibration: null,
  gripper_calibration: null,
  handeye_calibration: null,
  handeye_validation: null,
  cube_manipulation: { stage: "idle" },
};
let queuedMotion = null;
let parkPreviewReady = false;
let parkPreviewRequestId = null;
let pendingPayloadCalibrationMode = null;
let handeyeLaunchPending = false;
let handeyeLaunchMode = "dual";
let handeyeCameraWasActive = false;
let handeyeCenterReady = false;
let cubeDetection = null;
let cubePickPreviewReady = false;
let cubePickPreviewPose = null;
let cubePickPlanning = false;
let cubePickPreviewExpiresAtMs = 0;
let cubeStagePlanning = null;
let cubeStagePreviewReady = null;
let cubeStagePreviewPose = null;
let cubeStackAutomation = null;
const HANDEYE_PREVIEW_STAGES = new Set([
  "table_check",
  "ready",
  "center",
  "working",
  "run",
  "collecting",
  "manual",
  "manual_working",
  "review",
  "review_working",
]);
const jointCards = [];

function syncGripperInput(width, { dirty = false } = {}) {
  const maximum = Number(ui.gripperWidth.max) || 88;
  const minimum = Number(ui.gripperWidth.min) || 0;
  const bounded = Math.max(minimum, Math.min(maximum, Number(width)));
  if (!Number.isFinite(bounded)) return;
  ui.gripperWidth.value = String(bounded);
  ui.gripperWidthValue.textContent = `${bounded.toFixed(0)} mm`;
  ui.gripperWidth.dataset.previewDirty = dirty ? "true" : "false";
  const progress = maximum > minimum
    ? ((bounded - minimum) / (maximum - minimum)) * 100
    : 0;
  ui.gripperWidth.style.setProperty("--gripper-fill", `${progress.toFixed(2)}%`);
}

function syncParkButton() {
  if (!ui.parkButton) return;
  const previewPending = parkPreviewRequestId !== null;
  ui.parkButton.innerHTML = previewPending
    ? '<i class="fa-solid fa-circle-notch fa-spin" aria-hidden="true"></i><span>计算预览</span>'
    : parkPreviewReady
    ? '<i class="fa-solid fa-play" aria-hidden="true"></i><span>执行回参考</span>'
    : '<i class="fa-solid fa-rotate-left" aria-hidden="true"></i><span>预览回参考</span>';
  ui.parkButton.setAttribute(
    "aria-label",
    previewPending
      ? "正在计算回参考轨迹"
      : parkPreviewReady ? "执行回参考并保持" : "预览回参考并保持",
  );
  ui.parkButton.classList.toggle("preview-ready", parkPreviewReady);
  syncCubeStageButtons();
}

function cubeDetectionReady(detection = cubeDetection) {
  const pose = detection?.pose_base;
  return detection?.status === "tracking"
    && Array.isArray(pose?.base_xyz_m)
    && pose.base_xyz_m.length === 3
    && pose.base_xyz_m.every(Number.isFinite)
    && Number(detection.confidence) >= 0.72;
}

function cubePayloadFromDetection(detection) {
  const pose = detection?.pose_base;
  if (!cubeDetectionReady(detection) || !pose) return null;
  const estimatedSize = Number(pose.size_estimate_m);
  const sceneTable = window.arx5GetSceneCalibration?.()?.table ?? null;
  return {
    status: detection.status,
    base_xyz_m: pose.base_xyz_m.map(Number),
    yaw_rad: Number(pose.yaw_rad),
    size_m: Number.isFinite(estimatedSize) && estimatedSize > 0
      ? estimatedSize
      : 0.03,
    confidence: Number(detection.confidence),
    position_source: pose.position_source ?? "web-rgb",
    depth_samples: Number(pose.depth_valid_count ?? 0),
    accepted_samples: 2,
    timestamp: Number(detection.timestamp ?? Date.now() / 1000),
    ...getMotionSpeedParams(),
    scene_table: sceneTable,
  };
}

function cubePickPayload() {
  return cubePayloadFromDetection(cubeDetection);
}

const CUBE_STACK_MIN_STABLE_FRAMES = 6;

function stackCubePayloads() {
  const source = cubePickPayload();
  const sourceId = cubeDetection?.selected_candidate_id;
  const candidates = Array.isArray(cubeDetection?.candidates)
    ? cubeDetection.candidates
    : [];
  if (!source || !sourceId) return null;
  const stableCandidates = candidates.filter(
    (candidate) => Number(candidate?.stable_frames) >= CUBE_STACK_MIN_STABLE_FRAMES,
  );
  if (!stableCandidates.some(
    (candidate) => String(candidate.candidate_id) === String(sourceId),
  )) return null;
  const sourceXy = source.base_xyz_m.slice(0, 2);
  const targets = stableCandidates
    .filter((candidate) => String(candidate.candidate_id) !== String(sourceId))
    .map((candidate) => ({
      candidate,
      payload: cubePayloadFromDetection(candidate),
    }))
    .filter((item) => item.payload)
    .sort((left, right) => {
      const distance = (payload) => Math.hypot(
        payload.base_xyz_m[0] - sourceXy[0],
        payload.base_xyz_m[1] - sourceXy[1],
      );
      return distance(left.payload) - distance(right.payload);
    });
  if (targets.length === 0) return null;
  return {
    source: structuredClone(source),
    target: structuredClone(targets[0].payload),
  };
}

function syncCubePickButton() {
  const button = ui.cubePickButton;
  if (!button) return;
  const connected = Boolean(armState.connected);
  const busy = Boolean(armState.busy || armState.servo_active);
  const fault = armState.mode === "fault";
  const canExecute = armState.mode === "active";
  const ready = cubeDetectionReady();
  const stage = armState.cube_manipulation?.stage ?? "idle";
  if (cubePickPreviewReady && cubePickPreviewExpiresAtMs > 0
      && Date.now() >= cubePickPreviewExpiresAtMs) {
    cubePickPreviewReady = false;
    cubePickPreviewPose = null;
    cubePickPreviewExpiresAtMs = 0;
    window.dispatchEvent(new CustomEvent("arx5-cube-pick-state", {
      detail: { phase: "reset" },
    }));
    setMessage("抓取预览已超过 90 秒，请重新预览。", "info");
  }
  // Keep this action clickable while prerequisites are missing. The click
  // handler can then explain whether the missing prerequisite is the control
  // socket, robot connection, calibration, or cube tracking.
  button.disabled = stage !== "idle"
    || cubePickPlanning
    || cubeStackAutomation !== null
    || busy;
  button.classList.toggle("is-planning", cubePickPlanning);
  if (cubePickPlanning) {
    button.innerHTML = '<span class="cube-pick-spinner" aria-hidden="true"><i class="fa-solid fa-hand"></i></span><span>计算路径</span>';
    button.title = "正在计算 45° 斜抓路径并执行连续 IK 安全预检";
  } else if (cubePickPreviewReady) {
    const remainingSeconds = Math.max(
      0,
      Math.ceil((cubePickPreviewExpiresAtMs - Date.now()) / 1000),
    );
    button.innerHTML = `<i class="fa-solid fa-play" aria-hidden="true"></i><span>执行 ${remainingSeconds}s</span>`;
    button.title = canExecute
      ? `连续 IK 预检已通过；当前抓取路径剩余 ${remainingSeconds} 秒`
      : "预览已就绪；点击后会自动启用关节位置控制并执行抓取";
    button.classList.add("preview-ready");
  } else {
    button.innerHTML = '<i class="fa-solid fa-hand" aria-hidden="true"></i><span>抓取方块</span>';
    button.title = ready
      ? "先生成抓取动作预览，不会立即移动机械臂"
      : "需要稳定检测到 30 mm 方块并完成固定相机标定";
    button.classList.remove("preview-ready");
  }
  syncCubeStageButtons();
}

function syncCubeStageButtons() {
  const stage = armState.cube_manipulation?.stage ?? "idle";
  const busy = Boolean(armState.busy || armState.servo_active);
  const active = armState.mode === "active";
  const moveReady = cubeStagePreviewReady === "move";
  const placeReady = cubeStagePreviewReady === "place";
  const automatic = cubeStackAutomation !== null;
  if (ui.cubeMoveButton) {
    ui.cubeMoveButton.disabled = busy
      || automatic
      || cubeStagePlanning !== null
      || (stage !== "held" && !moveReady)
      || (moveReady && !active);
    ui.cubeMoveButton.classList.toggle("preview-ready", moveReady);
    ui.cubeMoveButton.innerHTML = cubeStagePlanning === "move"
      ? '<i class="fa-solid fa-circle-notch fa-spin" aria-hidden="true"></i><span>规划移动</span>'
      : moveReady
        ? '<i class="fa-solid fa-play" aria-hidden="true"></i><span>执行移动</span>'
        : '<i class="fa-solid fa-arrows-up-down-left-right" aria-hidden="true"></i><span>移动方块</span>';
    ui.cubeMoveButton.title = stage === "held"
      ? "先在 RGB 画面点选另一个方块，再预览移动到其上方的轨迹"
      : "抓取并抬升方块后可用";
  }
  if (ui.cubePlaceButton) {
    ui.cubePlaceButton.disabled = busy
      || automatic
      || cubeStagePlanning !== null
      || (stage !== "over_target" && !placeReady)
      || (placeReady && !active);
    ui.cubePlaceButton.classList.toggle("preview-ready", placeReady);
    ui.cubePlaceButton.innerHTML = cubeStagePlanning === "place"
      ? '<i class="fa-solid fa-circle-notch fa-spin" aria-hidden="true"></i><span>规划放置</span>'
      : placeReady
        ? '<i class="fa-solid fa-play" aria-hidden="true"></i><span>执行放置</span>'
        : '<i class="fa-solid fa-cubes-stacked" aria-hidden="true"></i><span>放下方块</span>';
    ui.cubePlaceButton.title = stage === "over_target"
      ? "预览垂直下降、松爪和撤离轨迹"
      : "移动到目标方块上方后可用";
  }
  if (ui.cubeHomeButton) {
    ui.cubeHomeButton.disabled = busy
      || automatic
      || parkPreviewRequestId !== null
      || stage !== "released"
      || !Array.isArray(armState.reference_joint_position_deg);
    ui.cubeHomeButton.classList.toggle("preview-ready", parkPreviewReady);
    ui.cubeHomeButton.innerHTML = parkPreviewRequestId !== null
      ? '<i class="fa-solid fa-circle-notch fa-spin" aria-hidden="true"></i><span>计算回零</span>'
      : parkPreviewReady
      ? '<i class="fa-solid fa-play" aria-hidden="true"></i><span>执行回零</span>'
      : '<i class="fa-solid fa-house" aria-hidden="true"></i><span>回零位</span>';
    ui.cubeHomeButton.title = parkPreviewReady
      ? "执行机械臂回零轨迹，并关闭夹爪"
      : "预览机械臂返回参考零位；执行时夹爪也会关闭";
  }
  if (ui.cubeStackButton) {
    ui.cubeStackButton.disabled = busy || automatic || stage !== "idle";
    ui.cubeStackButton.classList.toggle("is-planning", automatic);
    ui.cubeStackButton.innerHTML = automatic
      ? '<i class="fa-solid fa-circle-notch fa-spin" aria-hidden="true"></i><span>自动叠放</span>'
      : '<i class="fa-solid fa-layer-group" aria-hidden="true"></i><span>一键叠放</span>';
    ui.cubeStackButton.title = automatic
      ? `正在执行：${cubeStackAutomation.label}`
      : "当前选中方块为源方块，自动叠放到最近的另一个稳定方块上";
  }
}

export function cancelParkPreview(notifyViewer = true) {
  parkPreviewRequestId = null;
  parkPreviewReady = false;
  syncParkButton();
  if (notifyViewer) {
    window.dispatchEvent(new CustomEvent("arx5-park-preview-reset"));
  }
}

export function requestParkAction() {
  if (!armState.connected || armState.mode === "fault" || armState.busy
      || armState.servo_active) {
    setMessage("当前状态不能预览或执行回参考。", "error");
    return null;
  }
  if (parkPreviewRequestId !== null) {
    setMessage("回参考轨迹仍在计算，请等待结果。");
    return null;
  }
  if (parkPreviewReady) {
    const requestId = sendMotionCommand("park");
    if (requestId === null) return null;
    window.dispatchEvent(new CustomEvent("arx5-park-execution-start"));
    cancelParkPreview(false);
    return "execute";
  }
  const requestId = sendCommand(
    "preview_park",
    withMotionSpeed("preview_park", {}),
  );
  if (requestId !== null) {
    parkPreviewRequestId = requestId;
    syncParkButton();
    setMessage("正在计算回参考 Ghost 与末端轨迹……");
    window.dispatchEvent(new CustomEvent("arx5-park-preview-start", {
      detail: { request_id: requestId },
    }));
  }
  return requestId === null ? null : "preview";
}

export function getArmState() {
  return armState;
}

function publishArmState() {
  window.dispatchEvent(new CustomEvent("arx5-state", { detail: armState }));
}

function discardQueuedMotionState() {
  pendingMotionState = null;
  if (motionStateFrameRequest !== null) {
    window.cancelAnimationFrame(motionStateFrameRequest);
    motionStateFrameRequest = null;
  }
}

function flushMotionState(now) {
  motionStateFrameRequest = null;
  const nextState = pendingMotionState;
  pendingMotionState = null;
  if (!nextState) return;
  const renderUi = now - lastMotionStateUiUpdateAt >= MOTION_STATE_UI_INTERVAL_MS;
  updateState(nextState, { renderUi });
  if (renderUi) lastMotionStateUiUpdateAt = now;
  if (pendingMotionState && motionStateFrameRequest === null) {
    motionStateFrameRequest = window.requestAnimationFrame(flushMotionState);
  }
}

function queueMotionState(nextState) {
  pendingMotionState = nextState;
  if (motionStateFrameRequest !== null) return;
  motionStateFrameRequest = window.requestAnimationFrame(flushMotionState);
}

// The camera module loads after this file. Expose a read-only state bridge so
// it can request the cached hand-eye result without connecting the robot.
window.arx5RequestControlState = async () => {
  try {
    await ensureControlSocket();
    sendCommand("state");
  } catch (error) {
    console.warn("Unable to preload robot calibration state", error);
  }
};

function createJointCards() {
  jointLabels.forEach(([joint, description], index) => {
    const card = document.createElement("article");
    card.className = "joint-card card";
    card.innerHTML = `
      <div class="joint-card-head">
        <div class="joint-name"><strong>${joint}</strong><span>${description}</span></div>
        <span class="joint-angle">—°</span>
      </div>
      <div class="joint-target-row"><span>目标</span><output class="joint-target">—°</output></div>
      <input class="joint-slider" type="range" min="-1" max="1" step="0.5" value="0" disabled />
      <div class="joint-window"><span class="joint-move-window">本次范围 —</span><span class="joint-reference">参考 —°</span></div>
      <div class="joint-meta"><span class="joint-min">—</span><span class="joint-torque">— N·m</span><span class="joint-max">—</span></div>
      <div class="joint-actions">
        <button class="joint-jog" data-direction="-1" disabled>− 微调</button>
        <button class="joint-jog" data-direction="1" disabled>＋ 微调</button>
        <button class="joint-reset" disabled>↺ 回参考</button>
      </div>`;
    const item = {
      card,
      angle: card.querySelector(".joint-angle"),
      target: card.querySelector(".joint-target"),
      slider: card.querySelector(".joint-slider"),
      moveWindow: card.querySelector(".joint-move-window"),
      reference: card.querySelector(".joint-reference"),
      minimum: card.querySelector(".joint-min"),
      maximum: card.querySelector(".joint-max"),
      torque: card.querySelector(".joint-torque"),
      buttons: [...card.querySelectorAll(".joint-jog")],
      resetButton: card.querySelector(".joint-reset"),
      current: null,
      dragging: false,
      commandPending: false,
    };
    for (const button of item.buttons) {
      button.addEventListener("click", () => {
        const direction = Number(button.dataset.direction);
        const step = Number(ui.jointStep.value);
        sendMotionCommand("move_joint", {
          joint: index + 1,
          delta_deg: direction * step,
        });
      });
    }
    item.resetButton.addEventListener("click", () => {
      sendMotionCommand("reset_joint", {
        joint: index + 1,
      });
    });
    item.slider.addEventListener("pointerdown", () => { item.dragging = true; });
    item.slider.addEventListener("input", () => {
      item.dragging = true;
      updateJointTargetPreview(item);
    });
    item.slider.addEventListener("change", () => {
      item.dragging = false;
      const target = Number(item.slider.value);
      if (!Number.isFinite(item.current) || Math.abs(target - item.current) < 0.1) {
        setMessage(`${joint} 已经位于该目标附近。`);
        return;
      }
      item.commandPending = true;
      if (sendMotionCommand("move_joint_target", {
        joint: index + 1,
        target_deg: target,
      }) === null) {
        item.commandPending = false;
      }
    });
    item.slider.addEventListener("pointercancel", () => {
      item.dragging = false;
    });
    ui.jointGrid.append(card);
    jointCards.push(item);
  });
}

function updateJointTargetPreview(item) {
  const target = Number(item.slider.value);
  const delta = Number.isFinite(item.current) ? target - item.current : 0;
  const sign = delta >= 0 ? "+" : "";
  item.target.textContent = `${target.toFixed(1)}° (${sign}${delta.toFixed(1)}°)`;
}

function setPill(text, state) {
  ui.armStatus.dataset.statusText = text;
  ui.armStatus.classList.remove("pending", "ok", "warning", "bad");
  ui.armStatus.classList.add(state);
  ui.armStatus.title = `机械臂：${text}`;
}

function setControlBusy(text, active = false) {
  ui.controlBusy.textContent = text;
  const metric = ui.controlBusy.closest(".activity-status");
  if (!metric) return;
  metric.classList.toggle("active", active);
  metric.setAttribute("aria-hidden", String(!active));
  metric.title = `当前动作：${text}`;
}

export function setControlMessage(text, kind = "info") {
  ui.controlMessage.textContent = text;
  ui.controlMessage.title = text;
  ui.controlMessage.className = `control-message ${kind}`;
  ui.statusbar?.classList.toggle("has-error", kind === "error");
}

function setMessage(text, kind = "info") {
  setControlMessage(text, kind);
}

function socketIsOpen() {
  return controlSocket?.readyState === WebSocket.OPEN
    && controlProfileCompatible;
}

export function sendCommand(action, params = {}) {
  if (!socketIsOpen()) {
    setMessage("控制服务尚未连接，请先点击“连接机械臂”。", "error");
    return null;
  }
  requestSequence += 1;
  controlSocket.send(encodeCommand(action, params, requestSequence));
  return requestSequence;
}

function publishMotionCommand(action, params, requestId) {
  if (requestId === null) return;
  window.dispatchEvent(new CustomEvent("arx5-motion-command-start", {
    detail: { action, params, request_id: requestId },
  }));
}

export function sendMotionCommand(action, params = {}) {
  if (!armState.connected) {
    setMessage("请先连接机械臂。", "error");
    return null;
  }
  if (armState.mode === "fault") {
    setMessage("当前为故障阻尼，请先点击“清除故障”。", "error");
    return null;
  }
  if (armState.busy) {
    setMessage("上一动作还在执行，请等待完成。", "error");
    return null;
  }
  if (armState.servo_active) {
    setMessage("实时伺服正在运行，请先松开拖动。", "error");
    return null;
  }
  if (action !== "park") cancelParkPreview();
  const commandParams = withPostMotionMode(
    action,
    withMotionSpeed(action, params),
  );
  if (armState.mode === "damping") {
    queuedMotion = { action, params: commandParams };
    const requestId = sendCommand("activate");
    if (requestId === null) queuedMotion = null;
    else setMessage("正在平滑启用关节位置控制，随后执行刚才的操作……");
    return requestId;
  }
  if (armState.mode !== "active") {
    setMessage(`当前模式 ${armState.mode} 不能执行运动。`, "error");
    return null;
  }
  const requestId = sendCommand(action, commandParams);
  publishMotionCommand(action, commandParams, requestId);
  return requestId;
}

const CUBE_STACK_LABELS = {
  set_reference: "记录叠放起始参考位",
  activate: "启用关节位置控制",
  cube_stack_preview: "统一规划完整叠放与回程",
  cube_stack_execute: "连续执行叠放并回到起始参考位",
};

function stopCubeStackAutomation(error = null) {
  if (cubeStackAutomation?.previewTimer) {
    clearTimeout(cubeStackAutomation.previewTimer);
  }
  cubeStackAutomation = null;
  syncCubePickButton();
  syncCubeStageButtons();
  if (error) setMessage(`一键叠放已停止：${error}`, "error");
}

function holdCubeStackPreview(nextAction, params) {
  if (!cubeStackAutomation) return;
  cubeStackAutomation.expectedAction = `preview:${nextAction}`;
  cubeStackAutomation.label = `显示${CUBE_STACK_LABELS[nextAction] ?? nextAction}预览`;
  syncCubeStageButtons();
  setMessage(`一键叠放：${cubeStackAutomation.label}……`, "warning");
  cubeStackAutomation.previewTimer = window.setTimeout(() => {
    if (!cubeStackAutomation
        || cubeStackAutomation.expectedAction !== `preview:${nextAction}`) return;
    cubeStackAutomation.previewTimer = null;
    sendCubeStackStep(nextAction, params);
  }, 100);
}

function sendCubeStackStep(action, params = {}) {
  if (!cubeStackAutomation) return;
  cubeStackAutomation.expectedAction = action;
  cubeStackAutomation.label = CUBE_STACK_LABELS[action] ?? action;
  const commandParams = withPostMotionMode(
    action,
    { ...params, ...cubeStackAutomation.speed },
  );
  const requestId = sendCommand(action, commandParams);
  if (requestId === null) {
    stopCubeStackAutomation("控制服务不可用。");
    return;
  }
  if ([
    "cube_pick_execute",
    "cube_move_execute",
    "cube_place_execute",
    "cube_stack_execute",
  ].includes(action)) {
    publishMotionCommand(action, commandParams, requestId);
  }
  if (action === "cube_pick_execute") {
    window.dispatchEvent(new CustomEvent("arx5-cube-pick-state", {
      detail: { phase: "executing" },
    }));
  }
  syncCubeStageButtons();
  setMessage(`一键叠放：${cubeStackAutomation.label}……`, "warning");
}

function advanceCubeStackAutomation(message) {
  const automation = cubeStackAutomation;
  if (!automation || message.action !== automation.expectedAction) return;
  if (!message.ok) {
    stopCubeStackAutomation(message.error ?? `${automation.label}失败。`);
    return;
  }
  switch (message.action) {
    case "set_reference":
      if (armState.mode === "damping") sendCubeStackStep("activate");
      else sendCubeStackStep("cube_stack_preview", {
        source: automation.source,
        target: automation.target,
      });
      break;
    case "activate":
      sendCubeStackStep("cube_stack_preview", {
        source: automation.source,
        target: automation.target,
      });
      break;
    case "cube_stack_preview":
      holdCubeStackPreview("cube_stack_execute", {
        ...automation.speed,
        confirmed: true,
      });
      break;
    case "cube_stack_execute":
      stopCubeStackAutomation();
      setMessage(
        "一键叠放完成：连续轨迹已执行，机械臂与夹爪已回任务起始参考位并保持。",
        "success",
      );
      break;
    default:
      stopCubeStackAutomation(`未知阶段 ${message.action}。`);
  }
}

export function sendServoMessage(action, params = {}, kind = "cartesian") {
  if (!socketIsOpen() || !armState.connected) return null;
  if (action === "update" && controlSocket.bufferedAmount > 32 * 1024) {
    return null;
  }
  const servoParams = action === "start"
    ? {
        ...params,
        ...getMotionSpeedParams(),
        mode_after_motion: selectedArmMode,
      }
    : params;
  servoSequence += 1;
  controlSocket.send(encodeStream(
    kind === "gripper" ? "gripper" : "arm",
    kind === "gripper" ? "position" : "cartesian",
    action,
    servoSequence,
    servoParams,
  ));
  return servoSequence;
}

export function sendGripperServoMessage(action, params = {}) {
  return sendServoMessage(action, params, "gripper");
}

export function sendTeleoperationMessage(
  action,
  params = {},
  sessionId = null,
) {
  if (!socketIsOpen() || !armState.connected) return null;
  if (action === "update" && controlSocket.bufferedAmount > 32 * 1024) return null;
  teleoperationSequence += 1;
  controlSocket.send(encodeStream(
    "teleoperation",
    "target",
    action,
    teleoperationSequence,
    {
      ...params,
      ...(sessionId ? { session_id: sessionId } : {}),
      ...(action === "update" ? { client_time_ms: Date.now() } : {}),
    },
  ));
  return teleoperationSequence;
}

export function requestEmergencyStop() {
  cancelParkPreview();
  if (cubeStackAutomation) stopCubeStackAutomation("已触发急停。");
  return sendCommand("emergency");
}

function ensureControlSocket() {
  if (socketIsOpen()) return Promise.resolve();
  if (controlSocketReadyPromise) return controlSocketReadyPromise;
  if (controlSocket?.readyState === WebSocket.CLOSING) controlSocket = null;
  controlProfileCompatible = false;
  controlProfileMismatch = null;
  controlStateRequestId = null;
  setPill("连接服务…", "pending");
  controlSocketReadyPromise = new Promise((resolve, reject) => {
    const socket = new WebSocket(ui.controlUrl.value.trim());
    controlSocket = socket;
    socket.addEventListener("open", () => {
      setPill("校验配置…", "pending");
    }, { once: true });
    socket.addEventListener("message", (event) => {
      let message;
      try {
        message = decodeControlEvent(event.data);
      } catch (_error) {
        return;
      }
      if (message.type !== "hello") return;
      if (message.limits?.motion_profile_revision !== MOTION_PROFILE_REVISION) {
        event.stopImmediatePropagation();
        controlProfileMismatch = "网页与控制服务的运动配置版本不一致，请重启网页服务并强制刷新页面。";
        setPill("配置错误", "bad");
        setMessage(controlProfileMismatch, "error");
        socket.close(1008, "motion profile mismatch");
        reject(new Error(controlProfileMismatch));
        return;
      }
      controlProfileCompatible = true;
      resolve();
    });
    socket.addEventListener("message", handleControlMessage);
    socket.addEventListener("close", () => {
      const profileMismatch = controlProfileMismatch;
      connectionFailed = true;
      controlProfileCompatible = false;
      controlStateRequestId = null;
      connectionTransitionRequestId = null;
      selectedArmMode = "damping";
      modeSelectionRequestId = null;
      discardQueuedMotionState();
      cancelParkPreview();
      cubePickPlanning = false;
      cubePickPreviewReady = false;
      cubePickPreviewPose = null;
      cubePickPreviewExpiresAtMs = 0;
      cubeStagePlanning = null;
      cubeStagePreviewReady = null;
      cubeStagePreviewPose = null;
      cubeStackAutomation = null;
      window.dispatchEvent(new CustomEvent("arx5-cube-pick-state", {
        detail: { phase: "reset" },
      }));
      window.dispatchEvent(new CustomEvent("arx5-teleop-state", {
        detail: {
          type: "teleop_state",
          seq: teleoperationSequence,
          action: "stop",
          ok: false,
          session_id: activeTeleoperationSessionId,
          error: "网页控制连接已断开。",
        },
      }));
      activeTeleoperationSessionId = null;
      connectionTransition = null;
      controlSocketReadyPromise = null;
      armState = {
        connected: false,
        mode: "disconnected",
        busy: false,
        servo_active: false,
        servo_kind: null,
        payload_calibration: null,
        handeye_calibration: null,
        handeye_validation: null,
        cube_manipulation: { stage: "idle" },
      };
      handeyeLaunchPending = false;
      handeyeLaunchMode = "dual";
      restoreHandeyeCamera();
      updateState(armState);
      if (profileMismatch) {
        setPill("配置错误", "bad");
        setMessage(profileMismatch, "error");
      } else {
        setPill("未连接", "pending");
        setMessage("网页控制连接已断开；服务端会安全释放机械臂。", "info");
      }
      reject(new Error(profileMismatch ?? "控制服务连接已断开"));
    });
    socket.addEventListener("error", () => {
      controlSocketReadyPromise = null;
      connectionFailed = true;
      updateControls();
      if (controlProfileMismatch) {
        reject(new Error(controlProfileMismatch));
        return;
      }
      setPill("服务错误", "bad");
      setMessage(`无法连接 ${ui.controlUrl.value}。请确认网页服务仍在运行。`, "error");
      reject(new Error("控制服务连接失败"));
    }, { once: true });
  }).catch((error) => {
    controlSocketReadyPromise = null;
    throw error;
  });
  updateControls();
  return controlSocketReadyPromise;
}

function connectControl() {
  cancelParkPreview();
  selectedArmMode = "damping";
  modeSelectionRequestId = null;
  connectionTransition = "connect";
  updateControls();
  ensureControlSocket().then(() => {
    connectionTransitionRequestId = sendCommand("connect");
    if (connectionTransitionRequestId !== null) {
      setMessage("控制服务已连接，正在打开 CAN 控制器……");
    } else {
      connectionFailed = true;
      connectionTransition = null;
      updateControls();
    }
  }).catch(() => {
    connectionFailed = true;
    connectionTransition = null;
    connectionTransitionRequestId = null;
    updateControls();
  });
}

function handleControlMessage(event) {
  let message;
  try {
    message = decodeControlEvent(event.data);
  } catch (error) {
    setMessage(`控制服务返回了无效数据：${error.message}`, "error");
    return;
  }
  if (controlStateRequestId !== null && message.id === controlStateRequestId) {
    controlStateRequestId = null;
    if (message.state) updateState(message.state);
    return;
  }
  if (message.type === "hello") {
    updateState(message.state);
    return;
  }
  if (message.type === "payload_calibration") {
    if (message.calibration?.active) pendingPayloadCalibrationMode = null;
    if (message.state) updateState(message.state);
    renderPayloadCalibration(message.calibration);
    return;
  }
  if (message.type === "gripper_calibration") {
    if (message.state) updateState(message.state);
    renderGripperCalibration(message.calibration);
    return;
  }
  if (message.type === "handeye_plan") {
    window.dispatchEvent(new CustomEvent("arx5-handeye-plan", { detail: message.plan }));
    return;
  }
  if (message.type === "handeye_preview") {
    renderHandeyePreview(message.preview);
    return;
  }
  if (message.type === "handeye_calibration") {
    if (message.calibration?.active) handeyeLaunchPending = false;
    if (message.state) {
      const feedback = message.calibration?.active
        ? message.calibration?.feedback
        : null;
      updateState(feedback && Array.isArray(feedback.joint_position_deg)
        ? {
            ...message.state,
            ...feedback,
            connected: true,
            mode: feedback.mode ?? "active",
            busy: true,
            handeye_calibration: message.calibration,
          }
        : message.state);
    }
    renderHandeyeCalibration(message.calibration);
    if (["done", "cancelled", "incomplete", "error"].includes(message.calibration?.stage)) {
      restoreHandeyeCamera();
    }
    return;
  }
  if (message.type === "handeye_validation") {
    if (message.state) updateState(message.state);
    renderHandeyeValidation(message.validation);
    return;
  }
  if (message.type === "fatal") {
    connectionFailed = true;
    connectionTransition = null;
    connectionTransitionRequestId = null;
    updateControls();
    setMessage(message.message, "error");
    setPill("服务占用", "bad");
    return;
  }
  if (message.type === "teleop_state") {
    if (!Number.isInteger(message.seq)
        || message.seq <= lastTeleoperationResponseSequence) return;
    lastTeleoperationResponseSequence = message.seq;
    if (message.ok && message.session_id) {
      activeTeleoperationSessionId = message.session_id;
    }
    if (message.action === "stop") activeTeleoperationSessionId = null;
    if (message.state) updateState(message.state);
    window.dispatchEvent(new CustomEvent("arx5-teleop-state", { detail: message }));
    if (!message.ok) setMessage(message.error ?? "手势遥操作已停止。", "error");
    return;
  }
  if (message.type === "servo_state") {
    if (message.state) updateState(message.state);
    if (!message.ok) {
      setMessage(message.error ?? "实时伺服已停止。", "error");
    } else if (message.action === "start") {
      setMessage(
        message.kind === "gripper"
          ? "夹爪按住控制已启动；松开按键即可停止。"
          : "实时伺服已启动；保持平面键和鼠标按键进行拖动。",
        "success",
      );
    } else if (message.action === "stop") {
      setMessage("实时伺服已平滑停车。", "success");
    }
    window.dispatchEvent(new CustomEvent("arx5-servo-state", { detail: message }));
    return;
  }
  if (message.type === "command_result" && message.action === "preview_xyz_live") {
    if (message.state) updateState(message.state);
    window.dispatchEvent(new CustomEvent("arx5-command-result", { detail: message }));
    return;
  }
  if (message.type === "command_started") {
    discardQueuedMotionState();
    lastMotionStateUiUpdateAt = -Infinity;
    if (message.action === "cube_pick_preview") cubePickPlanning = true;
    if (message.action === "cube_move_preview") cubeStagePlanning = "move";
    if (message.action === "cube_place_preview") cubeStagePlanning = "place";
    armState.busy = true;
    updateControls();
    publishArmState();
    setControlBusy(`${actionNames[message.action] ?? message.action}…`, true);
    setMessage(`${actionNames[message.action] ?? message.action}正在执行，请等待完成。`);
    return;
  }
  if (message.type === "motion_state") {
    if (message.state) queueMotionState(message.state);
    return;
  }
  if (message.type === "command_result") {
    discardQueuedMotionState();
    lastMotionStateUiUpdateAt = -Infinity;
    const matchedConnectionRequest = connectionTransitionRequestId !== null
      && message.id === connectionTransitionRequestId;
    if (
      message.action === "connect"
      || (connectionTransition === "connect" && matchedConnectionRequest)
    ) {
      connectionFailed = message.ok !== true;
    } else if (message.action === "disconnect" && message.ok) {
      connectionFailed = false;
    }
    if (
      ["connect", "disconnect"].includes(message.action)
      || matchedConnectionRequest
    ) {
      connectionTransition = null;
      connectionTransitionRequestId = null;
    }
    if (message.action === "payload_calibration_start" && !message.ok) {
      pendingPayloadCalibrationMode = null;
      renderPayloadCalibration(message.state?.payload_calibration ?? null);
    }
    if (message.action === "gripper_calibration_start" && !message.ok) {
      renderGripperCalibration(message.state?.gripper_calibration ?? null);
    }
    if (message.action === "handeye_calibration_start" && !message.ok) {
      handeyeLaunchPending = false;
      renderHandeyeCalibration(message.state?.handeye_calibration ?? null);
      restoreHandeyeCamera();
    }
    jointCards.forEach((item) => { item.commandPending = false; });
    if (message.action === "cube_pick_preview") {
      cubePickPlanning = false;
      cubePickPreviewReady = message.ok === true;
      cubePickPreviewExpiresAtMs = cubePickPreviewReady
        ? Date.now() + 1000 * (Number(message.result?.valid_for_s) || 90)
        : 0;
      cubePickPreviewPose = cubePickPreviewReady && cubeDetection?.pose_base
        ? cubeDetection.pose_base.base_xyz_m.map(Number)
        : null;
      const graspLabel = message.result?.grasp_mode === "side"
        ? `${Number(message.result?.grasp_tilt_deg) || 45}°斜抓`
        : "俯抓";
      setMessage(
        message.ok
          ? `${graspLabel}末端轨迹已显示；90 秒内再次点击“执行抓取”才会移动真机。`
          : (message.error ?? "抓取规划失败。"),
        message.ok ? "success" : "error",
      );
      syncCubePickButton();
    } else if (["cube_move_preview", "cube_place_preview"].includes(message.action)) {
      const stage = message.action === "cube_move_preview" ? "move" : "place";
      cubeStagePlanning = null;
      cubeStagePreviewReady = message.ok === true ? stage : null;
      cubeStagePreviewPose = message.ok && stage === "move" && cubeDetection?.pose_base
        ? cubeDetection.pose_base.base_xyz_m.map(Number)
        : null;
      setMessage(
        message.ok
          ? `${stage === "move" ? "持物移动" : "堆叠放置"}轨迹已显示；再次点击对应按钮执行。`
          : (message.error ?? `${stage === "move" ? "移动" : "放置"}规划失败。`),
        message.ok ? "success" : "error",
      );
      syncCubeStageButtons();
    } else if (["cube_move_execute", "cube_place_execute"].includes(message.action)) {
      cubeStagePlanning = null;
      cubeStagePreviewReady = null;
      cubeStagePreviewPose = null;
      syncCubeStageButtons();
    } else if (message.action === "cube_pick_execute") {
      cubePickPreviewReady = false;
      cubePickPreviewPose = null;
      cubePickPreviewExpiresAtMs = 0;
      cubeStagePreviewReady = null;
      cubeStagePreviewPose = null;
      syncCubePickButton();
    } else if (message.action !== "state") {
      cubePickPreviewReady = false;
      cubePickPreviewPose = null;
      cubePickPreviewExpiresAtMs = 0;
      cubeStagePlanning = null;
      cubeStagePreviewReady = null;
      cubeStagePreviewPose = null;
      window.dispatchEvent(new CustomEvent("arx5-cube-pick-state", {
        detail: { phase: "reset" },
      }));
      syncCubePickButton();
    }
  }
  if (message.type === "command_result") {
    if (message.action === "preview_park") parkPreviewRequestId = null;
    if (message.ok && ["connect", "disconnect", "park"].includes(message.action)) {
      selectedArmMode = "damping";
    }
  }
  if (message.state) updateState(message.state);
  if (message.type === "command_result") {
    if (message.id === modeSelectionRequestId) {
      modeSelectionRequestId = null;
      if (!message.ok) {
        selectedArmMode = ["active", "damping"].includes(armState.mode)
          ? armState.mode
          : "damping";
      }
    }
    if (!armState.payload_calibration?.active
        && !armState.gripper_calibration?.active
        && !armState.handeye_calibration?.active) armState.busy = false;
    updateControls();
    publishArmState();
    window.dispatchEvent(new CustomEvent("arx5-command-result", { detail: message }));
    if (message.action === "preview_park") {
      if (message.ok && message.result?.kind === "park_preview") {
        parkPreviewReady = true;
        syncParkButton();
      } else {
        cancelParkPreview();
      }
    } else if (message.action === "park") {
      cancelParkPreview(false);
    } else if (message.action === "set_reference" && message.ok) {
      cancelParkPreview();
    }
    if (message.action === "damping" && message.ok && handeyeLaunchPending) {
      startHandeyeAfterRelease();
    }
    if (message.action === "damping" && !message.ok && handeyeLaunchPending) {
      handeyeLaunchPending = false;
      handeyeLaunchMode = "dual";
      restoreHandeyeCamera();
      renderHandeyeCalibration(armState.handeye_calibration);
    }
    if (message.action === "handeye_calibration_start") {
      handeyeLaunchPending = false;
      handeyeLaunchMode = "dual";
      if (message.ok) setMessage("手眼标定完成，结果已缓存。", "success");
      else if (message.error) setMessage(message.error, "error");
      renderHandeyeCalibration(message.state?.handeye_calibration ?? armState.handeye_calibration);
      if (!message.ok) restoreHandeyeCamera();
    }
    if (message.action === "handeye_calibration_cancel") {
      handeyeLaunchPending = false;
      handeyeLaunchMode = "dual";
      restoreHandeyeCamera();
    }
    if (message.action === "gripper_calibration_start") {
      renderGripperCalibration(message.state?.gripper_calibration ?? armState.gripper_calibration);
      if (message.ok) setMessage("夹爪零点标定完成，已保存全开读数。", "success");
      else if (message.error) setMessage(message.error, "error");
    }
    if (["handeye_validate", "handeye_enable", "handeye_disable"].includes(message.action)) {
      renderHandeyeValidation(message.state?.handeye_validation ?? armState.handeye_validation);
      setMessage(
        message.ok
          ? (message.action === "handeye_enable"
            ? "手眼标定结果已启用。"
            : message.action === "handeye_disable"
              ? "手眼标定结果已停用。"
              : "手眼标定结果验证完成。")
          : (message.error ?? "手眼标定结果操作失败。"),
        message.ok ? "success" : "error",
      );
    }
    if (message.ok && ["gripper_position", "gripper_force", "reset_gripper"]
      .includes(message.action)) {
      const finalWidth = Number(message.result?.final_width_mm);
      if (Number.isFinite(finalWidth)) syncGripperInput(finalWidth);
    }
    if (message.ok) {
      if (message.action === "activate" && queuedMotion) {
        const nextMotion = queuedMotion;
        queuedMotion = null;
        const requestId = sendCommand(nextMotion.action, nextMotion.params);
        publishMotionCommand(nextMotion.action, nextMotion.params, requestId);
        if (requestId !== null && nextMotion.action === "cube_pick_execute") {
          window.dispatchEvent(new CustomEvent("arx5-cube-pick-state", {
            detail: { phase: "executing" },
          }));
        }
        setMessage("关节位置控制已启用，正在执行刚才的操作……");
        return;
      }
      const action = actionNames[message.action] ?? message.action ?? "命令";
      let detail = "";
      if (message.result?.kind === "gripper_force") {
        detail = `，最终 ${message.result.final_width_mm.toFixed(1)} mm，停止原因：${message.result.stop_reason}`;
      } else if (message.result?.kind === "gripper_position") {
        detail = `，最终 ${message.result.final_width_mm.toFixed(1)} mm`;
      } else if (message.result?.kind === "joint_motion") {
        const error = Math.max(...message.result.final_error_deg.map((value) => Math.abs(value)));
        detail = `，最大终点误差 ${error.toFixed(2)}°`;
      } else if (message.result?.kind === "cartesian_preview") {
        detail = `，IK 可行，最大关节变化 ${message.result.max_joint_delta_deg.toFixed(1)}°`;
      } else if (message.result?.kind === "park_preview") {
        detail = `，最大关节变化 ${message.result.max_joint_delta_deg.toFixed(1)}°，轨迹 ${message.result.trajectory_duration_s.toFixed(1)} s`;
      } else if (message.result?.kind === "cartesian_motion") {
        detail = `，末端位置误差 ${message.result.final_position_error_mm.toFixed(2)} mm`;
      }
      setMessage(`${action}完成${detail}。`, "success");
    } else {
      if (message.action === "activate") {
        const canceledPark = queuedMotion?.action === "park";
        queuedMotion = null;
        cancelParkPreview();
        if (canceledPark) {
          window.dispatchEvent(new CustomEvent("arx5-park-execution-cancel"));
        }
      }
      setMessage(message.error ?? "命令失败。", "error");
    }
    advanceCubeStackAutomation(message);
  }
}

function renderPayloadCalibration(calibration = armState.payload_calibration) {
  if (!ui.payloadCalibrationTool) return;
  const active = Boolean(calibration?.active);
  const stage = calibration?.stage ?? null;
  const modeLabel = calibration?.mode === "grasped" ? "夹取物体" : "固定负载";
  if (active) ui.payloadCalibrationTool.open = true;
  ui.payloadCalibrationStage.textContent = active
    ? `${modeLabel} · ${stage ?? "准备中"}`
    : (stage === "done" ? "已缓存" : "缓存档案可复用");
  ui.payloadCalibrationMessage.textContent = calibration?.message
    ?? "空载基线已缓存；选择负载类型开始。";
  ui.payloadCalibrationMessage.className = "payload-calibration-message"
    + (calibration?.error ? " error" : stage === "done" ? " success" : "");
  ui.payloadCalibrationConfirmActions.hidden = !active;
  const safetyPending = !active && Boolean(pendingPayloadCalibrationMode);
  ui.payloadFixedButton.hidden = active || safetyPending;
  ui.payloadGraspedButton.hidden = active || safetyPending;
  ui.payloadSafetyButton.hidden = !safetyPending;
  ui.payloadSafetyBackButton.hidden = !safetyPending;
  ui.payloadCalibrationProfile.disabled = active;
  ui.payloadCenterReadyButton.hidden = stage !== "center";
  ui.payloadRunButton.hidden = stage !== "run";
  ui.payloadReleaseButton.hidden = stage !== "releasing";
  ui.payloadCancelButton.hidden = !active;
  if (active && calibration.progress) {
    const { current, total } = calibration.progress;
    if (Number.isFinite(current) && Number.isFinite(total) && total > 0) {
      ui.payloadCalibrationStage.textContent = `${modeLabel} · ${current}/${total}`;
    }
  }
  if (safetyPending) {
    ui.payloadCalibrationStage.textContent = pendingPayloadCalibrationMode === "grasped"
      ? "夹取物体 · 待安全确认"
      : "固定负载 · 待安全确认";
    ui.payloadCalibrationMessage.textContent =
      "确认机械臂远离桌面、D455、线缆和人员，实体急停可用；夹取物体时准备全程承托。";
  }
}

function renderGripperCalibration(calibration = armState.gripper_calibration) {
  if (!ui.gripperCalibrationMessage) return;
  const active = Boolean(calibration?.active);
  const stage = calibration?.stage ?? null;
  const isClosedPrompt = active && stage === "await_closed";
  const isOpenPrompt = active && stage === "await_open";
  const isDone = stage === "done";
  const isCached = stage === "cached";
  const isPersisted = isDone || isCached;
  const isError = Boolean(calibration?.error) || stage === "error";
  ui.gripperCalibrationStart.hidden = active || isPersisted;
  ui.gripperCalibrationClosedButton.hidden = !isClosedPrompt;
  ui.gripperCalibrationOpenButton.hidden = !isOpenPrompt;
  ui.gripperCalibrationCancelButton.hidden = !active;
  ui.gripperCalibrationSafetyConfirm.disabled = active;
  ui.gripperCalibrationSafetyConfirm.closest("label").hidden = isPersisted;
  ui.gripperCalibrationMessage.textContent = calibration?.message
    ?? "断开机械臂后可进行夹爪零点校准。";
  ui.gripperCalibrationMessage.className = "gripper-calibration-message"
    + (isError ? " error" : isPersisted ? " success" : "");
}

function restoreHandeyeCamera() {
  clearHandeyePreview();
  if (!handeyeCameraWasActive) return;
  handeyeCameraWasActive = false;
  if (typeof window.arx5StartCamera === "function") window.arx5StartCamera();
}

function renderHandeyePreview(preview) {
  if (!preview?.jpeg || !ui.handeyePreview) return;
  handeyeCenterReady = Boolean(preview.ready);
  window.dispatchEvent(new CustomEvent("arx5-handeye-board-pose", {
    detail: preview,
  }));
  ui.handeyePreview.src = `data:image/jpeg;base64,${preview.jpeg}`;
  ui.handeyePreview.hidden = false;
  ui.cameraGrid?.classList.add("handeye-guidance-active");
  ui.cameraEmptyState?.classList.add("is-hidden");
  if (ui.handeyeGuide) {
    ui.handeyeGuide.hidden = false;
    ui.handeyeGuide.classList.toggle("ready", Boolean(preview.ready));
  }
  if (ui.handeyeGuideMessage) {
    ui.handeyeGuideMessage.textContent = preview.message ?? "正在识别标定板";
  }
  if (ui.handeyeCenterReadyButton) {
    ui.handeyeCenterReadyButton.disabled = !handeyeCenterReady;
  }
  if (ui.handeyeManualSampleButton
      && armState.handeye_calibration?.stage === "manual") {
    ui.handeyeManualSampleButton.disabled = !handeyeCenterReady;
  }
  const metrics = [];
  const metricNumber = (value) => value == null ? Number.NaN : Number(value);
  const distance = metricNumber(preview.distance_m);
  const tilt = metricNumber(preview.tilt_deg);
  const j4Margin = metricNumber(preview.j4_margin_deg);
  const reprojection = metricNumber(preview.reprojection_error_px);
  if (Number.isFinite(distance)) metrics.push(`距离 ${distance.toFixed(2)} m`);
  if (Number.isFinite(tilt)) metrics.push(`倾角 ${tilt.toFixed(1)}°`);
  if (Number.isFinite(j4Margin)) metrics.push(`J4 余量 ${j4Margin.toFixed(1)}°`);
  if (Number.isFinite(reprojection)) metrics.push(`重投影 ${reprojection.toFixed(2)} px`);
  const boardPosition = preview.target_position_camera_m;
  if (Array.isArray(boardPosition) && boardPosition.length === 3
      && boardPosition.every((value) => Number.isFinite(Number(value)))) {
    metrics.push(`板位 x ${Number(boardPosition[0] * 1000).toFixed(0)} · y ${Number(boardPosition[1] * 1000).toFixed(0)} · z ${Number(boardPosition[2] * 1000).toFixed(0)} mm`);
  }
  if (ui.handeyeGuideMetrics) ui.handeyeGuideMetrics.textContent = metrics.join(" · ");
}

function clearHandeyePreview() {
  handeyeCenterReady = false;
  window.dispatchEvent(new CustomEvent("arx5-handeye-board-pose", { detail: null }));
  ui.cameraGrid?.classList.remove("handeye-guidance-active");
  if (ui.handeyePreview) {
    ui.handeyePreview.hidden = true;
    ui.handeyePreview.removeAttribute("src");
  }
  if (ui.handeyeGuide) ui.handeyeGuide.hidden = true;
  const handeyeOwnsCameraSurface = handeyeLaunchPending
    || Boolean(armState.handeye_calibration?.active);
  if (!window.arx5CameraIsActive?.() && !handeyeOwnsCameraSurface) {
    ui.cameraEmptyState?.classList.remove("is-hidden");
  }
}

function setHandeyePanelOpen() {
  if (!ui.handeyeCalibrationPanel) return;
  ui.handeyeCalibrationPanel.hidden = false;
}

function formatHandeyeDuration(seconds) {
  const value = Number(seconds);
  if (!Number.isFinite(value) || value < 0) return null;
  const rounded = Math.ceil(value);
  if (rounded < 60) return `${rounded} 秒`;
  const minutes = Math.floor(rounded / 60);
  const remainder = rounded % 60;
  return remainder ? `${minutes} 分 ${remainder} 秒` : `${minutes} 分`;
}

function handeyeProgressSummary(progress) {
  if (!progress) {
    return { label: "双相机自动采样", detail: null };
  }
  const phase = progress.phase === "eye_in_hand"
    ? "D405"
    : progress.phase === "eye_to_hand" ? "D455" : "手眼标定";
  if (progress.manual) {
    const target = Number(progress.target);
    const attempted = Number(progress.attempted ?? progress.current ?? 0);
    const details = [
      `有效 ${progress.valid ?? 0}${Number.isFinite(target) ? ` / ${target}` : ""}`,
    ];
    if (Number.isFinite(attempted)) details.push(`尝试 ${attempted}`);
    return {
      label: `${phase} 手动补采`,
      detail: details.join(" · "),
    };
  }
  const stage = progress.stage === "validation" ? "独立验证" : "自动采样";
  const attempted = Number(progress.attempted ?? progress.current ?? 0);
  const rawAttemptLimit = progress.attempt_limit ?? progress.total;
  const attemptLimit = rawAttemptLimit == null ? Number.NaN : Number(rawAttemptLimit);
  const valid = Number(progress.valid ?? 0);
  const target = Number(progress.target);
  const details = [];
  details.push(`有效 ${Number.isFinite(valid) ? valid : 0}${Number.isFinite(target) ? ` / ${target}` : ""}`);
  if (Number.isFinite(attemptLimit)) {
    details.push(`尝试 ${Number.isFinite(attempted) ? attempted : 0} / ${attemptLimit}`);
  }
  const estimated = formatHandeyeDuration(progress.estimated_remaining_s);
  const maximum = formatHandeyeDuration(progress.maximum_remaining_s);
  if (estimated) details.push(`预计剩余 ${estimated}`);
  else if (maximum) details.push(`最多约 ${maximum}`);
  return { label: `${phase} ${stage}`, detail: details.join(" · ") };
}

function handeyeQualityPhaseLabel(phase) {
  if (phase === "eye_in_hand") return "D405";
  if (phase === "eye_to_hand") return "D455";
  return "手眼标定";
}

function formatHandeyeQualityValue(value, unit) {
  const numeric = Number(value);
  if (!Number.isFinite(numeric)) return "—";
  if (unit === "%") return `${numeric.toFixed(1)}%`;
  if (unit === "mm") return `${numeric.toFixed(2)} mm`;
  if (unit === "deg") return `${numeric.toFixed(3)}°`;
  return String(numeric);
}

function renderHandeyeQuality(quality) {
  if (!ui.handeyeQualityPanel) return;
  if (!quality || typeof quality !== "object") {
    ui.handeyeQualityPanel.hidden = true;
    ui.handeyeQualityPanel.classList.remove("is-passing", "is-unavailable");
    return;
  }

  const available = quality.available === true;
  const accepted = quality.accepted === true;
  const sampleCount = Number(quality.sample_count);
  const fitSampleCount = Number(quality.fit_sample_count ?? quality.sample_count);
  const validationSampleCount = Number(quality.validation_sample_count);
  const minimumSamples = Number(quality.minimum_samples);
  const phase = handeyeQualityPhaseLabel(quality.phase);
  ui.handeyeQualityPanel.hidden = false;
  ui.handeyeQualityPanel.classList.toggle("is-passing", available && accepted);
  ui.handeyeQualityPanel.classList.toggle("is-unavailable", !available);
  if (ui.handeyeQualityStatus) {
    const sampleDetail = [
      Number.isFinite(fitSampleCount) ? `拟合 ${fitSampleCount}` : null,
      Number.isFinite(validationSampleCount) && validationSampleCount > 0
        ? `独立验证 ${validationSampleCount}`
        : null,
    ].filter(Boolean).join(" · ");
    if (quality.review_required === true) {
      ui.handeyeQualityStatus.textContent = `${phase} · 等待人工评审${sampleDetail ? ` · ${sampleDetail}` : ""}`;
    } else if (available) {
      ui.handeyeQualityStatus.textContent = `${phase} · ${accepted ? "达到目标" : "待改进"}${sampleDetail ? ` · ${sampleDetail}` : ""}`;
    } else if (Number.isFinite(sampleCount) && Number.isFinite(minimumSamples)) {
      ui.handeyeQualityStatus.textContent = `${phase} · 有效 ${sampleCount} / ${minimumSamples}`;
    } else {
      ui.handeyeQualityStatus.textContent = `${phase} · 暂不能评估`;
    }
  }
  if (ui.handeyeQualityMessage) {
    ui.handeyeQualityMessage.textContent = quality.detail || (available
      ? "当前结果与目标误差比较。"
      : "达到基本有效组数后开始实时误差评估。");
  }
  if (!ui.handeyeQualityMetrics) return;
  ui.handeyeQualityMetrics.replaceChildren();
  if (!available || !Array.isArray(quality.metrics)) return;
  for (const metric of quality.metrics) {
    if (!metric || typeof metric !== "object") continue;
    const row = document.createElement("div");
    row.className = `handeye-quality-metric${metric.diagnostic === true
      ? " diagnostic"
      : metric.passed === true ? " pass" : " fail"}`;
    const label = document.createElement("span");
    label.textContent = metric.label || "误差指标";
    const current = document.createElement("b");
    current.textContent = `当前 ${formatHandeyeQualityValue(metric.value, metric.unit)}`;
    const target = document.createElement("small");
    const compare = metric.direction === "min" ? "≥" : "≤";
    target.textContent = `${metric.diagnostic === true ? "参考" : "目标"} ${compare} ${formatHandeyeQualityValue(metric.target, metric.unit)}`;
    row.append(label, current, target);
    ui.handeyeQualityMetrics.append(row);
  }
}

function renderHandeyeCalibration(calibration = armState.handeye_calibration) {
  if (!ui.handeyeCalibrationPanel) return;
  const state = calibration ?? null;
  const active = Boolean(state?.active);
  const pending = handeyeLaunchPending;
  // Calibration JPEG frames arrive independently from feedback/state events.
  // Do not clear them on every state update, or the empty camera placeholder
  // races the preview and visibly flashes during collection.
  if (!active && !pending && !HANDEYE_PREVIEW_STAGES.has(state?.stage)) {
    clearHandeyePreview();
  }
  if (active || pending) {
    setHandeyePanelOpen(true);
    ui.cameraEmptyState?.classList.add("is-hidden");
  }
  ui.handeyeCalibrationStage.textContent = pending
    ? "释放设备 · 准备中"
    : (state?.stage === "cached" ? "已有缓存结果" : (state?.stage ?? "未开始"));
  ui.handeyeCalibrationMessage.textContent = pending
    ? "正在停止 D455/D405 视频并把机械臂控制权交给自动标定进程。"
    : (state?.message ?? "可单独标定 D405 或 D455；连续模式才会依次完成两台相机。");
  ui.handeyeCalibrationMessage.className = "handeye-calibration-message"
    + (state?.error ? " error" : state?.stage === "done" || state?.stage === "cached" ? " success" : "");
  const progress = state?.progress;
  const progressSummary = handeyeProgressSummary(progress);
  ui.handeyeCalibrationProgress.textContent = progressSummary.label;
  renderHandeyeQuality(state?.quality);
  [ui.handeyeD405Start, ui.handeyeD455Start, ui.handeyeCalibrationStart]
    .filter(Boolean)
    .forEach((button) => {
      button.disabled = active || pending || !ui.handeyeSafetyConfirm.checked;
      button.hidden = active;
    });
  const cancelling = ["cancelling", "cancel_center", "cancel_home", "cancel_remove"].includes(state?.stage);
  ui.handeyeCalibrationCancel.hidden = (!active && !pending) || cancelling;
  ui.handeyeCalibrationCancel.disabled = cancelling;
  ui.handeyeSafetyConfirm.disabled = active || pending;
  if (ui.handeyeReturnReference) {
    ui.handeyeReturnReference.disabled = active || pending;
  }
  ui.handeyeCalibrationConfirmActions.hidden = !active;
  ui.handeyeD405ReadyButton.hidden = state?.stage !== "ready";
  if (state?.stage === "ready") ui.handeyeD405ReadyButton.disabled = !handeyeCenterReady;
  ui.handeyeBoardReadyButton.hidden = state?.stage !== "board";
  ui.handeyeCenterReadyButton.hidden = state?.stage !== "center";
  if (state?.stage === "center") {
    ui.handeyeCenterReadyButton.disabled = !handeyeCenterReady;
  }
  ui.handeyeRunButton.hidden = state?.stage !== "plan";
  ui.handeyeManualHomeButton.hidden = state?.stage !== "manual";
  ui.handeyeManualSampleButton.hidden = state?.stage !== "manual";
  if (state?.stage === "manual") {
    ui.handeyeManualSampleButton.disabled = !handeyeCenterReady;
  }
  ui.handeyeManualDoneButton.hidden = state?.stage !== "manual";
  ui.handeyeManualDoneButton.disabled = !state?.manual_ready;
  ui.handeyeReviewMoreButton.hidden = state?.stage !== "review";
  ui.handeyeReviewAcceptButton.hidden = state?.stage !== "review";
  ui.handeyeRemoveButton.hidden = !["remove", "cancel_remove"].includes(state?.stage);
  const result = state?.result;
  if (result) {
    const externalClosure = Number(result.closure_translation_mm);
    const handClosure = Number(result.hand_closure_translation_mm);
    const externalRelocalization = Number(result.relocalization_translation_mm);
    const handRelocalization = Number(result.hand_relocalization_translation_mm);
    const counts = result.sample_counts ?? {};
    ui.handeyeCalibrationResult.textContent = result.dual
      ? [
          `D405 ${counts.eye_in_hand ?? "—"} 组`,
          Number.isFinite(handClosure) ? `${handClosure.toFixed(2)} mm RMS` : null,
          Number.isFinite(handRelocalization) ? `重定位 ${handRelocalization.toFixed(2)} mm` : null,
          `D455 ${counts.eye_to_hand ?? "—"} 组`,
          Number.isFinite(externalClosure) ? `${externalClosure.toFixed(2)} mm RMS` : null,
          Number.isFinite(externalRelocalization) ? `重定位 ${externalRelocalization.toFixed(2)} mm` : null,
        ].filter(Boolean).join(" · ")
      : [
          `${result.phase === "eye_in_hand" ? "D405" : "D455"} ${result.sample_count ?? "—"} 组 · ${result.best_method ?? "—"}`,
          Number.isFinite(externalClosure) ? `闭环 ${externalClosure.toFixed(2)} mm` : null,
          result.combined_profile_updated === true ? "已更新双相机配置" : null,
          result.standalone_profile_updated === true ? "已更新 D455 配置" : null,
        ].filter(Boolean).join(" · ");
  } else {
    ui.handeyeCalibrationResult.textContent = progressSummary.detail ?? "尚无结果";
  }
}

function renderHandeyeValidation(validation = armState.handeye_validation) {
  if (!ui.handeyeValidationStage) return;
  const state = validation ?? {};
  const enabled = state.enabled === true;
  const passed = state.passed === true;
  const available = state.available === true;
  const operatorReviewed = state.operator_reviewed === true;
  const canEnable = !state.stale
    && (passed || (state.validated === true && operatorReviewed));
  ui.handeyeValidationStage.textContent = enabled
    ? "已启用"
    : state.stale ? "缓存已更新" : passed ? "验证通过" : (available ? "待验证" : "无缓存");
  ui.handeyeValidationMessage.className = "handeye-validation-message"
    + (enabled || passed ? " success" : state.validated ? " error" : "");
  const checks = state.validation?.checks ?? {};
  const metric = (name, digits, suffix) => {
    const value = Number(checks[name]?.value);
    return Number.isFinite(value) ? `${value.toFixed(digits)} ${suffix}` : null;
  };
  const summary = [
    metric("closure_translation_rms_mm", 2, "mm RMS"),
    metric("closure_rotation_rms_deg", 2, "deg RMS"),
  ].filter(Boolean).join(" · ");
  ui.handeyeValidationMessage.textContent = enabled
    ? `已启用 D405 与 D455 外参；小方块坐标转换使用 T_base_camera${summary ? ` · ${summary}` : ""}。`
    : state.stale
      ? "当前求解已更新；请重新验证后再启用。"
      : passed
    ? `质量门通过${summary ? ` · ${summary}` : ""}，现在可以启用。`
      : state.validated
        ? operatorReviewed
          ? "质量建议阈值未通过；两阶段 20 组已人工评审，可明确确认后采用。"
          : "质量建议阈值未通过，结果不会用于抓取。"
        : available ? "点击验证缓存结果，检查质量门。" : "尚无缓存标定结果。";
  ui.handeyeValidateButton.disabled = !available || Boolean(armState.handeye_calibration?.active);
  ui.handeyeEnableButton.disabled = !canEnable || enabled;
  ui.handeyeDisableButton.hidden = !enabled;
}

function updateState(nextState, { renderUi = true } = {}) {
  armState = nextState.connected === false
    ? {
        connected: false,
        mode: nextState.mode ?? "disconnected",
        busy: Boolean(nextState.busy),
        servo_active: false,
        servo_kind: null,
        payload_profile: nextState.payload_profile ?? null,
        payload_mass_kg: nextState.payload_mass_kg ?? 0,
        payload_center_of_mass_tcp_m:
          nextState.payload_center_of_mass_tcp_m ?? null,
        payload_fit_rmse_nm: nextState.payload_fit_rmse_nm ?? null,
        payload_mount_mode: nextState.payload_mount_mode ?? null,
        payload_calibration: nextState.payload_calibration ?? null,
        gripper_calibration: nextState.gripper_calibration ?? null,
        handeye_calibration: nextState.handeye_calibration ?? null,
        handeye_validation: nextState.handeye_validation ?? null,
        cube_manipulation: nextState.cube_manipulation ?? { stage: "idle" },
      }
    : { ...armState, ...nextState };
  if (!renderUi) {
    publishArmState();
    return;
  }
  const connected = Boolean(armState.connected);
  const mode = armState.mode ?? "disconnected";
  ui.controlMode.textContent = modeNames[mode] ?? mode;
  const activityText = armState.servo_active
    ? "实时伺服"
    : (armState.busy ? "执行中" : "空闲");
  setControlBusy(activityText, Boolean(armState.servo_active || armState.busy));
  const feedbackAgeMs = Number(armState.state_age_ms);
  ui.feedbackAge.textContent = Number.isFinite(feedbackAgeMs)
    ? feedbackAgeMs.toFixed(0)
    : "—";
  ui.feedbackAge.closest(".feedback-metric").title = Number.isFinite(feedbackAgeMs)
    ? `机械臂反馈延迟 ${feedbackAgeMs.toFixed(0)} ms`
    : "机械臂反馈延迟不可用";
  ui.armGain.textContent = Number.isFinite(armState.arm_gain_percent)
    ? `${armState.arm_gain_percent.toFixed(0)}%`
    : "—";
  ui.gripperGain.textContent = Number.isFinite(armState.gripper_gain_percent)
    ? `${armState.gripper_gain_percent.toFixed(0)}%`
    : "—";
  const payloadMassKg = Number(armState.payload_mass_kg);
  const payloadMassG = Number.isFinite(payloadMassKg)
    ? Math.max(0, payloadMassKg * 1000.0)
    : 0;
  const payloadProfile = typeof armState.payload_profile === "string"
    ? armState.payload_profile
    : null;
  ui.payloadWeight.textContent = `${payloadMassG.toFixed(0)} g`;
  ui.payloadProfile.textContent = payloadProfile ?? "空载";
  ui.payloadMass.textContent = `${payloadMassG.toFixed(1)} g`;
  ui.payloadMetric.classList.toggle("active", Boolean(payloadProfile));
  const center = armState.payload_center_of_mass_tcp_m;
  const centerDescription = Array.isArray(center) && center.length === 3
    && center.every(Number.isFinite)
    ? `；TCP 重心 ${center.map((value) => (1000 * value).toFixed(1)).join(" / ")} mm`
    : "";
  const rmse = Number(armState.payload_fit_rmse_nm);
  const fitDescription = Number.isFinite(rmse)
    ? `；拟合 RMSE ${rmse.toFixed(3)} N·m`
    : "";
  const mountDescription = armState.payload_mount_mode === "grasped"
    ? "；夹取物体"
    : armState.payload_mount_mode === "fixed" ? "；固定负载" : "";
  ui.payloadMetric.title = payloadProfile
    ? `当前负载档案：${payloadProfile}；估计质量 ${payloadMassG.toFixed(1)} g${mountDescription}${centerDescription}${fitDescription}。当前仅显示档案，尚未写入 SDK 重力补偿。`
    : "当前负载档案：空载；估计质量 0 g。";
  renderPayloadCalibration(armState.payload_calibration);
  renderGripperCalibration(armState.gripper_calibration);
  renderHandeyeCalibration(armState.handeye_calibration);
  renderHandeyeValidation(armState.handeye_validation);

  if (!connected) {
    setPill("未连接", "pending");
  } else if (mode === "fault") {
    setPill("故障阻尼", "bad");
    if (armState.fault_reason) {
      setMessage(`故障阻尼：${armState.fault_reason}`, "error");
    }
  } else if (mode === "active") {
    setPill(armState.servo_active ? "实时伺服" : "关节位置", "ok");
  } else {
    setPill("自由阻尼", "ok");
  }

  const positions = armState.joint_position_deg;
  const minimums = armState.joint_min_deg;
  const maximums = armState.joint_max_deg;
  const torques = armState.joint_torque_nm;
  const references = armState.reference_joint_position_deg;
  jointCards.forEach((item, index) => {
    const position = positions?.[index];
    const minimum = minimums?.[index];
    const maximum = maximums?.[index];
    item.angle.textContent = Number.isFinite(position) ? `${position.toFixed(1)}°` : "—°";
    item.minimum.textContent = Number.isFinite(minimum) ? `${minimum.toFixed(0)}°` : "—";
    item.maximum.textContent = Number.isFinite(maximum) ? `${maximum.toFixed(0)}°` : "—";
    item.torque.textContent = Number.isFinite(torques?.[index])
      ? `${torques[index].toFixed(2)} N·m`
      : "— N·m";
    item.reference.textContent = Number.isFinite(references?.[index])
      ? `参考 ${references[index].toFixed(1)}°`
      : "参考 —°";
    item.current = Number.isFinite(position) ? position : null;
    if (Number.isFinite(position) && Number.isFinite(minimum) && Number.isFinite(maximum)) {
      const safeMinimum = minimum + 1;
      const safeMaximum = maximum - 1;
      const moveMinimum = safeMinimum;
      const moveMaximum = safeMaximum;
      item.slider.min = moveMinimum.toFixed(1);
      item.slider.max = moveMaximum.toFixed(1);
      item.moveWindow.textContent = `本次 ${moveMinimum.toFixed(1)}° … ${moveMaximum.toFixed(1)}°`;
      if (!item.dragging && !item.commandPending && !armState.busy) {
        item.slider.value = position.toFixed(1);
        updateJointTargetPreview(item);
      }
    } else {
      item.target.textContent = "—°";
      item.moveWindow.textContent = "本次范围 —";
    }
  });

  const gripper = armState.gripper_position_mm;
  ui.gripperActual.textContent = Number.isFinite(gripper) ? `${gripper.toFixed(1)} mm` : "— mm";
  if (Number.isFinite(gripper) && document.activeElement !== ui.gripperWidth
      && ui.gripperWidth.dataset.previewDirty !== "true" && !armState.busy) {
    syncGripperInput(gripper);
  } else if (!connected) {
    syncGripperInput(0);
  }
  ui.gripperReference.textContent = Number.isFinite(armState.reference_gripper_position_mm)
    ? `参考 ${armState.reference_gripper_position_mm.toFixed(1)} mm`
    : "参考 — mm";
  ui.gripperTorqueActual.textContent = Number.isFinite(armState.gripper_torque_nm)
    ? `${armState.gripper_torque_nm.toFixed(3)} N·m`
    : "—";
  if (Number.isFinite(armState.gripper_max_mm)) {
    ui.gripperWidth.max = String(armState.gripper_max_mm);
    ui.gripperMinimum.max = String(armState.gripper_max_mm);
  }
  updateControls();
  publishArmState();
}

function updateControls() {
  const connected = Boolean(armState.connected);
  const busy = Boolean(armState.busy || armState.servo_active);
  const active = connected && armState.mode === "active";
  const damping = connected && armState.mode === "damping";
  const fault = connected && armState.mode === "fault";
  const calibrationActive = Boolean(armState.payload_calibration?.active);
  const gripperCalibrationActive = Boolean(armState.gripper_calibration?.active);
  const handeyeActive = Boolean(armState.handeye_calibration?.active);
  const motionAvailable = connected && (active || damping) && !busy;
  const socketConnecting = controlSocket?.readyState === WebSocket.CONNECTING;
  const connectionBusy = socketConnecting || Boolean(connectionTransition);
  const disconnecting = connected && connectionTransition === "disconnect";
  const connectionLabel = disconnecting
    ? "断开中"
    : (connected
      ? "断开"
      : (connectionBusy ? "连接中" : (connectionFailed ? "重新连接" : "连接")));
  const connectionTitle = connected
    ? "断开机械臂"
    : (connectionFailed ? "重新连接机械臂" : "连接机械臂");
  ui.armModeSwitch.hidden = !connected;
  ui.armConnectButton.disabled = busy || connectionBusy || calibrationActive
    || gripperCalibrationActive || handeyeActive;
  ui.armConnectButton.classList.toggle("primary", !connected && !connectionBusy);
  ui.armConnectButton.classList.toggle("connected", connected && !disconnecting);
  ui.armConnectButton.classList.toggle(
    "retry",
    !connected && !connectionBusy && connectionFailed,
  );
  const connectionIcon = connectionFailed && !connected
    ? "fa-solid fa-rotate-right"
    : "fa-solid fa-robot";
  ui.armConnectButton.innerHTML = connectionBusy
    ? `<i class="fa-solid fa-circle-notch fa-spin" aria-hidden="true"></i><span>${connectionLabel}</span>`
    : `<i class="${connectionIcon}" aria-hidden="true"></i><span>${connectionLabel}</span>`;
  ui.armConnectButton.setAttribute("aria-label", connectionBusy ? connectionLabel : connectionTitle);
  const statusText = ui.armConnectButton.dataset.statusText ?? modeNames[armState.mode] ?? "未知";
  ui.armConnectButton.title = `${connectionBusy ? connectionLabel : connectionTitle}；当前状态：${statusText}`;
  ui.armModeButtons.forEach((button) => {
    const mode = button.dataset.armMode;
    const selected = selectedArmMode === mode;
    button.disabled = !connected || fault || busy || connectionBusy;
    button.classList.toggle("selected", selected);
    button.setAttribute("aria-pressed", String(selected));
    const modeLabel = mode === "active" ? "关节位置控制" : "自由阻尼";
    button.title = selected
      ? `动作完成后保持${modeLabel}`
      : fault
        ? "故障阻尼中，请先清除故障"
        : `切换到${modeLabel}，并作为动作完成后的模式`;
  });
  ui.setReferenceButton.disabled = !connected || busy || fault;
  ui.parkButton.disabled = !motionAvailable
    || parkPreviewRequestId !== null
    || !Array.isArray(armState.reference_joint_position_deg);
  ui.clearFaultButton.disabled = !fault || busy;
  ui.emergencyButton.disabled = !connected && !handeyeActive && !gripperCalibrationActive;
  ui.motionSpeed.disabled = busy;
  ui.payloadFixedButton.disabled = connected || calibrationActive || gripperCalibrationActive || busy;
  ui.payloadGraspedButton.disabled = connected || calibrationActive || gripperCalibrationActive || busy;
  ui.payloadCalibrationProfile.disabled = calibrationActive;
  [ui.handeyeD405Start, ui.handeyeD455Start, ui.handeyeCalibrationStart]
    .filter(Boolean)
    .forEach((button) => {
      button.disabled = handeyeActive || gripperCalibrationActive || handeyeLaunchPending
        || !ui.handeyeSafetyConfirm.checked;
    });
  if (ui.handeyeSafetyConfirm) ui.handeyeSafetyConfirm.disabled = handeyeActive || handeyeLaunchPending;
  if (ui.handeyeReturnReference) ui.handeyeReturnReference.disabled = handeyeActive || handeyeLaunchPending;
  for (const [index, item] of jointCards.entries()) {
    item.buttons.forEach((button) => { button.disabled = !motionAvailable; });
    item.slider.disabled = !motionAvailable;
    item.resetButton.disabled = !motionAvailable || !Number.isFinite(
      armState.reference_joint_position_deg?.[index]
    );
  }
  ui.gripperMoveButton.disabled = !motionAvailable;
  ui.gripperForceButton.disabled = !motionAvailable;
  ui.gripperQuickButtons.forEach((button) => { button.disabled = !motionAvailable; });
  ui.gripperResetButton.disabled = !motionAvailable;
  if (ui.gripperCalibrationStart) {
    ui.gripperCalibrationStart.disabled = connected || busy || calibrationActive
      || gripperCalibrationActive || handeyeActive
      || !ui.gripperCalibrationSafetyConfirm.checked;
  }
  if (!connected) {
    ui.gripperHint.textContent = "请先连接机械臂。";
  } else if (fault) {
    ui.gripperHint.textContent = "当前为故障阻尼，请先查看错误并清除故障。";
  } else if (damping) {
    ui.gripperHint.textContent = "可以操作；动作会临时启用位置控制，结束后按状态栏选择恢复。";
  } else if (busy) {
    ui.gripperHint.textContent = "正在执行上一动作，请等待完成。";
  } else {
    const firstMove = Number(armState.gripper_gain_percent) === 0;
    ui.gripperHint.textContent = firstMove
      ? "可以控制；首次位置动作会先缓慢启用夹爪增益。"
      : "位置控制用于预张开；抓取物体使用限扭矩夹取，姿态在 XYZ 旋转模式调整。";
  }
  syncCubePickButton();
}

createJointCards();
updateState(armState);

ui.armConnectButton.addEventListener("click", () => {
  if (!armState.connected) {
    connectControl();
    return;
  }
  cancelParkPreview();
  connectionTransition = "disconnect";
  updateControls();
  connectionTransitionRequestId = sendCommand("disconnect");
  if (connectionTransitionRequestId === null) {
    connectionTransition = null;
    updateControls();
  }
});
ui.armModeButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const mode = button.dataset.armMode;
    if (selectedArmMode === mode && armState.mode === mode) return;
    selectedArmMode = mode;
    updateControls();
    if (mode === "active") modeSelectionRequestId = sendCommand("activate");
    else if (mode === "damping") {
      cancelParkPreview();
      modeSelectionRequestId = sendCommand("damping");
    }
    if (modeSelectionRequestId === null) {
      selectedArmMode = ["active", "damping"].includes(armState.mode)
        ? armState.mode
        : "damping";
      updateControls();
    }
  });
});
ui.setReferenceButton.addEventListener("click", () => {
  cancelParkPreview();
  sendCommand("set_reference");
});
ui.parkButton.addEventListener("click", requestParkAction);
ui.clearFaultButton.addEventListener("click", () => sendCommand("clear_fault"));
ui.cubePickButton?.addEventListener("click", () => {
  if (!socketIsOpen()) {
    setMessage("控制服务尚未连接，请稍后重试。", "error");
    return;
  }
  if (!armState.connected) {
    setMessage("请先连接机械臂，再开始抓取方块。", "error");
    return;
  }
  if (armState.mode === "fault") {
    setMessage("当前为故障阻尼，请先清除故障。", "error");
    return;
  }
  let action;
  let params;
  if (cubePickPreviewReady) {
    action = "cube_pick_execute";
    // Execution is bound to the server-side preview snapshot.  The simple
    // RGB detector is expected to lose the cube as the arm occludes it.
    params = { ...getMotionSpeedParams(), confirmed: true };
  } else {
    const payload = cubePickPayload();
    if (!payload) {
      setMessage("请等待桌面 ROI 内的方块稳定跟踪后再抓取。", "error");
      return;
    }
    action = "cube_pick_preview";
    params = { ...payload, ...getMotionSpeedParams() };
  }

  if (armState.mode === "damping") {
    queuedMotion = { action, params };
    const activateRequestId = sendCommand("activate");
    if (activateRequestId === null) {
      queuedMotion = null;
    } else {
      setMessage("正在自动切换到关节位置控制，随后继续抓取操作……");
    }
    return;
  }
  if (armState.mode !== "active") {
    setMessage(`当前模式 ${armState.mode} 不能开始自动抓取。`, "error");
    return;
  }

  const requestId = sendCommand(action, params);
  if (requestId !== null && action === "cube_pick_preview") {
    cubePickPlanning = true;
    syncCubePickButton();
    window.dispatchEvent(new CustomEvent("arx5-cube-pick-state", {
      detail: { phase: "planning" },
    }));
  } else if (requestId !== null) {
    publishMotionCommand(action, params, requestId);
    window.dispatchEvent(new CustomEvent("arx5-cube-pick-state", {
      detail: { phase: "executing" },
    }));
  }
});
ui.cubeMoveButton?.addEventListener("click", () => {
  if (!socketIsOpen() || !armState.connected || armState.mode !== "active") {
    setMessage("请先连接机械臂并启用关节位置控制。", "error");
    return;
  }
  const payload = cubePickPayload();
  if (cubeStagePreviewReady === "move") {
    if (!payload) {
      cubeStagePreviewReady = null;
      cubeStagePreviewPose = null;
      syncCubeStageButtons();
      setMessage("目标方块跟踪已失效，请重新点选目标方块。", "error");
      return;
    }
    const params = { ...payload, ...getMotionSpeedParams(), confirmed: true };
    const requestId = sendCommand("cube_move_execute", params);
    publishMotionCommand("cube_move_execute", params, requestId);
    return;
  }
  if (armState.cube_manipulation?.stage !== "held") {
    setMessage("请先完成方块抓取和抬升。", "error");
    return;
  }
  if (!payload) {
    setMessage("请在 RGB 画面中点选另一个稳定方块作为堆叠目标。", "error");
    return;
  }
  sendCommand("cube_move_preview", {
    ...payload,
    ...getMotionSpeedParams(),
  });
});
ui.cubePlaceButton?.addEventListener("click", () => {
  if (!socketIsOpen() || !armState.connected || armState.mode !== "active") {
    setMessage("请先连接机械臂并启用关节位置控制。", "error");
    return;
  }
  if (cubeStagePreviewReady === "place") {
    const params = withPostMotionMode("cube_place_execute", {
      ...getMotionSpeedParams(),
      confirmed: true,
    });
    const requestId = sendCommand("cube_place_execute", params);
    publishMotionCommand("cube_place_execute", params, requestId);
    return;
  }
  if (armState.cube_manipulation?.stage !== "over_target") {
    setMessage("请先把持有的方块移动到目标方块上方。", "error");
    return;
  }
  sendCommand("cube_place_preview", getMotionSpeedParams());
});
ui.cubeHomeButton?.addEventListener("click", () => {
  if (armState.cube_manipulation?.stage !== "released") {
    setMessage("请先完成放置并松开方块。", "error");
    return;
  }
  requestParkAction();
});
ui.cubeStackButton?.addEventListener("click", () => {
  if (!socketIsOpen() || !armState.connected) {
    setMessage("请先连接机械臂。", "error");
    return;
  }
  if (!["active", "damping"].includes(armState.mode)
      || armState.busy || armState.servo_active
      || (armState.cube_manipulation?.stage ?? "idle") !== "idle") {
    setMessage("当前状态不能开始一键叠放。", "error");
    return;
  }
  const payloads = stackCubePayloads();
  if (!payloads) {
    setMessage("需要同时稳定识别至少两个方块；当前选中方块作为源方块。", "error");
    return;
  }
  const formatXy = (payload) => payload.base_xyz_m
    .slice(0, 2)
    .map((value) => `${(1000 * value).toFixed(0)}`)
    .join(", ");
  if (!window.confirm(
    `确认一键叠放？\n源方块：(${formatXy(payloads.source)}) mm`
    + `\n目标方块：(${formatXy(payloads.target)}) mm`
    + "\n系统会记录当前姿态，逐阶段安全预检，完成后返回并保持该姿态。"
    + "\n任一步失败都会立即停止并尽量保持当前位置。",
  )) {
    return;
  }
  const speed = getMotionSpeedParams();
  cubeStackAutomation = {
    ...payloads,
    speed,
    expectedAction: null,
    label: "准备",
  };
  syncCubeStageButtons();
  sendCubeStackStep("set_reference");
});
ui.emergencyButton.addEventListener("click", () => {
  requestEmergencyStop();
});
ui.motionSpeed.addEventListener("input", () => {
  cancelParkPreview();
  const speed = getMotionSpeedPercent();
  ui.motionSpeedValue.textContent = `${speed.toFixed(0)}%`;
  window.dispatchEvent(new CustomEvent("arx5-control-speed", {
    detail: getMotionSpeedParams(),
  }));
});
ui.gripperWidth.addEventListener("input", () => {
  syncGripperInput(ui.gripperWidth.value, { dirty: true });
});
ui.gripperMoveButton.addEventListener("click", () => {
  sendMotionCommand("gripper_position", { width_mm: Number(ui.gripperWidth.value) });
});
ui.gripperQuickButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const width = Number(button.dataset.width);
    syncGripperInput(width, { dirty: true });
    ui.gripperWidth.dispatchEvent(new Event("input", { bubbles: true }));
    sendMotionCommand("gripper_position", { width_mm: width });
  });
});
ui.gripperResetButton.addEventListener("click", () => {
  sendMotionCommand("reset_gripper");
});
ui.gripperForceButton.addEventListener("click", () => {
  sendMotionCommand("gripper_force", {
    torque_nm: Number(ui.gripperTorque.value),
    minimum_mm: Number(ui.gripperMinimum.value),
  });
});

async function launchGripperCalibration() {
  if (armState.connected) {
    setMessage("请先断开机械臂，再进行夹爪零点标定。", "error");
    return;
  }
  if (!ui.gripperCalibrationSafetyConfirm.checked) {
    setMessage("请先确认六轴已有支撑、夹爪周围无人无物体且实体急停可用。", "error");
    return;
  }
  try {
    await ensureControlSocket();
    sendCommand("gripper_calibration_start", { safety_confirmed: true });
  } catch (error) {
    setMessage(error.message, "error");
  }
}

function confirmGripperCalibration(stage) {
  sendCommand("gripper_calibration_confirm", { stage });
}

ui.gripperCalibrationStart?.addEventListener("click", launchGripperCalibration);
ui.gripperCalibrationClosedButton?.addEventListener("click", () => {
  confirmGripperCalibration("await_closed");
});
ui.gripperCalibrationOpenButton?.addEventListener("click", () => {
  confirmGripperCalibration("await_open");
});
ui.gripperCalibrationCancelButton?.addEventListener("click", () => {
  sendCommand("gripper_calibration_cancel");
});
ui.gripperCalibrationSafetyConfirm?.addEventListener("change", updateControls);

function preparePayloadCalibration(mode) {
  if (armState.connected) {
    setMessage("请先断开机械臂，再开始负载标定。", "error");
    return;
  }
  if (ui.payloadCalibrationProfile.value.trim() === "payload") {
    ui.payloadCalibrationProfile.value = mode === "grasped" ? "grasped-object" : "fixed-load";
  }
  const profile = ui.payloadCalibrationProfile.value.trim();
  if (!profile) {
    setMessage("请填写负载档案名。", "error");
    ui.payloadCalibrationProfile.focus();
    return;
  }
  pendingPayloadCalibrationMode = mode;
  renderPayloadCalibration(null);
}

async function launchPayloadCalibration() {
  const mode = pendingPayloadCalibrationMode;
  if (!mode) return;
  const profile = ui.payloadCalibrationProfile.value.trim();
  if (!profile) {
    setMessage("请填写负载档案名。", "error");
    ui.payloadCalibrationProfile.focus();
    return;
  }
  ui.payloadCalibrationMessage.textContent = "正在连接标定服务……";
  try {
    await ensureControlSocket();
    sendCommand("payload_calibration_start", {
      mode,
      profile,
      safety_confirmed: true,
    });
  } catch (error) {
    setMessage(error.message, "error");
  }
}

async function startHandeyeAfterRelease() {
  if (!handeyeLaunchPending) return;
  // The service acknowledged release, then exits and is restarted by its
  // supervisor so the kernel closes librealsense's remaining UVC descriptors.
  await new Promise((resolve) => window.setTimeout(resolve, 1500));
  if (!handeyeLaunchPending) return;
  const requestId = sendCommand("handeye_calibration_start", {
    mode: handeyeLaunchMode,
    safety_confirmed: true,
    return_to_reference_after_complete: ui.handeyeReturnReference?.checked !== false,
    scene_hint: typeof window.arx5GetSceneCalibration === "function"
      ? window.arx5GetSceneCalibration()
      : null,
  });
  if (requestId === null) {
    handeyeLaunchPending = false;
    handeyeLaunchMode = "dual";
    restoreHandeyeCamera();
    renderHandeyeCalibration(armState.handeye_calibration);
  }
}

async function launchHandeyeCalibration(mode = "dual") {
  if (handeyeLaunchPending || armState.handeye_calibration?.active) return;
  if (!ui.handeyeSafetyConfirm.checked) {
    setMessage("请先确认标定板、所选相机视野、工作区和实体急停均已准备好。", "error");
    return;
  }
  handeyeLaunchMode = mode;
  handeyeLaunchPending = true;
  handeyeCameraWasActive = Boolean(window.arx5CameraIsActive?.());
  try {
    if (handeyeCameraWasActive) {
      await window.arx5ReleaseCameraForCalibration?.();
      window.arx5StopCamera?.();
    }
    renderHandeyeCalibration(armState.handeye_calibration);
    await ensureControlSocket();
    if (armState.connected && armState.mode !== "damping") {
      selectedArmMode = "damping";
      updateControls();
      modeSelectionRequestId = sendCommand("damping");
      if (modeSelectionRequestId === null) {
        handeyeLaunchPending = false;
        handeyeLaunchMode = "dual";
        restoreHandeyeCamera();
        updateControls();
      }
      return;
    }
    startHandeyeAfterRelease();
  } catch (error) {
    handeyeLaunchPending = false;
    handeyeLaunchMode = "dual";
    restoreHandeyeCamera();
    renderHandeyeCalibration(armState.handeye_calibration);
    setMessage(error.message, "error");
  }
}

function cancelHandeyeCalibration() {
  if (handeyeLaunchPending && !armState.handeye_calibration?.active) {
    handeyeLaunchPending = false;
    handeyeLaunchMode = "dual";
    restoreHandeyeCamera();
    renderHandeyeCalibration(armState.handeye_calibration);
    return;
  }
  const stage = armState.handeye_calibration?.stage;
  if (![null, undefined, "starting", "board", "table_check"].includes(stage)
      && !window.confirm("取消后机械臂会安全停稳；若标定板在夹爪中，会先提示托住并张开夹爪，然后回到零位。")) {
    return;
  }
  sendCommand("handeye_calibration_cancel");
}

function confirmHandeyeCalibration(stage) {
  sendCommand("handeye_calibration_confirm", { stage });
}

function confirmHandeyeManual(action) {
  sendCommand("handeye_calibration_confirm", {
    stage: "manual",
    manual_action: action,
  });
}

function confirmHandeyeReview(action) {
  sendCommand("handeye_calibration_confirm", {
    stage: "review",
    review_action: action,
  });
}

function confirmPayloadCalibration(stage) {
  sendCommand("payload_calibration_confirm", { stage });
}

ui.payloadFixedButton?.addEventListener("click", () => preparePayloadCalibration("fixed"));
ui.payloadGraspedButton?.addEventListener("click", () => preparePayloadCalibration("grasped"));
ui.payloadSafetyButton?.addEventListener("click", launchPayloadCalibration);
ui.payloadSafetyBackButton?.addEventListener("click", () => {
  pendingPayloadCalibrationMode = null;
  renderPayloadCalibration(armState.payload_calibration);
});
ui.payloadCenterReadyButton?.addEventListener("click", () => confirmPayloadCalibration("center"));
ui.payloadRunButton?.addEventListener("click", () => confirmPayloadCalibration("run"));
ui.payloadReleaseButton?.addEventListener("click", () => confirmPayloadCalibration("releasing"));
ui.payloadCancelButton?.addEventListener("click", () => sendCommand("payload_calibration_cancel"));
ui.handeyeD405Start?.addEventListener("click", () => launchHandeyeCalibration("eye_in_hand"));
ui.handeyeD455Start?.addEventListener("click", () => launchHandeyeCalibration("eye_to_hand"));
ui.handeyeCalibrationStart?.addEventListener("click", () => launchHandeyeCalibration("dual"));
ui.handeyeCalibrationCancel?.addEventListener("click", cancelHandeyeCalibration);
ui.handeyeD405ReadyButton?.addEventListener("click", () => confirmHandeyeCalibration("ready"));
ui.handeyeBoardReadyButton?.addEventListener("click", () => confirmHandeyeCalibration("board"));
ui.handeyeCenterReadyButton?.addEventListener("click", () => confirmHandeyeCalibration("center"));
ui.handeyeRunButton?.addEventListener("click", () => confirmHandeyeCalibration("plan"));
ui.handeyeManualHomeButton?.addEventListener("click", () => {
  if (window.confirm("机械臂将保持夹紧标定板并回到六关节零位。请确认回零路径无桌面、D455、线缆和人员。")) {
    confirmHandeyeManual("home");
  }
});
ui.handeyeManualSampleButton?.addEventListener("click", () => confirmHandeyeManual("sample"));
ui.handeyeManualDoneButton?.addEventListener("click", () => confirmHandeyeManual("done"));
ui.handeyeReviewMoreButton?.addEventListener("click", () => confirmHandeyeReview("more"));
ui.handeyeReviewAcceptButton?.addEventListener("click", () => confirmHandeyeReview("accept"));
ui.handeyeRemoveButton?.addEventListener("click", () => confirmHandeyeCalibration(
  armState.handeye_calibration?.stage === "cancel_remove" ? "cancel_remove" : "remove",
));
ui.handeyeSafetyConfirm?.addEventListener("change", () => {
  renderHandeyeCalibration(armState.handeye_calibration);
});
ui.handeyeValidateButton?.addEventListener("click", () => sendCommand("handeye_validate"));
ui.handeyeEnableButton?.addEventListener("click", () => {
  const validation = armState.handeye_validation ?? {};
  if (validation.passed === true) {
    sendCommand("handeye_enable");
    return;
  }
  if (validation.operator_reviewed === true
      && window.confirm("质量建议阈值未通过。你已完成两阶段 20 组人工评审，确认仍要采用该外参用于方块坐标转换吗？")) {
    sendCommand("handeye_enable", { operator_override: true });
  }
});
ui.handeyeDisableButton?.addEventListener("click", () => sendCommand("handeye_disable"));
window.addEventListener("arx5-cube-detection", (event) => {
  cubeDetection = event.detail ?? null;
  if (cubeStagePreviewReady === "move" && cubeStagePreviewPose
      && cubeDetection?.pose_base?.base_xyz_m) {
    const current = cubeDetection.pose_base.base_xyz_m.map(Number);
    if (Math.hypot(
      current[0] - cubeStagePreviewPose[0],
      current[1] - cubeStagePreviewPose[1],
    ) > 0.015) {
      cubeStagePreviewReady = null;
      cubeStagePreviewPose = null;
      setMessage("目标方块位置已变化，移动预览已失效。", "error");
    }
  }
  syncCubePickButton();
});
setInterval(() => {
  if (cubePickPreviewReady) syncCubePickButton();
}, 1000);
setInterval(() => {
  if (!socketIsOpen() || !armState.connected || armState.busy
      || armState.servo_active || controlStateRequestId !== null) return;
  controlStateRequestId = sendCommand("state");
}, CONTROL_FEEDBACK_INTERVAL_MS);

window.addEventListener("beforeunload", () => {
  if (controlSocket) {
    if (["teleoperation", "teleoperation_preview"].includes(armState.servo_kind)) {
      sendTeleoperationMessage("stop", {}, activeTeleoperationSessionId);
    } else if (armState.servo_active) {
      sendServoMessage("stop", {}, armState.servo_kind ?? "cartesian");
    }
    controlSocket.close();
  }
});

window.addEventListener("arx5-control-speed", () => {
  updateState(armState);
});

syncGripperInput(ui.gripperWidth.value);
syncParkButton();
