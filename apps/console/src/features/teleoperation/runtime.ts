// @ts-nocheck
const MEDIAPIPE_RUNTIME_URL = "/mediapipe/vision_bundle.mjs";
const {
  DrawingUtils,
  FilesetResolver,
  HandLandmarker,
} = await import(/* @vite-ignore */ MEDIAPIPE_RUNTIME_URL);
import {
  binaryHandState,
  ClutchLatch,
  HandOpenCloseLatch,
  handPoseFromLandmarks,
  isClosedFist,
  isOpenHandGesture,
  isOpenPalm,
  selectLeftAndRightHands,
  isLateTeleoperationStart,
  TeleoperationMapper,
} from "./math";
import {
  getArmState,
  sendTeleoperationMessage,
  setControlMessage,
} from "../control/runtime";
import handModelUrl from "@weights/mediapipe/hand_landmarker.task?url";

const WASM_ROOT = new URL("./mediapipe/wasm/", document.baseURI).href;
const HAND_MODEL = handModelUrl;
const CAMERA_CONSTRAINTS = Object.freeze({
  audio: false,
  video: {
    width: { ideal: 1280 },
    height: { ideal: 720 },
    frameRate: { ideal: 30, max: 30 },
    facingMode: "user",
  },
});
const BROWSER_PROFILE_LIMITS = Object.freeze({
  fine: Object.freeze({
    maxTranslationStepM: 0.005,
    maxRotationStepRad: 3 * Math.PI / 180,
  }),
  fast: Object.freeze({
    maxTranslationStepM: 0.015,
    maxRotationStepRad: 8 * Math.PI / 180,
  }),
});

const STATE_LABELS = Object.freeze({
  disabled: "未启用",
  preview: "只读预览",
  waiting: "张开左手以标定",
  starting: "正在启动",
  tracking: "跟随中",
  braking: "平滑停车",
  lost: "手部丢失",
  fault: "已停止",
});

class HandTeleoperationController {
  constructor(panel) {
    this.panel = panel;
    this.video = panel.querySelector("#handCameraVideo");
    this.canvas = panel.querySelector("#handCameraOverlay");
    this.context = this.canvas?.getContext("2d") ?? null;
    this.drawing = this.context ? new DrawingUtils(this.context) : null;
    this.cameraStatus = panel.querySelector("#handCameraStatus");
    this.cameraStatusText = panel.querySelector("#handCameraStatusText");
    this.cameraStartButton = panel.querySelector("#handCameraStart");
    this.enabledInput = panel.querySelector("#teleoperationEnabled");
    this.stateOutput = panel.querySelector("#teleoperationState");
    this.profileControl = panel.querySelector("#teleoperationProfile");
    this.openWidthInput = panel.querySelector("#teleoperationOpenWidth");
    this.closingTorqueInput = panel.querySelector("#teleoperationClosingTorque");
    this.fpsOutput = panel.querySelector("#teleoperationFps");
    this.inferenceOutput = panel.querySelector("#teleoperationInferenceMs");
    this.rttOutput = panel.querySelector("#teleoperationRttMs");
    this.targetAgeOutput = panel.querySelector("#teleoperationTargetAgeMs");
    this.servoPeriodOutput = panel.querySelector("#teleoperationServoPeriodMs");
    this.visionTools = document.querySelector("#visionToolModes");
    this.visionShell = document.querySelector(".vision-tool-shell");
    this.robotStateOutput = document.querySelector("#robotHandState");

    this.state = "disabled";
    this.handModeActive = false;
    this.stream = null;
    this.landmarker = null;
    this.landmarkerPromise = null;
    this.frameCallbackId = null;
    this.frameInFlight = false;
    this.lastInferenceTimestampUs = -Infinity;
    this.clutch = new ClutchLatch();
    this.gripperGesture = new HandOpenCloseLatch();
    this.mapper = null;
    this.sessionId = null;
    this.pendingStartSequence = null;
    this.pendingStartDeadline = 0;
    this.cancelledStartSequences = new Set();
    this.lastValidPairAt = -Infinity;
    this.lastSendAt = -Infinity;
    this.sentAt = new Map();
    this.frameCount = 0;
    this.frameWindowStartedAt = performance.now();
    this.smoothedInferenceMs = null;
    this.toolSwitchSequence = 0;
    this.stopWaiters = new Set();
  }

  get armed() {
    return Boolean(this.enabledInput?.checked);
  }

  get profile() {
    return this.profileControl?.dataset.selected === "fast" ? "fast" : "fine";
  }

  get openWidthMm() {
    return Math.max(0, Math.min(88, Number(this.openWidthInput?.value) || 40));
  }

  get closingTorqueNm() {
    return Math.max(0.05, Math.min(
      0.25,
      Number(this.closingTorqueInput?.value) || 0.15,
    ));
  }

  initialize() {
    this.cameraStartButton?.addEventListener("click", () => {
      if (!this.handModeActive) {
        this.renderCameraStatus("请先选择 Hand 控制模式。", "warning", true);
        return;
      }
      this.startCamera();
    });
    this.enabledInput?.addEventListener("change", () => this.rearm());
    this.profileControl?.addEventListener("click", (event) => {
      const button = event.target.closest("[data-profile]");
      if (!button) return;
      this.selectSegment(this.profileControl, button, "profile");
      this.rearm();
    });
    this.visionTools?.addEventListener("click", (event) => {
      const button = event.target.closest("[data-vision-tool]");
      if (!button || button.disabled) return;
      this.selectVisionTool(button.dataset.visionTool);
    });
    window.addEventListener("arx5-teleop-state", (event) => {
      this.handleTeleoperationState(event.detail);
    });
    window.addEventListener("arx5-teleoperation-mode", (event) => {
      this.setHandMode(Boolean(event.detail?.active));
    });
    window.addEventListener("arx5-state", (event) => {
      this.handleArmState(event.detail);
    });
    document.addEventListener("visibilitychange", () => {
      if (document.hidden) this.deactivate("页面已隐藏");
    });
    this.renderCameraStatus("Mac 摄像头未启用", "idle", true);
    this.renderState();
  }

  selectSegment(control, selected, key) {
    control.dataset.selected = selected.dataset[key];
    for (const button of control.querySelectorAll("button")) {
      const active = button === selected;
      button.classList.toggle("selected", active);
      button.setAttribute("aria-pressed", String(active));
    }
  }

  showVisionTool(tool) {
    if (!this.visionTools || !["gesture", "handeye"].includes(tool)) return;
    this.visionTools.dataset.selected = tool;
    if (this.visionShell) this.visionShell.dataset.visionTool = tool;
    for (const button of this.visionTools.querySelectorAll("[data-vision-tool]")) {
      const selected = button.dataset.visionTool === tool;
      button.classList.toggle("selected", selected);
      button.setAttribute("aria-pressed", String(selected));
    }
    this.panel.hidden = tool !== "gesture";
    const calibration = document.querySelector("#handeyeCalibrationPanel");
    if (calibration) calibration.hidden = tool !== "handeye";
    window.dispatchEvent(new CustomEvent("arx5-vision-tool", {
      detail: { tool },
    }));
  }

  waitForStop(timeoutMs = 500) {
    return new Promise((resolve) => {
      let settled = false;
      const finish = () => {
        if (settled) return;
        settled = true;
        window.clearTimeout(timer);
        this.stopWaiters.delete(finish);
        resolve();
      };
      const timer = window.setTimeout(finish, timeoutMs);
      this.stopWaiters.add(finish);
    });
  }

  async selectVisionTool(tool) {
    if (!this.visionTools || !["gesture", "handeye"].includes(tool)) return;
    const switchSequence = ++this.toolSwitchSequence;
    if (tool === "handeye") {
      const needsStopWait = Boolean(this.sessionId || this.pendingStartSequence);
      this.deactivate("视觉工具已切换");
      if (needsStopWait) await this.waitForStop();
      if (switchSequence !== this.toolSwitchSequence) return;
      this.showVisionTool("handeye");
      return;
    }
    this.showVisionTool("gesture");
    if (this.handModeActive) this.startCamera();
    else this.deactivate("Hand 模式未启用");
  }

  setHandMode(active) {
    this.handModeActive = active;
    if (!active) {
      this.deactivate("已离开 Hand 模式");
      return;
    }
    this.selectVisionTool("gesture");
  }

  handleArmState(armState) {
    const calibrationActive = Boolean(armState?.handeye_calibration?.active);
    const gestureButton = this.visionTools?.querySelector('[data-vision-tool="gesture"]');
    if (gestureButton) gestureButton.disabled = calibrationActive;
    if (calibrationActive && this.visionTools?.dataset.selected !== "handeye") {
      this.selectVisionTool("handeye");
    }
    if (!armState?.connected || armState.mode === "fault") {
      this.disengage(armState?.mode === "fault" ? "控制器故障" : "机械臂未连接");
      if (armState?.mode === "fault") this.stopCamera();
    }
    const telemetry = armState?.teleoperation;
    this.writeMetric(this.targetAgeOutput, telemetry?.target_age_ms, "ms");
    this.writeMetric(this.servoPeriodOutput, telemetry?.servo_period_ms, "ms");
  }

  async ensureLandmarker() {
    if (this.landmarker) return this.landmarker;
    if (!this.landmarkerPromise) {
      this.landmarkerPromise = (async () => {
        const vision = await FilesetResolver.forVisionTasks(WASM_ROOT);
        try {
          return await HandLandmarker.createFromOptions(vision, {
            baseOptions: { modelAssetPath: HAND_MODEL, delegate: "GPU" },
            runningMode: "VIDEO",
            numHands: 2,
            minHandDetectionConfidence: 0.60,
            minHandPresenceConfidence: 0.60,
            minTrackingConfidence: 0.60,
          });
        } catch (error) {
          console.warn("GPU Hand Landmarker unavailable; using CPU", error);
          return HandLandmarker.createFromOptions(vision, {
            baseOptions: { modelAssetPath: HAND_MODEL, delegate: "CPU" },
            runningMode: "VIDEO",
            numHands: 2,
            minHandDetectionConfidence: 0.60,
            minHandPresenceConfidence: 0.60,
            minTrackingConfidence: 0.60,
          });
        }
      })();
    }
    try {
      this.landmarker = await this.landmarkerPromise;
      return this.landmarker;
    } catch (error) {
      // A failed WASM/model load must be retryable without discarding a
      // working camera preview.
      this.landmarkerPromise = null;
      throw error;
    }
  }

  async startCamera() {
    if (!this.handModeActive || !this.video) return;
    if (!this.stream) {
      try {
        // Request camera access in direct response to selecting Hand. Waiting
        // for the model/WASM first can hide the native permission prompt.
        this.renderCameraStatus("正在请求 Mac 摄像头权限…", "warning");
        this.renderState("正在请求 Mac 摄像头权限…");
        this.stream = await navigator.mediaDevices.getUserMedia(CAMERA_CONSTRAINTS);
        if (!this.handModeActive) {
          this.stopCamera();
          return;
        }
        this.video.srcObject = this.stream;
        for (const track of this.stream.getVideoTracks()) {
          track.addEventListener("ended", () => this.deactivate("摄像头已停止"));
        }
        await this.video.play();
        this.renderCameraStatus("Mac 摄像头已连接", "ready");
      } catch (error) {
        const message = error?.name === "NotAllowedError"
          ? "请在浏览器地址栏允许 Mac 摄像头，然后重新进入 Hand 模式。"
          : error?.name === "NotFoundError"
            ? "未找到可用的 Mac 摄像头。"
            : `Mac 摄像头不可用：${error.message}`;
        this.state = "fault";
        this.renderState(message);
        setControlMessage(message, "error");
        this.stopCamera();
        this.renderCameraStatus(message, "error", true);
        return;
      }
    }

    try {
      this.renderCameraStatus("Mac 摄像头已连接，正在加载手势模型…", "warning");
      this.renderState("Mac 摄像头已连接，正在加载手势模型…");
      await this.ensureLandmarker();
      if (!this.handModeActive) {
        this.stopCamera();
        return;
      }
      this.state = this.armed ? "waiting" : "preview";
      this.scheduleFrame();
      this.renderCameraStatus("手势模型已就绪，等待双手入镜", "ready");
      this.renderState();
    } catch (error) {
      const message = `手势模型加载失败：${error.message}`;
      this.state = "fault";
      this.renderState(message);
      this.renderCameraStatus(message, "error", true);
      setControlMessage(message, "error");
    }
  }

  scheduleFrame() {
    if (!this.stream || !this.video || this.frameCallbackId !== null) return;
    this.frameCallbackId = this.video.requestVideoFrameCallback((now) => {
      this.frameCallbackId = null;
      this.processFrame(now).finally(() => this.scheduleFrame());
    });
  }

  async processFrame(nowMs) {
    if (this.frameInFlight || !this.stream || !this.landmarker) return;
    const timestampUs = Math.round(nowMs * 1000);
    if (timestampUs <= this.lastInferenceTimestampUs) return;
    this.lastInferenceTimestampUs = timestampUs;
    this.frameInFlight = true;
    try {
      const startedAt = performance.now();
      const result = this.landmarker.detectForVideo(this.video, timestampUs / 1000);
      const inferenceMs = performance.now() - startedAt;
      this.recordFrame(nowMs, inferenceMs);
      this.drawHands(result);

      if (this.pendingStartSequence && nowMs > this.pendingStartDeadline) {
        this.rememberCancelledStart(this.pendingStartSequence);
        this.pendingStartSequence = null;
        this.pendingStartDeadline = 0;
        this.clutch.reset();
        this.renderState("启动响应超时");
      }

      const selected = selectLeftAndRightHands(result);
      if (!selected) {
        const detectedHands = result.landmarks?.length ?? 0;
        this.renderCameraStatus(
          detectedHands === 1 ? "已识别 1 只手，需要双手同时入镜" : "未识别到双手",
          "warning",
        );
        if (nowMs - this.lastValidPairAt > 100) {
          this.state = "lost";
          this.disengage("手部丢失");
        }
        return;
      }
      this.lastValidPairAt = nowMs;
      this.renderCameraStatus("已识别双手", "ready");
      const armState = getArmState();
      if (!this.handModeActive || !armState.connected) {
        this.disengage("机械臂未连接");
        return;
      }
      if (this.armed && !["active", "damping"].includes(armState.mode)) {
        this.enabledInput.checked = false;
        this.disengage("当前模式不能启用");
        return;
      }

      const leftOpen = isOpenPalm(selected.left.landmarks);
      const leftFist = isClosedFist(selected.left.landmarks);
      const clutchEngaged = this.clutch.update(leftOpen, leftFist);
      if (!clutchEngaged) {
        this.renderCameraStatus("已识别双手，张开左手以标定当前位置", "ready");
        if (this.sessionId || this.pendingStartSequence) {
          this.disengage(
            leftFist ? "左手握拳，零增量已重设" : "左手控制未接通",
          );
        } else {
          this.state = this.armed ? "waiting" : "preview";
          this.renderState();
        }
        return;
      }
      const nowSeconds = nowMs / 1000;
      if (!leftOpen) {
        this.renderCameraStatus("左手姿态不明确，机械臂保持", "warning");
        return;
      }
      if (
        !this.sessionId
        && !this.pendingStartSequence
        && this.cancelledStartSequences.size === 0
      ) {
        await this.engage(selected.left, selected.right, nowSeconds);
        return;
      }
      if (this.pendingStartSequence) {
        this.mapper.engage(handPoseFromLandmarks(selected.left), nowSeconds);
        this.gripperGesture.reset(binaryHandState(
          isOpenHandGesture(selected.right.landmarks),
          isClosedFist(selected.right.landmarks),
          this.gripperGesture.state,
        ));
        return;
      }
      if (this.sessionId) {
        this.renderCameraStatus("双手识别正常，机械臂跟随中", "ready");
        this.sendTarget(
          selected.left,
          selected.right,
          nowSeconds,
        );
      }
    } catch (error) {
      console.error("Hand teleoperation frame failed", error);
      this.state = "fault";
      this.renderCameraStatus(`手部估计失败：${error.message}`, "error", true);
      this.disengage("手部估计失败");
    } finally {
      this.frameInFlight = false;
    }
  }

  async engage(controlHand, gripperHand, nowSeconds) {
    const config = getArmState().teleoperation_config;
    const profile = config?.profiles?.[this.profile];
    if (!config?.operator_to_base || !profile) {
      this.renderState("遥操作配置未就绪");
      return;
    }
    this.mapper = new TeleoperationMapper({
      operatorToBase: config.operator_to_base,
      translationGain: profile.translation_gain,
      rotationGain: profile.rotation_gain,
      maxLinearVelocityMps: profile.tcp_linear_velocity_m_s,
      maxAngularVelocityRadS: profile.tcp_angular_velocity_rad_s,
      ...BROWSER_PROFILE_LIMITS[this.profile],
    });
    this.mapper.engage(handPoseFromLandmarks(controlHand), nowSeconds);
    this.gripperGesture.reset(binaryHandState(
      isOpenHandGesture(gripperHand.landmarks),
      isClosedFist(gripperHand.landmarks),
      this.gripperGesture.state,
    ));
    this.pendingStartSequence = sendTeleoperationMessage("start", {
      profile: this.profile,
      control_side: "left",
      open_width_mm: this.openWidthMm,
      closing_torque_nm: this.closingTorqueNm,
      preview_only: !this.armed,
    });
    if (this.pendingStartSequence === null) {
      this.disengage("控制连接不可用");
      return;
    }
    this.sentAt.set(this.pendingStartSequence, performance.now());
    this.pendingStartDeadline = performance.now() + 500;
    this.state = "starting";
    this.renderState();
  }

  sendTarget(controlHand, gripperHand, nowSeconds) {
    if (!this.sessionId || nowSeconds - this.lastSendAt < 1 / 30) return;
    const mapped = this.mapper.update(
      handPoseFromLandmarks(controlHand),
      nowSeconds,
    );
    const sequence = sendTeleoperationMessage("update", {
      ...mapped,
      gripper_state: this.gripperGesture.update(
        isOpenHandGesture(gripperHand.landmarks),
        isClosedFist(gripperHand.landmarks),
      ),
    }, this.sessionId);
    if (sequence !== null) this.sentAt.set(sequence, performance.now());
    this.lastSendAt = nowSeconds;
  }

  handleTeleoperationState(message) {
    if (!message) return;
    const sentAt = this.sentAt.get(message.seq);
    if (sentAt !== undefined) {
      this.writeMetric(this.rttOutput, performance.now() - sentAt, "ms");
      this.sentAt.delete(message.seq);
    }
    if (isLateTeleoperationStart(message, this.cancelledStartSequences)) {
      this.cancelledStartSequences.delete(message.seq);
      if (message.ok && typeof message.session_id === "string") {
        sendTeleoperationMessage("stop", {}, message.session_id);
      }
      return;
    }
    if (message.action === "start" && message.seq === this.pendingStartSequence) {
      this.pendingStartSequence = null;
      this.pendingStartDeadline = 0;
      if (!message.ok || !message.session_id) {
        this.sessionId = null;
        this.clutch.reset();
        this.state = "fault";
        this.renderState(message.error ?? "遥操作启动失败");
        return;
      }
      this.sessionId = message.session_id;
      this.state = this.armed ? "tracking" : "preview";
      this.renderState();
    }
    if (message.action === "update") {
      const telemetry = message.state?.teleoperation;
      this.writeMetric(this.targetAgeOutput, telemetry?.target_age_ms, "ms");
      this.writeMetric(this.servoPeriodOutput, telemetry?.servo_period_ms, "ms");
      if (!message.ok && !message.preview) this.disengage(message.error);
    }
    if (message.action === "stop" || message.type === "socket_closed") {
      for (const finish of [...this.stopWaiters]) finish();
      this.sessionId = null;
      this.pendingStartSequence = null;
      this.cancelledStartSequences.clear();
      this.clutch.reset();
      this.gripperGesture.reset("open");
      this.state = this.armed ? "waiting" : "preview";
      this.renderState(message.error);
    }
  }

  disengage(reason) {
    if (this.sessionId) sendTeleoperationMessage("stop", {}, this.sessionId);
    const startPending = Boolean(this.pendingStartSequence);
    if (startPending) this.rememberCancelledStart(this.pendingStartSequence);
    this.sessionId = null;
    if (!startPending) {
      this.pendingStartSequence = null;
      this.pendingStartDeadline = 0;
    }
    this.clutch.reset();
    this.gripperGesture.reset("open");
    this.mapper = null;
    this.state = this.armed ? "waiting" : "disabled";
    this.renderState(reason);
  }

  rememberCancelledStart(sequence) {
    if (!Number.isInteger(sequence)) return;
    this.cancelledStartSequences.add(sequence);
    while (this.cancelledStartSequences.size > 8) {
      this.cancelledStartSequences.delete(this.cancelledStartSequences.values().next().value);
    }
  }

  rearm() {
    this.disengage("设置已更新");
    if (this.handModeActive) {
      this.state = this.armed ? "waiting" : "preview";
      this.startCamera();
    }
    this.renderState();
  }

  deactivate(reason) {
    this.disengage(reason);
    if (this.enabledInput) this.enabledInput.checked = false;
    this.stopCamera();
    this.state = "disabled";
    this.renderState(reason);
  }

  stopCamera() {
    if (this.frameCallbackId !== null && this.video?.cancelVideoFrameCallback) {
      this.video.cancelVideoFrameCallback(this.frameCallbackId);
    }
    this.frameCallbackId = null;
    this.lastInferenceTimestampUs = -Infinity;
    for (const track of this.stream?.getTracks?.() ?? []) track.stop();
    this.stream = null;
    if (this.video) this.video.srcObject = null;
    this.clearOverlay();
    this.renderCameraStatus(
      this.handModeActive ? "Mac 摄像头已停止" : "Mac 摄像头未启用",
      "idle",
      true,
    );
  }

  renderCameraStatus(message, kind = "idle", showButton = false) {
    if (this.cameraStatusText) this.cameraStatusText.textContent = message;
    if (this.cameraStatus) this.cameraStatus.dataset.kind = kind;
    if (this.cameraStartButton) this.cameraStartButton.hidden = !showButton;
  }

  drawHands(result) {
    if (!this.context || !this.canvas || !this.video) return;
    const width = this.video.videoWidth || 1280;
    const height = this.video.videoHeight || 720;
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    this.context.clearRect(0, 0, width, height);
    this.context.fillStyle = "#28c76f";
    for (const hand of result.landmarks ?? []) {
      for (const landmark of hand) {
        this.context.beginPath();
        this.context.arc((1 - landmark.x) * width, landmark.y * height, 3, 0, 2 * Math.PI);
        this.context.fill();
      }
    }
  }

  clearOverlay() {
    if (this.context && this.canvas) {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
    }
  }

  recordFrame(nowMs, inferenceMs) {
    this.frameCount += 1;
    this.smoothedInferenceMs = this.smoothedInferenceMs === null
      ? inferenceMs
      : 0.85 * this.smoothedInferenceMs + 0.15 * inferenceMs;
    if (nowMs - this.frameWindowStartedAt >= 500) {
      const fps = 1000 * this.frameCount / (nowMs - this.frameWindowStartedAt);
      this.writeMetric(this.fpsOutput, fps, "fps");
      this.writeMetric(this.inferenceOutput, this.smoothedInferenceMs, "ms");
      this.frameCount = 0;
      this.frameWindowStartedAt = nowMs;
    }
  }

  writeMetric(element, value, unit) {
    if (!element) return;
    const number = Number(value);
    element.textContent = Number.isFinite(number) ? `${number.toFixed(0)} ${unit}` : "—";
  }

  renderState(reason = null) {
    if (this.stateOutput) {
      this.stateOutput.textContent = reason || STATE_LABELS[this.state] || this.state;
    }
    if (this.robotStateOutput) {
      this.robotStateOutput.textContent = reason || STATE_LABELS[this.state] || this.state;
    }
    this.panel.dataset.state = this.state;
  }
}

const panel = document.querySelector("#handTeleoperationPanel");
if (!panel) {
  // Static pages can load the module without the teleoperation workspace.
} else {
  new HandTeleoperationController(panel).initialize();
}
