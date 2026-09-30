// @ts-nocheck
import * as THREE from "three";
import { TransformControls } from "three/addons/controls/TransformControls.js";

import {
  TARGET_EDIT_STATE,
  applyLocalRoll,
  cloneTargetPose,
  quaternionFromArray,
  quaternionToArray,
} from "./target-pose-math";

const STATE_LABELS = Object.freeze({
  [TARGET_EDIT_STATE.IDLE]: "空闲",
  [TARGET_EDIT_STATE.EDIT_POSITION]: "XYZ 平移调整",
  [TARGET_EDIT_STATE.TARGET_READY]: "目标就绪",
  [TARGET_EDIT_STATE.EDIT_ORIENTATION]: "XYZ 旋转调整",
});

function smoothStep(value) {
  const t = THREE.MathUtils.clamp(value, 0, 1);
  return t * t * (3 - 2 * t);
}

export class TargetPoseEditor {
  constructor({
    scene,
    camera,
    controls,
    canvas,
    onPoseChange = () => {},
    onStateChange = () => {},
    onTargetReady = () => {},
    onDragStart = () => {},
    onDragChange = () => {},
    onDragEnd = () => {},
  }) {
    this.scene = scene;
    this.camera = camera;
    this.controls = controls;
    this.canvas = canvas;
    this.onPoseChange = onPoseChange;
    this.onStateChange = onStateChange;
    this.onTargetReady = onTargetReady;
    this.onDragStart = onDragStart;
    this.onDragChange = onDragChange;
    this.onDragEnd = onDragEnd;
    this.state = TARGET_EDIT_STATE.IDLE;
    this.targetPose = null;
    this.actualPose = null;
    this.drag = null;
    this.stageCheckpoint = null;
    this.rollRadians = 0;
    this.modeActive = true;
    this.controlsVisible = true;
    this.interactionEnabled = true;
    this.validationState = "pending";
    this.cameraAnimation = null;
    this.controlScale = 1;
    this.controlPoseOverride = null;
    this.suppressTransformReady = false;

    this.root = new THREE.Group();
    this.root.name = "tcp-target-editor";
    scene.add(this.root);
    this.#createVisuals();
    this.#createTransformControls();
    this.#syncVisuals();
  }

  #createVisuals() {
    this.actualMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.007, 20, 14),
      new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false }),
    );
    this.actualMarker.name = "actual-tcp-marker";
    this.actualMarker.renderOrder = 12;
    this.root.add(this.actualMarker);

    this.targetMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.009, 22, 16),
      new THREE.MeshBasicMaterial({
        color: 0xffc06a,
        transparent: true,
        opacity: 0.96,
        depthTest: false,
      }),
    );
    this.targetMarker.name = "target-tcp-marker";
    this.targetMarker.renderOrder = 13;
    this.root.add(this.targetMarker);

    this.transformPivot = new THREE.Object3D();
    this.transformPivot.name = "target-6dof-pivot";
    this.root.add(this.transformPivot);

    this.translationArrowGroup = new THREE.Group();
    this.translationArrowGroup.name = "tcp-positive-translation-arrows";
    this.translationArrows = [
      ["X", new THREE.Vector3(1, 0, 0), 0xff5555],
      ["Y", new THREE.Vector3(0, 1, 0), 0x5fd86f],
      ["Z", new THREE.Vector3(0, 0, 1), 0x568cff],
    ].map(([axis, direction, color]) => {
      const arrow = new THREE.ArrowHelper(
        direction,
        new THREE.Vector3(),
        0.075,
        color,
        0.02,
        0.012,
      );
      arrow.name = `tcp-translate-${axis.toLowerCase()}-arrow`;
      [arrow.line, arrow.cone].forEach((part) => {
        part.material.depthTest = false;
        part.material.depthWrite = false;
        part.material.transparent = true;
        part.material.opacity = 0.94;
        part.renderOrder = 15;
      });
      this.translationArrowGroup.add(arrow);
      return { axis, arrow, color };
    });
    this.root.add(this.translationArrowGroup);
  }

  #createTransformControls() {
    this.transformControls = new TransformControls(this.camera, this.canvas);
    this.transformControls.setTranslationSnap(0.0005);
    this.transformControls.setMode("translate");
    this.transformControls.setSpace("world");
    this.transformControls.setSize(1.0);
    this.transformHelper = this.transformControls.getHelper();
    this.transformHelper.name = "tcp-translation-controls";
    this.scene.add(this.transformHelper);
    this.#simplifyTranslationGizmo();

    this.rotationControls = new TransformControls(this.camera, this.canvas);
    this.rotationControls.setRotationSnap(THREE.MathUtils.degToRad(1));
    this.rotationControls.setMode("rotate");
    this.rotationControls.setSpace("local");
    this.rotationControls.setSize(0.7);
    this.rotationHelper = this.rotationControls.getHelper();
    this.rotationHelper.name = "tcp-rotation-controls";
    this.scene.add(this.rotationHelper);

    this.handleTranslationObjectChange = () => this.#transformObjectChanged("translate");
    this.handleRotationObjectChange = () => this.#transformObjectChanged("rotate");
    this.handleTranslationDraggingChanged = (event) => {
      this.#transformDraggingChanged("translate", event);
    };
    this.handleRotationDraggingChanged = (event) => {
      this.#transformDraggingChanged("rotate", event);
    };
    this.transformControls.addEventListener("objectChange", this.handleTranslationObjectChange);
    this.transformControls.addEventListener(
      "dragging-changed",
      this.handleTranslationDraggingChanged,
    );
    this.rotationControls.addEventListener("objectChange", this.handleRotationObjectChange);
    this.rotationControls.addEventListener(
      "dragging-changed",
      this.handleRotationDraggingChanged,
    );
  }

  #transformObjectChanged(kind) {
    if (!this.targetPose) return;
    if (kind === "translate") {
      this.targetPose.position = this.transformPivot.position.toArray();
      this.#poseChanged("axis-drag");
      return;
    }
    this.targetPose.orientation = quaternionToArray(this.transformPivot.quaternion);
    this.rollRadians = 0;
    this.#poseChanged("rotation-drag");
  }

  #transformDraggingChanged(kind, event) {
    const dragging = Boolean(event.value);
    if (dragging) {
      if (this.drag && this.drag.kind !== kind) return;
      this.controls.enabled = false;
      this.canvas.classList.add("target-edit-dragging");
      const editingState = kind === "translate"
        ? TARGET_EDIT_STATE.EDIT_POSITION
        : TARGET_EDIT_STATE.EDIT_ORIENTATION;
      if (this.state !== editingState) {
        this.stageCheckpoint = cloneTargetPose(this.targetPose);
        this.#setState(editingState);
      }
      this.drag = { kind };
      this.onDragStart(cloneTargetPose(this.targetPose), kind);
      this.#syncVisuals();
      return;
    }
    if (!this.drag || this.drag.kind !== kind) return;
    const completedMode = this.drag.kind;
    this.drag = null;
    this.canvas.classList.remove("target-edit-dragging");
    if (!this.cameraAnimation) this.controls.enabled = true;
    if (this.targetPose) {
      this.onDragEnd(
        cloneTargetPose(this.targetPose),
        completedMode,
        { cancelled: this.suppressTransformReady },
      );
    }
    if (!this.suppressTransformReady && this.targetPose) {
      this.#setState(TARGET_EDIT_STATE.TARGET_READY);
      this.onTargetReady(
        cloneTargetPose(this.targetPose),
        completedMode === "rotate" ? "orientation" : "position",
      );
    }
    this.#syncVisuals();
  }

  #simplifyTranslationGizmo() {
    // TransformControls keeps its mature axis picking and constrained-drag
    // math. Only its crowded translate visuals are replaced; rotate mode is
    // intentionally left untouched.
    const gizmo = this.transformControls?._gizmo;
    gizmo?.gizmo?.translate?.traverse((object) => {
      const materials = Array.isArray(object.material)
        ? object.material
        : object.material
          ? [object.material]
          : [];
      materials.forEach((material) => { material.visible = false; });
    });
    gizmo?.helper?.translate?.traverse((object) => {
      const materials = Array.isArray(object.material)
        ? object.material
        : object.material
          ? [object.material]
          : [];
      materials.forEach((material) => { material.visible = false; });
    });
    gizmo?.picker?.translate?.traverse((object) => {
      if (!object.isMesh) return;
      if (!["X", "Y", "Z"].includes(object.name)) {
        object.raycast = () => {};
        return;
      }
      object.geometry.computeBoundingBox();
      const center = object.geometry.boundingBox?.getCenter(new THREE.Vector3());
      const coordinate = center?.[object.name.toLowerCase()];
      if (!Number.isFinite(coordinate) || coordinate <= 0) object.raycast = () => {};
    });
  }

  setModeActive(active) {
    this.modeActive = Boolean(active);
    if (!this.modeActive) this.cancelPointer();
    this.#syncVisuals();
  }

  setInteractionEnabled(enabled) {
    const next = Boolean(enabled);
    if (next === this.interactionEnabled) return;
    this.interactionEnabled = next;
    if (!this.interactionEnabled) this.cancelPointer();
    this.#syncVisuals();
  }

  setControlsVisible(visible) {
    const next = Boolean(visible);
    if (next === this.controlsVisible) return;
    this.controlsVisible = next;
    if (!next) this.cancelPointer();
    this.#syncVisuals();
  }

  clearPose() {
    this.cancelPointer();
    this.targetPose = null;
    this.actualPose = null;
    this.controlPoseOverride = null;
    this.stageCheckpoint = null;
    this.rollRadians = 0;
    this.validationState = "pending";
    this.#setState(TARGET_EDIT_STATE.IDLE);
    this.#syncVisuals();
  }

  setActualPose(pose) {
    this.actualPose = cloneTargetPose(pose);
    if (this.actualPose) this.actualMarker.position.fromArray(this.actualPose.position);
    this.#syncVisuals();
  }

  syncToActualPose(pose) {
    const actualPose = cloneTargetPose(pose);
    if (!actualPose || this.drag) return false;
    this.targetPose = actualPose;
    this.controlPoseOverride = null;
    this.rollRadians = 0;
    this.validationState = "pending";
    if (this.state !== TARGET_EDIT_STATE.IDLE) this.#setState(TARGET_EDIT_STATE.IDLE);
    this.#syncVisuals();
    return true;
  }

  beginFeedbackFollow(pose) {
    const feedbackPose = cloneTargetPose(pose);
    if (!feedbackPose) return false;
    this.cancelPointer();
    this.controlPoseOverride = feedbackPose;
    if (this.targetPose) this.#setState(TARGET_EDIT_STATE.TARGET_READY);
    this.#syncVisuals();
    return true;
  }

  updateFeedbackFollow(pose) {
    const feedbackPose = cloneTargetPose(pose);
    if (!feedbackPose) return false;
    this.controlPoseOverride = feedbackPose;
    this.#syncVisuals();
    return true;
  }

  endFeedbackFollow() {
    if (!this.controlPoseOverride) return;
    this.controlPoseOverride = null;
    this.#syncVisuals();
  }

  setGripperWidth(widthMeters) {
    if (!this.targetPose || !Number.isFinite(widthMeters)) return;
    this.targetPose.gripperWidth = widthMeters;
  }

  setTargetPose(
    pose,
    { emit = false, resetRoll = false, markReady = true } = {},
  ) {
    this.targetPose = cloneTargetPose(pose);
    if (resetRoll) this.rollRadians = 0;
    this.validationState = "pending";
    if (markReady && this.targetPose && this.state === TARGET_EDIT_STATE.IDLE) {
      this.#setState(TARGET_EDIT_STATE.TARGET_READY);
    }
    this.#syncVisuals();
    if (emit && this.targetPose) {
      this.onPoseChange(cloneTargetPose(this.targetPose), "external");
    }
  }

  setRollRadians(value) {
    if (Number.isFinite(value)) this.rollRadians = value;
  }

  setValidationState(state) {
    this.validationState = ["pending", "valid", "invalid"].includes(state)
      ? state
      : "pending";
    const color = this.validationState === "valid"
      ? 0x79e0bf
      : this.validationState === "invalid"
        ? 0xff665f
        : 0xffc06a;
    this.targetMarker.material.color.setHex(color);
    this.#syncVisuals();
  }

  adjustRoll(deltaRadians, reason = "roll-key") {
    if (!this.targetPose || !Number.isFinite(deltaRadians)) return false;
    const previous = quaternionFromArray(this.targetPose.orientation);
    this.targetPose.orientation = quaternionToArray(applyLocalRoll(previous, deltaRadians));
    this.rollRadians += deltaRadians;
    this.validationState = "pending";
    this.#poseChanged(reason);
    return true;
  }

  confirmStage() {
    if (![TARGET_EDIT_STATE.EDIT_POSITION, TARGET_EDIT_STATE.EDIT_ORIENTATION]
      .includes(this.state)) return this.state === TARGET_EDIT_STATE.TARGET_READY;
    const completedKind = this.state === TARGET_EDIT_STATE.EDIT_ORIENTATION
      ? "orientation"
      : "position";
    const needsPreview = this.validationState === "pending";
    this.cancelPointer();
    this.#setState(TARGET_EDIT_STATE.TARGET_READY);
    this.#syncVisuals();
    if (needsPreview) this.onTargetReady(cloneTargetPose(this.targetPose), completedKind);
    return true;
  }

  cancelStage() {
    if (![TARGET_EDIT_STATE.EDIT_POSITION, TARGET_EDIT_STATE.EDIT_ORIENTATION]
      .includes(this.state)) return false;
    this.cancelPointer();
    if (this.stageCheckpoint) this.targetPose = cloneTargetPose(this.stageCheckpoint);
    this.#setState(this.targetPose
      ? TARGET_EDIT_STATE.TARGET_READY
      : TARGET_EDIT_STATE.IDLE);
    this.validationState = "pending";
    this.#poseChanged("cancel");
    return true;
  }

  handleShortcut(event) {
    if (!this.modeActive || event.metaKey || event.ctrlKey || event.altKey) return false;
    if (!this.interactionEnabled && event.code !== "Escape") return false;
    if (event.repeat) return ["Escape", "Enter"].includes(event.code);
    if (event.code === "Escape") return this.cancelStage();
    if (event.code === "Enter") return this.confirmStage();
    return false;
  }

  pointerDown() {
    return false;
  }

  pointerMove() {
    return false;
  }

  pointerUp() {
    return false;
  }

  cancelPointer() {
    const cancelledDrag = this.drag;
    this.suppressTransformReady = true;
    [this.transformControls, this.rotationControls].forEach((control) => {
      if (!control?.dragging) return;
      control.reset();
      control.dragging = false;
      control.axis = null;
    });
    this.suppressTransformReady = false;
    if (cancelledDrag && this.drag && this.targetPose) {
      this.onDragEnd(
        cloneTargetPose(this.targetPose),
        cancelledDrag.kind,
        { cancelled: true },
      );
    }
    this.drag = null;
    if (!this.cameraAnimation) this.controls.enabled = true;
    this.canvas.classList.remove("target-edit-dragging");
    this.#syncVisuals();
  }

  update(now) {
    if (this.cameraAnimation) {
      const elapsed = now - this.cameraAnimation.startTime;
      const fraction = smoothStep(elapsed / this.cameraAnimation.durationMs);
      this.camera.position.lerpVectors(
        this.cameraAnimation.fromPosition,
        this.cameraAnimation.toPosition,
        fraction,
      );
      this.controls.target.lerpVectors(
        this.cameraAnimation.fromTarget,
        this.cameraAnimation.toTarget,
        fraction,
      );
      if (elapsed >= this.cameraAnimation.durationMs) {
        this.camera.position.copy(this.cameraAnimation.toPosition);
        this.controls.target.copy(this.cameraAnimation.toTarget);
        this.cameraAnimation = null;
        if (!this.drag) this.controls.enabled = true;
      }
    }
    if (this.targetPose) {
      const distance = this.camera.position.distanceTo(
        new THREE.Vector3(...this.targetPose.position),
      );
      this.controlScale = THREE.MathUtils.clamp(distance / 0.8, 0.65, 1.8);
      this.targetMarker.scale.setScalar(this.controlScale);
      this.translationArrowGroup.scale.setScalar(
        THREE.MathUtils.clamp(distance / 0.8, 0.35, 2.75),
      );
      const activeAxis = this.transformControls.axis;
      this.translationArrows.forEach(({ axis, arrow, color }) => {
        arrow.setColor(activeAxis === axis ? 0xffffff : color);
      });
    }
  }

  dispose() {
    this.cancelPointer();
    this.cameraAnimation = null;
    this.transformControls.removeEventListener("objectChange", this.handleTranslationObjectChange);
    this.transformControls.removeEventListener(
      "dragging-changed",
      this.handleTranslationDraggingChanged,
    );
    this.rotationControls.removeEventListener("objectChange", this.handleRotationObjectChange);
    this.rotationControls.removeEventListener(
      "dragging-changed",
      this.handleRotationDraggingChanged,
    );
    this.transformControls.detach();
    this.transformControls.dispose();
    this.rotationControls.detach();
    this.rotationControls.dispose();
    this.translationArrows.forEach(({ arrow }) => {
      [arrow.line, arrow.cone].forEach((part) => {
        part.geometry.dispose();
        part.material.dispose();
      });
    });
    this.transformHelper.removeFromParent();
    this.rotationHelper.removeFromParent();
    this.root.removeFromParent();
  }

  #animateCamera(position, target) {
    this.cancelPointer();
    this.cameraAnimation = {
      startTime: performance.now(),
      durationMs: 360,
      fromPosition: this.camera.position.clone(),
      toPosition: position.clone(),
      fromTarget: this.controls.target.clone(),
      toTarget: target.clone(),
    };
    this.controls.enabled = false;
  }

  #poseChanged(reason) {
    this.validationState = "pending";
    this.#syncVisuals();
    const pose = cloneTargetPose(this.targetPose);
    this.onPoseChange(pose, reason);
    if (this.drag && ["axis-drag", "rotation-drag"].includes(reason)) {
      this.onDragChange(cloneTargetPose(pose), this.drag.kind);
    }
  }

  #setState(state) {
    this.state = state;
    this.onStateChange({ state, label: STATE_LABELS[state] });
  }

  #syncVisuals() {
    const hasActual = Boolean(this.actualPose);
    const hasTarget = Boolean(this.targetPose);
    const controlPose = this.controlPoseOverride ?? this.targetPose;
    this.actualMarker.visible = hasActual;
    this.targetMarker.visible = this.modeActive
      && hasTarget
      && !this.controlPoseOverride
      && this.validationState === "invalid";
    if (hasTarget) {
      this.targetMarker.position.fromArray(this.targetPose.position);
    }
    if (controlPose) {
      this.transformPivot.position.fromArray(controlPose.position);
      this.transformPivot.quaternion.copy(quaternionFromArray(controlPose.orientation));
      this.translationArrowGroup.position.fromArray(controlPose.position);
      this.translationArrowGroup.quaternion.identity();
    }

    const showTransform = this.modeActive && Boolean(controlPose);
    const controlsVisible = showTransform && this.controlsVisible;
    this.translationArrowGroup.visible = controlsVisible;
    const canInteract = Boolean(controlsVisible && this.interactionEnabled);
    this.transformControls.enabled = canInteract && this.drag?.kind !== "rotate";
    this.rotationControls.enabled = canInteract && this.drag?.kind !== "translate";
    if (controlsVisible) {
      if (this.transformControls.object !== this.transformPivot) {
        this.transformControls.attach(this.transformPivot);
      }
      if (this.rotationControls.object !== this.transformPivot) {
        this.rotationControls.attach(this.transformPivot);
      }
    } else if (this.transformControls.object) {
      this.transformControls.detach();
      this.rotationControls.detach();
    }
  }
}

export { TARGET_EDIT_STATE };
